"""Robot state aggregator — the robotics-engineer worker.

Fuses raw streams into one authoritative /robot/state JSON and supervises
safety: any hard impact, water ingress or excessive tilt latches /safety/stop,
which the base controller (and FE drivetrain) obey immediately.

Topics in:
  /robot/contacts  std_msgs/String  JSON contact events from the simulator
  /imu             sensor_msgs/Imu  tilt + dynamics
  /odom            nav_msgs/Odometry
  /scan            sensor_msgs/LaserScan  nearest-obstacle awareness
  /arm/state       std_msgs/String  JSON arm firmware state
  /base/state      std_msgs/String  JSON base controller state
  /safety/reset    std_msgs/Bool    operator reset request

Topics out:
  /safety/stop     std_msgs/Bool    latched e-stop @ 10 Hz
  /robot/state     std_msgs/String  fused state JSON @ 5 Hz
  /robot/events    std_msgs/String  safety event JSON (on trigger)
"""
from __future__ import annotations

import json
import math
import os
from datetime import UTC, datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String

from . import topics
from .safety import SafetyMonitor, arm_in_drop_phase, mask_arm_sector

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

SPEC_PATH = os.environ.get("ROBOT_SPEC_PATH", "/ros2_ws/shared/robot_spec.json")


def _load_safety_spec() -> dict:
    try:
        with open(SPEC_PATH) as f:
            return json.load(f)["safety"]
    except (OSError, KeyError, json.JSONDecodeError):
        return {"contact_impulse_stop_threshold": 3.0, "tilt_stop_deg": 30.0}


class RobotStateAggregatorNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_state_aggregator_node")
        spec = _load_safety_spec()
        self._monitor = SafetyMonitor(
            impulse_threshold=float(spec["contact_impulse_stop_threshold"]),
            tilt_limit_deg=float(spec["tilt_stop_deg"]),
        )
        self._tilt_deg = 0.0
        self._odom: dict = {}
        self._min_scan: float | None = None
        self._arm_state: dict = {}
        self._base_state: dict = {}
        self._last_contact: dict = {}

        self._pub_stop = self.create_publisher(Bool, topics.SAFETY_STOP, 10)
        self._pub_state = self.create_publisher(String, topics.ROBOT_STATE, 10)
        self._pub_events = self.create_publisher(String, topics.EVENTS, 10)

        self.create_subscription(String, topics.CONTACTS, self._on_contact, 20)
        self.create_subscription(Imu, topics.IMU, self._on_imu, SENSOR_QOS)
        self.create_subscription(Odometry, topics.ODOM, self._on_odom, 10)
        self.create_subscription(LaserScan, topics.SCAN, self._on_scan, SENSOR_QOS)
        self.create_subscription(String, topics.ARM_STATE, self._on_arm_state, 10)
        self.create_subscription(String, topics.BASE_STATE, self._on_base_state, 10)
        self.create_subscription(Bool, topics.SAFETY_RESET, self._on_reset, 10)

        self.create_timer(0.10, self._publish_stop)
        self.create_timer(0.20, self._publish_state)
        self.get_logger().info(
            f"RobotStateAggregator ready — impulse>={self._monitor.impulse_threshold} Ns, "
            f"tilt>={self._monitor.tilt_limit_deg} deg latch /safety/stop",
        )

    # ── Inputs ────────────────────────────────────────────────────────────────

    def _on_contact(self, msg: String) -> None:
        try:
            contact = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._last_contact = contact
        triggered = self._monitor.on_contact(
            impulse=float(contact.get("impulse", 0.0)),
            critical=bool(contact.get("critical", False)),
            detail=contact,
        )
        if triggered:
            self._emit_safety_events()
            self._publish_stop()
            self.get_logger().warning(
                f"E-STOP: contact part={contact.get('part')} "
                f"impulse={contact.get('impulse')} object={contact.get('object_id')}",
            )

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        # tilt = angle between body z-axis and world up
        up_z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
        if self._monitor.on_tilt(self._tilt_deg):
            self._emit_safety_events()
            self._publish_stop()
            self.get_logger().warning(f"E-STOP: tilt {self._tilt_deg:.1f} deg")

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = {
            "x": round(msg.pose.pose.position.x, 3),
            "y": round(msg.pose.pose.position.y, 3),
            "vx": round(msg.twist.twist.linear.x, 3),
            "wz": round(msg.twist.twist.angular.z, 3),
        }

    def _on_scan(self, msg: LaserScan) -> None:
        ranges: list[float] = list(msg.ranges)
        if arm_in_drop_phase(self._arm_state):
            ranges = mask_arm_sector(ranges, msg.angle_min, msg.angle_increment)
        valid = [r for r in ranges if msg.range_min < r < msg.range_max]
        self._min_scan = round(min(valid), 3) if valid else None

    def _on_arm_state(self, msg: String) -> None:
        self._arm_state = _safe_json(msg.data)

    def _on_base_state(self, msg: String) -> None:
        self._base_state = _safe_json(msg.data)

    def _on_reset(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self._monitor.reset():
            self.get_logger().info("Safety latch cleared by operator reset")
        else:
            self.get_logger().warning("Safety reset refused — hazard still active")
        self._publish_stop()

    # ── Outputs ───────────────────────────────────────────────────────────────

    def _emit_safety_events(self) -> None:
        for event in self._monitor.drain_events():
            self._pub_events.publish(String(data=json.dumps({
                "event": f"SAFETY_{event.reason}",
                "detail": event.detail,
                "timestamp": datetime.now(UTC).isoformat(),
            })))

    def _publish_stop(self) -> None:
        self._pub_stop.publish(Bool(data=self._monitor.stop))

    def _publish_state(self) -> None:
        self._pub_state.publish(String(data=json.dumps({
            "safety_stop": self._monitor.stop,
            "safety_critical": self._monitor.critical,
            "tilt_deg": round(self._tilt_deg, 2),
            "min_obstacle_m": self._min_scan,
            "arm_scan_filter": arm_in_drop_phase(self._arm_state),
            "odom": self._odom,
            "arm": {
                "status": self._arm_state.get("status"),
                "last_action": self._arm_state.get("last_action"),
                "joints_deg": self._arm_state.get("joints_deg"),
            },
            "base": {
                "status": self._base_state.get("status"),
                "vx": self._base_state.get("vx"),
                "wz": self._base_state.get("wz"),
            },
            "last_contact": self._last_contact or None,
            "timestamp": datetime.now(UTC).isoformat(),
        })))


def _safe_json(raw: str) -> dict:
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return {}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotStateAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""Mission executor — closes the autonomy loop.

Consumes perception + state, drives the robot through the arbitrated control
path (/cmd_vel/auto, /arm/command) and reports /mission/state. Optional LLM
target arbitration via llm_client (mock by default, OpenAI-compatible via env).

Sim note (documented trade-off): towel world coordinates come from
/ground_truth/objects, while the camera detections gate target confirmation.
Swapping to detection-only navigation is a documented exercise for AI
engineers — the seam is MissionInput.towels.

Topics in:
  /ground_truth/objects  std_msgs/String   world-frame object states (FE)
  /ground_truth/pose     geometry_msgs/PoseStamped  robot pose (FE)
  /odom                  nav_msgs/Odometry fallback pose when GT is absent
  /scan                  sensor_msgs/LaserScan  front-obstacle slowdown
  /arm/state             std_msgs/String   firmware status gating sequences
  /robot/control_mode    std_msgs/String   only acts in auto mode
  /safety/stop           std_msgs/Bool     aborts to IDLE
  /detected_objects      std_msgs/String   perception confirmation (optional gate)

Topics out:
  /cmd_vel/auto  geometry_msgs/Twist
  /arm/command   std_msgs/String
  /mission/state std_msgs/String JSON
"""
from __future__ import annotations

import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .llm_client import complete_json, create_llm_client
from .mission import MissionInput, MissionLogic

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

LAYOUT_PATH = os.environ.get("ONSEN_LAYOUT_PATH", "/ros2_ws/shared/onsen_layout.json")
LLM_SYSTEM_PROMPT = (
    "You are the task arbiter for a towel-collecting onsen robot. "
    "Given detections JSON, reply with JSON: "
    '{"action": "pick_object"|"continue_search"|"emergency_stop", '
    '"target_id": str|null, "reason": str}.'
)


def _towel_bin_center() -> tuple[float, float]:
    try:
        with open(LAYOUT_PATH) as f:
            layout = json.load(f)
        bin_def = next(b for b in layout["bins"] if b["type"] == "towel")
        return (float(bin_def["c"][0]), float(bin_def["c"][1]))
    except (OSError, StopIteration, KeyError, json.JSONDecodeError):
        return (0.0, 4.45)


class MissionExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_executor_node")
        self._logic = MissionLogic(bin_center=_towel_bin_center())
        self._llm = create_llm_client()
        self._llm_consult_interval = float(os.environ.get("MISSION_LLM_INTERVAL", "5.0"))
        self._last_llm_at = 0.0
        self._llm_verdict: dict = {}

        self._pose: dict | None = None
        self._pose_source = "none"
        self._towels: list[dict] = []
        self._holding = False
        self._arm_status = "IDLE"
        self._mode = "auto"
        self._safety = False
        self._min_front = None
        self._detections: list[dict] = []

        self._pub_twist = self.create_publisher(Twist, "/cmd_vel/auto", 10)
        self._pub_arm = self.create_publisher(String, "/arm/command", 10)
        self._pub_state = self.create_publisher(String, "/mission/state", 10)

        self.create_subscription(String, "/ground_truth/objects", self._on_objects, 10)
        self.create_subscription(PoseStamped, "/ground_truth/pose", self._on_gt_pose, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, SENSOR_QOS)
        self.create_subscription(String, "/arm/state", self._on_arm_state, 10)
        self.create_subscription(String, "/robot/control_mode", self._on_mode, 10)
        self.create_subscription(Bool, "/safety/stop", self._on_safety, 10)
        self.create_subscription(String, "/detected_objects", self._on_detections, 10)

        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f"MissionExecutor ready — bin at {self._logic.bin_center}, llm={self._llm.name}",
        )

    # ── Inputs ────────────────────────────────────────────────────────────────

    def _on_objects(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        objects = payload.get("objects", [])
        self._towels = [
            o for o in objects
            if o.get("class") == "towel" and not o.get("binned") and not o.get("held")
        ]
        self._holding = any(o.get("held") for o in objects)

    def _on_gt_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        self._pose = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "yaw": math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
        }
        self._pose_source = "ground_truth"

    def _on_odom(self, msg: Odometry) -> None:
        if self._pose_source == "ground_truth":
            return
        q = msg.pose.pose.orientation
        self._pose = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw": math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
        }
        self._pose_source = "odom"

    def _on_scan(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        if n == 0:
            return
        # forward sector = beams around the middle of the (-pi..pi) sweep
        window = n // 8
        mid = n // 2
        sector = msg.ranges[mid - window: mid + window]
        valid = [r for r in sector if msg.range_min < r < msg.range_max]
        self._min_front = min(valid) if valid else None

    def _on_arm_state(self, msg: String) -> None:
        try:
            self._arm_status = json.loads(msg.data).get("status", "IDLE")
        except json.JSONDecodeError:
            pass

    def _on_mode(self, msg: String) -> None:
        try:
            self._mode = json.loads(msg.data).get("mode", "auto")
        except json.JSONDecodeError:
            pass

    def _on_safety(self, msg: Bool) -> None:
        self._safety = bool(msg.data)

    def _on_detections(self, msg: String) -> None:
        try:
            self._detections = json.loads(msg.data).get("objects", [])
        except json.JSONDecodeError:
            pass

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        if self._pose is None:
            return
        self._maybe_consult_llm()

        out = self._logic.update(MissionInput(
            pose=self._pose,
            towels=self._towels,
            holding=self._holding,
            arm_status=self._arm_status,
            safety_stop=self._safety,
            mode=self._mode,
            min_front_obstacle=self._min_front,
        ))

        if out.twist is not None:
            twist = Twist()
            twist.linear.x = float(out.twist[0])
            twist.angular.z = float(out.twist[1])
            self._pub_twist.publish(twist)
        if out.arm_command:
            self._pub_arm.publish(String(data=out.arm_command))

        self._pub_state.publish(String(data=json.dumps({
            "state": out.state,
            "reason": out.reason,
            "target_id": out.target_id or (self._logic.target or {}).get("id"),
            "holding": self._holding,
            "towels_remaining": len(self._towels),
            "pose_source": self._pose_source,
            "llm": self._llm_verdict.get("action"),
            "llm_reason": self._llm_verdict.get("reason"),
        })))

    def _maybe_consult_llm(self) -> None:
        now = time.monotonic()
        if now - self._last_llm_at < self._llm_consult_interval:
            return
        self._last_llm_at = now
        context = json.dumps({
            "safety_stop": self._safety,
            "detections": self._detections,
            "towels_remaining": len(self._towels),
            "holding": self._holding,
        })
        self._llm_verdict = complete_json(self._llm, LLM_SYSTEM_PROMPT, context)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

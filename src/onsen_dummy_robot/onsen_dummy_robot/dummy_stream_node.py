"""Headless simulation source selector.

SIM_SOURCE environment variable picks where simulation data comes from:

  fe         (default) the browser frontend is the physics ground truth.
              This node idles — it only logs a heartbeat so the service stays up.
  synthetic  full headless loop for AI engineers: integrates the robot pose
              from /base/wheel_targets against the shared onsen layout
              (collision -> /robot/contacts), raycasts /scan from the same
              geometry, renders synthetic camera frames, simulates towel
              grasp/drop against the arm controller targets and publishes the
              same ground-truth topics as the FE. The whole autonomy stack
              (controllers, safety, AI worker, mission executor) runs unchanged.
  replay     rosbag playback owns the topics; this node idles like `fe`.

The synthetic mode is intentionally 2.5D (no rigid-body engine) but uses the
identical layout JSON, controller protocols and topic contract, so recorded FE
sessions and synthetic sessions are interchangeable for algorithm work.
"""
from __future__ import annotations

import json
import math
import os
import random
import time
from datetime import UTC, datetime

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .layout import OnsenLayout
from .scene_generator import SceneGenerator

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

WHEEL_RADIUS = 0.07
TRACK_WIDTH = 0.47
ROBOT_RADIUS = 0.33
GRIP_CLOSE_DEG = 30.0
PICK_WINDOW_X = (0.45, 0.85)
PICK_WINDOW_Y = 0.18
BIN_DROP_RADIUS = 0.6


class StandbyNode(Node):
    def __init__(self, mode: str) -> None:
        super().__init__("dummy_stream_node")
        self._mode = mode
        self.create_timer(30.0, self._heartbeat)
        self.get_logger().info(f"SIM_SOURCE={mode} — dummy stream on standby")

    def _heartbeat(self) -> None:
        self.get_logger().debug(f"standby ({self._mode})")


class SyntheticSimNode(Node):
    def __init__(self) -> None:
        super().__init__("dummy_stream_node")
        self._layout = OnsenLayout()
        self._scene = SceneGenerator(save_dataset=False, dataset_dir="dataset")
        self._rng = random.Random(42)

        self._x, self._y = self._layout.spawn_x, self._layout.spawn_y
        self._yaw = self._layout.spawn_yaw
        self._vl = self._vr = 0.0          # commanded side surface speeds (m/s)
        self._wheel_angles = [0.0] * 6
        self._arm_deg = [90.0, 90.0, 90.0, 90.0, 90.0, 70.0]
        self._was_closed = False
        self._held_id: str | None = None
        self._last_update = time.monotonic()
        self._odo = {"x": self._x, "y": self._y, "yaw": self._yaw}

        with open(os.environ.get("ONSEN_LAYOUT_PATH", "/ros2_ws/shared/onsen_layout.json")) as f:
            raw = json.load(f)
        self._towels = [
            {"id": p["id"], "pos": list(p["pos"]), "held": False, "binned": None}
            for p in raw["dynamic_props"] if p["type"] == "towel"
        ]
        self._bins = [
            {"id": b["id"], "type": b["type"], "c": b["c"]} for b in raw["bins"]
        ]

        self._pub_scan = self.create_publisher(LaserScan, "/scan", SENSOR_QOS)
        self._pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self._pub_imu = self.create_publisher(Imu, "/imu", SENSOR_QOS)
        self._pub_joints = self.create_publisher(JointState, "/joint_states", 10)
        self._pub_compressed = self.create_publisher(
            CompressedImage, "/camera/front/image_raw/compressed", SENSOR_QOS,
        )
        self._pub_caminfo = self.create_publisher(
            CameraInfo, "/camera/front/camera_info", SENSOR_QOS,
        )
        self._pub_contacts = self.create_publisher(String, "/robot/contacts", 20)
        self._pub_events = self.create_publisher(String, "/robot/events", 10)
        self._pub_gt_pose = self.create_publisher(PoseStamped, "/ground_truth/pose", 10)
        self._pub_gt_objects = self.create_publisher(String, "/ground_truth/objects", 10)
        self._tf = TransformBroadcaster(self)

        self.create_subscription(String, "/base/wheel_targets", self._on_wheel_targets, 10)
        self.create_subscription(String, "/arm/joint_targets", self._on_arm_targets, 10)

        self.create_timer(0.05, self._step)            # 20 Hz pose + odom + joints
        self.create_timer(0.125, self._publish_scan)   # 8 Hz
        self.create_timer(0.2, self._publish_camera)   # 5 Hz
        self.create_timer(0.5, self._publish_ground_truth)
        self.get_logger().info(
            f"SIM_SOURCE=synthetic — headless onsen sim, {len(self._towels)} towels",
        )

    # ── Control inputs (same contract as the FE) ─────────────────────────────

    def _on_wheel_targets(self, msg: String) -> None:
        try:
            w = json.loads(msg.data)["w"]
        except (json.JSONDecodeError, KeyError):
            return
        self._vl = float(np.mean(w[0:3])) * WHEEL_RADIUS
        self._vr = float(np.mean(w[3:6])) * WHEEL_RADIUS

    def _on_arm_targets(self, msg: String) -> None:
        try:
            deg = json.loads(msg.data)["deg"]
        except (json.JSONDecodeError, KeyError):
            return
        if len(deg) == 6:
            self._arm_deg = [float(v) for v in deg]
            self._update_grasp()

    # ── Simulation step ───────────────────────────────────────────────────────

    def _step(self) -> None:
        now = time.monotonic()
        dt = max(0.0, min(now - self._last_update, 0.1))
        self._last_update = now

        v = (self._vl + self._vr) / 2.0
        wz = (self._vr - self._vl) / TRACK_WIDTH
        self._yaw += wz * dt
        nx = self._x + v * math.cos(self._yaw) * dt
        ny = self._y + v * math.sin(self._yaw) * dt
        nx, ny, hit = self._layout.resolve_collision(nx, ny, ROBOT_RADIUS)
        if hit and abs(v) > 0.02:
            self._pub_contacts.publish(String(data=json.dumps({
                "part": "chassis_body",
                "impulse": round(abs(v) * 14.0, 3),
                "force": round(abs(v) * 14.0 * 20, 2),
                "object_kind": "wall",
                "object_id": "layout",
                "object_class": None,
                "critical": False,
                "robot_pose": {"x": round(nx, 3), "y": round(ny, 3), "yaw": round(self._yaw, 3)},
                "timestamp": _iso_now(),
            })))
        self._x, self._y = nx, ny

        # encoder odometry (commanded speeds -> drifts on collisions, like FE)
        self._odo["yaw"] += wz * dt
        self._odo["x"] += v * math.cos(self._odo["yaw"]) * dt
        self._odo["y"] += v * math.sin(self._odo["yaw"]) * dt
        for i in range(3):
            self._wheel_angles[i] += (self._vl / WHEEL_RADIUS) * dt
            self._wheel_angles[3 + i] += (self._vr / WHEEL_RADIUS) * dt

        if self._held_id:
            towel = self._towel(self._held_id)
            towel["pos"] = [
                self._x + 0.3 * math.cos(self._yaw),
                self._y + 0.3 * math.sin(self._yaw),
            ]

        stamp = self._ros_time()
        self._publish_odom(stamp, v, wz)
        self._publish_imu(stamp, wz)
        self._publish_joints(stamp)

    # ── Towel grasp emulation ─────────────────────────────────────────────────

    def _update_grasp(self) -> None:
        closed = self._arm_deg[5] <= GRIP_CLOSE_DEG
        if closed and not self._was_closed and self._held_id is None:
            for towel in self._towels:
                if towel["held"] or towel["binned"]:
                    continue
                rel = self._to_robot_frame(towel["pos"])
                if PICK_WINDOW_X[0] < rel[0] < PICK_WINDOW_X[1] and abs(rel[1]) < PICK_WINDOW_Y:
                    towel["held"] = True
                    self._held_id = towel["id"]
                    self._event("GRASP_SUCCESS", object_id=towel["id"])
                    break
        elif not closed and self._was_closed and self._held_id is not None:
            towel = self._towel(self._held_id)
            towel["held"] = False
            self._held_id = None
            for bin_def in self._bins:
                d = math.hypot(bin_def["c"][0] - self._x, bin_def["c"][1] - self._y)
                if d < BIN_DROP_RADIUS:
                    towel["binned"] = bin_def["id"]
                    towel["pos"] = list(bin_def["c"])
                    self._event(
                        "OBJECT_BINNED", object_id=towel["id"], bin_id=bin_def["id"],
                        correct=bin_def["type"] == "towel",
                    )
                    break
        self._was_closed = closed

    def _towel(self, towel_id: str) -> dict:
        return next(t for t in self._towels if t["id"] == towel_id)

    def _to_robot_frame(self, pos: list[float]) -> tuple[float, float]:
        dx, dy = pos[0] - self._x, pos[1] - self._y
        c, s = math.cos(-self._yaw), math.sin(-self._yaw)
        return (dx * c - dy * s, dx * s + dy * c)

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_scan(self) -> None:
        ranges = self._layout.scan(self._x, self._y, self._yaw)
        ranges = np.clip(
            ranges + np.random.normal(0.0, 0.01, ranges.shape),
            self._layout.lidar_range_min, self._layout.lidar_range_max,
        ).astype(np.float32)
        msg = LaserScan()
        msg.header.stamp = self._ros_time()
        msg.header.frame_id = "laser_link"
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2 * math.pi / len(ranges)
        msg.scan_time = 0.125
        msg.range_min = self._layout.lidar_range_min
        msg.range_max = self._layout.lidar_range_max
        msg.ranges = ranges.tolist()
        self._pub_scan.publish(msg)

    def _publish_camera(self) -> None:
        img_rgb, _gt = self._scene.generate()
        _, jpeg = cv2.imencode(
            ".jpg", cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 80],
        )
        stamp = self._ros_time()
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_front_link"
        msg.format = "jpeg"
        msg.data = jpeg.tobytes()
        self._pub_compressed.publish(msg)

        ci = CameraInfo()
        ci.header = msg.header
        ci.width, ci.height = 640, 480
        fx = 457.0
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0] * 5
        ci.k = [fx, 0.0, 320.0, 0.0, fx, 240.0, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [fx, 0.0, 320.0, 0.0, 0.0, fx, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._pub_caminfo.publish(ci)

    def _publish_odom(self, stamp: RosTime, v: float, wz: float) -> None:
        q = _yaw_to_quat(self._odo["yaw"])
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self._odo["x"]
        odom.pose.pose.position.y = self._odo["y"]
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = wz
        self._pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self._odo["x"]
        tf.transform.translation.y = self._odo["y"]
        tf.transform.rotation = q
        self._tf.sendTransform(tf)

        gt = PoseStamped()
        gt.header.stamp = stamp
        gt.header.frame_id = "map"
        gt.pose.position.x = self._x
        gt.pose.position.y = self._y
        gt.pose.orientation = _yaw_to_quat(self._yaw)
        self._pub_gt_pose.publish(gt)

    def _publish_imu(self, stamp: RosTime, wz: float) -> None:
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation = _yaw_to_quat(self._yaw)
        imu.angular_velocity.z = wz + self._rng.gauss(0.0, 0.002)
        imu.linear_acceleration.z = 9.81 + self._rng.gauss(0.0, 0.05)
        self._pub_imu.publish(imu)

    def _publish_joints(self, stamp: RosTime) -> None:
        js = JointState()
        js.header.stamp = stamp
        js.name = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_pitch_joint", "wrist_roll_joint", "gripper_joint",
            "wheel_front_left", "wheel_front_right", "wheel_mid_left",
            "wheel_mid_right", "wheel_rear_left", "wheel_rear_right",
        ]
        js.position = [math.radians(d - 90.0) for d in self._arm_deg] + list(self._wheel_angles)
        js.velocity = [0.0] * 12
        js.effort = [0.0] * 12
        self._pub_joints.publish(js)

    def _publish_ground_truth(self) -> None:
        self._pub_gt_objects.publish(String(data=json.dumps({
            "timestamp": _iso_now(),
            "objects": [
                {
                    "id": t["id"],
                    "class": "towel",
                    "position": {"x": round(t["pos"][0], 3), "y": round(t["pos"][1], 3), "z": 0.0},
                    "held": t["held"],
                    "binned": t["binned"],
                    "pickable": True,
                }
                for t in self._towels
            ],
        })))

    def _event(self, event: str, **extra) -> None:
        self._pub_events.publish(String(data=json.dumps({
            "event": event, "timestamp": _iso_now(), **extra,
        })))

    def _ros_time(self) -> RosTime:
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        rt = RosTime()
        rt.sec = sec
        rt.nanosec = nsec
        return rt


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


def _iso_now() -> str:
    return datetime.now(UTC).isoformat()


def main(args=None) -> None:
    rclpy.init(args=args)
    mode = os.environ.get("SIM_SOURCE", "fe").lower()
    node = SyntheticSimNode() if mode == "synthetic" else StandbyNode(mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

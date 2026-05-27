"""
dummy_stream_node.py
Publishes all synthetic onsen robot topics AND accepts control commands,
feeding them through as if from a real robot.

Control inputs (subscribed):
  /cmd_vel        geometry_msgs/Twist  — tank drive velocity
  /arm/action     std_msgs/String JSON — arm state or joint override

When /cmd_vel is received the robot switches from autonomous loop to
manual dead-reckoning.  After MANUAL_TIMEOUT seconds of silence it
resumes the autonomous patrol loop seamlessly.
"""
from __future__ import annotations

import json
import math
import os
import random
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import (
    CameraInfo, CompressedImage, Image, JointState, LaserScan,
)
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .arm_state_machine import ArmStateMachine, ArmState
from .lidar_generator import LidarGenerator
from .scene_generator import SceneGenerator

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

EVENTS = [
    "TARGET_DETECTED", "TARGET_LOST", "GRIP_FAILED",
    "OBJECT_DROPPED", "PERSON_TOO_CLOSE", "OBSTACLE_TOO_CLOSE",
    "STEAM_VISIBILITY_LOW", "WET_FLOOR_REFLECTION_HIGH",
]

WHEEL_RADIUS  = 0.07   # m
TRACK_WIDTH   = 0.35   # m
MANUAL_TIMEOUT = 3.0   # s — resume auto loop if no cmd_vel received

# Room bounds (same as lidar_generator room)
ROOM_HALF_X = 2.3
ROOM_HALF_Y = 1.8


class DummyStreamNode(Node):
    def __init__(self) -> None:
        super().__init__("dummy_stream_node")

        save_dataset = os.environ.get("SAVE_DATASET", "false").lower() == "true"
        dataset_dir  = os.path.join(os.getcwd(), "dataset")

        self._scene_gen = SceneGenerator(save_dataset=save_dataset, dataset_dir=dataset_dir)
        self._lidar_gen = LidarGenerator(num_rays=360)
        self._arm_sm    = ArmStateMachine()
        self._rng       = random.Random()

        # Robot pose
        self._robot_x:   float = 0.0
        self._robot_y:   float = 0.0
        self._robot_yaw: float = 0.0

        # Autonomous loop state
        self._loop_radius: float = 1.2
        self._loop_speed:  float = 0.18   # rad/s
        self._loop_angle:  float = 0.0

        # Manual control state
        self._cmd_vx:           float = 0.0
        self._cmd_wz:           float = 0.0
        self._last_cmd_vel_time: float = -999.0   # far past → auto mode on start

        # Wheel encoder accumulators
        self._left_wheel_angle:  float = 0.0
        self._right_wheel_angle: float = 0.0

        self._last_update  = time.monotonic()
        self._last_gt: dict[str, Any] = {}

        # ── Publishers ──────────────────────────────────────────────────────
        self._pub_image      = self.create_publisher(Image,           "/camera/front/image_raw",            SENSOR_QOS)
        self._pub_compressed = self.create_publisher(CompressedImage, "/camera/front/image_raw/compressed", SENSOR_QOS)
        self._pub_caminfo    = self.create_publisher(CameraInfo,      "/camera/front/camera_info",          SENSOR_QOS)
        self._pub_scan       = self.create_publisher(LaserScan,       "/scan",                              SENSOR_QOS)
        self._pub_odom       = self.create_publisher(Odometry,        "/odom",                              10)
        self._pub_joints     = self.create_publisher(JointState,      "/joint_states",                      10)
        self._pub_arm_state  = self.create_publisher(String,          "/arm/state",                         10)
        self._pub_events     = self.create_publisher(String,          "/robot/events",                      10)
        self._pub_ctrl_mode  = self.create_publisher(String,          "/robot/control_mode",                10)

        # ── Subscribers (control inputs) ────────────────────────────────────
        self.create_subscription(Twist,  "/cmd_vel",    self._on_cmd_vel,    10)
        self.create_subscription(String, "/arm/action", self._on_arm_action, 10)

        # ── TF broadcasters ─────────────────────────────────────────────────
        self._tf_broadcaster        = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transforms()

        # ── Timers ──────────────────────────────────────────────────────────
        self.create_timer(0.143, self._publish_camera)   # ~7 Hz
        self.create_timer(0.125, self._publish_lidar)    #  8 Hz
        self.create_timer(0.050, self._publish_odom)     # 20 Hz
        self.create_timer(0.100, self._publish_arm)      # 10 Hz
        self.create_timer(3.000, self._maybe_publish_event)

        self.get_logger().info("DummyStreamNode ready — /cmd_vel and /arm/action accepted")

    # ── Control input callbacks ──────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Receive velocity command from UI (or any ROS2 nav stack)."""
        self._cmd_vx = float(msg.linear.x)
        self._cmd_wz = float(msg.angular.z)
        self._last_cmd_vel_time = time.monotonic()

    def _on_arm_action(self, msg: String) -> None:
        """
        Receive arm control command from UI.
        Supported payloads:
          {"cmd": "set_state",  "state": "GRIP"}
          {"cmd": "set_joint",  "joint": "shoulder_pan_joint", "value": 0.5}
          {"cmd": "clear"}
        """
        try:
            action = json.loads(msg.data)
            cmd = action.get("cmd", "")
            if cmd == "set_state":
                self._arm_sm.force_state(action.get("state", "HOME"))
            elif cmd == "set_joint":
                self._arm_sm.set_joint_angle(
                    action.get("joint", ""), float(action.get("value", 0))
                )
            elif cmd == "clear":
                self._arm_sm.clear_overrides()
        except Exception as e:
            self.get_logger().warn(f"Bad /arm/action payload: {e}")

    # ── Static transforms ────────────────────────────────────────────────────

    def _publish_static_transforms(self) -> None:
        stamp = self._ros_time()
        defs = [
            ("base_link", "laser_link",        0.10, 0.00, 0.20),
            ("base_link", "camera_front_link", 0.18, 0.00, 0.15),
            ("base_link", "arm_base_link",     0.12, 0.05, 0.05),
        ]
        msgs = []
        for parent, child, tx, ty, tz in defs:
            ts = TransformStamped()
            ts.header.stamp    = stamp
            ts.header.frame_id = parent
            ts.child_frame_id  = child
            ts.transform.translation.x = tx
            ts.transform.translation.y = ty
            ts.transform.translation.z = tz
            ts.transform.rotation.w = 1.0
            msgs.append(ts)
        self._static_tf_broadcaster.sendTransform(msgs)

    # ── Camera ───────────────────────────────────────────────────────────────

    def _publish_camera(self) -> None:
        img_rgb, gt = self._scene_gen.generate()
        self._last_gt = gt
        stamp = self._ros_time()

        img_msg             = Image()
        img_msg.header.stamp    = stamp
        img_msg.header.frame_id = "camera_front_link"
        img_msg.height   = img_rgb.shape[0]
        img_msg.width    = img_rgb.shape[1]
        img_msg.encoding = "rgb8"
        img_msg.step     = img_rgb.shape[1] * 3
        img_msg.data     = img_rgb.tobytes()
        self._pub_image.publish(img_msg)

        _, jpeg_buf = cv2.imencode(
            ".jpg",
            cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR),
            [cv2.IMWRITE_JPEG_QUALITY, 80],
        )
        comp_msg        = CompressedImage()
        comp_msg.header = img_msg.header
        comp_msg.format = "jpeg"
        comp_msg.data   = jpeg_buf.tobytes()
        self._pub_compressed.publish(comp_msg)

        ci = CameraInfo()
        ci.header = img_msg.header
        ci.height = img_msg.height
        ci.width  = img_msg.width
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = fy = 520.0
        cx_f = img_msg.width  / 2.0
        cy_f = img_msg.height / 2.0
        ci.k = [fx, 0.0, cx_f, 0.0, fy, cy_f, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [fx, 0.0, cx_f, 0.0, 0.0, fy, cy_f, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._pub_caminfo.publish(ci)

    # ── LiDAR ────────────────────────────────────────────────────────────────

    def _publish_lidar(self) -> None:
        ranges = self._lidar_gen.generate(self._robot_x, self._robot_y, self._robot_yaw)
        stamp  = self._ros_time()
        msg = LaserScan()
        msg.header.stamp    = stamp
        msg.header.frame_id = "laser_link"
        msg.angle_min       = -math.pi
        msg.angle_max       =  math.pi
        msg.angle_increment = 2 * math.pi / len(ranges)
        msg.scan_time       = 1.0 / 8.0
        msg.range_min       = 0.05
        msg.range_max       = 8.0
        msg.ranges          = ranges.tolist()
        self._pub_scan.publish(msg)

    # ── Odometry + TF ────────────────────────────────────────────────────────

    def _publish_odom(self) -> None:
        now = time.monotonic()
        dt  = max(0.0, min(now - self._last_update, 0.1))   # clamp to 100 ms
        self._last_update = now

        manual = (now - self._last_cmd_vel_time) < MANUAL_TIMEOUT

        if manual:
            # Dead-reckoning from commanded velocity
            vx_fwd = self._cmd_vx
            wz     = self._cmd_wz
            self._robot_yaw += wz * dt
            self._robot_x   += vx_fwd * math.cos(self._robot_yaw) * dt
            self._robot_y   += vx_fwd * math.sin(self._robot_yaw) * dt
            # Clamp to room boundaries (soft wall)
            self._robot_x = max(-ROOM_HALF_X, min(ROOM_HALF_X, self._robot_x))
            self._robot_y = max(-ROOM_HALF_Y, min(ROOM_HALF_Y, self._robot_y))
            # Keep loop_angle in sync for a smooth handoff when releasing control
            self._loop_angle = math.atan2(self._robot_y, self._robot_x)
            v_fwd = vx_fwd
        else:
            # Autonomous circular patrol loop
            self._loop_angle += self._loop_speed * dt
            self._robot_x    = self._loop_radius * math.cos(self._loop_angle)
            self._robot_y    = self._loop_radius * math.sin(self._loop_angle)
            self._robot_yaw  = self._loop_angle + math.pi / 2.0
            v_fwd = self._loop_radius * self._loop_speed
            wz    = self._loop_speed

        # Differential-drive wheel encoder integration
        self._left_wheel_angle  += (v_fwd - wz * TRACK_WIDTH / 2.0) / WHEEL_RADIUS * dt
        self._right_wheel_angle += (v_fwd + wz * TRACK_WIDTH / 2.0) / WHEEL_RADIUS * dt

        stamp = self._ros_time()
        q     = _yaw_to_quat(self._robot_yaw)

        odom = Odometry()
        odom.header.stamp          = stamp
        odom.header.frame_id       = "odom"
        odom.child_frame_id        = "base_link"
        odom.pose.pose.position.x  = self._robot_x
        odom.pose.pose.position.y  = self._robot_y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x  = v_fwd * math.cos(self._robot_yaw) if manual else \
                                     -self._loop_radius * self._loop_speed * math.sin(self._loop_angle)
        odom.twist.twist.linear.y  = v_fwd * math.sin(self._robot_yaw) if manual else \
                                      self._loop_radius * self._loop_speed * math.cos(self._loop_angle)
        odom.twist.twist.angular.z = wz
        self._pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header.stamp       = stamp
        tf.header.frame_id    = "odom"
        tf.child_frame_id     = "base_link"
        tf.transform.translation.x = self._robot_x
        tf.transform.translation.y = self._robot_y
        tf.transform.rotation       = q
        self._tf_broadcaster.sendTransform(tf)

        # Publish control mode so UI can display it
        mode_msg = String()
        mode_msg.data = json.dumps({
            "mode":  "manual" if manual else "auto",
            "vx":    round(v_fwd if manual else 0.0, 3),
            "wz":    round(wz, 3),
        })
        self._pub_ctrl_mode.publish(mode_msg)

    # ── Arm + wheel joints ───────────────────────────────────────────────────

    def _publish_arm(self) -> None:
        joint_angles, arm_state_dict = self._arm_sm.update(0.10)
        stamp = self._ros_time()

        all_names     = list(joint_angles.keys()) + ["left_wheel_joint", "right_wheel_joint"]
        all_positions = [float(v) for v in joint_angles.values()] + [
            self._left_wheel_angle, self._right_wheel_angle,
        ]

        js             = JointState()
        js.header.stamp = stamp
        js.name         = all_names
        js.position     = all_positions
        js.velocity     = [0.0] * len(all_names)
        js.effort       = [0.0] * len(all_names)
        self._pub_joints.publish(js)

        arm_msg      = String()
        arm_msg.data = json.dumps(arm_state_dict)
        self._pub_arm_state.publish(arm_msg)

    # ── Robot events ─────────────────────────────────────────────────────────

    def _maybe_publish_event(self) -> None:
        if self._rng.random() > 0.30:
            return
        arm_state = self._arm_sm.get_state()
        if arm_state == ArmState.FAILED_GRIP:
            event_type = "GRIP_FAILED"
        elif arm_state == ArmState.LOWER_TO_TOWEL:
            event_type = "TARGET_DETECTED"
        elif self._rng.random() < 0.15:
            event_type = "PERSON_TOO_CLOSE"
        else:
            event_type = self._rng.choice(EVENTS)

        extra: dict[str, Any] = {}
        if self._last_gt:
            steam = self._last_gt.get("steam_level", "none")
            if steam in ("medium", "high"):
                event_type = "STEAM_VISIBILITY_LOW"
            if self._last_gt.get("wet_floor") and self._rng.random() < 0.3:
                event_type = "WET_FLOOR_REFLECTION_HIGH"
            extra["steam_level"] = steam
            extra["wet_floor"]   = self._last_gt.get("wet_floor", False)

        msg      = String()
        msg.data = json.dumps({
            "event": event_type,
            "timestamp": _iso_now(),
            "robot_pose": {
                "x":   round(self._robot_x,   3),
                "y":   round(self._robot_y,   3),
                "yaw": round(self._robot_yaw, 3),
            },
            **extra,
        })
        self._pub_events.publish(msg)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _ros_time(self) -> RosTime:
        t = self.get_clock().now()
        sec, nsec = t.seconds_nanoseconds()
        rt       = RosTime()
        rt.sec   = sec
        rt.nanosec = nsec
        return rt


def _yaw_to_quat(yaw: float) -> Quaternion:
    q   = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


def _iso_now() -> str:
    from datetime import datetime, timezone
    return datetime.now(timezone.utc).isoformat()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DummyStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

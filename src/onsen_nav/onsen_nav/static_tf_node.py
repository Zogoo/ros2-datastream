"""Static TF broadcaster — completes the TF tree from shared/robot_spec.json.

base_link -> laser_link, camera_{front,rear,depth}_link, imu_link,
sonar_link_{0..2}, arm_base_link. Everything derived from the spec; no
hardcoded geometry.
"""
from __future__ import annotations

import json
import math
import os

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

SPEC_PATH = os.environ.get("ROBOT_SPEC_PATH", "/ros2_ws/shared/robot_spec.json")


def _quat_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def build_transforms(spec: dict) -> list[dict]:
    """Pure: list of {child, xyz, rpy} dicts for every sensor mount."""
    sensors = spec["sensors"]
    out = [
        {"child": sensors["lidar"]["frame_id"], "xyz": sensors["lidar"]["position"],
         "rpy": (0.0, 0.0, 0.0)},
        {"child": sensors["imu"]["frame_id"], "xyz": sensors["imu"]["position"],
         "rpy": (0.0, 0.0, 0.0)},
        {"child": "arm_base_link", "xyz": spec["arm"]["base_offset"], "rpy": (0.0, 0.0, 0.0)},
    ]
    for key in ("camera_front", "camera_rear", "camera_depth"):
        cam = sensors[key]
        out.append({
            "child": cam["frame_id"],
            "xyz": cam["position"],
            "rpy": (
                0.0,
                math.radians(-float(cam.get("pitch_deg", 0.0))),
                math.radians(float(cam.get("yaw_deg", 0.0))),
            ),
        })
    sonar = sensors["sonar"]
    for i, bearing in enumerate(sonar["bearings_deg"]):
        rad = math.radians(bearing)
        out.append({
            "child": f"{sonar['frame_prefix']}{i}",
            "xyz": [sonar["nose_x"], 0.12 * math.sin(rad), sonar["height"]],
            "rpy": (0.0, 0.0, rad),
        })
    return out


class StaticTfNode(Node):
    def __init__(self) -> None:
        super().__init__("static_tf_node")
        with open(SPEC_PATH) as f:
            spec = json.load(f)
        broadcaster = StaticTransformBroadcaster(self)
        msgs = []
        for t in build_transforms(spec):
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.child_frame_id = t["child"]
            msg.transform.translation.x = float(t["xyz"][0])
            msg.transform.translation.y = float(t["xyz"][1])
            msg.transform.translation.z = float(t["xyz"][2])
            qx, qy, qz, qw = _quat_rpy(*t["rpy"])
            msg.transform.rotation.x = qx
            msg.transform.rotation.y = qy
            msg.transform.rotation.z = qz
            msg.transform.rotation.w = qw
            msgs.append(msg)
        broadcaster.sendTransform(msgs)
        self.get_logger().info(f"Published {len(msgs)} static transforms from robot_spec")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

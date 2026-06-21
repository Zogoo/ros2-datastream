"""Publishes /scan_low — the low-obstacle laser layer derived from the depth
camera (see depth_to_scan.py for why the apt converter doesn't fit a pitched
camera). Frame is base_link: ranges are horizontal distances from base center.
"""
from __future__ import annotations

import json
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan

from onsen_robot_state import topics

from .depth_to_scan import DepthCameraModel, depth_to_scan

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=2,
)

SPEC_PATH = os.environ.get("ROBOT_SPEC_PATH", "/ros2_ws/shared/robot_spec.json")


class DepthScanNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_scan_node")
        with open(SPEC_PATH) as f:
            spec = json.load(f)
        self._cam = DepthCameraModel.from_spec(spec["sensors"]["camera_depth"])
        self._pub = self.create_publisher(LaserScan, topics.SCAN_LOW, SENSOR_QOS)
        self.create_subscription(Image, topics.CAM_DEPTH, self._on_depth, SENSOR_QOS)
        self.get_logger().info("DepthScan ready — /scan_low from the depth camera")

    def _on_depth(self, msg: Image) -> None:
        if msg.encoding != "16UC1":
            return
        depth = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, msg.width)
        ranges, angle_min, angle_inc = depth_to_scan(depth, self._cam)

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = "base_link"
        scan.angle_min = angle_min
        scan.angle_max = angle_min + angle_inc * (len(ranges) - 1)
        scan.angle_increment = angle_inc
        scan.range_min = 0.05
        scan.range_max = 3.5
        scan.ranges = [float(r) if math.isfinite(r) else float("inf") for r in ranges]
        self._pub.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

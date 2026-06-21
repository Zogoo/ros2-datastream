"""Publishes the URDF on /robot_description and republishes coupled arm joints
on /joint_states_urdf for robot_state_publisher (the arm TF tree + RViz).

move_group is not packaged on this ROS distro (docs/research_notes.md); this
node + robot_state_publisher give the arm a real TF tree, and the analytic IK
in onsen_ai_worker is the runtime planner. Keeps /joint_states (FE truth)
untouched — robot_state_publisher is launched with a remap to consume the
coupled topic instead.
"""
from __future__ import annotations

import json
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from onsen_robot_state import topics

from .joint_bridge import remap_joint_state
from .urdf import build_urdf

SPEC_PATH = os.environ.get("ROBOT_SPEC_PATH", "/ros2_ws/shared/robot_spec.json")
LATCHED = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class JointBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("joint_bridge_node")
        with open(SPEC_PATH) as f:
            spec = json.load(f)
        self._ratio = float(spec["arm"]["joint_ratio"])

        self._pub_desc = self.create_publisher(String, "/robot_description", LATCHED)
        self._pub_desc.publish(String(data=build_urdf(spec)))

        self._pub_js = self.create_publisher(JointState, "/joint_states_urdf", 10)
        self.create_subscription(JointState, topics.JOINT_STATES, self._on_js, 10)
        self.get_logger().info("JointBridge ready — URDF latched, coupled joints republished")

    def _on_js(self, msg: JointState) -> None:
        names, positions = remap_joint_state(list(msg.name), list(msg.position), self._ratio)
        out = JointState()
        out.header = msg.header
        out.name = names
        out.position = positions
        self._pub_js.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""Arm TF tree: joint_bridge (coupled joints) + robot_state_publisher.

move_group is not packaged on this ROS distro (docs/research_notes.md), so this
launch provides the arm's TF tree for RViz/Foxglove. The URDF is generated from
shared/robot_spec.json at launch and passed to robot_state_publisher as the
`robot_description` parameter (the node requires it non-empty at init — a
latched topic alone races the startup); the bridge also publishes it on
/robot_description for any topic-based consumer. robot_state_publisher consumes
/joint_states_urdf (the coupled joint angles the bridge republishes).
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from onsen_arm.urdf import load_urdf


def generate_launch_description():
    urdf = load_urdf()
    return LaunchDescription([
        Node(
            package="onsen_arm", executable="joint_bridge_node",
            name="joint_bridge_node", output="screen",
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="arm_state_publisher", output="screen",
            parameters=[{"robot_description": urdf}],
            remappings=[("/joint_states", "/joint_states_urdf")],
        ),
    ])

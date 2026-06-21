"""URDF generation for the onsen arm from shared/robot_spec.json — pure logic.

The arm has a fixed servo->joint coupling (joint tilt = ratio x servo offset).
A plain URDF revolute joint rotates 1:1 with its joint value, so to make
robot_state_publisher render the arm where it physically is from the FE's
`/joint_states` (which publishes *servo-offset* radians), the link rotation a
joint of value q produces must equal the real tilt delta. We therefore bake
the coupling into the joint by feeding robot_state_publisher coupled joint
values (see joint_bridge.py) and keep the URDF a clean 1:1 chain in tilt space.

Joint names match the FE exactly (frontend/src/sensors/jointStates.js):
  shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
  wrist_pitch_joint, wrist_roll_joint, gripper_joint

This is the model MoveIt would consume; on this ROS distro move_group is not
packaged (see docs/research_notes.md), so the URDF feeds robot_state_publisher
for the TF tree + RViz/Foxglove, and the analytic IK in onsen_ai_worker is the
runtime solver. Limits here are the planning limits MoveIt-class tools expect.
"""
from __future__ import annotations

import json
import math
import os

SPEC_PATH = os.environ.get("ROBOT_SPEC_PATH", "/ros2_ws/shared/robot_spec.json")

# pitch joints span servo 0..180 -> tilt +/- 90*ratio deg about neutral
PITCH_LIMIT = math.radians(90.0)  # before ratio; multiplied below
PAN_LIMIT = math.radians(90.0)


def _cyl(radius: float, length: float, rgba: str) -> str:
    return (
        f'<visual><geometry><cylinder radius="{radius}" length="{length}"/></geometry>'
        f'<origin xyz="0 0 {length / 2}"/><material name="m"><color rgba="{rgba}"/>'
        f"</material></visual>"
        f'<collision><geometry><cylinder radius="{radius}" length="{length}"/></geometry>'
        f'<origin xyz="0 0 {length / 2}"/></collision>'
    )


def build_urdf(spec: dict) -> str:
    arm = spec["arm"]
    links = arm["links"]
    base = arm["base_offset"]
    ratio = float(arm["joint_ratio"])
    shoulder_z = float(arm["shoulder_z"])
    upper, forearm, wrist = links["upper"], links["forearm"], links["wrist"]
    vel = math.radians(float(arm["max_joint_speed_dps"]))
    pitch_lim = PITCH_LIMIT * ratio

    def revolute(name, parent, child, xyz, axis, lower, upper_lim):
        return (
            f'<joint name="{name}" type="revolute">'
            f'<parent link="{parent}"/><child link="{child}"/>'
            f'<origin xyz="{xyz}"/><axis xyz="{axis}"/>'
            f'<limit lower="{lower:.4f}" upper="{upper_lim:.4f}" '
            f'effort="20" velocity="{vel:.3f}"/></joint>'
        )

    parts = [
        '<?xml version="1.0"?>',
        '<robot name="onsen_arm">',
        '<link name="arm_base_link"/>',
        # pan about Z at the arm base
        f'<link name="shoulder_link">{_cyl(0.03, upper, "0.95 0.94 0.90 1")}</link>',
        revolute("shoulder_pan_joint", "arm_base_link", "pan_link",
                 f"{base[0]} {base[1]} {shoulder_z}", "0 0 1", -PAN_LIMIT, PAN_LIMIT),
        '<link name="pan_link"/>',
        # the three pitch joints rotate about the local Y (tilt from vertical)
        revolute("shoulder_lift_joint", "pan_link", "shoulder_link",
                 "0 0 0", "0 1 0", -pitch_lim, pitch_lim),
        f'<link name="forearm_link">{_cyl(0.028, forearm, "0.95 0.94 0.90 1")}</link>',
        revolute("elbow_joint", "shoulder_link", "forearm_link",
                 f"0 0 {upper}", "0 1 0", -pitch_lim, pitch_lim),
        f'<link name="wrist_link">{_cyl(0.024, wrist, "0.27 0.27 0.30 1")}</link>',
        revolute("wrist_pitch_joint", "forearm_link", "wrist_link",
                 f"0 0 {forearm}", "0 1 0", -pitch_lim, pitch_lim),
        revolute("wrist_roll_joint", "wrist_link", "gripper_link",
                 f"0 0 {wrist}", "0 0 1", -math.pi, math.pi),
        '<link name="gripper_link"/>',
        # gripper finger as a prismatic-ish revolute placeholder (servo angle)
        revolute("gripper_joint", "gripper_link", "fingertip_link",
                 "0 0 0", "0 1 0", 0.0, math.radians(90.0)),
        '<link name="fingertip_link"/>',
        "</robot>",
    ]
    return "".join(parts)


def load_urdf() -> str:
    with open(SPEC_PATH) as f:
        return build_urdf(json.load(f))

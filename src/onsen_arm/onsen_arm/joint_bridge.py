"""Servo->joint coupling conversion for the URDF — pure logic.

The FE publishes /joint_states arm positions as *servo-offset* radians
(q_pub_i = (d_i - 90) * pi/180). The URDF joints rotate 1:1 in tilt space, so
robot_state_publisher needs the coupled tilt values:

  shoulder_pan   = q_pub[0]                  (pan, 1:1)
  shoulder_lift  = ratio * q_pub[1]          (absolute tilt t1)
  elbow          = -ratio * q_pub[2]         (relative tilt t2 - t1)
  wrist_pitch    = -ratio * q_pub[3]         (relative tilt t3 - t2)
  wrist_roll     = q_pub[4]                  (1:1)
  gripper        = clamp(q_pub[5] + pi/2, 0, pi/2)

Derived from the FK in frontend/src/robot/kinematics.js. Wheel joints pass
through unchanged.
"""
from __future__ import annotations

import math

ARM_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_pitch_joint", "wrist_roll_joint", "gripper_joint",
]


def couple_arm_positions(servo_offset_rad: list[float], ratio: float) -> list[float]:
    """Convert 6 FE servo-offset radians -> 6 URDF joint values (tilt space)."""
    q = servo_offset_rad
    return [
        q[0],
        ratio * q[1],
        -ratio * q[2],
        -ratio * q[3],
        q[4],
        max(0.0, min(math.pi / 2, q[5] + math.pi / 2)),
    ]


def remap_joint_state(
    names: list[str], positions: list[float], ratio: float,
) -> tuple[list[str], list[float]]:
    """Returns (names, positions) with arm joints coupled, others passed through."""
    out_names: list[str] = []
    out_pos: list[float] = []
    arm_idx = {n: i for i, n in enumerate(names) if n in ARM_JOINTS}
    if len(arm_idx) == len(ARM_JOINTS):
        servo = [positions[arm_idx[n]] for n in ARM_JOINTS]
        coupled = couple_arm_positions(servo, ratio)
        out_names.extend(ARM_JOINTS)
        out_pos.extend(coupled)
    for n, p in zip(names, positions, strict=False):
        if n not in ARM_JOINTS:
            out_names.append(n)
            out_pos.append(p)
    return out_names, out_pos

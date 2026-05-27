"""
6-DOF arm state machine.
Joints: shoulder_pan, shoulder_lift, elbow, wrist_pitch, wrist_roll, gripper

External control API (called by dummy_stream_node from /arm/action messages):
  force_state(state_name)          — jump to a named state immediately
  set_joint_angle(joint, value)    — pin one joint angle (overrides automation)
  clear_overrides()                — release all manual pins (resume automation)
"""
from __future__ import annotations

import math
import random
import time
from enum import Enum, auto
from typing import Any


class ArmState(Enum):
    HOME             = auto()
    SEARCH           = auto()
    APPROACH_OBJECT  = auto()
    LOWER_TO_TOWEL   = auto()
    GRIP             = auto()
    LIFT             = auto()
    DROP_TO_TRAY     = auto()
    FAILED_GRIP      = auto()


STATE_DURATIONS: dict[ArmState, float] = {
    ArmState.HOME:            3.0,
    ArmState.SEARCH:          4.0,
    ArmState.APPROACH_OBJECT: 3.5,
    ArmState.LOWER_TO_TOWEL:  2.5,
    ArmState.GRIP:            1.5,
    ArmState.LIFT:            2.0,
    ArmState.DROP_TO_TRAY:    2.0,
    ArmState.FAILED_GRIP:     2.0,
}

# 6 DOF: pan, lift, elbow, wrist_pitch, wrist_roll, gripper
STATE_JOINT_ANGLES: dict[ArmState, tuple[float, ...]] = {
    ArmState.HOME:            ( 0.00,  0.20,  0.30,  0.00,  0.00, 0.00),
    ArmState.SEARCH:          ( 0.40,  0.10,  0.50, -0.20,  0.10, 0.00),
    ArmState.APPROACH_OBJECT: ( 0.60,  0.40,  0.70, -0.30,  0.05, 0.00),
    ArmState.LOWER_TO_TOWEL:  ( 0.60,  0.70,  1.00, -0.50,  0.00, 0.00),
    ArmState.GRIP:            ( 0.60,  0.72,  1.02, -0.52,  0.00, 0.60),
    ArmState.LIFT:            ( 0.60,  0.35,  0.60, -0.30,  0.15, 0.60),
    ArmState.DROP_TO_TRAY:    (-0.50,  0.25,  0.45, -0.20,  0.30, 0.00),
    ArmState.FAILED_GRIP:     ( 0.40,  0.50,  0.70, -0.40, -0.10, 0.00),
}

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "gripper_joint",
]

STATE_SEQUENCE = [
    ArmState.HOME,
    ArmState.SEARCH,
    ArmState.APPROACH_OBJECT,
    ArmState.LOWER_TO_TOWEL,
    ArmState.GRIP,
    ArmState.LIFT,
    ArmState.DROP_TO_TRAY,
]


class ArmStateMachine:
    def __init__(self) -> None:
        self._state        = ArmState.HOME
        self._state_start  = time.monotonic()
        self._seq_idx      = 0
        self._cycle_id     = 0
        self._target_obj_id = "obj_001"
        self._rng          = random.Random()
        self._current_angles = list(STATE_JOINT_ANGLES[ArmState.HOME])
        # Manual overrides: joint_name → target angle (cleared on state advance)
        self._joint_overrides: dict[str, float] = {}

    # ── Public control API ────────────────────────────────────────────────────

    def force_state(self, state_name: str) -> None:
        """Jump immediately to a named arm state (from /arm/action command)."""
        try:
            target = ArmState[state_name.upper()]
        except KeyError:
            return
        self._state = target
        if target in STATE_SEQUENCE:
            self._seq_idx = STATE_SEQUENCE.index(target)
        self._state_start = time.monotonic()
        self._joint_overrides.clear()

    def set_joint_angle(self, joint_name: str, value: float) -> None:
        """Pin a single joint to a specific angle (from /arm/action command)."""
        if joint_name in JOINT_NAMES:
            self._joint_overrides[joint_name] = float(value)

    def clear_overrides(self) -> None:
        """Release all manual joint pins and resume automation."""
        self._joint_overrides.clear()

    # ── Tick ──────────────────────────────────────────────────────────────────

    def update(self, dt: float) -> tuple[dict[str, float], dict[str, Any]]:
        elapsed = time.monotonic() - self._state_start
        if elapsed >= STATE_DURATIONS[self._state]:
            self._advance_state()

        # Build target: start from state defaults, apply per-joint overrides
        target = list(STATE_JOINT_ANGLES[self._state])
        for name, val in self._joint_overrides.items():
            idx = JOINT_NAMES.index(name)
            target[idx] = val

        # Smooth interpolation
        for i in range(len(self._current_angles)):
            diff = target[i] - self._current_angles[i]
            self._current_angles[i] += diff * min(1.0, dt * 3.0)

        joint_dict = dict(zip(JOINT_NAMES, self._current_angles))
        state_dict: dict[str, Any] = {
            "state":               self._state.name,
            "cycle_id":            self._cycle_id,
            "target_object_id":    self._target_obj_id,
            "success_probability": round(self._success_probability(), 2),
            "manual_overrides":    list(self._joint_overrides.keys()),
        }
        return joint_dict, state_dict

    def get_state(self) -> ArmState:
        return self._state

    # ── Internal ──────────────────────────────────────────────────────────────

    def _advance_state(self) -> None:
        self._seq_idx = (self._seq_idx + 1) % len(STATE_SEQUENCE)
        if STATE_SEQUENCE[self._seq_idx] == ArmState.GRIP and self._rng.random() < 0.20:
            self._state = ArmState.FAILED_GRIP
        else:
            self._state = STATE_SEQUENCE[self._seq_idx]
        if self._state == ArmState.HOME:
            self._cycle_id     += 1
            self._target_obj_id = f"obj_{self._rng.randint(1, 8):03d}"
        self._state_start = time.monotonic()
        self._joint_overrides.clear()   # release manual pins on transition

    def _success_probability(self) -> float:
        base = {
            ArmState.HOME:            1.0,
            ArmState.SEARCH:          0.95,
            ArmState.APPROACH_OBJECT: 0.88,
            ArmState.LOWER_TO_TOWEL:  0.78,
            ArmState.GRIP:            0.72,
            ArmState.LIFT:            0.82,
            ArmState.DROP_TO_TRAY:    0.90,
            ArmState.FAILED_GRIP:     0.10,
        }.get(self._state, 0.5)
        return max(0.0, min(1.0, base + self._rng.uniform(-0.05, 0.05)))

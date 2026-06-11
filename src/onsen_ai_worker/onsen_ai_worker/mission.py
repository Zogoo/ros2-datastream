"""Mission state machine — pure logic, no ROS imports.

Closes the autonomy loop: SEARCH -> APPROACH -> PICK -> TO_BIN -> ALIGN_BIN ->
DROP -> SEARCH. Navigation uses simple bearing-pursuit; pick/drop geometry is
derived from the arm FK constants in shared/robot_spec.json (PICK_SCOOP
fingertip ~0.67 m ahead of base center, DROP point over the left-side basket
offset, reused to drop over map bins).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

SCOOP_FORWARD = 0.667     # m, fingertip ahead of base center at PICK_SCOOP
DROP_OFFSET = (0.27, 0.26)  # m, release point in base_link at DROP_RELEASE (pan 178)
PICK_TOL_X = 0.08
PICK_TOL_Y = 0.08
ARRIVE_TOL = 0.10
YAW_TOL = 0.08

PICK_SEQUENCE = [
    "A PRE_PICK", "A PICK_LOWER", "A PICK_SCOOP",
    "A PICK_GRIP", "A PICK_LIFT", "A PICK_RETRACT",
]
DROP_SEQUENCE = ["A DROP_BASKET", "A DROP_RELEASE", "A HOME"]


@dataclass
class MissionInput:
    pose: dict[str, float]                 # {x, y, yaw}
    towels: list[dict[str, Any]]           # world-frame towels (not held/binned)
    holding: bool
    arm_status: str                        # IDLE | MOVING | STOPPED
    safety_stop: bool
    mode: str                              # auto | manual
    min_front_obstacle: float | None = None


@dataclass
class MissionOutput:
    state: str
    twist: tuple[float, float] | None = None   # (vx, wz); None = no command
    arm_command: str | None = None
    target_id: str | None = None
    reason: str = ""


@dataclass
class _SeqState:
    commands: list[str] = field(default_factory=list)
    index: int = 0
    sent: bool = False
    settle: int = 0


class MissionLogic:
    def __init__(self, bin_center: tuple[float, float]) -> None:
        self.bin_center = bin_center
        self.state = "IDLE"
        self.target: dict[str, Any] | None = None
        self._seq = _SeqState()
        self._search_spin = 0.0

    def update(self, inp: MissionInput) -> MissionOutput:
        if inp.safety_stop:
            return self._goto_state("IDLE", reason="safety stop latched")
        if inp.mode != "auto":
            return self._goto_state("IDLE", reason="manual mode")

        handler = getattr(self, f"_state_{self.state.lower()}", None)
        if handler is None:
            return self._goto_state("IDLE", reason=f"unknown state {self.state}")
        return handler(inp)

    # ── States ────────────────────────────────────────────────────────────────

    def _state_idle(self, inp: MissionInput) -> MissionOutput:
        self.state = "SEARCH"
        return MissionOutput(state="IDLE", reason="resuming search")

    def _state_search(self, inp: MissionInput) -> MissionOutput:
        if inp.holding:
            self.state = "TO_BIN"
            return MissionOutput(state="SEARCH", reason="already holding — deliver")
        if inp.towels:
            self.target = nearest(inp.towels, inp.pose)
            self.state = "APPROACH"
            return MissionOutput(
                state="SEARCH", target_id=self.target["id"],
                reason=f"towel {self.target['id']} selected",
            )
        self._search_spin += 1
        return MissionOutput(state="SEARCH", twist=(0.0, 0.45), reason="scanning for towels")

    def _state_approach(self, inp: MissionInput) -> MissionOutput:
        towel = self._refresh_target(inp)
        if towel is None:
            self.state = "SEARCH"
            return MissionOutput(state="APPROACH", reason="target lost")
        rel = world_to_robot(towel["position"], inp.pose)
        if abs(rel[0] - SCOOP_FORWARD) < PICK_TOL_X and abs(rel[1]) < PICK_TOL_Y:
            self.state = "PICK"
            self._seq = _SeqState(commands=list(PICK_SEQUENCE))
            return MissionOutput(state="APPROACH", twist=(0.0, 0.0), reason="in pick window")

    # navigate toward the stand-off point that places the towel in the pick window
        goal = standoff_point(
            (towel["position"]["x"], towel["position"]["y"]),
            (inp.pose["x"], inp.pose["y"]),
            SCOOP_FORWARD,
        )
        twist = pursue(inp.pose, goal, inp.min_front_obstacle)
        return MissionOutput(
            state="APPROACH", twist=twist, target_id=towel["id"],
            reason=f"driving to towel (rel x={rel[0]:.2f} y={rel[1]:.2f})",
        )

    def _state_pick(self, inp: MissionInput) -> MissionOutput:
        out = self._run_sequence(inp, "PICK")
        if out is not None:
            return out
        self.state = "TO_BIN" if inp.holding else "SEARCH"
        reason = "towel grasped — delivering" if inp.holding else "grasp failed — retry"
        return MissionOutput(state="PICK", reason=reason)

    def _state_to_bin(self, inp: MissionInput) -> MissionOutput:
        if not inp.holding:
            self.state = "SEARCH"
            return MissionOutput(state="TO_BIN", reason="payload lost")
        pos = (inp.pose["x"], inp.pose["y"])
        standoff = math.hypot(*DROP_OFFSET)
        goal = standoff_point(self.bin_center, pos, standoff)
        dist = math.hypot(goal[0] - pos[0], goal[1] - pos[1])
        if dist < ARRIVE_TOL:
            self.state = "ALIGN_BIN"
            return MissionOutput(state="TO_BIN", twist=(0.0, 0.0), reason="at bin standoff")
        return MissionOutput(
            state="TO_BIN", twist=pursue(inp.pose, goal, inp.min_front_obstacle),
            reason=f"driving to bin ({dist:.2f} m)",
        )

    def _state_align_bin(self, inp: MissionInput) -> MissionOutput:
        v = (self.bin_center[0] - inp.pose["x"], self.bin_center[1] - inp.pose["y"])
        beta = math.atan2(DROP_OFFSET[1], DROP_OFFSET[0])
        yaw_target = math.atan2(v[1], v[0]) - beta
        err = wrap_angle(yaw_target - inp.pose["yaw"])
        if abs(err) < YAW_TOL:
            self.state = "DROP"
            self._seq = _SeqState(commands=list(DROP_SEQUENCE))
            return MissionOutput(state="ALIGN_BIN", twist=(0.0, 0.0), reason="aligned with bin")
        wz = max(-0.8, min(0.8, 1.5 * err))
        return MissionOutput(state="ALIGN_BIN", twist=(0.0, wz), reason=f"rotating ({err:.2f} rad)")

    def _state_drop(self, inp: MissionInput) -> MissionOutput:
        out = self._run_sequence(inp, "DROP")
        if out is not None:
            return out
        self.state = "SEARCH"
        return MissionOutput(state="DROP", reason="drop complete — searching")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _run_sequence(self, inp: MissionInput, label: str) -> MissionOutput | None:
        """Steps through the queued arm commands, one per IDLE settle. Returns
        None when the sequence is finished."""
        seq = self._seq
        if seq.index >= len(seq.commands):
            return None
        if seq.sent:
            if inp.arm_status == "MOVING":
                seq.settle = 0
                return MissionOutput(
                    state=label, twist=(0.0, 0.0), reason=f"arm {seq.commands[seq.index]}",
                )
            seq.settle += 1
            if seq.settle < 3:  # debounce IDLE between segments
                return MissionOutput(state=label, twist=(0.0, 0.0), reason="settling")
            seq.index += 1
            seq.sent = False
            seq.settle = 0
            if seq.index >= len(seq.commands):
                return None
        command = seq.commands[seq.index]
        seq.sent = True
        return MissionOutput(state=label, twist=(0.0, 0.0), arm_command=command, reason=command)

    def _refresh_target(self, inp: MissionInput) -> dict[str, Any] | None:
        if self.target is None:
            return None
        for towel in inp.towels:
            if towel["id"] == self.target["id"]:
                self.target = towel
                return towel
        return None

    def _goto_state(self, state: str, reason: str) -> MissionOutput:
        if self.state != state:
            self.state = state
            self._seq = _SeqState()
        return MissionOutput(state=state, twist=None, reason=reason)


def nearest(towels: list[dict[str, Any]], pose: dict[str, float]) -> dict[str, Any]:
    return min(towels, key=lambda t: math.hypot(
        t["position"]["x"] - pose["x"], t["position"]["y"] - pose["y"],
    ))


def world_to_robot(position: dict[str, float], pose: dict[str, float]) -> tuple[float, float]:
    dx = position["x"] - pose["x"]
    dy = position["y"] - pose["y"]
    cos_y, sin_y = math.cos(-pose["yaw"]), math.sin(-pose["yaw"])
    return (dx * cos_y - dy * sin_y, dx * sin_y + dy * cos_y)


def standoff_point(
    target: tuple[float, float], robot: tuple[float, float], distance: float,
) -> tuple[float, float]:
    dx = target[0] - robot[0]
    dy = target[1] - robot[1]
    d = math.hypot(dx, dy) or 1e-6
    return (target[0] - dx / d * distance, target[1] - dy / d * distance)


def pursue(
    pose: dict[str, float], goal: tuple[float, float],
    min_front_obstacle: float | None,
) -> tuple[float, float]:
    dx = goal[0] - pose["x"]
    dy = goal[1] - pose["y"]
    dist = math.hypot(dx, dy)
    err = wrap_angle(math.atan2(dy, dx) - pose["yaw"])
    if abs(err) > 0.35:
        return (0.0, max(-0.9, min(0.9, 2.0 * err)))
    vx = max(0.06, min(0.30, 0.6 * dist))
    if min_front_obstacle is not None and min_front_obstacle < 0.45:
        vx = min(vx, 0.08)
    return (vx, max(-0.8, min(0.8, 1.4 * err)))


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

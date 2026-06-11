"""6-axis servo arm firmware emulation: command parsing, limits, calibration,
and time-based joint interpolation. Pure logic — no ROS imports — so AI
engineers can unit-test or embed it directly.

Command grammar (one command per line, mirrors the real controller firmware):
  Q                          query state -> "STATE j0 j1 j2 j3 j4 j5 IDLE|MOVING|STOPPED"
  A <ACTION>                 named pose / scripted action
  J d0 d1 d2 d3 d4 d5 ms     absolute move, degrees
  D joint delta ms           relative single-joint move
  M <ALIAS> amount ms        named relative move (BASE_LEFT, ARM_DOWN, ...)
  G pos ms                   gripper absolute, degrees
  SPEED pct                  global speed scale 1..100
  STOP                       halt + latch error until A RESET_ERROR
  CAL SHOW|SET|DEG|SAVE|LOAD|RESET
  RELAX [j] / WAKE [j]       cut / restore servo PWM
"""
from __future__ import annotations

import json
import os
import time

NUM_JOINTS = 6
GRIPPER = 5

HOME_POSE = [90.0, 90.0, 90.0, 90.0, 90.0, 70.0]

# Pose table tuned against the simulator FK (shared/robot_spec.json: joint_ratio
# 1.5, links 0.25/0.25/0.20 m, shoulder at z=0.50). PICK_SCOOP puts the
# fingertip ~4 cm above the floor 0.41 m ahead of the arm base; DROP_BASKET
# hovers over the side basket at pan 178. Verified by frontend/tests/kinematics.test.js.
ACTIONS: dict[str, list[float] | None] = {
    "HOME":          [90, 90, 90, 90, 90, 70],
    "READY":         [90, 110, 80, 75, 90, 70],
    "STOW":          [90, 30, 90, 90, 90, 10],
    "SEARCH_LEFT":   [140, 110, 80, 75, 90, 70],
    "SEARCH_CENTER": [90, 110, 80, 75, 90, 70],
    "SEARCH_RIGHT":  [40, 110, 80, 75, 90, 70],
    "PRE_PICK":      [90, 130, 57, 63, 90, 80],
    "PICK_APPROACH": [90, 140, 57, 67, 90, 80],
    "PICK_LOWER":    [90, 150, 57, 73, 90, 80],
    "PICK_SCOOP":    [90, 157, 57, 77, 90, 80],
    "PICK_GRIP":     [90, 157, 57, 77, 90, 12],
    "PICK_LIFT":     [90, 110, 57, 77, 90, 12],
    "PICK_RETRACT":  [90, 95, 80, 70, 90, 12],
    "DROP_BASKET":   [178, 100, 85, 85, 90, 12],
    "DROP_RELEASE":  [178, 100, 85, 85, 90, 75],
    "OPEN_GRIPPER":  None,
    "CLOSE_GRIPPER": None,
    "GRIPPER_HALF":  None,
    "SHAKE_LIGHT":   None,
    "RESET_ERROR":   None,
}

MOVE_ALIASES: dict[str, tuple[int, float]] = {
    "BASE_LEFT":          (0, +1), "BASE_RIGHT":        (0, -1),
    "ARM_UP":             (1, -1), "ARM_DOWN":          (1, +1),
    "ELBOW_UP":           (2, +1), "ELBOW_DOWN":        (2, -1),
    "WRIST_UP":           (3, +1), "WRIST_DOWN":        (3, -1),
    "WRIST_CW":           (4, +1), "WRIST_CCW":         (4, -1),
    "GRIPPER_CLOSE_STEP": (5, -1), "GRIPPER_OPEN_STEP": (5, +1),
}

DEFAULT_CAL: dict[str, list] = {
    "pulse_min": [500] * NUM_JOINTS,
    "pulse_max": [2500] * NUM_JOINTS,
    "deg_min":   [0.0] * NUM_JOINTS,
    "deg_max":   [180.0] * NUM_JOINTS,
}


class ArmFirmware:
    """Stateful arm controller. Call tick() at a fixed rate; handle() per command."""

    def __init__(self, cal_path: str | None = None) -> None:
        self._cal_path = cal_path
        self._cal = {k: list(v) for k, v in DEFAULT_CAL.items()}
        self._pos = list(HOME_POSE)
        self._segments: list[tuple[list[float], float]] = []  # (target_deg, duration_s)
        self._seg_start_pos: list[float] = list(self._pos)
        self._seg_start_t = 0.0
        self._speed_pct = 100
        self._stopped = False
        self._relaxed = [False] * NUM_JOINTS
        self._last_action = "HOME"
        if cal_path and os.path.exists(cal_path):
            self._load_cal()

    # ── Public state ──────────────────────────────────────────────────────────

    @property
    def positions(self) -> list[float]:
        return list(self._pos)

    @property
    def status(self) -> str:
        if self._stopped:
            return "STOPPED"
        return "MOVING" if self._segments else "IDLE"

    def state_dict(self) -> dict:
        return {
            "joints_deg": [round(p, 2) for p in self._pos],
            "status": self.status,
            "speed_pct": self._speed_pct,
            "relaxed": [i for i, r in enumerate(self._relaxed) if r],
            "last_action": self._last_action,
            "queue_depth": len(self._segments),
        }

    # ── Tick ──────────────────────────────────────────────────────────────────

    def tick(self, now: float | None = None) -> None:
        if self._stopped or not self._segments:
            return
        now = time.monotonic() if now is None else now
        target, duration = self._segments[0]
        t = 1.0 if duration <= 0 else min(1.0, (now - self._seg_start_t) / duration)
        for i in range(NUM_JOINTS):
            if self._relaxed[i]:
                continue
            self._pos[i] = self._seg_start_pos[i] + (target[i] - self._seg_start_pos[i]) * t
        if t >= 1.0:
            self._segments.pop(0)
            if self._segments:
                self._seg_start_pos = list(self._pos)
                self._seg_start_t = now

    # ── Command handling ──────────────────────────────────────────────────────

    def handle(self, line: str) -> list[str]:
        parts = line.strip().split()
        if not parts:
            return ["ERR EMPTY"]
        op = parts[0].upper()
        try:
            handler = {
                "Q": self._cmd_q, "A": self._cmd_a, "J": self._cmd_j,
                "D": self._cmd_d, "M": self._cmd_m, "G": self._cmd_g,
                "SPEED": self._cmd_speed, "STOP": self._cmd_stop,
                "CAL": self._cmd_cal, "RELAX": self._cmd_relax, "WAKE": self._cmd_wake,
            }.get(op)
            if handler is None:
                return [f"ERR UNKNOWN_CMD {op}"]
            return handler(parts[1:])
        except (ValueError, IndexError):
            return [f"ERR BAD_ARGS {line.strip()}"]

    # ── Individual commands ───────────────────────────────────────────────────

    def _cmd_q(self, _args: list[str]) -> list[str]:
        joints = " ".join(str(round(p)) for p in self._pos)
        return [f"STATE {joints} {self.status}"]

    def _cmd_a(self, args: list[str]) -> list[str]:
        name = args[0].upper()
        if name == "RESET_ERROR":
            self._stopped = False
            return ["OK RESET_ERROR"]
        if name not in ACTIONS:
            return [f"ERR UNKNOWN_ACTION {name}"]
        if self._stopped:
            return ["ERR STOPPED"]
        self._last_action = name
        if name == "OPEN_GRIPPER":
            target = self._cal["deg_max"][GRIPPER] - 100.0 + 95.0
            return self._move_single(GRIPPER, target, 400, f"OK ACTION {name}")
        if name == "CLOSE_GRIPPER":
            target = self._cal["deg_min"][GRIPPER] + 10.0
            return self._move_single(GRIPPER, target, 400, f"OK ACTION {name}")
        if name == "GRIPPER_HALF":
            mid = (self._cal["deg_min"][GRIPPER] + self._cal["deg_max"][GRIPPER]) / 2.0
            return self._move_single(GRIPPER, mid, 400, f"OK ACTION {name}")
        if name == "SHAKE_LIGHT":
            base = list(self._pos)
            for delta in (8, -16, 16, -8):
                wig = list(base)
                wig[4] = self._clamp_deg(4, base[4] + delta)
                self._queue(wig, 150)
            self._queue(base, 150)
            return ["OK ACTION SHAKE_LIGHT"]
        pose_def = ACTIONS[name]
        assert pose_def is not None  # scripted actions handled above
        pose = [self._clamp_deg(i, v) for i, v in enumerate(pose_def)]
        self._queue(pose, 900)
        return [f"OK ACTION {name}"]

    def _cmd_j(self, args: list[str]) -> list[str]:
        if self._stopped:
            return ["ERR STOPPED"]
        vals = [float(v) for v in args[:NUM_JOINTS]]
        ms = float(args[NUM_JOINTS]) if len(args) > NUM_JOINTS else 800.0
        for i, v in enumerate(vals):
            if not self._within(i, v):
                return [f"ERR LIMIT joint={i} value={int(v)}"]
        self._queue(vals, ms)
        return ["OK J " + " ".join(str(int(v)) for v in vals)]

    def _cmd_d(self, args: list[str]) -> list[str]:
        if self._stopped:
            return ["ERR STOPPED"]
        joint, delta = int(args[0]), float(args[1])
        ms = float(args[2]) if len(args) > 2 else 300.0
        if not 0 <= joint < NUM_JOINTS:
            return [f"ERR BAD_JOINT {joint}"]
        target = self._goal_pos()[joint] + delta
        if not self._within(joint, target):
            return [f"ERR LIMIT joint={joint} value={int(target)}"]
        res = self._move_single(joint, target, ms, f"OK D joint={joint} delta={_fmt(delta)}")
        return res

    def _cmd_m(self, args: list[str]) -> list[str]:
        if self._stopped:
            return ["ERR STOPPED"]
        alias = args[0].upper()
        if alias not in MOVE_ALIASES:
            return [f"ERR UNKNOWN_MOVE {alias}"]
        amount = float(args[1]) if len(args) > 1 else 5.0
        ms = float(args[2]) if len(args) > 2 else 300.0
        joint, sign = MOVE_ALIASES[alias]
        target = self._clamp_deg(joint, self._goal_pos()[joint] + sign * amount)
        self._move_single(joint, target, ms, "")
        return [f"OK M {alias} amount={_fmt(amount)}"]

    def _cmd_g(self, args: list[str]) -> list[str]:
        if self._stopped:
            return ["ERR STOPPED"]
        pos = float(args[0])
        ms = float(args[1]) if len(args) > 1 else 300.0
        if not self._within(GRIPPER, pos):
            return [f"ERR LIMIT joint={GRIPPER} value={int(pos)}"]
        self._move_single(GRIPPER, pos, ms, "")
        return [f"OK G {_fmt(pos)}"]

    def _cmd_speed(self, args: list[str]) -> list[str]:
        pct = int(args[0])
        if not 1 <= pct <= 100:
            return [f"ERR BAD_SPEED {pct}"]
        self._speed_pct = pct
        return [f"OK SPEED {pct}"]

    def _cmd_stop(self, _args: list[str]) -> list[str]:
        self._segments.clear()
        self._stopped = True
        return ["OK STOP"]

    def _cmd_cal(self, args: list[str]) -> list[str]:
        sub = args[0].upper() if args else "SHOW"
        if sub == "SHOW":
            lines = []
            for i in range(NUM_JOINTS):
                lines.append(
                    f"CAL joint={i} "
                    f"pulse=[{self._cal['pulse_min'][i]},{self._cal['pulse_max'][i]}] "
                    f"deg=[{_fmt(self._cal['deg_min'][i])},{_fmt(self._cal['deg_max'][i])}]"
                )
            return lines
        if sub == "SET":
            j, lo, hi = int(args[1]), int(args[2]), int(args[3])
            if not (0 <= j < NUM_JOINTS and 400 <= lo < hi <= 2600):
                return ["ERR BAD_CAL"]
            self._cal["pulse_min"][j], self._cal["pulse_max"][j] = lo, hi
            return [f"OK CAL SET joint={j} pulse=[{lo},{hi}]"]
        if sub == "DEG":
            j, dlo, dhi = int(args[1]), float(args[2]), float(args[3])
            if not (0 <= j < NUM_JOINTS and 0 <= dlo < dhi <= 180):
                return ["ERR BAD_CAL"]
            self._cal["deg_min"][j], self._cal["deg_max"][j] = dlo, dhi
            return [f"OK CAL DEG joint={j} deg=[{_fmt(dlo)},{_fmt(dhi)}]"]
        if sub == "SAVE":
            self._save_cal()
            return ["OK CAL SAVE"]
        if sub == "LOAD":
            self._load_cal()
            return ["OK CAL LOAD"]
        if sub == "RESET":
            self._cal = {k: list(v) for k, v in DEFAULT_CAL.items()}
            return ["OK CAL RESET"]
        return [f"ERR UNKNOWN_CAL {sub}"]

    def _cmd_relax(self, args: list[str]) -> list[str]:
        if args:
            j = int(args[0])
            if not 0 <= j < NUM_JOINTS:
                return [f"ERR BAD_JOINT {j}"]
            self._relaxed[j] = True
            return [f"OK RELAX {j}"]
        self._relaxed = [True] * NUM_JOINTS
        return ["OK RELAX ALL"]

    def _cmd_wake(self, args: list[str]) -> list[str]:
        if args:
            j = int(args[0])
            if not 0 <= j < NUM_JOINTS:
                return [f"ERR BAD_JOINT {j}"]
            self._relaxed[j] = False
            return [f"OK WAKE {j}"]
        self._relaxed = [False] * NUM_JOINTS
        return ["OK WAKE ALL"]

    # ── Internals ─────────────────────────────────────────────────────────────

    def _goal_pos(self) -> list[float]:
        return list(self._segments[-1][0]) if self._segments else list(self._pos)

    def _queue(self, target: list[float], ms: float) -> None:
        duration = (ms / 1000.0) * (100.0 / self._speed_pct)
        if not self._segments:
            self._seg_start_pos = list(self._pos)
            self._seg_start_t = time.monotonic()
        self._segments.append(([float(v) for v in target], duration))

    def _move_single(self, joint: int, value: float, ms: float, ok: str) -> list[str]:
        target = self._goal_pos()
        target[joint] = self._clamp_deg(joint, value)
        self._queue(target, ms)
        return [ok] if ok else []

    def _within(self, joint: int, value: float) -> bool:
        return self._cal["deg_min"][joint] <= value <= self._cal["deg_max"][joint]

    def _clamp_deg(self, joint: int, value: float) -> float:
        return max(self._cal["deg_min"][joint], min(self._cal["deg_max"][joint], value))

    def _save_cal(self) -> None:
        if not self._cal_path:
            return
        os.makedirs(os.path.dirname(self._cal_path), exist_ok=True)
        with open(self._cal_path, "w") as f:
            json.dump(self._cal, f, indent=2)

    def _load_cal(self) -> None:
        if not (self._cal_path and os.path.exists(self._cal_path)):
            return
        with open(self._cal_path) as f:
            saved = json.load(f)
        for key in DEFAULT_CAL:
            if key in saved and len(saved[key]) == NUM_JOINTS:
                self._cal[key] = list(saved[key])


def _fmt(v: float) -> str:
    return str(int(v)) if float(v).is_integer() else f"{v:g}"

"""6-wheel skid-steer base firmware emulation — pure logic, no ROS imports.

Wheel indexing: 0..2 left (front,mid,rear), 3..5 right (front,mid,rear).

Protocol (one command per line):
  Q                  -> "STATE vx wz w0..w5 IDLE|MOVING|STOPPED|SAFETY"
  V vx wz            twist command (m/s, rad/s)
  T left right       per-side surface speed (m/s)
  W i radps          single wheel angular velocity
  SPEED pct          global scale 1..100
  STOP               halt + latch error
  RESET_ERROR        clear STOP latch (safety latch clears only via safety input)
"""
from __future__ import annotations

import time

WHEEL_RADIUS = 0.07     # m
TRACK_WIDTH = 0.47      # m
MAX_WHEEL_RADPS = 12.0  # ~0.84 m/s surface speed
CMD_TIMEOUT_S = 1.0     # zero output if no twist refresh


class BaseFirmware:
    def __init__(self) -> None:
        self._wheels = [0.0] * 6
        self._mode = "twist"
        self._vx = 0.0
        self._wz = 0.0
        self._speed_pct = 100
        self._stopped = False
        self._safety = False
        self._last_cmd_t = -999.0

    # ── Inputs ────────────────────────────────────────────────────────────────

    def set_safety(self, engaged: bool) -> bool:
        """Returns True when the safety latch newly engages."""
        newly = engaged and not self._safety
        if newly:
            self._zero()
        self._safety = engaged
        return newly

    def set_twist(self, vx: float, wz: float) -> None:
        if self._stopped or self._safety:
            return
        self._mode = "twist"
        self._vx, self._wz = vx, wz
        left = vx - wz * TRACK_WIDTH / 2.0
        right = vx + wz * TRACK_WIDTH / 2.0
        self._set_sides(left, right, keep_twist=True)

    def handle(self, line: str) -> list[str]:
        parts = line.strip().split()
        if not parts:
            return ["ERR EMPTY"]
        op = parts[0].upper()
        try:
            if op == "Q":
                w = " ".join(f"{v:.2f}" for v in self.scaled_wheels())
                return [f"STATE {self._vx:.2f} {self._wz:.2f} {w} {self.status}"]
            if op == "STOP":
                self._zero()
                self._stopped = True
                return ["OK STOP"]
            if op == "RESET_ERROR":
                self._stopped = False
                return ["OK RESET_ERROR"]
            if op == "SPEED":
                pct = int(parts[1])
                if not 1 <= pct <= 100:
                    return [f"ERR BAD_SPEED {pct}"]
                self._speed_pct = pct
                return [f"OK SPEED {pct}"]
            if self._stopped:
                return ["ERR STOPPED"]
            if self._safety:
                return ["ERR SAFETY_STOP"]
            if op == "V":
                self.set_twist(float(parts[1]), float(parts[2]))
                return [f"OK V {parts[1]} {parts[2]}"]
            if op == "T":
                self._set_sides(float(parts[1]), float(parts[2]))
                return [f"OK T {parts[1]} {parts[2]}"]
            if op == "W":
                i, radps = int(parts[1]), float(parts[2])
                if not 0 <= i < 6:
                    return [f"ERR BAD_WHEEL {i}"]
                if abs(radps) > MAX_WHEEL_RADPS:
                    return [f"ERR LIMIT wheel={i} value={radps:g}"]
                self._mode = "wheel"
                self._wheels[i] = radps
                self._last_cmd_t = time.monotonic()
                return [f"OK W {i} {radps:g}"]
            return [f"ERR UNKNOWN_CMD {op}"]
        except (ValueError, IndexError):
            return [f"ERR BAD_ARGS {line.strip()}"]

    # ── State ─────────────────────────────────────────────────────────────────

    @property
    def status(self) -> str:
        if self._safety:
            return "SAFETY"
        if self._stopped:
            return "STOPPED"
        return "MOVING" if any(abs(w) > 1e-3 for w in self.scaled_wheels()) else "IDLE"

    def scaled_wheels(self, now: float | None = None) -> list[float]:
        if self._safety or self._stopped:
            return [0.0] * 6
        now = time.monotonic() if now is None else now
        if self._mode == "twist" and now - self._last_cmd_t > CMD_TIMEOUT_S:
            return [0.0] * 6
        scale = self._speed_pct / 100.0
        return [w * scale for w in self._wheels]

    def state_dict(self) -> dict:
        return {
            "status": self.status,
            "mode": self._mode,
            "vx": round(self._vx, 3),
            "wz": round(self._wz, 3),
            "speed_pct": self._speed_pct,
            "wheels_radps": [round(w, 3) for w in self.scaled_wheels()],
        }

    # ── Internals ─────────────────────────────────────────────────────────────

    def _set_sides(self, left: float, right: float, keep_twist: bool = False) -> None:
        if not keep_twist:
            self._vx = (left + right) / 2.0
            self._wz = (right - left) / TRACK_WIDTH
            self._mode = "twist"
        wl = _clamp(left / WHEEL_RADIUS, MAX_WHEEL_RADPS)
        wr = _clamp(right / WHEEL_RADIUS, MAX_WHEEL_RADPS)
        self._wheels = [wl, wl, wl, wr, wr, wr]
        self._last_cmd_t = time.monotonic()

    def _zero(self) -> None:
        self._wheels = [0.0] * 6
        self._vx = self._wz = 0.0


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))

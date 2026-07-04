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
  BIN DUMP|HOME|Q    collect-bin dump servo: tilt to dump angle / stow / query

The collect-bin channel also carries the load-cell feedback: the chassis has a
single-point strain-gauge load cell + HX711 under the bin floor (the standard
open-hardware weighing stack); observe_bin_load() feeds the measured kg in and
the firmware debounce-thresholds it into bin_full.
"""
from __future__ import annotations

import time

WHEEL_RADIUS = 0.07     # m
TRACK_WIDTH = 0.47      # m
MAX_WHEEL_RADPS = 12.0  # ~0.84 m/s surface speed
CMD_TIMEOUT_S = 1.0     # zero output if no twist refresh

# Collect-bin dump servo + load cell (mirrors shared/robot_spec.json basket).
BIN_DUMP_DEG = 110.0
BIN_TILT_SPEED_DPS = 90.0
# bin_full threshold: ~3 towels (0.25 kg each) — the top-deck tray holds more,
# but delivering at 3 keeps trip lengths sensible.
BIN_FULL_KG = 0.70
BIN_LOAD_DEBOUNCE_S = 1.0


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
        # Collect-bin dump servo (interpolated) + load-cell channel.
        self._bin_target = 0.0
        self._bin_tilt = 0.0
        self._bin_tilt_at = time.monotonic()
        self._bin_kg = 0.0
        self._bin_over_since: float | None = None
        self._bin_full = False

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
            if op == "BIN":
                sub = parts[1].upper() if len(parts) > 1 else "Q"
                if sub == "DUMP":
                    self._set_bin_target(BIN_DUMP_DEG)
                    return ["OK BIN DUMP"]
                if sub == "HOME":
                    self._set_bin_target(0.0)
                    return ["OK BIN HOME"]
                if sub == "Q":
                    return [
                        f"BIN {self.bin_tilt():.0f} {self._bin_kg:.3f} "
                        f"{'FULL' if self._bin_full else 'OK'}",
                    ]
                return [f"ERR BAD_BIN {sub}"]
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

    def _set_bin_target(self, deg: float, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        # Anchor the interpolation at the CURRENT tilt so re-commands mid-swing
        # don't teleport the estimate.
        self._bin_tilt = self.bin_tilt(now)
        self._bin_tilt_at = now
        self._bin_target = deg

    def bin_tilt(self, now: float | None = None) -> float:
        """Estimated dump-servo angle: interpolates from the last anchor toward
        the target at the servo's rated speed (deterministic, no timer task)."""
        now = time.monotonic() if now is None else now
        delta = self._bin_target - self._bin_tilt
        if delta == 0:
            return self._bin_tilt
        travelled = BIN_TILT_SPEED_DPS * (now - self._bin_tilt_at)
        if travelled >= abs(delta):
            return self._bin_target
        return self._bin_tilt + travelled * (1 if delta > 0 else -1)

    def observe_bin_load(self, kg: float, now: float | None = None) -> None:
        """Feed the measured load-cell weight (kg). bin_full latches after the
        reading stays over BIN_FULL_KG for BIN_LOAD_DEBOUNCE_S — one noisy or
        transient spike (a towel bouncing in) must not trigger a delivery."""
        now = time.monotonic() if now is None else now
        self._bin_kg = kg
        if kg < BIN_FULL_KG:
            self._bin_over_since = None
            self._bin_full = False
            return
        if self._bin_over_since is None:
            self._bin_over_since = now
        self._bin_full = (now - self._bin_over_since) >= BIN_LOAD_DEBOUNCE_S

    def state_dict(self) -> dict:
        return {
            "status": self.status,
            "mode": self._mode,
            "vx": round(self._vx, 3),
            "wz": round(self._wz, 3),
            "speed_pct": self._speed_pct,
            "wheels_radps": [round(w, 3) for w in self.scaled_wheels()],
            "bin_tilt_target_deg": round(self._bin_target, 1),
            "bin_tilt_deg": round(self.bin_tilt(), 1),
            "bin_kg": round(self._bin_kg, 3),
            "bin_full": self._bin_full,
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

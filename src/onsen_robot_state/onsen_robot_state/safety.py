"""Pure safety supervision logic — no ROS imports, fully unit-testable.

Latching rules (mirrors real AMR safety PLCs):
  - contact impulse >= threshold      -> STOP, latched
  - critical contact (water ingress)  -> STOP, latched as CRITICAL
  - tilt beyond limit                 -> STOP, latched while tilted
  - reset() clears latches only when no hazard is currently active
"""
from __future__ import annotations

import math
import time
from collections.abc import Sequence
from dataclasses import dataclass, field

# Arm pan 160-180 deg (DROP poses) puts the wrist in the LIDAR plane on the
# robot's left flank; pan deg - 90 = scan-frame bearing deg.
ARM_DROP_SECTOR_DEG = (70.0, 90.0)


def arm_in_drop_phase(arm_state: dict) -> bool:
    return str(arm_state.get("last_action", "")).startswith("DROP")


def mask_arm_sector(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    sector_deg: tuple[float, float] = ARM_DROP_SECTOR_DEG,
) -> list[float]:
    """Self-filter: invalidate beams that would hit the robot's own arm."""
    lo, hi = (math.radians(d) for d in sector_deg)
    return [
        math.inf if lo <= angle_min + i * angle_increment <= hi else r
        for i, r in enumerate(ranges)
    ]


@dataclass
class SafetyEvent:
    reason: str
    detail: dict
    ts: float = field(default_factory=time.time)


class SafetyMonitor:
    def __init__(self, impulse_threshold: float, tilt_limit_deg: float) -> None:
        self.impulse_threshold = impulse_threshold
        self.tilt_limit_deg = tilt_limit_deg
        self._latched = False
        self._critical = False
        self._tilt_active = False
        self._events: list[SafetyEvent] = []

    @property
    def stop(self) -> bool:
        return self._latched or self._tilt_active

    @property
    def critical(self) -> bool:
        return self._critical

    def on_contact(self, impulse: float, critical: bool, detail: dict | None = None) -> bool:
        """Returns True when this contact triggers (or re-triggers) the stop."""
        if critical:
            self._latched = True
            self._critical = True
            self._events.append(SafetyEvent("CRITICAL_CONTACT", detail or {}))
            return True
        if impulse >= self.impulse_threshold:
            triggered = not self._latched
            self._latched = True
            if triggered:
                self._events.append(SafetyEvent("IMPACT", detail or {"impulse": impulse}))
            return triggered
        return False

    def on_tilt(self, tilt_deg: float) -> bool:
        was = self._tilt_active
        self._tilt_active = tilt_deg >= self.tilt_limit_deg
        if self._tilt_active and not was:
            self._events.append(SafetyEvent("TILT", {"tilt_deg": round(tilt_deg, 1)}))
            self._latched = True
            return True
        return False

    def reset(self) -> bool:
        """Operator reset. Refused while a hazard is still present."""
        if self._tilt_active:
            return False
        self._latched = False
        self._critical = False
        return True

    def force_clear(self) -> None:
        """Unconditional clear — used when the e-stop feature is disarmed."""
        self._latched = False
        self._critical = False
        self._tilt_active = False
        self._events = []

    def drain_events(self) -> list[SafetyEvent]:
        events = self._events
        self._events = []
        return events

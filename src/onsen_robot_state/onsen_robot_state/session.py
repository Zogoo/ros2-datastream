"""Shared FE-session restart detection.

The FE simulator includes a per-page-load ``session_id`` nonce in every
``/sim/status`` heartbeat. A genuine restart (browser reload, new tab replacing
an old one) is a *change* of that id — a deterministic signal, unlike the old
``sim_time``-regression heuristic which thrashed whenever more than one FE tab
was briefly alive and published interleaved monotonic clocks.

``SessionWatch`` locks onto one session and only switches to a different id
after the locked session has been silent for ``stale_s`` seconds. This makes
two concurrent tabs harmless (the second tab is ignored, no reset storm) while
a real reload — where the old tab's heartbeats stop — still triggers exactly
one reset once the old session goes stale.

Used by mission_executor, towel_tracker and localizer so the contract lives in
one place (DRY).
"""
from __future__ import annotations

import json

# A locked session is considered gone after this many seconds of silence.
# Must exceed the FE heartbeat period (1.0 s) with margin so a live session is
# never mistaken for stale.
DEFAULT_STALE_S = 3.0


class SessionWatch:
    """Detects FE session restarts from /sim/status payloads.

    Stateless w.r.t. the wall clock: callers pass a monotonic ``now`` so the
    helper is fully deterministic and unit-testable.
    """

    def __init__(self, stale_s: float = DEFAULT_STALE_S) -> None:
        self._stale_s = stale_s
        self._session_id: str | None = None
        self._last_seen: float = 0.0
        # Legacy fallback state (FE builds with no session_id).
        self._last_sim_time: float | None = None

    def update_raw(self, data: str, now: float) -> bool:
        """Feed a raw /sim/status JSON string. Returns True exactly once on a
        detected restart. Malformed payloads are ignored (return False)."""
        try:
            payload = json.loads(data)
        except (json.JSONDecodeError, TypeError):
            return False
        return self.update(payload, now)

    def update(self, payload: dict, now: float) -> bool:
        """Feed a parsed /sim/status payload. Returns True exactly once when a
        genuine session restart is detected."""
        sid = payload.get("session_id")
        if sid is None:
            return self._update_legacy(payload)

        if self._session_id is None:
            # First heartbeat: lock on without signalling a restart.
            self._session_id = sid
            self._last_seen = now
            return False
        if sid == self._session_id:
            self._last_seen = now
            return False
        # A different id arrived. Only accept it as a restart once the locked
        # session has gone silent; otherwise it is a competing concurrent tab
        # and is ignored so two live tabs cannot cause a reset storm.
        if now - self._last_seen >= self._stale_s:
            self._session_id = sid
            self._last_seen = now
            return True
        return False

    def _update_legacy(self, payload: dict) -> bool:
        """Backward-compat for FE builds that omit session_id: fall back to the
        original sim_time-regression heuristic (single-publisher only)."""
        try:
            sim_time = float(payload.get("sim_time", 0.0))
        except (TypeError, ValueError):
            return False
        restarted = self._last_sim_time is not None and sim_time < self._last_sim_time - 1.0
        self._last_sim_time = sim_time
        return restarted

"""Map-frame towel tracking over per-frame detections — pure logic.

Detections arrive in base_link (depth-refined when possible) at ~5 Hz with
noise and dropouts; the mission needs stable world-frame targets. Classic
single-hypothesis tracking: nearest-neighbor gating, EMA position smoothing,
hit-count debounce, and visibility-aware expiry (a track is only deleted when
the robot is looking straight at where it should be and repeatedly sees
nothing — i.e. the towel was picked up or moved, not merely out of view).

Track dicts mirror the shape the mission FSM consumes
(`{id, class, position{x,y,z}}` — the documented MissionInput.towels seam).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

GATE_M = 0.5
DEDUP_M = 0.35      # observations this close to an existing track don't spawn a new one
EMA_ALPHA = 0.3
MIN_HITS = 3        # consecutive confirmations before a track is published (debounce)
VISIBLE_BEARING = math.radians(28.0)   # inside the 70 deg HFOV with margin
VISIBLE_RANGE = (0.45, 3.5)
MISSES_TO_EXPIRE = 8
STALE_AGE_S = 120.0   # unseen this long -> gone (stale sessions, moved towels)


@dataclass
class _Track:
    id: str
    x: float
    y: float
    hits: int = 1
    misses: int = 0
    last_seen: float = 0.0
    extra: dict[str, Any] = field(default_factory=dict)


class TowelTracker:
    def __init__(self) -> None:
        self._tracks: list[_Track] = []
        self._next_id = 1

    def update(
        self,
        detections_base: list[dict[str, Any]],
        robot_pose: tuple[float, float, float],
        stamp: float,
    ) -> list[dict[str, Any]]:
        """detections_base: towel detections with base_link `position` dicts."""
        rx, ry, ryaw = robot_pose
        cos_y, sin_y = math.cos(ryaw), math.sin(ryaw)
        observed = [
            (
                rx + cos_y * d["position"]["x"] - sin_y * d["position"]["y"],
                ry + sin_y * d["position"]["x"] + cos_y * d["position"]["y"],
                d,
            )
            for d in detections_base
            if d.get("position") is not None
        ]

        unmatched = list(observed)
        for track in self._tracks:
            best = None
            best_d = GATE_M
            for obs in unmatched:
                dist = math.hypot(obs[0] - track.x, obs[1] - track.y)
                if dist < best_d:
                    best, best_d = obs, dist
            if best is not None:
                unmatched.remove(best)
                track.x += EMA_ALPHA * (best[0] - track.x)
                track.y += EMA_ALPHA * (best[1] - track.y)
                track.hits += 1
                track.misses = 0
                track.last_seen = stamp
                track.extra = {"source": best[2].get("source"), "range": best[2].get("range")}
            elif self._expected_visible(track, robot_pose):
                track.misses += 1

        self._tracks = [
            t for t in self._tracks
            if t.misses < MISSES_TO_EXPIRE and stamp - t.last_seen < STALE_AGE_S
        ]

        for ox, oy, det in unmatched:
            # dedupe: a detection sitting on top of an existing track (localizer
            # jitter, double contours) must not spawn a co-located duplicate
            if any(math.hypot(ox - t.x, oy - t.y) < DEDUP_M for t in self._tracks):
                continue
            self._tracks.append(_Track(
                id=f"track_{self._next_id}", x=ox, y=oy, last_seen=stamp,
                extra={"source": det.get("source"), "range": det.get("range")},
            ))
            self._next_id += 1

        return self.tracks()

    def _expected_visible(self, track: _Track, robot_pose: tuple[float, float, float]) -> bool:
        rx, ry, ryaw = robot_pose
        dx, dy = track.x - rx, track.y - ry
        rng = math.hypot(dx, dy)
        if not (VISIBLE_RANGE[0] <= rng <= VISIBLE_RANGE[1]):
            return False
        bearing = _wrap(math.atan2(dy, dx) - ryaw)
        return abs(bearing) <= VISIBLE_BEARING

    def tracks(self) -> list[dict[str, Any]]:
        return [
            {
                "id": t.id,
                "class": "towel",
                "position": {"x": round(t.x, 3), "y": round(t.y, 3), "z": 0.02},
                "hits": t.hits,
                "misses": t.misses,
                "last_seen": t.last_seen,
                **t.extra,
            }
            for t in self._tracks
            if t.hits >= MIN_HITS
        ]

    def forget_near(self, x: float, y: float, radius: float = 0.4) -> None:
        """Drop tracks at a position — called after a confirmed grasp/binning."""
        self._tracks = [
            t for t in self._tracks if math.hypot(t.x - x, t.y - y) > radius
        ]


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

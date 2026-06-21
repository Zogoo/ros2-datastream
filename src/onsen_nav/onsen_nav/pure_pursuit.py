"""Regulated pure-pursuit path tracker — pure logic.

Implements the Nav2 RPP regulation rules (Coulter 1992 pure pursuit + curvature
and approach regulation, rotate-to-heading on large bearing errors) sized to
this robot's drive limits.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class TrackerParams:
    lookahead: float = 0.45
    v_max: float = 0.30
    v_min: float = 0.06
    w_max: float = 0.9
    rotate_threshold: float = 0.6     # rad bearing error -> rotate in place
    curvature_reg: float = 0.6       # v <= curvature_reg / |kappa| (RPP regulation)
    approach_gain: float = 0.6       # v <= approach_gain * dist_to_goal
    xy_tolerance: float = 0.12
    obstacle_slow_range: float = 0.45
    obstacle_stop_range: float = 0.22
    v_obstacle: float = 0.08


@dataclass
class TrackerOutput:
    vx: float
    wz: float
    done: bool
    distance_remaining: float
    blocked: bool = False


class PurePursuit:
    def __init__(self, params: TrackerParams | None = None) -> None:
        self.p = params or TrackerParams()

    def step(
        self,
        pose: tuple[float, float, float],
        path: list[tuple[float, float]],
        min_obstacle: float | None = None,
    ) -> TrackerOutput:
        x, y, yaw = pose
        goal = path[-1]
        dist_goal = math.hypot(goal[0] - x, goal[1] - y)
        if dist_goal < self.p.xy_tolerance:
            return TrackerOutput(0.0, 0.0, True, dist_goal)

        target = self._lookahead_point(x, y, path, dist_goal)
        bearing = wrap_angle(math.atan2(target[1] - y, target[0] - x) - yaw)

        if abs(bearing) > self.p.rotate_threshold:
            wz = clamp(2.0 * bearing, -self.p.w_max, self.p.w_max)
            return TrackerOutput(0.0, wz, False, dist_goal)

        # pure pursuit curvature: kappa = 2 sin(bearing) / L
        ld = max(0.05, math.hypot(target[0] - x, target[1] - y))
        kappa = 2.0 * math.sin(bearing) / ld

        v = self.p.v_max
        if abs(kappa) > 1e-6:
            v = min(v, self.p.curvature_reg / abs(kappa))
        v = min(v, self.p.approach_gain * dist_goal)

        blocked = False
        if min_obstacle is not None:
            if min_obstacle < self.p.obstacle_stop_range:
                return TrackerOutput(0.0, 0.0, False, dist_goal, blocked=True)
            if min_obstacle < self.p.obstacle_slow_range:
                v = min(v, self.p.v_obstacle)

        v = max(self.p.v_min, v)
        wz = clamp(v * kappa, -self.p.w_max, self.p.w_max)
        return TrackerOutput(v, wz, False, dist_goal, blocked=blocked)

    def _lookahead_point(
        self, x: float, y: float, path: list[tuple[float, float]], dist_goal: float,
    ) -> tuple[float, float]:
        """The carrot: project the robot onto the path polyline, then walk
        `lookahead` meters forward ALONG the path geometry. Searching from the
        nearest waypoint instead would flip the target behind the robot once a
        sparse waypoint is passed by more than the lookahead (limit cycle)."""
        if dist_goal <= self.p.lookahead:
            return path[-1]

        best_d2 = math.inf
        best_seg = 0
        best_point = path[0]
        for i in range(len(path) - 1):
            point, _ = project_on_segment((x, y), path[i], path[i + 1])
            d2 = (point[0] - x) ** 2 + (point[1] - y) ** 2
            if d2 < best_d2:
                best_d2, best_seg, best_point = d2, i, point

        remaining = self.p.lookahead
        cursor = best_point
        for i in range(best_seg, len(path) - 1):
            seg_end = path[i + 1]
            leg = math.hypot(seg_end[0] - cursor[0], seg_end[1] - cursor[1])
            if leg >= remaining:
                f = remaining / leg if leg > 0 else 0.0
                return (
                    cursor[0] + (seg_end[0] - cursor[0]) * f,
                    cursor[1] + (seg_end[1] - cursor[1]) * f,
                )
            remaining -= leg
            cursor = seg_end
        return path[-1]


def project_on_segment(
    p: tuple[float, float], a: tuple[float, float], b: tuple[float, float],
) -> tuple[tuple[float, float], float]:
    """Closest point on segment ab to p, and the parameter t in [0, 1]."""
    abx, aby = b[0] - a[0], b[1] - a[1]
    len2 = abx * abx + aby * aby
    if len2 == 0:
        return a, 0.0
    t = max(0.0, min(1.0, ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / len2))
    return (a[0] + abx * t, a[1] + aby * t), t


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

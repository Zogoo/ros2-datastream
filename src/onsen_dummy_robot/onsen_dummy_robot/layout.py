"""Shared onsen floor-plan geometry loaded from shared/onsen_layout.json.

Provides axis-aligned wall boxes for 2D lidar raycasting and circle-vs-box
collision used by the headless simulator. The frontend consumes the same
JSON file, so both worlds stay geometrically identical.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass

import numpy as np

LAYOUT_PATH = os.environ.get("ONSEN_LAYOUT_PATH", "/ros2_ws/shared/onsen_layout.json")


@dataclass(frozen=True)
class WallBox:
    cx: float
    cy: float
    hx: float  # half extent x
    hy: float  # half extent y
    height: float


class OnsenLayout:
    def __init__(self, path: str = LAYOUT_PATH) -> None:
        with open(path) as f:
            self._data = json.load(f)
        self.walls: list[WallBox] = [
            WallBox(w["c"][0], w["c"][1], w["size"][0] / 2.0, w["size"][1] / 2.0, w["h"])
            for w in self._data["walls"]
        ]
        for prop in self._data["static_props"]:
            self.walls.append(WallBox(
                prop["c"][0], prop["c"][1],
                prop["size"][0] / 2.0, prop["size"][1] / 2.0,
                prop.get("z0", 0.0) + prop["h"],
            ))
        for bin_def in self._data["bins"]:
            self.walls.append(WallBox(
                bin_def["c"][0], bin_def["c"][1],
                bin_def["size"][0] / 2.0, bin_def["size"][1] / 2.0, bin_def["h"],
            ))
        lidar = self._data["lidar"]
        self.lidar_height: float = lidar["height"]
        self.lidar_range_max: float = lidar["range_max"]
        self.lidar_range_min: float = lidar["range_min"]
        self.lidar_num_rays: int = lidar["num_rays"]
        self.spawn_x, self.spawn_y = self._data["robot_spawn"]["pos"]
        self.spawn_yaw: float = self._data["robot_spawn"]["yaw"]
        # Only structures tall enough to intersect the lidar plane are visible.
        self._lidar_walls = [w for w in self.walls if w.height >= self.lidar_height]

    # ── Lidar ────────────────────────────────────────────────────────────────

    def scan(self, ox: float, oy: float, yaw: float) -> np.ndarray:
        n = self.lidar_num_rays
        angles = yaw + np.linspace(-math.pi, math.pi, n, endpoint=False)
        ranges = np.full(n, self.lidar_range_max)
        for i in range(n):
            ranges[i] = self._ray(ox, oy, float(angles[i]))
        return np.clip(ranges, self.lidar_range_min, self.lidar_range_max).astype(np.float32)

    def _ray(self, ox: float, oy: float, angle: float) -> float:
        dx, dy = math.cos(angle), math.sin(angle)
        t_min = self.lidar_range_max
        for w in self._lidar_walls:
            t = _ray_aabb(ox, oy, dx, dy, w)
            if t is not None and t < t_min:
                t_min = t
        return t_min

    # ── Collision (robot footprint as circle) ────────────────────────────────

    def resolve_collision(self, x: float, y: float, radius: float) -> tuple[float, float, bool]:
        """Push a circle out of any overlapping wall box. Returns (x, y, hit)."""
        hit = False
        for w in self.walls:
            if w.height < 0.05:
                continue
            nx = max(w.cx - w.hx, min(x, w.cx + w.hx))
            ny = max(w.cy - w.hy, min(y, w.cy + w.hy))
            ddx, ddy = x - nx, y - ny
            dist_sq = ddx * ddx + ddy * ddy
            if dist_sq >= radius * radius:
                continue
            hit = True
            dist = math.sqrt(dist_sq)
            if dist < 1e-6:
                ddx, ddy, dist = x - w.cx, y - w.cy, max(1e-6, math.hypot(x - w.cx, y - w.cy))
            push = radius - dist
            x += ddx / dist * push
            y += ddy / dist * push
        return x, y, hit


def _ray_aabb(ox: float, oy: float, dx: float, dy: float, w: WallBox) -> float | None:
    """Slab-method ray vs axis-aligned box intersection in 2D."""
    inv_dx = 1.0 / dx if abs(dx) > 1e-12 else math.inf
    inv_dy = 1.0 / dy if abs(dy) > 1e-12 else math.inf
    t1 = (w.cx - w.hx - ox) * inv_dx
    t2 = (w.cx + w.hx - ox) * inv_dx
    t3 = (w.cy - w.hy - oy) * inv_dy
    t4 = (w.cy + w.hy - oy) * inv_dy
    t_near = max(min(t1, t2), min(t3, t4))
    t_far = min(max(t1, t2), max(t3, t4))
    if t_far < 0 or t_near > t_far:
        return None
    return t_near if t_near > 0 else None

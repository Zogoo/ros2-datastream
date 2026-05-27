"""
Synthetic 2D LiDAR generator.
Simulates a small rectangular room with benches and an occasional moving obstacle.
"""
from __future__ import annotations

import math
import random
import time
from typing import NamedTuple

import numpy as np


class Obstacle(NamedTuple):
    x: float  # room-frame metres
    y: float
    radius: float


# ── Room geometry ─────────────────────────────────────────────────────────────
# Room: roughly 4 m x 5 m.  Robot at origin facing +X.
ROOM_HALF_X = 2.5  # metres to front/back wall
ROOM_HALF_Y = 2.0  # metres to left/right wall

# Static obstacles (bench legs, buckets)
STATIC_OBSTACLES: list[Obstacle] = [
    Obstacle(-1.5, -1.4, 0.12),
    Obstacle(-1.5,  1.4, 0.12),
    Obstacle( 1.8, -1.4, 0.10),
    Obstacle( 1.8,  1.4, 0.10),
    Obstacle( 0.5, -1.6, 0.15),
    Obstacle(-0.8,  0.9, 0.08),
]


class LidarGenerator:
    """Generates LaserScan range arrays at configurable resolution."""

    def __init__(
        self,
        num_rays: int = 360,
        range_min: float = 0.05,
        range_max: float = 8.0,
        noise_std: float = 0.015,
    ) -> None:
        self._num_rays = num_rays
        self._range_min = range_min
        self._range_max = range_max
        self._noise_std = noise_std
        self._rng = random.Random()
        self._t0 = time.monotonic()

        # Dynamic obstacle (person-like) state
        self._dyn_x = 1.0
        self._dyn_y = 0.5
        self._dyn_speed = 0.08  # m/s in room coords per update

    def generate(self, robot_x: float, robot_y: float, robot_yaw: float) -> np.ndarray:
        """
        Return ranges array of length num_rays.
        robot_x, robot_y, robot_yaw: robot pose in room frame.
        """
        self._update_dynamic_obstacle()
        angles = np.linspace(-math.pi, math.pi, self._num_rays, endpoint=False)
        ranges = np.full(self._num_rays, self._range_max)

        for i, angle in enumerate(angles):
            world_angle = robot_yaw + angle
            r = self._cast_ray(robot_x, robot_y, world_angle)
            ranges[i] = np.clip(r, self._range_min, self._range_max)

        # Gaussian measurement noise
        noise = np.random.normal(0.0, self._noise_std, self._num_rays)
        ranges = np.clip(ranges + noise, self._range_min, self._range_max)

        # Occasional dropout (sensor noise, reflective surfaces)
        dropout_mask = np.random.random(self._num_rays) < 0.005
        ranges[dropout_mask] = self._range_max

        return ranges.astype(np.float32)

    # ── Ray casting ──────────────────────────────────────────────────────────

    def _cast_ray(self, ox: float, oy: float, angle: float) -> float:
        dx = math.cos(angle)
        dy = math.sin(angle)

        # Distance to axis-aligned room walls
        t_min = self._range_max
        t_min = min(t_min, self._ray_wall(ox, oy, dx, dy))

        # Static obstacles
        for obs in STATIC_OBSTACLES:
            t = self._ray_circle(ox, oy, dx, dy, obs.x, obs.y, obs.radius)
            if t is not None:
                t_min = min(t_min, t)

        # Dynamic obstacle
        t = self._ray_circle(ox, oy, dx, dy, self._dyn_x, self._dyn_y, 0.25)
        if t is not None:
            t_min = min(t_min, t)

        return t_min

    def _ray_wall(self, ox: float, oy: float, dx: float, dy: float) -> float:
        """Intersection with the rectangular room boundary."""
        t_candidates: list[float] = []
        eps = 1e-9

        for wall_val, axis in [
            ( ROOM_HALF_X, 'x'), (-ROOM_HALF_X, 'x'),
            ( ROOM_HALF_Y, 'y'), (-ROOM_HALF_Y, 'y'),
        ]:
            if axis == 'x' and abs(dx) > eps:
                t = (wall_val - ox) / dx
                if t > 0:
                    hit_y = oy + t * dy
                    if abs(hit_y) <= ROOM_HALF_Y + 0.01:
                        t_candidates.append(t)
            elif axis == 'y' and abs(dy) > eps:
                t = (wall_val - oy) / dy
                if t > 0:
                    hit_x = ox + t * dx
                    if abs(hit_x) <= ROOM_HALF_X + 0.01:
                        t_candidates.append(t)

        return min(t_candidates) if t_candidates else self._range_max

    @staticmethod
    def _ray_circle(
        ox: float, oy: float, dx: float, dy: float,
        cx: float, cy: float, r: float,
    ) -> float | None:
        """Ray-circle intersection, returns distance or None."""
        fx, fy = ox - cx, oy - cy
        a = dx * dx + dy * dy
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - r * r
        disc = b * b - 4.0 * a * c
        if disc < 0:
            return None
        t = (-b - math.sqrt(disc)) / (2.0 * a)
        return t if t > 0.01 else None

    # ── Dynamic obstacle ─────────────────────────────────────────────────────

    def _update_dynamic_obstacle(self) -> None:
        """Slowly move the person-like obstacle around the room."""
        t = time.monotonic() - self._t0
        # Slow drift pattern
        self._dyn_x = 1.2 * math.cos(t * 0.15)
        self._dyn_y = 0.8 * math.sin(t * 0.22)

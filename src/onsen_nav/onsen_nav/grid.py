"""Occupancy grids rasterized from shared/onsen_layout.json — pure logic.

Two distinct grids, because the planner and the localizer see different worlds:
  planner grid    everything the robot must not drive into: walls, furniture,
                  bins, and the pools as keepout (the rims are physically
                  unclimbable but the water is the actual kill hazard)
  field grid      only what the 0.62 m LIDAR plane actually returns (walls and
                  tall furniture) — the likelihood-field map for scan matching
"""
from __future__ import annotations

import json
import math

import cv2
import numpy as np

RESOLUTION = 0.05
POOL_KEEPOUT_MARGIN = 0.30
FREE, LETHAL = 0, 255


class NavGrid:
    def __init__(self, layout: dict, resolution: float = RESOLUTION) -> None:
        self.res = resolution
        bmin = layout["meta"]["building"]["min"]
        bmax = layout["meta"]["building"]["max"]
        self.origin = (float(bmin[0]), float(bmin[1]))
        self.width = round((bmax[0] - bmin[0]) / resolution)
        self.height = round((bmax[1] - bmin[1]) / resolution)
        self.lidar_z = float(layout.get("lidar", {}).get("height", 0.62))

        self.planner_grid = self._build_planner_grid(layout)
        self.field_grid = self._build_field_grid(layout)
        self._field_dt: np.ndarray | None = None

    @classmethod
    def from_file(cls, path: str, resolution: float = RESOLUTION) -> NavGrid:
        with open(path) as f:
            return cls(json.load(f), resolution)

    # ── coordinates (grid indexed [row=y, col=x]) ─────────────────────────────

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (
            int((y - self.origin[1]) / self.res),
            int((x - self.origin[0]) / self.res),
        )

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        return (
            self.origin[0] + (col + 0.5) * self.res,
            self.origin[1] + (row + 0.5) * self.res,
        )

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.height and 0 <= col < self.width

    def is_lethal(self, x: float, y: float) -> bool:
        """True when (x, y) falls on a mapped obstacle (wall/prop/bin/pool
        keepout) in the planner grid. Lets a caller distinguish a lidar return
        that coincides with a wall the planner already routed around from a
        genuinely unmapped/dynamic obstacle (a towel, a person) — see
        nav_server_node's front-sector obstacle gate."""
        row, col = self.world_to_cell(x, y)
        return self.in_bounds(row, col) and bool(self.planner_grid[row, col] == LETHAL)

    # ── rasterization ─────────────────────────────────────────────────────────

    def _blank(self) -> np.ndarray:
        return np.full((self.height, self.width), FREE, np.uint8)

    def _fill_box(self, grid: np.ndarray, cx: float, cy: float,
                  sx: float, sy: float, margin: float = 0.0) -> None:
        r0, c0 = self.world_to_cell(cx - sx / 2 - margin, cy - sy / 2 - margin)
        r1, c1 = self.world_to_cell(cx + sx / 2 + margin, cy + sy / 2 + margin)
        grid[max(r0, 0): min(r1 + 1, self.height), max(c0, 0): min(c1 + 1, self.width)] = LETHAL

    def _fill_rect(self, grid: np.ndarray, rect: list[float], margin: float = 0.0) -> None:
        x0, y0, x1, y1 = rect
        self._fill_box(grid, (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, margin)

    def _build_planner_grid(self, layout: dict) -> np.ndarray:
        grid = self._blank()
        for wall in layout["walls"]:
            self._fill_box(grid, *wall["c"], *wall["size"])
        for prop in layout["static_props"]:
            self._fill_box(grid, *prop["c"], *prop["size"])
        for bin_def in layout["bins"]:
            self._fill_box(grid, *bin_def["c"], *bin_def["size"])
        for pool in layout["pools"]:
            self._fill_rect(grid, pool["rect"], margin=POOL_KEEPOUT_MARGIN)
        return grid

    def _build_field_grid(self, layout: dict) -> np.ndarray:
        grid = self._blank()
        for wall in layout["walls"]:
            self._fill_box(grid, *wall["c"], *wall["size"])
        for prop in layout["static_props"]:
            top = float(prop.get("z0", 0.0)) + float(prop["h"])
            if top >= self.lidar_z:
                self._fill_box(grid, *prop["c"], *prop["size"])
        return grid

    # ── derived products ──────────────────────────────────────────────────────

    def inflated(self, radius: float) -> np.ndarray:
        """Planner grid grown by the robot radius — A* treats the robot as a point."""
        cells = math.ceil(radius / self.res)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * cells + 1, 2 * cells + 1))
        return cv2.dilate(self.planner_grid, kernel)

    def field_distance(self) -> np.ndarray:
        """Distance (m) from each cell to the nearest lidar-visible obstacle."""
        if self._field_dt is None:
            free = (self.field_grid == FREE).astype(np.uint8)
            self._field_dt = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.res
        return self._field_dt

    def occupancy_msg_data(self) -> list[int]:
        """nav_msgs/OccupancyGrid.data payload (row-major from origin) for /map."""
        return [100 if v == LETHAL else 0 for v in self.planner_grid.reshape(-1)]

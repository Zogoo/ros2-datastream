"""A* global planning on the inflated occupancy grid — pure logic.

8-connected A* with octile heuristic (the algorithm behind Nav2's NavFn-class
planners), followed by line-of-sight shortcutting so pure pursuit gets a sparse
waypoint polyline instead of a cell staircase.
"""
from __future__ import annotations

import heapq
import math

import numpy as np

SQRT2 = math.sqrt(2.0)
NEIGHBORS = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2),
]


def octile(a: tuple[int, int], b: tuple[int, int]) -> float:
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dr, dc) + (SQRT2 - 1.0) * min(dr, dc)


def plan_cells(
    grid: np.ndarray, start: tuple[int, int], goal: tuple[int, int],
) -> list[tuple[int, int]] | None:
    """A* over free cells (grid value 0). Returns the cell path or None."""
    h, w = grid.shape
    if not (0 <= start[0] < h and 0 <= start[1] < w):
        return None
    if not (0 <= goal[0] < h and 0 <= goal[1] < w):
        return None
    if grid[goal] != 0:
        snapped_goal = nearest_free(grid, goal, max_radius_cells=12)
        if snapped_goal is None:
            return None
        goal = snapped_goal
    if grid[start] != 0:
        snapped_start = nearest_free(grid, start, max_radius_cells=12)
        if snapped_start is None:
            return None
        start = snapped_start

    g = {start: 0.0}
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    open_heap: list[tuple[float, tuple[int, int]]] = [(octile(start, goal), start)]
    closed: set[tuple[int, int]] = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in parent:
                current = parent[current]
                path.append(current)
            return path[::-1]
        closed.add(current)
        for dr, dc, step in NEIGHBORS:
            nb = (current[0] + dr, current[1] + dc)
            if not (0 <= nb[0] < h and 0 <= nb[1] < w) or grid[nb] != 0 or nb in closed:
                continue
            cost = g[current] + step
            if cost < g.get(nb, math.inf):
                g[nb] = cost
                parent[nb] = current
                heapq.heappush(open_heap, (cost + octile(nb, goal), nb))
    return None


def nearest_free(
    grid: np.ndarray, cell: tuple[int, int], max_radius_cells: int,
) -> tuple[int, int] | None:
    """Spiral out to the closest free cell — tolerates goals set inside inflation."""
    h, w = grid.shape
    best = None
    best_d = math.inf
    for dr in range(-max_radius_cells, max_radius_cells + 1):
        for dc in range(-max_radius_cells, max_radius_cells + 1):
            r, c = cell[0] + dr, cell[1] + dc
            if 0 <= r < h and 0 <= c < w and grid[r, c] == 0:
                d = dr * dr + dc * dc
                if d < best_d:
                    best, best_d = (r, c), d
    return best


def line_of_sight(grid: np.ndarray, a: tuple[int, int], b: tuple[int, int]) -> bool:
    """Bresenham walk — true when every cell between a and b is free."""
    r0, c0 = a
    r1, c1 = b
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        if grid[r, c] != 0:
            return False
        if (r, c) == (r1, c1):
            return True
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc


def shortcut(grid: np.ndarray, cells: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Greedy line-of-sight simplification of the A* cell staircase."""
    if len(cells) <= 2:
        return cells
    result = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not line_of_sight(grid, cells[i], cells[j]):
            j -= 1
        result.append(cells[j])
        i = j
    return result


def plan_path(
    grid: np.ndarray,
    grid_origin: tuple[float, float],
    resolution: float,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
) -> list[tuple[float, float]] | None:
    """World-coordinate planning: A* + shortcutting -> waypoint polyline."""
    to_cell = lambda x, y: (  # noqa: E731
        int((y - grid_origin[1]) / resolution), int((x - grid_origin[0]) / resolution),
    )
    to_world = lambda r, c: (  # noqa: E731
        grid_origin[0] + (c + 0.5) * resolution, grid_origin[1] + (r + 0.5) * resolution,
    )
    cells = plan_cells(grid, to_cell(*start_xy), to_cell(*goal_xy))
    if cells is None:
        return None
    waypoints = [to_world(r, c) for r, c in shortcut(grid, cells)]
    # exact endpoints (cell centers are up to half a cell off)
    waypoints[0] = start_xy
    waypoints[-1] = goal_xy
    return waypoints

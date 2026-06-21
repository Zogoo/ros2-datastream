"""A* + shortcutting tests on synthetic grids and the real layout."""
import numpy as np
from onsen_nav.astar import line_of_sight, plan_cells, plan_path, shortcut
from onsen_nav.grid import NavGrid

LAYOUT = "shared/onsen_layout.json"
ROBOT_RADIUS = 0.30


def synthetic_grid():
    grid = np.zeros((20, 20), np.uint8)
    grid[10, 0:15] = 255   # wall with a gap at columns 15..19
    return grid


class TestSynthetic:
    def test_path_goes_through_gap(self):
        grid = synthetic_grid()
        path = plan_cells(grid, (5, 5), (15, 5))
        assert path is not None
        crossing = next(p for p in path if p[0] == 10)
        assert crossing[1] >= 15

    def test_fully_blocked_returns_none(self):
        grid = synthetic_grid()
        grid[10, :] = 255
        assert plan_cells(grid, (5, 5), (15, 5)) is None

    def test_line_of_sight(self):
        grid = synthetic_grid()
        assert line_of_sight(grid, (5, 5), (5, 15))
        assert not line_of_sight(grid, (5, 5), (15, 5))

    def test_shortcut_reduces_waypoints(self):
        grid = np.zeros((30, 30), np.uint8)
        path = plan_cells(grid, (0, 0), (29, 14))
        cut = shortcut(grid, path)
        assert len(cut) <= 3
        assert cut[0] == (0, 0)
        assert cut[-1] == (29, 14)


class TestRealLayout:
    def test_spawn_to_towel_bin_standoff(self):
        grid = NavGrid.from_file(LAYOUT)
        inflated = grid.inflated(ROBOT_RADIUS)
        path = plan_path(
            inflated, grid.origin, grid.res,
            start_xy=(0.0, -3.6),         # robot spawn
            goal_xy=(0.0, 3.7),           # standoff south of the towel bin
        )
        assert path is not None
        assert len(path) >= 2
        # every waypoint must sit on free inflated cells
        for x, y in path[1:-1]:
            assert inflated[grid.world_to_cell(x, y)] == 0

    def test_goal_inside_inflation_is_recovered(self):
        grid = NavGrid.from_file(LAYOUT)
        inflated = grid.inflated(ROBOT_RADIUS)
        # goal right at the bin face — inside inflation, must snap to nearest free
        path = plan_path(inflated, grid.origin, grid.res, (0.0, -3.6), (0.0, 4.2))
        assert path is not None

    def test_pool_never_crossed(self):
        grid = NavGrid.from_file(LAYOUT)
        inflated = grid.inflated(ROBOT_RADIUS)
        # corridor to the west bath door area — the goal snaps to the nearest
        # free cell outside the pool keepout; no waypoint may enter the pool
        path = plan_path(inflated, grid.origin, grid.res, (0.0, -4.4), (-1.0, -3.8))
        assert path is not None
        for x, y in path[:-1]:
            assert not (-5.83 <= x <= -1.57 and -4.53 <= y <= -3.07), \
                f"path crosses pool at ({x:.2f},{y:.2f})"

    def test_tight_room_start_snaps_to_free(self):
        grid = NavGrid.from_file(LAYOUT)
        inflated = grid.inflated(ROBOT_RADIUS)
        # (-2.5,-1.6) is free in shower_w even after inflation (verified)
        path = plan_path(inflated, grid.origin, grid.res, (-2.5, -1.6), (0.0, -3.6))
        assert path is not None

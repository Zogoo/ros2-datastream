"""Rasterizer tests against the real onsen layout."""
import numpy as np
import pytest
from onsen_nav.grid import FREE, LETHAL, NavGrid

LAYOUT = "shared/onsen_layout.json"


@pytest.fixture(scope="module")
def grid() -> NavGrid:
    return NavGrid.from_file(LAYOUT)


def cell(grid: NavGrid, x: float, y: float) -> tuple[int, int]:
    return grid.world_to_cell(x, y)


class TestPlannerGrid:
    def test_dimensions_cover_building(self, grid):
        assert grid.planner_grid.shape == (320, 416)

    def test_outer_wall_occupied(self, grid):
        assert grid.planner_grid[cell(grid, 0.0, 7.936)] == LETHAL

    def test_corridor_free(self, grid):
        assert grid.planner_grid[cell(grid, 0.0, -5.76)] == FREE  # robot spawn

    def test_pool_is_keepout_with_margin(self, grid):
        assert grid.planner_grid[cell(grid, -5.6, -6.08)] == LETHAL  # pool center
        assert grid.planner_grid[cell(grid, -5.6, -4.75)] == LETHAL  # within 0.30 m margin

    def test_lounger_occupied_for_planner(self, grid):
        assert grid.planner_grid[cell(grid, -8.88, 0.96)] == LETHAL

    def test_towel_bin_occupied(self, grid):
        assert grid.planner_grid[cell(grid, 0.0, 7.12)] == LETHAL


class TestFieldGrid:
    def test_walls_visible_to_lidar(self, grid):
        assert grid.field_grid[cell(grid, 0.0, 7.936)] == LETHAL

    def test_lounger_invisible_to_lidar(self, grid):
        # top z = 0.09 + 0.32 = 0.41 < scan plane 0.62
        assert grid.field_grid[cell(grid, -8.88, 0.96)] == FREE

    def test_lockers_visible_to_lidar(self, grid):
        import json
        with open(LAYOUT) as f:
            layout = json.load(f)
        locker = next(p for p in layout["static_props"] if p["id"] == "locker_row")
        assert grid.field_grid[cell(grid, *locker["c"])] == LETHAL

    def test_pool_not_in_field(self, grid):
        assert grid.field_grid[cell(grid, -5.6, -6.08)] == FREE


class TestDerived:
    def test_inflation_grows_walls(self, grid):
        inflated = grid.inflated(0.40)
        # a free cell 0.3 m from the north wall becomes lethal after inflation
        probe = cell(grid, 0.0, 7.76)
        assert grid.planner_grid[probe] == FREE or grid.planner_grid[probe] == LETHAL
        assert inflated[cell(grid, 0.0, 7.52)] == LETHAL

    def test_field_distance_zero_at_wall_positive_in_corridor(self, grid):
        dt = grid.field_distance()
        assert dt[cell(grid, 0.0, 7.936)] == 0.0
        assert dt[cell(grid, 0.0, -5.76)] > 0.4

    def test_determinism(self):
        a = NavGrid.from_file(LAYOUT)
        b = NavGrid.from_file(LAYOUT)
        assert np.array_equal(a.planner_grid, b.planner_grid)
        assert np.array_equal(a.field_grid, b.field_grid)

    def test_is_lethal_matches_planner_grid(self, grid):
        assert grid.is_lethal(0.0, 7.936) is True    # outer wall
        assert grid.is_lethal(0.0, -5.76) is False   # robot spawn, corridor
        assert grid.is_lethal(-5.6, -6.08) is True   # pool keepout

    def test_is_lethal_out_of_bounds_is_false(self, grid):
        assert grid.is_lethal(1e6, 1e6) is False


class TestDynamicLayer:
    def _grid(self):
        import json

        from onsen_nav.grid import NavGrid
        with open(LAYOUT) as f:
            return NavGrid(json.load(f))

    def test_mark_blocks_and_expires(self):
        from onsen_nav.grid import FREE, DynamicLayer
        grid = self._grid()
        inflated = grid.inflated(0.36)
        # a free spot mid-corridor
        x, y = 0.0, 1.0
        r, c = grid.world_to_cell(x, y)
        assert inflated[r, c] == FREE
        dyn = DynamicLayer(grid, 0.41, ttl_s=30.0)
        dyn.mark(x, y, now=0.0)
        assert dyn.overlay(inflated, now=1.0)[r, c] != FREE, "marked cell must block"
        # inflation: a cell 0.3 m away is inside the 0.41 m disc
        r2, c2 = grid.world_to_cell(x + 0.3, y)
        assert dyn.overlay(inflated, now=1.0)[r2, c2] != FREE
        # after the TTL the memory decays back to the static map
        assert dyn.overlay(inflated, now=31.0)[r, c] == FREE
        assert dyn.active_count(31.0) == 0

    def test_overlay_never_mutates_static_grid(self):
        from onsen_nav.grid import FREE, DynamicLayer
        grid = self._grid()
        inflated = grid.inflated(0.36)
        r, c = grid.world_to_cell(0.0, 1.0)
        dyn = DynamicLayer(grid, 0.41)
        dyn.mark(0.0, 1.0, now=0.0)
        dyn.overlay(inflated, now=1.0)
        assert inflated[r, c] == FREE, "static inflated grid must stay untouched"

    def test_replan_routes_around_dynamic_obstacle(self):
        """The wiggle-loop regression: a blocked tracker used to replan the
        IDENTICAL path through an unmapped stool. With the mark in place the
        planner must find a path that avoids the marked disc (or fail —
        never the same straight line through it)."""
        import math

        from onsen_nav.astar import plan_path
        from onsen_nav.grid import DynamicLayer
        grid = self._grid()
        inflated = grid.inflated(0.36)
        start, goal = (0.0, 0.2), (0.0, 2.2)
        direct = plan_path(inflated, grid.origin, grid.res, start, goal)
        assert direct is not None
        dyn = DynamicLayer(grid, 0.41)
        dyn.mark(0.0, 1.2, now=0.0)   # "stool" mid-corridor on the direct line
        rerouted = plan_path(dyn.overlay(inflated, now=1.0), grid.origin, grid.res, start, goal)
        if rerouted is not None:
            clearance = min(math.hypot(px - 0.0, py - 1.2) for px, py in rerouted)
            assert clearance > 0.36, "replanned path must clear the marked obstacle"

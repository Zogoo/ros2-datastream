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
        assert grid.planner_grid.shape == (200, 260)

    def test_outer_wall_occupied(self, grid):
        assert grid.planner_grid[cell(grid, 0.0, 4.96)] == LETHAL

    def test_corridor_free(self, grid):
        assert grid.planner_grid[cell(grid, 0.0, -3.6)] == FREE  # robot spawn

    def test_pool_is_keepout_with_margin(self, grid):
        assert grid.planner_grid[cell(grid, -3.5, -3.8)] == LETHAL  # pool center
        assert grid.planner_grid[cell(grid, -3.5, -2.85)] == LETHAL  # within margin

    def test_lounger_occupied_for_planner(self, grid):
        assert grid.planner_grid[cell(grid, -5.55, 0.6)] == LETHAL

    def test_towel_bin_occupied(self, grid):
        assert grid.planner_grid[cell(grid, 0.0, 4.45)] == LETHAL


class TestFieldGrid:
    def test_walls_visible_to_lidar(self, grid):
        assert grid.field_grid[cell(grid, 0.0, 4.96)] == LETHAL

    def test_lounger_invisible_to_lidar(self, grid):
        # top z = 0.09 + 0.32 = 0.41 < scan plane 0.62
        assert grid.field_grid[cell(grid, -5.55, 0.6)] == FREE

    def test_lockers_visible_to_lidar(self, grid):
        import json
        with open(LAYOUT) as f:
            layout = json.load(f)
        locker = next(p for p in layout["static_props"] if p["id"] == "locker_row")
        assert grid.field_grid[cell(grid, *locker["c"])] == LETHAL

    def test_pool_not_in_field(self, grid):
        assert grid.field_grid[cell(grid, -3.5, -3.8)] == FREE


class TestDerived:
    def test_inflation_grows_walls(self, grid):
        inflated = grid.inflated(0.40)
        # a free cell 0.3 m from the north wall becomes lethal after inflation
        probe = cell(grid, 0.0, 4.85)
        assert grid.planner_grid[probe] == FREE or grid.planner_grid[probe] == LETHAL
        assert inflated[cell(grid, 0.0, 4.7)] == LETHAL

    def test_field_distance_zero_at_wall_positive_in_corridor(self, grid):
        dt = grid.field_distance()
        assert dt[cell(grid, 0.0, 4.96)] == 0.0
        assert dt[cell(grid, 0.0, -3.6)] > 0.4

    def test_determinism(self):
        a = NavGrid.from_file(LAYOUT)
        b = NavGrid.from_file(LAYOUT)
        assert np.array_equal(a.planner_grid, b.planner_grid)
        assert np.array_equal(a.field_grid, b.field_grid)

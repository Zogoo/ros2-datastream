"""Scan matcher: synthesize a scan by raycasting the real field map at a known
pose, perturb the initial guess, and require the matcher to recover it."""
import math

import numpy as np
import pytest
from onsen_nav.grid import LETHAL, NavGrid
from onsen_nav.scan_matcher import LikelihoodField, MatchResult, match, valid_beams

LAYOUT = "shared/onsen_layout.json"


@pytest.fixture(scope="module")
def grid() -> NavGrid:
    return NavGrid.from_file(LAYOUT)


@pytest.fixture(scope="module")
def field(grid) -> LikelihoodField:
    return LikelihoodField(grid.field_distance(), grid.origin, grid.res)


def raycast_scan(grid: NavGrid, pose, n_beams=360, max_range=10.0):
    """DDA raycast against the field grid — an idealized lidar."""
    x, y, yaw = pose
    ranges = []
    step = grid.res / 2
    for i in range(n_beams):
        bearing = -math.pi + i * (2 * math.pi / n_beams)
        a = yaw + bearing
        r = 0.0
        hit = math.inf
        while r < max_range:
            r += step
            cell = grid.world_to_cell(x + r * math.cos(a), y + r * math.sin(a))
            if not grid.in_bounds(*cell):
                break
            if grid.field_grid[cell] == LETHAL:
                hit = r
                break
        ranges.append(hit)
    return ranges


TRUE_POSE = (0.0, -3.6, math.pi / 2)  # robot spawn, mid-corridor


class TestMatcher:
    def test_perfect_guess_scores_high(self, grid, field):
        scan = raycast_scan(grid, TRUE_POSE)
        r, b = valid_beams(scan, -math.pi, 2 * math.pi / 360, 0.15, 10.0)
        assert field.score(TRUE_POSE, r, b) > 0.8

    def test_recovers_translation_and_rotation_offset(self, grid, field):
        scan = raycast_scan(grid, TRUE_POSE)
        r, b = valid_beams(scan, -math.pi, 2 * math.pi / 360, 0.15, 10.0)
        perturbed = (TRUE_POSE[0] + 0.15, TRUE_POSE[1] - 0.10,
                     TRUE_POSE[2] + math.radians(4))
        result: MatchResult = match(field, perturbed, r, b)
        assert abs(result.x - TRUE_POSE[0]) < 0.05
        assert abs(result.y - TRUE_POSE[1]) < 0.05
        assert abs(result.yaw - TRUE_POSE[2]) < math.radians(1.5)
        assert result.score > 0.7

    def test_score_degrades_away_from_truth(self, grid, field):
        scan = raycast_scan(grid, TRUE_POSE)
        r, b = valid_beams(scan, -math.pi, 2 * math.pi / 360, 0.15, 10.0)
        good = field.score(TRUE_POSE, r, b)
        bad = field.score((TRUE_POSE[0] + 0.8, TRUE_POSE[1], TRUE_POSE[2]), r, b)
        assert good > bad

    def test_empty_scan_scores_zero(self, field):
        assert field.score(TRUE_POSE, np.array([]), np.array([])) == 0.0

    def test_deterministic(self, grid, field):
        scan = raycast_scan(grid, TRUE_POSE)
        r, b = valid_beams(scan, -math.pi, 2 * math.pi / 360, 0.15, 10.0)
        p = (0.1, -3.5, 1.5)
        assert match(field, p, r, b) == match(field, p, r, b)

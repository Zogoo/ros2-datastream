"""Likelihood-field scan matching against the known static map — pure logic.

The AMCL measurement model (Thrun, Burgard, Fox — Probabilistic Robotics,
ch. 6.4) with coarse-to-fine hill climbing over the map->odom correction
instead of a particle filter: the layout is known and the start pose is given,
so the belief is unimodal and a deterministic local search is sufficient
(and exactly reproducible, which the tests exploit).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

SIGMA = 0.10           # m, likelihood-field smoothing
BEAM_STRIDE = 4        # score every 4th beam (90 of 360)
# search stages: (half-range m, step m, half-range rad, step rad)
STAGES = [
    (0.20, 0.05, math.radians(6.0), math.radians(2.0)),
    (0.05, 0.0125, math.radians(1.5), math.radians(0.5)),
]


@dataclass
class MatchResult:
    x: float
    y: float
    yaw: float
    score: float       # mean per-beam likelihood in [0, 1]


class LikelihoodField:
    def __init__(
        self, distance_m: np.ndarray, origin: tuple[float, float], resolution: float,
    ) -> None:
        self.dt = distance_m
        self.origin = origin
        self.res = resolution
        self.h, self.w = distance_m.shape

    def beam_likelihoods(
        self, pose: tuple[float, float, float],
        ranges: np.ndarray, bearings: np.ndarray,
    ) -> np.ndarray:
        x, y, yaw = pose
        ex = x + ranges * np.cos(yaw + bearings)
        ey = y + ranges * np.sin(yaw + bearings)
        cols = ((ex - self.origin[0]) / self.res).astype(np.int32)
        rows = ((ey - self.origin[1]) / self.res).astype(np.int32)
        inside = (rows >= 0) & (rows < self.h) & (cols >= 0) & (cols < self.w)
        d = np.full(ranges.shape, 1.0)
        d[inside] = self.dt[rows[inside], cols[inside]]
        return np.exp(-(d * d) / (2.0 * SIGMA * SIGMA))

    def score(
        self, pose: tuple[float, float, float],
        ranges: np.ndarray, bearings: np.ndarray,
    ) -> float:
        if len(ranges) == 0:
            return 0.0
        return float(np.mean(self.beam_likelihoods(pose, ranges, bearings)))


def valid_beams(
    ranges: list[float], angle_min: float, angle_increment: float,
    range_min: float, range_max: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Subsampled finite returns (max-range/dropout beams carry no field info)."""
    r = np.asarray(ranges, dtype=np.float64)[::BEAM_STRIDE]
    bearings = (angle_min + np.arange(len(ranges)) * angle_increment)[::BEAM_STRIDE]
    keep = np.isfinite(r) & (r > range_min) & (r < range_max * 0.98)
    return r[keep], bearings[keep]


# Motion-prior weight: penalize deviation from the odom-predicted pose so a
# flat likelihood ridge (a straight corridor, where sliding along the axis
# barely changes the scan) resolves toward odom instead of drifting. This is
# the role AMCL's motion model plays; the hill-climb keeps it as a soft prior.
# Kept deliberately light: strong enough to break a flat-ridge tie, weak enough
# that a real likelihood gradient still corrects accumulated skid-steer odom
# drift (too heavy and the estimate just trails the drifting odom).
PRIOR_XY = 0.4      # per meter² of deviation
PRIOR_YAW = 0.15    # per rad² of deviation


def match(
    field: LikelihoodField,
    initial: tuple[float, float, float],
    ranges: np.ndarray,
    bearings: np.ndarray,
) -> MatchResult:
    """Coarse-to-fine grid hill climb around `initial`, regularized by a motion
    prior toward `initial` (the odom-predicted pose). Deterministic.

    The reported score is the pure measurement likelihood (prior excluded) so
    downstream consumers still read a true match-quality value."""
    ix, iy, iyaw = initial

    def objective(p: tuple[float, float, float]) -> tuple[float, float]:
        like = field.score(p, ranges, bearings)
        penalty = (
            PRIOR_XY * ((p[0] - ix) ** 2 + (p[1] - iy) ** 2)
            + PRIOR_YAW * (p[2] - iyaw) ** 2
        )
        return like - penalty, like

    best = initial
    best_obj, best_like = objective(best)
    for half_xy, step_xy, half_yaw, step_yaw in STAGES:
        bx, by, byaw = best
        xs = np.arange(bx - half_xy, bx + half_xy + 1e-9, step_xy)
        ys = np.arange(by - half_xy, by + half_xy + 1e-9, step_xy)
        yaws = np.arange(byaw - half_yaw, byaw + half_yaw + 1e-9, step_yaw)
        for cx in xs:
            for cy in ys:
                for cyaw in yaws:
                    cand = (float(cx), float(cy), float(cyaw))
                    obj, like = objective(cand)
                    if obj > best_obj:
                        best, best_obj, best_like = cand, obj, like
    return MatchResult(*best, score=best_like)

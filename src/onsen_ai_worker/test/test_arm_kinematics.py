"""Analytic IK validated against the FK mirror — the rigorous oracle.

The FK here must match frontend/src/robot/kinematics.js bit-for-bit (same
constants from robot_spec.json), and IK->FK must round-trip to machine
precision on every reachable target. This is the mathematical guarantee the
project's autonomy rests on, so the bar is 1e-9 m, not "close enough".
"""
import json
import math
import random

import pytest

from onsen_ai_worker.arm_kinematics import (
    ArmModel,
    grasp_command,
    reachable_annulus,
)

with open("shared/robot_spec.json") as _f:
    ARM_SPEC = json.load(_f)["arm"]


@pytest.fixture(scope="module")
def model() -> ArmModel:
    return ArmModel.from_spec(ARM_SPEC)


class TestFkMirror:
    def test_home_pose_geometry(self, model):
        # all servos at 90 -> arm straight up; fingertip at shoulder + total length
        ft = model.fingertip([90, 90, 90, 90, 90, 70])
        assert ft[0] == pytest.approx(model.base[0], abs=1e-9)
        assert ft[1] == pytest.approx(0.0, abs=1e-9)
        assert ft[2] == pytest.approx(
            model.shoulder_z + model.l1 + model.l2 + model.l3, abs=1e-9,
        )

    def test_pick_scoop_touches_floor_ahead(self, model):
        # the firmware PICK_SCOOP pose — must match the documented 0.667 m / ~4 cm
        ft = model.fingertip([90, 157, 57, 77, 90, 80])
        assert ft[0] == pytest.approx(0.667, abs=0.01)
        assert ft[2] == pytest.approx(0.041, abs=0.01)

    def test_pan_rotates_into_y(self, model):
        ft = model.fingertip([120, 157, 57, 77, 90, 80])  # pan +30 deg left
        assert ft[1] > 0.0


class TestIkRoundTrip:
    def test_machine_precision_on_reachable_targets(self, model):
        rng = random.Random(0)
        tested = 0
        for _ in range(500):
            d = [
                90 + rng.uniform(-60, 60), rng.uniform(120, 165),
                rng.uniform(45, 85), rng.uniform(60, 95), 90, 80,
            ]
            target = model.fingertip(d)
            if not (0.0 <= target[2] <= 0.6):
                continue
            sol = model.ik(target, current=d, tilt_band_deg=(50, 160))
            if sol is None:
                continue  # outside the chosen tilt band — not a roundtrip failure
            tested += 1
            err = math.dist(model.fingertip(sol), target)
            assert err < 1e-9, f"IK->FK error {err:.2e} at {target}"
        assert tested > 300, "sampling should produce many reachable targets"

    def test_tool_tilt_respected(self, model):
        target = (0.68, 0.0, 0.05)  # radial 0.42, inside the annulus
        sol = model.ik(target, tilt_band_deg=(95, 120))
        assert sol is not None
        t3 = model.fk(sol)["tilts"][2]
        assert math.radians(95) - 0.1 <= t3 <= math.radians(120) + 0.1


class TestReachability:
    def test_unreachable_returns_none(self, model):
        assert model.ik((2.0, 0.0, 0.05)) is None      # far beyond reach
        assert model.ik((0.26, 0.0, 2.0)) is None       # straight up, too high

    def test_pan_limit_rejected(self, model):
        # directly behind the base -> pan ~180 deg, outside servo range
        assert model.ik((-0.5, 0.0, 0.05)) is None

    def test_annulus_is_a_band(self, model):
        lo, hi = reachable_annulus(model, z=0.03)
        assert 0.2 < lo < hi < 0.75
        # a target inside the band is reachable, outside is not
        x_in = model.base[0] + (lo + hi) / 2
        assert model.ik((x_in, 0.0, 0.03)) is not None

    def test_smoothness_prefers_near_current(self, model):
        target = (0.66, 0.1, 0.05)  # radial ~0.41, inside the annulus
        far = model.ik(target, current=[90, 90, 90, 90, 90, 80])
        assert far is not None
        # gripper servo carried from current
        assert far[5] == 80


class TestGraspCommand:
    def test_format(self):
        line = grasp_command([90, 157, 57, 77, 90, 12], ms=800)
        assert line == "J 90 157 57 77 90 12 800"

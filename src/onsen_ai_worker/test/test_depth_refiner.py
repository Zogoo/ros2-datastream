"""DepthRefiner: synthetic 16UC1 frames with analytically known geometry."""
import json
import math

import numpy as np
import pytest

from onsen_ai_worker.detection import DepthRefiner

with open("shared/robot_spec.json") as _f:
    SPEC = json.load(_f)


@pytest.fixture()
def refiner() -> DepthRefiner:
    return DepthRefiner(SPEC["sensors"]["camera_depth"])


def frame_with_point(refiner: DepthRefiner, base_point: dict, spread_px: int = 8) -> np.ndarray:
    """Depth frame where a patch around the projection of base_point carries
    exactly that point's true depth."""
    depth = np.zeros((refiner.height, refiner.width), np.uint16)
    xr, yd, zf = refiner._base_to_camera(base_point)
    u = int(refiner.cx + refiner.fx * xr / zf)
    v = int(refiner.cy + refiner.fy * yd / zf)
    depth[
        max(v - spread_px, 0): v + spread_px + 1,
        max(u - spread_px, 0): u + spread_px + 1,
    ] = int(zf * 1000)
    return depth


class TestRoundTrip:
    def test_known_point_recovered(self, refiner):
        truth = {"x": 1.2, "y": 0.15, "z": 0.05}
        depth = frame_with_point(refiner, truth)
        position, rng, source = refiner.refine(truth, depth)
        assert source == "depth"
        assert abs(position["x"] - truth["x"]) < 0.03
        assert abs(position["y"] - truth["y"]) < 0.03
        assert rng == pytest.approx(math.hypot(truth["x"], truth["y"]), abs=0.05)

    def test_range_error_in_estimate_corrected(self, refiner):
        # ground-plane estimate is 30 cm long; the depth pixels carry the truth.
        truth = {"x": 1.0, "y": -0.2, "z": 0.04}
        depth = frame_with_point(refiner, truth, spread_px=20)
        biased = {"x": 1.3, "y": -0.26, "z": 0.0}
        position, _, source = refiner.refine(biased, depth)
        assert source == "depth"
        assert abs(position["x"] - truth["x"]) < 0.12, "depth must pull the range in"


class TestFallbacks:
    def test_no_depth_frame(self, refiner):
        est = {"x": 1.0, "y": 0.0, "z": 0.0}
        position, rng, source = refiner.refine(est, None)
        assert (position, rng, source) == (est, None, "ground_plane")

    def test_no_estimate(self, refiner):
        assert refiner.refine(None, np.zeros((240, 320), np.uint16)) == (None, None, "none")

    def test_empty_roi_falls_back(self, refiner):
        est = {"x": 1.0, "y": 0.0, "z": 0.0}
        position, _rng, source = refiner.refine(est, np.zeros((240, 320), np.uint16))
        assert source == "ground_plane"
        assert position == est

    def test_point_behind_camera_falls_back(self, refiner):
        est = {"x": -0.5, "y": 0.0, "z": 0.0}
        _, _, source = refiner.refine(est, np.zeros((240, 320), np.uint16))
        assert source == "ground_plane"

    def test_point_outside_fov_falls_back(self, refiner):
        est = {"x": 0.6, "y": 3.0, "z": 0.0}  # ~80 deg off-axis
        _, _, source = refiner.refine(est, np.zeros((240, 320), np.uint16))
        assert source == "ground_plane"


class TestHeightGate:
    FY = 457.0  # 640px / (2 tan(35deg))

    def _det(self, cls, bbox, rng):
        from onsen_ai_worker.detection import DEFAULT_PROFILES, RISK_MAP
        return {
            "class": cls, "confidence": 0.8, "bbox": bbox, "range": rng,
            "robot_class": DEFAULT_PROFILES.get(cls, {}).get("robot_class", "x"),
            "pickable": cls == "towel", "risk": RISK_MAP.get(cls, "avoid"),
        }

    def test_flat_bucket_reclassified_as_towel(self):
        from onsen_ai_worker.detection import height_gate
        # 20 px tall at 1.5 m -> 0.066 m: flat -> towel
        out = height_gate([self._det("bucket", [100, 200, 200, 220], 1.5)], self.FY)
        assert out[0]["class"] == "towel"
        assert out[0]["pickable"] is True

    def test_tall_towel_dropped(self):
        from onsen_ai_worker.detection import height_gate
        # 120 px tall at 1.5 m -> 0.39 m: no towel is that tall
        out = height_gate([self._det("towel", [100, 100, 200, 220], 1.5)], self.FY)
        assert out == []

    def test_true_stool_kept(self):
        from onsen_ai_worker.detection import height_gate
        # 70 px at 1.5 m -> 0.23 m: genuinely tall
        out = height_gate([self._det("stool", [100, 150, 200, 220], 1.5)], self.FY)
        assert out[0]["class"] == "stool"

    def test_no_range_passes_through(self):
        from onsen_ai_worker.detection import height_gate
        det = self._det("towel", [100, 100, 200, 220], None)
        assert height_gate([det], self.FY) == [det]


class TestWidthGate:
    FY = 457.0
    FX = 457.0

    def _det(self, cls, bbox, rng):
        from onsen_ai_worker.detection import DEFAULT_PROFILES, RISK_MAP
        return {
            "class": cls, "confidence": 0.8, "bbox": bbox, "range": rng,
            "robot_class": DEFAULT_PROFILES.get(cls, {}).get("robot_class", "x"),
            "pickable": cls == "towel", "risk": RISK_MAP.get(cls, "avoid"),
        }

    def test_wide_floor_strip_rejected_as_towel(self):
        from onsen_ai_worker.detection import height_gate
        # 900 px wide at 1.5 m -> ~2.95 m: a floor strip, not a 0.3 m towel
        out = height_gate([self._det("towel", [0, 200, 900, 230], 1.5)], self.FY, self.FX)
        assert out == []

    def test_towel_sized_box_kept(self):
        from onsen_ai_worker.detection import height_gate
        # 110 px wide, 20 px tall at 1.5 m -> 0.36 m x 0.066 m: a real towel
        out = height_gate([self._det("towel", [100, 200, 210, 220], 1.5)], self.FY, self.FX)
        assert len(out) == 1 and out[0]["class"] == "towel"

    def test_wide_flat_object_not_reclassified(self):
        from onsen_ai_worker.detection import height_gate
        # flat but 1.5 m wide -> floor, must NOT become a towel
        out = height_gate([self._det("bucket", [0, 200, 460, 225], 1.5)], self.FY, self.FX)
        assert out[0]["class"] == "bucket"

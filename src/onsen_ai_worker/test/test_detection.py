"""Detection + skin-profile tests on synthesized sim-style frames."""
import math

import cv2
import numpy as np
import pytest

from onsen_ai_worker import detection
from onsen_ai_worker.detection import CameraModel, Detector, iou, nms

DEFAULT_TOWEL_BGR = (216, 234, 240)   # #f0ead8 default towel
BLUE_SKIN_BGR = (159, 93, 42)         # FE 'striped' preset blue


@pytest.fixture(autouse=True)
def isolated_profile_store(tmp_path, monkeypatch):
    monkeypatch.setattr(detection, "PROFILE_STORE", str(tmp_path / "profiles.json"))


def sim_frame(towel_bgr, box=(200, 260, 340, 350)):
    """Dark wooden floor + one towel, like a front-camera frame."""
    img = np.zeros((480, 640, 3), np.uint8)
    for y in range(480):
        img[y, :] = (50 + y // 24, 75 + y // 20, 110 + y // 16)
    rng = np.random.default_rng(7)
    img = (img + rng.normal(0, 5, img.shape)).clip(0, 255).astype(np.uint8)
    x1, y1, x2, y2 = box
    cv2.rectangle(img, (x1, y1), (x2, y2), towel_bgr, -1)
    return cv2.GaussianBlur(img, (3, 3), 0)


class TestDetector:
    def test_default_towel_detected(self):
        det = Detector(camera_model=None)
        frame = sim_frame(DEFAULT_TOWEL_BGR)
        towels = [d for d in det.detect(frame) if d["class"] == "towel"]
        assert towels, "default towel skin must be detected out of the box"
        assert iou(towels[0]["bbox"], [200, 260, 340, 350]) > 0.5
        assert towels[0]["pickable"] is True

    def test_reskinned_towel_lost_then_recovered_by_resample(self):
        det = Detector(camera_model=None)
        frame = sim_frame(BLUE_SKIN_BGR)
        assert not [d for d in det.detect(frame) if d["class"] == "towel"]

        skin = np.full((128, 128, 3), BLUE_SKIN_BGR, np.uint8)
        band = det.resample_profile("towel", skin)
        assert band["class"] == "towel"
        towels = [d for d in det.detect(frame) if d["class"] == "towel"]
        assert towels, "resampled profile must re-acquire the re-skinned towel"

    def test_reset_profile_restores_defaults(self):
        det = Detector(camera_model=None)
        det.resample_profile("towel", np.full((64, 64, 3), BLUE_SKIN_BGR, np.uint8))
        assert det.reset_profile("towel") is True
        assert det.profiles_snapshot()["towel"]["lower"] == \
            detection.DEFAULT_PROFILES["towel"]["lower"]

    def test_reset_unknown_class_refused(self):
        assert Detector(camera_model=None).reset_profile("dragon") is False

    def test_profile_store_round_trip(self):
        det = Detector(camera_model=None)
        band = det.resample_profile("towel", np.full((64, 64, 3), BLUE_SKIN_BGR, np.uint8))
        reloaded = Detector(camera_model=None)
        assert reloaded.profiles_snapshot()["towel"]["lower"] == band["lower"]


class TestCameraModel:
    CAM_SPEC = {
        "width": 640, "height": 480, "hfov_deg": 70,
        "position": [0.31, 0.0, 0.38], "pitch_deg": -15,
    }

    def test_floor_point_ahead_is_positive_forward(self):
        cam = CameraModel(self.CAM_SPEC)
        p = cam.pixel_to_base_link(320, 460)
        assert p is not None
        assert p["x"] > 0.31
        assert math.isclose(p["y"], 0.0, abs_tol=0.05)

    def test_horizon_pixel_rejected(self):
        cam = CameraModel(self.CAM_SPEC)
        assert cam.pixel_to_base_link(320, 0) is None

    def test_lower_pixel_is_closer(self):
        cam = CameraModel(self.CAM_SPEC)
        near = cam.pixel_to_base_link(320, 470)
        far = cam.pixel_to_base_link(320, 300)
        assert near is not None and far is not None
        assert near["x"] < far["x"]


class TestNms:
    def test_overlapping_detections_suppressed(self):
        dets = [
            {"id": "a", "class": "towel", "confidence": 0.9, "bbox": [0, 0, 100, 100]},
            {"id": "b", "class": "towel", "confidence": 0.6, "bbox": [10, 10, 110, 110]},
            {"id": "c", "class": "towel", "confidence": 0.7, "bbox": [300, 300, 400, 400]},
        ]
        kept = nms(dets, iou_threshold=0.5)
        assert {d["id"] for d in kept} == {"a", "c"}

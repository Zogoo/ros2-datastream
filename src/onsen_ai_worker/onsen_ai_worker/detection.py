"""HSV-band object detection over the simulated camera stream.

Intentionally a transparent classical-CV baseline (per-class HSV bands +
contour analysis + NMS) so data scientists can inspect every step, replace it
with an ONNX model, or resample bands at runtime from uploaded skins while the
topic contract stays identical.

Position estimation is a proper ground-plane back-projection using the camera
intrinsics/extrinsics from shared/robot_spec.json — not a magic constant.
"""
from __future__ import annotations

import json
import math
import os
import threading
from typing import Any

import cv2
import numpy as np

SPEC_PATH = os.environ.get("ROBOT_SPEC_PATH", "/ros2_ws/shared/robot_spec.json")
PROFILE_STORE = os.environ.get("AI_PROFILE_STORE", "/ros2_ws/output/detection_profiles.json")

# Default bands tuned to the FE object palette (object_profiles.json colors)
# rendered under the warm hemisphere lighting. OpenCV HSV: H 0..180.
DEFAULT_PROFILES: dict[str, dict[str, Any]] = {
    "towel": {
        "lower": [0, 0, 150], "upper": [40, 70, 255],
        "min_area": 500, "robot_class": "pickable_soft_object", "pickable": True,
    },
    "bucket": {
        "lower": [10, 60, 130], "upper": [28, 180, 245],
        "min_area": 350, "robot_class": "non_pickable_hard_object", "pickable": False,
    },
    "stool": {
        "lower": [8, 80, 60], "upper": [25, 200, 135],
        "min_area": 400, "robot_class": "non_pickable_hard_object", "pickable": False,
    },
    "bottle": {
        "lower": [75, 30, 140], "upper": [100, 130, 255],
        "min_area": 120, "robot_class": "non_pickable_hard_object", "pickable": False,
    },
}

RISK_MAP = {
    "towel": "low",
    "bucket": "avoid",
    "stool": "avoid",
    "bottle": "avoid",
    "unknown_obstacle": "stop",
}


class CameraModel:
    """Pinhole ground-plane back-projection from the front camera mount."""

    def __init__(self, cam_spec: dict) -> None:
        self.width = cam_spec["width"]
        self.height = cam_spec["height"]
        hfov = math.radians(cam_spec["hfov_deg"])
        self.fx = (self.width / 2) / math.tan(hfov / 2)
        self.fy = self.fx
        self.cx = self.width / 2
        self.cy = self.height / 2
        self.mount_x = cam_spec["position"][0]
        self.mount_z = cam_spec["position"][2]
        self.pitch = math.radians(-cam_spec.get("pitch_deg", 0))  # positive = down

    def pixel_to_base_link(self, u: float, v: float) -> dict[str, float] | None:
        """Project the bbox ground-contact pixel onto the floor plane (z=0)."""
        angle_below_horizon = self.pitch + math.atan2(v - self.cy, self.fy)
        if angle_below_horizon <= math.radians(2):
            return None  # above the horizon — not a floor point
        forward = self.mount_z / math.tan(angle_below_horizon)
        forward = min(forward, 8.0)
        lateral = -((u - self.cx) / self.fx) * forward
        return {
            "x": round(self.mount_x + forward, 2),
            "y": round(lateral, 2),
            "z": 0.0,
        }


def load_camera_model() -> CameraModel:
    with open(SPEC_PATH) as f:
        spec = json.load(f)
    return CameraModel(spec["sensors"]["camera_front"])


class Detector:
    def __init__(self, camera_model: CameraModel | None = None) -> None:
        self._lock = threading.Lock()
        self.profiles = {k: dict(v) for k, v in DEFAULT_PROFILES.items()}
        self.camera = camera_model
        self._load_store()

    # ── Profile management (skin pipeline) ───────────────────────────────────

    def resample_profile(self, cls: str, img_bgr: np.ndarray) -> dict[str, Any]:
        """Derive a new HSV band from an uploaded skin image (robust percentiles
        over the center crop, padded for lighting variation)."""
        h, w = img_bgr.shape[:2]
        crop = img_bgr[h // 6: h - h // 6, w // 6: w - w // 6]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV).reshape(-1, 3).astype(np.float64)
        lo = np.percentile(hsv, 5, axis=0)
        hi = np.percentile(hsv, 95, axis=0)
        pad = np.array([8.0, 35.0, 45.0])
        lower = np.clip(lo - pad, 0, [180, 255, 255]).astype(int).tolist()
        upper = np.clip(hi + pad, 0, [180, 255, 255]).astype(int).tolist()
        with self._lock:
            base = self.profiles.get(cls, dict(DEFAULT_PROFILES.get("towel", {})))
            base = dict(base)
            base["lower"], base["upper"] = lower, upper
            self.profiles[cls] = base
            self._save_store()
        return {"class": cls, "lower": lower, "upper": upper}

    def reset_profile(self, cls: str) -> bool:
        with self._lock:
            if cls not in DEFAULT_PROFILES:
                return False
            self.profiles[cls] = dict(DEFAULT_PROFILES[cls])
            self._save_store()
            return True

    def profiles_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {k: dict(v) for k, v in self.profiles.items()}

    def _save_store(self) -> None:
        try:
            os.makedirs(os.path.dirname(PROFILE_STORE), exist_ok=True)
            with open(PROFILE_STORE, "w") as f:
                json.dump(self.profiles, f, indent=2)
        except OSError:
            pass

    def _load_store(self) -> None:
        try:
            with open(PROFILE_STORE) as f:
                saved = json.load(f)
            for cls, profile in saved.items():
                if cls in self.profiles:
                    self.profiles[cls].update(profile)
        except (OSError, json.JSONDecodeError):
            pass

    # ── Detection ─────────────────────────────────────────────────────────────

    def detect(self, img_bgr: np.ndarray) -> list[dict[str, Any]]:
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (7, 7), 0)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        results: list[dict[str, Any]] = []
        obj_id = 1
        with self._lock:
            profiles = {k: dict(v) for k, v in self.profiles.items()}

        for cls, profile in profiles.items():
            mask = cv2.inRange(
                hsv, np.array(profile["lower"], np.uint8), np.array(profile["upper"], np.uint8),
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < profile["min_area"]:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                if w < 10 or h < 8:
                    continue
                # Discard detections covering most of the frame (floor/wall bleed)
                if w * h > 0.5 * img_bgr.shape[0] * img_bgr.shape[1]:
                    continue

                fill = area / float(w * h)
                confidence = round(min(0.98, 0.4 + fill * 0.4 + min(area, 20000) / 50000.0), 2)
                position = None
                if self.camera is not None:
                    position = self.camera.pixel_to_base_link(x + w / 2, y + h)

                results.append({
                    "id": f"det_{obj_id:03d}",
                    "class": cls,
                    "confidence": confidence,
                    "bbox": [int(x), int(y), int(x + w), int(y + h)],
                    "robot_class": profile["robot_class"],
                    "pickable": profile["pickable"],
                    "risk": RISK_MAP.get(cls, "avoid"),
                    "estimated_position": position,
                })
                obj_id += 1

        return nms(results, iou_threshold=0.5)


def nms(detections: list[dict], iou_threshold: float = 0.5) -> list[dict]:
    if not detections:
        return detections
    det_sorted = sorted(detections, key=lambda d: -d["confidence"])
    keep: list[dict] = []
    suppressed: set[int] = set()
    for i, d in enumerate(det_sorted):
        if i in suppressed:
            continue
        keep.append(d)
        for j in range(i + 1, len(det_sorted)):
            if j not in suppressed and iou(d["bbox"], det_sorted[j]["bbox"]) > iou_threshold:
                suppressed.add(j)
    return keep


def iou(a: list[int], b: list[int]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    if inter == 0:
        return 0.0
    area_a = (ax2 - ax1) * (ay2 - ay1)
    area_b = (bx2 - bx1) * (by2 - by1)
    return inter / (area_a + area_b - inter)

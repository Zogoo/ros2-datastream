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
        # Tuned against live renders: towels are the least-saturated warm thing
        # on the floor (S~125 vs floor S~215); V down to 55 covers shadowed
        # towels at range. The depth height gate below resolves the residual
        # towel/bucket ambiguity that color alone cannot.
        "lower": [0, 0, 55], "upper": [45, 185, 255],
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


class DepthRefiner:
    """Refines a ground-plane position estimate with real depth measurements.

    The RGB detector's `estimated_position` assumes the bbox foot touches the
    floor — fine for bearing, weak for range. This projects that 3D point into
    the depth image, reads the median depth of a small ROI, and back-projects
    to a measured base_link position: the genuine answer to "how far is the
    towel". Falls back to the ground-plane estimate under occlusion/invalid
    depth (the source field says which path produced the number).
    """

    ROI = 5  # half-size of the sampling window, px

    def __init__(self, depth_spec: dict) -> None:
        self.width = depth_spec["width"]
        self.height = depth_spec["height"]
        hfov = math.radians(depth_spec["hfov_deg"])
        self.fx = (self.width / 2) / math.tan(hfov / 2)
        self.fy = self.fx
        self.cx = self.width / 2
        self.cy = self.height / 2
        self.mount_x = float(depth_spec["position"][0])
        self.mount_z = float(depth_spec["position"][2])
        self.pitch = math.radians(-float(depth_spec.get("pitch_deg", 0)))  # + = down
        self.range_min = float(depth_spec.get("range_min", 0.28))
        self.range_max = float(depth_spec.get("range_max", 3.0))

    def _base_to_camera(self, p: dict[str, float]) -> tuple[float, float, float]:
        """base_link point -> depth optical frame (x right, y down, z fwd)."""
        dx = p["x"] - self.mount_x
        dy = p["y"]
        dz = p.get("z", 0.0) - self.mount_z
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        z_fwd = cp * dx - sp * dz
        y_down = -(sp * dx + cp * dz)
        x_right = -dy
        return (x_right, y_down, z_fwd)

    def _camera_to_base(
        self, x_right: float, y_down: float, z_fwd: float,
    ) -> dict[str, float]:
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        dx = cp * z_fwd + sp * (-y_down)
        dz = -sp * z_fwd + cp * (-y_down)
        return {
            "x": round(self.mount_x + dx, 3),
            "y": round(-x_right, 3),
            "z": round(self.mount_z + dz, 3),
        }

    def refine(
        self,
        estimated_position: dict[str, float] | None,
        depth_mm: np.ndarray | None,
    ) -> tuple[dict[str, float] | None, float | None, str]:
        """Returns (position, range_m, source)."""
        if estimated_position is None:
            return None, None, "none"
        if depth_mm is None:
            return estimated_position, None, "ground_plane"

        xr, yd, zf = self._base_to_camera(estimated_position)
        if zf <= 0.05:
            return estimated_position, None, "ground_plane"
        u = int(self.cx + self.fx * xr / zf)
        v = int(self.cy + self.fy * yd / zf)
        if not (0 <= u < self.width and 0 <= v < self.height):
            return estimated_position, None, "ground_plane"

        roi = depth_mm[
            max(v - self.ROI, 0): v + self.ROI + 1,
            max(u - self.ROI, 0): u + self.ROI + 1,
        ].astype(np.float64) / 1000.0
        valid = roi[(roi > self.range_min) & (roi < self.range_max)]
        if valid.size < 4:
            return estimated_position, None, "ground_plane"

        z_meas = float(np.median(valid))
        position = self._camera_to_base(
            (u - self.cx) / self.fx * z_meas,
            (v - self.cy) / self.fy * z_meas,
            z_meas,
        )
        rng = round(math.hypot(position["x"], position["y"]), 3)
        return position, rng, "depth"


def load_depth_refiner() -> DepthRefiner:
    with open(SPEC_PATH) as f:
        spec = json.load(f)
    return DepthRefiner(spec["sensors"]["camera_depth"])


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
                # A towel on the floor subtends a bounded pixel height even at
                # the near pick range; a tall box is the warm wood floor/wall
                # bleeding into the towel band, not a towel. (The depth height
                # gate below is the metric backstop where range is available.)
                if cls == "towel" and h > 0.38 * img_bgr.shape[0]:
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


TOWEL_MAX_HEIGHT_M = 0.12
TALL_MIN_HEIGHT_M = 0.10
TOWEL_WIDTH_M = (0.10, 0.65)   # a folded/scattered ryokan towel's footprint


def height_gate(detections: list[dict], fy: float, fx: float | None = None) -> list[dict]:
    """RGB+depth fusion: physical size = bbox_px * range / focal.

    Towels are flat (<= ~12 cm tall) and have a bounded footprint width
    (0.1-0.65 m); stools/buckets stand 0.16-0.22 m. Colour cannot separate them
    from the warm wood floor under dim light, but measured size can:
      - a "towel" too tall or too wide is floor/furniture bleed -> drop
      - a compact, towel-sized flat "stool"/"bucket" is a towel -> reclass
    Only applies where a depth-measured range exists (otherwise pass through).
    """
    fx = fx if fx is not None else fy
    out = []
    for det in detections:
        rng = det.get("range")
        if rng is None:
            out.append(det)
            continue
        x1, y1, x2, y2 = det["bbox"]
        height_m = (y2 - y1) * rng / fy
        width_m = (x2 - x1) * rng / fx
        det = dict(det, estimated_height=round(height_m, 3), estimated_width=round(width_m, 3))
        towel_sized = TOWEL_WIDTH_M[0] <= width_m <= TOWEL_WIDTH_M[1]
        if det["class"] == "towel":
            if height_m > TOWEL_MAX_HEIGHT_M or not towel_sized:
                continue  # floor/wall bleed masquerading as a towel
        elif det["class"] in ("stool", "bucket") and height_m < TALL_MIN_HEIGHT_M and towel_sized:
            det.update({
                "class": "towel",
                "robot_class": DEFAULT_PROFILES["towel"]["robot_class"],
                "pickable": True,
                "risk": RISK_MAP["towel"],
                "confidence": round(min(det["confidence"], 0.75), 2),
            })
        out.append(det)
    return out


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

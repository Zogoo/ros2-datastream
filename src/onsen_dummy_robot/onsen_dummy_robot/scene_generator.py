"""
Synthetic onsen/bathroom scene generator.
Produces RGB images and structured ground-truth for each frame.
No physics or simulation engine — pure NumPy / OpenCV drawing.
"""
from __future__ import annotations

import math
import random
from typing import Any

import cv2
import numpy as np

# ── Image dimensions ──────────────────────────────────────────────────────────
IMG_W = 640
IMG_H = 480

# ── Object class catalogue ────────────────────────────────────────────────────
OBJECT_CLASSES = [
    "towel", "clothes", "slipper", "bottle", "bucket",
    "brush", "can", "plastic_trash", "bath_mat", "bench",
    "floor", "wall", "person_body_part", "unknown_obstacle",
]
ROBOT_CLASS_MAP = {
    "towel":           "pickable_soft_object",
    "clothes":         "pickable_soft_object",
    "slipper":         "pickable_soft_object",
    "bottle":          "non_pickable_hard_object",
    "bucket":          "non_pickable_hard_object",
    "brush":           "pickable_soft_object",
    "can":             "non_pickable_hard_object",
    "plastic_trash":   "pickable_soft_object",
    "bath_mat":        "pickable_soft_object",
    "bench":           "static_environment",
    "floor":           "static_environment",
    "wall":            "static_environment",
    "person_body_part":"safety_stop",
    "unknown_obstacle":"unknown_obstacle",
}
PICKABLE = {
    "towel", "clothes", "slipper", "brush", "plastic_trash", "bath_mat",
}
RISK_MAP = {
    "towel":           "low",
    "clothes":         "low",
    "slipper":         "low",
    "bottle":          "avoid",
    "bucket":          "avoid",
    "brush":           "low",
    "can":             "avoid",
    "plastic_trash":   "low",
    "bath_mat":        "low",
    "bench":           "avoid",
    "floor":           "low",
    "wall":            "avoid",
    "person_body_part":"stop",
    "unknown_obstacle":"stop",
}

# ── Colour palettes ───────────────────────────────────────────────────────────
TOWEL_COLORS = [
    (220, 220, 210), (180, 200, 210), (200, 180, 160),
    (160, 190, 170), (210, 195, 185), (230, 210, 200),
]
BATH_MAT_COLORS = [
    (120, 110, 100), (140, 130, 120), (100, 120, 110),
]
SLIPPER_COLORS = [
    (180, 140, 100), (100, 100, 100), (200, 180, 160),
]
BOTTLE_COLORS = [
    (160, 210, 220), (100, 180, 100), (220, 220, 80),
]
BUCKET_COLORS = [
    (200, 80, 80), (80, 160, 200), (200, 200, 80),
]
BRUSH_COLORS = [
    (160, 120, 80), (80, 80, 80), (200, 160, 100),
]
CAN_COLORS = [
    (180, 180, 180), (200, 100, 60), (60, 80, 160),
]
TRASH_COLORS = [
    (140, 160, 120), (160, 140, 100), (100, 140, 160),
]
BODY_COLORS = [
    (200, 160, 130), (180, 140, 110), (220, 180, 150),
]

# Tile/wood floor colours (warm tones)
FLOOR_TILE_COLORS = [
    (190, 170, 145), (200, 180, 155), (185, 165, 140),
    (195, 175, 150),
]
WALL_TILE_COLORS = [
    (210, 200, 185), (220, 210, 195), (205, 195, 180),
]
WOOD_COLORS = [
    (160, 120, 80), (170, 130, 90), (155, 115, 75),
]


# ── Perspective helpers ───────────────────────────────────────────────────────

def perspective_y(world_y: float, horizon: int = 200) -> int:
    """Map world depth [0..1] to image row. 0=horizon, 1=bottom of image."""
    return int(horizon + world_y * (IMG_H - horizon))


def perspective_x(
    world_x: float, world_y: float, center_x: int = IMG_W // 2,
    fov_scale: float = 0.6
) -> int:
    """Map world x [-1..1] to image column with perspective scaling."""
    scale = 0.2 + world_y * fov_scale
    return int(center_x + world_x * IMG_W * 0.5 * scale)


# ── Main generator ─────────────────────────────────────────────────────────────

class SceneGenerator:
    """Generates synthetic onsen bathroom scenes frame by frame."""

    def __init__(self, save_dataset: bool = False, dataset_dir: str = "/ros2_ws/dataset"):
        self._frame_counter = 0
        self._save_dataset = save_dataset
        self._dataset_dir = dataset_dir
        self._rng = random.Random()

    # ── Public API ──────────────────────────────────────────────────────────

    def generate(self) -> tuple[np.ndarray, dict[str, Any]]:
        """Return (rgb_image, ground_truth_dict)."""
        self._frame_counter += 1
        rng = self._rng

        # Scene variation parameters
        floor_type = rng.choice(["wood", "tile"])
        lighting = rng.choice(["normal", "normal", "low", "warm"])
        steam_level = rng.choice(["none", "none", "low", "medium", "high"])
        wet_floor = rng.random() < 0.6

        img = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)

        # Draw layers bottom-up
        self._draw_background(img, floor_type, lighting)
        self._draw_floor(img, floor_type, wet_floor, lighting)
        self._draw_benches(img, lighting)

        # Place objects and collect ground truth
        objects = self._place_objects(img, rng, lighting)

        # Safety: occasionally add a person silhouette
        if rng.random() < 0.12:
            person_gt = self._draw_person(img, rng, lighting)
            if person_gt:
                objects.append(person_gt)

        # Post-process effects
        self._apply_lighting(img, lighting)
        if wet_floor:
            self._apply_wet_floor(img)
        self._apply_steam(img, steam_level)

        ground_truth: dict[str, Any] = {
            "frame_id": f"frame_{self._frame_counter:06d}",
            "environment": "onsen_bathroom",
            "floor_type": floor_type,
            "lighting": lighting,
            "steam_level": steam_level,
            "wet_floor": wet_floor,
            "objects": objects,
        }

        if self._save_dataset:
            self._save_frame(img, ground_truth)

        return img, ground_truth

    # ── Drawing primitives ──────────────────────────────────────────────────

    def _draw_background(self, img: np.ndarray, floor_type: str, lighting: str) -> None:
        """Draw walls and ceiling."""
        horizon = 200

        # Ceiling
        ceiling_color = self._lighting_tint((160, 150, 135), lighting, factor=0.8)
        img[:horizon, :] = ceiling_color

        # Back wall (tiles)
        wall_base = self._rng.choice(WALL_TILE_COLORS)
        wall_color = self._lighting_tint(wall_base, lighting)
        img[horizon - 10:horizon + 60, :] = wall_color

        # Tile grid on back wall
        for tx in range(0, IMG_W, 40):
            cv2.line(img, (tx, horizon - 10), (tx, horizon + 60), self._darken(wall_color, 15), 1)
        for ty in range(horizon - 10, horizon + 60, 30):
            cv2.line(img, (0, ty), (IMG_W, ty), self._darken(wall_color, 15), 1)

        # Left and right walls (perspective trapezoids)
        left_wall_pts = np.array([
            [0, 0], [IMG_W // 4, horizon],
            [0, IMG_H],
        ], np.int32)
        right_wall_pts = np.array([
            [IMG_W, 0], [3 * IMG_W // 4, horizon],
            [IMG_W, IMG_H],
        ], np.int32)
        side_color = self._lighting_tint(self._rng.choice(WALL_TILE_COLORS), lighting, factor=0.9)
        cv2.fillPoly(img, [left_wall_pts], side_color)
        cv2.fillPoly(img, [right_wall_pts], side_color)

    def _draw_floor(
        self, img: np.ndarray, floor_type: str, wet: bool, lighting: str
    ) -> None:
        """Draw the perspective floor."""
        horizon = 200
        # Floor trapezoid
        floor_pts = np.array([
            [0, IMG_H], [IMG_W, IMG_H],
            [3 * IMG_W // 4, horizon], [IMG_W // 4, horizon],
        ], np.int32)

        base_color = self._rng.choice(WOOD_COLORS if floor_type == "wood" else FLOOR_TILE_COLORS)
        floor_color = self._lighting_tint(base_color, lighting)
        cv2.fillPoly(img, [floor_pts], floor_color)

        if floor_type == "tile":
            self._draw_floor_tiles(img, horizon, floor_color, lighting)
        else:
            self._draw_floor_wood(img, horizon, floor_color, lighting)

    def _draw_floor_tiles(
        self, img: np.ndarray, horizon: int, base_color: tuple, lighting: str
    ) -> None:
        """Overlay perspective tile grid on floor."""
        # Horizontal lines (perspective spacing)
        for row_frac in [0.1, 0.25, 0.45, 0.65, 0.85, 1.0]:
            y = int(horizon + row_frac * (IMG_H - horizon))
            scale = 0.2 + row_frac * 0.6
            x_left = int(IMG_W // 2 - IMG_W * 0.5 * scale)
            x_right = int(IMG_W // 2 + IMG_W * 0.5 * scale)
            cv2.line(img, (x_left, y), (x_right, y), self._darken(base_color, 12), 1)

        # Vertical lines (perspective converge to horizon center)
        vp_x = IMG_W // 2
        vp_y = horizon
        for x_frac in [-0.8, -0.5, -0.25, 0.0, 0.25, 0.5, 0.8]:
            x_bottom = int(IMG_W // 2 + x_frac * IMG_W * 0.5)
            cv2.line(
                img, (vp_x, vp_y), (x_bottom, IMG_H),
                self._darken(base_color, 12), 1,
            )

    def _draw_floor_wood(
        self, img: np.ndarray, horizon: int, base_color: tuple, lighting: str
    ) -> None:
        """Overlay perspective wood plank lines on floor."""
        vp_x = IMG_W // 2
        vp_y = horizon
        # Planks run horizontally but converge
        for frac in [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]:
            y = int(horizon + frac * (IMG_H - horizon))
            scale = 0.2 + frac * 0.6
            x_left = int(IMG_W // 2 - IMG_W * 0.5 * scale)
            x_right = int(IMG_W // 2 + IMG_W * 0.5 * scale)
            cv2.line(img, (x_left, y), (x_right, y), self._darken(base_color, 8), 1)
        # Grain marks
        for gx in [-0.7, -0.45, -0.2, 0.05, 0.3, 0.55]:
            x_bot = int(IMG_W // 2 + gx * IMG_W * 0.5)
            cv2.line(img, (vp_x, vp_y), (x_bot, IMG_H), self._darken(base_color, 5), 1)

    def _draw_benches(self, img: np.ndarray, lighting: str) -> None:
        """Draw static wooden benches near walls."""
        bench_color = self._lighting_tint((130, 95, 60), lighting, factor=0.85)
        leg_color = self._darken(bench_color, 25)
        # Left bench
        cv2.rectangle(img, (30, 300), (170, 330), bench_color, -1)
        cv2.rectangle(img, (30, 330), (170, 332), self._darken(bench_color, 20), -1)
        cv2.rectangle(img, (35, 330), (55, 380), leg_color, -1)
        cv2.rectangle(img, (145, 330), (165, 380), leg_color, -1)
        # Right bench
        cv2.rectangle(img, (470, 300), (610, 330), bench_color, -1)
        cv2.rectangle(img, (470, 330), (610, 332), self._darken(bench_color, 20), -1)
        cv2.rectangle(img, (475, 330), (495, 380), leg_color, -1)
        cv2.rectangle(img, (585, 330), (605, 380), leg_color, -1)

    def _place_objects(
        self, img: np.ndarray, rng: random.Random, lighting: str
    ) -> list[dict]:
        """Randomly place floor objects and return their ground-truth entries."""
        objects: list[dict] = []
        obj_id = 1

        # Floor region: y in [0.1 .. 1.0] (world depth), x in [-0.8 .. 0.8]
        n_objects = rng.randint(2, 8)

        candidates = ["towel"] * 4 + [
            "slipper", "bottle", "bucket", "brush",
            "can", "plastic_trash", "bath_mat",
        ]

        placed_bboxes: list[tuple[int, int, int, int]] = []

        for _ in range(n_objects):
            cls = rng.choice(candidates)
            depth = rng.uniform(0.15, 0.95)
            cx_world = rng.uniform(-0.75, 0.75)
            cx = perspective_x(cx_world, depth)
            cy = perspective_y(depth)

            # Size scales with depth
            scale = 0.3 + depth * 0.7

            gt = self._draw_object(img, cls, cx, cy, scale, rng, lighting, obj_id)
            if gt is None:
                continue

            bbox = gt["bbox"]
            # Reject heavy overlaps with already-placed objects
            if self._heavy_overlap(bbox, placed_bboxes):
                continue

            # Record estimated robot-frame position (rough)
            gt["estimated_position"] = {
                "x": round((1.0 - depth) * 4.0 + 0.3, 2),
                "y": round(cx_world * 2.0, 2),
                "z": 0.0,
            }
            placed_bboxes.append(bbox)
            objects.append(gt)
            obj_id += 1

        return objects

    def _draw_object(
        self,
        img: np.ndarray,
        cls: str,
        cx: int,
        cy: int,
        scale: float,
        rng: random.Random,
        lighting: str,
        obj_id: int,
    ) -> dict | None:
        """Draw one object at (cx, cy) with given scale. Returns GT dict or None."""
        methods = {
            "towel":        self._draw_towel,
            "slipper":      self._draw_slipper,
            "bottle":       self._draw_bottle,
            "bucket":       self._draw_bucket,
            "brush":        self._draw_brush,
            "can":          self._draw_can,
            "plastic_trash":self._draw_trash,
            "bath_mat":     self._draw_bath_mat,
        }
        draw_fn = methods.get(cls)
        if draw_fn is None:
            return None

        bbox = draw_fn(img, cx, cy, scale, rng, lighting)
        if bbox is None:
            return None

        x1, y1, x2, y2 = bbox
        state = self._object_state(cls, rng)

        return {
            "id": f"obj_{obj_id:03d}",
            "class": cls,
            "robot_class": ROBOT_CLASS_MAP.get(cls, "unknown_obstacle"),
            "bbox": [x1, y1, x2, y2],
            "pickable": cls in PICKABLE,
            "state": state,
            "risk": RISK_MAP.get(cls, "avoid"),
        }

    def _draw_towel(
        self,
        img: np.ndarray,
        cx: int, cy: int, scale: float,
        rng: random.Random, lighting: str,
    ) -> tuple | None:
        color = self._lighting_tint(rng.choice(TOWEL_COLORS), lighting)
        state = rng.choice(["folded", "crumpled", "spread"])
        if state == "spread":
            w, h = int(80 * scale), int(40 * scale)
            angle = rng.uniform(-30, 30)
            return self._draw_rotated_rect(img, cx, cy, w, h, angle, color)
        elif state == "folded":
            w, h = int(50 * scale), int(30 * scale)
            return self._draw_rect(img, cx, cy, w, h, color)
        else:  # crumpled — irregular polygon
            pts = self._crumple_pts(cx, cy, int(45 * scale), rng)
            cv2.fillPoly(img, [pts], color)
            cv2.polylines(img, [pts], True, self._darken(color, 20), 1)
            xs, ys = pts[:, 0], pts[:, 1]
            return (xs.min(), ys.min(), xs.max(), ys.max())

    def _draw_slipper(
        self,
        img: np.ndarray,
        cx: int, cy: int, scale: float,
        rng: random.Random, lighting: str,
    ) -> tuple | None:
        color = self._lighting_tint(rng.choice(SLIPPER_COLORS), lighting)
        w, h = int(40 * scale), int(18 * scale)
        angle = rng.uniform(-20, 20)
        bbox = self._draw_rotated_rect(img, cx, cy, w, h, angle, color)
        # Toe bump
        toe_r = max(4, int(10 * scale))
        cv2.circle(img, (cx + int(w * 0.3 * math.cos(math.radians(angle))),
                         cy + int(w * 0.3 * math.sin(math.radians(angle)))),
                   toe_r, self._darken(color, 15), -1)
        return bbox

    def _draw_bottle(
        self,
        img: np.ndarray,
        cx: int, cy: int, scale: float,
        rng: random.Random, lighting: str,
    ) -> tuple | None:
        color = self._lighting_tint(rng.choice(BOTTLE_COLORS), lighting)
        w, h = int(18 * scale), int(50 * scale)
        angle = rng.uniform(-10, 10)
        # Body
        bbox = self._draw_rotated_rect(img, cx, cy + int(h * 0.1), w, h, angle, color)
        # Cap
        cap_color = self._darken(color, 30)
        cv2.circle(img, (cx, cy - int(h * 0.45)), max(3, int(7 * scale)), cap_color, -1)
        return bbox

    def _draw_bucket(
        self,
        img: np.ndarray,
        cx: int, cy: int, scale: float,
        rng: random.Random, lighting: str,
    ) -> tuple | None:
        color = self._lighting_tint(rng.choice(BUCKET_COLORS), lighting)
        # Trapezoid (wider at top)
        top_w = int(40 * scale)
        bot_w = int(30 * scale)
        h = int(40 * scale)
        pts = np.array([
            [cx - top_w // 2, cy - h // 2],
            [cx + top_w // 2, cy - h // 2],
            [cx + bot_w // 2, cy + h // 2],
            [cx - bot_w // 2, cy + h // 2],
        ], np.int32)
        cv2.fillPoly(img, [pts], color)
        cv2.polylines(img, [pts], True, self._darken(color, 30), 2)
        # Handle
        cv2.ellipse(img, (cx, cy - h // 2), (top_w // 2, 8), 0, 0, 180,
                    self._darken(color, 40), 2)
        xs, ys = pts[:, 0], pts[:, 1]
        return (xs.min(), ys.min(), xs.max(), ys.max())

    def _draw_brush(
        self,
        img: np.ndarray,
        cx: int, cy: int, scale: float,
        rng: random.Random, lighting: str,
    ) -> tuple | None:
        color = self._lighting_tint(rng.choice(BRUSH_COLORS), lighting)
        w, h = int(60 * scale), int(12 * scale)
        angle = rng.uniform(-40, 40)
        bbox = self._draw_rotated_rect(img, cx, cy, w, h, angle, color)
        # Bristle end
        bristle_color = (200, 200, 150)
        bristle_cx = int(cx + w * 0.4 * math.cos(math.radians(angle)))
        bristle_cy = int(cy + w * 0.4 * math.sin(math.radians(angle)))
        self._draw_rotated_rect(
            img, bristle_cx, bristle_cy,
            int(15 * scale), int(h + 4),
            angle, bristle_color,
        )
        return bbox

    def _draw_can(
        self,
        img: np.ndarray,
        cx: int, cy: int, scale: float,
        rng: random.Random, lighting: str,
    ) -> tuple | None:
        color = self._lighting_tint(rng.choice(CAN_COLORS), lighting)
        r = max(6, int(14 * scale))
        h = int(35 * scale)
        # Cylinder body
        cv2.rectangle(img, (cx - r, cy - h // 2), (cx + r, cy + h // 2), color, -1)
        cv2.ellipse(img, (cx, cy - h // 2), (r, max(3, r // 3)), 0, 0, 360,
                    self._lighten(color, 20), -1)
        cv2.ellipse(img, (cx, cy + h // 2), (r, max(3, r // 3)), 0, 0, 360,
                    self._darken(color, 20), -1)
        cv2.rectangle(img, (cx - r, cy - h // 2), (cx + r, cy + h // 2),
                      self._darken(color, 20), 1)
        return (cx - r, cy - h // 2, cx + r, cy + h // 2)

    def _draw_trash(
        self,
        img: np.ndarray,
        cx: int, cy: int, scale: float,
        rng: random.Random, lighting: str,
    ) -> tuple | None:
        color = self._lighting_tint(rng.choice(TRASH_COLORS), lighting)
        pts = self._crumple_pts(cx, cy, int(25 * scale), rng)
        cv2.fillPoly(img, [pts], color)
        cv2.polylines(img, [pts], True, self._darken(color, 25), 1)
        xs, ys = pts[:, 0], pts[:, 1]
        return (xs.min(), ys.min(), xs.max(), ys.max())

    def _draw_bath_mat(
        self,
        img: np.ndarray,
        cx: int, cy: int, scale: float,
        rng: random.Random, lighting: str,
    ) -> tuple | None:
        color = self._lighting_tint(rng.choice(BATH_MAT_COLORS), lighting)
        w, h = int(90 * scale), int(50 * scale)
        angle = rng.uniform(-15, 15)
        bbox = self._draw_rotated_rect(img, cx, cy, w, h, angle, color)
        # Texture: horizontal lines
        for dy in range(-h // 2 + 5, h // 2, 8):
            lx = cx - w // 2 + 5
            rx = cx + w // 2 - 5
            ly = cy + dy
            cv2.line(img, (lx, ly), (rx, ly), self._lighten(color, 10), 1)
        return bbox

    def _draw_person(
        self, img: np.ndarray, rng: random.Random, lighting: str
    ) -> dict | None:
        """Draw a partial lower-body silhouette for safety-stop testing."""
        side = rng.choice(["left", "right"])
        cx = rng.randint(50, 200) if side == "left" else rng.randint(440, 590)
        cy = rng.randint(350, 440)
        scale = rng.uniform(0.7, 1.2)
        color = self._lighting_tint(rng.choice(BODY_COLORS), lighting)

        # Just legs visible (lower body partial)
        leg_w = int(20 * scale)
        leg_h = int(80 * scale)
        # Left leg
        cv2.rectangle(
            img,
            (cx - leg_w, cy - leg_h // 2),
            (cx, cy + leg_h // 2),
            color, -1,
        )
        # Right leg
        cv2.rectangle(
            img,
            (cx + 4, cy - leg_h // 2),
            (cx + leg_w + 4, cy + leg_h // 2),
            self._darken(color, 15), -1,
        )
        x1 = cx - leg_w
        y1 = cy - leg_h // 2
        x2 = cx + leg_w + 4
        y2 = cy + leg_h // 2
        return {
            "id": "obj_person",
            "class": "person_body_part",
            "robot_class": "safety_stop",
            "bbox": [x1, y1, x2, y2],
            "pickable": False,
            "state": "standing",
            "risk": "stop",
            "estimated_position": {"x": 0.8, "y": -1.0 if side == "left" else 1.0, "z": 0.0},
        }

    # ── Post-process effects ────────────────────────────────────────────────

    def _apply_lighting(self, img: np.ndarray, lighting: str) -> None:
        if lighting == "low":
            img[:] = (img * 0.55).astype(np.uint8)
        elif lighting == "warm":
            overlay = np.zeros_like(img)
            overlay[:, :] = (20, 10, 0)
            img[:] = np.clip(img.astype(np.int16) + overlay, 0, 255).astype(np.uint8)
            img[:] = (img * 0.88).astype(np.uint8)
        # normal: no change

    def _apply_wet_floor(self, img: np.ndarray) -> None:
        """Add subtle specular reflections on the floor area."""
        horizon = 200
        n_spots = self._rng.randint(3, 8)
        for _ in range(n_spots):
            sx = self._rng.randint(60, IMG_W - 60)
            sy = self._rng.randint(horizon + 30, IMG_H - 20)
            rw = self._rng.randint(15, 50)
            rh = self._rng.randint(3, 10)
            alpha = self._rng.uniform(0.15, 0.35)
            overlay = img.copy()
            cv2.ellipse(overlay, (sx, sy), (rw, rh), 0, 0, 360, (240, 240, 255), -1)
            cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)

    def _apply_steam(self, img: np.ndarray, level: str) -> None:
        if level == "none":
            return
        density = {"low": 0.15, "medium": 0.30, "high": 0.55}[level]
        fog = np.ones_like(img) * 200
        cv2.addWeighted(fog, density, img, 1.0 - density, 0, img)
        # Add wispy patches
        n_patches = {"low": 2, "medium": 5, "high": 10}[level]
        for _ in range(n_patches):
            px = self._rng.randint(0, IMG_W)
            py = self._rng.randint(0, IMG_H // 2 + 100)
            pw = self._rng.randint(40, 150)
            ph = self._rng.randint(20, 80)
            patch = img.copy()
            cv2.ellipse(patch, (px, py), (pw, ph), 0, 0, 360, (210, 210, 215), -1)
            cv2.addWeighted(patch, 0.2, img, 0.8, 0, img)

    # ── Geometry helpers ────────────────────────────────────────────────────

    @staticmethod
    def _draw_rect(
        img: np.ndarray, cx: int, cy: int, w: int, h: int, color: tuple
    ) -> tuple[int, int, int, int]:
        x1, y1 = cx - w // 2, cy - h // 2
        x2, y2 = cx + w // 2, cy + h // 2
        cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
        cv2.rectangle(img, (x1, y1), (x2, y2), SceneGenerator._darken(color, 20), 1)
        return (x1, y1, x2, y2)

    @staticmethod
    def _draw_rotated_rect(
        img: np.ndarray,
        cx: int, cy: int, w: int, h: int,
        angle_deg: float, color: tuple,
    ) -> tuple[int, int, int, int]:
        box = cv2.boxPoints(((float(cx), float(cy)), (float(w), float(h)), angle_deg))
        box = box.astype(np.int32)
        cv2.fillPoly(img, [box], color)
        cv2.polylines(img, [box], True, SceneGenerator._darken(color, 20), 1)
        xs, ys = box[:, 0], box[:, 1]
        return (xs.min(), ys.min(), xs.max(), ys.max())

    @staticmethod
    def _crumple_pts(cx: int, cy: int, radius: int, rng: random.Random) -> np.ndarray:
        n = rng.randint(6, 12)
        angles = sorted([rng.uniform(0, 2 * math.pi) for _ in range(n)])
        pts = []
        for a in angles:
            r = rng.uniform(radius * 0.4, radius)
            pts.append([int(cx + r * math.cos(a)), int(cy + r * math.sin(a))])
        return np.array(pts, np.int32)

    # ── Colour helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _darken(color: tuple, amount: int) -> tuple:
        return tuple(max(0, c - amount) for c in color)

    @staticmethod
    def _lighten(color: tuple, amount: int) -> tuple:
        return tuple(min(255, c + amount) for c in color)

    @staticmethod
    def _lighting_tint(color: tuple, lighting: str, factor: float = 1.0) -> tuple:
        if lighting == "low":
            return tuple(int(c * 0.6 * factor) for c in color)
        elif lighting == "warm":
            r, g, b = color
            return (min(255, int(r * factor * 1.05)),
                    int(g * factor * 0.92),
                    int(b * factor * 0.80))
        else:
            return tuple(int(c * factor) for c in color)

    # ── Overlap check ───────────────────────────────────────────────────────

    @staticmethod
    def _heavy_overlap(
        bbox: tuple[int, int, int, int],
        existing: list[tuple[int, int, int, int]],
        threshold: float = 0.5,
    ) -> bool:
        x1, y1, x2, y2 = bbox
        area = max(1, (x2 - x1) * (y2 - y1))
        for ex1, ey1, ex2, ey2 in existing:
            ix1 = max(x1, ex1)
            iy1 = max(y1, ey1)
            ix2 = min(x2, ex2)
            iy2 = min(y2, ey2)
            if ix2 > ix1 and iy2 > iy1:
                inter = (ix2 - ix1) * (iy2 - iy1)
                if inter / area > threshold:
                    return True
        return False

    # ── Object state labels ─────────────────────────────────────────────────

    @staticmethod
    def _object_state(cls: str, rng: random.Random) -> str:
        if cls == "towel":
            return rng.choice([
                "folded_on_floor", "crumpled_on_floor", "spread_on_floor",
                "hanging", "under_bench",
            ])
        elif cls == "bath_mat":
            return rng.choice(["on_floor", "crumpled"])
        elif cls in ("bottle", "can"):
            return rng.choice(["standing", "fallen"])
        elif cls == "bucket":
            return rng.choice(["upright", "tipped"])
        else:
            return "on_floor"

    # ── Dataset export ──────────────────────────────────────────────────────

    def _save_frame(self, img: np.ndarray, gt: dict) -> None:
        import json
        import os

        frame_id = gt["frame_id"]
        img_dir = os.path.join(self._dataset_dir, "images")
        ann_dir = os.path.join(self._dataset_dir, "annotations")
        os.makedirs(img_dir, exist_ok=True)
        os.makedirs(ann_dir, exist_ok=True)

        # Save image as BGR (OpenCV default)
        img_path = os.path.join(img_dir, f"{frame_id}.jpg")
        cv2.imwrite(img_path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

        ann_path = os.path.join(ann_dir, f"{frame_id}.json")
        with open(ann_path, "w") as f:
            json.dump(gt, f, indent=2)

"""
ai_worker_node.py
Subscribes to the camera image and uses OpenCV heuristics to detect
synthetic onsen objects. Publishes detection results and a simple task plan.

Design note: this is an intentionally simple baseline using color/shape analysis,
not a trained model. It is designed to be replaced by a real model later while
keeping the topic contract identical.
"""
from __future__ import annotations

import json
import os
import random
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

# ── HSV detection bands ───────────────────────────────────────────────────────
# Each entry: (class_name, lower_hsv, upper_hsv, min_area, robot_class, pickable)
# These thresholds match the synthetic SceneGenerator colour palette.
DETECTION_PROFILES = [
    # Towels: cream/off-white, light blue-grey
    ("towel",
     np.array([0,   0, 160], np.uint8), np.array([30,  60, 240], np.uint8),
     800, "pickable_soft_object", True),
    # Slippers: light brown
    ("slipper",
     np.array([10, 40,  80], np.uint8), np.array([30, 130, 200], np.uint8),
     300, "pickable_soft_object", True),
    # Bottles: light cyan / light green
    ("bottle",
     np.array([85, 40, 100], np.uint8), np.array([105, 180, 240], np.uint8),
     200, "non_pickable_hard_object", False),
    # Buckets: red, blue, yellow — detect as hard objects
    ("bucket",
     np.array([0,  100, 100], np.uint8), np.array([10, 255, 255], np.uint8),
     400, "non_pickable_hard_object", False),
    # Bucket (blue range)
    ("bucket",
     np.array([100, 100, 100], np.uint8), np.array([130, 255, 255], np.uint8),
     400, "non_pickable_hard_object", False),
    # Plastic trash: muted greens/greys
    ("plastic_trash",
     np.array([35, 20, 80],  np.uint8), np.array([85,  80, 180], np.uint8),
     150, "pickable_soft_object", True),
    # Bath mat: dark grey/brown
    ("bath_mat",
     np.array([0,   0,  60], np.uint8), np.array([30,  60, 155], np.uint8),
     500, "pickable_soft_object", True),
    # Person (warm skin tones)
    ("person_body_part",
     np.array([0,  50, 140], np.uint8), np.array([20, 150, 230], np.uint8),
     600, "safety_stop", False),
]

RISK_MAP = {
    "towel":           "low",
    "slipper":         "low",
    "bottle":          "avoid",
    "bucket":          "avoid",
    "plastic_trash":   "low",
    "bath_mat":        "low",
    "person_body_part":"stop",
    "unknown_obstacle":"stop",
}

NEXT_ACTION_MAP = {
    "person_body_part": "stop_for_person",
    "unknown_obstacle": "stop_for_person",
    "towel":            "pick_object",
    "slipper":          "pick_object",
    "plastic_trash":    "pick_object",
    "bath_mat":         "pick_object",
    "bottle":           "avoid_object",
    "bucket":           "avoid_object",
}


class AIWorkerNode(Node):
    def __init__(self) -> None:
        super().__init__("ai_worker_node")

        camera_input = os.environ.get("AI_CAMERA_INPUT", "raw").lower()
        self._sub_image = None
        self._sub_compressed = None
        if camera_input == "compressed":
            self._sub_compressed = self.create_subscription(
                CompressedImage, "/camera/front/image_raw/compressed",
                self._compressed_image_callback, SENSOR_QOS,
            )
        else:
            self._sub_image = self.create_subscription(
                Image, "/camera/front/image_raw",
                self._image_callback, SENSOR_QOS,
            )
        self._pub_detections = self.create_publisher(String, "/detected_objects", 10)
        self._pub_task_plan  = self.create_publisher(String, "/task_plan", 10)

        self._frame_count = 0
        self._rng = random.Random()
        self._min_process_interval = float(os.environ.get("AI_PROCESS_MIN_INTERVAL", "0.08"))
        self._last_processed_at = 0.0
        self._start_upload_server(port=5000)
        self.get_logger().info(f"AIWorkerNode started — waiting for images ({camera_input})")

    def _start_upload_server(self, port: int) -> None:
        node = self

        class _Handler(BaseHTTPRequestHandler):
            def do_OPTIONS(self):
                self._cors()
                self.end_headers()

            def do_POST(self):
                if self.path != '/upload':
                    self.send_error(404)
                    return
                length = int(self.headers.get('Content-Length', 0))
                if length == 0:
                    self.send_error(400, 'Empty body')
                    return
                raw = self.rfile.read(length)
                img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    self.send_error(400, 'Cannot decode image')
                    return
                detections = node._detect(img)
                node._publish_detections(detections)
                node._publish_task_plan(detections)
                body = json.dumps({'objects': detections}).encode()
                self._cors()
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', len(body))
                self.end_headers()
                self.wfile.write(body)

            def _cors(self):
                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')

            def log_message(self, *_):
                return

        server = HTTPServer(('', port), _Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info(f"Upload HTTP server listening on :{port}/upload")

    def _image_callback(self, msg: Image) -> None:
        img = self._ros_image_to_bgr(msg)
        if img is None:
            return
        self._process_frame(img)

    def _compressed_image_callback(self, msg: CompressedImage) -> None:
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return
        self._process_frame(img)

    def _process_frame(self, img: np.ndarray) -> None:
        now = time.monotonic()
        if now - self._last_processed_at < self._min_process_interval:
            return
        self._last_processed_at = now
        self._frame_count += 1
        detections = self._detect(img)
        self._publish_detections(detections)
        self._publish_task_plan(detections)

    # ── Detection ─────────────────────────────────────────────────────────────

    def _detect(self, img_bgr: np.ndarray) -> list[dict[str, Any]]:
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        # Blur to reduce noise from drawn edges
        hsv = cv2.GaussianBlur(hsv, (7, 7), 0)

        results: list[dict[str, Any]] = []
        obj_id = 1

        for (cls, lo, hi, min_area, robot_cls, pickable) in DETECTION_PROFILES:
            mask = cv2.inRange(hsv, lo, hi)
            # Morphological cleanup
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < min_area:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)

                # Skip tiny bounding boxes
                if w < 10 or h < 10:
                    continue

                # Confidence heuristic: larger area → higher confidence
                raw_conf = min(0.98, 0.45 + area / 25000.0)
                # Add stochastic jitter to make it look more realistic
                confidence = round(
                    max(0.30, min(0.98, raw_conf + self._rng.uniform(-0.08, 0.08))),
                    2,
                )

                # Rough depth from vertical position (lower = closer)
                center_y = y + h / 2
                depth_frac = (center_y - 200) / (480 - 200)
                depth_frac = max(0.05, min(1.0, depth_frac))
                est_x = round((1.0 - depth_frac) * 4.0 + 0.2, 2)
                cx_world = (x + w / 2 - 320) / 320.0
                est_y = round(cx_world * 2.0, 2)

                results.append({
                    "id": f"det_{obj_id:03d}",
                    "class": cls,
                    "confidence": confidence,
                    "bbox": [x, y, x + w, y + h],
                    "robot_class": robot_cls,
                    "pickable": pickable,
                    "risk": RISK_MAP.get(cls, "avoid"),
                    "estimated_position": {"x": est_x, "y": est_y, "z": 0.0},
                })
                obj_id += 1

        # De-duplicate heavily overlapping boxes of same class
        results = self._nms(results, iou_threshold=0.5)
        return results

    @staticmethod
    def _nms(
        detections: list[dict], iou_threshold: float = 0.5
    ) -> list[dict]:
        """Simple class-agnostic greedy NMS."""
        if not detections:
            return detections
        # Sort by confidence descending
        det_sorted = sorted(detections, key=lambda d: -d["confidence"])
        keep: list[dict] = []
        suppressed = set()
        for i, d in enumerate(det_sorted):
            if i in suppressed:
                continue
            keep.append(d)
            for j in range(i + 1, len(det_sorted)):
                if j in suppressed:
                    continue
                if AIWorkerNode._iou(d["bbox"], det_sorted[j]["bbox"]) > iou_threshold:
                    suppressed.add(j)
        return keep

    @staticmethod
    def _iou(a: list[int], b: list[int]) -> float:
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

    # ── Publishers ─────────────────────────────────────────────────────────────

    def _publish_detections(self, detections: list[dict]) -> None:
        payload = {
            "timestamp": _iso_now(),
            "frame_id": self._frame_count,
            "objects": detections,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._pub_detections.publish(msg)

    def _publish_task_plan(self, detections: list[dict]) -> None:
        # Priority: stop for person > avoid hard objects > pick soft objects > search
        task = "collect_onsen_floor_garbage"

        person = next((d for d in detections if d["class"] == "person_body_part"), None)
        if person:
            plan = {
                "task": task,
                "next_action": "stop_for_person",
                "target_object_id": person["id"],
                "reason": "person body part detected — safety stop",
            }
            self._pub_task_plan.publish(String(data=json.dumps(plan)))
            return

        # Find nearest pickable object by estimated_position.x (smaller = closer)
        pickable = [d for d in detections if d.get("pickable")]
        if pickable:
            target = min(pickable, key=lambda d: d["estimated_position"]["x"])
            plan = {
                "task": task,
                "next_action": NEXT_ACTION_MAP.get(target["class"], "navigate_to_object"),
                "target_object_id": target["id"],
                "reason": f"nearest pickable {target['class']} detected "
                          f"(confidence={target['confidence']})",
            }
        elif detections:
            # Hard objects present — navigate around
            target = detections[0]
            plan = {
                "task": task,
                "next_action": "avoid_object",
                "target_object_id": target["id"],
                "reason": f"non-pickable {target['class']} blocking path",
            }
        else:
            plan = {
                "task": task,
                "next_action": "continue_search",
                "target_object_id": None,
                "reason": "no objects detected in current frame",
            }

        self._pub_task_plan.publish(String(data=json.dumps(plan)))

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _ros_image_to_bgr(msg: Image) -> np.ndarray | None:
        """Convert ROS Image to OpenCV BGR array."""
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            data = data.reshape((msg.height, msg.width, -1))
            if msg.encoding == "rgb8":
                return cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "bgr8":
                return data.copy()
            else:
                return None
        except Exception:
            return None


def _iso_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AIWorkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

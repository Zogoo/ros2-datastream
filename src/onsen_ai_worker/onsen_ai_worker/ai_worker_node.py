"""AI worker ROS node: camera frames -> Detector -> /detected_objects and
TaskPlanner -> /task_plan. Hosts the HTTP API for uploads + skin-profile
resampling. Heavy logic lives in detection.py / planner.py (pure modules).
"""
from __future__ import annotations

import json
import os
import time
from datetime import UTC, datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String

from .detection import Detector, load_camera_model
from .http_api import start_http_api
from .planner import TaskPlanner

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class AIWorkerNode(Node):
    def __init__(self) -> None:
        super().__init__("ai_worker_node")

        try:
            camera_model = load_camera_model()
        except (OSError, KeyError) as exc:
            self.get_logger().warning(f"robot_spec.json unavailable ({exc}) — positions disabled")
            camera_model = None
        self._detector = Detector(camera_model)
        self._planner = TaskPlanner()
        self._safety_stop = False
        self._frame_count = 0
        self._last_processed_at = 0.0
        self._min_interval = float(os.environ.get("AI_PROCESS_MIN_INTERVAL", "0.15"))

        camera_input = os.environ.get("AI_CAMERA_INPUT", "compressed").lower()
        if camera_input == "raw":
            self.create_subscription(
                Image, "/camera/front/image_raw", self._on_image, SENSOR_QOS,
            )
        else:
            self.create_subscription(
                CompressedImage, "/camera/front/image_raw/compressed",
                self._on_compressed, SENSOR_QOS,
            )
        self.create_subscription(Bool, "/safety/stop", self._on_safety, 10)

        self._pub_detections = self.create_publisher(String, "/detected_objects", 10)
        self._pub_task_plan = self.create_publisher(String, "/task_plan", 10)

        port = int(os.environ.get("AI_HTTP_PORT", "5000"))
        start_http_api(port, self._detector, self._publish_results)
        self.get_logger().info(
            f"AIWorkerNode ready — input={camera_input}, HTTP API on :{port}",
        )

    def _on_safety(self, msg: Bool) -> None:
        self._safety_stop = bool(msg.data)

    def _on_image(self, msg: Image) -> None:
        img = ros_image_to_bgr(msg)
        if img is not None:
            self._process(img)

    def _on_compressed(self, msg: CompressedImage) -> None:
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            self._process(img)

    def _process(self, img: np.ndarray) -> None:
        now = time.monotonic()
        if now - self._last_processed_at < self._min_interval:
            return
        self._last_processed_at = now
        self._frame_count += 1
        self._publish_results(self._detector.detect(img))

    def _publish_results(self, detections: list[dict]) -> None:
        self._pub_detections.publish(String(data=json.dumps({
            "timestamp": datetime.now(UTC).isoformat(),
            "frame_id": self._frame_count,
            "objects": detections,
        })))
        plan = self._planner.plan(detections, safety_stop=self._safety_stop)
        self._pub_task_plan.publish(String(data=json.dumps(plan)))


def ros_image_to_bgr(msg: Image) -> np.ndarray | None:
    try:
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
    except ValueError:
        return None
    if msg.encoding == "rgb8":
        return cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
    if msg.encoding == "bgr8":
        return data.copy()
    return None


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

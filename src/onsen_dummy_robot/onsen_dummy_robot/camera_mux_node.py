from __future__ import annotations

from typing import Literal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


class CameraMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_mux_node")
        self._active_source: Literal["dummy", "fps"] = "dummy"

        self._pub_image = self.create_publisher(Image, "/camera/front/image_raw", SENSOR_QOS)
        self._pub_compressed = self.create_publisher(CompressedImage, "/camera/front/image_raw/compressed", SENSOR_QOS)
        self._pub_caminfo = self.create_publisher(CameraInfo, "/camera/front/camera_info", SENSOR_QOS)

        self.create_subscription(Image, "/camera/source/dummy/image_raw", self._on_dummy_image, SENSOR_QOS)
        self.create_subscription(CompressedImage, "/camera/source/dummy/image_raw/compressed", self._on_dummy_compressed, SENSOR_QOS)
        self.create_subscription(CameraInfo, "/camera/source/dummy/camera_info", self._on_dummy_info, SENSOR_QOS)

        self.create_subscription(Image, "/camera/source/fps/image_raw", self._on_fps_image, SENSOR_QOS)
        self.create_subscription(CompressedImage, "/camera/source/fps/image_raw/compressed", self._on_fps_compressed, SENSOR_QOS)
        self.create_subscription(CameraInfo, "/camera/source/fps/camera_info", self._on_fps_info, SENSOR_QOS)

        self.create_subscription(String, "/camera/source/select", self._on_select_source, 10)
        self._pub_source = self.create_publisher(String, "/camera/source/active", 10)
        self.create_timer(0.5, self._publish_active_source)
        self.get_logger().info("CameraMuxNode started — active source: dummy")

    def _on_select_source(self, msg: String) -> None:
        source = msg.data.strip().lower()
        if source in ("dummy", "fps") and source != self._active_source:
            self._active_source = source
            self.get_logger().info(f"Camera source switched to: {source}")

    def _publish_active_source(self) -> None:
        self._pub_source.publish(String(data=self._active_source))

    def _on_dummy_image(self, msg: Image) -> None:
        if self._active_source == "dummy":
            self._pub_image.publish(msg)

    def _on_dummy_compressed(self, msg: CompressedImage) -> None:
        if self._active_source == "dummy":
            self._pub_compressed.publish(msg)

    def _on_dummy_info(self, msg: CameraInfo) -> None:
        if self._active_source == "dummy":
            self._pub_caminfo.publish(msg)

    def _on_fps_image(self, msg: Image) -> None:
        if self._active_source == "fps":
            self._pub_image.publish(msg)

    def _on_fps_compressed(self, msg: CompressedImage) -> None:
        if self._active_source == "fps":
            self._pub_compressed.publish(msg)

    def _on_fps_info(self, msg: CameraInfo) -> None:
        if self._active_source == "fps":
            self._pub_caminfo.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


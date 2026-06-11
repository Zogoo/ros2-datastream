"""ROS2 wrapper around the arm firmware emulation.

Topics:
  /arm/command        std_msgs/String  in   firmware protocol line (e.g. "A HOME")
  /arm/response       std_msgs/String  out  firmware reply lines
  /arm/joint_targets  std_msgs/String  out  JSON {"deg":[6], "status", "ts"} @ 20 Hz
  /arm/state          std_msgs/String  out  JSON firmware state @ 10 Hz
"""
from __future__ import annotations

import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .arm_protocol import ArmFirmware

CAL_PATH = os.environ.get("ARM_CAL_PATH", "/ros2_ws/output/arm_calibration.json")


class ArmControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_controller_node")
        self._fw = ArmFirmware(cal_path=CAL_PATH)

        self._pub_response = self.create_publisher(String, "/arm/response", 50)
        self._pub_targets = self.create_publisher(String, "/arm/joint_targets", 10)
        self._pub_state = self.create_publisher(String, "/arm/state", 10)
        self.create_subscription(String, "/arm/command", self._on_command, 50)

        self.create_timer(0.05, self._tick)        # 20 Hz interpolation + targets
        self.create_timer(0.10, self._publish_state)
        self.get_logger().info("ArmControllerNode ready — /arm/command accepted")

    def _on_command(self, msg: String) -> None:
        for line in msg.data.splitlines():
            if not line.strip():
                continue
            for reply in self._fw.handle(line):
                self._pub_response.publish(String(data=reply))
            self.get_logger().debug(f"arm cmd: {line.strip()}")

    def _tick(self) -> None:
        self._fw.tick()
        self._pub_targets.publish(String(data=json.dumps({
            "deg": [round(p, 2) for p in self._fw.positions],
            "status": self._fw.status,
            "ts": time.time(),
        })))

    def _publish_state(self) -> None:
        self._pub_state.publish(String(data=json.dumps(self._fw.state_dict())))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

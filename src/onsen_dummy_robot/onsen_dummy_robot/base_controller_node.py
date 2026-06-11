"""ROS2 wrapper around the base firmware emulation (base_protocol.py).

Topics:
  /base/command       std_msgs/String     in   protocol line (Q, V, T, W, ...)
  /cmd_vel            geometry_msgs/Twist in   arbitrated twist interface
  /safety/stop        std_msgs/Bool       in   e-stop latch from the safety worker
  /base/response      std_msgs/String     out  protocol replies
  /base/wheel_targets std_msgs/String     out  JSON {"w":[rad/s x6],"ts"} @ 20 Hz
  /base/state         std_msgs/String     out  JSON controller state @ 10 Hz
"""
from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .base_protocol import BaseFirmware


class BaseControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("base_controller_node")
        self._fw = BaseFirmware()

        self._pub_response = self.create_publisher(String, "/base/response", 50)
        self._pub_targets = self.create_publisher(String, "/base/wheel_targets", 10)
        self._pub_state = self.create_publisher(String, "/base/state", 10)

        self.create_subscription(String, "/base/command", self._on_command, 50)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Bool, "/safety/stop", self._on_safety, 10)

        self.create_timer(0.05, self._publish_targets)
        self.create_timer(0.10, self._publish_state)
        self.get_logger().info("BaseControllerNode ready — /base/command + /cmd_vel accepted")

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._fw.set_twist(float(msg.linear.x), float(msg.angular.z))

    def _on_safety(self, msg: Bool) -> None:
        if self._fw.set_safety(bool(msg.data)):
            self.get_logger().warning("SAFETY STOP engaged")

    def _on_command(self, msg: String) -> None:
        for line in msg.data.splitlines():
            if not line.strip():
                continue
            for reply in self._fw.handle(line):
                self._pub_response.publish(String(data=reply))

    def _publish_targets(self) -> None:
        self._pub_targets.publish(String(data=json.dumps({
            "w": [round(w, 3) for w in self._fw.scaled_wheels()],
            "ts": time.time(),
        })))

    def _publish_state(self) -> None:
        self._pub_state.publish(String(data=json.dumps(self._fw.state_dict())))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BaseControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

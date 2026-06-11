from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class ControlArbitratorNode(Node):
    """Single owner of /cmd_vel. Manual UI commands always win; autonomous
    commands flow only in auto mode. Sources go stale after a timeout."""

    def __init__(self) -> None:
        super().__init__("control_arbitrator_node")
        self._mode = "auto"
        self._ui_timeout_s = float(self.declare_parameter("ui_timeout_s", 1.5).value)
        self._auto_timeout_s = float(self.declare_parameter("auto_timeout_s", 1.0).value)
        self._last_ui_ts = -999.0
        self._last_auto_ts = -999.0
        self._ui_cmd = Twist()
        self._auto_cmd = Twist()

        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_mode = self.create_publisher(String, "/robot/control_mode", 10)

        self.create_subscription(Twist, "/cmd_vel/ui", self._on_ui_cmd, 10)
        self.create_subscription(Twist, "/cmd_vel/auto", self._on_auto_cmd, 10)
        self.create_subscription(String, "/robot/control_mode/set", self._on_mode_set, 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info("ControlArbitratorNode started — mode: auto")

    def _on_mode_set(self, msg: String) -> None:
        mode = msg.data.strip().lower()
        if mode in ("auto", "manual"):
            self._mode = mode

    def _on_ui_cmd(self, msg: Twist) -> None:
        self._ui_cmd = msg
        self._last_ui_ts = time.monotonic()
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            self._mode = "manual"

    def _on_auto_cmd(self, msg: Twist) -> None:
        self._auto_cmd = msg
        self._last_auto_ts = time.monotonic()

    def _tick(self) -> None:
        now = time.monotonic()
        ui_fresh = now - self._last_ui_ts < self._ui_timeout_s
        auto_fresh = now - self._last_auto_ts < self._auto_timeout_s
        cmd = Twist()
        active_source = "none"
        if self._mode == "manual" and ui_fresh:
            cmd = self._ui_cmd
            active_source = "ui"
            self._pub_cmd.publish(cmd)
        elif self._mode == "auto" and auto_fresh:
            cmd = self._auto_cmd
            active_source = "auto"
            self._pub_cmd.publish(cmd)

        self._pub_mode.publish(String(data=json.dumps({
            "mode": self._mode,
            "active_source": active_source,
            "ui_fresh": ui_fresh,
            "auto_fresh": auto_fresh,
            "vx": round(float(cmd.linear.x), 3),
            "wz": round(float(cmd.angular.z), 3),
        })))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlArbitratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


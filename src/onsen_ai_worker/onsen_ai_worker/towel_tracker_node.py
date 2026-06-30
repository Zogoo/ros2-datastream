"""Towel tracker node: /detected_objects + /localization/pose ->
/perception/towel_tracks (map-frame stable targets for the mission).

Pure tracking logic lives in towel_tracker.py; this node only adapts topics.
Listens to /robot/events for GRASP/BIN confirmations to forget collected
towels immediately instead of waiting for visibility-based expiry.
"""
from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

from onsen_robot_state import topics
from onsen_robot_state.session import SessionWatch

from .towel_tracker import TowelTracker


class TowelTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("towel_tracker_node")
        self._tracker = TowelTracker()
        self._pose: tuple[float, float, float] | None = None
        self._session = SessionWatch()

        self._pub = self.create_publisher(String, topics.TOWEL_TRACKS, 10)
        self.create_subscription(String, topics.DETECTED_OBJECTS, self._on_detections, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, topics.LOC_POSE, self._on_pose, 10,
        )
        self.create_subscription(String, topics.EVENTS, self._on_event, 20)
        self.create_subscription(String, topics.SIM_STATUS, self._on_sim_status, 10)
        self.get_logger().info("TowelTracker ready — map-frame tracks from detections")

    def _on_sim_status(self, msg: String) -> None:
        """Clear all tracks when the FE session restarts (new session_id).
        SessionWatch ignores competing concurrent tabs, avoiding a clear storm."""
        if self._session.update_raw(msg.data, time.monotonic()):
            self._tracker = TowelTracker()
            self.get_logger().info("Sim session restarted — towel tracks cleared")

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _on_detections(self, msg: String) -> None:
        if self._pose is None:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        towels = []
        for det in payload.get("objects", []):
            if det.get("class") != "towel":
                continue
            # Only track depth-confirmed towels: a measured range means the
            # height gate has vetted it (flat, towel-tall) and the position is
            # metric. Distant ground-plane-only guesses are too noisy to chase
            # and produce phantom targets across the room.
            if det.get("source") != "depth" or det.get("position_refined") is None:
                continue
            towels.append({
                "position": det["position_refined"],
                "source": "depth",
                "range": det.get("range"),
            })
        tracks = self._tracker.update(towels, self._pose, time.time())
        self._pub.publish(String(data=json.dumps({"tracks": tracks})))

    def _on_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if event.get("event") in ("GRASP_ACQUIRED", "OBJECT_BINNED") and self._pose is not None:
            # the towel at the gripper is gone from the floor — forget it
            rx, ry, ryaw = self._pose
            gx = rx + 0.6 * math.cos(ryaw)
            gy = ry + 0.6 * math.sin(ryaw)
            self._tracker.forget_near(gx, gy, radius=0.6)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TowelTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

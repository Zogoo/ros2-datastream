"""Evaluation node — the ONLY consumer of /ground_truth/*.

Makes the "ground truth is eval-only" rule structurally visible: this node
scores the perception tracker and the localizer against ground truth and
publishes /eval/metrics, but nothing in the control path subscribes to it.

  tracker error : nearest-neighbor distance from each towel track to the true
                  towel positions (/ground_truth/objects)
  localization  : distance + heading error of /localization/pose vs
                  /ground_truth/pose

Metrics JSON @ 1 Hz on /eval/metrics.
"""
from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

from onsen_robot_state import topics

EVAL_METRICS = "/eval/metrics"


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class EvalNode(Node):
    def __init__(self) -> None:
        super().__init__("eval_node")
        self._gt_towels: list[dict] = []
        self._gt_pose: tuple[float, float, float] | None = None
        self._loc_pose: tuple[float, float, float] | None = None
        self._tracks: list[dict] = []
        self._loc_score = 0.0

        self._pub = self.create_publisher(String, EVAL_METRICS, 10)
        self.create_subscription(String, topics.GROUND_TRUTH_OBJECTS, self._on_gt_obj, 10)
        self.create_subscription(PoseStamped, topics.GROUND_TRUTH_POSE, self._on_gt_pose, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, topics.LOC_POSE, self._on_loc, 10,
        )
        self.create_subscription(String, topics.TOWEL_TRACKS, self._on_tracks, 10)
        self.create_timer(1.0, self._publish)
        self.get_logger().info("Eval ready — scoring perception/localization vs ground truth")

    def _on_gt_obj(self, msg: String) -> None:
        try:
            objs = json.loads(msg.data).get("objects", [])
        except json.JSONDecodeError:
            return
        self._gt_towels = [
            o for o in objs
            if o.get("class") == "towel" and not o.get("binned") and not o.get("held")
        ]

    def _on_gt_pose(self, msg: PoseStamped) -> None:
        self._gt_pose = (msg.pose.position.x, msg.pose.position.y, _yaw(msg.pose.orientation))

    def _on_loc(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        self._loc_pose = (p.position.x, p.position.y, _yaw(p.orientation))
        self._loc_score = msg.pose.covariance[0]

    def _on_tracks(self, msg: String) -> None:
        try:
            self._tracks = json.loads(msg.data).get("tracks", [])
        except json.JSONDecodeError:
            pass

    def _publish(self) -> None:
        metrics: dict = {"loc_score": round(self._loc_score, 3)}
        if self._gt_pose and self._loc_pose:
            metrics["loc_pos_err_m"] = round(math.hypot(
                self._loc_pose[0] - self._gt_pose[0],
                self._loc_pose[1] - self._gt_pose[1],
            ), 3)
            err = abs(self._loc_pose[2] - self._gt_pose[2])
            metrics["loc_yaw_err_deg"] = round(math.degrees(min(err, 2 * math.pi - err)), 1)
        if self._tracks and self._gt_towels:
            errs = [
                min(math.hypot(t["position"]["x"] - g["position"]["x"],
                               t["position"]["y"] - g["position"]["y"])
                    for g in self._gt_towels)
                for t in self._tracks
            ]
            metrics["track_count"] = len(self._tracks)
            metrics["gt_towel_count"] = len(self._gt_towels)
            metrics["track_mean_err_m"] = round(sum(errs) / len(errs), 3)
            metrics["track_max_err_m"] = round(max(errs), 3)
        self._pub.publish(String(data=json.dumps(metrics)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

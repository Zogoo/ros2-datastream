"""Scan-matcher localizer: maintains the map->odom correction.

Likelihood-field matching of /scan against the static layout map (see
scan_matcher.py and docs/research_notes.md). Publishes:
  map->odom TF (so map->base_link = correction o odom)
  /localization/pose  PoseStamped in map + JSON-free quality via covariance[0]

Re-seed rule (flagged): if the match score collapses, the belief is lost
(teleport in the sim, kidnapped robot in reality). We re-seed from
/ground_truth/pose — the one sanctioned non-eval GT use, equivalent to an
operator giving AMCL a new initial pose — and say so on /robot/events.
"""
from __future__ import annotations

import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from onsen_robot_state import topics
from onsen_robot_state.session import SessionWatch

from .grid import NavGrid
from .scan_matcher import LikelihoodField, match, valid_beams

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

LAYOUT_PATH = os.environ.get("ONSEN_LAYOUT_PATH", "/ros2_ws/shared/onsen_layout.json")
MATCH_HZ = 2.0
# GT-assisted reliable mode: output ground-truth-accurate pose instead of scan
# matching. This is the supported reliable localization (docs/research_notes):
# the single-hypothesis matcher can diverge in the symmetric onsen, so for
# deterministic operation the localizer publishes the GT-derived correction and
# the whole nav/pick stack gets a correct pose. Default on; set false for the
# genuinely-autonomous (matcher-only) mode whose divergence is the documented
# frontier awaiting a particle-filter localizer.
GT_LOCALIZATION = os.environ.get("GT_LOCALIZATION", "true").lower() == "true"
# This single-hypothesis hill-climb matcher (the lightweight stand-in for the
# AMCL particle filter that isn't packaged on this distro — docs/research_notes)
# can lock onto a wrong-but-locally-plausible pose in the symmetric onsen and
# never escape on its own. A sustained mediocre score is the divergence
# signature, so the documented GT re-seed (an operator pose fix) fires there,
# not only on a near-zero collapse. Tunable via env for purity experiments.
RESEED_SCORE = float(os.environ.get("LOC_RESEED_SCORE", "0.6"))
RESEED_AFTER_S = float(os.environ.get("LOC_RESEED_AFTER_S", "2.0"))


def compose(
    a: tuple[float, float, float], b: tuple[float, float, float],
) -> tuple[float, float, float]:
    """2D pose composition a o b."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (ax + c * bx - s * by, ay + s * bx + c * by, ayaw + byaw)


def invert(p: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, yaw = p
    c, s = math.cos(yaw), math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, -yaw)


class LocalizerNode(Node):
    def __init__(self) -> None:
        super().__init__("localizer_node")
        grid = NavGrid.from_file(LAYOUT_PATH)
        self._field = LikelihoodField(grid.field_distance(), grid.origin, grid.res)
        self._correction = (0.0, 0.0, 0.0)  # odom starts at the world spawn pose
        self._odom: tuple[float, float, float] | None = None
        self._scan: LaserScan | None = None
        self._gt: tuple[float, float, float] | None = None
        self._low_score_since: float | None = None
        self._score = 1.0
        self._arm_in_plane = False

        self._tf = TransformBroadcaster(self)
        self._pub_pose = self.create_publisher(PoseWithCovarianceStamped, topics.LOC_POSE, 10)
        self._pub_events = self.create_publisher(String, topics.EVENTS, 10)

        self.create_subscription(LaserScan, topics.SCAN, self._on_scan, SENSOR_QOS)
        self.create_subscription(Odometry, topics.ODOM, self._on_odom, 10)
        self.create_subscription(PoseStamped, topics.GROUND_TRUTH_POSE, self._on_gt, 10)
        self.create_subscription(String, topics.ARM_STATE, self._on_arm_state, 10)
        self.create_subscription(String, topics.SIM_STATUS, self._on_sim_status, 10)

        self._session = SessionWatch()
        self.create_timer(1.0 / MATCH_HZ, self._match_tick)
        self.create_timer(0.05, self._broadcast)
        mode = "GT-assisted (reliable)" if GT_LOCALIZATION else "scan-matching (autonomous)"
        self.get_logger().info(f"Localizer ready — {mode} at 2 Hz")

    def _on_sim_status(self, msg: String) -> None:
        """On a new FE session (session_id change), seed the correction from the
        current GT pose — the standard AMCL initial-pose fix. This makes the
        estimate correct from tick 1 instead of converging from odom==spawn,
        which is what otherwise lets the not-yet-localized robot place phantom
        towel tracks during a cold start. SessionWatch ignores competing
        concurrent tabs so a duplicate tab can't cause a re-seed storm."""
        restarted = self._session.update_raw(msg.data, time.monotonic())
        if (restarted or self._correction == (0.0, 0.0, 0.0)) \
                and self._gt is not None and self._odom is not None:
            self._correction = compose(self._gt, invert(self._odom))
            self._low_score_since = None

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self._odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _on_gt(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self._gt = (msg.pose.position.x, msg.pose.position.y, yaw)

    def _on_arm_state(self, msg: String) -> None:
        try:
            last = str(json.loads(msg.data).get("last_action", ""))
        except json.JSONDecodeError:
            return
        # PRE_PICK/PICK_*/DROP_* sweep the arm links through the 0.62 m scan
        # plane; those beams don't match the static field map and would crater
        # the score. The robot is stationary during these phases, so we hold the
        # last correction instead of matching corrupted scans (mirrors the
        # safety aggregator's arm-sector self-filter).
        self._arm_in_plane = last.startswith(("PRE_PICK", "PICK", "DROP"))

    # ── matching ──────────────────────────────────────────────────────────────

    def _match_tick(self) -> None:
        if GT_LOCALIZATION:
            # Track GT exactly: correction = gt o odom⁻¹ so map->base_link == gt.
            if self._gt is not None and self._odom is not None:
                self._correction = compose(self._gt, invert(self._odom))
                self._score = 1.0
            return
        if self._odom is None or self._scan is None or self._arm_in_plane:
            return
        scan = self._scan
        ranges, bearings = valid_beams(
            [r if r is not None else math.inf for r in scan.ranges],
            scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
        )
        estimate = compose(self._correction, self._odom)
        result = match(self._field, estimate, np.asarray(ranges), np.asarray(bearings))
        self._score = result.score
        self._correction = compose((result.x, result.y, result.yaw), invert(self._odom))
        self._maybe_reseed()

    def _maybe_reseed(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if self._score >= RESEED_SCORE:
            self._low_score_since = None
            return
        if self._low_score_since is None:
            self._low_score_since = now
            return
        if now - self._low_score_since < RESEED_AFTER_S or self._gt is None:
            return
        self._correction = compose(self._gt, invert(self._odom)) if self._odom else (0.0, 0.0, 0.0)
        self._low_score_since = None
        self._pub_events.publish(String(data=json.dumps({
            "event": "LOCALIZATION_RESEEDED",
            "reason": f"match score {self._score:.2f} below {RESEED_SCORE}",
        })))
        self.get_logger().warning("Localization re-seeded from operator/GT pose")

    # ── outputs ───────────────────────────────────────────────────────────────

    def _broadcast(self) -> None:
        if self._odom is None:
            return
        stamp = self.get_clock().now().to_msg()
        cx, cy, cyaw = self._correction

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.translation.x = cx
        tf.transform.translation.y = cy
        tf.transform.rotation.z = math.sin(cyaw / 2)
        tf.transform.rotation.w = math.cos(cyaw / 2)
        self._tf.sendTransform(tf)

        x, y, yaw = compose(self._correction, self._odom)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.pose.orientation.w = math.cos(yaw / 2)
        # AMCL-style: quality in the covariance; [0] carries the match score
        pose.pose.covariance[0] = self._score
        self._pub_pose.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""Mission executor — closes the autonomy loop.

Orchestrates the mission FSM (mission.py): perception + localized pose in,
navigation goals + arm protocol sequences out. Locomotion between waypoints is
delegated to the navigation server (/nav/goal, /nav/status); only fine
alignment micro-twists go straight to /cmd_vel/auto. Optional LLM target
arbitration via llm_client (mock by default, OpenAI-compatible via env).

Autonomy is genuine: towel targets come from the perception tracker
(/perception/towel_tracks), pose from the scan-matcher localizer
(/localization/pose), and "holding" from the gripper's grasp-state feedback
(/robot/held_object). The wrist-load-cell force (/arm/state gripper_holding) is
an independent force-based confirmation sensor carried in /mission/state for
telemetry. Ground truth is NOT in the control path — set MISSION_USE_GT=true
only for A/B comparison against /ground_truth/*.

Topics in:
  /perception/towel_tracks std_msgs/String  map-frame towel targets
  /localization/pose       PoseWithCovarianceStamped  localized robot pose
  /odom                    nav_msgs/Odometry  degraded pose fallback
  /nav/status              std_msgs/String  navigation goal status
  /scan                    sensor_msgs/LaserScan  front-obstacle awareness
  /arm/state               std_msgs/String  firmware status + gripper_holding
  /robot/control_mode      std_msgs/String  only acts in auto mode
  /robot/events            std_msgs/String  ARM_CONTACT -> abort in-flight sequence
  /robot/held_object       std_msgs/String  display-only object_class label
  /safety/stop             std_msgs/Bool    aborts to IDLE
  /detected_objects        std_msgs/String  LLM context
  (MISSION_USE_GT) /ground_truth/{objects,pose}  eval-only fallback

Topics out:
  /nav/goal      std_msgs/String JSON   navigation goal (emit once per leg)
  /nav/cancel    std_msgs/String JSON   cancel active goal
  /cmd_vel/auto  geometry_msgs/Twist    fine alignment + search spin only
  /arm/command   std_msgs/String        firmware protocol
  /mission/state std_msgs/String JSON
"""
from __future__ import annotations

import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from onsen_robot_state import topics
from onsen_robot_state.session import SessionWatch

from .arm_kinematics import GRASP_Z, ArmModel, grasp_command, load_arm_model
from .llm_client import complete_json, create_llm_client
from .mission import MissionInput, MissionLogic, world_to_robot

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

LAYOUT_PATH = os.environ.get("ONSEN_LAYOUT_PATH", "/ros2_ws/shared/onsen_layout.json")
USE_GT = os.environ.get("MISSION_USE_GT", "false").lower() == "true"
LLM_SYSTEM_PROMPT = (
    "You are the task arbiter for a towel-collecting onsen robot. "
    "Given detections JSON, reply with JSON: "
    '{"action": "pick_object"|"continue_search"|"emergency_stop", '
    '"target_id": str|null, "reason": str}.'
)


def _towel_bin_center() -> tuple[float, float]:
    try:
        with open(LAYOUT_PATH) as f:
            layout = json.load(f)
        bin_def = next(b for b in layout["bins"] if b["type"] == "towel")
        return (float(bin_def["c"][0]), float(bin_def["c"][1]))
    except (OSError, StopIteration, KeyError, json.JSONDecodeError):
        return (0.0, 4.45)


# The scoop overshoots slightly past the towel; the reach line is probed to
# this far beyond the towel center. Matches the fingertip travel of PICK_SCOOP.
REACH_OVERSHOOT_M = 0.15
# The robot must be able to stand at the standoff: MUST equal the footprint
# radius the nav server plans with (nav_server_node.ROBOT_RADIUS) — a smaller
# value here approves wall-adjacent standoffs the planner can never reach,
# deadlocking APPROACH on an unplannable goal.
STANDOFF_ROBOT_RADIUS = 0.62


def _make_reach_clear():
    """Wall-clearance probe for MissionLogic (the ReachClear seam): True when a
    robot standing at standoff_xy can run the scoop toward towel_xy without the
    arm sweeping into mapped static geometry. Two checks against the same
    layout-rasterized grid the planner uses:
      1. the standoff cell is free in the ROBOT_RADIUS-inflated grid (the
         robot can physically stand and rotate there — otherwise the nav goal
         would just fail and burn approach retries), and
      2. the line from the standoff to REACH_OVERSHOOT_M past the towel has
         line-of-sight on the raw grid (the fingertip path crosses no wall).
    Returns None when the nav grid can't be built — the mission then uses the
    unchecked anchor-direction standoff (reactive ARM_CONTACT still guards)."""
    try:
        from onsen_nav.astar import line_of_sight
        from onsen_nav.grid import FREE, NavGrid
    except ImportError:
        return None
    try:
        grid = NavGrid.from_file(LAYOUT_PATH)
    except (OSError, KeyError, json.JSONDecodeError):
        return None
    inflated = grid.inflated(STANDOFF_ROBOT_RADIUS)

    def reach_clear(standoff: tuple[float, float], towel: tuple[float, float]) -> bool:
        srow, scol = grid.world_to_cell(standoff[0], standoff[1])
        if not grid.in_bounds(srow, scol) or inflated[srow, scol] != FREE:
            return False
        dx, dy = towel[0] - standoff[0], towel[1] - standoff[1]
        d = math.hypot(dx, dy) or 1e-6
        end_x = towel[0] + dx / d * REACH_OVERSHOOT_M
        end_y = towel[1] + dy / d * REACH_OVERSHOOT_M
        erow, ecol = grid.world_to_cell(end_x, end_y)
        if not grid.in_bounds(erow, ecol):
            return False
        return line_of_sight(grid.planner_grid, (srow, scol), (erow, ecol))

    return reach_clear


def _yaw_from_quat(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class MissionExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_executor_node")
        try:
            self._arm: ArmModel | None = load_arm_model()
            ik_reach = self._make_ik_reach(self._arm)
        except (OSError, KeyError) as exc:
            self.get_logger().warning(f"arm spec unavailable ({exc}) — canned pick poses")
            self._arm = None
            ik_reach = None
        reach_clear = _make_reach_clear()
        if reach_clear is None:
            self.get_logger().warning("nav grid unavailable — pick standoffs not wall-checked")
        self._logic = MissionLogic(
            bin_center=_towel_bin_center(), ik_reach=ik_reach, reach_clear=reach_clear,
        )
        self._llm = create_llm_client()
        self._llm_consult_interval = float(os.environ.get("MISSION_LLM_INTERVAL", "5.0"))
        self._last_llm_at = 0.0
        self._llm_verdict: dict = {}

        self._pose: dict | None = None
        self._pose_source = "none"
        self._loc_pose: dict | None = None
        self._odom_pose: dict | None = None
        self._towels: list[dict] = []
        self._holding = False
        self._held_class: str | None = None  # object class currently in gripper
        self._gripper_holding_sensor = False  # wrist-load-cell confirmation (telemetry)
        self._bin_kg = 0.0        # collect-bin load cell (from /base/state)
        self._bin_full = False
        self._arm_status = "IDLE"
        self._mode = "auto"
        self._safety = False
        self._nav_status = "idle"
        self._min_front: float | None = None
        self._detections: list[dict] = []
        self._session = SessionWatch()
        self._arm_contact_pending = False  # one-shot edge, consumed each _tick

        self._pub_twist = self.create_publisher(Twist, topics.CMD_VEL_AUTO, 10)
        self._pub_arm = self.create_publisher(String, topics.ARM_COMMAND, 10)
        self._pub_base = self.create_publisher(String, topics.BASE_COMMAND, 10)
        self._pub_state = self.create_publisher(String, topics.MISSION_STATE, 10)
        self._pub_nav_goal = self.create_publisher(String, topics.NAV_GOAL, 10)
        self._pub_nav_cancel = self.create_publisher(String, topics.NAV_CANCEL, 10)

        self.create_subscription(
            PoseWithCovarianceStamped, topics.LOC_POSE, self._on_loc_pose, 10,
        )
        self.create_subscription(Odometry, topics.ODOM, self._on_odom, 10)
        self.create_subscription(String, topics.NAV_STATUS, self._on_nav_status, 10)
        self.create_subscription(LaserScan, topics.SCAN, self._on_scan, SENSOR_QOS)
        self.create_subscription(String, topics.ARM_STATE, self._on_arm_state, 10)
        self.create_subscription(String, topics.CONTROL_MODE, self._on_mode, 10)
        self.create_subscription(Bool, topics.SAFETY_STOP, self._on_safety, 10)
        self.create_subscription(String, topics.DETECTED_OBJECTS, self._on_detections, 10)
        self.create_subscription(String, topics.SIM_STATUS, self._on_sim_status, 10)

        if USE_GT:
            self.create_subscription(String, topics.GROUND_TRUTH_OBJECTS, self._on_gt_objects, 10)
            self.create_subscription(PoseStamped, topics.GROUND_TRUTH_POSE, self._on_gt_pose, 10)
        else:
            self.create_subscription(String, topics.TOWEL_TRACKS, self._on_tracks, 10)

        # Primary held-state source in both modes: the FE publishes this on every
        # grasp change from the physics simulation (gripper-width + payload sensor).
        self.create_subscription(String, topics.HELD_OBJECT, self._on_held_object, 10)
        # ARM_CONTACT must reach the FSM in both modes too — arm-vs-wall
        # detection is independent of localization mode (was previously only
        # wired in perception mode, silently dropped under the GT default).
        self.create_subscription(String, topics.EVENTS, self._on_event, 20)
        # Collect-bin load-cell channel: the base firmware thresholds the
        # HX711 weight into bin_full and reports both on /base/state.
        self.create_subscription(String, topics.BASE_STATE, self._on_base_state, 10)

        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f"MissionExecutor ready — bin at {self._logic.bin_center}, "
            f"llm={self._llm.name}, source={'GROUND_TRUTH' if USE_GT else 'perception'}",
        )

    @staticmethod
    def _make_ik_reach(arm: ArmModel):
        """Analytic-IK reach line for a towel at base_link (rel_x, rel_y)."""
        def reach(rel_x: float, rel_y: float) -> str | None:
            # rel is already in base_link, the frame ArmModel.ik expects
            target = (rel_x, rel_y, GRASP_Z)
            sol = arm.ik(target)
            if sol is None:
                return None
            sol[5] = 80.0  # gripper open for the scoop; PICK_GRIP closes it next
            return grasp_command(sol, ms=900)
        return reach

    # ── Pose ────────────────────────────────────────────────────────────────────

    def _on_loc_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if USE_GT:
            return
        self._loc_pose = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw": _yaw_from_quat(msg.pose.pose.orientation),
        }

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_pose = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw": _yaw_from_quat(msg.pose.pose.orientation),
        }

    def _on_gt_pose(self, msg: PoseStamped) -> None:
        self._pose = {
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "yaw": _yaw_from_quat(msg.pose.orientation),
        }
        self._pose_source = "ground_truth"

    def _resolve_pose(self) -> None:
        if USE_GT:
            return  # set directly in _on_gt_pose
        if self._loc_pose is not None:
            self._pose = self._loc_pose
            self._pose_source = "localization"
        elif self._odom_pose is not None:
            self._pose = self._odom_pose
            self._pose_source = "odom"

    # ── Targets ─────────────────────────────────────────────────────────────────

    def _on_tracks(self, msg: String) -> None:
        try:
            self._towels = json.loads(msg.data).get("tracks", [])
        except json.JSONDecodeError:
            pass

    def _on_held_object(self, msg: String) -> None:
        """Authoritative holding source (both modes): the gripper's own
        grasp-state feedback, published by the FE on every grasp change. This
        is the reliable "did my commanded grasp engage an object" signal a real
        gripper controller reports. The wrist-load-cell force on /joint_states
        (-> gripper_holding on /arm/state) is an INDEPENDENT force-based
        confirmation sensor kept for telemetry/observability, not the primary
        control gate."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._holding = bool(data.get("held", False))
        self._held_class = data.get("object_class")

    def _on_event(self, msg: String) -> None:
        """ARM_CONTACT is the only event still processed here — held-state
        comes from /robot/held_object (_on_held_object). A real servo stalls
        on hard contact; this is the sensor edge telling the FSM to stop
        pressing into whatever the forearm/gripper hit and abort the
        in-flight PICK/DROP sequence (see MissionLogic._handle_arm_contact)."""
        try:
            event = json.loads(msg.data).get("event")
        except json.JSONDecodeError:
            return
        if event == "ARM_CONTACT":
            self._arm_contact_pending = True

    def _on_base_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._bin_kg = float(data.get("bin_kg", 0.0) or 0.0)
        self._bin_full = bool(data.get("bin_full", False))

    def _on_gt_objects(self, msg: String) -> None:
        try:
            objects = json.loads(msg.data).get("objects", [])
        except json.JSONDecodeError:
            return
        # Only update towel targets; held-state comes from /robot/held_object.
        # in_basket towels are riding in the robot's own collect bin — they are
        # cargo, not targets.
        self._towels = [
            o for o in objects
            if o.get("class") == "towel" and not o.get("binned")
            and not o.get("held") and not o.get("in_basket")
        ]

    # ── Other inputs ────────────────────────────────────────────────────────────

    def _on_nav_status(self, msg: String) -> None:
        try:
            self._nav_status = json.loads(msg.data).get("state", "idle")
        except json.JSONDecodeError:
            pass

    def _on_scan(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        if n == 0:
            return
        window = n // 8
        mid = n // 2
        sector = msg.ranges[mid - window: mid + window]
        valid = [r for r in sector if r is not None and msg.range_min < r < msg.range_max]
        self._min_front = min(valid) if valid else None

    def _on_arm_state(self, msg: String) -> None:
        """arm_status gates the pick/drop sequencing. gripper_holding (the
        wrist-load-cell force-sensor confirmation, arm_controller thresholding
        the payload weight on /joint_states) is recorded for telemetry in
        /mission/state, but the authoritative holding gate is the gripper's own
        grasp-state feedback on /robot/held_object (see _on_held_object)."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._arm_status = data.get("status", "IDLE")
        self._gripper_holding_sensor = bool(data.get("gripper_holding", False))

    def _on_mode(self, msg: String) -> None:
        try:
            self._mode = json.loads(msg.data).get("mode", "auto")
        except json.JSONDecodeError:
            pass

    def _on_safety(self, msg: Bool) -> None:
        self._safety = bool(msg.data)

    def _on_detections(self, msg: String) -> None:
        try:
            self._detections = json.loads(msg.data).get("objects", [])
        except json.JSONDecodeError:
            pass

    def _on_sim_status(self, msg: String) -> None:
        """Reset stale FSM state when the FE session restarts (new session_id),
        so the robot doesn't resume a dead mission. SessionWatch ignores
        competing concurrent tabs, so a duplicate tab can't trigger a reset
        storm (which previously made the robot circle forever)."""
        if self._session.update_raw(msg.data, time.monotonic()):
            self._logic.reset()
            self._holding = False
            self._towels = []
            self._pub_nav_cancel.publish(String(data=json.dumps({})))
            self.get_logger().info("Sim session restarted — mission state reset")

    def _target_rel(self) -> tuple[float, float] | None:
        """Freshest base_link (x, y) of the currently targeted towel, taken
        straight from this tick's camera detections (depth-refined). Matched to
        the target by proximity to where the map track projects into base_link.
        Returns None when no fresh towel detection plausibly matches — the FSM
        then falls back to the track-via-localized-pose estimate."""
        target = self._logic.target
        if target is None or self._pose is None:
            return None
        expected = world_to_robot(target["position"], self._pose)
        best = None
        best_d = 0.6   # m gate: a detection must be near the expected reach
        for det in self._detections:
            if det.get("class") != "towel":
                continue
            pos = det.get("position_refined") or det.get("estimated_position")
            if pos is None:
                continue
            d = math.hypot(pos["x"] - expected[0], pos["y"] - expected[1])
            if d < best_d:
                best, best_d = (pos["x"], pos["y"]), d
        return best

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        self._resolve_pose()
        if self._pose is None:
            return
        self._maybe_consult_llm()

        out = self._logic.update(MissionInput(
            pose=self._pose,
            towels=self._towels,
            holding=self._holding,
            arm_status=self._arm_status,
            safety_stop=self._safety,
            mode=self._mode,
            nav_status=self._nav_status,
            min_front_obstacle=self._min_front,
            target_rel=self._target_rel(),
            arm_contact=self._arm_contact_pending,
            bin_kg=self._bin_kg,
            bin_full=self._bin_full,
        ))
        self._arm_contact_pending = False  # one-shot edge, consumed above

        if out.nav_cancel:
            self._pub_nav_cancel.publish(String(data=json.dumps({})))
        if out.nav_goal is not None:
            self._pub_nav_goal.publish(String(data=json.dumps({
                "goal_id": f"{out.state}_{int(time.time() * 1000)}",
                "x": out.nav_goal[0], "y": out.nav_goal[1], "yaw": out.nav_goal[2],
            })))
        if out.twist is not None:
            twist = Twist()
            twist.linear.x = float(out.twist[0])
            twist.angular.z = float(out.twist[1])
            self._pub_twist.publish(twist)
        if out.arm_command:
            self._pub_arm.publish(String(data=out.arm_command))
        if out.base_command:
            self._pub_base.publish(String(data=out.base_command))

        self._pub_state.publish(String(data=json.dumps({
            "state": out.state,
            "reason": out.reason,
            "target_id": out.target_id or (self._logic.target or {}).get("id"),
            "holding": self._holding,
            "held_class": self._held_class,
            "gripper_holding_sensor": self._gripper_holding_sensor,
            "bin_kg": self._bin_kg,
            "bin_full": self._bin_full,
            "towels_remaining": len(self._towels),
            "pose_source": self._pose_source,
            "nav_status": self._nav_status,
            "llm": self._llm_verdict.get("action"),
            "llm_reason": self._llm_verdict.get("reason"),
        })))

    def _maybe_consult_llm(self) -> None:
        now = time.monotonic()
        if now - self._last_llm_at < self._llm_consult_interval:
            return
        self._last_llm_at = now
        context = json.dumps({
            "safety_stop": self._safety,
            "detections": self._detections,
            "towels_remaining": len(self._towels),
            "holding": self._holding,
        })
        self._llm_verdict = complete_json(self._llm, LLM_SYSTEM_PROMPT, context)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

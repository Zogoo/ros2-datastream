"""Navigation server — the in-house NavigateToPose equivalent.

A* on the layout-rasterized inflated grid + regulated pure pursuit, driving
the robot via /cmd_vel/auto (the arbitrator still owns /cmd_vel; manual wins).
The goal interface is seam-compatible with Nav2's NavigateToPose action so a
future real-Nav2 swap only replaces the client class:

  /nav/goal    std_msgs/String JSON {"goal_id", "x", "y", "yaw"?}
  /nav/cancel  std_msgs/String JSON {"goal_id"?}  (empty = cancel current)
  /nav/status  std_msgs/String JSON {"goal_id", "state", "distance_remaining"}
               state: idle|active|succeeded|failed|cancelled

Also publishes /map (latched) and /nav/path for Foxglove inspection.
Safety: stops publishing and cancels on /safety/stop or manual control mode.
"""
from __future__ import annotations

import json
import math
import os

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from onsen_robot_state import topics

from .astar import plan_path
from .grid import DynamicLayer, NavGrid
from .pure_pursuit import PurePursuit, TrackerParams, wrap_angle

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)
LATCHED_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

LAYOUT_PATH = os.environ.get("ONSEN_LAYOUT_PATH", "/ros2_ws/shared/onsen_layout.json")
# Planner footprint: the swept radius of a rotate-in-place. The driven wheels
# are on the centre axle (x 0), so base_link IS the pivot and the swept radius
# equals the circumscribed radius — the far bumper corner at
# sqrt(0.412^2 + 0.312^2) ~ 0.52 m — plus a small margin. Inflation must cover
# it or turning near a wall grinds the ring corner into it (bumper feedback
# loop: hit -> escape restarted by the next hit -> stuck). The onsen layout is
# scaled up (shared/onsen_layout.json) so this footprint still has clear paths.
ROBOT_RADIUS = 0.55
YAW_TOLERANCE = 0.10
BLOCKED_FAIL_S = 8.0
BLOCKED_RECOVER_S = 3.0      # blocked this long -> try one recovery before giving up
RECOVER_BACKUP_S = 1.0
RECOVER_BACKUP_V = -0.08
POSE_STALE_S = 1.5
TICK_HZ = 10.0
# Bumper (Roomba-style): the contact skirt reports chassis hits on
# /robot/contacts; while a goal is active any such bump triggers an immediate
# escape (back away from the contacted side, replan from the new pose).
BUMP_MIN_IMPULSE = 0.3       # Ns; ignore feather brushes
BUMP_BACKUP_S = 1.2
BUMP_BACKUP_V = 0.10         # m/s magnitude; sign from the contacted side
MAX_BUMPS_PER_GOAL = 2       # third bump on one goal -> fail (mission re-plans)


class NavServerNode(Node):
    def __init__(self) -> None:
        super().__init__("nav_server_node")
        self._grid = NavGrid.from_file(LAYOUT_PATH)
        self._inflated = self._grid.inflated(ROBOT_RADIUS)
        # Obstacle memory for things the static map doesn't know — marked on
        # tracker blocks and bumper hits so the NEXT plan routes around them
        # (without it, recovery replanned through the same stool forever).
        self._dyn = DynamicLayer(self._grid, ROBOT_RADIUS + 0.05)
        self._tracker = PurePursuit(TrackerParams())

        self._goal: dict | None = None
        self._path: list[tuple[float, float]] | None = None
        self._state = "idle"
        self._distance = 0.0
        self._blocked_since: float | None = None
        # One recovery attempt (back up + replan) before a block is a hard
        # failure — matches Nav2's simplest recovery behavior tree, cheap
        # enough to add without a new dependency.
        self._recovering_until: float | None = None
        self._recovered_once = False
        self._recover_v = RECOVER_BACKUP_V  # escape velocity of the active recovery
        self._recover_wz = 0.0               # turn-away component for side bumps
        self._bumps = 0                      # bumper hits on the current goal
        self._last_cmd_vx = 0.0              # sign picks the bumper-escape direction

        self._pose: tuple[float, float, float] | None = None
        self._pose_at = 0.0
        self._min_front: float | None = None
        self._min_front_pos: tuple[float, float] | None = None
        self._min_low: float | None = None
        self._min_low_pos: tuple[float, float] | None = None
        self._mode = "auto"
        self._safety = False

        self._pub_twist = self.create_publisher(Twist, topics.CMD_VEL_AUTO, 10)
        self._pub_status = self.create_publisher(String, topics.NAV_STATUS, 10)
        self._pub_path = self.create_publisher(Path, topics.NAV_PATH, LATCHED_QOS)
        self._pub_map = self.create_publisher(OccupancyGrid, topics.MAP, LATCHED_QOS)

        self.create_subscription(String, topics.NAV_GOAL, self._on_goal, 10)
        self.create_subscription(String, topics.NAV_CANCEL, self._on_cancel, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, topics.LOC_POSE, self._on_pose, 10,
        )
        self.create_subscription(LaserScan, topics.SCAN, self._on_scan, SENSOR_QOS)
        self.create_subscription(LaserScan, topics.SCAN_LOW, self._on_scan_low, SENSOR_QOS)
        self.create_subscription(String, topics.CONTROL_MODE, self._on_mode, 10)
        self.create_subscription(Bool, topics.SAFETY_STOP, self._on_safety, 10)
        self.create_subscription(String, topics.CONTACTS, self._on_contact, 20)

        self._publish_map()
        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.create_timer(0.2, self._publish_status)
        self.get_logger().info(
            f"NavServer ready — {self._grid.width}x{self._grid.height} map, "
            f"A* + regulated pure pursuit",
        )

    # ── inputs ────────────────────────────────────────────────────────────────

    def _on_goal(self, msg: String) -> None:
        try:
            goal = json.loads(msg.data)
            goal_xy = (float(goal["x"]), float(goal["y"]))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            self.get_logger().warning(f"Bad nav goal: {msg.data[:120]}")
            return
        self._goal = {
            "goal_id": str(goal.get("goal_id", "goal")),
            "x": goal_xy[0], "y": goal_xy[1],
            "yaw": float(goal["yaw"]) if goal.get("yaw") is not None else None,
        }
        self._path = None
        self._state = "active"
        self._blocked_since = None
        self._recovering_until = None
        self._recovered_once = False
        self._bumps = 0

    def _on_cancel(self, _msg: String) -> None:
        if self._state == "active":
            self._finish("cancelled")

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self._pose_at = self._now()

    def _on_scan(self, msg: LaserScan) -> None:
        """Front-sector minimum range, but only over UNMAPPED hits. A* already
        keeps ROBOT_RADIUS clearance from every wall/prop it plans past, so in
        a corridor no wider than ~2x that clearance the lidar legitimately
        reads a mapped wall inside obstacle_slow_range on every tick — that
        used to crawl the tracker permanently, not just for genuine hazards
        (an unmapped/dynamic obstacle the planner never saw)."""
        n = len(msg.ranges)
        if n == 0 or self._pose is None:
            self._min_front = None
            return
        window = n // 8
        mid = n // 2
        x, y, yaw = self._pose
        best = None
        best_pos = None
        for i in range(mid - window, mid + window):
            r = msg.ranges[i]
            if r is None or not (msg.range_min < r < msg.range_max):
                continue
            bearing = yaw + msg.angle_min + i * msg.angle_increment
            hx = x + r * math.cos(bearing)
            hy = y + r * math.sin(bearing)
            if self._grid.is_lethal(hx, hy):
                continue
            if best is None or r < best:
                best = r
                best_pos = (hx, hy)
        self._min_front = best
        self._min_front_pos = best_pos

    def _on_scan_low(self, msg: LaserScan) -> None:
        # forward cone only (±20°): a towel beside the path must not stall us
        cone = math.radians(20.0)
        best = None
        best_rel = None
        for i, r in enumerate(msg.ranges):
            if r is None or not (msg.range_min < r < msg.range_max):
                continue
            a = msg.angle_min + i * msg.angle_increment
            if abs(a) >= cone:
                continue
            if best is None or r < best:
                best = r
                best_rel = (r, a)
        self._min_low = best
        if best_rel is not None and self._pose is not None:
            x, y, yaw = self._pose
            self._min_low_pos = (
                x + best_rel[0] * math.cos(yaw + best_rel[1]),
                y + best_rel[0] * math.sin(yaw + best_rel[1]),
            )
        else:
            self._min_low_pos = None

    def _on_mode(self, msg: String) -> None:
        try:
            self._mode = json.loads(msg.data).get("mode", "auto")
        except json.JSONDecodeError:
            return
        if self._mode != "auto" and self._state == "active":
            self._finish("cancelled")

    def _on_safety(self, msg: Bool) -> None:
        self._safety = bool(msg.data)
        if self._safety and self._state == "active":
            self._finish("failed")

    def _on_contact(self, msg: String) -> None:
        """Bumper escape (the Roomba behavior, with a map): a physical chassis
        hit while navigating means the world disagrees with the plan — back
        away from the hit (opposite the last commanded direction), then replan
        A* from the escaped pose. Repeated bumps on one goal fail it so the
        mission's retry/park logic takes over. Without this, the tracker kept
        pushing against the obstacle until the blocked timer expired, or wedged
        entirely — 'robot hits object and loses navigation'."""
        if self._state != "active":
            return
        try:
            contact = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        part = str(contact.get("part", ""))
        if not (part.startswith("bumper") or part.startswith("chassis")):
            return
        if float(contact.get("impulse", 0.0) or 0.0) < BUMP_MIN_IMPULSE:
            return
        # One escape at a time: while a recovery is running, further scrapes
        # are the same event (the ring is still leaving the surface).
        # Restarting the escape on every 150 ms contact report froze the
        # robot against the wall it was escaping from.
        if self._recovering_until is not None and self._now() < self._recovering_until:
            return
        self._bumps += 1
        if self._bumps > MAX_BUMPS_PER_GOAL:
            self.get_logger().warning(
                f"Bumper: {self._bumps} hits on goal {(self._goal or {}).get('goal_id')} — failing",
            )
            self._finish("failed")
            return
        # The 360° bumper ring reports the obstacle's body-frame bearing —
        # remember it in the dynamic layer so the replan avoids it, and escape
        # directly away from it (rear sectors -> pull forward).
        bearing = contact.get("bearing_deg")
        now = self._now()
        if bearing is not None and self._pose is not None:
            world_b = self._pose[2] + math.radians(float(bearing))
            self._dyn.mark(
                self._pose[0] + (ROBOT_RADIUS + 0.10) * math.cos(world_b),
                self._pose[1] + (ROBOT_RADIUS + 0.10) * math.sin(world_b),
                now,
            )
            obstacle_ahead = abs(float(bearing)) < 90.0
            # Side hits: also twist AWAY from the obstacle while escaping, or
            # a straight escape keeps grazing the wall with the ring corner.
            side = math.sin(math.radians(float(bearing)))
            self._recover_wz = -0.3 * (1 if side > 0.3 else -1 if side < -0.3 else 0)
        else:
            obstacle_ahead = self._last_cmd_vx >= 0
            self._recover_wz = 0.0
        self._recover_v = -BUMP_BACKUP_V if obstacle_ahead else BUMP_BACKUP_V
        self._recovering_until = now + BUMP_BACKUP_S
        self._path = None            # replan from the escaped pose
        self._blocked_since = None
        self.get_logger().info(
            f"Bumper {part} hit ({contact.get('object_id')}) — "
            f"escaping {self._recover_v:+.2f} m/s, marked + replanning",
        )

    # ── control loop ──────────────────────────────────────────────────────────

    def _tick(self) -> None:
        if self._state != "active" or self._goal is None:
            return
        if self._pose is None or self._now() - self._pose_at > POSE_STALE_S:
            return

        now = self._now()
        if self._recovering_until is not None:
            if now < self._recovering_until:
                self._publish_twist(self._recover_v, self._recover_wz)
                return
            # Escape finished — force a fresh plan from the escaped pose.
            self._recovering_until = None
            self._recover_v = RECOVER_BACKUP_V
            self._recover_wz = 0.0
            self._path = None
            self._blocked_since = None

        if self._path is None and not self._plan():
            return
        assert self._path is not None

        obstacle = min(
            (v for v in (self._min_front, self._min_low) if v is not None),
            default=None,
        )
        out = self._tracker.step(self._pose, self._path, obstacle)
        self._distance = out.distance_remaining

        if out.blocked:
            # Remember WHAT blocked us: stamp the closest unmapped return into
            # the dynamic layer so the recovery replan routes AROUND it.
            for pos in (self._min_front_pos, self._min_low_pos):
                if pos is not None:
                    self._dyn.mark(pos[0], pos[1], now)
            if self._blocked_since is None:
                self._blocked_since = now
            else:
                elapsed = now - self._blocked_since
                # One recovery (back up, replan) before a hard failure — the
                # simplest Nav2-style recovery behavior. Without it a robot
                # wedged against clutter (or a transient obstacle that will
                # clear) gives up on the whole target instead of trying an
                # escape maneuver first.
                if not self._recovered_once and elapsed > BLOCKED_RECOVER_S:
                    self._recovered_once = True
                    self._recover_v = RECOVER_BACKUP_V
                    self._recovering_until = now + RECOVER_BACKUP_S
                    self._publish_twist(self._recover_v, 0.0)
                    return
                if elapsed > BLOCKED_FAIL_S:
                    self._finish("failed")
                    return
        else:
            self._blocked_since = None
            self._recovered_once = False

        if out.done:
            if not self._align_final_yaw():
                return
            self._finish("succeeded")
            return
        self._publish_twist(out.vx, out.wz)

    def _plan(self) -> bool:
        assert self._pose is not None and self._goal is not None
        grid = self._dyn.overlay(self._inflated, self._now())
        path = plan_path(
            grid, self._grid.origin, self._grid.res,
            (self._pose[0], self._pose[1]), (self._goal["x"], self._goal["y"]),
        )
        if path is None:
            self._finish("failed")
            return False
        self._path = path
        self._publish_path(path)
        return True

    def _align_final_yaw(self) -> bool:
        """Returns True when aligned (or no yaw requested)."""
        assert self._goal is not None and self._pose is not None
        target_yaw = self._goal.get("yaw")
        if target_yaw is None:
            return True
        err = wrap_angle(target_yaw - self._pose[2])
        if abs(err) < YAW_TOLERANCE:
            return True
        wz = max(-0.8, min(0.8, 1.6 * err))
        self._publish_twist(0.0, wz)
        return False

    def _finish(self, state: str) -> None:
        self._state = state
        self._path = None
        self._blocked_since = None
        self._recovering_until = None
        self._recovered_once = False
        self._recover_v = RECOVER_BACKUP_V
        self._bumps = 0
        self._publish_twist(0.0, 0.0)
        self._publish_status()

    # ── outputs ───────────────────────────────────────────────────────────────

    def _publish_twist(self, vx: float, wz: float) -> None:
        if self._safety or self._mode != "auto":
            return
        self._last_cmd_vx = float(vx)
        twist = Twist()
        twist.linear.x = float(vx)
        twist.angular.z = float(wz)
        self._pub_twist.publish(twist)

    def _publish_status(self) -> None:
        self._pub_status.publish(String(data=json.dumps({
            "goal_id": (self._goal or {}).get("goal_id"),
            "state": self._state,
            "distance_remaining": round(self._distance, 3),
        })))

    def _publish_path(self, path: list[tuple[float, float]]) -> None:
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            msg.poses.append(pose)
        self._pub_path.publish(msg)

    def _publish_map(self) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.info.resolution = self._grid.res
        msg.info.width = self._grid.width
        msg.info.height = self._grid.height
        msg.info.origin = Pose()
        msg.info.origin.position.x = self._grid.origin[0]
        msg.info.origin.position.y = self._grid.origin[1]
        msg.info.origin.orientation.w = 1.0
        msg.data = self._grid.occupancy_msg_data()
        self._pub_map.publish(msg)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

"""Mission state machine — pure logic, no ROS imports.

Closes the autonomy loop with batch collection: SEARCH -> APPROACH ->
ALIGN_PICK -> PICK -> STOW (drop into the onboard collect bin) -> SEARCH ...
until the bin's load cell reports full (or no towels remain), then one
delivery trip: TO_BIN -> ALIGN_BIN -> UNLOAD (the arm lifts towels back out of
the bin and releases them over the floor-bin rim, one per cycle, until the
load cell reads empty) -> SEARCH. Locomotion is delegated to the navigation
server (A* + regulated pure pursuit) through a goal/status protocol mirroring
Nav2 NavigateToPose: APPROACH/TO_BIN emit a single `nav_goal` and then wait on
`nav_status`, commanding no twist while navigation owns the base. Only the
fine ALIGN micro-motions are issued directly as twists. Pick/stow/dump
geometry derives from the arm FK constants in shared/robot_spec.json.
"""
from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

# (rel_x, rel_y in base_link) -> firmware "J ..." reach line, or None if the
# measured towel is outside the arm's reachable annulus.
IkReach = Callable[[float, float], "str | None"]

# (standoff_xy, towel_xy) -> True when the arm's reach line from a robot at
# standoff_xy toward (and slightly past) the towel stays clear of static
# geometry — i.e. the scoop will not press the fingertip into a wall. Injected
# by the executor from the nav grid (same seam pattern as IkReach).
ReachClear = Callable[[tuple[float, float], tuple[float, float]], bool]

# Alternate approach bearings (rad, relative to the anchor direction) tried
# when the direct standoff would sweep the arm into a wall. Ordered by how
# little they deviate from the natural approach; a wall-adjacent towel is
# typically picked by approaching parallel to the wall (±90°).
APPROACH_BEARINGS = (0.0, 0.5, -0.5, 1.05, -1.05, 1.57, -1.57, 2.1, -2.1, 2.6, -2.6, 3.14)

SCOOP_FORWARD = 0.757     # m, fingertip ahead of base center at PICK_SCOOP
# (arm base moved to x 0.35 so the pick crescent clears the enlarged 0.80 m
# body's front bumper at x 0.412 — the standoff stands the robot far enough
# back that the bumper does not reach the towel during approach).
# Delivery geometry: towels are unloaded FROM the onboard bin BY THE ARM — the
# DROP_BIN pose releases at (0.37, 0.66) in base_link, z 0.66, the only
# mechanism on this robot that clears the 0.55 m floor-bin rim (a servo tilt
# of the deck bin cannot: its hinge is capped at the 0.58 lidar guard band, so
# contents would exit below the rim). ALIGN_BIN points this offset at the bin.
DELIVER_OFFSET = (0.37, 0.66)  # m, DROP_BIN release point in base_link
PICK_TOL_X = 0.08
PICK_TOL_Y = 0.08
YAW_TOL = 0.08
ALIGN_ENTER_RAD = 0.12   # start rotating toward the towel above this bearing error
ALIGN_EXIT_RAD = 0.05    # keep rotating until under this (hysteresis kills hunting)
RETARGET_DIST = 0.4       # m a towel must drift before the nav goal is re-issued
MAX_APPROACH_RETRIES = 3
NAV_LIMBO_TICKS = 4       # sustained failed/cancelled without ever seeing active
MAX_PICK_ATTEMPTS = 3     # failed grasps on one towel before it is parked
PARK_TICKS = 150         # ~30 s at 5 Hz before a parked towel is retried
GLOBAL_FAIL_LIMIT = 6    # consecutive failed picks (any/churning target) -> cooldown
COOLDOWN_TICKS = 100     # ~20 s of patrol before the robot attempts picks again

# Collect-bin thresholds (mirror the base firmware's load-cell channel).
BIN_MIN_DELIVER_KG = 0.1  # something is in the bin -> worth a delivery trip
STOW_VERIFY_KG = 0.15     # load-cell delta proving the stowed towel landed IN the bin
MAX_UNLOAD_CYCLES = 5     # arm unload attempts per delivery trip
MAX_STOW_RETRIES = 2      # arm-contact aborts during STOW before force-release

# Canned fallback (no IK): fixed scoop + grip pose, fixed 0.757 m ahead.
PICK_SEQUENCE = [
    "A PRE_PICK", "A PICK_LOWER", "A PICK_SCOOP",
    "A PICK_GRIP", "A PICK_LIFT", "A PICK_RETRACT",
]
# IK path: lift clear, reach the *measured* towel (J line), then close the
# gripper IN PLACE (CLOSE_GRIPPER touches only J5 — a canned PICK_GRIP would
# yank the arm back to a centred pose and miss an off-axis towel), then lift.
PICK_PREAMBLE = ["A PRE_PICK"]
PICK_GRASP = ["A CLOSE_GRIPPER", "A PICK_LIFT", "A PICK_RETRACT"]
# Stow the grasped towel into the onboard collect bin (poses FK-validated
# against the bin volume in frontend/tests/kinematics.test.js).
STOW_SEQUENCE = ["A DROP_BASKET", "A DROP_RELEASE", "A HOME"]
# One arm-unload cycle: reach into the own bin, grasp a stowed towel, swing it
# out over the floor bin, release above its rim. Repeated until the load cell
# reads empty (or the cycle guard trips).
UNLOAD_CYCLE = ["A BIN_PICK", "A CLOSE_GRIPPER", "A DROP_BIN", "A DROP_BIN_RELEASE"]


@dataclass
class MissionInput:
    pose: dict[str, float]                 # {x, y, yaw} (map frame, localized)
    towels: list[dict[str, Any]]           # world-frame towel tracks
    holding: bool
    arm_status: str                        # IDLE | MOVING | STOPPED
    safety_stop: bool
    mode: str                              # auto | manual
    nav_status: str = "idle"               # idle|active|succeeded|failed|cancelled
    min_front_obstacle: float | None = None
    # One-shot edge: the arm reported ARM_CONTACT this tick (forearm/gripper
    # touched static geometry mid-sequence). A real servo would stall here —
    # abort the in-flight PICK/DROP sequence instead of continuing to press
    # into the obstacle. Cleared by the caller after each update().
    arm_contact: bool = False
    # Freshest depth-refined base_link (x, y) of the targeted towel, measured
    # directly from the camera this tick. Localization-independent, so the final
    # grasp uses it in preference to the map-track-via-localized-pose estimate
    # (the PBVS-lite "measure in the sensor frame for the final approach").
    target_rel: tuple[float, float] | None = None
    # Collect-bin load-cell channel from the base firmware (/base/state):
    # measured payload weight and the debounced full latch.
    bin_kg: float = 0.0
    bin_full: bool = False


@dataclass
class MissionOutput:
    state: str
    twist: tuple[float, float] | None = None   # (vx, wz); None = no command
    arm_command: str | None = None
    base_command: str | None = None            # base protocol line (BIN DUMP/HOME)
    nav_goal: tuple[float, float, float] | None = None   # (x, y, yaw), emit once
    nav_cancel: bool = False
    target_id: str | None = None
    reason: str = ""


@dataclass
class _SeqState:
    commands: list[str] = field(default_factory=list)
    index: int = 0
    sent: bool = False
    settle: int = 0


class MissionLogic:
    def __init__(
        self,
        bin_center: tuple[float, float],
        ik_reach: IkReach | None = None,
        reach_clear: ReachClear | None = None,
    ) -> None:
        self.bin_center = bin_center
        # ik_reach(rel_x, rel_y) -> firmware "J ..." line reaching that base_link
        # point at GRASP_Z, or None if unreachable. When unset, the canned
        # PICK_SCOOP pose is used (fixed 0.757 m ahead).
        self._ik_reach = ik_reach
        # reach_clear(standoff_xy, towel_xy) -> arm sweep clear of walls from
        # that standoff. When unset, the anchor-direction standoff is used
        # unchecked (the reactive ARM_CONTACT abort still guards the sequence).
        self._reach_clear = reach_clear
        self.state = "IDLE"
        self.target: dict[str, Any] | None = None
        self._seq = _SeqState()
        self._nav_goal: tuple[float, float, float] | None = None  # last goal emitted
        # Robot position when APPROACH/TO_BIN was entered — freezes the
        # standoff approach direction for that leg (see _state_approach).
        self._nav_anchor: tuple[float, float] | None = None
        self._retries = 0
        self._was_active = False  # saw nav_status==active since the goal was emitted
        self._nav_limbo = 0       # ticks of terminal status without ever seeing active
        self._aligning = False    # ALIGN_PICK rotation hysteresis state
        # Per-target pick-attempt accounting: a towel that can't be grasped
        # (mislocated, unreachable, occluded) is parked so the arm stops looping
        # on it. Parked targets are retried after PARK_TICKS so a transient
        # localization error doesn't abandon a real towel forever.
        self._attempts: dict[str, int] = {}
        self._parked: dict[str, int] = {}
        self._consecutive_fails = 0   # failed picks in a row, across churning ids
        self._cooldown = 0            # ticks remaining of the global pick cooldown
        self._stow_retries = 0        # arm-contact aborts during STOW
        self._stow_entry_kg = 0.0     # load-cell reading when STOW began (verify delta)
        self._unload_cycles = 0       # arm-unload cycles this delivery trip
        self._unload_last_kg: float | None = None  # progress check between cycles
        # Weight known to be stuck in the bin after a stalled unload (outside
        # the fixed BIN_PICK reach). SEARCH only starts another delivery trip
        # once the bin holds MORE than this — otherwise a permanently stuck
        # leftover loops the robot between the floor bin and UNLOAD forever.
        self._leftover_kg = 0.0

    def update(self, inp: MissionInput) -> MissionOutput:
        if inp.safety_stop:
            return self._abort("safety stop latched")
        if inp.mode != "auto":
            return self._abort("manual mode")
        if inp.arm_contact and self.state in ("PICK", "STOW", "UNLOAD"):
            return self._handle_arm_contact(inp)

        handler = getattr(self, f"_state_{self.state.lower()}", None)
        if handler is None:
            return self._abort(f"unknown state {self.state}")
        return handler(inp)

    # ── States ────────────────────────────────────────────────────────────────

    def _state_idle(self, inp: MissionInput) -> MissionOutput:
        self.state = "SEARCH"
        return MissionOutput(state="IDLE", reason="resuming search")

    def _state_search(self, inp: MissionInput) -> MissionOutput:
        if inp.holding:
            # A towel in the gripper (e.g. after an aborted sequence) is stowed
            # into the collect bin first — never carried around the map.
            self._enter("STOW")
            return MissionOutput(state="SEARCH", reason="holding — stow into bin")
        if inp.bin_full:
            self._enter("TO_BIN")
            return MissionOutput(state="SEARCH", reason="bin full — delivering batch")
        self._age_parked()
        if self._cooldown > 0:
            self._cooldown -= 1
            return MissionOutput(
                state="SEARCH", twist=(0.0, 0.45),
                reason=f"pick cooldown ({self._cooldown} ticks) — phantom/unreachable churn",
            )
        candidates = [t for t in inp.towels if t["id"] not in self._parked]
        if candidates:
            self.target = nearest(candidates, inp.pose)
            self._retries = 0
            self._enter("APPROACH")
            return MissionOutput(
                state="SEARCH", target_id=self.target["id"],
                reason=f"towel {self.target['id']} selected",
            )
        if inp.bin_kg >= BIN_MIN_DELIVER_KG and inp.bin_kg > self._leftover_kg + 0.1:
            # Nothing left to pick but the bin has a partial batch — deliver it.
            # (The leftover guard skips re-delivering weight a stalled unload
            # already failed to lift; a fresh stow raises the reading past it.)
            self._enter("TO_BIN")
            return MissionOutput(state="SEARCH", reason="no towels left — delivering batch")
        # nothing graspable right now (none seen, or all parked) — scan in place
        reason = "all towels parked — waiting" if inp.towels else "scanning for towels"
        return MissionOutput(state="SEARCH", twist=(0.0, 0.45), reason=reason)

    def _state_approach(self, inp: MissionInput) -> MissionOutput:
        towel = self._refresh_target(inp)
        if towel is None:
            self._enter("SEARCH")
            return MissionOutput(state="APPROACH", nav_cancel=True, reason="target lost")

        rel = world_to_robot(towel["position"], inp.pose)
        if abs(rel[0] - SCOOP_FORWARD) < PICK_TOL_X and abs(rel[1]) < PICK_TOL_Y:
            self._enter("ALIGN_PICK")
            return MissionOutput(state="APPROACH", nav_cancel=True, reason="at pick standoff")

        # The approach direction is frozen to the pose seen when APPROACH was
        # entered, not recomputed from the live (moving) pose every tick.
        # Recomputing it live made the standoff point orbit the towel as the
        # A* path curved the robot's heading, drifting the goal past
        # RETARGET_DIST and forcing a replan — pure churn, since the towel
        # itself hadn't moved.
        if self._nav_anchor is None:
            self._nav_anchor = (inp.pose["x"], inp.pose["y"])
        goal = self._standoff_goal(towel["position"], self._nav_anchor)
        if goal is None:
            # No standable, arm-clear approach exists (towel in an alcove
            # narrower than the footprint) — park it and move on.
            self._parked[towel["id"]] = PARK_TICKS
            self._enter("SEARCH")
            return MissionOutput(state="APPROACH", nav_cancel=True,
                                 reason=f"{towel['id']} unreachable footprint — parked")
        out = self._navigate(inp, goal, "APPROACH", target_id=towel["id"])
        if out is not None:
            return out
        # navigation finished but we're not in the pick window — refine on foot
        self._enter("ALIGN_PICK")
        return MissionOutput(state="APPROACH", reason="nav done — fine align")

    def _state_align_pick(self, inp: MissionInput) -> MissionOutput:
        towel = self._refresh_target(inp)
        if towel is None:
            self._enter("SEARCH")
            return MissionOutput(state="ALIGN_PICK", reason="target lost")
        # Prefer the direct camera measurement of the towel in base_link (immune
        # to localization drift) for the final alignment + reach; fall back to
        # the map track converted through the localized pose only when no fresh
        # detection of the target is available this tick.
        rel = inp.target_rel or world_to_robot(towel["position"], inp.pose)
        if abs(rel[0] - SCOOP_FORWARD) < PICK_TOL_X and abs(rel[1]) < PICK_TOL_Y:
            self.state = "PICK"
            self._seq = _SeqState(commands=self._build_pick_sequence(rel))
            return MissionOutput(state="ALIGN_PICK", twist=(0.0, 0.0), reason="in pick window")
        # bearing first, then close the longitudinal gap — gentle, low speed.
        # Rotation uses HYSTERESIS (enter at 0.12 rad, exit at 0.05): a single
        # threshold made the skid-steer hunt left-right around it (servo lag +
        # 5 Hz ticks overshoot the boundary), visible as close-range wiggling.
        bearing = wrap_angle(math.atan2(rel[1], rel[0]))
        if abs(bearing) > ALIGN_ENTER_RAD or (self._aligning and abs(bearing) > ALIGN_EXIT_RAD):
            self._aligning = True
            wz = max(-0.5, min(0.5, 1.5 * bearing))
            return MissionOutput(state="ALIGN_PICK", twist=(0.0, wz), reason="aligning bearing")
        self._aligning = False
        vx = max(-0.08, min(0.10, 0.4 * (rel[0] - SCOOP_FORWARD)))
        return MissionOutput(state="ALIGN_PICK", twist=(vx, 0.0),
                             reason=f"closing gap rel_x={rel[0]:.2f}")

    def _state_pick(self, inp: MissionInput) -> MissionOutput:
        out = self._run_sequence(inp, "PICK")
        if out is not None:
            return out
        tid = self.target["id"] if self.target else None
        if inp.holding:
            if tid:
                self._attempts.pop(tid, None)
            self._consecutive_fails = 0
            self._enter("STOW")
            return MissionOutput(state="PICK", reason="towel grasped — stowing in bin")
        return self._record_pick_failure("PICK", "grasp failed — retry")

    def _state_stow(self, inp: MissionInput) -> MissionOutput:
        """Drop the grasped towel into the onboard collect bin, VERIFY it
        actually landed there via the load cell, then keep collecting (or
        deliver, when the load cell says the bin is full)."""
        if not self._seq.commands:
            self._seq = _SeqState(commands=list(STOW_SEQUENCE))
            # Load-cell reading at stow entry: the release is only believed
            # when the measured weight rises by a towel's worth. Without this
            # the robot dropped towels PAST the bin and kept "collecting"
            # forever, unaware nothing was accumulating.
            self._stow_entry_kg = inp.bin_kg
        out = self._run_sequence(inp, "STOW")
        if out is not None:
            return out
        if inp.holding and self._stow_retries < MAX_STOW_RETRIES:
            # Release did not register — bounded OPEN retries, never an
            # unbounded loop on a stuck holding sensor.
            self._stow_retries += 1
            self._seq = _SeqState(commands=["A OPEN_GRIPPER", "A HOME"])
            return MissionOutput(state="STOW", reason="release retry")
        self._stow_retries = 0
        if inp.bin_kg < self._stow_entry_kg + STOW_VERIFY_KG:
            # The towel left the gripper but the bin never got heavier — it
            # fell outside. It is back on the floor where perception will
            # re-detect it; go re-acquire instead of pretending it was stowed.
            self._enter("SEARCH")
            return MissionOutput(state="STOW",
                                 reason="stow missed — load cell saw no gain, re-searching")
        if inp.bin_full:
            self._enter("TO_BIN")
            return MissionOutput(state="STOW", reason="stowed — bin full, delivering")
        self._enter("SEARCH")
        return MissionOutput(state="STOW", reason="stowed — continue collecting")

    def _state_to_bin(self, inp: MissionInput) -> MissionOutput:
        if inp.bin_kg < BIN_MIN_DELIVER_KG and not inp.holding:
            self._enter("SEARCH")
            return MissionOutput(state="TO_BIN", nav_cancel=True,
                                 reason="bin empty — nothing to deliver")
        # Same anchor-freeze as APPROACH: bin_center is fixed, but recomputing
        # the standoff direction from the live pose every tick made the goal
        # drift past RETARGET_DIST as the A* path curved, churning replans.
        if self._nav_anchor is None:
            self._nav_anchor = (inp.pose["x"], inp.pose["y"])
        standoff = math.hypot(*DELIVER_OFFSET)
        gx, gy = standoff_point(self.bin_center, self._nav_anchor, standoff)
        # Face so the DROP_BIN release point lands over the floor bin.
        beta = math.atan2(DELIVER_OFFSET[1], DELIVER_OFFSET[0])
        goal_yaw = math.atan2(self.bin_center[1] - gy, self.bin_center[0] - gx) - beta
        out = self._navigate(inp, (gx, gy, goal_yaw), "TO_BIN")
        if out is not None:
            return out
        self._enter("ALIGN_BIN")
        return MissionOutput(state="TO_BIN", reason="at delivery standoff")

    def _state_align_bin(self, inp: MissionInput) -> MissionOutput:
        v = (self.bin_center[0] - inp.pose["x"], self.bin_center[1] - inp.pose["y"])
        beta = math.atan2(DELIVER_OFFSET[1], DELIVER_OFFSET[0])
        yaw_target = math.atan2(v[1], v[0]) - beta
        err = wrap_angle(yaw_target - inp.pose["yaw"])
        if abs(err) < YAW_TOL:
            self.state = "UNLOAD"
            self._unload_cycles = 0
            self._unload_last_kg = None
            self._seq = _SeqState()
            return MissionOutput(state="ALIGN_BIN", twist=(0.0, 0.0), reason="aligned with bin")
        wz = max(-0.8, min(0.8, 1.5 * err))
        return MissionOutput(state="ALIGN_BIN", twist=(0.0, wz), reason=f"rotating ({err:.2f} rad)")

    def _state_unload(self, inp: MissionInput) -> MissionOutput:
        """Empty the onboard bin BY ARM, one towel per cycle: reach into the
        bin, grasp, swing out over the floor bin, release above its rim. The
        load cell is the loop condition — cycles repeat until the bin reads
        empty, with two guards: a hard cycle cap, and a no-progress check (a
        cycle that doesn't reduce the weight means the remaining towel is out
        of the fixed reach — carry it to the next trip instead of looping)."""
        if self._seq.commands:
            out = self._run_sequence(inp, "UNLOAD")
            if out is not None:
                return out
        if inp.bin_kg < BIN_MIN_DELIVER_KG:
            self._leftover_kg = 0.0
            self._enter("SEARCH")
            return MissionOutput(state="UNLOAD", arm_command="A HOME",
                                 reason="bin empty — batch delivered")
        no_progress = (
            self._unload_last_kg is not None
            and inp.bin_kg > self._unload_last_kg - STOW_VERIFY_KG
        )
        if self._unload_cycles >= MAX_UNLOAD_CYCLES or (self._unload_cycles > 0 and no_progress):
            self._leftover_kg = inp.bin_kg
            self._enter("SEARCH")
            return MissionOutput(state="UNLOAD", arm_command="A HOME",
                                 reason="unload stalled — leftover rides to next trip")
        self._unload_cycles += 1
        self._unload_last_kg = inp.bin_kg
        self._seq = _SeqState(commands=list(UNLOAD_CYCLE))
        return MissionOutput(state="UNLOAD", twist=(0.0, 0.0),
                             reason=f"unload cycle {self._unload_cycles}")

    # ── Navigation protocol ─────────────────────────────────────────────────────

    def _navigate(
        self, inp: MissionInput, goal: tuple[float, float, float],
        label: str, target_id: str | None = None,
    ) -> MissionOutput | None:
        """Drives to `goal` via the nav server. Returns a MissionOutput while
        navigation is in progress, or None once the goal succeeded (the caller
        advances the FSM). Re-issues the goal if the target drifts far."""
        if self._nav_goal is None or self._goal_moved(goal):
            self._nav_goal = goal
            self._was_active = False
            self._nav_limbo = 0
            return MissionOutput(
                state=label, nav_goal=goal, target_id=target_id,
                reason=f"nav goal ({goal[0]:.2f}, {goal[1]:.2f})",
            )
        if inp.nav_status == "active":
            self._was_active = True
        if inp.nav_status == "succeeded":
            self._nav_goal = None
            return None
        if inp.nav_status in ("failed", "cancelled"):
            # _was_active guards against consuming the PREVIOUS goal's stale
            # terminal status right after emitting a new goal. But a goal that
            # fails AT PLAN TIME (unreachable standoff) never shows "active" at
            # this tick rate — without the limbo counter the FSM waited on it
            # forever. A few ticks of sustained "failed" is unambiguous.
            self._nav_limbo += 1
            if self._was_active or self._nav_limbo >= NAV_LIMBO_TICKS:
                self._retries += 1
                self._nav_goal = None
                if self._retries > MAX_APPROACH_RETRIES:
                    self._enter("SEARCH")
                    return MissionOutput(state=label, reason="nav failed — giving up target")
                return MissionOutput(
                    state=label, reason=f"nav {inp.nav_status} — retry {self._retries}",
                )
        return MissionOutput(state=label, target_id=target_id, reason="navigating")

    def _goal_moved(self, goal: tuple[float, float, float]) -> bool:
        if self._nav_goal is None:
            return True
        return math.hypot(goal[0] - self._nav_goal[0], goal[1] - self._nav_goal[1]) > RETARGET_DIST

    def _standoff_goal(
        self, towel: dict[str, float], anchor: tuple[float, float],
    ) -> tuple[float, float, float] | None:
        """Pick standoff, wall-aware when a reach_clear probe is injected.

        The natural standoff (approach along the anchor->towel line) is used
        when the arm's sweep from there is clear. For a wall-adjacent towel it
        often is not — the scoop overshoots past the towel into the wall, and
        without this check every such towel burned MAX_PICK_ATTEMPTS aborted
        ARM_CONTACT sequences before being parked. Instead, scan alternate
        approach bearings around the towel (least deviation first: a wall
        towel is normally picked by approaching parallel to the wall) and take
        the first standoff whose reach line is clear.

        Returns None when NO bearing has a standable, arm-clear standoff — the
        towel is unpickable for this footprint (e.g. lying in an alcove
        narrower than the robot). Emitting a goal anyway just burned nav
        retries on an unplannable target forever; the caller parks it."""
        tx, ty = towel["x"], towel["y"]
        base_bearing = math.atan2(ty - anchor[1], tx - anchor[0])
        for delta in APPROACH_BEARINGS if self._reach_clear else (0.0,):
            bearing = base_bearing + delta
            gx = tx - math.cos(bearing) * SCOOP_FORWARD
            gy = ty - math.sin(bearing) * SCOOP_FORWARD
            if self._reach_clear is None or self._reach_clear((gx, gy), (tx, ty)):
                return (gx, gy, bearing)
        return None

    def _build_pick_sequence(self, rel: tuple[float, float]) -> list[str]:
        """Replace the fixed PICK_SCOOP with an IK reach to the measured towel
        (base_link rel position) when a solver is available — this is what lets
        the arm grasp an off-centre towel instead of a canned spot."""
        if self._ik_reach is not None:
            reach = self._ik_reach(rel[0], rel[1])
            if reach is not None:
                return [*PICK_PREAMBLE, reach, *PICK_GRASP]
        return list(PICK_SEQUENCE)

    def _handle_arm_contact(self, inp: MissionInput) -> MissionOutput:
        """The arm's forearm/gripper touched static geometry mid-sequence (a
        real servo would stall on this). Abandon the in-flight PICK/STOW
        sequence immediately — never keep pressing into the obstacle — force
        the arm home, and re-route through the normal failure paths instead of
        a special case. Holding: retry the stow (bounded — after
        MAX_STOW_RETRIES force-release over the deck rather than loop
        forever); not holding: record it as a failed pick via the same
        park/cooldown bookkeeping as a missed grasp."""
        label = self.state
        if inp.holding:
            self._stow_retries += 1
            if self._stow_retries > MAX_STOW_RETRIES:
                self._stow_retries = 0
                self._enter("STOW")
                self._seq = _SeqState(commands=["A OPEN_GRIPPER", "A HOME"])
                return MissionOutput(state=label, reason="stow keeps stalling — force release")
            self._enter("STOW")
            return MissionOutput(state=label, arm_command="A HOME",
                                 reason="arm contact — retracting, retry stow")
        out = self._record_pick_failure(label, "arm contact — aborted pick")
        out.arm_command = "A HOME"
        return out

    def _record_pick_failure(self, label: str, reason: str) -> MissionOutput:
        """Shared bookkeeping for 'this pick attempt did not succeed', used both
        by a normal missed grasp and by an arm-contact abort. Two guards stop
        the arm looping forever:
          - per-target: park a specific towel after MAX_PICK_ATTEMPTS fails
          - global: after GLOBAL_FAIL_LIMIT fails in a row (even across churning
            phantom ids that defeat the per-target counter) enter a patrol
            cooldown so the robot stops hammering and lets perception settle.
        """
        tid = self.target["id"] if self.target else None
        self._consecutive_fails += 1
        if self._consecutive_fails >= GLOBAL_FAIL_LIMIT:
            self._cooldown = COOLDOWN_TICKS
            self._consecutive_fails = 0
            self._enter("SEARCH")
            return MissionOutput(state=label, reason="too many failed picks — cooldown")
        if tid:
            self._attempts[tid] = self._attempts.get(tid, 0) + 1
            if self._attempts[tid] >= MAX_PICK_ATTEMPTS:
                self._parked[tid] = PARK_TICKS
                self._attempts.pop(tid, None)
                self._enter("SEARCH")
                return MissionOutput(state=label, reason=f"{tid} unreachable — parked")
        self._enter("SEARCH")
        return MissionOutput(state=label, reason=reason)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _run_sequence(self, inp: MissionInput, label: str) -> MissionOutput | None:
        """Steps through the queued arm commands, one per IDLE settle. Returns
        None when the sequence is finished."""
        seq = self._seq
        if seq.index >= len(seq.commands):
            return None
        if seq.sent:
            if inp.arm_status == "MOVING":
                seq.settle = 0
                return MissionOutput(
                    state=label, twist=(0.0, 0.0), reason=f"arm {seq.commands[seq.index]}",
                )
            seq.settle += 1
            if seq.settle < 3:  # debounce IDLE between segments
                return MissionOutput(state=label, twist=(0.0, 0.0), reason="settling")
            seq.index += 1
            seq.sent = False
            seq.settle = 0
            if seq.index >= len(seq.commands):
                return None
        command = seq.commands[seq.index]
        seq.sent = True
        return MissionOutput(state=label, twist=(0.0, 0.0), arm_command=command, reason=command)

    def _refresh_target(self, inp: MissionInput) -> dict[str, Any] | None:
        if self.target is None:
            return None
        for towel in inp.towels:
            if towel["id"] == self.target["id"]:
                self.target = towel
                return towel
        return None

    def _age_parked(self) -> None:
        """Tick down parked towels; a target whose timer expires is retried."""
        self._parked = {
            tid: ticks - 1 for tid, ticks in self._parked.items() if ticks - 1 > 0
        }

    def reset(self) -> None:
        """Clear all FSM state — called when the simulator session restarts
        (a browser reload) so stale targets/sequences don't carry across."""
        self.state = "IDLE"
        self.target = None
        self._seq = _SeqState()
        self._nav_goal = None
        self._nav_anchor = None
        self._was_active = False
        self._retries = 0
        self._attempts.clear()
        self._parked.clear()
        self._consecutive_fails = 0
        self._cooldown = 0
        self._stow_retries = 0
        self._stow_entry_kg = 0.0
        self._unload_cycles = 0
        self._unload_last_kg = None
        self._leftover_kg = 0.0

    def _enter(self, state: str) -> None:
        """Transition to a new state, clearing any in-flight nav goal/sequence."""
        self.state = state
        self._seq = _SeqState()
        self._nav_goal = None
        self._nav_anchor = None
        self._was_active = False
        self._nav_limbo = 0

    def _abort(self, reason: str) -> MissionOutput:
        """Safety/manual preempt: cancel any active nav goal and idle."""
        had_goal = self._nav_goal is not None
        if self.state != "IDLE":
            self._enter("IDLE")
        return MissionOutput(state="IDLE", twist=None, nav_cancel=had_goal, reason=reason)


def nearest(towels: list[dict[str, Any]], pose: dict[str, float]) -> dict[str, Any]:
    return min(towels, key=lambda t: math.hypot(
        t["position"]["x"] - pose["x"], t["position"]["y"] - pose["y"],
    ))


def world_to_robot(position: dict[str, float], pose: dict[str, float]) -> tuple[float, float]:
    dx = position["x"] - pose["x"]
    dy = position["y"] - pose["y"]
    cos_y, sin_y = math.cos(-pose["yaw"]), math.sin(-pose["yaw"])
    return (dx * cos_y - dy * sin_y, dx * sin_y + dy * cos_y)


def standoff_point(
    target: tuple[float, float], robot: tuple[float, float], distance: float,
) -> tuple[float, float]:
    dx = target[0] - robot[0]
    dy = target[1] - robot[1]
    d = math.hypot(dx, dy) or 1e-6
    return (target[0] - dx / d * distance, target[1] - dy / d * distance)


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

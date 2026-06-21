"""Mission FSM tests for the navigation-delegated autonomy loop.

The FSM no longer drives the base directly between waypoints — it emits a
`nav_goal` and waits on `nav_status`. These tests drive that protocol (goal
emitted once, waits, succeeds/fails->retry, manual/safety preempt cancels) and
the unchanged arm pick/drop sequencing. The input shapes match exactly what
mission_executor_node builds from the tracker + localizer.
"""
import math

import pytest

from onsen_ai_worker.mission import (
    DROP_SEQUENCE,
    GLOBAL_FAIL_LIMIT,
    MAX_APPROACH_RETRIES,
    MAX_PICK_ATTEMPTS,
    PICK_SEQUENCE,
    PICK_TOL_X,
    SCOOP_FORWARD,
    MissionInput,
    MissionLogic,
    MissionOutput,
)

BIN_CENTER = (0.0, 4.45)


def towel(x, y, tid="towel_1"):
    return {"id": tid, "class": "towel", "position": {"x": x, "y": y, "z": 0.02}}


def make_input(
    pose, towels=(), holding=False, arm_status="IDLE",
    safety_stop=False, mode="auto", nav_status="idle", min_front=None,
    target_rel=None,
):
    return MissionInput(
        pose=pose, towels=list(towels), holding=holding, arm_status=arm_status,
        safety_stop=safety_stop, mode=mode, nav_status=nav_status,
        min_front_obstacle=min_front, target_rel=target_rel,
    )


def drive_nav_to_success(logic, pose, towels, max_ticks=10):
    """Plays the nav server: goal emitted -> active -> succeeded. Returns the
    emitted goal tuple."""
    goal = None
    status = "idle"
    for _ in range(max_ticks):
        out = logic.update(make_input(pose, towels, nav_status=status))
        if out.nav_goal is not None:
            goal = out.nav_goal
            status = "active"
            continue
        if logic.state != "APPROACH":
            break
        status = "succeeded"
    return goal


class TestSearch:
    def test_idle_resumes_search(self):
        logic = MissionLogic(BIN_CENTER)
        logic.update(make_input({"x": 0, "y": 0, "yaw": 0}))
        assert logic.state == "SEARCH"

    def test_search_spins_when_no_towels(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "SEARCH"
        out = logic.update(make_input({"x": 0, "y": 0, "yaw": 0}))
        assert out.twist is not None
        assert out.twist[0] == 0.0
        assert out.twist[1] != 0.0

    def test_search_locks_nearest_towel(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "SEARCH"
        out = logic.update(make_input(
            {"x": 0, "y": 0, "yaw": 0},
            towels=[towel(5.0, 0.0, "far"), towel(1.5, 0.2, "near")],
        ))
        assert out.target_id == "near"
        assert logic.state == "APPROACH"


class TestApproachNavProtocol:
    def test_goal_emitted_once_then_waits(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "SEARCH"
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        towels = [towel(3.0, 0.0)]
        logic.update(make_input(pose, towels))             # SEARCH -> APPROACH
        first = logic.update(make_input(pose, towels))     # emits goal
        assert first.nav_goal is not None
        assert first.twist is None, "must not drive directly while nav owns base"
        # standoff is SCOOP_FORWARD short of the towel, facing it
        gx, _gy, gyaw = first.nav_goal
        assert math.isclose(gx, 3.0 - SCOOP_FORWARD, abs_tol=0.02)
        assert abs(gyaw) < 0.05
        second = logic.update(make_input(pose, towels, nav_status="active"))
        assert second.nav_goal is None, "goal must be emitted only once"

    def test_nav_success_advances_to_align_pick(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "SEARCH"
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        towels = [towel(3.0, 0.0)]
        logic.update(make_input(pose, towels))
        drive_nav_to_success(logic, pose, towels)
        assert logic.state in ("ALIGN_PICK", "PICK")

    def test_nav_failure_retries_then_gives_up(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "SEARCH"
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        towels = [towel(3.0, 0.0)]
        logic.update(make_input(pose, towels))
        gave_up = False
        for _ in range(MAX_APPROACH_RETRIES + 2):
            logic.update(make_input(pose, towels))                       # emit goal
            logic.update(make_input(pose, towels, nav_status="active"))  # active
            logic.update(make_input(pose, towels, nav_status="failed"))  # fail->retry
            if logic.state == "SEARCH":
                gave_up = True
                break
        assert gave_up, "exhausted retries must release the target back to SEARCH"

    def test_target_lost_cancels_nav(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "SEARCH"
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        logic.update(make_input(pose, [towel(3.0, 0.0)]))
        logic.update(make_input(pose, [towel(3.0, 0.0)]))  # goal active
        out = logic.update(make_input(pose, []))           # towel gone
        assert out.nav_cancel is True
        assert logic.state == "SEARCH"

    def test_goal_reissued_when_towel_drifts(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "SEARCH"
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        logic.update(make_input(pose, [towel(3.0, 0.0)]))
        g1 = logic.update(make_input(pose, [towel(3.0, 0.0)]))
        assert g1.nav_goal is not None
        # towel jumps 0.6 m (> RETARGET_DIST) -> new goal
        g2 = logic.update(make_input(pose, [towel(3.0, 0.7)], nav_status="active"))
        assert g2.nav_goal is not None


class TestAlignPick:
    def _at_standoff(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "ALIGN_PICK"
        logic.target = towel(SCOOP_FORWARD, 0.0)
        return logic

    def test_in_window_enters_pick(self):
        logic = self._at_standoff()
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        out = logic.update(make_input(pose, [towel(SCOOP_FORWARD, 0.0)]))
        assert logic.state == "PICK"
        assert out.twist == (0.0, 0.0)

    def test_off_bearing_rotates(self):
        logic = self._at_standoff()
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        out = logic.update(make_input(pose, [towel(0.5, 0.4)]))
        assert out.twist is not None and abs(out.twist[1]) > 0.0


class TestPickSequence:
    def _enter_pick(self, logic):
        towels = [towel(SCOOP_FORWARD, 0.0)]
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        logic.state = "ALIGN_PICK"
        logic.target = towels[0]
        logic.update(make_input(pose, towels))
        assert logic.state == "PICK"
        return pose, towels

    def _drain(self, logic, pose, towels, holding):
        sent = []
        for _ in range(100):
            out = logic.update(make_input(pose, towels, holding=holding))
            if out.arm_command:
                sent.append(out.arm_command)
                logic.update(make_input(pose, towels, holding=holding, arm_status="MOVING"))
            if logic.state not in ("PICK", "DROP"):
                break
        return sent

    def test_full_pick_sequence_in_order(self):
        logic = MissionLogic(BIN_CENTER)
        pose, towels = self._enter_pick(logic)
        sent = self._drain(logic, pose, towels, holding=True)
        assert sent == PICK_SEQUENCE
        assert logic.state == "TO_BIN"

    def test_grasp_failure_returns_to_search(self):
        logic = MissionLogic(BIN_CENTER)
        pose, towels = self._enter_pick(logic)
        sent = self._drain(logic, pose, towels, holding=False)
        assert sent == PICK_SEQUENCE
        assert logic.state == "SEARCH"

    def test_safety_stop_aborts_and_cancels(self):
        logic = MissionLogic(BIN_CENTER)
        pose, towels = self._enter_pick(logic)
        logic.update(make_input(pose, towels))  # first command
        out = logic.update(make_input(pose, towels, safety_stop=True))
        assert out.state == "IDLE"
        assert logic.state == "IDLE"
        assert logic._seq.commands == []

    def test_manual_mode_pauses(self):
        logic = MissionLogic(BIN_CENTER)
        pose, towels = self._enter_pick(logic)
        out = logic.update(make_input(pose, towels, mode="manual"))
        assert out.state == "IDLE"
        assert out.twist is None

    def test_ik_reach_replaces_canned_scoop(self):
        captured = {}

        def fake_ik(rel_x, rel_y):
            captured["rel"] = (rel_x, rel_y)
            return "J 95 150 60 78 90 80 900"

        logic = MissionLogic(BIN_CENTER, ik_reach=fake_ik)
        pose, towels = self._enter_pick(logic)
        sent = self._drain(logic, pose, towels, holding=True)
        assert "J 95 150 60 78 90 80 900" in sent, "IK reach line must be issued"
        assert "A PICK_SCOOP" not in sent, "canned scoop must be replaced"
        # the solver was queried with the measured towel rel position
        assert captured["rel"][0] == pytest.approx(SCOOP_FORWARD, abs=PICK_TOL_X)

    def test_ik_unreachable_falls_back_to_canned(self):
        logic = MissionLogic(BIN_CENTER, ik_reach=lambda rx, ry: None)
        pose, towels = self._enter_pick(logic)
        sent = self._drain(logic, pose, towels, holding=True)
        assert "A PICK_SCOOP" in sent, "must fall back to the canned scoop"


class TestDelivery:
    def test_to_bin_navigates_then_aligns_then_drops(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "TO_BIN"
        pose = {"x": 0.0, "y": 0.0, "yaw": math.pi / 2}

        first = logic.update(make_input(pose, holding=True))
        assert first.nav_goal is not None, "TO_BIN must emit a nav goal"
        # play nav to success
        status = "active"
        for _ in range(8):
            out = logic.update(make_input(pose, holding=True, nav_status=status))
            if logic.state != "TO_BIN":
                break
            status = "succeeded"
        assert logic.state == "ALIGN_BIN"

        # rotate to alignment -> DROP with the drop sequence queued
        apose = dict(pose)
        for _ in range(200):
            out = logic.update(make_input(apose, holding=True))
            if logic.state != "ALIGN_BIN":
                break
            assert out.twist is not None
            apose["yaw"] += out.twist[1] * 0.1
        assert logic.state == "DROP"
        assert logic._seq.commands == DROP_SEQUENCE

    def test_payload_lost_cancels_and_searches(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "TO_BIN"
        out = logic.update(make_input({"x": 0, "y": 0, "yaw": 0}, holding=False))
        assert out.nav_cancel is True
        assert logic.state == "SEARCH"


class TestSessionReset:
    def test_reset_clears_active_mission(self):
        logic = MissionLogic(BIN_CENTER)
        # drive the FSM into a live PICK with a queued arm sequence + nav goal
        logic.state = "TO_BIN"
        logic.update(make_input({"x": 0, "y": 0, "yaw": 0}, holding=True))
        logic._nav_goal = (1.0, 2.0, 0.0)
        logic.target = {"id": "track_9"}
        logic.reset()
        assert logic.state == "IDLE"
        assert logic.target is None
        assert logic._nav_goal is None
        assert logic._seq.commands == []

    def test_reset_then_fresh_mission_runs(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "PICK"
        logic.reset()
        # after reset a fresh tick resumes cleanly from IDLE -> SEARCH
        out = logic.update(make_input({"x": 0, "y": 0, "yaw": 0}))
        assert out.state == "IDLE"
        assert logic.state == "SEARCH"


class TestPickLoopGuard:
    def _stage_pick(self, logic, tid="track_1"):
        """Put the FSM in PICK on a towel that the grasp will keep failing on."""
        t = towel(SCOOP_FORWARD, 0.0, tid)
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        logic.state = "ALIGN_PICK"
        logic.target = t
        logic.update(make_input(pose, [t]))   # ALIGN_PICK -> PICK
        return pose, t

    def _drain_failed_pick(self, logic, pose, towels):
        for _ in range(40):
            out = logic.update(make_input(pose, towels, holding=False))
            if out.arm_command:
                logic.update(make_input(pose, towels, holding=False, arm_status="MOVING"))
            if logic.state != "PICK":
                return out
        raise AssertionError("pick sequence never finished")

    def test_target_parked_after_max_attempts(self):
        logic = MissionLogic(BIN_CENTER)
        t = towel(SCOOP_FORWARD, 0.0, "track_1")
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        # repeatedly enter PICK and fail; after MAX_PICK_ATTEMPTS it must park
        parked = False
        for _ in range(MAX_PICK_ATTEMPTS + 2):
            logic.state = "ALIGN_PICK"
            logic.target = t
            logic.update(make_input(pose, [t]))           # -> PICK
            out = self._drain_failed_pick(logic, pose, [t])
            if "parked" in out.reason:
                parked = True
                break
        assert parked, "an ungraspable towel must be parked, not looped forever"

    def test_parked_towel_not_reselected(self):
        logic = MissionLogic(BIN_CENTER)
        logic._parked = {"track_1": 100}
        logic.state = "SEARCH"
        out = logic.update(make_input(
            {"x": 0, "y": 0, "yaw": 0}, [towel(1.5, 0.0, "track_1")],
        ))
        # the only towel is parked -> stay in SEARCH scanning, do not approach it
        assert logic.state == "SEARCH"
        assert "parked" in out.reason

    def test_parked_towel_retried_after_timeout(self):
        logic = MissionLogic(BIN_CENTER)
        logic._parked = {"track_1": 1}
        logic.state = "SEARCH"
        out = logic.update(make_input(
            {"x": 0, "y": 0, "yaw": 0}, [towel(1.5, 0.0, "track_1")],
        ))
        # timer expired this tick -> towel becomes selectable again
        assert logic.state == "APPROACH"
        assert out.target_id == "track_1"

    def test_successful_grasp_clears_attempts(self):
        logic = MissionLogic(BIN_CENTER)
        logic._attempts = {"track_1": 2}
        pose, t = self._stage_pick(logic, "track_1")
        for _ in range(40):
            out = logic.update(make_input(pose, [t], holding=True))
            if out.arm_command:
                logic.update(make_input(pose, [t], holding=True, arm_status="MOVING"))
            if logic.state != "PICK":
                break
        assert "track_1" not in logic._attempts


class TestGlobalCooldown:
    def test_churning_phantom_ids_trigger_cooldown(self):
        """Phantom tracks with a *new id every failed pick* defeat the per-id
        counter, but the global consecutive-fail guard must still stop the loop."""
        logic = MissionLogic(BIN_CENTER)
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        cooled = False
        for i in range(GLOBAL_FAIL_LIMIT + 2):
            t = towel(SCOOP_FORWARD, 0.0, f"track_{i}")  # fresh id each time
            logic.state = "ALIGN_PICK"
            logic.target = t
            logic.update(make_input(pose, [t]))  # -> PICK
            out = MissionOutput(state="PICK")
            for _ in range(40):
                out = logic.update(make_input(pose, [t], holding=False))
                if out.arm_command:
                    logic.update(make_input(pose, [t], holding=False, arm_status="MOVING"))
                if logic.state != "PICK":
                    break
            if "cooldown" in out.reason:
                cooled = True
                break
        assert cooled, "churning phantom ids must still hit the global cooldown"

    def test_cooldown_suppresses_target_selection(self):
        logic = MissionLogic(BIN_CENTER)
        logic._cooldown = 5
        logic.state = "SEARCH"
        out = logic.update(make_input({"x": 0, "y": 0, "yaw": 0}, [towel(1.5, 0.0)]))
        assert logic.state == "SEARCH"
        assert "cooldown" in out.reason
        assert logic._cooldown == 4  # ticked down, not selecting


class TestDirectBaseLinkGrasp:
    def test_target_rel_overrides_localized_estimate(self):
        """When a fresh base_link measurement is supplied, the pick window check
        uses it, not the (possibly diverged) map-track-via-pose conversion."""
        logic = MissionLogic(BIN_CENTER)
        logic.state = "ALIGN_PICK"
        # the map track + a *wrong* localized pose would convert to far away...
        logic.target = towel(5.0, 5.0, "track_1")
        bad_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        # ...but the direct base_link measurement says the towel is in the window
        logic.update(make_input(
            bad_pose, [towel(5.0, 5.0, "track_1")],
            target_rel=(SCOOP_FORWARD, 0.0),
        ))
        assert logic.state == "PICK", "fresh base_link measurement must win"

    def test_falls_back_to_track_when_no_measurement(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "ALIGN_PICK"
        logic.target = towel(SCOOP_FORWARD, 0.0, "track_1")
        logic.update(make_input(
            {"x": 0.0, "y": 0.0, "yaw": 0.0}, [towel(SCOOP_FORWARD, 0.0, "track_1")],
            target_rel=None,
        ))
        assert logic.state == "PICK"  # track-via-pose still works when aligned

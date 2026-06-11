"""Mission FSM tests: full happy path, grasp failure and safety abort.

The FSM is fed the same shapes the executor node builds from
/ground_truth/objects + /robot/state, so these are contract tests too.
"""
import math

from onsen_ai_worker.mission import (
    DROP_OFFSET,
    DROP_SEQUENCE,
    PICK_SEQUENCE,
    SCOOP_FORWARD,
    MissionInput,
    MissionLogic,
)

BIN_CENTER = (0.0, 4.45)


def towel(x, y, tid="towel_1"):
    return {"id": tid, "class": "towel", "position": {"x": x, "y": y, "z": 0.02}}


def make_input(
    pose, towels=(), holding=False, arm_status="IDLE",
    safety_stop=False, mode="auto", min_front=None,
):
    return MissionInput(
        pose=pose, towels=list(towels), holding=holding, arm_status=arm_status,
        safety_stop=safety_stop, mode=mode, min_front_obstacle=min_front,
    )


class TestSearchAndApproach:
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

    def test_approach_drives_toward_towel(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "SEARCH"
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        towels = [towel(3.0, 0.0)]
        logic.update(make_input(pose, towels))
        out = logic.update(make_input(pose, towels))
        assert logic.state == "APPROACH"
        assert out.twist is not None
        assert out.twist[0] > 0.0

    def test_pick_window_triggers_pick(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "SEARCH"
        towels = [towel(SCOOP_FORWARD, 0.0)]
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        logic.update(make_input(pose, towels))
        logic.update(make_input(pose, towels))
        assert logic.state == "PICK"

    def test_lost_target_falls_back_to_search(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "SEARCH"
        logic.update(make_input({"x": 0, "y": 0, "yaw": 0}, [towel(2.0, 0.0)]))
        logic.update(make_input({"x": 0, "y": 0, "yaw": 0}, []))
        assert logic.state == "SEARCH"

    def test_obstacle_caps_approach_speed(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "SEARCH"
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        towels = [towel(4.0, 0.0)]
        logic.update(make_input(pose, towels))
        out = logic.update(make_input(pose, towels, min_front=0.3))
        assert out.twist is not None
        assert out.twist[0] <= 0.08


class TestPickSequence:
    def _enter_pick(self, logic):
        towels = [towel(SCOOP_FORWARD, 0.0)]
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        logic.state = "SEARCH"
        logic.update(make_input(pose, towels))
        logic.update(make_input(pose, towels))
        assert logic.state == "PICK"
        return pose, towels

    def _drain_sequence(self, logic, pose, towels, holding):
        sent = []
        for _ in range(100):
            out = logic.update(make_input(pose, towels, holding=holding))
            if out.arm_command:
                sent.append(out.arm_command)
                logic.update(make_input(pose, towels, holding=holding, arm_status="MOVING"))
            if logic.state not in ("PICK", "DROP"):
                break
        return sent

    def test_full_pick_sequence_emitted_in_order(self):
        logic = MissionLogic(BIN_CENTER)
        pose, towels = self._enter_pick(logic)
        sent = self._drain_sequence(logic, pose, towels, holding=True)
        assert sent == PICK_SEQUENCE
        assert logic.state == "TO_BIN"

    def test_grasp_failure_retries_via_search(self):
        logic = MissionLogic(BIN_CENTER)
        pose, towels = self._enter_pick(logic)
        sent = self._drain_sequence(logic, pose, towels, holding=False)
        assert sent == PICK_SEQUENCE
        assert logic.state == "SEARCH"

    def test_safety_stop_aborts_mid_pick(self):
        logic = MissionLogic(BIN_CENTER)
        pose, towels = self._enter_pick(logic)
        logic.update(make_input(pose, towels))  # first command sent
        out = logic.update(make_input(pose, towels, safety_stop=True))
        assert out.state == "IDLE"
        assert logic.state == "IDLE"
        assert logic._seq.commands == []  # sequence must not resume after reset

    def test_manual_mode_pauses_mission(self):
        logic = MissionLogic(BIN_CENTER)
        pose, towels = self._enter_pick(logic)
        out = logic.update(make_input(pose, towels, mode="manual"))
        assert out.state == "IDLE"
        assert out.twist is None


class TestDelivery:
    def test_to_bin_drives_then_aligns_then_drops(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "TO_BIN"
        far_pose = {"x": 0.0, "y": 0.0, "yaw": math.pi / 2}
        out = logic.update(make_input(far_pose, holding=True))
        assert out.twist is not None
        assert logic.state == "TO_BIN"

        # teleport to the standoff point -> ALIGN_BIN
        standoff = math.hypot(*DROP_OFFSET)
        near_pose = {"x": 0.0, "y": BIN_CENTER[1] - standoff, "yaw": math.pi / 2}
        logic.update(make_input(near_pose, holding=True))
        assert logic.state == "ALIGN_BIN"

        # rotate until aligned -> DROP with the drop sequence queued
        pose = dict(near_pose)
        for _ in range(200):
            out = logic.update(make_input(pose, holding=True))
            if logic.state != "ALIGN_BIN":
                break
            assert out.twist is not None
            pose["yaw"] += out.twist[1] * 0.1
        assert logic.state == "DROP"
        assert logic._seq.commands == DROP_SEQUENCE

    def test_payload_lost_returns_to_search(self):
        logic = MissionLogic(BIN_CENTER)
        logic.state = "TO_BIN"
        logic.update(make_input({"x": 0, "y": 0, "yaw": 0}, holding=False))
        assert logic.state == "SEARCH"

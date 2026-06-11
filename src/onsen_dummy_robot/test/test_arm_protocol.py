"""Arm firmware transcript replay — verifies the exact serial grammar the
robot's arm controller exposes, including the user-facing sample session."""
from __future__ import annotations

from onsen_dummy_robot.arm_protocol import ACTIONS, ArmFirmware


def fw() -> ArmFirmware:
    return ArmFirmware(cal_path=None)


class TestTranscript:
    """The reference session from the firmware spec, end to end."""

    def test_full_session(self):
        arm = fw()
        assert arm.handle("Q") == ["STATE 90 90 90 90 90 70 IDLE"]
        assert arm.handle("A HOME") == ["OK ACTION HOME"]
        assert arm.handle("A READY") == ["OK ACTION READY"]
        assert arm.handle("J 90 70 120 110 90 40 1000") == ["OK J 90 70 120 110 90 40"]
        # relative beyond the limit: goal(0)=90, +200 -> 290 > 180
        assert arm.handle("D 0 200 300") == ["ERR LIMIT joint=0 value=290"]
        assert arm.handle("M BASE_LEFT 10 250") == ["OK M BASE_LEFT amount=10"]
        assert arm.handle("G 40 200") == ["OK G 40"]
        assert arm.handle("SPEED 50") == ["OK SPEED 50"]
        assert arm.handle("STOP") == ["OK STOP"]
        assert arm.handle("A HOME") == ["ERR STOPPED"]
        assert arm.handle("A RESET_ERROR") == ["OK RESET_ERROR"]
        assert arm.handle("A HOME") == ["OK ACTION HOME"]

    def test_query_reports_moving_then_idle(self):
        arm = fw()
        arm.handle("J 100 90 90 90 90 70 500")
        assert arm.handle("Q")[0].endswith("MOVING")
        arm._seg_start_t = 0.0
        arm.tick(now=10.0)
        assert arm.handle("Q")[0].endswith("IDLE")


class TestInterpolation:
    def test_segment_reaches_target_after_duration(self):
        arm = fw()
        arm.handle("J 120 90 90 90 90 70 1000")
        arm._seg_start_t = 0.0
        arm.tick(now=0.5)
        assert 100 < arm.positions[0] < 110
        arm.tick(now=1.1)
        assert arm.positions[0] == 120

    def test_speed_scale_stretches_duration(self):
        arm = fw()
        arm.handle("SPEED 50")
        arm.handle("J 120 90 90 90 90 70 1000")
        arm._seg_start_t = 0.0
        arm.tick(now=1.0)  # 1000 ms at 50% -> only halfway
        assert arm.positions[0] < 120

    def test_relaxed_joint_does_not_move(self):
        arm = fw()
        arm.handle("RELAX 0")
        arm.handle("J 120 90 90 90 90 70 100")
        arm._seg_start_t = 0.0
        arm.tick(now=10.0)
        assert arm.positions[0] == 90


class TestActions:
    def test_every_named_pose_is_within_servo_limits(self):
        for name, pose in ACTIONS.items():
            if pose is None:
                continue
            for value in pose:
                assert 0 <= value <= 180, f"{name} out of range"

    def test_pick_grip_closes_gripper(self):
        grip, release = ACTIONS["PICK_GRIP"], ACTIONS["DROP_RELEASE"]
        assert grip is not None and release is not None
        assert grip[5] <= 30
        assert release[5] >= 70

    def test_shake_light_queues_wiggle(self):
        arm = fw()
        assert arm.handle("A SHAKE_LIGHT") == ["OK ACTION SHAKE_LIGHT"]
        assert arm.state_dict()["queue_depth"] == 5

    def test_unknown_action(self):
        assert fw().handle("A FLY") == ["ERR UNKNOWN_ACTION FLY"]


class TestCalibration:
    def test_cal_deg_narrows_limits(self):
        arm = fw()
        arm.handle("CAL DEG 0 30 150")
        assert arm.handle("J 20 90 90 90 90 70 500") == ["ERR LIMIT joint=0 value=20"]
        assert arm.handle("J 40 90 90 90 90 70 500")[0].startswith("OK J")

    def test_cal_show_lists_all_joints(self):
        assert len(fw().handle("CAL SHOW")) == 6


class TestErrors:
    def test_bad_args(self):
        assert fw().handle("J nope")[0].startswith("ERR BAD_ARGS")

    def test_unknown_command(self):
        assert fw().handle("XYZ") == ["ERR UNKNOWN_CMD XYZ"]

    def test_bad_speed(self):
        assert fw().handle("SPEED 0") == ["ERR BAD_SPEED 0"]

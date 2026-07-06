"""URDF generation + servo-coupling bridge tests."""
import json
import math
import xml.dom.minidom

import pytest
from onsen_arm.joint_bridge import ARM_JOINTS, couple_arm_positions, remap_joint_state
from onsen_arm.urdf import build_urdf

with open("shared/robot_spec.json") as _f:
    SPEC = json.load(_f)
RATIO = SPEC["arm"]["joint_ratio"]


class TestUrdf:
    def test_valid_xml(self):
        dom = xml.dom.minidom.parseString(build_urdf(SPEC))
        root = dom.documentElement
        assert root is not None
        assert root.tagName == "robot"

    def test_all_fe_joint_names_present(self):
        urdf = build_urdf(SPEC)
        for name in ARM_JOINTS:
            assert f'name="{name}"' in urdf

    def test_pan_and_pitch_limits_from_spec(self):
        urdf = build_urdf(SPEC)
        # pitch joints span +/- 90*ratio degrees
        assert f'{math.radians(90) * RATIO:.4f}' in urdf

    def test_link_chain_connected(self):
        urdf = build_urdf(SPEC)
        assert 'parent link="arm_base_link"' in urdf
        assert "fingertip_link" in urdf


class TestCoupling:
    def test_home_is_neutral(self):
        # all servos at 90 -> all offsets 0 -> tilts 0, gripper at mid
        coupled = couple_arm_positions([0, 0, 0, 0, 0, 0], RATIO)
        assert coupled[:5] == pytest.approx([0, 0, 0, 0, 0])
        assert coupled[5] == pytest.approx(math.pi / 2)

    def test_shoulder_lift_scaled_by_ratio(self):
        # servo offset of 0.1 rad on shoulder -> ratio * 0.1 tilt
        coupled = couple_arm_positions([0, 0.1, 0, 0, 0, 0], RATIO)
        assert coupled[1] == pytest.approx(RATIO * 0.1)

    def test_elbow_wrist_sign_inverted(self):
        coupled = couple_arm_positions([0, 0, 0.1, 0.1, 0, 0], RATIO)
        assert coupled[2] == pytest.approx(-RATIO * 0.1)
        assert coupled[3] == pytest.approx(-RATIO * 0.1)

    def test_pan_is_one_to_one(self):
        coupled = couple_arm_positions([0.3, 0, 0, 0, 0, 0], RATIO)
        assert coupled[0] == pytest.approx(0.3)


class TestRemap:
    def test_arm_coupled_wheels_passthrough(self):
        names = [*ARM_JOINTS, "wheel_left", "wheel_right"]
        positions = [0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 1.23, 4.56]
        out_names, out_pos = remap_joint_state(names, positions, RATIO)
        assert "wheel_left" in out_names
        assert 1.23 in out_pos and 4.56 in out_pos
        # shoulder_lift coupled
        assert out_pos[out_names.index("shoulder_lift_joint")] == pytest.approx(RATIO * 0.1)

    def test_missing_arm_joints_passes_through(self):
        out_names, out_pos = remap_joint_state(["wheel_x"], [2.0], RATIO)
        assert out_names == ["wheel_x"]
        assert out_pos == [2.0]

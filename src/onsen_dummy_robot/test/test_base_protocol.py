"""Base firmware protocol tests: kinematics mapping, limits, latches."""
from __future__ import annotations

import math

from onsen_dummy_robot.base_protocol import (
    BIN_DUMP_DEG,
    BIN_FULL_KG,
    BIN_LOAD_DEBOUNCE_S,
    BIN_TILT_SPEED_DPS,
    MAX_WHEEL_RADPS,
    TRACK_WIDTH,
    WHEEL_RADIUS,
    BaseFirmware,
)


class TestTwist:
    def test_straight_drive_maps_to_equal_sides(self):
        fw = BaseFirmware()
        fw.handle("V 0.3 0")
        wheels = fw.scaled_wheels()
        assert all(math.isclose(w, 0.3 / WHEEL_RADIUS, rel_tol=1e-6) for w in wheels)

    def test_pure_rotation_maps_to_opposite_sides(self):
        fw = BaseFirmware()
        fw.handle("V 0 1.0")
        wheels = fw.scaled_wheels()
        expected = (1.0 * TRACK_WIDTH / 2) / WHEEL_RADIUS
        assert math.isclose(wheels[0], -expected, rel_tol=1e-6)
        assert math.isclose(wheels[3], expected, rel_tol=1e-6)

    def test_command_timeout_zeroes_output(self):
        fw = BaseFirmware()
        fw.handle("V 0.3 0")
        assert any(fw.scaled_wheels())
        assert fw.scaled_wheels(now=fw._last_cmd_t + 2.0) == [0.0] * 6


class TestWheelMode:
    def test_single_wheel_command(self):
        fw = BaseFirmware()
        assert fw.handle("W 2 5.0") == ["OK W 2 5"]
        assert fw.scaled_wheels()[2] == 5.0

    def test_wheel_limit(self):
        fw = BaseFirmware()
        assert fw.handle(f"W 0 {MAX_WHEEL_RADPS + 1}") == [
            f"ERR LIMIT wheel=0 value={MAX_WHEEL_RADPS + 1:g}",
        ]

    def test_bad_wheel_index(self):
        assert BaseFirmware().handle("W 9 1.0") == ["ERR BAD_WHEEL 9"]


class TestLatches:
    def test_stop_latch_blocks_motion_until_reset(self):
        fw = BaseFirmware()
        fw.handle("STOP")
        assert fw.handle("V 0.3 0") == ["ERR STOPPED"]
        assert fw.status == "STOPPED"
        fw.handle("RESET_ERROR")
        assert fw.handle("V 0.3 0") == ["OK V 0.3 0"]

    def test_safety_latch_zeroes_and_blocks(self):
        fw = BaseFirmware()
        fw.handle("V 0.3 0")
        assert fw.set_safety(True) is True
        assert fw.scaled_wheels() == [0.0] * 6
        assert fw.handle("V 0.3 0") == ["ERR SAFETY_STOP"]
        assert fw.status == "SAFETY"
        # RESET_ERROR must NOT clear the safety latch
        fw.handle("RESET_ERROR")
        assert fw.handle("V 0.3 0") == ["ERR SAFETY_STOP"]
        fw.set_safety(False)
        assert fw.handle("V 0.3 0") == ["OK V 0.3 0"]

    def test_twist_ignored_while_safety(self):
        fw = BaseFirmware()
        fw.set_safety(True)
        fw.set_twist(0.5, 0.0)
        assert fw.scaled_wheels() == [0.0] * 6


class TestSpeedScale:
    def test_speed_pct_scales_output(self):
        fw = BaseFirmware()
        fw.handle("SPEED 50")
        fw.handle("T 0.7 0.7")
        assert math.isclose(fw.scaled_wheels()[0], 0.5 * 0.7 / WHEEL_RADIUS, rel_tol=1e-6)

    def test_query_format(self):
        fw = BaseFirmware()
        reply = fw.handle("Q")[0]
        parts = reply.split()
        assert parts[0] == "STATE"
        assert len(parts) == 10
        assert parts[-1] == "IDLE"


class TestCollectBin:
    def test_dump_and_home_interpolate_at_servo_speed(self):
        fw = BaseFirmware()
        assert fw.handle("BIN DUMP") == ["OK BIN DUMP"]
        t0 = fw._bin_tilt_at
        # halfway through the swing
        half = BIN_DUMP_DEG / BIN_TILT_SPEED_DPS / 2
        assert math.isclose(fw.bin_tilt(t0 + half), BIN_DUMP_DEG / 2, rel_tol=1e-6)
        # fully dumped after the full travel time
        assert fw.bin_tilt(t0 + 10.0) == BIN_DUMP_DEG
        fw.handle("BIN HOME")
        t1 = fw._bin_tilt_at
        assert fw.bin_tilt(t1 + 10.0) == 0.0

    def test_bin_full_debounces_load(self):
        fw = BaseFirmware()
        fw.observe_bin_load(BIN_FULL_KG + 0.1, now=0.0)
        assert fw.state_dict()["bin_full"] is False, "single reading must not latch"
        fw.observe_bin_load(BIN_FULL_KG + 0.1, now=BIN_LOAD_DEBOUNCE_S + 0.1)
        assert fw.state_dict()["bin_full"] is True
        # emptying the bin clears the latch immediately
        fw.observe_bin_load(0.0, now=BIN_LOAD_DEBOUNCE_S + 0.2)
        assert fw.state_dict()["bin_full"] is False

    def test_light_load_never_fills(self):
        fw = BaseFirmware()
        for i in range(10):
            fw.observe_bin_load(BIN_FULL_KG - 0.1, now=float(i))
        assert fw.state_dict()["bin_full"] is False

    def test_bin_query_reply(self):
        fw = BaseFirmware()
        fw.observe_bin_load(0.25, now=0.0)
        reply = fw.handle("BIN Q")[0]
        assert reply.startswith("BIN ")
        assert "0.250" in reply

    def test_state_dict_carries_bin_channel(self):
        fw = BaseFirmware()
        fw.handle("BIN DUMP")
        s = fw.state_dict()
        assert s["bin_tilt_target_deg"] == BIN_DUMP_DEG
        assert "bin_kg" in s and "bin_full" in s

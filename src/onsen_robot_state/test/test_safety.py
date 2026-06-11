"""Safety monitor tests — driven by realistic contact payloads as published
by the FE simulator on /robot/contacts."""
import math

from onsen_robot_state.safety import (
    SafetyMonitor,
    arm_in_drop_phase,
    mask_arm_sector,
)

# Captured from the FE contact reporter schema
STOOL_IMPACT = {
    "part": "chassis", "impulse": 4.2, "normal": [0.97, -0.24, 0.0],
    "object_id": "stool_3", "object_class": "stool", "critical": False,
}
TOWEL_BRUSH = {
    "part": "chassis", "impulse": 0.4, "normal": [0.1, 0.99, 0.0],
    "object_id": "towel_1", "object_class": "towel", "critical": False,
}
WATER_INGRESS = {
    "part": "chassis", "impulse": 0.2, "normal": [0.0, 0.0, 1.0],
    "object_id": "cold_bath_w", "object_class": "water", "critical": True,
}


def monitor() -> SafetyMonitor:
    return SafetyMonitor(impulse_threshold=3.0, tilt_limit_deg=30.0)


class TestContactLatching:
    def test_hard_impact_latches(self):
        m = monitor()
        assert m.on_contact(4.2, False, STOOL_IMPACT) is True
        assert m.stop is True

    def test_soft_brush_does_not_latch(self):
        m = monitor()
        assert m.on_contact(0.4, False, TOWEL_BRUSH) is False
        assert m.stop is False

    def test_critical_contact_latches_critical(self):
        m = monitor()
        assert m.on_contact(0.2, True, WATER_INGRESS) is True
        assert m.stop is True
        assert m.critical is True

    def test_retrigger_does_not_duplicate_events(self):
        m = monitor()
        m.on_contact(4.2, False, STOOL_IMPACT)
        m.drain_events()
        assert m.on_contact(5.0, False, STOOL_IMPACT) is False
        assert m.drain_events() == []

    def test_reset_clears_latch(self):
        m = monitor()
        m.on_contact(4.2, False, STOOL_IMPACT)
        assert m.reset() is True
        assert m.stop is False
        assert m.critical is False


class TestTilt:
    def test_tilt_latches_and_blocks_reset(self):
        m = monitor()
        assert m.on_tilt(35.0) is True
        assert m.stop is True
        assert m.reset() is False, "reset must be refused while still tilted"
        m.on_tilt(5.0)
        assert m.reset() is True
        assert m.stop is False

    def test_event_emitted_once_per_tilt_episode(self):
        m = monitor()
        m.on_tilt(35.0)
        m.on_tilt(40.0)
        assert len(m.drain_events()) == 1


class TestArmScanSelfFilter:
    def test_drop_phase_detection(self):
        assert arm_in_drop_phase({"last_action": "DROP_BASKET", "status": "MOVING"})
        assert arm_in_drop_phase({"last_action": "DROP_RELEASE", "status": "IDLE"})
        assert not arm_in_drop_phase({"last_action": "HOME", "status": "MOVING"})
        assert not arm_in_drop_phase({})

    def test_mask_invalidates_only_arm_sector(self):
        n = 360
        angle_min = -math.pi
        inc = 2 * math.pi / n
        ranges = [2.0] * n
        masked = mask_arm_sector(ranges, angle_min, inc)
        for i, r in enumerate(masked):
            angle_deg = math.degrees(angle_min + i * inc)
            if 70.0 <= angle_deg <= 90.0:
                assert math.isinf(r), f"beam at {angle_deg:.0f} deg must be filtered"
            else:
                assert r == 2.0, f"beam at {angle_deg:.0f} deg must pass through"

    def test_masked_beams_never_win_min_obstacle(self):
        n = 360
        angle_min = -math.pi
        inc = 2 * math.pi / n
        # arm appears as a 0.3 m return at +80 deg; nearest real wall at 1.5 m
        ranges = [1.5] * n
        arm_beam = round((math.radians(80) - angle_min) / inc)
        ranges[arm_beam] = 0.3
        masked = mask_arm_sector(ranges, angle_min, inc)
        assert min(r for r in masked if math.isfinite(r)) == 1.5

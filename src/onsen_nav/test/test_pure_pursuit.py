"""Regulated pure-pursuit tracker tests."""
import math

from onsen_nav.pure_pursuit import PurePursuit, TrackerParams


def tracker() -> PurePursuit:
    return PurePursuit(TrackerParams())


class TestTracking:
    def test_straight_ahead_drives_forward(self):
        out = tracker().step((0, 0, 0), [(0.0, 0.0), (3.0, 0.0)])
        assert out.vx > 0.2
        assert abs(out.wz) < 0.05
        assert not out.done

    def test_goal_behind_rotates_in_place(self):
        out = tracker().step((0, 0, 0), [(0.0, 0.0), (-2.0, 0.0)])
        assert out.vx == 0.0
        assert abs(out.wz) > 0.5

    def test_arrival_within_tolerance(self):
        out = tracker().step((2.95, 0.02, 0), [(0.0, 0.0), (3.0, 0.0)])
        assert out.done
        assert out.vx == 0.0 and out.wz == 0.0

    def test_curvature_regulation_slows_turns(self):
        straight = tracker().step((0, 0, 0), [(0.0, 0.0), (3.0, 0.0)])
        # target 30 deg off-axis -> curvature -> regulated speed
        curved = tracker().step((0, 0, 0), [(0.0, 0.0), (3.0, 1.7)])
        assert curved.vx <= straight.vx

    def test_approach_slowdown(self):
        far = tracker().step((0, 0, 0), [(0.0, 0.0), (5.0, 0.0)])
        near = tracker().step((4.7, 0, 0), [(0.0, 0.0), (5.0, 0.0)])
        assert near.vx < far.vx

    def test_obstacle_stop_and_slow(self):
        t = tracker()
        path = [(0.0, 0.0), (3.0, 0.0)]
        assert t.step((0, 0, 0), path, min_obstacle=0.15).blocked
        slowed = t.step((0, 0, 0), path, min_obstacle=0.35)
        assert slowed.vx <= TrackerParams().v_obstacle + 1e-9

    def test_passed_waypoint_never_flips_target_behind(self):
        # regression: robot 0.6 m past wp0 on a long sparse segment — the carrot
        # must stay ahead (old nearest-waypoint search produced a limit cycle)
        path = [(0.0, 0.0), (5.0, 0.0)]
        out = tracker().step((0.6, 0.05, 0.0), path)
        assert out.vx > 0.0, "must keep driving forward"
        assert abs(out.wz) < 0.3, "must not whip around toward the passed waypoint"

    def test_carrot_interpolates_along_segment(self):
        path = [(0.0, 0.0), (10.0, 0.0)]
        out = tracker().step((2.0, 0.0, 0.0), path)
        # lookahead point is exactly 0.45 ahead on the segment -> straight line
        assert out.vx > 0.2
        assert abs(out.wz) < 1e-6

    def test_lookahead_follows_polyline_corner(self):
        # L-shaped path; from the corner area the target must be on the second leg
        path = [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0)]
        out = tracker().step((1.0, 0.1, math.pi / 2), path)
        assert out.vx > 0.0
        assert out.distance_remaining > 1.0

    def test_obstacle_slowdown_preserves_steering_authority(self):
        """Regression: wz used to be v*kappa using the OBSTACLE-slowed v, so
        crawling near a wall crushed steering (couldn't correct bearing to
        curve past it) -> stop-rotate-crawl limit cycle. wz must now come from
        the curvature/approach-regulated speed, not the post-obstacle speed."""
        t = tracker()
        path = [(0.0, 0.0), (3.0, 1.7)]  # off-axis target -> nonzero curvature
        clear = t.step((0, 0, 0), path, min_obstacle=None)
        slowed = t.step((0, 0, 0), path, min_obstacle=0.35)  # inside slow range
        assert slowed.vx < clear.vx, "obstacle proximity must still slow the robot"
        assert slowed.wz == clear.wz, "but must not reduce steering rate"

"""Towel tracker: association, smoothing, debounce, visibility-aware expiry."""
import math

from onsen_ai_worker.towel_tracker import MIN_HITS, MISSES_TO_EXPIRE, TowelTracker

ORIGIN = (0.0, 0.0, 0.0)  # robot at map origin facing +x


def det(x: float, y: float) -> dict:
    """A towel detection in base_link."""
    return {"position": {"x": x, "y": y, "z": 0.02}, "source": "depth", "range": math.hypot(x, y)}


class TestAssociation:
    def test_track_confirmed_after_min_hits(self):
        tracker = TowelTracker()
        for i in range(MIN_HITS - 1):
            assert tracker.update([det(1.5, 0.2)], ORIGIN, i * 0.2) == []
        tracks = tracker.update([det(1.52, 0.21)], ORIGIN, MIN_HITS * 0.2)
        assert len(tracks) == 1
        assert tracks[0]["hits"] == MIN_HITS

    def test_noise_jitter_associates_to_same_track(self):
        tracker = TowelTracker()
        for i in range(10):
            tracks = tracker.update([det(1.5 + 0.03 * (i % 3 - 1), 0.2)], ORIGIN, i * 0.2)
        assert len(tracks) == 1
        assert tracks[0]["id"] == "track_1"

    def test_two_towels_two_tracks(self):
        tracker = TowelTracker()
        dets = [det(1.5, 0.4), det(2.0, -0.6)]
        for i in range(MIN_HITS):
            tracks = tracker.update(dets, ORIGIN, i * 0.2)
        assert len(tracks) == 2

    def test_ema_converges_to_truth(self):
        tracker = TowelTracker()
        for i in range(30):
            tracks = tracker.update([det(2.0, 0.0)], ORIGIN, i * 0.2)
        assert abs(tracks[0]["position"]["x"] - 2.0) < 0.01

    def test_world_frame_under_robot_motion(self):
        tracker = TowelTracker()
        # towel fixed at map (2.0, 1.0); robot moves and turns between frames
        for i in range(MIN_HITS - 1):
            tracker.update([det(2.0, 1.0)], (0.0, 0.0, 0.0), i * 0.2)
        # robot at (1.0, 0.0) yaw 45 deg: towel rel = R(-45)*(1.0, 1.0)
        c = math.cos(-math.pi / 4)
        s = math.sin(-math.pi / 4)
        rel = (c * 1.0 - s * 1.0, s * 1.0 + c * 1.0)
        tracks = tracker.update([det(*rel)], (1.0, 0.0, math.pi / 4), MIN_HITS * 0.2)
        assert len(tracks) == 1
        assert abs(tracks[0]["position"]["x"] - 2.0) < 0.05
        assert abs(tracks[0]["position"]["y"] - 1.0) < 0.05


class TestExpiry:
    def test_track_survives_when_robot_looks_away(self):
        tracker = TowelTracker()
        for i in range(MIN_HITS):
            tracker.update([det(1.5, 0.0)], ORIGIN, i * 0.2)
        # robot turns around — towel behind, not expected visible
        away = (0.0, 0.0, math.pi)
        for i in range(2 * MISSES_TO_EXPIRE):
            tracks = tracker.update([], away, 1.0 + i * 0.2)
        assert len(tracks) == 1, "out-of-view towels must persist"

    def test_track_expires_when_visibly_gone(self):
        tracker = TowelTracker()
        for i in range(MIN_HITS):
            tracker.update([det(1.5, 0.0)], ORIGIN, i * 0.2)
        for i in range(MISSES_TO_EXPIRE):
            tracks = tracker.update([], ORIGIN, 1.0 + i * 0.2)
        assert tracks == [], "a towel the robot stares at but cannot see is gone"

    def test_stale_track_expires_even_unseen(self):
        from onsen_ai_worker.towel_tracker import STALE_AGE_S
        tracker = TowelTracker()
        for i in range(MIN_HITS):
            tracker.update([det(1.5, 0.0)], ORIGIN, i * 0.2)
        away = (0.0, 0.0, math.pi)
        tracks = tracker.update([], away, STALE_AGE_S + 1.0)
        assert tracks == []

    def test_forget_near_removes_grasped_towel(self):
        tracker = TowelTracker()
        for i in range(MIN_HITS):
            tracker.update([det(0.6, 0.0)], ORIGIN, i * 0.2)
        assert tracker.tracks(), "precondition: track published before forget"
        tracker.forget_near(0.6, 0.0)
        assert tracker.tracks() == []

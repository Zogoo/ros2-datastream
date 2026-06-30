"""Tests for SessionWatch — FE session-restart detection.

The key property: a restart is signalled exactly once per genuine session
change, and two concurrent tabs (interleaved heartbeats) never cause a storm.
"""
from onsen_robot_state.session import DEFAULT_STALE_S, SessionWatch


def _hb(session_id=None, sim_time=None):
    payload = {"alive": True}
    if session_id is not None:
        payload["session_id"] = session_id
    if sim_time is not None:
        payload["sim_time"] = sim_time
    return payload


def test_first_heartbeat_is_not_a_restart():
    w = SessionWatch()
    assert w.update(_hb("A"), now=0.0) is False


def test_same_session_never_restarts():
    w = SessionWatch()
    w.update(_hb("A"), now=0.0)
    for t in range(1, 100):
        assert w.update(_hb("A"), now=float(t)) is False


def test_reload_after_old_session_goes_silent_resets_once():
    w = SessionWatch(stale_s=3.0)
    w.update(_hb("A"), now=0.0)
    # New tab "B" arrives while "A" was just seen — not yet a restart.
    assert w.update(_hb("B"), now=1.0) is False
    assert w.update(_hb("B"), now=2.0) is False
    # Once A has been silent >= stale_s, B is accepted as the live session.
    assert w.update(_hb("B"), now=3.5) is True
    # And only once.
    assert w.update(_hb("B"), now=4.5) is False


def test_two_concurrent_tabs_do_not_storm():
    """A and B both alive, heartbeats interleaved every ~1 s. The locked
    session (A) is always seen within stale_s, so B is ignored: zero resets."""
    w = SessionWatch(stale_s=3.0)
    w.update(_hb("A"), now=0.0)
    resets = 0
    t = 0.5
    for _ in range(50):
        if w.update(_hb("B"), now=t):
            resets += 1
        t += 0.5
        if w.update(_hb("A"), now=t):
            resets += 1
        t += 0.5
    assert resets == 0


def test_stale_default_exceeds_heartbeat_period():
    # The lock must survive a single missed heartbeat (period 1.0 s).
    assert DEFAULT_STALE_S > 1.0


def test_legacy_sim_time_regression_still_detected():
    w = SessionWatch()
    assert w.update(_hb(sim_time=10.0), now=0.0) is False
    assert w.update(_hb(sim_time=10.5), now=1.0) is False
    # A regression > 1.0 s signals a restart on the legacy (no session_id) path.
    assert w.update(_hb(sim_time=0.0), now=2.0) is True


def test_malformed_payload_is_ignored():
    w = SessionWatch()
    assert w.update_raw("not json", now=0.0) is False
    assert w.update_raw('{"session_id": "A"}', now=0.0) is False
    assert w.update_raw('{"session_id": "B"}', now=5.0) is True

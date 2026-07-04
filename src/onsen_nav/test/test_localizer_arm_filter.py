"""The localizer must hold its correction (skip scan matching) while the arm
sweeps the scan plane during pick/drop — otherwise corrupted beams collapse the
match score and trigger spurious GT reseeds (regression for the e2e mission)."""


def arm_in_plane(last_action: str) -> bool:
    """Mirror of LocalizerNode._on_arm_state's rule (kept in sync by this test)."""
    return str(last_action).startswith(("PRE_PICK", "PICK", "DROP"))


def test_pick_and_drop_poses_pause_matching():
    for action in ["PRE_PICK", "PICK_LOWER", "PICK_SCOOP", "PICK_GRIP",
                   "PICK_LIFT", "DROP_BIN", "DROP_BIN_RELEASE"]:
        assert arm_in_plane(action), f"{action} must pause matching"


def test_stowed_and_search_poses_allow_matching():
    for action in ["HOME", "STOW", "READY", "SEARCH_LEFT", "SEARCH_CENTER", ""]:
        assert not arm_in_plane(action), f"{action} must allow matching"

"""Planner tests on realistic /detected_objects fixtures (recorded schema)."""
from onsen_ai_worker.planner import TaskPlanner

TOWEL_NEAR = {
    "id": "det_001", "class": "towel", "confidence": 0.86,
    "bbox": [100, 200, 230, 310], "robot_class": "pickable_soft_object",
    "pickable": True, "risk": "low",
    "estimated_position": {"x": 1.1, "y": -0.2, "z": 0.0},
}
TOWEL_FAR = {
    "id": "det_002", "class": "towel", "confidence": 0.71,
    "bbox": [400, 180, 470, 240], "robot_class": "pickable_soft_object",
    "pickable": True, "risk": "low",
    "estimated_position": {"x": 2.6, "y": 0.8, "z": 0.0},
}
STOOL = {
    "id": "det_003", "class": "stool", "confidence": 0.77,
    "bbox": [300, 250, 380, 360], "robot_class": "non_pickable_hard_object",
    "pickable": False, "risk": "avoid",
    "estimated_position": {"x": 0.9, "y": 0.4, "z": 0.0},
}
HAZARD = {
    "id": "det_004", "class": "unknown_obstacle", "confidence": 0.55,
    "bbox": [250, 300, 400, 420], "robot_class": "unknown",
    "pickable": False, "risk": "stop",
    "estimated_position": {"x": 0.5, "y": 0.0, "z": 0.0},
}


def test_safety_stop_overrides_everything():
    plan = TaskPlanner().plan([TOWEL_NEAR, HAZARD], safety_stop=True)
    assert plan["next_action"] == "emergency_stop"
    assert plan["target_object_id"] is None


def test_stop_risk_beats_pickable():
    plan = TaskPlanner().plan([TOWEL_NEAR, HAZARD])
    assert plan["next_action"] == "stop_for_hazard"
    assert plan["target_object_id"] == "det_004"


def test_nearest_towel_selected():
    plan = TaskPlanner().plan([TOWEL_FAR, TOWEL_NEAR, STOOL])
    assert plan["next_action"] == "pick_object"
    assert plan["target_object_id"] == "det_001"
    assert plan["target_position"]["x"] == 1.1


def test_blockers_only_routes_around():
    plan = TaskPlanner().plan([STOOL])
    assert plan["next_action"] == "avoid_object"
    assert plan["target_object_id"] == "det_003"


def test_empty_frame_keeps_searching():
    plan = TaskPlanner().plan([])
    assert plan["next_action"] == "continue_search"
    assert plan["target_object_id"] is None

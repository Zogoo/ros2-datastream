"""Rule-based task planning over detections. Pure logic, no ROS imports.

Priority: safety stop > nearest pickable > avoid hard obstacle > keep searching.
The LLM client (llm_client.py) can wrap this planner's output with higher-level
reasoning; the topic contract on /task_plan stays the same either way.
"""
from __future__ import annotations

from typing import Any

TASK = "collect_onsen_floor_towels"

NEXT_ACTION_MAP = {
    "towel": "pick_object",
    "bucket": "avoid_object",
    "stool": "avoid_object",
    "bottle": "avoid_object",
}


class TaskPlanner:
    def plan(self, detections: list[dict[str, Any]], safety_stop: bool = False) -> dict[str, Any]:
        if safety_stop:
            return {
                "task": TASK,
                "next_action": "emergency_stop",
                "target_object_id": None,
                "reason": "safety stop latched — awaiting operator reset",
            }

        stoppers = [d for d in detections if d.get("risk") == "stop"]
        if stoppers:
            return {
                "task": TASK,
                "next_action": "stop_for_hazard",
                "target_object_id": stoppers[0]["id"],
                "reason": f"{stoppers[0]['class']} requires an immediate stop",
            }

        pickable = [
            d for d in detections
            if d.get("pickable") and d.get("estimated_position") is not None
        ]
        if pickable:
            target = min(pickable, key=lambda d: d["estimated_position"]["x"])
            return {
                "task": TASK,
                "next_action": NEXT_ACTION_MAP.get(target["class"], "navigate_to_object"),
                "target_object_id": target["id"],
                "target_class": target["class"],
                "target_position": target["estimated_position"],
                "reason": (
                    f"nearest pickable {target['class']} at "
                    f"{target['estimated_position']['x']}m (conf {target['confidence']})"
                ),
            }

        blockers = [d for d in detections if not d.get("pickable")]
        if blockers:
            nearest = min(
                blockers,
                key=lambda d: (d.get("estimated_position") or {"x": 99})["x"],
            )
            return {
                "task": TASK,
                "next_action": "avoid_object",
                "target_object_id": nearest["id"],
                "target_class": nearest["class"],
                "reason": f"non-pickable {nearest['class']} in view — route around",
            }

        return {
            "task": TASK,
            "next_action": "continue_search",
            "target_object_id": None,
            "reason": "no objects detected in current frame",
        }

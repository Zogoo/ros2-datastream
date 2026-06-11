# Onsen Robot Simulator v2 — Status

All plan items from `onsen_robot_simulator_v2_bf641f6d.plan.md` are implemented.
This file previously tracked the v2 gap audit; everything in it is resolved.

## Done (this iteration)

- **Compose/Docker**: v2 service set (`base_controller`, `arm_controller`,
  `control_arbitrator`, `robot_state`, `ai_worker`, `mission_executor`,
  `dummy_robot`, bridges, record/play/e2e profiles); root Dockerfile builds all
  three packages + bundles `shared/`; FE build context fixed to repo root
- **topics.py**: Python topic registry mirroring `frontend/src/ros/topics.js`
- **Gripper physics**: fixed-joint grasp on the still-dynamic towel,
  carried-load weight transferred to the chassis, release with fingertip velocity
- **Skins**: per-instance skins + bundled presets (RYOKAN/STRIPED/CHARCOAL) +
  detection-eval notebook (`03_skin_detection_eval.ipynb`, recall 0.87→0.00→0.95)
- **Safety**: arm DROP-sector scan self-filter; `warning()` logger fix
  (`.warn` crashes nodes on this rclpy — it killed the aggregator mid-e-stop);
  e-stop latching is now **opt-in** (FE `E-STOP` toggle -> `/safety/enable`,
  default disarmed, `SAFETY_ENABLED=true` to arm at boot)
- **Depth camera**: 320×240 per plan
- **Tests**: 70 pytest (safety latching/self-filter, mission FSM, planner,
  detection/skins, LLM client, protocol transcripts) + 22 vitest + ruff + mypy,
  all via `make check`
- **E2E**: 9-scenario Playwright suite (compose profile `e2e`), with a
  `window.__sim` staging hook
- **Docs**: README + architecture/topics/robot_design/ai_worker_guide/
  robotics_worker_guide rewritten; ai_knowledge.md refreshed

## Notable fixes found during verification

- Console-script wrappers needed the executable bit (colcon installs them as-is;
  `ros2 run` reports "No executable found")
- `ObjectManager.spawn` generated ids colliding with layout prop ids
  (`towel_1`…), corrupting `/ground_truth/objects` lookups
- Chassis collider friction lowered to 0.05 (smooth skid plate) so the nose
  slides up step corners instead of wedging — stair climbing now works
- Loungers rearranged on the resting deck so a clear lane onto the deck exists
- nginx proxy pinned to `127.0.0.1:5000` (with `localhost`, nginx round-robins
  onto `::1`, which OrbStack forwards to macOS AirPlay → intermittent 403)

## Deliberate deviations from the plan (documented in docs/architecture.md)

- Mission navigation uses `/ground_truth/objects` with detection confirmation
  (the detection-only seam is the documented AI-engineer exercise)
- Synthetic headless camera stays procedural (bag replay covers photo-real work)
- Custom raycast suspension instead of Rapier's vehicle controller (equivalent
  behavior, simpler to reason about)

# Onsen Robot — AI Agent Knowledge Base

> Handoff document for AI agents working on this repository.
> Current generation: **Simulator v2** (physics-true browser sim + ROS2 control stack).

## 1. What this is

The browser frontend (Vite + Three.js + Rapier, `frontend/src/`) is the
ground-truth physics world: onsen environment, 6-wheel sprung robot, 6-axis
arm. All sensors are derived from physics/render state, noise-degraded, and
published over rosbridge. ROS2 nodes are firmware/safety/perception/autonomy —
the same shape as a real robot. Full rationale: `docs/architecture.md`.

**v1 is gone.** No `camera_mux`, no `app.js`, no Python-generated camera as
the primary path. If you see references to them, they're stale.

## 2. Run + verify

```bash
docker compose up            # full stack; FE at http://localhost:8080 (ONE tab)
make check                   # ruff + mypy + pytest (in Docker) + eslint + vitest
docker compose --profile e2e up --abort-on-container-exit   # 9-scenario Playwright suite
```

`SIM_SOURCE=fe|synthetic|replay` selects the world source (`docs/robotics_worker_guide.md`).

## 3. File map

| Area | Where |
|---|---|
| FE world/physics | `frontend/src/{physics,env}/` |
| FE robot + arm (fixed-joint grasp, carried mass) | `frontend/src/robot/` |
| FE sensors | `frontend/src/sensors/` |
| FE topic registry | `frontend/src/ros/topics.js` (mirrors `onsen_robot_state/topics.py`) |
| E2E debug hook | `window.__sim` (set in `frontend/src/main.js`) |
| Arm/base firmware (pure logic) | `src/onsen_dummy_robot/onsen_dummy_robot/{arm,base}_protocol.py` |
| Firmware ROS wrappers | `{arm,base}_controller_node.py`, `control_arbitrator_node.py` |
| Headless sim | `dummy_stream_node.py` (`SIM_SOURCE`), `layout.py`, `scene_generator.py` |
| Safety + fusion | `src/onsen_robot_state/onsen_robot_state/{safety.py, robot_state_aggregator_node.py}` |
| AI worker | `src/onsen_ai_worker/onsen_ai_worker/{detection,planner,llm_client,mission,http_api}.py` |
| Autonomy node | `mission_executor_node.py` |
| Notebooks | `src/onsen_ai_worker/notebooks/01..03` |
| Shared specs (single source of truth) | `shared/{onsen_layout,robot_spec,object_profiles}.json` |
| Topic contract | `docs/topics.md` |
| E2E suite | `frontend/e2e/` (Playwright, compose profile `e2e`) |

## 4. Contracts you must not break

- Topic names: change only in `topics.py` + `topics.js` (mirrored registries)
- `/detected_objects`, `/task_plan` JSON schemas (FE HUD + executor + tests)
- Firmware reply strings (`OK …` / `ERR LIMIT joint=0 value=295` …) — the
  brief's transcript is replayed verbatim in tests and e2e
- `shared/*.json` schemas — FE vitest validates them; geometry consumed by
  both FE and Python

## 5. Known failure mode: "FE connected but frozen"

ROS2 DDS discovery failure between containers — rosbridge serves the browser
but receives nothing from other containers.

Triggers: removing `FASTDDS_BUILTIN_TRANSPORTS: UDPv4`; adding
`use_events_executor:=true` to rosbridge; bridge networking without DDS tuning.

```bash
docker compose exec base_controller /entrypoint.sh ros2 topic hz /odom   # backend publishing?
docker compose exec rosbridge env | grep FASTDDS                        # env present?
docker compose down && docker compose up -d --build                     # then hard-refresh ONE tab
```

## 6. Do-not-change list

| Action | Consequence |
|---|---|
| Remove `FASTDDS_BUILTIN_TRANSPORTS: UDPv4` | FE connected but no data |
| Add `use_events_executor:=true` to rosbridge | unstable WebSocket forwarding |
| FE publishing `/cmd_vel` directly | bypasses arbitrator, breaks manual/auto |
| Multiple browser tabs | two physics worlds double-publishing |
| FE build context `./frontend` in compose | build breaks — Dockerfile expects repo root (`shared/` is bundled) |
| Dropping `onsen_robot_state` from the root Dockerfile colcon build | safety node image missing |

## 7. Quality gates (all must pass before claiming done)

1. `make check-py` — ruff, mypy, 70+ pytest (protocol transcripts, safety
   latching + scan self-filter, mission FSM, planner, detection/skins, LLM client)
2. `make check-fe` — eslint + 22 vitest (FK pose validation, noise models, spec schemas)
3. `docker compose build` — both images
4. e2e profile for integration-level claims

## 8. Extension seams

- Detection: swap `Detector.detect()` (ONNX) — schema stays
- LLM: `LLM_BASE_URL`/`LLM_API_KEY`/`LLM_MODEL` (OpenAI-compatible), mock when unset
- Navigation: publish `/cmd_vel/auto`; detection-only nav = replace
  `MissionInput.towels` in `mission_executor_node.py`
- New FE sensor: implement `update(dt)` + publish, register in `main.js` loop

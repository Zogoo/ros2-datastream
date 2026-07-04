# Onsen Robot Simulator v2

A physics-true browser simulator of a towel-collecting onsen cleaning robot,
wired into a full ROS2 control stack.

The frontend (Three.js + Rapier) is the **ground-truth world**: a Japanese onsen
built to scale from the floor plan, a 6-wheel independently-sprung robot with a
6-axis arm, and every sensor (LIDAR, RGB/depth cameras, sonar, IMU, encoders,
bumpers) derived from real raycasts and render passes — then degraded through
documented noise models and published to ROS2. The ROS2 side runs what a real
robot runs: firmware emulators, a control arbitrator, a safety aggregator, and
an LLM-ready AI worker that closes the autonomy loop.

```
browser sim (ground truth)  <-- ws://9090 -->  ROS2 (firmware / safety / AI)
```

## Quick start

```bash
docker compose up
```

Then open **http://localhost:8080** (single tab — the tab *is* the simulator).

| Service | Role | Address |
|---|---|---|
| `frontend` | Physics sim + control UI | http://localhost:8080 |
| `rosbridge` | FE <-> ROS2 WebSocket | ws://localhost:9090 |
| `foxglove_bridge` | Foxglove Studio | ws://localhost:8765 |
| `ai_worker` | Detection + depth-ranging + planning + HTTP API | http://localhost:5000 |
| `towel_tracker` | Map-frame towel tracks from detections | — |
| `mission_executor` | Autonomy loop: Nav2-style goals + analytic-IK pick (AUTO mode) | — |
| `nav_server` | A* planner + regulated pure-pursuit tracker (`/nav/goal`) | — |
| `localizer` | Likelihood-field scan matcher (`map->odom`) | — |
| `nav_static_tf` | Static TF tree from robot_spec | — |
| `depth_scan` | Low-obstacle `/scan_low` from the depth camera | — |
| `arm_description` | URDF + coupled joint bridge + robot_state_publisher | — |
| `base_controller` | Wheel firmware (`/base/command`) | — |
| `arm_controller` | Arm firmware (`/arm/command`) | — |
| `control_arbitrator` | Manual/auto `/cmd_vel` owner | — |
| `robot_state` | Safety e-stop + `/robot/state` fusion | — |
| `dummy_robot` | Headless sim source (`SIM_SOURCE`) | — |

`docker compose --profile eval up` adds the `eval` node (the only ground-truth
consumer: scores localization + tracking on `/eval/metrics`).

Rebuild after code changes: `docker compose up --build`.

## Autonomy

In **AUTO** mode the robot is fully autonomous with ground truth out of the
control path: the **localizer** (likelihood-field scan matching against a map
rasterized from the layout) provides pose, **perception** ranges towels with
the depth camera and the **tracker** turns detections into stable map-frame
targets, the **nav server** (A* + regulated pure pursuit) plans and drives to
each waypoint, and the **arm** reaches the *measured* towel pose with a
closed-form analytic IK solver (verified against the simulator's FK to machine
precision). Design rationale, paper/plugin comparisons, and the IK derivation
are in [docs/research_notes.md](docs/research_notes.md).

## Controls

| Input | Action |
|---|---|
| `W/A/S/D`, arrows, D-pad, gamepad | Drive (switches to MANUAL automatically) |
| `Space` | Stop |
| `T` / THROW TOWEL | Toss a towel from a random direction |
| `1–9` | Arm poses (HOME … DROP_RELEASE) |
| AUTO / MANUAL | Control arbitration mode |
| E-STOP: OFF/ARMED | Arm the safety latch (optional, off by default; armed = hard impacts/tilt/water latch `/safety/stop`) |
| ORBIT / FOLLOW / FPS | Camera views |
| DRAG | Pick up and fling any movable object with the mouse |
| SKINS | Re-texture object classes/instances, sync AI detection profiles |
| CONSOLE | Raw firmware console (`Q`, `A HOME`, `J …` to arm; `V`, `W …` to base) |
| AI UPLOAD | Send the current camera frame to the AI worker |

**AUTO mode** hands the base to `mission_executor`: it searches, approaches the
nearest towel, runs the firmware pick sequence, drives to the towel bin and
drops — scored on `/robot/events` (`OBJECT_BINNED`, `correct: true`).

## Quality gates

```bash
make check        # everything below
make check-py     # ruff + mypy + pytest (runs inside the ROS image)
make check-fe     # eslint + vitest
make e2e          # Playwright browser suite against the live stack
```

The e2e suite (`docker compose --profile e2e up`) covers the nine acceptance
scenarios: boot, manual drive, stair climbing, the full towel mission, the
firmware console transcript, safety e-stop + recovery, sensor stream health,
skin upload/profile sync, and water hazards.

## Headless modes (no browser)

`SIM_SOURCE` on the `dummy_robot` service picks the data source:

| Mode | Behavior |
|---|---|
| `fe` (default) | Browser is ground truth; `dummy_robot` stands by |
| `synthetic` | Python 2.5D sim raycasts the same `shared/onsen_layout.json`; whole autonomy stack runs unchanged |
| `replay` | rosbag playback owns the topics |

```bash
SIM_SOURCE=synthetic docker compose up                 # headless world
docker compose --profile record up                     # record ./bags/onsen_run
SIM_SOURCE=replay docker compose --profile play up     # loop a recorded run
```

## Verify topics are flowing

```bash
docker compose exec base_controller /entrypoint.sh ros2 topic list
docker compose exec base_controller /entrypoint.sh ros2 topic echo /odom --once
docker compose exec base_controller /entrypoint.sh ros2 topic hz /scan
docker compose exec arm_controller  /entrypoint.sh ros2 topic pub --once /arm/command std_msgs/msg/String "{data: Q}"
docker compose exec arm_controller  /entrypoint.sh ros2 topic echo /arm/response
```

Full topic contract: [docs/topics.md](docs/topics.md). Expected healthy rates:
`/odom` 20 Hz, `/scan` 8 Hz, `/imu` 50 Hz, cameras ~5 Hz, `/joint_states` 20 Hz.

## Environment variables

| Variable | Default | Effect |
|---|---|---|
| `ROS_DOMAIN_ID` | 42 | DDS domain (must match across containers) |
| `FASTDDS_BUILTIN_TRANSPORTS` | `UDPv4` | **Do not remove** — cross-container DDS discovery |
| `SIM_SOURCE` | `fe` | `fe` / `synthetic` / `replay` |
| `REALISM_PROFILE` | `low` | Sensor degradation: `low` / `medium` / `high` |
| `LLM_BASE_URL` / `LLM_API_KEY` / `LLM_MODEL` | unset | OpenAI-compatible endpoint for the planner; deterministic mock when unset |
| `SAVE_DATASET` | `false` | Synthetic mode dataset dump |

## Documentation

| Doc | Audience |
|---|---|
| [docs/architecture.md](docs/architecture.md) | Everyone — system shape, data flow, design decisions |
| [docs/topics.md](docs/topics.md) | Everyone — the authoritative topic contract |
| [docs/robot_design.md](docs/robot_design.md) | Robotics engineers — chassis/suspension/sensor placement rationale |
| [docs/ai_worker_guide.md](docs/ai_worker_guide.md) | Data scientists — topics, bags, notebooks, LLM config, skin pipeline |
| [docs/robotics_worker_guide.md](docs/robotics_worker_guide.md) | Robotics engineers — firmware protocols, safety aggregator, headless modes |
| [docs/ai_knowledge.md](docs/ai_knowledge.md) | AI agents — handoff knowledge base |

## Known invariants (do not change)

- `FASTDDS_BUILTIN_TRANSPORTS: UDPv4` in every ROS container (rosbridge loses
  cross-container discovery without it)
- No `use_events_executor:=true` on rosbridge (unstable forwarding)
- FE publishes `/cmd_vel/ui`, never `/cmd_vel` (the arbitrator owns it)
- One browser tab at a time (two tabs = two physics worlds double-publishing)

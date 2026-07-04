# Architecture

## The inversion that defines v2

v1 generated fake sensor data in Python and used the browser as a viewer.
v2 inverts this: **the browser frontend is the ground-truth world** — a Rapier
rigid-body simulation of the onsen and the robot — and every sensor stream is
*derived* from that world (raycasts, render passes, rigid-body state), degraded
through explicit noise models, then published to ROS2. The ROS2 nodes are
exactly what runs on a real robot: firmware, arbitration, safety, perception,
autonomy.

```
┌────────────────────── browser (Vite + Three.js + Rapier) ──────────────────────┐
│ physics world: onsen layout, movable weighted objects, water buoyancy, steam   │
│ robot: 6-wheel raycast suspension chassis, racked decks, 6-axis arm + gripper  │
│ sensors: lidar (raycast), front/rear RGB (render), depth (depth buffer),       │
│          sonar (cone cast), imu (body state), odom (encoder integration),      │
│          contacts (collision events), ground truth                            │
│ ui: D-pad/WASD/gamepad, arm firmware console, skins, scenarios, views          │
└──────────────────────────────┬─────────────────────────────────────────────────┘
                       rosbridge ws://9090
┌──────────────────────────────┴─────────────────────────────────────────────────┐
│ control_arbitrator   /cmd_vel/ui + /cmd_vel/auto + mode -> /cmd_vel            │
│ base_controller      wheel firmware: /base/command, /cmd_vel -> wheel targets  │
│ arm_controller       serial firmware: /arm/command -> joint targets           │
│ robot_state          safety fusion: contacts/imu/scan -> /safety/stop latch    │
│ nav_static_tf        /tf_static tree from robot_spec                           │
│ localizer            likelihood-field scan match -> map->odom, /localization   │
│ depth_scan           depth camera -> /scan_low (low obstacles)                 │
│ nav_server           A* + regulated pure pursuit: /nav/goal -> /cmd_vel/auto   │
│ ai_worker            detection + depth ranging + planner -> /detected_objects  │
│ towel_tracker        detections -> /perception/towel_tracks (map frame)        │
│ arm_description      URDF + coupled joints -> robot_state_publisher TF         │
│ mission_executor     autonomy FSM -> /nav/goal + analytic-IK /arm/command      │
│ dummy_robot          SIM_SOURCE=fe|synthetic|replay headless source           │
│ eval (profile)       ONLY ground-truth consumer -> /eval/metrics               │
│ rosbridge / foxglove_bridge / rosbag record + play                             │
└────────────────────────────────────────────────────────────────────────────────┘
```

## Autonomy stack (AUTO mode, ground truth out of the control path)

```
/scan ─────────────► localizer ──► map->odom TF ──► /localization/pose ─┐
layout map (A* grid)                                                     ├─► mission_executor (FSM)
/camera/* ─► ai_worker (HSV + depth range + height gate) ─► /detected_objects   │     │
                                       └─► towel_tracker ─► /perception/towel_tracks ┘     │
                                                                          nav goals │     │ IK reach
                                                                                    ▼     ▼
                                                          nav_server (A* + pure pursuit)  arm_controller
                                                                 │                            │
                                                                 └──► /cmd_vel/auto ──► arbitrator ──► base
```

- **Localization**: likelihood-field scan matching against a map rasterized
  deterministically from `shared/onsen_layout.json`. Corrects the drifting
  `/odom` into a `map->odom` transform. (AMCL's measurement model, hill-climb
  instead of a particle filter — the start pose is known.)
- **Perception**: the HSV detector's bbox + the depth image give a metric towel
  range (back-projection through the depth intrinsics); a height gate uses that
  range to separate flat towels from stools/buckets that share the warm palette.
  The tracker keeps only depth-confirmed towels as stable map-frame targets.
- **Navigation**: A* on the inflated occupancy grid (pools are lethal keepout) +
  regulated pure pursuit, exposed as a NavigateToPose-shaped goal/status seam.
- **Manipulation**: a closed-form analytic IK solver (pan + isosceles planar 2R
  + wrist, exact for the servo coupling) reaches the *measured* towel pose; the
  gripper closes in place. Validated against the FE FK to machine precision.

Why in-house rather than Nav2/MoveIt: neither is packaged for the `lyrical`
distro (verified in Phase 0). The algorithms here are the published methods
those frameworks implement, behind seam-compatible interfaces — see
`docs/research_notes.md` for the comparison and the IK derivation.

## Control path (one owner per topic)

1. UI or gamepad -> `/cmd_vel/ui`; mission executor -> `/cmd_vel/auto`
2. `control_arbitrator` owns `/cmd_vel` (manual wins when fresh; mode on
   `/robot/control_mode`)
3. `base_controller` (firmware emulation) converts `/cmd_vel` or raw
   `/base/command` protocol lines into per-wheel velocities on
   `/base/wheel_targets` — and obeys `/safety/stop` unconditionally
4. The FE drivetrain consumes wheel targets; physics produces motion; encoders
   integrate `/odom` *from wheel rotation, not ground truth*, so skid-steer
   drift is real

The arm is identical in shape: `/arm/command` (serial protocol) ->
`arm_controller` -> 20 Hz interpolated `/arm/joint_targets` -> FE servo lag ->
`/joint_states` reports the *measured* (lagging) joints.

## Safety path

The FE publishes real collision events (`/robot/contacts`, with impulse and
part names). `robot_state` (package `onsen_robot_state`) latches
`/safety/stop` on hard impact, water ingress (critical contact) or tilt; the
base firmware zeroes wheels immediately; the mission executor aborts to IDLE.
Reset is operator-driven (`/safety/reset`) and refused while a hazard is still
active. While the arm reports a DROP phase, the aggregator self-filters the
scan sector the arm sweeps through (70–90°) — the same self-filtering a real
robot does.

## Single sources of truth

| File | Consumed by |
|---|---|
| `shared/onsen_layout.json` | FE world builder, Python synthetic mode, mission executor (bin positions) |
| `shared/robot_spec.json` | FE robot builder + sensor sims, safety thresholds, detection camera model, docs |
| `shared/object_profiles.json` | FE object spawner + physics, skin defaults |
| `onsen_robot_state/topics.py` / `frontend/src/ros/topics.js` | every node / every FE module |

A broken layout fails schema tests, not runtime.

## Key trade-offs (deliberate, documented)

- **Towels are rigid low-profile boxes**, not cloth: grasp/flatten/float
  behavior is preserved at a fraction of the complexity (KISS)
- **Skins are visual-only**: physics stays bound to the class so a re-skinned
  towel still grasps and floats — only *perception* is challenged
- **Mission navigation uses `/ground_truth/objects` for towel world positions**
  while camera detections gate confirmation. Detection-only navigation is the
  documented exercise seam (`MissionInput.towels`) for AI engineers
- **Synthetic headless camera stays procedural** (OpenCV scene): layout-true
  raycast rendering in Python would duplicate the FE renderer for marginal
  benefit; bag replay covers photo-real headless work
- **No nav stack**: bearing-pursuit navigation keeps the autonomy loop
  readable; swapping in Nav2 via `/cmd_vel/auto` is an extension point

# Topic contract (authoritative)

Names live in exactly two mirrored registries:
`src/onsen_robot_state/onsen_robot_state/topics.py` (Python) and
`frontend/src/ros/topics.js` (FE). Change them there or nowhere.

> In containers, prefix `ros2` with `/entrypoint.sh`:
> `docker compose exec base_controller /entrypoint.sh ros2 topic list`

## Sensors (FE simulator -> ROS)

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 8 Hz | 360 beams batched across physics ticks; material/steam dropout; real self-hits during arm DROP |
| `/camera/front/image_raw/compressed` | `sensor_msgs/CompressedImage` | ~5 Hz | 640×480 JPEG, pitch −15° |
| `/camera/front/camera_info` | `sensor_msgs/CameraInfo` | ~5 Hz | intrinsics derived from the actual projection |
| `/camera/rear/image_raw/compressed` + `camera_info` | — | ~5 Hz | reversing coverage, pitch −10° |
| `/camera/depth/image_raw` | `sensor_msgs/Image` 16UC1 (mm) | ~5 Hz | 320×240, σ ∝ z², invalid at grazing/specular |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | ~5 Hz | |
| `/sonar/range_0..2` | `sensor_msgs/Range` | 15 Hz | bearings −25/0/+25°, cone min-hit, steam-immune |
| `/imu` | `sensor_msgs/Imu` | 50 Hz | at CoG; suspension oscillation visible; bias random-walk |
| `/odom` + `/tf` | `nav_msgs/Odometry` | 20 Hz | encoder-integrated — drifts under slip (by design) |
| `/joint_states` | `sensor_msgs/JointState` | 20 Hz | measured (lagging) arm joints + 6 wheels |
| `/robot/contacts` | `std_msgs/String` JSON | event | `{part, impulse, normal, object_id, object_class, critical}` |
| `/ground_truth/pose` | `geometry_msgs/PoseStamped` | 10 Hz | for drift quantification |
| `/ground_truth/objects` | `std_msgs/String` JSON | 5 Hz | true object states for labeling/eval |
| `/sim/status` | `std_msgs/String` JSON | 1 Hz | `{alive, session_id, sim_time, fps}` heartbeat; `session_id` change = FE restart (see SessionWatch) |
| `/robot/held_object` | `std_msgs/String` JSON | on change | `{held, object_id, object_class, position}` — gripper payload sensor |
| `/robot/bin_load` | `std_msgs/String` JSON | 2 Hz | `{kg, count, tilt_deg}` — collect-bin load cell (strain gauge + HX711); base firmware thresholds into `bin_full` |

## Control

| Topic | Type | Producer -> Consumer |
|---|---|---|
| `/cmd_vel/ui` | `geometry_msgs/Twist` | FE -> arbitrator |
| `/cmd_vel/auto` | `geometry_msgs/Twist` | mission_executor -> arbitrator |
| `/robot/control_mode/set` | `std_msgs/String` | FE -> arbitrator (`auto`/`manual`) |
| `/cmd_vel` | `geometry_msgs/Twist` | arbitrator -> base_controller (canonical) |
| `/robot/control_mode` | `std_msgs/String` JSON | arbitrator -> all (`{mode, active_source, ui_fresh, vx, wz}`) |

## Base firmware (`base_controller`)

| Topic | Direction | Payload |
|---|---|---|
| `/base/command` | in | protocol line: `Q`, `V vx wz`, `T vl vr`, `W id vel`, `SPEED pct`, `STOP`, `RESET_ERROR` |
| `/base/response` | out | protocol replies (`OK …` / `ERR …`) |
| `/base/wheel_targets` | out 20 Hz | `{"w": [rad/s ×6], "ts"}` — consumed by the FE drivetrain |
| `/base/state` | out 10 Hz | `{status, vx, wz, wheels, …}` |

## Arm firmware (`arm_controller`)

| Topic | Direction | Payload |
|---|---|---|
| `/arm/command` | in | serial protocol: `Q`, `A <ACTION>`, `J d0..d5 ms`, `D j Δ ms`, `M ALIAS amt ms`, `G pos ms`, `SPEED`, `STOP`, `CAL …`, `RELAX/WAKE` |
| `/arm/response` | out | firmware replies, e.g. `STATE 90 90 90 90 90 70 IDLE`, `ERR LIMIT joint=0 value=295` |
| `/arm/joint_targets` | out 20 Hz | `{"deg": [×6], "status", "ts"}` — FE servo targets |
| `/arm/state` | out 10 Hz | `{joints_deg, status, speed_pct, relaxed, last_action, queue_depth}` |

## Safety + fused state (`robot_state`)

| Topic | Direction | Payload |
|---|---|---|
| `/safety/stop` | out 10 Hz | `std_msgs/Bool` latched e-stop (always `false` while disarmed) |
| `/safety/reset` | in | `std_msgs/Bool` operator reset (refused while hazard active) |
| `/safety/enable` | in | `std_msgs/Bool` arm/disarm the e-stop latch (FE `E-STOP` toggle; default disarmed, `SAFETY_ENABLED=true` to arm at boot); disarming clears latches |
| `/robot/state` | out 5 Hz | fused JSON: `{safety_enabled, safety_stop, safety_critical, tilt_deg, min_obstacle_m, arm_scan_filter, odom, arm, base, last_contact}` |
| `/robot/events` | out | safety + sim events (`SAFETY_IMPACT`, `OBJECT_BINNED`, `TOWEL_THROWN`, `GRASP_ACQUIRED`, `GRASP_RELEASED`, …) |

`GRASP_ACQUIRED`/`GRASP_RELEASED` are emitted by the FE gripper (payload-sensor
emulation) when the grasp joint forms/breaks — the mission's `holding` signal,
not ground truth.

## Navigation (`onsen_nav`: static TF, localizer, depth-scan, nav server)

| Topic | Direction | Payload |
|---|---|---|
| `/tf_static` | out once | base_link -> laser/cameras/imu/sonar/arm_base, from robot_spec |
| `/scan_low` | out ~5 Hz | `sensor_msgs/LaserScan` low-obstacle layer from the depth camera (bath rims/stools/towels below the lidar plane), base_link frame |
| `/localization/pose` | out 20 Hz | `PoseWithCovarianceStamped` AMCL-style; `covariance[0]` = scan-match score; publishes `map->odom` TF |
| `/map` | out latched | `nav_msgs/OccupancyGrid` rasterized from the layout (walls + furniture + pool keepout) |
| `/nav/goal` | in | JSON `{goal_id, x, y, yaw?}` — NavigateToPose-equivalent |
| `/nav/cancel` | in | JSON `{}` cancels the active goal |
| `/nav/status` | out 5 Hz | JSON `{goal_id, state, distance_remaining}`; state `idle|active|succeeded|failed|cancelled` |
| `/nav/path` | out | `nav_msgs/Path` the A* + shortcut waypoint polyline (map frame) |

The nav server drives `/cmd_vel/auto` (the arbitrator still owns `/cmd_vel`;
manual wins). See `docs/research_notes.md` for why this is in-house A* +
regulated pure pursuit + likelihood-field matching rather than Nav2 (not
packaged for this distro) — same published methods, seam-compatible interface.

## Arm description (`onsen_arm`)

| Topic | Direction | Payload |
|---|---|---|
| `/robot_description` | out latched | URDF generated from robot_spec (move_group would consume it; here it feeds robot_state_publisher) |
| `/joint_states_urdf` | out 20 Hz | FE servo-offset joints re-published with the 1.5 coupling applied, for robot_state_publisher's arm TF |

## Perception + autonomy (`ai_worker`, `towel_tracker`, `mission_executor`)

| Topic | Producer | Payload |
|---|---|---|
| `/detected_objects` | ai_worker | `{…, objects[{…, estimated_position, position_refined, range, source: "depth"\|"ground_plane", estimated_height}]}` (depth-refined range + height gate) |
| `/task_plan` | ai_worker | `{task, next_action, target_object_id, reason}` |
| `/perception/towel_tracks` | towel_tracker | `{tracks[{id, class, position{x,y,z}, hits, source, range}]}` map-frame depth-confirmed towel tracks |
| `/mission/state` | mission_executor | `{state, reason, target_id, holding, towels_remaining, pose_source, nav_status, llm, llm_reason}` |
| `/eval/metrics` | eval (profile `eval`) | `{loc_pos_err_m, loc_yaw_err_deg, track_mean_err_m, …}` — the only ground-truth consumer |

Mission states: `IDLE -> SEARCH -> APPROACH -> ALIGN_PICK -> PICK -> TO_BIN -> ALIGN_BIN -> DROP -> SEARCH`.
APPROACH/TO_BIN delegate locomotion to `/nav/goal`; PICK reaches the *measured*
towel pose via analytic IK (`J` command), not a canned pose. Targets come from
`/perception/towel_tracks` and pose from `/localization/pose` — ground truth is
eval-only (`MISSION_USE_GT=true` restores the GT path for A/B comparison).

## HTTP API (ai_worker :5000, proxied at FE `/api/`)

| Endpoint | Effect |
|---|---|
| `POST /upload` | run detection on an image; publishes results |
| `GET /profiles` | current per-class HSV detection bands |
| `POST /profiles/<class>` | resample the band from an uploaded skin image |
| `DELETE /profiles/<class>` | restore the default band |

## Expected rates (healthy stack)

| Topic | Hz |
|---|---|
| `/odom`, `/joint_states` | 20 |
| `/imu` | 50 |
| `/sonar/range_*` | 15 |
| `/scan` | 8 |
| cameras (front/rear/depth) | ~5 each |
| `/detected_objects` | tied to camera |
| `/robot/state` | 5 |
| `/safety/stop` | 10 |

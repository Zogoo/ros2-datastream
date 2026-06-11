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
| `/sim/status` | `std_msgs/String` JSON | 1 Hz | `{alive, sim_time, fps}` heartbeat (detects throttled tabs) |

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
| `/safety/stop` | out 10 Hz | `std_msgs/Bool` latched e-stop |
| `/safety/reset` | in | `std_msgs/Bool` operator reset (refused while hazard active) |
| `/robot/state` | out 5 Hz | fused JSON: `{safety_stop, safety_critical, tilt_deg, min_obstacle_m, arm_scan_filter, odom, arm, base, last_contact}` |
| `/robot/events` | out | safety + sim events (`SAFETY_IMPACT`, `OBJECT_BINNED`, `TOWEL_THROWN`, …) |

## Perception + autonomy (`ai_worker`, `mission_executor`)

| Topic | Producer | Payload |
|---|---|---|
| `/detected_objects` | ai_worker | `{timestamp, frame_id, objects[{id, class, confidence, bbox, robot_class, pickable, risk, estimated_position}]}` |
| `/task_plan` | ai_worker | `{task, next_action, target_object_id, reason}` |
| `/mission/state` | mission_executor | `{state, reason, target_id, holding, towels_remaining, pose_source, llm, llm_reason}` |

Mission states: `IDLE -> SEARCH -> APPROACH -> PICK -> TO_BIN -> ALIGN_BIN -> DROP -> SEARCH`.

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

# Onsen Robot — ROS2 Data Stream

A ROS2 Lyrical Docker Compose environment that streams realistic synthetic sensor data
from a small tank-drive cleaning robot in a Japanese onsen/bathroom setting.

**Not a physics simulator.** No Gazebo, no robot model.  
All data is generated in Python (NumPy + OpenCV).  
Designed for: data pipeline development, AI model integration, LLM planner research.

---

## Quick start

```bash
docker compose up
```

First run builds two images (`onsen-ros:latest`, `onsen-frontend:latest`) then starts all services.
Subsequent runs reuse the cache and start in seconds.
Use `docker compose up --build` after changing source files or the Dockerfile.

---

## Autonomy upgrade notes

- Camera path is now source-aware:
  - Dummy camera publishes to `/camera/source/dummy/*`
  - FPS camera (from frontend) publishes to `/camera/source/fps/*` when FPS view is active
  - `camera_mux_node` republishes the active source to canonical topics:
    - `/camera/front/image_raw`
    - `/camera/front/image_raw/compressed`
    - `/camera/front/camera_info`
- Base motion path is now arbitrated:
  - Frontend publishes manual commands to `/cmd_vel/ui`
  - `control_arbitrator_node` owns canonical `/cmd_vel` and mode state
  - Mode command topic: `/robot/control_mode/set` (`auto` or `manual`)
- Low-spec defaults remain lightweight:
  - `REALISM_PROFILE=low` by default
  - `SAVE_DATASET=false` by default

---

## Services and ports

| Service | What it does | Address |
|---|---|---|
| `dummy_robot` | Publishes all synthetic sensor topics + accepts control input | — |
| `camera_mux` | Selects dummy/FPS camera source and republishes canonical camera topics | — |
| `control_arbitrator` | Owns auto/manual mode and routes manual control to canonical `/cmd_vel` | — |
| `ai_worker` | Subscribes to camera, detects objects, publishes AI topics + HTTP upload | `localhost:5000` |
| `rosbridge` | WebSocket bridge for the frontend | `ws://localhost:9090` |
| `foxglove_bridge` | Foxglove Studio WebSocket | `ws://localhost:8765` |
| `frontend` | 3D Android-camera-style control UI | `http://localhost:8080` |

---

## Web UI — primary interface

Open **http://localhost:8080** after `docker compose up`.

```
┌─────────────────────────────────────────────────────────┐
│ ● CONNECTED   AUTO   ONSEN ROBOT   12:34:56   ◉ LIVE   │  ← status bar
├─────────────────────────────────────────────────────────┤
│                                                         │
│   ARM STATE        [3D viewfinder]       DETECTIONS     │
│   HOME                                  2 objects       │
│   CYCLE: 0    [tank robot + 6DOF arm]   towel 86%       │
│   PROB: 1.00         orbits/follows      slipper 71%    │
│                                                         │
│            [▲]                                          │
│         [◄][■][►]    lidar ○    cam □                   │
│            [▼]                                          │
├─────────────────────────────────────────────────────────┤
│  ORBIT  FOLLOW  FPS   ◉   ⬆ UPLOAD   SEARCH / reason   │
├─────────────────────────────────────────────────────────┤
│  ARM:  HOME  SRCH  APPR  LOWR  GRIP  LIFT  DROP  JOINTS▾│
└─────────────────────────────────────────────────────────┘
```

### Robot control

| Input | Action |
|---|---|
| `W / ↑` | Forward |
| `S / ↓` | Backward |
| `A / ←` | Rotate left |
| `D / →` | Rotate right |
| `Space` | Stop |
| `1–7` | Arm states: HOME → SEARCH → APPROACH → LOWER → GRIP → LIFT → DROP |
| D-pad buttons | Same as keyboard, touch-friendly |

Use AUTO/MANUAL buttons in the UI:
- **MANUAL**: D-pad/WASD commands are applied to robot base via `/cmd_vel/ui`.
- **AUTO**: manual commands are ignored by arbitrator and dummy robot runs autonomous patrol loop.

### View modes

- **ORBIT** — free camera, drag to rotate
- **FOLLOW** — camera trails behind the robot
- **FPS** — first-person from the robot's camera mount, and FPS frames are published to ROS camera source topics

### Upload images to AI worker

Click **⬆ UPLOAD**, pick any JPEG/PNG.  
The AI worker runs the same HSV object-detection pipeline on it and publishes results
to `/detected_objects` and `/task_plan` — identical to what live camera frames produce.  
The Detections HUD updates immediately.

---

## Verify ROS topics are flowing

Run these in a second terminal while the stack is up.

> **Note:** `docker compose exec` bypasses the container entrypoint, so ROS2 is not on PATH by default.
> Prefix every `ros2` command with `/entrypoint.sh` to source the environment first.

```bash
# All active topics
docker compose exec dummy_robot /entrypoint.sh ros2 topic list

# Robot position + orientation (20 Hz)
docker compose exec dummy_robot /entrypoint.sh ros2 topic echo /odom --once

# Arm + wheel joint angles (10 Hz)
docker compose exec dummy_robot /entrypoint.sh ros2 topic echo /joint_states --once

# LiDAR scan — 360 ranges (8 Hz)
docker compose exec dummy_robot /entrypoint.sh ros2 topic echo /scan --once

# AI detections
docker compose exec dummy_robot /entrypoint.sh ros2 topic echo /detected_objects

# Task planner output
docker compose exec dummy_robot /entrypoint.sh ros2 topic echo /task_plan

# Robot events (occasional)
docker compose exec dummy_robot /entrypoint.sh ros2 topic echo /robot/events

# Control mode (auto / manual)
docker compose exec dummy_robot /entrypoint.sh ros2 topic echo /robot/control_mode

# Active camera source (dummy|fps)
docker compose exec dummy_robot /entrypoint.sh ros2 topic echo /camera/source/active

# Camera source command
docker compose exec dummy_robot /entrypoint.sh ros2 topic pub --once /camera/source/select std_msgs/msg/String "{data: fps}"

# Topic publish rates
docker compose exec dummy_robot /entrypoint.sh ros2 topic hz /odom
docker compose exec dummy_robot /entrypoint.sh ros2 topic hz /joint_states
docker compose exec dummy_robot /entrypoint.sh ros2 topic hz /camera/front/image_raw

# Camera bandwidth
docker compose exec dummy_robot /entrypoint.sh ros2 topic bw /camera/front/image_raw
```

## Topic guide (meaning + data to check)

Use this as a practical reference during manual QA.

| Topic | Type | Produced by | What it means | Key fields to inspect |
|---|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | `dummy_robot` | Robot pose and velocity in `odom` frame | `pose.pose.position.x/y`, `pose.pose.orientation`, `twist.twist.linear`, `twist.twist.angular.z` |
| `/joint_states` | `sensor_msgs/JointState` | `dummy_robot` | Arm + wheel joint angles | `name[]`, `position[]` |
| `/arm/state` | `std_msgs/String` (JSON) | `dummy_robot` | Arm state-machine status | JSON keys: `state`, `cycle_id`, `target_object_id`, `success_probability`, `manual_overrides` |
| `/scan` | `sensor_msgs/LaserScan` | `dummy_robot` | 2D lidar ranges around robot | `ranges[]`, `angle_min`, `angle_max`, `range_max` |
| `/robot/events` | `std_msgs/String` (JSON) | `dummy_robot` | Low-frequency event stream | JSON keys: `event`, `timestamp`, `robot_pose`, `steam_level`, `wet_floor` |
| `/cmd_vel/ui` | `geometry_msgs/Twist` | `frontend` | Raw manual drive command from UI | `linear.x`, `angular.z` |
| `/robot/control_mode/set` | `std_msgs/String` | `frontend` + `control_arbitrator` | Mode command (`auto` or `manual`) | `data` |
| `/cmd_vel` | `geometry_msgs/Twist` | `control_arbitrator` | Canonical base command after arbitration | `linear.x`, `angular.z` |
| `/robot/control_mode` | `std_msgs/String` (JSON) | `control_arbitrator` | Current mode + active command source | JSON keys: `mode`, `active_source`, `ui_fresh`, `vx`, `wz` |
| `/camera/source/select` | `std_msgs/String` | `frontend` | Request camera source (`dummy` or `fps`) | `data` |
| `/camera/source/active` | `std_msgs/String` | `camera_mux` | Current selected camera source | `data` (`dummy` or `fps`) |
| `/camera/front/image_raw` | `sensor_msgs/Image` | `camera_mux` | Canonical uncompressed camera stream for ROS consumers | `height`, `width`, `encoding`, `data` |
| `/camera/front/image_raw/compressed` | `sensor_msgs/CompressedImage` | `camera_mux` | Canonical compressed stream for UI and AI worker | `format`, `data` |
| `/camera/front/camera_info` | `sensor_msgs/CameraInfo` | `camera_mux` | Intrinsics for canonical camera stream | `k`, `d`, `p`, `width`, `height` |
| `/detected_objects` | `std_msgs/String` (JSON) | `ai_worker` | Perception output from camera frames | JSON keys: `timestamp`, `frame_id`, `objects[]` |
| `/task_plan` | `std_msgs/String` (JSON) | `ai_worker` | Next high-level action proposal | JSON keys: `task`, `next_action`, `target_object_id`, `reason` |

### Robot location quick check

```bash
# One sample
docker compose exec dummy_robot /entrypoint.sh ros2 topic echo /odom --once

# Live stream
docker compose exec dummy_robot /entrypoint.sh ros2 topic echo /odom
```

- Position: `pose.pose.position.x` and `pose.pose.position.y` (meters)
- Heading: derived from `pose.pose.orientation` quaternion (UI `YAW` already converts this)

### Expected rates when healthy

| Topic | Expected Hz |
|---|---|
| `/odom` | 20 |
| `/joint_states` | 10 |
| `/camera/front/image_raw` | ~7 |
| `/scan` | 8 |
| `/detected_objects` | ~7 (tied to camera) |

For low-spec hardware, FPS-source camera is intentionally throttled (~4 Hz).

---

## Foxglove Studio (alternative data view)

1. Open [Foxglove Studio](https://foxglove.dev/studio)
2. **Open connection → Foxglove WebSocket → `ws://localhost:8765`**
3. Useful panels:
   - **Image** → `/camera/front/image_raw`
   - **3D** → `/tf`, `/scan`, `/joint_states`
   - **Plot** → `/odom` (position x/y over time)
   - **Raw Messages** → `/detected_objects`, `/task_plan`

---

## Topic contract

### Published by `dummy_robot`

| Topic | Type | Rate |
|---|---|---|
| `/camera/front/image_raw` | `sensor_msgs/Image` (rgb8, 640×480) | 7 Hz |
| `/camera/front/image_raw/compressed` | `sensor_msgs/CompressedImage` (jpeg) | 7 Hz |
| `/camera/front/camera_info` | `sensor_msgs/CameraInfo` | 7 Hz |
| `/scan` | `sensor_msgs/LaserScan` (360 rays, ±π, max 8 m) | 8 Hz |
| `/odom` | `nav_msgs/Odometry` | 20 Hz |
| `/tf` | `base_link` in `odom` frame | 20 Hz |
| `/tf_static` | `base_link→laser_link`, `→camera_front_link`, `→arm_base_link` | once |
| `/joint_states` | `sensor_msgs/JointState` (6 arm + 2 wheel joints) | 10 Hz |
| `/arm/state` | `std_msgs/String` (JSON) | 10 Hz |
| `/robot/events` | `std_msgs/String` (JSON) | ~0.1 Hz |
| `/robot/control_mode` | `std_msgs/String` (JSON) | 20 Hz |

### Published by `ai_worker`

| Topic | Type | Rate |
|---|---|---|
| `/detected_objects` | `std_msgs/String` (JSON) | ~7 Hz |
| `/task_plan` | `std_msgs/String` (JSON) | ~7 Hz |

### Subscribed by `dummy_robot` (control inputs)

| Topic | Type | Publisher |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | web UI / nav stack |
| `/arm/action` | `std_msgs/String` (JSON) | web UI |

---

## JSON schemas

### `/arm/state`
```json
{
  "state": "LOWER_TO_TOWEL",
  "cycle_id": 12,
  "target_object_id": "obj_005",
  "success_probability": 0.72,
  "manual_overrides": []
}
```

### `/robot/control_mode`
```json
{
  "mode": "manual",
  "active_source": "ui",
  "ui_fresh": true,
  "vx": 0.35,
  "wz": 0.0
}
```

### `/robot/events`
```json
{
  "event": "PERSON_TOO_CLOSE",
  "timestamp": "2025-01-01T12:00:00+00:00",
  "robot_pose": { "x": 0.8, "y": -1.1, "yaw": 1.57 },
  "steam_level": "medium",
  "wet_floor": true
}
```

### `/detected_objects`
```json
{
  "timestamp": "2025-01-01T12:00:00+00:00",
  "frame_id": 42,
  "objects": [
    {
      "id": "det_001",
      "class": "towel",
      "confidence": 0.86,
      "bbox": [100, 200, 230, 310],
      "robot_class": "pickable_soft_object",
      "pickable": true,
      "risk": "low",
      "estimated_position": { "x": 1.1, "y": -0.2, "z": 0.0 }
    }
  ]
}
```

### `/task_plan`
```json
{
  "task": "collect_onsen_floor_garbage",
  "next_action": "pick_object",
  "target_object_id": "det_001",
  "reason": "nearest pickable towel detected (confidence=0.86)"
}
```

### `/arm/action` (control input)
```json
{ "cmd": "set_state", "state": "GRIP" }
{ "cmd": "set_joint", "joint": "shoulder_pan_joint", "value": 0.5 }
{ "cmd": "clear" }
```

---

## Record / replay

```bash
# Record to ./bags/ (MCAP format)
docker compose --profile record up

# Replay a recorded bag on loop
docker compose --profile play up
```

Inspect a bag:
```bash
docker compose exec dummy_robot /entrypoint.sh ros2 bag info /bags/onsen_dummy_run
```

---

## Save dataset frames

```bash
SAVE_DATASET=true docker compose up
```

Saves to:
- `dataset/images/frame_NNNNNN.jpg`
- `dataset/annotations/frame_NNNNNN.json` — ground-truth JSON per frame (COCO-compatible)

---

## Environment variables

| Variable | Default | Description |
|---|---|---|
| `ROS_DOMAIN_ID` | `42` | ROS2 DDS discovery domain |
| `SAVE_DATASET` | `false` | Write frames + annotations to `dataset/` |

---

## Arm states

```
HOME → SEARCH → APPROACH_OBJECT → LOWER_TO_TOWEL → GRIP → LIFT → DROP_TO_TRAY → (repeat)
                                                      ↓
                                               FAILED_GRIP (~20%)
```

---

## Extending the stack

**Replace the AI worker with a real model**  
Keep `/detected_objects` and `/task_plan` JSON schemas unchanged.
Swap `ai_worker_node.py` with ONNX / TensorRT inference — the frontend and rosbag pipeline see no difference.

**Add an LLM planner**  
Subscribe to `/detected_objects` + `/robot/events`, publish to `/task_plan`.

**Add a nav stack**  
Subscribe to `/scan` + `/odom`, publish `/cmd_vel`.
The dummy robot incorporates it automatically; the web UI shows MANUAL mode during active commands.

**COCO dataset export**  
Set `SAVE_DATASET=true`, drive the robot around, collect `dataset/annotations/*.json`.

---

## Architecture

```
docker compose up
│
├── dummy_robot ─────────────────────────────────────────────────────►
│     SceneGenerator (OpenCV)  →  /camera/front/image_raw
│                                 /camera/front/image_raw/compressed
│     LidarGenerator (NumPy)   →  /scan
│     OdomPublisher            →  /odom  /tf  /tf_static
│     ArmStateMachine          →  /joint_states  /arm/state
│     EventPublisher           →  /robot/events  /robot/control_mode
│     ◄── /cmd_vel  /arm/action  (from rosbridge or nav stack)
│
├── ai_worker ◄── /camera/front/image_raw ──────────────────────────►
│     OpenCV HSV detector       →  /detected_objects
│     Task planner              →  /task_plan
│     HTTP POST :5000/upload    →  /detected_objects  /task_plan
│
├── rosbridge  ws://localhost:9090  (all topics ↔ browser)
│
├── foxglove_bridge  ws://localhost:8765  (all topics → Foxglove Studio)
│
├── frontend  http://localhost:8080
│     Three.js 3D viewfinder  ←→  rosbridge WebSocket
│     D-pad / WASD / joint sliders → /cmd_vel  /arm/action
│     Camera PiP ← /camera/front/image_raw/compressed
│     LiDAR minimap ← /scan
│     nginx /api/ proxy → ai_worker :5000
│
├── rosbag_recorder  [profile: record]  →  ./bags/  (MCAP)
└── rosbag_player    [profile: play]    ←  ./bags/
```

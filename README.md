# onsen_robot_data_stream

A ROS2 Jazzy Docker Compose demo environment that publishes realistic synthetic
topics mimicking a small tank-drive cleaning robot operating in a Japanese
onsen / sauna / bathroom-like indoor environment.

**This is not a physics simulator.**
There is no Gazebo, Isaac Sim, or robot model involved.
All data is generated in Python using NumPy and OpenCV.
The purpose is data-pipeline development, model integration testing,
LLM planner research, and ROS2 topic-contract validation.

---

## Prerequisites

- Docker ≥ 24
- Docker Compose ≥ 2.20
- ~3 GB disk for the Docker image

---

## Build

```bash
cd onsen_robot_data_stream
docker compose build
```

---

## Run

### Core services (robot + AI + Foxglove bridge)

```bash
docker compose up dummy_robot ai_worker foxglove_bridge
```

### View in Foxglove Studio

1. Open [Foxglove Studio](https://foxglove.dev/studio) in your browser
2. Click **Open connection** → **Foxglove WebSocket**
3. Enter: `ws://localhost:8765`
4. Add panels: Image (`/camera/front/image_raw`), 3D (`/tf`, `/scan`), Plot (`/odom`), Raw Messages (`/detected_objects`, `/task_plan`)

---

## Record a ROS bag

```bash
docker compose --profile record up rosbag_recorder
```

Bag is saved to `./bags/onsen_dummy_run/` in MCAP format.

Recorded topics:
- `/camera/front/image_raw`
- `/camera/front/camera_info`
- `/scan`
- `/odom`
- `/tf`
- `/tf_static`
- `/joint_states`
- `/arm/state`
- `/robot/events`
- `/detected_objects`
- `/task_plan`

---

## Replay a bag

```bash
docker compose --profile play up rosbag_player
```

---

## Inspect topics

```bash
# List all active topics
docker compose exec dummy_robot ros2 topic list

# Echo detections
docker compose exec dummy_robot ros2 topic echo /detected_objects

# Echo task plan
docker compose exec dummy_robot ros2 topic echo /task_plan

# Echo robot events
docker compose exec dummy_robot ros2 topic echo /robot/events

# Echo arm state
docker compose exec dummy_robot ros2 topic echo /arm/state

# Check topic bandwidth
docker compose exec dummy_robot ros2 topic bw /camera/front/image_raw

# Inspect a bag
docker compose exec rosbag_recorder ros2 bag info /bags/onsen_dummy_run
```

---

## Save dataset frames

```bash
SAVE_DATASET=true docker compose up dummy_robot ai_worker foxglove_bridge
```

Frames are saved to:
- `dataset/images/frame_NNNNNN.jpg`
- `dataset/annotations/frame_NNNNNN.json` (ground-truth JSON per frame)

---

## Environment variables

| Variable | Default | Description |
|---|---|---|
| `ROS_DOMAIN_ID` | `42` | ROS2 discovery domain |
| `SAVE_DATASET` | `false` | Save generated frames to `dataset/` |

---

## Topic contract

### Camera
| Topic | Type |
|---|---|
| `/camera/front/image_raw` | `sensor_msgs/msg/Image` (rgb8, 640×480) |
| `/camera/front/camera_info` | `sensor_msgs/msg/CameraInfo` |

### LiDAR
| Topic | Type |
|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` (360 rays, ±π) |

### Robot movement
| Topic | Type |
|---|---|
| `/odom` | `nav_msgs/msg/Odometry` |
| `/tf` | via `tf2_ros.TransformBroadcaster` |
| `/tf_static` | `base_link→laser_link`, `base_link→camera_front_link`, `base_link→arm_base_link` |

### Arm
| Topic | Type |
|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` |
| `/arm/state` | `std_msgs/msg/String` (JSON) |

### Events & AI output
| Topic | Type |
|---|---|
| `/robot/events` | `std_msgs/msg/String` (JSON) |
| `/detected_objects` | `std_msgs/msg/String` (JSON) |
| `/task_plan` | `std_msgs/msg/String` (JSON) |

---

## JSON schemas

### `/arm/state`
```json
{
  "state": "LOWER_TO_TOWEL",
  "cycle_id": 12,
  "target_object_id": "obj_005",
  "success_probability": 0.72
}
```

### `/robot/events`
```json
{
  "event": "PERSON_TOO_CLOSE",
  "timestamp": "2025-01-01T12:00:00+00:00",
  "robot_pose": {"x": 0.8, "y": -1.1, "yaw": 1.57},
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
      "estimated_position": {"x": 1.1, "y": -0.2, "z": 0.0}
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

---

## Object classes

| Class | Robot class | Risk |
|---|---|---|
| `towel` | `pickable_soft_object` | low |
| `clothes` | `pickable_soft_object` | low |
| `slipper` | `pickable_soft_object` | low |
| `brush` | `pickable_soft_object` | low |
| `plastic_trash` | `pickable_soft_object` | low |
| `bath_mat` | `pickable_soft_object` | low |
| `bottle` | `non_pickable_hard_object` | avoid |
| `bucket` | `non_pickable_hard_object` | avoid |
| `can` | `non_pickable_hard_object` | avoid |
| `bench` | `static_environment` | avoid |
| `floor` | `static_environment` | low |
| `wall` | `static_environment` | avoid |
| `person_body_part` | `safety_stop` | **stop** |
| `unknown_obstacle` | `unknown_obstacle` | **stop** |

---

## Arm states

`HOME` → `SEARCH` → `APPROACH_OBJECT` → `LOWER_TO_TOWEL` → `GRIP` → `LIFT` → `DROP_TO_TRAY` → (repeat)

With ~20% chance of `FAILED_GRIP` injected at `GRIP`.

---

## Extending this project

- **Replace AI worker**: swap `ai_worker_node.py` with a real ONNX / TensorRT model.
  Keep the `/detected_objects` and `/task_plan` JSON schemas unchanged.
- **Add LLM planner**: subscribe to `/detected_objects` + `/robot/events` and publish to `/task_plan`.
- **Add nav stack**: subscribe to `/scan` + `/odom` and publish `/cmd_vel` — the dummy robot
  will incorporate it automatically if `cmd_vel` subscriber is wired.
- **COCO dataset export**: set `SAVE_DATASET=true` and collect `dataset/annotations/*.json`.

---

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│  Docker network (host mode — all on ROS_DOMAIN_ID=42)    │
│                                                           │
│  dummy_robot ──────────────────────────────────────────► │
│    SceneGenerator (OpenCV) → /camera/front/image_raw      │
│    LidarGenerator (NumPy)  → /scan                        │
│    OdomPublisher           → /odom  /tf                   │
│    ArmStateMachine         → /joint_states  /arm/state    │
│    EventPublisher          → /robot/events                │
│                                                           │
│  ai_worker ◄── /camera/front/image_raw                   │
│    OpenCV HSV detector     → /detected_objects            │
│    Task planner            → /task_plan                   │
│                                                           │
│  foxglove_bridge ◄── all topics → ws://localhost:8765     │
│                                                           │
│  rosbag_recorder [profile:record]                         │
│    → bags/onsen_dummy_run/ (MCAP)                         │
│                                                           │
│  rosbag_player [profile:play]                             │
│    bags/onsen_dummy_run/ → all topics                     │
└──────────────────────────────────────────────────────────┘
```

# AI worker guide (data scientists)

Everything you need to analyze robot data, swap in models, and drive the robot
from an LLM — with or without the browser open.

## What you get

| Stream | Where | Use |
|---|---|---|
| Camera frames | `/camera/front/image_raw/compressed` (+ rear, depth) | detection, VLM input |
| LIDAR | `/scan` | mapping, obstacle features |
| Robot state | `/robot/state` (fused JSON) | one subscription for pose/safety/arm/base |
| Ground truth | `/ground_truth/objects`, `/ground_truth/pose` | labels, drift and recall metrics |
| Actuators | `/arm/command` + `/arm/response`, `/base/command` + `/base/response` | raw firmware protocol, LLM tool-calling target |
| Detections / plans | `/detected_objects`, `/task_plan`, `/mission/state` | the stock pipeline's outputs |

## Quick start

```bash
docker compose up                      # full stack + browser sim
docker compose --profile record up     # also record ./bags/onsen_run (MCAP)
SIM_SOURCE=replay docker compose --profile play up   # iterate on a recorded run, no browser
SIM_SOURCE=synthetic docker compose up               # fully headless 2.5D world
```

Notebooks (in `src/onsen_ai_worker/notebooks/`):

1. **01_explore_robot_data** — live-topic quickstart and bag analysis
2. **02_llm_drives_arm_to_towel** — the flagship loop: feed `/arm/state` +
   detections to an LLM, command the arm over `/arm/command` until it reaches
   the towel; runs against replay with no FE open
3. **03_skin_detection_eval** — quantifies detection recall before/after a skin
   change and the profile-resampling recovery (0.87 → 0.00 → 0.95 with seed 42)

## The detection pipeline (and how to replace it)

`detection.py` is a transparent classical-CV baseline: per-class HSV bands ->
morphology -> contours -> NMS -> ground-plane back-projection using the real
camera intrinsics from `shared/robot_spec.json`. The `Detector` class is the
swap seam: keep `detect(img) -> list[dict]` and the `/detected_objects` schema,
and an ONNX/TensorRT model drops in without touching the node.

### Skin pipeline

Re-texturing objects in the FE genuinely changes the camera frames. The FE
`sync AI profile` checkbox POSTs the skin to `POST /api/profiles/<class>`,
which resamples that class's HSV band (5th–95th percentile + lighting pad).

- `GET /profiles` — inspect current bands
- `DELETE /profiles/<class>` — restore defaults
- Disable auto-sync in the FE to *study* detection failure instead (notebook 03)

## LLM integration

`llm_client.py` selects by env: with `LLM_BASE_URL` + `LLM_API_KEY` set, any
OpenAI-compatible `/chat/completions` endpoint (OpenAI, Ollama, vLLM, LM
Studio); otherwise a deterministic mock so the stack runs keyless and tests
are reproducible.

```bash
LLM_BASE_URL=http://localhost:11434/v1 LLM_API_KEY=ollama LLM_MODEL=qwen2.5 docker compose up
```

`mission_executor` consults the LLM every `MISSION_LLM_INTERVAL` (default 5 s)
with safety state + detections; its verdict is reported on `/mission/state`
(`llm`, `llm_reason`).

### Navigation seam (deliberate)

The mission executor takes towel **world positions** from
`/ground_truth/objects` and uses `/detected_objects` as confirmation. Pure
detection-based navigation (back-projected `estimated_position` +
approach-servoing) is the documented exercise: replace `MissionInput.towels`
in `mission_executor_node.py`.

## Recording datasets

`SAVE_DATASET=true` (synthetic mode) writes `dataset/images/*.jpg` +
COCO-compatible annotations. From the FE path, record bags and extract frames —
ground-truth labels come from `/ground_truth/objects` aligned by timestamp.

## Rules of the road

- Keep `/detected_objects` and `/task_plan` JSON schemas stable — the FE HUD,
  mission executor and tests all consume them
- Publish autonomy commands to `/cmd_vel/auto` (the arbitrator owns `/cmd_vel`)
- `/odom` drifts on purpose; use `/ground_truth/pose` to measure that drift,
  not to cheat your controller

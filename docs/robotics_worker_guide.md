# Robotics worker guide (robotics engineers)

The robotics-engineer surface: firmware protocols, the safety aggregator, and
the headless/replay workflow for working on robot data streams without the FE.

## Firmware emulators

Both controllers are pure-logic state machines (`arm_protocol.py`,
`base_protocol.py` — no ROS imports) wrapped by thin nodes, so you can unit
test or embed them directly.

### Arm (`/arm/command` -> `/arm/response`)

The full serial grammar from the real controller:

```
Q                          -> STATE j0 j1 j2 j3 j4 j5 IDLE|MOVING|STOPPED
A HOME|READY|STOW|PRE_PICK|PICK_*|DROP_*|SEARCH_*|OPEN_GRIPPER|...
J 90 70 120 110 90 40 1000 -> absolute move (deg, ms)
D 0 5 300                  -> relative single joint
M ARM_DOWN 4 300           -> named relative move
G 35 200                   -> gripper absolute
SPEED 50 | STOP | A RESET_ERROR
CAL SHOW|SET|DEG|SAVE|LOAD|RESET   (persists to output/arm_calibration.json)
RELAX [j] | WAKE [j]
```

Limits return `ERR LIMIT joint=0 value=295`; after `STOP`, everything is
`ERR STOPPED` until `A RESET_ERROR`. The transcript from the project brief is
replayed verbatim in `src/onsen_dummy_robot/test/test_arm_protocol.py` and
end-to-end through ROS in e2e scenario 5.

### Base (`/base/command`, `/cmd_vel`)

```
Q              -> STATE vx wz w0..w5 IDLE|TWIST|WHEEL|STOPPED|SAFETY
V 0.3 0.5      -> twist mode (vx m/s, wz rad/s)
T 0.4 0.2      -> tank mode (left/right surface speeds)
W 2 6.0        -> individual wheel rad/s
SPEED 50 | STOP | RESET_ERROR
```

Twist/wheel commands time out to zero (watchdog). `/safety/stop` zeroes and
latches regardless of input; `RESET_ERROR` cannot clear the safety latch —
only the aggregator can (by dropping `/safety/stop`).

## Safety aggregator (`onsen_robot_state`)

The e-stop latch is **opt-in**: disarmed by default (`SAFETY_ENABLED=false`),
armed via the FE `E-STOP` toggle (`/safety/enable`) or env. While disarmed the
aggregator still fuses `/robot/state` (incl. contacts/tilt telemetry) but never
latches `/safety/stop`; disarming clears any active latch.

`SafetyMonitor` (pure logic, `safety.py`) implements PLC-style latching when armed:

| Trigger | Source | Latch |
|---|---|---|
| impulse ≥ `contact_impulse_stop_threshold` (3.0 N·s) | `/robot/contacts` | until `/safety/reset` |
| critical contact (water) | `/robot/contacts` `critical: true` | critical latch |
| tilt ≥ 30° | `/imu` | refuses reset while still tilted |

`/robot/state` (5 Hz) is the one-stop fused view: odom, arm, base, tilt,
nearest obstacle, last contact, `arm_scan_filter`. The scan-sector self-filter
masks 70–90° while the arm reports a DROP phase so the wrist never reads as an
obstacle.

Aggregation patterns to extend here: speed limiting near obstacles, zone-based
interlocks (no arm motion while moving), battery/thermal models.

## Headless workflow

```bash
SIM_SOURCE=synthetic docker compose up
```

The Python 2.5D sim (`dummy_stream_node.py`) integrates the pose from
`/base/wheel_targets` against the same `shared/onsen_layout.json`, raycasts
`/scan` from the same geometry, simulates grasp/drop against the arm targets
and publishes the same topics — the controllers, safety node, AI worker and
mission executor run **unchanged**. Collisions publish `/robot/contacts` so
the whole safety path is exercisable headless.

```bash
docker compose --profile record up                  # capture a session (MCAP, ./bags)
SIM_SOURCE=replay docker compose --profile play up  # loop it for algorithm work
```

Recorded FE sessions and synthetic sessions are interchangeable for algorithm
work because they share layout, protocols and noise constants.

## Testing

```bash
make check-py   # ruff + mypy + pytest in the ROS image
```

- `test_arm_protocol.py` / `test_base_protocol.py` — protocol transcripts
- `test_safety.py` — latching, reset refusal, scan self-filter
- `test_mission.py` — autonomy FSM incl. abort-on-safety mid-pick
- e2e scenario 6 — full safety loop through the live stack (ram a stool ->
  latch -> wheels zero -> operator reset -> recover)

## Invariants

- `FASTDDS_BUILTIN_TRANSPORTS: UDPv4` in every ROS container
- rosbridge without `use_events_executor`
- One `/cmd_vel` owner (the arbitrator); firmware nodes never bypass it
- `/safety/stop` consumers must treat it as unconditional

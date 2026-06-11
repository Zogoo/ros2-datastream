# Robot design rationale

All dimensions live in `shared/robot_spec.json` (single source for the FE
builder, Python nodes, tests and this document). Frame: `base_link` X forward,
Y left, Z up.

## Constraints derived from the environment and firmware

- Corridor 1.60 m, doorways 1.10 m (`shared/onsen_layout.json`)
- Traversable stairs: resting-deck steps, risers 45 mm and 90 mm
- Bath rims 280 mm — must be **unclimbable** (water = kill hazard)
- Wet tile friction zones (μ 0.45 wet / 0.8 dry)
- Firmware fixes the actuator class: 6 bus servos 0–180°, pulse 500–2500 µs;
  wheel radius 0.07 m, track 0.47 m, max 12 rad/s -> 0.84 m/s top speed

## Chassis and drivetrain

- Tub 0.62 × 0.42 × 0.14 m, overall width incl. wheels 0.52 m, LIDAR top 0.67 m,
  mass budget 17 kg (frame 5.5, hub motors 3.0, battery 2.5, electronics 1.0,
  sensors 0.7, arm 1.8, basket + payload 2.5)
- Doorway margin (1.10 − 0.52)/2 = 0.29 m per side; spin diameter
  √(0.68² + 0.52²) = 0.86 m < 1.10 m -> can turn around inside any doorway
- 6 wheels Ø140 × 50 mm at x = ±0.24, 0 (wheelbase 0.48 m), each an independent
  trailing-arm coil-over realized as a suspension raycast
- **Suspension from the mass budget, not guessed**: static load ≈ 28 N/wheel,
  60 mm travel with 15 mm sag -> k = 1850 N/m; damping ratio 0.65 -> c = 94 N·s/m
- Step capability: 45 mm riser = 0.64 × wheel radius — climbable with
  independent springs; the 90 mm second step works because the front axle is
  already on the first step (phase climbing). The 280 mm rim is 2× wheel
  diameter — geometrically impossible, the intended passive safety barrier,
  with the water-contact e-stop as backstop
- CoG at z ≈ 0.18 m (battery flat on the tub floor) -> static tip angles ≈ 50°;
  arm at full reach + 0.3 kg towel shifts CoG < 40 mm — deep inside the support
  polygon

## Rack architecture

| Deck | z | Contents | Why |
|---|---|---|---|
| 0 (tub) | 0.06–0.20 | battery (center-rear), drivers, e-stop relay | counterweights the front arm; IP-skirted |
| 1 | 0.34 | SBC, IMU at CoG, front sensor cluster, rear camera | open sides keep arm sweep + airflow clear |
| 2 | 0.44 | arm base front-center (x +0.26), basket left-aft (rim 0.50), LIDAR mast rear-center | pick workspace ahead of the robot; basket bearing matches `DROP_BASKET` pan 178° |

Nothing except the mast and a DROP-phase wrist exceeds z 0.58 — a 40 mm guard
band under the 0.62 m scan plane.

## Sensor suite (modeled after real hardware)

| Sensor | Mount | Model | Key numbers |
|---|---|---|---|
| 360° LIDAR | mast, z 0.62 | RPLIDAR A1-class | 0.15–10 m, 360×1°, 8 Hz, σ ≈ 1 % of range |
| Front RGB | z 0.38, pitch −15° | budget CMOS | HFOV 70°, 640×480, ~5 Hz |
| Rear RGB | z 0.38, pitch −10° | same | reversing coverage |
| Depth | z 0.36, pitch −20° | D435-class | 87° HFOV, 0.28–3 m, 320×240, σ ∝ z² |
| Sonar ×3 | nose, z 0.10, ±25/0° | HC-SR04-class | 15° cones, 0.02–4 m, steam-immune |
| Contact skirt | z 0.08, 4 sides | bumper strips | named `part` in `/robot/contacts` |
| IMU | CoG | MEMS | 50 Hz, bias random-walk |

## Visibility and occlusion analysis (the core decision)

The LIDAR plane at 0.62 m **sees**: walls (2.6 m), shower partitions (1.4 m),
lockers (1.9 m), make-up counter (0.85 m), upper sauna bench (0.85 m) —
reliable SLAM-grade geometry.

It does **not** see: bath rims 0.28, loungers 0.41, lower sauna bench 0.45,
bins 0.50–0.55, stools/buckets/towels < 0.30.

Hence the **two-tier perception design**: high LIDAR for mapping and
localization; depth camera + sonar for the low-obstacle layer. This is the
single most important placement decision and is validated by e2e scenario 7
(bath rim invisible to `/scan`, present in depth frames).

Known limitations (documented, mitigated):

- **Near-field pick blind zone**: depth floor coverage starts 0.31 m ahead but
  picks happen at 0.15–0.50 m — the final 20 cm run on target memory
  (measure-then-dead-reckon), standard on real pick robots
- **Side blind zone below 0.34 m**: covered only by the contact skirt and the
  arbitrator's rotation speed cap — matches real budget service robots
- **Arm in scan plane**: only DROP poses cross z 0.62, ~1 s in the 70–90°
  sector; the safety aggregator self-filters that sector while `/arm/state`
  reports a DROP phase (`arm_scan_filter: true` in `/robot/state`)

## Arm

- Shoulder at z 0.50; links 0.25 / 0.25 / 0.20 m -> 0.70 m reach. Floor pick
  0.45 m past the nose needs √(0.48² + 0.50²) = 0.69 m — reachable with margin
- Joint order = firmware J0–J5: pan (0 right, 90 fwd, 180 left), shoulder,
  elbow, wrist pitch, wrist roll, gripper (0–90 mm parallel fingers)
- Every named firmware pose is FK-validated in `frontend/tests/kinematics.test.js`
  (PICK_SCOOP fingertip within 15 mm of floor, DROP_BASKET above basket rim,
  STOW under z 0.58) — the pose table and the 3D model cannot drift apart
- Servo dynamics: 200 °/s at SPEED 100, first-order lag τ = 80 ms — visible
  tracking error in `/joint_states`
- Payload check: 0.3 kg towel at full reach ≈ 2.1 N·m at the shoulder — inside
  bus-servo torque (~25 kg·cm geared)
- **Grasp is a physics joint**: closing the gripper within the grasp radius
  creates a fixed joint to the still-dynamic towel; carried mass is pushed
  back onto the chassis (suspension visibly settles); release restores normal
  dynamics with the fingertip velocity

## Industrial design

Two-tone service-robot language (warm white shrouds, charcoal skirt) for onsen
interiors; rounded link shrouds (no pinch points, wipe-clean); recessed hooded
sensor windows (anti-fog suggestion); amber LED status ring at the skirt
(idle/moving/picking/e-stop — also a robot-state cue visible in camera
recordings); rear carry handle and labeled e-stop.

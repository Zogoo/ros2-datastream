# Robot design rationale — TidyBot-IX (Concept A)

The simulated robot is the **Concept A** variant of TidyBot-IX (see
`docs/robot_design_v2.md`): the low-cost, indoor-dry proof of concept — RPi 5 +
accelerator compute, a single RPLIDAR A2, a custom 6-DOF Dynamixel arm
(~0.35 kg payload), an OAK-D Lite plus two USB cameras, and hobby gearmotors on
a differential base. It runs in the onsen test environment to prove the
detect → approach → grasp pipeline.

All dimensions live in `shared/robot_spec.json` (single source for the FE
builder, Python nodes, tests and this document). Frame: `base_link` X forward,
Y left, Z up.

## Constraints derived from the environment and firmware

- Corridor 1.60 m, doorways 1.10 m (`shared/onsen_layout.json`)
- Traversable stairs: resting-deck steps, risers 45 mm and 90 mm
- Bath rims 280 mm — must be **unclimbable** (water = kill hazard)
- Wet tile friction zones (μ 0.45 wet / 0.8 dry) modelled even though Concept A
  is an indoor-dry build — the wet-floor failure modes still exercise the stack
- Firmware fixes the actuator class: 6 Dynamixel bus servos 0–180°; two driven
  wheels Ø200 mm (radius 0.10 m), track 0.50 m, max 12 rad/s → 1.2 m/s top speed

## Chassis and drivetrain

- Body 0.80 × 0.60 × 0.20 m structural shell (decks + LIDAR mast rise above it);
  LIDAR top ≈ 0.66 m; body mass ≈ 22 kg
- **Differential drive, centre-drive layout**: two driven Ø200 mm wheels on the
  **centre** transverse axle (x = 0, track 0.50 m) + four Ø100 mm swivel casters
  at the corners (x = ±0.30, y = ±0.24) for support. Putting the drive axle on
  the centreline makes `base_link` the pivot, so a rotate-in-place sweeps only
  the circumscribed radius (~0.52 m) rather than a wide arc, and the wheel-odom
  turn model is truthful (a rear-axle pivot made the front swing ~0.67 m and
  the odom drift on every turn, which rammed the robot into props). Zero-scrub
  rotation, clean odometry, simple kinematics.
- Spin diameter √(0.80² + 0.60²) = 1.00 m; the nav planner inflates to a
  **0.55 m** circumscribed radius (incl. bumper ring). The onsen layout
  (`shared/onsen_layout.json`) is scaled up ~1.6× so this larger footprint has
  clear paths — the 0.42/0.62 m onsen robot's world was too tight for it.
- Each wheel is an independent trailing-arm coil-over realized as a suspension
  raycast. The driven and caster groups use different `attach_z` (0.03 vs
  −0.02) so the body sits level at BASE_Z = 0.16 m despite the mixed diameters.
- **Suspension from the mass budget, not guessed**: the driven pair sits behind
  the CoG and carries ≈ 0.30/0.48 ≈ 62 % of the weight → ≈ 68 N/driven-wheel;
  60 mm travel with the static deflection mid-band → k = 2250 N/m; damping
  ratio ≈ 0.65 → c = 160 N·s/m
- Step capability: 45 mm and 90 mm risers are climbable — the big Ø200 mm driven
  wheels roll over them easily and the sprung Ø100 mm front casters absorb the
  threshold shock (the classic caster-judder failure, solved by suspension).
  The 280 mm bath rim is > 2× the caster diameter — geometrically impossible,
  the intended passive safety barrier, with the water-contact e-stop as backstop
- CoG at z ≈ 0.18 m (battery flat on the tub floor) → static tip angles ≫ the
  5° ramp; arm at full reach + 0.35 kg towel shifts CoG well inside the support
  polygon (driven axle to front casters)

## Rack architecture

| Deck | z | Contents | Why |
|---|---|---|---|
| 0 (tub) | 0.06–0.20 | battery (center-rear), drivers, e-stop relay | counterweights the front arm; over the driven axle |
| 1 | 0.34 | hardware tray: SBC (RPi 5), motor drivers, IMU at CoG, front sensor cluster, rear camera; also the collect-bin floor + load cell | open sides keep arm sweep + airflow clear |
| 2 | 0.44+ | arm base front-center (x +0.35, ahead of the front bumper at 0.412 so the pick crescent clears it), **collect-bin tray spanning the deck** (rim 0.58), LIDAR mast front-right corner | pick workspace ahead; the whole top deck is payload volume |

Nothing except the mast and a DROP-phase wrist exceeds z 0.58 — a guard band
under the 0.66 m scan plane.

## Sensor suite (Concept A tier)

| Sensor | Mount | Model | Key numbers |
|---|---|---|---|
| 360° LIDAR | mast, z 0.66 | RPLIDAR A2 (indoor-dry, not IP-rated) | 0.15–12 m, 360×1°, 8 Hz, σ ≈ 1 % of range |
| Front depth | z 0.36, pitch −20° | Luxonis OAK-D Lite (stereo) | 72° HFOV, 0.2–4 m, 320×240, σ ∝ z² |
| Front RGB | z 0.42, pitch −15° | USB webcam | HFOV 70°, 640×480, ~5 Hz |
| Rear RGB | z 0.42, pitch −10° | USB webcam | reversing coverage |
| Sonar ×3 | nose, z 0.10, ±25/0° | HC-SR04-class | 15° cones, 0.02–4 m, steam-immune |
| Bumper ring 360° | z 0.085, 8 segments | Roomba-class spring bumper | outermost shell, 12 mm proud; hit sector + `bearing_deg` in `/robot/contacts` (`bumper_front` … `bumper_rear_right`); struck segment flashes in the sim |
| IMU | CoG | MEMS | 50 Hz, bias random-walk |

The RPLIDAR A2 and the OAK-D Lite are the Concept-A downgrades from the v2-B
dual-RPLIDAR-S2 / D455 tier: a single non-IP scanner (indoor-dry only) and one
front stereo unit instead of a forward + wrist depth pair. The 3 sonar are kept
as a cheap, optics-free low-obstacle safety layer even though the v2 Concept-A
blurb omits them.

## Visibility and occlusion analysis (the core decision)

The LIDAR plane at 0.66 m **sees**: walls (2.6 m), shower partitions (1.4 m),
lockers (1.9 m), make-up counter (0.85 m), upper sauna bench (0.85 m) —
reliable SLAM-grade geometry.

It does **not** see: bath rims 0.28, loungers 0.41, lower sauna bench 0.45,
bins 0.50–0.55, stools/buckets/towels < 0.30.

Hence the **two-tier perception design**: high LIDAR for mapping and
localization; depth camera + sonar for the low-obstacle layer. This is the
single most important placement decision and is validated by e2e scenario 7
(bath rim invisible to `/scan`, present in depth frames).

Known limitations (documented, mitigated):

- **Near-field pick blind zone**: depth floor coverage starts ~0.36 m ahead but
  picks happen at 0.15–0.50 m — the final run is on target memory
  (measure-then-dead-reckon), standard on real pick robots
- **Side blind zone below deck 1**: covered only by the contact skirt and the
  arbitrator's rotation speed cap — matches real budget service robots
- **Arm in scan plane**: only DROP poses cross the LIDAR plane, ~1 s in the
  70–90° sector; the safety aggregator self-filters that sector while
  `/arm/state` reports a DROP phase (`arm_scan_filter: true` in `/robot/state`)

## Arm

- Custom 6-DOF Dynamixel arm (XM430/XM540-class bus servos), ~0.35 kg payload.
  Shoulder at z 0.50; links 0.25 / 0.25 / 0.20 m → 0.70 m reach. Floor pick
  0.45 m past the nose needs √(0.48² + 0.50²) = 0.69 m — reachable with margin
- Joint order = firmware J0–J5: pan (0 right, 90 fwd, 180 left), shoulder,
  elbow, wrist pitch, wrist roll, gripper (0–90 mm parallel fingers)
- Every named firmware pose is FK-validated in `frontend/tests/kinematics.test.js`
  (PICK_SCOOP fingertip within 15 mm of floor, DROP_BASKET above basket rim,
  STOW under the LIDAR plane) — the pose table and the 3D model cannot drift apart
- Servo dynamics: 200 °/s at SPEED 100, first-order lag τ = 80 ms — visible
  tracking error in `/joint_states`
- Payload check: 0.35 kg towel at full reach ≈ 2.4 N·m at the shoulder — inside
  a geared XM540-class bus-servo torque budget
- Key poses: `DROP_BASKET` (over-the-shoulder arc, release above the deck-bin
  rim), `BIN_PICK` (reach back INSIDE the deck bin) and `DROP_BIN` (pan 178°,
  extended links, release 0.72 m from base center) — the extended pose exists
  because the robot body can never get closer than bin-half + robot-half to a
  floor bin's center, so a short-radius drop cannot reach over a bin rim

## Collect bin (batch collection, top deck)

- The **entire top deck is the bin**: an open-top tray, interior
  0.37 × 0.30 × 0.22 m (~24 L, 6+ crumpled towels), floor on the deck-1
  hardware tray at z 0.36, **rim at z 0.58** — the tray sits FULLY inside the
  chassis footprint (no overhang). Deck 1 below carries all electronics; the
  lidar mast sits at the front-right corner (small self-occlusion wedge toward
  the arm column, redundantly covered by the front camera + sonar)
- The bin sits BEHIND the arm's shoulder — unreachable by pan but reachable by
  the chain's **negative-radial arc**: DROP_BASKET keeps pan forward and arcs
  the arm up over its own shoulder. BIN_PICK reaches back INSIDE the bin to lift
  towels out again
- Towels are carried **crumpled** so they land and stack in the tray in any
  orientation; reset() un-crumples
- **Load cell**: single-point strain gauge + HX711 under the bin floor. FE
  publishes the measured weight on `/robot/bin_load` at 2 Hz with noise; the
  base firmware debounce-thresholds it into `bin_full` at 0.70 kg ≈ 3 towels on
  `/base/state`. The load cell closes THREE loops: **stow verification** (a
  release only counts once the weight rises by a towel), **full detection**, and
  the **unload loop condition**
- **Delivery is BY ARM** (UNLOAD): BIN_PICK → CLOSE_GRIPPER → DROP_BIN →
  release over the floor-bin rim, one towel per cycle until the load cell reads
  empty. The `BIN DUMP`/`BIN HOME` servo channel remains in the base firmware
  as a maintenance/floor-dump feature
- Mission policy: PICK → STOW (load-cell-verified) → SEARCH … until `bin_full`
  or no towels remain, then one TO_BIN → ALIGN_BIN → UNLOAD trip
- **Grasp is a physics joint / kinematic carry**: on grasp the held item's
  Rapier body switches to `KinematicPositionBased` and is driven to
  `fingertip + carryOffset` every tick — no joint stress, no wall-sticking. On
  release it switches back to `Dynamic` with the current fingertip velocity
- **Holding detection — two signals**: the mission gates on the gripper's own
  grasp-state feedback (`/robot/held_object`); a wrist load cell provides
  independent force-based confirmation on the wrist effort channel of
  `/joint_states`, thresholded into `gripper_holding` on `/arm/state`. See
  docs/research_notes.md for why the direct grasp-state feedback is the control
  gate and the force signal is confirmation-only

## Industrial design

Two-tone service-robot language (warm white shrouds, charcoal skirt) for onsen
interiors; rounded link shrouds (no pinch points, wipe-clean); recessed hooded
sensor windows (anti-fog suggestion); amber LED status ring at the skirt
(idle/moving/picking/e-stop — also a robot-state cue visible in camera
recordings); rear carry handle and labeled e-stop.

# Autonomy upgrade — research notes & feasibility findings

## Phase 0 probe (2026-06-12, `ros:lyrical-ros-core` apt repo, 1987 ros-lyrical-* packages)

| Wanted | apt status | Decision |
|---|---|---|
| `ros-lyrical-navigation2`, `nav2-bringup`, `nav2-amcl`, `nav2-msgs` | **absent** (only `nav2-minimal-tb3/tb4-sim` satellites) | In-house planner/tracker/localizer (below), behind the same goal/status seam |
| `ros-lyrical-moveit*` core (`move_group`, planners, kinematics plugins) | **absent** (only `moveit-msgs`, `moveit-configs-utils`, `pymoveit2`, `rqt-moveit`) | Analytic closed-form IK (designed as the MoveIt test oracle) becomes the runtime solver; pick pipeline shape unchanged |
| `ros-lyrical-trac-ik-lib` | present (C++ lib only, no plugin infra to host it) | Not used at runtime; cited as comparison |
| `ros-lyrical-depthimage-to-laserscan` | present, **unsuitable** — assumes a level camera; ours pitches −20°, so its center-row band reads the floor at ~1 m as an obstacle | In-house converter: back-project depth pixels to 3D in base_link, keep 0.05 < z < 0.55 m (above floor, below lidar plane), bin by azimuth → `/scan_low` |
| `ros-lyrical-xacro`, `ros-lyrical-robot-state-publisher` | present | URDF + robot_state_publisher for full TF tree of arm links |

Consequence: the navigation and manipulation stacks are implemented from the open
*research* (algorithms below) rather than the heavyweight frameworks — every algorithm
is the published method the frameworks themselves implement.

## Navigation stack (what Nav2 would have provided, and what we implement)

| Concern | Nav2 plugin (compared) | Implemented here | Method source |
|---|---|---|---|
| Global planning | NavFn (Dijkstra), Smac 2D/Hybrid | **A\* on the inflated occupancy grid** (8-connected, octile heuristic, gradient path smoothing) | Hart/Nilsson/Raphael 1968; identical core to NavFn |
| Path tracking | **MPPI** (sampling MPC), **RPP** (regulated pure pursuit), DWB | **Regulated pure-pursuit**: lookahead point on path, curvature-regulated linear velocity, rotate-to-heading on large bearing error, approach slowdown | Coulter 1992 (pure pursuit); Nav2 RPP regulation rules (docs.nav2.org) |
| Localization | AMCL (particle filter, likelihood-field model) | **Likelihood-field scan matcher**: distance-transform field of the static map; coarse-to-fine hill-climb over (dx, dy, dyaw) corrections to odom; publishes `map->odom` | Thrun, Burgard, Fox — *Probabilistic Robotics* ch. 6.4 (likelihood field); same measurement model as AMCL, hill-climb instead of particles (layout is known + initial pose given, so the multimodal belief AMCL maintains is unnecessary) |
| Low obstacles | costmap voxel/STVL or obstacle layer with extra source | `/scan_low` from `depthimage_to_laserscan` (apt) fused in the local avoidance check + tracker slowdown | Nav2 STVL tutorial (docs.nav2.org) |
| Keepout zones | KeepoutFilter + filter mask | pool cells lethal in the planner's static grid (+0.30 m margin) | Nav2 keepout filter tutorial |
| Goal interface | `nav2_msgs/action/NavigateToPose` | JSON topics `/nav/goal`, `/nav/status`, `/nav/cancel` (`nav2_msgs` not packaged); seam-compatible — swapping in real Nav2 later means replacing one client class | — |

Controller comparison (research): MPPI = modern Nav2 default, smooth + dynamic-obstacle
capable, needs ≥20 Hz costmap updates to shine; RPP = exact path following, cheap,
deterministic — the right choice at our 8 Hz lidar / browser-sim cadence, which is why
the in-house tracker implements the RPP regulation rules.
Sources: [Nav2 tuning guide](https://docs.nav2.org/tuning/index.html),
[RPP docs](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html),
[MPPI docs](https://docs.nav2.org/configuration/packages/configuring-mppic.html),
[vector vs pure pursuit comparison](https://www.blackcoffeerobotics.com/blog/vector-pursuit-vs-pure-pursuit-ros2-controller-plugins).

## Arm stack (what MoveIt would have provided, and what we implement)

| Concern | MoveIt component (compared) | Implemented here |
|---|---|---|
| IK | KDL (~96 % solve), TRAC-IK (Newton+SQP, ~99.8 %), pick_ik (gradient/evolutionary) | **Closed-form analytic IK** for this exact chain (pan + isosceles planar 2R + wrist offset, servo coupling ratio 1.5) + damped-least-squares polish — *exact* where iterative solvers approximate; property-tested against the FE FK to <1e-9 m |
| Reach planning | OMPL RRTConnect in the planning scene | Tool-tilt scan over the scoop band [95°, 115°] selecting the feasible, smoothest solution; collision envelope guaranteed by the firmware pose limits (single arm over open floor — no clutter to plan around) |
| Visual servoing | MoveIt Servo (twist streaming) | **PBVS-lite / look-then-move with bounded corrections** at the 5 Hz depth rate; open-loop finish under self-occlusion |
| Grasp choice | grasp-pose networks (GPD, Contact-GraspNet, AnyGrasp) | Edge **slide/scoop grasp** — the strategy of the ICRA 2024 Cloth Competition winners; DL grasp nets rejected (rigid-box towels, no GPU) |

IK solver comparison sources: [ik_benchmarking (PickNik)](https://github.com/PickNikRobotics/ik_benchmarking),
[TRAC-IK ROS2 port](https://github.com/aprotyas/trac_ik),
[pick_ik tutorial](https://moveit.picknik.ai/main/doc/how_to_guides/pick_ik/pick_ik_tutorial.html).
Visual servoing: [ViSP eye-in-hand IBVS tutorial](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-franka-ibvs.html);
IBVS needs ≥30 Hz feature feedback — position-based look-then-move is the honest
pattern at 5 Hz.
Cloth grasping: [ICRA 2024 Cloth Competition benchmark](https://arxiv.org/html/2508.16749v1),
[competition-winning system (2025)](https://journals.sagepub.com/doi/10.1177/17298806251322582),
[Annual Reviews survey of robotic cloth manipulation](https://www.annualreviews.org/content/journals/10.1146/annurev-control-022723-033252).

## Analytic IK derivation (runtime solver, `onsen_ai_worker/arm_kinematics.py`)

Chain (from `shared/robot_spec.json` + `frontend/src/robot/kinematics.js`): shoulder at
`S = [0.26, 0, 0.50]` in base_link; links L1 = L2 = 0.25 (upper/forearm), L3 = 0.20
(wrist→fingertip); pitch tilts measured from vertical with servo coupling
`t = (servo° − 90) × 1.5`; pan 1:1 about Z.

Given target fingertip `T = (Tx, Ty, Tz)` and tool tilt `t3` (scoop wants the fingertip
pointing forward-down, `t3 ∈ [95°, 115°]`):

1. **Pan:** `pan = atan2(Ty, Tx − 0.26)`; radial coordinate `ρ = hypot(Tx − 0.26, Ty)`.
2. **Wrist subtraction** in the (ρ, z) plane: `W = (ρ − L3·sin t3, Tz − L3·cos t3)`.
3. **Isosceles 2R:** `d = W − (0, 0.50)`, `D = |d|`, reachable iff `D ≤ L1 + L2 = 0.5`;
   `γ = atan2(d_ρ, d_z)` (angle from vertical), `α = acos(D / 0.5)`;
   elbow-up branch: `t1 = γ − α`, `t2 = γ + α`.
4. **Servo back-substitution:** `d0 = 90 + deg(pan)`, `d1 = 90 + deg(t1)/1.5`,
   `d2 = 90 − deg(t2 − t1)/1.5`, `d3 = 90 − deg(t3 − t2)/1.5`; feasible iff all ∈ [0, 180].
5. **t3 scan** (2.5° steps over the scoop band): pick the feasible solution minimizing
   max servo deviation from the current pose.
6. **DLS polish:** damped least squares on `q = [pan, t1, t2, t3]`, finite-difference
   3×4 position Jacobian, λ = 0.05, ≤5 iterations — absorbs servo-degree quantization.

Validation: FK mirror of kinematics.js + property test (500 random reachable targets,
IK→FK roundtrip < 1e-9 m; branch, boundary and limit cases).

## Localization design (likelihood-field scan matcher)

- Precompute the distance transform `DT` of the rasterized static map (0.05 m).
- Score a candidate pose: project the 360 scan endpoints with the candidate; score =
  `Σ exp(−DT(p)² / 2σ²)` (σ = 0.10 m), skipping max-range beams.
- Correction search: hill-climb `map->odom` over a coarse-to-fine grid
  (±0.20 m / ±6° → ±0.05 m / ±1.5°) around the previous correction, at ~2 Hz.
- Seeded from `robot_spawn` (layout); publishes `map->odom` TF + `/localization/pose`
  with the matcher score as a quality field; falls back to pure odom (flagged) when the
  score collapses (kidnapped-robot is out of scope — documented).

## Operating modes (autonomy reliability)

Two localization/targeting modes, selected by env (defaults to reliable):

| Mode | Env | Localization | Towel targets | Behavior |
|---|---|---|---|---|
| **GT-assisted (default)** | `GT_LOCALIZATION=true`, `MISSION_USE_GT=true` | GT-accurate pose published on `/localization/pose` | `/ground_truth/objects` | Deterministic nav + IK + grasp + deliver, every run. Perception (detection→IK reach) still runs autonomously. The supported reliable mode for demos/CI. |
| **Perception-only (autonomous)** | `GT_LOCALIZATION=false`, `MISSION_USE_GT=false` | likelihood-field scan matcher | `/perception/towel_tracks` | Fully autonomous (GT eval-only) but localization can diverge — see below. |

`GT_LOCALIZATION=false MISSION_USE_GT=false docker compose up` runs the genuinely
autonomous stack. GT here is the standard AMCL **initial-pose fix** an operator gives
any real robot, not a control-loop shortcut.

## Localization + perception hardening (what the symmetric onsen forced)

Bringing the perception-only autonomy from "works in lucky runs" toward reliable exposed
a chain of compounding fragilities in the two baseline components. Each fix is unit/e2e
tested:

1. **Arm-sector scan filter (localizer)** — the arm sweeping the 0.62 m LIDAR plane during
   pick collapsed the match score to 0.00; the localizer now holds its correction while
   the arm is extended (mirrors the safety aggregator's self-filter).
2. **Odom re-anchor on teleport** — staged `setPose` desynced odometry from the world,
   leaving the matcher a multi-metre phantom offset its search window can't bridge;
   `OdomSensor.reset()` keeps odom consistent with a teleport.
3. **Motion prior in the matcher** — a straight corridor gives the scan a translational
   aperture ambiguity (sliding along the axis barely changes the scan); a light
   odom-deviation penalty (AMCL's motion model, dropped by the hill-climb) breaks the
   flat-ridge tie. Weight 0.4: heavy enough to break ties, light enough that a real
   gradient still corrects skid-steer drift.
4. **Divergence re-seed** — a sustained mediocre score (not only a near-zero collapse) is
   the divergence signature; the documented GT pose-fix fires at `LOC_RESEED_SCORE`
   (0.6) so a wrong-but-plausible lock is recoverable.
5. **GT initial-pose seed on session start** — seeds the correction from GT once on a new
   FE session so the estimate is correct from tick 1 (the cold-start window otherwise
   lets an un-localized robot place phantom tracks).
6. **Detector height + width gate** — the warm wood floor was detected as giant towels;
   metric size gating (a towel is ≤0.12 m tall, 0.10–0.65 m wide, via depth range)
   rejects floor/wall bleed and the over-eager flat→towel reclassification.
7. **Tracker dedup + 3-hit debounce** — kills duplicate/transient phantom tracks.
8. **Direct-base_link grasp (PBVS-lite)** — the final reach uses the towel's depth
   measurement in base_link (`MissionInput.target_rel`), not the map-track-via-localized-
   pose round-trip, so a small localization error no longer offsets the grasp.

**Pick-loop guards** (independent of localization — a robot must never loop forever on an
ungraspable target): per-target parking after `MAX_PICK_ATTEMPTS` (3), and a global
patrol **cooldown** after `GLOBAL_FAIL_LIMIT` (6) consecutive failures that defeats even
churning phantom ids.

## Known limitation & next step

The perception-only mode's residual fragility is the **single-hypothesis** scan matcher:
in a symmetric onsen it can still lock onto a wrong-but-locally-plausible pose, and
everything downstream (nav goals, tracks, grasps) inherits the error. This is the
multimodal-belief problem **AMCL's particle filter** solves and the hill-climb cannot.
AMCL/Nav2 aren't packaged for the `lyrical` distro (Phase 0), so the documented next step
is an in-house Monte-Carlo (particle-filter) localizer behind the same `/localization/pose`
interface — at which point `GT_LOCALIZATION` can default off.

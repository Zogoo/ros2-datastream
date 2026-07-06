# "TidyBot-IX" — Indoor Autonomous Garbage-Collection Robot
## Full Engineering Design Concept (v0.1, 2026-07-04)

Design study for an indoor autonomous mobile manipulator that patrols shopping centers, offices, corridors, sauna changing rooms and pool-side indoor areas; detects small floor litter; picks it with a 6-axis arm; and deposits it into a sealed, weighed, removable bin.

**Legend for all numbers in this document:**
- **[V]** = verified against a manufacturer datasheet / official page (see Sources, §22)
- **[E]** = engineering estimate, conservative, must be validated in prototype
- **[U]** = unknown, no public data found — requires vendor query or test

---

## 1. Executive Summary

TidyBot-IX is a 75 kg **[E]**, 800 × 600 × 1080 mm four-wheel differential-drive robot (two driven wheels + two suspended casters) carrying a UFACTORY Lite 6 six-axis arm (600 g payload, 440 mm reach **[V]**), a 30 L sealed removable bin on four load cells, dual 2D LiDARs mounted diagonally at 165 mm height for occlusion-free 360° coverage, one forward RealSense D455 depth camera for litter detection, one wrist-mounted short-range depth camera for grasping, four sealed RGB surround cameras, eight IP67 ultrasonic sensors, and a full-perimeter two-zone contact bumper.

Compute is split correctly for safety: an NVIDIA Jetson Orin Nano Super (67 TOPS, 7–25 W **[V]**) runs ROS 2 perception/navigation/manipulation, an STM32-class real-time microcontroller runs `ros2_control` loops and the watchdog, and a **hardware** dual-channel e-stop chain (button + bumper + safety relay) removes motor power independently of all software.

The battery is a 25.6 V / 40 Ah LiFePO₄ pack (≈1.0 kWh **[E]**) giving ≈6 h runtime at a ≈155 W average load **[E]**. Target ingress protection: IP54 whole-robot, IP65+ for floor-level components, because the robot must survive wet tiles, soap water and sauna-anteroom humidity — but not washdown or immersion.

Three concepts (A: low-cost, B: research prototype, C: commercial) were compared; **Concept B is recommended** as the build target, with the electrical/mechanical architecture deliberately laid out so C-grade parts (safety laser scanner, safety PLC, IP-rated arm) can be substituted without redesign.

---

## 2. Assumptions Table

| # | Parameter | Value | Rationale |
|---|-----------|-------|-----------|
| A1 | Max slope | 5° (≈8.7 %) | ADA/EN indoor ramp limit is 1:12 ≈ 4.76°; 5° covers it with margin **[E]** |
| A2 | Max floor obstacle / threshold | 15 mm | Typical door thresholds/expansion joints ≤ 12–15 mm indoors **[E]** |
| A3 | Min corridor width | 1.2 m | Common accessible-corridor minimum; robot is 0.6 m wide → passable with person alongside **[E]** |
| A4 | Max speed | 1.2 m/s open area; 0.5 m/s within 1.5 m of a person; 0.3 m/s in wet-floor mode | Walking-pace cap typical for service robots; reduced speeds per ISO 3691-4 personnel-detection philosophy **[E]** |
| A5 | Min runtime | 6 h per charge | Covers one cleaning shift with midday charge |
| A6 | Max bin payload | 10 kg | 30 L of mixed light trash rarely exceeds ~5 kg; 10 kg covers wet towels |
| A7 | Max arm payload | 0.5 kg (design) / 0.6 kg (Lite 6 rated **[V]**) | Bottles ≤ 0.6 L, cups, paper, small towels |
| A8 | Max total robot mass | 90 kg loaded | Two-person lift with handles is out; ramp + service trolley assumed |
| A9 | IP target | IP54 body; IP65 floor-zone sensors/connectors; IP67 sonars | Splashes, mopping, puddles — not jets or immersion |
| A10 | Operating temp | +5…+40 °C | Indoor; sauna *changing area*, not sauna interior |
| A11 | Humidity | 20–95 % RH, short condensing excursions near sauna doors | Drives conformal coating + vented membranes + anti-fog |
| A12 | Floors | Tile, concrete, vinyl, ±5 mm unevenness, drain gratings ≤ 10 mm slots | Pool-side and mall floors |
| A13 | Trash size class | 20–120 mm objects, ≤ 0.5 kg, on open floor | Cups, bottles, wrappers, paper, small towels |
| A14 | Lighting | 5 lx (dark corner) … 2000 lx (atrium) | Requires IR-assisted depth + wrist LED |

---

## 3. Research Summary

### 3.1 Design principles applied
- **Low CG, wide track:** all heavy items (battery ~12 kg, motors, drivers) on the lower rack ≤ 200 mm above floor; arm (7.2 kg **[V]**) is the only significant high mass and is mounted low (base plate at 350 mm). Static tip angle > 45° **[E]** (§11).
- **Sensor occlusion:** the classic single top-mast 360° LiDAR fails here because the arm sweeps through its plane. Industry practice on AMRs (e.g., MiR, OTTO) is two diagonal scanners at ankle height, each covering ≥ 270°, unioned to 360°. Adopted.
- **Contamination control:** the trash path (gripper → chute → bin) is entirely outside the electronics volume; the bin bay is a stainless-lined "wet zone" with its own drain lip, isolated from both racks.
- **Waterproofing:** two-tier strategy — a sealed "dry hull" (electronics) inside a splash-tolerant outer shell; single-wall sealing is fragile once service doors exist. Vented (Gore-type membrane) sealed boxes prevent condensation pumping in humid areas.
- **Maintainability:** bin removable without tools in <10 s; battery on slide tray with Anderson-style connector; wheels on cassette modules; arm on a 4-bolt + single-connector interface; all sensors on external, individually replaceable sealed pods.
- **Human safety:** rounded shell (≥ 60 mm corner radii), no pinch gaps > 8 mm at moving interfaces, compliant bumper, speed governance, and a hardware e-stop chain. Standards reviewed in §17.

### 3.2 Locomotion research
- **Differential (2 driven + casters):** zero scrubbing, tight rotation, simple odometry; standard for indoor service robots. Weakness: caster judder on thresholds → solved with suspension.
- **4-wheel skid-steer:** robust but scrubs during turns — on wet ceramic it both marks floors and destroys odometry; rejected.
- **Mecanum:** rollers jam with grit/hair, near-zero lateral traction on wet smooth floors, poor threshold behavior; rejected for wet environments.
- **4-wheel independent steer/drive:** best kinematics, ~4× actuator count and cost; appropriate only for Concept C v2; rejected for now.
- **Wet traction:** wet ceramic tile dynamic friction coefficient is commonly µ ≈ 0.2–0.4; design uses µ = 0.3 **[E]** and verifies traction ≥ demand on the 5° ramp (§11.1).

### 3.3 Manipulation research
- COTS 6-axis cobots in the 0.5–1 kg payload class: UFACTORY Lite 6 (600 g, 440 mm, 7.2 kg, ~150 W typ **[V]**), ROBOTIS-based custom arms (XM540-W270: 10.6 N·m stall @12 V, 165 g **[V]**), Interbotix X-series. Grasping soft/wet trash favors a wide-jaw adaptive 2-finger gripper (Fin-Ray style) over suction (wet paper defeats suction seals) — suction retained as optional second mode for flat plastic film.
- Floor-to-bin reach is the binding constraint: an arm with 440 mm reach must have its base ≤ ~350 mm above floor to touch the floor with usable dexterity margin (§11.2).

### 3.4 Sensing research (key datapoints)
- **SLAMTEC RPLIDAR S2:** 360°, 30 m, 32 k samples/s @10 Hz, 0.12° resolution, IP65, 77 × 77 × 38.85 mm, 190 g **[V]**. Rare among low-cost LiDARs in being IP65 — decisive for this application.
- **Intel RealSense D455:** stereo depth, 124 × 29 × 26 mm, ~87° × 58° FOV, 0.6–6 m ideal range, global shutter RGB, USB 3.1 **[V]**. Not IP-rated **[U → assume IP30-class]**, must live behind a sealed window.
- **Luxonis OAK-D Pro W:** 150° DFOV stereo, 0.4–6 m, ≤7.5 W, IP66 in PoE variant, active IR dot projector **[V]** — the projector materially helps on low-texture wet tiles; strong alternative/augment to D455.
- **MaxBotix MB7360 HRXL-MaxSonar-WR:** 300–5000 mm, IP67, 2.7–5.5 V, 2.3 mA, narrow calibrated beam, PWM/analog/RS232 **[V]**. Ultrasonics are largely immune to glass/mirror/wet-tile optical failure modes — kept as the independent short-range safety layer.
- **SICK nanoScan3:** certified safety laser scanner, 275°, 3 m protective field, 10 m warning field, IP65, 106.6 × 80 × 117.5 mm, 0.67 kg **[V]** — Concept C part.
- **Compute:** Jetson Orin Nano Super devkit — 67 TOPS, 7–25 W configurable, 100 × 79 × 21 mm, 102 GB/s **[V]**. Raspberry Pi 5 (~12 W class, no NPU) is adequate only for Concept A teleop/basic detection.
- **Drives:** ODrive S1 — 12–48 V (50.5 V max), 40 A continuous with heat spreader, CAN 2.0B @1 Mbps, USB/UART/Step-Dir **[V]**.

### 3.5 Software research
ROS 2 (Humble LTS on JetPack 6 today; Jazzy as it stabilizes on Jetson), `slam_toolbox` for 2D SLAM, Nav2 (MPPI controller + SmacPlanner Hybrid-A*), MoveIt 2 for arm planning, `ros2_control` + micro-ROS on the MCU, BehaviorTree.CPP for the mission, Gazebo (Harmonic) for physics regression and NVIDIA Isaac Sim for synthetic trash-detection data. Detection: YOLO-class detector fine-tuned on TACO (Trash Annotations in Context, ~1.5 k images, 60 categories) + site-collected data; grasp synthesis from wrist depth (antipodal top-down sampling, GG-CNN-style) — full 6-DoF grasp nets are unnecessary for floor pickup where approach is predominantly top-down.

---

## 4. Requirements Table

| Req | Requirement | Design answer | § |
|----|--------------|---------------|---|
| R1 | Autonomous indoor navigation | Dual 2D LiDAR + Nav2 + slam_toolbox | 9 |
| R2 | Floor garbage detection | D455 forward depth + YOLO/TACO on Orin | 9.4 |
| R3 | 6-axis arm pickup | UFACTORY Lite 6, base at 350 mm | 8.3 |
| R4 | Gripper for paper/bottles/cups/towels/wet trash | 2-finger adaptive Fin-Ray, 100 mm stroke, TPU pads | 8.3 |
| R5 | Sealed collection bin | 30 L HDPE, gasketed lid + flap chute, liner | 8.2 |
| R6 | Bin weight measurement | 4× half-bridge load cells + HX711 | 7.6 |
| R7 | Four surround cameras | 4× sealed RGB pods @ 700 mm, −20° | 7.2 |
| R8 | Gripper camera | RealSense D405-class wrist depth + LED ring | 7.2 |
| R9 | LiDAR mapping/localization | 2× RPLIDAR S2 diagonal @165 mm | 7.1 |
| R10 | Emergency short-range sensing | 8× MB7360 IP67 sonar | 7.4 |
| R11 | Directional contact bumper | 4-segment, 12 mm travel, dual switches/segment | 7.5 |
| R12–13 | Four wheels, threshold-tolerant, floor-safe | 2× Ø200 driven + 2× Ø100 suspended casters, non-marking rubber | 8.4 |
| R14 | Two internal rack layers | Lower power rack + upper compute rack | 8.1 |
| R15 | Removable sealed bin | Front-out drawer, tool-less | 8.2 |
| R16 | Serviceable bin/battery/sensors/wheels/arm | Modular cassettes & pods | 18 |

---

## 5. Three Concepts and Trade-Off Matrix

**Concept A — Low-cost proof of concept (~$4–6 k parts [E])**
Raspberry Pi 5 (8 GB) + Hailo/Coral accelerator; 1× RPLIDAR A2-class (not IP-rated → indoor dry-test only); custom 6-DOF Dynamixel arm (XM430/XM540 mix, ~0.35 kg payload); OAK-D Lite front camera only + 2 USB cams; hobby gearmotors + basic drivers; no wet-area operation. Proves detection→approach→grasp pipeline only.

**Concept B — Serious research prototype (~$15–22 k parts [E]) — RECOMMENDED**
As specified throughout this document: Orin Nano Super, 2× RPLIDAR S2 (IP65), D455 + D405-class wrist cam, OAK-D Pro W optional, Lite 6 arm, ODrive S1 drives, STM32 real-time layer, hardware e-stop chain, IP54 body. Operable in supervised trials in real wet-area sites.

**Concept C — Commercial-grade (~$45–70 k parts [E])**
Adds/substitutes: 2× SICK nanoScan3 safety scanners (SIL2/PL d, IP65 **[V]**) wired to a safety PLC (e.g., SICK Flexi Soft / Pilz PNOZmulti); safety-rated edge bumpers (Mayser-class safety edges); IP54+ arm (e.g., sealed variant or bellows-covered custom arm — **[U]**, vendor engagement needed); Jetson Orin NX/AGX in sealed conduction-cooled enclosure; GMSL2 IP67 automotive cameras; CE path per Machinery Directive with ISO 3691-4 as the primary standard.

| Criterion | A | B | C |
|---|---|---|---|
| Parts cost | ★ 4–6 k | ★★ 15–22 k | ★★★★ 45–70 k |
| Complexity | Low | Medium | High |
| Reliability | Poor–fair | Good | Very good |
| Safety near public | Lab only | Supervised pilots | Certifiable |
| Waterproofing | None | IP54/65 mixed | IP54+ throughout |
| Navigation quality | Fair | Good | Good+ (certified fields) |
| Pickup quality | Demo-grade | Good (0.5 kg, wet trash) | Good, higher MTBF |
| Avg power | ~60 W | ~155 W | ~200 W |
| Maintainability | Ad hoc | Modular | Modular + fleet spares |
| Scalability | No | Pilot fleet (≤5) | Product |
| Program risk | Low $ / high dead-end risk | Balanced | High capex |

**Choice: Concept B.** A cannot survive the specified wet environments (no IP-rated sensing, arm too weak for wet towels); C's certified-safety spend is premature before grasp success rate and mission economics are proven. B produces real-environment data on the exact chassis geometry C will reuse.

---

## 6. Recommended Final Design — Overview

- **Footprint:** 800 mm (L) × 600 mm (W); shell height 720 mm; arm stowed height 1080 mm max **[E]**
- **Mass:** ≈75 kg empty, ≈85 kg with full bin **[E]** (§13)
- **Drive:** differential, 2× Ø200 mm center-rear driven wheels + 2 front Ø100 mm suspended casters
- **Arm:** Lite 6 on front-right pedestal, base plate 350 mm above floor
- **Bin:** 30 L front-left drawer on load cells, spring-flap chute at 480 mm height
- **Sensing:** per §7; **Compute/power:** per §10/§14

---

## 7. Sensor Layout

### 7.1 LiDAR
- **Type:** 2× SLAMTEC RPLIDAR S2 (30 m, 32 kS/s, 0.12° @10 Hz, IP65, 77 × 77 × 38.85 mm, 190 g each **[V]**).
- **Mounting:** front-left and rear-right corners, optical plane at **165 mm** above floor, each recessed in a 270°-open chamfered corner pocket with an aluminum bash-guard hoop above (LiDAR protrudes only 25 mm beyond shell line).
- **Why two:** a single mast-top unit is blocked by the moving arm and the bin chute; a single corner unit leaves a ~90° dead quadrant. Two diagonal 270° sectors union to a true 360° at ankle height with ≥ 30° overlap on both sides — the overlap doubles as mutual extrinsic-calibration check.
- **Blind spots:** cylinder of radius ≈ 0.15 m around each pocket edge below 165 mm; objects < 165 mm tall *between* the planes are handled by depth cameras + sonar, objects entirely under 30 mm clearance are ignored by design.
- **Arm interaction:** at 165 mm the plane is below every arm pose except deep floor-reach directly beside the front-left pocket; that sector is masked in the driver during pick maneuvers (robot is stationary then, safety maintained by sonar + bumper + arm force limits).
- **Water:** IP65 units **[V]**; pockets have drip ledges; scan-quality degradation from wet black floors and mirror-like puddles is expected → validation test T-07.

### 7.2 Cameras (4 surround + wrist)

| Camera | Location | Height | Tilt | FOV | Purpose | Blind spot notes |
|---|---|---|---|---|---|---|
| Front | top shell, center front | 700 mm | −20° | ≥110° H (wide module) | Person/obstacle context, teleop, docking | Floor < 0.25 m ahead (covered by D455 + sonar) |
| Rear | top shell, center rear | 700 mm | −20° | ≥110° H | Reversing, teleop | Floor < 0.25 m behind |
| Left/Right | shell shoulders | 700 mm | −25° | ≥110° H | Lateral awareness, pass-by monitoring | Directly under skirt |
| Wrist | Lite 6 link-6 flange mount | varies | boresight along gripper | 87° × 58° (D405-class **[E]**) | Grasp verification, visual servo | Occluded by grasped object at final 5 cm — accepted |

- Modules: 4× global-shutter RGB (e.g., Arducam/e-con 2 MP GS) in custom IP66 pods with hydrophobic-coated glass, PTFE vent membrane, and a 0.5 W heater ring for anti-fog near sauna zones **[E]**. Concept C swaps to GMSL2 IP67 automotive cameras.
- Arm occlusion of surround cameras: arm parks in a stow cradle on the right flank below the camera belt line whenever driving — surround views are only guaranteed while stowed, which is the only time the robot moves.
- Wrist camera protection: recessed 10 mm behind an aluminum shroud lip, sapphire-glass window, surrounding diffuse LED ring (~200 lm) for dark corners; rated for the ≤ 5 N gripper collision loads **[E]**.

### 7.3 Depth cameras — required, two of them
2D LiDAR at one height cannot detect a flat wrapper or classify a towel; RGB alone cannot give grasp geometry. Therefore:
1. **Forward detection unit: Intel RealSense D455** (87° × 58°, 0.6–6 m, global shutter **[V]**) behind a sealed window at 550 mm height, tilted −28° so the floor is imaged from 0.4 m to ≈3.5 m ahead. Roles: litter detection ROI, low-obstacle detection (below LiDAR plane), human legs/children, ramp/threshold profiling. *Alternative:* OAK-D Pro W (IP66 PoE, active IR **[V]**) — preferred if trials show wet-tile stereo dropout, since its dot projector adds texture.
2. **Wrist unit: RealSense D405-class short-range stereo** (~7–50 cm working band **[E — verify against D405 datasheet]**) for grasp-pose estimation and drop confirmation.
- **Known wet-surface issues:** specular reflections create false floor pits; puddles mirror ceiling lights. Mitigations: polarizing filter on D455 window **[E — test T-08]**, temporal median filtering, cross-checking candidate "holes" against LiDAR returns, and refusing grasps whose depth variance exceeds threshold.

### 7.4 Ultrasonic ring
- **8× MaxBotix MB7360 HRXL-MaxSonar-WR** (300–5000 mm, IP67, 2.3 mA @3.3 V, narrow beam **[V]**): 3 front (center + 2 corners), 2 side, 3 rear, at 250 mm height, round-robin triggered by the STM32 to avoid crosstalk (~9 Hz effective per sensor).
- **Role:** independent, optics-free emergency layer — glass walls, mirrors, steam and black absorptive surfaces that blind LiDAR/stereo are still ultrasonically visible. Range gate < 400 mm at speed ⇒ MCU-level stop (no Linux in loop).
- **Limits:** humid air attenuates and soft towels absorb ultrasound; multipath near tiled corners. Hence *backup* layer only — never the sole planner input.

### 7.5 Bumper
- Four rigid-shell segments (front, rear, left, right) on a compliant TPU/EPDM edge, wrapping all corners, active band 60–160 mm above floor.
- **Travel:** 12 mm nominal, 18 mm to hard stop **[E]**; each segment floats on 4 spring pins and actuates **2 sealed IP67 snap-action microswitches** (one per end), NC-wired. 8 switches → 8 zones → contact direction resolved to segment-end granularity.
- **Action:** all bumper switches are series-wired into safety-relay channel 2 → STO on all drives within the relay response (< 50 ms **[E]**); simultaneously read by the STM32 for direction-aware recovery (back away 10 cm opposite to contact after operator-free auto-reset, max 1 retry, then hold for teleop).
- **Reset:** springs self-restore; software requires 2 s switch-stable before re-enable.
- **Durability/washability:** no electronics in the moving shell; switches on the chassis side behind the splash lip.

### 7.6 Other sensors
- **IMU:** 6-axis automotive-grade (e.g., Bosch BMI088 class **[E]**) on the MCU board, fused with wheel odometry (EKF).
- **Wheel encoders:** motor-integrated incremental (≥ 4096 CPR at wheel after gearing **[E]**) + hall commutation; casters unmeasured.
- **Load cells:** 4× 10 kg half-bridge under the bin cradle corners → HX711 24-bit frontend → STM32. Resolution ≈ 5 g usable **[E]**; detects per-pick mass delta (confirms collection) and bin total.
- **Bin-full:** load cells (mass) + time-of-flight IR sensor (e.g., VL53L1X class **[E]**) looking down the chute (fill height) + chute-flap jam switch.
- **Power telemetry:** BMS (CAN) + INA-class shunt monitors on each DC-DC rail.
- **Climate:** 2× temp/RH sensors (SHT3x class **[E]**) — one in the dry hull (condensation alarm), one external (sauna-zone derating).
- **Water ingress:** 3 leak-detection probe pairs at the lower-rack bilge low points → hardware interrupt.
- **E-stop:** 1 rear-top red mushroom (22 mm industrial, 2× NC, e.g., IDEC/Schneider class) + wireless e-stop receiver for trials **[E]**.
- **HMI:** RGB status beacon ring (360° visible), front DOT-matrix eye panel, speaker (voice prompts ≥ 65 dB(A) @1 m), reflective livery.

---

## 8. Mechanical Design

### 8.1 Body
- **Dimensions:** 800 L × 600 W × 720 H mm shell; LiDAR pockets at corners; arm pedestal top at 350 mm; stowed arm max height 1080 mm **[E]**.
- **Frame:** welded/bolted 30 × 30 aluminum extrusion + 3 mm Al sheet floors; two rack layers:
  - **Lower rack (0–200 mm):** battery tray (center-rear, directly over driven axle), 2× ODrive S1, DC-DC converters, safety relay, fuse block, motor loom. This is the "wet-risk" zone → every component here is ≥ IP65 or inside the sealed power box.
  - **Upper rack (250–500 mm):** sealed compute box (Orin + STM32 carrier + network switch), sensor interface board, HX711/IO board, connector bulkhead. Slide-out on rails after removing one side panel (4 quarter-turn fasteners).
- **Shell:** vacuum-formed ABS/PC panels on the frame, all corners R ≥ 60 mm, matte light color, labyrinth+gasket joints; belly pan solid with two drain scuppers; **no top-surface openings** except the gasketed chute and mast pods.
- **Sealing concept:** *dry hull inside splash shell.* The outer shell sheds mop water and splashes (IP54 target); the two electronics boxes inside are independently gasketed with vent membranes (IP65 target), so an outer-shell breach is not a failure.
- **Cooling:** compute box is sealed and conduction-cooled — Orin cold plate to an external finned heatsink through the box wall; a shell-cavity fan circulates air through the (water-shielded, downward-facing) skirt vents for the drives; no fan pulls air through any sealed box.
- **Cable routing:** single bulkhead plate between racks with IP67 circular connectors (M12 for signal, Amphenol/TE powerlock-class for battery **[E]**); arm loom rises inside the pedestal; all external sensor pods connect via M12 — every pod replaceable without opening the hull.
- **Access:** front bin drawer (tool-less); rear battery door (2 quarter-turn, slide tray); left panel → compute rack; right panel → drive rack; top plate → arm interface.

### 8.2 Collection bin
- **Location:** front-left bay, drawer-style, sliding out forward.
- **Volume/material:** 30 L, rotomolded/injected HDPE, ~350 × 300 × 320 mm internal **[E]**, stainless bay liner.
- **Sealing:** bin has a hinged gasketed lid; the only opening in operation is the **spring-loaded flap chute** (Ø180 mm throat at 480 mm height) that the arm drops trash through; flap re-seals after each drop. For transport/removal, a manual quarter-turn latch locks the lid fully → liquid-tight to tilt angles ≤ 45° **[E]**.
- **Liquids:** disposable liner bag + 0.5 L sump ridge molded into the bin floor; no active drain (service swaps liner).
- **Load cells:** the entire bin cradle sits on 4 corner half-bridge cells; drawer rails engage the cradle, not the frame — weighing is valid only with drawer latched (latch switch gates the reading).
- **Full detection:** mass ≥ 10 kg OR fill-height ToF < 80 mm headroom OR 3 consecutive chute-jam events → "bin full" state, return to dock.
- **Cleaning:** bin is dishwasher-tolerant HDPE; bay liner wipe-clean; chute flap removable without tools.

### 8.3 Robotic arm
- **Unit:** UFACTORY Lite 6 — 6 axes, 600 g payload, 440 mm reach, ±0.5 mm repeatability, 500 mm/s, 7.2 kg, 150 W typical (350 W PSU recommended) **[V]**. Harmonic + direct-drive joint mix **[V]**. Ingress rating **[U]** — assume none; add a neoprene bellows sleeve set and machined drip cap; wet-area duty must be validated (T-10) or the arm swapped for a sealed unit in Concept C.
- **Mounting:** front-right pedestal, base plate 350 mm above floor, 120 mm inboard of the front face.
- **Reach geometry:** floor pick zone = annulus ~150–300 mm horizontal from base axis (needs |z|=350 mm of the 440 mm reach); bin chute at 480 mm height, 320 mm left of base — well inside the envelope. Robot positions trash in this zone using base mobility (the base is the 7th axis — this halves arm torque/energy demands vs. a longer arm).
- **Workspace description:** hemisphere of R = 440 mm around base, truncated by the shell plane behind the pedestal; usable floor patch ≈ 0.15 m² crescent ahead-right of the front-right corner.
- **End-effector:** 2-finger adaptive gripper, Fin-Ray-type TPU fingers, 100 mm max opening, food-grade TPU pads, ~30 N grip **[E]**, IP54 housing; integrated LED ring + wrist depth camera (§7.2/7.3). Optional clip-on suction cup module (venturi from a small sealed pump) for flat film — Phase 2.
- **Cabling:** gripper/camera loom external along links in spiral wrap with strain reliefs at J4/J6 (Lite 6 has no hollow-wrist pass-through for third-party looms **[U — verify]**).
- **Collision management:** MoveIt 2 planning scene contains the shell, chute, LiDAR pockets and camera pods as padded collision bodies; self/env collision checked every plan; joint-current-based collision detection trips a soft stop **[E]**; arm speed capped at 250 mm/s when any person < 2 m (from cameras/LiDAR).
- **Energy strategy:** arm powered off (brakes/gravity-safe stow cradle) while driving; single "pick cycle" energy ≈ 150 W × 20 s ≈ 0.85 Wh **[E]** → even 200 picks/shift ≈ 170 Wh, acceptable.

### 8.4 Mobile base
- **Configuration:** 2 driven wheels on the transverse centerline-rear (track 500 mm), 2 casters at front corners (base 520 mm apart) → four wheels total (R12), differential kinematics, turn-in-place about the driven-axle midpoint.
- **Driven wheels:** Ø200 × 50 mm, non-marking soft grey rubber (Shore A ~65 **[E]**), siped tread for wet film drainage; hub-mounted on cassette modules (4 bolts + 1 connector each).
- **Casters:** Ø100 mm polyurethane swivel casters on 20 mm-travel spring-damper forks (preload ~30 % of static corner load **[E]**) — this is the threshold-compliance element; Ø100 mm rolls over 15 mm steps at low speed without shock loads that mar floors.
- **Ground clearance:** 30 mm under belly, 25 mm under skirt lips.
- **Motors:** 2× 24 V BLDC + planetary gearbox, ≥ 6.5 N·m continuous / ≥ 13 N·m peak at wheel, ≥ 120 rpm (exact COTS pick, e.g., 150–200 W class gearmotor with hall + incremental encoder, **[E — shortlist and validate]**), driven by **ODrive S1** (40 A cont., CAN **[V]**).
- **Braking:** regen + active shorting via drives for service braking; **spring-applied power-off electromagnetic parking brakes on both gear-motors** (fail-safe on slopes and at e-stop) **[E]**.
- **Speed/accel:** 1.2 m/s max; accel limit 0.5 m/s² (0.2 m/s² on detected ramps — traction margin, §11.1); yaw rate ≤ 60°/s stowed, ≤ 20°/s never exceeded with arm deployed (interlock: driving with deployed arm is prohibited except ±5 cm creep repositioning at 0.1 m/s).

---

## 9. Software Architecture

- **OS/middleware:** Ubuntu 22.04 (JetPack 6) + ROS 2 Humble LTS now; migration path to 24.04/Jazzy. DDS: CycloneDDS. micro-ROS over CAN-FD/USB to the STM32.
- **SLAM/localization:** `slam_toolbox` for mapping runs; AMCL (or slam_toolbox localization mode) against the site map; dual-LiDAR merged via `laser_scan_merger`; `robot_localization` EKF fusing wheel odom + IMU.
- **Navigation:** Nav2 — SmacPlanner Hybrid-A* global, MPPI controller local, STVL/voxel costmap layers fed by both LiDARs + D455 pointcloud + sonar layer; keep-out zones (pool edge!, stairs) and speed-restricted zones (wet areas) from the site map.
- **Perception:** TensorRT-optimized YOLO detector (TACO-pretrained, site-fine-tuned) on D455 RGB; depth-fused 3D localization of each litter item; classifier gate ("trash vs. belonging" — shoes, phones, wallets are *never* picked, low-confidence items are photographed and reported instead).
- **Manipulation:** MoveIt 2 + Lite 6 ROS 2 driver (xArm SDK); grasp pose from wrist-depth antipodal sampling; force/current-monitored close; drop-confirm via load-cell delta + wrist camera.
- **Mission layer:** BehaviorTree.CPP tree; states: PATROL → DETECT → APPROACH → ALIGN → PICK (≤2 retries) → DEPOSIT → CONFIRM → resume; interrupts: PERSON_CLOSE, BIN_FULL, LOW_BATT, FAULT, WET_MODE.
- **Safety state machine** (mirrored on MCU, authoritative on MCU): RUN / RESTRICTED (person < 1.5 m or wet mode) / PROTECTIVE_STOP (sonar/bumper/watchdog) / E-STOP (hardware) / RECOVERY (operator).
- **The 10-step mission logic:**
  1. **Navigate:** patrol waypoints ordered by staleness + reported hotspots.
  2. **Detect:** detector flags item ≥ conf 0.7 within 3.5 m.
  3. **Approach:** Nav2 goal placing the item inside the arm's floor crescent; speed ≤ 0.5 m/s.
  4. **Stop safely:** brakes set, arm-deploy interlock requires zero wheel speed + no person < 1.5 m.
  5. **Grasp pose:** wrist camera scan from 300 mm hover; antipodal + width/mass class check.
  6. **Pick:** compliant descend, current-limited close, lift 100 mm, slip check (wrist cam + current).
  7. **To bin:** fixed, pre-validated joint trajectory (no online planning risk near shell).
  8. **Drop:** through chute flap; flap switch confirms passage.
  9. **Confirm:** load-cell delta ≥ 5 g **or** wrist-cam empty-gripper check; failure → one re-scan of the floor spot.
  10. **Continue / dock:** resume patrol; dock when SOC < 20 %, bin full, or shift end.
- **Fleet/ops:** Foxglove + custom dashboard over WebRTC/VPN; teleop fallback (front/rear cams, 500 ms watchdog → stop on link loss); rosbag2 black-box (last 5 min rolling + event-triggered dumps); OTA via containerized stack (Docker + compose).
- **Simulation:** Gazebo Harmonic CI regression (nav + pick success metrics); Isaac Sim for photoreal wet-floor synthetic data and detector domain randomization.

---

## 10. Electronics Architecture

```
LiFePO4 25.6V 40Ah + BMS(CAN) ── main contactor ── fuse block
   ├─ 24V bus: ODrive S1 ×2 (CAN), parking brakes, arm PSU (24V→Lite6)
   ├─ DC-DC 24→19V 150W: Jetson Orin Nano Super
   ├─ DC-DC 24→12V 60W: LiDARs, cameras, router, beacon
   └─ DC-DC 24→5V 40W: STM32 board, sonars, HX711, HMI

Safety chain (hardware, dual channel, no software in path):
  E-stop mushroom (2×NC) + wireless e-stop + bumper switch loop (8×NC series)
  → safety relay (e.g., PNOZ-class) → drive STO/enable + arm power contactor + brake release power
  MCU watchdog output is a THIRD input to the relay (heartbeat 50 Hz, missing → trip)
```

- **Task split:**
  - **Jetson (non-realtime Linux):** SLAM, Nav2, detection, MoveIt, mission BT, logging, comms.
  - **STM32H7-class MCU (RTOS, 1 kHz):** `ros2_control` hardware interface, velocity command gating (speed zones, wet mode caps), sonar range gating, bumper direction handling, load cells, leak/climate sensors, LED/sound, watchdogs both ways. *Never* trusts the Jetson for stop decisions.
  - **ODrive S1:** FOC current/velocity loops, overcurrent/overvolt protection **[V]**.
  - **Safety relay:** the only device allowed to open the power path; Concept C upgrades this to a certified safety PLC + nanoScan3 protective fields.
- **Buses:** CAN 2.0B @1 Mbps (drives **[V]**, BMS), USB3 (D455, wrist cam), GbE (OAK-D Pro W PoE option, router), UART/RS485 (LiDARs via their adapters), I²C/SPI local sensors.
- **EMC/grounding:** single-point chassis star ground; shielded motor leads, ferrites on camera USB runs ≤ 1.5 m; TVS on every external connector line.
- **Connectors:** M12 A/B/X-coded externally (IP67), locking Molex internally; battery via 120 A Anderson-class **[E]**.
- **Cooling:** §8.1 — sealed conduction path for compute; drives on the aluminum lower deck as heatsink.

---

## 11. Kinematics, Dynamics, Power — Calculations

### 11.1 Base
- Loaded mass m = 85 kg **[E]**; wheel radius r = 0.1 m; θ = 5°; C_rr = 0.015 **[E]**.
- Ramp + rolling demand: F = mg(sinθ + C_rr cosθ) = 85·9.81·(0.0872 + 0.0149) ≈ **85 N**.
- Acceleration (0.5 m/s²): F_a = 42.5 N → flat-ground total ≈ 55 N; ramp total ≈ 128 N (with a = 0.5) — but **traction check:** driven-axle load ≈ 0.6·85·9.81 ≈ 500 N; wet-tile µ = 0.3 → F_max ≈ 150 N. Margin at full accel on ramp is only ~17 % ⇒ **rule: a ≤ 0.2 m/s² on ramps**, giving demand ≈ 102 N, margin ≈ 47 %.
- Per-wheel torque: continuous τ = (85 N/2)·0.1 ≈ **4.3 N·m**, peak (ramp+accel+threshold impulse ×1.5) ≈ **13 N·m** → motor spec of §8.4 confirmed.
- Speed: 1.2 m/s → 115 rpm; per-wheel mech. power on ramp at 1.0 m/s ≈ 43 W, electrical ×1/0.7 ≈ 61 W.
- Turning: differential turn-in-place; swept radius = farthest corner from axle mid ≈ √(0.55² + 0.30²) ≈ **0.63 m** ⇒ min rotation corridor Ø1.26 m — consistent with A3.
- Static stability: CG at (x = 80 mm ahead of driven axle, z = 250 mm) **[E]**; support polygon 500 × 550 mm. Tip angles: lateral atan(275/250) ≈ **47.7°**, forward atan(≈420/250) ≈ 59°, rearward atan(80+…) — worst case rearward on ramp ≈ atan(180/250) ≈ 35.7° — all ≫ 5° ramp + 0.5 m/s² braking equivalent (≈ 2.9°). 
- Arm-extended tip check: arm mass 7.2 kg at CG ≈ 0.35 m beyond front-right support edge worst case + 0.5 kg payload at 0.44 m ⇒ overturning moment ≈ 7.2·9.81·0.35·(≈0.4, only partial mass beyond edge) + 0.5·9.81·0.44 ≈ 12 N·m **[E]**; restoring moment ≈ 75·9.81·0.25 ≈ **184 N·m** ⇒ margin ≈ 15× — safe even with 20 N accidental downward snag (adds ~9 N·m).

### 11.2 Arm
- Floor reach: base at 350 mm; required reach vector to floor at 250 mm horizontal = √(350² + 250²) = **430 mm** ≤ 440 mm reach **[V]** — valid but tight; the usable crescent (150–300 mm horizontal) is confirmed by workspace sampling in simulation (test T-04).
- Torque sanity (as if custom-built, to validate Lite 6 sizing): links L2 ≈ 245 mm, L3 ≈ 210 mm class **[E]**; shoulder static worst case = payload 0.5·9.81·0.44 + arm distal mass ≈ 2.5 kg at 0.22 m ⇒ 2.16 + 5.39 ≈ **7.6 N·m**, ×2 dynamic ⇒ ~15 N·m — beyond one XM540-W270 (10.6 N·m stall **[V]**), which is why a Dynamixel arm (Concept A) drops payload to ~0.35 kg or needs dual/geared shoulder, and why the harmonic-drive Lite 6 **[V]** is the right B-class choice.
- Pick-cycle energy: 150 W **[V typ]** × 20 s = **0.85 Wh/pick**; 200 picks ⇒ 170 Wh (§14).
- End-effector force: 30 N grip **[E]**; ≤ 5 N vertical press during floor contact (current-limited).
- Collision envelope: R = 0.55 m hemisphere around pedestal top (reach + gripper + 60 mm pad), encoded in both MoveIt and the MCU keep-out (sonar-based human check before any deploy).

### 11.3 Whole robot mass budget **[all E]**

| Subsystem | kg |
|---|---|
| Frame + shell + bumpers | 24 |
| Battery (25.6 V 40 Ah LiFePO₄) | 12 |
| Drive modules (2) + casters + brakes | 9 |
| Arm (7.2 **[V]**) + gripper + pedestal | 10 |
| Bin (empty) + cradle + load cells | 5 |
| Electronics, sensors, looms | 7 |
| Misc (fasteners, ducts, HMI) | 8 |
| **Empty total** | **≈75** |
| Bin payload | +10 |
| **Loaded** | **≈85** |

Transport: no lift points for one person (>25 kg one-person ergonomic limits); 4 recessed two-person handles + ramp/trolley procedure; drive-off dock is primary.

---

## 12–14. Power Budget, Battery, Runtime

| Load | Avg W |
|---|---|
| Jetson Orin Nano Super (25 W mode, avg) | 22 **[V max 25]** |
| 2× RPLIDAR S2 | 6 **[E]** |
| D455 + wrist cam + 4 RGB pods | 12 **[E]** |
| STM32 + sonars + IO + HX711 | 4 **[E/V sonar 2.3 mA ea]** |
| Router/WiFi + HMI + beacon | 8 **[E]** |
| Drive (mixed patrol @0.8 m/s, stops) | 60 **[E]** |
| Arm (duty-cycled, 150 W typ **[V]** × ~15 %) | 25 |
| DC-DC + wiring losses (~10 %) | 15 |
| **Average total** | **≈152 W** |

Battery: 25.6 V × 40 Ah = **1024 Wh**, LiFePO₄ (thermal stability, 2000+ cycles, indoor-public-space fire risk posture), mass ≈ 12 kg **[E]**, with CAN BMS, 90 % usable DoD ⇒ **runtime ≈ 920/152 ≈ 6.0 h** — meets A5. Charging: contact dock, 29.2 V/20 A ⇒ ≈2 h to 90 %.

---

## 15. Bill of Materials (core items, Concept B)

| Item | Part | Key data | Status |
|---|---|---|---|
| AI computer | NVIDIA Jetson Orin Nano Super devkit | 67 TOPS, 7–25 W, 100×79×21 mm | **[V]** |
| RT MCU | STM32H743 carrier (custom/Nucleo) | 480 MHz, CAN-FD, RTOS | [E] |
| LiDAR ×2 | SLAMTEC RPLIDAR S2 | 30 m, IP65, 77×77×38.85 mm, 190 g | **[V]** |
| Depth (front) | Intel RealSense D455 (alt: OAK-D Pro W PoE, IP66) | 87°×58°, 0.6–6 m / 150° DFOV, ≤7.5 W | **[V]** |
| Depth (wrist) | RealSense D405-class | ~7–50 cm band | [E — verify] |
| RGB ×4 | 2 MP global-shutter modules in IP66 pods | ≥110° H | [E] |
| Sonar ×8 | MaxBotix MB7360 | 0.3–5 m, IP67, 2.3 mA | **[V]** |
| Arm | UFACTORY Lite 6 | 600 g, 440 mm, 7.2 kg, 150 W typ | **[V]** (IP **[U]**) |
| Gripper | Custom Fin-Ray 2-finger, TPU | 100 mm, ~30 N | [E] |
| Drives ×2 | ODrive S1 | 12–48 V, 40 A cont., CAN | **[V]** |
| Wheel motors ×2 | 24 V BLDC planetary gearmotor 150–200 W, brake, encoder | ≥6.5/13 N·m | [E — shortlist] |
| Battery | 25.6 V 40 Ah LiFePO₄ + CAN BMS | 1024 Wh, ~12 kg | [E] |
| Safety relay | PNOZ-class dual-channel | + STO wiring | [E] |
| E-stop | 22 mm mushroom, 2NC (IDEC/Schneider class) | IP65 head | [E] |
| Load cells | 4× 10 kg half-bridge + HX711 | 24-bit | [E] |
| IMU | BMI088-class | 6-axis | [E] |
| Connectors | M12 (signal), Anderson 120 A (battery) | IP67 | [E] |
| Concept-C upgrades | 2× SICK nanoScan3 + Flexi/PNOZmulti PLC + Mayser edges + GMSL2 cams | nanoScan3: 275°, 3 m PF, IP65 | **[V]** |

---

## 16. Risk Analysis (top items)

| Risk | L | S | Mitigation |
|---|---|---|---|
| Stereo/LiDAR failure on wet reflective floor | H | H | Sonar layer, polarizer test T-08, wet-mode 0.3 m/s, keep-out near pool edge |
| Lite 6 not humidity-tolerant | M | H | Bellows + drip cap, T-10 soak test, C-plan sealed arm |
| Grasp success < 80 % on towels/wet paper | M | M | Fin-Ray compliance, retry logic, suction module Phase 2, report-not-pick fallback |
| Child interaction with arm | M | H | Arm deploy only when no person < 1.5 m, 250 mm/s cap, force limit, instant retract on proximity |
| Odometry loss on wet tile (slip) | M | M | IMU fusion, LiDAR-dominant localization, slip detector (encoder vs. IMU) |
| Condensation in enclosures | M | M | Vent membranes, internal RH alarm, heater rings on optics |
| Bin odor/biohazard | M | L | Sealed lid, liner protocol, daily swap SOP |
| Threshold impact damages casters | L | M | Suspension forks, 0.3 m/s threshold-crossing rule |
| Battery fault in public space | L | H | LiFePO₄ chemistry, CAN BMS with hard cutoffs, fused rails, thermal sensor shutdown |

---

## 17. Safety Analysis & Standards

- **ISO 3691-4:2023** (driverless industrial trucks incl. AMRs **[V]**): primary reference for the *mobile base* — personnel-detection fields, speed zones, braking, warning devices, mode control. Formal compliance needs certified sensors (Concept C nanoScan3 + safety PLC); Concept B follows its *architecture* (detection field → restricted speed → protective stop) without certified components.
- **ISO 13482** (personal-care/service robots; revision in progress **[V]**): relevant because the robot operates among the general public rather than in an industrial site; informs public-interaction requirements (children, unaware bystanders).
- **ISO 10218 / ISO/TS 15066 concepts:** applied to the arm — power/force limiting philosophy, speed-and-separation monitoring for deploy/stow decisions. (Not formally applicable — those target industrial cells — but the PFL limits are the right benchmark.)
- **IEC 60529:** IP rating verification methods (§2 A9). **IEC 62133/UN 38.3:** battery pack. **EN 61496:** what "certified" means for C-grade scanners.
- **Implementation summary:** hardware e-stop chain (button, bumper, watchdog → relay → STO + spring brakes); software stops layered above; speed governance in MCU (not Linux); arm interlocks (stationary base + person-free zone); warning beacon + voice; safe startup (self-test of every safety input incl. forced bumper test at dock); safe shutdown (arm to cradle, brakes on, contactor open); loss-of-localization → stop, relocalize in place, else creep-teleop request; wet-floor mode (auto from site zones or drive-slip detection) → 0.3 m/s, accel 0.2 m/s², longer stop margins; sensor-failure matrix (any safety-layer sensor fault ⇒ RESTRICTED or STOP, never "ignore"); manual recovery mode = physical key switch + handheld e-stop, 0.3 m/s cap.

---

## 18. Maintenance Strategy

Daily (5 min): bin swap + liner, lens/LiDAR window wipe, bumper travel check. Weekly: caster hair/grit strip-down, gasket inspection, load-cell zero check, log review. Monthly: wheel tread wear gauge (replace < 2 mm tread), arm bellows inspection, e-stop chain functional test, torque check on pedestal bolts. All wear parts are cassette/pod modules (§8.1); MTTR targets: bin 10 s, battery 5 min, wheel module 15 min, any sensor pod 10 min, arm 30 min.

---

## 19. Testing & Validation Plan

| ID | Test | Pass criterion |
|---|---|---|
| T-01 | Tilt-table static stability (empty/loaded/arm-out) | No tip ≤ 15° any heading |
| T-02 | Ramp 5° traction wet (soaped tile sheet) | Stop-and-hold + restart, no slip > 5 cm |
| T-03 | Threshold 15 mm @0.3 m/s, 1000 cycles | No structural damage, no floor marking |
| T-04 | Arm floor-pick workspace sampling (sim + real) | ≥ 95 % reachability in declared crescent |
| T-05 | Grasp benchmark: 10 object classes × 50 trials, incl. wet towel/paper | ≥ 80 % first-try, ≥ 95 % ≤ 2 tries |
| T-06 | E-stop chain: button/bumper/watchdog each | Motor power off ≤ 100 ms, brakes engage |
| T-07 | LiDAR quality on wet black floor + glass wall course | Localization error < 5 cm sustained |
| T-08 | Depth on puddles/mirror tiles ± polarizer | False-obstacle rate; pick best config |
| T-09 | IP54 spray test per IEC 60529 + mop-splash SOP | No ingress to either sealed box |
| T-10 | Arm 95 % RH soak, 48 h + operation | No fault; else escalate to sealed arm |
| T-11 | 6 h endurance mission, 100+ picks | Runtime ≥ 6 h, zero safety events |
| T-12 | Public-pilot supervised trials (children scenarios scripted with mannequins first) | Zero contact events; approach-stop stats |

---

## 20. Orthographic Projection Description

- **Front view (600 mm wide × 720 mm shell):** rounded-rectangle shell; full-width bumper band 60–160 mm; left: bin drawer face with recessed pull + status LED; right: arm pedestal shoulder with stowed arm silhouette above shell line (folded Z), wrist camera facing down into stow cradle; center at 550 mm: dark sealed sensor window (D455 behind); at 700 mm: front camera pod; 3 sonar apertures at 250 mm; front-left corner shows LiDAR pocket slot at 165 mm with guard hoop.
- **Side view (800 mm long):** wheel arrangement — front Ø100 caster under a spring fork, rear Ø200 driven wheel; 30 mm belly line; skirt vents rear-bottom (downward-facing); side camera pod at 700 mm tilted −25°; quarter-turn service panel seams; chute flap bulge at 480 mm on the left side profile; arm pedestal from 200→350 mm; stowed arm along the right flank; rear-top e-stop mushroom and beacon ring at 720 mm.
- **Top view:** 800 × 600 rounded rectangle, R60 corners; diagonal LiDAR pockets front-left/rear-right (270° clear sectors drawn); bin lid + Ø180 chute ring front-left; arm base circle front-right with 440 mm reach arc crescent ahead-right; four camera pods on the perimeter beltline; battery tray outline center-rear between driven wheels.
- **Rear view:** battery door with two quarter-turn latches and Anderson connector cover; rear camera pod at 700 mm; 3 sonars at 250 mm; bumper band; beacon + e-stop on top edge; charging contacts at 120 mm height, centered.

## 21. 3D Concept Drawing Prompts

1. **Exterior:** "Industrial design render, indoor service robot, 800×600×720 mm rounded matte light-grey shell with dark grey bumper band, small 6-axis arm folded on right shoulder, corner-recessed lidar pucks at ankle height, four flush camera pods on a beltline, front-left trash drawer, amber beacon ring on top rear, non-marking grey wheels barely visible under a 30 mm skirt, shopping-mall tile floor, soft studio light, orthographic-style 3/4 view."
2. **Cutaway:** "Technical cutaway illustration of the same robot: lower deck with central rear battery tray between two hub-driven wheels, two motor controllers and fuse block beside it; upper deck sealed compute box with external finned heatsink; front-left 30 L bin on four corner load cells with sealed flap chute; front-right arm pedestal with internal cable riser; labeled dry-hull boxes and drain scuppers."
3. **Sensor layout:** "Diagrammatic view with colored translucent FOV cones: two 270° lidar fans at 165 mm (cyan) overlapping to 360°, forward depth-camera frustum tilted to floor (green), four surround camera cones at 700 mm (yellow), eight narrow sonar lobes at 250 mm (magenta), wrist camera cone from gripper (orange), bumper band highlighted red."
4. **Electronics rack:** "Exploded view: two aluminum rack layers on extrusion frame; bottom: LiFePO₄ pack, BMS, 2 ODrive-class drives, safety relay, DC-DC bricks, M12 bulkhead; top: Jetson carrier in gasketed box with cold-plate wall, STM32 IO board, network switch; harness routed through single bulkhead with circular connectors."
5. **Arm workspace:** "Side and top schematic of a 440 mm-reach 6-axis arm on a 350 mm pedestal at the robot's front-right; shaded reachable hemisphere truncated by the body; highlighted floor-pick crescent 150–300 mm from base axis; dashed trajectory from floor pick over to a 480 mm-high bin chute; red keep-out volumes over lidar pocket and camera pods."

---

## 22. Sources

- SLAMTEC RPLIDAR S2: [slamtec.com/en/s2/spec](https://www.slamtec.com/en/s2/spec), [S2 datasheet PDF](https://files.seeedstudio.com/products/114992738/document/SLAMTEC_rplidar_datasheet_S2M1_v1.0_en.pdf)
- Jetson Orin Nano Super: [NVIDIA product page](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/), [NVIDIA dev blog](https://developer.nvidia.com/blog/nvidia-jetson-orin-nano-developer-kit-gets-a-super-boost/)
- RealSense D455: [Intel spec page](https://www.intel.com/content/www/us/en/products/sku/205847/intel-realsense-depth-camera-d455/specifications.html), [D400 series datasheet](https://www.intelrealsense.com/wp-content/uploads/2020/06/Intel-RealSense-D400-Series-Datasheet-June-2020.pdf)
- UFACTORY Lite 6: [ufactory.us/product/lite-6](https://www.ufactory.us/product/lite-6), [RobotShop listing](https://www.robotshop.com/products/ufactory-6-axis-robot-arm-lite-6)
- Dynamixel XM540-W270: [ROBOTIS e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/)
- SICK nanoScan3: [SICK product info PDF](https://www.sick.com/media/docs/5/75/075/product_information_nanoscan3_en_im0087075.pdf), [datasheet](https://www.sick.com/media/pdf/0/80/980/dataSheet_NANS3-CAAZ30AN1_1100334_en.pdf)
- MaxBotix MB7360: [maxbotix.com/products/mb7360](https://maxbotix.com/products/mb7360)
- Luxonis OAK-D Pro W: [docs.luxonis.com](https://docs.luxonis.com/hardware/products/OAK-D%20Pro%20W), [shop page](https://shop.luxonis.com/products/oak-d-pro-w)
- ODrive S1: [docs.odriverobotics.com S1 datasheet](https://docs.odriverobotics.com/v/latest/hardware/s1-datasheet.html)
- ISO 3691-4:2023: [iso.org/standard/83545.html](https://www.iso.org/standard/83545.html); ISO 13482 revision context: [automate.org AMR safety update](https://www.automate.org/robotics/industry-insights/autonomous-mobile-robot-safety-updates-new-features)
- ROS 2 / Nav2 / MoveIt 2 / slam_toolbox: docs.ros.org, docs.nav2.org, moveit.picknik.ai (framework choices; no numeric claims sourced from these)

*All values tagged [E] or [U] must be closed out during detailed design; do not release drawings for tooling against untagged assumptions.*
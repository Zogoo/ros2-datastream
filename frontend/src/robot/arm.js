import * as THREE from 'three';
import { armFk, gripperOpening, servoStep } from './kinematics.js';
import { GROUP_ARM, GROUP_OBJECT, GROUP_WORLD, OBJECT_FILTER, groups } from '../physics/world.js';

const GRAVITY = 9.81;
// While held the item belongs to GROUP_OBJECT but collides with nothing —
// walls and furniture cannot interact with a carried prop.
const HELD_GROUPS = groups(GROUP_OBJECT, 0);
// Free props use the normal object filter so they rest on the floor.
const FREE_GROUPS = groups(GROUP_OBJECT, OBJECT_FILTER);

// Static-geometry kinds the arm can legitimately touch (the gripper brushes
// the floor at the bottom of every scoop) vs. a genuine wall/prop/bin strike.
const WALL_CONTACT_KINDS = new Set(['wall', 'prop', 'bin_wall', 'platform']);
const WALL_CONTACT_DEBOUNCE_MS = 500; // avoid flooding /robot/events on sustained contact

const HOME = [90, 90, 90, 90, 90, 70];
const GRIP = 5;
// A pinched towel hangs under the jaws and swings into place: first-order
// settle toward "hanging below the fingertip" (pendulum-ish lag, no snap).
const HANG_SETTLE_TAU_S = 0.12;
const ACCEL_FILTER_TAU_S = 0.06;    // wrist load-cell / slip-check bandwidth
const JAW_CLEARANCE_M = 0.005;      // jaws must open this much wider than the bite to get around it

/** Gripper events queue — drained by main.js onto /robot/events. These are
 *  the edges a real gripper controller sees on its own jaw encoder: the jaws
 *  stalled on something (ACQUIRED), opened (RELEASED), or collapsed shut
 *  mid-carry because the payload pulled out (SLIPPED). */
export const graspEvents = [];

/**
 * 6-axis arm: servo lag toward firmware joint targets, FK-driven visuals,
 * kinematic colliders for forearm/gripper, and a physical parallel-jaw pinch.
 *
 * Grasp model (no magic snap radius):
 *  - The jaws close on an item only if, while CLOSING from an opening wider
 *    than the cloth bite, the finger pads straddle part of the item's
 *    footprint and reach down into it (ObjectManager.pinchCandidate).
 *  - On contact the gripper servo STALLS at the compressed bite width — the
 *    measured J5 angle on /joint_states stays above the commanded angle, and
 *    the gripper effort channel reads the grip force. That stall is the
 *    grasp-success signal a real gripper controller reports.
 *  - Holding capacity = grip_force_max * bite quality (* cloth variability).
 *    Until the towel is lifted off the floor the floor carries it; after
 *    lift-off, if weight + inertial load exceeds the capacity the cloth pulls
 *    out of the jaws (GRASP_SLIPPED) — heavy soaked towels and shallow hem
 *    pinches fail the way they do on hardware.
 *  - Carry: the held body is kinematic, hanging below the fingertip in the
 *    gripper's heading and settling with a short lag. On release it becomes
 *    dynamic again with its carry velocity.
 */
export class Arm {
  constructor(physics, robot, armSpec, objects, rng = null) {
    this.physics = physics;
    this.robot = robot;
    this.spec = armSpec;
    this.objects = objects;
    this.rng = rng;

    this.current = [...HOME];
    this.target = [...HOME];
    this.heldItem = null;
    this._grip = null;          // per-grasp state, see _attach
    this._closing = false;
    this._closeStartOpening = 0;
    this.lastFingertip = null;
    this.fingertipVel = { x: 0, y: 0, z: 0 };
    this._lastWallContactAt = 0;
    // Per-joint effort, reported on /joint_states: wrist channel = wrist load
    // cell (N), gripper channel = jaw grip force (N). See _updateEffort.
    this.effort = new Array(6).fill(0);

    this._buildKinematicColliders();
    this._buildVisuals();
  }

  _buildKinematicColliders() {
    const { R, world } = this.physics;
    const makeBody = () => world.createRigidBody(
      R.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 0, 2),
    );
    // Rapier's ActiveCollisionTypes.DEFAULT (DYNAMIC_DYNAMIC|DYNAMIC_FIXED|
    // DYNAMIC_KINEMATIC) does NOT include KINEMATIC_FIXED — a kinematic body
    // (this arm) against a fixed body (a wall) is silently excluded from
    // collision detection entirely unless explicitly opted in, independent of
    // ActiveEvents. Without this, ARM_CONTACT can never fire.
    const armCollisionTypes = R.ActiveCollisionTypes.DEFAULT | R.ActiveCollisionTypes.KINEMATIC_FIXED;

    this.forearmBody = makeBody();
    const forearmCol = world.createCollider(
      R.ColliderDesc.capsule(this.spec.links.forearm / 2, 0.03)
        .setCollisionGroups(groups(GROUP_ARM, GROUP_WORLD | GROUP_OBJECT))
        .setActiveEvents(R.ActiveEvents.COLLISION_EVENTS)
        .setActiveCollisionTypes(armCollisionTypes),
      this.forearmBody,
    );
    this.physics.registerMeta(forearmCol, { kind: 'robot', part: 'arm_forearm' });

    this.gripperBody = makeBody();
    const gripCol = world.createCollider(
      R.ColliderDesc.ball(0.045)
        .setCollisionGroups(groups(GROUP_ARM, GROUP_WORLD | GROUP_OBJECT))
        // The open fingers straddle cloth rather than shove it: the gripper
        // volume still DETECTS objects/walls (ARM_CONTACT, lidar self-hits)
        // but applies no contact force to movable props — otherwise the
        // descending jaw ball squirted every towel out from under itself.
        .setSolverGroups(groups(GROUP_ARM, GROUP_WORLD))
        .setActiveEvents(R.ActiveEvents.COLLISION_EVENTS)
        .setActiveCollisionTypes(armCollisionTypes),
      this.gripperBody,
    );
    this.physics.registerMeta(gripCol, { kind: 'robot', part: 'gripper' });
  }

  _buildVisuals() {
    this.group = new THREE.Group();
    const linkMat = new THREE.MeshStandardMaterial({ color: 0xf2efe6, roughness: 0.35 });
    const jointMat = new THREE.MeshStandardMaterial({ color: 0x44464d, roughness: 0.5 });
    const fingerMat = new THREE.MeshStandardMaterial({ color: 0x2e2e33, roughness: 0.6 });

    const linkGeom = new THREE.CylinderGeometry(0.028, 0.028, 1, 12);
    this.linkMeshes = [0, 1, 2].map(() => {
      const m = new THREE.Mesh(linkGeom, linkMat);
      m.castShadow = true;
      this.group.add(m);
      return m;
    });
    this.jointMeshes = [0, 1, 2, 3].map(() => {
      const m = new THREE.Mesh(new THREE.SphereGeometry(0.04, 12, 10), jointMat);
      this.group.add(m);
      return m;
    });
    this.fingers = [0, 1].map(() => {
      const m = new THREE.Mesh(new THREE.BoxGeometry(0.015, 0.02, 0.08), fingerMat);
      this.group.add(m);
      return m;
    });
  }

  setTargets(degrees) {
    for (let i = 0; i < 6; i++) {
      if (Number.isFinite(degrees[i])) this.target[i] = degrees[i];
    }
  }

  holding() {
    return this.heldItem !== null;
  }

  /** What the gripper controller itself can report: whether the jaws are
   *  stalled on an object, the measured jaw width and the grip force. It does
   *  NOT know what the object is — identity/class come from perception (or
   *  /ground_truth/objects for evaluation). */
  gripperStatus() {
    return {
      held: this.heldItem !== null,
      jaw_width_m: round3(gripperOpening(this.current[GRIP], this.spec)),
      grip_force_n: round3(this.effort[GRIP]),
      payload_n: round3(this.effort[3]),
    };
  }

  forceRelease() {
    if (this.heldItem) this._endGrasp('GRASP_RELEASED');
  }

  /**
   * Reactive wall-contact detection — call once per physics step, AFTER
   * `physics.step()` (collision events only exist post-step). Uses Rapier's
   * COLLISION_EVENTS (geometric start/stop, independent of body type/force —
   * unlike CONTACT_FORCE_EVENTS, which kinematic bodies do not reliably
   * generate since they aren't part of the force solver). The forearm/gripper
   * are kinematic and pass through geometry with no physical resolution, so
   * without this the arm could silently clip through a wall during a
   * PICK/DROP sequence; this is the sensor edge a real servo's stall/current
   * limit would report, published as ARM_CONTACT for the mission FSM to abort
   * on (see mission.py _handle_arm_contact).
   */
  drainWallContacts() {
    this.physics.eventQueue.drainCollisionEvents((h1, h2, started) => {
      if (!started) return;
      const meta1 = this.physics.metaOf(h1);
      const meta2 = this.physics.metaOf(h2);
      const isArmPart = (m) => m?.kind === 'robot' && (m.part === 'arm_forearm' || m.part === 'gripper');
      const armMeta = isArmPart(meta1) ? meta1 : isArmPart(meta2) ? meta2 : null;
      if (!armMeta) return;
      const otherMeta = armMeta === meta1 ? meta2 : meta1;
      if (!otherMeta || !WALL_CONTACT_KINDS.has(otherMeta.kind)) return;

      const now = performance.now();
      if (now - this._lastWallContactAt < WALL_CONTACT_DEBOUNCE_MS) return;
      this._lastWallContactAt = now;
      graspEvents.push({ event: 'ARM_CONTACT', object_id: otherMeta.id ?? null, object_class: otherMeta.kind });
    });
  }

  update(dt) {
    for (let i = 0; i < 6; i++) {
      this.current[i] = servoStep(
        this.current[i], this.target[i], dt, this.spec.servo_lag_tau_s, this.spec.max_joint_speed_dps,
      );
    }
    // Jaws cannot close through the cloth they are squeezing: the gripper
    // servo stalls at the bite width (visible on /joint_states J5).
    if (this._grip) this.current[GRIP] = Math.max(this.current[GRIP], this._grip.stallDeg);

    const fk = armFk(this.current, this.spec);
    const pts = ['shoulder', 'elbow', 'wrist', 'fingertip'].map((k) => this.robot.worldPoint(fk[k]));
    this.fkWorld = pts;

    if (this.lastFingertip && dt > 0) {
      const tip = pts[3];
      this.fingertipVel = {
        x: (tip.x - this.lastFingertip.x) / dt,
        y: (tip.y - this.lastFingertip.y) / dt,
        z: (tip.z - this.lastFingertip.z) / dt,
      };
    }
    this.lastFingertip = { ...pts[3] };

    this._syncKinematics(pts);
    this._updateGrasp(pts[3]);
    this._carry(pts[3], fk.pan, dt);
    this._transferCarriedLoad(dt, pts[3]);
    this._updateEffort();
    this._syncVisuals(pts);
  }

  _syncKinematics(pts) {
    const mid = midpoint(pts[1], pts[2]);
    this.forearmBody.setNextKinematicTranslation(mid);
    this.forearmBody.setNextKinematicRotation(quatFromYTo(sub(pts[2], pts[1])));
    this.gripperBody.setNextKinematicTranslation(pts[3]);
  }

  _updateGrasp(tip) {
    const g = this.spec.gripper;
    const opening = gripperOpening(this.current[GRIP], this.spec);

    if (this._grip) {
      // Jaws commanded open past the cloth -> release.
      if (opening > this._grip.biteWidth + 0.003) this._endGrasp('GRASP_RELEASED');
      return;
    }

    const closing = this.target[GRIP] < this.current[GRIP] - 0.5;
    if (closing && !this._closing) this._closeStartOpening = opening;
    this._closing = closing;
    if (!closing) return;

    const cand = this.objects.pinchCandidate(tip, g);
    if (!cand || opening > cand.biteWidth) return;          // pads not yet on the cloth
    if (this._closeStartOpening < cand.biteWidth + JAW_CLEARANCE_M) return; // jaws never got around it
    this._attach(cand);
  }

  /** The pads met cloth: stall the jaws, crumple the towel into the pinch and
   *  switch it to a kinematic carry. The floor still supports it until the
   *  arm lifts it off (see _carry). */
  _attach(cand) {
    const { R } = this.physics;
    const g = this.spec.gripper;
    const item = cand.item;
    const variability = this.rng ? this.rng.uniform() * 0.35 + 0.75 : 1.0;  // cloth is not a rigid part
    const quality = Math.min(1, cand.quality * variability);

    const p = item.body.translation();
    const bottom = p.z - this.objects.halfExtents(item).z;
    item.body.setBodyType(R.RigidBodyType.KinematicPositionBased, true);
    item.collider.setCollisionGroups(HELD_GROUPS);
    // Handling bunches a towel for good — the heap shape is what lets it
    // stack in the collect bin. ObjectManager.reset() refolds.
    this.objects.setClothState(item, 'crumpled', HELD_GROUPS);
    item.held = true;
    this.heldItem = item;

    this._grip = {
      biteWidth: cand.biteWidth,
      stallDeg: (cand.biteWidth / g.max_opening_m) * 180,
      quality,
      capacityN: g.grip_force_max_n * quality,
      hang: item.cloth?.hang_m ?? this.objects.halfExtents(item).z,
      restZ: bottom + this.objects.halfExtents(item).z,
      liftedOff: false,
      pos: { x: p.x, y: p.y, z: bottom + this.objects.halfExtents(item).z },
      vel: { x: 0, y: 0, z: 0 },
      acc: { x: 0, y: 0, z: 0 },
      loadN: 0,
    };
    graspEvents.push({ event: 'GRASP_ACQUIRED', object_id: item.id, object_class: item.cls });
  }

  /**
   * Drive the kinematic held item: resting on the floor (pinched, not yet
   * lifted) until the fingertip rises a hang-length above it, then hanging
   * below the jaws in the gripper's heading, settling with a short lag. The
   * filtered acceleration of the hanging mass sets the load on the pinch; if
   * it exceeds the holding capacity the towel slips out.
   */
  _carry(tip, pan, dt) {
    const grip = this._grip;
    if (!this.heldItem || !grip || dt <= 0) return;
    const item = this.heldItem;

    const hangTarget = { x: tip.x, y: tip.y, z: tip.z - grip.hang };
    if (!grip.liftedOff) {
      if (hangTarget.z <= grip.restZ + 0.005) {
        item.body.setNextKinematicTranslation(grip.pos);
        grip.loadN = 0;
        return;
      }
      grip.liftedOff = true;
    }
    const a = 1 - Math.exp(-dt / HANG_SETTLE_TAU_S);
    const next = {
      x: grip.pos.x + (hangTarget.x - grip.pos.x) * a,
      y: grip.pos.y + (hangTarget.y - grip.pos.y) * a,
      z: grip.pos.z + (hangTarget.z - grip.pos.z) * a,
    };
    const vel = { x: (next.x - grip.pos.x) / dt, y: (next.y - grip.pos.y) / dt, z: (next.z - grip.pos.z) / dt };
    const b = 1 - Math.exp(-dt / ACCEL_FILTER_TAU_S);
    for (const k of ['x', 'y', 'z']) {
      grip.acc[k] += ((vel[k] - grip.vel[k]) / dt - grip.acc[k]) * b;
    }
    grip.vel = vel;
    grip.pos = next;

    // Hang in the gripper's heading (robot yaw + arm pan).
    const yaw = this.robot.pose().yaw + pan;
    item.body.setNextKinematicTranslation(next);
    item.body.setNextKinematicRotation({ w: Math.cos(yaw / 2), x: 0, y: 0, z: Math.sin(yaw / 2) });

    grip.loadN = item.massKg * Math.hypot(grip.acc.x, grip.acc.y, grip.acc.z + GRAVITY);
    if (grip.loadN > grip.capacityN) this._endGrasp('GRASP_SLIPPED');
  }

  _endGrasp(event) {
    const { R } = this.physics;
    const item = this.heldItem;
    const grip = this._grip;
    this.heldItem = null;
    this._grip = null;
    this._closing = false;

    // Release-clearance guard: if the carried item was against/inside static
    // geometry at the moment of release (e.g. an ARM_CONTACT-aborted
    // sequence), relocate the item above the robot's own footprint before
    // switching it back to Dynamic. The robot cannot itself be embedded in a
    // wall it is contacting with its arm, so this position is always clear —
    // avoiding the depenetration-ejection this fixed originally.
    if (this._isEmbeddedInWalls(item)) {
      const base = this.robot.body.translation();
      item.body.setNextKinematicTranslation({ x: base.x, y: base.y, z: base.z + 0.35 });
    }

    // Restore dynamic physics BEFORE restoring collisions so Rapier can solve
    // the first contact tick correctly without depenetration explosions.
    item.body.setBodyType(R.RigidBodyType.Dynamic, true);
    item.collider.setCollisionGroups(FREE_GROUPS);
    item.held = false;
    item.body.setLinvel(grip?.liftedOff ? grip.vel : { x: 0, y: 0, z: 0 }, true);
    item.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    // Stays crumpled — see _attach.
    graspEvents.push({ event, object_id: item.id, object_class: item.cls });
  }

  /** Small-ball probe at the item's center: true if it overlaps THICK static
   *  geometry (walls, platforms, props) an ejected towel could be trapped
   *  inside. Floor is excluded (a released item legitimately rests on it) and
   *  so are bin walls: every DROP_BIN release hovers the towel right at the
   *  2 cm-thin bin rim, which cannot trap it — a rim-straddling towel just
   *  falls one way or the other once dynamic. Treating bin_wall as "embedded"
   *  teleported the towel back over the robot at the exact moment of a bin
   *  drop, making every delivery miss. Item is still kinematic here, so this
   *  only answers yes/no — no penetration-depth math required. */
  _isEmbeddedInWalls(item) {
    const { R, world } = this.physics;
    const pos = item.body.translation();
    const TRAPPING_KINDS = new Set(['wall', 'platform', 'prop']);
    let embedded = false;
    world.intersectionsWithShape(
      pos, { w: 1, x: 0, y: 0, z: 0 }, new R.Ball(0.05),
      (collider) => {
        const meta = this.physics.metaOf(collider.handle);
        if (meta && TRAPPING_KINDS.has(meta.kind)) {
          embedded = true;
          return false; // stop the query — one hit is enough
        }
        return true;
      },
    );
    return embedded;
  }

  /** Push carried weight back onto the chassis so the suspension visibly
   *  settles — only once the payload actually hangs from the arm. */
  _transferCarriedLoad(dt, fingertip) {
    if (!this._grip?.liftedOff || dt <= 0) return;
    this.robot.body.applyImpulseAtPoint(
      { x: 0, y: 0, z: -this._grip.loadN * dt }, fingertip, true,
    );
  }

  /**
   * Force channels on /joint_states `effort`:
   *  - wrist_pitch_joint: wrist load cell (N) = the payload's weight plus its
   *    inertial load, m·|a − g|, pose-INDEPENDENT (shoulder torque would vanish
   *    in the near-vertical PICK_LIFT pose — see docs/research_notes.md). It
   *    reads ~0 while the pinched towel still lies on the floor. Always carries
   *    sensor noise, so consumers must threshold it (arm_protocol does).
   *  - gripper_joint: jaw grip force (N) — servo stall current while squeezing
   *    an object, ~0 when the jaws are free or closed on air.
   */
  _updateEffort() {
    const g = this.spec.gripper;
    const noise = (std) => (this.rng ? this.rng.gaussian(0, std) : 0);
    const n = g.wrist_load_cell_noise_n ?? 0;
    this.effort.fill(0);
    this.effort[3] = round3((this._grip?.liftedOff ? this._grip.loadN : 0) + noise(n));
    this.effort[GRIP] = round3(this._grip ? g.grip_force_max_n + noise(n * 5) : Math.abs(noise(n)));
  }

  _syncVisuals(pts) {
    for (let i = 0; i < 3; i++) {
      orientCylinder(this.linkMeshes[i], pts[i], pts[i + 1]);
    }
    pts.forEach((p, i) => this.jointMeshes[i].position.set(p.x, p.y, p.z));

    const opening = gripperOpening(this.current[5], this.spec);
    const tip = pts[3];
    const dir = normalize(sub(tip, pts[2]));
    const side = normalize(cross(dir, { x: 0, y: 0, z: 1 }));
    for (const [i, sign] of [[0, 1], [1, -1]]) {
      const off = (opening / 2 + 0.012) * sign;
      this.fingers[i].position.set(tip.x + side.x * off, tip.y + side.y * off, tip.z - 0.03);
    }
  }

  statusForHud() {
    return this.current.map((d) => Math.round(d)).join(' ');
  }
}

const sub = (a, b) => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z });
const midpoint = (a, b) => ({ x: (a.x + b.x) / 2, y: (a.y + b.y) / 2, z: (a.z + b.z) / 2 });
const cross = (a, b) => ({
  x: a.y * b.z - a.z * b.y, y: a.z * b.x - a.x * b.z, z: a.x * b.y - a.y * b.x,
});
function normalize(v) {
  const len = Math.hypot(v.x, v.y, v.z) || 1;
  return { x: v.x / len, y: v.y / len, z: v.z / len };
}

const _up = new THREE.Vector3(0, 1, 0);
const _dir = new THREE.Vector3();
const _quat = new THREE.Quaternion();

function orientCylinder(mesh, from, to) {
  _dir.set(to.x - from.x, to.y - from.y, to.z - from.z);
  const len = _dir.length() || 0.001;
  mesh.scale.set(1, len, 1);
  mesh.position.set((from.x + to.x) / 2, (from.y + to.y) / 2, (from.z + to.z) / 2);
  _quat.setFromUnitVectors(_up, _dir.normalize());
  mesh.quaternion.copy(_quat);
}

function quatFromYTo(dir) {
  _dir.set(dir.x, dir.y, dir.z).normalize();
  _quat.setFromUnitVectors(_up, _dir);
  return { w: _quat.w, x: _quat.x, y: _quat.y, z: _quat.z };
}

const round3 = (v) => Math.round(v * 1000) / 1000;

import * as THREE from 'three';
import { armFk, gripperOpening, isGripperClosed, servoStep } from './kinematics.js';
import { GROUP_ARM, GROUP_OBJECT, GROUP_WORLD, OBJECT_FILTER, groups } from '../physics/world.js';

const GRAVITY = 9.81;
// While held the item belongs to GROUP_OBJECT but collides with nothing —
// walls and furniture cannot interact with a carried prop.
const HELD_GROUPS = groups(GROUP_OBJECT, 0);
// Free props use the normal object filter so they rest on the floor.
const FREE_GROUPS = groups(GROUP_OBJECT, OBJECT_FILTER);

// A grasped cloth bunches into a crumple and STAYS crumpled once handled —
// applied to both the visual scale and the physics collider half-extents, so
// a handled towel (0.195 x 0.143 x 0.081 m) fits the onboard collect bin in
// any orientation (flat rigid towels would not). Reset restores flat.
export const CRUMPLE_SCALE = { x: 0.65, y: 0.65, z: 1.8 };
const GRIP_SCALE = new THREE.Vector3(CRUMPLE_SCALE.x, CRUMPLE_SCALE.y, CRUMPLE_SCALE.z);

// Static-geometry kinds the arm can legitimately touch (the gripper brushes
// the floor at the bottom of every scoop) vs. a genuine wall/prop/bin strike.
const WALL_CONTACT_KINDS = new Set(['wall', 'prop', 'bin_wall', 'platform']);
const WALL_CONTACT_DEBOUNCE_MS = 500; // avoid flooding /robot/events on sustained contact

const HOME = [90, 90, 90, 90, 90, 70];

/** Grasp events queue — drained by main.js onto /robot/events. Emulates a
 *  gripper payload/force sensor: a sensor signal, not ground truth. */
export const graspEvents = [];

/**
 * 6-axis arm: servo lag toward firmware joint targets, FK-driven visuals,
 * kinematic colliders for forearm/gripper, and kinematic-body carry.
 *
 * Carry strategy: on grasp the held item's Rapier body is switched to
 * KinematicPositionBased and driven to (fingertip + carryOffset) every tick.
 * This guarantees the item follows the arm exactly with no joint stress, no
 * depenetration surprises on release, and no wall-sticking — HELD_GROUPS
 * already prevents collisions while carried. On release the body is switched
 * back to Dynamic and given the current fingertip velocity.
 */
export class Arm {
  constructor(physics, robot, armSpec, objects) {
    this.physics = physics;
    this.robot = robot;
    this.spec = armSpec;
    this.objects = objects;

    this.current = [...HOME];
    this.target = [...HOME];
    this.heldItem = null;
    this._carryOffset = null;   // world-frame offset: item_center − fingertip at pickup
    this._heldMass = 0;         // dynamic-body mass cached at grasp (kinematic reports 0)
    this.wasClosed = false;
    this.lastFingertip = null;
    this.fingertipVel = { x: 0, y: 0, z: 0 };
    this._lastWallContactAt = 0;
    // Per-joint effort (N·m), reported on /joint_states. Only the shoulder
    // channel is populated — see _updateEffort.
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

  /** Returns {object_id, object_class, position} of the held item, or null. */
  heldObjectInfo() {
    if (!this.heldItem) return null;
    const p = this.heldItem.body.translation();
    return {
      object_id: this.heldItem.id,
      object_class: this.heldItem.cls,
      position: { x: p.x, y: p.y, z: p.z },
    };
  }

  forceRelease() {
    if (this.heldItem) this._release();
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
    this._carryKinematic(pts[3]);
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

  _updateGrasp(fingertip) {
    const closed = isGripperClosed(this.current[5], this.spec);
    if (closed && !this.wasClosed && !this.heldItem) {
      const item = this.objects.nearestPickable(
        [fingertip.x, fingertip.y, fingertip.z], this.spec.gripper.grasp_radius_m,
      );
      if (item) this._attach(item, fingertip);
    } else if (!closed && this.wasClosed && this.heldItem) {
      this._release();
    }
    this.wasClosed = closed;
  }

  /**
   * Switch the item to kinematic carry. We drive its body translation directly
   * each tick instead of using an impulse joint. Eliminates wall-sticking (no
   * joint pulling the item into wall geometry) and depenetration surprises on
   * release. Visual scale is morphed to a bunched-cloth shape.
   */
  _attach(item, fingertip) {
    const { R } = this.physics;
    const ip = item.body.translation();
    this._carryOffset = { x: ip.x - fingertip.x, y: ip.y - fingertip.y, z: ip.z - fingertip.z };
    // Capture the mass BEFORE switching to kinematic — Rapier reports zero mass
    // for a kinematic body, so the wrist load-cell reading and the chassis load
    // transfer both need the dynamic-body mass cached here.
    this._heldMass = item.body.mass();
    item.body.setBodyType(R.RigidBodyType.KinematicPositionBased, true);
    item.collider.setCollisionGroups(HELD_GROUPS);
    item.held = true;
    this.heldItem = item;
    // Crumple the cloth: visual scale AND collider shape, kept after release
    // (a handled towel stays bunched — this is what lets it fit the collect
    // bin). ObjectManager.reset() restores the flat shape.
    item.mesh.scale.copy(GRIP_SCALE);
    const shape = item.collider.shape;
    if (!item.crumpled && shape.halfExtents) {
      item.flatHalfExtents = { ...shape.halfExtents };
      item.collider.setHalfExtents({
        x: shape.halfExtents.x * CRUMPLE_SCALE.x,
        y: shape.halfExtents.y * CRUMPLE_SCALE.y,
        z: shape.halfExtents.z * CRUMPLE_SCALE.z,
      });
      item.crumpled = true;
    }
    graspEvents.push({ event: 'GRASP_ACQUIRED', object_id: item.id, object_class: item.cls });
  }

  _release() {
    this._endGrasp('GRASP_RELEASED', this.fingertipVel);
  }

  /**
   * Drive the kinematic held item to follow the fingertip exactly.
   * Kinematic bodies ignore all physics forces so the item cannot be knocked
   * loose or dragged into walls while carried.
   */
  _carryKinematic(fingertip) {
    if (!this.heldItem || !this._carryOffset) return;
    this.heldItem.body.setNextKinematicTranslation({
      x: fingertip.x + this._carryOffset.x,
      y: fingertip.y + this._carryOffset.y,
      z: fingertip.z + this._carryOffset.z,
    });
  }

  _endGrasp(event, releaseVel) {
    const { R } = this.physics;
    const item = this.heldItem;
    this.heldItem = null;
    this._carryOffset = null;
    this._heldMass = 0;

    // Release-clearance guard: if the fingertip was against/inside static
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
    item.body.setLinvel(releaseVel, true);
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

  /** Push carried weight back onto the chassis so the suspension visibly settles. */
  _transferCarriedLoad(dt, fingertip) {
    if (!this.heldItem || dt <= 0) return;
    this.robot.body.applyImpulseAtPoint(
      { x: 0, y: 0, z: -this._heldMass * GRAVITY * dt }, fingertip, true,
    );
  }

  /**
   * Wrist load-cell reading (N): the axial force a wrist-mounted force sensor
   * measures = the weight of whatever hangs off the gripper (mass * g), and
   * nothing when empty. Reported on the wrist joint's effort channel.
   *
   * This is pose-INDEPENDENT, which is the whole point: shoulder *torque*
   * (mass * g * horizontal lever arm) genuinely vanishes when the arm lifts the
   * payload near-vertical — the exact PICK_LIFT pose the pick sequence ends in —
   * so it is a useless presence sensor there. Payload weight at the wrist is a
   * real physical quantity a load cell reads regardless of arm configuration.
   * This is the honest presence signal for a scoop grasp, which (unlike a
   * two-finger parallel gripper) has no jaw-width to verify a hold with (see
   * docs/research_notes.md). arm_controller thresholds it into gripper_holding,
   * closing the loop without any ground-truth shortcut.
   */
  _updateEffort() {
    this.effort.fill(0);
    if (!this.heldItem) return;
    this.effort[3] = round3(this._heldMass * GRAVITY);  // wrist_pitch_joint
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

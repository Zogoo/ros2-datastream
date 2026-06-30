import * as THREE from 'three';
import { armFk, gripperOpening, isGripperClosed, servoStep } from './kinematics.js';
import { GROUP_ARM, GROUP_OBJECT, GROUP_WORLD, OBJECT_FILTER, groups } from '../physics/world.js';

const GRAVITY = 9.81;
// While held the item belongs to GROUP_OBJECT but collides with nothing —
// walls and furniture cannot interact with a carried prop.
const HELD_GROUPS = groups(GROUP_OBJECT, 0);
// Free props use the normal object filter so they rest on the floor.
const FREE_GROUPS = groups(GROUP_OBJECT, OBJECT_FILTER);

// Visual scale applied to a towel-class item while gripped (bunched cloth appearance).
const GRIP_SCALE = new THREE.Vector3(0.70, 0.70, 2.5);
const FREE_SCALE = new THREE.Vector3(1, 1, 1);

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
    this.wasClosed = false;
    this.lastFingertip = null;
    this.fingertipVel = { x: 0, y: 0, z: 0 };

    this._buildKinematicColliders();
    this._buildVisuals();
  }

  _buildKinematicColliders() {
    const { R, world } = this.physics;
    const makeBody = () => world.createRigidBody(
      R.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 0, 2),
    );
    this.forearmBody = makeBody();
    const forearmCol = world.createCollider(
      R.ColliderDesc.capsule(this.spec.links.forearm / 2, 0.03)
        .setCollisionGroups(groups(GROUP_ARM, GROUP_WORLD | GROUP_OBJECT)),
      this.forearmBody,
    );
    this.physics.registerMeta(forearmCol, { kind: 'robot', part: 'arm_forearm' });

    this.gripperBody = makeBody();
    const gripCol = world.createCollider(
      R.ColliderDesc.ball(0.045).setCollisionGroups(groups(GROUP_ARM, GROUP_WORLD | GROUP_OBJECT)),
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
    item.body.setBodyType(R.RigidBodyType.KinematicPositionBased, true);
    item.collider.setCollisionGroups(HELD_GROUPS);
    item.held = true;
    this.heldItem = item;
    item.mesh.scale.copy(GRIP_SCALE);
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
    // Restore dynamic physics BEFORE restoring collisions so Rapier can solve
    // the first contact tick correctly without depenetration explosions.
    item.body.setBodyType(R.RigidBodyType.Dynamic, true);
    item.collider.setCollisionGroups(FREE_GROUPS);
    item.held = false;
    item.body.setLinvel(releaseVel, true);
    item.mesh.scale.copy(FREE_SCALE);
    graspEvents.push({ event, object_id: item.id, object_class: item.cls });
  }

  /** Push carried weight back onto the chassis so the suspension visibly settles. */
  _transferCarriedLoad(dt, fingertip) {
    if (!this.heldItem || dt <= 0) return;
    const m = this.heldItem.body.mass();
    this.robot.body.applyImpulseAtPoint({ x: 0, y: 0, z: -m * GRAVITY * dt }, fingertip, true);
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

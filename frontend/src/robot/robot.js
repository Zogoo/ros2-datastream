import * as THREE from 'three';
import { WheelSuspension, rotateQuat } from '../physics/vehicle.js';
import { GROUP_ROBOT, GROUP_WORLD, groups } from '../physics/world.js';
import { Arm } from './arm.js';

export const BASE_Z = 0.13; // body origin height above ground at suspension rest

const LED_COLORS = {
  idle: 0x4da6ff, moving: 0x7ddc7d, picking: 0xffb74d, estop: 0xff3b30,
};

/** The robot: dynamic chassis + raycast suspension + racked decks + 6-axis arm. */
export class Robot {
  constructor(physics, scene, spec, spawn, objects) {
    this.physics = physics;
    this.spec = spec;
    this.objects = objects;

    const yaw = spawn.yaw;
    this.body = physics.world.createRigidBody(
      physics.R.RigidBodyDesc.dynamic()
        .setTranslation(spawn.pos[0], spawn.pos[1], BASE_Z + 0.02)
        .setRotation(yawQuat(yaw))
        .setCanSleep(false)
        .setAngularDamping(1.2)
        .setLinearDamping(0.05),
    );

    const c = spec.chassis;
    // Friction tuned for the smooth ABS skirt: low enough that the beveled
    // nose slides over step corners below the 60 mm clearance (34 deg
    // approach angle) instead of wedging, high enough that the flat front
    // face cannot ratchet up the 280 mm bath rim.
    const tub = physics.R.ColliderDesc.cuboid(c.size[0] / 2, c.size[1] / 2, c.size[2] / 2)
      .setMass(c.mass)
      .setFriction(0.15)
      .setCollisionGroups(groups(GROUP_ROBOT, GROUP_WORLD))
      .setActiveEvents(physics.R.ActiveEvents.COLLISION_EVENTS | physics.R.ActiveEvents.CONTACT_FORCE_EVENTS)
      .setContactForceEventThreshold(1.0);
    this.tubCollider = physics.world.createCollider(tub, this.body);
    physics.registerMeta(this.tubCollider, { kind: 'robot', part: 'chassis' });

    this._buildBasketColliders();
    this.suspension = new WheelSuspension(physics, this.body, spec);
    this.arm = new Arm(physics, this, spec.arm, objects);

    this.group = new THREE.Group();
    scene.add(this.group);
    this._buildVisuals();
    scene.add(this.arm.group);

    this.safetyStop = false;
    this.ledState = 'idle';
  }

  _buildBasketColliders() {
    const b = this.spec.basket;
    const [sx, sy, sz] = b.size;
    const lc = [b.center[0], b.center[1], b.center[2] - BASE_Z];
    const t = 0.015;
    const parts = [
      [[lc[0], lc[1], lc[2] - sz / 2], [sx, sy, t]],
      [[lc[0] - sx / 2, lc[1], lc[2]], [t, sy, sz]],
      [[lc[0] + sx / 2, lc[1], lc[2]], [t, sy, sz]],
      [[lc[0], lc[1] - sy / 2, lc[2]], [sx, t, sz]],
      [[lc[0], lc[1] + sy / 2, lc[2]], [sx, t, sz]],
    ];
    for (const [pos, size] of parts) {
      const desc = this.physics.R.ColliderDesc.cuboid(size[0] / 2, size[1] / 2, size[2] / 2)
        .setTranslation(pos[0], pos[1], pos[2])
        .setMass(0.15)
        .setCollisionGroups(groups(GROUP_ROBOT, GROUP_WORLD));
      const col = this.physics.world.createCollider(desc, this.body);
      this.physics.registerMeta(col, { kind: 'robot', part: 'basket' });
    }
  }

  _buildVisuals() {
    const g = this.group;
    const c = this.spec.chassis;
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0xf2efe6, roughness: 0.4 });
    const darkMat = new THREE.MeshStandardMaterial({ color: 0x2e2e33, roughness: 0.7 });
    const accentMat = new THREE.MeshStandardMaterial({ color: 0x44464d, roughness: 0.5 });

    const tub = new THREE.Mesh(new THREE.BoxGeometry(c.size[0], c.size[1], c.size[2]), bodyMat);
    tub.castShadow = true;
    g.add(tub);

    const skirt = new THREE.Mesh(
      new THREE.BoxGeometry(c.size[0] + 0.03, c.size[1] + 0.03, 0.05), darkMat,
    );
    skirt.position.z = lz(0.08);
    g.add(skirt);

    this.ledMat = new THREE.MeshStandardMaterial({
      color: LED_COLORS.idle, emissive: LED_COLORS.idle, emissiveIntensity: 1.2,
    });
    const led = new THREE.Mesh(new THREE.BoxGeometry(c.size[0] + 0.035, c.size[1] + 0.035, 0.012), this.ledMat);
    led.position.z = lz(0.105);
    g.add(led);

    this.wheelMeshes = [];
    const wheelGeom = new THREE.CylinderGeometry(
      this.spec.wheels.radius, this.spec.wheels.radius, this.spec.wheels.width, 20,
    );
    const wheelMat = new THREE.MeshStandardMaterial({ color: 0x1c1c1f, roughness: 0.95 });
    for (const w of this.suspension.wheels) {
      const mesh = new THREE.Mesh(wheelGeom, wheelMat);
      mesh.castShadow = true;
      g.add(mesh);
      this.wheelMeshes.push({ mesh, wheel: w });
    }

    const standMat = accentMat;
    const standGeom = new THREE.CylinderGeometry(0.012, 0.012, this.spec.decks.deck1_z - 0.20, 8);
    for (const sx of [0.24, -0.24]) {
      for (const sy of [0.15, -0.15]) {
        const post = new THREE.Mesh(standGeom, standMat);
        post.rotation.x = Math.PI / 2;
        post.position.set(sx, sy, lz((0.20 + this.spec.decks.deck1_z) / 2));
        g.add(post);
      }
    }
    const deck1 = new THREE.Mesh(new THREE.BoxGeometry(0.54, 0.36, 0.012), bodyMat);
    deck1.position.z = lz(this.spec.decks.deck1_z);
    g.add(deck1);

    const armMount = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.07, 0.06, 16), accentMat);
    armMount.rotation.x = Math.PI / 2;
    armMount.position.set(this.spec.arm.base_offset[0], 0, lz(this.spec.decks.deck2_z + 0.03));
    g.add(armMount);

    const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.40, 10), accentMat);
    mast.rotation.x = Math.PI / 2;
    mast.position.set(this.spec.sensors.lidar.position[0], 0, lz(0.40));
    g.add(mast);
    const puck = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.045, 20), darkMat);
    puck.rotation.x = Math.PI / 2;
    puck.position.set(this.spec.sensors.lidar.position[0], 0, lz(this.spec.sensors.lidar.position[2]));
    g.add(puck);

    const camGeom = new THREE.BoxGeometry(0.04, 0.09, 0.025);
    for (const key of ['camera_front', 'camera_rear', 'camera_depth']) {
      const s = this.spec.sensors[key];
      const cam = new THREE.Mesh(camGeom, darkMat);
      cam.position.set(s.position[0], s.position[1], lz(s.position[2]));
      g.add(cam);
    }

    const sonarGeom = new THREE.CylinderGeometry(0.015, 0.015, 0.01, 12);
    for (const bearing of this.spec.sensors.sonar.bearings_deg) {
      const s = new THREE.Mesh(sonarGeom, darkMat);
      const rad = (bearing * Math.PI) / 180;
      s.rotation.z = Math.PI / 2;
      s.position.set(this.spec.sensors.sonar.nose_x, 0.12 * Math.sin(rad), lz(this.spec.sensors.sonar.height));
      g.add(s);
    }

    const basketMat = new THREE.MeshStandardMaterial({
      color: 0x6f8a99, roughness: 0.6, transparent: true, opacity: 0.85,
    });
    const b = this.spec.basket;
    const basket = new THREE.Mesh(new THREE.BoxGeometry(b.size[0], b.size[1], b.size[2]), basketMat);
    basket.position.set(b.center[0], b.center[1], lz(b.center[2]));
    g.add(basket);

    const estop = new THREE.Mesh(
      new THREE.CylinderGeometry(0.025, 0.025, 0.025, 12),
      new THREE.MeshStandardMaterial({ color: 0xd0312d, roughness: 0.4 }),
    );
    estop.rotation.x = Math.PI / 2;
    estop.position.set(-0.27, -0.14, lz(0.22));
    g.add(estop);
  }

  update(dt, isWetAt) {
    if (this.safetyStop) this.suspension.setTargets([0, 0, 0, 0, 0, 0]);
    this.suspension.update(dt, isWetAt);
    this.arm.update(dt);
    this._syncVisuals();
  }

  _syncVisuals() {
    const pos = this.body.translation();
    const rot = this.body.rotation();
    this.group.position.set(pos.x, pos.y, pos.z);
    this.group.quaternion.set(rot.x, rot.y, rot.z, rot.w);

    for (const { mesh, wheel } of this.wheelMeshes) {
      // Cylinder axis is local Y = the axle; spin about it.
      mesh.position.set(wheel.local.x, wheel.local.y, wheel.local.z - wheel.suspensionLen);
      mesh.rotation.set(0, wheel.spinAngle, 0);
    }

    const wanted = this.safetyStop ? 'estop'
      : this.arm.holding() ? 'picking'
        : this.isMoving() ? 'moving' : 'idle';
    if (wanted !== this.ledState) {
      this.ledState = wanted;
      this.ledMat.color.setHex(LED_COLORS[wanted]);
      this.ledMat.emissive.setHex(LED_COLORS[wanted]);
    }
  }

  isMoving() {
    const v = this.body.linvel();
    const w = this.body.angvel();
    return Math.hypot(v.x, v.y) > 0.03 || Math.abs(w.z) > 0.05;
  }

  /** base_link (z from ground) -> world */
  worldPoint(p) {
    const pos = this.body.translation();
    const rot = this.body.rotation();
    const local = { x: p[0], y: p[1], z: p[2] - BASE_Z };
    const r = rotateQuat(rot, local);
    return { x: pos.x + r.x, y: pos.y + r.y, z: pos.z + r.z };
  }

  worldDir(d) {
    return rotateQuat(this.body.rotation(), { x: d[0], y: d[1], z: d[2] });
  }

  pose() {
    const pos = this.body.translation();
    const rot = this.body.rotation();
    return { x: pos.x, y: pos.y, z: pos.z, yaw: yawFromQuat(rot), quat: rot };
  }
}

export function yawQuat(yaw) {
  return { w: Math.cos(yaw / 2), x: 0, y: 0, z: Math.sin(yaw / 2) };
}

export function yawFromQuat(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

const lz = (z) => z - BASE_Z;

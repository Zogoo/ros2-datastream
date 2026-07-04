import * as THREE from 'three';
import { WheelSuspension, rotateQuat } from '../physics/vehicle.js';
import { GROUP_OBJECT, GROUP_ROBOT, GROUP_WORLD, groups } from '../physics/world.js';
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
      .setCollisionGroups(groups(GROUP_ROBOT, GROUP_WORLD | GROUP_OBJECT))
      .setActiveEvents(physics.R.ActiveEvents.COLLISION_EVENTS | physics.R.ActiveEvents.CONTACT_FORCE_EVENTS)
      .setContactForceEventThreshold(1.0);
    this.tubCollider = physics.world.createCollider(tub, this.body);
    physics.registerMeta(this.tubCollider, { kind: 'robot', part: 'chassis' });

    this._buildBasketColliders();
    this._buildBumperRing();
    this.suspension = new WheelSuspension(physics, this.body, spec);
    this.arm = new Arm(physics, this, spec.arm, objects);

    this.group = new THREE.Group();
    scene.add(this.group);
    this._buildVisuals();
    scene.add(this.arm.group);

    this.safetyStop = false;
    this.ledState = 'idle';
  }

  /** Collect bin: the ENTIRE top deck behind the arm mount is an open-top
   *  tray, fully inside the chassis footprint (no overhang — the previous
   *  side basket bumped walls). The arm stows towels into it with a rear
   *  over-the-shoulder reach (negative-radial FK). For emptying, the whole
   *  tray hinges along its rear-TOP edge and swings up-and-over the tail — a
   *  "high dump" (the sweeper-truck mechanism): contents exit above the
   *  0.55 m floor-bin rim behind the robot. Colliders are repositioned wrt
   *  the chassis while tilting, so contained towels are physically pushed
   *  out by the rotating walls, not scripted. */
  _buildBasketColliders() {
    const b = this.spec.basket;
    const [sx, sy, sz] = b.size;
    const lc = [b.center[0], b.center[1], b.center[2] - BASE_Z];
    const t = b.wall_t ?? 0.015;
    // Hinge: rear-top edge (rim height), running along the body Y axis.
    this.binHinge = { x: lc[0] - sx / 2 - t / 2, z: lc[2] + sz / 2 };
    this.binTilt = 0;          // deg, actuator state (FE applies servo dynamics)
    this.binTiltTarget = 0;    // deg, target commanded by the base firmware
    this._lastBinTheta = 0;
    const parts = [
      [[lc[0], lc[1], lc[2] - sz / 2], [sx, sy, t]],
      [[lc[0] - sx / 2, lc[1], lc[2]], [t, sy, sz]],
      [[lc[0] + sx / 2, lc[1], lc[2]], [t, sy, sz]],
      [[lc[0], lc[1] - sy / 2, lc[2]], [sx, t, sz]],
      [[lc[0], lc[1] + sy / 2, lc[2]], [sx, t, sz]],
    ];
    this.binColliders = [];
    for (const [pos, size] of parts) {
      const desc = this.physics.R.ColliderDesc.cuboid(size[0] / 2, size[1] / 2, size[2] / 2)
        .setTranslation(pos[0], pos[1], pos[2])
        .setMass(0.15)
        .setCollisionGroups(groups(GROUP_ROBOT, GROUP_WORLD | GROUP_OBJECT));
      const col = this.physics.world.createCollider(desc, this.body);
      this.physics.registerMeta(col, { kind: 'robot', part: 'basket' });
      this.binColliders.push({ col, local: [...pos] });
    }
  }

  /** 360° physical bumper ring (Roomba-style): 8 segments — 4 faces + 4
   *  corners — standing 12 mm proud of the skirt at z 0.085. These colliders
   *  are the robot's outermost surface, so any push-back is taken by the
   *  bumper FIRST; the contact-force direction resolves which segment
   *  (micro-switch) fired. Segment meshes flash on a hit so the trigger is
   *  visible in the FE. */
  _buildBumperRing() {
    const c = this.spec.chassis;
    const hx = c.size[0] / 2 + 0.012;   // standoff from the chassis shell
    const hy = c.size[1] / 2 + 0.012;
    const t = 0.02;                      // ring thickness
    const h = 0.05;                      // band height
    const z = 0.085 - BASE_Z;
    const cornerLen = 0.11;
    // [sector, cx, cy, sizeX, sizeY, yawDeg]
    this.bumperLayout = [
      ['front', hx, 0, t, c.size[1] * 0.62, 0],
      ['rear', -hx, 0, t, c.size[1] * 0.62, 0],
      ['left', 0, hy, c.size[0] * 0.62, t, 0],
      ['right', 0, -hy, c.size[0] * 0.62, t, 0],
      ['front_left', hx - 0.045, hy - 0.045, t, cornerLen, -45],
      ['front_right', hx - 0.045, -(hy - 0.045), t, cornerLen, 45],
      ['rear_left', -(hx - 0.045), hy - 0.045, t, cornerLen, 45],
      ['rear_right', -(hx - 0.045), -(hy - 0.045), t, cornerLen, -45],
    ];
    for (const [sector, cx, cy, sx, sy, yawDeg] of this.bumperLayout) {
      void sector;
      const desc = this.physics.R.ColliderDesc.cuboid(sx / 2, sy / 2, h / 2)
        .setTranslation(cx, cy, z)
        .setRotation({
          w: Math.cos((yawDeg * Math.PI) / 360), x: 0, y: 0, z: Math.sin((yawDeg * Math.PI) / 360),
        })
        .setMass(0.05)
        .setFriction(0.1)
        .setCollisionGroups(groups(GROUP_ROBOT, GROUP_WORLD | GROUP_OBJECT))
        .setActiveEvents(this.physics.R.ActiveEvents.CONTACT_FORCE_EVENTS)
        .setContactForceEventThreshold(0.5);
      const col = this.physics.world.createCollider(desc, this.body);
      this.physics.registerMeta(col, { kind: 'robot', part: 'bumper' });
    }
  }

  /** Visual feedback for a bumper hit: flash the struck segment. */
  flashBumper(sector) {
    const seg = this.bumperMeshes?.[sector];
    if (!seg) return;
    seg.material.color.setHex(0xff7a1a);
    seg.material.emissive.setHex(0xff7a1a);
    seg.material.emissiveIntensity = 0.9;
    clearTimeout(seg.userData.flashTimer);
    seg.userData.flashTimer = setTimeout(() => {
      seg.material.color.setHex(0x1c1c1f);
      seg.material.emissive.setHex(0x000000);
      seg.material.emissiveIntensity = 0;
    }, 400);
  }

  setBinTiltTarget(deg) {
    if (Number.isFinite(deg)) this.binTiltTarget = Math.max(0, Math.min(120, deg));
  }

  /** Animate the dump servo toward its target and swing the bin's colliders +
   *  visual about the rear-top hinge (rotation about +Y: the tray pitches
   *  up-and-over the tail, opening ends up facing back-down, contents fall
   *  out behind the robot above the floor-bin rim). */
  _updateBin(dt) {
    const speed = this.spec.basket.tilt_speed_dps ?? 90;
    const delta = this.binTiltTarget - this.binTilt;
    if (delta !== 0) {
      this.binTilt += Math.sign(delta) * Math.min(Math.abs(delta), speed * dt);
    }
    const theta = (this.binTilt * Math.PI) / 180;
    if (theta === this._lastBinTheta) return;
    this._lastBinTheta = theta;
    if (this.binMesh) this.binMesh.rotation.y = theta;
    const { x: hx, z: hz } = this.binHinge;
    const cos = Math.cos(theta);
    const sin = Math.sin(theta);
    const rot = { w: Math.cos(theta / 2), x: 0, y: Math.sin(theta / 2), z: 0 };
    for (const { col, local } of this.binColliders) {
      const rx = local[0] - hx;
      const rz = local[2] - hz;
      col.setTranslationWrtParent({
        x: hx + rx * cos + rz * sin,
        y: local[1],
        z: hz - rx * sin + rz * cos,
      });
      col.setRotationWrtParent(rot);
    }
  }

  /** True when a world-frame point lies inside the (untilted) bin volume —
   *  the geometric truth behind the load-cell reading and the ground-truth
   *  in_basket flag. */
  binContains(p) {
    const pos = this.body.translation();
    const q = this.body.rotation();
    const conj = { w: q.w, x: -q.x, y: -q.y, z: -q.z };
    const local = rotateQuat(conj, { x: p.x - pos.x, y: p.y - pos.y, z: p.z - pos.z });
    const b = this.spec.basket;
    const lc = [b.center[0], b.center[1], b.center[2] - BASE_Z];
    return Math.abs(local.x - lc[0]) < b.size[0] / 2
      && Math.abs(local.y - lc[1]) < b.size[1] / 2
      && local.z > lc[2] - b.size[2] / 2 - 0.02
      && local.z < lc[2] + b.size[2] / 2 + 0.06;
  }

  /** Load-cell ground truth: total mass (kg) and count of free objects
   *  resting inside the bin volume. */
  binLoad() {
    let kg = 0;
    let count = 0;
    for (const item of this.objects.items) {
      if (item.held || item.binned) continue;
      if (this.binContains(item.body.translation())) {
        kg += item.body.mass();
        count += 1;
      }
    }
    return { kg, count };
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

    // 360° bumper ring: one mesh per segment (matches the collider layout),
    // visibly proud of the skirt; flashBumper() lights the struck segment.
    this.bumperMeshes = {};
    for (const [sector, cx, cy, sx, sy, yawDeg] of this.bumperLayout) {
      const mat = new THREE.MeshStandardMaterial({ color: 0x1c1c1f, roughness: 0.5 });
      const seg = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, 0.05), mat);
      seg.position.set(cx, cy, lz(0.085));
      seg.rotation.z = (yawDeg * Math.PI) / 180;
      g.add(seg);
      this.bumperMeshes[sector] = seg;
    }

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

    const lidarPos = this.spec.sensors.lidar.position;
    const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.40, 10), accentMat);
    mast.rotation.x = Math.PI / 2;
    mast.position.set(lidarPos[0], lidarPos[1], lz(0.40));
    g.add(mast);
    const puck = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.045, 20), darkMat);
    puck.rotation.x = Math.PI / 2;
    puck.position.set(lidarPos[0], lidarPos[1], lz(lidarPos[2]));
    g.add(puck);

    // Deck-1 hardware tray: SBC, motor-driver stack and IMU carrier live on
    // the lower deck, leaving the entire top deck to the collect bin.
    const sbc = new THREE.Mesh(new THREE.BoxGeometry(0.10, 0.07, 0.03), darkMat);
    sbc.position.set(-0.12, 0.08, lz(this.spec.decks.deck1_z + 0.022));
    g.add(sbc);
    const drivers = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.06, 0.04), accentMat);
    drivers.position.set(-0.12, -0.08, lz(this.spec.decks.deck1_z + 0.026));
    g.add(drivers);
    const imuChip = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.03, 0.012), darkMat);
    imuChip.position.set(0.05, 0.0, lz(this.spec.decks.deck1_z + 0.012));
    g.add(imuChip);

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

    // Collect bin: visibly open-top (4 walls + floor) spanning the top deck,
    // grouped and pivoted at the rear-top hinge so the maintenance dump-servo
    // tilt reads mechanically.
    const basketMat = new THREE.MeshStandardMaterial({
      color: 0x6f8a99, roughness: 0.6, transparent: true, opacity: 0.7, side: THREE.DoubleSide,
    });
    const b = this.spec.basket;
    const [bx, by, bz] = b.size;
    const bt = b.wall_t ?? 0.015;
    this.binMesh = new THREE.Group();
    this.binMesh.position.set(this.binHinge.x, 0, this.binHinge.z);
    const binPanels = [
      [[0, 0, -bz / 2], [bx, by, bt]],
      [[-bx / 2, 0, 0], [bt, by, bz]],
      [[bx / 2, 0, 0], [bt, by, bz]],
      [[0, -by / 2, 0], [bx, bt, bz]],
      [[0, by / 2, 0], [bx, bt, bz]],
    ];
    const lc = [b.center[0] - this.binHinge.x, b.center[1], lz(b.center[2]) - this.binHinge.z];
    for (const [pos, size] of binPanels) {
      const panel = new THREE.Mesh(new THREE.BoxGeometry(size[0], size[1], size[2]), basketMat);
      panel.position.set(lc[0] + pos[0], lc[1] + pos[1], lc[2] + pos[2]);
      this.binMesh.add(panel);
    }
    // Hinge bar runs along body Y at the group origin (cylinder axis = Y).
    const hingeBar = new THREE.Mesh(
      new THREE.CylinderGeometry(0.012, 0.012, by + 0.04, 10), accentMat,
    );
    this.binMesh.add(hingeBar);
    g.add(this.binMesh);

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
    this._updateBin(dt);
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

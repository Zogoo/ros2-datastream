import { GROUP_WORLD, groups } from './world.js';

const QUERY_SUSPENSION = groups(0xffff, GROUP_WORLD);

/** Custom raycast suspension for the 6-wheel skid-steer base.
 *
 * Per wheel and per physics tick:
 *   - cast a ray down from the chassis attach point
 *   - spring force  F = k * compression - c * compressionVelocity
 *   - longitudinal traction toward the commanded wheel surface speed
 *   - lateral friction resisting side slip (reduced so skid-steer can rotate)
 *
 * Wheel targets arrive as rad/s from /base/wheel_targets and pass through a
 * first-order servo lag, so encoders show realistic tracking error.
 */
export class WheelSuspension {
  constructor(physics, chassisBody, spec) {
    this.physics = physics;
    this.body = chassisBody;
    this.spec = spec.wheels;
    this.chassisMass = spec.chassis.mass;

    this.wheels = [];
    const { axle_x: axles, track_width: track, suspension } = this.spec;
    for (const x of axles) {
      for (const side of [1, -1]) {
        this.wheels.push({
          local: { x, y: (side * track) / 2, z: suspension.attach_z },
          side,
          targetRadps: 0,
          actualRadps: 0,
          spinAngle: 0,
          encoderAngle: 0,
          suspensionLen: suspension.rest_length,
          inContact: false,
          normalForce: 0,
          wetZone: false,
        });
      }
    }
  }

  /** wheel order: 0..2 left (front,mid,rear), 3..5 right — matches base_controller. */
  setTargets(radpsArray) {
    const left = [this.wheelAt(0, 1), this.wheelAt(1, 1), this.wheelAt(2, 1)];
    const right = [this.wheelAt(0, -1), this.wheelAt(1, -1), this.wheelAt(2, -1)];
    left.forEach((w, i) => { w.targetRadps = radpsArray[i] ?? 0; });
    right.forEach((w, i) => { w.targetRadps = radpsArray[3 + i] ?? 0; });
  }

  wheelAt(axleIdx, side) {
    return this.wheels[axleIdx * 2 + (side === 1 ? 0 : 1)];
  }

  update(dt, isWetAt) {
    const body = this.body;
    const rot = body.rotation();
    const pos = body.translation();
    const linvel = body.linvel();
    const angvel = body.angvel();
    const { radius, suspension, friction, drive_gain: driveGain, servo_lag_tau_s: tau } = this.spec;

    const up = rotateQuat(rot, { x: 0, y: 0, z: 1 });
    const fwd = rotateQuat(rot, { x: 1, y: 0, z: 0 });
    const left = rotateQuat(rot, { x: 0, y: 1, z: 0 });
    const down = { x: -up.x, y: -up.y, z: -up.z };
    const alpha = 1 - Math.exp(-dt / tau);

    for (const w of this.wheels) {
      w.actualRadps += (w.targetRadps - w.actualRadps) * alpha;
      w.spinAngle += w.actualRadps * dt;
      w.encoderAngle += w.actualRadps * dt;

      const attach = addVec(pos, rotateQuat(rot, w.local));
      const maxToi = suspension.rest_length + suspension.travel + radius;
      const hit = this.physics.castRay(attach, down, maxToi, QUERY_SUSPENSION, body);

      if (!hit) {
        w.inContact = false;
        w.normalForce = 0;
        w.suspensionLen = suspension.rest_length + suspension.travel;
        continue;
      }
      const suspLen = Math.max(0, hit.toi - radius);
      w.suspensionLen = suspLen;
      w.inContact = true;

      const compression = Math.min(suspension.travel, Math.max(0, suspension.rest_length - suspLen));
      const pointVel = velocityAt(linvel, angvel, attach, pos);
      const compVel = -dot(pointVel, up);
      let springF = suspension.stiffness * compression + suspension.damping * compVel;
      if (springF < 0) springF = 0;
      w.normalForce = springF;

      const impulse = scaleVec(up, springF * dt);
      body.applyImpulseAtPoint(impulse, attach, true);

      // Traction at the contact point
      const contact = addVec(attach, scaleVec(down, hit.toi));
      w.wetZone = isWetAt ? isWetAt(contact.x, contact.y) : false;
      const mu = w.wetZone ? friction.wet : friction.dry;
      const vContact = velocityAt(linvel, angvel, contact, pos);
      const vLong = dot(vContact, fwd);
      const vLat = dot(vContact, left);

      const targetSurface = w.actualRadps * radius;
      const maxTraction = mu * springF;
      let fLong = driveGain * (targetSurface - vLong) * (this.chassisMass / 6);
      fLong = clamp(fLong, -maxTraction, maxTraction);

      let fLat = -driveGain * vLat * (this.chassisMass / 6);
      const maxLat = maxTraction * friction.lateral_factor;
      fLat = clamp(fLat, -maxLat, maxLat);

      const tractionImpulse = addVec(scaleVec(fwd, fLong * dt), scaleVec(left, fLat * dt));
      body.applyImpulseAtPoint(tractionImpulse, contact, true);
    }
  }

  /** Per-side mean surface speed (m/s) read from the lagged servo state. */
  sideSurfaceSpeeds() {
    const { radius } = this.spec;
    const mean = (idxs) => idxs.reduce((s, i) => s + this.wheels[i].actualRadps, 0) / idxs.length;
    return { left: mean([0, 2, 4]) * radius, right: mean([1, 3, 5]) * radius };
  }
}

const dot = (a, b) => a.x * b.x + a.y * b.y + a.z * b.z;
const addVec = (a, b) => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z });
const scaleVec = (a, s) => ({ x: a.x * s, y: a.y * s, z: a.z * s });
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

function velocityAt(linvel, angvel, point, center) {
  const r = { x: point.x - center.x, y: point.y - center.y, z: point.z - center.z };
  return {
    x: linvel.x + angvel.y * r.z - angvel.z * r.y,
    y: linvel.y + angvel.z * r.x - angvel.x * r.z,
    z: linvel.z + angvel.x * r.y - angvel.y * r.x,
  };
}

export function rotateQuat(q, v) {
  const { w, x, y, z } = q;
  const ix = w * v.x + y * v.z - z * v.y;
  const iy = w * v.y + z * v.x - x * v.z;
  const iz = w * v.z + x * v.y - y * v.x;
  const iw = -x * v.x - y * v.y - z * v.z;
  return {
    x: ix * w + iw * -x + iy * -z - iz * -y,
    y: iy * w + iw * -y + iz * -x - ix * -z,
    z: iz * w + iw * -z + ix * -y - iy * -x,
  };
}

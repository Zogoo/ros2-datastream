import { TOPICS } from '../ros/topics.js';
import { rotateQuat } from '../physics/vehicle.js';

/** MEMS IMU read off the chassis rigid body: gyro from angvel, accel from
 *  delta-v plus gravity projected into the body frame. White noise + slow bias
 *  random-walk. Suspension oscillation and step impacts appear for free.
 *
 *  Orientation is what an IMU without a magnetometer can actually estimate:
 *  a Mahony complementary filter over the NOISY, BIASED gyro and accel. Roll
 *  and pitch are gravity-corrected (the safety tilt check stays valid); yaw is
 *  gyro-integrated from power-on (starts at 0, drifts with the bias) — it is
 *  NOT the map heading. Covariance says so. */
const MAHONY_KP = 2.0;           // accel correction gain (rad/s per unit error) — ~0.5 s settle
const ZERO = { x: 0, y: 0, z: 0 };
const add = (a, b) => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z });
const ACCEL_TRUST_BAND = 0.1;    // only trust gravity when |a| is within 10 % of g

export class ImuSensor {
  constructor(spec, robot, rng, ros, clock) {
    this.spec = spec.sensors.imu;
    this.robot = robot;
    this.rng = rng;
    this.ros = ros;
    this.clock = clock;
    this.accumulator = 0;
    this.sinceLast = 0;
    this.lastVel = { x: 0, y: 0, z: 0 };
    this.gyroBias = { x: 0, y: 0, z: 0 };
    this.accelBias = { x: 0, y: 0, z: 0 };
    this.lastTilt = 0;
    this.qEst = null;   // filter state, initialised level at yaw 0 on first sample
  }

  update(dt) {
    const body = this.robot.body;
    const q = body.rotation();
    const angvel = body.angvel();
    const linvel = body.linvel();
    const conj = { w: q.w, x: -q.x, y: -q.y, z: -q.z };

    // Internal high-rate loop (a MEMS IMU samples at kHz internally and
    // band-limits its outputs): every physics step propagates the attitude
    // filter and accumulates gyro/accel for the published average. Point-
    // sampling instantaneous angvel at 50 Hz aliased one-step impact spikes
    // into a phantom 30 deg tilt that held the safety latch.
    const accelWorld = dt > 0 ? {
      x: (linvel.x - this.lastVel.x) / dt,
      y: (linvel.y - this.lastVel.y) / dt,
      z: (linvel.z - this.lastVel.z) / dt + 9.81,
    } : { x: 0, y: 0, z: 9.81 };
    this.lastVel = { ...linvel };
    const gyroBody = rotateQuat(conj, angvel);
    const accelBody = rotateQuat(conj, accelWorld);
    const gyroB = add(gyroBody, this.gyroBias);
    const accelB = add(accelBody, this.accelBias);
    this._filter(gyroB, accelB, q, dt);
    this._sum = this._sum ?? { g: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } };
    for (const k of ['x', 'y', 'z']) {
      this._sum.g[k] += gyroB[k] * dt;
      this._sum.a[k] += accelB[k] * dt;
    }

    const period = 1 / this.spec.hz;
    this.accumulator += dt;
    this.sinceLast += dt;
    if (this.accumulator < period) return;
    // Carry the remainder: a 50 Hz sensor on a 60 Hz physics step publishes
    // at 50 Hz on average (resetting to 0 here used to halve it to 30 Hz).
    this.accumulator -= period;
    const elapsed = this.sinceLast;
    this.sinceLast = 0;
    const avg = (v) => ({ x: v.x / elapsed, y: v.y / elapsed, z: v.z / elapsed });
    const gyroAvg = avg(this._sum.g);
    const accelAvg = avg(this._sum.a);
    this._sum = null;

    for (const axis of ['x', 'y', 'z']) {
      this.gyroBias[axis] += this.rng.gaussian(0, this.spec.bias_walk_std);
      this.accelBias[axis] += this.rng.gaussian(0, this.spec.bias_walk_std);
    }

    const up = rotateQuat(q, { x: 0, y: 0, z: 1 });
    this.lastTilt = Math.acos(Math.max(-1, Math.min(1, up.z))) * (180 / Math.PI);

    const est = this.qEst;
    this.ros.publish(TOPICS.imu, {
      header: { stamp: this.clock.stamp(), frame_id: this.spec.frame_id },
      orientation: { x: round4(est.x), y: round4(est.y), z: round4(est.z), w: round4(est.w) },
      // roll/pitch ~0.6 deg; yaw is relative + drifting (large variance)
      orientation_covariance: [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 0.1],
      angular_velocity: this._noisy(gyroAvg, ZERO, this.spec.gyro_noise_std),
      angular_velocity_covariance: cov(this.spec.gyro_noise_std),
      linear_acceleration: this._noisy(accelAvg, ZERO, this.spec.accel_noise_std),
      linear_acceleration_covariance: cov(this.spec.accel_noise_std),
    });
  }

  /** Mahony complementary filter step on the measured gyro/accel. */
  _filter(gyro, accel, qTrue, dt) {
    if (!this.qEst) {
      // power-on: roll/pitch from gravity (as the filter would converge to),
      // heading defined as 0
      const yaw = Math.atan2(2 * (qTrue.w * qTrue.z + qTrue.x * qTrue.y), 1 - 2 * (qTrue.y * qTrue.y + qTrue.z * qTrue.z));
      const unYaw = { w: Math.cos(-yaw / 2), x: 0, y: 0, z: Math.sin(-yaw / 2) };
      this.qEst = quatMul(unYaw, qTrue);
    }
    const qe = this.qEst;
    let wx = gyro.x;
    let wy = gyro.y;
    let wz = gyro.z;
    const an = Math.hypot(accel.x, accel.y, accel.z);
    if (Math.abs(an - 9.81) < 9.81 * ACCEL_TRUST_BAND) {
      // estimated world-up in the body frame vs measured specific force
      const v = rotateQuat({ w: qe.w, x: -qe.x, y: -qe.y, z: -qe.z }, { x: 0, y: 0, z: 1 });
      const ax = accel.x / an;
      const ay = accel.y / an;
      const az = accel.z / an;
      wx += MAHONY_KP * (ay * v.z - az * v.y);
      wy += MAHONY_KP * (az * v.x - ax * v.z);
      wz += MAHONY_KP * (ax * v.y - ay * v.x);
    }
    const h = dt / 2;
    const next = quatMul(qe, { w: 1, x: wx * h, y: wy * h, z: wz * h });
    const n = Math.hypot(next.w, next.x, next.y, next.z);
    this.qEst = { w: next.w / n, x: next.x / n, y: next.y / n, z: next.z / n };
    return this.qEst;
  }

  _noisy(v, bias, std) {
    return {
      x: round4(v.x + bias.x + this.rng.gaussian(0, std)),
      y: round4(v.y + bias.y + this.rng.gaussian(0, std)),
      z: round4(v.z + bias.z + this.rng.gaussian(0, std)),
    };
  }
}

const cov = (std) => [std * std, 0, 0, 0, std * std, 0, 0, 0, std * std];
const round4 = (v) => Math.round(v * 10000) / 10000;
const quatMul = (a, b) => ({
  w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
  y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
  z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
});

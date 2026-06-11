import { TOPICS } from '../ros/topics.js';
import { rotateQuat } from '../physics/vehicle.js';

/** IMU read straight off the chassis rigid body: gyro from angvel, accel from
 *  delta-v plus gravity projected into the body frame. White noise + slow bias
 *  random-walk. Suspension oscillation and step impacts appear for free. */
export class ImuSensor {
  constructor(spec, robot, rng, ros, clock) {
    this.spec = spec.sensors.imu;
    this.robot = robot;
    this.rng = rng;
    this.ros = ros;
    this.clock = clock;
    this.accumulator = 0;
    this.lastVel = { x: 0, y: 0, z: 0 };
    this.gyroBias = { x: 0, y: 0, z: 0 };
    this.accelBias = { x: 0, y: 0, z: 0 };
    this.lastTilt = 0;
  }

  update(dt) {
    this.accumulator += dt;
    if (this.accumulator < 1 / this.spec.hz) return;
    const period = this.accumulator;
    this.accumulator = 0;

    const body = this.robot.body;
    const q = body.rotation();
    const angvel = body.angvel();
    const linvel = body.linvel();

    const accelWorld = {
      x: (linvel.x - this.lastVel.x) / period,
      y: (linvel.y - this.lastVel.y) / period,
      z: (linvel.z - this.lastVel.z) / period + 9.81,
    };
    this.lastVel = { ...linvel };

    const conj = { w: q.w, x: -q.x, y: -q.y, z: -q.z };
    const accelBody = rotateQuat(conj, accelWorld);
    const gyroBody = rotateQuat(conj, angvel);

    for (const axis of ['x', 'y', 'z']) {
      this.gyroBias[axis] += this.rng.gaussian(0, this.spec.bias_walk_std);
      this.accelBias[axis] += this.rng.gaussian(0, this.spec.bias_walk_std);
    }

    const up = rotateQuat(q, { x: 0, y: 0, z: 1 });
    this.lastTilt = Math.acos(Math.max(-1, Math.min(1, up.z))) * (180 / Math.PI);

    this.ros.publish(TOPICS.imu, {
      header: { stamp: this.clock.stamp(), frame_id: this.spec.frame_id },
      orientation: { x: q.x, y: q.y, z: q.z, w: q.w },
      orientation_covariance: [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001],
      angular_velocity: this._noisy(gyroBody, this.gyroBias, this.spec.gyro_noise_std),
      angular_velocity_covariance: cov(this.spec.gyro_noise_std),
      linear_acceleration: this._noisy(accelBody, this.accelBias, this.spec.accel_noise_std),
      linear_acceleration_covariance: cov(this.spec.accel_noise_std),
    });
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

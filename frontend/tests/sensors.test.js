import { describe, expect, it } from 'vitest';
import spec from '../../shared/robot_spec.json';
import { ImuSensor } from '../src/sensors/imu.js';
import { LidarSensor } from '../src/sensors/lidar.js';
import { createRng } from '../src/core/rng.js';

const DT = 1 / 60;
const clock = { step: DT, stamp: () => ({ sec: 0, nanosec: 0 }) };
const capture = () => {
  const msgs = [];
  return { msgs, publish: (_topic, m) => msgs.push(m) };
};
const yawQ = (yaw) => ({ w: Math.cos(yaw / 2), x: 0, y: 0, z: Math.sin(yaw / 2) });

describe('IMU', () => {
  const robotAt = (yaw) => ({
    body: {
      rotation: () => yawQ(yaw),
      angvel: () => ({ x: 0, y: 0, z: 0 }),
      linvel: () => ({ x: 0, y: 0, z: 0 }),
    },
  });

  it('publishes at its spec rate on a 60 Hz physics step (not 30 Hz)', () => {
    const ros = capture();
    const imu = new ImuSensor(spec, robotAt(0), createRng(1), ros, clock);
    for (let i = 0; i < 600; i++) imu.update(DT);   // 10 s
    expect(ros.msgs.length).toBeGreaterThanOrEqual(spec.sensors.imu.hz * 10 - 1);
    expect(ros.msgs.length).toBeLessThanOrEqual(spec.sensors.imu.hz * 10 + 1);
  });

  it('reports heading relative to power-on, not the true map yaw', () => {
    const ros = capture();
    const imu = new ImuSensor(spec, robotAt(2.0), createRng(1), ros, clock);
    for (let i = 0; i < 60; i++) imu.update(DT);
    const q = ros.msgs.at(-1).orientation;
    const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    expect(Math.abs(yaw)).toBeLessThan(0.05);
    expect(ros.msgs.at(-1).orientation_covariance[8]).toBeGreaterThan(0.01);
  });

  it('keeps roll/pitch level when the robot is level (safety tilt stays valid)', () => {
    const ros = capture();
    const imu = new ImuSensor(spec, robotAt(1.0), createRng(3), ros, clock);
    for (let i = 0; i < 60 * 30; i++) imu.update(DT);
    const q = ros.msgs.at(-1).orientation;
    const upZ = 1 - 2 * (q.x * q.x + q.y * q.y);
    expect(Math.acos(Math.min(1, upZ)) * (180 / Math.PI)).toBeLessThan(2);
  });
});

describe('IMU impact transient', () => {
  it('a one-step impact jolt does not leave a phantom tilt (safety reset must not be refused)', () => {
    // Body jolts to 15 deg roll in ONE physics step, then settles back over 3
    // steps — the angvel spikes Rapier produces on a hard collision.
    const rollAt = [0, 0, 0, 15, 10, 5, 0];
    let step = 0;
    let roll = 0;
    let rate = 0;
    const robot = {
      body: {
        rotation: () => ({ w: Math.cos((roll * Math.PI) / 360), x: Math.sin((roll * Math.PI) / 360), y: 0, z: 0 }),
        angvel: () => ({ x: rate, y: 0, z: 0 }),
        linvel: () => ({ x: 0, y: 0, z: 0 }),
      },
    };
    const ros = capture();
    const imu = new ImuSensor(spec, robot, createRng(5), ros, clock);
    for (let i = 0; i < 120; i++) {
      const next = rollAt[Math.min(step + 1, rollAt.length - 1)];
      rate = (((next - roll) * Math.PI) / 180) / DT;
      roll = next;
      step += 1;
      imu.update(DT);
    }
    const q = ros.msgs.at(-1).orientation;
    const tilt = Math.acos(Math.min(1, 1 - 2 * (q.x * q.x + q.y * q.y))) * (180 / Math.PI);
    expect(tilt).toBeLessThan(2);
  });
});

describe('LIDAR', () => {
  it('angle_max matches the beam count (REP-138: n = (max - min) / inc + 1)', () => {
    const ros = capture();
    const physics = { castRay: () => null };
    const robot = {
      worldPoint: () => ({ x: 0, y: 0, z: 0.66 }),
      worldDir: ([x, y, z]) => ({ x, y, z }),
    };
    const world = { steamDensityAt: () => 0 };
    const lidar = new LidarSensor(spec, physics, robot, world, createRng(1), ros, clock);
    for (let i = 0; i < 60; i++) lidar.update();
    const scan = ros.msgs[0];
    const n = Math.round((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1;
    expect(n).toBe(scan.ranges.length);
  });
});

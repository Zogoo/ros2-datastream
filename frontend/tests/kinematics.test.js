import { describe, it, expect } from 'vitest';
import { armFk, gripperOpening, isGripperClosed, servoStep, integrateOdometry } from '../src/robot/kinematics.js';
import spec from '../../shared/robot_spec.json';

const arm = spec.arm;

describe('arm forward kinematics', () => {
  it('HOME pose points straight up over the shoulder', () => {
    const fk = armFk([90, 90, 90, 90, 90, 70], arm);
    expect(fk.fingertip[0]).toBeCloseTo(arm.base_offset[0], 3);
    expect(fk.fingertip[1]).toBeCloseTo(0, 3);
    const reach = arm.links.upper + arm.links.forearm + arm.links.wrist;
    expect(fk.fingertip[2]).toBeCloseTo(arm.shoulder_z + reach, 3);
  });

  it('PICK_SCOOP reaches the floor in front of the robot', () => {
    const fk = armFk([90, 157, 57, 77, 90, 80], arm);
    expect(fk.fingertip[2]).toBeLessThan(0.06);
    expect(fk.fingertip[2]).toBeGreaterThan(-0.02);
    const forward = fk.fingertip[0] - arm.base_offset[0];
    expect(forward).toBeGreaterThan(0.3);
    expect(forward).toBeLessThan(0.55);
    expect(Math.abs(fk.fingertip[1])).toBeLessThan(0.01);
  });

  it('DROP_BASKET hovers over the side basket', () => {
    const fk = armFk([178, 100, 85, 85, 90, 12], arm);
    const basket = spec.basket;
    expect(Math.abs(fk.fingertip[0] - basket.center[0])).toBeLessThan(basket.size[0] / 2);
    expect(Math.abs(fk.fingertip[1] - basket.center[1])).toBeLessThan(basket.size[1] / 2 + 0.04);
    expect(fk.fingertip[2]).toBeGreaterThan(basket.rim_z);
  });

  it('pan rotates the fingertip about Z', () => {
    const fwd = armFk([90, 130, 57, 63, 90, 80], arm);
    const left = armFk([180, 130, 57, 63, 90, 80], arm);
    const rFwd = Math.hypot(fwd.fingertip[0] - arm.base_offset[0], fwd.fingertip[1]);
    const rLeft = Math.hypot(left.fingertip[0] - arm.base_offset[0], left.fingertip[1]);
    expect(rFwd).toBeCloseTo(rLeft, 5);
    expect(left.fingertip[1]).toBeGreaterThan(0.2);
  });

  it('STOW keeps every joint below the LIDAR optical plane', () => {
    const fk = armFk([90, 30, 90, 90, 90, 10], arm);
    const lidarZ = spec.sensors.lidar.position[2];
    for (const key of ['elbow', 'wrist', 'fingertip']) {
      expect(fk[key][2]).toBeLessThan(lidarZ - 0.03);
      expect(fk[key][2]).toBeGreaterThan(0.2);
    }
  });
});

describe('gripper model', () => {
  it('maps servo angle to opening width', () => {
    expect(gripperOpening(0, arm)).toBe(0);
    expect(gripperOpening(180, arm)).toBeCloseTo(arm.gripper.max_opening_m, 6);
  });

  it('close threshold matches the firmware PICK_GRIP pose', () => {
    expect(isGripperClosed(12, arm)).toBe(true);
    expect(isGripperClosed(70, arm)).toBe(false);
  });
});

describe('servo lag', () => {
  it('converges to target without overshoot', () => {
    let pos = 90;
    for (let i = 0; i < 200; i++) pos = servoStep(pos, 150, 1 / 60, 0.08, 200);
    expect(pos).toBeCloseTo(150, 1);
  });

  it('respects the joint speed limit', () => {
    const next = servoStep(0, 180, 1 / 60, 0.001, 200);
    expect(next - 0).toBeLessThanOrEqual(200 / 60 + 1e-9);
  });
});

describe('skid-steer odometry', () => {
  it('drives straight when both sides match', () => {
    let pose = { x: 0, y: 0, yaw: 0 };
    for (let i = 0; i < 60; i++) pose = integrateOdometry(pose, 0.3, 0.3, 0.47, 1 / 60);
    expect(pose.x).toBeCloseTo(0.3, 3);
    expect(pose.y).toBeCloseTo(0, 6);
    expect(pose.yaw).toBeCloseTo(0, 6);
  });

  it('turns in place with opposite side speeds', () => {
    let pose = { x: 0, y: 0, yaw: 0 };
    for (let i = 0; i < 60; i++) pose = integrateOdometry(pose, -0.235, 0.235, 0.47, 1 / 60);
    expect(Math.hypot(pose.x, pose.y)).toBeLessThan(0.01);
    expect(pose.yaw).toBeCloseTo(1.0, 2);
  });
});

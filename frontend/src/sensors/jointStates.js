import { TOPICS } from '../ros/topics.js';

const ARM_JOINT_NAMES = [
  'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
  'wrist_pitch_joint', 'wrist_roll_joint', 'gripper_joint',
];
const WHEEL_JOINT_NAMES = [
  'wheel_front_left', 'wheel_front_right', 'wheel_mid_left',
  'wheel_mid_right', 'wheel_rear_left', 'wheel_rear_right',
];

/** Measured joint state: lagging servo positions (deg -> rad) + wheel encoder
 *  angles, so target-vs-measured tracking error is observable. */
export class JointStateSensor {
  constructor(spec, robot, ros, clock) {
    this.hz = spec.sensors.joint_states.hz;
    this.robot = robot;
    this.ros = ros;
    this.clock = clock;
    this.accumulator = 0;
  }

  update(dt) {
    this.accumulator += dt;
    if (this.accumulator < 1 / this.hz) return;
    this.accumulator %= 1 / this.hz;

    const armRad = this.robot.arm.current.map((d) => round3(((d - 90) * Math.PI) / 180));
    const wheels = this.robot.suspension.wheels;
    const wheelAngles = wheels.map((w) => round3(w.encoderAngle));
    const wheelVels = wheels.map((w) => round3(w.actualRadps));

    this.ros.publish(TOPICS.jointStates, {
      header: { stamp: this.clock.stamp(), frame_id: '' },
      name: [...ARM_JOINT_NAMES, ...WHEEL_JOINT_NAMES],
      position: [...armRad, ...wheelAngles],
      velocity: [0, 0, 0, 0, 0, 0, ...wheelVels],
      effort: new Array(12).fill(0),
    });
  }
}

const round3 = (v) => Math.round(v * 1000) / 1000;

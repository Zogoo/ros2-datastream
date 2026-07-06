import { TOPICS } from '../ros/topics.js';

const ARM_JOINT_NAMES = [
  'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
  'wrist_pitch_joint', 'wrist_roll_joint', 'gripper_joint',
];
const WHEEL_JOINT_NAMES = ['wheel_left', 'wheel_right'];

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
    // Only the two driven wheels carry encoders; the front casters are unmeasured.
    const driven = this.robot.suspension.driven;
    const wheelAngles = driven.map((w) => round3(w.encoderAngle));
    const wheelVels = driven.map((w) => round3(w.actualRadps));

    this.ros.publish(TOPICS.jointStates, {
      header: { stamp: this.clock.stamp(), frame_id: '' },
      name: [...ARM_JOINT_NAMES, ...WHEEL_JOINT_NAMES],
      position: [...armRad, ...wheelAngles],
      velocity: [...ARM_JOINT_NAMES.map(() => 0), ...wheelVels],
      effort: [...this.robot.arm.effort, ...driven.map(() => 0)],
    });
  }
}

const round3 = (v) => Math.round(v * 1000) / 1000;

import { TOPICS } from '../ros/topics.js';
import { integrateOdometry } from '../robot/kinematics.js';
import { yawQuat } from '../robot/robot.js';

/** Wheel-encoder odometry: integrates the *measured* wheel speeds through
 *  skid-steer kinematics. NOT ground truth — skid turns slip by construction,
 *  so this drifts exactly like a real robot. Ground truth goes out separately
 *  on /ground_truth/pose for drift evaluation. */
export class OdomSensor {
  constructor(spec, robot, ros, clock) {
    this.spec = spec;
    this.robot = robot;
    this.ros = ros;
    this.clock = clock;
    this.hz = spec.sensors.odom.hz;
    this.accumulator = 0;
    const p = robot.pose();
    this.pose = { x: p.x, y: p.y, yaw: p.yaw, v: 0, w: 0 };
    this.wheelAngles = [0, 0];
  }

  update(dt) {
    const { left, right } = this.robot.suspension.sideSurfaceSpeeds();
    this.pose = integrateOdometry(this.pose, left, right, this.spec.wheels.track_width, dt);
    this.wheelAngles[0] += (left / this.spec.wheels.radius) * dt;
    this.wheelAngles[1] += (right / this.spec.wheels.radius) * dt;

    this.accumulator += dt;
    if (this.accumulator < 1 / this.hz) return;
    this.accumulator %= 1 / this.hz;
    this._publish();
  }

  _publish() {
    const stamp = this.clock.stamp();
    const q = yawQuat(this.pose.yaw);
    this.ros.publish(TOPICS.odom, {
      header: { stamp, frame_id: 'odom' },
      child_frame_id: 'base_link',
      pose: {
        pose: {
          position: { x: round3(this.pose.x), y: round3(this.pose.y), z: 0 },
          orientation: q,
        },
        covariance: new Array(36).fill(0),
      },
      twist: {
        twist: {
          linear: { x: round3(this.pose.v), y: 0, z: 0 },
          angular: { x: 0, y: 0, z: round3(this.pose.w) },
        },
        covariance: new Array(36).fill(0),
      },
    });
    this.ros.publish(TOPICS.tf, {
      transforms: [{
        header: { stamp, frame_id: 'odom' },
        child_frame_id: 'base_link',
        transform: {
          translation: { x: round3(this.pose.x), y: round3(this.pose.y), z: 0 },
          rotation: q,
        },
      }],
    });

    const gt = this.robot.pose();
    this.ros.publish(TOPICS.groundTruthPose, {
      header: { stamp, frame_id: 'map' },
      pose: {
        position: { x: round3(gt.x), y: round3(gt.y), z: round3(gt.z) },
        orientation: gt.quat,
      },
    });
  }
}

const round3 = (v) => Math.round(v * 1000) / 1000;

import { TOPICS } from '../ros/topics.js';

/** Ground-truth object states + sim heartbeat for dataset labeling, detection
 *  evaluation and paused-world detection on the ROS side. */
export class GroundTruthPublisher {
  constructor(objects, robot, ros, clock) {
    this.objects = objects;
    this.robot = robot;
    this.ros = ros;
    this.clock = clock;
    this.accumulator = 0;
    this.statusAccumulator = 0;
    this.frame = 0;
  }

  update(dt, fps) {
    this.accumulator += dt;
    if (this.accumulator >= 0.5) {
      this.accumulator %= 0.5;
      this.frame += 1;
      this.ros.publish(TOPICS.groundTruthObjects, {
        data: JSON.stringify({
          frame: this.frame,
          timestamp: new Date().toISOString(),
          objects: this.objects.groundTruth(),
        }),
      });
    }

    this.statusAccumulator += dt;
    if (this.statusAccumulator >= 1.0) {
      this.statusAccumulator %= 1.0;
      this.ros.publish(TOPICS.simStatus, {
        data: JSON.stringify({
          alive: true,
          sim_time: Math.round(this.clock.simTime * 100) / 100,
          fps: Math.round(fps),
          timestamp: new Date().toISOString(),
        }),
      });
    }
  }
}

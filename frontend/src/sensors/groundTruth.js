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
    // Per-page-load nonce: lets ROS nodes detect a genuine session restart by
    // an id *change* rather than the fragile sim_time-regression heuristic,
    // which thrashed when more than one FE tab published interleaved clocks.
    this.sessionId = `${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 10)}`;
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
          session_id: this.sessionId,
          sim_time: Math.round(this.clock.simTime * 100) / 100,
          fps: Math.round(fps),
          timestamp: new Date().toISOString(),
        }),
      });
    }
  }
}

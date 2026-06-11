import { TOPICS } from '../ros/topics.js';
import { GROUP_WORLD, groups } from '../physics/world.js';

const QUERY = groups(0xffff, GROUP_WORLD);

/** Ultrasonic ring: each transducer casts a fan of rays inside its cone and
 *  reports the minimum hit — the real beam-width artifact (wide objects read
 *  closer). Steam-immune by physics: sound, not light. */
export class SonarSensor {
  constructor(spec, physics, robot, rng, ros, clock) {
    this.spec = spec.sensors.sonar;
    this.physics = physics;
    this.robot = robot;
    this.rng = rng;
    this.ros = ros;
    this.clock = clock;
    this.accumulator = 0;
    this.lastRanges = this.spec.bearings_deg.map(() => this.spec.range_max);
  }

  update(dt) {
    this.accumulator += dt;
    if (this.accumulator < 1 / this.spec.hz) return;
    this.accumulator %= 1 / this.spec.hz;

    const rad = Math.PI / 180;
    const cone = this.spec.cone_half_angle_deg * rad;
    const n = this.spec.rays_per_cone;

    this.spec.bearings_deg.forEach((bearingDeg, idx) => {
      const origin = this.robot.worldPoint([
        this.spec.nose_x,
        0.12 * Math.sin(bearingDeg * rad),
        this.spec.height,
      ]);
      let min = Infinity;
      for (let i = 0; i < n; i++) {
        const yawOff = bearingDeg * rad + (i / (n - 1) - 0.5) * 2 * cone;
        const pitchOff = this.rng.gaussian(0, cone / 3);
        const dir = this.robot.worldDir([
          Math.cos(yawOff) * Math.cos(pitchOff),
          Math.sin(yawOff) * Math.cos(pitchOff),
          Math.sin(pitchOff),
        ]);
        const hit = this.physics.castRay(origin, dir, this.spec.range_max, QUERY, this.robot.body);
        if (hit && hit.toi < min) min = hit.toi;
      }
      let range = Number.isFinite(min) ? min + this.rng.gaussian(0, this.spec.noise_std) : this.spec.range_max;
      range = Math.max(this.spec.range_min, Math.min(this.spec.range_max, range));
      this.lastRanges[idx] = range;

      this.ros.publish(TOPICS.sonar(idx), {
        header: { stamp: this.clock.stamp(), frame_id: `${this.spec.frame_prefix}${idx}` },
        radiation_type: 0,
        field_of_view: 2 * cone,
        min_range: this.spec.range_min,
        max_range: this.spec.range_max,
        range: Math.round(range * 1000) / 1000,
      });
    });
  }
}

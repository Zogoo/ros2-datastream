import { TOPICS } from '../ros/topics.js';
import { GROUP_WORLD, GROUP_ARM, groups } from '../physics/world.js';

const QUERY = groups(0xffff, GROUP_WORLD | GROUP_ARM);

/** Spinning 2D LIDAR: real raycasts batched across physics ticks so one full
 *  revolution takes 1/hz seconds, like a rotating head. Includes range noise,
 *  dropouts and steam degradation. Arm self-hits are real (kinematic arm colliders). */
export class LidarSensor {
  constructor(spec, physics, robot, world, rng, ros, clock) {
    this.spec = spec.sensors.lidar;
    this.physics = physics;
    this.robot = robot;
    this.world = world;
    this.rng = rng;
    this.ros = ros;
    this.clock = clock;

    this.n = this.spec.num_rays;
    this.ranges = new Float32Array(this.n);
    this.index = 0;
    this.raysPerTick = Math.ceil((this.n * this.spec.hz) / 60);
    this.lastScan = null;
  }

  update() {
    const origin = this.robot.worldPoint(this.spec.position);
    const yaw = this.robot.pose().yaw;
    const steam = this.world.steamDensityAt(origin.x, origin.y);
    const angleInc = (2 * Math.PI) / this.n;

    for (let k = 0; k < this.raysPerTick; k++) {
      const i = this.index;
      const beamAngle = -Math.PI + i * angleInc;
      const a = yaw + beamAngle;
      const dir = { x: Math.cos(a), y: Math.sin(a), z: 0 };
      const hit = this.physics.castRay(origin, dir, this.spec.range_max, QUERY, this.robot.body);

      let range = hit ? hit.toi : Infinity;
      if (Number.isFinite(range)) {
        range += this.rng.gaussian(0, range * this.spec.noise_frac_of_range);
        if (steam > 0) range += this.rng.gaussian(0, this.spec.steam_noise_std * steam);
        const dropout = this.spec.dropout_prob + steam * this.spec.steam_dropout_prob;
        if (this.rng.uniform() < dropout) range = Infinity;
        if (range < this.spec.range_min) range = this.spec.range_min;
      }
      this.ranges[i] = range;
      this.index += 1;
      if (this.index >= this.n) {
        this.index = 0;
        this._publish();
      }
    }
  }

  _publish() {
    const scanTime = 1 / this.spec.hz;
    // No-return beams report range_max (consumers discard >= range_max per REP-117 practice).
    const ranges = Array.from(this.ranges, (r) => (
      Number.isFinite(r) ? Math.min(Math.round(r * 1000) / 1000, this.spec.range_max) : this.spec.range_max
    ));
    this.lastScan = ranges;
    this.ros.publish(TOPICS.scan, {
      header: { stamp: this.clock.stamp(), frame_id: this.spec.frame_id },
      angle_min: -Math.PI,
      angle_max: Math.PI,
      angle_increment: (2 * Math.PI) / this.n,
      time_increment: scanTime / this.n,
      scan_time: scanTime,
      range_min: this.spec.range_min,
      range_max: this.spec.range_max,
      ranges,
      intensities: [],
    });
  }
}

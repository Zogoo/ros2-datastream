/** Fixed-timestep accumulator clock. Clamps long frame gaps (hidden tabs)
 *  so physics never explodes, and provides ROS-style stamps. */
export class SimClock {
  constructor(hz) {
    this.step = 1 / hz;
    this.accumulator = 0;
    this.simTime = 0;
    this.lastReal = null;
    this.maxFrameDelta = 0.25;
  }

  /** Returns the number of fixed steps to run this frame. */
  advance(nowMs) {
    if (this.lastReal === null) this.lastReal = nowMs;
    let delta = (nowMs - this.lastReal) / 1000;
    this.lastReal = nowMs;
    if (delta > this.maxFrameDelta) delta = this.maxFrameDelta;
    this.accumulator += delta;
    let steps = 0;
    while (this.accumulator >= this.step) {
      this.accumulator -= this.step;
      steps += 1;
    }
    this.simTime += steps * this.step;
    return steps;
  }

  /** ROS builtin_interfaces/Time stamp anchored to wall clock. */
  stamp() {
    const ms = Date.now();
    return { sec: Math.floor(ms / 1000), nanosec: (ms % 1000) * 1e6 };
  }
}

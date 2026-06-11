/** Deterministic seeded RNG (mulberry32) with gaussian sampling so noise
 *  models are reproducible run to run. */
export function createRng(seed) {
  let s = seed >>> 0;
  const next = () => {
    s |= 0; s = (s + 0x6d2b79f5) | 0;
    let t = Math.imul(s ^ (s >>> 15), 1 | s);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  return {
    uniform: next,
    gaussian(mean = 0, std = 1) {
      let u = 0, v = 0;
      while (u === 0) u = next();
      while (v === 0) v = next();
      return mean + std * Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
    },
  };
}

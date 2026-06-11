import { describe, it, expect } from 'vitest';
import { createRng } from '../src/core/rng.js';
import { SimClock } from '../src/core/clock.js';

describe('seeded rng', () => {
  it('is deterministic for a given seed', () => {
    const a = createRng(42);
    const b = createRng(42);
    for (let i = 0; i < 50; i++) expect(a.uniform()).toBe(b.uniform());
  });

  it('gaussian has roughly the requested moments', () => {
    const rng = createRng(7);
    const n = 20000;
    let sum = 0;
    let sumSq = 0;
    for (let i = 0; i < n; i++) {
      const v = rng.gaussian(2, 0.5);
      sum += v;
      sumSq += v * v;
    }
    const mean = sum / n;
    const std = Math.sqrt(sumSq / n - mean * mean);
    expect(mean).toBeCloseTo(2, 1);
    expect(std).toBeCloseTo(0.5, 1);
  });
});

describe('fixed timestep clock', () => {
  it('produces 60 steps per simulated second', () => {
    const clock = new SimClock(60);
    let steps = 0;
    for (let t = 0; t <= 1000; t += 16.67) steps += clock.advance(t);
    expect(steps).toBeGreaterThanOrEqual(59);
    expect(steps).toBeLessThanOrEqual(61);
  });

  it('clamps huge frame gaps (background tab)', () => {
    const clock = new SimClock(60);
    clock.advance(0);
    const steps = clock.advance(10000);
    expect(steps).toBeLessThanOrEqual(16);
  });
});

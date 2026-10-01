import { beforeAll, describe, expect, it } from 'vitest';
import * as THREE from 'three';
import spec from '../../shared/robot_spec.json';
import profiles from '../../shared/object_profiles.json';
import { PhysicsWorld } from '../src/physics/world.js';
import { ObjectManager } from '../src/env/objects.js';
import { Arm, graspEvents } from '../src/robot/arm.js';
import { armFk, gripperOpening } from '../src/robot/kinematics.js';
import { createRng } from '../src/core/rng.js';

// Real Rapier world + real ObjectManager/Arm; only the robot chassis and the
// onsen layout are stubbed (robot base fixed at the origin, yaw 0).
const DT = 1 / 60;
const OPEN = [90, 157, 57, 77, 90, 80];   // PICK_SCOOP: jaws open over the floor

beforeAll(async () => {
  await PhysicsWorld.create(60);   // RAPIER.init()
});

function makeSim({ pool = null, wet = false } = {}) {
  const physics = new PhysicsWorld(60);
  physics.addStaticBox({ center: [0, 0, -0.05], size: [20, 20, 0.1], meta: { kind: 'floor', id: 'f' } });
  const world = {
    poolAt: (x, y) => (pool && x > pool.rect[0] && x < pool.rect[2] && y > pool.rect[1] && y < pool.rect[3] ? pool : null),
    isWetAt: () => wet,
    binAt: () => null,
  };
  const layout = { dynamic_props: [], meta: { building: { min: [-5, -5], max: [5, 5] } } };
  const objects = new ObjectManager(layout, profiles, physics, new THREE.Scene(), world);
  const robot = {
    body: { applyImpulseAtPoint() {}, translation: () => ({ x: 0, y: 0, z: 0.16 }) },
    worldPoint: (p) => ({ x: p[0], y: p[1], z: p[2] }),
    pose: () => ({ x: 0, y: 0, yaw: 0 }),
  };
  const arm = new Arm(physics, robot, spec.arm, objects, createRng(7));
  arm.current = [...OPEN];
  arm.target = [...OPEN];
  const tip = () => {
    const f = armFk(arm.current, spec.arm).fingertip;
    return { x: f[0], y: f[1], z: f[2] };
  };
  const step = (n = 1) => {
    for (let i = 0; i < n; i++) {
      arm.update(DT);
      objects.update(DT);
      physics.step();
    }
  };
  return { physics, objects, arm, tip, step };
}

function settle(sim, frames = 60) {
  sim.step(frames);
  graspEvents.length = 0;
}

describe('parallel-jaw pinch on cloth', () => {
  it('closing over a folded towel stalls the jaws on the cloth and grasps it', () => {
    const sim = makeSim();
    const t = sim.tip();
    const towel = sim.objects.spawn('towel', [t.x + 0.05, t.y, 0.03], 'towel_a', { state: 'folded', wetness: 0.3 });
    settle(sim);
    sim.arm.target[5] = 12;
    sim.step(60);
    expect(sim.arm.holding()).toBe(true);
    expect(towel.held).toBe(true);
    // jaw servo stalled on the compressed bite, NOT at the commanded 12 deg
    const jaw = gripperOpening(sim.arm.current[5], spec.arm);
    expect(jaw).toBeCloseTo(profiles.classes.towel.cloth.bite_width_m.folded, 3);
    expect(sim.arm.gripperStatus().held).toBe(true);
    expect(sim.arm.gripperStatus()).not.toHaveProperty('object_id');
    expect(towel.state).toBe('crumpled');
  });

  it('jaws closing 12 cm beside a towel catch nothing (no magic snap radius)', () => {
    const sim = makeSim();
    const t = sim.tip();
    // towel footprint half-width 0.11 in y; tip is 0.12 + 0.11 = 0.23 from centre
    sim.objects.spawn('towel', [t.x, t.y + 0.23, 0.03], 'towel_b', { state: 'folded', wetness: 0.3 });
    settle(sim);
    sim.arm.target[5] = 12;
    sim.step(60);
    expect(sim.arm.holding()).toBe(false);
    expect(sim.arm.current[5]).toBeLessThan(15);   // closed on air
  });

  it('jaws that were already nearly shut cannot get around the cloth', () => {
    const sim = makeSim();
    sim.arm.current[5] = 20;   // 10 mm open < 12 mm bite + clearance
    sim.arm.target[5] = 20;
    const t = sim.tip();
    sim.objects.spawn('towel', [t.x, t.y, 0.03], 'towel_c', { state: 'folded', wetness: 0.3 });
    settle(sim);
    sim.arm.target[5] = 12;
    sim.step(40);
    expect(sim.arm.holding()).toBe(false);
  });

  it('a lifted towel hangs below the jaws and follows the gripper', () => {
    const sim = makeSim();
    const t = sim.tip();
    const towel = sim.objects.spawn('towel', [t.x, t.y, 0.03], 'towel_d', { state: 'crumpled', wetness: 0.2 });
    settle(sim);
    sim.arm.target[5] = 12;
    sim.step(40);
    expect(sim.arm.holding()).toBe(true);
    sim.arm.target = [90, 110, 57, 77, 90, 12];   // PICK_LIFT
    sim.step(120);
    expect(sim.arm.holding()).toBe(true);
    const tip = sim.tip();
    const p = towel.body.translation();
    expect(Math.hypot(p.x - tip.x, p.y - tip.y)).toBeLessThan(0.02);
    expect(tip.z - p.z).toBeCloseTo(profiles.classes.towel.cloth.hang_m, 2);
    expect(sim.arm.effort[3]).toBeGreaterThan(towel.massKg * 9.81 * 0.8);   // wrist load cell sees it
  });

  it('opening the jaws releases the towel and it falls', () => {
    const sim = makeSim();
    const t = sim.tip();
    const towel = sim.objects.spawn('towel', [t.x, t.y, 0.03], 'towel_e', { state: 'crumpled', wetness: 0.2 });
    settle(sim);
    sim.arm.target[5] = 12;
    sim.step(40);
    sim.arm.target = [90, 110, 57, 77, 90, 12];
    sim.step(90);
    sim.arm.target[5] = 80;
    sim.step(5);
    expect(sim.arm.holding()).toBe(false);
    expect(graspEvents.some((e) => e.event === 'GRASP_RELEASED')).toBe(true);
    sim.step(90);
    expect(towel.body.translation().z).toBeLessThan(0.06);
  });

  it('a soaked towel pinched only by its hem pulls out of the jaws on lift', () => {
    const sim = makeSim();
    const t = sim.tip();
    // tip 1 cm past the towel's long edge: only the hem is between the pads
    sim.objects.spawn('towel', [t.x, t.y - 0.12, 0.03], 'towel_f', { state: 'folded', wetness: 1.0 });
    settle(sim);
    sim.arm.target[5] = 12;
    sim.step(40);
    expect(sim.arm.holding()).toBe(true);
    sim.arm.target = [90, 110, 57, 77, 90, 12];
    sim.step(120);
    expect(sim.arm.holding()).toBe(false);
    expect(graspEvents.some((e) => e.event === 'GRASP_SLIPPED')).toBe(true);
  });
});

describe('cloth physics', () => {
  it('damp towels weigh more than dry ones (mass = dry * (1 + capacity * wetness))', () => {
    const sim = makeSim();
    const dry = sim.objects.spawn('towel', [0, 0, 0.05], 'd', { wetness: 0 });
    const damp = sim.objects.spawn('towel', [1, 0, 0.05], 'w', { wetness: 0.5 });
    expect(dry.massKg).toBeCloseTo(0.12, 3);
    expect(damp.massKg).toBeCloseTo(0.12 * (1 + 2.5 * 0.5), 3);
  });

  it('a towel in a pool soaks up water, gets heavier, and sinks', () => {
    const pool = { rect: [-1, -1, 1, 1], water_z: 0.3 };
    const sim = makeSim({ pool });
    const towel = sim.objects.spawn('towel', [0, 0, 0.35], 'p', { state: 'crumpled', wetness: 0.1 });
    sim.step(60);
    const floatingZ = towel.body.translation().z;
    expect(floatingZ).toBeGreaterThan(0.2);          // trapped air floats it at first
    sim.step(60 * 40);
    expect(towel.wetness).toBeGreaterThan(0.95);
    expect(towel.massKg).toBeGreaterThan(0.4);
    expect(towel.body.translation().z).toBeLessThan(0.06);   // settled on the pool floor
  });

  it('a towel lying on a wet floor slowly gets damp', () => {
    const sim = makeSim({ wet: true });
    const towel = sim.objects.spawn('towel', [0, 0, 0.03], 'wf', { state: 'folded', wetness: 0 });
    sim.step(60 * 60);
    expect(towel.wetness).toBeGreaterThan(0.1);
    expect(towel.wetness).toBeLessThanOrEqual(profiles.classes.towel.cloth.wet_floor_wetness);
  });
});

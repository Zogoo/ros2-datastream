import { describe, it, expect } from 'vitest';
import spec from '../../shared/robot_spec.json';
import layout from '../../shared/onsen_layout.json';
import profiles from '../../shared/object_profiles.json';

describe('robot_spec.json invariants', () => {
  it('suspension geometry is self-consistent', () => {
    const s = spec.wheels.suspension;
    expect(s.travel).toBeLessThanOrEqual(s.rest_length);
    // body z at rest = attach_z subtracted from (rest_length + wheel radius)
    expect(s.rest_length + spec.wheels.radius - s.attach_z).toBeCloseTo(0.13, 3);
  });

  it('static spring deflection sits inside the travel band', () => {
    const k = spec.wheels.suspension.stiffness;
    const weightPerWheel = (spec.chassis.mass * 9.81) / 6;
    const deflection = weightPerWheel / k;
    expect(deflection).toBeGreaterThan(0.2 * spec.wheels.suspension.travel);
    expect(deflection).toBeLessThan(0.8 * spec.wheels.suspension.travel);
  });

  it('lidar sits above the stow guard and below doors', () => {
    const lidarZ = spec.sensors.lidar.position[2];
    expect(lidarZ).toBeGreaterThan(spec.decks.deck2_z);
    expect(lidarZ).toBeLessThan(1.0);
  });

  it('robot footprint fits through every doorway with margin', () => {
    const widthWithBasket = spec.basket.center[1] + spec.basket.size[1] / 2
      + spec.chassis.size[1] / 2;
    for (const door of layout.doors) {
      expect(door.width).toBeGreaterThan(widthWithBasket + 0.2);
    }
  });

  it('wheels can climb the platform steps but not the bath rims', () => {
    const travel = spec.wheels.suspension.travel;
    for (const platform of layout.platforms) {
      expect(platform.h).toBeLessThanOrEqual(travel + 0.04);
    }
    const rimWalls = layout.walls.filter((w) => w.id.includes('_rim_'));
    expect(rimWalls.length).toBeGreaterThan(0);
    for (const rim of rimWalls) {
      expect(rim.h).toBeGreaterThan(spec.chassis.ground_clearance + 0.1);
    }
  });
});

describe('object_profiles.json invariants', () => {
  it('every dynamic prop class in the layout has a physics profile', () => {
    for (const prop of layout.dynamic_props) {
      expect(profiles.classes[prop.type], `missing profile: ${prop.type}`).toBeDefined();
    }
  });

  it('pickable objects fit the gripper opening', () => {
    for (const [name, p] of Object.entries(profiles.classes)) {
      if (!p.pickable) continue;
      const minDim = p.shape === 'box' ? Math.min(...p.size) : Math.min(p.radius * 2, p.height);
      expect(minDim, `${name} too large to grip`).toBeLessThanOrEqual(spec.arm.gripper.max_opening_m);
    }
  });
});

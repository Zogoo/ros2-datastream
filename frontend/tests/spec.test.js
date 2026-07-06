import { describe, it, expect } from 'vitest';
import spec from '../../shared/robot_spec.json';
import layout from '../../shared/onsen_layout.json';
import profiles from '../../shared/object_profiles.json';

describe('robot_spec.json invariants', () => {
  it('suspension geometry is self-consistent', () => {
    const { suspension: s, driven, casters } = spec.wheels;
    expect(s.travel).toBeLessThanOrEqual(s.rest_length);
    // Body z at rest = rest_length + wheel radius - attach_z. The driven (Ø200)
    // and caster (Ø100) groups use different attach_z so the body still sits
    // level at the same BASE_Z on mixed-diameter wheels.
    expect(s.rest_length + driven.radius - driven.attach_z).toBeCloseTo(0.16, 3);
    expect(s.rest_length + casters.radius - casters.attach_z).toBeCloseTo(0.16, 3);
  });

  it('static spring deflection sits inside the travel band', () => {
    const { suspension: s } = spec.wheels;
    // Centre-drive: the two driven wheels sit on the CoG axle and carry
    // essentially the whole weight; the corner casters only resist pitch/roll.
    const drivenLoad = (spec.chassis.mass * 9.81) / 2;
    const deflection = drivenLoad / s.stiffness;
    expect(deflection).toBeGreaterThan(0.2 * s.travel);
    expect(deflection).toBeLessThan(0.8 * s.travel);
  });

  it('lidar sits above the stow guard and below doors', () => {
    const lidarZ = spec.sensors.lidar.position[2];
    expect(lidarZ).toBeGreaterThan(spec.decks.deck2_z);
    expect(lidarZ).toBeLessThan(1.0);
  });

  it('robot footprint fits through every doorway with margin', () => {
    // Bin sits fully inside the footprint now, so width is the chassis plus the
    // bumper ring standoff.
    const halfWidth = spec.chassis.size[1] / 2 + 0.012;
    for (const door of layout.doors) {
      expect(door.width).toBeGreaterThan(2 * halfWidth + 0.1);
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

import RAPIER from '@dimforge/rapier3d-compat';

export const GROUP_WORLD = 0x0001;   // static structure: walls, partitions, bins, floor
export const GROUP_ROBOT = 0x0002;   // robot chassis / contact skirt
export const GROUP_ARM = 0x0004;     // kinematic arm links + gripper
export const GROUP_OBJECT = 0x0008;  // movable props: towels, stools, buckets, bottles

// Movable props collide with the world, the robot, the arm and each other —
// but are a distinct membership from the static world so a *held* prop can be
// excluded from wall collisions without also disabling floor/robot collisions
// for the free props. (Bug fix: walls and objects were both GROUP_WORLD, so a
// carried towel was ripped off the gripper by walls.)
export const OBJECT_FILTER = GROUP_WORLD | GROUP_ROBOT | GROUP_ARM | GROUP_OBJECT;

export const groups = (memberships, filter) => ((memberships << 16) | filter) >>> 0;

/** Z-up physics world (matches the ROS frame of onsen_layout.json). */
export class PhysicsWorld {
  static async create(hz) {
    await RAPIER.init();
    return new PhysicsWorld(hz);
  }

  constructor(hz) {
    this.R = RAPIER;
    this.world = new RAPIER.World({ x: 0, y: 0, z: -9.81 });
    this.world.timestep = 1 / hz;
    this.eventQueue = new RAPIER.EventQueue(true);
    this.colliderMeta = new Map();
  }

  registerMeta(collider, meta) {
    this.colliderMeta.set(collider.handle, meta);
  }

  metaOf(handle) {
    return this.colliderMeta.get(handle);
  }

  addStaticBox({ center, size, friction = 0.8, restitution = 0.05, meta = null, sensor = false }) {
    const body = this.world.createRigidBody(
      this.R.RigidBodyDesc.fixed().setTranslation(center[0], center[1], center[2]),
    );
    const desc = this.R.ColliderDesc.cuboid(size[0] / 2, size[1] / 2, size[2] / 2)
      .setFriction(friction)
      .setRestitution(restitution)
      .setCollisionGroups(groups(GROUP_WORLD, 0xffff))
      .setSensor(sensor);
    const collider = this.world.createCollider(desc, body);
    if (meta) this.registerMeta(collider, meta);
    return { body, collider };
  }

  addDynamicBody({ position, shape, mass, friction, restitution, meta = null }) {
    const body = this.world.createRigidBody(
      this.R.RigidBodyDesc.dynamic()
        .setTranslation(position[0], position[1], position[2])
        .setLinearDamping(0.2)
        .setAngularDamping(0.5),
    );
    let desc;
    if (shape.type === 'box') {
      desc = this.R.ColliderDesc.cuboid(shape.size[0] / 2, shape.size[1] / 2, shape.size[2] / 2);
    } else {
      desc = this.R.ColliderDesc.cylinder(shape.height / 2, shape.radius);
      // Rapier cylinders are Y-up; rotate so the axis is world Z.
      desc.setRotation({ w: Math.SQRT1_2, x: Math.SQRT1_2, y: 0, z: 0 });
    }
    desc
      .setMass(mass)
      .setFriction(friction)
      .setRestitution(restitution)
      .setCollisionGroups(groups(GROUP_OBJECT, OBJECT_FILTER));
    const collider = this.world.createCollider(desc, body);
    if (meta) this.registerMeta(collider, meta);
    return { body, collider };
  }

  /** Raycast helper. filter is an interaction-groups value for the query. */
  castRay(origin, dir, maxToi, filter, excludeBody = null) {
    const ray = new this.R.Ray(origin, dir);
    const hit = this.world.castRay(ray, maxToi, true, undefined, filter, undefined, excludeBody);
    return hit ? { toi: hit.timeOfImpact ?? hit.toi, collider: hit.collider } : null;
  }

  step() {
    this.world.step(this.eventQueue);
  }
}

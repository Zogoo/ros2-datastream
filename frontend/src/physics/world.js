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

  addDynamicBody({
    position, shape, mass, friction, restitution, meta = null,
    linearDamping = 0.2, angularDamping = 0.5,
  }) {
    const body = this.world.createRigidBody(
      this.R.RigidBodyDesc.dynamic()
        .setTranslation(position[0], position[1], position[2])
        .setLinearDamping(linearDamping)
        .setAngularDamping(angularDamping),
    );
    const collider = this.world.createCollider(
      this.dynamicColliderDesc(shape, { mass, friction, restitution }), body,
    );
    if (meta) this.registerMeta(collider, meta);
    return { body, collider };
  }

  /** Collider for a movable prop. shape.type: 'box' (size), 'roundBox'
   *  (size + border — a soft-edged heap that neither rolls nor snags on its
   *  corners, used for crumpled cloth) or 'cylinder' (radius, height). */
  dynamicColliderDesc(shape, { mass, friction, restitution }, collisionGroups = null) {
    let desc;
    if (shape.type === 'box') {
      desc = this.R.ColliderDesc.cuboid(shape.size[0] / 2, shape.size[1] / 2, shape.size[2] / 2);
    } else if (shape.type === 'roundBox') {
      const r = shape.border;
      desc = this.R.ColliderDesc.roundCuboid(
        shape.size[0] / 2 - r, shape.size[1] / 2 - r, shape.size[2] / 2 - r, r,
      );
    } else {
      desc = this.R.ColliderDesc.cylinder(shape.height / 2, shape.radius);
      // Rapier cylinders are Y-up; rotate so the axis is world Z.
      desc.setRotation({ w: Math.SQRT1_2, x: Math.SQRT1_2, y: 0, z: 0 });
    }
    return desc
      .setMass(mass)
      .setFriction(friction)
      .setRestitution(restitution)
      .setCollisionGroups(collisionGroups ?? groups(GROUP_OBJECT, OBJECT_FILTER))
      // Explicit solver membership (default is "all groups") so a collider
      // can opt out of pushing props via its solver filter — the gripper
      // volume does (see Arm._buildKinematicColliders).
      .setSolverGroups(groups(GROUP_OBJECT, 0xffff));
  }

  /** Swap a body's collider for a new shape (cloth crumpling). Rapier cannot
   *  morph a cuboid into a round cuboid in place, so the collider is rebuilt
   *  on the same body, keeping its metadata registration. */
  replaceCollider(body, oldCollider, desc) {
    const meta = this.colliderMeta.get(oldCollider.handle);
    this.colliderMeta.delete(oldCollider.handle);
    this.world.removeCollider(oldCollider, true);
    const collider = this.world.createCollider(desc, body);
    if (meta) this.registerMeta(collider, meta);
    return collider;
  }

  /** Raycast helper. filter is an interaction-groups value for the query. */
  castRay(origin, dir, maxToi, filter, excludeBody = null) {
    const ray = new this.R.Ray(origin, dir);
    const hit = this.world.castRay(ray, maxToi, true, undefined, filter, undefined, excludeBody);
    return hit ? { toi: hit.timeOfImpact ?? hit.toi, collider: hit.collider } : null;
  }

  /** Raycast returning the surface normal too (for specular sensors). */
  castRayNormal(origin, dir, maxToi, filter, excludeBody = null) {
    const ray = new this.R.Ray(origin, dir);
    const hit = this.world.castRayAndGetNormal(ray, maxToi, true, undefined, filter, undefined, excludeBody);
    return hit ? { toi: hit.timeOfImpact ?? hit.toi, normal: hit.normal, collider: hit.collider } : null;
  }

  step() {
    this.world.step(this.eventQueue);
  }
}

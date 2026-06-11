import * as THREE from 'three';

/** Movable objects: spawn from layout, physics profiles from object_profiles.json,
 *  per-class skinnable materials, bin scoring, water buoyancy and reset. */
export class ObjectManager {
  constructor(layout, profiles, physics, scene, world) {
    this.layout = layout;
    this.profiles = profiles.classes;
    this.physics = physics;
    this.scene = scene;
    this.world = world;
    this.items = [];
    this.classMaterials = new Map();
    this.binnedEvents = [];
    this._nextId = 1;
    for (const prop of layout.dynamic_props) {
      this.spawn(prop.type, [prop.pos[0], prop.pos[1], this._spawnZ(prop.type)], prop.id);
    }
  }

  _spawnZ(cls) {
    const p = this.profiles[cls];
    return (p.shape === 'box' ? p.size[2] : p.height) / 2 + 0.02;
  }

  materialFor(cls) {
    if (!this.classMaterials.has(cls)) {
      this.classMaterials.set(cls, new THREE.MeshStandardMaterial({
        color: new THREE.Color(this.profiles[cls].color),
        roughness: 0.85,
      }));
    }
    return this.classMaterials.get(cls);
  }

  spawn(cls, position, id = null) {
    const p = this.profiles[cls];
    const shape = p.shape === 'box'
      ? { type: 'box', size: p.size }
      : { type: 'cylinder', radius: p.radius, height: p.height };
    const objId = id ?? this._uniqueId(cls);
    const { body, collider } = this.physics.addDynamicBody({
      position,
      shape,
      mass: p.mass,
      friction: p.friction,
      restitution: p.restitution,
      meta: { kind: 'object', id: objId, cls },
    });

    let geom;
    if (p.shape === 'box') geom = new THREE.BoxGeometry(p.size[0], p.size[1], p.size[2]);
    else {
      geom = new THREE.CylinderGeometry(p.radius, p.radius, p.height, 16);
      geom.rotateX(Math.PI / 2);
    }
    const mesh = new THREE.Mesh(geom, this.materialFor(cls));
    mesh.castShadow = true;
    this.scene.add(mesh);

    const item = {
      id: objId, cls, body, collider, mesh,
      pickable: p.pickable, buoyant: p.buoyant, binTarget: p.bin,
      held: false, binned: null,
      spawn: [...position],
    };
    this.items.push(item);
    return item;
  }

  /** Generated ids must never collide with layout prop ids — duplicates would
   *  corrupt /ground_truth/objects and every id-based lookup. */
  _uniqueId(cls) {
    let id;
    do {
      id = `${cls}_${this._nextId++}`;
    } while (this.items.some((i) => i.id === id));
    return id;
  }

  throwTowel(rng) {
    const [bx0, by0, bx1, by1] = this.layout.meta.building.min.concat(this.layout.meta.building.max);
    const x = bx0 + 1 + rng.uniform() * (bx1 - bx0 - 2);
    const y = by0 + 1 + rng.uniform() * (by1 - by0 - 2);
    const item = this.spawn('towel', [x, y, 1.6]);
    item.body.setLinvel({
      x: rng.gaussian(0, 0.8), y: rng.gaussian(0, 0.8), z: 0.5,
    }, true);
    item.body.setAngvel({ x: rng.gaussian(0, 2), y: rng.gaussian(0, 2), z: rng.gaussian(0, 2) }, true);
    return item;
  }

  update(dt) {
    for (const item of this.items) {
      const pos = item.body.translation();
      const rot = item.body.rotation();
      item.mesh.position.set(pos.x, pos.y, pos.z);
      item.mesh.quaternion.set(rot.x, rot.y, rot.z, rot.w);

      if (!item.held) {
        this._applyBuoyancy(item, pos, dt);
        this._checkBinned(item, pos);
      }
    }
  }

  _applyBuoyancy(item, pos, _dt) {
    if (!item.buoyant) return;
    const pool = this.world.poolAt(pos.x, pos.y);
    if (!pool || pos.z > pool.water_z + 0.03) return;
    const depth = Math.min(1, (pool.water_z + 0.03 - pos.z) / 0.12);
    const mass = item.body.mass();
    item.body.applyImpulse({ x: 0, y: 0, z: mass * 9.81 * 1.35 * depth * (1 / 60) }, true);
    const v = item.body.linvel();
    item.body.applyImpulse({ x: -v.x * mass * 0.05, y: -v.y * mass * 0.05, z: -v.z * mass * 0.08 }, true);
  }

  _checkBinned(item, pos) {
    if (item.binned) return;
    const bin = this.world.binAt(pos.x, pos.y, pos.z);
    if (!bin) return;
    item.binned = bin.id;
    this.binnedEvents.push({
      event: 'OBJECT_BINNED',
      object_id: item.id,
      object_class: item.cls,
      bin_id: bin.id,
      correct: item.binTarget === bin.type,
    });
  }

  drainBinnedEvents() {
    const events = this.binnedEvents;
    this.binnedEvents = [];
    return events;
  }

  nearestPickable(point, maxDist) {
    let best = null;
    let bestDist = maxDist;
    for (const item of this.items) {
      if (!item.pickable || item.held || item.binned) continue;
      const p = item.body.translation();
      const d = Math.hypot(p.x - point[0], p.y - point[1], p.z - point[2]);
      if (d < bestDist) { best = item; bestDist = d; }
    }
    return best;
  }

  groundTruth() {
    return this.items.map((item) => {
      const p = item.body.translation();
      return {
        id: item.id,
        class: item.cls,
        position: { x: round3(p.x), y: round3(p.y), z: round3(p.z) },
        held: item.held,
        binned: item.binned,
        pickable: item.pickable,
      };
    });
  }

  reset() {
    const fromLayout = new Set(this.layout.dynamic_props.map((p) => p.id));
    this.items = this.items.filter((item) => {
      if (!fromLayout.has(item.id)) {
        this.scene.remove(item.mesh);
        this.physics.world.removeRigidBody(item.body);
        return false;
      }
      item.held = false;
      item.binned = null;
      item.body.setBodyType(this.physics.R.RigidBodyType.Dynamic, true);
      item.body.setTranslation({ x: item.spawn[0], y: item.spawn[1], z: item.spawn[2] }, true);
      item.body.setRotation({ w: 1, x: 0, y: 0, z: 0 }, true);
      item.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      item.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
      return true;
    });
  }
}

const round3 = (v) => Math.round(v * 1000) / 1000;

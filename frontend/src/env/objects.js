import * as THREE from 'three';
import { GROUP_OBJECT, OBJECT_FILTER, groups } from '../physics/world.js';

const GRAVITY = 9.81;
const FREE_GROUPS = groups(GROUP_OBJECT, OBJECT_FILTER);

/** Movable objects: spawn from layout, physics profiles from object_profiles.json,
 *  per-class skinnable materials, bin scoring, water buoyancy and reset.
 *
 *  Towels are modelled as CLOTH, not bricks (see object_profiles.json `cloth`):
 *  - two physical states: `folded` (a 34x22 cm, 16 mm slab — how a towel is
 *    dropped folded) and `crumpled` (a rounded 22x17x7 cm heap — how a used or
 *    handled towel lies). Handling a towel crumples it for good; reset() refolds.
 *  - wetness 0..1: used onsen towels are damp, soak up pool water within
 *    seconds and slowly wick moisture from wet floors. Mass follows
 *    dry * (1 + capacity * wetness), so the load cell, the wrist load cell and
 *    the grip-slip limit all see the real (variable) payload. A soaked towel
 *    loses its trapped air and sinks; wet cotton renders darker.
 *  - high air drag / angular damping: cloth falls flat instead of tumbling
 *    and bouncing like a rigid slab. */
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
      const opts = this._layoutClothOpts(prop);
      this.spawn(prop.type, [prop.pos[0], prop.pos[1], this._spawnZ(prop.type, opts.state)], prop.id, opts);
    }
  }

  /** Deterministic per-id cloth state for layout towels: an explicit
   *  `state`/`wetness` in the layout wins, otherwise a stable hash of the id
   *  picks folded vs crumpled and the initial dampness (same world every run). */
  _layoutClothOpts(prop) {
    const cloth = this.profiles[prop.type]?.cloth;
    if (!cloth) return {};
    const h = hash01(prop.id);
    const [w0, w1] = cloth.initial_wetness;
    return {
      state: prop.state ?? (h < 0.5 ? 'crumpled' : 'folded'),
      wetness: prop.wetness ?? w0 + hash01(`${prop.id}#w`) * (w1 - w0),
    };
  }

  _spawnZ(cls, state = 'folded') {
    const p = this.profiles[cls];
    if (p.cloth) return this._clothSize(p, state)[2] / 2 + 0.02;
    return (p.shape === 'box' ? p.size[2] : p.height) / 2 + 0.02;
  }

  _clothSize(profile, state) {
    return state === 'crumpled' ? profile.cloth.crumpled_size : profile.size;
  }

  _clothShape(profile, state) {
    const size = this._clothSize(profile, state);
    return state === 'crumpled'
      ? { type: 'roundBox', size, border: profile.cloth.crumpled_border_m }
      : { type: 'box', size };
  }

  materialFor(cls) {
    if (!this.classMaterials.has(cls)) {
      this.classMaterials.set(cls, new THREE.MeshStandardMaterial({
        color: new THREE.Color(this.profiles[cls].color),
        roughness: 0.85,
        // Cloth geometry carries per-vertex colors so wetness can darken ONE
        // towel without cloning the class material (which would detach it
        // from class-level skins).
        vertexColors: !!this.profiles[cls].cloth,
      }));
    }
    return this.classMaterials.get(cls);
  }

  /** opts (cloth only): { state: 'folded'|'crumpled', wetness: 0..1 }. */
  spawn(cls, position, id = null, opts = {}) {
    const p = this.profiles[cls];
    const objId = id ?? this._uniqueId(cls);
    const cloth = p.cloth ?? null;
    const state = cloth ? (opts.state ?? 'folded') : null;
    const wetness = cloth
      ? clamp01(opts.wetness ?? cloth.initial_wetness[0] + hash01(`${objId}#w`) * (cloth.initial_wetness[1] - cloth.initial_wetness[0]))
      : 0;
    const massKg = cloth ? p.mass * (1 + cloth.water_capacity * wetness) : p.mass;

    let shape;
    if (cloth) shape = this._clothShape(p, state);
    else if (p.shape === 'box') shape = { type: 'box', size: p.size };
    else shape = { type: 'cylinder', radius: p.radius, height: p.height };

    const { body, collider } = this.physics.addDynamicBody({
      position,
      shape,
      mass: massKg,
      friction: p.friction,
      restitution: p.restitution,
      meta: { kind: 'object', id: objId, cls },
      ...(cloth ? { linearDamping: cloth.linear_damping, angularDamping: cloth.angular_damping } : {}),
    });

    let geom;
    if (cloth) geom = clothGeometry(this._clothSize(p, state), state, objId);
    else if (p.shape === 'box') geom = new THREE.BoxGeometry(p.size[0], p.size[1], p.size[2]);
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
      cloth, state, wetness, massKg,
      spawnState: state, spawnWetness: wetness,
      _shade: -1,
    };
    if (cloth) this._applyWetShade(item);
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
    const [w0, w1] = this.profiles.towel.cloth.initial_wetness;
    // A tossed towel lands as a heap, not neatly folded.
    const item = this.spawn('towel', [x, y, 1.6], null, {
      state: 'crumpled', wetness: w0 + rng.uniform() * (w1 - w0),
    });
    item.body.setLinvel({
      x: rng.gaussian(0, 0.8), y: rng.gaussian(0, 0.8), z: 0.5,
    }, true);
    item.body.setAngvel({ x: rng.gaussian(0, 2), y: rng.gaussian(0, 2), z: rng.gaussian(0, 2) }, true);
    return item;
  }

  /** Change a cloth item's physical state: rebuilds collider + mesh geometry.
   *  collisionGroups lets a HELD towel keep its carry filter. */
  setClothState(item, state, collisionGroups = null) {
    if (!item.cloth || item.state === state) return;
    const p = this.profiles[item.cls];
    const desc = this.physics.dynamicColliderDesc(
      this._clothShape(p, state),
      { mass: item.massKg, friction: p.friction, restitution: p.restitution },
      collisionGroups ?? item.collider.collisionGroups(),
    );
    item.collider = this.physics.replaceCollider(item.body, item.collider, desc);
    item.mesh.geometry.dispose();
    item.mesh.geometry = clothGeometry(this._clothSize(p, state), state, item.id);
    item.state = state;
    item._shade = -1;
    this._applyWetShade(item);
  }

  /** Half-extents of the item's current collider (body frame). */
  halfExtents(item) {
    const p = this.profiles[item.cls];
    if (item.cloth) {
      const s = this._clothSize(p, item.state);
      return { x: s[0] / 2, y: s[1] / 2, z: s[2] / 2 };
    }
    if (p.shape === 'box') return { x: p.size[0] / 2, y: p.size[1] / 2, z: p.size[2] / 2 };
    return { x: p.radius, y: p.radius, z: p.height / 2 };
  }

  update(dt) {
    for (const item of this.items) {
      const pos = item.body.translation();
      const rot = item.body.rotation();
      item.mesh.position.set(pos.x, pos.y, pos.z);
      item.mesh.quaternion.set(rot.x, rot.y, rot.z, rot.w);

      if (!item.held) {
        const pool = item.buoyant ? this.world.poolAt(pos.x, pos.y) : null;
        const submerged = pool ? this._submergedFraction(item, pos, pool) : 0;
        if (item.cloth) this._updateWetness(item, pos, submerged, dt);
        if (submerged > 0) this._applyBuoyancy(item, submerged, dt);
        this._checkBinned(item, pos);
      }
    }
  }

  _submergedFraction(item, pos, pool) {
    const h = 2 * this.halfExtents(item).z;
    return clamp01((pool.water_z - (pos.z - h / 2)) / h);
  }

  /** Terry cotton soaks fast when submerged, wicks slowly from a wet floor,
   *  and (on the timescale of a cleaning run) does not dry. */
  _updateWetness(item, pos, submerged, dt) {
    const c = item.cloth;
    let w = item.wetness;
    if (submerged > 0.05) {
      w += (1 - w) * (1 - Math.exp(-dt / c.soak_tau_s)) * submerged;
    } else if (w < c.wet_floor_wetness
      && pos.z < this.halfExtents(item).z + 0.04
      && this.world.isWetAt(pos.x, pos.y)) {
      w += (c.wet_floor_wetness - w) * (1 - Math.exp(-dt / c.wet_floor_tau_s));
    }
    if (w === item.wetness) return;
    item.wetness = clamp01(w);
    const massKg = this.profiles[item.cls].mass * (1 + c.water_capacity * item.wetness);
    if (Math.abs(massKg - item.massKg) > 0.005) {
      item.massKg = massKg;
      item.collider.setMass(massKg);
    }
    this._applyWetShade(item);
  }

  _applyWetShade(item) {
    const shade = 1 - item.cloth.wet_darkening * item.wetness;
    if (Math.abs(shade - item._shade) < 0.01) return;
    item._shade = shade;
    const col = item.mesh.geometry.getAttribute('color');
    if (!col) return;
    col.array.fill(shade);
    col.needsUpdate = true;
  }

  /** Buoyancy + water drag as forces integrated over dt (frame-rate
   *  independent). Cloth: trapped air floats a dry towel (B 1.5) but a soaked
   *  one is denser than water (B 0.75) and settles to the pool floor. Rigid
   *  props keep the fixed 1.35 buoyancy of a sealed hollow object. */
  _applyBuoyancy(item, submerged, dt) {
    const mass = item.cloth ? item.massKg : item.body.mass();
    let lift;
    let drag;
    if (item.cloth) {
      const c = item.cloth;
      lift = c.buoyancy_dry + (c.buoyancy_soaked - c.buoyancy_dry) * item.wetness;
      drag = c.water_drag;
    } else {
      lift = 1.35;
      drag = 3.0;
    }
    const v = item.body.linvel();
    const k = mass * submerged * dt;
    item.body.applyImpulse({
      x: -v.x * drag * k,
      y: -v.y * drag * k,
      z: GRAVITY * lift * k - v.z * drag * 1.6 * k,
    }, true);
  }

  /** Scoring: an object counts as binned only once it is genuinely INSIDE a
   *  bin (interior footprint, below the rim) — not merely dropped near one. */
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

  /**
   * What a closing parallel-jaw gripper would actually catch. The finger pads
   * span [tip.z - jawReach, tip.z]; a pickable item is caught when the pads
   * straddle part of its footprint (a hem may be caught just past its edge)
   * AND reach down into it by at least minBite. Returns the best candidate
   * with a bite quality 0..1 (how much cloth the jaws hold — a deep bite into
   * a heap holds hard, a skim over a folded slab barely holds) and the width
   * at which the jaws stall on the compressed cloth. null = the jaws close on
   * air. No magic snap radius: geometry decides.
   */
  pinchCandidate(tip, gripper) {
    const jawBottom = tip.z - gripper.jaw_reach_m;
    let best = null;
    for (const item of this.items) {
      if (!item.pickable || item.held || item.binned) continue;
      const pos = item.body.translation();
      const q = item.body.rotation();
      const he = this.halfExtents(item);
      const local = rotateByConj(q, { x: tip.x - pos.x, y: tip.y - pos.y, z: tip.z - pos.z });
      const inside = Math.min(he.x - Math.abs(local.x), he.y - Math.abs(local.y));
      if (inside < -gripper.jaw_edge_catch_m) continue;

      const halfH = worldHalfHeight(q, he);
      const top = pos.z + halfH;
      const bottom = pos.z - halfH;
      if (tip.z < bottom) continue;  // gripper under the object: nothing between the pads
      const bite = top - Math.max(jawBottom, bottom);
      if (bite < gripper.min_bite_m) continue;

      const fullBite = item.cloth ? item.cloth.full_bite_depth_m[item.state] : 2 * he.z;
      const depthQ = Math.min(1, bite / fullBite);
      const edgeQ = inside >= 0 ? 1 : 0.55;  // pinching only the hem
      const quality = depthQ * edgeQ;
      const biteWidth = item.cloth ? item.cloth.bite_width_m[item.state] : 2 * Math.min(he.x, he.y);
      if (!best || quality > best.quality) best = { item, quality, biteWidth };
    }
    return best;
  }

  /** inBasket: optional predicate (worldPos -> bool) marking objects riding
   *  in the robot's own collect bin, so the mission doesn't re-target them. */
  groundTruth(inBasket = null) {
    return this.items.map((item) => {
      const p = item.body.translation();
      return {
        id: item.id,
        class: item.cls,
        position: { x: round3(p.x), y: round3(p.y), z: round3(p.z) },
        held: item.held,
        binned: item.binned,
        pickable: item.pickable,
        in_basket: inBasket ? inBasket(p) : false,
        ...(item.cloth ? {
          state: item.state, wetness: round3(item.wetness), mass_kg: round3(item.massKg),
        } : {}),
      };
    });
  }

  reset() {
    const fromLayout = new Set(this.layout.dynamic_props.map((p) => p.id));
    this.items = this.items.filter((item) => {
      if (!fromLayout.has(item.id)) {
        this.scene.remove(item.mesh);
        item.mesh.geometry.dispose();
        this.physics.world.removeRigidBody(item.body);
        return false;
      }
      item.held = false;
      item.binned = null;
      item.body.setBodyType(this.physics.R.RigidBodyType.Dynamic, true);
      if (item.cloth) {
        // Refold / re-dampen to the layout's initial condition.
        this.setClothState(item, item.spawnState, FREE_GROUPS);
        item.collider.setCollisionGroups(FREE_GROUPS);
        item.wetness = item.spawnWetness;
        item.massKg = this.profiles[item.cls].mass * (1 + item.cloth.water_capacity * item.wetness);
        item.collider.setMass(item.massKg);
        this._applyWetShade(item);
      }
      item.body.setTranslation({ x: item.spawn[0], y: item.spawn[1], z: item.spawn[2] }, true);
      item.body.setRotation({ w: 1, x: 0, y: 0, z: 0 }, true);
      item.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      item.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
      return true;
    });
  }
}

// ── Cloth geometry ──────────────────────────────────────────────────────────

/** Visual mesh for a towel in body frame, matching the collider extents.
 *  folded: a slab with a soft quilted top and rounded fold edge; crumpled: a
 *  lumpy heap (noise-displaced ellipsoid with a flat underside). Deterministic
 *  per id so a given towel always looks the same. Carries a vertex-color
 *  attribute (wetness shading). */
export function clothGeometry(size, state, id) {
  const [sx, sy, sz] = size;
  const rnd = seeded(hash01(id) * 2 ** 31);
  const waves = [0, 1, 2, 3].map(() => ({
    f: [rnd() * 6 - 3, rnd() * 6 - 3, rnd() * 6 - 3], ph: rnd() * Math.PI * 2, a: 0.25 / (1 + rnd()),
  }));
  const noise = (x, y, z) => waves.reduce(
    (acc, w) => acc + w.a * Math.sin(w.f[0] * x + w.f[1] * y + w.f[2] * z + w.ph), 0,
  );

  let geom;
  if (state === 'crumpled') {
    geom = new THREE.IcosahedronGeometry(1, 3);
    const pos = geom.getAttribute('position');
    for (let i = 0; i < pos.count; i++) {
      const x = pos.getX(i);
      const y = pos.getY(i);
      const z = pos.getZ(i);
      const r = 0.85 + 0.35 * noise(x * 2, y * 2, z * 2);
      const px = Math.max(-1, Math.min(1, x * r)) * (sx / 2);
      const py = Math.max(-1, Math.min(1, y * r)) * (sy / 2);
      // heap: flat underside resting on the floor, domed lumpy top
      const pz = z < -0.2 ? -sz / 2 : Math.max(-sz / 2, Math.min(sz / 2, (z * r) * (sz / 2)));
      pos.setXYZ(i, px, py, pz);
    }
  } else {
    geom = new THREE.BoxGeometry(sx, sy, sz, 12, 8, 1);
    const pos = geom.getAttribute('position');
    for (let i = 0; i < pos.count; i++) {
      const x = pos.getX(i);
      const y = pos.getY(i);
      const z = pos.getZ(i);
      if (z > 0) {
        // quilted terry top + slightly thinner free edges (the fold is at -x)
        const edge = Math.min(1, (sx / 2 - x) / 0.04, (sy / 2 - Math.abs(y)) / 0.03);
        pos.setZ(i, z * (0.55 + 0.45 * Math.max(0, edge)) + 0.002 * noise(x * 40, y * 40, 0));
      }
    }
  }
  geom.computeVertexNormals();
  const colors = new Float32Array(geom.getAttribute('position').count * 3).fill(1);
  geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  return geom;
}

// ── helpers ─────────────────────────────────────────────────────────────────

const round3 = (v) => Math.round(v * 1000) / 1000;
const clamp01 = (v) => Math.max(0, Math.min(1, v));

/** Stable string hash -> [0, 1). */
export function hash01(str) {
  let h = 2166136261;
  for (let i = 0; i < str.length; i++) {
    h ^= str.charCodeAt(i);
    h = Math.imul(h, 16777619);
  }
  return (h >>> 0) / 4294967296;
}

function seeded(seed) {
  let s = (seed >>> 0) || 1;
  return () => {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

/** Rotate v by the conjugate of unit quaternion q (world -> body). */
function rotateByConj(q, v) {
  const x = -q.x;
  const y = -q.y;
  const z = -q.z;
  const w = q.w;
  const tx = 2 * (y * v.z - z * v.y);
  const ty = 2 * (z * v.x - x * v.z);
  const tz = 2 * (x * v.y - y * v.x);
  return {
    x: v.x + w * tx + (y * tz - z * ty),
    y: v.y + w * ty + (z * tx - x * tz),
    z: v.z + w * tz + (x * ty - y * tx),
  };
}

/** World-frame vertical half-height of an oriented box (AABB z half-extent). */
function worldHalfHeight(q, he) {
  // third row of the rotation matrix
  const r20 = 2 * (q.x * q.z - q.w * q.y);
  const r21 = 2 * (q.y * q.z + q.w * q.x);
  const r22 = 1 - 2 * (q.x * q.x + q.y * q.y);
  return Math.abs(r20) * he.x + Math.abs(r21) * he.y + Math.abs(r22) * he.z;
}

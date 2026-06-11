import * as THREE from 'three';
import { buildMaterials } from './materials.js';

const WOOD_ROOMS = new Set(['corridor', 'resting']);

/** Builds the onsen world (meshes + static colliders) from shared/onsen_layout.json. */
export class OnsenWorld {
  constructor(layout, profiles, physics, scene) {
    this.layout = layout;
    this.profiles = profiles;
    this.physics = physics;
    this.scene = scene;
    this.materials = buildMaterials();
    this.bins = [];
    this.steamZones = [];
    this._build();
  }

  _build() {
    const { layout, physics, scene, materials } = this;

    for (const room of layout.rooms) {
      const [x0, y0, x1, y1] = room.rect;
      const mat = WOOD_ROOMS.has(room.id) ? materials.woodFloor : materials.tileFloor;
      const floor = new THREE.Mesh(new THREE.PlaneGeometry(x1 - x0, y1 - y0), mat);
      floor.position.set((x0 + x1) / 2, (y0 + y1) / 2, 0);
      floor.receiveShadow = true;
      scene.add(floor);

      physics.addStaticBox({
        center: [(x0 + x1) / 2, (y0 + y1) / 2, -0.05],
        size: [x1 - x0, y1 - y0, 0.1],
        friction: this._floorFriction(room.id),
        meta: { kind: 'floor', id: room.id },
      });
      if (room.id === 'sauna') {
        this.steamZones.push({ rect: room.rect, density: 0.6 });
      }
    }

    for (const wall of layout.walls) {
      const [sx, sy] = wall.size;
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, wall.h), materials.wall);
      mesh.position.set(wall.c[0], wall.c[1], wall.h / 2);
      mesh.castShadow = mesh.receiveShadow = true;
      scene.add(mesh);
      physics.addStaticBox({
        center: [wall.c[0], wall.c[1], wall.h / 2],
        size: [sx, sy, wall.h],
        meta: { kind: 'wall', id: wall.id },
      });
    }

    for (const p of layout.platforms) {
      const [x0, y0, x1, y1] = p.rect;
      const mesh = new THREE.Mesh(
        new THREE.BoxGeometry(x1 - x0, y1 - y0, p.h), this.materials.woodProp,
      );
      mesh.position.set((x0 + x1) / 2, (y0 + y1) / 2, p.h / 2);
      mesh.receiveShadow = true;
      scene.add(mesh);
      physics.addStaticBox({
        center: [(x0 + x1) / 2, (y0 + y1) / 2, p.h / 2],
        size: [x1 - x0, y1 - y0, p.h],
        friction: this.profiles.floor_friction.wood,
        meta: { kind: 'platform', id: p.id },
      });
    }

    const propMat = {
      lounger: materials.woodProp, counter: materials.counter,
      bench: materials.woodProp, locker: materials.locker, table: materials.woodProp,
    };
    for (const prop of layout.static_props) {
      const [sx, sy] = prop.size;
      const z0 = prop.z0 ?? 0;
      const mesh = new THREE.Mesh(
        new THREE.BoxGeometry(sx, sy, prop.h), propMat[prop.type] ?? materials.woodProp,
      );
      mesh.position.set(prop.c[0], prop.c[1], z0 + prop.h / 2);
      mesh.castShadow = mesh.receiveShadow = true;
      scene.add(mesh);
      physics.addStaticBox({
        center: [prop.c[0], prop.c[1], z0 + prop.h / 2],
        size: [sx, sy, prop.h],
        meta: { kind: 'prop', id: prop.id, type: prop.type },
      });
    }

    for (const pool of layout.pools) {
      const [x0, y0, x1, y1] = pool.rect;
      const water = new THREE.Mesh(
        new THREE.PlaneGeometry(x1 - x0, y1 - y0), materials.water,
      );
      water.position.set((x0 + x1) / 2, (y0 + y1) / 2, pool.water_z);
      scene.add(water);
      this.steamZones.push({ rect: pool.rect, density: 0.35 });
    }

    for (const bin of layout.bins) {
      this._buildBin(bin);
    }
  }

  _buildBin(bin) {
    const { physics, scene, materials } = this;
    const [sx, sy] = bin.size;
    const mat = bin.type === 'towel' ? materials.binTowel : materials.binTrash;
    const t = 0.02;
    const sides = [
      { c: [bin.c[0] - sx / 2, bin.c[1]], size: [t, sy, bin.h] },
      { c: [bin.c[0] + sx / 2, bin.c[1]], size: [t, sy, bin.h] },
      { c: [bin.c[0], bin.c[1] - sy / 2], size: [sx, t, bin.h] },
      { c: [bin.c[0], bin.c[1] + sy / 2], size: [sx, t, bin.h] },
    ];
    for (const s of sides) {
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(s.size[0], s.size[1], s.size[2]), mat);
      mesh.position.set(s.c[0], s.c[1], bin.h / 2);
      mesh.castShadow = true;
      scene.add(mesh);
      physics.addStaticBox({
        center: [s.c[0], s.c[1], bin.h / 2],
        size: s.size,
        meta: { kind: 'bin_wall', id: bin.id },
      });
    }
    this.bins.push({
      id: bin.id,
      type: bin.type,
      rect: [bin.c[0] - sx / 2, bin.c[1] - sy / 2, bin.c[0] + sx / 2, bin.c[1] + sy / 2],
      rimZ: bin.h,
    });
  }

  _floorFriction(roomId) {
    const f = this.profiles.floor_friction;
    if (this.profiles.wet_rooms.includes(roomId)) return f.wet;
    return WOOD_ROOMS.has(roomId) ? f.wood : f.dry;
  }

  isWetAt(x, y) {
    for (const id of this.profiles.wet_rooms) {
      const room = this.layout.rooms.find((r) => r.id === id);
      if (room && inRect(x, y, room.rect)) return true;
    }
    return false;
  }

  poolAt(x, y) {
    return this.layout.pools.find((p) => inRect(x, y, p.rect)) ?? null;
  }

  steamDensityAt(x, y) {
    let d = 0;
    for (const zone of this.steamZones) {
      if (inRect(x, y, zone.rect)) d = Math.max(d, zone.density);
    }
    return d;
  }

  binAt(x, y, z) {
    return this.bins.find((b) => inRect(x, y, b.rect) && z < b.rimZ) ?? null;
  }
}

export const inRect = (x, y, [x0, y0, x1, y1]) => x >= x0 && x <= x1 && y >= y0 && y <= y1;

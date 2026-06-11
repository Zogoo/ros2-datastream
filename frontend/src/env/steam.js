import * as THREE from 'three';

const PARTICLES_PER_ZONE = 40;

/** Lightweight steam: drifting translucent sprites over pools and the sauna. */
export class Steam {
  constructor(zones, scene) {
    this.zones = zones;
    this.points = [];
    const tex = makePuffTexture();
    for (const zone of zones) {
      const mat = new THREE.SpriteMaterial({
        map: tex, color: 0xf2efe8, transparent: true,
        opacity: 0.12 + zone.density * 0.15, depthWrite: false,
      });
      for (let i = 0; i < PARTICLES_PER_ZONE; i++) {
        const sprite = new THREE.Sprite(mat);
        const s = 0.5 + Math.random() * 0.9;
        sprite.scale.set(s, s, 1);
        this._respawn(sprite, zone);
        sprite.position.z = Math.random() * 1.6 + 0.2;
        sprite.userData.zone = zone;
        sprite.userData.speed = 0.1 + Math.random() * 0.2;
        scene.add(sprite);
        this.points.push(sprite);
      }
    }
  }

  _respawn(sprite, zone) {
    const [x0, y0, x1, y1] = zone.rect;
    sprite.position.set(x0 + Math.random() * (x1 - x0), y0 + Math.random() * (y1 - y0), 0.2);
  }

  update(dt) {
    for (const sprite of this.points) {
      sprite.position.z += sprite.userData.speed * dt;
      sprite.position.x += Math.sin(sprite.position.z * 2) * 0.02 * dt;
      if (sprite.position.z > 2.2) this._respawn(sprite, sprite.userData.zone);
    }
  }
}

function makePuffTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = 64;
  const ctx = canvas.getContext('2d');
  const grad = ctx.createRadialGradient(32, 32, 4, 32, 32, 30);
  grad.addColorStop(0, 'rgba(255,255,255,0.8)');
  grad.addColorStop(1, 'rgba(255,255,255,0)');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, 64, 64);
  return new THREE.CanvasTexture(canvas);
}

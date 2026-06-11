import * as THREE from 'three';
import { TOPICS } from '../ros/topics.js';

const el = (id) => document.getElementById(id);

/** Scenario controls: throw towels, drag-and-fling objects with the pointer,
 *  reset the scene, send a camera frame to the AI worker. */
export class Scenario {
  constructor(objects, robot, rng, ros, camera, renderer, frontCamera, hud) {
    this.objects = objects;
    this.robot = robot;
    this.rng = rng;
    this.ros = ros;
    this.camera = camera;
    this.hud = hud;

    this.dragEnabled = false;
    this.dragged = null;
    this.dragPlaneZ = 0.35;
    this.lastDragPos = null;
    this.dragVelocity = { x: 0, y: 0, z: 0 };
    this.raycaster = new THREE.Raycaster();
    this.pointer = new THREE.Vector2();

    el('throw-towel-btn').addEventListener('click', () => this.throwTowel());
    el('reset-scene-btn').addEventListener('click', () => {
      this.robot.arm.forceRelease();
      this.objects.reset();
      this.hud.ticker('SCENE_RESET');
    });
    el('drag-mode-btn').addEventListener('click', () => {
      this.dragEnabled = !this.dragEnabled;
      el('drag-mode-btn').textContent = `DRAG: ${this.dragEnabled ? 'ON' : 'OFF'}`;
      el('drag-mode-btn').classList.toggle('active', this.dragEnabled);
    });
    el('upload-btn').addEventListener('click', () => this._uploadFrame(frontCamera));

    const dom = renderer.domElement;
    dom.addEventListener('pointerdown', (e) => this._onPointerDown(e, dom));
    dom.addEventListener('pointermove', (e) => this._onPointerMove(e, dom));
    dom.addEventListener('pointerup', () => this._onPointerUp());
  }

  throwTowel() {
    const item = this.objects.throwTowel(this.rng);
    this.hud.ticker(`TOWEL_THROWN ${item.id}`);
    this.ros.publish(TOPICS.events, {
      data: JSON.stringify({ event: 'TOWEL_THROWN', object_id: item.id, timestamp: new Date().toISOString() }),
    });
  }

  _setPointer(e, dom) {
    const rect = dom.getBoundingClientRect();
    this.pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
  }

  _onPointerDown(e, dom) {
    if (!this.dragEnabled) return;
    this._setPointer(e, dom);
    this.raycaster.setFromCamera(this.pointer, this.camera);
    const meshes = this.objects.items.filter((i) => !i.held && !i.binned).map((i) => i.mesh);
    const hits = this.raycaster.intersectObjects(meshes, false);
    if (!hits.length) return;
    const item = this.objects.items.find((i) => i.mesh === hits[0].object);
    if (!item) return;
    this.dragged = item;
    item.held = true;
    item.body.setBodyType(this.objects.physics.R.RigidBodyType.KinematicPositionBased, true);
    this.lastDragPos = null;
  }

  _onPointerMove(e, dom) {
    if (!this.dragged) return;
    this._setPointer(e, dom);
    this.raycaster.setFromCamera(this.pointer, this.camera);
    const ray = this.raycaster.ray;
    if (Math.abs(ray.direction.z) < 1e-4) return;
    const t = (this.dragPlaneZ - ray.origin.z) / ray.direction.z;
    if (t < 0) return;
    const target = {
      x: ray.origin.x + ray.direction.x * t,
      y: ray.origin.y + ray.direction.y * t,
      z: this.dragPlaneZ,
    };
    if (this.lastDragPos) {
      this.dragVelocity = {
        x: (target.x - this.lastDragPos.x) * 30,
        y: (target.y - this.lastDragPos.y) * 30,
        z: 0.5,
      };
    }
    this.lastDragPos = target;
    this.dragged.body.setNextKinematicTranslation(target);
  }

  _onPointerUp() {
    if (!this.dragged) return;
    const item = this.dragged;
    this.dragged = null;
    item.held = false;
    item.body.setBodyType(this.objects.physics.R.RigidBodyType.Dynamic, true);
    item.body.setLinvel(clampVel(this.dragVelocity, 4), true);
    this.dragVelocity = { x: 0, y: 0, z: 0 };
  }

  async _uploadFrame(frontCamera) {
    if (!frontCamera.lastDataUrl) {
      this.hud.ticker('NO_FRAME_YET');
      return;
    }
    try {
      const blob = await (await fetch(frontCamera.lastDataUrl)).blob();
      const res = await fetch('/api/upload', { method: 'POST', body: blob });
      const body = await res.json();
      this.hud.ticker(`AI: ${body.objects?.length ?? 0} detections`);
    } catch {
      this.hud.ticker('AI_WORKER_UNREACHABLE');
    }
  }
}

function clampVel(v, max) {
  const mag = Math.hypot(v.x, v.y, v.z);
  if (mag <= max) return v;
  const s = max / mag;
  return { x: v.x * s, y: v.y * s, z: v.z * s };
}

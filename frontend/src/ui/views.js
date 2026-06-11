import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

/** ORBIT / FOLLOW / FPS view management for the main render camera. */
export class Views {
  constructor(camera, renderer, robot) {
    this.camera = camera;
    this.robot = robot;
    this.mode = 'orbit';
    this.orbit = new OrbitControls(camera, renderer.domElement);
    this.orbit.target.set(0, 0, 0.5);
    camera.position.set(6, -8, 7);
    camera.up.set(0, 0, 1);
    this.orbit.update();

    for (const btn of document.querySelectorAll('[data-view]')) {
      btn.addEventListener('click', () => this.setMode(btn.dataset.view));
    }
  }

  setMode(mode) {
    this.mode = mode;
    this.orbit.enabled = mode === 'orbit';
    for (const btn of document.querySelectorAll('[data-view]')) {
      btn.classList.toggle('active', btn.dataset.view === mode);
    }
  }

  update(frontCamSpec) {
    const pose = this.robot.pose();
    if (this.mode === 'orbit') {
      this.orbit.update();
      return;
    }
    if (this.mode === 'follow') {
      const dist = 2.2;
      const target = new THREE.Vector3(
        pose.x - Math.cos(pose.yaw) * dist,
        pose.y - Math.sin(pose.yaw) * dist,
        pose.z + 1.4,
      );
      this.camera.position.lerp(target, 0.08);
      this.camera.lookAt(pose.x, pose.y, pose.z + 0.3);
      return;
    }
    // FPS: ride the front camera mount
    const p = this.robot.worldPoint(frontCamSpec.position);
    const pitch = (frontCamSpec.pitch_deg * Math.PI) / 180;
    const fwd = this.robot.worldDir([Math.cos(pitch), 0, Math.sin(pitch)]);
    this.camera.position.set(p.x, p.y, p.z);
    this.camera.lookAt(p.x + fwd.x, p.y + fwd.y, p.z + fwd.z);
  }
}

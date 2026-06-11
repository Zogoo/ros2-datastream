import * as THREE from 'three';
import { TOPICS } from '../ros/topics.js';

/** RGB camera simulated by a real offscreen WebGL render from the mount pose.
 *  Output: JPEG CompressedImage + CameraInfo with intrinsics derived from the
 *  actual projection. Steam zones get a contrast/warmth grade; frames can drop. */
export class RgbCamera {
  constructor(key, spec, realism, renderer, scene, robot, world, rng, ros, clock) {
    this.key = key;
    this.spec = spec.sensors[key];
    this.realism = realism;
    this.renderer = renderer;
    this.scene = scene;
    this.robot = robot;
    this.world = world;
    this.rng = rng;
    this.ros = ros;
    this.clock = clock;

    const { width, height, hfov_deg: hfov } = this.spec;
    this.target = new THREE.WebGLRenderTarget(width, height);
    const vfov = 2 * Math.atan(Math.tan((hfov * Math.PI) / 360) * (height / width)) * (180 / Math.PI);
    this.camera = new THREE.PerspectiveCamera(vfov, width / height, 0.05, 30);
    this.camera.up.set(0, 0, 1);

    this.pixels = new Uint8Array(width * height * 4);
    this.canvas = document.createElement('canvas');
    this.canvas.width = width;
    this.canvas.height = height;
    this.ctx = this.canvas.getContext('2d');
    this.imageData = this.ctx.createImageData(width, height);

    this.accumulator = 0;
    this.topicImage = key === 'camera_front' ? TOPICS.camFrontCompressed : TOPICS.camRearCompressed;
    this.topicInfo = key === 'camera_front' ? TOPICS.camFrontInfo : TOPICS.camRearInfo;
    this.lastDataUrl = null;
  }

  poseCamera() {
    const s = this.spec;
    const pos = this.robot.worldPoint(s.position);
    this.camera.position.set(pos.x, pos.y, pos.z);
    const yawOff = ((s.yaw_deg ?? 0) * Math.PI) / 180;
    const pitch = ((s.pitch_deg ?? 0) * Math.PI) / 180;
    const fwdLocal = [
      Math.cos(yawOff) * Math.cos(pitch),
      Math.sin(yawOff) * Math.cos(pitch),
      Math.sin(pitch),
    ];
    const fwd = this.robot.worldDir(fwdLocal);
    this.camera.lookAt(pos.x + fwd.x, pos.y + fwd.y, pos.z + fwd.z);
  }

  update(dt) {
    this.accumulator += dt;
    if (this.accumulator < 1 / this.spec.hz) return;
    this.accumulator %= 1 / this.spec.hz;

    const dropProb = this.spec.drop_prob?.[this.realism] ?? 0;
    if (this.rng.uniform() < dropProb) return;

    this.poseCamera();
    this.renderer.setRenderTarget(this.target);
    this.renderer.render(this.scene, this.camera);
    this.renderer.readRenderTargetPixels(
      this.target, 0, 0, this.spec.width, this.spec.height, this.pixels,
    );
    this.renderer.setRenderTarget(null);

    this._composeAndPublish();
  }

  _composeAndPublish() {
    const { width, height } = this.spec;
    const src = this.pixels;
    const dst = this.imageData.data;
    const pos = this.robot.worldPoint(this.spec.position);
    const steam = this.world.steamDensityAt(pos.x, pos.y);

    // Flip vertically (GL is bottom-up) with optional steam grade.
    for (let y = 0; y < height; y++) {
      const srcRow = (height - 1 - y) * width * 4;
      const dstRow = y * width * 4;
      for (let x = 0; x < width * 4; x += 4) {
        let r = src[srcRow + x];
        let g = src[srcRow + x + 1];
        let b = src[srcRow + x + 2];
        if (steam > 0) {
          const f = steam * 0.45;
          r = r * (1 - f) + 235 * f;
          g = g * (1 - f) + 228 * f;
          b = b * (1 - f) + 215 * f;
        }
        dst[dstRow + x] = r;
        dst[dstRow + x + 1] = g;
        dst[dstRow + x + 2] = b;
        dst[dstRow + x + 3] = 255;
      }
    }
    this.ctx.putImageData(this.imageData, 0, 0);

    const quality = this.spec.jpeg_quality?.[this.realism] ?? 0.8;
    const dataUrl = this.canvas.toDataURL('image/jpeg', quality);
    this.lastDataUrl = dataUrl;
    const stamp = this.clock.stamp();
    const header = { stamp, frame_id: this.spec.frame_id };

    this.ros.publish(this.topicImage, {
      header,
      format: 'jpeg',
      data: dataUrl.slice(dataUrl.indexOf(',') + 1),
    });
    this.ros.publish(this.topicInfo, cameraInfo(header, this.spec, this.camera));
  }
}

export function cameraInfo(header, spec, camera) {
  const { width, height } = spec;
  const fy = (height / 2) / Math.tan((camera.fov * Math.PI) / 360);
  const fx = fy;
  const cx = width / 2;
  const cy = height / 2;
  return {
    header,
    height,
    width,
    distortion_model: 'plumb_bob',
    d: [0, 0, 0, 0, 0],
    k: [fx, 0, cx, 0, fy, cy, 0, 0, 1],
    r: [1, 0, 0, 0, 1, 0, 0, 0, 1],
    p: [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0],
    binning_x: 0,
    binning_y: 0,
    roi: { x_offset: 0, y_offset: 0, height: 0, width: 0, do_rectify: false },
  };
}

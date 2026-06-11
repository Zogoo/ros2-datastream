import * as THREE from 'three';
import { TOPICS } from '../ros/topics.js';
import { cameraInfo } from './rgbCamera.js';

/** RGB-D depth channel: renders the scene with a depth-packing material, reads
 *  back the depth buffer, linearizes to metric depth and quantizes to 16UC1 mm
 *  (RealSense-style). Noise grows ~z^2; grazing/out-of-range pixels are 0. */
export class DepthCamera {
  constructor(spec, renderer, scene, robot, rng, ros, clock) {
    this.spec = spec.sensors.camera_depth;
    this.renderer = renderer;
    this.scene = scene;
    this.robot = robot;
    this.rng = rng;
    this.ros = ros;
    this.clock = clock;

    const { width, height, hfov_deg: hfov } = this.spec;
    this.near = 0.05;
    this.far = this.spec.range_max + 2;
    this.target = new THREE.WebGLRenderTarget(width, height);
    const vfov = 2 * Math.atan(Math.tan((hfov * Math.PI) / 360) * (height / width)) * (180 / Math.PI);
    this.camera = new THREE.PerspectiveCamera(vfov, width / height, this.near, this.far);
    this.camera.up.set(0, 0, 1);

    this.depthMaterial = new THREE.MeshDepthMaterial({ depthPacking: THREE.RGBADepthPacking });
    this.pixels = new Uint8Array(width * height * 4);
    this.depthMm = new Uint8Array(width * height * 2);
    this.accumulator = 0;
  }

  update(dt, poseFn) {
    this.accumulator += dt;
    if (this.accumulator < 1 / this.spec.hz) return;
    this.accumulator %= 1 / this.spec.hz;

    poseFn(this.camera, this.spec);
    const prevOverride = this.scene.overrideMaterial;
    this.scene.overrideMaterial = this.depthMaterial;
    this.renderer.setRenderTarget(this.target);
    this.renderer.render(this.scene, this.camera);
    this.renderer.readRenderTargetPixels(
      this.target, 0, 0, this.spec.width, this.spec.height, this.pixels,
    );
    this.renderer.setRenderTarget(null);
    this.scene.overrideMaterial = prevOverride;

    this._convertAndPublish();
  }

  _convertAndPublish() {
    const { width, height, range_min: rmin, range_max: rmax, noise_coeff: nc } = this.spec;
    const src = this.pixels;
    const out = this.depthMm;
    const UnpackFactors = [1 / (256 ** 3), 1 / (256 ** 2), 1 / 256, 1];

    for (let y = 0; y < height; y++) {
      const srcRow = (height - 1 - y) * width * 4;
      const outRow = y * width * 2;
      for (let x = 0; x < width; x++) {
        const i = srcRow + x * 4;
        const packed = (src[i] * UnpackFactors[3] + src[i + 1] * UnpackFactors[2]
          + src[i + 2] * UnpackFactors[1] + src[i + 3] * UnpackFactors[0]) / 256;
        // packed is the perspective (non-linear) depth; linearize to view-space meters.
        const viewZ = packed >= 1.0 ? Infinity
          : (this.near * this.far) / (this.far - packed * (this.far - this.near));
        let mm = 0;
        if (viewZ > rmin && viewZ < rmax) {
          const noisy = viewZ + this.rng.gaussian(0, nc * viewZ * viewZ);
          mm = Math.max(0, Math.min(65535, Math.round(noisy * 1000)));
        }
        const o = outRow + x * 2;
        out[o] = mm & 0xff;
        out[o + 1] = mm >> 8;
      }
    }

    const stamp = this.clock.stamp();
    const header = { stamp, frame_id: this.spec.frame_id };
    this.ros.publish(TOPICS.camDepth, {
      header,
      height,
      width,
      encoding: '16UC1',
      is_bigendian: 0,
      step: width * 2,
      data: bytesToBase64(out),
    });
    this.ros.publish(TOPICS.camDepthInfo, cameraInfo(header, this.spec, this.camera));
  }
}

function bytesToBase64(bytes) {
  let binary = '';
  const chunk = 0x8000;
  for (let i = 0; i < bytes.length; i += chunk) {
    binary += String.fromCharCode.apply(null, bytes.subarray(i, i + chunk));
  }
  return btoa(binary);
}

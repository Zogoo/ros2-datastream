import { TOPICS } from '../ros/topics.js';

const MAX_VX = 0.6;
const MAX_WZ = 1.5;
const ARM_KEY_POSES = [
  'A HOME', 'A READY', 'A STOW', 'A PRE_PICK', 'A PICK_LOWER',
  'A PICK_GRIP', 'A PICK_LIFT', 'A DROP_BASKET', 'A DROP_RELEASE',
];

/** Manual drive (keyboard / d-pad / gamepad) -> /cmd_vel/ui at 10 Hz,
 *  AUTO/MANUAL mode buttons, number-key arm poses. */
export class Controls {
  constructor(ros, onThrowTowel) {
    this.ros = ros;
    this.keys = new Set();
    this.dpad = { fwd: false, back: false, left: false, right: false };
    this.mode = 'auto';
    this.publishTimer = 0;

    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT') return;
      this.keys.add(e.code);
      if (e.code === 'Space') this._stop();
      if (e.code === 'KeyT') onThrowTowel();
      const poseIdx = parseInt(e.key, 10) - 1;
      if (poseIdx >= 0 && poseIdx < ARM_KEY_POSES.length) {
        this.ros.publish(TOPICS.armCommand, { data: ARM_KEY_POSES[poseIdx] });
      }
    });
    window.addEventListener('keyup', (e) => this.keys.delete(e.code));

    const bind = (id, key) => {
      const el = document.getElementById(id);
      const set = (v) => () => { this.dpad[key] = v; };
      el.addEventListener('pointerdown', set(true));
      el.addEventListener('pointerup', set(false));
      el.addEventListener('pointerleave', set(false));
    };
    bind('dpad-fwd', 'fwd');
    bind('dpad-back', 'back');
    bind('dpad-left', 'left');
    bind('dpad-right', 'right');
    document.getElementById('dpad-stop').addEventListener('click', () => this._stop());

    for (const btn of document.querySelectorAll('[data-drive-mode]')) {
      btn.addEventListener('click', () => this.setMode(btn.dataset.driveMode, true));
    }

    // E-stop is opt-in: latching in the safety aggregator only happens while armed.
    this.safetyArmed = false;
    const armBtn = document.getElementById('estop-arm-btn');
    armBtn.addEventListener('click', () => {
      this.safetyArmed = !this.safetyArmed;
      armBtn.textContent = `E-STOP: ${this.safetyArmed ? 'ARMED' : 'OFF'}`;
      armBtn.classList.toggle('active', this.safetyArmed);
      this.ros.publish(TOPICS.safetyEnable, { data: this.safetyArmed });
    });
    for (const btn of document.querySelectorAll('#arm-pose-btns .arm-btn')) {
      btn.addEventListener('click', () => this.ros.publish(TOPICS.armCommand, { data: btn.dataset.cmd }));
    }
  }

  setMode(mode, publish = false) {
    this.mode = mode;
    for (const btn of document.querySelectorAll('[data-drive-mode]')) {
      btn.classList.toggle('active', btn.dataset.driveMode === mode);
    }
    if (publish) this.ros.publish(TOPICS.controlModeSet, { data: mode });
  }

  _stop() {
    this.ros.publish(TOPICS.cmdVelUi, twist(0, 0));
  }

  _gamepadAxes() {
    const pads = navigator.getGamepads?.() ?? [];
    for (const pad of pads) {
      if (!pad) continue;
      const vx = -deadzone(pad.axes[1]);
      const wz = -deadzone(pad.axes[0]);
      if (vx !== 0 || wz !== 0) return { vx, wz };
    }
    return null;
  }

  update(dt) {
    this.publishTimer += dt;
    if (this.publishTimer < 0.1) return;
    this.publishTimer = 0;

    let vx = 0;
    let wz = 0;
    if (this.keys.has('KeyW') || this.keys.has('ArrowUp') || this.dpad.fwd) vx += 1;
    if (this.keys.has('KeyS') || this.keys.has('ArrowDown') || this.dpad.back) vx -= 1;
    if (this.keys.has('KeyA') || this.keys.has('ArrowLeft') || this.dpad.left) wz += 1;
    if (this.keys.has('KeyD') || this.keys.has('ArrowRight') || this.dpad.right) wz -= 1;

    const pad = this._gamepadAxes();
    if (pad) {
      vx = pad.vx;
      wz = pad.wz;
    }

    if (vx !== 0 || wz !== 0) {
      if (this.mode !== 'manual') this.setMode('manual', true);
      this.ros.publish(TOPICS.cmdVelUi, twist(vx * MAX_VX, wz * MAX_WZ));
      this.active = true;
    } else if (this.active) {
      this.active = false;
      this._stop();
    }
  }
}

const twist = (vx, wz) => ({
  linear: { x: vx, y: 0, z: 0 },
  angular: { x: 0, y: 0, z: wz },
});

const deadzone = (v, t = 0.15) => (Math.abs(v) < t ? 0 : v);

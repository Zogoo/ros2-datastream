import { TOPICS } from '../ros/topics.js';

const el = (id) => document.getElementById(id);

/** All HUD panels: pose, arm, detections, mission, LIDAR minimap, camera PiP,
 *  connection + e-stop badges, event ticker. */
export class Hud {
  constructor(ros, robot, lidar, frontCamera, odom, objects) {
    this.ros = ros;
    this.robot = robot;
    this.lidar = lidar;
    this.frontCamera = frontCamera;
    this.odom = odom;
    this.objects = objects;

    this.lidarCtx = el('lidar-canvas').getContext('2d');
    this.pipCtx = el('cam-pip').getContext('2d');
    this.tickerTimeout = null;
    this.accumulator = 0;

    ros.onStatusChange = (connected) => {
      const badge = el('conn-status');
      badge.textContent = connected ? '\u25CF CONNECTED' : '\u25CF DISCONNECTED';
      badge.className = connected ? 'ok' : 'err';
    };

    ros.subscribeJson(TOPICS.controlMode, (m) => {
      const badge = el('ctrl-mode-badge');
      badge.textContent = m.mode.toUpperCase();
      badge.className = m.mode === 'manual' ? 'badge-manual' : 'badge-auto';
    });
    ros.subscribeJson(TOPICS.detectedObjects, (m) => this._renderDetections(m));
    ros.subscribeJson(TOPICS.taskPlan, (m) => {
      el('task-reason').textContent = `${m.next_action}: ${m.reason ?? ''}`.slice(0, 64);
    });
    ros.subscribeJson(TOPICS.missionState, (m) => {
      el('mission-state').textContent = m.state ?? '--';
    });
    ros.subscribe(TOPICS.safetyStop, (m) => {
      el('estop-badge').classList.toggle('hidden', !m.data);
    });
    ros.subscribeJson(TOPICS.events, (m) => this.ticker(m.event ?? JSON.stringify(m).slice(0, 48)));
  }

  ticker(text) {
    el('event-ticker').textContent = text;
    clearTimeout(this.tickerTimeout);
    this.tickerTimeout = setTimeout(() => { el('event-ticker').textContent = ''; }, 4000);
  }

  _renderDetections(m) {
    const objs = m.objects ?? [];
    el('detect-count').textContent = `${objs.length} objects`;
    el('detect-list').innerHTML = objs.slice(0, 6)
      .map((o) => `${o.class} ${(o.confidence * 100).toFixed(0)}%`)
      .join('<br>');
  }

  update(dt, fps) {
    this.accumulator += dt;
    if (this.accumulator < 0.2) return;
    this.accumulator = 0;

    const pose = this.robot.pose();
    el('px').textContent = pose.x.toFixed(2);
    el('py').textContent = pose.y.toFixed(2);
    el('pyaw').textContent = ((pose.yaw * 180) / Math.PI).toFixed(1);
    el('ox').textContent = this.odom.pose.x.toFixed(2);
    el('oy').textContent = this.odom.pose.y.toFixed(2);
    el('sim-fps').textContent = `${Math.round(fps)} FPS`;
    el('clock').textContent = new Date().toLocaleTimeString();
    el('pub-counter').textContent = `TX:${this.ros.txCount}`;
    el('arm-status-val').textContent = this.robot.arm.holding() ? 'HOLDING' : 'IDLE';
    el('arm-joints-val').textContent = this.robot.arm.statusForHud();
    const binned = this.objects.items.filter((i) => i.binned).length;
    el('basket-count').textContent = `BINNED: ${binned}`;

    this._drawLidar();
    this._drawPip();
  }

  _drawLidar() {
    const ctx = this.lidarCtx;
    const size = 160;
    ctx.fillStyle = '#050507';
    ctx.fillRect(0, 0, size, size);
    const scan = this.lidar.lastScan;
    if (!scan) return;
    const scale = (size / 2) / this.lidar.spec.range_max;
    ctx.fillStyle = '#7ddc7d';
    const c = size / 2;
    for (let i = 0; i < scan.length; i += 2) {
      const r = scan[i];
      if (r >= this.lidar.spec.range_max) continue;
      const a = -Math.PI + (i * 2 * Math.PI) / scan.length;
      ctx.fillRect(c + Math.cos(a) * r * scale, c - Math.sin(a) * r * scale, 1.5, 1.5);
    }
    ctx.fillStyle = '#ffb74d';
    ctx.fillRect(c - 2, c - 2, 4, 4);
  }

  _drawPip() {
    if (!this.frontCamera.canvas) return;
    this.pipCtx.drawImage(this.frontCamera.canvas, 0, 0, 160, 120);
  }
}

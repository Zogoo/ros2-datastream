import WebSocket from 'ws';

const ROSBRIDGE_URL = process.env.ROSBRIDGE_URL ?? 'ws://localhost:9090';

/** Minimal rosbridge client for assertions from the test runner side. */
export class RosProbe {
  constructor() {
    this.messages = new Map(); // topic -> [msg]
    this.ws = null;
  }

  async connect() {
    this.ws = new WebSocket(ROSBRIDGE_URL);
    await new Promise((resolve, reject) => {
      this.ws.once('open', resolve);
      this.ws.once('error', reject);
    });
    this.ws.on('message', (raw) => {
      const msg = JSON.parse(raw);
      if (msg.op === 'publish') {
        if (!this.messages.has(msg.topic)) this.messages.set(msg.topic, []);
        this.messages.get(msg.topic).push(msg.msg);
      }
    });
  }

  subscribe(topic, throttleMs = 0) {
    this.messages.set(topic, []);
    this.ws.send(JSON.stringify({ op: 'subscribe', topic, throttle_rate: throttleMs }));
  }

  async publish(topic, type, msg) {
    this.ws.send(JSON.stringify({ op: 'advertise', topic, type }));
    await sleep(150); // let rosbridge register the publisher
    this.ws.send(JSON.stringify({ op: 'publish', topic, msg }));
  }

  /** The e-stop latch lives in the robot_state node and survives page reloads —
   *  every test that drives must start from a cleared latch. */
  async clearSafety() {
    this.subscribe('/safety/stop');
    await this.publish('/safety/reset', 'std_msgs/Bool', { data: true });
    await this.waitFor('/safety/stop', (m) => m.data === false, 15_000, 'safety latch cleared');
  }

  received(topic) {
    return this.messages.get(topic) ?? [];
  }

  clear(topic) {
    this.messages.set(topic, []);
  }

  /** Waits until predicate matches any received message on topic. */
  async waitFor(topic, predicate, timeoutMs = 30_000, label = topic) {
    const start = Date.now();
    while (Date.now() - start < timeoutMs) {
      const hit = this.received(topic).find(predicate);
      if (hit) return hit;
      await sleep(200);
    }
    throw new Error(`timeout waiting for ${label} (${this.received(topic).length} msgs seen)`);
  }

  close() {
    this.ws?.close();
  }
}

export const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

/** Boots the sim page and waits for rosbridge connection + first physics.
 *  Generous timeouts: the first cold load compiles the Rapier WASM and warms
 *  SwiftShader, which can take >30 s in a fresh headless container. */
export async function bootSim(page) {
  await page.goto('/');
  await page.waitForFunction(() => window.__sim !== undefined, null, { timeout: 90_000 });
  await page.locator('#conn-status').filter({ hasText: 'CONNECTED' }).waitFor({ timeout: 60_000 });
}

export async function setManual(page) {
  await page.locator('[data-drive-mode="manual"]').click();
}

export async function setAuto(page) {
  await page.locator('[data-drive-mode="auto"]').click();
}

/** Holds a D-pad button for ms (pointer events drive /cmd_vel/ui). */
export async function holdButton(page, selector, ms) {
  const btn = page.locator(selector);
  await btn.dispatchEvent('pointerdown');
  await sleep(ms);
  await btn.dispatchEvent('pointerup');
}

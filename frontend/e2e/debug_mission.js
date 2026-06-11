import { chromium } from '@playwright/test';
import WebSocket from 'ws';

const ws = new WebSocket('ws://localhost:9090');
await new Promise((r) => ws.once('open', r));
const states = [];
ws.on('message', (raw) => {
  const m = JSON.parse(raw);
  if (m.op === 'publish' && m.topic === '/mission/state') states.push(JSON.parse(m.msg.data));
});
const send = (o) => ws.send(JSON.stringify(o));
send({ op: 'subscribe', topic: '/mission/state', throttle_rate: 400 });
send({ op: 'advertise', topic: '/safety/reset', type: 'std_msgs/Bool' });

const browser = await chromium.launch({ args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader-webgl'] });
const page = await browser.newPage();
await page.goto(process.env.FE_URL ?? 'http://localhost:8080');
await page.waitForFunction(() => window.__sim !== undefined, null, { timeout: 60000 });
await page.locator('#conn-status').filter({ hasText: 'CONNECTED' }).waitFor({ timeout: 30000 });
await new Promise((r) => setTimeout(r, 500));
send({ op: 'publish', topic: '/safety/reset', msg: { data: true } });
await new Promise((r) => setTimeout(r, 500));

await page.evaluate(() => {
  window.__sim.setPose(0, 1.9, Math.PI / 2);
  window.__sim.spawn('towel', 0.05, 2.9, 0.1);
});
await page.locator('[data-drive-mode="auto"]').click();

let last = '';
for (let i = 0; i < 60; i++) {
  await new Promise((r) => setTimeout(r, 2000));
  const s = states.at(-1) ?? {};
  const p = await page.evaluate(() => window.__sim.pose());
  const held = await page.evaluate(() => window.__sim.holding());
  const line = `state=${s.state} holding=${s.holding} feHeld=${held} reason="${s.reason}" robot=(${p.x.toFixed(2)},${p.y.toFixed(2)},${p.yaw.toFixed(2)})`;
  if (line !== last) console.log(`t=${i * 2}s ${line}`);
  last = line;
  if (s.holding) { console.log('GRASPED'); break; }
}
await browser.close();
ws.close();

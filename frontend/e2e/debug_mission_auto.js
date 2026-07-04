import { chromium } from '@playwright/test';
import WebSocket from 'ws';

const ws = new WebSocket('ws://localhost:9090');
await new Promise((r) => ws.once('open', r));
let mission = {}, navStatus = {}, events = [];
ws.on('message', (raw) => {
  const m = JSON.parse(raw);
  if (m.op !== 'publish') return;
  if (m.topic === '/mission/state') mission = JSON.parse(m.msg.data);
  if (m.topic === '/nav/status') navStatus = JSON.parse(m.msg.data);
  if (m.topic === '/robot/events') events.push(JSON.parse(m.msg.data));
});
const send = (o) => ws.send(JSON.stringify(o));
send({ op: 'subscribe', topic: '/mission/state', throttle_rate: 400 });
send({ op: 'subscribe', topic: '/nav/status', throttle_rate: 400 });
send({ op: 'subscribe', topic: '/robot/events' });

const browser = await chromium.launch({ args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader-webgl'] });
const page = await browser.newPage();
await page.goto('http://localhost:8080');
await page.waitForFunction(() => window.__sim !== undefined, null, { timeout: 60000 });
await page.locator('#conn-status').filter({ hasText: 'CONNECTED' }).waitFor({ timeout: 30000 });
// let localizer + tracker warm up
await new Promise((r) => setTimeout(r, 4000));
await page.evaluate(() => {
  window.__sim.setPose(0, -1.0, Math.PI / 2);
  window.__sim.spawn('towel', 0.1, 0.6, 0.1);
});
await page.locator('[data-drive-mode="auto"]').click();

let last = '';
let binned = null;
for (let i = 0; i < 150; i++) {
  await new Promise((r) => setTimeout(r, 2000));
  const gt = await page.evaluate(() => window.__sim.pose());
  const held = await page.evaluate(() => window.__sim.holding());
  const line = `mission=${mission.state} nav=${navStatus.state} holding=${mission.holding}/${held} src=${mission.pose_source} reason="${mission.reason}"`;
  if (line !== last) console.log(`t=${i*2}s ${line} gt=(${gt.x.toFixed(2)},${gt.y.toFixed(2)})`);
  last = line;
  const b = events.find((e) => e.event === 'OBJECT_BINNED' && e.object_class === 'towel');
  if (b) { binned = b; break; }
}
console.log('BINNED:', JSON.stringify(binned));
await browser.close();
ws.close();

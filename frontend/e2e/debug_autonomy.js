import { chromium } from '@playwright/test';
import WebSocket from 'ws';

const ws = new WebSocket('ws://localhost:9090');
await new Promise((r) => ws.once('open', r));
let mission = {}, nav = {}, binned = null, grasp = null;
ws.on('message', (raw) => {
  const m = JSON.parse(raw);
  if (m.op !== 'publish') return;
  if (m.topic === '/mission/state') mission = JSON.parse(m.msg.data);
  if (m.topic === '/nav/status') nav = JSON.parse(m.msg.data);
  if (m.topic === '/robot/events') {
    const e = JSON.parse(m.msg.data);
    if (e.event === 'OBJECT_BINNED') binned = e;
    if (e.event === 'GRASP_ACQUIRED') grasp = e;
  }
});
const send = (o) => ws.send(JSON.stringify(o));
for (const t of ['/mission/state', '/nav/status', '/robot/events']) send({ op: 'subscribe', topic: t, throttle_rate: 300 });

const browser = await chromium.launch({ args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader-webgl'] });
const page = await browser.newPage();
await page.goto('http://localhost:8080');
await page.waitForFunction(() => window.__sim !== undefined, null, { timeout: 60000 });
await page.locator('#conn-status').filter({ hasText: 'CONNECTED' }).waitFor({ timeout: 30000 });
// stage: robot in corridor, towel ahead and OFF-AXIS (tests IK pan, not canned scoop)
await page.evaluate(() => {
  window.__sim.setPose(0, -1.0, Math.PI / 2);
  window.__sim.spawn('towel', 0.45, 0.2, 0.1);  // ahead-left, ~25 deg off-axis
});
await new Promise((r) => setTimeout(r, 8000));  // localizer converge before AUTO
await page.locator('[data-drive-mode="auto"]').click();

let last = '';
for (let i = 0; i < 240; i++) {
  await new Promise((r) => setTimeout(r, 1000));
  const gt = await page.evaluate(() => window.__sim.pose());
  const held = await page.evaluate(() => window.__sim.holding());
  const line = `mission=${mission.state} nav=${nav.state} held=${held} graspEvt=${!!grasp} binned=${!!binned} reason="${(mission.reason||'').slice(0,40)}"`;
  if (line !== last) console.log(`t=${i}s ${line} gt=(${gt.x.toFixed(2)},${gt.y.toFixed(2)})`);
  last = line;
  if (binned) { console.log('SUCCESS: towel binned', JSON.stringify(binned).slice(0,120)); break; }
}
await browser.close();
ws.close();

import { chromium } from '@playwright/test';
import WebSocket from 'ws';

const ws = new WebSocket('ws://localhost:9090');
await new Promise((r) => ws.once('open', r));
let navStatus = {};
let locPose = null;
ws.on('message', (raw) => {
  const m = JSON.parse(raw);
  if (m.op !== 'publish') return;
  if (m.topic === '/nav/status') navStatus = JSON.parse(m.msg.data);
  if (m.topic === '/localization/pose') locPose = m.msg.pose;
});
const send = (o) => ws.send(JSON.stringify(o));
send({ op: 'subscribe', topic: '/nav/status', throttle_rate: 300 });
send({ op: 'subscribe', topic: '/localization/pose', throttle_rate: 300 });
send({ op: 'advertise', topic: '/nav/goal', type: 'std_msgs/String' });

const browser = await chromium.launch({ args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader-webgl'] });
const page = await browser.newPage();
await page.goto('http://localhost:8080');
await page.waitForFunction(() => window.__sim !== undefined, null, { timeout: 60000 });
await page.locator('#conn-status').filter({ hasText: 'CONNECTED' }).waitFor({ timeout: 30000 });
await page.locator('[data-drive-mode="auto"]').click();
await new Promise((r) => setTimeout(r, 3000));  // let localizer converge

// goal: makeup area, far north-west — a multi-room route from spawn
send({ op: 'publish', topic: '/nav/goal',
  msg: { data: JSON.stringify({ goal_id: 'verify1', x: -3.0, y: 3.5, yaw: 3.14 }) } });

let last = '';
for (let i = 0; i < 90; i++) {
  await new Promise((r) => setTimeout(r, 1000));
  const gt = await page.evaluate(() => window.__sim.pose());
  const lp = locPose ? `loc=(${locPose.pose.position.x.toFixed(2)},${locPose.pose.position.y.toFixed(2)}) score=${locPose.covariance[0].toFixed(2)}` : 'loc=none';
  const line = `nav=${navStatus.state} dist=${navStatus.distance_remaining} ${lp} gt=(${gt.x.toFixed(2)},${gt.y.toFixed(2)})`;
  if (line !== last) console.log(`t=${i}s ${line}`);
  last = line;
  if (navStatus.state === 'succeeded' || navStatus.state === 'failed') break;
}
const gt = await page.evaluate(() => window.__sim.pose());
console.log(`FINAL: nav=${navStatus.state} gt=(${gt.x.toFixed(2)},${gt.y.toFixed(2)},${gt.yaw.toFixed(2)})`);
await browser.close();
ws.close();

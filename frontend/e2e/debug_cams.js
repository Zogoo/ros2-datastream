import fs from 'fs';
import { chromium } from '@playwright/test';
import WebSocket from 'ws';

const ws = new WebSocket('ws://localhost:9090');
await new Promise((r) => ws.once('open', r));
let front = null, rear = null;
ws.on('message', (raw) => {
  const m = JSON.parse(raw);
  if (m.op !== 'publish') return;
  if (m.topic === '/camera/front/image_raw/compressed') front = m.msg.data;
  if (m.topic === '/camera/rear/image_raw/compressed') rear = m.msg.data;
});
const send = (o) => ws.send(JSON.stringify(o));
send({ op: 'subscribe', topic: '/camera/front/image_raw/compressed', throttle_rate: 500 });
send({ op: 'subscribe', topic: '/camera/rear/image_raw/compressed', throttle_rate: 500 });

const browser = await chromium.launch({ args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader-webgl'] });
const page = await browser.newPage();
await page.goto('http://localhost:8080');
await page.waitForFunction(() => window.__sim !== undefined, null, { timeout: 60000 });
await page.locator('#conn-status').filter({ hasText: 'CONNECTED' }).waitFor({ timeout: 30000 });
await page.evaluate(() => {
  window.__sim.setPose(0, -3.6, Math.PI / 2);
  window.__sim.spawn('towel', 0.3, -2.2, 0.1);
});
await new Promise((r) => setTimeout(r, 6000));
const pose = await page.evaluate(() => window.__sim.pose());
console.log('robot pose:', JSON.stringify(pose));
if (front) fs.writeFileSync('/e2e/cam_front.jpg', Buffer.from(front, 'base64'));
if (rear) fs.writeFileSync('/e2e/cam_rear.jpg', Buffer.from(rear, 'base64'));
await page.screenshot({ path: '/e2e/scene.png' });
console.log('saved front:', !!front, 'rear:', !!rear);
await browser.close();
ws.close();

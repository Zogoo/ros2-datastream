import { chromium } from '@playwright/test';
import WebSocket from 'ws';

const ws = new WebSocket('ws://localhost:9090');
await new Promise((r) => ws.once('open', r));
let tracks = [];
let lastDet = null;
let camFrames = 0;
let depthFrames = 0;
ws.on('message', (raw) => {
  const m = JSON.parse(raw);
  if (m.op !== 'publish') return;
  if (m.topic === '/perception/towel_tracks') tracks = JSON.parse(m.msg.data).tracks;
  if (m.topic === '/detected_objects') lastDet = JSON.parse(m.msg.data).objects;
  if (m.topic === '/camera/front/image_raw/compressed') camFrames++;
  if (m.topic === '/camera/depth/image_raw') depthFrames++;
});
const send = (o) => ws.send(JSON.stringify(o));
send({ op: 'subscribe', topic: '/perception/towel_tracks', throttle_rate: 300 });
send({ op: 'subscribe', topic: '/detected_objects', throttle_rate: 300 });
send({ op: 'subscribe', topic: '/camera/front/image_raw/compressed', throttle_rate: 500 });
send({ op: 'subscribe', topic: '/camera/depth/image_raw', throttle_rate: 500 });

const browser = await chromium.launch({ args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader-webgl'] });
const page = await browser.newPage();
await page.goto('http://localhost:8080');
await page.waitForFunction(() => window.__sim !== undefined, null, { timeout: 60000 });
await page.locator('#conn-status').filter({ hasText: 'CONNECTED' }).waitFor({ timeout: 30000 });
await page.locator('[data-drive-mode="manual"]').click();  // mission must not interfere
send({ op: 'advertise', topic: '/arm/command', type: 'std_msgs/String' });
send({ op: 'publish', topic: '/arm/command', msg: { data: 'A HOME' } });
// wait for the DDS pipeline to warm up (rosbridge advertise -> discovery)
for (let i = 0; i < 60 && lastDet === null; i++) await new Promise((r) => setTimeout(r, 1000));
console.log('pipeline warm after', lastDet === null ? 'NEVER' : 'ok');

// stage: robot at spawn facing north; towels at known map positions ahead
await page.evaluate(() => {
  window.__sim.setPose(0, -3.6, Math.PI / 2);
  window.__sim.spawn('towel', 0.3, -2.2, 0.1);   // 1.4 m ahead, right
  window.__sim.spawn('towel', -0.4, -1.8, 0.1);  // 1.9 m ahead, left
});
await new Promise((r) => setTimeout(r, 12000));

console.log('ALL detections:', JSON.stringify((lastDet ?? []).map((d) => ({
  class: d.class, conf: d.confidence, bbox: d.bbox,
  est: d.estimated_position, refined: d.position_refined, range: d.range, source: d.source,
}))));
// dump the camera frame the detector sees
const frame = await page.evaluate(() => {
  const c = document.querySelector('#cam-pip');
  return c ? c.toDataURL('image/png') : null;
});
if (frame) {
  const fs = await import('fs');
  fs.writeFileSync('/e2e/last_frame.png', Buffer.from(frame.split(',')[1], 'base64'));
  console.log('frame saved');
}
console.log('tracks:', JSON.stringify(tracks));
console.log('camFrames:', camFrames, 'depthFrames:', depthFrames);
await browser.close();
ws.close();

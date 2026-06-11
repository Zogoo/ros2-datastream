import { chromium } from '@playwright/test';
import WebSocket from 'ws';

const ws = new WebSocket('ws://localhost:9090');
await new Promise((r) => ws.once('open', r));
const contacts = [];
const events = [];
ws.on('message', (raw) => {
  const m = JSON.parse(raw);
  if (m.op !== 'publish') return;
  if (m.topic === '/robot/contacts') contacts.push(JSON.parse(m.msg.data));
  if (m.topic === '/robot/events') events.push(JSON.parse(m.msg.data));
});
const send = (o) => ws.send(JSON.stringify(o));
send({ op: 'subscribe', topic: '/robot/contacts' });
send({ op: 'subscribe', topic: '/robot/events' });
send({ op: 'advertise', topic: '/safety/reset', type: 'std_msgs/Bool' });

const browser = await chromium.launch({ args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader-webgl'] });
const page = await browser.newPage();
await page.goto('http://localhost:8080');
await page.waitForFunction(() => window.__sim !== undefined, null, { timeout: 60000 });
await page.locator('#conn-status').filter({ hasText: 'CONNECTED' }).waitFor({ timeout: 30000 });
await new Promise((r) => setTimeout(r, 500));
send({ op: 'publish', topic: '/safety/reset', msg: { data: true } });
await new Promise((r) => setTimeout(r, 500));
await page.locator('[data-drive-mode="manual"]').click();

await page.evaluate(() => window.__sim.setPose(-2.6, 1.3, Math.PI));
const btn = page.locator('#dpad-fwd');
await btn.dispatchEvent('pointerdown');
let pose = { x: -2.6 };
for (let i = 0; i < 40 && pose.x > -4.05; i++) {
  await new Promise((r) => setTimeout(r, 200));
  pose = await page.evaluate(() => window.__sim.pose());
}
await btn.dispatchEvent('pointerup');
await new Promise((r) => setTimeout(r, 900));
const after = await page.evaluate(() => window.__sim.pose());
const es = await page.evaluate(() => window.__sim.safetyStop());
console.log(`x=${after.x.toFixed(2)} z=${after.z.toFixed(3)} estop=${es}`);
console.log('contacts>1:', JSON.stringify(contacts.filter((c) => c.impulse > 1)));
console.log('events:', JSON.stringify(events));
await browser.close();
ws.close();

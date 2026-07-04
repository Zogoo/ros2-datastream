// Verify the 360° bumper ring: drive into a wall (front), reverse into it
// (rear check via reversing), and print the contact parts + bearings.
import { chromium } from '@playwright/test';
import WebSocket from 'ws';

const contacts = [];
const ws = new WebSocket('ws://localhost:9090');
ws.on('open', () => ws.send(JSON.stringify({ op: 'subscribe', topic: '/robot/contacts', type: 'std_msgs/String' })));
ws.on('message', (raw) => {
  const m = JSON.parse(raw);
  if (m.op === 'publish') {
    const c = JSON.parse(m.msg.data);
    contacts.push(c);
    console.log(`CONTACT part=${c.part} bearing=${c.bearing_deg} obj=${c.object_id} impulse=${c.impulse}`);
  }
});

const run = async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage();
  await page.goto('http://localhost:8080');
  await page.waitForFunction(() => window.__sim, null, { timeout: 60000 });
  await page.waitForTimeout(2000);
  await page.locator('[data-drive-mode="manual"]').click();
  // face the east corridor wall head-on and drive into it
  await page.evaluate(() => window.__sim.setPose(0.2, 1.0, 0));
  await page.waitForTimeout(300);
  await page.locator('#dpad-fwd').dispatchEvent('pointerdown');
  await page.waitForTimeout(4000);
  await page.locator('#dpad-fwd').dispatchEvent('pointerup');
  console.log('--- now reversing into the west wall ---');
  await page.evaluate(() => window.__sim.setPose(-0.2, 1.0, 0));
  await page.waitForTimeout(300);
  await page.locator('#dpad-back').dispatchEvent('pointerdown');
  await page.waitForTimeout(4000);
  await page.locator('#dpad-back').dispatchEvent('pointerup');
  await browser.close();
  ws.close();
  process.exit(0);
};
run();

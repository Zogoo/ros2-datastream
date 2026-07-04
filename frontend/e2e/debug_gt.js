import { chromium } from '@playwright/test';
import WebSocket from 'ws';

const run = async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage();
  await page.goto('http://localhost:8080');
  await page.waitForFunction(() => window.__sim, null, { timeout: 60000 });
  await page.waitForTimeout(2000);
  await page.locator('[data-drive-mode="manual"]').click();
  await page.evaluate(() => {
    window.__sim.setPose(0, 1.9, Math.PI / 2);
    window.__sim.spawn('towel', 0.05, 2.9, 0.1);
  });
  await page.waitForTimeout(2500);
  const ws = new WebSocket('ws://localhost:9090');
  ws.on('open', () => ws.send(JSON.stringify({ op: 'subscribe', topic: '/ground_truth/objects', type: 'std_msgs/String' })));
  ws.on('message', async (raw) => {
    const m = JSON.parse(raw);
    if (m.op !== 'publish') return;
    const objs = JSON.parse(m.msg.data).objects.filter((o) => o.class === 'towel');
    for (const o of objs) {
      console.log(`${o.id} pos=(${o.position.x},${o.position.y},${o.position.z}) held=${o.held} binned=${o.binned} in_basket=${o.in_basket}`);
    }
    ws.close();
    await browser.close();
    process.exit(0);
  });
};
run();

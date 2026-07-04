// The wiggle regression scenario: an unmapped stool between robot and towel.
// Expect: block/bump -> dynamic-layer mark -> replan AROUND -> pick + stow.
import { chromium } from '@playwright/test';
import WebSocket from 'ws';

const ws = new WebSocket('ws://localhost:9090');
let lastState = '';
ws.on('open', () => {
  ws.send(JSON.stringify({ op: 'subscribe', topic: '/mission/state', type: 'std_msgs/String' }));
});
ws.on('message', (raw) => {
  const m = JSON.parse(raw);
  if (m.op !== 'publish') return;
  const s = JSON.parse(m.msg.data);
  const key = `${s.state}|${s.reason}`;
  if (key !== lastState) {
    lastState = key;
    console.log(`MISSION ${s.state} :: ${s.reason} (bin ${s.bin_kg}kg)`);
  }
});

const run = async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage();
  await page.goto('http://localhost:8080');
  await page.waitForFunction(() => window.__sim, null, { timeout: 60000 });
  await page.waitForTimeout(1500);
  await page.locator('[data-drive-mode="manual"]').click();
  await page.waitForTimeout(2500);
  await page.evaluate(() => {
    window.__sim.setPose(0, 0.4, Math.PI / 2);
    window.__sim.spawn('stool', 0.25, 1.5, 0.15);   // unmapped obstacle, off-center
    window.__sim.spawn('towel', -0.05, 2.7, 0.1);
  });
  await page.locator('[data-drive-mode="auto"]').click();
  await page.waitForTimeout(90000);
  const pose = await page.evaluate(() => window.__sim.pose());
  console.log(`FINAL pose x=${pose.x.toFixed(2)} y=${pose.y.toFixed(2)}`);
  await browser.close();
  ws.close();
  process.exit(0);
};
run();

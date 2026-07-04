import { chromium } from '@playwright/test';
const run = async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage();
  await page.goto('http://localhost:8080');
  await page.waitForFunction(() => window.__sim, null, { timeout: 60000 });
  await page.waitForTimeout(1500);
  await page.locator('[data-drive-mode="manual"]').click();
  await page.waitForTimeout(2500);
  const staged = await page.evaluate(() => {
    window.__sim.setPose(0, 1.9, Math.PI / 2);
    return window.__sim.spawn('towel', 0.05, 2.9, 0.1);
  });
  await page.locator('[data-drive-mode="auto"]').click();
  for (let t = 0; t < 60; t += 3) {
    await page.waitForTimeout(3000);
    const s = await page.evaluate((id) => {
      const p = window.__sim.pose();
      const t9 = window.__sim.objectState('towel_9');
      const st = window.__sim.objectState(id);
      return { p, t9, st };
    }, staged);
    console.log(`t=${t + 3}s robot=(${s.p.x.toFixed(2)},${s.p.y.toFixed(2)}) ` +
      `towel_9=(${s.t9.x.toFixed(2)},${s.t9.y.toFixed(2)}) held=${s.t9.held} ` +
      `staged=(${s.st.x.toFixed(2)},${s.st.y.toFixed(2)}) held=${s.st.held}`);
  }
  await browser.close();
  process.exit(0);
};
run();

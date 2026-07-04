import { chromium } from '@playwright/test';
const run = async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1400, height: 900 } });
  await page.goto('http://localhost:8080');
  await page.waitForFunction(() => window.__sim, null, { timeout: 60000 });
  await page.waitForTimeout(2500);
  await page.locator('[data-drive-mode="manual"]').click();
  await page.evaluate(() => window.__sim.setPose(0, 1.0, Math.PI / 2));
  await page.getByText('FOLLOW', { exact: true }).click();
  await page.waitForTimeout(1200);
  await page.screenshot({ path: '/tmp/robot_bumper.png' });
  await browser.close();
};
run();

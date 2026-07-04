import { chromium } from '@playwright/test';
const run = async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage();
  await page.goto('http://localhost:8080');
  await page.waitForFunction(() => window.__sim, null, { timeout: 60000 });
  await new Promise((r) => setTimeout(r, 25000));
  await browser.close();
  process.exit(0);
};
run();

import { chromium } from '@playwright/test';
const run = async () => {
  const browser = await chromium.launch();
  const a = await browser.newPage();
  await a.goto('http://localhost:8080');
  await a.waitForFunction(() => window.__sim, null, { timeout: 60000 });
  await a.waitForTimeout(3000);
  const b = await browser.newPage();
  await b.goto('http://localhost:8080');
  await b.waitForFunction(() => window.__sim, null, { timeout: 60000 });
  await b.waitForTimeout(5000);
  const aMuted = await a.evaluate(() => window.__sim.muted());
  const bMuted = await b.evaluate(() => window.__sim.muted());
  console.log(`older page muted: ${aMuted} (expect true); newer page muted: ${bMuted} (expect false)`);
  await browser.close();
  process.exit(0);
};
run();

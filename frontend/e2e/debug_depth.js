import { chromium } from '@playwright/test';
const browser = await chromium.launch({ args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader-webgl'] });
const page = await browser.newPage();
await page.goto('http://localhost:8080');
await page.waitForFunction(() => window.__sim !== undefined, null, { timeout: 60000 });
await new Promise((r) => setTimeout(r, 4000));
console.log(JSON.stringify(await page.evaluate(() => window.__sim.depthStats())));
await browser.close();

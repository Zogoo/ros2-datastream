import { chromium } from '@playwright/test';
const browser = await chromium.launch({ args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader-webgl'] });
const page = await browser.newPage();
await page.goto('http://localhost:8080');
await page.waitForFunction(() => window.__sim !== undefined, null, { timeout: 60000 });
await page.locator('#conn-status').filter({ hasText: 'CONNECTED' }).waitFor({ timeout: 30000 });
console.log('SIM_READY');
await new Promise((r) => setTimeout(r, parseInt(process.env.HOLD_S ?? '40') * 1000));
await browser.close();

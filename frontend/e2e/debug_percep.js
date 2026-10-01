// Stage scenario 10 (robot (0,-1) facing north, towel at (0.45,0.5)) in MANUAL
// and dump what perception reports: detections, tracks, mission state.
import { chromium } from '@playwright/test';
import WebSocket from 'ws';
const ws = new WebSocket('ws://localhost:9090');
await new Promise((r) => ws.once('open', r));
const last = {};
ws.on('message', (raw) => { const m = JSON.parse(raw); if (m.op === 'publish') last[m.topic] = m.msg; });
for (const t of ['/detected_objects', '/perception/towel_tracks', '/mission/state']) ws.send(JSON.stringify({ op: 'subscribe', topic: t }));
const browser = await chromium.launch({ args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader-webgl'] });
const page = await browser.newPage();
await page.goto('http://localhost:8080');
await page.waitForFunction(() => window.__sim !== undefined, null, { timeout: 60000 });
await page.locator('#conn-status').filter({ hasText: 'CONNECTED' }).waitFor({ timeout: 30000 });
await page.locator('[data-drive-mode="manual"]').click();
await new Promise((r) => setTimeout(r, 4000));
const state = process.argv[2] ?? 'folded';
await page.evaluate((st) => { window.__sim.setPose(0, -1.0, Math.PI / 2); window.__sim.objects.spawn('towel', [0.45, 0.5, 0.05], 'probe_towel', { state: st, wetness: 0.4 }); }, state);
for (let i = 0; i < 4; i++) {
  await new Promise((r) => setTimeout(r, 3000));
  const det = last['/detected_objects'] ? JSON.parse(last['/detected_objects'].data) : null;
  const objs = (det?.objects ?? det ?? []).map?.((o) => `${o.class}:${o.confidence?.toFixed?.(2)} h=${o.estimated_height} w=${o.estimated_width} r=${o.range} pos=${JSON.stringify(o.estimated_position)}`) ?? det;
  console.log(`t=${(i + 1) * 3}s det=${JSON.stringify(objs).slice(0, 500)}`);
  console.log(`   tracks=${(last['/perception/towel_tracks']?.data ?? '').slice(0, 300)}`);
}
await page.screenshot({ path: '/tmp/onsen_probe.png' }).catch(() => {});
await browser.close(); ws.close();

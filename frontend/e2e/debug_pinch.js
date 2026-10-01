// Controlled pick geometry probe: robot parked (MANUAL), a towel placed at a
// given offset from the canned PICK_SCOOP point, then the firmware pick
// sequence is run by hand. Prints fingertip vs towel and the grasp outcome.
//   node debug_pinch.js [state folded|crumpled]
import { chromium } from '@playwright/test';
import WebSocket from 'ws';

const state = process.argv[2] ?? 'folded';
const ws = new WebSocket('ws://localhost:9090');
await new Promise((r) => ws.once('open', r));
const send = (o) => ws.send(JSON.stringify(o));
send({ op: 'advertise', topic: '/arm/command', type: 'std_msgs/String' });
send({ op: 'advertise', topic: '/safety/reset', type: 'std_msgs/Bool' });
const arm = (cmd) => send({ op: 'publish', topic: '/arm/command', msg: { data: cmd } });
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

const browser = await chromium.launch({ args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader-webgl'] });
const page = await browser.newPage();
page.on('pageerror', (e) => console.log('PAGEERROR', e.message));
await page.goto('http://localhost:8080');
await page.waitForFunction(() => window.__sim !== undefined, null, { timeout: 60000 });
await page.locator('#conn-status').filter({ hasText: 'CONNECTED' }).waitFor({ timeout: 30000 });
send({ op: 'publish', topic: '/safety/reset', msg: { data: true } });
await page.locator('[data-drive-mode="manual"]').click();

for (const [dx, dy] of [[0, 0], [0.08, 0], [-0.08, 0], [0, 0.08], [0, 0.12]]) {
  arm('A HOME');
  await sleep(2500);
  const id = await page.evaluate(([dx, dy, st]) => {
    window.__sim.setPose(0, 0, 0);
    // canned scoop point is 0.757 m ahead of base centre
    const it = window.__sim.objects.spawn('towel', [0.757 + dx, dy, 0.05], null, { state: st, wetness: 0.4 });
    return it.id;
  }, [dx, dy, state]);
  await sleep(1500);
  for (const c of ['A PRE_PICK', 'A PICK_LOWER', 'A PICK_SCOOP']) { arm(c); await sleep(1800); }
  const before = await page.evaluate((oid) => ({ tip: window.__sim.armFingertip(), o: window.__sim.objectState(oid), pose: window.__sim.pose() }), id);
  arm('A PICK_GRIP');
  await sleep(1500);
  const grip = await page.evaluate(() => window.__sim.grip());
  arm('A PICK_LIFT');
  await sleep(2500);
  const after = await page.evaluate((oid) => ({ g: window.__sim.grip(), o: window.__sim.objectState(oid) }), id);
  const f = (v) => v.toFixed(3);
  console.log(`off=(${dx},${dy}) tip=(${f(before.tip.x)},${f(before.tip.y)},${f(before.tip.z)}) towel=(${f(before.o.x)},${f(before.o.y)},${f(before.o.z)}) robot=(${f(before.pose.x)},${f(before.pose.y)}) | grip held=${grip.held} jaw=${grip.jaw_width_m} cap=${grip.capacity_n?.toFixed(2)} | after lift held=${after.g.held} load=${after.g.load_n?.toFixed(2)} towel_z=${f(after.o.z)}`);
  arm('A OPEN_GRIPPER');
  await sleep(1000);
  await page.evaluate((oid) => {
    const o = window.__sim.objects;
    const it = o.items.find((i) => i.id === oid);
    if (it) it.body.setTranslation({ x: -5, y: -5, z: 0.1 }, true);
  }, id);
}
await browser.close();
ws.close();

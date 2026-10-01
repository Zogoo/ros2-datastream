// Trace a staged pick-and-deliver (scenario 04 staging): mission state, the
// gripper's grip load vs holding capacity, grasp events, and where the towel
// finally lands relative to the floor bin. Usage (stack running):
//   node debug_grip.js [x y]   (towel spawn, default 0.08 4.64)
// At every gripper close it also prints the jaw tip vs nearby towels, the
// mission's target and the raw detections; released towels are followed 2 s.
import { chromium } from '@playwright/test';
import WebSocket from 'ws';

const [tx, ty] = process.argv.length >= 4 ? [Number(process.argv[2]), Number(process.argv[3])] : [0.08, 4.64];
const ws = new WebSocket('ws://localhost:9090');
await new Promise((r) => ws.once('open', r));
let mission = {};
let lastDet = null;
const events = [];
ws.on('message', (raw) => {
  const m = JSON.parse(raw);
  if (m.op !== 'publish') return;
  if (m.topic === '/mission/state') mission = JSON.parse(m.msg.data);
  if (m.topic === '/robot/events') events.push(JSON.parse(m.msg.data));
  if (m.topic === '/detected_objects') lastDet = JSON.parse(m.msg.data);
});
const send = (o) => ws.send(JSON.stringify(o));
for (const t of ['/mission/state', '/detected_objects']) send({ op: 'subscribe', topic: t, throttle_rate: 200 });
send({ op: 'subscribe', topic: '/robot/events' });   // unthrottled: events must not be dropped
send({ op: 'advertise', topic: '/safety/reset', type: 'std_msgs/Bool' });

const browser = await chromium.launch({ args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader-webgl'] });
const page = await browser.newPage();
page.on('pageerror', (e) => console.log('PAGEERROR', e.message, e.stack?.split('\n').slice(0, 3).join(' | ')));
page.on('crash', () => console.log('PAGE CRASHED'));
page.on('console', (m) => { if (m.type() === 'error') console.log('CONSOLE', m.text().slice(0, 200)); });
browser.on('disconnected', () => console.log('BROWSER DISCONNECTED'));
await page.goto('http://localhost:8080');
await page.waitForFunction(() => window.__sim !== undefined, null, { timeout: 60000 });
await page.locator('#conn-status').filter({ hasText: 'CONNECTED' }).waitFor({ timeout: 30000 });
send({ op: 'publish', topic: '/safety/reset', msg: { data: true } });
await page.locator('[data-drive-mode="manual"]').click();
await new Promise((r) => setTimeout(r, 6000));
const id = await page.evaluate(([x, y]) => {
  window.__sim.setPose(0, 3.04, Math.PI / 2);
  return window.__sim.spawn('towel', x, y, 0.1);
}, [tx, ty]);
await page.locator('[data-drive-mode="auto"]').click();

let last = '';
let peak = 0;
const t0 = Date.now();
for (let i = 0; i < 1500; i++) {
  await new Promise((r) => setTimeout(r, 200));
  const s = await page.evaluate((oid) => ({ g: window.__sim.grip(), o: window.__sim.objectState(oid) }), id);
  if (s.g.load_n) peak = Math.max(peak, s.g.load_n);
  const line = `m=${mission.state} held=${s.g.held} jaw=${s.g.jaw_width_m}`;
  const evs = events.splice(0);
  const ev = evs.map((e) => e.event).join(',');
  for (const e of evs) {
    if (e.event === 'OBJECT_BINNED') console.log(`   >>> OBJECT_BINNED ${e.object_id} -> ${e.bin_id} correct=${e.correct}`);
    if (e.event === 'GRASP_RELEASED' || e.event === 'GRASP_SLIPPED') {
      const oid = e.object_id;
      setTimeout(async () => {
        try {
          const st = await page.evaluate((x) => window.__sim.objectState(x), oid);
          const p = await page.evaluate(() => window.__sim.pose());
          console.log(`   >>> ${e.event} ${oid} +2s at (${st.x.toFixed(2)},${st.y.toFixed(2)},${st.z.toFixed(2)}) binned=${st.binned} robot=(${p.x.toFixed(2)},${p.y.toFixed(2)},${(p.yaw * 57.3).toFixed(0)}deg) mission=${mission.state}`);
        } catch { /* page gone */ }
      }, 2000);
    }
  }
  if (line !== last || ev) {
    const cap = s.g.capacity_n ? ` load=${s.g.load_n?.toFixed(2)}/${s.g.capacity_n.toFixed(2)}N` : '';
    console.log(`t=${((Date.now() - t0) / 1000).toFixed(1)}s ${line}${cap} towel=(${s.o.x.toFixed(2)},${s.o.y.toFixed(2)},${s.o.z.toFixed(2)}) binned=${s.o.binned} ${ev ? `EV[${ev}]` : ''} "${(mission.reason || '').slice(0, 50)}"`);
  }
  last = line;
  if (/CLOSE_GRIPPER|PICK_GRIP/.test(mission.reason || '') && !s._logged) {
    const info = await page.evaluate(() => {
      const tip = window.__sim.armFingertip();
      const p = window.__sim.pose();
      const rel = (o) => { const dx = o.x - p.x; const dy = o.y - p.y; return [dx * Math.cos(p.yaw) + dy * Math.sin(p.yaw), -dx * Math.sin(p.yaw) + dy * Math.cos(p.yaw)].map((v) => v.toFixed(3)).join(','); };
      const towels = window.__sim.objects.items.filter((it) => it.cls === 'towel' && !it.held).map((it) => ({ id: it.id, st: it.state, q: it.body.translation(), r: it.body.rotation() }))
        .filter((t) => Math.hypot(t.q.x - tip.x, t.q.y - tip.y) < 0.6)
        .map((t) => `${t.id}(${t.st}) rel=(${rel(t.q)}) z=${t.q.z.toFixed(3)} yaw=${(2 * Math.atan2(t.r.z, t.r.w) * 57.3).toFixed(0)}`);
      return `robot=(${p.x.toFixed(2)},${p.y.toFixed(2)},${(p.yaw * 57.3).toFixed(0)}deg) tip rel=(${rel(tip)}) z=${tip.z.toFixed(3)} near: ${towels.join(' | ')}`;
    });
    const tgt = mission.target_id;
    const tp = tgt ? await page.evaluate((x) => window.__sim.objectState(x), tgt) : null;
    const dets = (lastDet?.objects ?? []).map((d) => { const q = d.position_refined || d.estimated_position; return `${d.class}${q ? `(${q.x.toFixed(2)},${q.y.toFixed(2)})` : ''}${d.source === 'depth' ? 'D' : 'G'} bb=${d.bbox} h=${d.estimated_height} w=${d.estimated_width}`; });
    console.log(`   CLOSE ${info}\n     target=${tgt} at ${tp ? `(${tp.x.toFixed(2)},${tp.y.toFixed(2)},${tp.z.toFixed(2)}) held=${tp.held} binned=${tp.binned}` : '-'}\n     dets=${dets.join(' ; ')}`);
  }
  if (s.o.binned) { console.log(`BINNED OK peak_load=${peak.toFixed(2)}N`); break; }
}
await browser.close();
ws.close();

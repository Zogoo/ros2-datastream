import { chromium } from '@playwright/test';
import WebSocket from 'ws';
const ws = new WebSocket('ws://localhost:9090');
await new Promise((r) => ws.once('open', r));
let mission={}, nav={}, tracks=[], loc=null;
ws.on('message', (raw) => {
  const m = JSON.parse(raw); if (m.op!=='publish') return;
  if (m.topic==='/mission/state') mission=JSON.parse(m.msg.data);
  if (m.topic==='/nav/status') nav=JSON.parse(m.msg.data);
  if (m.topic==='/perception/towel_tracks') tracks=JSON.parse(m.msg.data).tracks;
  if (m.topic==='/localization/pose') loc=m.msg.pose;
});
const send=(o)=>ws.send(JSON.stringify(o));
for (const t of ['/mission/state','/nav/status','/perception/towel_tracks','/localization/pose']) send({op:'subscribe',topic:t,throttle_rate:400});
const browser = await chromium.launch({ args:['--enable-unsafe-swiftshader','--use-angle=swiftshader-webgl'] });
const page = await browser.newPage();
await page.goto('http://localhost:8080');
await page.waitForFunction(()=>window.__sim!==undefined,null,{timeout:60000});
await page.locator('#conn-status').filter({hasText:'CONNECTED'}).waitFor({timeout:30000});
await new Promise(r=>setTimeout(r,8000));
await page.evaluate(()=>{ window.__sim.setPose(0,1.9,Math.PI/2); window.__sim.spawn('towel',0.05,2.9,0.1); });
await page.locator('[data-drive-mode="auto"]').click();
let last='';
for (let i=0;i<70;i++){
  await new Promise(r=>setTimeout(r,1000));
  const gt=await page.evaluate(()=>window.__sim.pose());
  const lc=loc?`loc=(${loc.pose.position.x.toFixed(2)},${loc.pose.position.y.toFixed(2)}) sc=${loc.covariance[0].toFixed(2)}`:'loc=?';
  const tk=tracks.map(t=>`${t.id}@(${t.position.x.toFixed(2)},${t.position.y.toFixed(2)})`).join(' ');
  const line=`m=${mission.state} nav=${nav.state} dist=${nav.distance_remaining} tgt=${mission.target_id} held=${mission.holding} ${lc} gt=(${gt.x.toFixed(2)},${gt.y.toFixed(2)}) tracks=[${tk}]`;
  if (line!==last) console.log(`t=${i}s ${line}`);
  last=line;
  if (mission.holding) { console.log('GRASPED'); break; }
}
await browser.close(); ws.close();

import { chromium } from '@playwright/test';
import WebSocket from 'ws';
const ws = new WebSocket('ws://localhost:9090');
await new Promise((r) => ws.once('open', r));
let mission={}, grasp=false, binned=false;
ws.on('message', (raw) => {
  const m = JSON.parse(raw); if (m.op!=='publish') return;
  if (m.topic==='/mission/state') mission=JSON.parse(m.msg.data);
  if (m.topic==='/robot/events') { const e=JSON.parse(m.msg.data);
    if (e.event==='GRASP_ACQUIRED') grasp=true; if (e.event==='OBJECT_BINNED') binned=true; }
});
const send=(o)=>ws.send(JSON.stringify(o));
for (const t of ['/mission/state','/robot/events']) send({op:'subscribe',topic:t,throttle_rate:300});
const browser = await chromium.launch({ args:['--enable-unsafe-swiftshader','--use-angle=swiftshader-webgl'] });
const page = await browser.newPage();
await page.goto('http://localhost:8080');
await page.waitForFunction(()=>window.__sim!==undefined,null,{timeout:60000});
await page.locator('#conn-status').filter({hasText:'CONNECTED'}).waitFor({timeout:30000});
await new Promise(r=>setTimeout(r,9000));  // localizer GT-seed + converge
await page.evaluate(()=>{ window.__sim.setPose(0,-1.0,Math.PI/2); window.__sim.spawn('towel',0.4,0.3,0.1); });
await page.locator('[data-drive-mode="auto"]').click();
let last='';
for (let i=0;i<90;i++){
  await new Promise(r=>setTimeout(r,1000));
  const held=await page.evaluate(()=>window.__sim.holding());
  const line=`m=${mission.state} held=${held} graspEvt=${grasp} binned=${binned}`;
  if(line!==last) console.log(`t=${i}s ${line} reason="${(mission.reason||'').slice(0,38)}"`);
  last=line;
  if(binned){console.log('BINNED OK');break;}
}
await browser.close(); ws.close();

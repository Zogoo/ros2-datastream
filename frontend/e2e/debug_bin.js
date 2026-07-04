import { chromium } from '@playwright/test';
import WebSocket from 'ws';
const ws = new WebSocket('ws://localhost:9090');
await new Promise((r) => ws.once('open', r));
let objects=[], events=[];
ws.on('message', (raw) => {
  const m = JSON.parse(raw); if (m.op!=='publish') return;
  if (m.topic==='/ground_truth/objects') objects=JSON.parse(m.msg.data).objects;
  if (m.topic==='/robot/events') events.push(JSON.parse(m.msg.data).event);
});
const send=(o)=>ws.send(JSON.stringify(o));
send({op:'subscribe',topic:'/ground_truth/objects',throttle_rate:500});
send({op:'subscribe',topic:'/robot/events',throttle_rate:200});
const browser = await chromium.launch({ args:['--enable-unsafe-swiftshader','--use-angle=swiftshader-webgl'] });
const page = await browser.newPage();
await page.goto('http://localhost:8080');
await page.waitForFunction(()=>window.__sim!==undefined,null,{timeout:60000});
await page.locator('#conn-status').filter({hasText:'CONNECTED'}).waitFor({timeout:30000});
await new Promise(r=>setTimeout(r,9000));
await page.evaluate(()=>{ window.__sim.setPose(0,2.5,Math.PI/2); window.__sim.spawn('towel',0.1,3.3,0.1); });
await page.locator('[data-drive-mode="auto"]').click();
// watch towel positions near the bin (0, 4.45) through a drop
for (let i=0;i<70;i++){
  await new Promise(r=>setTimeout(r,1000));
  const towels=objects.filter(o=>o.class==='towel');
  const nearBin=towels.map(t=>`${t.id}@(${t.position.x.toFixed(2)},${t.position.y.toFixed(2)},${t.position.z.toFixed(2)})held=${t.held}binned=${t.binned}`);
  if (events.includes('OBJECT_BINNED')) { console.log('BINNED EVENT FIRED'); break; }
  if (i%5===0 || nearBin.some(s=>s.includes('held=true'))) console.log(`t=${i}s towels=[${nearBin.join(' ')}]`);
}
console.log('events seen:', [...new Set(events)].join(','));
await browser.close(); ws.close();

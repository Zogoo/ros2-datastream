import { chromium } from '@playwright/test';
import WebSocket from 'ws';
const ws = new WebSocket('ws://localhost:9090');
await new Promise((r)=>ws.once('open',r));
let mission={}; const events=[];
ws.on('message',(raw)=>{const m=JSON.parse(raw);if(m.op!=='publish')return;
  if(m.topic==='/mission/state')mission=JSON.parse(m.msg.data);
  if(m.topic==='/robot/events'){const e=JSON.parse(m.msg.data).event;if(e&&e.startsWith('GRASP'))events.push(e);}});
for(const t of ['/mission/state','/robot/events'])ws.send(JSON.stringify({op:'subscribe',topic:t,throttle_rate:100}));
const b=await chromium.launch({args:['--enable-unsafe-swiftshader','--use-angle=swiftshader-webgl']});
const p=await b.newPage(); await p.goto('http://localhost:8080');
await p.waitForFunction(()=>window.__sim!==undefined,null,{timeout:60000});
await p.locator('#conn-status').filter({hasText:'CONNECTED'}).waitFor({timeout:30000});
await new Promise(r=>setTimeout(r,9000));
await p.evaluate(()=>{ window.__sim.setPose(0,1.9,Math.PI/2); window.__sim.spawn('towel',0.05,2.9,0.1); });
await p.locator('[data-drive-mode="auto"]').click();
let maxByState={};
for(let i=0;i<120;i++){
  await new Promise(r=>setTimeout(r,500));
  const d=await p.evaluate(()=>window.__sim.graspDist());
  if(d>=0){ const st=mission.state||'?'; maxByState[st]=Math.max(maxByState[st]||0,d); }
  if(events.includes('GRASP_LOST')){console.log('GRASP_LOST at state',mission.state,'dist',d.toFixed(3));break;}
  if(events.includes('OBJECT_BINNED')){console.log('BINNED');break;}
}
console.log('max grasp dist by state:',JSON.stringify(Object.fromEntries(Object.entries(maxByState).map(([k,v])=>[k,+v.toFixed(3)]))));
console.log('events:',events.join(','));
await b.close(); ws.close();

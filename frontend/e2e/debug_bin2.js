import { chromium } from '@playwright/test';
import WebSocket from 'ws';
const ws = new WebSocket('ws://localhost:9090');
await new Promise((r) => ws.once('open', r));
let objects=[]; const events=[];
ws.on('message', (raw) => { const m=JSON.parse(raw); if(m.op!=='publish')return;
  if(m.topic==='/ground_truth/objects') objects=JSON.parse(m.msg.data).objects;
  if(m.topic==='/robot/events') events.push(JSON.parse(m.msg.data).event); });
const send=(o)=>ws.send(JSON.stringify(o));
send({op:'subscribe',topic:'/ground_truth/objects',throttle_rate:400});
send({op:'subscribe',topic:'/robot/events',throttle_rate:100});
const b=await chromium.launch({args:['--enable-unsafe-swiftshader','--use-angle=swiftshader-webgl']});
const p=await b.newPage(); await p.goto('http://localhost:8080');
await p.waitForFunction(()=>window.__sim!==undefined,null,{timeout:60000});
await p.locator('#conn-status').filter({hasText:'CONNECTED'}).waitFor({timeout:30000});
await new Promise(r=>setTimeout(r,9000));
await p.evaluate(()=>{ window.__sim.setPose(0,3.0,Math.PI/2); window.__sim.spawn('towel',0.1,3.6,0.1); });
await p.locator('[data-drive-mode="auto"]').click();
let prevHeld=false;
for(let i=0;i<60;i++){
  await new Promise(r=>setTimeout(r,1000));
  const tw=objects.filter(o=>o.class==='towel');
  const held=tw.some(t=>t.held);
  // print every towel each second so we always get data
  const line=tw.map(t=>`${t.id}(${t.position.x.toFixed(2)},${t.position.y.toFixed(2)},${t.position.z.toFixed(2)})h${t.held?1:0}b${t.binned?1:0}`).join(' ');
  console.log(`t=${i} bin@(0,4.45) | ${line}`);
  if(prevHeld&&!held) console.log('  >>> RELEASED this tick');
  prevHeld=held;
  if(events.includes('OBJECT_BINNED')){console.log('OBJECT_BINNED FIRED');break;}
}
console.log('events:',[...new Set(events)].join(','));
await b.close(); ws.close();

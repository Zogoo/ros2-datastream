import { chromium } from '@playwright/test';
import WebSocket from 'ws';
const ws = new WebSocket('ws://localhost:9090');
await new Promise((r)=>ws.once('open',r));
const events=[];
ws.on('message',(raw)=>{const m=JSON.parse(raw);if(m.op!=='publish')return;
  if(m.topic==='/robot/events'){const e=JSON.parse(m.msg.data).event; if(e&&e.startsWith('GRASP'))events.push(e);}});
ws.send(JSON.stringify({op:'subscribe',topic:'/robot/events',throttle_rate:100}));
const b=await chromium.launch({args:['--enable-unsafe-swiftshader','--use-angle=swiftshader-webgl']});
const p=await b.newPage(); await p.goto('http://localhost:8080');
await p.waitForFunction(()=>window.__sim!==undefined,null,{timeout:60000});
await p.locator('#conn-status').filter({hasText:'CONNECTED'}).waitFor({timeout:30000});
await new Promise(r=>setTimeout(r,9000));
await p.evaluate(()=>{ window.__sim.setPose(0,-1.0,Math.PI/2); window.__sim.spawn('towel',0.3,0.0,0.1); });
await p.locator('[data-drive-mode="auto"]').click();
// wait for grasp
let grasped=false;
for(let i=0;i<60;i++){ await new Promise(r=>setTimeout(r,1000));
  if(await p.evaluate(()=>window.__sim.holding())){grasped=true;break;} }
console.log('grasped:',grasped,'after',events.join(','));
if(!grasped){await b.close();ws.close();process.exit(0);}
// now force the robot hard against a side wall while holding, manual override
await p.locator('[data-drive-mode="manual"]').click();
const id=(await p.evaluate(()=>{const ts=window.__sim; return null;}));
// drive into the east/west corridor wall repeatedly
for(let i=0;i<30;i++){
  ws.send(JSON.stringify({op:'advertise',topic:'/cmd_vel/ui',type:'geometry_msgs/Twist'}));
  ws.send(JSON.stringify({op:'publish',topic:'/cmd_vel/ui',msg:{linear:{x:0.6,y:0,z:0},angular:{x:0,y:0,z:1.2}}}));
  await new Promise(r=>setTimeout(r,200));
}
await new Promise(r=>setTimeout(r,1500));
const held=await p.evaluate(()=>window.__sim.holding());
const gt=await p.evaluate(()=>window.__sim.pose());
// fingertip vs towel: measure how far the towel is from the robot (should be ~arm reach if held, far if floated)
const fk=await p.evaluate(()=>{const a=window.__sim; return a.armFingertip?a.armFingertip():null;});
console.log('after wall-ram: holding=',held,'robot=(',gt.x.toFixed(2),gt.y.toFixed(2),')');
console.log('grasp events=',events.join(','));
await b.close(); ws.close();

import { chromium } from '@playwright/test';
import WebSocket from 'ws';
const ws = new WebSocket('ws://localhost:9090');
await new Promise((r)=>ws.once('open',r));
const events=[];
ws.on('message',(raw)=>{const m=JSON.parse(raw);if(m.op!=='publish')return;
  if(m.topic==='/robot/events'){const e=JSON.parse(m.msg.data).event; if(e.startsWith('GRASP'))events.push(e);}});
ws.send(JSON.stringify({op:'subscribe',topic:'/robot/events',throttle_rate:100}));
const b=await chromium.launch({args:['--enable-unsafe-swiftshader','--use-angle=swiftshader-webgl']});
const p=await b.newPage(); await p.goto('http://localhost:8080');
await p.waitForFunction(()=>window.__sim!==undefined,null,{timeout:60000});
await p.locator('#conn-status').filter({hasText:'CONNECTED'}).waitFor({timeout:30000});
await new Promise(r=>setTimeout(r,3000));
// stage: manual, grab a towel directly via arm, then drive into a wall
await p.locator('[data-drive-mode="manual"]').click();
// put robot in open corridor, spawn towel right at the gripper, close gripper
await p.evaluate(()=>{ window.__sim.setPose(0,-3.0,Math.PI/2); window.__sim.spawn('towel',0.0,-2.33,0.05); });
ws.send(JSON.stringify({op:'advertise',topic:'/arm/command',type:'std_msgs/String'}));
const arm=(c)=>ws.send(JSON.stringify({op:'publish',topic:'/arm/command',msg:{data:c}}));
arm('A PRE_PICK'); await new Promise(r=>setTimeout(r,1500));
arm('A PICK_LOWER'); await new Promise(r=>setTimeout(r,1500));
arm('A PICK_SCOOP'); await new Promise(r=>setTimeout(r,1500));
arm('A CLOSE_GRIPPER'); await new Promise(r=>setTimeout(r,1500));
arm('A PICK_LIFT'); await new Promise(r=>setTimeout(r,1500));
const heldAfterPick=await p.evaluate(()=>window.__sim.holding());
console.log('after pick: holding=',heldAfterPick,'events=',events.join(','));
// now drive the robot hard into the side wall while holding
for(let i=0;i<25;i++){
  ws.send(JSON.stringify({op:'advertise',topic:'/cmd_vel/ui',type:'geometry_msgs/Twist'}));
  ws.send(JSON.stringify({op:'publish',topic:'/cmd_vel/ui',msg:{linear:{x:0.6,y:0,z:0},angular:{x:0,y:0,z:0.9}}}));
  await new Promise(r=>setTimeout(r,200));
}
await new Promise(r=>setTimeout(r,1500));
const held=await p.evaluate(()=>window.__sim.holding());
const gt=await p.evaluate(()=>window.__sim.pose());
// find the towel: is it near the gripper (carried) or floating away?
const tw=await p.evaluate(()=>window.__sim.objectState('towel_1'));
console.log('after wall-ram: holding=',held,'robot=(',gt.x.toFixed(2),gt.y.toFixed(2),')');
console.log('towel pos=',tw?`(${tw.x.toFixed(2)},${tw.y.toFixed(2)},${tw.z.toFixed(2)}) held=${tw.held}`:'gone');
console.log('grasp events=',events.join(','));
await b.close(); ws.close();

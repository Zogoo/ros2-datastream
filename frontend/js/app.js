/**
 * Onsen Robot — 3D Live Frontend
 *
 * Phase 1 : ROS connection + publishers (/cmd_vel, /arm/action)
 * Phase 2 : Three.js closed environment + tank robot + 6-DOF arm
 * Phase 3 : Sensor overlays — LiDAR minimap + camera PiP
 * Phase 4 : Full input control — keyboard WASD, D-pad, arm buttons, joint sliders
 *
 * All inputs publish real ROS2 topics via rosbridge, so the robot responds
 * exactly as it would to a real driver or autonomy stack.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ─── Config ───────────────────────────────────────────────────────────────────
const ROS_WS   = `ws://${window.location.hostname}:9090`;
const ROOM     = { w: 5.0, d: 4.0, h: 3.0 };
const CMD_VEL_HZ  = 10;   // publish rate while control is active
const MAX_VX      = 0.35;  // m/s forward/back
const MAX_WZ      = 0.80;  // rad/s rotation

// ─── Palette ──────────────────────────────────────────────────────────────────
const C = {
  chassis: 0x2a2a2a, track: 0x1a1a1a, wheel: 0x383838,
  armLink: 0xe65c00, joint: 0xffaa00, gripper: 0xb8b8b8,
  sensor:  0x444444,
  floor:   0xc8b89a, wall: 0xd4c9b5, ceiling: 0xe2ddd4,
  bench:   0x8b6914, benchLeg: 0x6b4f10,
};

// ─── App state ────────────────────────────────────────────────────────────────
const state = {
  pose:       { x: 0, y: 0, yaw: 0 },
  joints:     {},
  scan:       null,
  armState:   { state: 'HOME', cycle_id: 0, success_probability: 1.0 },
  detections: [],
  taskPlan:   { next_action: 'SEARCH', reason: '' },
  connected:  false,
  viewMode:   'orbit',
  // Control
  cmdVx:      0.0,
  cmdWz:      0.0,
  manualMode: false,
};

// Keys currently held (movement)
const keysHeld = new Set();
// D-pad buttons currently pressed
const dpadActive = { fwd: false, back: false, left: false, right: false };

// ─── Three.js handles ─────────────────────────────────────────────────────────
let renderer, scene, threeCamera, controls;
let robotGroup;
let armJointGroups = [];   // [{group, axis}]
let gripperL, gripperR;
let leftWheelMeshes  = [];
let rightWheelMeshes = [];
let cameraMarker;

// ─── ROS handles ──────────────────────────────────────────────────────────────
let ros, pubCmdVel, pubArmAction;
let cmdVelTimer = null;

// ════════════════════════════════════════════════════════════════════════════════
// Bootstrap
// ════════════════════════════════════════════════════════════════════════════════

window.addEventListener('load', main);

function main() {
  setupRenderer();
  setupLights();
  createEnvironment();
  createRobot();
  buildJointSliders();
  setupLidarCanvas();
  connectROS();
  setupKeyboard();
  setupDpad();
  setupArmButtons();
  setupViewModeButtons();
  setupShutterButton();
  animate();
  clockTick();
}

// ════════════════════════════════════════════════════════════════════════════════
// Phase 2 — Three.js scene
// ════════════════════════════════════════════════════════════════════════════════

function setupRenderer() {
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.shadowMap.enabled  = true;
  renderer.shadowMap.type     = THREE.PCFSoftShadowMap;
  renderer.toneMapping        = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 0.85;

  const vf = document.getElementById('viewfinder');
  renderer.setSize(vf.clientWidth, vf.clientHeight);
  vf.insertBefore(renderer.domElement, vf.firstChild);

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x1a1510);
  scene.fog = new THREE.Fog(0xffeedd, 5, 18);

  threeCamera = new THREE.PerspectiveCamera(62, vf.clientWidth / vf.clientHeight, 0.05, 40);
  threeCamera.position.set(0, 2.8, 5.5);
  threeCamera.lookAt(0, 0.2, 0);

  controls = new OrbitControls(threeCamera, renderer.domElement);
  controls.target.set(0, 0.3, 0);
  controls.minDistance     = 0.5;
  controls.maxDistance     = 12;
  controls.maxPolarAngle   = Math.PI * 0.52;
  controls.enableDamping   = true;
  controls.dampingFactor   = 0.08;
  controls.update();

  window.addEventListener('resize', () => {
    threeCamera.aspect = vf.clientWidth / vf.clientHeight;
    threeCamera.updateProjectionMatrix();
    renderer.setSize(vf.clientWidth, vf.clientHeight);
  });
}

function setupLights() {
  scene.add(new THREE.AmbientLight(0xffe4b5, 0.55));
  const dir = new THREE.DirectionalLight(0xfff5dc, 0.9);
  dir.position.set(1, 3.5, 2);
  dir.castShadow = true;
  dir.shadow.mapSize.set(1024, 1024);
  dir.shadow.camera.near = 0.1; dir.shadow.camera.far = 20;
  dir.shadow.camera.left = dir.shadow.camera.bottom = -4;
  dir.shadow.camera.right = dir.shadow.camera.top   =  4;
  scene.add(dir);
  const fill = new THREE.PointLight(0xffaa44, 0.3, 8);
  fill.position.set(-1, 0.1, -1);
  scene.add(fill);
}

function makeTileTexture(bg, line, size = 256, tile = 40) {
  const c = document.createElement('canvas');
  c.width = c.height = size;
  const ctx = c.getContext('2d');
  ctx.fillStyle = bg;    ctx.fillRect(0, 0, size, size);
  ctx.strokeStyle = line; ctx.lineWidth = 1;
  for (let i = 0; i <= size; i += tile) {
    ctx.beginPath(); ctx.moveTo(i, 0);    ctx.lineTo(i, size); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, i);    ctx.lineTo(size, i); ctx.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  return t;
}

function createEnvironment() {
  const hw = ROOM.w / 2, hd = ROOM.d / 2;

  // Floor
  const floorTex = makeTileTexture('#c8b89a', '#b09070', 256, 38);
  floorTex.repeat.set(5, 4);
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(ROOM.w, ROOM.d),
    new THREE.MeshLambertMaterial({ map: floorTex })
  );
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  scene.add(floor);

  // Ceiling
  const ceil = new THREE.Mesh(
    new THREE.PlaneGeometry(ROOM.w, ROOM.d),
    new THREE.MeshLambertMaterial({ color: C.ceiling, side: THREE.BackSide })
  );
  ceil.rotation.x = Math.PI / 2;
  ceil.position.y = ROOM.h;
  scene.add(ceil);

  // Walls
  const wallTex = makeTileTexture('#d4c9b5', '#b8a890', 256, 48);
  wallTex.repeat.set(5, 3);
  const wallMat = new THREE.MeshLambertMaterial({ map: wallTex });
  [
    [ROOM.w, ROOM.h, 0,   ROOM.h/2, -hd,  0         ],
    [ROOM.w, ROOM.h, 0,   ROOM.h/2,  hd,  Math.PI   ],
    [ROOM.d, ROOM.h, -hw, ROOM.h/2,  0,   Math.PI/2 ],
    [ROOM.d, ROOM.h,  hw, ROOM.h/2,  0,  -Math.PI/2 ],
  ].forEach(([w, h, x, y, z, ry]) => {
    const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), wallMat);
    m.position.set(x, y, z); m.rotation.y = ry; m.receiveShadow = true;
    scene.add(m);
  });

  // Benches
  addBench(-hw + 0.22, 0,  1.8);
  addBench( hw - 0.22, 0, -1.8);
}

function addBench(x, z_offset, length) {
  const mat = new THREE.MeshLambertMaterial({ color: C.bench });
  const leg = new THREE.MeshLambertMaterial({ color: C.benchLeg });
  const top = new THREE.Mesh(new THREE.BoxGeometry(0.35, 0.06, Math.abs(length)), mat);
  top.position.set(x, 0.42, z_offset);
  top.castShadow = top.receiveShadow = true;
  scene.add(top);
  [[0.3], [-0.3]].forEach(([lz]) => {
    [-0.13, 0.13].forEach(lx => {
      const l = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.42, 0.06), leg);
      l.position.set(x + lx, 0.21, z_offset + lz); l.castShadow = true;
      scene.add(l);
    });
  });
}

function createRobot() {
  robotGroup = new THREE.Group();
  scene.add(robotGroup);

  const chassisMat = new THREE.MeshLambertMaterial({ color: C.chassis });

  // Chassis
  const chassis = new THREE.Mesh(new THREE.BoxGeometry(0.50, 0.12, 0.36), chassisMat);
  chassis.position.y = 0.12; chassis.castShadow = true;
  robotGroup.add(chassis);

  // Sensor tray
  const tray = new THREE.Mesh(new THREE.BoxGeometry(0.44, 0.025, 0.30),
    new THREE.MeshLambertMaterial({ color: C.sensor }));
  tray.position.set(0, 0.195, 0); robotGroup.add(tray);

  // LiDAR drum
  const drum = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 0.04, 12),
    new THREE.MeshLambertMaterial({ color: 0x222222 }));
  drum.position.set(0.05, 0.235, 0); robotGroup.add(drum);

  // Camera housing
  const cam = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.035, 0.03),
    new THREE.MeshLambertMaterial({ color: 0x111111 }));
  cam.position.set(0.255, 0.15, 0); robotGroup.add(cam);

  // Camera link marker (world-position proxy for FPS mode)
  cameraMarker = new THREE.Object3D();
  cameraMarker.position.set(0.27, 0.15, 0);
  robotGroup.add(cameraMarker);

  // Tracks
  createTrack(robotGroup, -1);
  createTrack(robotGroup,  1);

  // 6-DOF arm
  buildArm(robotGroup);
}

function createTrack(parent, side) {
  const zOff = side * 0.215;
  const belt = new THREE.Mesh(new THREE.BoxGeometry(0.54, 0.10, 0.085),
    new THREE.MeshLambertMaterial({ color: C.track }));
  belt.position.set(0, 0.07, zOff); belt.castShadow = true;
  parent.add(belt);

  const meshes = [];
  [-0.20, 0, 0.20].forEach(xOff => {
    const w = new THREE.Mesh(
      new THREE.CylinderGeometry(0.068, 0.068, 0.072, 10),
      new THREE.MeshLambertMaterial({ color: C.wheel })
    );
    w.rotation.x = Math.PI / 2;
    w.position.set(xOff, 0.068, zOff);
    w.castShadow = true;
    parent.add(w); meshes.push(w);
  });
  if (side < 0) leftWheelMeshes  = meshes;
  else          rightWheelMeshes = meshes;
}

// 6-DOF arm: shoulder_pan(Y) → lift(Z) → elbow(Z) → wrist_pitch(Z) → wrist_roll(X) → gripper
const ARM_SEGS = [
  { axis: 'y', len: 0.14, r: 0.024 },
  { axis: 'z', len: 0.20, r: 0.022 },
  { axis: 'z', len: 0.17, r: 0.019 },
  { axis: 'z', len: 0.11, r: 0.017 },
  { axis: 'x', len: 0.08, r: 0.015 },
  { axis: null },
];
const ARM_JOINT_NAMES = [
  'shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
  'wrist_pitch_joint','wrist_roll_joint','gripper_joint',
];

function buildArm(parent) {
  const linkMat    = new THREE.MeshLambertMaterial({ color: C.armLink });
  const jointMat   = new THREE.MeshLambertMaterial({ color: C.joint  });
  const gripperMat = new THREE.MeshLambertMaterial({ color: C.gripper });

  const armRoot = new THREE.Group();
  armRoot.position.set(0.22, 0.21, 0.08);
  parent.add(armRoot);

  let anchor = armRoot;
  armJointGroups = [];

  ARM_SEGS.forEach(seg => {
    const jg = new THREE.Group();
    anchor.add(jg);
    armJointGroups.push({ group: jg, axis: seg.axis });

    if (seg.axis !== null) {
      const cyl = new THREE.Mesh(
        new THREE.CylinderGeometry(seg.r * 0.9, seg.r, seg.len, 8), linkMat);
      cyl.position.y = seg.len / 2; cyl.castShadow = true; jg.add(cyl);

      const sph = new THREE.Mesh(new THREE.SphereGeometry(seg.r * 1.35, 8, 6), jointMat);
      jg.add(sph);

      const next = new THREE.Group(); next.position.y = seg.len;
      jg.add(next); anchor = next;
    } else {
      // Gripper fingers
      const fg = new THREE.BoxGeometry(0.012, 0.055, 0.012);
      gripperL = new THREE.Mesh(fg, gripperMat); gripperL.position.set(-0.022, 0.027, 0);
      gripperR = new THREE.Mesh(fg, gripperMat); gripperR.position.set( 0.022, 0.027, 0);
      jg.add(gripperL, gripperR);
    }
  });
}

// ════════════════════════════════════════════════════════════════════════════════
// Phase 1 — ROS connection + publishers
// ════════════════════════════════════════════════════════════════════════════════

function connectROS() {
  ros = new ROSLIB.Ros({ url: ROS_WS });

  ros.on('connection', () => {
    setConnStatus(true);
    pubCmdVel = new ROSLIB.Topic({
      ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist',
    });
    pubArmAction = new ROSLIB.Topic({
      ros, name: '/arm/action', messageType: 'std_msgs/String',
    });
    pubCmdVel.advertise();
    pubArmAction.advertise();
    subscribeAll();
  });
  ros.on('error', () => setConnStatus(false));
  ros.on('close', () => { setConnStatus(false); setTimeout(connectROS, 3000); });
}

function subscribeAll() {
  sub('/odom',                              'nav_msgs/Odometry',          200, onOdom);
  sub('/joint_states',                      'sensor_msgs/JointState',     100, onJointStates);
  sub('/scan',                              'sensor_msgs/LaserScan',      150, onScan);
  sub('/camera/front/image_raw/compressed', 'sensor_msgs/CompressedImage',200, onCompressedImage);
  sub('/arm/state',                         'std_msgs/String',            200, onArmState);
  sub('/detected_objects',                  'std_msgs/String',            300, onDetections);
  sub('/task_plan',                         'std_msgs/String',            300, onTaskPlan);
  sub('/robot/events',                      'std_msgs/String',            0,   onEvent);
  sub('/robot/control_mode',                'std_msgs/String',            200, onControlMode);
}

function sub(name, type, throttle, cb) {
  new ROSLIB.Topic({ ros, name, messageType: type, throttle_rate: throttle, queue_length: 1 })
    .subscribe(cb);
}

// ── ROS subscriptions ─────────────────────────────────────────────────────────

function onOdom(msg) {
  const p = msg.pose.pose, q = p.orientation;
  state.pose.x   = p.position.x;
  state.pose.y   = p.position.y;
  state.pose.yaw = Math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
  updatePoseHUD();
}

function onJointStates(msg) {
  msg.name.forEach((n, i) => state.joints[n] = msg.position[i]);
  applyJointsToRobot();
  syncSlidersFromJoints();
}

function onScan(msg)            { state.scan = msg.ranges; drawLidar(msg.ranges); }
function onCompressedImage(msg) {
  const img = document.getElementById('cam-feed');
  img.src = 'data:image/jpeg;base64,' + msg.data;
  img.classList.remove('no-signal');
}
function onArmState(msg)  { try { state.armState = JSON.parse(msg.data); updateArmHUD(); } catch(_){} }
function onDetections(msg){ try { state.detections = (JSON.parse(msg.data).objects||[]); updateDetectHUD(); } catch(_){} }
function onTaskPlan(msg)  {
  try {
    state.taskPlan = JSON.parse(msg.data);
    document.getElementById('task-action').textContent = state.taskPlan.next_action || '';
    document.getElementById('task-reason').textContent = state.taskPlan.reason       || '';
  } catch(_) {}
}
function onEvent(msg)     { try { showEventTicker(JSON.parse(msg.data).event); } catch(_){} }
function onControlMode(msg) {
  try {
    const d = JSON.parse(msg.data);
    state.manualMode = d.mode === 'manual';
    const badge = document.getElementById('ctrl-mode-badge');
    badge.textContent = state.manualMode ? 'MANUAL' : 'AUTO';
    badge.className   = state.manualMode ? 'badge-manual' : 'badge-auto';
  } catch(_) {}
}

// ── Publishing helpers ────────────────────────────────────────────────────────

let _pubCount = 0;
function _bumpPubCounter() {
  _pubCount++;
  const el = document.getElementById('pub-counter');
  if (el) el.textContent = `TX:${_pubCount}`;
}

function publishCmdVel(vx, wz) {
  if (!pubCmdVel) return;
  pubCmdVel.publish({ linear: { x: vx, y: 0, z: 0 }, angular: { x: 0, y: 0, z: wz } });
  _bumpPubCounter();
}

function publishArmAction(payload) {
  if (!pubArmAction) return;
  pubArmAction.publish({ data: JSON.stringify(payload) });
  _bumpPubCounter();
}

// ════════════════════════════════════════════════════════════════════════════════
// Phase 4 — Control input handling
// ════════════════════════════════════════════════════════════════════════════════

// ── Keyboard ─────────────────────────────────────────────────────────────────

function setupKeyboard() {
  const MOVE_KEYS = new Set(['w','s','a','d','arrowup','arrowdown','arrowleft','arrowright',' ']);

  document.addEventListener('keydown', e => {
    const k = e.key.toLowerCase();
    if (MOVE_KEYS.has(k)) {
      e.preventDefault();
      keysHeld.add(k);
      startCmdVelLoop();
      // Send immediately — don't wait for the first interval tick
      const { vx, wz } = resolveVelocity();
      publishCmdVel(vx, wz);
    }
    // Arm state shortcuts: 1-7
    const armKeys = ['1','2','3','4','5','6','7'];
    const armStates = ['HOME','SEARCH','APPROACH_OBJECT','LOWER_TO_TOWEL','GRIP','LIFT','DROP_TO_TRAY'];
    const idx = armKeys.indexOf(k);
    if (idx >= 0) triggerArmState(armStates[idx]);
  });

  document.addEventListener('keyup', e => {
    keysHeld.delete(e.key.toLowerCase());
    if (keysHeld.size === 0 && !anyDpadActive()) stopCmdVelLoop();
  });

  window.addEventListener('blur', () => { keysHeld.clear(); stopCmdVelLoop(); });
}

function resolveVelocity() {
  let vx = 0, wz = 0;
  if (keysHeld.has('w') || keysHeld.has('arrowup'))    vx += MAX_VX;
  if (keysHeld.has('s') || keysHeld.has('arrowdown'))  vx -= MAX_VX;
  if (keysHeld.has('a') || keysHeld.has('arrowleft'))  wz += MAX_WZ;
  if (keysHeld.has('d') || keysHeld.has('arrowright')) wz -= MAX_WZ;
  if (keysHeld.has(' ')) { vx = 0; wz = 0; }

  // D-pad contributions
  if (dpadActive.fwd)   vx += MAX_VX;
  if (dpadActive.back)  vx -= MAX_VX;
  if (dpadActive.left)  wz += MAX_WZ;
  if (dpadActive.right) wz -= MAX_WZ;

  return { vx: Math.max(-MAX_VX, Math.min(MAX_VX, vx)),
           wz: Math.max(-MAX_WZ, Math.min(MAX_WZ, wz)) };
}

function startCmdVelLoop() {
  if (cmdVelTimer) return;
  cmdVelTimer = setInterval(() => {
    const { vx, wz } = resolveVelocity();
    publishCmdVel(vx, wz);
    highlightDpadVisuals(vx, wz);
  }, 1000 / CMD_VEL_HZ);
}

function stopCmdVelLoop() {
  if (cmdVelTimer) { clearInterval(cmdVelTimer); cmdVelTimer = null; }
  publishCmdVel(0, 0);   // explicit stop
  highlightDpadVisuals(0, 0);
}

// ── D-pad ─────────────────────────────────────────────────────────────────────

function setupDpad() {
  const map = {
    'dpad-fwd':   { key: 'fwd',   vx:  MAX_VX, wz:  0       },
    'dpad-back':  { key: 'back',  vx: -MAX_VX, wz:  0       },
    'dpad-left':  { key: 'left',  vx:  0,      wz:  MAX_WZ  },
    'dpad-right': { key: 'right', vx:  0,      wz: -MAX_WZ  },
    'dpad-stop':  { key: null,    vx:  0,      wz:  0       },
  };

  Object.entries(map).forEach(([id, cfg]) => {
    const btn = document.getElementById(id);
    if (!btn) return;

    const press = () => {
      if (cfg.key) { dpadActive[cfg.key] = true; startCmdVelLoop(); }
      else { keysHeld.clear(); Object.keys(dpadActive).forEach(k => dpadActive[k] = false); stopCmdVelLoop(); }
      btn.classList.add('held');
    };
    const release = () => {
      if (cfg.key) dpadActive[cfg.key] = false;
      btn.classList.remove('held');
      if (!anyDpadActive() && keysHeld.size === 0) stopCmdVelLoop();
    };

    // Mouse
    btn.addEventListener('mousedown',  press);
    btn.addEventListener('mouseup',    release);
    btn.addEventListener('mouseleave', release);
    // Touch
    btn.addEventListener('touchstart', e => { e.preventDefault(); press();   }, { passive: false });
    btn.addEventListener('touchend',   e => { e.preventDefault(); release(); }, { passive: false });
  });
}

function anyDpadActive() {
  return Object.values(dpadActive).some(Boolean);
}

function highlightDpadVisuals(vx, wz) {
  document.getElementById('dpad-fwd')  ?.classList.toggle('held', vx >  0.01);
  document.getElementById('dpad-back') ?.classList.toggle('held', vx < -0.01);
  document.getElementById('dpad-left') ?.classList.toggle('held', wz >  0.01);
  document.getElementById('dpad-right')?.classList.toggle('held', wz < -0.01);
}

// ── Arm state buttons ─────────────────────────────────────────────────────────

function setupArmButtons() {
  document.querySelectorAll('.arm-btn').forEach(btn => {
    btn.addEventListener('click', () => triggerArmState(btn.dataset.state));
  });
}

function triggerArmState(stateName) {
  publishArmAction({ cmd: 'set_state', state: stateName });
  document.querySelectorAll('.arm-btn').forEach(b => {
    b.classList.toggle('active', b.dataset.state === stateName);
  });
}

// ── Joint sliders ─────────────────────────────────────────────────────────────

// Don't sync sliders from ROS for 2s after the user last touched one
let _lastSliderInteract = 0;

// min / max / default for each joint
const JOINT_DEFS = [
  { name: 'shoulder_pan_joint',   min: -1.5, max: 1.5,  step: 0.01 },
  { name: 'shoulder_lift_joint',  min: -0.3, max: 1.5,  step: 0.01 },
  { name: 'elbow_joint',          min: -0.3, max: 1.5,  step: 0.01 },
  { name: 'wrist_pitch_joint',    min: -1.5, max: 1.0,  step: 0.01 },
  { name: 'wrist_roll_joint',     min: -1.5, max: 1.5,  step: 0.01 },
  { name: 'gripper_joint',        min:  0.0, max: 1.0,  step: 0.01 },
];

function buildJointSliders() {
  const panel = document.getElementById('joint-panel');
  if (!panel) return;

  JOINT_DEFS.forEach(def => {
    const row = document.createElement('div');
    row.className = 'joint-row';

    const label = document.createElement('span');
    label.className   = 'joint-name';
    label.textContent = def.name.replace('_joint', '').replace(/_/g, ' ').toUpperCase();

    const slider = document.createElement('input');
    slider.type      = 'range';
    slider.className = 'joint-slider';
    slider.min   = def.min;
    slider.max   = def.max;
    slider.step  = def.step;
    slider.value = 0;
    slider.dataset.joint = def.name;

    const valDisplay = document.createElement('span');
    valDisplay.className = 'joint-val';
    valDisplay.textContent = '0.00';

    const resetBtn = document.createElement('button');
    resetBtn.className   = 'joint-reset';
    resetBtn.textContent = 'RST';
    resetBtn.title       = 'Clear override';

    // Publish on input — mark interaction time so sync backs off for 2s
    slider.addEventListener('input', () => {
      _lastSliderInteract = Date.now();
      const v = parseFloat(slider.value);
      valDisplay.textContent = v.toFixed(2);
      publishArmAction({ cmd: 'set_joint', joint: def.name, value: v });
    });

    resetBtn.addEventListener('click', () => {
      _lastSliderInteract = 0;   // allow immediate re-sync after reset
      publishArmAction({ cmd: 'clear' });
    });

    row.append(label, slider, valDisplay, resetBtn);
    panel.appendChild(row);
  });

  // Toggle panel visibility
  document.getElementById('joints-toggle')?.addEventListener('click', () => {
    const hidden = panel.classList.toggle('hidden');
    document.getElementById('joints-toggle').textContent = `JOINTS ${hidden ? '▾' : '▴'}`;
  });
}

// Sync slider positions from live /joint_states feedback
// Backs off for 2s after the user last touched a slider so overrides feel responsive
function syncSlidersFromJoints() {
  if (Date.now() - _lastSliderInteract < 2000) return;
  document.querySelectorAll('.joint-slider').forEach(slider => {
    const name = slider.dataset.joint;
    const val  = state.joints[name];
    if (val === undefined) return;
    if (!slider.matches(':active')) {
      slider.value = val;
      slider.nextElementSibling.textContent = val.toFixed(2);
    }
  });
}

// ════════════════════════════════════════════════════════════════════════════════
// Phase 3 — Sensor overlays
// ════════════════════════════════════════════════════════════════════════════════

let lidarCtx;
function setupLidarCanvas() {
  lidarCtx = document.getElementById('lidar-canvas').getContext('2d');
}

function drawLidar(ranges) {
  if (!lidarCtx || !ranges) return;
  const W = 160, H = 160, cx = W/2, cy = H/2;
  const maxR = 4.0, scale = (W/2 - 4) / maxR;

  lidarCtx.fillStyle = '#050f05';
  lidarCtx.fillRect(0, 0, W, H);

  lidarCtx.strokeStyle = '#0a280a';
  lidarCtx.fillStyle   = '#0d3d0d';
  lidarCtx.font        = '8px monospace';
  lidarCtx.lineWidth   = 1;
  [1, 2, 3, 4].forEach(r => {
    lidarCtx.beginPath(); lidarCtx.arc(cx, cy, r * scale, 0, Math.PI*2); lidarCtx.stroke();
    lidarCtx.fillText(`${r}m`, cx + r * scale + 2, cy - 2);
  });

  lidarCtx.beginPath();
  lidarCtx.moveTo(cx, 2); lidarCtx.lineTo(cx, H-2);
  lidarCtx.moveTo(2, cy); lidarCtx.lineTo(W-2, cy);
  lidarCtx.stroke();

  lidarCtx.fillStyle = '#00ff44';
  const n = ranges.length, angleMin = -Math.PI, inc = 2*Math.PI/n;
  for (let i = 0; i < n; i++) {
    const r = ranges[i];
    if (r < 0.05 || r >= 8.0) continue;
    const a = angleMin + i * inc;
    const px = cx + Math.cos(a) * Math.min(r, maxR) * scale;
    const py = cy - Math.sin(a) * Math.min(r, maxR) * scale;
    lidarCtx.fillRect(px-1, py-1, 2, 2);
  }

  lidarCtx.fillStyle = '#ff3333';
  lidarCtx.beginPath(); lidarCtx.arc(cx, cy, 4, 0, Math.PI*2); lidarCtx.fill();
  lidarCtx.strokeStyle = '#ff3333'; lidarCtx.lineWidth = 1.5;
  lidarCtx.beginPath(); lidarCtx.moveTo(cx, cy); lidarCtx.lineTo(cx + scale*0.35, cy); lidarCtx.stroke();
}

// ════════════════════════════════════════════════════════════════════════════════
// Robot pose + joints → Three.js
// ════════════════════════════════════════════════════════════════════════════════

function applyPoseToRobot() {
  if (!robotGroup) return;
  robotGroup.position.x = state.pose.x;
  robotGroup.position.z = -state.pose.y;
  robotGroup.rotation.y = -state.pose.yaw;
}

function applyJointsToRobot() {
  if (!armJointGroups.length) return;
  armJointGroups.forEach(({ group, axis }, i) => {
    const a = state.joints[ARM_JOINT_NAMES[i]] || 0;
    if (axis === 'y') group.rotation.y = a;
    else if (axis === 'z') group.rotation.z = a;
    else if (axis === 'x') group.rotation.x = a;
  });
  const grip = state.joints['gripper_joint'] || 0;
  if (gripperL) gripperL.position.x = -(0.022 + grip * 0.025);
  if (gripperR) gripperR.position.x =  (0.022 + grip * 0.025);

  const la = state.joints['left_wheel_joint']  || 0;
  const ra = state.joints['right_wheel_joint'] || 0;
  leftWheelMeshes.forEach(w  => w.rotation.z = la);
  rightWheelMeshes.forEach(w => w.rotation.z = ra);
}

// ════════════════════════════════════════════════════════════════════════════════
// View modes
// ════════════════════════════════════════════════════════════════════════════════

function setupViewModeButtons() {
  document.querySelectorAll('.mode-btn[data-mode]').forEach(btn => {
    btn.addEventListener('click', () => setViewMode(btn.dataset.mode));
  });
}

function setViewMode(mode) {
  state.viewMode     = mode;
  controls.enabled   = (mode === 'orbit');
  document.querySelectorAll('.mode-btn[data-mode]').forEach(b =>
    b.classList.toggle('active', b.dataset.mode === mode));
}

function updateCameraForMode() {
  if (!robotGroup) return;
  const rp = robotGroup.position, ry = robotGroup.rotation.y;

  if (state.viewMode === 'orbit') {
    controls.target.lerp(new THREE.Vector3(rp.x, 0.3, rp.z), 0.04);
    controls.update();
  } else if (state.viewMode === 'follow') {
    controls.enabled = false;
    const off = new THREE.Vector3(-2.2, 1.4, 0).applyEuler(new THREE.Euler(0, ry, 0));
    threeCamera.position.lerp(rp.clone().add(off), 0.08);
    threeCamera.lookAt(rp.x, rp.y + 0.25, rp.z);
  } else if (state.viewMode === 'fps') {
    controls.enabled = false;
    if (cameraMarker) {
      const wp = new THREE.Vector3();
      cameraMarker.getWorldPosition(wp);
      threeCamera.position.lerp(wp, 0.12);
      const fwd = new THREE.Vector3(1,0,0).applyEuler(new THREE.Euler(0, ry, 0));
      threeCamera.lookAt(wp.clone().add(fwd));
    }
  }
}

// ════════════════════════════════════════════════════════════════════════════════
// HUD helpers
// ════════════════════════════════════════════════════════════════════════════════

function updatePoseHUD() {
  document.getElementById('px').textContent   = state.pose.x.toFixed(2);
  document.getElementById('py').textContent   = state.pose.y.toFixed(2);
  document.getElementById('pyaw').textContent = (state.pose.yaw * 180/Math.PI).toFixed(1) + '°';
}

function updateArmHUD() {
  const a = state.armState;
  document.getElementById('arm-state-val').textContent = a.state || 'HOME';
  document.getElementById('arm-cycle').textContent     = `CYCLE: ${a.cycle_id ?? 0}`;
  document.getElementById('arm-prob').textContent      = `PROB: ${(a.success_probability ?? 1).toFixed(2)}`;
  // Sync active arm button
  document.querySelectorAll('.arm-btn').forEach(b =>
    b.classList.toggle('active', b.dataset.state === a.state));
}

function updateDetectHUD() {
  const objs = state.detections;
  document.getElementById('detect-count').textContent = `${objs.length} object${objs.length !== 1 ? 's' : ''}`;
  const top4 = [...objs].sort((a,b) => b.confidence - a.confidence).slice(0, 4);
  document.getElementById('detect-list').innerHTML =
    top4.map(o => `${o.class} <span style="color:#00e676">${(o.confidence*100).toFixed(0)}%</span>`).join('<br>');
}

let tickerTimer;
function showEventTicker(text) {
  const el = document.getElementById('event-ticker');
  el.textContent = text; el.classList.add('show');
  clearTimeout(tickerTimer);
  tickerTimer = setTimeout(() => el.classList.remove('show'), 3500);
}

function setConnStatus(ok) {
  const el = document.getElementById('conn-status');
  el.textContent  = ok ? '● CONNECTED' : '● DISCONNECTED';
  el.className    = ok ? 'ok' : 'err';
  state.connected = ok;
}

// ════════════════════════════════════════════════════════════════════════════════
// Shutter / recording toggle
// ════════════════════════════════════════════════════════════════════════════════

function setupShutterButton() {
  document.getElementById('shutter-btn')?.addEventListener('click', () => {
    const btn = document.getElementById('shutter-btn');
    btn.classList.toggle('active');
    const rec = document.getElementById('rec-indicator');
    const on  = btn.classList.contains('active');
    rec.textContent = on ? '◉ REC' : '◉ LIVE';
    rec.classList.toggle('active', on);
  });
}

// ════════════════════════════════════════════════════════════════════════════════
// Animation loop
// ════════════════════════════════════════════════════════════════════════════════

function animate() {
  requestAnimationFrame(animate);
  applyPoseToRobot();
  applyJointsToRobot();   // run every frame so arm is smooth at 60fps
  updateCameraForMode();
  renderer.render(scene, threeCamera);
}

function clockTick() {
  const el = document.getElementById('clock');
  (function tick() {
    el.textContent = new Date().toTimeString().slice(0, 8);
    setTimeout(tick, 1000);
  })();
}

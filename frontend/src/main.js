import * as THREE from 'three';
import '../css/style.css';
import { config } from './config.js';
import { SimClock } from './core/clock.js';
import { createRng } from './core/rng.js';
import { RosBridge } from './ros/bridge.js';
import { TOPICS } from './ros/topics.js';
import { PhysicsWorld } from './physics/world.js';
import { OnsenWorld } from './env/layout.js';
import { ObjectManager } from './env/objects.js';
import { Steam } from './env/steam.js';
import { Robot, yawQuat } from './robot/robot.js';
import { LidarSensor } from './sensors/lidar.js';
import { RgbCamera } from './sensors/rgbCamera.js';
import { DepthCamera } from './sensors/depthCamera.js';
import { SonarSensor } from './sensors/sonar.js';
import { ImuSensor } from './sensors/imu.js';
import { OdomSensor } from './sensors/odom.js';
import { JointStateSensor } from './sensors/jointStates.js';
import { ContactSensor } from './sensors/contacts.js';
import { GroundTruthPublisher } from './sensors/groundTruth.js';
import { Controls } from './ui/controls.js';
import { Views } from './ui/views.js';
import { Hud } from './ui/hud.js';
import { ArmConsole } from './ui/armConsole.js';
import { SkinManager } from './ui/skins.js';
import { Scenario } from './ui/scenario.js';

async function boot() {
  const spec = config.robot;
  const rng = createRng(spec.rng_seed);
  const clock = new SimClock(config.physicsHz);

  // ── Rendering ─────────────────────────────────────────────────────────────
  const viewfinder = document.getElementById('viewfinder');
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.shadowMap.enabled = true;
  renderer.domElement.classList.add('webgl');
  viewfinder.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x14161a);
  scene.fog = new THREE.Fog(0x14161a, 14, 30);

  const hemi = new THREE.HemisphereLight(0xfff4e0, 0x33302a, 0.85);
  hemi.position.set(0, 0, 1);
  scene.add(hemi);
  const sun = new THREE.DirectionalLight(0xffe8c0, 1.4);
  sun.position.set(6, -4, 9);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.left = -10;
  sun.shadow.camera.right = 10;
  sun.shadow.camera.top = 10;
  sun.shadow.camera.bottom = -10;
  scene.add(sun);

  const camera = new THREE.PerspectiveCamera(55, 1, 0.05, 60);
  camera.up.set(0, 0, 1);

  const resize = () => {
    const { clientWidth: w, clientHeight: h } = viewfinder;
    renderer.setSize(w, h);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  };
  window.addEventListener('resize', resize);

  // ── Physics + world + robot ───────────────────────────────────────────────
  const physics = await PhysicsWorld.create(config.physicsHz);
  const world = new OnsenWorld(config.layout, config.objects, physics, scene);
  const objects = new ObjectManager(config.layout, config.objects, physics, scene, world);
  const steam = new Steam(world.steamZones, scene);
  const robot = new Robot(physics, scene, spec, config.layout.robot_spawn, objects);

  // ── ROS ───────────────────────────────────────────────────────────────────
  const ros = new RosBridge(config.rosUrl);

  const lidar = new LidarSensor(spec, physics, robot, world, rng, ros, clock);
  const camFront = new RgbCamera('camera_front', spec, config.realism, renderer, scene, robot, world, rng, ros, clock);
  const camRear = new RgbCamera('camera_rear', spec, config.realism, renderer, scene, robot, world, rng, ros, clock);
  const camDepth = new DepthCamera(spec, renderer, scene, robot, rng, ros, clock);
  const sonar = new SonarSensor(spec, physics, robot, rng, ros, clock);
  const imu = new ImuSensor(spec, robot, rng, ros, clock);
  const odom = new OdomSensor(spec, robot, ros, clock);
  const joints = new JointStateSensor(spec, robot, ros, clock);
  const contacts = new ContactSensor(spec, physics, robot, world, ros, clock);
  const groundTruth = new GroundTruthPublisher(objects, robot, ros, clock);

  ros.subscribeJson(TOPICS.armJointTargets, (m) => {
    if (Array.isArray(m.deg)) robot.arm.setTargets(m.deg);
  });
  ros.subscribeJson(TOPICS.baseWheelTargets, (m) => {
    if (Array.isArray(m.w)) robot.suspension.setTargets(m.w);
  });
  ros.subscribe(TOPICS.safetyStop, (m) => {
    robot.safetyStop = !!m.data;
  });
  ros.connect();

  // ── UI ────────────────────────────────────────────────────────────────────
  const controls = new Controls(ros, () => scenario.throwTowel());
  const views = new Views(camera, renderer, robot);
  const hud = new Hud(ros, robot, lidar, camFront, odom, objects);
  new ArmConsole(ros);
  new SkinManager(objects, hud);
  const scenario = new Scenario(objects, robot, rng, ros, camera, renderer, camFront, hud);
  resize();

  // ── Main loop ─────────────────────────────────────────────────────────────
  let fps = 60;
  let lastFrame = performance.now();
  const dt = 1 / config.physicsHz;

  function frame(now) {
    const frameDelta = now - lastFrame;
    lastFrame = now;
    fps = fps * 0.95 + (1000 / Math.max(frameDelta, 1)) * 0.05;

    const steps = clock.advance(now);
    for (let s = 0; s < steps; s++) {
      robot.update(dt, (x, y) => world.isWetAt(x, y));
      objects.update(dt);
      physics.step();
      contacts.update();
      lidar.update();
      imu.update(dt);
      odom.update(dt);
      joints.update(dt);
      sonar.update(dt);
    }
    const renderDt = frameDelta / 1000;
    steam.update(renderDt);
    camFront.update(renderDt);
    camRear.update(renderDt);
    camDepth.update(renderDt, (cam, depthSpec) => {
      const p = robot.worldPoint(depthSpec.position);
      cam.position.set(p.x, p.y, p.z);
      const pitch = (depthSpec.pitch_deg * Math.PI) / 180;
      const fwd = robot.worldDir([Math.cos(pitch), 0, Math.sin(pitch)]);
      cam.lookAt(p.x + fwd.x, p.y + fwd.y, p.z + fwd.z);
    });
    groundTruth.update(renderDt, fps);
    controls.update(renderDt);

    for (const event of objects.drainBinnedEvents()) {
      ros.publish(TOPICS.events, {
        data: JSON.stringify({ ...event, timestamp: new Date().toISOString() }),
      });
      hud.ticker(`${event.event} ${event.object_class} -> ${event.bin_id}${event.correct ? '' : ' (WRONG BIN)'}`);
    }

    views.update(spec.sensors.camera_front);
    renderer.render(scene, camera);
    hud.update(renderDt, fps);
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);

  // E2E hook: lets the Playwright suite stage deterministic scenarios.
  // Physics stays fully live — this only teleports/spawns, never fakes data.
  window.__sim = {
    setPose(x, y, yaw = 0) {
      robot.body.setTranslation({ x, y, z: 0.16 }, true);
      robot.body.setRotation(yawQuat(yaw), true);
      robot.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      robot.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    },
    setVelocity(vx, vy) {
      robot.body.setLinvel({ x: vx, y: vy, z: 0 }, true);
    },
    spawn(cls, x, y, z = 0.3) {
      return objects.spawn(cls, [x, y, z]).id;
    },
    pose: () => robot.pose(),
    objectState: (id) => {
      const item = objects.items.find((i) => i.id === id);
      if (!item) return null;
      const p = item.body.translation();
      return { x: p.x, y: p.y, z: p.z, held: item.held, binned: item.binned };
    },
    holding: () => robot.arm.holding(),
    safetyStop: () => robot.safetyStop,
    fps: () => fps,
    world,
    objects,
  };
}

boot();

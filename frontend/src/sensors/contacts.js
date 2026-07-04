import { TOPICS } from '../ros/topics.js';

/** Contact reporter: drains Rapier contact-force events on robot colliders and
 *  publishes named-part contact JSON. Also raises the critical water-entry
 *  event when the chassis dips into a pool. */
export class ContactSensor {
  constructor(spec, physics, robot, world, ros, clock) {
    this.spec = spec;
    this.physics = physics;
    this.robot = robot;
    this.world = world;
    this.ros = ros;
    this.clock = clock;
    this.waterLatched = false;
    this.lastPublishByKey = new Map();
  }

  update() {
    this.physics.eventQueue.drainContactForceEvents((event) => {
      const meta1 = this.physics.metaOf(event.collider1());
      const meta2 = this.physics.metaOf(event.collider2());
      const robotMeta = meta1?.kind === 'robot' ? meta1 : meta2?.kind === 'robot' ? meta2 : null;
      if (!robotMeta) return;
      const otherMeta = robotMeta === meta1 ? meta2 : meta1;
      if (!otherMeta || otherMeta.kind === 'floor' || otherMeta.kind === 'platform') return;

      const force = event.totalForceMagnitude();
      const impulse = force / 60;
      // 360° bumper ring (Roomba-style): resolve which skirt sector was hit
      // from the contact force direction. The world pushes the robot AWAY
      // from the obstacle, so the obstacle bears opposite the force the
      // robot receives. Rapier reports the force on collider1 — flip when
      // the robot is collider2.
      let bearingDeg = null;
      const f = event.totalForce();
      const mag = Math.hypot(f.x, f.y);
      if (mag > 1e-6) {
        const sign = robotMeta === meta1 ? 1 : -1;
        const obsX = -sign * f.x / mag;
        const obsY = -sign * f.y / mag;
        const yaw = this.robot.pose().yaw;
        bearingDeg = (Math.atan2(obsY, obsX) - yaw) * (180 / Math.PI);
        bearingDeg = ((bearingDeg + 540) % 360) - 180;  // wrap to (-180, 180]
      }
      const part = this._partFor(robotMeta, otherMeta, bearingDeg);
      if (part.startsWith('bumper_')) this.robot.flashBumper?.(part.slice(7));
      this._publishContact({
        part,
        bearing_deg: bearingDeg === null ? null : Math.round(bearingDeg),
        impulse: Math.round(impulse * 1000) / 1000,
        force: Math.round(force * 100) / 100,
        object_kind: otherMeta.kind,
        object_id: otherMeta.id ?? null,
        object_class: otherMeta.cls ?? otherMeta.type ?? null,
        critical: false,
      });
    });

    const pose = this.robot.pose();
    const pool = this.world.poolAt(pose.x, pose.y);
    const inWater = pool !== null && pose.z < pool.water_z + 0.05;
    if (inWater && !this.waterLatched) {
      this.waterLatched = true;
      this._publishContact({
        part: 'chassis', impulse: 99, force: 999,
        object_kind: 'water', object_id: pool.id, object_class: 'water', critical: true,
      });
    } else if (!inWater) {
      this.waterLatched = false;
    }
  }

  _partFor(robotMeta, otherMeta, bearingDeg = null) {
    // The bumper ring is the outermost shell, so it takes hits first; chassis
    // contacts (something striking above the ring band) resolve to the same
    // sector naming so consumers see one 360° bumper.
    if (robotMeta.part === 'bumper' || robotMeta.part === 'chassis') {
      if (!otherMeta?.id) return 'chassis';
      return `bumper_${bumperSector(bearingDeg)}`;
    }
    return robotMeta.part;
  }

  _publishContact(payload) {
    const key = `${payload.part}:${payload.object_id}`;
    const now = performance.now();
    const last = this.lastPublishByKey.get(key) ?? 0;
    if (now - last < 150 && !payload.critical) return;
    this.lastPublishByKey.set(key, now);

    const pose = this.robot.pose();
    this.ros.publish(TOPICS.contacts, {
      data: JSON.stringify({
        ...payload,
        robot_pose: { x: round3(pose.x), y: round3(pose.y), yaw: round3(pose.yaw) },
        timestamp: new Date().toISOString(),
      }),
    });
  }
}

const round3 = (v) => Math.round(v * 1000) / 1000;

/** Roomba-style bumper ring: map a body-frame obstacle bearing (deg, 0 =
 *  straight ahead, +left) onto one of 8 skirt sectors. Null bearing (force
 *  too small to resolve) reports as front — the conservative escape. */
export function bumperSector(bearingDeg) {
  if (bearingDeg === null || !Number.isFinite(bearingDeg)) return 'front';
  const sectors = [
    'front', 'front_left', 'left', 'rear_left',
    'rear', 'rear_right', 'right', 'front_right',
  ];
  const idx = Math.round(((bearingDeg + 360) % 360) / 45) % 8;
  return sectors[idx];
}

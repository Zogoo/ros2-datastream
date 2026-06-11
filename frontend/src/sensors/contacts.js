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
      this._publishContact({
        part: this._partFor(robotMeta, otherMeta),
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

  _partFor(robotMeta, otherMeta) {
    if (robotMeta.part !== 'chassis') return robotMeta.part;
    if (!otherMeta?.id) return 'chassis';
    return `chassis_${this._sideOf(otherMeta)}`;
  }

  _sideOf(_otherMeta) {
    return 'body';
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

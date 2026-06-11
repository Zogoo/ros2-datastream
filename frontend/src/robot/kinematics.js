/** Pure forward kinematics for the 6-axis arm, in base_link frame (X fwd, Z up).
 *
 * Firmware convention (degrees 0..180, 90 = neutral):
 *   J0 pan: 90 = forward, >90 = left   (about Z)
 *   J1 shoulder, J2 elbow, J3 wrist pitch: servo-to-joint ratio `joint_ratio`
 *   J4 wrist roll (visual only), J5 gripper (servo angle -> finger opening)
 *
 * Segment tilt angles are measured from vertical-up, positive pitching forward:
 *   t1 = (d1 - 90) * ratio
 *   t2 = t1 + (90 - d2) * ratio
 *   t3 = t2 + (90 - d3) * ratio
 */
export function armFk(degrees, armSpec) {
  const ratio = armSpec.joint_ratio;
  const L = armSpec.links;
  const base = armSpec.base_offset;
  const shoulderZ = armSpec.shoulder_z;
  const rad = Math.PI / 180;

  const pan = (degrees[0] - 90) * rad;
  const t1 = (degrees[1] - 90) * ratio * rad;
  const t2 = t1 + (90 - degrees[2]) * ratio * rad;
  const t3 = t2 + (90 - degrees[3]) * ratio * rad;

  const dirX = Math.cos(pan);
  const dirY = Math.sin(pan);

  const shoulder = [base[0], base[1], shoulderZ];
  const seg = (from, len, tilt) => [
    from[0] + len * Math.sin(tilt) * dirX,
    from[1] + len * Math.sin(tilt) * dirY,
    from[2] + len * Math.cos(tilt),
  ];
  const elbow = seg(shoulder, L.upper, t1);
  const wrist = seg(elbow, L.forearm, t2);
  const fingertip = seg(wrist, L.wrist, t3);

  return { shoulder, elbow, wrist, fingertip, tilts: [t1, t2, t3], pan };
}

export function gripperOpening(deg, armSpec) {
  const clamped = Math.max(0, Math.min(180, deg));
  return (clamped / 180) * armSpec.gripper.max_opening_m;
}

export function isGripperClosed(deg, armSpec) {
  return deg <= armSpec.gripper.close_deg_threshold;
}

/** First-order servo lag toward target with speed limit. Returns new position. */
export function servoStep(current, target, dt, tauS, maxSpeedDps) {
  const alpha = 1 - Math.exp(-dt / tauS);
  let next = current + (target - current) * alpha;
  const maxDelta = maxSpeedDps * dt;
  const delta = next - current;
  if (Math.abs(delta) > maxDelta) next = current + Math.sign(delta) * maxDelta;
  return next;
}

/** Skid-steer odometry integration from per-side wheel surface speeds. */
export function integrateOdometry(pose, vLeft, vRight, trackWidth, dt) {
  const v = (vLeft + vRight) / 2;
  const w = (vRight - vLeft) / trackWidth;
  const yaw = pose.yaw + w * dt;
  return {
    x: pose.x + v * Math.cos(yaw) * dt,
    y: pose.y + v * Math.sin(yaw) * dt,
    yaw,
    v,
    w,
  };
}

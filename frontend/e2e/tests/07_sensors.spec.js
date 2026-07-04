import { expect, test } from '@playwright/test';
import { RosProbe, bootSim, setManual, sleep } from '../helpers/ros.js';

// Validates the two-tier perception claim: walls in /scan, low obstacles
// (bath rims) only in the depth channel — plus basic stream health.
test('sensors: lidar/cameras/depth/sonar/imu publish at healthy rates', async ({ page }) => {
  test.setTimeout(180_000);
  const probe = new RosProbe();
  await probe.connect();
  for (const t of ['/scan', '/imu', '/sonar/range_1', '/odom', '/joint_states']) probe.subscribe(t);
  probe.subscribe('/camera/front/image_raw/compressed', 200);
  probe.subscribe('/camera/rear/image_raw/compressed', 200);
  probe.subscribe('/camera/depth/image_raw', 200);
  probe.subscribe('/scan_low', 200);

  await bootSim(page);
  await setManual(page); // keep the mission executor's hands off the base
  // park in the corridor facing the west cold bath: rim ahead, below the scan
  // plane. Pose places the LIDAR (front-right corner mast since the top-deck
  // bin redesign) on the same sightline the original rear-center mast had.
  await page.evaluate(() => window.__sim.setPose(-0.44, -3.95, Math.PI));

  await sleep(10_000);

  const counts = Object.fromEntries(
    ['/scan', '/imu', '/sonar/range_1', '/odom', '/joint_states',
      '/camera/front/image_raw/compressed', '/camera/rear/image_raw/compressed',
      '/camera/depth/image_raw',
    ].map((t) => [t, probe.received(t).length]),
  );
  expect(counts['/scan'], 'lidar ~8 Hz').toBeGreaterThan(40);
  expect(counts['/imu'], 'imu ~50 Hz').toBeGreaterThan(200);
  expect(counts['/sonar/range_1'], 'sonar ~15 Hz').toBeGreaterThan(60);
  expect(counts['/odom'], 'odom ~20 Hz').toBeGreaterThan(100);
  expect(counts['/joint_states'], 'joints ~20 Hz').toBeGreaterThan(100);
  expect(counts['/camera/front/image_raw/compressed'], 'front cam').toBeGreaterThan(10);
  expect(counts['/camera/rear/image_raw/compressed'], 'rear cam').toBeGreaterThan(10);
  expect(counts['/camera/depth/image_raw'], 'depth cam').toBeGreaterThan(10);

  const scan = probe.received('/scan').at(-1);
  const finite = scan.ranges.filter((r) => r !== null && Number.isFinite(r));
  expect(finite.length, 'lidar must see the walls').toBeGreaterThan(180);

  // bath rim (0.28 m) is invisible at the 0.62 m scan plane but fills the
  // bottom of the depth frame: forward beams must read the far wall instead
  const mid = Math.floor(scan.ranges.length / 2);
  const fwd = scan.ranges.slice(mid - 5, mid + 5).filter((r) => Number.isFinite(r));
  for (const r of fwd) expect(r, 'scan plane must clear the bath rim').toBeGreaterThan(2.0);

  const depth = probe.received('/camera/depth/image_raw').at(-1);
  expect(depth.width).toBe(320);
  expect(depth.height).toBe(240);
  expect(depth.encoding).toBe('16UC1');

  // content, not just shape: the rim ahead must yield real metric depths
  // (regression guard for the RGBADepthPacking channel-order bug)
  const bytes = Buffer.from(depth.data, 'base64');
  let valid = 0;
  for (let i = 0; i < bytes.length; i += 2) {
    const mm = bytes[i] | (bytes[i + 1] << 8);
    if (mm >= 280 && mm <= 3000) valid++;
  }
  expect(valid, 'depth frame must contain measured pixels').toBeGreaterThan(5000);

  // /scan_low (depth-derived low-obstacle layer) must see the 0.28 m rim
  // that the lidar plane cannot
  const low = probe.received('/scan_low').at(-1);
  const finiteLow = low.ranges.filter((r) => r !== null && Number.isFinite(r));
  expect(finiteLow.length, '/scan_low must return low obstacles').toBeGreaterThan(3);
  expect(Math.min(...finiteLow)).toBeLessThan(2.0);

  probe.close();
});

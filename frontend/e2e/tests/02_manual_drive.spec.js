import { expect, test } from '@playwright/test';
import { RosProbe, bootSim, holdButton, setManual } from '../helpers/ros.js';

test('manual drive: D-pad forward advances /odom and spins wheels', async ({ page }) => {
  const probe = new RosProbe();
  await probe.connect();
  await probe.clearSafety();
  probe.subscribe('/odom');
  probe.subscribe('/base/wheel_targets');

  await bootSim(page);
  await setManual(page);

  // face up the open corridor so 2 s of driving is unobstructed
  await page.evaluate(() => window.__sim.setPose(0, -3.6, Math.PI / 2));
  const before = await page.evaluate(() => window.__sim.pose());

  probe.clear('/base/wheel_targets');
  await holdButton(page, '#dpad-fwd', 2000);

  const after = await page.evaluate(() => window.__sim.pose());
  const travelled = Math.hypot(after.x - before.x, after.y - before.y);
  expect(travelled).toBeGreaterThan(0.3);

  await probe.waitFor('/odom', (m) => Math.abs(m.twist.twist.linear.x) > 0.05 ||
    Math.hypot(m.pose.pose.position.x - before.x, m.pose.pose.position.y - before.y) > 0.2,
  15_000, 'odom reflects motion');

  const spinning = probe.received('/base/wheel_targets')
    .some((m) => JSON.parse(m.data).w.some((w) => Math.abs(w) > 0.5));
  expect(spinning, 'wheel targets must be non-zero while driving').toBe(true);

  probe.close();
});

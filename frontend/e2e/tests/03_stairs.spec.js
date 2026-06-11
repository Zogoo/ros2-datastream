import { expect, test } from '@playwright/test';
import { RosProbe, bootSim, setManual, sleep } from '../helpers/ros.js';

// Resting deck: 45 mm step at x=-3.9..-3.55 then the 90 mm deck behind it.
test('stairs: suspension climbs the resting-deck steps without e-stop', async ({ page }) => {
  const probe = new RosProbe();
  await probe.connect();
  await probe.clearSafety();
  probe.subscribe('/imu', 100);
  probe.subscribe('/safety/stop');

  await bootSim(page);
  await setManual(page);
  await page.evaluate(() => window.__sim.setPose(-2.6, 1.3, Math.PI));
  const before = await page.evaluate(() => window.__sim.pose());

  // drive west until the chassis is up on the deck, then stop (the deck's
  // west half is furnished — driving blindly would ram a lounger)
  const btn = page.locator('#dpad-fwd');
  await btn.dispatchEvent('pointerdown');
  let pose = before;
  for (let i = 0; i < 40 && pose.x > -3.97; i++) {
    await sleep(200);
    pose = await page.evaluate(() => window.__sim.pose());
  }
  await btn.dispatchEvent('pointerup');
  await sleep(800);
  const after = await page.evaluate(() => window.__sim.pose());

  expect(after.x, 'front wheels must be up on the deck').toBeLessThan(-3.9);
  expect(after.z, 'chassis must have climbed both risers').toBeGreaterThan(before.z + 0.05);

  const imuZ = probe.received('/imu').map((m) => m.linear_acceleration.z);
  const spread = Math.max(...imuZ) - Math.min(...imuZ);
  expect(spread, 'step impacts must show in the IMU').toBeGreaterThan(0.5);

  const estopped = probe.received('/safety/stop').some((m) => m.data === true);
  expect(estopped, 'climbing the documented steps must not e-stop').toBe(false);

  probe.close();
});

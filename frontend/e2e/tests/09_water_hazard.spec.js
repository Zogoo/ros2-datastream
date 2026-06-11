import { expect, test } from '@playwright/test';
import { RosProbe, bootSim, holdButton, setManual, sleep } from '../helpers/ros.js';

test('water: towels float in the bath; the rim is geometrically unclimbable', async ({ page }) => {
  const probe = new RosProbe();
  await probe.connect();
  await probe.clearSafety();
  probe.subscribe('/safety/stop');

  await bootSim(page);
  await setManual(page); // the mission executor must not chase this towel

  // drop a towel into the west cold bath (water_z = 0.16)
  const towelId = await page.evaluate(() => window.__sim.spawn('towel', -3.5, -3.8, 0.8));
  await sleep(4000);
  const towel = await page.evaluate((id) => window.__sim.objectState(id), towelId);
  expect(towel.z, 'towel must settle floating near the water line').toBeGreaterThan(0.08);
  expect(towel.z).toBeLessThan(0.35);

  // force-drive at the rim: the 0.28 m wall is 2x wheel radius — no climb
  await setManual(page);
  await page.evaluate(() => window.__sim.setPose(-3.5, -2.45, -Math.PI / 2));
  await holdButton(page, '#dpad-fwd', 4000);
  const pose = await page.evaluate(() => window.__sim.pose());
  expect(pose.y, 'rim must block the robot before the water').toBeGreaterThan(-3.07);
  expect(pose.z, 'chassis must not be on top of the rim').toBeLessThan(0.30);

  probe.close();
});

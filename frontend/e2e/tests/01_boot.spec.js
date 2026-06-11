import { expect, test } from '@playwright/test';
import { RosProbe, bootSim } from '../helpers/ros.js';

test('boot: FE loads, rosbridge connects, heartbeat + HUD populate', async ({ page }) => {
  const probe = new RosProbe();
  await probe.connect();
  probe.subscribe('/sim/status');
  probe.subscribe('/odom');

  await bootSim(page);

  const status = await probe.waitFor('/sim/status', (m) => {
    const s = JSON.parse(m.data);
    return s.alive === true && s.sim_time > 0;
  }, 30_000, 'sim heartbeat');
  expect(JSON.parse(status.data).fps).toBeGreaterThan(0);

  await probe.waitFor('/odom', () => true, 15_000, 'odometry');

  await expect(page.locator('#hud-arm')).toBeVisible();
  await expect(page.locator('#hud-detect')).toBeVisible();
  await expect(page.locator('#hud-mission')).toBeVisible();
  await expect(page.locator('#lidar-panel')).toBeVisible();
  await expect(page.locator('#arm-joints-val')).not.toHaveText('');

  probe.close();
});

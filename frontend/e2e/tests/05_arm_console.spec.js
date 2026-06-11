import { expect, test } from '@playwright/test';
import { RosProbe, bootSim } from '../helpers/ros.js';

// Replays the firmware test transcript from the brief end-to-end through
// the FE console -> rosbridge -> arm_controller -> /arm/response.
test('arm console: serial-protocol transcript round-trips through ROS', async ({ page }) => {
  const probe = new RosProbe();
  await probe.connect();
  probe.subscribe('/arm/response');
  probe.subscribe('/arm/state');

  await bootSim(page);
  await page.locator('#console-toggle').click();

  const send = async (line) => {
    await page.locator('#console-input').fill(line);
    await page.locator('#console-send').click();
  };
  const expectReply = (pattern) =>
    probe.waitFor('/arm/response', (m) => pattern.test(m.data), 20_000, `reply ${pattern}`);
  const armSettled = (joints) => {
    probe.clear('/arm/state');
    return probe.waitFor('/arm/state', (m) => {
      const s = JSON.parse(m.data);
      return s.status === 'IDLE' &&
        joints.every((deg, i) => Math.abs(s.joints_deg[i] - deg) < 1.5);
    }, 20_000, `arm settled at [${joints}]`);
  };

  await send('A HOME');
  await expectReply(/^OK ACTION HOME$/);
  await armSettled([90, 90, 90, 90, 90, 70]);

  probe.clear('/arm/response');
  await send('Q');
  await expectReply(/^STATE 90 90 90 90 90 70 IDLE$/);

  await send('J 90 70 120 110 90 40 1000');
  await expectReply(/^OK J 90 70 120 110 90 40$/);
  await armSettled([90, 70, 120, 110, 90, 40]);

  probe.clear('/arm/response');
  await send('Q');
  await expectReply(/^STATE 90 70 120 110 90 40 IDLE$/);

  await send('D 0 5 300');
  await expectReply(/^OK D joint=0 delta=5$/);
  await armSettled([95, 70, 120, 110, 90, 40]);
  await send('D 0 200 300');
  await expectReply(/^ERR LIMIT joint=0 value=295$/);

  await send('SPEED 50');
  await expectReply(/^OK SPEED 50$/);
  await send('STOP');
  await expectReply(/^OK STOP$/);
  await send('A HOME');
  await expectReply(/^ERR STOPPED$/);
  await send('A RESET_ERROR');
  await expectReply(/^OK RESET_ERROR$/);
  probe.clear('/arm/response');
  await send('A HOME');
  await expectReply(/^OK ACTION HOME$/);

  // responses are also mirrored into the on-screen console log
  expect(await page.locator('#console-log').textContent()).toContain('ERR STOPPED');

  probe.close();
});

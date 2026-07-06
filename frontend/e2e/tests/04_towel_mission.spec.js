import { expect, test } from '@playwright/test';
import { RosProbe, bootSim, setAuto, setManual } from '../helpers/ros.js';

// The core acceptance test: AUTO mode picks a thrown towel and bins it.
test('towel mission: autonomy approaches, picks and delivers to the towel bin', async ({ page }) => {
  test.setTimeout(300_000);
  const probe = new RosProbe();
  await probe.connect();
  probe.subscribe('/mission/state');
  probe.subscribe('/robot/events');

  await bootSim(page);
  await probe.clearSafety();

  // Hold MANUAL while staging: control mode defaults to auto, so the mission
  // would otherwise lock a target from the pre-teleport spawn pose (a race —
  // it then chases a towel across the map instead of the staged one).
  await setManual(page);

  // stage: robot in the corridor facing north, towel 1 m ahead, bin close by
  // (keeps the deliver leg short so grasp retries don't eat the timeout)
  await page.evaluate(() => {
    window.__sim.setPose(0, 3.04, Math.PI / 2);
    window.__sim.spawn('towel', 0.08, 4.64, 0.1);
  });
  await setAuto(page);

  await probe.waitFor('/mission/state',
    (m) => !['IDLE', 'SEARCH'].includes(JSON.parse(m.data).state),
    60_000, 'mission engages a towel');

  await probe.waitFor('/mission/state',
    (m) => JSON.parse(m.data).holding === true,
    180_000, 'towel grasped');

  const binned = await probe.waitFor('/robot/events', (m) => {
    const e = JSON.parse(m.data);
    return e.event === 'OBJECT_BINNED' && e.object_class === 'towel';
  }, 240_000, 'towel delivered to a bin');
  expect(JSON.parse(binned.data).correct).toBe(true);

  probe.close();
});

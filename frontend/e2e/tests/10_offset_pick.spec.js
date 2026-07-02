import { expect, test } from '@playwright/test';
import { RosProbe, bootSim, setAuto, setManual, sleep } from '../helpers/ros.js';

// Autonomy + analytic IK: a towel placed off the robot's centre line is picked
// by panning the arm to the measured pose — a reach the canned centred
// PICK_SCOOP could never make. Targets come from perception (GT off).
test('offset pick: IK reaches an off-axis towel that canned poses cannot', async ({ page }) => {
  test.setTimeout(300_000);
  const probe = new RosProbe();
  await probe.connect();
  probe.subscribe('/mission/state');
  probe.subscribe('/robot/events');
  probe.subscribe('/arm/command');

  await bootSim(page);
  // The safety latch lives in the robot_state NODE and survives FE reloads —
  // scenario 09 (water hazard) legitimately trips the tilt e-stop and runs
  // before this file, so clear it or the mission aborts to IDLE every tick.
  await probe.clearSafety();
  // Hold MANUAL while staging: control mode defaults to auto, so the mission
  // would otherwise lock a target from the pre-teleport spawn pose and chase
  // it across the map instead of the staged towel.
  await setManual(page);
  await sleep(4000); // localizer + tracker warm-up

  // robot facing north; towel offset to the right and ahead, within the
  // reachable annulus once the robot squares up to it
  await page.evaluate(() => {
    window.__sim.setPose(0, -1.0, Math.PI / 2);
    window.__sim.spawn('towel', 0.45, 0.5, 0.1);
  });
  await setAuto(page);

  // an IK reach command (raw J line) must be issued — proof the arm solved for
  // the measured pose rather than replaying a named pose
  await probe.waitFor('/arm/command',
    (m) => /^J \d+ \d+ \d+ \d+ \d+ \d+ \d+$/.test(m.data),
    180_000, 'IK reach J command');

  const binned = await probe.waitFor('/robot/events', (m) => {
    const e = JSON.parse(m.data);
    return e.event === 'OBJECT_BINNED' && e.object_class === 'towel';
  }, 240_000, 'off-axis towel delivered');
  expect(JSON.parse(binned.data).correct).toBe(true);

  probe.close();
});

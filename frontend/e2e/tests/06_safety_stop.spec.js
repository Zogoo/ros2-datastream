import { expect, test } from '@playwright/test';
import { RosProbe, bootSim, holdButton, setManual, sleep } from '../helpers/ros.js';

test('safety: hard impact latches e-stop, zeroes wheels, operator reset recovers', async ({ page }) => {
  const probe = new RosProbe();
  await probe.connect();
  probe.subscribe('/robot/contacts');
  probe.subscribe('/safety/stop');
  probe.subscribe('/base/wheel_targets');

  await bootSim(page);
  await probe.clearSafety();
  await setManual(page);

  // runaway scenario: launch at 2.5 m/s into the towel-bin wall while the
  // D-pad keeps the wheels pushing (manual top speed alone gives a borderline
  // ~3 N·s impulse the suspension soaks up; a 2 kg stool would just get
  // pushed — correct physics either way)
  await page.evaluate(() => window.__sim.setPose(0, 3.4, Math.PI / 2));
  const fwd = page.locator('#dpad-fwd');
  await fwd.dispatchEvent('pointerdown');  // wheels keep pushing through the flight
  await page.evaluate(() => window.__sim.setVelocity(0, 2.5));
  await sleep(2000);
  await fwd.dispatchEvent('pointerup');

  await probe.waitFor('/robot/contacts',
    (m) => JSON.parse(m.data).impulse >= 3.0, 20_000, 'hard contact event');
  await probe.waitFor('/safety/stop', (m) => m.data === true, 20_000, 'e-stop latch');
  await expect(page.locator('#estop-badge')).toBeVisible();

  // wheel targets must zero regardless of operator input
  await sleep(500);
  probe.clear('/base/wheel_targets');
  await holdButton(page, '#dpad-fwd', 800);
  const targets = probe.received('/base/wheel_targets').map((m) => JSON.parse(m.data).w);
  expect(targets.length).toBeGreaterThan(0);
  for (const w of targets) {
    expect(Math.max(...w.map(Math.abs))).toBe(0);
  }

  // recovery: operator reset clears the aggregator latch -> base unlatches
  probe.clear('/safety/stop');
  await probe.publish('/safety/reset', 'std_msgs/Bool', { data: true });
  await probe.waitFor('/safety/stop', (m) => m.data === false, 20_000, 'latch released');
  await expect(page.locator('#estop-badge')).toBeHidden();

  probe.close();
});

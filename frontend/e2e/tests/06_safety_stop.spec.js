import { expect, test } from '@playwright/test';
import { RosProbe, armSafety, bootSim, setManual, sleep } from '../helpers/ros.js';

test('safety: armed e-stop latches on hard impact, zeroes wheels, operator reset recovers', async ({ page }) => {
  const probe = new RosProbe();
  await probe.connect();
  probe.subscribe('/robot/contacts');
  probe.subscribe('/safety/stop');
  probe.subscribe('/base/wheel_targets');

  await bootSim(page);
  await probe.clearSafety();
  await setManual(page);
  await armSafety(page); // latching only happens while the UI has armed it

  // climb onto the resting deck and ram lounger_2 — wedging heavy furniture
  // under drive reliably produces a >3 N·s contact (flat wall pushes don't:
  // the suspension spreads those out, and light props just get shoved)
  await page.evaluate(() => window.__sim.setPose(-3.2, 0.6, Math.PI));
  const btn = page.locator('#dpad-fwd');
  await btn.dispatchEvent('pointerdown');
  for (let i = 0; i < 45; i++) {
    await sleep(200);
    if (await page.evaluate(() => window.__sim.safetyStop())) break;
  }
  await btn.dispatchEvent('pointerup');

  await probe.waitFor('/robot/contacts',
    (m) => JSON.parse(m.data).impulse >= 3.0, 20_000, 'hard contact event');
  await probe.waitFor('/safety/stop', (m) => m.data === true, 20_000, 'e-stop latch');
  await expect(page.locator('#estop-badge')).toBeVisible();

  // wheel targets must zero regardless of operator input
  await sleep(500);
  probe.clear('/base/wheel_targets');
  await btn.dispatchEvent('pointerdown');
  await sleep(800);
  await btn.dispatchEvent('pointerup');
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

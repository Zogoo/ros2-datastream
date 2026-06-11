import { expect, test } from '@playwright/test';
import { bootSim } from '../helpers/ros.js';

test('skins: preset retextures towels and resyncs the AI detection profile', async ({ page }) => {
  await bootSim(page);

  const defaults = await page.evaluate(async () =>
    (await (await fetch('/api/profiles')).json()).towel);

  await page.locator('#skins-toggle').click();
  await page.locator('#skin-class').selectOption('towel');
  await page.locator('.skin-preset[data-preset="striped"]').click();
  await expect(page.locator('#skin-status')).toContainText('AI profile updated', { timeout: 20_000 });

  const updated = await page.evaluate(async () =>
    (await (await fetch('/api/profiles')).json()).towel);
  expect(updated.lower).not.toEqual(defaults.lower);

  // per-instance skin: pick a single towel, apply a different preset
  await page.locator('#skin-class').selectOption('towel');
  const hasInstance = await page.locator('#skin-instance option').count() > 1;
  expect(hasInstance, 'instance dropdown must list towels').toBe(true);
  await page.locator('#skin-instance').selectOption({ index: 1 });
  await page.locator('.skin-preset[data-preset="charcoal"]').click();
  await expect(page.locator('#skin-status')).toContainText('skin applied');

  // reset restores the default class profile on the AI side
  await page.locator('#skin-instance').selectOption('');
  await page.locator('#skin-reset-btn').click();
  await expect.poll(async () => page.evaluate(async () =>
    (await (await fetch('/api/profiles')).json()).towel.lower), { timeout: 20_000 })
    .toEqual(defaults.lower);
});

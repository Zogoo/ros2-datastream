import { defineConfig } from '@playwright/test';

// One worker, sequential files: the FE tab IS the simulator, so two parallel
// pages would double-publish every topic (single-active-tab rule).
export default defineConfig({
  testDir: './tests',
  workers: 1,
  fullyParallel: false,
  timeout: 120_000,
  retries: 1,
  reporter: 'list',
  use: {
    baseURL: process.env.FE_URL ?? 'http://localhost:8080',
    viewport: { width: 1280, height: 800 },
    launchOptions: {
      // WebGL in headless chromium needs SwiftShader explicitly allowed
      args: ['--enable-unsafe-swiftshader', '--use-angle=swiftshader-webgl'],
    },
  },
});

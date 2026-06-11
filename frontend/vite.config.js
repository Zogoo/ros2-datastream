import { defineConfig } from 'vite';

export default defineConfig({
  server: { port: 8080 },
  build: {
    outDir: 'dist',
    target: 'es2022',
    chunkSizeWarningLimit: 2500,
  },
  test: {
    environment: 'node',
    include: ['tests/**/*.test.js'],
  },
});

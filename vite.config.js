import { defineConfig } from 'vite';
import { fileURLToPath } from 'node:url';
import { metaAiPlugin } from './server/meta-ai.js';

export default defineConfig({
  plugins: [metaAiPlugin()],
  build: {
    rollupOptions: {
      input: Object.fromEntries(['index', 'motor', 'scan', 'scan-navigation'].map(name => [name, fileURLToPath(new URL(`./${name}.html`, import.meta.url))])),
    },
  },
});

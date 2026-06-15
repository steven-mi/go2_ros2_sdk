import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

export default defineConfig({
  plugins: [react()],
  server: {
    host: '0.0.0.0',
    port: 5173,
    strictPort: false,
    watch: {
      usePolling: true,
    },
    proxy: {
      '/rosbridge': {
        target: 'ws://127.0.0.1:9090',
        ws: true,
        changeOrigin: true,
        rewrite: () => '',
      },
    },
  },
  preview: {
    host: '0.0.0.0',
    port: 8080,
  },
  build: {
    outDir: 'dist',
    emptyOutDir: true,
  },
});

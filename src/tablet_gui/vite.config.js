import { sveltekit } from '@sveltejs/kit/vite';

/** @type {import('vite').UserConfig} */
const config = {
  plugins: [sveltekit()],
  server: {
    host: '0.0.0.0',
    port: 5173,
    strictPort: true,
    allowedHosts: ['linuxros2.tail360dc7.ts.net'],
    hmr: {
      host: 'linuxros2.tail360dc7.ts.net',
      protocol: 'ws',
      port: 5173
    }
  },
  preview: {
    host: '0.0.0.0',
    port: 4173,
    strictPort: true
  },
  ssr: {
    noExternal: ['three', 'roslib']
  }
};

export default config;

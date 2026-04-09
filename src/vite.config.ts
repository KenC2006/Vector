import { defineConfig } from 'vite'
import { copyFileSync, existsSync } from 'fs'
import { resolve } from 'path'

function syncPresetsPlugin() {
  const src = resolve(__dirname, '..', 'core', 'presets', 'generic_presets.json')
  const dest = resolve(__dirname, 'public', 'generic_presets.json')
  return {
    name: 'sync-presets',
    buildStart() {
      if (existsSync(src)) {
        copyFileSync(src, dest)
        console.log('[sync-presets] Copied generic_presets.json to public/')
      }
    },
  }
}

export default defineConfig({
  plugins: [syncPresetsPlugin()],
  server: {
    port: 1420,
    strictPort: true,
  },
})

import { defineConfig, type ViteDevServer } from 'vite'
import { copyFileSync, existsSync } from 'fs'
import { resolve } from 'path'

/** core/presets/generic_presets.json is the catalog; public/ serves a copy.
 *  Copy it at build start, and in dev also whenever the core file changes
 *  (then reload the page), so an edited catalog is never stale in the app. */
function syncPresetsPlugin() {
  const src = resolve(__dirname, '..', 'core', 'presets', 'generic_presets.json')
  const dest = resolve(__dirname, 'public', 'generic_presets.json')
  const sync = () => {
    if (existsSync(src)) copyFileSync(src, dest)
  }
  return {
    name: 'sync-presets',
    buildStart() {
      sync()
      console.log('[sync-presets] Copied generic_presets.json to public/')
    },
    configureServer(server: ViteDevServer) {
      server.watcher.add(src)
      server.watcher.on('change', (file: string) => {
        if (resolve(file) !== src) return
        sync()
        console.log('[sync-presets] Catalog changed — copied to public/ and reloading')
        server.ws.send({ type: 'full-reload' })
      })
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

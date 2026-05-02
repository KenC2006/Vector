import { createServer } from 'vite'

const modulePath = process.argv[2]
if (!modulePath) {
  console.error('usage: node scripts/run-vite-module.mjs <module>')
  process.exit(1)
}

const server = await createServer({
  appType: 'custom',
  server: { middlewareMode: true },
  logLevel: 'error',
})

try {
  const normalized = modulePath.startsWith('/') ? modulePath : `/${modulePath.replaceAll('\\', '/')}`
  await server.ssrLoadModule(normalized)
} finally {
  await server.close()
}

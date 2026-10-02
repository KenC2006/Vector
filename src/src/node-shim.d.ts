// Minimal Node.js type stubs for the test-corpus runner (topologyCorpus.ts).
// The browser bundle doesn't use these, so we avoid pulling in all of @types/node.
// If the test harness grows, replace this with `npm i -D @types/node`.

declare module 'node:fs' {
  export function readFileSync(path: string, encoding: 'utf-8' | 'utf8'): string
  export function existsSync(path: string): boolean
}

declare module 'node:path' {
  export function resolve(...segments: string[]): string
  export function dirname(path: string): string
}

declare module 'node:url' {
  export function fileURLToPath(url: string | URL): string
}

declare const process: {
  exit(code?: number): never
}

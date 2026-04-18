# Playwright Testing Guide for Vector

## Prerequisites

```bash
# Playwright is already installed globally
npx playwright --version  # should show 1.59+

# Start the Vite dev server (frontend only, no Tauri)
cd src && npm run dev
# Serves on http://localhost:1420
```

## Key Concepts

Vector is a Tauri app. When testing in browser-only mode (no `cargo tauri dev`):
- The frontend loads fully: Three.js viewport, Monaco editor, rich visuals, all generators
- **Tauri IPC is unavailable** — ignore errors about "Tauri IPC not available" and Python core
- The AI chat / assembly pipeline won't work (needs Python backend)
- Everything else works: URDF parsing, 3D rendering, rich visuals, component colors

## Injecting URDF into the Monaco Editor

The Monaco editor is **not** exposed on `window`. Use Playwright keyboard interaction:

```javascript
// 1. Focus the editor
const editorEl = await page.$('.monaco-editor .view-lines')
await editorEl.click()

// 2. Select all existing content
await page.keyboard.press('Control+a')

// 3. Paste new URDF via clipboard
await page.evaluate((urdf) => navigator.clipboard.writeText(urdf), myUrdfString)
await page.keyboard.press('Control+v')

// 4. Wait for debounced reparse (500ms) + rich visuals
await page.waitForTimeout(2000)
```

**Why not `window.monaco.editor.getEditors()`?** — Monaco is imported as an ES module, not set on `window`. The `monaco` object is local to `main.ts`.

**Why not set the textarea directly?** — The textarea `#sim-script-editor` is the simulation script editor, not the URDF editor. The URDF editor is Monaco.

## Checking Scene State

```javascript
const state = await page.evaluate(() => {
  const text = document.body.innerText
  return {
    links: text.match(/Links?:?\s*\d+/)?.[0],   // e.g. "Links: 5"
    joints: text.match(/Joints?:?\s*\d+/)?.[0],  // e.g. "Joints: 4"
  }
})
```

## Capturing Errors

```javascript
const errors = []
page.on('console', msg => {
  if (msg.type() === 'error') errors.push(msg.text())
})
page.on('pageerror', err => errors.push(err.message))

// Filter out known non-issues in browser-only mode
const realErrors = errors.filter(e =>
  !e.includes('Tauri IPC') &&
  !e.includes('AI completions') &&
  !e.includes('Python') &&
  !e.includes('504') &&
  !e.includes('Outdated Optimize')
)
```

## Zooming the 3D Viewport

```javascript
const canvas = await page.$('canvas')
if (canvas) {
  await canvas.click()
  // Scroll to zoom in
  for (let i = 0; i < 15; i++) {
    await page.mouse.wheel(0, -120)
    await page.waitForTimeout(50)
  }
}
```

## Rich Visual System

When a URDF is loaded, the app:
1. Parses URDF XML → creates primitive Three.js meshes (boxes, cylinders)
2. Calls `applyRichVisuals()` which replaces primitives with detailed parametric meshes
3. For components with GLB mesh overrides, loads the GLB and scales to match

Console log to watch for: `[richVisuals] GLB cache preloaded: N components from M files`

## Example: Full Color Test

See `test-colors.mjs` in the project root for a working example that:
- Launches Chromium
- Loads the app
- Pastes a test URDF with multiple component types
- Verifies 5 links rendered with zero color-system errors
- Takes a screenshot

Run: `node test-colors.mjs`

## Gotchas

- **Kill the right process**: Never use `taskkill //F //IM node.exe` — it kills ALL node processes including Claude Code. To kill just the Vite server, find its PID with `netstat -ano | findstr :1420` and kill that specific PID.
- **Stale Vite cache**: If you get "504 Outdated Optimize Dep" errors, delete `src/node_modules/.vite/` and restart the dev server.
- **Headless clipboard**: `navigator.clipboard.writeText()` may not work in headless mode. Use `headless: false` or fall back to `document.execCommand('insertText', false, text)`.
- **Monaco focus**: The editor must be focused (clicked) before keyboard commands work. Always click `.view-lines` first.

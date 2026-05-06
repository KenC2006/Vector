// Phase 1 grep gate (PLACEMENT_REWRITE_PLAN.md §3.5):
// every URDF/MuJoCo Z-up ↔ Three.js Y-up basis swap must go through
// `src/src/coordinates.ts`. This script fails CI when an inline swap shows
// up anywhere else in src/src.
//
// The swap signature we look for is the literal pattern of constructing a
// THREE.Vector3 with the second slot pulled from index 2 / .z and the third
// slot negated from index 1 / .y (or vice-versa for the inverse). Adding a
// new variant of the swap requires routing through coordinates.ts; if the
// pattern legitimately needs to live outside (e.g. a future audited shim),
// add it to the ALLOWED set below with a comment.

import { readFile, readdir } from 'node:fs/promises'
import { fileURLToPath } from 'node:url'
import path from 'node:path'

const repoRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..')
const SRC_DIR = path.join(repoRoot, 'src', 'src')

// Files allowed to perform the swap. Keep this list short; every entry is
// a deviation from "single seam".
const ALLOWED = new Set([
  'coordinates.ts',          // the single owner
  'carrySnapMath.ts',        // re-exports coordinates.urdfVecToSceneVec
])

// Patterns that indicate a Z-up ↔ Y-up basis swap. False positives are
// acceptable so long as the offending file is added to ALLOWED with a
// rationale; we'd rather over-flag than miss one.
const SWAP_PATTERNS = [
  // new THREE.Vector3(<x>, <z>, -<y>) — URDF→scene
  /new\s+THREE\.Vector3\s*\([^,)]+,\s*[^,)]*\.z[^,)]*,\s*-[^,)]+\.y[^,)]*\)/,
  /new\s+THREE\.Vector3\s*\([^,)]+,\s*[^,)]*\[\s*2\s*\][^,)]*,\s*-\(?[^,)]+\[\s*1\s*\]/,
  // new THREE.Vector3(<x>, -<z>, <y>) — scene→URDF
  /new\s+THREE\.Vector3\s*\([^,)]+,\s*-[^,)]+\.z[^,)]*,\s*[^,)]+\.y[^,)]*\)/,
]

async function listTsFiles(dir, rootLen) {
  const out = []
  for (const entry of await readdir(dir, { withFileTypes: true })) {
    const abs = path.join(dir, entry.name)
    if (entry.isDirectory()) {
      out.push(...(await listTsFiles(abs, rootLen)))
    } else if (entry.isFile() && entry.name.endsWith('.ts')) {
      out.push(abs.slice(rootLen + 1))
    }
  }
  return out
}

async function main() {
  const files = await listTsFiles(SRC_DIR, SRC_DIR.length)

  const violations = []
  for (const rel of files) {
    if (ALLOWED.has(rel)) continue
    const absPath = path.join(SRC_DIR, rel)
    const text = await readFile(absPath, 'utf8')
    const lines = text.split(/\r?\n/)
    for (let i = 0; i < lines.length; i++) {
      const line = lines[i]
      for (const pat of SWAP_PATTERNS) {
        if (pat.test(line)) {
          violations.push({ rel, line: i + 1, text: line.trim() })
          break
        }
      }
    }
  }

  if (violations.length > 0) {
    console.error('[coordinates-seam] inline Z↔Y basis swaps detected outside coordinates.ts:')
    for (const v of violations) {
      console.error(`  ${v.rel}:${v.line}  ${v.text}`)
    }
    console.error(
      '\nRoute the swap through `urdfVecToSceneVec` / `urdfFrameToScene` in\n' +
      'src/src/coordinates.ts, or — if this file legitimately must perform the\n' +
      'swap inline — add its name to the ALLOWED set in\n' +
      'scripts/check-coordinates-seam.mjs with a rationale.'
    )
    process.exit(1)
  }

  console.log(`[coordinates-seam] ok: scanned ${files.length} files, 0 inline swaps outside coordinates.ts`)
}

main().catch(err => {
  console.error('[coordinates-seam] internal error:', err)
  process.exit(2)
})

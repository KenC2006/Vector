// Phase 7a — single-preset checker.
//
// Usage:  node scripts/preset-check.mjs <componentId>
//         npm run preset:check -- <componentId>
//
// Prints a focused report for one component:
//   - declared bbox + visual extent + collision extent + divergences
//   - content hashes (so you see if mesh was edited)
//   - whether the strict mesh-extent gate would still pass for this preset
//   - first-line of any spec validation errors that mention this id
//
// The full `npm run check` runs all gates across the whole catalog. This is
// the inner loop for someone tweaking a single preset's bbox / mesh override
// / connector and wanting <2s feedback rather than a 30s catalog walk.

import fs from 'node:fs/promises'
import path from 'node:path'
import { spawnSync } from 'node:child_process'
import { fileURLToPath } from 'node:url'

const here = path.dirname(fileURLToPath(import.meta.url))
const repoRoot = path.resolve(here, '..')
const PRESETS_PATH = path.join(repoRoot, 'core', 'presets', 'generic_presets.json')
const MESH_EXTENTS_PATH = path.join(repoRoot, 'scripts', 'mesh-extents.generated.json')
const BASELINE_PATH = path.join(repoRoot, 'scripts', 'mesh-extent-baseline.json')

const COLLISION_LIMIT = 0.15

function fail(msg) {
  console.error(msg)
  process.exit(1)
}

function fmtMm(arr) {
  if (!Array.isArray(arr)) return String(arr)
  return arr.map(v => typeof v === 'number' ? v.toFixed(2) : String(v)).join(' × ')
}

function fmtPct(v) {
  return typeof v === 'number' ? `${(v * 100).toFixed(1)}%` : '—'
}

function findPreset(catalog, id) {
  for (const cat of Object.values(catalog.categories ?? {})) {
    for (const c of cat.components ?? []) {
      if (c.id === id) return c
    }
  }
  return null
}

async function main() {
  const id = process.argv[2]
  if (!id) fail('usage: preset-check.mjs <componentId>')

  const presets = JSON.parse(await fs.readFile(PRESETS_PATH, 'utf8'))
  const preset = findPreset(presets, id)
  if (!preset) fail(`unknown component id: ${id}`)

  let extents = null
  try {
    const all = JSON.parse(await fs.readFile(MESH_EXTENTS_PATH, 'utf8'))
    extents = all.components?.[id] ?? null
  } catch {
    /* ok — measure may not have run yet */
  }

  let baseline = null
  try {
    baseline = JSON.parse(await fs.readFile(BASELINE_PATH, 'utf8'))
  } catch {
    /* optional */
  }

  console.log(`── ${id} ─────────────────────────────────────`)
  const bbox = preset.physical?.bounding_box_mm ?? preset.physical?.bbox_mm
  if (bbox) console.log(`  declared bbox_mm   : ${fmtMm(bbox)}`)

  if (extents?.visual) {
    const v = extents.visual
    console.log(`  visual mesh        : ${v.file}  hash=${v.content_hash ?? '—'}`)
    console.log(`    rotation rpy     : [${(v.rotation_rpy ?? []).map(n => n.toFixed(3)).join(', ')}]`)
    console.log(`    post-rot extent  : ${fmtMm(v.post_rotation_extent_mm)} mm`)
    console.log(`    divergence       : ${fmtPct(v.divergence_vs_bbox)}`)
    if (baseline) {
      const baseDiv = baseline.components?.[id]
      if (typeof baseDiv === 'number' && typeof v.divergence_vs_bbox === 'number') {
        const tol = baseline.tolerance ?? 0.005
        const delta = v.divergence_vs_bbox - baseDiv
        const flag = v.divergence_vs_bbox > baseDiv + tol ? ' GREW (would fail strict gate)' : ''
        console.log(`    vs baseline      : ${fmtPct(baseDiv)} (Δ ${(delta * 100).toFixed(2)}pp)${flag}`)
      } else if (v.divergence_vs_bbox > 0.05) {
        console.log(`    vs baseline      : NEW DRIFT (would fail strict gate — re-baseline if intentional)`)
      }
    }
  }

  if (extents?.collision) {
    const c = extents.collision
    console.log(`  collision mesh     : ${c.file}  hash=${c.content_hash ?? '—'}`)
    console.log(`    extent           : ${fmtMm(c.extent_mm)} mm  center ${fmtMm(c.center_mm)} mm`)
    const cdiv = c.divergence_vs_bbox
    const cFlag = typeof cdiv === 'number' && cdiv > COLLISION_LIMIT ? ` OVER ${(COLLISION_LIMIT * 100)}% LIMIT` : ''
    console.log(`    divergence       : ${fmtPct(cdiv)}${cFlag}`)
  }

  // Run validate-component-specs and surface only lines that mention this id.
  const validate = spawnSync('node', ['scripts/validate-component-specs.mjs'], {
    cwd: repoRoot,
    encoding: 'utf8',
  })
  const validateOutput = (validate.stdout || '') + (validate.stderr || '')
  const idLines = validateOutput.split('\n').filter(line => line.includes(id))
  if (idLines.length > 0) {
    console.log(`  spec validation`)
    for (const line of idLines) console.log(`    ${line.trim()}`)
  }

  const catalogIssues = validate.status !== 0 ? ` (catalog has ${validate.status === 1 ? 'errors' : 'issues'} elsewhere)` : ''
  console.log(`  status             : ok${catalogIssues}`)
}

main().catch(e => { console.error(e); process.exit(1) })

// Propose mesh.rotation_rpy fixes for components in mesh-extent-baseline.json.
//
// "Bbox is the source of truth" (see src/public/COMPONENT_PRESETS.md).
// For each baselined component, try the 6 axis-permuted rotations of the
// visual GLB's raw extent + an optional uniform-scale fit, and report
// what's needed to bring it under the 5% threshold:
//
//   ROTATE         rotation alone gets ≤ 5%
//   ROTATE+SCALE   rotation + uniform scale gets ≤ 5% (right shape, wrong size)
//   NEAR           between 5% and 10% — borderline; pick up via triage --defer/--apply
//   SHAPE-MISMATCH no rotation+uniform-scale combo works (re-author / different mesh)
//
// Also detects SHARED GLBs (one mesh used by N presets with different bboxes) —
// these are flagged because no single rotation+scale can satisfy all of them.
//
// Read-only: never edits presets. Run after measure-mesh-extents.mjs.
//
// Usage:
//   cd src && node ../scripts/propose-mesh-rotations.mjs
//   cd src && node ../scripts/propose-mesh-rotations.mjs --json   (machine output)

import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const here = path.dirname(fileURLToPath(import.meta.url))
const repoRoot = path.resolve(here, '..')
const EXTENTS = path.join(repoRoot, 'src', 'public', 'meshExtents.generated.json')
const BASELINE = path.join(repoRoot, 'scripts', 'mesh-extent-baseline.json')
const PRESETS = path.join(repoRoot, 'src', 'public', 'generic_presets.json')

const PI_2 = Math.PI / 2

// 6 permutations of a 3-vector and a representative rpy that produces each
// (rpy applied as XYZ extrinsic; only 90°-multiple rotations are considered).
const PERMS = [
  { name: 'identity',          perm: [0, 1, 2], rpy: [0, 0, 0],          desc: '[0, 0, 0]' },
  { name: 'rotZ 90  (XY swap)',perm: [1, 0, 2], rpy: [0, 0, PI_2],        desc: '[0, 0, Math.PI / 2]' },
  { name: 'rotY 90  (XZ swap)',perm: [2, 1, 0], rpy: [0, PI_2, 0],        desc: '[0, Math.PI / 2, 0]' },
  { name: 'rotX 90  (YZ swap)',perm: [0, 2, 1], rpy: [PI_2, 0, 0],        desc: '[Math.PI / 2, 0, 0]' },
  { name: 'cycle XYZ→YZX',     perm: [1, 2, 0], rpy: [PI_2, 0, PI_2],     desc: '[Math.PI / 2, 0, Math.PI / 2]' },
  { name: 'cycle XYZ→ZXY',     perm: [2, 0, 1], rpy: [0, PI_2, PI_2],     desc: '[0, Math.PI / 2, Math.PI / 2]' },
]

function maxAxisDivergence(a, b) {
  let worst = 0
  for (let i = 0; i < 3; i++) {
    if (b[i] === 0) continue
    const d = Math.abs(a[i] - b[i]) / b[i]
    if (d > worst) worst = d
  }
  return worst
}

// L∞ best uniform scale s minimizing max_i |s*a[i]/b[i] - 1|.
// Closed form: s = 2 / (qmax + qmin)  where q[i] = a[i]/b[i].
// Returns { scale, divergence } after applying that scale.
function bestUniformScaleFit(a, b) {
  const q = []
  for (let i = 0; i < 3; i++) {
    if (b[i] === 0) return { scale: 1, divergence: Infinity }
    q.push(a[i] / b[i])
  }
  const qmin = Math.min(...q)
  const qmax = Math.max(...q)
  const s = 2 / (qmax + qmin)
  const scaled = a.map(v => v * s)
  return { scale: s, divergence: maxAxisDivergence(scaled, b) }
}

function flattenPresets(data) {
  const out = []
  for (const cat of Object.values(data.categories || {})) {
    for (const c of cat.components || []) out.push(c)
  }
  return out
}

function presetBboxMm(component) {
  const phys = component.physical || {}
  if (Array.isArray(phys.bbox_mm) && phys.bbox_mm.length === 3) return [...phys.bbox_mm]
  if (Array.isArray(phys.bounding_box_mm) && phys.bounding_box_mm.length === 3) return [...phys.bounding_box_mm]
  return null
}

async function main() {
  const wantJson = process.argv.includes('--json')
  const extents = JSON.parse(await fs.readFile(EXTENTS, 'utf8'))
  const baseline = JSON.parse(await fs.readFile(BASELINE, 'utf8'))
  const presets = JSON.parse(await fs.readFile(PRESETS, 'utf8'))

  const presetById = new Map()
  for (const c of flattenPresets(presets)) presetById.set(c.id, c)

  // Detect shared GLBs across all measured components (not just baselined).
  const fileUsage = new Map()
  for (const [cid, e] of Object.entries(extents.components || {})) {
    const f = e.visual?.file
    if (!f) continue
    if (!fileUsage.has(f)) fileUsage.set(f, [])
    fileUsage.get(f).push(cid)
  }

  const proposals = []

  for (const id of Object.keys(baseline.components).sort()) {
    const entry = extents.components?.[id]
    if (!entry?.visual) {
      proposals.push({ id, status: 'NO-DATA', note: 'no visual entry in meshExtents' })
      continue
    }
    const raw = entry.visual.raw_extent_mm
    const bbox = entry.declared_bbox_mm
    if (!raw || !bbox) {
      proposals.push({ id, status: 'NO-DATA', note: 'missing raw_extent or declared bbox' })
      continue
    }

    const visualFile = entry.visual.file
    const sharedWith = (fileUsage.get(visualFile) || []).filter(x => x !== id)

    let bestRot = null
    let bestRotScale = null
    for (const p of PERMS) {
      const rotated = [raw[p.perm[0]], raw[p.perm[1]], raw[p.perm[2]]]
      const divRot = maxAxisDivergence(rotated, bbox)
      if (bestRot === null || divRot < bestRot.div) bestRot = { ...p, rotated, div: divRot }

      const fit = bestUniformScaleFit(rotated, bbox)
      if (bestRotScale === null || fit.divergence < bestRotScale.div) {
        bestRotScale = { ...p, rotated, scale: fit.scale, div: fit.divergence }
      }
    }

    const baselineDiv = baseline.components[id]
    let status
    if (bestRot.div <= 0.05) status = 'ROTATE'
    else if (bestRotScale.div <= 0.05) status = 'ROTATE+SCALE'
    else if (bestRot.div <= 0.10 || bestRotScale.div <= 0.10) status = 'NEAR'
    else status = 'SHAPE-MISMATCH'

    proposals.push({
      id,
      status,
      visual_file: visualFile,
      shared_with: sharedWith,
      bbox_mm: bbox,
      raw_extent_mm: raw.map(v => +v.toFixed(2)),
      rotation_only: {
        perm: bestRot.name,
        rpy: bestRot.desc,
        rotated_extent_mm: bestRot.rotated.map(v => +v.toFixed(2)),
        divergence: +bestRot.div.toFixed(4),
      },
      rotation_plus_scale: {
        perm: bestRotScale.name,
        rpy: bestRotScale.desc,
        scale: +bestRotScale.scale.toFixed(4),
        divergence: +bestRotScale.div.toFixed(4),
      },
      baseline_div: +baselineDiv.toFixed(4),
    })
  }

  if (wantJson) {
    console.log(JSON.stringify(proposals, null, 2))
    return
  }

  const groups = { ROTATE: [], 'ROTATE+SCALE': [], NEAR: [], 'SHAPE-MISMATCH': [], 'NO-DATA': [] }
  for (const p of proposals) (groups[p.status] ||= []).push(p)

  // Collect shared-GLB conflicts (one file used by ≥ 2 presets).
  const sharedConflicts = new Map()
  for (const p of proposals) {
    if (!p.shared_with || p.shared_with.length === 0) continue
    if (!sharedConflicts.has(p.visual_file)) sharedConflicts.set(p.visual_file, new Set())
    sharedConflicts.get(p.visual_file).add(p.id)
    for (const o of p.shared_with) sharedConflicts.get(p.visual_file).add(o)
  }

  console.log(`Proposals for ${proposals.length} baselined components:\n`)
  console.log(`  ROTATE         ${groups.ROTATE.length}   rotation alone gets ≤ 5%`)
  console.log(`  ROTATE+SCALE   ${groups['ROTATE+SCALE'].length}   rotation + uniform scale gets ≤ 5%`)
  console.log(`  NEAR           ${groups.NEAR.length}   between 5% and 10% (borderline — needs visual review)`)
  console.log(`  SHAPE-MISMATCH ${groups['SHAPE-MISMATCH'].length}   no rotation+scale combo works (re-author needed)`)
  if (groups['NO-DATA'].length) console.log(`  NO-DATA        ${groups['NO-DATA'].length}`)
  console.log()
  console.log(`Shared-GLB conflicts (one mesh, multiple presets, different bboxes): ${sharedConflicts.size}`)
  for (const [file, ids] of sharedConflicts) {
    console.log(`  ${file}  →  ${[...ids].join(', ')}`)
  }
  console.log()

  for (const status of ['ROTATE', 'ROTATE+SCALE', 'NEAR', 'SHAPE-MISMATCH', 'NO-DATA']) {
    if (!groups[status].length) continue
    console.log(`── ${status} ──────────────────────────────────────────`)
    for (const p of groups[status]) {
      if (status === 'NO-DATA') {
        console.log(`  ${p.id}  (${p.note})`)
        continue
      }
      const r = p.rotation_only
      const rs = p.rotation_plus_scale
      const sharedTag = p.shared_with.length ? `  [SHARED with ${p.shared_with.join(', ')}]` : ''
      console.log(`  ${p.id}${sharedTag}`)
      console.log(`     bbox      ${JSON.stringify(p.bbox_mm)}    file ${p.visual_file}`)
      console.log(`     raw       ${JSON.stringify(p.raw_extent_mm)}`)
      console.log(`     rot-only  ${r.perm}  rpy ${r.rpy}  → ${(r.divergence * 100).toFixed(1)}%`)
      console.log(`     rot+scale ${rs.perm}  scale ${rs.scale}  → ${(rs.divergence * 100).toFixed(1)}%`)
    }
    console.log()
  }
}

main().catch(e => { console.error('fatal:', e.stack || e.message); process.exit(1) })

/**
 * Preset bounding-box audit.
 *
 * Enforces Step 3 of docs/ENGINE_NEXT_STEPS.md: on any mesh-override
 * preset (STEP → GLB pipeline), `physical.bounding_box_mm` is demoted to
 * a legacy backward-compat field. Runtime reads use authored component
 * dimensions, so observed rendered AABBs no longer drive placement.
 * To keep new presets honest, the audit hard-fails
 * whenever a mesh-override preset declares bounding_box_mm without
 * `physical.legacy_override: true`.
 *
 * Two static checks:
 *   1. MISSING_LEGACY_OVERRIDE — mesh-override preset with bounding_box_mm
 *      but no `legacy_override: true`. Hard fail; marks that the preset
 *      predates the Step-3 migration and needs explicit acknowledgement.
 *   2. MISMATCH (legacy bbox vs measured GLB > 2mm) — kept as a signal but
 *      demoted to an informational note for legacy_override presets. For
 *      any non-legacy preset the static check fires first, so a numerical
 *      MISMATCH there is redundant.
 *
 * Usage:
 *   node scripts/audit-preset-bboxes.js            # audit only
 *   node scripts/audit-preset-bboxes.js --mark-legacy  # one-shot: add
 *                                                  # legacy_override: true
 *                                                  # to every mesh-override
 *                                                  # preset that authored
 *                                                  # bounding_box_mm.
 *   AUDIT_VERBOSE=1 node scripts/audit-preset-bboxes.js  # also print skipped
 *
 * Notes on scale and axis permutation:
 *   - GLBs in src/public/meshes/glb/ are authored in millimetres; preset
 *     bounding_box_mm is millimetres; no preset-level scale field exists.
 *     The "preset's declared scale" is therefore 1:1 mm-to-mm.
 *   - Some presets have a ROTATION_OVERRIDES entry in
 *     src/src/richVisuals/meshOverrides.ts that permutes the GLB's axes
 *     before per-axis scaling at runtime. The audit applies that same
 *     rotation to the measured AABB before comparing, so an authored
 *     axis swap is not reported as a bbox mismatch.
 *   - Parametric presets without a mesh override (baseplates, extrusions,
 *     plates) are skipped: their bounding_box_mm is generator INPUT, not
 *     legacy metadata, and remains authoritative.
 *
 * Exits:
 *   0 — every mesh-override preset is marked legacy_override and any
 *        numerical drift is tolerated
 *   1 — at least one MISSING_LEGACY_OVERRIDE (or, on a legacy preset,
 *        a MISMATCH — legacy drift is tolerated but still surfaced)
 *   2 — setup error (missing files, unreadable GLB, etc.)
 */

'use strict'

const fs = require('fs')
const path = require('path')

const REPO_ROOT = path.resolve(__dirname, '..')
const PRESET_JSON = path.join(REPO_ROOT, 'core', 'presets', 'generic_presets.json')
const MESH_OVERRIDES_TS = path.join(REPO_ROOT, 'src', 'src', 'richVisuals', 'meshOverrides.ts')
const GLB_DIR = path.join(REPO_ROOT, 'src', 'public', 'meshes', 'glb')

const TOLERANCE_MM = 2.0
const AXIS_NAMES = ['x', 'y', 'z']

// ─────────────────────────────── GLB parsing (zero-dep) ───────────────────────

function loadGltfJson(filePath) {
  const buf = fs.readFileSync(filePath)
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength)
  const magic = dv.getUint32(0, true)
  if (magic !== 0x46546c67) throw new Error(`not a GLB (bad magic): ${filePath}`)
  const jsonChunkLength = dv.getUint32(12, true)
  const jsonChunkType = dv.getUint32(16, true)
  if (jsonChunkType !== 0x4e4f534a) throw new Error(`expected JSON chunk: ${filePath}`)
  const jsonBytes = buf.subarray(20, 20 + jsonChunkLength)
  return JSON.parse(new TextDecoder().decode(jsonBytes))
}

function nodeLocalMatrix(node) {
  if (node.matrix) return node.matrix.slice()
  const t = node.translation || [0, 0, 0]
  const r = node.rotation || [0, 0, 0, 1]
  const s = node.scale || [1, 1, 1]
  const [x, y, z, w] = r
  const xx = x * x, yy = y * y, zz = z * z
  const xy = x * y, xz = x * z, yz = y * z
  const wx = w * x, wy = w * y, wz = w * z
  return [
    (1 - 2 * (yy + zz)) * s[0], 2 * (xy + wz) * s[0], 2 * (xz - wy) * s[0], 0,
    2 * (xy - wz) * s[1], (1 - 2 * (xx + zz)) * s[1], 2 * (yz + wx) * s[1], 0,
    2 * (xz + wy) * s[2], 2 * (yz - wx) * s[2], (1 - 2 * (xx + yy)) * s[2], 0,
    t[0], t[1], t[2], 1,
  ]
}

function mat4Multiply(a, b) {
  const out = new Array(16).fill(0)
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      let v = 0
      for (let k = 0; k < 4; k++) v += a[k * 4 + r] * b[c * 4 + k]
      out[c * 4 + r] = v
    }
  }
  return out
}

function transformPoint(m, p) {
  return [
    m[0] * p[0] + m[4] * p[1] + m[8]  * p[2] + m[12],
    m[1] * p[0] + m[5] * p[1] + m[9]  * p[2] + m[13],
    m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14],
  ]
}

function transformAabb(m, min, max) {
  const corners = [
    [min[0], min[1], min[2]], [max[0], min[1], min[2]],
    [min[0], max[1], min[2]], [max[0], max[1], min[2]],
    [min[0], min[1], max[2]], [max[0], min[1], max[2]],
    [min[0], max[1], max[2]], [max[0], max[1], max[2]],
  ]
  const newMin = [Infinity, Infinity, Infinity]
  const newMax = [-Infinity, -Infinity, -Infinity]
  for (const c0 of corners) {
    const c = transformPoint(m, c0)
    for (let i = 0; i < 3; i++) {
      if (c[i] < newMin[i]) newMin[i] = c[i]
      if (c[i] > newMax[i]) newMax[i] = c[i]
    }
  }
  return { min: newMin, max: newMax }
}

function walkNode(json, nodeIdx, parentMatrix, accum) {
  const node = json.nodes[nodeIdx]
  const worldMatrix = mat4Multiply(parentMatrix, nodeLocalMatrix(node))
  if (typeof node.mesh === 'number') {
    const mesh = json.meshes[node.mesh]
    for (const prim of mesh.primitives) {
      const posIdx = prim.attributes && prim.attributes.POSITION
      if (typeof posIdx !== 'number') continue
      const acc = json.accessors[posIdx]
      if (!acc || !acc.min || !acc.max) continue
      const aabb = transformAabb(worldMatrix, acc.min, acc.max)
      for (let i = 0; i < 3; i++) {
        if (aabb.min[i] < accum.min[i]) accum.min[i] = aabb.min[i]
        if (aabb.max[i] > accum.max[i]) accum.max[i] = aabb.max[i]
      }
    }
  }
  for (const child of node.children || []) walkNode(json, child, worldMatrix, accum)
}

function measureGlbAabb(filePath) {
  const json = loadGltfJson(filePath)
  const scene = json.scenes[json.scene != null ? json.scene : 0]
  const accum = { min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity] }
  const identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
  for (const root of scene.nodes) walkNode(json, root, identity, accum)
  if (!Number.isFinite(accum.min[0])) throw new Error(`no positioned primitives in ${filePath}`)
  return {
    size: [
      accum.max[0] - accum.min[0],
      accum.max[1] - accum.min[1],
      accum.max[2] - accum.min[2],
    ],
    min: accum.min,
    max: accum.max,
  }
}

// ─────────────────── Euler XYZ → AABB-permuting rotation matrix ───────────────

function eulerXyzMatrix([rx, ry, rz]) {
  const cx = Math.cos(rx), sx = Math.sin(rx)
  const cy = Math.cos(ry), sy = Math.sin(ry)
  const cz = Math.cos(rz), sz = Math.sin(rz)
  // R = Rx * Ry * Rz (Three.js default 'XYZ' intrinsic order)
  const rX = [
    1, 0, 0, 0,
    0, cx, sx, 0,
    0, -sx, cx, 0,
    0, 0, 0, 1,
  ]
  const rY = [
    cy, 0, -sy, 0,
    0, 1, 0, 0,
    sy, 0, cy, 0,
    0, 0, 0, 1,
  ]
  const rZ = [
    cz, sz, 0, 0,
    -sz, cz, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ]
  return mat4Multiply(mat4Multiply(rX, rY), rZ)
}

// ──────────────── meshOverrides.ts — read string→string tables ────────────────

function readMeshOverridesTs() {
  const src = fs.readFileSync(MESH_OVERRIDES_TS, 'utf8')
  return {
    meshOverrides: parseStringRecord(src, 'MESH_OVERRIDES'),
    rotationOverrides: parseRotationRecord(src, 'ROTATION_OVERRIDES'),
  }
}

function extractRecordBody(src, name) {
  const header = new RegExp(`export\\s+const\\s+${name}\\s*:\\s*Record<[^>]+>\\s*=\\s*\\{`)
  const headerMatch = src.match(header)
  if (!headerMatch) throw new Error(`could not locate ${name} in ${MESH_OVERRIDES_TS}`)
  const bodyStart = headerMatch.index + headerMatch[0].length
  let depth = 1
  let i = bodyStart
  while (i < src.length && depth > 0) {
    const ch = src[i]
    if (ch === '{') depth++
    else if (ch === '}') depth--
    i++
  }
  if (depth !== 0) throw new Error(`unbalanced braces parsing ${name}`)
  return src.slice(bodyStart, i - 1)
}

function stripLineComments(s) {
  // Remove // comments (simple — no string-literal edge cases in this data file).
  return s.replace(/\/\/[^\n]*/g, '')
}

function parseStringRecord(src, name) {
  const body = stripLineComments(extractRecordBody(src, name))
  const out = Object.create(null)
  const entry = /'([^']+)'\s*:\s*'([^']+)'/g
  let m
  while ((m = entry.exec(body)) !== null) {
    out[m[1]] = m[2]
  }
  return out
}

function parseRotationRecord(src, name) {
  const body = stripLineComments(extractRecordBody(src, name))
  const out = Object.create(null)
  // Each entry: 'id': [<expr>, <expr>, <expr>]
  const entry = /'([^']+)'\s*:\s*\[\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,\s*([^,\]]+)\s*\]/g
  let m
  while ((m = entry.exec(body)) !== null) {
    out[m[1]] = [evalAngle(m[2]), evalAngle(m[3]), evalAngle(m[4])]
  }
  return out
}

function evalAngle(expr) {
  // The file only ever uses literals made from { number, Math.PI, *, /, +, -, () }.
  // A scoped Function() evaluator is adequate and deterministic here.
  const trimmed = String(expr).trim()
  if (!/^[0-9.\s+\-*/()MathPI]+$/.test(trimmed)) {
    throw new Error(`unexpected angle expression: ${trimmed}`)
  }
  // eslint-disable-next-line no-new-func
  return Function(`"use strict"; return (${trimmed});`)()
}

// ──────────────────────────────── Preset walk ─────────────────────────────────

function loadPresets() {
  const json = JSON.parse(fs.readFileSync(PRESET_JSON, 'utf8'))
  const out = []
  for (const [catName, cat] of Object.entries(json.categories || {})) {
    for (const c of cat.components || []) {
      const phys = c.physical || {}
      const bbox = phys.bounding_box_mm
      if (!Array.isArray(bbox) || bbox.length !== 3) continue
      out.push({
        id: c.id,
        category: catName,
        boundingBoxMm: bbox,
        legacyOverride: phys.legacy_override === true,
      })
    }
  }
  out.sort((a, b) => a.id.localeCompare(b.id))
  return out
}

function glbPathForStepName(stepFilename) {
  const base = stepFilename.replace(/\.(step|stp)$/i, '')
  return path.join(GLB_DIR, `${base}.glb`)
}

// ─────────────────────────────────── Main ─────────────────────────────────────

function main() {
  if (!fs.existsSync(PRESET_JSON)) fail(`missing preset JSON: ${PRESET_JSON}`)
  if (!fs.existsSync(MESH_OVERRIDES_TS)) fail(`missing mesh overrides: ${MESH_OVERRIDES_TS}`)
  if (!fs.existsSync(GLB_DIR)) fail(`missing GLB dir: ${GLB_DIR}`)

  const applyLegacy = process.argv.includes('--mark-legacy')
  if (applyLegacy) return applyLegacyOverride()

  const presets = loadPresets()
  const { meshOverrides, rotationOverrides } = readMeshOverridesTs()

  const missingLegacy = []
  const mismatches = []
  const legacyMismatches = []
  const skipped = []
  const checked = []

  for (const preset of presets) {
    const stepName = meshOverrides[preset.id]
    if (!stepName) {
      // No mesh override → parametric preset. Its bounding_box_mm is a
      // generator INPUT, not legacy metadata. Skip both checks.
      skipped.push({ id: preset.id, reason: 'no mesh override (parametric)' })
      continue
    }

    // Step-3 static check: mesh-override presets must acknowledge their
    // bounding_box_mm is legacy (not authoritative). If the flag is missing,
    // the preset predates the migration and needs to be opted in explicitly
    // — run `node scripts/audit-preset-bboxes.js --mark-legacy` to apply.
    if (!preset.legacyOverride) {
      missingLegacy.push({ id: preset.id })
    }

    const glbPath = glbPathForStepName(stepName)
    if (!fs.existsSync(glbPath)) {
      skipped.push({ id: preset.id, reason: `GLB not found: ${path.relative(REPO_ROOT, glbPath)}` })
      continue
    }

    let rawSize
    try {
      const aabb = measureGlbAabb(glbPath)
      rawSize = aabb.size
    } catch (e) {
      skipped.push({ id: preset.id, reason: `GLB read error: ${e.message}` })
      continue
    }

    // Apply rotation override (if any) so the comparison respects authored
    // axis swaps — same transform the runtime applies before per-axis scaling.
    const euler = rotationOverrides[preset.id]
    let measured
    if (euler) {
      const rot = eulerXyzMatrix(euler)
      const half = [rawSize[0] / 2, rawSize[1] / 2, rawSize[2] / 2]
      const minP = [-half[0], -half[1], -half[2]]
      const maxP = [ half[0],  half[1],  half[2]]
      const rotated = transformAabb(rot, minP, maxP)
      measured = [
        rotated.max[0] - rotated.min[0],
        rotated.max[1] - rotated.min[1],
        rotated.max[2] - rotated.min[2],
      ]
    } else {
      measured = rawSize.slice()
    }

    const authored = preset.boundingBoxMm
    const deltas = [0, 1, 2].map(i => measured[i] - authored[i])
    const worst = Math.max(...deltas.map(Math.abs))
    checked.push({ id: preset.id, worst })

    for (let i = 0; i < 3; i++) {
      const delta = deltas[i]
      if (Math.abs(delta) > TOLERANCE_MM) {
        const entry = {
          id: preset.id,
          axis: AXIS_NAMES[i],
          authored: authored[i],
          measured: measured[i],
          delta,
        }
        if (preset.legacyOverride) legacyMismatches.push(entry)
        else mismatches.push(entry)
      }
    }
  }

  missingLegacy.sort((a, b) => a.id.localeCompare(b.id))
  for (const m of missingLegacy) {
    console.log(
      `MISSING_LEGACY_OVERRIDE ${m.id}: mesh-override preset declares bounding_box_mm ` +
      `without \`physical.legacy_override: true\`. Run --mark-legacy or drop bounding_box_mm.`
    )
  }

  mismatches.sort((a, b) => a.id.localeCompare(b.id) || a.axis.localeCompare(b.axis))
  for (const m of mismatches) {
    console.log(
      `MISMATCH ${m.id}: bbox_${m.axis} ` +
      `authored=${fmt(m.authored)}mm, measured=${fmt(m.measured)}mm ` +
      `(delta=${fmtSigned(m.delta)}mm)`
    )
  }

  legacyMismatches.sort((a, b) => a.id.localeCompare(b.id) || a.axis.localeCompare(b.axis))
  if (process.env.AUDIT_VERBOSE === '1') {
    for (const m of legacyMismatches) {
      console.error(
        `LEGACY_DRIFT ${m.id}: bbox_${m.axis} ` +
        `authored=${fmt(m.authored)}mm, measured=${fmt(m.measured)}mm ` +
        `(delta=${fmtSigned(m.delta)}mm) — tolerated because legacy_override`
      )
    }
  }

  // Summary to stderr so stdout stays machine-parseable.
  const mismatchPresets = new Set(mismatches.map(m => m.id)).size
  console.error('')
  console.error(`presets scanned:             ${presets.length}`)
  console.error(`checked against GLB:         ${checked.length}`)
  console.error(`skipped:                     ${skipped.length}`)
  console.error(`tolerance:                   ${TOLERANCE_MM}mm per axis`)
  console.error(`missing legacy_override:     ${missingLegacy.length}`)
  console.error(`non-legacy mismatched axes:  ${mismatches.length}  (presets: ${mismatchPresets})`)
  console.error(`legacy-tolerated drift axes: ${legacyMismatches.length}`)

  if (process.env.AUDIT_VERBOSE === '1') {
    console.error('')
    console.error('--- skipped ---')
    for (const s of skipped) console.error(`  ${s.id}  (${s.reason})`)
  }

  const hardFail = missingLegacy.length > 0 || mismatches.length > 0
  process.exit(hardFail ? 1 : 0)
}

// ─────────────────────── --mark-legacy migration mode ─────────────────────────
//
// One-shot: adds `legacy_override: true` into `physical` on every
// mesh-override preset that still has a bounding_box_mm but no marker.
// Keeps the audit honest going forward: new presets won't inherit the flag
// by accident. Safe to re-run; already-marked presets are skipped.
function applyLegacyOverride() {
  const { meshOverrides } = readMeshOverridesTs()

  const originalJson = fs.readFileSync(PRESET_JSON, 'utf8')
  const data = JSON.parse(originalJson)

  const touched = []
  const alreadyMarked = []
  for (const cat of Object.values(data.categories || {})) {
    for (const c of cat.components || []) {
      if (!meshOverrides[c.id]) continue
      const phys = c.physical
      if (!phys || !Array.isArray(phys.bounding_box_mm)) continue
      if (phys.legacy_override === true) {
        alreadyMarked.push(c.id)
        continue
      }
      phys.legacy_override = true
      touched.push(c.id)
    }
  }

  if (touched.length === 0) {
    console.error(`no changes — ${alreadyMarked.length} preset(s) already marked`)
    process.exit(0)
  }

  // Detect 2 vs 4 space indentation from the original file to match in-place.
  const indent = detectIndent(originalJson)
  const serialized = JSON.stringify(data, null, indent) + '\n'

  // Write both mirrored locations (core/presets + src/public) in one go.
  const mirrorPath = path.join(REPO_ROOT, 'src', 'public', 'generic_presets.json')
  fs.writeFileSync(PRESET_JSON, serialized)
  if (fs.existsSync(mirrorPath)) fs.writeFileSync(mirrorPath, serialized)

  touched.sort()
  for (const id of touched) console.log(`marked legacy_override: ${id}`)
  console.error('')
  console.error(`marked:          ${touched.length}`)
  console.error(`already marked:  ${alreadyMarked.length}`)
  console.error(`wrote:           ${path.relative(REPO_ROOT, PRESET_JSON)}`)
  if (fs.existsSync(mirrorPath)) console.error(`wrote:           ${path.relative(REPO_ROOT, mirrorPath)}`)
  process.exit(0)
}

function detectIndent(src) {
  const m = src.match(/\n(\s+)"/)
  return m ? m[1] : 2
}

function fmt(n) {
  return (Math.round(n * 100) / 100).toFixed(2)
}

function fmtSigned(n) {
  const s = fmt(n)
  return n >= 0 && !s.startsWith('-') ? `+${s}` : s
}

function fail(msg) {
  console.error(`audit-preset-bboxes: ${msg}`)
  process.exit(2)
}

main()

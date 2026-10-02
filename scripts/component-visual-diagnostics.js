/**
 * Component visual diagnostics.
 *
 * Phase 1: inspect the
 * current visual stack without changing runtime behavior.
 *
 * Usage:
 * node scripts/component-visual-diagnostics.js
 * node scripts/component-visual-diagnostics.js --all
 * node scripts/component-visual-diagnostics.js --json
 * node scripts/component-visual-diagnostics.js --strict
 *
 * The report compares:
 * - preset bounding_box_mm / cross_section_mm
 * - rich visual generator bounds
 * - mesh override asset presence
 * - raw GLB bounds and post-rotation bounds
 * - current per-axis scaling factors
 */

'use strict'

const fs = require('fs')
const path = require('path')

process.stdout.on('error', err => {
  if (err && err.code === 'EPIPE') process.exit(0)
  throw err
})

const REPO_ROOT = path.resolve(__dirname, '..')
const SRC_ROOT = path.join(REPO_ROOT, 'src')
const PRESET_JSON = path.join(REPO_ROOT, 'core', 'presets', 'generic_presets.json')
const PUBLIC_PRESET_JSON = path.join(SRC_ROOT, 'public', 'generic_presets.json')
// Source CAD (STEP/STP) lives outside public/: only the converted GLBs ship.
const STEP_DIR = path.join(REPO_ROOT, 'assets', 'step')
const GLB_DIR = path.join(SRC_ROOT, 'public', 'meshes', 'glb')

const AXIS_NAMES = ['x', 'y', 'z']
const BOUNDS_TOLERANCE_MM = 2
const NON_UNIFORM_WARN_RATIO = Number(process.env.VISUAL_DIAG_NON_UNIFORM_RATIO || '1.75')
const LARGE_SCALE_WARN = Number(process.env.VISUAL_DIAG_LARGE_SCALE || '2.5')
const TINY_SCALE_WARN = Number(process.env.VISUAL_DIAG_TINY_SCALE || '0.4')

const args = new Set(process.argv.slice(2))
const SHOW_ALL = args.has('--all')
const JSON_OUT = args.has('--json')
const STRICT = args.has('--strict')

registerTypeScriptRequire()

const THREE = require(path.join(SRC_ROOT, 'node_modules', 'three'))
const { findRichGenerator } = require(path.join(SRC_ROOT, 'src', 'richVisuals', 'generators', 'index.ts'))
const { getComponentColor } = require(path.join(SRC_ROOT, 'src', 'richVisuals', 'materials.ts'))
const {
  MESH_OVERRIDES,
  ROTATION_OVERRIDES,
  getMeshVisualMetadata,
  getRotationOverride,
  getShaftOverlay,
} = require(path.join(SRC_ROOT, 'src', 'richVisuals', 'meshOverrides.ts'))

function registerTypeScriptRequire() {
  const ts = require(path.join(SRC_ROOT, 'node_modules', 'typescript'))
  require.extensions['.ts'] = function loadTs(module, filename) {
    const source = fs.readFileSync(filename, 'utf8')
    const out = ts.transpileModule(source, {
      compilerOptions: {
        module: ts.ModuleKind.CommonJS,
        target: ts.ScriptTarget.ES2022,
        esModuleInterop: true,
      },
    })
    module._compile(out.outputText, filename)
  }
}

function main() {
  if (!fs.existsSync(PRESET_JSON)) fail(`missing preset JSON: ${PRESET_JSON}`)
  if (!fs.existsSync(GLB_DIR)) fail(`missing GLB dir: ${GLB_DIR}`)
  if (!fs.existsSync(STEP_DIR)) fail(`missing source STEP dir: ${STEP_DIR}`)

  const presets = loadPresets(PRESET_JSON)
  const publicMirrorMatches = comparePresetMirrors()
  const rows = presets.map(analyzePreset)
  const findings = collectFindings(rows)

  if (JSON_OUT) {
    console.log(JSON.stringify({
      summary: summarize(rows, findings, publicMirrorMatches),
      findings,
      rows: SHOW_ALL ? rows : rows.filter(r => r.findings.length > 0),
    }, null, 2))
  } else {
    printTextReport(rows, findings, publicMirrorMatches)
  }

  if (STRICT && findings.length > 0) process.exit(1)
}

function loadPresets(filePath) {
  const data = JSON.parse(fs.readFileSync(filePath, 'utf8'))
  const out = []
  for (const [category, cat] of Object.entries(data.categories || {})) {
    for (const component of cat.components || []) {
      out.push({ category, component })
    }
  }
  out.sort((a, b) => a.component.id.localeCompare(b.component.id))
  return out
}

function comparePresetMirrors() {
  if (!fs.existsSync(PUBLIC_PRESET_JSON)) return false
  return fs.readFileSync(PRESET_JSON, 'utf8') === fs.readFileSync(PUBLIC_PRESET_JSON, 'utf8')
}

function analyzePreset({ category, component }) {
  const id = component.id
  const bboxMm = effectiveBboxMm(component)
  const meshFilename = MESH_OVERRIDES[id] || null
  const glbPath = meshFilename ? glbPathForMesh(meshFilename) : null
  const stepPath = meshFilename ? path.join(STEP_DIR, meshFilename) : null
  const glbExists = !!glbPath && fs.existsSync(glbPath)
  const stepExists = !!stepPath && fs.existsSync(stepPath)
  const rotation = getRotationOverride(id)
  const shaftOverlay = getShaftOverlay(id)
  const scalePolicy = getMeshVisualMetadata(id)?.scalePolicy ?? 'per-axis'
  const collisionMesh = component.physical?.collision_mesh || null
  const collisionSource = collisionMesh ? 'authored_mesh' : 'bbox'

  const rich = measureRichBoundsMm(id, bboxMm)

  let glb = null
  if (glbExists) {
    try {
      const raw = measureGlbAabb(glbPath)
      const rawSizeMm = normalizeGlbSizeMm(raw.size, bboxMm)
      const rotatedSizeMm = rotation ? rotateAabbSize(rawSizeMm, rotation) : rawSizeMm.slice()
      const targetForScale = bboxMm.slice()
      if (shaftOverlay) targetForScale[2] = Math.max(1, targetForScale[2] - shaftOverlay.shaft_length_mm)
      const perAxis = targetForScale.map((v, i) => rotatedSizeMm[i] > 0.0001 ? v / rotatedSizeMm[i] : 1)
      const scaleFactors = scalePolicy === 'uniform' ? perAxis.map(() => Math.min(...perAxis)) : perAxis
      glb = {
        rawBoundsMm: rawSizeMm,
        postRotationBoundsMm: rotatedSizeMm,
        units: raw.units,
        rotation,
        scalePolicy,
        scaleFactors,
        nonUniformRatio: ratio(scaleFactors),
      }
    } catch (e) {
      glb = { error: e.message }
    }
  }

  const source = resolveCurrentSource({ meshFilename, glbExists, rich })
  const resolvedBoundsMm =
    source === 'mesh' && glb && !glb.error ? glb.scaleFactors.map((f, i) => glb.postRotationBoundsMm[i] * f) :
    source === 'rich' && rich.boundsMm ? rich.boundsMm :
    bboxMm

  const row = {
    id,
    category,
    presetBoundsMm: bboxMm,
    richGeneratorBoundsMm: rich.boundsMm,
    richGeneratorError: rich.error,
    meshOverride: meshFilename,
    glbExists,
    stepExists,
    collisionSource,
    collisionMesh,
    currentSource: source,
    resolvedBoundsMm,
    glb,
    findings: [],
  }
  row.findings = findIssues(row)
  return row
}

function effectiveBboxMm(component) {
  const phys = component.physical || {}
  const bb = phys.bounding_box_mm || phys.cross_section_mm || [40, 40, 40]
  return [bb[0] || 40, bb[1] || 40, bb[2] || 40]
}

function resolveCurrentSource({ meshFilename, glbExists, rich }) {
  if (meshFilename && glbExists) return 'mesh'
  if (rich.boundsMm) return 'rich'
  return 'box'
}

function measureRichBoundsMm(id, bboxMm) {
  const generator = findRichGenerator(id)
  if (!generator) return { boundsMm: null, error: null }
  try {
    const dims = { x: bboxMm[0] / 1000, y: bboxMm[1] / 1000, z: bboxMm[2] / 1000 }
    const color = getComponentColor(id)
    const group = generator(id, dims, color.tint)
    group.updateMatrixWorld(true)
    const box = new THREE.Box3().setFromObject(group)
    const size = new THREE.Vector3()
    box.getSize(size)
    if (size.x <= 0 && size.y <= 0 && size.z <= 0) return { boundsMm: null, error: 'empty generator bounds' }
    return { boundsMm: [size.x * 1000, size.y * 1000, size.z * 1000], error: null }
  } catch (e) {
    return { boundsMm: null, error: e.message }
  }
}

function glbPathForMesh(filename) {
  const base = filename.replace(/\.(step|stp)$/i, '')
  return path.join(GLB_DIR, `${base}.glb`)
}

function loadGlbJson(filePath) {
  const buf = fs.readFileSync(filePath)
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength)
  const magic = dv.getUint32(0, true)
  if (magic !== 0x46546c67) throw new Error('not a GLB')
  const jsonChunkLength = dv.getUint32(12, true)
  const jsonChunkType = dv.getUint32(16, true)
  if (jsonChunkType !== 0x4e4f534a) throw new Error('expected JSON chunk')
  return JSON.parse(new TextDecoder().decode(buf.subarray(20, 20 + jsonChunkLength)))
}

function measureGlbAabb(filePath) {
  const json = loadGlbJson(filePath)
  const scene = json.scenes?.[json.scene ?? 0]
  if (!scene) throw new Error('missing default scene')
  const accum = emptyBounds()
  const identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
  for (const root of scene.nodes || []) walkGltfNode(json, root, identity, accum)
  if (!Number.isFinite(accum.min[0])) throw new Error('no POSITION accessor bounds')
  const size = sizeOfBounds(accum)
  const maxDim = Math.max(...size)
  return { size, min: accum.min, max: accum.max, units: maxDim < 2 ? 'm-or-unitless' : 'mm' }
}

function walkGltfNode(json, nodeIdx, parentMatrix, accum) {
  const node = json.nodes[nodeIdx]
  const world = mat4Multiply(parentMatrix, nodeLocalMatrix(node))
  if (typeof node.mesh === 'number') {
    const mesh = json.meshes[node.mesh]
    for (const prim of mesh.primitives || []) {
      const posIdx = prim.attributes && prim.attributes.POSITION
      if (typeof posIdx !== 'number') continue
      const acc = json.accessors[posIdx]
      if (!acc?.min || !acc?.max) continue
      expandBounds(accum, transformAabb({ min: acc.min, max: acc.max }, world))
    }
  }
  for (const child of node.children || []) walkGltfNode(json, child, world, accum)
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

function transformAabb(bounds, matrix) {
  const corners = aabbCorners(bounds.min, bounds.max)
  const out = emptyBounds()
  for (const c of corners) expandPoint(out, transformPoint(matrix, c))
  return out
}

function transformAabbByThreeMatrix(bounds, matrix) {
  const out = emptyBounds()
  for (const c of aabbCorners(bounds.min, bounds.max)) {
    const v = new THREE.Vector3(c[0], c[1], c[2]).applyMatrix4(matrix)
    expandPoint(out, [v.x, v.y, v.z])
  }
  return out
}

function aabbCorners(min, max) {
  return [
    [min[0], min[1], min[2]], [max[0], min[1], min[2]],
    [min[0], max[1], min[2]], [max[0], max[1], min[2]],
    [min[0], min[1], max[2]], [max[0], min[1], max[2]],
    [min[0], max[1], max[2]], [max[0], max[1], max[2]],
  ]
}

function rpyMatrix([r, p, y]) {
  return new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler(r, p, y, 'XYZ'))
}

function rotateAabbSize(size, rotation) {
  const half = size.map(v => v / 2)
  const matrix = rpyMatrix(rotation)
  const rotated = transformAabbByThreeMatrix(
    { min: [-half[0], -half[1], -half[2]], max: [half[0], half[1], half[2]] },
    matrix,
  )
  return sizeOfBounds(rotated)
}

function normalizeGlbSizeMm(size, presetBoundsMm) {
  const maxRaw = Math.max(...size)
  const maxPreset = Math.max(...presetBoundsMm)
  if (maxRaw < 2 && maxPreset > 10) return size.map(v => v * 1000)
  return size
}

function findIssues(row) {
  const issues = []
  if (row.meshOverride) {
    if (!row.glbExists) issues.push({ code: 'MISSING_GLB', detail: row.meshOverride.replace(/\.(step|stp)$/i, '.glb') })
    if (!row.stepExists) issues.push({ code: 'MISSING_SOURCE_STEP', detail: row.meshOverride })
    if (!ROTATION_OVERRIDES[row.id]) issues.push({ code: 'NO_ROTATION_METADATA', detail: 'mesh override has no explicit rotation entry' })
  }
  if (row.richGeneratorError) issues.push({ code: 'RICH_GENERATOR_ERROR', detail: row.richGeneratorError })
  if (row.glb?.error) issues.push({ code: 'GLB_READ_ERROR', detail: row.glb.error })
  if (row.glb && !row.glb.error && row.glb.scalePolicy === 'per-axis') {
    const sf = row.glb.scaleFactors
    if (row.glb.nonUniformRatio > NON_UNIFORM_WARN_RATIO) {
      issues.push({ code: 'NON_UNIFORM_SCALE', detail: `ratio=${fmt(row.glb.nonUniformRatio)} scale=[${fmtList(sf)}]` })
    }
    if (sf.some(v => v > LARGE_SCALE_WARN || v < TINY_SCALE_WARN)) {
      issues.push({ code: 'LARGE_SCALE_FACTOR', detail: `scale=[${fmtList(sf)}]` })
    }
  }
  addBoundsIssue(issues, 'RICH_VS_PRESET_BOUNDS', row.richGeneratorBoundsMm, row.presetBoundsMm)
  if (row.glb && !row.glb.error) addBoundsIssue(issues, 'GLB_POST_ROTATION_VS_PRESET_BOUNDS', row.glb.postRotationBoundsMm, row.presetBoundsMm)
  return issues
}

function addBoundsIssue(issues, code, observed, expected) {
  if (!observed || !expected) return
  const deltas = observed.map((v, i) => v - expected[i])
  const worst = Math.max(...deltas.map(Math.abs))
  if (worst <= BOUNDS_TOLERANCE_MM) return
  const parts = deltas
    .map((d, i) => Math.abs(d) > BOUNDS_TOLERANCE_MM ? `${AXIS_NAMES[i]}=${fmtSigned(d)}mm` : null)
    .filter(Boolean)
    .join(' ')
  issues.push({ code, detail: parts })
}

function collectFindings(rows) {
  const out = []
  for (const row of rows) {
    for (const issue of row.findings) out.push({ id: row.id, category: row.category, ...issue })
  }
  return out
}

function summarize(rows, findings, publicMirrorMatches) {
  const byCode = {}
  for (const f of findings) byCode[f.code] = (byCode[f.code] || 0) + 1
  return {
    presets: rows.length,
    meshOverrides: rows.filter(r => r.meshOverride).length,
    glbPresent: rows.filter(r => r.glbExists).length,
    sourceStepPresent: rows.filter(r => r.stepExists).length,
    collisionSources: countBy(rows, r => r.collisionSource),
    publicPresetMirrorMatches: publicMirrorMatches,
    findings: byCode,
  }
}

function countBy(items, keyFn) {
  const out = {}
  for (const item of items) {
    const key = keyFn(item)
    out[key] = (out[key] || 0) + 1
  }
  return out
}

function printTextReport(rows, findings, publicMirrorMatches) {
  const summary = summarize(rows, findings, publicMirrorMatches)
  console.log('Component visual diagnostics')
  console.log('')
  console.log(`presets:                    ${summary.presets}`)
  console.log(`mesh overrides:             ${summary.meshOverrides}`)
  console.log(`GLB files present:          ${summary.glbPresent}`)
  console.log(`source STEP/STP present:    ${summary.sourceStepPresent}`)
  console.log(`collision authored meshes:  ${summary.collisionSources.authored_mesh || 0}`)
  console.log(`no collision mesh (bbox):     ${summary.collisionSources.bbox || 0}`)
  console.log(`preset mirror matches:      ${summary.publicPresetMirrorMatches ? 'yes' : 'no'}`)
  console.log(`bounds tolerance:           ${BOUNDS_TOLERANCE_MM}mm`)
  console.log(`non-uniform warn ratio:     ${NON_UNIFORM_WARN_RATIO}`)
  console.log('')

  const grouped = new Map()
  for (const f of findings) {
    if (!SHOW_ALL && f.code === 'NO_ROTATION_METADATA') continue
    const list = grouped.get(f.code) || []
    list.push(f)
    grouped.set(f.code, list)
  }

  if (grouped.size === 0) {
    console.log(SHOW_ALL ? 'No findings.' : 'No findings after default noise filter. Use --all to include metadata inventory.')
  } else {
    for (const [code, list] of Array.from(grouped.entries()).sort()) {
      console.log(`${code} (${list.length})`)
      for (const f of list) console.log(`  ${f.id}: ${f.detail}`)
      console.log('')
    }
  }

  if (!SHOW_ALL) {
    const hidden = findings.filter(f => f.code === 'NO_ROTATION_METADATA').length
    if (hidden > 0) console.log(`Hidden metadata inventory: ${hidden} NO_ROTATION_METADATA findings. Re-run with --all to list them.`)
  }
}

function emptyBounds() {
  return { min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity] }
}

function expandPoint(bounds, p) {
  for (let i = 0; i < 3; i++) {
    if (p[i] < bounds.min[i]) bounds.min[i] = p[i]
    if (p[i] > bounds.max[i]) bounds.max[i] = p[i]
  }
}

function expandBounds(bounds, other) {
  expandPoint(bounds, other.min)
  expandPoint(bounds, other.max)
}

function sizeOfBounds(bounds) {
  return [
    bounds.max[0] - bounds.min[0],
    bounds.max[1] - bounds.min[1],
    bounds.max[2] - bounds.min[2],
  ]
}

function ratio(values) {
  const positive = values.filter(v => Number.isFinite(v) && v > 0.000001)
  if (positive.length === 0) return 1
  return Math.max(...positive) / Math.min(...positive)
}

function fmt(n) {
  return (Math.round(n * 100) / 100).toFixed(2)
}

function fmtSigned(n) {
  const s = fmt(n)
  return n >= 0 && !s.startsWith('-') ? `+${s}` : s
}

function fmtList(values) {
  return values.map(fmt).join(',')
}

function fail(msg) {
  console.error(`component-visual-diagnostics: ${msg}`)
  process.exit(2)
}

main()

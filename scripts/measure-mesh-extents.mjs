// Measure post-rotation AABB for every authored visual GLB and collision OBJ.
//
// Phase 1 of docs/COMPONENT_UNIFICATION_PLAN.md: every consumer that asks
// "how big is this collision shape, really?" must read a number computed
// once at build time, not infer it from the preset bbox or from the live
// rendered scene. This script writes that number into
// src/public/meshExtents.generated.json, which the resolver loads at runtime
// for collision.bounds and bbox-vs-mesh divergence warnings.
//
// Usage:
//   cd src && node ../scripts/measure-mesh-extents.mjs
//
// Reuses the GLB loader / rotation parsing from generate-collision-meshes.mjs.

import fs from 'node:fs/promises'
import path from 'node:path'
import { createHash } from 'node:crypto'
import { fileURLToPath } from 'node:url'

const here = path.dirname(fileURLToPath(import.meta.url))
const repoRoot = path.resolve(here, '..')
const GLB_DIR = path.join(repoRoot, 'src', 'public', 'meshes', 'glb')
const COLLISION_DIR = path.join(repoRoot, 'src', 'public', 'meshes', 'collision')
const PUBLIC_PRESETS = path.join(repoRoot, 'src', 'public', 'generic_presets.json')
const MESH_OVERRIDES_TS = path.join(repoRoot, 'src', 'src', 'richVisuals', 'meshOverrides.ts')
const OUT_FILE = path.join(repoRoot, 'src', 'public', 'meshExtents.generated.json')

// ─────────────────────────── GLB binary helpers ──────────────────────────────

const COMPONENT_TYPE_FLOAT = 5126
const COMPONENT_BYTE_SIZE = { 5120: 1, 5121: 1, 5122: 2, 5123: 2, 5125: 4, 5126: 4 }
const TYPE_NUM_COMPONENTS = { SCALAR: 1, VEC2: 2, VEC3: 3, VEC4: 4, MAT2: 4, MAT3: 9, MAT4: 16 }

// Phase 7b: short content-addressable hash so a mesh swap that preserves the
// AABB still shows up as a diff in meshExtents.generated.json (and therefore
// in `git status`). Not gated — it's a visibility signal, not enforcement.
function shortContentHash(buf) {
  return createHash('sha256').update(buf).digest('hex').slice(0, 16)
}

async function loadGlb(filePath) {
  const buf = await fs.readFile(filePath)
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength)
  if (dv.getUint32(0, true) !== 0x46546c67) throw new Error('not a GLB')
  const jsonLen = dv.getUint32(12, true)
  if (dv.getUint32(16, true) !== 0x4e4f534a) throw new Error('expected JSON chunk')
  const json = JSON.parse(new TextDecoder().decode(buf.subarray(20, 20 + jsonLen)))
  const binOffset = 20 + jsonLen
  if (binOffset + 8 > buf.byteLength) throw new Error('missing BIN chunk')
  if (dv.getUint32(binOffset + 4, true) !== 0x004e4942) throw new Error('expected BIN chunk')
  const binLen = dv.getUint32(binOffset, true)
  return { json, bin: buf.subarray(binOffset + 8, binOffset + 8 + binLen) }
}

function readAccessorVec3(json, bin, accessorIdx) {
  const acc = json.accessors[accessorIdx]
  if (acc.type !== 'VEC3') throw new Error(`accessor ${accessorIdx} not VEC3`)
  if (acc.componentType !== COMPONENT_TYPE_FLOAT) {
    throw new Error(`accessor ${accessorIdx} not FLOAT (got ${acc.componentType})`)
  }
  const view = json.bufferViews[acc.bufferView]
  const elementSize = COMPONENT_BYTE_SIZE[acc.componentType] * TYPE_NUM_COMPONENTS[acc.type]
  const stride = view.byteStride || elementSize
  const baseOffset = (view.byteOffset || 0) + (acc.byteOffset || 0)
  const out = new Array(acc.count)
  const dv = new DataView(bin.buffer, bin.byteOffset, bin.byteLength)
  for (let i = 0; i < acc.count; i++) {
    const o = baseOffset + i * stride
    out[i] = [dv.getFloat32(o, true), dv.getFloat32(o + 4, true), dv.getFloat32(o + 8, true)]
  }
  return out
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
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
    let v = 0
    for (let k = 0; k < 4; k++) v += a[k * 4 + r] * b[c * 4 + k]
    out[c * 4 + r] = v
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

function rotationMatrixXyz(r) {
  const [x, y, z] = r
  const cx = Math.cos(x), sx = Math.sin(x)
  const cy = Math.cos(y), sy = Math.sin(y)
  const cz = Math.cos(z), sz = Math.sin(z)
  const rX = [1, 0, 0, 0,  0, cx, sx, 0,  0, -sx, cx, 0,  0, 0, 0, 1]
  const rY = [cy, 0, -sy, 0,  0, 1, 0, 0,  sy, 0, cy, 0,  0, 0, 0, 1]
  const rZ = [cz, sz, 0, 0,  -sz, cz, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1]
  return mat4Multiply(mat4Multiply(rX, rY), rZ)
}

function emptyBounds() {
  return { min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity] }
}
function expandBounds(b, p) {
  for (let i = 0; i < 3; i++) {
    if (p[i] < b.min[i]) b.min[i] = p[i]
    if (p[i] > b.max[i]) b.max[i] = p[i]
  }
}
function sizeOf(b) {
  return [b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2]]
}

function collectGlbVertices(json, bin) {
  const scene = json.scenes[json.scene ?? 0]
  const identity = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
  const verts = []
  function walk(nodeIdx, parent) {
    const node = json.nodes[nodeIdx]
    const world = mat4Multiply(parent, nodeLocalMatrix(node))
    if (typeof node.mesh === 'number') {
      for (const prim of json.meshes[node.mesh].primitives) {
        const posIdx = prim.attributes?.POSITION
        if (typeof posIdx !== 'number') continue
        for (const p of readAccessorVec3(json, bin, posIdx)) verts.push(transformPoint(world, p))
      }
    }
    for (const child of node.children || []) walk(child, world)
  }
  for (const root of scene.nodes) walk(root, identity)
  return verts
}

// Most authored GLBs are in millimeters; some are in meters. Heuristic: if the
// raw extent is implausibly large (>>1 m on any axis given typical robot parts),
// treat as mm and divide by 1000. Otherwise treat as meters.
function unitScaleForRawSize(rawSize) {
  return Math.max(...rawSize) > 5 ? 0.001 : 1
}

// ─────────────────────────── OBJ loader ──────────────────────────────────────

async function loadObjVertices(filePath) {
  const text = await fs.readFile(filePath, 'utf8')
  const verts = []
  for (const rawLine of text.split('\n')) {
    if (rawLine[0] !== 'v' || rawLine[1] !== ' ') continue
    const parts = rawLine.trim().split(/\s+/)
    if (parts.length < 4) continue
    verts.push([Number(parts[1]), Number(parts[2]), Number(parts[3])])
  }
  return verts
}

// ─────────────────────────── meshOverrides parsing ───────────────────────────

function stripLineComments(s) { return s.replace(/\/\/[^\n]*/g, '') }

function extractRecordBody(src, name) {
  const header = new RegExp(`export\\s+const\\s+${name}\\s*:[^=]+?=\\s*\\{`)
  const match = src.match(header)
  if (!match) throw new Error(`could not locate ${name}`)
  let i = match.index + match[0].length
  let depth = 1
  while (i < src.length && depth > 0) {
    const ch = src[i]
    if (ch === '{') depth++
    else if (ch === '}') depth--
    i++
  }
  if (depth !== 0) throw new Error(`unbalanced braces parsing ${name}`)
  return src.slice(match.index + match[0].length, i - 1)
}

function parseStringRecord(src, name) {
  const body = stripLineComments(extractRecordBody(src, name))
  const out = new Map()
  const re = /'([^']+)'\s*:\s*'([^']+)'/g
  let m
  while ((m = re.exec(body)) !== null) out.set(m[1], m[2])
  return out
}

function evalAngle(expr) {
  const trimmed = String(expr).trim()
  if (!/^[0-9.\s+\-*/()MathPI]+$/.test(trimmed)) throw new Error(`unexpected angle expression: ${trimmed}`)
  return Function(`"use strict"; return (${trimmed});`)()
}

function parseRotationRecord(src, name) {
  const body = stripLineComments(extractRecordBody(src, name))
  const out = new Map()
  const re = /'([^']+)'\s*:\s*\[\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,\s*([^,\]]+)\s*\]/g
  let m
  while ((m = re.exec(body)) !== null) {
    out.set(m[1], [evalAngle(m[2]), evalAngle(m[3]), evalAngle(m[4])])
  }
  return out
}

// ──────────────────────────────── Main ───────────────────────────────────────

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

function maxAxisDivergence(a, b) {
  let worst = 0
  for (let i = 0; i < 3; i++) {
    if (b[i] === 0) continue
    const d = Math.abs(a[i] - b[i]) / b[i]
    if (d > worst) worst = d
  }
  return worst
}

async function safeAccess(p) {
  try { await fs.access(p); return true } catch { return false }
}

// ─────────────────────────── CI gate (Phase 5) ───────────────────────────────
//
// The script always *measures* and writes meshExtents.generated.json. Whether
// it *fails* on divergence is controlled by flags so the gate can be wired
// into CI now without bricking the build on the 61 pre-existing drifts.
//
//   --strict                fail on any new divergence > 5% not in baseline,
//                           or any baselined divergence that grew by >tolerance
//   --update-baseline       overwrite scripts/mesh-extent-baseline.json with
//                           current divergences (use after deliberately fixing
//                           or accepting a spec change)
//   --threshold=0.05        divergence ratio that counts as "drift" (default 5%)

const BASELINE_FILE = path.join(repoRoot, 'scripts', 'mesh-extent-baseline.json')

function parseFlags(argv) {
  const out = { strict: false, updateBaseline: false, threshold: 0.05 }
  for (const a of argv) {
    if (a === '--strict') out.strict = true
    else if (a === '--update-baseline') out.updateBaseline = true
    else if (a.startsWith('--threshold=')) out.threshold = Number(a.slice(12))
  }
  return out
}

async function loadBaseline() {
  try {
    const text = await fs.readFile(BASELINE_FILE, 'utf8')
    return JSON.parse(text)
  } catch {
    return { tolerance: 0.005, components: {} }
  }
}

function compareAgainstBaseline(result, baseline, threshold) {
  const tol = baseline.tolerance ?? 0.005
  const known = baseline.components || {}
  const newDrifts = []
  const grewDrifts = []
  for (const [id, entry] of Object.entries(result)) {
    const div = entry.visual?.divergence_vs_bbox
    if (typeof div !== 'number' || div <= threshold) continue
    if (!(id in known)) {
      newDrifts.push({ id, div })
    } else if (div > known[id] + tol) {
      grewDrifts.push({ id, div, baseline: known[id] })
    }
  }
  return { newDrifts, grewDrifts }
}

async function main() {
  const flags = parseFlags(process.argv.slice(2))
  const meshOverridesText = await fs.readFile(MESH_OVERRIDES_TS, 'utf8')
  const meshOverrides = parseStringRecord(meshOverridesText, 'MESH_OVERRIDES')
  const rotationOverrides = parseRotationRecord(meshOverridesText, 'ROTATION_OVERRIDES')
  const presetData = JSON.parse(await fs.readFile(PUBLIC_PRESETS, 'utf8'))

  const result = {}
  let measured = 0
  let warnings = 0

  for (const component of flattenPresets(presetData)) {
    const id = component.id
    const visualMesh = meshOverrides.get(id)
    const collisionMesh = component.physical?.collision_mesh
    if (!visualMesh && !collisionMesh) continue

    const entry = {}
    const declaredBbox = presetBboxMm(component)
    if (declaredBbox) entry.declared_bbox_mm = declaredBbox

    if (visualMesh) {
      const base = visualMesh.replace(/\.(step|stp|glb)$/i, '')
      const glbPath = path.join(GLB_DIR, `${base}.glb`)
      if (await safeAccess(glbPath)) {
        try {
          const glbBytes = await fs.readFile(glbPath)
          const visualHash = shortContentHash(glbBytes)
          const { json, bin } = await loadGlb(glbPath)
          const rawVerts = collectGlbVertices(json, bin)
          if (rawVerts.length > 0) {
            const rawBounds = emptyBounds()
            for (const v of rawVerts) expandBounds(rawBounds, v)
            const rawSize = sizeOf(rawBounds)
            const unitScale = unitScaleForRawSize(rawSize)
            const rot = rotationOverrides.get(id) || [0, 0, 0]
            const rotMat = rotationMatrixXyz(rot)
            const rotated = emptyBounds()
            for (const v of rawVerts) {
              expandBounds(rotated, transformPoint(rotMat, [v[0] * unitScale, v[1] * unitScale, v[2] * unitScale]))
            }
            const postRotMm = sizeOf(rotated).map(v => v * 1000)
            entry.visual = {
              file: `${base}.glb`,
              content_hash: visualHash,
              rotation_rpy: rot,
              raw_extent_mm: rawSize.map(v => v * unitScale * 1000),
              post_rotation_extent_mm: postRotMm,
              vertex_count: rawVerts.length,
            }
            if (declaredBbox) {
              const div = maxAxisDivergence(postRotMm, declaredBbox)
              entry.visual.divergence_vs_bbox = div
              if (div > 0.05) {
                entry.visual.warning = `post-rotation visual extent diverges from declared bbox by ${(div * 100).toFixed(1)}%`
                warnings++
              }
            }
          }
        } catch (e) {
          entry.visual = { file: `${base}.glb`, error: e.message }
        }
      }
    }

    if (collisionMesh) {
      const objPath = path.join(COLLISION_DIR, collisionMesh)
      if (await safeAccess(objPath)) {
        try {
          const collisionBytes = await fs.readFile(objPath)
          const collisionHash = shortContentHash(collisionBytes)
          const verts = await loadObjVertices(objPath)
          if (verts.length > 0) {
            const b = emptyBounds()
            for (const v of verts) expandBounds(b, v)
            const extentMm = sizeOf(b).map(v => v * 1000)
            entry.collision = {
              file: collisionMesh,
              content_hash: collisionHash,
              extent_mm: extentMm,
              center_mm: [
                (b.max[0] + b.min[0]) / 2 * 1000,
                (b.max[1] + b.min[1]) / 2 * 1000,
                (b.max[2] + b.min[2]) / 2 * 1000,
              ],
              vertex_count: verts.length,
            }
            if (declaredBbox) {
              const div = maxAxisDivergence(extentMm, declaredBbox)
              entry.collision.divergence_vs_bbox = div
            }
          }
        } catch (e) {
          entry.collision = { file: collisionMesh, error: e.message }
        }
      }
    }

    if (entry.visual || entry.collision) {
      result[id] = entry
      measured++
    }
  }

  const sortedIds = Object.keys(result).sort()
  const sortedResult = {}
  for (const id of sortedIds) sortedResult[id] = result[id]

  const output = {
    generated_at: new Date().toISOString(),
    notes: 'Generated by scripts/measure-mesh-extents.mjs. Re-run when GLBs/OBJs change.',
    components: sortedResult,
  }

  await fs.writeFile(OUT_FILE, JSON.stringify(output, null, 2) + '\n', 'utf8')
  console.log(`Measured ${measured} components → ${path.relative(repoRoot, OUT_FILE)}`)
  if (warnings > 0) console.log(`${warnings} components have visual-extent vs bbox divergence > ${(flags.threshold * 100).toFixed(0)}%`)

  if (flags.updateBaseline) {
    const components = {}
    for (const [id, entry] of Object.entries(sortedResult)) {
      const div = entry.visual?.divergence_vs_bbox
      if (typeof div === 'number' && div > flags.threshold) components[id] = +div.toFixed(4)
    }
    const existing = await loadBaseline()
    const baseline = {
      notes: existing.notes ?? 'Baseline of pre-existing visual-vs-declared-bbox divergences. Strict gate fails if a new component appears or a baselined divergence grows by > tolerance.',
      generated_at: new Date().toISOString(),
      tolerance: existing.tolerance ?? 0.005,
      components: Object.fromEntries(Object.entries(components).sort()),
    }
    await fs.writeFile(BASELINE_FILE, JSON.stringify(baseline, null, 2) + '\n', 'utf8')
    console.log(`Baseline updated: ${Object.keys(components).length} entries → ${path.relative(repoRoot, BASELINE_FILE)}`)
    return
  }

  if (flags.strict) {
    const baseline = await loadBaseline()
    const { newDrifts, grewDrifts } = compareAgainstBaseline(sortedResult, baseline, flags.threshold)

    // Phase 5 step 4: collision divergence is a hard gate — no baseline. The
    // collision mesh is the canonical envelope the placement compiler reads
    // (via resolved.collision.bounds), so a bbox that diverges from it
    // misplaces every downstream child. 15% accommodates the shaft-overhang
    // case (high_torque ~14.7%); anything beyond is a preset data bug.
    const COLLISION_LIMIT = 0.15
    const collisionFails = []
    for (const [id, entry] of Object.entries(sortedResult)) {
      const div = entry.collision?.divergence_vs_bbox
      if (typeof div === 'number' && div > COLLISION_LIMIT) {
        collisionFails.push({ id, div })
      }
    }

    if (newDrifts.length === 0 && grewDrifts.length === 0 && collisionFails.length === 0) {
      console.log(`Strict check passed (baseline: ${Object.keys(baseline.components || {}).length} known visual drifts, tolerance ${baseline.tolerance ?? 0.005}; collision limit ${COLLISION_LIMIT * 100}%).`)
      return
    }
    console.error(`\nStrict mesh-extent check FAILED:`)
    for (const { id, div } of newDrifts) {
      console.error(`  NEW visual drift: ${id}  divergence ${(div * 100).toFixed(1)}% (not in baseline)`)
    }
    for (const { id, div, baseline: b } of grewDrifts) {
      console.error(`  GREW visual: ${id}  ${(b * 100).toFixed(1)}% → ${(div * 100).toFixed(1)}%`)
    }
    for (const { id, div } of collisionFails) {
      console.error(`  COLLISION: ${id}  divergence ${(div * 100).toFixed(1)}% > limit ${(COLLISION_LIMIT * 100).toFixed(0)}%`)
    }
    console.error(`\nFix by editing physical.bbox_mm / parametric in src/public/generic_presets.json,`)
    console.error(`or — if the new visual divergence is intentional — re-run with --update-baseline.`)
    console.error(`(Collision divergence has no baseline — it must be fixed in the preset.)`)
    process.exit(1)
  }
}

main().catch(e => {
  console.error('fatal:', e.stack || e.message)
  process.exit(1)
})

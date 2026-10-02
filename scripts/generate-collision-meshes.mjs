// Generate convex-hull collision meshes for every GLB in src/public/meshes/glb/.
//
// Step 4 of docs/ENGINE_NEXT_STEPS.md: replace primitive <box> collision
// emission with convex-hull mesh collision (production standard per Spot,
// Isaac Sim, etc.). One-shot script — re-run only when GLBs are added or
// updated.
//
// Output: src/public/meshes/collision/{base}_collision.obj for each GLB.
//
// Usage:
//   cd src && node ../scripts/generate-collision-meshes.mjs
//
// (Run from src/ so the resolver finds three in the local node_modules.
//  Or set NODE_PATH appropriately.)
//
// Algorithm: walk each GLB, decode every POSITION accessor, transform
// vertices through the node hierarchy into world coordinates, feed all
// vertices to three's ConvexHull (Quickhull3D port). Write the resulting
// hull as an OBJ (v + f lines, 1-indexed, no normals).
//
// Output is normalized into the component (catalog) frame: the runtime
// rotation override from src/src/richVisuals/visualOverrides.json is applied,
// then the hull is scaled to the preset bbox and centered.

import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
// Three.js lives in src/node_modules — import by relative path because the
// script is run from scripts/ which has no node_modules of its own. The
// addon's own `import { Vector3 } from 'three'` still resolves via node's
// upwards walk from inside the package directory.
import { Vector3 } from '../src/node_modules/three/build/three.module.js'
import { ConvexHull } from '../src/node_modules/three/examples/jsm/math/ConvexHull.js'

const here = path.dirname(fileURLToPath(import.meta.url))
const repoRoot = path.resolve(here, '..')
const GLB_DIR = path.join(repoRoot, 'src', 'public', 'meshes', 'glb')
const OUT_DIR = path.join(repoRoot, 'src', 'public', 'meshes', 'collision')
const CORE_PRESETS = path.join(repoRoot, 'core', 'presets', 'generic_presets.json')
const PUBLIC_PRESETS = path.join(repoRoot, 'src', 'public', 'generic_presets.json')
const VISUAL_OVERRIDES_JSON = path.join(repoRoot, 'src', 'src', 'richVisuals', 'visualOverrides.json')

// ─────────────────────────── GLB binary helpers ──────────────────────────────

const COMPONENT_TYPE_FLOAT = 5126
const COMPONENT_BYTE_SIZE = {
  5120: 1, // BYTE
  5121: 1, // UNSIGNED_BYTE
  5122: 2, // SHORT
  5123: 2, // UNSIGNED_SHORT
  5125: 4, // UNSIGNED_INT
  5126: 4, // FLOAT
}
const TYPE_NUM_COMPONENTS = {
  SCALAR: 1, VEC2: 2, VEC3: 3, VEC4: 4,
  MAT2: 4, MAT3: 9, MAT4: 16,
}

async function loadGlb(filePath) {
  const buf = await fs.readFile(filePath)
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength)
  if (dv.getUint32(0, true) !== 0x46546c67) throw new Error('not a GLB')
  const jsonLen = dv.getUint32(12, true)
  if (dv.getUint32(16, true) !== 0x4e4f534a) throw new Error('expected JSON chunk')
  const json = JSON.parse(new TextDecoder().decode(buf.subarray(20, 20 + jsonLen)))
  const binOffset = 20 + jsonLen
  // Some GLBs may omit the BIN chunk if all data is in external URIs;
  // every GLB shipped here has an embedded BIN chunk, so require one.
  if (binOffset + 8 > buf.byteLength) throw new Error('missing BIN chunk')
  if (dv.getUint32(binOffset + 4, true) !== 0x004e4942) throw new Error('expected BIN chunk')
  const binLen = dv.getUint32(binOffset, true)
  const bin = buf.subarray(binOffset + 8, binOffset + 8 + binLen)
  return { json, bin }
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
  // Source DataView spans the whole BIN chunk so we can read at arbitrary
  // strided offsets without copying.
  const dv = new DataView(bin.buffer, bin.byteOffset, bin.byteLength)
  for (let i = 0; i < acc.count; i++) {
    const o = baseOffset + i * stride
    out[i] = [
      dv.getFloat32(o, true),
      dv.getFloat32(o + 4, true),
      dv.getFloat32(o + 8, true),
    ]
  }
  return out
}

// ─────────────────────────── Node matrix walk ────────────────────────────────

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

function rotationMatrixXyz(r) {
  const [x, y, z] = r
  const cx = Math.cos(x), sx = Math.sin(x)
  const cy = Math.cos(y), sy = Math.sin(y)
  const cz = Math.cos(z), sz = Math.sin(z)
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

function emptyBounds() {
  return {
    min: [Infinity, Infinity, Infinity],
    max: [-Infinity, -Infinity, -Infinity],
  }
}

function boundsOf(points) {
  const b = emptyBounds()
  for (const p of points) {
    for (let i = 0; i < 3; i++) {
      if (p[i] < b.min[i]) b.min[i] = p[i]
      if (p[i] > b.max[i]) b.max[i] = p[i]
    }
  }
  return b
}

function sizeOfBounds(b) {
  return [b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2]]
}

function centerOfBounds(b) {
  return [(b.max[0] + b.min[0]) / 2, (b.max[1] + b.min[1]) / 2, (b.max[2] + b.min[2]) / 2]
}

function collectVertices(json, bin) {
  const scene = json.scenes[json.scene ?? 0]
  const identity = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
  const vertices = []
  let primitiveCount = 0

  function walk(nodeIdx, parentMatrix) {
    const node = json.nodes[nodeIdx]
    const worldMatrix = mat4Multiply(parentMatrix, nodeLocalMatrix(node))
    if (typeof node.mesh === 'number') {
      const mesh = json.meshes[node.mesh]
      for (const prim of mesh.primitives) {
        const posIdx = prim.attributes?.POSITION
        if (typeof posIdx !== 'number') continue
        const positions = readAccessorVec3(json, bin, posIdx)
        for (const p of positions) vertices.push(transformPoint(worldMatrix, p))
        primitiveCount++
      }
    }
    for (const child of node.children || []) walk(child, worldMatrix)
  }

  for (const root of scene.nodes) walk(root, identity)
  return { vertices, primitiveCount }
}

// Per-component visual overrides (mesh file, rotation, shaft overlay) —
// the same JSON meshOverrides.ts imports at runtime.
async function loadVisualOverrides() {
  const data = JSON.parse(await fs.readFile(VISUAL_OVERRIDES_JSON, 'utf8'))
  return {
    meshOverrides: new Map(Object.entries(data.meshOverrides)),
    rotationOverrides: new Map(Object.entries(data.rotationOverrides).map(([id, e]) => [id, e.rpy])),
    shaftOverlays: new Map(Object.entries(data.shaftOverlays).map(([id, e]) => [id, { shaft_length_mm: e.shaft_length_mm, shaft_radius_mm: e.shaft_radius_mm }])),
  }
}

function flattenPresets(data) {
  const out = []
  for (const [category, cat] of Object.entries(data.categories || {})) {
    for (const component of cat.components || []) out.push({ category, component })
  }
  return out
}

function effectiveBboxMm(component) {
  const phys = component.physical || {}
  const bb = phys.bounding_box_mm || phys.cross_section_mm || [40, 40, 40]
  return [bb[0] || 40, bb[1] || 40, bb[2] || 40]
}

function normalizeVerticesToComponentFrame(rawVertices, component, rotationOverrides, shaftOverlays) {
  const componentId = component.id
  const bboxMm = effectiveBboxMm(component)
  const targetMeters = bboxMm.map(v => v / 1000)
  const rawBounds = boundsOf(rawVertices)
  const rawSize = sizeOfBounds(rawBounds)
  const unitScale = Math.max(...rawSize) > Math.max(...targetMeters) * 10 ? 0.001 : 1
  const rot = rotationOverrides.get(componentId) || [0, 0, 0]
  const rotMatrix = rotationMatrixXyz(rot)

  let points = rawVertices.map(p => transformPoint(rotMatrix, [p[0] * unitScale, p[1] * unitScale, p[2] * unitScale]))
  const rotatedBounds = boundsOf(points)
  const rotatedSize = sizeOfBounds(rotatedBounds)
  const shaftOverlay = shaftOverlays.get(componentId)
  const shaftLenM = shaftOverlay ? shaftOverlay.shaft_length_mm / 1000 : 0
  const targetZ = shaftOverlay ? Math.max(0.001, targetMeters[2] - shaftLenM) : targetMeters[2]
  const targetForScale = [targetMeters[0], targetMeters[1], targetZ]

  // Collision meshes are a physics/debug envelope, not the final shaded CAD
  // visual, so always normalize to the component envelope even for visual
  // components whose render mesh intentionally skips per-axis scaling.
  const scale = targetForScale.map((v, i) => rotatedSize[i] > 0.0001 ? v / rotatedSize[i] : 1)
  points = points.map(p => [p[0] * scale[0], p[1] * scale[1], p[2] * scale[2]])

  const finalCenter = centerOfBounds(boundsOf(points))
  points = points.map(p => [p[0] - finalCenter[0], p[1] - finalCenter[1], p[2] - finalCenter[2] - shaftLenM / 2])
  return points
}

// ─────────────────────────── Convex hull → OBJ ───────────────────────────────

function computeHullObj(vertices) {
  if (vertices.length < 4) {
    throw new Error(`only ${vertices.length} vertices — need ≥4 for a 3D hull`)
  }
  // ConvexHull expects { x, y, z } objects. Vector3 instances satisfy that.
  const points = vertices.map(([x, y, z]) => new Vector3(x, y, z))
  const hull = new ConvexHull().setFromPoints(points)

  // Each face's edge linked-list traversal yields 3+ vertices per triangle face.
  // ConvexHull always emits triangulated faces so we can read edge.head().point
  // three times. Dedupe via a Map keyed on rounded coords.
  const vertMap = new Map()
  const outVerts = []
  const faceTris = []

  function key(p) {
    return `${p.x.toFixed(6)},${p.y.toFixed(6)},${p.z.toFixed(6)}`
  }
  function vid(p) {
    const k = key(p)
    let i = vertMap.get(k)
    if (i === undefined) {
      outVerts.push([p.x, p.y, p.z])
      i = outVerts.length // 1-based for OBJ
      vertMap.set(k, i)
    }
    return i
  }

  for (const face of hull.faces) {
    const indices = []
    let edge = face.edge
    do {
      indices.push(vid(edge.head().point))
      edge = edge.next
    } while (edge !== face.edge)
    // Fan-triangulate in case any face is non-triangular (defensive — three's
    // ConvexHull triangulates faces by default but the doubly-connected edge
    // list in principle supports n-gons).
    for (let i = 1; i < indices.length - 1; i++) {
      faceTris.push([indices[0], indices[i], indices[i + 1]])
    }
  }

  // Vertices have already been normalized to component-local meters.
  // The OBJs are consumed by urdf_to_mjcf at scale=1.
  let obj = '# Normalized convex hull collision mesh - generated by scripts/generate-collision-meshes.mjs (units: meters)\n'
  for (const v of outVerts) {
    obj += `v ${v[0].toFixed(6)} ${v[1].toFixed(6)} ${v[2].toFixed(6)}\n`
  }
  for (const f of faceTris) {
    obj += `f ${f[0]} ${f[1]} ${f[2]}\n`
  }

  return { obj, vertexCount: outVerts.length, faceCount: faceTris.length }
}

// ──────────────────────────────── Main ───────────────────────────────────────

const glbCache = new Map()

async function loadGlbVertices(glbPath) {
  const cached = glbCache.get(glbPath)
  if (cached) return cached
  const { json, bin } = await loadGlb(glbPath)
  const collected = collectVertices(json, bin)
  glbCache.set(glbPath, collected)
  return collected
}

async function processComponent(component, meshFile, rotationOverrides, shaftOverlays) {
  const componentId = component.id
  const base = meshFile.replace(/\.(step|stp)$/i, '')
  const glbPath = path.join(GLB_DIR, `${base}.glb`)
  const start = Date.now()
  const { vertices, primitiveCount } = await loadGlbVertices(glbPath)
  if (vertices.length === 0) throw new Error('no vertices')
  const normalized = normalizeVerticesToComponentFrame(vertices, component, rotationOverrides, shaftOverlays)
  const { obj, vertexCount, faceCount } = computeHullObj(normalized)
  const collisionMesh = `${componentId}_collision.obj`
  const outPath = path.join(OUT_DIR, collisionMesh)
  await fs.writeFile(outPath, obj, 'utf8')
  component.physical.collision_mesh = collisionMesh
  return {
    componentId,
    base,
    collisionMesh,
    inputVerts: vertices.length,
    primitives: primitiveCount,
    hullVerts: vertexCount,
    hullFaces: faceCount,
    ms: Date.now() - start,
  }
}

async function main() {
  await fs.mkdir(OUT_DIR, { recursive: true })
  const { meshOverrides, rotationOverrides, shaftOverlays } = await loadVisualOverrides()
  const presetData = JSON.parse(await fs.readFile(CORE_PRESETS, 'utf8'))
  const processable = []

  for (const { component } of flattenPresets(presetData)) {
    const meshFile = meshOverrides.get(component.id)
    if (!meshFile || !component.physical?.bounding_box_mm) continue
    const glbPath = path.join(GLB_DIR, `${meshFile.replace(/\.(step|stp)$/i, '')}.glb`)
    try {
      await fs.access(glbPath)
      processable.push({ component, meshFile })
    } catch {
      // Leave existing collision_mesh alone when the GLB is not shipped.
    }
  }

  console.log(`Processing ${processable.length} mesh-backed preset components from ${path.relative(repoRoot, GLB_DIR)}`)
  console.log(`Writing to ${path.relative(repoRoot, OUT_DIR)}/`)
  console.log('')

  let ok = 0
  const failures = []
  for (const { component, meshFile } of processable) {
    try {
      const r = await processComponent(component, meshFile, rotationOverrides, shaftOverlays)
      console.log(
        `  ${r.componentId.padEnd(38)}  ` +
        `${String(r.primitives).padStart(3)} prim  ` +
        `${String(r.inputVerts).padStart(6)} in →  ` +
        `${String(r.hullVerts).padStart(4)} v / ${String(r.hullFaces).padStart(4)} f  ` +
        `(${r.ms}ms)`,
      )
      ok++
    } catch (e) {
      console.error(`  ${component.id.padEnd(38)}  FAIL: ${e.message}`)
      failures.push({ name: component.id, error: e.message })
    }
  }

  console.log('')
  console.log(`OK: ${ok} / ${processable.length}`)
  if (failures.length) {
    console.log(`FAILURES: ${failures.length}`)
    for (const f of failures) console.log(`  ${f.name}: ${f.error}`)
    process.exit(1)
  }

  const serialized = JSON.stringify(presetData, null, 2) + '\n'
  await fs.writeFile(CORE_PRESETS, serialized, 'utf8')
  await fs.writeFile(PUBLIC_PRESETS, serialized, 'utf8')
  console.log(`Updated ${path.relative(repoRoot, CORE_PRESETS)} and ${path.relative(repoRoot, PUBLIC_PRESETS)}`)
}

main().catch(e => {
  console.error('fatal:', e.stack || e.message)
  process.exit(2)
})

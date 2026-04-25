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
// Output is mesh-local coords matching the GLB's authoring frame — we do
// NOT apply the runtime ROTATION_OVERRIDES from meshOverrides.ts. Per
// docs/ENGINE_NEXT_STEPS.md Step 4 scope, downstream sim consumers either
// receive the same rotation in the joint origin or accept the raw frame.

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

  // GLBs in this project are authored in millimeters and the frontend converts
  // to meters at load time (richVisuals/index.ts: meshGroup.scale.setScalar(0.001)).
  // The OBJs are consumed by urdf_to_mjcf at scale=1, so write them in meters
  // here. Without this scale, MuJoCo loads a foot pad as a 16-meter sphere and
  // the auto-lift logic launches the robot ~165 m above the floor.
  const MM_TO_M = 0.001
  let obj = '# Convex hull collision mesh — generated by scripts/generate-collision-meshes.mjs (units: meters)\n'
  for (const v of outVerts) {
    obj += `v ${(v[0] * MM_TO_M).toFixed(6)} ${(v[1] * MM_TO_M).toFixed(6)} ${(v[2] * MM_TO_M).toFixed(6)}\n`
  }
  for (const f of faceTris) {
    obj += `f ${f[0]} ${f[1]} ${f[2]}\n`
  }

  return { obj, vertexCount: outVerts.length, faceCount: faceTris.length }
}

// ──────────────────────────────── Main ───────────────────────────────────────

async function processGlb(glbPath) {
  const base = path.basename(glbPath, '.glb')
  const start = Date.now()
  const { json, bin } = await loadGlb(glbPath)
  const { vertices, primitiveCount } = collectVertices(json, bin)
  if (vertices.length === 0) throw new Error('no vertices')
  const { obj, vertexCount, faceCount } = computeHullObj(vertices)
  const outPath = path.join(OUT_DIR, `${base}_collision.obj`)
  await fs.writeFile(outPath, obj, 'utf8')
  return {
    base,
    inputVerts: vertices.length,
    primitives: primitiveCount,
    hullVerts: vertexCount,
    hullFaces: faceCount,
    ms: Date.now() - start,
  }
}

async function main() {
  await fs.mkdir(OUT_DIR, { recursive: true })
  const entries = (await fs.readdir(GLB_DIR))
    .filter(n => n.toLowerCase().endsWith('.glb'))
    .sort()

  console.log(`Processing ${entries.length} GLB files from ${path.relative(repoRoot, GLB_DIR)}`)
  console.log(`Writing to ${path.relative(repoRoot, OUT_DIR)}/`)
  console.log('')

  let ok = 0
  const failures = []
  for (const name of entries) {
    const full = path.join(GLB_DIR, name)
    try {
      const r = await processGlb(full)
      console.log(
        `  ${r.base.padEnd(36)}  ` +
        `${String(r.primitives).padStart(3)} prim  ` +
        `${String(r.inputVerts).padStart(6)} in →  ` +
        `${String(r.hullVerts).padStart(4)} v / ${String(r.hullFaces).padStart(4)} f  ` +
        `(${r.ms}ms)`,
      )
      ok++
    } catch (e) {
      console.error(`  ${name.padEnd(36)}  FAIL: ${e.message}`)
      failures.push({ name, error: e.message })
    }
  }

  console.log('')
  console.log(`OK: ${ok} / ${entries.length}`)
  if (failures.length) {
    console.log(`FAILURES: ${failures.length}`)
    for (const f of failures) console.log(`  ${f.name}: ${f.error}`)
    process.exit(1)
  }
}

main().catch(e => {
  console.error('fatal:', e.stack || e.message)
  process.exit(2)
})

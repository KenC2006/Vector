// Measure raw bbox of a GLB file (in source units, typically mm).
// Usage: node scripts/measure-glb-bbox.mjs <path-to-glb> [...more]
//
// Output: filename : x_mm × y_mm × z_mm
//
// Used by §9 path B preset bbox audit to compare GLB extents against
// preset bounding_box_mm declarations.

import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const here = path.dirname(fileURLToPath(import.meta.url))
const repoRoot = path.resolve(here, '..')

async function loadGltfDoc(filePath) {
  const buf = await fs.readFile(filePath)
  // GLB header: magic 'glTF' (0x46546C67), version, length, then JSON chunk
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength)
  const magic = dv.getUint32(0, true)
  if (magic !== 0x46546C67) throw new Error(`Not a GLB: ${filePath}`)
  const jsonChunkLength = dv.getUint32(12, true)
  const jsonChunkType = dv.getUint32(16, true)
  if (jsonChunkType !== 0x4E4F534A) throw new Error(`Expected JSON chunk: ${filePath}`)
  const jsonBytes = buf.subarray(20, 20 + jsonChunkLength)
  const json = JSON.parse(new TextDecoder().decode(jsonBytes))
  // Binary chunk follows JSON chunk
  const binChunkOffset = 20 + jsonChunkLength
  const binChunkLength = dv.getUint32(binChunkOffset, true)
  const binChunkType = dv.getUint32(binChunkOffset + 4, true)
  if (binChunkType !== 0x004E4942) throw new Error(`Expected BIN chunk: ${filePath}`)
  const bin = buf.subarray(binChunkOffset + 8, binChunkOffset + 8 + binChunkLength)
  return { json, bin }
}

function getAccessorMinMax(json, accessorIdx) {
  const acc = json.accessors[accessorIdx]
  if (acc?.min && acc?.max) return { min: acc.min, max: acc.max }
  return null
}

function nodeMatrix(node) {
  // Returns 4x4 matrix as 16-element array (column-major).
  if (node.matrix) return node.matrix.slice()
  const t = node.translation || [0, 0, 0]
  const r = node.rotation || [0, 0, 0, 1] // quaternion xyzw
  const s = node.scale || [1, 1, 1]
  const [x, y, z, w] = r
  const xx = x * x, yy = y * y, zz = z * z
  const xy = x * y, xz = x * z, yz = y * z
  const wx = w * x, wy = w * y, wz = w * z
  // Column-major
  return [
    (1 - 2 * (yy + zz)) * s[0], 2 * (xy + wz) * s[0], 2 * (xz - wy) * s[0], 0,
    2 * (xy - wz) * s[1], (1 - 2 * (xx + zz)) * s[1], 2 * (yz + wx) * s[1], 0,
    2 * (xz + wy) * s[2], 2 * (yz - wx) * s[2], (1 - 2 * (xx + yy)) * s[2], 0,
    t[0], t[1], t[2], 1,
  ]
}

function multiply(a, b) {
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
  const x = m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12]
  const y = m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13]
  const z = m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14]
  return [x, y, z]
}

function transformAabb(m, min, max) {
  // Compute AABB after transform by visiting all 8 corners.
  const corners = [
    [min[0], min[1], min[2]], [max[0], min[1], min[2]],
    [min[0], max[1], min[2]], [max[0], max[1], min[2]],
    [min[0], min[1], max[2]], [max[0], min[1], max[2]],
    [min[0], max[1], max[2]], [max[0], max[1], max[2]],
  ].map(c => transformPoint(m, c))
  const newMin = [Infinity, Infinity, Infinity]
  const newMax = [-Infinity, -Infinity, -Infinity]
  for (const c of corners) {
    for (let i = 0; i < 3; i++) {
      if (c[i] < newMin[i]) newMin[i] = c[i]
      if (c[i] > newMax[i]) newMax[i] = c[i]
    }
  }
  return { min: newMin, max: newMax }
}

function walkNode(json, nodeIdx, parentMatrix, accumMin, accumMax) {
  const node = json.nodes[nodeIdx]
  const localMatrix = nodeMatrix(node)
  const worldMatrix = multiply(parentMatrix, localMatrix)
  if (typeof node.mesh === 'number') {
    const mesh = json.meshes[node.mesh]
    for (const prim of mesh.primitives) {
      const posIdx = prim.attributes?.POSITION
      if (typeof posIdx !== 'number') continue
      const mm = getAccessorMinMax(json, posIdx)
      if (!mm) continue
      const aabb = transformAabb(worldMatrix, mm.min, mm.max)
      for (let i = 0; i < 3; i++) {
        if (aabb.min[i] < accumMin[i]) accumMin[i] = aabb.min[i]
        if (aabb.max[i] > accumMax[i]) accumMax[i] = aabb.max[i]
      }
    }
  }
  for (const child of node.children || []) {
    walkNode(json, child, worldMatrix, accumMin, accumMax)
  }
}

async function measure(filePath) {
  const { json } = await loadGltfDoc(filePath)
  const scene = json.scenes[json.scene ?? 0]
  const accumMin = [Infinity, Infinity, Infinity]
  const accumMax = [-Infinity, -Infinity, -Infinity]
  const identity = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
  for (const root of scene.nodes) walkNode(json, root, identity, accumMin, accumMax)
  return {
    size: [accumMax[0] - accumMin[0], accumMax[1] - accumMin[1], accumMax[2] - accumMin[2]],
    min: accumMin,
    max: accumMax,
  }
}

async function main() {
  const args = process.argv.slice(2)
  if (args.length === 0) {
    console.error('Usage: node measure-glb-bbox.mjs <path-to-glb> [more...]')
    process.exit(1)
  }
  for (const arg of args) {
    const abs = path.isAbsolute(arg) ? arg : path.resolve(process.cwd(), arg)
    try {
      const { size } = await measure(abs)
      const fmt = size.map(v => v.toFixed(2)).join(' x ')
      console.log(`${path.basename(abs)} : ${fmt}`)
    } catch (e) {
      console.error(`${path.basename(arg)} : ERROR ${e.message}`)
    }
  }
}

main()

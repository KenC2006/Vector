// Sweep sample radius + include heavy_duty preset (same GLB, larger bbox).
// Goal: confirm whether the actual body-bottom flatness is at z=-12.834 or
// something else, and whether the user's 14mm comes from a different
// effective face radius or preset.

import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const here = path.dirname(fileURLToPath(import.meta.url))
const repoRoot = path.resolve(here, '..')

function loadGlbDoc(filePath) {
  return fs.readFile(filePath).then(buf => {
    const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength)
    const jsonChunkLength = dv.getUint32(12, true)
    const jsonBytes = buf.subarray(20, 20 + jsonChunkLength)
    const json = JSON.parse(new TextDecoder().decode(jsonBytes))
    const binOffset = 20 + jsonChunkLength
    const binLength = dv.getUint32(binOffset, true)
    const bin = buf.subarray(binOffset + 8, binOffset + 8 + binLength)
    return { json, bin }
  })
}
function accessorReader(json, bin, accIdx) {
  const acc = json.accessors[accIdx]
  const bv = json.bufferViews[acc.bufferView]
  const offset = (bv.byteOffset || 0) + (acc.byteOffset || 0)
  const compSz = (acc.type === 'VEC3' ? 3 : acc.type === 'VEC2' ? 2 : 1) *
    (acc.componentType === 5126 ? 4 : acc.componentType === 5123 ? 2 : 4)
  const stride = bv.byteStride || compSz
  const view = new DataView(bin.buffer, bin.byteOffset + offset, acc.count * stride)
  return { acc, view, stride }
}
function loadAllTriangles(json, bin) {
  const tris = []
  const scene = json.scenes[json.scene ?? 0]
  const walk = (nodeIdx) => {
    const node = json.nodes[nodeIdx]
    if (typeof node.mesh === 'number') {
      const mesh = json.meshes[node.mesh]
      for (const prim of mesh.primitives) {
        const posIdx = prim.attributes.POSITION
        if (typeof posIdx !== 'number') continue
        const { acc: posAcc, view: posView, stride: posStride } = accessorReader(json, bin, posIdx)
        const positions = []
        for (let i = 0; i < posAcc.count; i++) {
          positions.push([
            posView.getFloat32(i * posStride, true),
            posView.getFloat32(i * posStride + 4, true),
            posView.getFloat32(i * posStride + 8, true),
          ])
        }
        if (typeof prim.indices === 'number') {
          const { acc: idxAcc, view: idxView } = accessorReader(json, bin, prim.indices)
          const getIdx = idxAcc.componentType === 5123
            ? (i) => idxView.getUint16(i * 2, true)
            : idxAcc.componentType === 5125
              ? (i) => idxView.getUint32(i * 4, true)
              : (i) => idxView.getUint8(i)
          for (let i = 0; i < idxAcc.count; i += 3) {
            tris.push([positions[getIdx(i)], positions[getIdx(i + 1)], positions[getIdx(i + 2)]])
          }
        } else {
          for (let i = 0; i < positions.length; i += 3) {
            tris.push([positions[i], positions[i + 1], positions[i + 2]])
          }
        }
      }
    }
    for (const child of node.children || []) walk(child)
  }
  for (const root of scene.nodes) walk(root)
  return tris
}
function rayTri(o, d, v0, v1, v2) {
  const e1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]]
  const e2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]]
  const h = [
    d[1] * e2[2] - d[2] * e2[1],
    d[2] * e2[0] - d[0] * e2[2],
    d[0] * e2[1] - d[1] * e2[0],
  ]
  const a = e1[0] * h[0] + e1[1] * h[1] + e1[2] * h[2]
  if (Math.abs(a) < 1e-12) return null
  const f = 1 / a
  const s = [o[0] - v0[0], o[1] - v0[1], o[2] - v0[2]]
  const u = f * (s[0] * h[0] + s[1] * h[1] + s[2] * h[2])
  if (u < 0 || u > 1) return null
  const q = [
    s[1] * e1[2] - s[2] * e1[1],
    s[2] * e1[0] - s[0] * e1[2],
    s[0] * e1[1] - s[1] * e1[0],
  ]
  const v = f * (d[0] * q[0] + d[1] * q[1] + d[2] * q[2])
  if (v < 0 || u + v > 1) return null
  const t = f * (e2[0] * q[0] + e2[1] * q[1] + e2[2] * q[2])
  return t > 0 ? t : null
}
function normalizeTris(tris, bbox, shaftLenMm) {
  let mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity]
  for (const t of tris) for (const v of t) for (let i = 0; i < 3; i++) {
    if (v[i] < mn[i]) mn[i] = v[i]
    if (v[i] > mx[i]) mx[i] = v[i]
  }
  const sz = [mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]]
  const ctr = [(mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2, (mn[2] + mx[2]) / 2]
  const targetZ = Math.max(0.001, bbox[2] - shaftLenMm)
  const s = [bbox[0] / sz[0], bbox[1] / sz[1], targetZ / sz[2]]
  const zOff = -shaftLenMm / 2
  return tris.map(t => t.map(v => [
    (v[0] - ctr[0]) * s[0],
    (v[1] - ctr[1]) * s[1],
    (v[2] - ctr[2]) * s[2] + zOff,
  ]))
}
function raycast(origin, dir, tris) {
  let bestT = Infinity, hit = null
  for (const t of tris) {
    const r = rayTri(origin, dir, t[0], t[1], t[2])
    if (r !== null && r < bestT) { bestT = r; hit = r }
  }
  return hit === null ? null : { t: hit, point: [origin[0] + dir[0] * hit, origin[1] + dir[1] * hit, origin[2] + dir[2] * hit] }
}

async function analyze(presetId, glbFile, bbox, shaftLenMm, connOriginZ, connAxis) {
  const glbPath = path.join(repoRoot, 'src/public/meshes/glb', glbFile)
  const { json, bin } = await loadGlbDoc(glbPath)
  const tris = normalizeTris(loadAllTriangles(json, bin), bbox, shaftLenMm)

  let mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity]
  for (const t of tris) for (const v of t) for (let i = 0; i < 3; i++) {
    if (v[i] < mn[i]) mn[i] = v[i]
    if (v[i] > mx[i]) mx[i] = v[i]
  }
  console.log(`\n=== ${presetId} ===  bbox=[${bbox.join(',')}] shaftLen=${shaftLenMm}mm`)
  console.log(`  body AABB: X=[${mn[0].toFixed(2)},${mx[0].toFixed(2)}] Y=[${mn[1].toFixed(2)},${mx[1].toFixed(2)}] Z=[${mn[2].toFixed(2)},${mx[2].toFixed(2)}]`)

  const perpMin = Math.min(bbox[0], bbox[1]) // perpendicular to Z axis
  for (const radiusFactor of [0.1, 0.2, 0.3, 0.4, 0.5]) {
    const sampleR = radiusFactor * perpMin
    const gridN = 9
    const samples = []
    for (let i = 0; i < gridN; i++) {
      for (let j = 0; j < gridN; j++) {
        const uFrac = (i + 0.5) / gridN * 2 - 1
        const vFrac = (j + 0.5) / gridN * 2 - 1
        if (uFrac * uFrac + vFrac * vFrac > 1) continue
        samples.push([uFrac * sampleR, vFrac * sampleR])
      }
    }
    const hits = []
    // Ray origin: below connector (for bottom face axis=(0,0,-1), "below" = ~bbox.z
    // below the connector). Direction opposite of connAxis.
    const axis = connAxis // [0,0,-1] for bottom, [0,0,1] for top
    for (const [x, y] of samples) {
      const origin = [x, y, connOriginZ + axis[2] * 100] // 100mm outside along axis
      const dir = [-axis[0], -axis[1], -axis[2]]
      const h = raycast(origin, dir, tris)
      if (!h) continue
      // axial along axis from connector origin
      const dx = h.point[0] - 0
      const dy = h.point[1] - 0
      const dz = h.point[2] - connOriginZ
      const axial = dx * axis[0] + dy * axis[1] + dz * axis[2]
      hits.push(axial)
    }
    hits.sort((a, b) => a - b)
    if (hits.length === 0) {
      console.log(`  r=${(radiusFactor * 100).toFixed(0)}% (${sampleR.toFixed(1)}mm): no hits`)
      continue
    }
    const p10 = hits[Math.floor(0.1 * hits.length)]
    const p50 = hits[Math.floor(0.5 * hits.length)]
    const p90 = hits[Math.floor(0.9 * hits.length)]
    console.log(`  sampleR=${(radiusFactor * 100).toFixed(0)}% (${sampleR.toFixed(1)}mm): hits=${hits.length}/${samples.length}  axial min=${hits[0].toFixed(2)} p10=${p10.toFixed(2)} p50=${p50.toFixed(2)} p90=${p90.toFixed(2)} max=${hits[hits.length - 1].toFixed(2)}`)
  }
}

async function main() {
  console.log(`\n────── HIGH_TORQUE bottom connector at z=-17 ──────`)
  await analyze('actuator_servo_high_torque', 'servo_high_torque.glb', [46.5, 36, 34], 5, -17, [0, 0, -1])
  console.log(`\n────── HIGH_TORQUE top connector at z=+12 ──────`)
  await analyze('actuator_servo_high_torque', 'servo_high_torque.glb', [46.5, 36, 34], 5, 12, [0, 0, 1])
  console.log(`\n────── HEAVY_DUTY bottom connector at z=-27 ──────`)
  await analyze('actuator_servo_heavy_duty', 'servo_high_torque.glb', [54, 42, 54], 5, -27, [0, 0, -1])
  console.log(`\n────── STANDARD bottom connector at z=-18.5 ──────`)
  await analyze('actuator_servo_standard', 'servo_standard.glb', [40, 20, 37], 0, -18.5, [0, 0, -1])
}

main().catch(e => { console.error(e); process.exit(1) })

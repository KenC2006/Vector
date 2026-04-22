// Drill into actuator_servo_high_torque's RENDERED top face to check whether
// a chamfer actually exists in the mesh (not just bbox). Simulates the
// runtime pipeline: raw GLB → rotation override → per-axis scale + shaft
// overlay shift → sample the top face surface via a grid of down-pointing
// rays. Reports axial depth histogram binned by radial band.
//
// This is a refinement of analyze-contact.mjs — instead of listing all
// vertices in a cylinder (noisy), we actually RAYCAST down from above the
// body top at a grid of tangent points, so we only get SURFACE hits and
// can see whether the rim is chamfered.

import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const here = path.dirname(fileURLToPath(import.meta.url))
const repoRoot = path.resolve(here, '..')

// Re-use the GLB load helpers from analyze-contact.mjs inline (same code,
// didn't extract a helper library to keep the script self-contained).
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
  // Flatten every triangle from every primitive (no node-tree walking —
  // servo_high_torque has 1 primitive under identity transform).
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

// Ray-triangle intersection (Möller-Trumbore). Returns t (distance along
// ray) or null. Ray = origin + t*dir.
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

function xformTris(tris, M) {
  // Apply 4x4 column-major M to each triangle vertex.
  function xp(p) {
    return [
      M[0] * p[0] + M[4] * p[1] + M[8] * p[2] + M[12],
      M[1] * p[0] + M[5] * p[1] + M[9] * p[2] + M[13],
      M[2] * p[0] + M[6] * p[1] + M[10] * p[2] + M[14],
    ]
  }
  return tris.map(t => [xp(t[0]), xp(t[1]), xp(t[2])])
}

function normalizeTris(tris, rot, bbox, shaftLenMm) {
  // Step 1: rotation override (if any). Apply to triangle vertices.
  let out = tris
  if (rot) {
    const [rx, ry, rz] = rot
    const cx = Math.cos(rx), sx = Math.sin(rx)
    const cy = Math.cos(ry), sy = Math.sin(ry)
    const cz = Math.cos(rz), sz = Math.sin(rz)
    // three.js Euler 'XYZ' order matrix, column-major.
    const R = [
      cy * cz, cy * sz, -sy, 0,
      sx * sy * cz - cx * sz, sx * sy * sz + cx * cz, sx * cy, 0,
      cx * sy * cz + sx * sz, cx * sy * sz - sx * cz, cx * cy, 0,
      0, 0, 0, 1,
    ]
    out = xformTris(out, R)
  }
  // Step 2: AABB → per-axis scale + recenter + shaft-overlay shift.
  let mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity]
  for (const t of out) for (const v of t) for (let i = 0; i < 3; i++) {
    if (v[i] < mn[i]) mn[i] = v[i]
    if (v[i] > mx[i]) mx[i] = v[i]
  }
  const sz = [mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]]
  const ctr = [(mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2, (mn[2] + mx[2]) / 2]
  const targetZ = Math.max(0.001, bbox[2] - shaftLenMm)
  const s = [bbox[0] / sz[0], bbox[1] / sz[1], targetZ / sz[2]]
  const zOff = -shaftLenMm / 2
  out = out.map(t => t.map(v => [
    (v[0] - ctr[0]) * s[0],
    (v[1] - ctr[1]) * s[1],
    (v[2] - ctr[2]) * s[2] + zOff,
  ]))
  return out
}

function raycastDown(origin, dir, tris) {
  let bestT = Infinity, hit = null
  for (const t of tris) {
    const r = rayTri(origin, dir, t[0], t[1], t[2])
    if (r !== null && r < bestT) { bestT = r; hit = r }
  }
  return hit === null ? null : {
    t: hit,
    point: [origin[0] + dir[0] * hit, origin[1] + dir[1] * hit, origin[2] + dir[2] * hit],
  }
}

async function main() {
  const presetId = 'actuator_servo_high_torque'
  const glbPath = path.join(repoRoot, 'src/public/meshes/glb/servo_high_torque.glb')
  const bbox = [46.5, 36, 34]
  const rot = null // no rotation override for this GLB
  const shaftLenMm = 5 // SHAFT_OVERLAYS['actuator_servo_high_torque']

  const { json, bin } = await loadGlbDoc(glbPath)
  const rawTris = loadAllTriangles(json, bin)
  const tris = normalizeTris(rawTris, rot, bbox, shaftLenMm)

  console.log(`\n=== ${presetId} (${rawTris.length} triangles) ===`)
  console.log(`bbox=[${bbox.join(',')}] shaftLen=${shaftLenMm}mm → body occupies -17..+12mm Z`)

  // Replicate the RUNTIME shaft overlay cylinder (radius 4mm, height 5mm,
  // z = 12..17). For ICP sampling purposes this adds the tip surface.
  // Build as 2 triangles at the top cap of the cylinder; that's what
  // raycasting from above would hit at r<4mm.
  const shaftR = 4
  const shaftTopZ = 17
  // Procedural cylinder has a top cap; approximate with a regular n-gon
  // fan. 24 segments mirrors CylinderGeometry defaults.
  const SEG = 24
  for (let i = 0; i < SEG; i++) {
    const a0 = (i / SEG) * Math.PI * 2
    const a1 = ((i + 1) / SEG) * Math.PI * 2
    tris.push([
      [0, 0, shaftTopZ],
      [shaftR * Math.cos(a0), shaftR * Math.sin(a0), shaftTopZ],
      [shaftR * Math.cos(a1), shaftR * Math.sin(a1), shaftTopZ],
    ])
  }

  // Sample the body-top connector: origin (0,0,12), axis (0,0,1).
  // Raycast from (x, y, 50) going (0,0,-1) over a grid of (x, y) in a
  // 14.4 mm radius disc (0.4 × 36 mm perpendicular extent = 14.4 mm,
  // matches SAMPLE_RADIUS_FACTOR in contactCleanup.ts).
  const sampleR = 14.4
  const samples = []
  const gridN = 9 // 81-cell grid, ≈64 hits after disc clip
  for (let i = 0; i < gridN; i++) {
    for (let j = 0; j < gridN; j++) {
      const uFrac = (i + 0.5) / gridN * 2 - 1
      const vFrac = (j + 0.5) / gridN * 2 - 1
      if (uFrac * uFrac + vFrac * vFrac > 1) continue
      const x = uFrac * sampleR
      const y = vFrac * sampleR
      samples.push({ x, y, r: Math.hypot(x, y) })
    }
  }

  const hits = []
  for (const s of samples) {
    const h = raycastDown([s.x, s.y, 50], [0, 0, -1], tris)
    if (!h) continue
    // Axial coordinate relative to connector origin (0,0,12).
    const axial = h.point[2] - 12
    hits.push({ r: s.r, axial })
  }
  hits.sort((a, b) => a.axial - b.axial)

  console.log(`\nSamples: ${samples.length}, hits: ${hits.length}`)
  console.log(`Hit axial (mm, + = above body-top = shaft or asymmetry):`)
  console.log(`  min  = ${hits[0].axial.toFixed(3)}`)
  console.log(`  p10  = ${hits[Math.floor(0.1 * hits.length)].axial.toFixed(3)}`)
  console.log(`  p25  = ${hits[Math.floor(0.25 * hits.length)].axial.toFixed(3)}`)
  console.log(`  p50  = ${hits[Math.floor(0.5 * hits.length)].axial.toFixed(3)}`)
  console.log(`  p75  = ${hits[Math.floor(0.75 * hits.length)].axial.toFixed(3)}`)
  console.log(`  p90  = ${hits[Math.floor(0.9 * hits.length)].axial.toFixed(3)}`)
  console.log(`  max  = ${hits[hits.length - 1].axial.toFixed(3)}`)

  const bands = [
    [0, 0.25],
    [0.25, 0.50],
    [0.50, 0.75],
    [0.75, 1.00],
  ]
  for (const [lo, hi] of bands) {
    const inBand = hits.filter(h => h.r >= lo * sampleR && h.r < hi * sampleR)
    if (inBand.length === 0) continue
    const sorted = [...inBand].sort((a, b) => a.axial - b.axial)
    console.log(`  r${(lo * 100).toFixed(0)}-${(hi * 100).toFixed(0)}% (r=${(lo * sampleR).toFixed(1)}-${(hi * sampleR).toFixed(1)}mm): n=${inBand.length} min=${sorted[0].axial.toFixed(3)} p50=${sorted[Math.floor(sorted.length / 2)].axial.toFixed(3)} max=${sorted[sorted.length - 1].axial.toFixed(3)}`)
  }

  // What does my ICP algorithm compute for a flat-coupler-on-chamfered-servo?
  // Parent coupler bottom = flat, axial_p = 0 everywhere.
  // Child servo top = hits above. axial_c = hit.axial.
  // Gap = -axial_p - axial_c = -hit.axial.
  // Sort ascending: negative (shaft), zero (body flat), positive (chamfer if any).
  // Wait, hit.axial = 0 for flat, +5 for shaft tip (r<4), <0 for chamfer.
  //   So gaps = -hit.axial: 0 for flat, -5 for shaft, >0 for chamfer.
  // Sorted ascending: [-5, -5, -5, -5, 0, 0, 0, ..., +chamfer, +chamfer, ...]
  // p90 = the value at index 0.9*N. If N=60, index 54. If 60 samples = 4 shaft + 40 flat + 16 chamfer (example), p90 ≈ chamfer depth.
  const gaps = hits.map(h => -h.axial)
  gaps.sort((a, b) => a - b)
  const p90 = gaps[Math.floor(0.9 * gaps.length)]
  console.log(`\nICP nudge estimate (p90 of -axial_c, parent assumed flat):`)
  console.log(`  p10=${gaps[Math.floor(0.1 * gaps.length)].toFixed(3)}mm`)
  console.log(`  p50=${gaps[Math.floor(0.5 * gaps.length)].toFixed(3)}mm`)
  console.log(`  p90=${p90.toFixed(3)}mm`)
  console.log(`  → Expected nudge (capped 3mm): ${Math.max(0, Math.min(p90, 3)).toFixed(3)}mm`)
}

main().catch(e => { console.error(e); process.exit(1) })

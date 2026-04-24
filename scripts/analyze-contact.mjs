// Ground-truth contact-face analysis for Step 2 ICP nudge.
//
// Loads each relevant GLB, applies the runtime normalization
// (rotation override → per-axis scale → optional shaft-overlay shift)
// EXACTLY the way src/src/richVisuals/index.ts::applyMeshToLink does at
// runtime, then samples every vertex inside a disc around each authored
// connector and reports the axial height distribution.
//
// Why this exists: Phase 3 of the Session-3 diagnostic. The user's smoke
// test showed ICP returning 0 nudge on the known-chamfered
// actuator_servo_high_torque top. We need the actual mesh-measured
// chamfer depth to decide whether ICP is broken, the GLB isn't chamfered,
// or the sampling radius is wrong.
//
// Usage: node scripts/analyze-contact.mjs [--json] > scripts/contact-analysis-output.txt

import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const here = path.dirname(fileURLToPath(import.meta.url))
const repoRoot = path.resolve(here, '..')

// ── Runtime-normalization constants (mirror src/src/richVisuals/meshOverrides.ts) ──

const ROTATION_OVERRIDES = {
  structural_servo_coupler_disc: [Math.PI / 2, 0, 0],
  transmission_bearing_deep_groove: [0, Math.PI / 2, 0],
  transmission_bearing_large: [0, Math.PI / 2, 0],
  compute_sbc_small: [Math.PI / 2, 0, 0],
  compute_motor_driver_dual: [Math.PI / 2, 0, 0],
  mobility_mecanum_wheel: [Math.PI / 2, 0, 0],
  power_lipo_3s_2200: [Math.PI / 2, 0, Math.PI / 2],
  power_lipo_4s_5000: [Math.PI / 2, 0, Math.PI / 2],
  power_lipo_6s_10000: [Math.PI / 2, 0, Math.PI / 2],
}

const SHAFT_OVERLAYS = {
  actuator_servo_high_torque: { shaft_length_mm: 5, shaft_radius_mm: 4 },
  actuator_servo_heavy_duty: { shaft_length_mm: 5, shaft_radius_mm: 6 },
}

// Preset-id → GLB file (mirrors MESH_OVERRIDES entries we care about).
const COMPONENTS = [
  {
    id: 'actuator_servo_high_torque',
    glb: 'servo_high_torque.glb',
    bbox: [46.5, 36, 34],
  },
  {
    id: 'actuator_servo_heavy_duty',
    glb: 'servo_high_torque.glb', // shared GLB
    bbox: [54, 42, 54],
  },
  {
    id: 'actuator_servo_standard',
    glb: 'servo_standard.glb',
    bbox: [40, 20, 37],
  },
  {
    id: 'actuator_servo_micro',
    glb: 'servo_small.glb',
    bbox: [23, 12.2, 29],
  },
  {
    id: 'structural_servo_coupler_disc',
    glb: 'servo_coupler_disc.glb',
    bbox: [32, 32, 8],
  },
]

// ── GLB parser (vertex-level, reads POSITION accessor binary data) ─────────

function loadGlbDoc(filePath) {
  return fs.readFile(filePath).then(buf => {
    const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength)
    const magic = dv.getUint32(0, true)
    if (magic !== 0x46546c67) throw new Error(`not a GLB: ${filePath}`)
    const jsonChunkLength = dv.getUint32(12, true)
    const jsonChunkType = dv.getUint32(16, true)
    if (jsonChunkType !== 0x4e4f534a) throw new Error(`expected JSON chunk`)
    const jsonBytes = buf.subarray(20, 20 + jsonChunkLength)
    const json = JSON.parse(new TextDecoder().decode(jsonBytes))
    const binOffset = 20 + jsonChunkLength
    const binLength = dv.getUint32(binOffset, true)
    const binType = dv.getUint32(binOffset + 4, true)
    if (binType !== 0x004e4942) throw new Error(`expected BIN chunk`)
    const bin = buf.subarray(binOffset + 8, binOffset + 8 + binLength)
    return { json, bin }
  })
}

function accessorPositions(json, bin, accIdx) {
  const acc = json.accessors[accIdx]
  if (acc.componentType !== 5126 || acc.type !== 'VEC3') {
    throw new Error(`unsupported position accessor: type=${acc.type} ct=${acc.componentType}`)
  }
  const bv = json.bufferViews[acc.bufferView]
  const offset = (bv.byteOffset || 0) + (acc.byteOffset || 0)
  const stride = bv.byteStride || 12
  const out = []
  const view = new DataView(bin.buffer, bin.byteOffset + offset, acc.count * stride)
  for (let i = 0; i < acc.count; i++) {
    const off = i * stride
    out.push([
      view.getFloat32(off, true),
      view.getFloat32(off + 4, true),
      view.getFloat32(off + 8, true),
    ])
  }
  return out
}

function nodeLocalMat(node) {
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
function mulMat(a, b) {
  const out = new Array(16).fill(0)
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
    let v = 0
    for (let k = 0; k < 4; k++) v += a[k * 4 + r] * b[c * 4 + k]
    out[c * 4 + r] = v
  }
  return out
}
function applyMat(m, p) {
  return [
    m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12],
    m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13],
    m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14],
  ]
}
function eulerXYZMat(r) {
  // intrinsic XYZ: Rz * Ry * Rx applied to column vec → in column-major:
  const [rx, ry, rz] = r
  const cx = Math.cos(rx), sx = Math.sin(rx)
  const cy = Math.cos(ry), sy = Math.sin(ry)
  const cz = Math.cos(rz), sz = Math.sin(rz)
  // Using three.js Euler 'XYZ' order convention (Rx.Ry.Rz applied LEFT→RIGHT to point = p' = Rx Ry Rz p)
  // Equivalent rotation matrix below. Column-major 4x4.
  return [
    cy * cz,
    cy * sz,
    -sy,
    0,
    sx * sy * cz - cx * sz,
    sx * sy * sz + cx * cz,
    sx * cy,
    0,
    cx * sy * cz + sx * sz,
    cx * sy * sz - sx * cz,
    cx * cy,
    0,
    0, 0, 0, 1,
  ]
}

function walkCollect(json, bin, nodeIdx, parent, outVerts) {
  const node = json.nodes[nodeIdx]
  const world = mulMat(parent, nodeLocalMat(node))
  if (typeof node.mesh === 'number') {
    const mesh = json.meshes[node.mesh]
    for (const prim of mesh.primitives) {
      const posIdx = prim.attributes?.POSITION
      if (typeof posIdx !== 'number') continue
      const verts = accessorPositions(json, bin, posIdx)
      for (const v of verts) outVerts.push(applyMat(world, v))
    }
  }
  for (const child of node.children || []) walkCollect(json, bin, child, world, outVerts)
}

async function loadAllVerts(glbPath) {
  const { json, bin } = await loadGlbDoc(glbPath)
  const scene = json.scenes[json.scene ?? 0]
  const identity = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
  const verts = []
  for (const root of scene.nodes) walkCollect(json, bin, root, identity, verts)
  return verts
}

// ── Runtime normalization ──────────────────────────────────────────────────

/**
 * Replicate applyMeshToLink's vertex transform for a given preset:
 *   1. Apply rotation override (bakes into vertices).
 *   2. Compute AABB of rotated verts.
 *   3. Per-axis scale so AABB extents match (bbox.x, bbox.y, targetZ)
 *      where targetZ = bbox.z - shaft_length (or bbox.z if no overlay).
 *   4. Recenter on (0,0,0).
 *   5. Shift Z by -shaft_length/2 if shaft overlay (body sits low, shaft above).
 *
 * Returns { verts (normalized), bodyTopZ, bboxTopZ, shaftLenMm }.
 */
function normalizeVerts(rawVerts, presetId, bboxMm) {
  const rot = ROTATION_OVERRIDES[presetId] ?? null
  const overlay = SHAFT_OVERLAYS[presetId] ?? null
  let verts = rawVerts
  if (rot) {
    const R = eulerXYZMat(rot)
    verts = verts.map(v => applyMat(R, v))
  }
  // AABB
  let mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity]
  for (const v of verts) {
    for (let i = 0; i < 3; i++) {
      if (v[i] < mn[i]) mn[i] = v[i]
      if (v[i] > mx[i]) mx[i] = v[i]
    }
  }
  const size = [mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]]
  const center = [(mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2, (mn[2] + mx[2]) / 2]
  const shaftLenMm = overlay ? overlay.shaft_length_mm : 0
  const targetZ = Math.max(0.001, bboxMm[2] - shaftLenMm)
  const s = [bboxMm[0] / size[0], bboxMm[1] / size[1], targetZ / size[2]]
  // Scale + recenter + shaft-offset:
  const zOff = -shaftLenMm / 2
  const nVerts = verts.map(v => [
    (v[0] - center[0]) * s[0],
    (v[1] - center[1]) * s[1],
    (v[2] - center[2]) * s[2] + zOff,
  ])
  // In the final rendered frame, body top = (bbox.z - shaft_len) / 2 + zOff
  //   = (bbox.z - shaft_len) / 2 - shaft_len/2
  //   = bbox.z/2 - shaft_len.
  // bbox top including the procedural shaft = bbox.z/2.
  const bboxTopZ = bboxMm[2] / 2
  const bodyTopZ = bboxTopZ - shaftLenMm
  return { verts: nVerts, bodyTopZ, bboxTopZ, shaftLenMm, scaleApplied: s }
}

// ── Disc sampling + histogram ──────────────────────────────────────────────

/**
 * For a given connector (origin, axis) and a sample radius R:
 * project every vertex whose (u, v) offset from origin is within R onto
 * the axis, and return their axial coords relative to the origin. Returns
 * an array of axial values (mm) plus their (uFrac, vFrac) radii for
 * downstream histogram work.
 */
function sampleContactProfile(verts, originMm, axis, radiusMm) {
  // Tangent basis identical to contactCleanup.ts / buildInPlaneBasis.
  const n = norm3(axis)
  const ax = Math.abs(n[0]), ay = Math.abs(n[1]), az = Math.abs(n[2])
  let helper
  if (ax <= ay && ax <= az) helper = [1, 0, 0]
  else if (ay <= az) helper = [0, 1, 0]
  else helper = [0, 0, 1]
  const u = norm3(cross3(n, helper))
  const v = norm3(cross3(n, u))
  const samples = []
  for (const P of verts) {
    const d = [P[0] - originMm[0], P[1] - originMm[1], P[2] - originMm[2]]
    const uu = dot3(d, u)
    const vv = dot3(d, v)
    const rho = Math.hypot(uu, vv)
    if (rho > radiusMm) continue
    const axial = dot3(d, n)
    samples.push({ axial, rho, u: uu, v: vv })
  }
  return samples
}

function norm3(a) {
  const L = Math.hypot(a[0], a[1], a[2]) || 1
  return [a[0] / L, a[1] / L, a[2] / L]
}
function cross3(a, b) {
  return [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]]
}
function dot3(a, b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }

function percentile(arr, p) {
  if (arr.length === 0) return NaN
  const s = [...arr].sort((a, b) => a - b)
  const idx = Math.min(s.length - 1, Math.max(0, Math.floor(s.length * p)))
  return s[idx]
}

function statsFor(samples) {
  const ax = samples.map(s => s.axial)
  ax.sort((a, b) => a - b)
  const n = ax.length
  if (n === 0) return { n: 0 }
  return {
    n,
    min: ax[0],
    p10: percentile(ax, 0.1),
    p50: percentile(ax, 0.5),
    p90: percentile(ax, 0.9),
    max: ax[n - 1],
    mean: ax.reduce((a, b) => a + b, 0) / n,
  }
}

// ── Main ────────────────────────────────────────────────────────────────────

async function loadPresetConnectors() {
  const raw = await fs.readFile(path.join(repoRoot, 'src', 'public', 'generic_presets.json'), 'utf8')
  const data = JSON.parse(raw)
  const byId = {}
  for (const cat of Object.values(data.categories)) {
    for (const c of cat.components) {
      if (c.connectors) byId[c.id] = c.connectors
    }
  }
  return byId
}

async function main() {
  const jsonMode = process.argv.includes('--json')
  const connectorsById = await loadPresetConnectors()
  const results = []
  for (const comp of COMPONENTS) {
    const glbPath = path.join(repoRoot, 'src', 'public', 'meshes', 'glb', comp.glb)
    let rawVerts
    try {
      rawVerts = await loadAllVerts(glbPath)
    } catch (e) {
      results.push({ id: comp.id, error: `load failed: ${e.message}` })
      continue
    }
    const { verts, bodyTopZ, bboxTopZ, shaftLenMm, scaleApplied } = normalizeVerts(rawVerts, comp.id, comp.bbox)
    const connectors = connectorsById[comp.id] || []
    const connectorResults = []
    for (const conn of connectors) {
      // Face radius = 0.4 × min perpendicular extent (match contactCleanup default).
      // Compute perp extents from bbox:
      const absA = [Math.abs(conn.axis_xyz[0]), Math.abs(conn.axis_xyz[1]), Math.abs(conn.axis_xyz[2])]
      const perp = []
      if (absA[0] < 0.99) perp.push(comp.bbox[0])
      if (absA[1] < 0.99) perp.push(comp.bbox[1])
      if (absA[2] < 0.99) perp.push(comp.bbox[2])
      const minPerp = perp.length > 0 ? Math.min(...perp) : Math.max(...comp.bbox)
      const sampleRadius = 0.4 * minPerp
      const samples = sampleContactProfile(verts, conn.origin_xyz_mm, conn.axis_xyz, sampleRadius)
      const st = statsFor(samples)
      // Also break down by radial bands (inner vs rim) to detect through-holes / shaft columns.
      const bands = [
        { tag: 'center r<25%', lo: 0, hi: 0.25 * sampleRadius },
        { tag: 'r25-50%',       lo: 0.25 * sampleRadius, hi: 0.50 * sampleRadius },
        { tag: 'r50-75%',       lo: 0.50 * sampleRadius, hi: 0.75 * sampleRadius },
        { tag: 'r75-100%',      lo: 0.75 * sampleRadius, hi: sampleRadius },
      ].map(b => {
        const inBand = samples.filter(s => s.rho >= b.lo && s.rho < b.hi)
        return { tag: b.tag, n: inBand.length, stats: statsFor(inBand) }
      })
      connectorResults.push({
        connectorId: conn.id,
        origin_mm: conn.origin_xyz_mm,
        axis: conn.axis_xyz,
        engagement_depth_mm: conn.engagement_depth_mm ?? 0,
        sampleRadius_mm: sampleRadius,
        stats: st,
        bands,
      })
    }
    results.push({
      id: comp.id,
      glb: comp.glb,
      bbox_mm: comp.bbox,
      scaleApplied,
      shaftLenMm,
      bodyTopZ_mm: bodyTopZ,
      bboxTopZ_mm: bboxTopZ,
      vertexCount: verts.length,
      connectors: connectorResults,
    })
  }
  if (jsonMode) {
    console.log(JSON.stringify(results, null, 2))
    return
  }
  // Pretty-print
  for (const r of results) {
    console.log(`\n=== ${r.id} ===`)
    if (r.error) { console.log(`  ERROR: ${r.error}`); continue }
    console.log(`  GLB: ${r.glb} | bbox=[${r.bbox_mm.join(',')}] mm | verts=${r.vertexCount}`)
    console.log(`  scale applied: [${r.scaleApplied.map(v => v.toFixed(3)).join(', ')}], shaftLen=${r.shaftLenMm}mm`)
    console.log(`  bodyTopZ=${r.bodyTopZ_mm.toFixed(2)} mm | bboxTopZ=${r.bboxTopZ_mm.toFixed(2)} mm`)
    for (const c of r.connectors) {
      console.log(`  -- connector "${c.connectorId}" origin=[${c.origin_mm.join(',')}] axis=[${c.axis.join(',')}] engagement=${c.engagement_depth_mm}mm, sample-r=${c.sampleRadius_mm.toFixed(1)}mm`)
      if (c.stats.n === 0) { console.log(`       no vertices in radius`); continue }
      const s = c.stats
      console.log(`       axial (mm, + = OUTWARD from body) n=${s.n}: min=${s.min.toFixed(2)} p10=${s.p10.toFixed(2)} p50=${s.p50.toFixed(2)} p90=${s.p90.toFixed(2)} max=${s.max.toFixed(2)} mean=${s.mean.toFixed(2)}`)
      for (const b of c.bands) {
        if (b.n === 0) { console.log(`       ${b.tag}: no samples`); continue }
        console.log(`       ${b.tag}: n=${b.n} min=${b.stats.min.toFixed(2)} p50=${b.stats.p50.toFixed(2)} p90=${b.stats.p90.toFixed(2)} max=${b.stats.max.toFixed(2)}`)
      }
    }
  }
}

main().catch(e => { console.error(e); process.exit(1) })

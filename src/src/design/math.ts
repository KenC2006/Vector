/**
 * Placement math of the design compiler (core/designer/geometry.py and the
 * anchor/orientation rules of compile.py), ported 1:1 so the live placement
 * preview lands exactly where the compiler will put the part.
 * Units: mm, radians inside, degrees at the spec boundary.
 */
import type { AtSpec, AxisSpec, CompiledPart, DesignPart, Mat3, Primitive, Vec3 } from './types'
import { componentSizeMm, getComponent, type CatalogComponent } from './catalog.ts'

// ── vectors / matrices ──────────────────────────────────────────────────────

export const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
export const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
export const scale = (a: Vec3, s: number): Vec3 => [a[0] * s, a[1] * s, a[2] * s]
export const dot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
export const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0],
]
export const norm = (a: Vec3): number => Math.hypot(a[0], a[1], a[2])
export function unit(a: Vec3): Vec3 {
  const n = norm(a)
  if (n < 1e-9) throw new Error('zero-length direction')
  return [a[0] / n, a[1] / n, a[2] / n]
}

export const I3: Mat3 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
export const col = (R: Mat3, j: number): Vec3 => [R[0][j], R[1][j], R[2][j]]
export const fromCols = (x: Vec3, y: Vec3, z: Vec3): Mat3 => [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
export const mulV = (R: Mat3, v: Vec3): Vec3 => [dot(R[0], v), dot(R[1], v), dot(R[2], v)]
export const transpose = (R: Mat3): Mat3 => [[R[0][0], R[1][0], R[2][0]], [R[0][1], R[1][1], R[2][1]], [R[0][2], R[1][2], R[2][2]]]
export function mulM(A: Mat3, B: Mat3): Mat3 {
  const out = [[0, 0, 0], [0, 0, 0], [0, 0, 0]] as Mat3
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) out[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j]
  return out
}

// ── directions ──────────────────────────────────────────────────────────────

const AXIS_WORDS: Record<string, Vec3> = {
  '+x': [1, 0, 0], '-x': [-1, 0, 0], x: [1, 0, 0],
  '+y': [0, 1, 0], '-y': [0, -1, 0], y: [0, 1, 0],
  '+z': [0, 0, 1], '-z': [0, 0, -1], z: [0, 0, 1],
  forward: [1, 0, 0], back: [-1, 0, 0], backward: [-1, 0, 0],
  left: [0, 1, 0], right: [0, -1, 0], up: [0, 0, 1], down: [0, 0, -1],
}

export function vec(v: AxisSpec | Vec3): Vec3 {
  if (typeof v === 'string') {
    const w = AXIS_WORDS[v.trim().toLowerCase()]
    if (!w) throw new Error(`unknown direction '${v}'`)
    return [...w] as Vec3
  }
  return [Number(v[0]), Number(v[1]), Number(v[2])]
}

export function rotAxisAngle(axis: Vec3, angle: number): Mat3 {
  const a = unit(axis)
  const K: Mat3 = [[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]]
  const KK = mulM(K, K)
  const s = Math.sin(angle), c = 1 - Math.cos(angle)
  const out = [[0, 0, 0], [0, 0, 0], [0, 0, 0]] as Mat3
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) out[i][j] = I3[i][j] + s * K[i][j] + c * KK[i][j]
  return out
}

/** Rotation whose local +Z/+X point along the given world directions. */
export function frameFromAxes(zAxis?: AxisSpec, xAxis?: AxisSpec): Mat3 {
  const z = zAxis !== undefined ? unit(vec(zAxis)) : [0, 0, 1] as Vec3
  let x: Vec3
  if (xAxis !== undefined) x = unit(vec(xAxis))
  else x = Math.abs(z[0]) < 0.9 ? [1, 0, 0] : scale([0, 0, 1], -Math.sign(z[0]))
  x = sub(x, scale(z, dot(x, z)))
  if (norm(x) < 1e-6) x = Math.abs(z[1]) < 0.9 ? cross(z, [0, 1, 0]) : cross(z, [1, 0, 0])
  x = unit(x)
  return fromCols(x, cross(z, x), z)
}

/** Smallest rotation taking unit vector a onto b. */
export function rotationBetween(a: Vec3, b: Vec3): Mat3 {
  a = unit(a); b = unit(b)
  const c = dot(a, b)
  if (c > 1 - 1e-12) return I3.map(r => [...r]) as Mat3
  if (c < -1 + 1e-12) {
    let perp = cross(a, [1, 0, 0])
    if (norm(perp) < 1e-6) perp = cross(a, [0, 1, 0])
    return rotAxisAngle(perp, Math.PI)
  }
  return rotAxisAngle(cross(a, b), Math.acos(c))
}

function spinToMatchX(R: Mat3, axis: Vec3, wantX: Vec3): Mat3 {
  const x = col(R, 0)
  let xp = sub(x, scale(axis, dot(x, axis)))
  let wp = sub(wantX, scale(axis, dot(wantX, axis)))
  if (norm(xp) < 1e-6 || norm(wp) < 1e-6) return R
  xp = unit(xp); wp = unit(wp)
  const ang = Math.atan2(dot(axis, cross(xp, wp)), dot(xp, wp))
  return mulM(rotAxisAngle(axis, ang), R)
}

/** URDF rpy (fixed-axis XYZ = R = Rz·Ry·Rx), radians. */
export function rpyToMatrix(r: number, p: number, y: number): Mat3 {
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y)
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ]
}

// ── anchors ─────────────────────────────────────────────────────────────────

export const FACE_NORMALS: Record<string, Vec3> = {
  '+x': [1, 0, 0], '-x': [-1, 0, 0], '+y': [0, 1, 0], '-y': [0, -1, 0], '+z': [0, 0, 1], '-z': [0, 0, -1],
}
export const FACE_ALIASES: Record<string, string> = {
  front: '+x', back: '-x', left: '+y', right: '-y', top: '+z', bottom: '-z',
}

/** Everything the anchor math needs about one part. */
export interface PartGeom {
  name: string
  spec: DesignPart
  component: CatalogComponent | null
  size: Vec3
  centerLocal: Vec3
  R: Mat3
  p: Vec3
}

export interface Anchor {
  key: string
  /** Local point (part frame, mm). */
  p: Vec3
  /** Local outward normal, or null for centers. */
  n: Vec3 | null
  kind: 'connector' | 'face' | 'primitive' | 'center'
  cls?: string
}

function primRot(prim: Primitive): Mat3 {
  const r = prim.rpy_deg ?? [0, 0, 0]
  return rpyToMatrix(r[0] * Math.PI / 180, r[1] * Math.PI / 180, r[2] * Math.PI / 180)
}
function primHalf(prim: Primitive): Vec3 {
  if (prim.shape === 'box' || prim.shape === 'mesh') return scale(prim.size_mm ?? [1, 1, 1], 0.5)
  if (prim.shape === 'cylinder') return [prim.radius_mm ?? 0, prim.radius_mm ?? 0, (prim.length_mm ?? 0) / 2]
  const r = prim.radius_mm ?? 0
  return [r, r, r]
}
function primCenter(prim: Primitive): Vec3 {
  return prim.xyz_mm ? [...prim.xyz_mm] as Vec3 : [0, 0, 0]
}

/** Local size + bbox center of a part spec (mirrors compile._shell_bounds / catalog bounds). */
export function partExtent(spec: DesignPart): { size: Vec3; centerLocal: Vec3; component: CatalogComponent | null } {
  if (spec.shape) {
    if (spec.shape.length === 0) return { size: [0, 0, 0], centerLocal: [0, 0, 0], component: null }
    const lo: Vec3 = [Infinity, Infinity, Infinity], hi: Vec3 = [-Infinity, -Infinity, -Infinity]
    for (const prim of spec.shape) {
      const c = primCenter(prim), R = primRot(prim), h = primHalf(prim)
      for (let i = 0; i < 3; i++) {
        const e = Math.abs(R[i][0]) * h[0] + Math.abs(R[i][1]) * h[1] + Math.abs(R[i][2]) * h[2]
        lo[i] = Math.min(lo[i], c[i] - e)
        hi[i] = Math.max(hi[i], c[i] + e)
      }
    }
    return { size: sub(hi, lo), centerLocal: scale(add(hi, lo), 0.5), component: null }
  }
  const comp = getComponent(spec.component)
  if (!comp) throw new Error(`${spec.name}: unknown component '${spec.component}'`)
  return { size: componentSizeMm(comp, spec.length_mm), centerLocal: [0, 0, 0], component: comp }
}

/** All anchors a part exposes: connectors, bbox faces, shell primitive faces. */
export function partAnchors(g: Pick<PartGeom, 'spec' | 'component' | 'size' | 'centerLocal'>): Anchor[] {
  const out: Anchor[] = []
  for (const c of g.component?.connectors ?? []) {
    out.push({ key: c.id, p: [...c.origin_xyz_mm] as Vec3, n: unit(vec(c.axis_xyz)), kind: 'connector', cls: c.cls })
  }
  for (const [k, n] of Object.entries(FACE_NORMALS)) {
    out.push({ key: k, p: add(g.centerLocal, [n[0] * g.size[0] / 2, n[1] * g.size[1] / 2, n[2] * g.size[2] / 2]), n, kind: 'face' })
  }
  if (!g.component) {
    for (const prim of g.spec.shape ?? []) {
      if (!prim.name) continue
      const c = primCenter(prim), R = primRot(prim), h = primHalf(prim)
      for (const [k, nl] of Object.entries(FACE_NORMALS)) {
        out.push({
          key: `${prim.name}.${k}`,
          p: add(c, mulV(R, [nl[0] * h[0], nl[1] * h[1], nl[2] * h[2]])),
          n: mulV(R, nl),
          kind: 'primitive',
        })
      }
    }
  }
  return out
}

/** Local point + normal of one anchor key (mirrors Part.anchor). */
export function anchorLocal(g: Pick<PartGeom, 'name' | 'spec' | 'component' | 'size' | 'centerLocal'>, key: string): { p: Vec3; n: Vec3 | null } {
  let k = key.trim()
  k = FACE_ALIASES[k] ?? k
  if (k === 'center' || k === 'origin' || k === '') return { p: [0, 0, 0], n: null }
  if (!g.component && !k.includes('.')) {
    const prim = (g.spec.shape ?? []).find(q => q.name === k)
    if (prim) return { p: primCenter(prim), n: null }
  }
  if (!g.component && k.includes('.') && !(k in FACE_NORMALS)) {
    const [primName, faceRaw] = k.split('.', 2)
    const face = FACE_ALIASES[faceRaw] ?? faceRaw
    const prim = (g.spec.shape ?? []).find(q => q.name === primName)
    if (!prim) throw new Error(`${g.name}: no primitive named '${primName}'`)
    if (face === 'center' || face === '') return { p: primCenter(prim), n: null }
    const nl = FACE_NORMALS[face]
    if (!nl) throw new Error(`${g.name}: bad primitive face '${face}'`)
    const R = primRot(prim), h = primHalf(prim)
    return { p: add(primCenter(prim), mulV(R, [nl[0] * h[0], nl[1] * h[1], nl[2] * h[2]])), n: mulV(R, nl) }
  }
  const a = partAnchors(g).find(x => x.key === k)
  if (!a) throw new Error(`${g.name}: unknown anchor '${key}'`)
  return { p: a.p, n: a.n }
}

export const worldPoint = (g: PartGeom, local: Vec3): Vec3 => add(g.p, mulV(g.R, local))

// ── resolving a spec against placed parts ───────────────────────────────────

export function geomFromCompiled(name: string, spec: DesignPart, cp: CompiledPart): PartGeom {
  const component = spec.shape ? null : getComponent(spec.component)
  return { name, spec, component, size: cp.size, centerLocal: cp.center_local, R: cp.R, p: cp.p }
}

/** -> world point, world normal, referenced part/anchor (mirrors _resolve_point). */
export function resolveAt(at: AtSpec | undefined, placed: Map<string, PartGeom>): { pt: Vec3; n: Vec3 | null; ref: string | null; anchor: string | null } {
  if (at === undefined) return { pt: [0, 0, 0], n: null, ref: null, anchor: null }
  const obj = typeof at === 'string' ? { ref: at } : Array.isArray(at) ? null : at
  if (!obj) return { pt: vec(at as Vec3), n: null, ref: null, anchor: null }
  const dotI = obj.ref.indexOf('.')
  const pname = dotI < 0 ? obj.ref : obj.ref.slice(0, dotI)
  const anchor = dotI < 0 ? 'center' : obj.ref.slice(dotI + 1)
  const rp = placed.get(pname)
  if (!rp) throw new Error(`unknown part '${pname}'`)
  const loc = anchorLocal(rp, anchor)
  return {
    pt: add(worldPoint(rp, loc.p), obj.offset ? vec(obj.offset) : [0, 0, 0]),
    n: loc.n ? mulV(rp.R, loc.n) : null,
    ref: pname,
    anchor,
  }
}

/** World pose of a part spec given the parts already placed (mirrors the
 *  orientation block of compile._resolve_part; joints/rest angles excluded). */
export function poseForSpec(spec: DesignPart, placed: Map<string, PartGeom>): { R: Mat3; p: Vec3; geom: PartGeom } {
  const ext = partExtent(spec)
  const g: PartGeom = { name: spec.name, spec, component: ext.component, size: ext.size, centerLocal: ext.centerLocal, R: I3, p: [0, 0, 0] }
  const target = resolveAt(spec.at ?? [0, 0, 0], placed)
  const align = anchorLocal(g, spec.align ?? 'center')
  const explicit = spec.z_axis !== undefined || (spec.x_axis !== undefined && target.n === null)
  let R: Mat3
  if (explicit || align.n === null || target.n === null) {
    R = frameFromAxes(spec.z_axis, spec.x_axis)
  } else {
    const into = scale(target.n, -1)
    R = rotationBetween(align.n, into)
    if (spec.x_axis !== undefined) R = spinToMatchX(R, into, unit(vec(spec.x_axis)))
  }
  const spin = Number(spec.spin_deg ?? 0)
  if (spin) {
    const about = target.n !== null && !explicit ? scale(target.n, -1) : col(R, 2)
    R = mulM(rotAxisAngle(about, spin * Math.PI / 180), R)
  }
  const p = sub(target.pt, mulV(R, align.p))
  g.R = R
  g.p = p
  return { R, p, geom: g }
}

/** True when the part is flush-mated (orientation comes from the mate). */
export function isFlushMate(spec: DesignPart, placed: Map<string, PartGeom>): boolean {
  if (spec.z_axis !== undefined) return false
  try {
    const t = resolveAt(spec.at, placed)
    if (t.n === null) return false
    const ext = partExtent(spec)
    const g = { name: spec.name, spec, component: ext.component, size: ext.size, centerLocal: ext.centerLocal }
    return anchorLocal(g, spec.align ?? 'center').n !== null
  } catch {
    return false
  }
}

/** Rotation matrix → world directions of its local Z and X (for z_axis/x_axis). */
export function axesOf(R: Mat3): { z: Vec3; x: Vec3 } {
  const r6 = (v: Vec3): Vec3 => v.map(x => Math.round(x * 1e6) / 1e6 || 0) as Vec3
  return { z: r6(col(R, 2)), x: r6(col(R, 0)) }
}

/** Axis + angle of a rotation matrix. */
export function axisAngle(R: Mat3): { axis: Vec3; angle: number } {
  const c = Math.max(-1, Math.min(1, (R[0][0] + R[1][1] + R[2][2] - 1) / 2))
  const angle = Math.acos(c)
  if (angle < 1e-9) return { axis: [0, 0, 1], angle: 0 }
  let axis: Vec3 = [R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]]
  if (norm(axis) < 1e-9) {
    // 180°: axis from the largest diagonal of (R + I) / 2
    const i = [0, 1, 2].reduce((b, k) => (R[k][k] > R[b][b] ? k : b), 0)
    axis = [R[0][i] + (i === 0 ? 1 : 0), R[1][i] + (i === 1 ? 1 : 0), R[2][i] + (i === 2 ? 1 : 0)]
  }
  return { axis: unit(axis), angle }
}

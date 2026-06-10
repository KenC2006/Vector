/**
 * Custom link geometry — primitive-composition path for novel-mode robot bodies.
 *
 * When an AssemblyComponent declares a `link_geometry` array, the visual
 * resolver bypasses the preset-mesh/GLB pipeline and renders the link as a
 * union of authored URDF primitives (box / cylinder / sphere). This is the
 * "Option 1" path described in the design plan: instead of expanding the
 * preset catalog with every body-shape variant a designer might need,
 * Claude composes free-form body shells (humanoid torso, drone frame, snake
 * segment, tank hull, sculpture, ...) from URDF-legal primitives.
 *
 * The format is intentionally compact and prompt-friendly:
 *   { shape: 'box',      size_mm: [w, d, h], xyz_mm?, rpy?, color? }
 *   { shape: 'cylinder', radius_mm, length_mm,      xyz_mm?, rpy?, color? }
 *   { shape: 'sphere',   radius_mm,                 xyz_mm?,       color? }
 *
 * Per-primitive `xyz_mm` is the centre of that primitive in the LINK frame
 * (millimetres). `rpy` is radians (URDF convention). `color` is an optional
 * RGB triplet in [0,1] — if omitted the structural-grey palette is used.
 *
 * Authored bounding box / collision / mate connectors / inertia are all
 * derived from the AABB union of the primitives, so a body shell composed
 * here behaves like any other catalog component for placement, physics, and
 * downstream auto-orientation logic.
 */

import type { UrdfVisualDesc } from './componentMeshes'

export type LinkPrimitive =
  | {
      shape: 'box'
      size_mm: [number, number, number]
      /** Optional author-assigned name — primitive-anchor placement
       * (`attach_primitive`/`attach_anchor`) references primitives by name. */
      name?: string
      xyz_mm?: [number, number, number]
      rpy?: [number, number, number]
      color?: [number, number, number]
    }
  | {
      shape: 'cylinder'
      radius_mm: number
      length_mm: number
      name?: string
      xyz_mm?: [number, number, number]
      rpy?: [number, number, number]
      color?: [number, number, number]
    }
  | {
      shape: 'sphere'
      radius_mm: number
      name?: string
      xyz_mm?: [number, number, number]
      color?: [number, number, number]
    }

const DEFAULT_COLOR: [number, number, number, number] = [0.66, 0.70, 0.72, 1.0]

function rgbOrDefault(c?: [number, number, number]): [number, number, number, number] {
  if (!c || c.length !== 3) return DEFAULT_COLOR
  return [c[0] ?? 0.66, c[1] ?? 0.70, c[2] ?? 0.72, 1.0]
}

function mmTriplet(v: unknown): [number, number, number] {
  if (Array.isArray(v) && v.length === 3) {
    return [Number(v[0]) || 0, Number(v[1]) || 0, Number(v[2]) || 0]
  }
  return [0, 0, 0]
}

/** Convert a list of LinkPrimitive specs (mm) into UrdfVisualDesc (metres). */
export function buildVisualsFromLinkGeometry(primitives: LinkPrimitive[]): UrdfVisualDesc[] {
  const out: UrdfVisualDesc[] = []
  for (const p of primitives) {
    const xyzMm = mmTriplet(p.xyz_mm)
    const xyz: [number, number, number] = [xyzMm[0] / 1000, xyzMm[1] / 1000, xyzMm[2] / 1000]
    const rpy = ('rpy' in p && p.rpy) ? mmTriplet(p.rpy) : ([0, 0, 0] as [number, number, number])
    const color = rgbOrDefault(p.color)
    if (p.shape === 'box') {
      const s = p.size_mm
      if (!Array.isArray(s) || s.length !== 3) continue
      out.push({
        origin_xyz: xyz,
        origin_rpy: rpy,
        geometry: {
          type: 'box',
          size: [Math.max(0.001, s[0] / 1000), Math.max(0.001, s[1] / 1000), Math.max(0.001, s[2] / 1000)],
        },
        color_rgba: color,
      })
    } else if (p.shape === 'cylinder') {
      out.push({
        origin_xyz: xyz,
        origin_rpy: rpy,
        geometry: {
          type: 'cylinder',
          radius: Math.max(0.0005, (Number(p.radius_mm) || 0) / 1000),
          length: Math.max(0.001, (Number(p.length_mm) || 0) / 1000),
        },
        color_rgba: color,
      })
    } else if (p.shape === 'sphere') {
      out.push({
        origin_xyz: xyz,
        origin_rpy: [0, 0, 0],
        geometry: {
          type: 'sphere',
          radius: Math.max(0.0005, (Number(p.radius_mm) || 0) / 1000),
        },
        color_rgba: color,
      })
    }
  }
  return out
}

/** True when an AssemblyComponent-like object has at least one usable primitive. */
export function hasLinkGeometry(instance: { link_geometry?: unknown } | undefined): boolean {
  if (!instance) return false
  const g = instance.link_geometry
  return Array.isArray(g) && g.length > 0
}

// ── Geometry derivation (WS3: link_geometry is first-class in the resolver) ──
//
// Bounds, default face connectors, collision descriptors, and mass all derive
// from the primitive union so a custom body shell mounts children on its REAL
// surfaces instead of the donor preset's bbox. All "*Mm" helpers stay in mm
// (the componentResolver's unit); volume is m³ for mass derivation.

/** Default density for authored body shells with no explicit mass_kg. 600
 * kg/m³ approximates a printed/hollow shell — far lighter than solid ABS yet
 * heavy enough that a torso outweighs the servos bolted to it. Exported so
 * eval/mass warnings reference one constant. */
export const LINK_GEOMETRY_DEFAULT_DENSITY_KG_M3 = 600

/** URDF fixed-axis rotation: R = Rz(yaw)·Ry(pitch)·Rx(roll). Pure-number
 * implementation (no THREE) so the resolver path stays dependency-light. */
function rpyMatrix3(rpy: [number, number, number]): number[][] {
  const [r, p, y] = rpy
  const cr = Math.cos(r), sr = Math.sin(r)
  const cp = Math.cos(p), sp = Math.sin(p)
  const cy = Math.cos(y), sy = Math.sin(y)
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ]
}

function intrinsicHalfMm(p: LinkPrimitive): [number, number, number] | null {
  if (p.shape === 'box') {
    const s = p.size_mm
    if (!Array.isArray(s) || s.length !== 3) return null
    return [Math.abs(s[0]) / 2, Math.abs(s[1]) / 2, Math.abs(s[2]) / 2]
  }
  if (p.shape === 'cylinder') {
    const r = Math.abs(Number(p.radius_mm) || 0)
    const l = Math.abs(Number(p.length_mm) || 0)
    return [r, r, l / 2]
  }
  const r = Math.abs(Number(p.radius_mm) || 0)
  return [r, r, r]
}

/** Author-assigned primitive names, in order, skipping unnamed entries. */
export function primitiveNames(prims: LinkPrimitive[]): string[] {
  const out: string[] = []
  for (const p of prims) if (typeof p.name === 'string' && p.name.trim()) out.push(p.name)
  return out
}

/** Link-frame AABB of one primitive, in mm. Per-axis support of an oriented
 * box is exact: half'[i] = Σ_j |R[i][j]|·half[j]. */
export function primitiveLocalAabbMm(p: LinkPrimitive): { min: [number, number, number]; max: [number, number, number] } | null {
  const half = intrinsicHalfMm(p)
  if (!half) return null
  const c = mmTriplet(p.xyz_mm)
  // Spheres have no rpy field; `'rpy' in p` narrows to box|cylinder.
  const rpy = ('rpy' in p && p.rpy) ? mmTriplet(p.rpy) : ([0, 0, 0] as [number, number, number])
  const rot = rpyMatrix3(rpy)
  const worldHalf = [0, 1, 2].map(i =>
    Math.abs(rot[i][0]) * half[0] + Math.abs(rot[i][1]) * half[1] + Math.abs(rot[i][2]) * half[2],
  )
  return {
    min: [c[0] - worldHalf[0], c[1] - worldHalf[1], c[2] - worldHalf[2]],
    max: [c[0] + worldHalf[0], c[1] + worldHalf[1], c[2] + worldHalf[2]],
  }
}

/** Union AABB of the primitive set in the link frame, mm. NOTE: unlike preset
 * bboxes, the union center is generally NOT the link origin. */
export function linkGeometryUnionAabbMm(prims: LinkPrimitive[]): { center: [number, number, number]; half: [number, number, number] } | null {
  let min: [number, number, number] | null = null
  let max: [number, number, number] | null = null
  for (const p of prims) {
    const aabb = primitiveLocalAabbMm(p)
    if (!aabb) continue
    if (!min || !max) {
      min = [...aabb.min]
      max = [...aabb.max]
    } else {
      for (let k = 0; k < 3; k++) {
        min[k] = Math.min(min[k], aabb.min[k])
        max[k] = Math.max(max[k], aabb.max[k])
      }
    }
  }
  if (!min || !max) return null
  return {
    center: [(min[0] + max[0]) / 2, (min[1] + max[1]) / 2, (min[2] + max[2]) / 2],
    half: [(max[0] - min[0]) / 2, (max[1] - min[1]) / 2, (max[2] - min[2]) / 2],
  }
}

/** 6 face connectors on the union AABB. Same id vocabulary as the bbox
 * defaults (top/bottom/front/back/left/right, axes OUTWARD) but offset by the
 * union center — origins are center ± half, not ± half. */
export function linkGeometryConnectors(prims: LinkPrimitive[]): import('./mateConnectors').MateConnector[] {
  const aabb = linkGeometryUnionAabbMm(prims)
  if (!aabb) return []
  const [cx, cy, cz] = aabb.center
  const [hx, hy, hz] = aabb.half
  return [
    { id: 'top',    origin_xyz_mm: [cx, cy, cz + hz], axis_xyz: [0, 0, 1],  type: 'planar' },
    { id: 'bottom', origin_xyz_mm: [cx, cy, cz - hz], axis_xyz: [0, 0, -1], type: 'planar' },
    { id: 'front',  origin_xyz_mm: [cx + hx, cy, cz], axis_xyz: [1, 0, 0],  type: 'planar' },
    { id: 'back',   origin_xyz_mm: [cx - hx, cy, cz], axis_xyz: [-1, 0, 0], type: 'planar' },
    { id: 'right',  origin_xyz_mm: [cx, cy + hy, cz], axis_xyz: [0, 1, 0],  type: 'planar' },
    { id: 'left',   origin_xyz_mm: [cx, cy - hy, cz], axis_xyz: [0, -1, 0], type: 'planar' },
  ]
}

/** Total solid volume of the primitives in m³ (overlaps double-count — fine
 * for a mass heuristic). */
export function linkGeometryVolumeM3(prims: LinkPrimitive[]): number {
  let mm3 = 0
  for (const p of prims) {
    if (p.shape === 'box') {
      const s = p.size_mm
      if (Array.isArray(s) && s.length === 3) mm3 += Math.abs(s[0] * s[1] * s[2])
    } else if (p.shape === 'cylinder') {
      const r = Math.abs(Number(p.radius_mm) || 0)
      const l = Math.abs(Number(p.length_mm) || 0)
      mm3 += Math.PI * r * r * l
    } else {
      const r = Math.abs(Number(p.radius_mm) || 0)
      mm3 += (4 / 3) * Math.PI * r * r * r
    }
  }
  return mm3 * 1e-9
}

/** Per-primitive collision descriptors (meters) — identical geometry to the
 * visuals, color stripped. The URDF emitter writes one <collision> per entry
 * so MJCF gets the real silhouette instead of one giant AABB box. */
export function linkGeometryCollisionDescriptors(prims: LinkPrimitive[]): UrdfVisualDesc[] {
  return buildVisualsFromLinkGeometry(prims).map(v => ({
    ...v,
    color_rgba: [0, 0, 0, 0] as [number, number, number, number],
  }))
}

// ── Primitive anchors (WS5) ──────────────────────────────────────────────────
//
// `attach_primitive` names a primitive on the parent's link_geometry;
// `attach_anchor` names a semantic point ON that primitive's real surface.
// The resolver returns a MateConnector in the link frame (mm, axis outward)
// that the placement compiler injects into the parent's connector list and
// routes through the ordinary fastened-mate solver — so a shoulder servo
// mounts at the END of a shoulder cylinder, not at the body's AABB face.
//
// Anchor vocabulary by shape:
//   box:      +x_face -x_face +y_face -y_face +z_face -z_face
//   cylinder: +axis_end -axis_end tangent_+x tangent_-x tangent_+y
//             tangent_-y tangent_+z tangent_-z   (tangents in LINK frame;
//             a tangent parallel to the cylinder axis is undefined → null)
//   sphere:   +x_pole -x_pole +y_pole -y_pole +z_pole -z_pole
//
// Math matches core/ai/primitive_anchors.py — pinned by the shared
// primitive-anchor parity corpus (scripts/primitive-anchor-corpus.json).

const _BOX_ANCHORS = ['+x_face', '-x_face', '+y_face', '-y_face', '+z_face', '-z_face'] as const
const _CYL_ANCHORS = ['+axis_end', '-axis_end', 'tangent_+x', 'tangent_-x', 'tangent_+y', 'tangent_-y', 'tangent_+z', 'tangent_-z'] as const
const _SPHERE_ANCHORS = ['+x_pole', '-x_pole', '+y_pole', '-y_pole', '+z_pole', '-z_pole'] as const

const _UNIT: Record<'x' | 'y' | 'z', [number, number, number]> = {
  x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1],
}

function _signedUnit(sign: string, axis: string): [number, number, number] | null {
  const u = _UNIT[axis as 'x' | 'y' | 'z']
  if (!u) return null
  const s = sign === '+' ? 1 : sign === '-' ? -1 : 0
  if (s === 0) return null
  return [u[0] * s, u[1] * s, u[2] * s]
}

/** Valid (well-defined) anchor names for a primitive, accounting for its
 * rotation — cylinder tangents parallel to the rotated axis are excluded. */
export function anchorNamesForPrimitive(p: LinkPrimitive): string[] {
  if (p.shape === 'box') return [..._BOX_ANCHORS]
  if (p.shape === 'sphere') return [..._SPHERE_ANCHORS]
  return _CYL_ANCHORS.filter(a => resolveAnchorOnPrimitive(p, a) !== null)
}

function resolveAnchorOnPrimitive(
  p: LinkPrimitive,
  anchorName: string,
): { originMm: [number, number, number]; axis: [number, number, number] } | null {
  const c = mmTriplet(p.xyz_mm)
  const rpy = ('rpy' in p && p.rpy) ? mmTriplet(p.rpy) : ([0, 0, 0] as [number, number, number])
  const rot = rpyMatrix3(rpy)
  const rotate = (v: [number, number, number]): [number, number, number] => ([
    rot[0][0] * v[0] + rot[0][1] * v[1] + rot[0][2] * v[2],
    rot[1][0] * v[0] + rot[1][1] * v[1] + rot[1][2] * v[2],
    rot[2][0] * v[0] + rot[2][1] * v[1] + rot[2][2] * v[2],
  ])

  if (p.shape === 'box') {
    const m = anchorName.match(/^([+-])([xyz])_face$/)
    if (!m) return null
    const dir = _signedUnit(m[1], m[2])
    if (!dir) return null
    const s = p.size_mm
    if (!Array.isArray(s) || s.length !== 3) return null
    const axisIdx = m[2] === 'x' ? 0 : m[2] === 'y' ? 1 : 2
    const local: [number, number, number] = [0, 0, 0]
    local[axisIdx] = (Math.abs(s[axisIdx]) / 2) * (m[1] === '+' ? 1 : -1)
    const off = rotate(local)
    const axis = rotate(dir)
    return { originMm: [c[0] + off[0], c[1] + off[1], c[2] + off[2]], axis }
  }

  if (p.shape === 'sphere') {
    const m = anchorName.match(/^([+-])([xyz])_pole$/)
    if (!m) return null
    const dir = _signedUnit(m[1], m[2])
    if (!dir) return null
    const r = Math.abs(Number(p.radius_mm) || 0)
    // Sphere rpy is irrelevant (and zeroed by the visual builder) — poles are
    // link-frame directions.
    return {
      originMm: [c[0] + r * dir[0], c[1] + r * dir[1], c[2] + r * dir[2]],
      axis: dir,
    }
  }

  // cylinder
  const r = Math.abs(Number(p.radius_mm) || 0)
  const L = Math.abs(Number(p.length_mm) || 0)
  const a = rotate([0, 0, 1])   // rotated cylinder axis, link frame

  const end = anchorName.match(/^([+-])axis_end$/)
  if (end) {
    const s = end[1] === '+' ? 1 : -1
    return {
      originMm: [c[0] + (L / 2) * a[0] * s, c[1] + (L / 2) * a[1] * s, c[2] + (L / 2) * a[2] * s],
      axis: [a[0] * s, a[1] * s, a[2] * s],
    }
  }
  const tan = anchorName.match(/^tangent_([+-])([xyz])$/)
  if (tan) {
    const d = _signedUnit(tan[1], tan[2])
    if (!d) return null
    const dDotA = d[0] * a[0] + d[1] * a[1] + d[2] * a[2]
    const radial: [number, number, number] = [
      d[0] - dDotA * a[0], d[1] - dDotA * a[1], d[2] - dDotA * a[2],
    ]
    const len = Math.hypot(radial[0], radial[1], radial[2])
    if (len < 1e-6) return null   // tangent parallel to the cylinder axis — undefined
    const u: [number, number, number] = [radial[0] / len, radial[1] / len, radial[2] / len]
    return {
      originMm: [c[0] + r * u[0], c[1] + r * u[1], c[2] + r * u[2]],
      axis: u,
    }
  }
  return null
}

/** Resolve (primitiveName, anchorName) on a link_geometry set to a synthetic
 * MateConnector in the link frame. Null when the primitive name doesn't exist
 * or the anchor is invalid/undefined for that primitive — callers must have
 * validated first (BAD_PRIMITIVE_REF / BAD_ANCHOR), so a null here is a
 * defensive signal, never a silent fallthrough. */
export function resolvePrimitiveAnchorPose(
  prims: LinkPrimitive[] | undefined,
  primitiveName: string,
  anchorName: string,
): import('./mateConnectors').MateConnector | null {
  if (!prims) return null
  const prim = prims.find(p => p.name === primitiveName)
  if (!prim) return null
  const resolved = resolveAnchorOnPrimitive(prim, anchorName)
  if (!resolved) return null
  return {
    id: `prim:${primitiveName}:${anchorName}`,
    origin_xyz_mm: resolved.originMm,
    axis_xyz: resolved.axis,
    type: 'planar',
    cls: 'mount_face',
    single: false,
  }
}

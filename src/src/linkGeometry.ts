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
      xyz_mm?: [number, number, number]
      rpy?: [number, number, number]
      color?: [number, number, number]
    }
  | {
      shape: 'cylinder'
      radius_mm: number
      length_mm: number
      xyz_mm?: [number, number, number]
      rpy?: [number, number, number]
      color?: [number, number, number]
    }
  | {
      shape: 'sphere'
      radius_mm: number
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

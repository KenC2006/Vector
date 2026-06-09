// Phase 3b — multi-child face distribution helpers extracted from
// urdfAssembly.ts. Pure functions of (total, parent half-extents, face,
// inset, optional child sizes, optional connector). The position-builder
// keeps a module-local cache keyed deterministically on the inputs.

import * as THREE from 'three'
import { tangentBasisFromAxis, type MateConnector } from '../mateConnectors.ts'

export interface ParentHalfExtentsM {
  hx: number
  hy: number
  hz: number
}

/** Map a face name to its two tangent half-extents (U and V axes on that face). */
export function faceUVHalfExtents(b: ParentHalfExtentsM, face: string): { hu: number; hv: number } {
  switch (face) {
    case 'top': case 'bottom': return { hu: b.hx, hv: b.hy }
    case 'front': case 'back': return { hu: b.hy, hv: b.hz }
    case 'left': case 'right': return { hu: b.hx, hv: b.hz }
    default: return { hu: b.hx, hv: b.hy }
  }
}

/** Project parent AABB half-extents onto the tangent plane of a connector's
 *  axis. For axis-aligned connectors the result equals `faceUVHalfExtents`;
 *  tilted-axis connectors get the AABB extent along their actual (u,v)
 *  tangent basis. Support along unit w: hx*|wx|+hy*|wy|+hz*|wz|. */
export function faceUVHalfExtentsFromConnector(
  b: ParentHalfExtentsM,
  connector: MateConnector,
): { hu: number; hv: number } {
  const axis = new THREE.Vector3(
    connector.axis_xyz[0],
    connector.axis_xyz[1],
    connector.axis_xyz[2],
  )
  const { u, v } = tangentBasisFromAxis(axis)
  const hu = b.hx * Math.abs(u.x) + b.hy * Math.abs(u.y) + b.hz * Math.abs(u.z)
  const hv = b.hx * Math.abs(v.x) + b.hy * Math.abs(v.y) + b.hz * Math.abs(v.z)
  return { hu, hv }
}

/** Splay angle (radians) for N limbs sharing a face. */
export function splayAngleForLegCount(n: number): number {
  if (n <= 2)  return 0.262  // ~15°
  if (n === 3) return 0.436  // ~25°
  if (n === 4) return 0.524  // ~30°
  if (n <= 6)  return 0.611  // ~35°
  return 0.698               // ~40° for 7+
}

/** Which pre-rotation half-extent ends up vertical after an axis-aligned RPY
 *  rotation. Only ±90° roll or pitch swap an axis onto Z; smaller angles leave
 *  Z dominant, so they keep childZ. Without this, sideways cylinders (wheels,
 *  rollers, casters, horizontal bearings) get placed using their pre-rotation
 *  thickness rather than their post-rotation radius and clip into the parent. */
export function verticalExtentForRotation(
  childX: number, childY: number, childZ: number,
  rollRad: number, pitchRad: number,
): number {
  const RIGHT_ANGLE = Math.PI / 2
  const nearRight = (v: number) => Math.abs(Math.abs(v) - RIGHT_ANGLE) < 0.1
  if (nearRight(rollRad)) return childY
  if (nearRight(pitchRad)) return childX
  return childZ
}

const _multiChildPositionsCache = new Map<string, Array<{ u: number; v: number }>>()

/** Test/diagnostic only — clears the position cache. Production callers do not
 *  need this; the cache is correctness-neutral. */
export function _resetMultiChildPositionsCache(): void {
  _multiChildPositionsCache.clear()
}

export function _buildMultiChildPositions(
  total: number,
  parent: ParentHalfExtentsM,
  face: string,
  inset: number,
  childSizes?: Array<{ hu: number; hv: number }>,
  parentConnector?: MateConnector | null,
): Array<{ u: number; v: number }> {
  const { hu: extU, hv: extV } = parentConnector
    ? faceUVHalfExtentsFromConnector(parent, parentConnector)
    : faceUVHalfExtents(parent, face)
  const cacheKey = `${total}:${face}:${extU.toFixed(6)},${extV.toFixed(6)}:${inset}:${childSizes ? childSizes.map(s => `${s.hu},${s.hv}`).join(';') : ''}`
  const cached = _multiChildPositionsCache.get(cacheKey)
  if (cached) return cached
  let positions: Array<{ u: number; v: number }>

  if (total >= 3) {
    // Radial distribution: children spread evenly around the face center on
    // an ellipse inscribed in the face's UV extents. Replaces the legacy
    // 4-corner-plus-edge-cycle pattern that piled extra children onto the
    // face CENTER for N≥5 (hexapods got two legs inside the body) and forced
    // "dog with extra legs" stances on radial creatures. The ellipse adapts
    // to non-square faces.
    //
    // Corner parity for total===4: phase by 45° and stretch by √2 (clamped to
    // the face extent) so quadrupeds keep their corner stance — at the
    // cardinal-diagonal angles cos/sin are ±1/√2, and the √2 stretch lands
    // children exactly on the legacy (±inset·extU, ±inset·extV) corners.
    positions = []
    const phase = total === 4 ? Math.PI / 4 : 0
    const stretch = total === 4 ? Math.SQRT2 : 1
    for (let i = 0; i < total; i++) {
      const theta = (2 * Math.PI * i) / total + phase
      const u = inset * extU * stretch * Math.cos(theta)
      const v = inset * extV * stretch * Math.sin(theta)
      positions.push({
        u: Math.max(-extU, Math.min(extU, u)),
        v: Math.max(-extV, Math.min(extV, v)),
      })
    }
  } else if (total === 2) {
    positions = [
      { u: -inset * extU, v: 0 },
      { u: inset * extU, v: 0 },
    ]
  } else {
    positions = []
    const step = (2 * inset * extU) / Math.max(total - 1, 1)
    for (let i = 0; i < total; i++) {
      positions.push({ u: -inset * extU + i * step, v: 0 })
    }
  }

  // Overlap resolution: push apart positions that would cause child AABBs to
  // clip. Iterates a few passes until no more overlaps or pass limit reached.
  if (childSizes && childSizes.length === total) {
    const margin = 0.002
    for (let pass = 0; pass < 3; pass++) {
      let movedAny = false
      for (let i = 0; i < total; i++) {
        for (let j = i + 1; j < total; j++) {
          const du = positions[j].u - positions[i].u
          const dv = positions[j].v - positions[i].v
          const minSepU = childSizes[i].hu + childSizes[j].hu + margin
          const minSepV = childSizes[i].hv + childSizes[j].hv + margin
          const overlapU = minSepU - Math.abs(du)
          const overlapV = minSepV - Math.abs(dv)
          if (overlapU > 0 && overlapV > 0) {
            if (overlapU <= overlapV) {
              const push = overlapU / 2 + 0.001
              const signU = du >= 0 ? 1 : -1
              positions[i].u -= signU * push
              positions[j].u += signU * push
            } else {
              const push = overlapV / 2 + 0.001
              const signV = dv >= 0 ? 1 : -1
              positions[i].v -= signV * push
              positions[j].v += signV * push
            }
            movedAny = true
          }
        }
      }
      if (!movedAny) break
    }
  }

  _multiChildPositionsCache.set(cacheKey, positions)
  return positions
}

export function _computeMultiChildOffsets(
  total: number, index: number,
  parent: ParentHalfExtentsM,
  face: string,
  insetOverride?: number,
  childSizes?: Array<{ hu: number; hv: number }>,
  parentConnector?: MateConnector | null,
): { u: number; v: number } {
  const inset = insetOverride ?? 0.7
  const safeIndex = Math.min(index, Math.max(total - 1, 0))
  const positions = _buildMultiChildPositions(total, parent, face, inset, childSizes, parentConnector)
  return positions[safeIndex] || { u: 0, v: 0 }
}

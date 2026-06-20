// Phase 3b — pure servo-split placement helpers extracted from urdfAssembly.ts.
//
// Just the geometry today: limb placement on a servo horn, child placement on
// a servo body, and the compound-carrier visual descriptors. The downstream
// `_body` / `_horn` / `_compound_carrier` LINK-NAME routing currently lives
// inline at urdfAssembly.ts ~1311–1366 / ~5056–5211 and is the responsibility
// of sub-phase 3b.4 — at that point compileAssembly emits CompiledLink with
// `physicalLinks` + `childAttachTarget`, and the inline string-suffix
// manipulation in urdfAssembly is replaced with a single read from those
// fields.

import * as THREE from 'three'
import type { UrdfVisualDesc } from '../componentMeshes.ts'
import { worldOffsetFromParent } from './transforms.ts'

export function servoDrivenStructuralLimbPlacement(
  parentAxisName: 'x' | 'y' | 'z',
  attachFace: string | null | undefined,
  childBodyHY: number,
  childBodyHZ: number,
  parentWorld?: THREE.Matrix4,
  /** Axial seat height above the horn origin for the on-top (z-axis) case —
   * the parent servo's real horn-top surface. When omitted, falls back to the
   * legacy fixed `adapterGap`. Ignored by the radial (x/y) branches, whose
   * limbs sit within the horn disc radius. */
  standoffM?: number,
): { xyz: string; rpy: string } | null {
  const adapterGap = 0.008
  const offset = adapterGap + childBodyHZ
  const desiredWorldZ = attachFace === 'top' ? 1 : -1
  const signedRadial = (axis: [number, number, number]) => {
    const baseSign = attachFace === 'top' ? -1 : 1
    const radial = worldOffsetFromParent(parentWorld, [
      axis[0] * baseSign,
      axis[1] * baseSign,
      axis[2] * baseSign,
    ])
    return radial.z * desiredWorldZ < 0 ? -baseSign : baseSign
  }

  if (parentAxisName === 'x') {
    const sign = signedRadial([1, 0, 0])
    const x = sign * offset
    const pitch = sign >= 0 ? Math.PI / 2 : -Math.PI / 2
    return {
      xyz: `${x.toFixed(4)} 0.0000 ${childBodyHY.toFixed(4)}`,
      // Slim links are thin plates: length is local Z, broad face normal is
      // local Y. Map local Z to the radial direction and local Y onto the horn
      // shaft normal so the plate seats on its broad face instead of edge-on.
      rpy: `${(Math.PI / 2).toFixed(4)} ${pitch.toFixed(4)} 0`,
    }
  }

  if (parentAxisName === 'y') {
    const sign = signedRadial([0, 1, 0])
    const y = sign * offset
    const roll = sign >= 0 ? -Math.PI / 2 : Math.PI / 2
    return {
      xyz: `0.0000 ${y.toFixed(4)} ${childBodyHY.toFixed(4)}`,
      rpy: `${roll.toFixed(4)} 0 0`,
    }
  }

  const sign = signedRadial([0, 0, 1])
  // On-top (axial) seat: place the beam's near face on the servo's real horn
  // output surface (standoffM) rather than a fixed 8mm clearance that floated
  // the beam above the horn.
  const axialOffset = (standoffM ?? adapterGap) + childBodyHZ
  return {
    xyz: `0.0000 0.0000 ${(sign * axialOffset).toFixed(4)}`,
    rpy: '0 0 0',
  }
}

export function servoDrivenChildPlacement(
  parentAxisName: 'x' | 'y' | 'z',
  attachFace: string | null | undefined,
  _childBodyHX: number,
  _childBodyHY: number,
  childBodyHZ: number,
  invertRadialSide = false,
  childIsServo = false,
): { xyz: string; rpy: string } | null {
  const adapterGap = 0.008
  if (parentAxisName === 'z') {
    // Floor matches the x-axis branch below: chained servos need to clear the
    // parent body's far face, not just their own half-Z. Without this the
    // carrier offset is ~hz_child only, and the next servo body overlaps the
    // parent servo body in world space.
    const offset = childIsServo ? Math.max(adapterGap + childBodyHZ, 0.060) : adapterGap + childBodyHZ
    return { xyz: `0.0000 0.0000 ${offset.toFixed(4)}`, rpy: '0 0 0' }
  }
  const offset = childIsServo ? Math.max(adapterGap + childBodyHZ, 0.060) : adapterGap + childBodyHZ
  const sign = childIsServo && parentAxisName === 'x' ? 1 : (attachFace === 'top' ? -1 : 1)
  if (parentAxisName === 'x') {
    const x = sign * offset
    const rpy = sign > 0 ? '0 1.5708 0' : '0 -1.5708 0'
    return { xyz: `${x.toFixed(4)} 0.0000 0.0000`, rpy }
  }
  const y = sign * offset
  const roll = (invertRadialSide ? sign : -sign) * Math.PI / 2
  const rpy = `${roll.toFixed(4)} 0 0`
  return { xyz: `0.0000 ${y.toFixed(4)} 0.0000`, rpy }
}

export function servoCompoundCarrierVisuals(xm: number, ym: number, zm: number, reach = 0): UrdfVisualDesc[] {
  const plateT = Math.max(Math.min(xm, ym) * 0.075, 0.0025)
  const sideY = ym / 2 + plateT * 4.2
  const sideSize: [number, number, number] = [xm * 1.18, plateT, zm * 1.18]
  const tieSize: [number, number, number] = [plateT * 1.2, ym + plateT * 8.4, plateT * 1.2]
  const color: [number, number, number, number] = [0.42, 0.46, 0.50, 1]
  const dark: [number, number, number, number] = [0.31, 0.34, 0.38, 1]
  const visuals: UrdfVisualDesc[] = [
    { origin_xyz: [0, sideY, 0], origin_rpy: [0, 0, 0], geometry: { type: 'box', size: sideSize }, color_rgba: color },
    { origin_xyz: [0, -sideY, 0], origin_rpy: [0, 0, 0], geometry: { type: 'box', size: sideSize }, color_rgba: color },
    { origin_xyz: [-xm * 0.42, 0, -zm * 0.42], origin_rpy: [0, 0, 0], geometry: { type: 'box', size: tieSize }, color_rgba: dark },
  ]
  // When the carrier reaches across a non-trivial gap (parent servo's horn to
  // this servo's body), span it with a single central standoff column instead
  // of the previous pair of skinny offset rods. Topped with a horn-side anchor
  // disc so the bracket has a clear foot on the parent's output rather than
  // ending in mid-air.
  // The yoke's lower side plate already sits at carrier-Y = -sideY, so the
  // visible gap between the yoke and the parent horn is only (reach - sideY),
  // not the full reach. Filling the full reach made the bridge look twice as
  // long as needed (it ran straight through the yoke region).
  const yokeBottomY = -sideY
  const hornSurfaceY = -reach
  const visibleGap = Math.max((-hornSurfaceY) - (-yokeBottomY) - plateT, 0)
  if (visibleGap > plateT * 2) {
    const anchorR = Math.max(xm * 0.42, 0.013)
    const anchorH = visibleGap * 0.6
    const standoffLen = visibleGap - anchorH
    const standoffSide = Math.min(xm * 0.55, ym * 0.55)
    const standoffStartY = yokeBottomY  // top of standoff sits at the yoke's lower face
    if (standoffLen > plateT) {
      visuals.push({
        origin_xyz: [0, standoffStartY - standoffLen / 2, 0],
        origin_rpy: [0, 0, 0],
        geometry: { type: 'box', size: [standoffSide, standoffLen, standoffSide] },
        color_rgba: dark,
      })
    }
    visuals.push({
      origin_xyz: [0, standoffStartY - standoffLen - anchorH * 0.5, 0],
      origin_rpy: [Math.PI / 2, 0, 0],
      geometry: { type: 'cylinder', radius: anchorR, length: anchorH },
      color_rgba: dark,
    })
  }
  return visuals
}

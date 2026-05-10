// Phase 3b — face-mount placement extracted from urdfAssembly.ts:computeFacePlacement.
//
// Pure: takes resolved parent half-extents + center as data instead of
// looking them up from a URDF DOM. The urdfAssembly wrapper resolves the
// parent via parentBoundsFromLink() and forwards. parentLinkName is kept
// only for diagnostic logging and drivetrain-id inference.
//
// All math is otherwise verbatim from the original — bit-identical output is
// the parity gate for sub-phase 3b.4 (frontend cutover).

import { isDrivetrainComponentId, isTireComponentId } from '../componentResolver.ts'
import type { MateConnector } from '../mateConnectors.ts'
import { componentIdFromLinkName } from './componentNaming.ts'
import {
  _computeMultiChildOffsets,
  splayAngleForLegCount,
  verticalExtentForRotation,
} from './multiChild.ts'
import { isNovelMode } from './context.ts'

/** Parent bounds in METERS, after RPY-bake / parametric splice / split-servo
 *  body-vs-horn routing (resolved by the caller). */
export interface ParentBoundsM {
  hx: number
  hy: number
  hz: number
  cx: number
  cy: number
  cz: number
}

export interface FacePlacementHints {
  parentIsDrivetrain?: boolean
  childIsTire?: boolean
  childIsDrivetrain?: boolean
  childUsesRollingBottomPose?: boolean
  /** Tier-A novel-mode authoring override. When set, replaces the auto-computed
   * splay angle (from `splayAngleForLegCount`) for this child only. Allows
   * varied posture across limbs in the same design — e.g. front legs sprawled,
   * back legs upright. Clamped to ±75°. Set to undefined to use auto. */
  splayAngleDegOverride?: number
}

export interface FacePlacementResult {
  xyz: string
  rpy: string
}

/** Pure face-mount placement.
 *
 *  @param parent       Resolved parent half-extents (m) and center offsets (m).
 *  @param parentLinkName Used for diagnostic logging and drivetrain inference
 *                        when `placementHints.parentIsDrivetrain` is unset.
 *  @param childX/Y/Z   Child AABB extents (m), full not half.
 *  @param outFlags     Optional out-parameter — set viaConnector=true when an
 *                      authored parent or child connector influenced the result.
 */
export function computeFacePlacement(
  parent: ParentBoundsM,
  parentLinkName: string,
  childX: number, childY: number, childZ: number,
  attachFace: string | null,
  isChildElongated: boolean,
  childIndex: number,
  totalOnFace: number,
  orientation: string,
  noSplay: boolean,
  childComponentId: string,
  elevationAngleDeg: number,
  childSizes: Array<{ hu: number; hv: number }> | undefined,
  childCenterOffset: { cx: number; cy: number; cz: number },
  parentConnectors: MateConnector[] | undefined,
  outFlags: { viaConnector: boolean } | undefined,
  childConnectors: MateConnector[] | undefined,
  placementHints: FacePlacementHints,
  parentParametricLengthMm: number | undefined,
): FacePlacementResult {
  const gap = 0
  const parentBodyHX = parent.hx - Math.abs(parent.cx)
  const parentBodyHY = parent.hy - Math.abs(parent.cy)
  // Parametric extrusions (length_mm + cross_section_mm): the body's Z tip is
  // at length_mm/2 — the URDF visual AABB also includes pivot bosses/axle caps
  // perpendicular to the extrusion axis. Those bosses live ON the end face,
  // not past it, so the extruded body tip is the true end-mount surface.
  const parentBodyHZ = (parentParametricLengthMm && parentParametricLengthMm > 0)
    ? parentParametricLengthMm / 2000
    : parent.hz - Math.abs(parent.cz)
  const childBodyHX = childX / 2 - Math.abs(childCenterOffset.cx)
  const childBodyHY = childY / 2 - Math.abs(childCenterOffset.cy)
  const childBodyHZ = childZ / 2 - Math.abs(childCenterOffset.cz)

  const face = attachFace || 'top'

  const parentIsDrivetrain = placementHints.parentIsDrivetrain
    ?? isDrivetrainComponentId(componentIdFromLinkName(parentLinkName))
  const childIsTire = placementHints.childIsTire
    ?? isTireComponentId(childComponentId)
  if (parentIsDrivetrain && childIsTire) {
    const motorHalfZ = parent.hz
    const tireHalfAxle = childZ / 2
    const dz = motorHalfZ + tireHalfAxle
    return { xyz: `0.0000 0.0000 ${dz.toFixed(4)}`, rpy: '0 0 0' }
  }

  const authoredConn = parentConnectors?.find(c => c.id === face) ?? null
  const connOriginM = authoredConn
    ? [
        authoredConn.origin_xyz_mm[0] / 1000,
        authoredConn.origin_xyz_mm[1] / 1000,
        authoredConn.origin_xyz_mm[2] / 1000,
      ] as const
    : null
  const engagementM = (authoredConn?.engagement_depth_mm ?? 0) / 1000
  const oppositeFaceMap: Record<string, string> = {
    top: 'bottom', bottom: 'top',
    front: 'back', back: 'front',
    left: 'right', right: 'left',
  }
  const childFace = oppositeFaceMap[face]
  const childAuthoredConn = childFace
    ? (childConnectors?.find(c => c.id === childFace) ?? null)
    : null
  const childConnOriginM = childAuthoredConn
    ? [
        childAuthoredConn.origin_xyz_mm[0] / 1000,
        childAuthoredConn.origin_xyz_mm[1] / 1000,
        childAuthoredConn.origin_xyz_mm[2] / 1000,
      ] as const
    : null
  if (outFlags && (connOriginM || childConnOriginM)) outFlags.viaConnector = true

  // ── Pre-compute splay and splay-aware inset for bottom-face legs ──
  const childIsDrivetrain = placementHints.childIsDrivetrain
    ?? isDrivetrainComponentId(childComponentId)
  const usesRollingBottomPose = placementHints.childUsesRollingBottomPose
    ?? (childIsDrivetrain || childIsTire || childComponentId.startsWith('mobility_swerve_'))
  let splayAngle = 0
  let insetOverride: number | undefined
  const authoredSplayDeg = placementHints.splayAngleDegOverride
  const hasAuthoredSplay = typeof authoredSplayDeg === 'number' && Number.isFinite(authoredSplayDeg)
  if (face === 'bottom' && !usesRollingBottomPose && !noSplay && totalOnFace >= 2) {
    splayAngle = splayAngleForLegCount(totalOnFace)
    // Tier-A creative override: when Claude authored splay_angle_deg on this
    // child, replace the auto-derived value. Clamp to ±75° to keep stances
    // physically sensible (90°+ would lay legs flat on the ground).
    if (hasAuthoredSplay) {
      const clamped = Math.max(-75, Math.min(75, authoredSplayDeg!))
      splayAngle = clamped * Math.PI / 180
    }
    insetOverride = Math.max(0.4, 0.7 - (Math.abs(splayAngle) / (Math.PI / 2)) * 0.3)
  } else if (isNovelMode() && hasAuthoredSplay && !noSplay && totalOnFace >= 2) {
    // Novel-mode top/side face splay — opt-in via Claude's splay_angle_deg.
    // Lets sensors/antennae fan radially outward from a top face center, or
    // limbs fan from a side face. Default behavior (no override) preserves
    // batteries/compute mounted flat on top — splay only fires when Claude
    // explicitly authors it. Bottom face still gets auto-splay above.
    const clamped = Math.max(-75, Math.min(75, authoredSplayDeg!))
    splayAngle = clamped * Math.PI / 180
    insetOverride = Math.max(0.4, 0.7 - (Math.abs(splayAngle) / (Math.PI / 2)) * 0.3)
  }

  let tu = 0, tv = 0
  if (totalOnFace > 1) {
    const offsets = _computeMultiChildOffsets(totalOnFace, childIndex, parent, face, insetOverride, childSizes, authoredConn)
    tu = offsets.u
    tv = offsets.v
  }
  if (connOriginM) {
    switch (face) {
      case 'top': case 'bottom':  tu += connOriginM[0]; tv += connOriginM[1]; break
      case 'front': case 'back':  tu += connOriginM[1]; tv += connOriginM[2]; break
      case 'left': case 'right':  tu += connOriginM[0]; tv += connOriginM[2]; break
    }
  }

  const orientDeg = parseFloat(orientation)
  const hasNumericOrient = !isNaN(orientDeg) && orientDeg !== 0

  let shouldRotateHorizontal = false
  if (isChildElongated) {
    if (orientation === 'horizontal') {
      shouldRotateHorizontal = true
    }
  }

  if (shouldRotateHorizontal && face === 'top') {
    const dims = [childX, childY, childZ]
    const sortedDims = [...dims].sort((a, b) => a - b)
    const longest = sortedDims[2]
    const longestAxisIdx = dims.indexOf(longest)
    if (longestAxisIdx === 2) {
      const vExtent = verticalExtentForRotation(childX, childY, childZ, 0, Math.PI / 2)
      const oz = (connOriginM ? connOriginM[2] : parent.hz) + vExtent / 2 + gap - engagementM
      const yaw = hasNumericOrient ? ` ${(orientDeg * Math.PI / 180).toFixed(4)}` : ' 0'
      return { xyz: `${tu.toFixed(4)} ${tv.toFixed(4)} ${oz.toFixed(4)}`, rpy: `0 1.5708${yaw}` }
    }
  }

  const elevRad = elevationAngleDeg * (Math.PI / 180)

  switch (face) {
    case 'top': {
      const childContact = childConnOriginM ? -childConnOriginM[2] : childBodyHZ
      const oz = (connOriginM ? connOriginM[2] : parentBodyHZ) + childContact + gap - engagementM
      const yawRad = hasNumericOrient ? orientDeg * Math.PI / 180 : 0
      // Tier-A novel-mode opt-in radial splay. When Claude authored
      // splay_angle_deg on a top-face child, fan it outward from the face
      // center along its radial position. Mirror of the bottom-face trig
      // (same magnitudes, opposite signs because face normal is +Z, not -Z).
      // Children without authored splay sit flat as before — batteries,
      // compute boards, and most top-mounts keep their existing behavior.
      if (isNovelMode() && hasAuthoredSplay && splayAngle !== 0 && (tu !== 0 || tv !== 0)) {
        const theta = Math.atan2(tv, tu)
        const rollRad  = -splayAngle * Math.sin(theta)
        const pitchRad =  splayAngle * Math.cos(theta) - elevRad
        return { xyz: `${tu.toFixed(4)} ${tv.toFixed(4)} ${oz.toFixed(4)}`, rpy: `${rollRad.toFixed(4)} ${pitchRad.toFixed(4)} ${yawRad.toFixed(4)}` }
      }
      const rpy = elevRad !== 0 || hasNumericOrient
        ? `0 ${(-elevRad).toFixed(4)} ${yawRad.toFixed(4)}`
        : '0 0 0'
      return { xyz: `${tu.toFixed(4)} ${tv.toFixed(4)} ${oz.toFixed(4)}`, rpy }
    }
    case 'bottom': {
      let rollRad = 0
      let pitchRad = 0
      if (usesRollingBottomPose) {
        // Flip roll sign on the -Y side of the parent so the body's +Z (the
        // hub-output / wheel-mount axis) ends up pointing world -Y instead of
        // +Y. Adding π to yaw alone doesn't mirror under three.js intrinsic
        // XYZ — yaw applied after a non-zero roll leaves body +Z unchanged,
        // which dropped back-side wheels INBOARD instead of outboard.
        rollRad = (childIsDrivetrain && tv < 0) ? Math.PI / 2 : -Math.PI / 2
      } else if (splayAngle > 0 && (tu !== 0 || tv !== 0)) {
        if (isNovelMode()) {
          // Radial splay: tilt each leg along its actual angular position
          // around the face center, not along cardinal axes. Without this,
          // a hexapod at radial positions (60° spacing) gets cardinal
          // roll/pitch (±splay if tu/tv != 0) which ignores the angle and
          // bends every leg into NE/NW/SE/SW dog-stance directions. The trig
          // form below smoothly aligns each leg's tilt with its radial
          // outward direction; at cardinal angles (0°, 90°, 180°, 270°) it
          // matches the standard-mode rule, so dog-style 4-leg layouts
          // would be a continuous extension if novel mode were enabled.
          const theta = Math.atan2(tv, tu)
          rollRad  =  splayAngle * Math.sin(theta)
          pitchRad = -splayAngle * Math.cos(theta)
        } else {
          rollRad  = tv > 0 ?  splayAngle : tv < 0 ? -splayAngle : 0
          pitchRad = tu > 0 ? -splayAngle : tu < 0 ?  splayAngle : 0
        }
      }
      let yawRad = hasNumericOrient ? orientDeg * Math.PI / 180 : 0
      if (elevRad !== 0) pitchRad += elevRad
      const rpyStr = `${rollRad.toFixed(4)} ${pitchRad.toFixed(4)} ${yawRad.toFixed(4)}`
      const vExtent = verticalExtentForRotation(childBodyHX * 2, childBodyHY * 2, childBodyHZ * 2, rollRad, pitchRad)
      const isRotated = usesRollingBottomPose || splayAngle > 0
      const childContact = (childConnOriginM && !isRotated) ? childConnOriginM[2] : vExtent / 2
      const oz = (connOriginM ? connOriginM[2] : -parentBodyHZ) - childContact - gap + engagementM
      return { xyz: `${tu.toFixed(4)} ${tv.toFixed(4)} ${oz.toFixed(4)}`, rpy: rpyStr }
    }
    case 'front': {
      const zOffset = tv + (elevRad !== 0 ? parentBodyHX * Math.sin(elevRad) : 0)
      const rpy = elevRad !== 0 ? `0 ${(-elevRad).toFixed(4)} 0` : '0 0 0'
      const childContact = childConnOriginM ? -childConnOriginM[0] : childBodyHX
      const ox = (connOriginM ? connOriginM[0] : parentBodyHX) + childContact + gap - engagementM
      return { xyz: `${ox.toFixed(4)} ${tu.toFixed(4)} ${zOffset.toFixed(4)}`, rpy }
    }
    case 'back': {
      const zOffset = tv + (elevRad !== 0 ? parentBodyHX * Math.sin(elevRad) : 0)
      const rpy = elevRad !== 0 ? `0 ${elevRad.toFixed(4)} 0` : '0 0 0'
      const childContact = childConnOriginM ? childConnOriginM[0] : childBodyHX
      const ox = (connOriginM ? connOriginM[0] : -parentBodyHX) - childContact - gap + engagementM
      return { xyz: `${ox.toFixed(4)} ${tu.toFixed(4)} ${zOffset.toFixed(4)}`, rpy }
    }
    case 'right': {
      const zOffset = tv + (elevRad !== 0 ? parentBodyHY * Math.sin(elevRad) : 0)
      const rpy = elevRad !== 0 ? `${elevRad.toFixed(4)} 0 0` : '0 0 0'
      const childContact = childConnOriginM ? -childConnOriginM[1] : childBodyHY
      const oy = (connOriginM ? connOriginM[1] : parentBodyHY) + childContact + gap - engagementM
      return { xyz: `${tu.toFixed(4)} ${oy.toFixed(4)} ${zOffset.toFixed(4)}`, rpy }
    }
    case 'left': {
      const zOffset = tv + (elevRad !== 0 ? parentBodyHY * Math.sin(elevRad) : 0)
      const rpy = elevRad !== 0 ? `${(-elevRad).toFixed(4)} 0 0` : '0 0 0'
      const childContact = childConnOriginM ? childConnOriginM[1] : childBodyHY
      const oy = (connOriginM ? connOriginM[1] : -parentBodyHY) - childContact - gap + engagementM
      return { xyz: `${tu.toFixed(4)} ${oy.toFixed(4)} ${zOffset.toFixed(4)}`, rpy }
    }
    case 'coaxial':
      return { xyz: `${tu.toFixed(4)} ${tv.toFixed(4)} 0.0000`, rpy: '-1.5708 0.0000 0.0000' }
    default:
      return { xyz: `0 0 ${(parentBodyHZ + childBodyHZ + gap).toFixed(4)}`, rpy: '0 0 0' }
  }
}

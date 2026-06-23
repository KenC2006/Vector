// Phase 3b — face-mount placement extracted from urdfAssembly.ts:computeFacePlacement.
//
// Pure: takes resolved parent half-extents + center as data instead of
// looking them up from a URDF DOM. The urdfAssembly wrapper resolves the
// parent via parentBoundsFromLink() and forwards. parentLinkName is kept
// only for diagnostic logging and drivetrain-id inference.
//
// All math is otherwise verbatim from the original — bit-identical output is
// the parity gate for sub-phase 3b.4 (frontend cutover).

import * as THREE from 'three'
import { isDrivetrainComponentId, isTireComponentId } from '../componentResolver.ts'
import { quatToRpy, rpyToQuat } from '../rotationIO.ts'
import type { MateConnector } from '../mateConnectors.ts'
import { componentIdFromLinkName } from './componentNaming.ts'
import {
  _computeMultiChildOffsets,
  splayAngleForLegCount,
  verticalExtentForRotation,
} from './multiChild.ts'

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
}

export interface FacePlacementResult {
  xyz: string
  rpy: string
}

/** Parsed `orientation` grammar (WS6). Accepted forms:
 *  - 'auto' / '' / undefined → engine default (vertical)
 *  - 'vertical'              → explicit default
 *  - 'horizontal'            → long axis rotated into/along the face
 *  - 'horizontal+30'         → horizontal, then 30° yaw about the face normal
 *  - '45' / '-30'            → yaw about the face normal only
 *  Previously 'horizontal+N' parsed as NaN and side-face yaw was ignored —
 *  documented controls that silently did nothing. */
export interface ParsedOrientation {
  horizontal: boolean
  yawDeg: number
  auto: boolean
}

const FACE_NORMAL: Record<string, [number, number, number]> = {
  top: [0, 0, 1], bottom: [0, 0, -1],
  front: [1, 0, 0], back: [-1, 0, 0],
  right: [0, 1, 0], left: [0, -1, 0],
}

/** Minimum axis alignment (cosine) for a descriptive connector to stand in for
 *  a cardinal face. 0.7 ≈ within 45° of the face normal — close enough to be
 *  the same surface, strict enough that a sideways connector (an L-bracket's
 *  `wall_inner`) can never hijack a "top" mount. */
const FACE_CONNECTOR_AXIS_MIN = 0.7

/** Pick the authored connector that represents a cardinal face.
 *
 *  Exact id match wins (top/bottom/… — bit-identical to the legacy bbox path,
 *  so box parts and parts that author cardinal connectors are unaffected).
 *  Otherwise fall back to the authored connector whose OUTWARD axis is closest
 *  to the face normal. This lets non-box parts whose real mounting surface is
 *  named descriptively — an L-bracket's `plate_top`, a caster's `mount_top` —
 *  snap a face mount onto that surface instead of floating on the empty-air
 *  bbox face. Returns null when nothing is authored or no axis is close enough,
 *  preserving the legacy behavior (the caller then uses the bbox half-extent). */
function findFaceConnector(
  connectors: MateConnector[] | undefined,
  face: string,
): MateConnector | null {
  if (!connectors || connectors.length === 0) return null
  const exact = connectors.find(c => c.id === face)
  if (exact) return exact
  const normal = FACE_NORMAL[face]
  if (!normal) return null
  let best: MateConnector | null = null
  let bestDot = FACE_CONNECTOR_AXIS_MIN
  for (const c of connectors) {
    const [ax, ay, az] = c.axis_xyz
    const len = Math.hypot(ax, ay, az) || 1
    const dot = (ax * normal[0] + ay * normal[1] + az * normal[2]) / len
    if (dot > bestDot) { bestDot = dot; best = c }
  }
  return best
}

/** Compose "spin about the face's outward normal" with a base rotation,
 * returning a URDF rpy string. q = Q(normal, spin) · Q(baseRpy). */
function composeNormalSpin(
  normal: [number, number, number],
  spinRad: number,
  baseRpy: [number, number, number],
): string {
  const qBase = rpyToQuat(baseRpy)
  if (spinRad === 0) {
    const [r, p, y] = quatToRpy(qBase)
    return `${r.toFixed(4)} ${p.toFixed(4)} ${y.toFixed(4)}`
  }
  const qSpin = new THREE.Quaternion().setFromAxisAngle(
    new THREE.Vector3(normal[0], normal[1], normal[2]), spinRad,
  )
  const [r, p, y] = quatToRpy(qSpin.multiply(qBase))
  return `${r.toFixed(4)} ${p.toFixed(4)} ${y.toFixed(4)}`
}

export function parseOrientation(raw: string | null | undefined): ParsedOrientation {
  const v = (raw ?? '').trim().toLowerCase()
  if (!v || v === 'auto') return { horizontal: false, yawDeg: 0, auto: true }
  if (v === 'vertical') return { horizontal: false, yawDeg: 0, auto: false }
  if (v === 'horizontal') return { horizontal: true, yawDeg: 0, auto: false }
  const combo = v.match(/^horizontal\+(-?\d+(?:\.\d+)?)$/)
  if (combo) return { horizontal: true, yawDeg: Number(combo[1]), auto: false }
  const n = Number(v)
  if (Number.isFinite(n)) return { horizontal: false, yawDeg: n, auto: false }
  return { horizontal: false, yawDeg: 0, auto: true }
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

  const authoredConn = findFaceConnector(parentConnectors, face)
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
    ? findFaceConnector(childConnectors, childFace)
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
  if (face === 'bottom' && !usesRollingBottomPose && !noSplay && totalOnFace >= 2) {
    splayAngle = splayAngleForLegCount(totalOnFace)
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

  const parsedOrient = parseOrientation(orientation)
  const yawRad0 = parsedOrient.yawDeg * Math.PI / 180
  const hasNumericOrient = yawRad0 !== 0
  const orientDeg = parsedOrient.yawDeg
  // 'horizontal' rotates the child's LONG axis out of the default vertical:
  //  - top/bottom faces: long axis lies flat in the face plane (along +X,
  //    then yawed about the face normal) — booms, rails, flat-mounted tubes.
  //  - side faces: long axis extends OUTWARD along the face normal — tails,
  //    horizontal booms off a body side.
  const wantsHorizontal = parsedOrient.horizontal && isChildElongated
    && [childX, childY, childZ].indexOf([childX, childY, childZ].slice().sort((a, b) => a - b)[2]) === 2

  if (wantsHorizontal && (face === 'top' || face === 'bottom')) {
    const vExtent = verticalExtentForRotation(childX, childY, childZ, 0, Math.PI / 2)
    const sign = face === 'top' ? 1 : -1
    const baseZ = connOriginM ? connOriginM[2] : sign * parent.hz
    const oz = baseZ + sign * (vExtent / 2 + gap) - sign * engagementM
    const yaw = ` ${yawRad0.toFixed(4)}`
    return { xyz: `${tu.toFixed(4)} ${tv.toFixed(4)} ${oz.toFixed(4)}`, rpy: `0 1.5708${yaw}` }
  }

  // Side-face horizontal: child +Z (long axis) maps onto the face's outward
  // normal; contact extent along the normal is the child's half-LENGTH. Yaw
  // spins about the same normal (consistent with the documented "yaw rotation
  // around the face normal" rule). elevation_angle is a vertical-mount
  // concept and is not composed here.
  if (wantsHorizontal && (face === 'front' || face === 'back' || face === 'left' || face === 'right')) {
    const halfLen = childBodyHZ
    const normal = FACE_NORMAL[face]
    const baseRot: Record<string, [number, number, number]> = {
      front: [0, Math.PI / 2, 0],    // +Z → +X
      back:  [0, -Math.PI / 2, 0],   // +Z → -X
      right: [-Math.PI / 2, 0, 0],   // +Z → +Y
      left:  [Math.PI / 2, 0, 0],    // +Z → -Y
    }
    const rpyStr = composeNormalSpin(normal, yawRad0, baseRot[face])
    if (face === 'front' || face === 'back') {
      const sign = face === 'front' ? 1 : -1
      const baseX = connOriginM ? connOriginM[0] : sign * parentBodyHX
      const ox = baseX + sign * (halfLen + gap) - sign * engagementM
      return { xyz: `${ox.toFixed(4)} ${tu.toFixed(4)} ${tv.toFixed(4)}`, rpy: rpyStr }
    }
    const sign = face === 'right' ? 1 : -1
    const baseY = connOriginM ? connOriginM[1] : sign * parentBodyHY
    const oy = baseY + sign * (halfLen + gap) - sign * engagementM
    return { xyz: `${tu.toFixed(4)} ${oy.toFixed(4)} ${tv.toFixed(4)}`, rpy: rpyStr }
  }

  const elevRad = elevationAngleDeg * (Math.PI / 180)

  switch (face) {
    case 'top': {
      const childContact = childConnOriginM ? -childConnOriginM[2] : childBodyHZ
      const oz = (connOriginM ? connOriginM[2] : parentBodyHZ) + childContact + gap - engagementM
      const yawRad = hasNumericOrient ? orientDeg * Math.PI / 180 : 0
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
        // Radial splay: tilt each leg along its actual angular position
        // around the face center, not along cardinal axes. A hexapod at
        // radial positions (60° spacing) gets a tilt aligned with its
        // outward direction; at the cardinal/diagonal angles of a 4-corner
        // quadruped layout the trig reduces to the legacy ±splay rule's
        // diagonal blend, so dog stances stay outward-splayed.
        const theta = Math.atan2(tv, tu)
        rollRad  =  splayAngle * Math.sin(theta)
        pitchRad = -splayAngle * Math.cos(theta)
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
      const rpy = (elevRad !== 0 || hasNumericOrient)
        ? composeNormalSpin(FACE_NORMAL.front, yawRad0, [0, -elevRad, 0])
        : '0 0 0'
      const childContact = childConnOriginM ? -childConnOriginM[0] : childBodyHX
      const ox = (connOriginM ? connOriginM[0] : parentBodyHX) + childContact + gap - engagementM
      return { xyz: `${ox.toFixed(4)} ${tu.toFixed(4)} ${zOffset.toFixed(4)}`, rpy }
    }
    case 'back': {
      const zOffset = tv + (elevRad !== 0 ? parentBodyHX * Math.sin(elevRad) : 0)
      const rpy = (elevRad !== 0 || hasNumericOrient)
        ? composeNormalSpin(FACE_NORMAL.back, yawRad0, [0, elevRad, 0])
        : '0 0 0'
      const childContact = childConnOriginM ? childConnOriginM[0] : childBodyHX
      const ox = (connOriginM ? connOriginM[0] : -parentBodyHX) - childContact - gap + engagementM
      return { xyz: `${ox.toFixed(4)} ${tu.toFixed(4)} ${zOffset.toFixed(4)}`, rpy }
    }
    case 'right': {
      const zOffset = tv + (elevRad !== 0 ? parentBodyHY * Math.sin(elevRad) : 0)
      const rpy = (elevRad !== 0 || hasNumericOrient)
        ? composeNormalSpin(FACE_NORMAL.right, yawRad0, [elevRad, 0, 0])
        : '0 0 0'
      const childContact = childConnOriginM ? -childConnOriginM[1] : childBodyHY
      const oy = (connOriginM ? connOriginM[1] : parentBodyHY) + childContact + gap - engagementM
      return { xyz: `${tu.toFixed(4)} ${oy.toFixed(4)} ${zOffset.toFixed(4)}`, rpy }
    }
    case 'left': {
      const zOffset = tv + (elevRad !== 0 ? parentBodyHY * Math.sin(elevRad) : 0)
      const rpy = (elevRad !== 0 || hasNumericOrient)
        ? composeNormalSpin(FACE_NORMAL.left, yawRad0, [-elevRad, 0, 0])
        : '0 0 0'
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

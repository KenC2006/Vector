import * as THREE from 'three'
import { quatToRpy } from './rotationIO.ts'

// docs/SERVO_SPLIT_PLAN.md — axis-to-body-pose solver.
//
// Under the split-link servo model, the internal revolute axis is baked as
// body-local +Z (horn direction). The AI still writes `joint_axis: x|y|z` in
// the world frame, so the emitter must rotate the servo body so that its local
// +Z lines up with the AI's desired world axis. This solver returns that rpy
// plus the tilt angle the caller uses to decide whether to insert a bracket.

export type MountFaceLocal = 'bottom' | 'x_plus' | 'x_minus' | 'y_plus' | 'y_minus'

export interface SplitLinkPreset {
  body_mass_frac: number
  output_origin_xyz_mm: [number, number, number]
  output_axis_xyz: [number, number, number]
  output_half_extents_mm: [number, number, number]
  bracket_tilt_threshold_deg: number
}

export interface SolveInput {
  /** World-space outward normal of the face on the parent that the servo mounts to. */
  parentFaceNormalWorld: THREE.Vector3
  /** World-space rotation axis the horn (body-local +Z) must align with. */
  desiredHornAxisWorld: THREE.Vector3
  /** Parent link's world orientation (so we can express the result parent-local). */
  parentWorldQuat: THREE.Quaternion
  preset: { split_link: SplitLinkPreset }
}

export interface SolveResult {
  /** RPY in the parent frame for a DIRECT mount (no bracket). Caller may
   *  ignore this when bodyTiltDeg > bracket_tilt_threshold_deg and route the
   *  servo through an auto-inserted bracket instead. */
  bodyLocalRpy: [number, number, number]
  /** Angle between parentFaceNormal and desiredHornAxis, in degrees. 0 → horn
   *  aligned with the parent's outward face normal (servo sits upright on the
   *  parent with no bracket needed). 90 → horn perpendicular (needs L-bracket
   *  for a realistic mount). 180 → horn points back into the parent (needs
   *  U-bracket or a side-face mount). */
  bodyTiltDeg: number
  /** Which body face the solver designated as the servo's mounting face.
   *  For tilt < 135° the servo's natural bottom face (-Z local) is used;
   *  above that it falls back to a side face so the body need not invert.
   *
   *  ADVISORY ONLY. `bodyLocalRpy` below is the shortest-arc rotation that
   *  satisfies the HORN constraint only (`Q·(+Z) = desiredHornAxisWorld`).
   *  There is no guarantee `mountFaceLocal`'s outward normal actually presses
   *  flat against the parent face — that's the bracket path's job. Treat this
   *  field as a hint for the bracket/validator layer, not a physical claim. */
  mountFaceLocal: MountFaceLocal
  /** False when the solver had to drop a constraint (horn-into-parent and
   *  similar geometrically impossible direct mounts). The caller should
   *  insert a bracket — the pose returned still aligns the horn. */
  feasible: boolean
}

const DEG = THREE.MathUtils.RAD2DEG

/** Compute the servo body's pose so its local +Z (horn) points along the
 *  AI-declared world rotation axis. `bodyTiltDeg` tells the caller whether
 *  a bracket must be auto-inserted (see docs/SERVO_SPLIT_PLAN.md §bracket). */
export function solveServoBodyPose(input: SolveInput): SolveResult {
  const n = input.parentFaceNormalWorld.clone().normalize()
  const h = input.desiredHornAxisWorld.clone().normalize()

  const tiltRad = n.angleTo(h)
  const bodyTiltDeg = tiltRad * DEG

  let mountFaceLocal: MountFaceLocal
  let feasible = true

  if (bodyTiltDeg < 135) {
    // Natural mount face: body bottom (-Z) bolts toward the parent. For
    // tilt > bracket_tilt_threshold_deg the caller inserts a bracket instead
    // of rotating the body; this field stays 'bottom' either way since the
    // body's mount face is the same.
    mountFaceLocal = 'bottom'
  } else {
    // Horn nearly anti-parallel to parent normal → direct mount would invert
    // the body. Fall back to a side face so the body lies on its side and
    // the horn can point back toward the parent. A U-bracket (or the
    // caller's side-mount path) is the clean solution.
    mountFaceLocal = 'x_plus'
    feasible = false
  }

  // Primary constraint: Q · (0, 0, 1) = desiredHornAxisWorld. Shortest-arc
  // rotation leaves a 1-DOF roll ambiguity about the horn axis; we don't
  // resolve it here because the output is axially symmetric about that axis
  // for the purposes of joint axis emission. attach_rpy on the AI side still
  // flows through the upstream placement stack.
  const bodyWorldQuat = new THREE.Quaternion().setFromUnitVectors(
    new THREE.Vector3(0, 0, 1),
    h,
  )

  const localQuat = input.parentWorldQuat.clone().invert().multiply(bodyWorldQuat)
  const bodyLocalRpy = quatToRpy(localQuat)

  return {
    bodyLocalRpy,
    bodyTiltDeg,
    mountFaceLocal,
    feasible,
  }
}

export type BracketKind = 'none' | 'structural_bracket_l' | 'structural_bracket_u'

/** Pick which bracket absorbs the tilt. Matches SERVO_SPLIT_PLAN §bracket rules:
 *  tilt < threshold → no bracket, body mounts directly.
 *  45°–135°        → L-bracket (~90° turn).
 *  135°–225°       → U-bracket (~180° turn). Tilt caps at 180° in our solver
 *                    so the upper band is really [135, 180]. */
export function pickBracket(bodyTiltDeg: number, thresholdDeg: number): BracketKind {
  if (bodyTiltDeg <= thresholdDeg) return 'none'
  if (bodyTiltDeg < 135) return 'structural_bracket_l'
  return 'structural_bracket_u'
}

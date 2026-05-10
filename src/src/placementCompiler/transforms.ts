// Phase 3b — pure transform helpers extracted from urdfAssembly.ts.
//
// Everything in this module is THREE.js / string-arithmetic only. No DOM,
// no scene state, no preset reads beyond what is passed in. urdfAssembly.ts
// re-imports these so its public behavior is unchanged.

import * as THREE from 'three'
import { quatToRpy, rpyToQuat } from '../rotationIO.ts'

export function parseRpyString(rpy: string): [number, number, number] {
  const parts = rpy.split(/\s+/).map(Number)
  return [parts[0] || 0, parts[1] || 0, parts[2] || 0]
}

export function formatRpyTuple(rpy: [number, number, number]): string {
  return rpy.map(v => Number(v).toFixed(4)).join(' ')
}

export function parseXyzString(xyz: string): [number, number, number] {
  const parts = xyz.split(/\s+/).map(Number)
  return [parts[0] || 0, parts[1] || 0, parts[2] || 0]
}

export function rpyMatrix(rpy: string): THREE.Matrix4 {
  return new THREE.Matrix4().makeRotationFromQuaternion(rpyToQuat(parseRpyString(rpy)))
}

export function transformFromXyzRpy(xyz: string, rpy: string): THREE.Matrix4 {
  const [x, y, z] = parseXyzString(xyz)
  const rot = rpyMatrix(rpy)
  rot.setPosition(x, y, z)
  return rot
}

export function formatRpyFromMatrix(m: THREE.Matrix4): string {
  const q = new THREE.Quaternion().setFromRotationMatrix(m)
  return formatRpyTuple(quatToRpy(q))
}

export function worldLevelRpyForParent(parentWorld: THREE.Matrix4 | undefined): string {
  const parentRot = (parentWorld ?? new THREE.Matrix4()).clone()
  parentRot.setPosition(0, 0, 0)
  return formatRpyFromMatrix(parentRot.invert())
}

export function axisNameFromUrdf(axis: string): 'x' | 'y' | 'z' {
  const values = axis.split(/\s+/).map(Number)
  const ax = Math.abs(values[0] || 0)
  const ay = Math.abs(values[1] || 0)
  const az = Math.abs(values[2] || 0)
  if (ax >= ay && ax >= az) return 'x'
  if (ay >= ax && ay >= az) return 'y'
  return 'z'
}

export function axisNameFromComponentAxis(axis?: string): 'x' | 'y' | 'z' {
  const v = (axis || 'z').trim().toLowerCase()
  if (v === 'x' || v === 'y' || v === 'z') return v
  return axisNameFromUrdf(v)
}

export function servoShaftAlignRpy(axisName: 'x' | 'y' | 'z'): string {
  if (axisName === 'x') return '0 1.5708 0'
  if (axisName === 'y') return '-1.5708 0 0'
  return '0 0 0'
}

export function servoDesiredWorldRotation(axisName: 'x' | 'y' | 'z', axisSign = 1): THREE.Matrix4 {
  // Split servos always rotate about local +Z. The graph's joint_axis is a
  // robot-frame semantic axis, so do not interpret it in the already-rotated
  // parent horn frame. These rotations also pick a stable radial zero:
  // for Y-pitch servos local +Y points down in world space, so limb links hang
  // below the horn before rest-pose spin is applied.
  if (axisName === 'y' && axisSign < 0) {
    const mirrored = new THREE.Matrix4()
    // Mirror the physical shaft onto world -Y while keeping local +Y as the
    // radial-down zero for leg chains.
    mirrored.set(
      -1,  0,  0, 0,
       0,  0, -1, 0,
       0, -1,  0, 0,
       0,  0,  0, 1,
    )
    return mirrored
  }
  return rpyMatrix(servoShaftAlignRpy(axisName))
}

export function servoAxisSignFromParentWorld(
  parentWorld: THREE.Matrix4 | undefined,
  axisName: 'x' | 'y' | 'z',
  localOffset?: [number, number, number],
): number {
  if (axisName !== 'y') return 1
  // Mirror decision is based on where the SERVO will sit in world Y, not the
  // parent's origin. For a hip mounted on the baseplate (parent at Y=0), the
  // back-side hip would otherwise pick sign=+1 like the front, leaving back
  // legs un-mirrored relative to the knees (which inherit a thigh whose
  // world Y is already negative). When localOffset is supplied, project it
  // into world via parentWorld and use that Y; otherwise fall back to the
  // parent's world Y for compatibility with older callers.
  //
  // This sign is consumed by TWO downstream paths via the call sites in
  // placementCompiler/index.ts:
  //   1. servoMountRpyForParentWorld → servoDesiredWorldRotation: picks an
  //      antiparallel rotation matrix that flips horn local +Z to world -Y.
  //      This is GEOMETRICALLY REQUIRED for symmetric leg/arm pairs — without
  //      it, both sides' horns face the same world direction.
  //   2. servoLocalRestRpyFromJointRpy: multiplies authored `attach_rpy` by
  //      this sign so a single rest pose (e.g. `attach_rpy=[0, 0.5, 0]`) on
  //      both sides produces world-symmetric bends.
  //
  // In novel mode, path #1 is still required (mirror geometry), but #2 is
  // suppressed at the call site (rest poses applied verbatim per leg, so
  // Claude has full per-component authority). We return the true geometric
  // sign here; the call site decides whether to forward it to the rest-pose
  // path or pin it to +1.
  const probe = new THREE.Vector3(
    localOffset?.[0] ?? 0,
    localOffset?.[1] ?? 0,
    localOffset?.[2] ?? 0,
  )
  if (parentWorld) probe.applyMatrix4(parentWorld)
  return probe.y < -1e-6 ? -1 : 1
}

export function servoMountRpyForParentWorld(
  parentWorld: THREE.Matrix4 | undefined,
  axisName: 'x' | 'y' | 'z',
  localOffset?: [number, number, number],
): string {
  const parentRot = (parentWorld ?? new THREE.Matrix4()).clone()
  parentRot.setPosition(0, 0, 0)
  const axisSign = servoAxisSignFromParentWorld(parentWorld, axisName, localOffset)
  const localRot = new THREE.Matrix4().multiplyMatrices(
    parentRot.invert(),
    servoDesiredWorldRotation(axisName, axisSign),
  )
  return formatRpyFromMatrix(localRot)
}

export function servoPlanarMountRpyForParentWorld(parentWorld: THREE.Matrix4 | undefined, attachFace: string | null | undefined, fallbackRpy: string): string {
  if (attachFace !== 'top' && attachFace !== 'bottom') return fallbackRpy
  const [, , yaw] = parseRpyString(fallbackRpy)
  const desiredWorld = rpyMatrix(attachFace === 'bottom'
    ? `${Math.PI} 0 ${yaw || 0}`
    : `0 0 ${yaw || 0}`)
  const parentRot = (parentWorld ?? new THREE.Matrix4()).clone()
  parentRot.setPosition(0, 0, 0)
  const localRot = new THREE.Matrix4().multiplyMatrices(parentRot.invert(), desiredWorld)
  return formatRpyFromMatrix(localRot)
}

export function servoLocalRestRpyFromJointRpy(rpy: [number, number, number], axisName: 'x' | 'y' | 'z', axisSign = 1): string {
  const axisIndex = axisName === 'x' ? 0 : axisName === 'y' ? 1 : 2
  // Authored attach_rpy is a semantic joint-axis rest angle. Side-axis servos
  // are mirrored left/right in hardware, so convert that semantic angle into
  // the split servo's local +Z horn frame.
  return formatRpyTuple([0, 0, -(rpy[axisIndex] || 0) * axisSign])
}

export function worldOffsetFromParent(parentWorld: THREE.Matrix4 | undefined, localOffset: [number, number, number]): THREE.Vector3 {
  const parentRot = (parentWorld ?? new THREE.Matrix4()).clone()
  parentRot.setPosition(0, 0, 0)
  return new THREE.Vector3(localOffset[0], localOffset[1], localOffset[2]).applyMatrix4(parentRot)
}

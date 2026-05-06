import * as THREE from 'three'
import {
  isDrivetrainComponentId as resolverIsDrivetrainComponentId,
  isFootPadComponentId as resolverIsFootPadComponentId,
  isTireComponentId as resolverIsTireComponentId,
} from './componentResolver.ts'
import type { AttachmentNodeClass, AttachmentNodeDef } from './componentSpec.ts'

export type { AttachmentNodeClass, AttachmentNodeDef } from './componentSpec.ts'

export interface AttachmentNodeRuntime {
  mountLink: string
  parentLink: string
  nodeId: string
  label: string
  cls: AttachmentNodeClass
  single: boolean
  worldPosition: THREE.Vector3
  worldQuaternion: THREE.Quaternion
}

export function isMountLinkName(linkName: string): boolean {
  return linkName.includes('__mount__')
}

export function parseMountLinkName(mountLink: string): { parentLink: string; nodeId: string } | null {
  const parts = mountLink.split('__mount__')
  if (parts.length !== 2) return null
  const parentLink = parts[0]
  const nodeId = parts[1]
  if (!parentLink || !nodeId) return null
  return { parentLink, nodeId }
}

export function makeMountLinkName(parentLink: string, nodeId: string): string {
  return `${parentLink}__mount__${nodeId}`
}

export function isTireComponentId(componentId: string): boolean {
  return resolverIsTireComponentId(componentId)
}

export function isDrivetrainComponentId(componentId: string): boolean {
  return resolverIsDrivetrainComponentId(componentId)
}

export function isFootPadComponentId(componentId: string): boolean {
  return resolverIsFootPadComponentId(componentId)
}

/**
 * Resolve a face name ("top", "front", etc.) to the corresponding attachment node.
 * Returns the node's position offset from the component center.
 */
export function resolveFaceToPort(
  face: string,
  nodes: AttachmentNodeDef[],
): AttachmentNodeDef | undefined {
  if (face === 'coaxial') {
    return nodes.find(n => n.nodeId === 'hub_bore')
      ?? nodes.find(n => n.cls === 'shaft')
      ?? nodes.find(n => n.cls === 'bore')
      ?? nodes.find(n => n.nodeId === 'top')
  }
  const faceToNodeId: Record<string, string> = {
    'top': 'top', 'bottom': 'bottom',
    'front': 'x_plus', 'back': 'x_minus',
    'right': 'y_plus', 'left': 'y_minus',
  }
  const nodeId = faceToNodeId[face] || 'top'
  return nodes.find(n => n.nodeId === nodeId)
}

export function nodeOccupied(
  node: AttachmentNodeRuntime,
  kinematicGraph: Record<string, { name: string; parent?: string; children: string[] }>,
): boolean {
  const mount = kinematicGraph[node.mountLink]
  if (!mount) return false
  return (mount.children?.length ?? 0) > 0
}

export function isCycleIfReparent(
  movingRoot: string,
  newParentLink: string,
  kinematicGraph: Record<string, { name: string; parent?: string; children: string[] }>,
): boolean {
  const stack = [movingRoot]
  const seen = new Set<string>()
  while (stack.length) {
    const cur = stack.pop()!
    if (cur === newParentLink) return true
    if (seen.has(cur)) continue
    seen.add(cur)
    for (const ch of kinematicGraph[cur]?.children ?? []) stack.push(ch)
  }
  return false
}

export function distanceScore(a: THREE.Vector3, b: THREE.Vector3): number {
  return a.distanceTo(b)
}

/**
 * Returns true if two node classes can mate with each other.
 */
export function nodesCompatible(srcCls: AttachmentNodeClass, targetCls: AttachmentNodeClass): boolean {
  if (srcCls === 'generic' || targetCls === 'generic') return true
  if (srcCls === 'mount_face' && targetCls === 'mount_face') return true
  if (srcCls === 'rail' && targetCls === 'rail') return true
  if (srcCls === 'shaft' && targetCls === 'bore') return true
  if (srcCls === 'bore' && targetCls === 'shaft') return true
  return false
}

/**
 * Human-readable description of why two classes don't mate.
 * Returns '' when they are compatible.
 */
export function incompatibleReason(srcCls: AttachmentNodeClass, targetCls: AttachmentNodeClass): string {
  if (nodesCompatible(srcCls, targetCls)) return ''
  return `${srcCls} -> ${targetCls} incompatible`
}

export interface ResolvedConnectionJoint {
  joint_type: 'fixed' | 'revolute' | 'continuous' | 'prismatic'
  axis_xyz: [number, number, number]
}

export function resolveConnectionJoint(
  parentPort: AttachmentNodeDef,
  childPort: AttachmentNodeDef,
  fallback: ResolvedConnectionJoint = { joint_type: 'fixed', axis_xyz: [0, 0, 1] },
): ResolvedConnectionJoint {
  const shaftBore =
    (parentPort.cls === 'shaft' && childPort.cls === 'bore') ||
    (parentPort.cls === 'bore' && childPort.cls === 'shaft')
  if (shaftBore) {
    const shaftPort = parentPort.cls === 'shaft' ? parentPort : childPort
    return {
      joint_type: 'fixed',
      axis_xyz: shaftPort.kinematic?.axis_xyz ?? fallback.axis_xyz,
    }
  }
  if (parentPort.cls === 'rail' && childPort.cls === 'rail') {
    return {
      joint_type: 'prismatic',
      axis_xyz: parentPort.kinematic?.axis_xyz ?? childPort.kinematic?.axis_xyz ?? fallback.axis_xyz,
    }
  }
  return fallback
}

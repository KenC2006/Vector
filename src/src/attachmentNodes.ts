import * as THREE from 'three'

export type AttachmentNodeClass =
  | 'mount_face'
  | 'shaft'
  | 'bore'
  | 'rail'
  | 'generic'

export interface AttachmentNodeDef {
  nodeId: string
  label: string
  cls: AttachmentNodeClass
  /** Local frame relative to the component's main link frame (URDF coordinates). */
  origin_xyz: [number, number, number]
  origin_rpy: [number, number, number]
  /** If true, only one connection may attach to this node. */
  single: boolean
}

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

export function defaultFaceNodesForBoxDims(
  hx: number, hy: number, hz: number,
): AttachmentNodeDef[] {
  // URDF frame conventions in this app are consistent enough for simple face nodes.
  // Nodes are expressed in the component link frame.
  return [
    { nodeId: 'top',     label: 'Top',  cls: 'mount_face', origin_xyz: [0,   0,   hz],  origin_rpy: [0, 0, 0], single: true },
    { nodeId: 'bottom',  label: 'Bot',  cls: 'mount_face', origin_xyz: [0,   0,  -hz],  origin_rpy: [0, 0, 0], single: true },
    { nodeId: 'x_plus',  label: '+X',   cls: 'mount_face', origin_xyz: [hx,  0,   0],   origin_rpy: [0, 0, 0], single: true },
    { nodeId: 'x_minus', label: '-X',   cls: 'mount_face', origin_xyz: [-hx, 0,   0],   origin_rpy: [0, 0, 0], single: true },
    { nodeId: 'y_plus',  label: '+Y',   cls: 'mount_face', origin_xyz: [0,   hy,  0],   origin_rpy: [0, 0, 0], single: true },
    { nodeId: 'y_minus', label: '-Y',   cls: 'mount_face', origin_xyz: [0,  -hy,  0],   origin_rpy: [0, 0, 0], single: true },
  ]
}

export interface ExtractMountNodesArgs {
  kinematicGraph: Record<string, { name: string; parent?: string; children: string[] }>
  kinematicJoints: Record<string, { name: string; type: string; parentLink: string; childLink: string }>
  linkGroups: Map<string, THREE.Group>
}

export function extractMountNodes(args: ExtractMountNodesArgs): AttachmentNodeRuntime[] {
  const out: AttachmentNodeRuntime[] = []
  for (const linkName of Object.keys(args.kinematicGraph)) {
    if (!isMountLinkName(linkName)) continue
    const parsed = parseMountLinkName(linkName)
    if (!parsed) continue
    const mountGroup = args.linkGroups.get(linkName)
    if (!mountGroup) continue
    mountGroup.updateMatrixWorld(true)
    const worldPosition = new THREE.Vector3()
    const worldQuaternion = new THREE.Quaternion()
    const worldScale = new THREE.Vector3()
    mountGroup.matrixWorld.decompose(worldPosition, worldQuaternion, worldScale)

    out.push({
      mountLink: linkName,
      parentLink: parsed.parentLink,
      nodeId: parsed.nodeId,
      label: parsed.nodeId,
      cls: 'mount_face',
      single: true,
      worldPosition,
      worldQuaternion,
    })
  }
  return out
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
  // Cycle if newParent is inside movingRoot subtree.
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


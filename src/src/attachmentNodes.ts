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
  /** Optional joint semantics for a connection made at this node. */
  kinematic?: {
    joint_type?: 'fixed' | 'revolute' | 'continuous' | 'prismatic'
    axis_xyz?: [number, number, number]
  }
  /** If true, only one connection may attach to this node. */
  single: boolean
  /** docs/SERVO_SPLIT_PLAN.md — for split-link servos, tags which sub-link
   *  this port belongs to. `'output'` routes a child's joint parent to the
   *  horn (internal revolute downstream); `'body'` (or undefined) routes to
   *  the body link (fixed external mate). Ignored for non-split presets. */
  subLink?: 'body' | 'output'
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

export function isTireComponentId(componentId: string): boolean {
  return (
    componentId.startsWith('mobility_wheel_') ||
    componentId.startsWith('mobility_mecanum_') ||
    componentId.startsWith('mobility_omni_') ||
    componentId.startsWith('mobility_caster_')
  )
}

export function isDrivetrainComponentId(componentId: string): boolean {
  return componentId.startsWith('drivetrain_')
}

/**
 * Generate component-specific ports based on component ID and mounting_logic.
 * Servos get a shaft_output port on top, mounting ports on bottom/sides.
 * Extrusions get end ports at +Z and -Z tips.
 * This bridges the AI's face-based topology to the port-based snap system.
 */
export function componentPortsForPreset(
  componentId: string,
  hx: number, hy: number, hz: number,
  mountingLogic?: Record<string, unknown>,
): AttachmentNodeDef[] {
  // Tires: single hub_bore node at the wheel center. The cylinder axis is local Z
  // (set by wheelShape's π/2 X-rotation), so a shaft coming in along Z mates.
  // Rim/face nodes are physically meaningless on a spinning tire.
  if (isTireComponentId(componentId)) {
    return [{
      nodeId: 'hub_bore',
      label: 'Hub Bore',
      cls: 'bore',
      origin_xyz: [0, 0, 0],
      origin_rpy: [0, 0, 0],
      kinematic: { joint_type: 'fixed', axis_xyz: [0, 0, 1] },
      single: true,
    }]
  }

  const nodes = defaultFaceNodesForBoxDims(hx, hy, hz)

  // Drivetrain assemblies expose a shaft/bore contract for child tires. The
  // drivetrain link owns the spinning joint; the tire-to-shaft connection is fixed.
  if (isDrivetrainComponentId(componentId) || mountingLogic?.output === 'axial_shaft') {
    const topNode = nodes.find(n => n.nodeId === 'top')
    if (topNode) {
      topNode.cls = 'shaft'
      topNode.label = 'Axial Shaft'
      topNode.kinematic = { joint_type: 'fixed', axis_xyz: [0, 0, 1] }
    }
  }

  // Servos: mark top as shaft output, bottom as bracket mount. Under the
  // split-link model (docs/SERVO_SPLIT_PLAN.md), `top` lives on the output
  // sub-link (horn) and every other face on the body sub-link.
  if (componentId.startsWith('actuator_servo') || componentId.startsWith('actuator_continuous')) {
    const topNode = nodes.find(n => n.nodeId === 'top')
    if (topNode) {
      topNode.cls = 'shaft'
      topNode.label = 'Shaft Output'
      topNode.kinematic = { joint_type: 'fixed', axis_xyz: [0, 0, 1] }
      topNode.subLink = 'output'
    }
    const botNode = nodes.find(n => n.nodeId === 'bottom')
    if (botNode) { botNode.label = 'Bracket Mount'; botNode.subLink = 'body' }
    for (const faceId of ['x_plus', 'x_minus', 'y_plus', 'y_minus'] as const) {
      const n = nodes.find(x => x.nodeId === faceId)
      if (n) n.subLink = 'body'
    }
  }

  // Motors: shaft on top
  if (componentId.startsWith('motor_') || componentId.startsWith('actuator_bldc')) {
    const topNode = nodes.find(n => n.nodeId === 'top')
    if (topNode) { topNode.cls = 'shaft'; topNode.label = 'Shaft'; topNode.kinematic = { joint_type: 'fixed', axis_xyz: [0, 0, 1] } }
  }

  // Extrusions: label ends clearly
  if (componentId.includes('extrusion')) {
    const topNode = nodes.find(n => n.nodeId === 'top')
    if (topNode) topNode.label = 'End A (+Z)'
    const botNode = nodes.find(n => n.nodeId === 'bottom')
    if (botNode) botNode.label = 'End B (-Z)'
  }

  // Grippers/effectors: mark as terminal (single output face)
  if (componentId.startsWith('effector_')) {
    const topNode = nodes.find(n => n.nodeId === 'top')
    if (topNode) topNode.label = 'Tool Output'
  }

  return nodes
}

/** docs/SERVO_SPLIT_PLAN.md — translate an AI-authored link name ("servo_1")
 *  plus a port into the URDF link the mate actually attaches to. For split-
 *  link servos, `port.subLink === 'output'` maps to the `_output` sub-link
 *  (horn); all other ports map to the `_body` sub-link. Returns the raw link
 *  name unchanged when the port has no subLink tag. */
export function resolveSublinkName(
  baseLinkName: string,
  port: { subLink?: 'body' | 'output' } | null | undefined,
): string {
  if (!port?.subLink) return baseLinkName
  return `${baseLinkName}_${port.subLink}`
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

/**
 * Returns true if two node classes can mate with each other.
 * Compatibility table:
 *   mount_face ↔ mount_face
 *   shaft      ↔ bore        (and bore ↔ shaft)
 *   rail       ↔ rail
 *   generic    ↔ anything    (fallback)
 *   anything   ↔ generic     (fallback)
 */
export function nodesCompatible(srcCls: AttachmentNodeClass, targetCls: AttachmentNodeClass): boolean {
  if (srcCls === 'generic' || targetCls === 'generic') return true
  if (srcCls === 'mount_face' && targetCls === 'mount_face') return true
  if (srcCls === 'rail'       && targetCls === 'rail')       return true
  if (srcCls === 'shaft'      && targetCls === 'bore')       return true
  if (srcCls === 'bore'       && targetCls === 'shaft')      return true
  return false
}

/**
 * Human-readable description of why two classes don't mate.
 * Returns '' when they are compatible.
 */
export function incompatibleReason(srcCls: AttachmentNodeClass, targetCls: AttachmentNodeClass): string {
  if (nodesCompatible(srcCls, targetCls)) return ''
  return `${srcCls} ↔ ${targetCls} incompatible`
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

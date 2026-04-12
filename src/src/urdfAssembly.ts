import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { TransformControls } from 'three/addons/controls/TransformControls.js'
import { STLExporter } from 'three/addons/exporters/STLExporter.js'
import { invoke } from '@tauri-apps/api/core'
import { generateVisuals, CATEGORY_COLORS } from './componentMeshes'
import type { UrdfVisualDesc } from './componentMeshes'
import { isMountLinkName, makeMountLinkName, defaultFaceNodesForBoxDims, nodesCompatible, incompatibleReason } from './attachmentNodes'
import type { AttachmentNodeRuntime, AttachmentNodeClass } from './attachmentNodes'
import { hasMeshOverride } from './richVisuals/meshOverrides'
import { SLOW_MESH_BLACKLIST } from './richVisuals/index'

export interface ParsedRobotLike {
  group: THREE.Group
  linkGroups: Map<string, THREE.Group>
  joints: Map<string, { group: THREE.Group; axis: THREE.Vector3; type: string }>
}

export interface UrdfAssemblyContext {
  scene: THREE.Scene
  camera: THREE.PerspectiveCamera
  canvas: HTMLCanvasElement
  controls: OrbitControls
  showToast: (message: string, type?: 'success' | 'warning' | 'error' | 'info') => void
  switchPanel: (name: string) => void
  getUrdfText: () => string
  setUrdfText: (content: string) => void
  reparseUrdf: (xmlOverride?: string) => void
  getParsedRobot: () => ParsedRobotLike
  getKinematicGraph: () => Record<string, { name: string; mass: number; parent?: string; children: string[] }>
  getKinematicJoints: () => Record<string, { name: string; type: string; axis: string; parentLink: string; childLink: string }>
  isViewport3D: () => boolean
  /** `build` = place & snap; `inspect` = click mesh to focus & dashboard (no carry). */
  getInteractionMode: () => 'build' | 'inspect'
  /** Inspect mode: user clicked a URDF link mesh (or null = empty space). */
  onInspectLinkFocused: (linkName: string | null) => void
  /** After URDF reparse / model refresh (restore inspect dimming if needed). */
  onAfterModelUpdated?: () => void
  /** Clear scene root translation before a full replace (e.g. reset). */
  zeroAssemblyWorldPosition?: () => void
  /** Seat assembly on Y=0 after load/reset (not called on every reparse). */
  groundAssembly?: () => void
}

// ── Preset types ──────────────────────────────────────────────────────────────

interface PresetPhysical {
  mass_kg?: number
  mass_kg_per_100mm?: number
  bounding_box_mm?: number[]
  cross_section_mm?: number[]
  inertia_primitive: string
}

interface PresetComponent {
  id: string
  name: string
  description: string
  physical: PresetPhysical
  mechanical_electrical: Record<string, unknown>
  mounting_logic: Record<string, unknown>
  sim_metadata: Record<string, unknown>
}

interface PresetCategory {
  description: string
  components: PresetComponent[]
}

interface PresetData {
  categories: Record<string, PresetCategory>
}

export interface UrdfAssemblyApi {
  onModelUpdated(): void
  recordUndoExternal(content: string): void
  exitCarryMode(): void
  onInteractionModeChanged(mode: 'build' | 'inspect'): void
  /** Sync 3D selection / gizmo / inspector (used when opening Properties from Focus panel). */
  setSelectedLink(linkName: string | null): void
}

function parseNums(s: string, len = 3): number[] {
  const arr = (s || '').trim().split(/\s+/).map(v => Number(v))
  const out: number[] = []
  for (let i = 0; i < len; i++) out.push(Number.isFinite(arr[i]) ? arr[i] : 0)
  return out
}

function fmt(n: number): string {
  return Number.isFinite(n) ? n.toFixed(6).replace(/\.?0+$/, '') : '0'
}

function ensureOrigin(el: Element, doc: Document): Element {
  let origin = el.querySelector(':scope > origin')
  if (!origin) {
    origin = doc.createElement('origin')
    origin.setAttribute('xyz', '0 0 0')
    origin.setAttribute('rpy', '0 0 0')
    el.appendChild(origin)
  }
  if (!origin.getAttribute('xyz')) origin.setAttribute('xyz', '0 0 0')
  if (!origin.getAttribute('rpy')) origin.setAttribute('rpy', '0 0 0')
  return origin
}

function isTypingTarget(t: EventTarget | null): boolean {
  if (!(t instanceof HTMLElement)) return false
  const tag = t.tagName
  if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return true
  if (t.closest('.monaco-editor')) return true
  return false
}

/** Kinematic roots (no parent). Prefer base_link when world is the only virtual root — matches ROS fixed-base URDFs. */
/** Lift a box-shaped carry ghost in world space so its AABB clears y ≈ 0 (floor).
 *  cx/cy/cz are the visual center offset within the carry group's local frame. */
function clampCarryMatrixAboveFloor(worldMat: THREE.Matrix4, hx: number, hy: number, hz: number, cx = 0, cy = 0, cz = 0): THREE.Matrix4 {
  const m = worldMat.clone()
  let minY = Infinity
  for (const sx of [-1, 1] as const) {
    for (const sy of [-1, 1] as const) {
      for (const sz of [-1, 1] as const) {
        const v = new THREE.Vector3(cx + sx * hx, cy + sy * hy, cz + sz * hz).applyMatrix4(m)
        minY = Math.min(minY, v.y)
      }
    }
  }
  const margin = 0.002
  if (minY >= margin) return m
  const lift = margin - minY
  return new THREE.Matrix4().multiplyMatrices(new THREE.Matrix4().makeTranslation(0, lift, 0), m)
}

function resolveFreePlacementParent(graph: Record<string, { name: string; parent?: string }>): string {
  const roots = Object.values(graph).filter(
    l => l?.name && !isMountLinkName(l.name) && !l.parent,
  )
  const primary = roots[0]?.name
  if (!primary) return 'base_link'
  if (primary === 'world' && graph['base_link']) return 'base_link'
  return primary
}

export function initUrdfAssembly(ctx: UrdfAssemblyContext): UrdfAssemblyApi {
  const gizmo = new TransformControls(ctx.camera, ctx.canvas)
  gizmo.setMode('translate')
  gizmo.setSpace('world')
  gizmo.setSize(0.8)
  ctx.scene.add(gizmo.getHelper())

  let selectedLink: string | null = null
  let gizmoBasePivotWorld = new THREE.Matrix4()
  let pointerDown = new THREE.Vector2()
  let urdfUndo: string[] = []
  let urdfRedo: string[] = []
  let rootDragWarned = false

  // ── Attachment nodes (mount-frame links) ────────────────────────────────────
  const nodesGroup = new THREE.Group()
  nodesGroup.name = 'attachment_nodes'
  ctx.scene.add(nodesGroup)
  const nodeRingsGroup = new THREE.Group()
  nodeRingsGroup.name = 'attachment_node_rings'
  ctx.scene.add(nodeRingsGroup)
  const nodeMeshByMount = new Map<string, THREE.Mesh>()
  const nodeRingsByMount = new Map<string, THREE.Group>()
  // Cached occluder list for visibility raycasting — rebuilt with the node graph,
  // not re-collected on every camera-change event.
  // Per-link bounding-box cache. Invalidated in rebuildMountNodes() (called only
  // when the URDF model changes), so computeLinkLocalBoundingBox() does real work
  // only once per model update rather than once per click.
  const linkBBoxCache = new Map<string, THREE.Box3 | null>()
  // Synthesized attachment face nodes. `mountLink` uses the canonical key format
  // `<parentLinkName>__mount__<faceId>` from attachmentNodes.ts (NOT a real URDF link).
  // `localPos` is the face center in the parent link's local frame.
  interface MountNodeEntry extends AttachmentNodeRuntime {
    localPos: THREE.Vector3
  }
  let mountNodes: MountNodeEntry[] = []
  let lastSnapCheckMs = 0
  const occupiedNodeKeys = new Set<string>()

  const NODE_MAT_NEUTRAL = new THREE.MeshBasicMaterial({ color: 0x2f7bff, transparent: true, opacity: 0.95, depthTest: false })
  const NODE_MAT_COMPAT = new THREE.MeshBasicMaterial({ color: 0x44b3ff, transparent: true, opacity: 1, depthTest: false })
  const NODE_MAT_BEST = new THREE.MeshBasicMaterial({ color: 0x2dff8a, transparent: true, opacity: 1, depthTest: false })
  const NODE_MAT_OCCUPIED = new THREE.MeshBasicMaterial({ color: 0xff5533, transparent: true, opacity: 0.95, depthTest: false })
  const NODE_GEO = new THREE.BoxGeometry(0.012, 0.012, 0.012)
  const SNAP_RADIUS_M = 0.05
  const SNAP_ANGLE_RAD = Math.PI / 4
  // Scratch objects reused in hot snap loops to avoid per-frame GC pressure
  const _snapScratchMat = new THREE.Matrix4()
  const _snapScratchInv = new THREE.Matrix4()
  const _snapScratchScale = new THREE.Vector3(1, 1, 1)

  const ghostGroup = new THREE.Group()
  ghostGroup.name = 'snap_ghost'
  ghostGroup.visible = false
  ctx.scene.add(ghostGroup)
  const ghostBox = new THREE.Box3()
  const ghostBoxHelper = new THREE.Box3Helper(ghostBox, 0x33ff99)
  ghostBoxHelper.renderOrder = 999
  ghostGroup.add(ghostBoxHelper)
  let bestMountCandidate: {
    mountLink: string
    targetParentLink: string
    sourceMountLink: string
    reason?: string
    desiredLinkWorld: THREE.Matrix4
  } | null = null

  function updateGhostBoxAtTransform(linkGroup: THREE.Group, desiredLinkWorld: THREE.Matrix4) {
    if (!selectedLink) return
    linkGroup.updateMatrixWorld(true)

    const box = new THREE.Box3().setFromObject(linkGroup)
    if (!isFinite(box.min.x) || !isFinite(box.max.x)) return
    const currentLinkWorld = linkGroup.matrixWorld.clone()
    const delta = desiredLinkWorld.clone().multiply(currentLinkWorld.invert())
    box.applyMatrix4(delta)

    ghostBox.copy(box)
    ghostBoxHelper.updateMatrixWorld(true)
  }

  function clearBestCandidateHighlight() {
    for (const [mount, mesh] of nodeMeshByMount.entries()) {
      mesh.material = isMountOccupied(mount) ? NODE_MAT_OCCUPIED : NODE_MAT_NEUTRAL
    }
  }

  function setNodeMeshState(mountLink: string, state: 'neutral' | 'compatible' | 'best' | 'occupied') {
    const mesh = nodeMeshByMount.get(mountLink)
    if (!mesh) return
    mesh.material = state === 'best' ? NODE_MAT_BEST
      : state === 'compatible' ? NODE_MAT_COMPAT
      : state === 'occupied' ? NODE_MAT_OCCUPIED
      : NODE_MAT_NEUTRAL
  }

  function angleBetweenNodes(a: THREE.Quaternion, b: THREE.Quaternion): number {
    return a.angleTo(b)
  }

  function validateSnapTarget(
    movingLink: string,
    sourceNodeClass: AttachmentNodeClass,
    sourceWorldPos: THREE.Vector3,
    sourceWorldQuat: THREE.Quaternion,
    target: { mountLink: string; parentLink: string; nodeId: string; cls: AttachmentNodeClass; worldPosition: THREE.Vector3; worldQuaternion: THREE.Quaternion },
  ): { ok: boolean; reason: string; dist: number } {
    if (target.parentLink === movingLink) return { ok: false, reason: 'same-component', dist: Infinity }
    if (isMountOccupied(target.mountLink)) return { ok: false, reason: 'occupied', dist: Infinity }
    const dist = target.worldPosition.distanceTo(sourceWorldPos)
    if (dist > SNAP_RADIUS_M) return { ok: false, reason: 'too-far', dist }

    // Topology: do not attach into own subtree.
    const graph = ctx.getKinematicGraph()
    const stack = [movingLink]
    const seen = new Set<string>()
    while (stack.length) {
      const cur = stack.pop()!
      if (cur === target.parentLink) return { ok: false, reason: 'cycle', dist }
      if (seen.has(cur)) continue
      seen.add(cur)
      for (const ch of graph[cur]?.children ?? []) stack.push(ch)
    }

    // Type compatibility via nodesCompatible table.
    if (!nodesCompatible(sourceNodeClass, target.cls)) {
      return { ok: false, reason: incompatibleReason(sourceNodeClass, target.cls), dist }
    }

    const ang = angleBetweenNodes(sourceWorldQuat, target.worldQuaternion)
    if (ang > SNAP_ANGLE_RAD) return { ok: false, reason: 'orientation', dist }

    return { ok: true, reason: 'ok', dist }
  }

  function getSourceNodesForSelected(): Array<{
    mountLink: string
    nodeId: string
    worldPos: THREE.Vector3
    worldQuat: THREE.Quaternion
    localToSelected: THREE.Matrix4
    cls: AttachmentNodeClass
  }> {
    if (!selectedLink) return []
    const selectedGroup = ctx.getParsedRobot().linkGroups.get(selectedLink)
    if (!selectedGroup) return []
    selectedGroup.updateMatrixWorld(true)
    // Face nodes share the link's orientation and sit at localPos within the link frame,
    // so the local-to-selected transform is just a translation by localPos.
    const out: Array<{
      mountLink: string
      nodeId: string
      worldPos: THREE.Vector3
      worldQuat: THREE.Quaternion
      localToSelected: THREE.Matrix4
      cls: AttachmentNodeClass
    }> = []
    for (const n of mountNodes) {
      if (n.parentLink !== selectedLink) continue
      const localToSelected = new THREE.Matrix4().makeTranslation(n.localPos.x, n.localPos.y, n.localPos.z)
      out.push({
        mountLink: n.mountLink,
        nodeId: n.nodeId,
        worldPos: n.worldPosition.clone(),
        worldQuat: n.worldQuaternion.clone(),
        localToSelected,
        cls: n.cls,
      })
    }
    return out
  }

  function updateBestCandidateDuringDrag(_pivot: THREE.Group) {
    // Only refresh the moving component's nodes — static target nodes don't move
    // during drag so their world positions from rebuildMountNodes() are still valid.
    refreshNodeWorldTransforms(selectedLink ?? undefined)

    // Throttle the candidate search to ~20fps — the DFS + distance loop is
    // expensive on large robots and doesn't need to run every mouse-move frame.
    const now = performance.now()
    if (now - lastSnapCheckMs < 50) return
    lastSnapCheckMs = now

    bestMountCandidate = null
    if (mountNodes.length === 0) {
      clearBestCandidateHighlight()
      ghostGroup.visible = false
      return
    }
    if (!selectedLink) return
    const selectedGroup = ctx.getParsedRobot().linkGroups.get(selectedLink)
    if (!selectedGroup) return
    selectedGroup.updateMatrixWorld(true)
    const sourceNodes = getSourceNodesForSelected()
    if (sourceNodes.length === 0) {
      ghostGroup.visible = false
      clearBestCandidateHighlight()
      return
    }

    // Choose closest valid source-target pair.
    let best: {
      mountLink: string
      targetParentLink: string
      sourceMountLink: string
      dist: number
      reason: string
      desiredLinkWorld: THREE.Matrix4
    } | null = null
    let firstFailureReason: string | null = null

    for (const srcNode of sourceNodes) {
      for (const target of mountNodes) {
        if (target.parentLink === selectedLink) continue
        const verdict = validateSnapTarget(selectedLink, srcNode.cls, srcNode.worldPos, srcNode.worldQuat, target)
        if (!verdict.ok) {
          if (!firstFailureReason) firstFailureReason = verdict.reason
          continue
        }
        _snapScratchInv.copy(srcNode.localToSelected).invert()
        _snapScratchMat.compose(target.worldPosition, target.worldQuaternion, _snapScratchScale)
        const desiredLinkWorld = _snapScratchMat.clone().multiply(_snapScratchInv)
        if (!best || verdict.dist < best.dist) {
          best = {
            mountLink: target.mountLink,
            targetParentLink: target.parentLink,
            sourceMountLink: srcNode.mountLink,
            dist: verdict.dist,
            reason: 'ok',
            desiredLinkWorld,
          }
        }
      }
    }

    clearBestCandidateHighlight()
    nodesGroup.visible = true
    for (const target of mountNodes) {
      setNodeMeshState(target.mountLink, isMountOccupied(target.mountLink) ? 'occupied' : 'neutral')
    }
    for (const target of mountNodes) {
      if (target.parentLink === selectedLink) continue
      setNodeMeshState(target.mountLink, isMountOccupied(target.mountLink) ? 'occupied' : 'compatible')
    }

    if (!best || !Number.isFinite(best.dist) || !best.desiredLinkWorld) {
      ghostGroup.visible = false
      bestMountCandidate = firstFailureReason
        ? { mountLink: '', targetParentLink: '', sourceMountLink: '', reason: firstFailureReason, desiredLinkWorld: new THREE.Matrix4() }
        : null
      return
    }
    bestMountCandidate = {
      mountLink: best.mountLink,
      targetParentLink: best.targetParentLink,
      sourceMountLink: best.sourceMountLink,
      reason: best.reason,
      desiredLinkWorld: best.desiredLinkWorld,
    }
    setNodeMeshState(best.mountLink, 'best')

    // Ghost preview: show subtree bounding box at solved snapped pose.
    ghostGroup.visible = true
    updateGhostBoxAtTransform(selectedGroup, best.desiredLinkWorld)
  }

  function isMountOccupied(mountKey: string): boolean {
    return occupiedNodeKeys.has(mountKey)
  }

  // Compute the bounding box of a link's own geometry in its local frame,
  // excluding child-link pivot subtrees so we don't include downstream components.
  function computeLinkLocalBoundingBox(linkGroup: THREE.Group): THREE.Box3 | null {
    const pivotGroups = new Set<THREE.Object3D>()
    for (const [, jointInfo] of ctx.getParsedRobot().joints) {
      pivotGroups.add(jointInfo.group)
    }

    linkGroup.updateMatrixWorld(true)
    const linkWorldInv = linkGroup.matrixWorld.clone().invert()

    const box = new THREE.Box3()
    let hasGeom = false

    const tempBox = new THREE.Box3()
    const tempMatrix = new THREE.Matrix4()
    function walk(obj: THREE.Object3D) {
      if (obj !== linkGroup && pivotGroups.has(obj)) return
      const mesh = obj as THREE.Mesh
      if (mesh.isMesh && mesh.geometry) {
        if (!mesh.geometry.boundingBox) mesh.geometry.computeBoundingBox()
        const bb = mesh.geometry.boundingBox
        if (bb) {
          mesh.updateMatrixWorld(true)
          tempMatrix.multiplyMatrices(linkWorldInv, mesh.matrixWorld)
          tempBox.copy(bb).applyMatrix4(tempMatrix)
          box.union(tempBox)
          hasGeom = true
        }
      }
      for (const child of obj.children) walk(child)
    }
    walk(linkGroup)

    return hasGeom ? box : null
  }

  function rebuildMountNodes() {
    nodesGroup.clear()
    nodeRingsGroup.clear()
    nodeMeshByMount.clear()
    nodeRingsByMount.clear()
    mountNodes = []
    occupiedNodeKeys.clear()
    linkBBoxCache.clear()
    _pickTargetCache = null

    const graph = ctx.getKinematicGraph()
    const kinJoints = ctx.getKinematicJoints()
    const parsed = ctx.getParsedRobot()

    const OCCUPIED_DIST_M = 0.025

    for (const linkName of Object.keys(graph)) {
      if (isMountLinkName(linkName)) continue
      const lg = parsed.linkGroups.get(linkName)
      if (!lg) continue
      let localBox: THREE.Box3 | null
      if (linkBBoxCache.has(linkName)) {
        localBox = linkBBoxCache.get(linkName)!
      } else {
        localBox = computeLinkLocalBoundingBox(lg)
        linkBBoxCache.set(linkName, localBox)
      }
      if (!localBox || localBox.isEmpty()) continue

      const center = localBox.getCenter(new THREE.Vector3())
      const size = localBox.getSize(new THREE.Vector3())
      const half = new THREE.Vector3(size.x / 2, size.y / 2, size.z / 2)

      // Gather child-joint origin xyz in parent-local frame, used to determine occupancy.
      const childOriginsLocal: THREE.Vector3[] = []
      for (const j of Object.values(kinJoints)) {
        if (j.parentLink !== linkName) continue
        if (isMountLinkName(j.childLink)) continue
        // The URDF parser retains joint origin in the pivot group transform of the child link.
        const childLg = parsed.linkGroups.get(j.childLink)
        if (!childLg) continue
        // Find the pivot group whose parent is lg
        let pivot: THREE.Object3D | null = childLg.parent
        if (!pivot) continue
        // pivot's position is joint origin in parent-local (lg) frame
        childOriginsLocal.push(new THREE.Vector3().copy(pivot.position))
      }

      lg.updateMatrixWorld(true)
      const linkWorldQuat = new THREE.Quaternion()
      lg.matrixWorld.decompose(new THREE.Vector3(), linkWorldQuat, new THREE.Vector3())

      const faceDefs = defaultFaceNodesForBoxDims(half.x, half.y, half.z)

      for (const f of faceDefs) {
        const localPos = new THREE.Vector3(
          center.x + f.origin_xyz[0],
          center.y + f.origin_xyz[1],
          center.z + f.origin_xyz[2],
        )
        const nodeKey = makeMountLinkName(linkName, f.nodeId)
        const worldPosition = localPos.clone().applyMatrix4(lg.matrixWorld)
        const worldQuaternion = linkWorldQuat.clone()

        // Occupancy: is there a child joint whose origin sits near this face?
        let occupied = false
        for (const co of childOriginsLocal) {
          if (co.distanceTo(localPos) <= OCCUPIED_DIST_M) { occupied = true; break }
        }
        if (occupied) occupiedNodeKeys.add(nodeKey)

        mountNodes.push({
          mountLink: nodeKey,
          parentLink: linkName,
          nodeId: f.nodeId,
          label: f.label,
          cls: f.cls,
          single: f.single,
          localPos,
          worldPosition,
          worldQuaternion,
        })

        const mesh = new THREE.Mesh(NODE_GEO, occupied ? NODE_MAT_OCCUPIED : NODE_MAT_NEUTRAL)
        mesh.position.copy(worldPosition)
        mesh.quaternion.copy(worldQuaternion)
        mesh.renderOrder = 999
        nodesGroup.add(mesh)
        nodeMeshByMount.set(nodeKey, mesh)

        const rings = makeNodeAxisRings()
        rings.position.copy(worldPosition)
        rings.quaternion.copy(worldQuaternion)
        nodeRingsGroup.add(rings)
        nodeRingsByMount.set(nodeKey, rings)
      }
    }
    // Nodes are only shown while actively dragging a component; the drag
    // handler flips this on/off. Keep the group hidden by default.
    nodesGroup.visible = false
    applyNodeRingVisibility()

  }

  /** Refresh world transforms for mount nodes.
   *  Pass `onlyLink` to limit refresh to nodes owned by that link — use this
   *  during drag where only the selected component is moving and all static
   *  target nodes already have valid world positions from rebuildMountNodes(). */
  function refreshNodeWorldTransforms(onlyLink?: string) {
    const parsed = ctx.getParsedRobot()
    const worldQuatTmp = new THREE.Quaternion()
    const worldPosTmp = new THREE.Vector3()
    const worldScaleTmp = new THREE.Vector3()
    // Avoid redundant updateMatrixWorld calls for the same link (each link has 6 face nodes)
    const updatedLinks = new Set<string>()
    for (const n of mountNodes) {
      if (onlyLink !== undefined && n.parentLink !== onlyLink) continue
      const lg = parsed.linkGroups.get(n.parentLink)
      if (!lg) continue
      if (!updatedLinks.has(n.parentLink)) {
        lg.updateMatrixWorld(true)
        updatedLinks.add(n.parentLink)
      }
      lg.matrixWorld.decompose(worldPosTmp, worldQuatTmp, worldScaleTmp)
      n.worldQuaternion.copy(worldQuatTmp)
      n.worldPosition.copy(n.localPos).applyMatrix4(lg.matrixWorld)

      const mesh = nodeMeshByMount.get(n.mountLink)
      if (mesh) {
        mesh.position.copy(n.worldPosition)
        mesh.quaternion.copy(n.worldQuaternion)
      }
      const rings = nodeRingsByMount.get(n.mountLink)
      if (rings) {
        rings.position.copy(n.worldPosition)
        rings.quaternion.copy(n.worldQuaternion)
      }
    }
  }


  const buildTree = document.getElementById('build-tree') as HTMLDivElement | null
  const buildEmpty = document.getElementById('build-empty') as HTMLDivElement | null
  const bsParts = document.getElementById('bs-parts') as HTMLSpanElement | null
  const bsMass = document.getElementById('bs-mass') as HTMLSpanElement | null
  const bsJoints = document.getElementById('bs-joints') as HTMLSpanElement | null
  const inspBody = document.querySelector('#panel-inspector .insp-body') as HTMLDivElement | null
  const inspTitle = document.getElementById('insp-title') as HTMLSpanElement | null
  const toolboxSearch = document.getElementById('toolbox-search') as HTMLInputElement | null
  const btnFocusBase = document.getElementById('btn-load-example') as HTMLButtonElement | null
  const btnResetRobot = document.getElementById('btn-clear-assembly') as HTMLButtonElement | null
  const toggleMountRingsBtn = document.getElementById('toggle-mount-rings') as HTMLButtonElement | null
  let showNodeRings = false

  function makeNodeAxisRings(): THREE.Group {
    const rings = new THREE.Group()
    const radius = 0.018
    const tube = 0.0015
    const ringGeo = new THREE.TorusGeometry(radius, tube, 8, 40)
    const matX = new THREE.MeshBasicMaterial({ color: 0xff6666, transparent: true, opacity: 0.9, depthTest: false })
    const matY = new THREE.MeshBasicMaterial({ color: 0x66ff66, transparent: true, opacity: 0.9, depthTest: false })
    const matZ = new THREE.MeshBasicMaterial({ color: 0x66aaff, transparent: true, opacity: 0.9, depthTest: false })
    const rx = new THREE.Mesh(ringGeo, matX)
    rx.rotation.y = Math.PI / 2
    const ry = new THREE.Mesh(ringGeo, matY)
    ry.rotation.x = Math.PI / 2
    const rz = new THREE.Mesh(ringGeo, matZ)
    rings.add(rx, ry, rz)
    rings.name = 'node-axis-rings'
    rings.visible = showNodeRings
    return rings
  }

  function applyNodeRingVisibility() {
    nodeRingsGroup.visible = showNodeRings && nodesGroup.visible
  }

  function recordUndo() {
    urdfUndo.push(ctx.getUrdfText())
    if (urdfUndo.length > 80) urdfUndo.shift()
    urdfRedo = []
  }

  /** Pretty-print an XML Document with 2-space indentation. */
  function formatXml(doc: Document): string {
    const INDENT = '  '
    function indent(node: Node, depth: number): void {
      const children = Array.from(node.childNodes)
      // Remove existing text-only whitespace nodes so we can re-insert our own.
      for (const child of children) {
        if (child.nodeType === Node.TEXT_NODE && child.textContent?.trim() === '') {
          node.removeChild(child)
        }
      }
      const elements = Array.from(node.childNodes).filter(c => c.nodeType === Node.ELEMENT_NODE)
      if (elements.length === 0) return
      for (const el of elements) {
        node.insertBefore(doc.createTextNode('\n' + INDENT.repeat(depth + 1)), el)
        indent(el, depth + 1)
      }
      node.appendChild(doc.createTextNode('\n' + INDENT.repeat(depth)))
    }
    indent(doc.documentElement, 0)
    return new XMLSerializer().serializeToString(doc)
  }

  function commitUrdf(mutator: (doc: Document) => boolean, opts?: { defer?: boolean }): boolean {
    const current = ctx.getUrdfText()
    const parser = new DOMParser()
    const doc = parser.parseFromString(current, 'application/xml')
    if (doc.documentElement.nodeName === 'parsererror') {
      ctx.showToast('Cannot edit invalid URDF', 'error')
      return false
    }
    recordUndo()
    const changed = mutator(doc)
    if (!changed) {
      urdfUndo.pop()
      return false
    }
    const xml = formatXml(doc)
    ctx.setUrdfText(xml)
    // Defer to next animation frame when called from pointer-up handlers to avoid
    // blocking the frame that clears the drag (full scene rebuild can take 100+ ms).
    if (opts?.defer) {
      requestAnimationFrame(() => ctx.reparseUrdf(xml))
    } else {
      ctx.reparseUrdf(xml)
    }
    return true
  }

  let _pickTargetCache: THREE.Mesh[] | null = null
  function getPickTargets(): THREE.Mesh[] {
    if (_pickTargetCache) return _pickTargetCache
    const targets: THREE.Mesh[] = []
    ctx.getParsedRobot().group.traverse(o => {
      if (o instanceof THREE.Mesh) {
        const name = (o.userData as Record<string, unknown>).urdfLinkName
        if (typeof name === 'string' && name && !isMountLinkName(name)) targets.push(o)
      }
    })
    _pickTargetCache = targets
    return targets
  }

  function makeRaycaster(e: { clientX: number; clientY: number }): THREE.Raycaster {
    const rect = ctx.canvas.getBoundingClientRect()
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    )
    const ray = new THREE.Raycaster()
    ray.setFromCamera(ndc, ctx.camera)
    return ray
  }

  function getParentJointForLink(linkName: string): { name: string; type: string; axis: string; parentLink: string; childLink: string } | null {
    const joints = ctx.getKinematicJoints()
    return Object.values(joints).find(j => j.childLink === linkName) ?? null
  }

  function getPivotGroupForLink(linkName: string): THREE.Group | null {
    const parentJoint = getParentJointForLink(linkName)
    if (!parentJoint) return null
    return ctx.getParsedRobot().joints.get(parentJoint.name)?.group ?? null
  }

  function refreshBuildPanel() {
    if (!buildTree || !buildEmpty || !bsParts || !bsMass || !bsJoints) return
    const graph = ctx.getKinematicGraph()
    const joints = ctx.getKinematicJoints()
    const links = Object.values(graph).filter(l => !isMountLinkName(l.name))
    const totalMass = links.reduce((sum, l) => sum + (l.mass || 0), 0)
    bsParts.textContent = String(links.length)
    bsMass.textContent = `${Math.round(totalMass * 1000)} g`
    bsJoints.textContent = String(Object.keys(joints).length)
    buildEmpty.classList.toggle('hidden', links.length > 0)

    const jointByChild = new Map<string, { name: string; type: string; parentLink: string }>()
    for (const j of Object.values(joints)) jointByChild.set(j.childLink, j)
    const roots = links.filter(l => !l.parent)
    const rows: Array<{ link: string; depth: number }> = []
    const walk = (name: string, depth: number) => {
      if (isMountLinkName(name)) return
      rows.push({ link: name, depth })
      const node = graph[name]
      if (!node) return
      for (const c of node.children) walk(c, depth + 1)
    }
    for (const r of roots) walk(r.name, 0)

    buildTree.innerHTML = ''
    for (const row of rows) {
      const meta = graph[row.link]
      const edge = jointByChild.get(row.link)
      const el = document.createElement('div')
      el.className = 'bt-row' + (selectedLink === row.link ? ' selected' : '')
      el.style.paddingLeft = `${10 + row.depth * 14}px`

      // Detect category from link name (preset links use ID prefixes)
      let catDot = ''
      if (presetData) {
        for (const [catName, cat] of Object.entries(presetData.categories)) {
          if (cat.components.some(c => row.link.startsWith(c.id))) {
            const cc = CATEGORY_COLORS[catName] ?? [0.6, 0.6, 0.6, 1]
            catDot = `<span class="bt-cat-dot" style="background:rgb(${Math.round(cc[0]*255)},${Math.round(cc[1]*255)},${Math.round(cc[2]*255)})"></span>`
            break
          }
        }
      }

      el.innerHTML = `
        <span class="bt-joint-badge">${edge ? edge.type.slice(0, 3) : 'root'}</span>
        ${catDot}
        <span class="bt-name">${row.link}</span>
        <span class="bt-mass">${Math.round((meta?.mass || 0) * 1000)}g</span>
      `
      el.addEventListener('click', () => selectLink(row.link))
      buildTree.appendChild(el)
    }
  }

  function renderInspector() {
    if (!inspBody || !inspTitle) return
    if (!selectedLink) {
      inspTitle.textContent = 'Properties'
      inspBody.innerHTML = '<div class="insp-empty">Select a URDF link in viewport or build tree</div>'
      return
    }
    const joints = ctx.getKinematicJoints()
    const parentJoint = Object.values(joints).find(j => j.childLink === selectedLink) || null
    // Find synthetic face nodes belonging to the currently selected link, plus any
    // child links whose joint origins match those faces (= occupying components).
    const nodesForLink = mountNodes.filter(n => n.parentLink === selectedLink)
    const childJointsByOrigin = Object.values(joints).filter(j => j.parentLink === selectedLink && !isMountLinkName(j.childLink))
    const mountRows = nodesForLink.map(n => {
      const occupied = isMountOccupied(n.mountLink)
      let occBy = ''
      if (occupied) {
        // Find the child joint whose pivot position is closest to this face
        const parsed = ctx.getParsedRobot()
        let bestDist = Infinity
        for (const j of childJointsByOrigin) {
          const childLg = parsed.linkGroups.get(j.childLink)
          const pivot = childLg?.parent
          if (!pivot) continue
          const d = pivot.position.distanceTo(n.localPos)
          if (d < bestDist) { bestDist = d; occBy = j.childLink }
        }
      }
      return `<div class="insp-row">
        <span class="insp-key">${n.nodeId}</span>
        <span class="insp-val">${occupied ? `occupied by ${occBy}` : 'free'}</span>
        ${occupied && occBy ? `<button type="button" class="bi-action-btn" data-detach-mount="${n.mountLink}" data-detach-child="${occBy}">Detach</button>` : ''}
      </div>`
    }).join('')
    inspTitle.textContent = selectedLink
    inspBody.innerHTML = `
      <div class="bi-section">
        <div class="bi-section-title">Link</div>
        <div class="insp-row"><span class="insp-key">Name</span><span class="insp-val">${selectedLink}</span></div>
      </div>
      <div class="bi-section">
        <div class="bi-section-title">Attachment Nodes</div>
        ${nodesForLink.length ? mountRows : '<div class="insp-empty">No mount nodes on this link</div>'}
      </div>
      <div class="bi-section">
        <div class="bi-section-title">Parent Joint</div>
        <div class="insp-row"><span class="insp-key">Name</span><span class="insp-val">${parentJoint?.name ?? 'root'}</span></div>
        <div class="insp-row"><span class="insp-key">Type</span><span class="insp-val">${parentJoint?.type ?? '--'}</span></div>
        ${parentJoint ? `
          <div class="joint-limits-row">
            <span class="joint-lim-label">Origin XYZ</span>
            <input id="urdf-origin-x" class="joint-lim-input" />
            <input id="urdf-origin-y" class="joint-lim-input" />
            <input id="urdf-origin-z" class="joint-lim-input" />
          </div>
          <div class="joint-limits-row">
            <span class="joint-lim-label">Origin RPY</span>
            <input id="urdf-origin-r" class="joint-lim-input" />
            <input id="urdf-origin-p" class="joint-lim-input" />
            <input id="urdf-origin-yaw" class="joint-lim-input" />
          </div>
        ` : '<div class="insp-empty">Root link has no parent joint origin</div>'}
      </div>
      <div class="insp-actions-group">
        ${parentJoint ? '<button type="button" class="bi-action-btn apply-btn" id="urdf-apply-origin">Apply Changes</button>' : ''}
        <button type="button" class="bi-action-btn" id="btn-export-link-stl" style="width:100%">Export Link STL</button>
        <button type="button" class="bi-action-btn" id="btn-duplicate-link" style="width:100%">Duplicate Subtree</button>
        <button type="button" class="bi-action-btn danger-btn" id="urdf-delete-link">Delete Link</button>
      </div>
    `
    // Wire up delete button (available for all links)
    const deleteBtn = document.getElementById('urdf-delete-link')
    if (deleteBtn && selectedLink) {
      const linkToDelete = selectedLink
      deleteBtn.addEventListener('click', () => deleteLink(linkToDelete))
    }
    // Wire up duplicate button
    const dupBtn = document.getElementById('btn-duplicate-link')
    if (dupBtn && selectedLink) {
      const linkToDup = selectedLink
      dupBtn.addEventListener('click', () => duplicateSubtree(linkToDup))
    }

    // Wire up detach buttons
    inspBody.querySelectorAll('button[data-detach-mount]').forEach(btn => {
      btn.addEventListener('click', () => {
        const mount = (btn as HTMLElement).getAttribute('data-detach-mount') || ''
        const child = (btn as HTMLElement).getAttribute('data-detach-child') || ''
        if (!mount || !child) return
        const childParentJoint = getParentJointForLink(child)
        if (!childParentJoint) return
        const ok = commitUrdf(documentXml => {
          const jointEl = documentXml.querySelector(`joint[name="${childParentJoint.name}"]`)
          if (!jointEl) return false
          const pEl = jointEl.querySelector('parent')
          if (!pEl) return false
          // Reattach the detached child to the currently selected link at origin.
          pEl.setAttribute('link', selectedLink!)
          const origin = ensureOrigin(jointEl, documentXml)
          origin.setAttribute('xyz', '0 0 0')
          origin.setAttribute('rpy', origin.getAttribute('rpy') || '0 0 0')
          return true
        })
        if (ok) ctx.showToast(`Detached ${child} from ${mount}`, 'success')
      })
    })
    // Wire up per-link STL export button
    const exportLinkBtn = document.getElementById('btn-export-link-stl')
    if (exportLinkBtn) {
      exportLinkBtn.addEventListener('click', exportSelectedLinkSTL)
    }

    if (!parentJoint) return

    const parser = new DOMParser()
    const doc = parser.parseFromString(ctx.getUrdfText(), 'application/xml')
    const jointEl = doc.querySelector(`joint[name="${parentJoint.name}"]`)
    const origin = jointEl ? ensureOrigin(jointEl, doc) : null
    const xyz = parseNums(origin?.getAttribute('xyz') || '0 0 0', 3)
    const rpy = parseNums(origin?.getAttribute('rpy') || '0 0 0', 3)

    const x = document.getElementById('urdf-origin-x') as HTMLInputElement | null
    const y = document.getElementById('urdf-origin-y') as HTMLInputElement | null
    const z = document.getElementById('urdf-origin-z') as HTMLInputElement | null
    const r = document.getElementById('urdf-origin-r') as HTMLInputElement | null
    const p = document.getElementById('urdf-origin-p') as HTMLInputElement | null
    const yw = document.getElementById('urdf-origin-yaw') as HTMLInputElement | null
    if (!x || !y || !z || !r || !p || !yw) return
    x.value = String(xyz[0]); y.value = String(xyz[1]); z.value = String(xyz[2])
    r.value = String(rpy[0]); p.value = String(rpy[1]); yw.value = String(rpy[2])

    const applyBtn = document.getElementById('urdf-apply-origin')
    applyBtn?.addEventListener('click', () => {
      commitUrdf(documentXml => {
        const j = documentXml.querySelector(`joint[name="${parentJoint.name}"]`)
        if (!j) return false
        const o = ensureOrigin(j, documentXml)
        o.setAttribute('xyz', `${fmt(Number(x.value))} ${fmt(Number(y.value))} ${fmt(Number(z.value))}`)
        o.setAttribute('rpy', `${fmt(Number(r.value))} ${fmt(Number(p.value))} ${fmt(Number(yw.value))}`)
        return true
      })
      ctx.showToast('Updated joint origin in URDF', 'success')
    })
  }

  function deleteLink(linkName: string) {
    if (!linkName) return
    const graph = ctx.getKinematicGraph()
    const node = graph[linkName]
    if (!node) return

    // Prevent deleting root link if it's the only one
    if (!node.parent && Object.keys(graph).length <= 1) {
      ctx.showToast('Cannot delete the only remaining link', 'warning')
      return
    }

    // Collect subtree: the link itself + all descendants
    const toRemove = new Set<string>()
    const walk = (name: string) => {
      toRemove.add(name)
      const n = graph[name]
      if (n) n.children.forEach(walk)
    }
    walk(linkName)

    const childCount = toRemove.size - 1
    const label = childCount > 0 ? `"${linkName}" and ${childCount} child link${childCount > 1 ? 's' : ''}` : `"${linkName}"`

    const changed = commitUrdf(doc => {
      const robot = doc.documentElement
      if (!robot || robot.nodeName !== 'robot') return false

      // Remove all links in subtree
      for (const name of toRemove) {
        const linkEl = doc.querySelector(`link[name="${name}"]`)
        if (linkEl) robot.removeChild(linkEl)
      }

      // Remove all joints whose parent or child is in the subtree
      const joints = doc.querySelectorAll('joint')
      joints.forEach(j => {
        const parentName = j.querySelector('parent')?.getAttribute('link')
        const childName = j.querySelector('child')?.getAttribute('link')
        if ((parentName && toRemove.has(parentName)) || (childName && toRemove.has(childName))) {
          robot.removeChild(j)
        }
      })

      return true
    })

    if (changed) {
      gizmo.detach()
      selectedLink = null
      ctx.showToast(`Deleted ${label}`, 'success')
      refreshBuildPanel()
      renderInspector()
    }
  }

  function duplicateSubtree(linkName: string) {
    if (!linkName) return
    const graph = ctx.getKinematicGraph()
    const node = graph[linkName]
    if (!node) return

    // Collect subtree links in BFS order (skip synthesized mount nodes)
    const subtreeLinks: string[] = []
    const walkDup = (name: string) => {
      if (isMountLinkName(name)) return
      subtreeLinks.push(name)
      const n = graph[name]
      if (n) n.children.filter(c => !isMountLinkName(c)).forEach(walkDup)
    }
    walkDup(linkName)

    // Build a unique suffix: find a free index
    let suffix = 2
    while (graph[`${linkName}_dup${suffix}`]) suffix++

    // Build name mapping: old -> new
    const nameMap = new Map<string, string>()
    for (const name of subtreeLinks) {
      // Derive new name by appending dup suffix to the base of each link
      nameMap.set(name, `${name}_dup${suffix}`)
    }

    const changed = commitUrdf(doc => {
      const robot = doc.documentElement
      if (!robot || robot.nodeName !== 'robot') return false

      // Clone and rename each link in subtree
      for (const name of subtreeLinks) {
        const linkEl = doc.querySelector(`link[name="${name}"]`)
        if (!linkEl) continue
        const cloned = linkEl.cloneNode(true) as Element
        cloned.setAttribute('name', nameMap.get(name)!)
        // Also rename any mount link children that reference this link
        robot.appendChild(cloned)
      }

      // Clone all joints that connect nodes within the subtree,
      // plus the parent joint for the root link
      const joints = Array.from(doc.querySelectorAll('joint'))
      const subtreeSet = new Set(subtreeLinks)
      for (const j of joints) {
        const parentAttr = j.querySelector('parent')?.getAttribute('link') ?? ''
        const childAttr = j.querySelector('child')?.getAttribute('link') ?? ''
        const parentInSubtree = subtreeSet.has(parentAttr)
        const childInSubtree = subtreeSet.has(childAttr)

        if (!childInSubtree) continue  // only clone joints whose child is in the subtree

        const cloned = j.cloneNode(true) as Element
        // Rename the joint itself
        cloned.setAttribute('name', `${j.getAttribute('name')}_dup${suffix}`)
        // Update parent link reference (if in subtree, remap; otherwise keep original parent)
        const newParentEl = cloned.querySelector('parent')
        if (newParentEl) {
          const mappedParent = nameMap.get(parentAttr)
          newParentEl.setAttribute('link', mappedParent ?? parentAttr)
        }
        // Update child link reference
        const newChildEl = cloned.querySelector('child')
        if (newChildEl) {
          newChildEl.setAttribute('link', nameMap.get(childAttr) ?? childAttr)
        }
        // For the root joint (parent NOT in subtree), offset position slightly so it doesn't overlap
        if (!parentInSubtree) {
          const originEl = cloned.querySelector('origin') ?? (() => {
            const o = doc.createElement('origin'); cloned.appendChild(o); return o
          })()
          const xyz = (originEl.getAttribute('xyz') ?? '0 0 0').split(' ').map(Number)
          xyz[2] = (xyz[2] || 0) + 0.05  // nudge +5cm in Z so the duplicate is visible
          originEl.setAttribute('xyz', xyz.map(v => v.toFixed(4)).join(' '))
        }
        robot.appendChild(cloned)
      }

      return true
    })

    if (changed) {
      const newRoot = nameMap.get(linkName)!
      ctx.showToast(`Duplicated "${linkName}" → "${newRoot}"`, 'success')
      selectLink(newRoot)
      refreshBuildPanel()
    }
  }

  function selectLink(name: string | null) {
    if (name && isMountLinkName(name)) {
      selectedLink = null
      gizmo.detach()
      nodesGroup.visible = false
      applyNodeRingVisibility()
      ghostGroup.visible = false
      bestMountCandidate = null
      clearBestCandidateHighlight()
      refreshBuildPanel()
      renderInspector()
      updateParentIndicator()
      return
    }
    selectedLink = name
    gizmo.detach()
    rootDragWarned = false
    if (name) {
      const pivot = getPivotGroupForLink(name)
      if (pivot) {
        gizmo.attach(pivot)
        pivot.updateMatrixWorld(true)
        gizmoBasePivotWorld.copy(pivot.matrixWorld)
      } else {
        ctx.showToast('Root movement disabled (no world joint yet)', 'warning')
        rootDragWarned = true
      }
    }
    if (!name) {
      nodesGroup.visible = false
      applyNodeRingVisibility()
      ghostGroup.visible = false
      bestMountCandidate = null
      clearBestCandidateHighlight()
    }
    // rebuildMountNodes() is NOT called here — node geometry only changes when
    // the URDF model changes (onModelUpdated) or carry/drag starts. Calling it
    // on every click was recreating all geometry and recomputing all bounding
    // boxes even though the structure hadn't changed, causing visible stutter.
    refreshBuildPanel()
    renderInspector()
    updateParentIndicator()
  }

  const parentNameEl = document.getElementById('tb-parent-name') as HTMLSpanElement | null
  const parentIndicator = document.getElementById('tb-parent-indicator') as HTMLDivElement | null

  function updateParentIndicator() {
    if (!parentNameEl || !parentIndicator) return
    if (selectedLink) {
      parentNameEl.textContent = selectedLink
      parentIndicator.classList.add('has-selection')
    } else {
      parentNameEl.textContent = 'none selected'
      parentIndicator.classList.remove('has-selection')
    }
  }

  // ── Component Presets (loaded from generic_presets.json) ────────────────────

  const compItems = document.getElementById('comp-items') as HTMLDivElement | null
  const compDetail = document.getElementById('comp-detail') as HTMLDivElement | null
  let presetData: PresetData | null = null

  function computeBoxInertia(mass: number, xm: number, ym: number, zm: number) {
    return {
      ixx: mass / 12 * (ym * ym + zm * zm),
      iyy: mass / 12 * (xm * xm + zm * zm),
      izz: mass / 12 * (xm * xm + ym * ym),
    }
  }

  function computeCylinderInertia(mass: number, rm: number, hm: number) {
    const ixx = mass / 12 * (3 * rm * rm + hm * hm)
    return { ixx, iyy: ixx, izz: mass / 2 * rm * rm }
  }

  function computeSphereInertia(mass: number, rm: number) {
    const i = 2 / 5 * mass * rm * rm
    return { ixx: i, iyy: i, izz: i }
  }

  function getParentBounds(doc: Document, parentLinkName: string): { hx: number; hy: number; hz: number } {
    // Extract parent link's bounding half-extents from its URDF geometry
    const linkEl = doc.querySelector(`link[name="${parentLinkName}"]`)
    if (!linkEl) return { hx: 0.05, hy: 0.05, hz: 0.05 }

    const vis = linkEl.querySelector('visual geometry')
    if (!vis) return { hx: 0.05, hy: 0.05, hz: 0.05 }

    const boxEl = vis.querySelector('box')
    if (boxEl) {
      const size = (boxEl.getAttribute('size') || '0.1 0.1 0.1').split(/\s+/).map(Number)
      return { hx: (size[0] || 0.1) / 2, hy: (size[1] || 0.1) / 2, hz: (size[2] || 0.1) / 2 }
    }
    const cylEl = vis.querySelector('cylinder')
    if (cylEl) {
      const r = Number(cylEl.getAttribute('radius')) || 0.05
      const h = Number(cylEl.getAttribute('length')) || 0.1
      return { hx: r, hy: r, hz: h / 2 }
    }
    const sphEl = vis.querySelector('sphere')
    if (sphEl) {
      const r = Number(sphEl.getAttribute('radius')) || 0.05
      return { hx: r, hy: r, hz: r }
    }
    return { hx: 0.05, hy: 0.05, hz: 0.05 }
  }

  function computePlacement(
    doc: Document, parentLinkName: string,
    comp: PresetComponent,
    childX: number, _childY: number, childZ: number,
  ): { xyz: string; rpy: string } {
    const parent = getParentBounds(doc, parentLinkName)
    const mount = (comp.mounting_logic?.primary as string) || 'face_mount'
    const gap = 0.005 // 5mm clearance

    // face_mount / pcb_solder / bracket_mount → stack on top (Z+) of parent
    if (mount === 'face_mount' || mount === 'pcb_solder' || mount === 'bracket_mount') {
      const oz = parent.hz + childZ / 2 + gap
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }

    // axial_shaft → coaxial along Z, placed at parent's top face
    if (mount === 'axial_shaft') {
      const oz = parent.hz + childZ / 2 + gap
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }

    // rail_slot / side_rail_mount → mount on the side (X+) of parent
    if (mount === 'rail_slot' || mount === 'side_rail_mount' || mount === 'clamp_mount') {
      const ox = parent.hx + childX / 2 + gap
      return { xyz: `${ox.toFixed(4)} 0 0`, rpy: '0 0 0' }
    }

    // hub_bore → coaxial, flush with parent face
    if (mount === 'hub_bore') {
      const oz = parent.hz + childZ / 2 + gap
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }

    // press_fit → inside parent bore, centered
    if (mount === 'press_fit') {
      return { xyz: '0 0 0', rpy: '0 0 0' }
    }

    // linear_rod → extend along Z from parent
    if (mount === 'linear_rod') {
      const oz = parent.hz + childZ / 2 + gap
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }

    // tool_changer_master/slave → stack on bottom (Z-) if slave
    if (mount === 'tool_changer_slave') {
      const oz = -(parent.hz + childZ / 2 + gap)
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }
    if (mount === 'tool_changer_master') {
      const oz = parent.hz + childZ / 2 + gap
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }

    // Default: stack on top
    const oz = parent.hz + childZ / 2 + gap
    return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
  }

  // Resolve which category a component belongs to
  function findCategory(comp: PresetComponent): string {
    if (!presetData) return 'structural'
    for (const [catName, cat] of Object.entries(presetData.categories)) {
      if (cat.components.some(c => c.id === comp.id)) return catName
    }
    return 'structural'
  }

  /** Compute the AABB of a component's visual geometry descriptors.
   *  Cylinders are treated as axis-along-Z (matching the URDF parser's rotation.x = PI/2).
   *  Returns half-extents and center offset in the component link's local frame. */
  function computeCarryGhostBounds(comp: PresetComponent): { hx: number; hy: number; hz: number; cx: number; cy: number; cz: number } {
    const catName = findCategory(comp)
    const visuals = generateVisuals(comp as Parameters<typeof generateVisuals>[0], catName)
    let minX = Infinity, maxX = -Infinity
    let minY = Infinity, maxY = -Infinity
    let minZ = Infinity, maxZ = -Infinity
    for (const vis of visuals) {
      const [ox, oy, oz] = vis.origin_xyz
      const g = vis.geometry
      let ex = 0, ey = 0, ez = 0
      if (g.type === 'box') { ex = g.size[0] / 2; ey = g.size[1] / 2; ez = g.size[2] / 2 }
      else if (g.type === 'cylinder') { ex = g.radius; ey = g.radius; ez = g.length / 2 }
      else if (g.type === 'sphere') { ex = g.radius; ey = g.radius; ez = g.radius }
      minX = Math.min(minX, ox - ex); maxX = Math.max(maxX, ox + ex)
      minY = Math.min(minY, oy - ey); maxY = Math.max(maxY, oy + ey)
      minZ = Math.min(minZ, oz - ez); maxZ = Math.max(maxZ, oz + ez)
    }
    if (!isFinite(minX)) {
      const bb = comp.physical.bounding_box_mm ?? comp.physical.cross_section_mm ?? [40, 40, 40]
      return { hx: (bb[0] ?? 40) / 2000, hy: (bb[1] ?? 40) / 2000, hz: (bb[2] ?? 40) / 2000, cx: 0, cy: 0, cz: 0 }
    }
    return {
      hx: (maxX - minX) / 2,
      hy: (maxY - minY) / 2,
      hz: (maxZ - minZ) / 2,
      cx: (maxX + minX) / 2,
      cy: (maxY + minY) / 2,
      cz: (maxZ + minZ) / 2,
    }
  }

  function addVisualElement(doc: Document, link: Element, vis: UrdfVisualDesc, matIdx: number) {
    const visual = doc.createElement('visual')
    const vo = doc.createElement('origin')
    vo.setAttribute('xyz', vis.origin_xyz.map(v => v.toFixed(6)).join(' '))
    vo.setAttribute('rpy', vis.origin_rpy.map(v => v.toFixed(6)).join(' '))
    const geometry = doc.createElement('geometry')
    const g = vis.geometry
    if (g.type === 'box') {
      const el = doc.createElement('box')
      el.setAttribute('size', g.size.map(v => v.toFixed(6)).join(' '))
      geometry.appendChild(el)
    } else if (g.type === 'cylinder') {
      const el = doc.createElement('cylinder')
      el.setAttribute('radius', g.radius.toFixed(6))
      el.setAttribute('length', g.length.toFixed(6))
      geometry.appendChild(el)
    } else {
      const el = doc.createElement('sphere')
      el.setAttribute('radius', g.radius.toFixed(6))
      geometry.appendChild(el)
    }
    visual.appendChild(vo)
    visual.appendChild(geometry)
    // Material with colour
    const mat = doc.createElement('material')
    mat.setAttribute('name', `comp_mat_${matIdx}`)
    const color = doc.createElement('color')
    color.setAttribute('rgba', vis.color_rgba.map(v => v.toFixed(3)).join(' '))
    mat.appendChild(color)
    visual.appendChild(mat)
    link.appendChild(visual)
  }

  // Core URDF mutation shared by addComponent (heuristic) and addComponentWithSnap (exact pose).
  function addComponentCore(
    comp: PresetComponent,
    parentLink: string,
    xyzStr: string,
    rpyStr: string,
  ): boolean {
    const graph = ctx.getKinematicGraph()
    const nextIdx = Object.keys(graph).length + 1
    const childName = `${comp.id}_${nextIdx}`
    const jointName = `joint_${comp.id}_${nextIdx}`

    const phys = comp.physical
    const mass = phys.mass_kg ?? phys.mass_kg_per_100mm ?? 0.1
    const bb = phys.bounding_box_mm ?? phys.cross_section_mm ?? [40, 40, 40]
    const shape = phys.inertia_primitive || 'box'
    const xm = (bb[0] ?? 40) / 1000
    const ym = (bb[1] ?? 40) / 1000
    const zm = (bb[2] ?? 40) / 1000

    let inertia: { ixx: number; iyy: number; izz: number }
    if (shape === 'cylinder') {
      inertia = computeCylinderInertia(mass, Math.max(xm, ym) / 2, zm)
    } else if (shape === 'sphere') {
      inertia = computeSphereInertia(mass, xm / 2)
    } else {
      inertia = computeBoxInertia(mass, xm, ym, zm)
    }

    const catName = findCategory(comp)
    const visuals = generateVisuals(comp as Parameters<typeof generateVisuals>[0], catName)

    const changed = commitUrdf(doc => {
      const robot = doc.documentElement
      if (!robot || robot.nodeName !== 'robot') return false

      const link = doc.createElement('link')
      link.setAttribute('name', childName)

      const inertialEl = doc.createElement('inertial')
      const massEl = doc.createElement('mass')
      massEl.setAttribute('value', mass.toFixed(4))
      const inertiaEl = doc.createElement('inertia')
      inertiaEl.setAttribute('ixx', inertia.ixx.toFixed(6))
      inertiaEl.setAttribute('iyy', inertia.iyy.toFixed(6))
      inertiaEl.setAttribute('izz', inertia.izz.toFixed(6))
      inertiaEl.setAttribute('ixy', '0'); inertiaEl.setAttribute('ixz', '0'); inertiaEl.setAttribute('iyz', '0')
      inertialEl.appendChild(massEl); inertialEl.appendChild(inertiaEl)
      link.appendChild(inertialEl)

      visuals.forEach((vis, i) => addVisualElement(doc, link, vis, i))

      const collision = doc.createElement('collision')
      const co = doc.createElement('origin')
      co.setAttribute('xyz', '0 0 0'); co.setAttribute('rpy', '0 0 0')
      const collGeom = doc.createElement('geometry')
      if (shape === 'cylinder') {
        const el = doc.createElement('cylinder')
        el.setAttribute('radius', (Math.max(xm, ym) / 2).toFixed(6))
        el.setAttribute('length', zm.toFixed(6))
        collGeom.appendChild(el)
      } else if (shape === 'sphere') {
        const el = doc.createElement('sphere')
        el.setAttribute('radius', (Math.max(xm, ym, zm) / 2).toFixed(6))
        collGeom.appendChild(el)
      } else {
        const el = doc.createElement('box')
        el.setAttribute('size', `${xm.toFixed(6)} ${ym.toFixed(6)} ${zm.toFixed(6)}`)
        collGeom.appendChild(el)
      }
      collision.appendChild(co); collision.appendChild(collGeom)
      link.appendChild(collision)

      const joint = doc.createElement('joint')
      joint.setAttribute('name', jointName)
      const category = comp.id.split('_')[0]
      const isActuated = category === 'actuator' || category === 'motor'
      joint.setAttribute('type', isActuated ? 'revolute' : 'fixed')

      const parentEl = doc.createElement('parent'); parentEl.setAttribute('link', parentLink)
      const childEl = doc.createElement('child'); childEl.setAttribute('link', childName)
      const origin = doc.createElement('origin')
      origin.setAttribute('xyz', xyzStr); origin.setAttribute('rpy', rpyStr)
      joint.appendChild(parentEl); joint.appendChild(childEl); joint.appendChild(origin)

      if (isActuated) {
        const axis = doc.createElement('axis'); axis.setAttribute('xyz', '0 0 1')
        joint.appendChild(axis)
        const limit = doc.createElement('limit')
        limit.setAttribute('lower', '-3.14159'); limit.setAttribute('upper', '3.14159')
        const maxTorque = (comp.mechanical_electrical.max_torque_nm as number) ??
                          (comp.mechanical_electrical.holding_torque_nm as number) ?? 10
        limit.setAttribute('effort', String(maxTorque)); limit.setAttribute('velocity', '3.14')
        joint.appendChild(limit)
      }
      robot.appendChild(link); robot.appendChild(joint)
      return true
    }, { defer: true })

    if (changed) {
      ctx.showToast(`Added ${comp.name} as "${childName}"`, 'success')
      selectLink(childName)
    }
    return changed
  }

  function addComponent(comp: PresetComponent) {
    if (!selectedLink) {
      ctx.showToast('Select a parent link first, or drag from toolbox to place', 'warning')
      return
    }
    const parentLink = selectedLink
    const phys = comp.physical
    const bb = phys.bounding_box_mm ?? phys.cross_section_mm ?? [40, 40, 40]
    const xm = (bb[0] ?? 40) / 1000
    const ym = (bb[1] ?? 40) / 1000
    const zm = (bb[2] ?? 40) / 1000
    const doc = new DOMParser().parseFromString(ctx.getUrdfText(), 'application/xml')
    const placement = computePlacement(doc, parentLink, comp, xm, ym, zm)
    addComponentCore(comp, parentLink, placement.xyz, placement.rpy)
  }

  // ── Carry mode ───────────────────────────────────────────────────────────────

  let carryComp: PresetComponent | null = null
  let carryGroup: THREE.Group | null = null
  let carryGhostBounds: { hx: number; hy: number; hz: number; cx: number; cy: number; cz: number } | null = null
  let carryWorldPos = new THREE.Vector3()
  let carryFrozen = false          // true after manual nudge — mouse no longer drives position
  let carryRotStep = 0             // 0..3 → 0°, 90°X, 90°Y, 90°Z
  interface SnapCandidate {
    mountLink: string
    targetParentLink: string
    dist: number
    desiredGhostWorld: THREE.Matrix4
    srcNodeId: string
    targetNodeId: string
  }
  let carrySnapCandidates: SnapCandidate[] = []
  let carrySnapIdx = 0
  let carryBestMount: {
    targetParentLink: string
    mountLink: string
    desiredGhostWorld: THREE.Matrix4
  } | null = null

  const carryGhostMat = new THREE.MeshBasicMaterial({
    color: 0x44aaff, transparent: true, opacity: 0.35, depthTest: true, side: THREE.DoubleSide,
  })
  const carryEdgeMat = new THREE.LineBasicMaterial({ color: 0x88ccff, transparent: true, opacity: 0.75 })
  const carryGroundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0)

  // Carry HUD
  const carryHud = document.createElement('div')
  carryHud.id = 'carry-hud'
  carryHud.style.cssText = `
    position:absolute; bottom:48px; left:50%; transform:translateX(-50%);
    background:rgba(0,0,0,0.72); color:#d4d4d4; font-size:12px;
    padding:6px 14px; border-radius:6px; pointer-events:none;
    display:none; white-space:nowrap; z-index:100;
    border:1px solid rgba(255,255,255,0.12);
  `
  ctx.canvas.parentElement?.appendChild(carryHud)

  function setCarryHud(msg: string) { carryHud.style.display = msg ? 'block' : 'none'; carryHud.textContent = msg }

  function carryRotQuat(): THREE.Quaternion {
    const q = new THREE.Quaternion()
    if (carryRotStep === 1) q.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2)
    else if (carryRotStep === 2) q.setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI / 2)
    else if (carryRotStep === 3) q.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2)
    return q
  }

  function updateCarryHudText() {
    if (!carryComp || !carryGroup) return
    const p = carryGroup.position
    const rotLabel = ['0°', '+90°X', '+90°Y', '+90°Z'][carryRotStep]
    let snapHint = ''
    if (carryBestMount) {
      const n = carrySnapCandidates.length
      const idxLabel = n > 1 ? ` [${carrySnapIdx + 1}/${n}]` : ''
      snapHint = `Snap→${carryBestMount.targetParentLink}${idxLabel}  `
    }
    const tabHint = carrySnapCandidates.length > 1 ? '  Tab=cycle' : ''
    setCarryHud(
      `${snapHint}${carryComp.name}  x:${p.x.toFixed(3)} y:${p.y.toFixed(3)} z:${p.z.toFixed(3)}  rot:${rotLabel}` +
      `${tabHint}  ←→↑↓ nudge  Shift=10×  R rotate  Enter commit  Esc cancel`
    )
  }

  function enterCarryMode(comp: PresetComponent) {
    if (carryGroup) exitCarryMode()
    carryComp = comp
    carryFrozen = false
    carryRotStep = 0
    carrySnapCandidates = []
    carrySnapIdx = 0

    const bounds = computeCarryGhostBounds(comp)
    carryGhostBounds = bounds
    const { hx, hy, hz, cx, cy, cz } = bounds

    const geo = new THREE.BoxGeometry(hx * 2, hy * 2, hz * 2)
    const mesh = new THREE.Mesh(geo, carryGhostMat)
    mesh.position.set(cx, cy, cz)
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo), carryEdgeMat)
    edges.position.set(cx, cy, cz)
    carryGroup = new THREE.Group()
    carryGroup.name = 'carry_ghost'
    carryGroup.add(mesh, edges)
    ctx.scene.add(carryGroup)

    rebuildMountNodes()
    nodesGroup.visible = true
    applyNodeRingVisibility()
    setCarryHud(`Placing ${comp.name} — hover to snap, click or Enter to commit, Esc to cancel`)
    requestAnimationFrame(() => {
      if (!carryComp || !carryGroup) return
      const r = ctx.canvas.getBoundingClientRect()
      updateCarryFromMouse({
        clientX: r.left + r.width * 0.5,
        clientY: r.top + r.height * 0.5,
      } as MouseEvent)
    })
  }

  function exitCarryMode() {
    if (carryGroup) { ctx.scene.remove(carryGroup); carryGroup = null }
    carryComp = null
    carryGhostBounds = null
    carryBestMount = null
    carryFrozen = false
    carryRotStep = 0
    carrySnapCandidates = []
    carrySnapIdx = 0
    nodesGroup.visible = false
    ghostGroup.visible = false
    clearBestCandidateHighlight()
    applyNodeRingVisibility()
    setCarryHud('')
    ctx.canvas.style.cursor = ''
  }

  function getCarrySourceNodes() {
    if (!carryComp || !carryGroup) return []
    const { hx, hy, hz, cx, cy, cz } = carryGhostBounds ?? computeCarryGhostBounds(carryComp)
    carryGroup.updateMatrixWorld(true)
    return defaultFaceNodesForBoxDims(hx, hy, hz).map(f => {
      // Face positions are relative to carry group origin — include the visual center offset
      const lx = cx + f.origin_xyz[0]
      const ly = cy + f.origin_xyz[1]
      const lz = cz + f.origin_xyz[2]
      const localFacePos = new THREE.Vector3(lx, ly, lz)
      const worldPos = localFacePos.clone().applyMatrix4(carryGroup!.matrixWorld)
      const worldQuat = new THREE.Quaternion().setFromRotationMatrix(carryGroup!.matrixWorld)
      const localToGhost = new THREE.Matrix4().makeTranslation(lx, ly, lz)
      return { nodeId: f.nodeId, cls: f.cls, worldPos, worldQuat, localToGhost }
    })
  }

  function updateCarrySnap() {
    if (!carryComp || !carryGroup) return
    refreshNodeWorldTransforms()
    const sourceNodes = getCarrySourceNodes()

    // Collect all valid candidates, sorted closest-first
    const candidates: SnapCandidate[] = []
    for (const src of sourceNodes) {
      _snapScratchInv.copy(src.localToGhost).invert()
      for (const target of mountNodes) {
        if (isMountOccupied(target.mountLink)) continue
        const dist = target.worldPosition.distanceTo(src.worldPos)
        if (dist > SNAP_RADIUS_M) continue
        if (!nodesCompatible(src.cls, target.cls)) continue
        // No angle check for carry mode — the ghost can be freely rotated with R key.
        _snapScratchMat.compose(target.worldPosition, target.worldQuaternion, _snapScratchScale)
        const desiredGhostWorld = _snapScratchMat.clone().multiply(_snapScratchInv)
        candidates.push({ mountLink: target.mountLink, targetParentLink: target.parentLink, dist, desiredGhostWorld, srcNodeId: src.nodeId, targetNodeId: target.nodeId })
      }
    }
    candidates.sort((a, b) => a.dist - b.dist)

    // Update candidate list; preserve idx when candidates haven't changed
    const prevCount = carrySnapCandidates.length
    carrySnapCandidates = candidates
    if (carrySnapIdx >= candidates.length) carrySnapIdx = 0
    if (!carryFrozen) {
      // Auto-mode: always use best (idx 0), allow idx reset if count changed
      if (prevCount !== candidates.length) carrySnapIdx = 0
    }

    clearBestCandidateHighlight()
    for (const t of mountNodes) setNodeMeshState(t.mountLink, isMountOccupied(t.mountLink) ? 'occupied' : 'neutral')

    if (candidates.length === 0) {
      carryBestMount = null
      updateCarryHudText()
      return
    }

    const chosen = candidates[carrySnapIdx]
    setNodeMeshState(chosen.mountLink, 'best')
    carryBestMount = { targetParentLink: chosen.targetParentLink, mountLink: chosen.mountLink, desiredGhostWorld: chosen.desiredGhostWorld }

    // Move ghost only when not frozen (manual nudge/rotate beats solver)
    if (!carryFrozen) {
      const snapPos = new THREE.Vector3(); const snapQuat = new THREE.Quaternion()
      chosen.desiredGhostWorld.decompose(snapPos, snapQuat, new THREE.Vector3())
      carryGroup.position.copy(snapPos); carryGroup.quaternion.copy(snapQuat)
      carryGroup.updateMatrixWorld(true)
    }
    updateCarryHudText()
  }

  function updateCarryFromMouse(e: MouseEvent) {
    if (!carryComp || !carryGroup) return
    if (!carryFrozen) {
      const ray = makeRaycaster(e)
      const hits = ray.intersectObjects(getPickTargets(), false)
      if (hits.length > 0) {
        carryWorldPos.copy(hits[0].point)
      } else {
        const planeHit = new THREE.Vector3()
        if (ray.ray.intersectPlane(carryGroundPlane, planeHit)) carryWorldPos.copy(planeHit)
      }
      carryGroup.position.copy(carryWorldPos)
      carryGroup.quaternion.copy(carryRotQuat())
      carryGroup.updateMatrixWorld(true)

      // Clamp ghost so its AABB bottom never clips below the floor (Y=0).
      // The hit point is the surface the cursor is over; the ghost center is placed
      // there, so without a lift the lower half always goes underground.
      const { hx, hy, hz, cx, cy, cz } = carryGhostBounds ?? computeCarryGhostBounds(carryComp)
      const lifted = clampCarryMatrixAboveFloor(carryGroup.matrixWorld, hx, hy, hz, cx, cy, cz)
      carryGroup.position.setFromMatrixPosition(lifted)
      carryGroup.updateMatrixWorld(true)

      carrySnapIdx = 0  // reset to auto-best when mouse drives
      updateCarrySnap()
    }
    ctx.canvas.style.cursor = 'crosshair'
    updateCarryHudText()
  }

  function commitCarry() {
    if (!carryComp) return
    const comp = carryComp
    const mount = carryBestMount
    const ghostWorldFree = !mount && carryGroup ? carryGroup.matrixWorld.clone() : null
    exitCarryMode()

    if (mount) {
      const parentLinkGroup = ctx.getParsedRobot().linkGroups.get(mount.targetParentLink)
      if (!parentLinkGroup) {
        ctx.showToast(
          `Could not attach: 3D group missing for parent link "${mount.targetParentLink}" (try reparse or reload)`,
          'error',
        )
        return
      }
      parentLinkGroup.updateMatrixWorld(true)
      const parentWorldInv = parentLinkGroup.matrixWorld.clone().invert()
      const childLocal = parentWorldInv.clone().multiply(mount.desiredGhostWorld.clone())
      const localPos = new THREE.Vector3().setFromMatrixPosition(childLocal)
      const localQuat = new THREE.Quaternion()
      childLocal.decompose(new THREE.Vector3(), localQuat, new THREE.Vector3())
      const localEuler = new THREE.Euler().setFromQuaternion(localQuat, 'XYZ')
      addComponentCore(comp, mount.targetParentLink,
        `${fmt(localPos.x)} ${fmt(localPos.y)} ${fmt(localPos.z)}`,
        `${fmt(localEuler.x)} ${fmt(localEuler.y)} ${fmt(localEuler.z)}`)
    } else if (ghostWorldFree) {
      // Free-space: joint pose from ghost in world → parent link frame (carry mode does not depend on selection)
      const graph = ctx.getKinematicGraph()
      const parent = resolveFreePlacementParent(graph)
      const parentLinkGroup = ctx.getParsedRobot().linkGroups.get(parent)
      if (!parentLinkGroup) {
        ctx.showToast(
          `Could not attach: 3D group missing for parent link "${parent}" (try reparse or reload)`,
          'error',
        )
        return
      }
      const { hx: ghx, hy: ghy, hz: ghz, cx: gcx, cy: gcy, cz: gcz } = computeCarryGhostBounds(comp)
      const ghostAdjusted = clampCarryMatrixAboveFloor(ghostWorldFree, ghx, ghy, ghz, gcx, gcy, gcz)
      parentLinkGroup.updateMatrixWorld(true)
      const parentWorldInv = parentLinkGroup.matrixWorld.clone().invert()
      const childLocal = parentWorldInv.clone().multiply(ghostAdjusted)
      const localPos = new THREE.Vector3().setFromMatrixPosition(childLocal)
      const localQuat = new THREE.Quaternion()
      childLocal.decompose(new THREE.Vector3(), localQuat, new THREE.Vector3())
      const localEuler = new THREE.Euler().setFromQuaternion(localQuat, 'XYZ')
      addComponentCore(comp, parent,
        `${fmt(localPos.x)} ${fmt(localPos.y)} ${fmt(localPos.z)}`,
        `${fmt(localEuler.x)} ${fmt(localEuler.y)} ${fmt(localEuler.z)}`)
    } else {
      const graph = ctx.getKinematicGraph()
      const parent = resolveFreePlacementParent(graph)
      const phys = comp.physical
      const bb = phys.bounding_box_mm ?? phys.cross_section_mm ?? [40, 40, 40]
      const xm = (bb[0] ?? 40) / 1000
      const ym = (bb[1] ?? 40) / 1000
      const zm = (bb[2] ?? 40) / 1000
      const doc = new DOMParser().parseFromString(ctx.getUrdfText(), 'application/xml')
      const placement = computePlacement(doc, parent, comp, xm, ym, zm)
      addComponentCore(comp, parent, placement.xyz, placement.rpy)
    }
  }

  function getCompactSpec(comp: PresetComponent): string {
    const me = comp.mechanical_electrical
    if (me.max_torque_nm) return `${me.max_torque_nm} Nm`
    if (me.holding_torque_nm) return `${me.holding_torque_nm} Nm`
    if (me.max_force_n) return `${me.max_force_n} N`
    if (me.grip_force_n) return `${me.grip_force_n} N`
    if (me.fov_h_deg) return `${me.fov_h_deg}° FOV`
    if (me.range_m) return `${me.range_m}m range`
    if (me.capacity_mah) return `${me.capacity_mah}mAh`
    if (me.gear_ratio_options) return 'gearbox'
    if (me.max_load_n) return `${me.max_load_n}N`
    const mass = comp.physical.mass_kg ?? comp.physical.mass_kg_per_100mm
    if (mass != null) return `${mass >= 1 ? mass.toFixed(1) : Math.round(mass * 1000)}${mass >= 1 ? 'kg' : 'g'}`
    return ''
  }

  function renderComponentDetail(comp: PresetComponent) {
    if (!compDetail) return
    const mass = comp.physical.mass_kg ?? comp.physical.mass_kg_per_100mm
    const massLabel = comp.physical.mass_kg_per_100mm ? `${(comp.physical.mass_kg_per_100mm * 1000).toFixed(0)}g/100mm` :
                      mass != null ? (mass >= 1 ? `${mass.toFixed(2)} kg` : `${Math.round(mass * 1000)} g`) : '—'
    const bb = comp.physical.bounding_box_mm
    const dims = bb ? `${bb[0]}×${bb[1]}×${bb[2]} mm` : '—'
    const shape = comp.physical.inertia_primitive || 'box'

    // Build mechanical specs
    const me = comp.mechanical_electrical
    const specs = Object.entries(me).slice(0, 6).map(([k, v]) => {
      const label = k.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase())
      const val = Array.isArray(v) ? v.join(' – ') : String(v)
      return `<div class="tb-kv"><span class="tb-kv-key">${label}</span><span class="tb-kv-val">${val}</span></div>`
    }).join('')

    const mounting = comp.mounting_logic.primary ?? '—'

    compDetail.innerHTML = `
      <div class="tb-detail-name">${comp.name}</div>
      <div class="tb-detail-desc">${comp.description}</div>
      <div class="tb-detail-chiprow">
        <span class="tb-chip">${shape}</span>
        <span class="tb-chip">${massLabel}</span>
        <span class="tb-chip">${mounting}</span>
      </div>
      <div class="tb-detail-sec">
        <div class="tb-detail-sec-title">Dimensions</div>
        <div class="tb-kv"><span class="tb-kv-key">Bounding Box</span><span class="tb-kv-val">${dims}</span></div>
      </div>
      <div class="tb-detail-sec">
        <div class="tb-detail-sec-title">Specs</div>
        ${specs}
      </div>
    `
  }

  function renderComponents(filter: string) {
    if (!compItems || !presetData) return
    const q = filter.trim().toLowerCase()
    compItems.innerHTML = ''

    for (const [catName, cat] of Object.entries(presetData.categories)) {
      const comps = cat.components.filter(c => {
        // Only show components that have real mesh files
        if (!hasMeshOverride(c.id)) return false
        // Exclude components whose meshes are too large/slow to load
        if (SLOW_MESH_BLACKLIST.has(c.id)) return false
        // Apply search filter
        return !q || c.name.toLowerCase().includes(q) || c.id.toLowerCase().includes(q) || c.description.toLowerCase().includes(q)
      })
      if (comps.length === 0) continue

      // Category header
      const catLabel = catName.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase())
      const catEl = document.createElement('div')
      catEl.className = 'tb-cat'
      catEl.innerHTML = `<span class="tb-cat-arrow">▾</span> ${catLabel} <span style="opacity:0.4;margin-left:auto;font-size:10px">${comps.length}</span>`
      let collapsed = false
      catEl.addEventListener('click', () => {
        collapsed = !collapsed
        listEl.classList.toggle('collapsed', collapsed)
        catEl.querySelector('.tb-cat-arrow')!.textContent = collapsed ? '▸' : '▾'
      })
      compItems.appendChild(catEl)

      const listEl = document.createElement('div')
      listEl.className = 'tb-list'
      const cc = CATEGORY_COLORS[catName] ?? [0.6, 0.6, 0.6, 1]
      const dotColor = `rgb(${Math.round(cc[0]*255)},${Math.round(cc[1]*255)},${Math.round(cc[2]*255)})`
      for (const comp of comps) {
        const el = document.createElement('div')
        el.className = 'tb-item'
        const spec = getCompactSpec(comp)
        el.innerHTML = `
          <span class="tb-cat-dot" style="background:${dotColor}"></span>
          <div class="tb-item-info">
            <div class="tb-item-name">${comp.name}</div>
            <div class="tb-item-meta">${spec}</div>
          </div>
        `
        el.addEventListener('click', () => {
          if (ctx.getInteractionMode() === 'inspect') {
            ctx.showToast('Switch to Build mode to place components', 'info')
            return
          }
          // Show detail
          renderComponentDetail(comp)
          // Highlight
          compItems!.querySelectorAll('.tb-item').forEach(i => i.classList.remove('selected'))
          el.classList.add('selected')
          // Enter carry mode — ghost follows mouse until committed
          enterCarryMode(comp)
        })
        listEl.appendChild(el)
      }
      compItems.appendChild(listEl)
    }

    if (compItems.children.length === 0) {
      compItems.innerHTML = '<div class="tb-empty">No matching components</div>'
    }
  }

  // Load presets JSON from public directory
  fetch('/generic_presets.json')
    .then(r => r.ok ? r.json() : Promise.reject(r.status))
    .then((data: PresetData) => {
      presetData = data
      renderComponents('')
    })
    .catch(() => {
      if (compItems) compItems.innerHTML = '<div class="tb-empty">Failed to load component presets</div>'
    })

  btnFocusBase?.addEventListener('click', () => {
    const graph = ctx.getKinematicGraph()
    const base = Object.values(graph).find(l => !l.parent)?.name || 'base_link'
    selectLink(base)
  })
  btnResetRobot?.addEventListener('click', () => {
    if (!confirm('Reset robot to a minimal base_link URDF?')) return
    recordUndo()
    const empty = `<?xml version="1.0"?><robot name="robot"><link name="base_link"><inertial><mass value="0.1"/><inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/></inertial><visual><geometry><box size="0.1 0.1 0.05"/></geometry></visual></link></robot>`
    ctx.zeroAssemblyWorldPosition?.()
    ctx.setUrdfText(empty)
    ctx.reparseUrdf()
    ctx.groundAssembly?.()
    selectLink('base_link')
    ctx.showToast('Robot reset', 'info')
  })
  // ── STL Export ─────────────────────────────────────────────────────────────

  const stlExporter = new STLExporter()

  async function saveStlToFile(group: THREE.Object3D, defaultName: string) {
    try {
      const path = await invoke<string | null>('save_file_dialog', { default_name: defaultName })
      if (!path) return

      // Export as ASCII STL (text-based, works with save_file)
      const stlString = stlExporter.parse(group, { binary: false }) as string
      await invoke('save_file', { path, content: stlString })

      const filename = path.split(/[\\/]/).pop() || defaultName
      ctx.showToast(`Exported ${filename}`, 'success')
    } catch (err) {
      ctx.showToast(`Export failed: ${err}`, 'error')
    }
  }

  async function exportSelectedLinkSTL() {
    if (!selectedLink) {
      ctx.showToast('Select a link to export', 'warning')
      return
    }
    const linkGroup = ctx.getParsedRobot().linkGroups.get(selectedLink)
    if (!linkGroup) {
      ctx.showToast('Link geometry not found', 'error')
      return
    }
    await saveStlToFile(linkGroup, `${selectedLink}.stl`)
  }

  async function exportFullRobotSTL() {
    const robot = ctx.getParsedRobot()
    await saveStlToFile(robot.group, 'robot.stl')
  }

  async function exportUrdfPackage() {
    const urdf = ctx.getUrdfText().trim()
    if (!urdf) { ctx.showToast('Nothing to export — URDF is empty', 'warning'); return }

    const folder = await invoke<string | null>('open_folder_dialog')
    if (!folder) return

    // Derive robot name from <robot name="..."> or fallback
    const nameMatch = urdf.match(/<robot[^>]+name="([^"]+)"/)
    const robotName = nameMatch ? nameMatch[1] : 'robot'

    // Write URDF
    const urdfPath = `${folder}/${robotName}.urdf`
    await invoke('save_file', { path: urdfPath, content: urdf })

    // Write minimal package.xml (ROS 2 format)
    const pkgXml = `<?xml version="1.0"?>
<package format="3">
  <name>${robotName}</name>
  <version>0.0.1</version>
  <description>${robotName} robot description</description>
  <maintainer email="robot@example.com">Vector Builder</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
`
    await invoke('save_file', { path: `${folder}/package.xml`, content: pkgXml })

    ctx.showToast(`Exported URDF package to ${folder}`, 'success')
  }

  // Wire export buttons
  const btnExportStl = document.getElementById('btn-export-stl')
  btnExportStl?.addEventListener('click', exportFullRobotSTL)
  const btnExportPkg = document.getElementById('btn-export-urdf-pkg')
  btnExportPkg?.addEventListener('click', () => { void exportUrdfPackage() })
  toolboxSearch?.addEventListener('input', () => renderComponents(toolboxSearch.value))

  toggleMountRingsBtn?.addEventListener('click', () => {
    showNodeRings = !showNodeRings
    toggleMountRingsBtn.classList.toggle('active', showNodeRings)
    applyNodeRingVisibility()
  })

  gizmo.addEventListener('dragging-changed', ev => {
    const on = Boolean((ev as unknown as { value: boolean }).value)
    ctx.controls.enabled = !on
    if (on && selectedLink) {
      const pivot = getPivotGroupForLink(selectedLink)
      if (pivot) {
        pivot.updateMatrixWorld(true)
        gizmoBasePivotWorld.copy(pivot.matrixWorld)
        rebuildMountNodes()
        nodesGroup.visible = true
        lastSnapCheckMs = 0  // ensure first drag frame runs a snap check immediately
        updateBestCandidateDuringDrag(pivot)
      }
      return
    }
    if (!on && selectedLink) {
      // Drag ended — hide all connection nodes regardless of snap outcome.
      nodesGroup.visible = false
      const parentJoint = getParentJointForLink(selectedLink)
      const pivot = getPivotGroupForLink(selectedLink)
      if (!parentJoint || !pivot) {
        if (!rootDragWarned) {
          ctx.showToast('Root movement disabled (no world joint yet)', 'warning')
          rootDragWarned = true
        }
        selectLink(selectedLink)
        return
      }

      // If we had a valid snap candidate, commit as a fixed joint reparent onto the target link.
      if (bestMountCandidate?.mountLink && bestMountCandidate.reason === 'ok') {
        const targetMount = bestMountCandidate.mountLink
        const targetParent = bestMountCandidate.targetParentLink
        const targetOccupied = isMountOccupied(targetMount)
        if (targetOccupied) {
          ctx.showToast('Snap target is occupied', 'warning')
          bestMountCandidate = null
          ghostGroup.visible = false
          clearBestCandidateHighlight()
          selectLink(selectedLink)
          return
        }
        const graph = ctx.getKinematicGraph()
        // topology / cycle check: don't attach into own subtree
        const stack = [selectedLink]
        const seen = new Set<string>()
        let cycle = false
        while (stack.length) {
          const cur = stack.pop()!
          if (cur === targetParent) { cycle = true; break }
          if (seen.has(cur)) continue
          seen.add(cur)
          for (const ch of graph[cur]?.children ?? []) stack.push(ch)
        }
        if (cycle) {
          ctx.showToast('Invalid snap (would create cycle)', 'warning')
          bestMountCandidate = null
          ghostGroup.visible = false
          clearBestCandidateHighlight()
          selectLink(selectedLink)
          return
        }

        pivot.updateMatrixWorld(true)
        const parentLinkGroup = ctx.getParsedRobot().linkGroups.get(targetParent)
        if (!parentLinkGroup) {
          ctx.showToast('Snap target not found in scene', 'warning')
          bestMountCandidate = null
          ghostGroup.visible = false
          clearBestCandidateHighlight()
          selectLink(selectedLink)
          return
        }
        parentLinkGroup.updateMatrixWorld(true)
        const parentWorldInv = parentLinkGroup.matrixWorld.clone().invert()
        // Compute selected-link frame pose in target parent frame from solved preview transform.
        const childLocalInParent = parentWorldInv.multiply(bestMountCandidate.desiredLinkWorld.clone())
        const newLocalPos = new THREE.Vector3().setFromMatrixPosition(childLocalInParent)
        const newLocalQuat = new THREE.Quaternion()
        childLocalInParent.decompose(new THREE.Vector3(), newLocalQuat, new THREE.Vector3())
        const newLocalEuler = new THREE.Euler().setFromQuaternion(newLocalQuat, 'XYZ')

        // Capture old pivot world orientation before commitUrdf triggers a reparse.
        const oldPivotWorldQuat = new THREE.Quaternion()
        pivot.matrixWorld.decompose(new THREE.Vector3(), oldPivotWorldQuat, new THREE.Vector3())
        const parentWorldQuat = new THREE.Quaternion()
        parentLinkGroup.matrixWorld.decompose(new THREE.Vector3(), parentWorldQuat, new THREE.Vector3())

        const ok = commitUrdf(documentXml => {
          const jointEl = documentXml.querySelector(`joint[name="${parentJoint.name}"]`)
          if (!jointEl) return false
          // Preserve the existing joint type (revolute, prismatic, fixed, etc.)
          // so snapping a motor/actuator keeps it actuated.
          const pEl = jointEl.querySelector('parent')
          if (!pEl) return false
          pEl.setAttribute('link', targetParent)
          const origin = ensureOrigin(jointEl, documentXml)
          origin.setAttribute('xyz', `${fmt(newLocalPos.x)} ${fmt(newLocalPos.y)} ${fmt(newLocalPos.z)}`)
          origin.setAttribute('rpy', `${fmt(newLocalEuler.x)} ${fmt(newLocalEuler.y)} ${fmt(newLocalEuler.z)}`)


          // Reconcile joint axis for revolute / prismatic / continuous joints.
          // <axis xyz> is expressed in the joint frame. When we reparent and reorient
          // the joint we must re-express the same physical world-space axis in the
          // new joint frame, otherwise the motor spins around the wrong direction.
          const jType = jointEl.getAttribute('type') || 'fixed'
          if (jType === 'revolute' || jType === 'prismatic' || jType === 'continuous') {
            const axisEl = jointEl.querySelector('axis')
            const axisStr = axisEl?.getAttribute('xyz') || '0 0 1'
            const axisArr = axisStr.trim().split(/\s+/).map(Number)
            const axisInOldJoint = new THREE.Vector3(
              Number.isFinite(axisArr[0]) ? axisArr[0] : 0,
              Number.isFinite(axisArr[1]) ? axisArr[1] : 0,
              Number.isFinite(axisArr[2]) ? axisArr[2] : 1,
            ).normalize()

            // 1. Lift axis into world space using the old joint frame orientation.
            const axisWorld = axisInOldJoint.clone().applyQuaternion(oldPivotWorldQuat)

            // 2. New joint world orientation = new parent world quat × new local quat.
            const newJointWorldQuat = parentWorldQuat.clone().multiply(newLocalQuat)

            // 3. Pull axis back down into the new joint frame.
            const newAxisInJoint = axisWorld.clone()
              .applyQuaternion(newJointWorldQuat.clone().invert())
              .normalize()

            const axisElToWrite = axisEl ?? documentXml.createElement('axis')
            axisElToWrite.setAttribute('xyz',
              `${fmt(newAxisInJoint.x)} ${fmt(newAxisInJoint.y)} ${fmt(newAxisInJoint.z)}`)
            if (!axisEl) jointEl.appendChild(axisElToWrite)
          }

          return true
        }, { defer: true })
        bestMountCandidate = null
        ghostGroup.visible = false
        clearBestCandidateHighlight()
        if (ok) ctx.showToast(`Connected ${selectedLink} to ${targetParent}`, 'success')
        selectLink(selectedLink)
        return
      }
      if (bestMountCandidate?.reason && bestMountCandidate.reason !== 'ok') {
        const reasonLabel = bestMountCandidate.reason === 'occupied' ? 'target occupied'
          : bestMountCandidate.reason === 'cycle' ? 'would create cycle'
          : bestMountCandidate.reason === 'too-far' ? 'too far'
          : bestMountCandidate.reason === 'orientation' ? 'orientation mismatch'
          : bestMountCandidate.reason === 'type-mismatch' ? 'incompatible node types'
          : bestMountCandidate.reason === 'same-component' ? 'same component'
          : bestMountCandidate.reason
        ctx.showToast(`Snap rejected: ${reasonLabel}`, 'warning')
      }
      ghostGroup.visible = false
      clearBestCandidateHighlight()

      if (gizmo.mode !== 'translate') {
        ctx.showToast('Rotate persistence is not enabled yet', 'info')
        selectLink(selectedLink)
        return
      }

      pivot.updateMatrixWorld(true)
      const parentObj = pivot.parent
      if (!parentObj) {
        selectLink(selectedLink)
        return
      }
      parentObj.updateMatrixWorld(true)

      const parentWorldInv = parentObj.matrixWorld.clone().invert()
      const childLocal = parentWorldInv.multiply(pivot.matrixWorld.clone())

      const newLocalPos = new THREE.Vector3().setFromMatrixPosition(childLocal)

      const parser = new DOMParser()
      const doc = parser.parseFromString(ctx.getUrdfText(), 'application/xml')
      if (doc.documentElement.nodeName === 'parsererror') {
        selectLink(selectedLink)
        return
      }
      const j = doc.querySelector(`joint[name="${parentJoint.name}"]`)
      if (!j) {
        selectLink(selectedLink)
        return
      }
      const o = ensureOrigin(j, doc)
      const oldLocalPos = parseNums(o.getAttribute('xyz') || '0 0 0', 3)
      const oldPosVec = new THREE.Vector3(oldLocalPos[0], oldLocalPos[1], oldLocalPos[2])
      if (oldPosVec.distanceToSquared(newLocalPos) < 1e-12) {
        selectLink(selectedLink)
        return
      }

      const ok = commitUrdf(documentXml => {
        const jointEl = documentXml.querySelector(`joint[name="${parentJoint.name}"]`)
        if (!jointEl) return false
        const origin = ensureOrigin(jointEl, documentXml)
        origin.setAttribute('xyz', `${fmt(newLocalPos.x)} ${fmt(newLocalPos.y)} ${fmt(newLocalPos.z)}`)
        return true
      }, { defer: true })
      if (ok) ctx.showToast(`Moved ${selectedLink} (joint origin updated)`, 'success')
      selectLink(selectedLink)
    }
  })

  gizmo.addEventListener('change', () => {
    if (!selectedLink) return
    if (!ctx.isViewport3D()) return
    const pivot = getPivotGroupForLink(selectedLink)
    if (!pivot) return

    // Floor constraint: keep the lowest vertex of the component's geometry at world Y ≥ 0.
    // We use Box3.setFromObject to find the true AABB minimum Y (accounts for geometry
    // offset from pivot origin). Parent rotation means world Y ≠ local Y, so the
    // correction is converted via the parent's inverse world quaternion.
    if (gizmo.mode === 'translate' && pivot.parent) {
      pivot.updateMatrixWorld(true)
      const aabb = new THREE.Box3().setFromObject(pivot)
      const minY = aabb.min.y
      if (minY < 0) {
        const correctionWorld = new THREE.Vector3(0, -minY, 0)
        const parentWorldQuat = new THREE.Quaternion()
        const parentWorldScale = new THREE.Vector3()
        pivot.parent.updateMatrixWorld(true)
        pivot.parent.matrixWorld.decompose(new THREE.Vector3(), parentWorldQuat, parentWorldScale)
        const localDelta = correctionWorld
          .applyQuaternion(parentWorldQuat.clone().invert())
          .divide(parentWorldScale)
        pivot.position.add(localDelta)
        pivot.updateMatrixWorld(true)
      }
    }

    updateBestCandidateDuringDrag(pivot)
  })

  ctx.canvas.addEventListener('pointerdown', e => {
    pointerDown.set(e.clientX, e.clientY)
  })

  ctx.canvas.addEventListener('click', e => {
    if (!ctx.isViewport3D()) return
    const moved = Math.abs(e.clientX - pointerDown.x) > 8 || Math.abs(e.clientY - pointerDown.y) > 8
    const movedCarry = Math.abs(e.clientX - pointerDown.x) > 22 || Math.abs(e.clientY - pointerDown.y) > 22
    if (carryComp) {
      if (movedCarry) return
      commitCarry()
      return
    }
    if (moved) return
    const ray = makeRaycaster(e)
    const hits = ray.intersectObjects(getPickTargets(), false)
    if (ctx.getInteractionMode() === 'inspect') {
      if (hits.length === 0) {
        ctx.onInspectLinkFocused(null)
        return
      }
      const first = hits[0].object
      const link = (first.userData as Record<string, unknown>).urdfLinkName
      if (typeof link === 'string' && link && !isMountLinkName(link)) {
        ctx.onInspectLinkFocused(link)
      } else {
        ctx.onInspectLinkFocused(null)
      }
      return
    }
    if (hits.length === 0) {
      selectLink(null)
      return
    }
    const first = hits[0].object
    const link = (first.userData as Record<string, unknown>).urdfLinkName
    if (typeof link === 'string' && link) {
      selectLink(link)
    }
  })

  ctx.canvas.addEventListener('mousemove', e => {
    if (!ctx.isViewport3D()) return
    if (carryComp) { updateCarryFromMouse(e); return }
    const ray = makeRaycaster(e)
    const hits = ray.intersectObjects(getPickTargets(), false)
    ctx.canvas.style.cursor = hits.length > 0 ? 'pointer' : ''
  })

  document.addEventListener('keydown', e => {
    const inTyping = isTypingTarget(e.target)
    if (inTyping) return
    if (!ctx.isViewport3D()) return
    const k = e.key.toLowerCase()

    if ((e.ctrlKey || e.metaKey) && k === 'z' && !e.shiftKey) {
      e.preventDefault()
      const prev = urdfUndo.pop()
      if (!prev) return
      urdfRedo.push(ctx.getUrdfText())
      ctx.setUrdfText(prev)
      ctx.reparseUrdf()
      ctx.showToast('Undo', 'info')
      return
    }
    if ((e.ctrlKey || e.metaKey) && (k === 'y' || (k === 'z' && e.shiftKey))) {
      e.preventDefault()
      const next = urdfRedo.pop()
      if (!next) return
      urdfUndo.push(ctx.getUrdfText())
      ctx.setUrdfText(next)
      ctx.reparseUrdf()
      ctx.showToast('Redo', 'info')
      return
    }
    if (k === 'escape' && carryComp) { exitCarryMode(); return }
    if (k === 'enter' && carryComp) {
      e.preventDefault()
      commitCarry()
      return
    }

    // ── Carry mode nudge / rotate ──
    if (carryComp && carryGroup) {
      const isNudgeKey = ['arrowleft','arrowright','arrowup','arrowdown','pageup','pagedown'].includes(k)
      if (isNudgeKey) {
        e.preventDefault()
        const step = e.shiftKey ? 0.01 : 0.001
        if (k === 'arrowleft')  carryGroup.position.x -= step
        if (k === 'arrowright') carryGroup.position.x += step
        if (k === 'arrowup')    carryGroup.position.z -= step
        if (k === 'arrowdown')  carryGroup.position.z += step
        if (k === 'pageup')     carryGroup.position.y += step
        if (k === 'pagedown') {
          carryGroup.position.y -= step
          // Clamp so the ghost never clips below the floor.
          const { hx: phx, hy: phy, hz: phz, cx: pcx, cy: pcy, cz: pcz } = carryGhostBounds ?? computeCarryGhostBounds(carryComp)
          carryGroup.updateMatrixWorld(true)
          const lifted = clampCarryMatrixAboveFloor(carryGroup.matrixWorld, phx, phy, phz, pcx, pcy, pcz)
          carryGroup.position.setFromMatrixPosition(lifted)
        }
        carryWorldPos.copy(carryGroup.position)
        carryFrozen = true
        carryGroup.updateMatrixWorld(true)
        updateCarrySnap()
        updateCarryHudText()
        return
      }
      if (k === 'r') {
        e.preventDefault()
        carryRotStep = (carryRotStep + 1) % 4
        carryGroup.quaternion.copy(carryRotQuat())
        carryFrozen = true
        carryGroup.updateMatrixWorld(true)
        updateCarrySnap()
        updateCarryHudText()
        return
      }
      if (k === 'tab') {
        e.preventDefault()
        if (carrySnapCandidates.length > 0) {
          carrySnapIdx = e.shiftKey
            ? (carrySnapIdx - 1 + carrySnapCandidates.length) % carrySnapCandidates.length
            : (carrySnapIdx + 1) % carrySnapCandidates.length
          const chosen = carrySnapCandidates[carrySnapIdx]
          carryBestMount = { targetParentLink: chosen.targetParentLink, mountLink: chosen.mountLink, desiredGhostWorld: chosen.desiredGhostWorld }
          carryFrozen = true
          const snapPos = new THREE.Vector3(); const snapQuat = new THREE.Quaternion()
          chosen.desiredGhostWorld.decompose(snapPos, snapQuat, new THREE.Vector3())
          carryGroup!.position.copy(snapPos); carryGroup!.quaternion.copy(snapQuat)
          carryGroup!.updateMatrixWorld(true)
          clearBestCandidateHighlight()
          for (const t of mountNodes) setNodeMeshState(t.mountLink, isMountOccupied(t.mountLink) ? 'occupied' : 'neutral')
          setNodeMeshState(chosen.mountLink, 'best')
        } else {
          carryFrozen = false
          updateCarrySnap()
        }
        updateCarryHudText()
        return
      }
    }

    if (k === 'i') { ctx.switchPanel('inspector'); return }
    if (k === 't') { ctx.switchPanel('toolbox'); return }
    if (k === 'r' && selectedLink && gizmo.object) {
      gizmo.setMode(gizmo.mode === 'translate' ? 'rotate' : 'translate')
      ctx.showToast(`Gizmo: ${gizmo.mode}`, 'info')
      return
    }
    if ((k === 'delete' || k === 'backspace') && selectedLink) {
      e.preventDefault()
      deleteLink(selectedLink)
      return
    }
  })

  function onModelUpdated() {
    if (selectedLink) {
      if (!ctx.getParsedRobot().linkGroups.has(selectedLink)) {
        selectedLink = null
        gizmo.detach()
      } else {
        const pivot = getPivotGroupForLink(selectedLink)
        if (pivot) {
          gizmo.attach(pivot)
          pivot.updateMatrixWorld(true)
          gizmoBasePivotWorld.copy(pivot.matrixWorld)
        } else {
          gizmo.detach()
        }
      }
    }
    ghostGroup.visible = false
    bestMountCandidate = null
    rebuildMountNodes()
    refreshBuildPanel()
    renderInspector()
    ctx.onAfterModelUpdated?.()
  }

  function onInteractionModeChanged(mode: 'build' | 'inspect') {
    if (mode === 'inspect') {
      exitCarryMode()
      selectLink(null)
      gizmo.detach()
      refreshBuildPanel()
      renderInspector()
    }
  }

  refreshBuildPanel()
  renderInspector()

  return {
    onModelUpdated,
    recordUndoExternal: (content: string) => {
      urdfUndo.push(content)
      if (urdfUndo.length > 80) urdfUndo.shift()
      urdfRedo = []
    },
    exitCarryMode,
    onInteractionModeChanged,
    setSelectedLink: selectLink,
  }
}


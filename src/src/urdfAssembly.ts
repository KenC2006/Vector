import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { TransformControls } from 'three/addons/controls/TransformControls.js'
import { STLExporter } from 'three/addons/exporters/STLExporter.js'
import { invoke } from '@tauri-apps/api/core'
import { generateVisuals, CATEGORY_COLORS } from './componentMeshes'
import type { UrdfVisualDesc } from './componentMeshes'
import { isMountLinkName, makeMountLinkName, defaultFaceNodesForBoxDims, nodesCompatible, incompatibleReason, componentPortsForPreset, resolveFaceToPort } from './attachmentNodes'
import type { AttachmentNodeRuntime, AttachmentNodeClass } from './attachmentNodes'
import { hasMeshOverride } from './richVisuals/meshOverrides'
import { SLOW_MESH_BLACKLIST, getRenderedMeshDims } from './richVisuals/index'
import { quatToRpy } from './rotationIO'
import { validateTopology as runValidateTopology, autoRepairTopology as runAutoRepair } from './topologyValidation.ts'
import type { ValidationPreset, ValidationContext } from './topologyValidation.ts'
import { cloneAssemblyGraph, graphsEquivalent } from './urdfGraphEquivalence.ts'
import type { AssemblyComponent, AssemblyGraph, GraphEquivalenceResult } from './urdfGraphEquivalence.ts'
// Mate-connector resolver (Phase 1/2, docs/MATE_CONNECTOR_MIGRATION.md). Pure
// module, bit-identical to the bbox math when mating default face connectors
// with `fastened` — see mateCorpus.ts for the parity proof. Feature-flagged
// so the legacy path can still be exercised for A/B comparison.
import {
  generateDefaultConnectors,
  mergeConnectors,
  resolveMate,
  findConnector,
  childConnectorIdForAttachFace,
  type MateConnector,
  type MateType,
} from './mateConnectors.ts'
import { applyMutation as runApplyMutation } from './graphMutations.ts'
import type { GraphMutation, MutationResult } from './graphMutations.ts'
// Render-time alignment pass (Option C — see docs/ENGINE_ARCHITECTURE.md).
// Kept in its own module so the test harness can import it without pulling in
// this file's DOM/tauri dependencies. Re-exported below for external callers.
import { reconcileNodePlacement } from './reconcileAlignment.ts'
import type { ReconcileResult } from './reconcileAlignment.ts'
export { reconcileNodePlacement } from './reconcileAlignment.ts'
export type { ReconcileInputs, ReconcileResult, ReconcileShift } from './reconcileAlignment.ts'
// Re-export so existing importers (topologyValidation.ts, topologyCorpus.ts,
// viewportChat.ts, main.ts) keep working from their usual location.
export { cloneAssemblyGraph, graphsEquivalent } from './urdfGraphEquivalence.ts'
export type { AssemblyComponent, AssemblyGraph, GraphEquivalenceResult } from './urdfGraphEquivalence.ts'

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
  /** Returns true while simulation is running — carry/edit blocked during sim. */
  isSimActive?: () => boolean
  /** Inspect mode: user clicked a URDF link mesh (or null = empty space). */
  onInspectLinkFocused: (linkName: string | null) => void
  /** Returns the active file name (for per-file graph persistence). */
  getActiveFileName?: () => string
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
  /** Phase 3 mate-connector authorship (docs/MATE_CONNECTOR_MIGRATION.md).
   *  Optional; merged OVER the 6 auto-generated defaults by id so a preset
   *  can add new connectors (shaft_out, plate_top) or override a default
   *  whose bbox-derived pose doesn't match the rendered mesh. */
  connectors?: MateConnector[]
}

interface PresetCategory {
  description: string
  components: PresetComponent[]
}

interface PresetData {
  categories: Record<string, PresetCategory>
}

// AssemblyComponent / AssemblyGraph are defined in ./urdfGraphEquivalence.ts
// and re-exported at the top of this file (keeps the pure-from-Node test harness
// free of THREE/DOM imports).

export interface TopologyOp {
  op: 'add' | 'remove' | 'modify'
  link_name: string
  component_id?: string
  attach_to?: string | null
  attach_face?: string
  joint_type?: string
  joint_axis?: string
  length_mm?: number
  orientation?: string
  elevation_angle?: number
  attach_rpy?: number[]
}

export interface UrdfAssemblyApi {
  onModelUpdated(): void
  recordUndoExternal(content: string): void
  exitCarryMode(): void
  onInteractionModeChanged(mode: 'build' | 'inspect'): void
  /** Sync 3D selection / gizmo / inspector (used when opening Properties from Focus panel). */
  setSelectedLink(linkName: string | null): void
  /** Resolve an AI assembly graph using the frontend snap/placement system. Returns final URDF and any topology errors. */
  resolveAssemblyGraph(graph: AssemblyGraph): { urdf: string | null; topologyErrors?: string[]; topologyWarnings?: string[] }
  /** Render-time alignment: measure real AABBs of rendered meshes and shift pivots
   *  so child contact surfaces meet their parent's attach face. No-op if no graph
   *  has been resolved yet. Safe to call multiple times (EPS-guarded, idempotent). */
  reconcileNodePlacement(): ReconcileResult
  /** Get a deep-cloned snapshot of the last successfully resolved AssemblyGraph. Cloned so
   *  callers (chat context, IPC marshaling) can't mutate the canonical in-memory copy. */
  getLastAssemblyGraph(): AssemblyGraph | null
  /** Reverse-parse current URDF into an AssemblyGraph for iterative editing (lossy fallback — prefer getLastAssemblyGraph). */
  urdfToAssemblyGraph(urdfXml: string): AssemblyGraph | null
  /** Structural + parametric equality for two AssemblyGraphs. Use to detect drift when a
   *  reverse-parse is unavoidable (import-URDF path). */
  graphsEquivalent(a: AssemblyGraph, b: AssemblyGraph): GraphEquivalenceResult
  /** Apply modify_topology operations to an existing AssemblyGraph and return the modified version. */
  applyTopologyOps(graph: AssemblyGraph, operations: TopologyOp[]): AssemblyGraph
  /** WS2 tool-call edit surface: apply a single typed mutation with per-call
   *  validation. Runs against a deep clone of `graph`; on success the new graph
   *  is returned and the caller commits via resolveAssemblyGraph. On failure,
   *  a structured error returns to the Claude tool loop for same-turn self-
   *  correction — no full-graph redesign fired. */
  applyGraphMutation(graph: AssemblyGraph, mutation: GraphMutation): MutationResult
  /** Re-run attachment node placement based on current scene geometry. Call after async GLB meshes settle. */
  rebuildMountNodes(): void
  /** Snapshot the current undo/redo stacks (call before switching files). */
  getUndoState(): { undo: string[]; redo: string[] }
  /** Restore a previously saved undo/redo snapshot (call after switching files). */
  restoreUndoState(state: { undo: string[]; redo: string[] }): void
  /** True while resolveAssemblyGraph is batching edits. Callers (e.g. reparseURDF) skip heavy per-mesh rebuilds when active. */
  isBulkAssemblyMode(): boolean
  /** Look up a component's authoritative bounding box from the preset catalog (in mm).
   *  Returns null when the preset has only a 2-tuple cross_section_mm (extrusions),
   *  where per-instance length_mm makes the link's URDF box the authoritative source. */
  getPresetBoundingBoxMm(compId: string): [number, number, number] | null
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

/**
 * Re-express a joint's `<axis xyz>` in the new joint frame after the joint has been
 * moved or reoriented. The physical world-space axis direction is preserved.
 *
 * Only applies to revolute / prismatic / continuous joints (fixed joints have no axis).
 *
 * TODO: when ball/floating joints are added, a single axis vector is insufficient;
 *       that will require a per-DOF quaternion stack at the joint level.
 *
 * @param jointEl         The `<joint>` XML element to update.
 * @param doc             The owning Document (used to create the `<axis>` element if missing).
 * @param oldJointWorldQ  Quaternion of the joint frame BEFORE the move (drag-start snapshot).
 * @param parentWorldQ    World quaternion of the joint's NEW parent link.
 * @param newLocalQuat    New local quaternion of the joint in the parent frame.
 */
function reconcileJointAxis(
  jointEl: Element,
  doc: Document,
  oldJointWorldQ: THREE.Quaternion,
  parentWorldQ: THREE.Quaternion,
  newLocalQuat: THREE.Quaternion,
): void {
  const jType = jointEl.getAttribute('type') || 'fixed'
  if (jType !== 'revolute' && jType !== 'prismatic' && jType !== 'continuous') return

  const axisEl = jointEl.querySelector('axis')
  const axisStr = axisEl?.getAttribute('xyz') || '0 0 1'
  const axisArr = axisStr.trim().split(/\s+/).map(Number)
  const axisInOldJoint = new THREE.Vector3(
    Number.isFinite(axisArr[0]) ? axisArr[0] : 0,
    Number.isFinite(axisArr[1]) ? axisArr[1] : 0,
    Number.isFinite(axisArr[2]) ? axisArr[2] : 1,
  ).normalize()

  // 1. Lift axis into world space using the old joint frame orientation.
  const axisWorld = axisInOldJoint.clone().applyQuaternion(oldJointWorldQ)

  // 2. New joint world orientation = new parent world quat × new local quat.
  const newJointWorldQ = parentWorldQ.clone().multiply(newLocalQuat)

  // 3. Pull axis back into the new joint frame and renormalize to guard float drift.
  const newAxisInJoint = axisWorld.clone()
    .applyQuaternion(newJointWorldQ.clone().invert())
    .normalize()

  const axisElToWrite = axisEl ?? doc.createElement('axis')
  axisElToWrite.setAttribute('xyz',
    `${fmt(newAxisInJoint.x)} ${fmt(newAxisInJoint.y)} ${fmt(newAxisInJoint.z)}`)
  if (!axisEl) jointEl.appendChild(axisElToWrite)
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

  // Bulk-assembly mode: while active, commitUrdf mutates an in-memory URDF buffer
  // instead of writing to the Monaco editor + triggering a full reparse. Set by
  // resolveAssemblyGraph around its placement loop so a 50-component build fires
  // exactly one reparse at the end instead of one per component (~30× faster).
  let _bulkMode = false
  let _bulkUrdfBuffer: string | null = null

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
  let snapRadiusM = 0.05          // adjustable via [ ] in carry mode
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
    if (dist > snapRadiusM) return { ok: false, reason: 'too-far', dist }

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

  /**
   * Sync reconcile's in-scene pivot shifts back into the URDF text so the next
   * reparse doesn't undo them. Called by resolveAssemblyGraph after the
   * initial reconcile pass — see the adjacent comment there for the timing
   * problem this solves (editor-change debounce → reparse → wiped shifts →
   * duplicate reconcile replay).
   *
   * For each joint in the URDF, read the live pivot.position (which reconcile
   * has already updated) and write it back as the joint's origin xyz. We also
   * rewrite rpy from pivot.quaternion — reconcile today only shifts position,
   * but keeping both fields in sync avoids a divergence hazard if a future
   * pass learns to rotate pivots.
   */
  function persistReconcileShiftsToUrdf(linkGroups: Map<string, THREE.Group>): void {
    const raw = ctx.getUrdfText()
    const doc = new DOMParser().parseFromString(raw, 'application/xml')
    if (doc.querySelector('parsererror')) return

    let changed = false
    const jointEls = Array.from(doc.querySelectorAll('joint'))
    for (const jointEl of jointEls) {
      const childEl = jointEl.querySelector('child')
      const childName = childEl?.getAttribute('link')
      if (!childName) continue
      const childGroup = linkGroups.get(childName)
      const pivot = childGroup?.parent as THREE.Group | null
      if (!pivot) continue
      const origin = jointEl.querySelector('origin')
      if (!origin) continue

      const x = pivot.position.x
      const y = pivot.position.y
      const z = pivot.position.z
      const newXyz = `${x.toFixed(4)} ${y.toFixed(4)} ${z.toFixed(4)}`
      if (origin.getAttribute('xyz') !== newXyz) {
        origin.setAttribute('xyz', newXyz)
        changed = true
      }
    }
    if (!changed) return

    // Pretty-print using the same rule resolveAssemblyGraph uses for its own
    // post-placement formatting pass, so the editor diff stays small.
    const serialized = new XMLSerializer().serializeToString(doc)
    const lines = serialized.replace(/></g, '>\n<').split('\n')
    let indent = 0
    const prettyUrdf = lines.map(line => {
      const trimmed = line.trim()
      if (!trimmed) return ''
      if (trimmed.startsWith('</')) indent = Math.max(0, indent - 1)
      const result = '  '.repeat(indent) + trimmed
      if (trimmed.startsWith('<') && !trimmed.startsWith('</') && !trimmed.startsWith('<?') && !trimmed.endsWith('/>')) {
        indent++
      }
      return result
    }).filter(l => l.length > 0).join('\n')

    // setUrdfText without reparse — the live scene is already in the aligned
    // state, so a reparse right now would just rebuild redundantly. The
    // debounced editor-change reparse (main.ts) will fire ~500ms later and
    // rebuild from the aligned text; that rebuild then triggers a meshLoaded
    // debounce → reconcile → zero deltas (genuine no-op, which is the point).
    ctx.setUrdfText(prettyUrdf)
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
    const parser = new DOMParser()
    const doc = parser.parseFromString(getCurrentUrdfText(), 'application/xml')
    if (doc.documentElement.nodeName === 'parsererror') {
      ctx.showToast('Cannot edit invalid URDF', 'error')
      return false
    }
    // In bulk mode the pre-bulk snapshot was already pushed by setBulkAssemblyMode(true);
    // per-iteration recordUndo would spam N identical entries and evict older history.
    if (!_bulkMode) recordUndo()
    const changed = mutator(doc)
    if (!changed) {
      if (!_bulkMode) urdfUndo.pop()
      return false
    }
    // Skip formatXml in bulk mode — buffer is only read by the next DOMParser pass,
    // and the pretty-print step at the end of resolveAssemblyGraph reformats anyway.
    if (_bulkMode) {
      _bulkUrdfBuffer = new XMLSerializer().serializeToString(doc)
      return true
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

  function setBulkAssemblyMode(active: boolean): void {
    if (active === _bulkMode) return
    if (active) {
      // Record one pre-bulk snapshot so Ctrl+Z reverts the entire assembly in a
      // single step. Individual commitUrdf calls skip recordUndo while bulk is active.
      recordUndo()
      _bulkMode = true
      _bulkUrdfBuffer = ctx.getUrdfText()
    } else {
      _bulkMode = false
      const pending = _bulkUrdfBuffer
      _bulkUrdfBuffer = null
      if (pending !== null) {
        ctx.setUrdfText(pending)
        ctx.reparseUrdf(pending)
      }
    }
  }

  // Returns the current URDF text, preferring the in-memory bulk buffer when
  // bulk-assembly mode is active (so mid-loop readers see the latest mutations
  // without a reparse).
  function getCurrentUrdfText(): string {
    return _bulkMode && _bulkUrdfBuffer !== null ? _bulkUrdfBuffer : ctx.getUrdfText()
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
            <span class="joint-lim-label">Origin RPY (°)</span>
            <input id="urdf-origin-r" class="joint-lim-input" title="Roll (degrees)" />
            <input id="urdf-origin-p" class="joint-lim-input" title="Pitch (degrees)" />
            <input id="urdf-origin-yaw" class="joint-lim-input" title="Yaw (degrees)" />
          </div>
        ` : '<div class="insp-empty">Root link has no parent joint origin</div>'}
      </div>
      <div class="insp-actions-group">
        ${parentJoint ? `
          <button type="button" class="bi-action-btn apply-btn" id="urdf-apply-origin">Apply Changes</button>
          <button type="button" class="bi-action-btn" id="urdf-reset-rotation">Reset Rotation</button>
        ` : ''}
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
    // XYZ in metres; RPY stored in radians, displayed in degrees.
    const fmtDeg = (rad: number) => String(+(rad * 180 / Math.PI).toFixed(4))
    x.value = String(xyz[0]); y.value = String(xyz[1]); z.value = String(xyz[2])
    r.value = fmtDeg(rpy[0]); p.value = fmtDeg(rpy[1]); yw.value = fmtDeg(rpy[2])

    const applyBtn = document.getElementById('urdf-apply-origin')
    applyBtn?.addEventListener('click', () => {
      // Convert degrees back to radians before writing URDF.
      const toRad = (v: string) => Number(v) * Math.PI / 180
      commitUrdf(documentXml => {
        const j = documentXml.querySelector(`joint[name="${parentJoint.name}"]`)
        if (!j) return false
        const o = ensureOrigin(j, documentXml)
        o.setAttribute('xyz', `${fmt(Number(x.value))} ${fmt(Number(y.value))} ${fmt(Number(z.value))}`)
        o.setAttribute('rpy', `${fmt(toRad(r.value))} ${fmt(toRad(p.value))} ${fmt(toRad(yw.value))}`)
        return true
      })
      ctx.showToast('Updated joint origin in URDF', 'success')
    })

    const resetRotBtn = document.getElementById('urdf-reset-rotation')
    resetRotBtn?.addEventListener('click', () => {
      const pivot = getPivotGroupForLink(selectedLink!)
      const parentObj = pivot?.parent
      if (!pivot || !parentObj) return
      pivot.updateMatrixWorld(true)
      parentObj.updateMatrixWorld(true)

      // Capture pre-reset orientations so actuated joint axes can be re-expressed.
      const oldJointWorldQ = new THREE.Quaternion()
      pivot.matrixWorld.decompose(new THREE.Vector3(), oldJointWorldQ, new THREE.Vector3())
      const parentWorldQ = new THREE.Quaternion()
      parentObj.matrixWorld.decompose(new THREE.Vector3(), parentWorldQ, new THREE.Vector3())

      commitUrdf(documentXml => {
        const j = documentXml.querySelector(`joint[name="${parentJoint.name}"]`)
        if (!j) return false
        const o = ensureOrigin(j, documentXml)
        o.setAttribute('rpy', '0 0 0')
        // newLocalQuat = identity: joint frame aligns with parent frame after reset.
        reconcileJointAxis(j, documentXml, oldJointWorldQ, parentWorldQ, new THREE.Quaternion())
        return true
      })
      ctx.showToast('Reset rotation to 0°', 'success')
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
    updateComponentCompatibility()
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
  // comp.id → list item element, for compatibility updates without full re-render
  const compItemEls = new Map<string, HTMLElement>()

  // ── Category icons ──────────────────────────────────────────────────────────
  const CATEGORY_ICONS: Record<string, string> = {
    actuators:    `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><circle cx="7" cy="7" r="2.2"/><path d="M7 1v1.5M7 11.5V13M1 7h1.5M11.5 7H13M2.93 2.93l1.06 1.06M10.01 10.01l1.06 1.06M2.93 11.07l1.06-1.06M10.01 3.99l1.06-1.06" stroke-linecap="round"/></svg>`,
    motors:       `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><circle cx="7" cy="7" r="4.5"/><circle cx="7" cy="7" r="1.5"/><path d="M7 2.5v1.8M7 9.7v1.8M2.5 7h1.8M9.7 7h1.8" stroke-linecap="round"/></svg>`,
    sensors:      `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><ellipse cx="7" cy="7" rx="5" ry="3.5"/><circle cx="7" cy="7" r="1.5"/><path d="M2.5 5C3.5 2.5 10.5 2.5 11.5 5" stroke-linecap="round"/></svg>`,
    compute:      `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><rect x="3" y="3" width="8" height="8" rx="1"/><path d="M5 1v2M9 1v2M5 11v2M9 11v2M1 5h2M1 9h2M11 5h2M11 9h2" stroke-linecap="round"/></svg>`,
    power:        `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><path d="M8.5 1.5L5 7.5h4L5 12.5" stroke-linecap="round" stroke-linejoin="round"/></svg>`,
    structural:   `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><rect x="1.5" y="4" width="11" height="6" rx="0.5"/><line x1="1.5" y1="6.5" x2="12.5" y2="6.5"/><line x1="1.5" y1="7.5" x2="12.5" y2="7.5"/></svg>`,
    transmission: `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><circle cx="4.5" cy="7" r="2.5"/><circle cx="9.5" cy="7" r="2"/><line x1="7" y1="7" x2="7.5" y2="7" stroke-width="1.5"/></svg>`,
    end_effectors:`<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><path d="M4 13V7.5L2 5V3h1.5l1.5 2.5L7 7M10 13V7.5l2-2.5V3H10.5L9 5.5 7 7" stroke-linecap="round" stroke-linejoin="round"/><line x1="7" y1="7" x2="7" y2="13" stroke-linecap="round"/></svg>`,
    mobility:     `<svg viewBox="0 0 14 14" fill="none" stroke="currentColor" stroke-width="1.2"><circle cx="7" cy="7" r="5"/><circle cx="7" cy="7" r="1.5"/><path d="M7 2v2M7 10v2M2 7h2M10 7h2" stroke-linecap="round"/></svg>`,
  }

  // Map from preset mounting_logic.primary → whether it works with any face-mount link.
  // Rail-only types need a structural/extrusion parent.
  function mountIsCompatible(comp: PresetComponent): boolean {
    if (!selectedLink) return false
    const primary = ((comp.mounting_logic as Record<string, unknown>).primary ?? '') as string
    if (primary === 'side_rail_mount' || primary === 'rail_slot') {
      return selectedLink.includes('extrusion') || selectedLink.includes('rail')
    }
    return true
  }

  function updateComponentCompatibility() {
    for (const [id, el] of compItemEls) {
      if (!presetData) break
      for (const cat of Object.values(presetData.categories)) {
        const comp = cat.components.find(c => c.id === id)
        if (comp) { el.classList.toggle('compatible', mountIsCompatible(comp)); break }
      }
    }
  }

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

  function getParentBounds(doc: Document, parentLinkName: string): { hx: number; hy: number; hz: number; cx: number; cy: number; cz: number } {
    // Primary: use the actual rendered mesh dims from meshDimsCache — this is exactly what
    // rebuildMountNodes uses via computeLinkLocalBoundingBox, so joint origins align with nodes.
    // Strip trailing _N instance number to recover the component ID (e.g. "servo_micro_2" → "servo_micro").
    const compIdMatch = parentLinkName.match(/^(.+)_(\d+)$/)
    const compId = compIdMatch?.[1] ?? parentLinkName
    const renderedDims = getRenderedMeshDims(compId)
    if (renderedDims && renderedDims.x > 0.001) {
      // GLB is re-centered on its AABB in applyMeshToLink, so AABB center sits at the
      // link origin — report cx=cy=cz=0 for the rendered-mesh path.
      return {
        hx: renderedDims.x / 2,
        hy: renderedDims.y / 2,
        hz: renderedDims.z / 2,
        cx: 0, cy: 0, cz: 0,
      }
    }

    // Fallback: iterate ALL <visual> elements in the URDF, accounting for each element's
    // <origin xyz> offset. This handles multi-piece shapes (body + shaft, body + horn, etc.)
    // where previously only the first visual was read, missing protrusions in URDF Z (up).
    const linkEl = doc.querySelector(`link[name="${parentLinkName}"]`)
    if (!linkEl) return { hx: 0.05, hy: 0.05, hz: 0.05, cx: 0, cy: 0, cz: 0 }

    const visuals = linkEl.querySelectorAll('visual')
    if (!visuals.length) return { hx: 0.05, hy: 0.05, hz: 0.05, cx: 0, cy: 0, cz: 0 }

    // Compute the full axis-aligned bounding box across all visuals, then derive
    // half-extents.  Previous code used `abs(offset) + half_extent` which equals a
    // full extent from the link origin, not a half-extent — causing over-sized bounds
    // for multi-visual links with offset pieces (e.g. servo horn, motor shaft).
    let minX = Infinity, maxX = -Infinity
    let minY = Infinity, maxY = -Infinity
    let minZ = Infinity, maxZ = -Infinity

    for (const visual of Array.from(visuals)) {
      const originEl = visual.querySelector('origin')
      const xyz = (originEl?.getAttribute('xyz') || '0 0 0').split(/\s+/).map(Number)
      const ox = xyz[0] || 0, oy = xyz[1] || 0, oz = xyz[2] || 0

      const geom = visual.querySelector('geometry')
      if (!geom) continue

      const boxEl = geom.querySelector('box')
      const cylEl = geom.querySelector('cylinder')
      const sphEl = geom.querySelector('sphere')

      let ex = 0, ey = 0, ez = 0
      if (boxEl) {
        const size = (boxEl.getAttribute('size') || '0 0 0').split(/\s+/).map(Number)
        ex = (size[0] || 0) / 2; ey = (size[1] || 0) / 2; ez = (size[2] || 0) / 2
      } else if (cylEl) {
        const r = Number(cylEl.getAttribute('radius')) || 0
        const h = Number(cylEl.getAttribute('length')) || 0
        ex = r; ey = r; ez = h / 2
      } else if (sphEl) {
        const r = Number(sphEl.getAttribute('radius')) || 0
        ex = r; ey = r; ez = r
      }

      minX = Math.min(minX, ox - ex); maxX = Math.max(maxX, ox + ex)
      minY = Math.min(minY, oy - ey); maxY = Math.max(maxY, oy + ey)
      minZ = Math.min(minZ, oz - ez); maxZ = Math.max(maxZ, oz + ez)
    }

    if (!isFinite(minX)) return { hx: 0.05, hy: 0.05, hz: 0.05, cx: 0, cy: 0, cz: 0 }
    // Guard against degenerate zero-extent dims only; legitimate thin parts
    // (6mm coupler disc, 3mm IMU, PCBs) must report their real half-extent or
    // placement stacks the next child above a phantom gap.
    const xExtent = (maxX - minX) / 2
    const yExtent = (maxY - minY) / 2
    const zExtent = (maxZ - minZ) / 2
    // AABB center offset from link origin — non-zero when the link has asymmetric
    // protrusions (e.g. a servo's shaft sticks up beyond the body's symmetric ±half).
    // Placement uses this to distinguish "body face distance" from "AABB half".
    return {
      hx: xExtent > 0 ? xExtent : 0.005,
      hy: yExtent > 0 ? yExtent : 0.005,
      hz: zExtent > 0 ? zExtent : 0.005,
      cx: xExtent > 0 ? (maxX + minX) / 2 : 0,
      cy: yExtent > 0 ? (maxY + minY) / 2 : 0,
      cz: zExtent > 0 ? (maxZ + minZ) / 2 : 0,
    }
  }

  function computePlacement(
    doc: Document, parentLinkName: string,
    comp: PresetComponent,
    childX: number, _childY: number, childZ: number,
    childCenterOffset: { cx: number; cy: number; cz: number } = { cx: 0, cy: 0, cz: 0 },
  ): { xyz: string; rpy: string } {
    const parent = getParentBounds(doc, parentLinkName)
    const mount = (comp.mounting_logic?.primary as string) || 'face_mount'
    const gap = 0
    // Body half-extents (see computeFacePlacement for rationale) — use these for
    // face-normal stacking so asymmetric protrusions don't introduce gaps.
    const parentBodyHX = parent.hx - Math.abs(parent.cx)
    const parentBodyHZ = parent.hz - Math.abs(parent.cz)
    const childBodyHX = childX / 2 - Math.abs(childCenterOffset.cx)
    const childBodyHZ = childZ / 2 - Math.abs(childCenterOffset.cz)

    // face_mount / pcb_solder / bracket_mount → stack on top (Z+) of parent
    if (mount === 'face_mount' || mount === 'pcb_solder' || mount === 'bracket_mount') {
      const oz = parentBodyHZ + childBodyHZ + gap
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }

    // axial_shaft → coaxial along Z, placed at parent's top face
    if (mount === 'axial_shaft') {
      const oz = parentBodyHZ + childBodyHZ + gap
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }

    // rail_slot / side_rail_mount → mount on the side (X+) of parent
    if (mount === 'rail_slot' || mount === 'side_rail_mount' || mount === 'clamp_mount') {
      const ox = parentBodyHX + childBodyHX + gap
      return { xyz: `${ox.toFixed(4)} 0 0`, rpy: '0 0 0' }
    }

    // hub_bore → coaxial, flush with parent face
    if (mount === 'hub_bore') {
      const oz = parentBodyHZ + childBodyHZ + gap
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }

    // press_fit → inside parent bore, centered
    if (mount === 'press_fit') {
      return { xyz: '0 0 0', rpy: '0 0 0' }
    }

    // linear_rod → extend along Z from parent
    if (mount === 'linear_rod') {
      const oz = parentBodyHZ + childBodyHZ + gap
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }

    // tool_changer_master/slave → stack on bottom (Z-) if slave
    if (mount === 'tool_changer_slave') {
      const oz = -(parentBodyHZ + childBodyHZ + gap)
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }
    if (mount === 'tool_changer_master') {
      const oz = parentBodyHZ + childBodyHZ + gap
      return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
    }

    // Default: stack on top
    const oz = parentBodyHZ + childBodyHZ + gap
    return { xyz: `0 0 ${oz.toFixed(4)}`, rpy: '0 0 0' }
  }

  /**
   * Returns the appropriate splay angle (radians) for a given number of limbs on the bottom face.
   * Scales from a gentle tilt for bipods up to a wide stance for hexapods and beyond.
   */
  function splayAngleForLegCount(n: number): number {
    if (n <= 2)  return 0.262  // ~15°
    if (n === 3) return 0.436  // ~25°
    if (n === 4) return 0.524  // ~30°
    if (n <= 6)  return 0.611  // ~35°
    return 0.698               // ~40° for 7+
  }

  /**
   * Face-based placement for AI assembly resolver.
   * Uses the explicit attach_face from Claude's topology instead of mounting_logic.
   * Supports multiple children on the same face with automatic offset distribution.
   *
   * @param childIndex - which child this is on this face (0-based)
   * @param totalOnFace - total children that will be on this face
   */
  // Which pre-rotation half-extent ends up vertical after an axis-aligned
  // RPY rotation. Only ±90° roll or pitch swap an axis onto Z; smaller angles
  // (e.g. leg splay) leave Z dominant, so they keep childZ.
  //
  // Without this, a sideways cylinder (wheel, roller, caster, horizontal
  // bearing) is placed using its pre-rotation thickness rather than its
  // post-rotation radius, and the part clips into its parent by (radius − thickness)/2.
  function verticalExtentForRotation(
    childX: number, childY: number, childZ: number,
    rollRad: number, pitchRad: number,
  ): number {
    const RIGHT_ANGLE = Math.PI / 2
    const nearRight = (v: number) => Math.abs(Math.abs(v) - RIGHT_ANGLE) < 0.1
    if (nearRight(rollRad)) return childY   // ±90° roll: Y → vertical
    if (nearRight(pitchRad)) return childX  // ±90° pitch: X → vertical
    return childZ
  }

  // ── Phase 2 connector branch (docs/MATE_CONNECTOR_MIGRATION.md) ───────────
  // Feature flag. Defaults on: when no preset has authored connectors, the new
  // path only fires for components that explicitly opt in via
  // attach_connector/mate_connector/mate_type. For every other component,
  // computeFacePlacement runs unchanged — bit-identical to WS5 output. Toggle
  // via globalThis.VECTOR_USE_MATE_CONNECTORS for A/B or rollback.
  function useMateConnectors(): boolean {
    const g = (globalThis as unknown as { VECTOR_USE_MATE_CONNECTORS?: boolean })
      .VECTOR_USE_MATE_CONNECTORS
    return g !== false
  }
  function hasMateConnectorFields(c: AssemblyComponent): boolean {
    return !!(c.attach_connector || c.mate_connector || c.mate_type)
  }

  /**
   * Resolve a child's joint origin via the mate-connector closed-form
   * composition. Returns null to signal "fall through to legacy path" — the
   * caller uses that to preserve bit-identical bbox math whenever the new
   * fields aren't authored or the feature flag is off.
   *
   * Phase 2 scope: default face connectors only (attach_face → connector id
   * via the opposite-face convention). Authored per-preset connectors land
   * in Phase 3 alongside the problem-child presets (L-bracket, servo shaft).
   * Multi-child distribution / splay / elevation / orientation keywords stay
   * on the legacy path — the migration doc calls these out explicitly as
   * orthogonal post-passes, not resolver concerns.
   */
  function computeMatePlacement(
    comp: AssemblyComponent,
    parentPresetBboxMm: { hxMm: number; hyMm: number; hzMm: number },
    childPresetBboxMm:  { hxMm: number; hyMm: number; hzMm: number },
    parentAuthored?: MateConnector[],
    childAuthored?:  MateConnector[],
  ): { xyz: string; rpy: string } | null {
    if (!useMateConnectors()) return null
    // Only fire when the COMPONENT opts in (attach_connector/mate_connector/
    // mate_type). Authored preset connectors are vocabulary, not behavior —
    // they sit available for Claude/auto-repair to reference by name via
    // mate_connector. Auto-firing whenever a preset ships authored connectors
    // would bypass legacy splay/multi-child distribution/orientation for every
    // child of the parent, which is exactly the risk flagged by the migration
    // doc "Multi-child distribution" note.
    if (!hasMateConnectorFields(comp)) return null

    // Defaults first, authored-on-preset overrides by id (mergeConnectors contract).
    const parentDefaults = generateDefaultConnectors(parentPresetBboxMm)
    const childDefaults  = generateDefaultConnectors(childPresetBboxMm)
    const parentConnectors = mergeConnectors(parentDefaults, parentAuthored)
    const childConnectors  = mergeConnectors(childDefaults,  childAuthored)

    // Infer connector ids from attach_face when the new fields are partial.
    // "top" on the parent implies "bottom" on the child, matching the default-
    // connector naming. An explicitly authored attach_connector/mate_connector
    // wins; the legacy attach_face is only consulted as a fallback.
    const parentConnectorId = comp.attach_connector ?? comp.attach_face ?? 'top'
    const inferredChildId   = comp.attach_face ? childConnectorIdForAttachFace(comp.attach_face) : null
    const childConnectorId  = comp.mate_connector ?? inferredChildId ?? 'bottom'

    const parentConn = findConnector(parentConnectors, parentConnectorId)
    const childConn  = findConnector(childConnectors,  childConnectorId)
    if (!parentConn || !childConn) {
      // Fail loudly per the migration doc: never silently fall back to a
      // guessed connector. Return null so the caller can log + skip/error.
      console.warn(
        `[mate] connector lookup failed for ${comp.link_name}: parent="${parentConnectorId}" (${parentConn ? 'ok' : 'MISS'}), ` +
        `child="${childConnectorId}" (${childConn ? 'ok' : 'MISS'}). Falling through to legacy bbox path.`,
      )
      return null
    }

    const mateType: MateType = ((comp.mate_type as MateType) ?? 'fastened')
    if (mateType !== 'fastened' && mateType !== 'planar' && mateType !== 'concentric') {
      console.warn(`[mate] unknown mate_type="${mateType}" for ${comp.link_name}; falling through to legacy`)
      return null
    }

    const childLocal = resolveMate(new THREE.Matrix4(), parentConn, childConn, mateType, {})
    const pos = new THREE.Vector3()
    const quat = new THREE.Quaternion()
    const scl = new THREE.Vector3()
    childLocal.decompose(pos, quat, scl)
    const [r, p, y] = quatToRpy(quat)

    return {
      xyz: `${pos.x.toFixed(4)} ${pos.y.toFixed(4)} ${pos.z.toFixed(4)}`,
      rpy: `${r.toFixed(4)} ${p.toFixed(4)} ${y.toFixed(4)}`,
    }
  }

  function computeFacePlacement(
    doc: Document, parentLinkName: string,
    childX: number, childY: number, childZ: number,
    attachFace: string | null,
    isChildElongated: boolean = false,
    childIndex: number = 0,
    totalOnFace: number = 1,
    orientation: string = 'auto',
    noSplay: boolean = false,
    childComponentId: string = '',
    elevationAngleDeg: number = 0,
    childSizes?: Array<{ hu: number; hv: number }>,
    childCenterOffset: { cx: number; cy: number; cz: number } = { cx: 0, cy: 0, cz: 0 },
  ): { xyz: string; rpy: string } {
    const parent = getParentBounds(doc, parentLinkName)
    const gap = 0
    // Distance from parent's link origin to its body face along each axis.
    // For a symmetric part this equals hz; for a servo (shaft on +Z, body
    // centered at origin) it collapses to the body half-height.
    // Formula: body_face_distance = axis_half_extent - |axis_center_offset|.
    const parentBodyHX = parent.hx - Math.abs(parent.cx)
    const parentBodyHY = parent.hy - Math.abs(parent.cy)
    const parentBodyHZ = parent.hz - Math.abs(parent.cz)
    // Child's link-origin-to-body-face distance along each axis. Lets the
    // child's body (not the tip of an off-center protrusion) sit flush.
    const childBodyHX = childX / 2 - Math.abs(childCenterOffset.cx)
    const childBodyHY = childY / 2 - Math.abs(childCenterOffset.cy)
    const childBodyHZ = childZ / 2 - Math.abs(childCenterOffset.cz)
    console.log(`[placement] ${childComponentId || '?'} on ${parentLinkName} face=${attachFace || 'top'} | parent hx=${parent.hx.toFixed(4)} hy=${parent.hy.toFixed(4)} hz=${parent.hz.toFixed(4)} | child ${childX.toFixed(4)}×${childY.toFixed(4)}×${childZ.toFixed(4)}`)

    const face = attachFace || 'top'

    // ── 1a/1d: Pre-compute splay and splay-aware inset for bottom-face legs ──
    // Hoist isWheel so it's visible inside the switch below.
    const isWheel = childComponentId.includes('wheel') || childComponentId.includes('caster')
    let splayAngle = 0
    let insetOverride: number | undefined
    if (face === 'bottom' && !isWheel && !noSplay && totalOnFace >= 2) {
      splayAngle = splayAngleForLegCount(totalOnFace)
      // Shrink the corner inset proportionally so post-splay tips stay within parent footprint.
      // At 0 splay inset=0.7; at ~40° (max) inset≈0.51.
      insetOverride = Math.max(0.4, 0.7 - (splayAngle / (Math.PI / 2)) * 0.3)
    }

    // ── Multi-child tangential offsets (uses splay-corrected inset on bottom face) ──
    let tu = 0, tv = 0
    if (totalOnFace > 1) {
      const offsets = _computeMultiChildOffsets(totalOnFace, childIndex, parent, face, insetOverride, childSizes)
      tu = offsets.u
      tv = offsets.v
    }

    // ── 1c: Numeric orientation — yaw rotation around the face normal ──
    // If orientation is a number string (e.g. '45'), treat it as degrees of yaw on the face.
    const orientDeg = parseFloat(orientation)
    const hasNumericOrient = !isNaN(orientDeg) && orientDeg !== 0

    // ── 1c: Horizontal/vertical keyword handling (elongated components on top face) ──
    let shouldRotateHorizontal = false
    if (isChildElongated) {
      if (orientation === 'horizontal') {
        shouldRotateHorizontal = true
      }
      // 'vertical' and 'auto' keep shouldRotateHorizontal = false (no accumulated rotation)
    }

    if (shouldRotateHorizontal && face === 'top') {
      // Pitch-90° only helps when the long axis is Z (e.g. vertical extrusions). For
      // components whose long axis is already X or Y (batteries, sensor packs), pitching
      // stands them up — fall through to the normal 'top' case, applying just yaw.
      // Use sortedDims.indexOf(longest) so a tied axis (e.g. childX === childZ) resolves
      // to the original index of the first match, not Z.
      const dims = [childX, childY, childZ]
      const sortedDims = [...dims].sort((a, b) => a - b)
      const longest = sortedDims[2]
      const longestAxisIdx = dims.indexOf(longest)
      if (longestAxisIdx === 2) {
        // Pitch 90° swings X onto Z — use childX as the vertical extent.
        const vExtent = verticalExtentForRotation(childX, childY, childZ, 0, Math.PI / 2)
        const oz = parent.hz + vExtent / 2 + gap
        const yaw = hasNumericOrient ? ` ${(orientDeg * Math.PI / 180).toFixed(4)}` : ' 0'
        return { xyz: `${tu.toFixed(4)} ${tv.toFixed(4)} ${oz.toFixed(4)}`, rpy: `0 1.5708${yaw}` }
      }
    }

    // ── 1b: Elevation angle for side faces (degrees → radians) ──
    const elevRad = elevationAngleDeg * (Math.PI / 180)

    // ── Face normal offset + tangential multi-child offset ──
    // Normal-direction offsets use BODY half-extents (hz - |cz|, etc.), so asymmetric
    // protrusions (servo shaft) don't inflate the stack spacing and leave visible gaps.
    // Tangential offsets (tu, tv) and face-distribution still use AABB half-extents — a
    // protrusion's footprint is real when arranging multiple children.
    switch (face) {
      case 'top': {
        const oz = parentBodyHZ + childBodyHZ + gap
        // 1c: numeric orientation → yaw (Z-rotation) on top face
        const rpy = hasNumericOrient ? `0 0 ${(orientDeg * Math.PI / 180).toFixed(4)}` : '0 0 0'
        return { xyz: `${tu.toFixed(4)} ${tv.toFixed(4)} ${oz.toFixed(4)}`, rpy }
      }
      case 'bottom': {
        // Compute rotation first — vertical extent depends on it.
        // 1a: topology-aware splay — splayAngle was pre-computed above
        let rollRad = 0
        let pitchRad = 0
        if (isWheel) {
          // Wheels need -90° roll to orient the cylinder laterally (axle along Y)
          // Standard ROS convention: rpy="-pi/2 0 0" with axis="0 0 1"
          rollRad = -Math.PI / 2
        } else if (splayAngle > 0 && (tu !== 0 || tv !== 0)) {
          // Roll tilts along X (forward/back based on tv), Pitch tilts along Y (left/right based on tu)
          rollRad  = tv > 0 ?  splayAngle : tv < 0 ? -splayAngle : 0
          pitchRad = tu > 0 ? -splayAngle : tu < 0 ?  splayAngle : 0
        }
        // 6b: numeric orientation → yaw (Z-rotation) on bottom face, matching top/side
        // behavior. AI emitting orientation:"45" on hip-abduction servos to point each
        // hip toward its corner now lands instead of being silently dropped.
        const yawRad = hasNumericOrient ? orientDeg * Math.PI / 180 : 0
        const rpyStr = `${rollRad.toFixed(4)} ${pitchRad.toFixed(4)} ${yawRad.toFixed(4)}`
        // Rotation-aware vertical extent uses body half-extents so a rolled wheel
        // or pitched bracket snaps to the body, not to a shaft/horn tip.
        const vExtent = verticalExtentForRotation(childBodyHX * 2, childBodyHY * 2, childBodyHZ * 2, rollRad, pitchRad)
        const oz = -(parentBodyHZ + vExtent / 2 + gap)
        return { xyz: `${tu.toFixed(4)} ${tv.toFixed(4)} ${oz.toFixed(4)}`, rpy: rpyStr }
      }
      case 'front': {
        // 1b: elevation_angle tilts the component upward (positive) or downward (negative)
        const zOffset = tv + (elevRad !== 0 ? parentBodyHX * Math.sin(elevRad) : 0)
        const rpy = elevRad !== 0 ? `0 ${(-elevRad).toFixed(4)} 0` : '0 0 0'
        return { xyz: `${(parentBodyHX + childBodyHX + gap).toFixed(4)} ${tu.toFixed(4)} ${zOffset.toFixed(4)}`, rpy }
      }
      case 'back': {
        const zOffset = tv + (elevRad !== 0 ? parentBodyHX * Math.sin(elevRad) : 0)
        // Back face pitches the opposite direction (component faces -X, so positive pitch is still up)
        const rpy = elevRad !== 0 ? `0 ${elevRad.toFixed(4)} 0` : '0 0 0'
        return { xyz: `${(-(parentBodyHX + childBodyHX + gap)).toFixed(4)} ${tu.toFixed(4)} ${zOffset.toFixed(4)}`, rpy }
      }
      case 'right': {
        const zOffset = tv + (elevRad !== 0 ? parentBodyHY * Math.sin(elevRad) : 0)
        // Right face: elevation is a roll about X
        const rpy = elevRad !== 0 ? `${elevRad.toFixed(4)} 0 0` : '0 0 0'
        return { xyz: `${tu.toFixed(4)} ${(parentBodyHY + childBodyHY + gap).toFixed(4)} ${zOffset.toFixed(4)}`, rpy }
      }
      case 'left': {
        const zOffset = tv + (elevRad !== 0 ? parentBodyHY * Math.sin(elevRad) : 0)
        // Left face: elevation is an inverted roll about X
        const rpy = elevRad !== 0 ? `${(-elevRad).toFixed(4)} 0 0` : '0 0 0'
        return { xyz: `${tu.toFixed(4)} ${(-(parentBodyHY + childBodyHY + gap)).toFixed(4)} ${zOffset.toFixed(4)}`, rpy }
      }
      default:
        return { xyz: `0 0 ${(parentBodyHZ + childBodyHZ + gap).toFixed(4)}`, rpy: '0 0 0' }
    }
  }

  /** Map a face name to its two tangent half-extents (U and V axes on that face). */
  function faceUVHalfExtents(b: { hx: number; hy: number; hz: number }, face: string): { hu: number; hv: number } {
    switch (face) {
      case 'top': case 'bottom': return { hu: b.hx, hv: b.hy }
      case 'front': case 'back': return { hu: b.hy, hv: b.hz }
      case 'left': case 'right': return { hu: b.hx, hv: b.hz }
      default: return { hu: b.hx, hv: b.hy }
    }
  }

  /** Build a preset with length_mm override applied to bounding_box_mm (for extrusions). */
  function buildVisPreset(preset: PresetComponent, comp: { length_mm?: number }): PresetComponent {
    const phys = preset.physical
    const bb = phys.bounding_box_mm ?? phys.cross_section_mm ?? [40, 40, 40]
    if (comp.length_mm && phys.cross_section_mm) {
      return { ...preset, physical: { ...phys, bounding_box_mm: [bb[0] ?? 40, bb[1] ?? 40, comp.length_mm] } } as PresetComponent
    }
    return preset
  }

  /**
   * Build the resolved positions array for all children on a face.
   * Cached per face group key to avoid recomputing for each child.
   */
  const _multiChildPositionsCache = new Map<string, Array<{ u: number; v: number }>>()

  function _buildMultiChildPositions(
    total: number,
    parent: { hx: number; hy: number; hz: number },
    face: string,
    inset: number,
    childSizes?: Array<{ hu: number; hv: number }>,
  ): Array<{ u: number; v: number }> {
    // Cache key: deterministic for same inputs
    const cacheKey = `${total}:${face}:${parent.hx},${parent.hy},${parent.hz}:${inset}:${childSizes ? childSizes.map(s => `${s.hu},${s.hv}`).join(';') : ''}`
    const cached = _multiChildPositionsCache.get(cacheKey)
    if (cached) return cached

    const { hu: extU, hv: extV } = faceUVHalfExtents(parent, face)
    let positions: Array<{ u: number; v: number }>

    if (total === 2) {
      positions = [
        { u: -inset * extU, v: 0 },
        { u: inset * extU, v: 0 },
      ]
    } else if (total === 3) {
      positions = [
        { u: 0, v: inset * extV },
        { u: -inset * extU, v: -inset * 0.5 * extV },
        { u: inset * extU, v: -inset * 0.5 * extV },
      ]
    } else if (total === 4) {
      positions = [
        { u: inset * extU, v: inset * extV },
        { u: -inset * extU, v: inset * extV },
        { u: inset * extU, v: -inset * extV },
        { u: -inset * extU, v: -inset * extV },
      ]
    } else if (total === 6) {
      positions = []
      for (let i = 0; i < 6; i++) {
        const col = i % 3
        const row = Math.floor(i / 3)
        positions.push({
          u: (col - 1) * inset * extU,
          v: (row === 0 ? inset : -inset) * extV,
        })
      }
    } else {
      positions = []
      const step = (2 * inset * extU) / Math.max(total - 1, 1)
      for (let i = 0; i < total; i++) {
        positions.push({ u: -inset * extU + i * step, v: 0 })
      }
    }

    // ── Overlap resolution: push apart positions that would cause child AABBs to clip ──
    if (childSizes && childSizes.length === total) {
      const margin = 0.002
      for (let pass = 0; pass < 3; pass++) {
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
            }
          }
        }
      }
      console.log(`[placement] Multi-child overlap resolution: ${total} children, positions:`, positions.map((p, i) => `${i}:(${p.u.toFixed(4)},${p.v.toFixed(4)}) size(${childSizes[i].hu.toFixed(4)},${childSizes[i].hv.toFixed(4)})`))
    }

    _multiChildPositionsCache.set(cacheKey, positions)
    return positions
  }

  /**
   * Compute tangential UV offset for a single child on a shared face.
   * Delegates to _buildMultiChildPositions (cached) and returns the position for this index.
   */
  function _computeMultiChildOffsets(
    total: number, index: number,
    parent: { hx: number; hy: number; hz: number },
    face: string,
    insetOverride?: number,
    childSizes?: Array<{ hu: number; hv: number }>,
  ): { u: number; v: number } {
    const inset = insetOverride ?? 0.7
    const safeIndex = Math.min(index, Math.max(total - 1, 0))
    const positions = _buildMultiChildPositions(total, parent, face, inset, childSizes)
    return positions[safeIndex] || { u: 0, v: 0 }
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
   *  Prefers actual rendered mesh dims (from loaded GLB) over parametric URDF primitives
   *  so ghost bounds and node positions derive from the same geometry source.
   *  Returns half-extents, center offset, and dominant shape for the carry ghost. */
  function computeCarryGhostBounds(comp: PresetComponent): { hx: number; hy: number; hz: number; cx: number; cy: number; cz: number; shape: 'box' | 'cylinder' } {
    // Fix 1+2: use actual rendered mesh size when the GLB has been loaded and cached.
    // This ensures ghost bounds agree with the real visual geometry rather than URDF primitives.
    const renderedDims = getRenderedMeshDims(comp.id)
    if (renderedDims && renderedDims.x > 0.001) {
      return {
        hx: renderedDims.x / 2,
        hy: renderedDims.y / 2,
        hz: renderedDims.z / 2,
        cx: 0, cy: 0, cz: 0,
        shape: 'box',
      }
    }

    // Fallback: derive from parametric URDF primitive definitions
    const catName = findCategory(comp)
    const visuals = generateVisuals(comp as Parameters<typeof generateVisuals>[0], catName)
    let minX = Infinity, maxX = -Infinity
    let minY = Infinity, maxY = -Infinity
    let minZ = Infinity, maxZ = -Infinity
    let allCylinders = visuals.length > 0
    for (const vis of visuals) {
      const [ox, oy, oz] = vis.origin_xyz
      const g = vis.geometry
      let ex = 0, ey = 0, ez = 0
      if (g.type === 'box') { ex = g.size[0] / 2; ey = g.size[1] / 2; ez = g.size[2] / 2; allCylinders = false }
      else if (g.type === 'cylinder') { ex = g.radius; ey = g.radius; ez = g.length / 2 }
      else if (g.type === 'sphere') { ex = g.radius; ey = g.radius; ez = g.radius; allCylinders = false }
      minX = Math.min(minX, ox - ex); maxX = Math.max(maxX, ox + ex)
      minY = Math.min(minY, oy - ey); maxY = Math.max(maxY, oy + ey)
      minZ = Math.min(minZ, oz - ez); maxZ = Math.max(maxZ, oz + ez)
    }
    const shape: 'box' | 'cylinder' = allCylinders ? 'cylinder' : 'box'
    if (!isFinite(minX)) {
      const bb = comp.physical.bounding_box_mm ?? comp.physical.cross_section_mm ?? [40, 40, 40]
      return { hx: (bb[0] ?? 40) / 2000, hy: (bb[1] ?? 40) / 2000, hz: (bb[2] ?? 40) / 2000, cx: 0, cy: 0, cz: 0, shape }
    }
    return {
      hx: (maxX - minX) / 2,
      hy: (maxY - minY) / 2,
      hz: (maxZ - minZ) / 2,
      cx: (maxX + minX) / 2,
      cy: (maxY + minY) / 2,
      cz: (maxZ + minZ) / 2,
      shape,
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

  function addCollisionElement(doc: Document, link: Element, vis: UrdfVisualDesc) {
    const collision = doc.createElement('collision')
    const co = doc.createElement('origin')
    co.setAttribute('xyz', vis.origin_xyz.map(v => v.toFixed(6)).join(' '))
    co.setAttribute('rpy', vis.origin_rpy.map(v => v.toFixed(6)).join(' '))
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
    collision.appendChild(co)
    collision.appendChild(geometry)
    link.appendChild(collision)
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
      visuals.forEach(vis => addCollisionElement(doc, link, vis))

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

  // ── Carry mode ───────────────────────────────────────────────────────────────

  let carryComp: PresetComponent | null = null
  let carryGroup: THREE.Group | null = null
  let carryGhostBounds: { hx: number; hy: number; hz: number; cx: number; cy: number; cz: number; shape: 'box' | 'cylinder' } | null = null
  let carryWorldPos = new THREE.Vector3()
  let carryFrozen = false          // true after manual nudge — mouse no longer drives position
  let carryUserAngle = 0           // accumulated user rotation in radians
  let carryUserAxis: 'x' | 'y' | 'z' = 'z'  // local axis to rotate around (z = face normal)
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

  // ── Carry rotation arc indicator ─────────────────────────────────────────────
  const carryArcMat = new THREE.LineBasicMaterial({
    color: 0xffcc44, transparent: true, opacity: 0.85, depthTest: false,
  })
  let carryArcLine: THREE.Line | null = null

  function updateCarryArc() {
    if (carryArcLine) {
      carryGroup?.remove(carryArcLine)
      carryArcLine.geometry.dispose()
      carryArcLine = null
    }
    if (!carryGroup || carryUserAngle === 0 || !carryGhostBounds) return

    const { hx, hy, hz, cx, cy, cz } = carryGhostBounds
    const r = Math.min(hx, hy, hz) * 0.85

    // Build arc points in the plane perpendicular to the rotation axis.
    // Wedge shape: center → ref-point → arc → center.
    const steps = Math.max(2, Math.ceil(Math.abs(carryUserAngle) / (Math.PI / 24)))
    const pts: THREE.Vector3[] = [new THREE.Vector3(0, 0, 0)]  // center spoke start
    for (let i = 0; i <= steps; i++) {
      const a = (carryUserAngle / steps) * i
      if (carryUserAxis === 'z') {
        pts.push(new THREE.Vector3(Math.cos(a) * r, Math.sin(a) * r, 0))
      } else if (carryUserAxis === 'x') {
        pts.push(new THREE.Vector3(0, Math.cos(a) * r, Math.sin(a) * r))
      } else {
        pts.push(new THREE.Vector3(Math.cos(a) * r, 0, Math.sin(a) * r))
      }
    }
    pts.push(new THREE.Vector3(0, 0, 0))  // close back to center

    const geo = new THREE.BufferGeometry().setFromPoints(pts)
    carryArcLine = new THREE.Line(geo, carryArcMat)
    carryArcLine.renderOrder = 1000
    carryArcLine.position.set(cx, cy, cz)
    carryGroup.add(carryArcLine)
  }

  function computeCarryUserQuat(): THREE.Quaternion {
    if (carryUserAngle === 0) return new THREE.Quaternion()
    const axis = carryUserAxis === 'x' ? new THREE.Vector3(1, 0, 0)
      : carryUserAxis === 'y' ? new THREE.Vector3(0, 1, 0)
      : new THREE.Vector3(0, 0, 1)
    return new THREE.Quaternion().setFromAxisAngle(axis, carryUserAngle)
  }

  // Apply the current userQuat to carryGroup using the snap base orientation (if snapped)
  // or free orientation (if not). Call after changing carryUserAngle or carryUserAxis.
  function applyCarryUserRotation() {
    if (!carryGroup) return
    if (carryBestMount) {
      const snapPos = new THREE.Vector3(); const snapQuat = new THREE.Quaternion()
      carryBestMount.desiredGhostWorld.clone().decompose(snapPos, snapQuat, new THREE.Vector3())
      carryGroup.position.copy(snapPos)
      carryGroup.quaternion.copy(snapQuat.clone().multiply(computeCarryUserQuat()))
    } else {
      carryGroup.quaternion.copy(computeCarryUserQuat())
    }
    carryGroup.updateMatrixWorld(true)
    updateCarryArc()
  }

  function updateCarryHudText() {
    if (!carryComp || !carryGroup) return
    const p = carryGroup.position
    const rotDeg = Math.round(carryUserAngle * 180 / Math.PI)
    const rotLabel = rotDeg === 0 ? `0°` : `${rotDeg > 0 ? '+' : ''}${rotDeg}°${carryUserAxis.toUpperCase()}`
    let snapHint = ''
    if (carryBestMount) {
      const n = carrySnapCandidates.length
      const idxLabel = n > 1 ? ` [${carrySnapIdx + 1}/${n}]` : ''
      snapHint = `Snap→${carryBestMount.targetParentLink}${idxLabel}  `
    }
    const tabHint = carrySnapCandidates.length > 1 ? '  Tab=cycle' : ''
    const snapHint2 = snapRadiusM !== 0.05 ? `  snap:${(snapRadiusM * 100).toFixed(0)}cm` : ''
    setCarryHud(
      `${snapHint}${carryComp.name}  x:${p.x.toFixed(3)} y:${p.y.toFixed(3)} z:${p.z.toFixed(3)}  rot:${rotLabel}${snapHint2}` +
      `${tabHint}  ←→↑↓ nudge  R/Shift+R ±15°  X/Y/Z axis  [/] snap radius  Enter commit  Esc cancel`
    )
  }

  function enterCarryMode(comp: PresetComponent) {
    if (carryGroup) exitCarryMode()
    // Deselect any active link so the gizmo is detached and OrbitControls
    // are guaranteed enabled before we take over mouse handling.
    if (selectedLink) selectLink(null)
    ctx.controls.enabled = true
    carryComp = comp
    carryFrozen = false
    carryUserAngle = 0
    carryUserAxis = 'z'
    carrySnapCandidates = []
    carrySnapIdx = 0

    const bounds = computeCarryGhostBounds(comp)
    carryGhostBounds = bounds
    const { hx, hy, hz, cx, cy, cz, shape } = bounds

    // Fix 4: use geometry that matches the component's dominant shape.
    // CylinderGeometry axis is along Y in Three.js; radius = max(hx,hy), height = hz*2.
    const geo: THREE.BufferGeometry = shape === 'cylinder'
      ? new THREE.CylinderGeometry(Math.max(hx, hy), Math.max(hx, hy), hz * 2, 32)
      : new THREE.BoxGeometry(hx * 2, hy * 2, hz * 2)
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
    if (carryArcLine) { carryArcLine.geometry.dispose(); carryArcLine = null }
    if (carryGroup) { ctx.scene.remove(carryGroup); carryGroup = null }
    carryComp = null
    carryGhostBounds = null
    carryBestMount = null
    carryFrozen = false
    carryUserAngle = 0
    carryUserAxis = 'z'
    carrySnapCandidates = []
    carrySnapIdx = 0
    nodesGroup.visible = false
    ghostGroup.visible = false
    clearBestCandidateHighlight()
    applyNodeRingVisibility()
    setCarryHud('')
    ctx.canvas.style.cursor = ''
    // Re-enable OrbitControls in case a gizmo drag left them disabled
    // (the gizmo dragging-changed handler fires on drag-end, but can be missed
    // if carry mode was entered mid-drag or Escape interrupted a drag).
    ctx.controls.enabled = true
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
        if (dist > snapRadiusM) continue
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
      carryGroup.position.copy(snapPos)
      carryGroup.quaternion.copy(snapQuat.clone().multiply(computeCarryUserQuat()))
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
      carryGroup.quaternion.copy(computeCarryUserQuat())
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
    // Capture actual ghost world (includes user rotation) before exitCarryMode clears carryGroup.
    const ghostWorldFree = !mount && carryGroup ? carryGroup.matrixWorld.clone() : null
    const ghostWorldSnap = mount && carryGroup ? carryGroup.matrixWorld.clone() : null
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
      // Use carryGroup's actual matrixWorld (which already includes userQuat).
      const childLocal = parentWorldInv.clone().multiply(ghostWorldSnap ?? mount.desiredGhostWorld)
      const localPos = new THREE.Vector3().setFromMatrixPosition(childLocal)
      const localQuat = new THREE.Quaternion()
      childLocal.decompose(new THREE.Vector3(), localQuat, new THREE.Vector3())
      const [lr, lp, ly] = quatToRpy(localQuat)
      addComponentCore(comp, mount.targetParentLink,
        `${fmt(localPos.x)} ${fmt(localPos.y)} ${fmt(localPos.z)}`,
        `${fmt(lr)} ${fmt(lp)} ${fmt(ly)}`)
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
      const [lr, lp, ly] = quatToRpy(localQuat)
      addComponentCore(comp, parent,
        `${fmt(localPos.x)} ${fmt(localPos.y)} ${fmt(localPos.z)}`,
        `${fmt(lr)} ${fmt(lp)} ${fmt(ly)}`)
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
    const mounting = ((comp.mounting_logic as Record<string, unknown>).primary ?? '—') as string

    const me = comp.mechanical_electrical
    const specs = Object.entries(me).slice(0, 6).map(([k, v]) => {
      const label = k.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase())
      const val = Array.isArray(v) ? v.join(' – ') : String(v)
      return `<div class="tb-kv"><span class="tb-kv-key">${label}</span><span class="tb-kv-val">${val}</span></div>`
    }).join('')

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
    compItemEls.clear()

    for (const [catName, cat] of Object.entries(presetData.categories)) {
      const comps = cat.components.filter(c => {
        if (!hasMeshOverride(c.id)) return false
        if (SLOW_MESH_BLACKLIST.has(c.id)) return false
        return !q || c.name.toLowerCase().includes(q) || c.id.toLowerCase().includes(q) || c.description.toLowerCase().includes(q)
      })
      if (comps.length === 0) continue

      const catLabel = catName.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase())
      const catEl = document.createElement('div')
      catEl.className = 'tb-cat'
      const iconSvg = CATEGORY_ICONS[catName] ?? ''
      catEl.innerHTML = `
        <span class="tb-cat-icon">${iconSvg}</span>
        <span class="tb-cat-label">${catLabel}</span>
        <span class="tb-cat-count">${comps.length}</span>
        <span class="tb-cat-arrow">▾</span>
      `
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
      const iconColor = `rgb(${Math.round(cc[0]*255)},${Math.round(cc[1]*255)},${Math.round(cc[2]*255)})`

      for (const comp of comps) {
        const el = document.createElement('div')
        el.className = 'tb-item'
        if (mountIsCompatible(comp)) el.classList.add('compatible')
        const spec = getCompactSpec(comp)
        el.innerHTML = `
          <span class="tb-cat-icon tb-item-icon" style="color:${iconColor}">${iconSvg}</span>
          <div class="tb-item-info">
            <div class="tb-item-name">${comp.name}</div>
            <div class="tb-item-meta">${spec}</div>
          </div>
        `
        compItemEls.set(comp.id, el)

        el.addEventListener('click', () => {
          if (ctx.getInteractionMode() === 'inspect') {
            ctx.showToast('Switch to Build mode to place components', 'info')
            return
          }
          if (ctx.isSimActive?.()) {
            ctx.showToast('Exit simulation before placing components', 'info')
            return
          }
          compItems!.querySelectorAll('.tb-item').forEach(i => i.classList.remove('selected'))
          el.classList.add('selected')
          renderComponentDetail(comp)
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
      // Log the connector setup per preset — 6 default face connectors plus
      // any Phase 3 authored connectors (shaft_out, plate_top, wall_inner,
      // mount_back, etc.). This is the load-time signal that JSON authoring
      // took effect; presets with authored connectors stand out in the log
      // so "did my preset edit reach the runtime?" is obvious at startup.
      if (useMateConnectors()) {
        for (const cat of Object.values(data.categories)) {
          for (const comp of cat.components) {
            const authored = comp.connectors?.length ?? 0
            if (authored > 0) {
              const ids = comp.connectors!.map(c => c.id).join(', ')
              console.log(`[mate] connectors: 6 defaults + ${authored} authored = ${6 + authored} (${comp.id}) — authored: [${ids}]`)
            } else {
              console.log(`[mate] connectors: 6 defaults (${comp.id})`)
            }
          }
        }
      }
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
    try { ctx.groundAssembly?.() } catch (e) { console.warn('[assembly] groundAssembly failed:', e) }
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
    // Carry mode owns the interaction; ignore gizmo drag events while it is active.
    if (carryComp) { ctx.controls.enabled = true; return }
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
        const [nlr, nlp, nly] = quatToRpy(newLocalQuat)

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
          origin.setAttribute('rpy', `${fmt(nlr)} ${fmt(nlp)} ${fmt(nly)}`)


          // Re-express joint axis in new frame so actuated joints spin correctly.
          reconcileJointAxis(jointEl, documentXml, oldPivotWorldQuat, parentWorldQuat, newLocalQuat)

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

      pivot.updateMatrixWorld(true)
      const parentObj = pivot.parent
      if (!parentObj) {
        selectLink(selectedLink)
        return
      }
      parentObj.updateMatrixWorld(true)

      const parentWorldInv = parentObj.matrixWorld.clone().invert()
      const childLocal = parentWorldInv.clone().multiply(pivot.matrixWorld.clone())

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

      if (gizmo.mode !== 'translate') {
        // Rotate mode — persist gizmo rotation to URDF joint origin rpy + reconcile axis.
        const newLocalQuat = new THREE.Quaternion()
        childLocal.decompose(new THREE.Vector3(), newLocalQuat, new THREE.Vector3())
        const [nlr, nlp, nly] = quatToRpy(newLocalQuat)

        // Skip trivial drags (pure pivot-point adjustment with no real rotation).
        const oldRpy = parseNums(o.getAttribute('rpy') || '0 0 0', 3)
        if (Math.abs(oldRpy[0] - nlr) + Math.abs(oldRpy[1] - nlp) + Math.abs(oldRpy[2] - nly) < 1e-9) {
          selectLink(selectedLink)
          return
        }

        // Old joint world orientation from drag-start snapshot.
        const oldJointWorldQ = new THREE.Quaternion()
        gizmoBasePivotWorld.decompose(new THREE.Vector3(), oldJointWorldQ, new THREE.Vector3())
        const parentWorldQ = new THREE.Quaternion()
        parentObj.matrixWorld.decompose(new THREE.Vector3(), parentWorldQ, new THREE.Vector3())

        const ok = commitUrdf(documentXml => {
          const jointEl = documentXml.querySelector(`joint[name="${parentJoint.name}"]`)
          if (!jointEl) return false
          const origin = ensureOrigin(jointEl, documentXml)
          // Write both xyz and rpy — position can shift slightly during a rotate drag.
          origin.setAttribute('xyz', `${fmt(newLocalPos.x)} ${fmt(newLocalPos.y)} ${fmt(newLocalPos.z)}`)
          origin.setAttribute('rpy', `${fmt(nlr)} ${fmt(nlp)} ${fmt(nly)}`)
          reconcileJointAxis(jointEl, documentXml, oldJointWorldQ, parentWorldQ, newLocalQuat)
          return true
        }, { defer: true })
        if (ok) ctx.showToast(`Rotated ${selectedLink} (joint origin updated)`, 'success')
        selectLink(selectedLink)
        return
      }

      // Translate mode — write position only.
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
        carryUserAngle += e.shiftKey ? -Math.PI / 12 : Math.PI / 12  // ±15°
        carryFrozen = true
        applyCarryUserRotation()
        updateCarrySnap()
        updateCarryHudText()
        return
      }
      if (k === '[' || k === ']') {
        e.preventDefault()
        const delta = k === '[' ? -0.01 : 0.01
        snapRadiusM = Math.max(0.01, Math.min(0.30, snapRadiusM + delta))
        updateCarrySnap()
        updateCarryHudText()
        return
      }
      if ((k === 'x' || k === 'y' || k === 'z') && !e.ctrlKey && !e.metaKey) {
        e.preventDefault()
        if (carryUserAxis !== k) {
          carryUserAxis = k as 'x' | 'y' | 'z'
          carryUserAngle = 0  // reset angle when switching axis
        }
        carryFrozen = true
        applyCarryUserRotation()
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
          carryGroup!.position.copy(snapPos)
          carryGroup!.quaternion.copy(snapQuat.clone().multiply(computeCarryUserQuat()))
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

  // ── AI Assembly Graph Resolver ───────────────────────────────────────────────
  const GRAPH_STORAGE_KEY = 'vector_assembly_graphs'
  let _lastAssemblyGraph: AssemblyGraph | null = null

  // Restore stored graph for current file on init
  try {
    const fileName = ctx.getActiveFileName?.() || 'robot.urdf'
    const stored = JSON.parse(localStorage.getItem(GRAPH_STORAGE_KEY) || '{}')
    if (stored[fileName]) {
      _lastAssemblyGraph = stored[fileName]
      console.log(`[assembly] Restored stored graph for "${fileName}" (${_lastAssemblyGraph!.components.length} components)`)
    }
  } catch { /* localStorage parse error — ignore */ }

  function _persistGraph(graph: AssemblyGraph | null) {
    try {
      const fileName = ctx.getActiveFileName?.() || 'robot.urdf'
      const stored = JSON.parse(localStorage.getItem(GRAPH_STORAGE_KEY) || '{}')
      if (graph) {
        stored[fileName] = graph
      } else {
        delete stored[fileName]
      }
      localStorage.setItem(GRAPH_STORAGE_KEY, JSON.stringify(stored))
    } catch { /* localStorage full or unavailable — non-critical */ }
  }

  function resolveAssemblyGraph(graph: AssemblyGraph): { urdf: string | null; topologyErrors?: string[]; topologyWarnings?: string[] } {
    _multiChildPositionsCache.clear()
    console.log('[assembly] Resolving assembly graph:', JSON.stringify(graph, null, 2))
    console.log(`[assembly] ${graph.components.length} components, base_link: ${graph.base_link}`)
    if (!presetData) {
      console.error('[assembly] Presets not loaded')
      ctx.showToast('Component presets not loaded yet', 'error')
      return { urdf: null }
    }

    // Find preset component by id
    function findPreset(componentId: string): PresetComponent | null {
      for (const cat of Object.values(presetData!.categories)) {
        for (const comp of cat.components) {
          if (comp.id === componentId) return comp
        }
      }
      return null
    }

    // ── Validation + auto-repair (delegated to pure topologyValidation module) ──
    const validationCtx = {
      findPreset: (id: string): ValidationPreset | null => findPreset(id) as ValidationPreset | null,
    }

    console.log(`[assembly][autorepair] Scanning ${graph.components.length} components for auto-repairable issues...`)
    console.log(`[assembly][autorepair] Input link_names: [${graph.components.map(c => c.link_name).join(', ')}]`)
    const { repairs } = runAutoRepair(graph, validationCtx)
    if (repairs.length > 0) {
      for (const r of repairs) console.log(`[assembly][autorepair] ${r.kind}: ${r.message}`)
      console.log(`[assembly][autorepair] Post-repair topology: ${graph.components.map(c => `${c.link_name}→${c.attach_to || 'ROOT'}`).join(', ')}`)
    } else {
      console.log(`[assembly][autorepair] No repairs needed — topology clean`)
    }

    const { errors: topologyErrors, warnings: topologyWarnings } = runValidateTopology(graph.components, validationCtx)
    if (topologyWarnings.length > 0) {
      for (const w of topologyWarnings) console.warn(`[assembly][topology][warning] ${w}`)
    }
    if (topologyErrors.length > 0) {
      console.error('[assembly] Topology validation failed:', topologyErrors)
      ctx.showToast(`Invalid topology: ${topologyErrors[0]}`, 'error')
      return { urdf: null, topologyErrors, topologyWarnings: topologyWarnings.length > 0 ? topologyWarnings : undefined }
    }

    // Topological sort: process components in dependency order
    const components = [...graph.components]
    const processed = new Set<string>()
    const nameMap = new Map<string, string>() // Claude's link_name -> actual generated link_name
    let placedCount = 0

    // Process root first (attach_to === null)
    let root = components.find(c => !c.attach_to)
    if (!root) {
      // Auto-prepend baseplate if Claude referenced a base_link but didn't include it
      const baseLinkName = graph.base_link || 'structural_baseplate_1'
      console.log(`[assembly] No root component — auto-prepending baseplate as "${baseLinkName}"`)
      root = {
        link_name: baseLinkName,
        component_id: 'structural_baseplate',
        attach_to: null,
        attach_face: 'top',
        joint_type: 'fixed',
        joint_axis: 'z',
      }
      components.unshift(root)
    }

    // For the root, we need a base link. Start with an empty robot.
    const rootPreset = findPreset(root.component_id)
    if (!rootPreset) {
      ctx.showToast(`Unknown component: ${root.component_id}`, 'error')
      return { urdf: null }
    }

    // Create a fresh minimal URDF with just a base_link
    const phys = rootPreset.physical
    const bb = phys.bounding_box_mm ?? phys.cross_section_mm ?? [40, 40, 40]
    const xm = (bb[0] ?? 40) / 1000
    const ym = (bb[1] ?? 40) / 1000
    let zm = (bb[2] ?? 40) / 1000
    if (root.length_mm && phys.cross_section_mm) {
      zm = root.length_mm / 1000
    }
    const mass = phys.mass_kg ?? phys.mass_kg_per_100mm ?? 0.1
    const shape = phys.inertia_primitive || 'box'
    let inertia: { ixx: number; iyy: number; izz: number }
    if (shape === 'cylinder') inertia = computeCylinderInertia(mass, Math.max(xm, ym) / 2, zm)
    else if (shape === 'sphere') inertia = computeSphereInertia(mass, xm / 2)
    else inertia = computeBoxInertia(mass, xm, ym, zm)

    const rootLinkName = root.link_name
    const catName = findCategory(rootPreset)
    const rootVisualPreset = buildVisPreset(rootPreset, root)
    const visuals = generateVisuals(rootVisualPreset as Parameters<typeof generateVisuals>[0], catName)

    // Build root link XML
    let visualsXml = ''
    visuals.forEach((vis, i) => {
      const geomXml = vis.geometry.type === 'box'
        ? `<box size="${vis.geometry.size.map(v => v.toFixed(6)).join(' ')}"/>`
        : vis.geometry.type === 'cylinder'
        ? `<cylinder radius="${vis.geometry.radius.toFixed(6)}" length="${vis.geometry.length.toFixed(6)}"/>`
        : `<sphere radius="${vis.geometry.radius.toFixed(6)}"/>`
      visualsXml += `
    <visual>
      <origin xyz="${vis.origin_xyz.map(v => v.toFixed(6)).join(' ')}" rpy="${vis.origin_rpy.map(v => v.toFixed(6)).join(' ')}"/>
      <geometry>${geomXml}</geometry>
      <material name="comp_mat_${i}"><color rgba="${vis.color_rgba.map(v => v.toFixed(3)).join(' ')}"/></material>
    </visual>`
    })

    // Collision geometry — one element per visual piece for accurate hitboxes
    let collisionsXml = ''
    visuals.forEach(vis => {
      const cGeomXml = vis.geometry.type === 'box'
        ? `<box size="${vis.geometry.size.map(v => v.toFixed(6)).join(' ')}"/>`
        : vis.geometry.type === 'cylinder'
        ? `<cylinder radius="${vis.geometry.radius.toFixed(6)}" length="${vis.geometry.length.toFixed(6)}"/>`
        : `<sphere radius="${vis.geometry.radius.toFixed(6)}"/>`
      collisionsXml += `
    <collision>
      <origin xyz="${vis.origin_xyz.map(v => v.toFixed(6)).join(' ')}" rpy="${vis.origin_rpy.map(v => v.toFixed(6)).join(' ')}"/>
      <geometry>${cGeomXml}</geometry>
    </collision>`
    })

    const baseUrdf = `<?xml version="1.0"?>
<robot name="assembled_robot">
  <link name="${rootLinkName}">
    <inertial>
      <mass value="${mass.toFixed(4)}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="${inertia.ixx.toFixed(6)}" iyy="${inertia.iyy.toFixed(6)}" izz="${inertia.izz.toFixed(6)}" ixy="0" ixz="0" iyz="0"/>
    </inertial>${visualsXml}${collisionsXml}
  </link>
</robot>`

    // Set the editor to this base URDF and reparse
    const editor = (window as any).__vectorEditor
    if (!editor) return { urdf: null }
    editor.setValue(baseUrdf)
    ctx.reparseUrdf()

    // try/finally is load-bearing: if the loop throws we MUST clear _bulkMode,
    // else every future commitUrdf in the session writes to an orphaned buffer.
    setBulkAssemblyMode(true)
    try {

    nameMap.set(root.link_name, rootLinkName)
    processed.add(root.link_name)
    placedCount++
    console.log(`[assembly] Root placed: ${rootLinkName} (${root.component_id})`)

    // Pre-compute how many children attach to each parent:face pair
    // so we can distribute them (e.g., 4 wheels on bottom corners)
    const faceChildCounts = new Map<string, number>()
    const faceChildIndex = new Map<string, number>()
    // Pre-collect child UV-projected half-sizes per face group so
    // _computeMultiChildOffsets can prevent overlapping placements.
    const faceChildSizes = new Map<string, Array<{ hu: number; hv: number }>>()
    // Cache bounds from pre-pass so the main loop doesn't recompute them.
    const boundsCache = new Map<string, ReturnType<typeof computeCarryGhostBounds>>()
    for (const comp of components) {
      if (!comp.attach_to) continue
      const key = `${comp.attach_to}:${comp.attach_face || 'top'}`
      faceChildCounts.set(key, (faceChildCounts.get(key) || 0) + 1)
      faceChildIndex.set(key, 0) // will increment as we place

      const cPreset = findPreset(comp.component_id)
      if (cPreset) {
        const cb = computeCarryGhostBounds(buildVisPreset(cPreset, comp))
        boundsCache.set(comp.link_name, cb)
        const { hu, hv } = faceUVHalfExtents(cb, comp.attach_face || 'top')
        if (!faceChildSizes.has(key)) faceChildSizes.set(key, [])
        faceChildSizes.get(key)!.push({ hu, hv })
      } else {
        if (!faceChildSizes.has(key)) faceChildSizes.set(key, [])
        faceChildSizes.get(key)!.push({ hu: 0.02, hv: 0.02 })
      }
    }
    console.log('[assembly] Face child distribution:', Object.fromEntries(faceChildCounts))

    // Track arm chain depth: how many revolute-Y-on-top joints in sequence
    // Used to apply default rest pose angles (shoulder=45°, elbow=-90°)
    const armDepth = new Map<string, number>()
    armDepth.set(root.link_name, 0)

    // Track port occupancy: how many children are connected to each parent port
    // Key format: "parent_link_name::face_name"
    const portOccupancy = new Map<string, number>()

    // Opposite face mapping: child's contact surface is opposite of parent's attach face
    const oppositeFace: Record<string, string> = {
      top: 'bottom', bottom: 'top',
      front: 'back', back: 'front',
      left: 'right', right: 'left',
    }

    // Now iterate remaining components in dependency order
    const remaining = components.filter(c => c.attach_to !== null)
    let maxIter = remaining.length * 2 // safety valve
    while (remaining.length > 0 && maxIter-- > 0) {
      const nextIdx = remaining.findIndex(c => processed.has(c.attach_to!))
      if (nextIdx === -1) break // no more placeable components

      const comp = remaining.splice(nextIdx, 1)[0]
      const preset = findPreset(comp.component_id)
      if (!preset) {
        console.warn(`[assembly] Unknown component: ${comp.component_id}, skipping`)
        processed.add(comp.link_name)
        continue
      }

      const parentLinkName = nameMap.get(comp.attach_to!) || comp.attach_to!

      // Select the parent link so addComponentCore attaches to it
      selectLink(parentLinkName)

      // Use cached bounds from pre-pass when available, otherwise compute.
      const cPhys = preset.physical
      const childBounds = boundsCache.get(comp.link_name) ?? computeCarryGhostBounds(buildVisPreset(preset, comp))
      const cxm = childBounds.hx * 2
      const cym = childBounds.hy * 2
      // For extrusions with per-instance length, the mesh cache may hold dims from a
      // different-length instance (cache is keyed by component ID).  Always use the
      // explicit length when specified.
      let czm = childBounds.hz * 2
      if (comp.length_mm && cPhys.cross_section_mm) {
        czm = comp.length_mm / 1000
      }

      // ── Port-based connection validation ──
      // Resolve parent and child ports, check compatibility and occupancy.
      const parentCompDef = components.find(c => c.link_name === comp.attach_to)
      const parentPreset = findPreset(parentCompDef?.component_id || '')
      const childPreset = findPreset(comp.component_id)
      const attachFace = comp.attach_face || 'top'
      const childFace = oppositeFace[attachFace] || 'bottom'

      let parentPort: ReturnType<typeof resolveFaceToPort> = undefined
      let childPort: ReturnType<typeof resolveFaceToPort> = undefined

      if (parentPreset) {
        const pPhys = parentPreset.physical
        const pBb = pPhys.bounding_box_mm ?? pPhys.cross_section_mm ?? [40, 40, 40]
        const parentPorts = componentPortsForPreset(
          parentPreset.id, (pBb[0] ?? 40) / 2000, (pBb[1] ?? 40) / 2000, (pBb[2] ?? 40) / 2000,
          parentPreset.mounting_logic
        )
        parentPort = resolveFaceToPort(attachFace, parentPorts)
        console.log(`[assembly][ports] Parent port resolved: ${parentPreset.id}.${attachFace} → ${parentPort ? `${parentPort.nodeId}(${parentPort.cls}:${parentPort.label})` : 'NOT FOUND'}`)

        if (childPreset) {
          const cPhysP = childPreset.physical
          const cBbP = cPhysP.bounding_box_mm ?? cPhysP.cross_section_mm ?? [40, 40, 40]
          const childPorts = componentPortsForPreset(
            childPreset.id, (cBbP[0] ?? 40) / 2000, (cBbP[1] ?? 40) / 2000, (cBbP[2] ?? 40) / 2000,
            childPreset.mounting_logic
          )
          childPort = resolveFaceToPort(childFace, childPorts)
          console.log(`[assembly][ports] Child port resolved: ${childPreset.id}.${childFace} → ${childPort ? `${childPort.nodeId}(${childPort.cls}:${childPort.label})` : 'NOT FOUND'}`)
        } else {
          console.warn(`[assembly][ports] Child preset not found for component_id="${comp.component_id}" — cannot resolve child port`)
        }
      } else {
        console.warn(`[assembly][ports] Parent preset not found for component_id="${parentCompDef?.component_id}" (parent of ${comp.component_id}) — cannot resolve ports`)
      }

      // Log compatibility result
      if (parentPort && childPort) {
        const compat = nodesCompatible(childPort.cls, parentPort.cls)
        // Expected-mismatch suppression: brackets and coupler discs exist precisely
        // to mate a shaft against a mount_face (shaft drives the disc via friction
        // /screws — the real mechanical interface). Auto-repair inserts these
        // intentionally, so the resulting shaft↔mount_face pair is the design
        // intent, not an error. Quiet the log line (12× spam per quadruped)
        // while still leaving compat=false for any downstream code that cares.
        const isCouplingPair = !compat
          && childPort.cls === 'shaft'
          && parentPort.cls === 'mount_face'
          && (parentPreset!.id.startsWith('structural_bracket')
              || parentPreset!.id.startsWith('structural_servo_coupler'))
        if (!isCouplingPair) {
          const reason = compat ? '' : ` — ${incompatibleReason(childPort.cls, parentPort.cls)}`
          console.log(`[assembly][ports] Connection: ${comp.component_id}(${childPort.cls}:${childPort.label}) → ${parentPreset!.id}.${parentPort.nodeId}(${parentPort.cls}:${parentPort.label}) — compatible=${compat}${reason}`)
        }
      } else {
        console.log(`[assembly][ports] Connection: ${comp.component_id} → ${comp.attach_to}.${attachFace} — port resolution incomplete (parent=${!!parentPort}, child=${!!childPort})`)
      }

      // Track per-port occupancy: enforce single-use ports
      const portOccupancyKey = `${comp.attach_to}::${attachFace}`
      const currentOccupancy = portOccupancy.get(portOccupancyKey) || 0
      console.log(`[assembly][ports] Occupancy: ${portOccupancyKey} = ${currentOccupancy} → ${currentOccupancy + 1}${parentPort?.single ? ' (single-use)' : ''}`)
      if (parentPort?.single && currentOccupancy > 0 && parentPort.cls === 'shaft') {
        console.warn(`[assembly][ports] WARNING: ${parentPreset!.id}.${parentPort.nodeId} (shaft) already has ${currentOccupancy} child(ren) — multiple children on a shaft output is unusual`)
      }
      portOccupancy.set(portOccupancyKey, currentOccupancy + 1)

      // Get multi-child placement info
      const faceKey = `${comp.attach_to}:${comp.attach_face || 'top'}`
      const totalOnFace = faceChildCounts.get(faceKey) || 1
      const childIdx = faceChildIndex.get(faceKey) || 0
      faceChildIndex.set(faceKey, childIdx + 1)

      const doc = new DOMParser().parseFromString(getCurrentUrdfText(), 'application/xml')
      const sortedDims = [cxm, cym, czm].sort((a, b) => a - b)
      const isElongated = sortedDims[2] > sortedDims[0] * 2.5 && sortedDims[1] < sortedDims[0] * 2.0
      const orientation = comp.orientation || 'auto'
      const isWheelRelated = comp.component_id.includes('wheel') || comp.component_id.includes('caster')
        || components.some(c => c.attach_to === comp.link_name && (c.component_id.includes('wheel') || c.component_id.includes('caster')))
      // Splay is a leg-tilt concept meant for extrusions/tubes standing in for legs. Skip it for
      // passive hardware (brackets, plates, sensor/compute/power blocks) — especially the
      // auto-inserted shaft↔mount_face brackets and servo coupler discs, which represent the
      // START of a leg chain (not a stance element); splaying them tilts the whole chain ~30°.
      const isPassiveHardware = comp.component_id.startsWith('structural_bracket')
        || comp.component_id.startsWith('structural_joint_plate')
        || comp.component_id.startsWith('structural_sheet')
        || comp.component_id.startsWith('structural_servo_coupler')
        || comp.component_id.startsWith('power_')
        || comp.component_id.startsWith('sensor_')
        || comp.component_id.startsWith('compute_')
      const noSplay = isWheelRelated || comp.joint_type === 'revolute' || isPassiveHardware
      const elevAngle = comp.elevation_angle ?? 0

      // Phase 2/3: if this component has authored mate-connector fields OR
      // either preset carries authored connectors, AND the feature flag is on,
      // resolve via closed-form frame composition instead of the bbox half-
      // extent math. Returns null to fall through to legacy whenever neither
      // side opts in.
      let placement: { xyz: string; rpy: string }
      const parentPhys = parentPreset?.physical
      const parentBb = parentPhys?.bounding_box_mm ?? parentPhys?.cross_section_mm ?? [40, 40, 40]
      const matePlacement = (parentPreset && childPreset)
        ? computeMatePlacement(
            comp,
            { hxMm: (parentBb[0] ?? 40) / 2, hyMm: (parentBb[1] ?? 40) / 2, hzMm: (parentBb[2] ?? 40) / 2 },
            { hxMm: cxm * 500,                hyMm: cym * 500,                hzMm: czm * 500 },
            parentPreset.connectors,
            childPreset.connectors,
          )
        : null

      if (matePlacement) {
        placement = matePlacement
        console.log(`[mate] Placed ${comp.link_name} via connector path: parent=${comp.attach_connector ?? comp.attach_face}, child=${comp.mate_connector ?? '(default)'}, type=${comp.mate_type ?? 'fastened'} → ${JSON.stringify(placement)}`)
      } else {
        placement = computeFacePlacement(doc, parentLinkName, cxm, cym, czm, comp.attach_face, isElongated, childIdx, totalOnFace, orientation, noSplay, comp.component_id, elevAngle, faceChildSizes.get(faceKey), { cx: childBounds.cx, cy: childBounds.cy, cz: childBounds.cz })
      }
      console.log(`[assembly] Placing ${comp.component_id} -> parent=${parentLinkName}, face=${comp.attach_face}, child ${childIdx+1}/${totalOnFace}, elongated=${isElongated}, orient=${orientation}, elev=${elevAngle}°, noSplay=${noSplay}, placement=${JSON.stringify(placement)}, joint=${comp.joint_type} axis=${comp.joint_axis}`)

      // Override joint type/axis from the topology
      const axisMap: Record<string, string> = { x: '1 0 0', y: '0 1 0', z: '0 0 1' }
      const jointAxis = axisMap[comp.joint_axis?.toLowerCase()] || '0 0 1'

      // Use addComponentCore but we need to override joint type and axis
      // Since addComponentCore auto-determines joint type from category,
      // we'll directly build the URDF element for more control.
      // Use placedCount (not getKinematicGraph().length): the graph only updates
      // on reparse, and bulk mode skips per-iteration reparses — so without this,
      // every child would get `_2` and produce duplicate URDF link names.
      const nextIdx2 = placedCount + 1
      const childName = `${preset.id}_${nextIdx2}`
      const jointName = `joint_${preset.id}_${nextIdx2}`

      const cMass = cPhys.mass_kg ?? cPhys.mass_kg_per_100mm ?? 0.1
      const cShape = cPhys.inertia_primitive || 'box'
      let cInertia: { ixx: number; iyy: number; izz: number }
      if (cShape === 'cylinder') cInertia = computeCylinderInertia(cMass, Math.max(cxm, cym) / 2, czm)
      else if (cShape === 'sphere') cInertia = computeSphereInertia(cMass, cxm / 2)
      else cInertia = computeBoxInertia(cMass, cxm, cym, czm)

      const cCatName = findCategory(preset)
      const visualPreset = buildVisPreset(preset, comp)
      const cVisuals = generateVisuals(visualPreset as Parameters<typeof generateVisuals>[0], cCatName)

      const changed = commitUrdf(urdfDoc => {
        const robot = urdfDoc.querySelector('robot')
        if (!robot) return false

        const link = urdfDoc.createElement('link')
        link.setAttribute('name', childName)

        const inertialEl = urdfDoc.createElement('inertial')
        const massEl = urdfDoc.createElement('mass')
        massEl.setAttribute('value', cMass.toFixed(4))
        const inertiaEl = urdfDoc.createElement('inertia')
        inertiaEl.setAttribute('ixx', cInertia.ixx.toFixed(6))
        inertiaEl.setAttribute('iyy', cInertia.iyy.toFixed(6))
        inertiaEl.setAttribute('izz', cInertia.izz.toFixed(6))
        inertiaEl.setAttribute('ixy', '0'); inertiaEl.setAttribute('ixz', '0'); inertiaEl.setAttribute('iyz', '0')
        inertialEl.appendChild(massEl); inertialEl.appendChild(inertiaEl)
        link.appendChild(inertialEl)

        cVisuals.forEach((vis, i) => addVisualElement(urdfDoc, link, vis, i))
        cVisuals.forEach(vis => addCollisionElement(urdfDoc, link, vis))

        const joint = urdfDoc.createElement('joint')
        joint.setAttribute('name', jointName)
        joint.setAttribute('type', comp.joint_type || 'fixed')

        const parentEl = urdfDoc.createElement('parent'); parentEl.setAttribute('link', parentLinkName)
        const childEl = urdfDoc.createElement('child'); childEl.setAttribute('link', childName)
        const origin = urdfDoc.createElement('origin')
        origin.setAttribute('xyz', placement.xyz)
        // Apply default arm rest pose: bend revolute-Y joints in vertical chains
        // so arms look like arms (L-shape) instead of straight poles at rest
        let finalRpy = placement.rpy
        const isArmJoint = comp.joint_type === 'revolute'
          && comp.joint_axis?.toLowerCase() === 'y'
          && comp.attach_face === 'top'
        const parentDepth = armDepth.get(comp.attach_to!) || 0
        if (isArmJoint) {
          const depth = parentDepth + 1
          armDepth.set(comp.link_name, depth)
          // Shoulder (depth 1): pitch forward 45°, Elbow (depth 2): bend back -90°
          const defaultPitch = depth === 1 ? 0.7854 : depth === 2 ? -1.5708 : 0
          if (defaultPitch !== 0) {
            const rpyParts = placement.rpy.split(' ').map(Number)
            rpyParts[1] = (rpyParts[1] || 0) + defaultPitch
            finalRpy = rpyParts.map(v => v.toFixed(4)).join(' ')
            console.log(`[assembly] Arm rest pose: ${comp.link_name} depth=${depth}, added pitch=${defaultPitch.toFixed(2)} rad`)
          }
        } else {
          // Propagate arm depth through non-revolute components (extrusions, grippers)
          // so the next revolute-Y joint gets the correct depth
          armDepth.set(comp.link_name, comp.attach_face === 'top' ? parentDepth : 0)
        }
        // Explicit attach_rpy from AI overrides all auto-computed rpy (placement + arm rest-pose).
        // Mirrors claude_client.py:583-587. Threshold matches Python's 0.001 rad (~0.057°).
        const explicitRpy = comp.attach_rpy
        if (Array.isArray(explicitRpy) && explicitRpy.length === 3
            && explicitRpy.some(v => Math.abs(v) > 0.001)) {
          finalRpy = explicitRpy.map(v => Number(v).toFixed(4)).join(' ')
          console.log(`[assembly] attach_rpy override: ${comp.link_name} rpy=[${explicitRpy.join(', ')}]`)
        }
        origin.setAttribute('rpy', finalRpy)
        const axis = urdfDoc.createElement('axis'); axis.setAttribute('xyz', jointAxis)
        joint.appendChild(parentEl); joint.appendChild(childEl); joint.appendChild(origin); joint.appendChild(axis)

        if (comp.joint_type === 'revolute' || comp.joint_type === 'prismatic') {
          const limit = urdfDoc.createElement('limit')
          limit.setAttribute('lower', '-3.14159'); limit.setAttribute('upper', '3.14159')
          const me = preset.mechanical_electrical || {}
          const maxTorque = (me.max_torque_nm as number) ?? (me.holding_torque_nm as number) ?? 10
          limit.setAttribute('effort', String(maxTorque)); limit.setAttribute('velocity', '3.14')
          joint.appendChild(limit)
        }

        robot.appendChild(link); robot.appendChild(joint)
        return true
      })

      if (changed) {
        nameMap.set(comp.link_name, childName)
        placedCount++
        console.log(`[assembly] ✓ Placed ${childName} at xyz=${placement.xyz} rpy=${placement.rpy}`)
        // Reparse so next component sees updated geometry. Skipped in bulk mode
        // (getCurrentUrdfText reads from the buffer; getParentBounds falls back
        // to URDF-visual dims when the rendered-mesh cache is stale).
        if (!_bulkMode) ctx.reparseUrdf()
      } else {
        console.warn(`[assembly] ✗ Failed to place ${comp.component_id} on ${parentLinkName}`)
      }
      processed.add(comp.link_name)
    }

    console.log(`[assembly] Done: ${placedCount}/${graph.components.length} components placed`)
    if (remaining.length > 0) {
      console.warn(`[assembly] ${remaining.length} components could not be placed:`, remaining.map(c => c.link_name))
    }

    // Port occupancy summary
    console.log(`[assembly][ports] Final occupancy:`, Object.fromEntries(portOccupancy))
    const shaftOverloads = [...portOccupancy.entries()].filter(([_key, count]) => count > 1)
    if (shaftOverloads.length > 0) {
      console.warn(`[assembly][ports] Ports with multiple children:`, shaftOverloads.map(([k, c]) => `${k}=${c}`).join(', '))
    }

    } finally {
      // Always exit bulk mode — flushes the accumulated URDF to the editor and
      // triggers the single reparse covering every component placed in the loop.
      setBulkAssemblyMode(false)
    }

    // Pretty-print the URDF so the editor doesn't show everything on one line.
    // commitUrdf uses DOM createElement which serializes without whitespace.
    const rawUrdf = ctx.getUrdfText()
    const lines = rawUrdf.replace(/></g, '>\n<').replace(/\n\n+/g, '\n').split('\n')
    let indent = 0
    const prettyUrdf = lines.map(line => {
      const trimmed = line.trim()
      if (!trimmed) return ''
      // Decrease indent for closing tags
      if (trimmed.startsWith('</')) indent = Math.max(0, indent - 1)
      const result = '  '.repeat(indent) + trimmed
      // Increase indent for opening tags (not self-closing or closing)
      if (trimmed.startsWith('<') && !trimmed.startsWith('</') && !trimmed.startsWith('<?') && !trimmed.endsWith('/>')) {
        indent++
      }
      return result
    }).join('\n')
    const ed = (window as any).__vectorEditor
    if (ed && prettyUrdf !== rawUrdf) {
      ed.setValue(prettyUrdf)
      ctx.reparseUrdf()
    }

    // Store the resolved graph with URDF link names (remapped via nameMap) so the
    // reconcile pass below + later modify_topology ops reference the same names
    // Claude sees in the URDF.
    const remappedComponents = graph.components.map(c => ({
      ...c,
      link_name: nameMap.get(c.link_name) || c.link_name,
      attach_to: c.attach_to ? (nameMap.get(c.attach_to) || c.attach_to) : null,
    }))
    const remappedBase = nameMap.get(graph.base_link) || graph.base_link
    _lastAssemblyGraph = { base_link: remappedBase, ground_offset: graph.ground_offset, components: remappedComponents }
    _persistGraph(_lastAssemblyGraph)
    console.log(`[assembly] Stored assembly graph (${remappedComponents.length} components, URDF names) for modify_topology`)

    // Render-time alignment: shift pivots so visible mesh faces meet. Must run
    // BEFORE groundAssembly because groundRobot measures post-reconcile world
    // extents. Safe when meshes haven't loaded yet — the EPS guard no-ops any
    // link whose parent/child AABB is unavailable or already aligned, and the
    // debounced onMeshLoaded path re-fires reconcile once GLBs settle.
    try {
      ctx.getParsedRobot().group.updateMatrixWorld(true)
      const reconcileRes = reconcileNodePlacement({
        graph: _lastAssemblyGraph,
        linkGroups: ctx.getParsedRobot().linkGroups,
        joints: ctx.getParsedRobot().joints,
      })
      if (reconcileRes.adjustedCount > 0) {
        // Persist the shifted pivot positions into the URDF text. Without this,
        // the debounced 500ms editor-change reparse (main.ts:743) rebuilds the
        // scene from the original (unshifted) URDF xyz values, wiping every
        // shift — and then the debounced onMeshLoaded callback re-fires reconcile
        // and replays the identical 37 shifts. Baking shifts into the text makes
        // the reparsed scene come up already-aligned, so the second reconcile
        // pass finds zero deltas (genuine no-op).
        persistReconcileShiftsToUrdf(ctx.getParsedRobot().linkGroups)
        // Mount nodes were placed against the pre-reconcile pivot positions;
        // rebuild so the attachment rings follow the real rendered geometry.
        rebuildMountNodes()
      }
    } catch (e) {
      console.warn('[assembly] reconcileNodePlacement failed:', e)
    }

    // Ground the robot so it sits on the floor plane (Y=0 in Three.js)
    try { ctx.groundAssembly?.() } catch (e) { console.warn('[assembly] groundAssembly failed:', e) }

    ctx.showToast(`Assembled ${placedCount} components`, 'success')

    return { urdf: ctx.getUrdfText(), topologyWarnings: topologyWarnings.length > 0 ? topologyWarnings : undefined }
  }

  // ── Reverse Parser: URDF → AssemblyGraph ──────────────────────────────────

  function urdfToAssemblyGraph(urdfXml: string): AssemblyGraph | null {
    try {
      const doc = new DOMParser().parseFromString(urdfXml, 'application/xml')
      if (doc.querySelector('parsererror')) return null

      const links = Array.from(doc.querySelectorAll('link'))
      const joints = Array.from(doc.querySelectorAll('joint'))
      if (links.length === 0) return null

      // Build joint lookup: child_link → { parent_link, joint_type, axis, origin_xyz, rpy }
      const jointMap = new Map<string, {
        parentLink: string
        jointType: string
        axis: string
        xyz: number[]
        rpy: number[]
      }>()
      for (const j of joints) {
        const parentEl = j.querySelector('parent')
        const childEl = j.querySelector('child')
        if (!parentEl || !childEl) continue
        const parentLink = parentEl.getAttribute('link') || ''
        const childLink = childEl.getAttribute('link') || ''
        const type = j.getAttribute('type') || 'fixed'
        const originEl = j.querySelector('origin')
        const xyz = originEl ? parseNums(originEl.getAttribute('xyz') || '0 0 0') : [0, 0, 0]
        const rpy = originEl ? parseNums(originEl.getAttribute('rpy') || '0 0 0') : [0, 0, 0]
        const axisEl = j.querySelector('axis')
        const axisVec = axisEl ? parseNums(axisEl.getAttribute('xyz') || '0 0 1') : [0, 0, 1]
        // Convert axis vector to string
        let axisStr = 'z'
        if (Math.abs(axisVec[0]) > Math.abs(axisVec[1]) && Math.abs(axisVec[0]) > Math.abs(axisVec[2])) axisStr = 'x'
        else if (Math.abs(axisVec[1]) > Math.abs(axisVec[2])) axisStr = 'y'
        jointMap.set(childLink, { parentLink, jointType: type, axis: axisStr, xyz, rpy })
      }

      // Find root link (not a child of any joint)
      const childLinks = new Set(jointMap.keys())
      const rootLink = links.find(l => !childLinks.has(l.getAttribute('name') || ''))
      if (!rootLink) return null
      const rootName = rootLink.getAttribute('name') || 'base_link'

      // Extract component_id from link name: strip trailing _N suffix
      function extractComponentId(linkName: string): string {
        const match = linkName.match(/^(.+?)_(\d+)$/)
        return match ? match[1] : linkName
      }

      // Infer attach_face from joint origin xyz relative to parent.
      // Uses rpy as tiebreaker: non-zero pitch suggests front/back face,
      // non-zero roll suggests left/right face (from elevation_angle).
      function inferFace(xyz: number[], rpy: number[]): string {
        const [x, y, z] = xyz
        const [roll, pitch] = rpy
        const ax = Math.abs(x), ay = Math.abs(y), az = Math.abs(z)

        // If rpy has significant pitch/roll, the component was on a side face
        // with elevation_angle — use the rpy to determine which face
        const hasPitch = Math.abs(pitch) > 0.05
        const hasRoll = Math.abs(roll) > 0.05

        if (hasPitch && !hasRoll && az > ax * 0.5) {
          // Pitch rotation + Z offset: likely front/back face with elevation
          return x >= 0 ? 'front' : 'back'
        }
        if (hasRoll && !hasPitch && az > ay * 0.5) {
          // Roll rotation + Z offset: likely left/right face with elevation
          return y >= 0 ? 'right' : 'left'
        }

        // Default: pure coordinate-based inference
        if (az >= ax && az >= ay) return z >= 0 ? 'top' : 'bottom'
        if (ax >= ay) return x >= 0 ? 'front' : 'back'
        return y >= 0 ? 'right' : 'left'
      }

      const components: AssemblyComponent[] = []

      // Root component
      const rootCompId = extractComponentId(rootName)
      components.push({
        link_name: rootName,
        component_id: rootCompId,
        attach_to: null,
        attach_face: null,
        joint_type: 'fixed',
        joint_axis: 'z',
      })

      // Process all non-root links in dependency order (BFS from root)
      const queue = [rootName]
      const visited = new Set([rootName])
      while (queue.length > 0) {
        const parentName = queue.shift()!
        // Find all children of this parent
        for (const [childName, jInfo] of jointMap) {
          if (jInfo.parentLink !== parentName || visited.has(childName)) continue
          visited.add(childName)
          queue.push(childName)

          const compId = extractComponentId(childName)
          const face = inferFace(jInfo.xyz, jInfo.rpy)

          components.push({
            link_name: childName,
            component_id: compId,
            attach_to: parentName,
            attach_face: face,
            joint_type: jInfo.jointType,
            joint_axis: jInfo.axis,
          })
        }
      }

      console.log(`[assembly] Reverse-parsed URDF → ${components.length} components`)
      return {
        base_link: rootName,
        ground_offset: true,
        components,
      }
    } catch (err) {
      console.error('[assembly] Failed to reverse-parse URDF:', err)
      return null
    }
  }

  // ── Topology Operations: apply add/remove/modify to AssemblyGraph ──────────

  function applyTopologyOps(graph: AssemblyGraph, operations: TopologyOp[]): AssemblyGraph {
    const components = [...graph.components.map(c => ({ ...c }))]

    for (const op of operations) {
      if (op.op === 'remove') {
        // Remove the target and all its descendants
        const toRemove = new Set<string>()
        toRemove.add(op.link_name)
        // BFS to find all descendants
        let changed = true
        while (changed) {
          changed = false
          for (const c of components) {
            if (c.attach_to && toRemove.has(c.attach_to) && !toRemove.has(c.link_name)) {
              toRemove.add(c.link_name)
              changed = true
            }
          }
        }
        // Filter out removed components
        const before = components.length
        for (let i = components.length - 1; i >= 0; i--) {
          if (toRemove.has(components[i].link_name)) components.splice(i, 1)
        }
        console.log(`[topology] Removed ${op.link_name} and ${toRemove.size - 1} descendants (${before} → ${components.length} components)`)

      } else if (op.op === 'add') {
        if (!op.component_id) {
          console.warn(`[topology] add op missing component_id for ${op.link_name}`)
          continue
        }
        components.push({
          link_name: op.link_name,
          component_id: op.component_id,
          attach_to: op.attach_to ?? null,
          attach_face: op.attach_face ?? 'top',
          joint_type: op.joint_type ?? 'fixed',
          joint_axis: op.joint_axis ?? 'z',
          length_mm: op.length_mm,
          orientation: op.orientation,
          elevation_angle: op.elevation_angle,
          attach_rpy: op.attach_rpy,
        })
        console.log(`[topology] Added ${op.link_name} (${op.component_id}) → ${op.attach_to}:${op.attach_face}`)

      } else if (op.op === 'modify') {
        const existing = components.find(c => c.link_name === op.link_name)
        if (!existing) {
          console.warn(`[topology] modify target not found: ${op.link_name}`)
          continue
        }
        // Only update fields that were explicitly provided
        if (op.component_id !== undefined) existing.component_id = op.component_id
        if (op.attach_to !== undefined) existing.attach_to = op.attach_to
        if (op.attach_face !== undefined) existing.attach_face = op.attach_face
        if (op.joint_type !== undefined) existing.joint_type = op.joint_type
        if (op.joint_axis !== undefined) existing.joint_axis = op.joint_axis
        if (op.length_mm !== undefined) existing.length_mm = op.length_mm
        if (op.orientation !== undefined) existing.orientation = op.orientation
        if (op.elevation_angle !== undefined) existing.elevation_angle = op.elevation_angle
        if (op.attach_rpy !== undefined) existing.attach_rpy = op.attach_rpy
        console.log(`[topology] Modified ${op.link_name}: ${JSON.stringify(op)}`)
      }
    }

    return {
      base_link: graph.base_link,
      ground_offset: true,
      components,
    }
  }

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
    resolveAssemblyGraph,
    reconcileNodePlacement: (): ReconcileResult => {
      if (!_lastAssemblyGraph) {
        return { adjustedCount: 0, residualMaxMm: 0, shifts: [] }
      }
      const parsed = ctx.getParsedRobot()
      return reconcileNodePlacement({
        graph: _lastAssemblyGraph,
        linkGroups: parsed.linkGroups,
        joints: parsed.joints,
      })
    },
    getLastAssemblyGraph: () => _lastAssemblyGraph ? cloneAssemblyGraph(_lastAssemblyGraph) : null,
    urdfToAssemblyGraph,
    applyTopologyOps,
    applyGraphMutation: (graph: AssemblyGraph, mutation: GraphMutation): MutationResult => {
      // Build a ValidationContext from the live preset catalog so graphMutations
      // enforces the same rules the placement engine does. findPreset returns
      // null when presets haven't loaded yet — the mutation dispatch itself
      // surfaces that as UNKNOWN_COMPONENT so the tool loop sees a real error.
      const validationCtx: ValidationContext = {
        findPreset: (id: string): ValidationPreset | null => {
          if (!presetData) return null
          for (const cat of Object.values(presetData.categories)) {
            const p = cat.components.find(c => c.id === id)
            if (p) return p as ValidationPreset
          }
          return null
        },
      }
      return runApplyMutation(graph, mutation, validationCtx)
    },
    graphsEquivalent,
    rebuildMountNodes,
    getUndoState: () => ({ undo: [...urdfUndo], redo: [...urdfRedo] }),
    restoreUndoState: (state: { undo: string[]; redo: string[] }) => {
      urdfUndo = [...state.undo]
      urdfRedo = [...state.redo]
    },
    isBulkAssemblyMode: () => _bulkMode,
    getPresetBoundingBoxMm: (compId: string): [number, number, number] | null => {
      if (!presetData) return null
      for (const cat of Object.values(presetData.categories)) {
        const p = cat.components.find(c => c.id === compId)
        if (!p) continue
        const bb = p.physical.bounding_box_mm
        // Only return when a 3-tuple bbox exists. Extrusions (cross_section_mm only)
        // get their length from per-instance length_mm — caller falls back to
        // measureLinkDims, which reads the URDF's length-aware <box size>.
        if (Array.isArray(bb) && bb.length === 3) {
          return [bb[0] ?? 40, bb[1] ?? 40, bb[2] ?? 40]
        }
        return null
      }
      return null
    },
  }
}


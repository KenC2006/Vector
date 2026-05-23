import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'
import { TransformControls } from 'three/addons/controls/TransformControls.js'
import { STLExporter } from 'three/addons/exporters/STLExporter.js'
import { invoke } from '@tauri-apps/api/core'
import { CATEGORY_COLORS, SERVO_HORN_ORIGIN_Z_RATIO } from './componentMeshes'
import type { UrdfVisualDesc } from './componentMeshes'
import {
  isMountLinkName,
  makeMountLinkName,
  nodesCompatible,
  incompatibleReason,
  resolveFaceToPort,
  resolveConnectionJoint,
} from './attachmentNodes'
import type { AttachmentNodeRuntime } from './attachmentNodes'
import type { AttachmentNodeClass, ComponentSpec } from './componentSpec.ts'
import { reportComponentSpecDeprecations } from './componentSpec.ts'
import { ensureMeshExtentsLoaded } from './meshExtents.ts'
import { hasMeshOverride, SLOW_MESH_BLACKLIST } from './richVisuals/meshOverrides'
import { componentVisualWorldQuat, preloadComponentMesh } from './richVisuals'
import { nudgeAlongNormal, shouldApplyRuntimeNudge, NUDGE_MIN_MM, type NudgeDiagnostics } from './contactCleanup'
import { quatToRpy, rpyToQuat } from './rotationIO'
import { resolveComponentVisual, resolveSplitServoVisual, visualBoundsFromDescriptors } from './componentVisualResolver'
import type { ComponentVisualBounds, ResolvedComponentVisual } from './componentVisualResolver'
import { isParametricSpec, resolveComponent, resolveComponentBboxMm, resolveComponentMassKg } from './componentResolver.ts'
import { composeGhostWorldForConnectorSnap } from './carrySnapMath.ts'
import { urdfVecToSceneVec, URDF_TO_SCENE_Q } from './coordinates.ts'
import { validateTopology as runValidateTopology, autoRepairTopology as runAutoRepair } from './topologyValidation.ts'
import type { ValidationPreset, ValidationContext } from './topologyValidation.ts'
import { normalizeAssembly, formatDiagnosticForPrompt } from './archetypeNormalizer.ts'
import type { RequestedFeatures } from './archetypeNormalizer.ts'
import { setArchetypeMode } from './placementCompiler/context.ts'
import { cloneAssemblyGraph, graphsEquivalent } from './urdfGraphEquivalence.ts'
import type { AssemblyComponent, AssemblyGraph, GraphEquivalenceResult } from './urdfGraphEquivalence.ts'
// Mate-connector resolver (Phase 1/2,). Pure
// module, bit-identical to the bbox math when mating default face connectors
// with `fastened` — see mateCorpus.ts for the parity proof. Feature-flagged
// so the legacy path can still be exercised for A/B comparison.
import {
  findConnector,
  childConnectorIdForAttachFace,
  type MateConnector,
} from './mateConnectors.ts'
import { applyMutation as runApplyMutation, validateConnectorReferences as runValidateConnectorRefs } from './graphMutations.ts'
import type { GraphMutation, MutationResult } from './graphMutations.ts'
// Render-time alignment pass (Option C — ).
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
  reparseUrdf: (xmlOverride?: string, opts?: { skipGround?: boolean; ground?: boolean }) => void
  getParsedRobot: () => ParsedRobotLike
  getKinematicGraph: () => Record<string, { name: string; mass: number; parent?: string; children: string[] }>
  getKinematicJoints: () => Record<string, { name: string; type: string; axis: string; parentLink: string; childLink: string }>
  isViewport3D: () => boolean
  /** `build` = place & snap; `inspect` = click mesh to inspect in Properties (no carry). */
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
//
// `PresetComponent` is an alias for the unified `ComponentSpec` (Phase 6).
// The local name is retained for callsite readability — every preset object
// flowing through this file IS a ComponentSpec, just with `sim_metadata` and
// `mounting_logic` typed loosely via the index signature.

type PresetComponent = ComponentSpec

interface PresetCategory {
  description: string
  components: PresetComponent[]
}

interface PresetData {
  categories: Record<string, PresetCategory>
}

// AssemblyJointType + pure transform / joint / naming helpers moved to the
// placement-compiler subtree (Phase 3b.1). Re-imported here so existing call
// sites inside this file keep working with no behavior change.
import {
  parseXyzString,
  transformFromXyzRpy,
  axisNameFromUrdf,
  axisNameFromComponentAxis,
} from './placementCompiler/transforms.ts'
import {
  resolveJointLimitsRad,
  normalizeJointType,
  axisNameToTuple,
  axisTupleToUrdf,
} from './placementCompiler/joints.ts'
import {
  componentIdFromLinkName,
  isSplitServoComponentId,
} from './placementCompiler/componentNaming.ts'
import {
  faceUVHalfExtents,
  _resetMultiChildPositionsCache,
} from './placementCompiler/multiChild.ts'
import {
  servoCompoundCarrierVisuals,
} from './placementCompiler/servoSplit.ts'
import { compileAssembly } from './placementCompiler/index.ts'

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
  // Phase 4 — optional named-connector
  // overrides that mirror the AssemblyComponent fields. Forwarded verbatim
  // so Claude's modify_topology add/modify ops can target shaft_out /
  // shaft_hole / plate_top etc. instead of falling back to attach_face.
  attach_connector?: string
  mate_connector?: string
  mate_type?: string
  // Tier-A novel-mode authoring (mirrors AssemblyComponent fields). Stripped
  // by `strip_forbidden_fields` in standard mode.
  placement_offset_mm?: number[]
  splay_angle_deg?: number
}

/** One row of engine-computed placement ground-truth (what the placement loop
 * actually emitted per child). `linkName` / `parentLinkName` are final URDF
 * names after nameMap remap. Threaded to the validator so Gemini can compare
 * screenshot inspection against the engine's authoritative xyz/rpy. */
export interface EnginePlacementEntry {
  linkName: string
  parentLinkName: string
  xyz: string   // space-separated meters, as written to URDF
  rpy: string   // space-separated radians, as written to URDF
}

/** One row of ICP contact-cleanup diagnostics per mated pair. Mirrors the
 * `[icp][trace]` console line. `confidence` is derived from paired-ratio +
 * reason: "high" when paired ≥ 75% OR reason mentions "confident-cap",
 * "low" otherwise. gap values are in mm; negative = child slightly overlaps
 * parent (flush). Validator uses gap_p50_mm / confidence to refute screenshot
 * claims of "floating N mm". */
export interface EngineIcpEntry {
  linkName: string
  parentConnector: string   // `${parentPresetId}.${parentConnectorId}`
  childConnector: string    // `${childPresetId}.${childConnectorId}`
  pairedCount: number
  sampleCount: number
  gapP50Mm: number | null
  gapP90Mm: number | null
  gapMinMm: number | null
  gapMaxMm: number | null
  nudgeMm: number
  reason: string
  confidence: 'high' | 'low'
}

/** Ground-truth payload returned from resolveAssemblyGraph and forwarded to
 * the Gemini validator. */
export interface EngineSummary {
  placements: EnginePlacementEntry[]
  icpGaps: EngineIcpEntry[]
}

export interface UrdfAssemblyApi {
  onModelUpdated(): void
  recordUndoExternal(content: string): void
  exitCarryMode(): void
  onInteractionModeChanged(mode: 'build' | 'inspect'): void
  /** Sync 3D selection and Properties panel. Inspect mode keeps build gizmos hidden. */
  setSelectedLink(linkName: string | null): void
  /** Resolve an AI assembly graph using the frontend snap/placement system. Returns final URDF and any topology errors. */
  resolveAssemblyGraph(graph: AssemblyGraph): { urdf: string | null; topologyErrors?: string[]; topologyWarnings?: string[]; engineSummary?: EngineSummary }
  /** Render-time alignment: measure real AABBs of rendered meshes and shift pivots
   * so child contact surfaces meet their parent's attach face. No-op if no graph
   * has been resolved yet. Safe to call multiple times (EPS-guarded, idempotent). */
  reconcileNodePlacement(): ReconcileResult
  /** Get a deep-cloned snapshot of the last successfully resolved AssemblyGraph. Cloned so
   * callers (chat context, IPC marshaling) can't mutate the canonical in-memory copy. */
  getLastAssemblyGraph(): AssemblyGraph | null
  /** Re-read the stored graph from localStorage for the current active filename.
   *  Call from main.ts whenever the active file changes — without it the in-memory
   *  graph stays pinned to whatever was loaded at module init, so opening a
   *  checkpoint or switching files leaves bake/edit reading the wrong robot's
   *  graph. Returns true if a graph was found, false if cleared. */
  refreshAssemblyGraphForActiveFile(): boolean
  /** Replace the in-memory AssemblyGraph and persist it to localStorage under
   *  the current filename. Used by checkpoint restore to install the captured
   *  graph losslessly — alternative to losing fields via the URDF round-trip
   *  fallback. Pass null to clear. */
  setLastAssemblyGraph(graph: AssemblyGraph | null): void
  /** Reverse-parse current URDF into an AssemblyGraph for iterative editing (lossy fallback — prefer getLastAssemblyGraph). */
  urdfToAssemblyGraph(urdfXml: string): AssemblyGraph | null
  /** Structural + parametric equality for two AssemblyGraphs. Use to detect drift when a
   * reverse-parse is unavoidable (import-URDF path). */
  graphsEquivalent(a: AssemblyGraph, b: AssemblyGraph): GraphEquivalenceResult
  /** Apply modify_topology operations to an existing AssemblyGraph and return the modified version.
   * `archetypeOverride` lets the caller force `_archetype_mode` on the returned
   * graph — used so Claude's per-turn `archetype_mode` declaration on
   * `modify_topology` carries through. Without this override, the field is
   * inherited from the input graph (preserving novel-mode designs across
   * topology edits). */
  applyTopologyOps(graph: AssemblyGraph, operations: TopologyOp[], archetypeOverride?: 'standard' | 'novel'): AssemblyGraph
  /** WS2 tool-call edit surface: apply a single typed mutation with per-call
   * validation. Runs against a deep clone of `graph`; on success the new graph
   * is returned and the caller commits via resolveAssemblyGraph. On failure,
   * a structured error returns to the Claude tool loop for same-turn self-
   * correction — no full-graph redesign fired. */
  applyGraphMutation(graph: AssemblyGraph, mutation: GraphMutation): MutationResult
  /** Re-run attachment node placement based on current scene geometry. Call after async GLB meshes settle. */
  rebuildMountNodes(): void
  refreshMountNodeTransforms(): void
  /** Snapshot the current undo/redo stacks (call before switching files). */
  getUndoState(): { undo: string[]; redo: string[] }
  /** Restore a previously saved undo/redo snapshot (call after switching files). */
  restoreUndoState(state: { undo: string[]; redo: string[] }): void
  /** True while resolveAssemblyGraph is batching edits. Callers (e.g. reparseURDF) skip heavy per-mesh rebuilds when active. */
  isBulkAssemblyMode(): boolean
  /** Look up a component's authoritative bounding box from the preset catalog (in mm).
   * Returns null when the preset has only a 2-tuple cross_section_mm (extrusions),
   * where per-instance length_mm makes the link's URDF box the authoritative source. */
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
 * that will require a per-DOF quaternion stack at the joint level.
 *
 * @param jointEl The `<joint>` XML element to update.
 * @param doc The owning Document (used to create the `<axis>` element if missing).
 * @param oldJointWorldQ Quaternion of the joint frame BEFORE the move (drag-start snapshot).
 * @param parentWorldQ World quaternion of the joint's NEW parent link.
 * @param newLocalQuat New local quaternion of the joint in the parent frame.
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
 * cx/cy/cz are the visual center offset within the carry group's local frame. */
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
  const margin = 0
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
  let interactionMode: 'build' | 'inspect' = 'build'
  let gizmoBasePivotWorld = new THREE.Matrix4()
  // Lowest world-Y of the selected link's subtree at drag start. The floor
  // constraint uses this as its floor threshold instead of y=0 so a component
  // that was already resting on (or a hair below) the floor at drag start
  // doesn't get bumped upward on the first drag frame — only real
  // drag-through-floor motion triggers a lift.
  let gizmoDragStartMinY = 0
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
    localAxis: THREE.Vector3 | null
    worldAxis: THREE.Vector3 | null
    frameLinkName: string
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
    const selectedGroup = getInteractionLinkGroup(selectedLink)
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
    const selectedGroup = getInteractionLinkGroup(selectedLink)
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

    // Inspect mode (incl. forced-inspect during AI / sim) must never leave
    // mount nodes visible after a rebuild — the prior carry/drag session may
    // have left nodesGroup.visible=true, and rebuildMountNodes is the only
    // funnel after URDF reparses.
    if (interactionMode !== 'build') {
      nodesGroup.visible = false
      applyNodeRingVisibility()
    }

    const graph = ctx.getKinematicGraph()
    const kinJoints = ctx.getKinematicJoints()
    const parsed = ctx.getParsedRobot()

    const OCCUPIED_DIST_M = 0.025

    for (const linkName of Object.keys(graph)) {
      if (isMountLinkName(linkName)) continue
      const frameLinkName = resolveInteractionFrameLinkName(linkName, parsed)
      if (!frameLinkName) continue
      const lg = parsed.linkGroups.get(frameLinkName)
      if (!lg) continue
      let localBox: THREE.Box3 | null
      if (linkBBoxCache.has(linkName)) {
        localBox = linkBBoxCache.get(linkName)!
      } else {
        const componentId = componentIdFromLinkName(linkName)
        const preset = findPresetById(componentId)
        if (preset) {
          // Phase 4c — resolver-driven bbox is the
          // source of truth for every preset-backed link. The resolver's AABB
          // is zero-centered around the link origin (post-parametric, pre-
          // rotation), which matches what the placement compiler reads. Using
          // it here makes mount-node positions independent of async mesh load
          // state and identical to placement-compiler inputs.
          const resolved = resolveComponentVisual({ preset, category: findCategory(preset) })
          localBox = new THREE.Box3(
            new THREE.Vector3(-resolved.bounds.hx, -resolved.bounds.hy, -resolved.bounds.hz),
            new THREE.Vector3(resolved.bounds.hx, resolved.bounds.hy, resolved.bounds.hz),
          )
        } else {
          // No preset (custom / unknown link) — fall back to scene-walk.
          localBox = computeLinkLocalBoundingBox(lg)
        }
        linkBBoxCache.set(linkName, localBox)
      }
      if (!localBox || localBox.isEmpty()) continue

      const center = localBox.getCenter(new THREE.Vector3())
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

      const componentId = componentIdFromLinkName(linkName)
      const preset = findPresetById(componentId)
      const connectors = preset ? resolveComponentConnectors(preset) : []
      // Phase 4c — with the resolver-driven
      // (zero-centered) bbox above, the previous hub_bore center-compensation
      // for tires (`origin_xyz: [-center, -center, -center]`) collapses to
      // `[0, 0, 0]`, which is already what `resolveComponentPortsForBounds`
      // declares for tire hub_bore. The branch is no longer needed.
      const faceDefs = preset ? resolveComponentPorts(preset) : []

      for (const f of faceDefs) {
        const connector = findConnectorForNode(connectors, f.nodeId)
        const localAxis = axisFromPortOrConnector(f.origin_xyz, connector, false)
        const localPos = new THREE.Vector3(
          center.x + f.origin_xyz[0],
          center.y + f.origin_xyz[1],
          center.z + f.origin_xyz[2],
        )
        const nodeKey = makeMountLinkName(linkName, f.nodeId)
        const worldPosition = localPos.clone().applyMatrix4(lg.matrixWorld)
        const worldQuaternion = linkWorldQuat.clone()
        const worldAxis = localAxis ? localAxis.clone().applyQuaternion(linkWorldQuat).normalize() : null

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
          frameLinkName,
          localPos,
          localAxis,
          worldAxis,
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
   * Pass `onlyLink` to limit refresh to nodes owned by that link — use this
   * during drag where only the selected component is moving and all static
   * target nodes already have valid world positions from rebuildMountNodes(). */
  function refreshNodeWorldTransforms(onlyLink?: string) {
    const parsed = ctx.getParsedRobot()
    const worldQuatTmp = new THREE.Quaternion()
    const worldPosTmp = new THREE.Vector3()
    const worldScaleTmp = new THREE.Vector3()
    // Avoid redundant updateMatrixWorld calls for the same link (each link has 6 face nodes)
    const updatedLinks = new Set<string>()
    for (const n of mountNodes) {
      if (onlyLink !== undefined && n.parentLink !== onlyLink) continue
      const lg = parsed.linkGroups.get(n.frameLinkName)
      if (!lg) continue
      if (!updatedLinks.has(n.frameLinkName)) {
        lg.updateMatrixWorld(true)
        updatedLinks.add(n.frameLinkName)
      }
      lg.matrixWorld.decompose(worldPosTmp, worldQuatTmp, worldScaleTmp)
      n.worldQuaternion.copy(worldQuatTmp)
      n.worldPosition.copy(n.localPos).applyMatrix4(lg.matrixWorld)
      if (n.localAxis) {
        n.worldAxis = n.localAxis.clone().applyQuaternion(worldQuatTmp).normalize()
      }

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

  function commitUrdf(mutator: (doc: Document) => boolean, opts?: { defer?: boolean; skipGround?: boolean }): boolean {
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
    const reparseOpts = opts?.skipGround ? { skipGround: true } : undefined
    if (opts?.defer) {
      requestAnimationFrame(() => ctx.reparseUrdf(xml, reparseOpts))
    } else {
      ctx.reparseUrdf(xml, reparseOpts)
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

  function resolveInteractionFrameLinkName(linkName: string, parsed: ParsedRobotLike = ctx.getParsedRobot()): string | null {
    if (parsed.linkGroups.has(linkName)) return linkName
    if (parsed.linkGroups.has(`${linkName}_body`)) return `${linkName}_body`
    if (parsed.linkGroups.has(`${linkName}_horn`)) return `${linkName}_horn`
    return null
  }

  function getInteractionLinkGroup(linkName: string): THREE.Group | null {
    const parsed = ctx.getParsedRobot()
    const frameLinkName = resolveInteractionFrameLinkName(linkName, parsed)
    return frameLinkName ? (parsed.linkGroups.get(frameLinkName) ?? null) : null
  }

  function isLogicalSplitServoLink(linkName: string): boolean {
    const parsed = ctx.getParsedRobot()
    return (
      !!ctx.getKinematicGraph()[linkName] &&
      !parsed.linkGroups.has(linkName) &&
      parsed.linkGroups.has(`${linkName}_body`) &&
      parsed.linkGroups.has(`${linkName}_horn`)
    )
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
    const joint = Object.values(joints).find(j => j.childLink === linkName) ?? null
    if (!joint) return null
    if (isLogicalSplitServoLink(linkName) && ctx.getParsedRobot().joints.has(`${joint.name}_mount`)) {
      return {
        ...joint,
        name: `${joint.name}_mount`,
        type: 'fixed',
        childLink: `${linkName}_body`,
      }
    }
    return joint
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
    const graphNode = ctx.getKinematicGraph()[selectedLink]
    const mass = graphNode?.mass ?? 0
    const massLabel = mass >= 1 ? `${mass.toFixed(2)} kg` : `${Math.round(mass * 1000)} g`
    const children = (graphNode?.children ?? []).filter(child => !isMountLinkName(child))
    const childLabel = children.length ? children.join(', ') : 'No child links'
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
        <div class="insp-row"><span class="insp-key">Mass</span><span class="insp-val">${massLabel}</span></div>
        <div class="insp-row"><span class="insp-key">Children</span><span class="insp-val">${childLabel}</span></div>
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

    // Resolve _body/_horn suffixes to the merged servo base name in the graph
    let resolvedName = linkName
    if (!graph[linkName]) {
      if (linkName.endsWith('_body') || linkName.endsWith('_horn')) {
        const suffix = linkName.endsWith('_body') ? '_body' : '_horn'
        const baseName = linkName.slice(0, -suffix.length)
        if (graph[baseName]) resolvedName = baseName
      }
    }

    const node = graph[resolvedName]
    if (!node) return

    // Prevent deleting root link if it's the only one
    if (!node.parent && Object.keys(graph).length <= 1) {
      ctx.showToast('Cannot delete the only remaining link', 'warning')
      return
    }

    // Collect subtree using reconstituted graph names
    const toRemove = new Set<string>()
    const walk = (name: string) => {
      toRemove.add(name)
      const n = graph[name]
      if (n) n.children.forEach(walk)
    }
    walk(resolvedName)

    const childCount = toRemove.size - 1
    const label = childCount > 0 ? `"${resolvedName}" and ${childCount} child link${childCount > 1 ? 's' : ''}` : `"${resolvedName}"`

    const changed = commitUrdf(doc => {
      const robot = doc.documentElement
      if (!robot || robot.nodeName !== 'robot') return false

      // Expand split servo names: base name X → also remove X_body and X_horn from URDF
      const urdfNamesToRemove = new Set<string>()
      for (const name of toRemove) {
        urdfNamesToRemove.add(name)
        if (doc.querySelector(`link[name="${name}_body"]`)) urdfNamesToRemove.add(name + '_body')
        if (doc.querySelector(`link[name="${name}_horn"]`)) urdfNamesToRemove.add(name + '_horn')
      }

      // Remove all URDF links in subtree (including split servo phantom links)
      for (const name of urdfNamesToRemove) {
        const linkEl = doc.querySelector(`link[name="${name}"]`)
        if (linkEl) robot.removeChild(linkEl)
      }

      // Remove all joints whose parent or child is in the removal set
      const joints = doc.querySelectorAll('joint')
      joints.forEach(j => {
        const parentName = j.querySelector('parent')?.getAttribute('link')
        const childName = j.querySelector('child')?.getAttribute('link')
        if ((parentName && urdfNamesToRemove.has(parentName)) || (childName && urdfNamesToRemove.has(childName))) {
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
    // Resolve split servo _body/_horn mesh clicks to the merged base name
    if (name) {
      const graph = ctx.getKinematicGraph()
      if (!graph[name] && (name.endsWith('_body') || name.endsWith('_horn'))) {
        const suffix = name.endsWith('_body') ? '_body' : '_horn'
        const baseName = name.slice(0, -suffix.length)
        if (graph[baseName]) name = baseName
      }
    }
    selectedLink = name
    gizmo.detach()
    rootDragWarned = false
    if (name && interactionMode === 'build') {
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
  function findPresetById(componentId: string): PresetComponent | null {
    if (!presetData) return null
    for (const cat of Object.values(presetData.categories)) {
      for (const comp of cat.components) {
        if (comp.id === componentId) return comp
      }
    }
    return null
  }
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

  // Phase 2: placement reads parent bounds
  // from the resolver, not from a re-parsed URDF visual AABB. Returns bounds
  // in METERS — matching the contract callers expect from the retired
  // (Phase 2) getParentBounds DOM-read path.
  //
  // Lookup strategy:
  // 1. If the caller already has the parent preset (the AI assembly path
  // always does), they pass it in. Otherwise we strip the trailing
  // _N suffix from the link name and look it up in the catalog.
  // 2. Split-servo `_body`/`_horn` link suffixes route to
  // resolveSplitServoVisual so a child mounted on the body sees body
  // bounds, not the whole-servo envelope.
  // 3. Unknown link names (base_link, synthetic carriers, presets that
  // haven't loaded yet) get the same 50 mm cube fallback the old URDF
  // path returned.
  function parentBoundsFromLink(
    parentLinkName: string,
    options?: { preset?: PresetComponent | null; instanceLengthMm?: number },
  ): { hx: number; hy: number; hz: number; cx: number; cy: number; cz: number } {
    const FALLBACK = { hx: 0.05, hy: 0.05, hz: 0.05, cx: 0, cy: 0, cz: 0 }
    const isBody = parentLinkName.endsWith('_body')
    const isHorn = parentLinkName.endsWith('_horn')
    const baseLinkName = isBody || isHorn
      ? parentLinkName.slice(0, parentLinkName.lastIndexOf('_'))
      : parentLinkName

    let preset = options?.preset ?? null
    if (!preset) {
      preset = findPresetById(componentIdFromLinkName(baseLinkName))
    }
    if (!preset) return FALLBACK

    if ((isBody || isHorn) && isSplitServoComponentId(preset.id)) {
      const split = resolveSplitServoVisual({
        preset: preset as Parameters<typeof resolveSplitServoVisual>[0]['preset'],
        category: 'actuators',
        instance: options?.instanceLengthMm ? { length_mm: options.instanceLengthMm } : undefined,
      })
      const b = isBody ? split.bodyCollision.bounds : split.hornCollision.bounds
      return { hx: b.hx, hy: b.hy, hz: b.hz, cx: b.cx, cy: b.cy, cz: b.cz }
    }

    const resolved = resolveComponent({
      spec: preset as Parameters<typeof resolveComponent>[0]['spec'],
      instance: options?.instanceLengthMm ? { length_mm: options.instanceLengthMm } : undefined,
    })
    const [hxMm, hyMm, hzMm] = resolved.bounds.half
    const [cxMm, cyMm, czMm] = resolved.bounds.center
    return {
      hx: hxMm / 1000,
      hy: hyMm / 1000,
      hz: hzMm / 1000,
      cx: cxMm / 1000,
      cy: cyMm / 1000,
      cz: czMm / 1000,
    }
  }

  function computePlacement(
    _doc: Document, parentLinkName: string,
    comp: PresetComponent,
    childX: number, _childY: number, childZ: number,
    childCenterOffset: { cx: number; cy: number; cz: number } = { cx: 0, cy: 0, cz: 0 },
  ): { xyz: string; rpy: string } {
    const parent = parentBoundsFromLink(parentLinkName)
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

  // splayAngleForLegCount + verticalExtentForRotation moved to
  // placementCompiler/multiChild.ts (Phase 3b.2). Imported below.

  // ── Phase 2 connector branch ───────────
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
  // hasMateConnectorFields moved to placementCompiler/mate.ts (Phase 3b.2).

  // ── Scene-level ICP contact cleanup (Step 2 — docs/ENGINE_NEXT_STEPS.md) ──
  // Runs AFTER reparse + rich-visuals + reconcileNodePlacement so it can
  // raycast against the REAL rendered meshes (per-axis-scaled GLBs with
  // shaft overlays baked in). The in-loop hook was a dead-end: bulk
  // assembly commits URDF text then reparses ONCE at the end, so nothing
  // but the root is in the scene during the placement loop. See
  // scripts/contact-analysis-output.txt for the Phase 1 diagnosis.

  /** Standalone preset lookup that doesn't depend on the `findPreset`
   * closure inside `resolveAssemblyGraph`. Reads the outer `presetData`. */
  function _findPresetForCleanup(componentId: string): PresetComponent | null {
    if (!presetData) return null
    for (const cat of Object.values(presetData.categories)) {
      for (const comp of cat.components) {
        if (comp.id === componentId) return comp
      }
    }
    return null
  }

  // Phase 2c: contact cleanup is now an
  // analytic check against resolved.collision, not a scene raycast. With
  // resolver-driven placement (Phase 2a) and resolver-driven collision
  // bounds (Phase 1), the gap between mating faces is determined by the
  // authored connector geometry — there is no measurement to take.
  //
  // The function still returns shifts/icpEntries for the validator payload,
  // but it never moves a pivot. Persistent non-zero analytic gaps are logged
  // as warnings so the spec author can fix the bbox or connector authoring
  // (the plan §3.6 calls this out as the shaft_out=17 vs hz=14.5 case).
  //
  // The raycast utility `nudgeAlongNormal` lives on for the alignmentCorpus
  // unit tests; production no longer wires it.
  function runContactCleanupPass(): { adjustedCount: number; shifts: Array<{ linkName: string; dMm: number }>; icpEntries: EngineIcpEntry[] } {
    void shouldApplyRuntimeNudge // Phase 2c: kept exported for the corpus, no longer wired here
    void nudgeAlongNormal
    void NUDGE_MIN_MM
    const _unusedDiag: NudgeDiagnostics | null = null; void _unusedDiag

    const graphSnap = _lastAssemblyGraph
    const shifts: Array<{ linkName: string; dMm: number }> = []
    const icpEntries: EngineIcpEntry[] = []
    if (!graphSnap) return { adjustedCount: 0, shifts, icpEntries }

    for (const comp of graphSnap.components) {
      if (!comp.attach_to) continue
      const parentComp = graphSnap.components.find(c => c.link_name === comp.attach_to)
      if (!parentComp) continue
      const parentPreset = _findPresetForCleanup(parentComp.component_id)
      const childPreset = _findPresetForCleanup(comp.component_id)
      if (!parentPreset || !childPreset) continue

      const pAll = resolveComponentConnectors(parentPreset, parentComp)
      const cAll = resolveComponentConnectors(childPreset, comp)
      const parentConnectorId = comp.attach_connector ?? comp.attach_face ?? 'top'
      const inferredChildId = comp.attach_face ? childConnectorIdForAttachFace(comp.attach_face) : null
      const childConnectorId = comp.mate_connector ?? inferredChildId ?? 'bottom'
      const parentConn = findConnector(pAll, parentConnectorId)
      const childConn = findConnector(cAll, childConnectorId)
      if (!parentConn || !childConn) continue

      // Analytic gap along the parent connector axis. With resolver-driven
      // placement, parent connector world origin equals child connector world
      // origin, and the parent collision face along +axis is at:
      // parentColl.center · axis + parentColl.half · |axis| − parentConn.origin · axis
      // Same expression on the child side (with axis negated for antiparallel
      // mate). Sum them and the result is the residual gap; under correct
      // authoring it is 0.
      const parentColl = resolveComponent({
        spec: parentPreset as Parameters<typeof resolveComponent>[0]['spec'],
        instance: parentComp.length_mm ? { length_mm: parentComp.length_mm } : undefined,
      }).collision.bounds
      const childColl = resolveComponent({
        spec: childPreset as Parameters<typeof resolveComponent>[0]['spec'],
        instance: comp.length_mm ? { length_mm: comp.length_mm } : undefined,
      }).collision.bounds
      const ax = parentConn.axis_xyz
      const axLen = Math.hypot(ax[0], ax[1], ax[2]) || 1
      const aU = [ax[0] / axLen, ax[1] / axLen, ax[2] / axLen] as const
      const dot3 = (a: readonly number[], b: readonly number[]) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
      const absDot3 = (a: readonly number[], b: readonly number[]) => Math.abs(a[0]) * Math.abs(b[0]) + Math.abs(a[1]) * Math.abs(b[1]) + Math.abs(a[2]) * Math.abs(b[2])

      const pFaceMm = dot3(parentColl.center, aU) + absDot3(parentColl.half, aU) - dot3(parentConn.origin_xyz_mm, aU)
      // Child connector axis is antiparallel by mate convention; treat both as outward.
      const cAxisMag = Math.hypot(childConn.axis_xyz[0], childConn.axis_xyz[1], childConn.axis_xyz[2]) || 1
      const cAU = [childConn.axis_xyz[0] / cAxisMag, childConn.axis_xyz[1] / cAxisMag, childConn.axis_xyz[2] / cAxisMag] as const
      const cFaceMm = dot3(childColl.center, cAU) + absDot3(childColl.half, cAU) - dot3(childConn.origin_xyz_mm, cAU)
      const analyticGapMm = -(pFaceMm + cFaceMm)

      let reason = 'resolver-deterministic placement; no nudge needed'
      let confidence: 'high' | 'low' = 'high'
      if (Math.abs(analyticGapMm) > 1.0) {
        reason = `analytic gap ${analyticGapMm.toFixed(2)}mm — author bbox/connector mismatch (Phase 5 will gate)`
        confidence = 'low'
        console.warn(`[icp][analytic] ${comp.link_name} ${parentPreset.id}.${parentConnectorId}→${childPreset.id}.${childConnectorId} ${reason}`)
      }

      icpEntries.push({
        linkName: comp.link_name,
        parentConnector: `${parentPreset.id}.${parentConnectorId}`,
        childConnector: `${childPreset.id}.${childConnectorId}`,
        pairedCount: 0,
        sampleCount: 0,
        gapP50Mm: analyticGapMm,
        gapP90Mm: analyticGapMm,
        gapMinMm: analyticGapMm,
        gapMaxMm: analyticGapMm,
        nudgeMm: 0,
        reason,
        confidence,
      })
    }

    return { adjustedCount: 0, shifts, icpEntries }
  }

  /** Build a preset with length_mm override applied to bounding_box_mm (for extrusions). */
  function buildVisPreset(preset: PresetComponent, comp: { length_mm?: number }): PresetComponent {
    if (comp.length_mm && isParametricSpec(preset)) {
      const bb = resolveComponentBboxMm(preset)
      return { ...preset, physical: { ...preset.physical, bounding_box_mm: [bb[0], bb[1], comp.length_mm] } } as PresetComponent
    }
    return preset
  }


  // Resolve which category a component belongs to
  function findCategory(comp: PresetComponent): string {
    if (!presetData) return 'structural'
    for (const [catName, cat] of Object.entries(presetData.categories)) {
      if (cat.components.some(c => c.id === comp.id)) return catName
    }
    return 'structural'
  }

  type CarryGhostBounds = ComponentVisualBounds
  type CarryGhostPreview = {
    bounds: CarryGhostBounds
    visuals: UrdfVisualDesc[]
    previewGroup?: THREE.Group
    authoredFrame: ResolvedComponentVisual['authoredFrame']
  }

  function computeCarryGhostPreview(comp: PresetComponent): CarryGhostPreview {
    const catName = findCategory(comp)
    // Split servos: the placed body/horn links render the URDF primitives
    // emitted by resolveSplitServoVisual (servoBodyShape + servoHornShape).
    // Force the carry ghost down the same primitive path — merging body and
    // horn descriptors with the horn's z=hornOriginZ offset so the ghost
    // outline matches the placed silhouette piece-for-piece. Without this
    // override the resolver's previewGroup (rich NURBS or cached mesh) wins
    // and the ghost renders a shape that doesn't exist anywhere in the
    // placed component.
    const resolved = resolveComponentVisual({ preset: comp, category: catName })
    if (isSplitServoComponentId(comp.id)) {
      const split = resolveSplitServoVisual({
        preset: comp as Parameters<typeof resolveSplitServoVisual>[0]['preset'],
        category: catName,
      })
      const merged: UrdfVisualDesc[] = [
        ...split.bodyVisuals,
        ...split.hornVisuals.map(v => ({
          ...v,
          origin_xyz: [v.origin_xyz[0], v.origin_xyz[1], v.origin_xyz[2] + split.hornOriginZ] as [number, number, number],
        })),
      ]
      const bounds = visualBoundsFromDescriptors(merged) ?? resolved.bounds
      return { bounds, visuals: merged, previewGroup: undefined, authoredFrame: resolved.authoredFrame }
    }
    const bounds = resolved.previewGroup ? resolved.bounds : (resolved.visualBounds ?? resolved.bounds)
    return { bounds, visuals: resolved.visuals, previewGroup: resolved.previewGroup, authoredFrame: resolved.authoredFrame }
  }

  function makeCarryGhostVisualGroup(visuals: UrdfVisualDesc[]): THREE.Group {
    const group = new THREE.Group()
    const urdfToSceneQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(-Math.PI / 2, 0, 0, 'XYZ'))

    for (const vis of visuals) {
      const wrapper = new THREE.Group()
      const [ox, oy, oz] = vis.origin_xyz
      wrapper.position.set(ox, oz, -oy)
      wrapper.quaternion.copy(urdfToSceneQuat.clone().multiply(rpyToQuat(vis.origin_rpy)))

      const g = vis.geometry
      let geo: THREE.BufferGeometry
      if (g.type === 'box') {
        geo = new THREE.BoxGeometry(g.size[0], g.size[1], g.size[2])
      } else if (g.type === 'cylinder') {
        geo = new THREE.CylinderGeometry(g.radius, g.radius, g.length, 32)
      } else {
        geo = new THREE.SphereGeometry(g.radius, 24, 16)
      }

      const mesh = new THREE.Mesh(geo, carryGhostMat)
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo), carryEdgeMat)
      if (g.type === 'cylinder') {
        mesh.rotation.x = Math.PI / 2
        edges.rotation.x = Math.PI / 2
      }
      wrapper.add(mesh, edges)
      group.add(wrapper)
    }

    return group
  }

  function makeCarryGhostPreviewGroup(
    group: THREE.Group,
    authoredFrame: ResolvedComponentVisual['authoredFrame'],
  ): THREE.Group {
    const ghost = group.clone(true)
    // Same adapter the render path uses, only with target='scene_y_up' since
    // the carry parent is the scene (Y-up) rather than a URDF link group.
    // This is what makes carry and render orientations identical by
    // construction — a Z-up authored mesh that previously appeared sideways
    // here, or a Y-up rich generator whose render path silently re-rotated,
    // both now route through one explicit transform.
    ghost.quaternion.copy(componentVisualWorldQuat(authoredFrame, 'scene_y_up'))
    const edgeItems: Array<{ parent: THREE.Object3D; mesh: THREE.Mesh }> = []
    ghost.traverse(child => {
      if (child instanceof THREE.Mesh) {
        child.material = carryGhostMat
        child.castShadow = false
        child.receiveShadow = false
        edgeItems.push({ parent: child.parent ?? ghost, mesh: child })
      }
    })
    for (const { parent, mesh } of edgeItems) {
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry), carryEdgeMat)
      edges.position.copy(mesh.position)
      edges.quaternion.copy(mesh.quaternion)
      edges.scale.copy(mesh.scale)
      parent.add(edges)
    }
    return ghost
  }

  /** Compute the AABB of a component in URDF coordinates.
   * Placement math still prefers rendered mesh dims when available, but carry
   * visualization uses computeCarryGhostPreview() so it preserves per-visual
   * rotations and multi-primitive outlines instead of drawing a generic box. */
  function computeCarryGhostBounds(comp: PresetComponent): CarryGhostBounds {
    const catName = findCategory(comp)
    return resolveComponentVisual({ preset: comp, category: catName }).bounds
  }

  function resolveVisualHalfBoundsMm(
    preset: PresetComponent,
    instance?: { length_mm?: number },
  ): { hxMm: number; hyMm: number; hzMm: number } {
    const resolved = resolveComponent({ spec: preset, instance, category: findCategory(preset) })
    return {
      hxMm: resolved.bounds.half[0],
      hyMm: resolved.bounds.half[1],
      hzMm: resolved.bounds.half[2],
    }
  }

  function resolveComponentConnectors(
    preset: PresetComponent,
    instance?: { length_mm?: number },
  ): MateConnector[] {
    return resolveComponent({ spec: preset, instance, category: findCategory(preset) }).connectors
  }

  function connectorIdForNodeId(nodeId: string): string {
    const mapped: Record<string, string> = {
      x_plus: 'front',
      x_minus: 'back',
      y_plus: 'right',
      y_minus: 'left',
    }
    return mapped[nodeId] ?? nodeId
  }

  function findConnectorForNode(connectors: MateConnector[], nodeId: string): MateConnector | undefined {
    return findConnector(connectors, nodeId) ?? findConnector(connectors, connectorIdForNodeId(nodeId)) ?? undefined
  }

  function axisFromPortOrConnector(
    portOrigin: [number, number, number],
    connector?: MateConnector,
    remapToSceneLocal = false,
  ): THREE.Vector3 | null {
    const raw = connector
      ? new THREE.Vector3(connector.axis_xyz[0], connector.axis_xyz[1], connector.axis_xyz[2])
      : new THREE.Vector3(portOrigin[0], portOrigin[1], portOrigin[2])
    if (!(raw.length() > 1e-9)) return null
    const axis = remapToSceneLocal ? urdfVecToSceneVec(raw) : raw
    return axis.length() > 1e-9 ? axis.normalize() : null
  }

  function resolveComponentPorts(
    preset: PresetComponent,
    instance?: { length_mm?: number },
  ) {
    return resolveComponent({ spec: preset, instance, category: findCategory(preset) }).ports
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

  function addBoundsCollisionElement(doc: Document, link: Element, bounds: ComponentVisualBounds) {
    const collision = doc.createElement('collision')
    const co = doc.createElement('origin')
    co.setAttribute('xyz', [bounds.cx, bounds.cy, bounds.cz].map(v => v.toFixed(6)).join(' '))
    co.setAttribute('rpy', '0 0 0')
    const geometry = doc.createElement('geometry')
    if (bounds.shape === 'cylinder') {
      // AABB convention for cylinders: hz is the half-length along the
      // symmetry axis, hx/hy are the radius. URDF's cylinder primitive defaults
      // its axis to local Z, which matches.
      const cylEl = doc.createElement('cylinder')
      cylEl.setAttribute('radius', Math.max(bounds.hx, bounds.hy).toFixed(6))
      cylEl.setAttribute('length', (bounds.hz * 2).toFixed(6))
      geometry.appendChild(cylEl)
    } else {
      const boxEl = doc.createElement('box')
      boxEl.setAttribute('size', [
        bounds.hx * 2,
        bounds.hy * 2,
        bounds.hz * 2,
      ].map(v => v.toFixed(6)).join(' '))
      geometry.appendChild(boxEl)
    }
    collision.appendChild(co)
    collision.appendChild(geometry)
    link.appendChild(collision)
  }

  // Phase 5b: URDF collision is always the canonical AABB envelope the placement
  // compiler reads (resolved.collision.bounds). MJCF inherits the same envelope,
  // so contact and placement agree by construction. The authored mesh file is
  // still tracked on the resolver (and used for the rich visual), but it is
  // intentionally NOT emitted as a collision shape — its convex hull can drift
  // from the AABB by up to the 15 % collision-divergence ceiling, which would
  // mean MuJoCo contacts wouldn't line up with where children were placed.
  function addResolvedCollisionElements(doc: Document, link: Element, resolved: ResolvedComponentVisual) {
    addBoundsCollisionElement(doc, link, resolved.collision.bounds)
  }

  function addResolvedCollisionSourceElements(
    doc: Document,
    link: Element,
    collision: ResolvedComponentVisual['collision'],
    _primitiveVisuals: UrdfVisualDesc[],
  ) {
    addBoundsCollisionElement(doc, link, collision.bounds)
  }

  // Phase 3 — carry-ghost ↔ commit invariant.
  // The user-visible carry ghost is rendered at `carryGroup.matrixWorld`; the
  // commit path decomposes that matrix in the parent's local frame, formats
  // the result as URDF xyz/rpy strings, and feeds them to addComponentCore.
  // The invariant: reconstructing the predicted world from those strings
  // (parentWorld · transformFromXyzRpy(xyzStr, rpyStr)) must match the ghost
  // world the user just saw, within a tolerance comfortably above the
  // `fmt`/`quatToRpy` round-trip noise floor. Drift past this means the
  // ghost is rendering one pose while we're persisting another — exactly
  // the "visible jump on commit" symptom Phase 3 was scoped to catch.
  //
  // This is observability only: a warning, not a throw, so a real drift
  // surfaces in the console (and, if needed, can be promoted to a hard
  // failure later) without breaking the user's commit mid-action.
  function _assertCarryCommitInvariant(
    label: string,
    ghostWorld: THREE.Matrix4,
    parentWorld: THREE.Matrix4,
    xyzStr: string,
    rpyStr: string,
  ): void {
    const predicted = parentWorld.clone().multiply(transformFromXyzRpy(xyzStr, rpyStr))
    // Carry frame ≠ link frame: the carry ghost has the URDF→scene basis swap
    // baked in below `carryGroup` (the -90°X on its child), while the render
    // path applies that swap one level higher on `worldGroup`. So the correct
    // link-frame pose is `ghostWorld * URDF_TO_SCENE_Q`. Compare against that
    // — comparing against ghostWorld directly would warn on every commit.
    const URDF_TO_SCENE_M = new THREE.Matrix4().makeRotationFromQuaternion(URDF_TO_SCENE_Q)
    const ghostLinkFrame = ghostWorld.clone().multiply(URDF_TO_SCENE_M)
    const gp = new THREE.Vector3(); const gq = new THREE.Quaternion()
    ghostLinkFrame.decompose(gp, gq, new THREE.Vector3())
    const pp = new THREE.Vector3(); const pq = new THREE.Quaternion()
    predicted.decompose(pp, pq, new THREE.Vector3())
    const posErr = pp.distanceTo(gp)              // metres
    // angle between unit quaternions, |dot|=1 means same orientation
    const dot = Math.min(1, Math.abs(gq.dot(pq)))
    const angErr = 2 * Math.acos(dot)             // radians
    const POS_TOL_M = 0.0015                      // 1.5 mm — toFixed(4) is 0.1mm, leave headroom
    const ANG_TOL_RAD = 0.01                      // ~0.57°
    if (posErr > POS_TOL_M || angErr > ANG_TOL_RAD) {
      console.warn(
        `[carry-commit-drift] ${label}: posErr=${(posErr * 1000).toFixed(3)}mm ` +
        `angErr=${((angErr * 180) / Math.PI).toFixed(3)}° (xyz="${xyzStr}" rpy="${rpyStr}")`
      )
    }
  }

  // Phase 3 — post-reparse drift check.
  // After `commitUrdf` rewrites the document and `reparseUrdf` rebuilds the
  // scene graph, downstream passes (reconcileAlignment, contactCleanup,
  // resolveAssemblyGraph) can shift the persisted link. If that shift is
  // larger than the user-visible noise floor we want to know: the user
  // committed at one pose and is now seeing another, which is the actual
  // "visible jump on commit" bug Phase 3 was scoped against.
  //
  // The check is scheduled via a 2-rAF chain so the reparse (itself
  // potentially rAF-deferred) and at least one render pass have completed
  // before we look up the link group. Pure observability — warns to
  // console only.
  function _scheduleCarryReparseDriftCheck(
    label: string,
    physicalLinkName: string,
    ghostWorld: THREE.Matrix4,
  ): void {
    requestAnimationFrame(() => requestAnimationFrame(() => {
      const linkGroup = ctx.getParsedRobot().linkGroups.get(physicalLinkName)
      if (!linkGroup) {
        // Common when the link got renamed / split; not a drift signal.
        return
      }
      linkGroup.updateMatrixWorld(true)
      // See `_assertCarryCommitInvariant`: link frame = ghost frame conjugated
      // by the basis swap, not ghost frame directly.
      const URDF_TO_SCENE_M = new THREE.Matrix4().makeRotationFromQuaternion(URDF_TO_SCENE_Q)
      const ghostLinkFrame = ghostWorld.clone().multiply(URDF_TO_SCENE_M)
      const gp = new THREE.Vector3(); const gq = new THREE.Quaternion()
      ghostLinkFrame.decompose(gp, gq, new THREE.Vector3())
      const lp = new THREE.Vector3(); const lq = new THREE.Quaternion()
      linkGroup.matrixWorld.decompose(lp, lq, new THREE.Vector3())
      const posErr = lp.distanceTo(gp)
      const dot = Math.min(1, Math.abs(gq.dot(lq)))
      const angErr = 2 * Math.acos(dot)
      // Slightly looser than the algebraic check — reconcile/contact-cleanup
      // can legitimately nudge by ~1 mm; we want to know about jumps the
      // user would actually perceive.
      const POS_TOL_M = 0.003                     // 3 mm
      const ANG_TOL_RAD = 0.02                    // ~1.15°
      if (posErr > POS_TOL_M || angErr > ANG_TOL_RAD) {
        console.warn(
          `[carry-reparse-drift] ${label} (link=${physicalLinkName}): ` +
          `posErr=${(posErr * 1000).toFixed(2)}mm ` +
          `angErr=${((angErr * 180) / Math.PI).toFixed(2)}°`,
        )
      }
    }))
  }

  // Core URDF mutation shared by addComponent (heuristic) and addComponentWithSnap (exact pose).
  function addComponentCore(
    comp: PresetComponent,
    parentLink: string,
    xyzStr: string,
    rpyStr: string,
    /** Optional debug hook — receives the base child link name BEFORE
     * commitUrdf triggers reparse. Used by the carry-commit drift check
     * to schedule a post-reparse pose comparison. Split servos still get
     * the un-suffixed `${comp.id}_${idx}` here; the hook can append
     * `_body` if it wants to look up the physical link. */
    onLinkCommitted?: (baseLinkName: string, isSplitServo: boolean) => void,
  ): boolean {
    const graph = ctx.getKinematicGraph()
    const nextIdx = Object.keys(graph).length + 1
    const childName = `${comp.id}_${nextIdx}`
    const jointName = `joint_${comp.id}_${nextIdx}`

    const phys = comp.physical
    const mass = resolveComponentMassKg(comp)
    const catName = findCategory(comp)
    const resolvedForSizing = resolveComponentVisual({ preset: comp, category: catName })
    const shape = phys.inertia_primitive || 'box'
    const xm = resolvedForSizing.bounds.hx * 2
    const ym = resolvedForSizing.bounds.hy * 2
    const zm = resolvedForSizing.bounds.hz * 2

    let inertia: { ixx: number; iyy: number; izz: number }
    if (shape === 'cylinder') {
      inertia = computeCylinderInertia(mass, Math.max(xm, ym) / 2, zm)
    } else if (shape === 'sphere') {
      inertia = computeSphereInertia(mass, xm / 2)
    } else {
      inertia = computeBoxInertia(mass, xm, ym, zm)
    }

    const category = comp.id.split('_')[0]
    const isActuated = isSplitServoComponentId(comp.id)
    const isNonSplitActuated = !isActuated && (category === 'actuator' || category === 'motor')

    const changed = commitUrdf(doc => {
      const robot = doc.documentElement
      if (!robot || robot.nodeName !== 'robot') return false

      if (isActuated) {
        // Split-servo emit: body link (fixed to parent) + horn link (revolute from body)
        const bodyLinkName = `${childName}_body`
        const hornLinkName = `${childName}_horn`
        const mountJointName = `${jointName}_mount`

        const bodyMass = mass * 0.95
        const hornMass = mass * 0.05
        const bodyInertia = computeBoxInertia(bodyMass, xm, ym, zm * 0.88)
        const hornInertia = computeBoxInertia(hornMass, xm * 0.7, ym * 0.7, zm * 0.12)
        const splitVisual = resolveSplitServoVisual({ preset: comp, category: catName })
        const hornOriginZ = splitVisual.hornOriginZ.toFixed(6)
        const bodyVisuals = splitVisual.bodyVisuals
        const hornVisuals = splitVisual.hornVisuals

        // Body link
        const bodyLink = doc.createElement('link')
        bodyLink.setAttribute('name', bodyLinkName)
        const bodyInertialEl = doc.createElement('inertial')
        const bodyMassEl = doc.createElement('mass'); bodyMassEl.setAttribute('value', bodyMass.toFixed(4))
        const bodyInertiaEl = doc.createElement('inertia')
        bodyInertiaEl.setAttribute('ixx', bodyInertia.ixx.toFixed(6)); bodyInertiaEl.setAttribute('iyy', bodyInertia.iyy.toFixed(6)); bodyInertiaEl.setAttribute('izz', bodyInertia.izz.toFixed(6))
        bodyInertiaEl.setAttribute('ixy', '0'); bodyInertiaEl.setAttribute('ixz', '0'); bodyInertiaEl.setAttribute('iyz', '0')
        bodyInertialEl.appendChild(bodyMassEl); bodyInertialEl.appendChild(bodyInertiaEl)
        bodyLink.appendChild(bodyInertialEl)
        bodyVisuals.forEach((vis, i) => addVisualElement(doc, bodyLink, vis, i))
        addResolvedCollisionSourceElements(doc, bodyLink, splitVisual.bodyCollision, bodyVisuals)

        // Mount joint: fixed, parent → body
        const mountJoint = doc.createElement('joint'); mountJoint.setAttribute('name', mountJointName); mountJoint.setAttribute('type', 'fixed')
        const mountParentEl = doc.createElement('parent'); mountParentEl.setAttribute('link', parentLink)
        const mountChildEl = doc.createElement('child'); mountChildEl.setAttribute('link', bodyLinkName)
        const mountOrigin = doc.createElement('origin'); mountOrigin.setAttribute('xyz', xyzStr); mountOrigin.setAttribute('rpy', rpyStr)
        mountJoint.appendChild(mountParentEl); mountJoint.appendChild(mountChildEl); mountJoint.appendChild(mountOrigin)

        // Horn link
        const hornLink = doc.createElement('link')
        hornLink.setAttribute('name', hornLinkName)
        const hornInertialEl = doc.createElement('inertial')
        const hornMassEl = doc.createElement('mass'); hornMassEl.setAttribute('value', hornMass.toFixed(4))
        const hornInertiaEl = doc.createElement('inertia')
        hornInertiaEl.setAttribute('ixx', hornInertia.ixx.toFixed(6)); hornInertiaEl.setAttribute('iyy', hornInertia.iyy.toFixed(6)); hornInertiaEl.setAttribute('izz', hornInertia.izz.toFixed(6))
        hornInertiaEl.setAttribute('ixy', '0'); hornInertiaEl.setAttribute('ixz', '0'); hornInertiaEl.setAttribute('iyz', '0')
        hornInertialEl.appendChild(hornMassEl); hornInertialEl.appendChild(hornInertiaEl)
        hornLink.appendChild(hornInertialEl)
        hornVisuals.forEach((vis, i) => addVisualElement(doc, hornLink, vis, i))
        addResolvedCollisionSourceElements(doc, hornLink, splitVisual.hornCollision, hornVisuals)

        // Revolute joint: body → horn at horn origin
        const revJoint = doc.createElement('joint'); revJoint.setAttribute('name', jointName); revJoint.setAttribute('type', 'revolute')
        const revParentEl = doc.createElement('parent'); revParentEl.setAttribute('link', bodyLinkName)
        const revChildEl = doc.createElement('child'); revChildEl.setAttribute('link', hornLinkName)
        const revOrigin = doc.createElement('origin'); revOrigin.setAttribute('xyz', `0 0 ${hornOriginZ}`); revOrigin.setAttribute('rpy', '0 0 0')
        const revAxis = doc.createElement('axis'); revAxis.setAttribute('xyz', '0 0 1')
        const revLimit = doc.createElement('limit')
        const [revLo, revHi] = resolveJointLimitsRad(comp)
        revLimit.setAttribute('lower', revLo.toFixed(5)); revLimit.setAttribute('upper', revHi.toFixed(5))
        const maxTorque = (comp.mechanical_electrical.max_torque_nm as number) ??
                          (comp.mechanical_electrical.holding_torque_nm as number) ?? 10
        revLimit.setAttribute('effort', String(maxTorque)); revLimit.setAttribute('velocity', '3.14')
        revJoint.appendChild(revParentEl); revJoint.appendChild(revChildEl); revJoint.appendChild(revOrigin); revJoint.appendChild(revAxis); revJoint.appendChild(revLimit)

        robot.appendChild(bodyLink); robot.appendChild(mountJoint)
        robot.appendChild(hornLink); robot.appendChild(revJoint)
      } else {
        const resolvedVisual = resolvedForSizing
        const visuals = resolvedVisual.visuals
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
        addResolvedCollisionElements(doc, link, resolvedVisual)

        const joint = doc.createElement('joint')
        joint.setAttribute('name', jointName)
        joint.setAttribute('type', isNonSplitActuated ? 'revolute' : 'fixed')
        const parentEl = doc.createElement('parent'); parentEl.setAttribute('link', parentLink)
        const childEl = doc.createElement('child'); childEl.setAttribute('link', childName)
        const origin = doc.createElement('origin')
        origin.setAttribute('xyz', xyzStr); origin.setAttribute('rpy', rpyStr)
        joint.appendChild(parentEl); joint.appendChild(childEl); joint.appendChild(origin)
        if (isNonSplitActuated) {
          const axis = doc.createElement('axis'); axis.setAttribute('xyz', '0 0 1')
          joint.appendChild(axis)
          const limit = doc.createElement('limit')
          const [lLo, lHi] = resolveJointLimitsRad(comp)
          limit.setAttribute('lower', lLo.toFixed(5)); limit.setAttribute('upper', lHi.toFixed(5))
          const maxTorque = (comp.mechanical_electrical.max_torque_nm as number) ??
                            (comp.mechanical_electrical.holding_torque_nm as number) ?? 10
          limit.setAttribute('effort', String(maxTorque)); limit.setAttribute('velocity', '3.14')
          joint.appendChild(limit)
        }
        robot.appendChild(link); robot.appendChild(joint)
      }
      return true
    }, { defer: true })

    const displayName = isActuated ? `${childName}_horn` : childName
    if (changed) {
      ctx.showToast(`Added ${comp.name} as "${childName}"`, 'success')
      selectLink(displayName)
      onLinkCommitted?.(childName, isActuated)
    }
    return changed
  }

  // ── Carry mode ───────────────────────────────────────────────────────────────

  let carryComp: PresetComponent | null = null
  let carryGroup: THREE.Group | null = null
  let carryGhostBounds: CarryGhostBounds | null = null
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
    carryArcLine.position.set(cx, cz, -cy)
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

  /** Swap the carry ghost's visual contents with the freshly-resolved preview
   * (mesh path now that the GLB is cached) while preserving the carryGroup's
   * current world transform — so the in-flight carry pose isn't lost when the
   * GLB load completes mid-carry. Bounds may also change between rich and mesh
   * paths; refreshed too. */
  function rebuildCarryGhostVisual(comp: PresetComponent) {
    if (!carryGroup) return
    const preview = computeCarryGhostPreview(comp)
    while (carryGroup.children.length > 0) {
      const child = carryGroup.children[0]
      carryGroup.remove(child)
    }
    carryGroup.add(preview.previewGroup
      ? makeCarryGhostPreviewGroup(preview.previewGroup, preview.authoredFrame)
      : makeCarryGhostVisualGroup(preview.visuals))
    carryGhostBounds = preview.bounds
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

    // Warm the mesh cache so first-of-type carry doesn't fall back to the
    // parametric placeholder (whose AABB differs from the GLB and triggers
    // an over-lift on commit). When the load completes mid-carry, rebuild the
    // ghost so its orientation/shape matches what the placed component will
    // render with — without the rebuild, ghost stays parametric (Y-up authored)
    // while the placed model uses the GLB (Z-up authored) and the user sees a
    // visible rotation mismatch between the two.
    preloadComponentMesh(comp.id, () => {
      if (carryComp?.id !== comp.id || !carryGroup) return
      rebuildCarryGhostVisual(comp)
    })

    const preview = computeCarryGhostPreview(comp)
    const bounds = preview.bounds
    carryGhostBounds = bounds
    carryGroup = new THREE.Group()
    carryGroup.name = 'carry_ghost'
    carryGroup.add(preview.previewGroup ? makeCarryGhostPreviewGroup(preview.previewGroup, preview.authoredFrame) : makeCarryGhostVisualGroup(preview.visuals))
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
    const { cx, cy, cz } = carryGhostBounds ?? computeCarryGhostPreview(carryComp).bounds
    const sourcePorts = resolveComponentPorts(carryComp)
    const sourceConnectors = resolveComponentConnectors(carryComp)
    carryGroup.updateMatrixWorld(true)
    return sourcePorts.map(f => {
      const connector = findConnectorForNode(sourceConnectors, f.nodeId)
      // bounds center + port origin live in URDF Z-up; carryGroup is Three.js Y-up.
      // Single seam: coordinates.urdfVecToSceneVec.
      const localFacePos = urdfVecToSceneVec([
        cx + f.origin_xyz[0],
        cy + f.origin_xyz[1],
        cz + f.origin_xyz[2],
      ])
      const lx = localFacePos.x, ly = localFacePos.y, lz = localFacePos.z
      const localAxis = axisFromPortOrConnector(f.origin_xyz, connector, true)
      const worldPos = localFacePos.clone().applyMatrix4(carryGroup!.matrixWorld)
      const worldQuat = new THREE.Quaternion().setFromRotationMatrix(carryGroup!.matrixWorld)
      const localToGhost = new THREE.Matrix4().makeTranslation(lx, ly, lz)
      const worldAxis = localAxis ? localAxis.clone().applyQuaternion(worldQuat).normalize() : null
      return { nodeId: f.nodeId, cls: f.cls, worldPos, worldQuat, localToGhost, localFacePos, localAxis, worldAxis }
    })
  }

  function updateCarrySnap() {
    if (!carryComp || !carryGroup) return
    refreshNodeWorldTransforms()
    const sourceNodes = getCarrySourceNodes()

    // Collect all valid candidates, sorted closest-first
    const candidates: SnapCandidate[] = []
    for (const src of sourceNodes) {
      for (const target of mountNodes) {
        if (isMountOccupied(target.mountLink)) continue
        const dist = target.worldPosition.distanceTo(src.worldPos)
        if (dist > snapRadiusM) continue
        if (!nodesCompatible(src.cls, target.cls)) continue
        // No angle check for carry mode — the ghost can be freely rotated with R key.
        const desiredGhostWorld = composeGhostWorldForConnectorSnap(
          { position: src.localFacePos, axis: src.localAxis, quaternion: src.worldQuat },
          { position: target.worldPosition, axis: target.worldAxis, quaternion: target.worldQuaternion },
        )
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
      const { hx, hy, hz, cx, cy, cz } = carryGhostBounds ?? computeCarryGhostPreview(carryComp).bounds
      // hz (URDF Z) is the Three.js Y (vertical) half-extent; swap hy↔hz and cy↔cz.
      const lifted = clampCarryMatrixAboveFloor(carryGroup.matrixWorld, hx, hz, hy, cx, cz, -cy)
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
    const ghostBoundsAtCommit = carryGhostBounds ?? computeCarryGhostPreview(comp).bounds
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
      // Phase 3 / carry-frame fix.
      // The ghost child carries a `componentVisualWorldQuat(z_up, scene_y_up)`
      // (-90°X) so its Z-up authored content displays correctly in scene Y-up.
      // The render path applies that swap one level higher, on `worldGroup`,
      // so the link group has identity quaternion below it. Result: at the
      // matrixWorld layer, `linkGroup.matrixWorld * inner * content` must
      // equal `carryGroup.matrixWorld * (-90X) * inner * content` for ghost
      // and placed visual to match — i.e. the link frame must be the carry
      // frame conjugated by the basis swap. `parentWorldInv` already supplies
      // the left side (worldGroup's +90X via inversion); the right side has
      // to be applied here, otherwise the persisted RPY bakes in a 90° flip
      // and tires lie flat / beams stand on end after commit.
      const URDF_TO_SCENE_M = new THREE.Matrix4().makeRotationFromQuaternion(URDF_TO_SCENE_Q)
      const childLocal = parentWorldInv.clone()
        .multiply(ghostWorldSnap ?? mount.desiredGhostWorld)
        .multiply(URDF_TO_SCENE_M)
      const localPos = new THREE.Vector3().setFromMatrixPosition(childLocal)
      const localQuat = new THREE.Quaternion()
      childLocal.decompose(new THREE.Vector3(), localQuat, new THREE.Vector3())
      const [lr, lp, ly] = quatToRpy(localQuat)
      const xyzStr = `${fmt(localPos.x)} ${fmt(localPos.y)} ${fmt(localPos.z)}`
      const rpyStr = `${fmt(lr)} ${fmt(lp)} ${fmt(ly)}`
      const capturedGhostWorld = ghostWorldSnap ?? mount.desiredGhostWorld
      _assertCarryCommitInvariant(
        `mount→${mount.targetParentLink}`,
        capturedGhostWorld,
        parentLinkGroup.matrixWorld,
        xyzStr, rpyStr,
      )
      addComponentCore(comp, mount.targetParentLink, xyzStr, rpyStr,
        (childName, isSplit) => {
          // Servo body is the link the mount joint connects to (Three.js
          // parent of horn). For non-splits the link IS childName.
          const physical = isSplit ? `${childName}_body` : childName
          _scheduleCarryReparseDriftCheck(
            `mount→${mount.targetParentLink}`, physical, capturedGhostWorld,
          )
        })
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
      const { hx: ghx, hy: ghy, hz: ghz, cx: gcx, cy: gcy, cz: gcz } = ghostBoundsAtCommit
      // ghz (URDF Z) is the Three.js Y (vertical) half-extent; swap ghy↔ghz and gcy↔gcz.
      // The clamp uses gcz as the Three.js Y center offset (non-zero for fallback path) so the
      // corners sample the actual mesh AABB, and the carryGroup origin (not visual center) becomes
      // the joint position — which is correct: URDF-primitive visuals sit at origin_xyz=[0,0,hz]
      // above the link origin, so joint Z=0 places their bottom on the floor.
      const ghostAdjusted = clampCarryMatrixAboveFloor(ghostWorldFree, ghx, ghz, ghy, gcx, gcz, -gcy)
      parentLinkGroup.updateMatrixWorld(true)
      const parentWorldInv = parentLinkGroup.matrixWorld.clone().invert()
      // Same basis-conjugation fix as the snapped branch — see comment above.
      const URDF_TO_SCENE_M_FREE = new THREE.Matrix4().makeRotationFromQuaternion(URDF_TO_SCENE_Q)
      const childLocal = parentWorldInv.clone()
        .multiply(ghostAdjusted)
        .multiply(URDF_TO_SCENE_M_FREE)
      const localPos = new THREE.Vector3().setFromMatrixPosition(childLocal)
      const localQuat = new THREE.Quaternion()
      childLocal.decompose(new THREE.Vector3(), localQuat, new THREE.Vector3())
      const [lr, lp, ly] = quatToRpy(localQuat)
      const xyzStr = `${fmt(localPos.x)} ${fmt(localPos.y)} ${fmt(localPos.z)}`
      const rpyStr = `${fmt(lr)} ${fmt(lp)} ${fmt(ly)}`
      _assertCarryCommitInvariant(
        `free→${parent}`,
        ghostAdjusted,
        parentLinkGroup.matrixWorld,
        xyzStr, rpyStr,
      )
      addComponentCore(comp, parent, xyzStr, rpyStr,
        (childName, isSplit) => {
          const physical = isSplit ? `${childName}_body` : childName
          _scheduleCarryReparseDriftCheck(`free→${parent}`, physical, ghostAdjusted)
        })
    } else {
      const graph = ctx.getKinematicGraph()
      const parent = resolveFreePlacementParent(graph)
      const resolved = resolveComponentVisual({ preset: comp, category: findCategory(comp) })
      const xm = resolved.bounds.hx * 2
      const ym = resolved.bounds.hy * 2
      const zm = resolved.bounds.hz * 2
      const doc = new DOMParser().parseFromString(ctx.getUrdfText(), 'application/xml')
      const placement = computePlacement(doc, parent, comp, xm, ym, zm, {
        cx: resolved.bounds.cx,
        cy: resolved.bounds.cy,
        cz: resolved.bounds.cz,
      })
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
    const halfBounds = resolveVisualHalfBoundsMm(comp)
    const bb = [halfBounds.hxMm * 2, halfBounds.hyMm * 2, halfBounds.hzMm * 2]
    const dims = `${Math.round(bb[0])}×${Math.round(bb[1])}×${Math.round(bb[2])} mm`
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
        return !q || (c.name ?? '').toLowerCase().includes(q) || c.id.toLowerCase().includes(q) || (c.description ?? '').toLowerCase().includes(q)
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
      reportComponentSpecDeprecations(data as unknown as Parameters<typeof reportComponentSpecDeprecations>[0])
      void ensureMeshExtentsLoaded()
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
      const path = await invoke<string | null>('save_file_dialog', {
        default_name: defaultName,
        filters: [['STL Files', ['stl']]],
      })
      if (!path) return

      // STLExporter walks every Mesh in the tree — including the torus rings
      // and connector-overlay spheres that aren't real robot geometry. Detach
      // them for the duration of the export, then put them back exactly where
      // they were. No scene rebuild, no transform math.
      type Detached = { node: THREE.Object3D; parent: THREE.Object3D }
      const detached: Detached[] = []
      group.traverse(obj => {
        if (!(obj instanceof THREE.Mesh)) return
        const ud = obj.userData as Record<string, unknown> | undefined
        const isOverlay = ud?.__connectorOverlay === true
        const isRing = obj.geometry instanceof THREE.TorusGeometry
        if ((isOverlay || isRing) && obj.parent) {
          detached.push({ node: obj, parent: obj.parent })
        }
      })
      for (const d of detached) d.parent.remove(d.node)
      let stlString: string
      try {
        stlString = stlExporter.parse(group, { binary: false }) as string
      } finally {
        for (const d of detached) d.parent.add(d.node)
      }
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
    const linkGroup = getInteractionLinkGroup(selectedLink)
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
        const startAabb = new THREE.Box3().setFromObject(pivot)
        gizmoDragStartMinY = isFinite(startAabb.min.y) ? startAabb.min.y : 0
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
        const parentLinkGroup = getInteractionLinkGroup(targetParent)
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
        }, { defer: true, skipGround: true })
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
        }, { defer: true, skipGround: true })
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
      }, { defer: true, skipGround: true })
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
    // Only enforce while the user is actively dragging the gizmo. The gizmo also
    // fires 'change' on attach/detach; applying the lift there would bump any link
    // whose AABB dips a hair below y=0 (float rounding after groundRobot, or a
    // wheel resting on the floor) upward the moment the user clicked it.
    const gizmoDragging = (gizmo as unknown as { dragging?: boolean }).dragging === true
    if (gizmoDragging && gizmo.mode === 'translate' && pivot.parent) {
      pivot.updateMatrixWorld(true)
      const aabb = new THREE.Box3().setFromObject(pivot)
      const minY = aabb.min.y
      // Threshold = min(0, drag-start min.y). Components resting on the floor
      // (wheels) have start min.y ≈ 0 with float-level negative noise; without
      // this, the first drag frame triggered a spurious lift by that noise.
      const floorThreshold = Math.min(0, gizmoDragStartMinY)
      if (minY < floorThreshold) {
        const correctionWorld = new THREE.Vector3(0, floorThreshold - minY, 0)
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
          const { hx: phx, hy: phy, hz: phz, cx: pcx, cy: pcy, cz: pcz } = carryGhostBounds ?? computeCarryGhostPreview(carryComp).bounds
          carryGroup.updateMatrixWorld(true)
          const lifted = clampCarryMatrixAboveFloor(carryGroup.matrixWorld, phx, phz, phy, pcx, pcz, -pcy)
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
    // Inspect mode (incl. forced-inspect during AI generation and sim) must not
    // re-attach the move gizmo or expose mount nodes when the URDF reparses,
    // otherwise a fresh AI-spawned robot lands with the previous selection's
    // gizmo live and placement nodes visible — letting the user drag parts
    // while the chat is supposedly the only writer.
    if (interactionMode === 'inspect') {
      selectedLink = null
      gizmo.detach()
      nodesGroup.visible = false
      ghostGroup.visible = false
      bestMountCandidate = null
      applyNodeRingVisibility()
      rebuildMountNodes()
      refreshBuildPanel()
      renderInspector()
      ctx.onAfterModelUpdated?.()
      return
    }
    if (selectedLink) {
      if (!resolveInteractionFrameLinkName(selectedLink)) {
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
    interactionMode = mode
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
  // Provenance of the in-memory graph. 'ai' means resolveAssemblyGraph just
  // ran (engine owns the URDF — reconcile may write back). 'restored' means
  // the graph came from localStorage / reverse-parse / checkpoint restore
  // (user owns the URDF — reconcile must NOT mutate it). Without this gate,
  // loading a file from disk drifts the editor away from the on-disk text by
  // sub-mm reconcile shifts, breaking byte-equal round-trips.
  let _graphSource: 'ai' | 'restored' | null = null

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

  function resolveAssemblyGraph(graph: AssemblyGraph): { urdf: string | null; topologyErrors?: string[]; topologyWarnings?: string[]; engineSummary?: EngineSummary } {
    _resetMultiChildPositionsCache()
    // Clear any archetype mode from a previous compile (set further down once
    // we've parsed the graph). Leaving stale state would let a previous run's
    // novel-mode bleed into a fresh standard-mode compile.
    setArchetypeMode(null)
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

    // Phase 3: archetype normalizer runs
    // before topology auto-repair so cosmetic-tail removal happens at the
    // semantic-graph layer, and the AI sees structured `[ai_topology/...]`
    // diagnostics in the redesign prompt instead of free-text post-hoc errors.
    const requestedFeatures: RequestedFeatures = (graph as { requested_features?: RequestedFeatures })
      .requested_features ?? {}
    const declaredArchetype = (graph as { _archetype_mode?: 'standard' | 'novel' })._archetype_mode
    // Stash on the placement-compiler context so deeply-nested helpers
    // (multiChild distribution, splay) can branch on novel mode without
    // every signature growing an `archetypeMode` parameter. Reset after
    // compilation completes, in the finally branch below.
    setArchetypeMode(declaredArchetype)
    const archResult = normalizeAssembly(graph.components, requestedFeatures, declaredArchetype)
    if (archResult.diagnostics.length > 0) {
      graph.components = archResult.components
    }

    const { repairs } = runAutoRepair(graph, validationCtx)
    void repairs

    const { errors: topologyErrors, warnings: topologyWarnings } = runValidateTopology(graph.components, validationCtx)
    // Surface archetype diagnostics on the same channel the retry/redesign
    // prompt already pulls from (viewportChat.formatWarningsForPrompt). The
    // owner tag in `formatDiagnosticForPrompt` lets the AI distinguish its
    // own topology mistakes from spec/compiler/exporter issues.
    for (const d of archResult.diagnostics) {
      if (d.severity === 'info') continue
      topologyWarnings.push(formatDiagnosticForPrompt(d))
    }
    if (topologyWarnings.length > 0) {
      for (const w of topologyWarnings) console.warn(`[assembly][topology][warning] ${w}`)
    }

    // Connector-reference pre-pass: walk the graph once and surface any
    // attach_connector / mate_connector that doesn't resolve on its target
    // preset as a topology error. Without this the placement loop would hit
    // the dev-throw at urdfAssembly.ts:2048 (computeMatePlacement) and
    // hard-crash an AI-generated build instead of letting the redesign loop
    // see a recoverable signal. The dev-throw stays as defense-in-depth for
    // genuine preset coverage holes that escape this pre-pass. See
    // memory/project_engine_regression_connector_miss.md.
    const connectorErrors = runValidateConnectorRefs(graph.components, validationCtx)
    for (const e of connectorErrors) topologyErrors.push(e)

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
    const catName = findCategory(rootPreset)
    const rootResolved = resolveComponentVisual({
      preset: rootPreset,
      category: catName,
      instance: root,
    })
    const xm = rootResolved.bounds.hx * 2
    const ym = rootResolved.bounds.hy * 2
    const zm = rootResolved.bounds.hz * 2
    const mass = resolveComponentMassKg({ id: root.component_id, physical: phys }, root)
    const shape = phys.inertia_primitive || 'box'
    let inertia: { ixx: number; iyy: number; izz: number }
    if (shape === 'cylinder') inertia = computeCylinderInertia(mass, Math.max(xm, ym) / 2, zm)
    else if (shape === 'sphere') inertia = computeSphereInertia(mass, xm / 2)
    else inertia = computeBoxInertia(mass, xm, ym, zm)

    const rootLinkName = root.link_name
    const visuals = rootResolved.visuals

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

    // Phase 5b: collision is always the canonical AABB envelope (collision.bounds),
    // matching what the placement compiler and split-link emitter use.
    let collisionsXml = ''
    {
      const bounds = rootResolved.collision.bounds
      collisionsXml += `
    <collision>
      <origin xyz="${[bounds.cx, bounds.cy, bounds.cz].map(v => v.toFixed(6)).join(' ')}" rpy="0 0 0"/>
      <geometry><box size="${[bounds.hx * 2, bounds.hy * 2, bounds.hz * 2].map(v => v.toFixed(6)).join(' ')}"/></geometry>
    </collision>`
    }

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

    // Layer 3 — track which children were placed via the authored connector
    // path so reconcile can skip them. Key = comp.link_name (Claude's name,
    // pre-remap). Set when computeMatePlacement fires OR Layer-2's
    // computeFacePlacement override engages. Threaded into _lastAssemblyGraph
    // below so reconcileAlignment.ts sees the flag without a separate channel.
    // Hoisted above the try block so the post-try remappedComponents map can
    // read it.
    const viaConnectorMap = new Map<string, boolean>()

    // Ground-truth placement rows (xyz/rpy actually written to URDF, per child)
    // captured inline during the placement loop. Threaded to the Gemini validator
    // so screenshot misreads can be refuted against authoritative engine output
    //. Hoisted above the try so
    // the final return (which runs outside the try) can read it.
    const placementEntries: EnginePlacementEntry[] = []

    // Hoisted above the try so the shadow-compile parity harness (Phase 3b.4.A)
    // can read the world-transform map after the try/finally without a
    // separate channel. Reset on every resolve.
    const linkWorldTransforms = new Map<string, THREE.Matrix4>()

    // try/finally is load-bearing: if the loop throws we MUST clear _bulkMode,
    // else every future commitUrdf in the session writes to an orphaned buffer.
    setBulkAssemblyMode(true)
    try {

    nameMap.set(root.link_name, rootLinkName)
    processed.add(root.link_name)
    placedCount++
    linkWorldTransforms.set(rootLinkName, new THREE.Matrix4())
    console.log(`[assembly] Root placed: ${rootLinkName} (${root.component_id})`)

    const componentByName = new Map(components.map(c => [c.link_name, c]))
    for (const comp of components) {
      const axis = axisNameFromComponentAxis(comp.joint_axis)
      const parent = comp.attach_to ? componentByName.get(comp.attach_to) : undefined
      const hasServoChild = components.some(c => c.attach_to === comp.link_name && isSplitServoComponentId(c.component_id))
      const isCompoundHipBaseServo = isSplitServoComponentId(comp.component_id)
        && axis !== 'z'
        && (comp.attach_face === 'top' || comp.attach_face === 'bottom')
        && !!parent
        && parent.component_id.startsWith('structural_baseplate')
        && hasServoChild
      if (isCompoundHipBaseServo) {
        comp.joint_axis = 'z'
      }
    }

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

    // Track port occupancy: how many children are connected to each parent port
    // Key format: "parent_link_name::face_name"
    const portOccupancy = new Map<string, number>()

    // Opposite face mapping: child's contact surface is opposite of parent's attach face
    const oppositeFace: Record<string, string> = {
      top: 'bottom', bottom: 'top',
      front: 'back', back: 'front',
      left: 'right', right: 'left',
    }

    // Phase 3b.4.K — sub-step A. Build the CompiledGraph upfront so the
    // emit loop can override its inline-computed placement values with
    // compiler-derived ones. This is the source of truth from this point on;
    // sub-step B replaces the inline math, sub-step C deletes it.
    const compileInputForEmit: AssemblyGraph = {
      base_link: graph.base_link,
      ground_offset: graph.ground_offset,
      components: components.map(c => ({ ...c })),
    }
    const compiledForEmit = compileAssembly(compileInputForEmit, {
      useMateConnectors: useMateConnectors(),
      resolveComponent: (componentId, instance) => {
        const preset = findPreset(componentId)
        if (!preset) return null
        const resolved = resolveComponentVisual({
          preset,
          category: findCategory(preset),
          instance,
        })
        // Phase 5: placement compiler reads the collision envelope (authored
        // mesh AABB + measured center when present, preset bbox otherwise).
        // resolved.bounds is mesh-target-bbox with cz=0 — fine for rendering
        // but loses the collision mesh's center offset, which is what makes
        // the body's mount face land where the GLB renders it. See Phase 5
        // notes in the unification plan.
        const b = resolved.collision.bounds
        return {
          componentId,
          bounds: { half: [b.hx, b.hy, b.hz], center: [b.cx, b.cy, b.cz], shape: b.shape },
          connectors: resolveComponentConnectors(preset, instance),
          presetConnectors: preset.connectors,
          assembledOuterRadiusM: typeof preset.mounting_logic?.assembled_outer_radius_mm === 'number'
            ? preset.mounting_logic.assembled_outer_radius_mm / 1000 : undefined,
          parametricLengthMm: (instance?.length_mm && isParametricSpec(preset))
            ? instance.length_mm : undefined,
          jointLimitsRad: resolveJointLimitsRad(preset),
          maxTorqueNm: typeof preset.mechanical_electrical?.max_torque_nm === 'number'
            ? preset.mechanical_electrical.max_torque_nm
            : (typeof preset.mechanical_electrical?.holding_torque_nm === 'number'
              ? preset.mechanical_electrical.holding_torque_nm : undefined),
        }
      },
    })
    const compiledByLogical = new Map<string, typeof compiledForEmit.links[number]>()
    for (const l of compiledForEmit.links) compiledByLogical.set(l.logicalName, l)
    console.log(`[assembly] compileAssembly produced ${compiledForEmit.links.length} links (skipped=${compiledForEmit.skippedClasses.length})`)

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
      // different-length instance (cache is keyed by component ID). Always use the
      // explicit length when specified.
      let czm = childBounds.hz * 2
      if (comp.length_mm && isParametricSpec(preset)) {
        czm = comp.length_mm / 1000
      }

      // ── Port-based connection validation ──
      // Resolve parent and child ports, check compatibility and occupancy.
      const parentCompDef = components.find(c => c.link_name === comp.attach_to)
      const parentPreset = findPreset(parentCompDef?.component_id || '')
      const childPreset = findPreset(comp.component_id)
      const attachFace = comp.attach_face || 'top'
      const childFace = attachFace === 'coaxial' ? 'coaxial' : (oppositeFace[attachFace] || 'bottom')

      let parentPort: ReturnType<typeof resolveFaceToPort> = undefined
      let childPort: ReturnType<typeof resolveFaceToPort> = undefined

      if (parentPreset) {
        const parentPorts = resolveComponentPorts(parentPreset, parentCompDef ?? undefined)
        parentPort = resolveFaceToPort(attachFace, parentPorts)

        if (childPreset) {
          const childPorts = resolveComponentPorts(childPreset, comp)
          childPort = resolveFaceToPort(childFace, childPorts)
        }
      }

      // Track per-port occupancy: enforce single-use ports
      const portOccupancyKey = `${comp.attach_to}::${parentPort?.nodeId ?? attachFace}`
      const currentOccupancy = portOccupancy.get(portOccupancyKey) || 0
      if (parentPort?.single && currentOccupancy > 0 && parentPort.cls === 'shaft') {
        console.warn(`[assembly][ports] ${parentPreset!.id}.${parentPort.nodeId} (shaft) already has ${currentOccupancy} child(ren) — multiple children on a shaft output is unusual`)
      }
      portOccupancy.set(portOccupancyKey, currentOccupancy + 1)

      const requestedJointType = normalizeJointType(comp.joint_type)
      const connectionJoint = parentPort && childPort
        ? resolveConnectionJoint(parentPort, childPort, {
          joint_type: requestedJointType,
          axis_xyz: axisNameToTuple(comp.joint_axis),
        })
        : { joint_type: requestedJointType, axis_xyz: axisNameToTuple(comp.joint_axis) }
      const jointType = connectionJoint.joint_type
      comp.joint_type = jointType

      // Placement xyz/rpy and joint axis are sourced from the placement
      // compiler below. Initialize placeholders consumed by the URDF emission.
      let placement: { xyz: string; rpy: string } = { xyz: '0 0 0', rpy: '0 0 0' }
      let jointAxis = axisTupleToUrdf(connectionJoint.axis_xyz)

      // Use addComponentCore but we need to override joint type and axis
      // Since addComponentCore auto-determines joint type from category,
      // we'll directly build the URDF element for more control.
      // Use placedCount (not getKinematicGraph().length): the graph only updates
      // on reparse, and bulk mode skips per-iteration reparses — so without this,
      // every child would get `_2` and produce duplicate URDF link names.
      const nextIdx2 = placedCount + 1
      const childName = `${preset.id}_${nextIdx2}`
      const jointName = `joint_${preset.id}_${nextIdx2}`

      const cMass = resolveComponentMassKg(preset, comp)
      const cShape = cPhys.inertia_primitive || 'box'
      let cInertia: { ixx: number; iyy: number; izz: number }
      if (cShape === 'cylinder') cInertia = computeCylinderInertia(cMass, Math.max(cxm, cym) / 2, czm)
      else if (cShape === 'sphere') cInertia = computeSphereInertia(cMass, cxm / 2)
      else cInertia = computeBoxInertia(cMass, cxm, cym, czm)

      const cCatName = findCategory(preset)
      const resolvedVisual = resolveComponentVisual({
        preset,
        category: cCatName,
        instance: comp,
      })
      const cVisuals = resolvedVisual.visuals
      const cIsActuated = isSplitServoComponentId(preset.id)
      const servoAxisName = axisNameFromUrdf(jointAxis)
      const servoUsesSideYoke = cIsActuated && servoAxisName !== 'z'
      const useCompoundServoCarrier = !!(
        cIsActuated &&
        parentCompDef &&
        isSplitServoComponentId(parentCompDef.component_id)
      )
      const parentWorldTransform = linkWorldTransforms.get(parentLinkName)

      // ── Phase 3b.4.K — placement values sourced from CompiledGraph ────────
      // The placement compiler is now the single owner of: face/mate/servo
      // placement, distal-beam-bottom flip, arm rest pose, attach_rpy override,
      // foot pad world-leveling, servo mount rpy, side-axis dz nudge. See
      // placementCompiler/index.ts. Every component must have a CompiledLink
      // (parity-verified: matched=N/N, unmatched=0).
      const _cl = compiledByLogical.get(comp.link_name)
      if (!_cl) {
        console.warn(`[assembly] No CompiledLink for ${comp.link_name} — skipping`)
        processed.add(comp.link_name)
        continue
      }
      const _fmt4 = (t: [number, number, number]) =>
        t.map(v => Number(v || 0).toFixed(4)).join(' ')
      const _fmtAxis = (t: [number, number, number]) =>
        t.map(v => Math.round(v).toString()).join(' ')
      let finalRpy: string
      let servoHornZeroRpy = '0 0 0'
      let servoBodyMountRpy: string
      if (cIsActuated) {
        // Servo joints[]: optional carrier (compound), mount (fixed),
        // revolute (last). XYZ/mountRpy live on whichever fixed joint sits
        // between parent and body — carrier when compound, mount otherwise.
        const _revIdx = _cl.joints.length - 1
        const _bodyMountIdx = useCompoundServoCarrier ? 0 : (_cl.joints.length === 3 ? 1 : 0)
        const _bodyMount = _cl.joints[_bodyMountIdx]
        const _rev = _cl.joints[_revIdx]
        placement = { xyz: _fmt4(_bodyMount.originXyz), rpy: _fmt4(_bodyMount.originRpy) }
        servoBodyMountRpy = _fmt4(_bodyMount.originRpy)
        servoHornZeroRpy = _fmt4(_rev.originRpy)
        finalRpy = placement.rpy
      } else {
        const _j = _cl.joints[0]
        placement = { xyz: _fmt4(_j.originXyz), rpy: _fmt4(_j.originRpy) }
        finalRpy = _fmt4(_j.originRpy)
        jointAxis = _fmtAxis(_j.axis)
        servoBodyMountRpy = finalRpy   // unused for non-actuated; satisfies type
      }
      if (_cl.placedViaConnector) viaConnectorMap.set(comp.link_name, true)
      const servoHornOriginZ = cIsActuated ? czm * SERVO_HORN_ORIGIN_Z_RATIO : 0

      const changed = commitUrdf(urdfDoc => {
        const robot = urdfDoc.querySelector('robot')
        if (!robot) return false

        if (cIsActuated) {
          // Split-servo emit: body link (fixed to parent) + horn link (revolute from body)
          const bodyLinkName = `${childName}_body`
          const hornLinkName = `${childName}_horn`
          const carrierLinkName = `${childName}_compound_carrier`
          const carrierJointName = `${jointName}_compound_carrier`
          const mountJointName = `${jointName}_mount`

          const bodyMass = cMass * 0.95
          const hornMass = cMass * 0.05
          const carrierMass = useCompoundServoCarrier ? Math.max(cMass * 0.18, 0.025) : 0
          const bodyInertia = computeBoxInertia(bodyMass, cxm, cym, czm * 0.88)
          const hornInertia = computeBoxInertia(hornMass, cxm * 0.7, cym * 0.7, czm * 0.12)
          const carrierInertia = computeBoxInertia(carrierMass || 0.001, cxm * 1.2, cym * 1.4, czm * 1.2)
          const splitVisual = resolveSplitServoVisual({
            preset,
            category: cCatName,
            includeSideYoke: servoUsesSideYoke,
          })
          const hornOriginZ = splitVisual.hornOriginZ.toFixed(6)
          const bodyVisuals = splitVisual.bodyVisuals
          const hornVisuals = splitVisual.hornVisuals

          // Body link
          const bodyLink = urdfDoc.createElement('link'); bodyLink.setAttribute('name', bodyLinkName)
          const bodyInertialEl = urdfDoc.createElement('inertial')
          const bodyMassEl = urdfDoc.createElement('mass'); bodyMassEl.setAttribute('value', bodyMass.toFixed(4))
          const bodyInertiaEl = urdfDoc.createElement('inertia')
          bodyInertiaEl.setAttribute('ixx', bodyInertia.ixx.toFixed(6)); bodyInertiaEl.setAttribute('iyy', bodyInertia.iyy.toFixed(6)); bodyInertiaEl.setAttribute('izz', bodyInertia.izz.toFixed(6))
          bodyInertiaEl.setAttribute('ixy', '0'); bodyInertiaEl.setAttribute('ixz', '0'); bodyInertiaEl.setAttribute('iyz', '0')
          bodyInertialEl.appendChild(bodyMassEl); bodyInertialEl.appendChild(bodyInertiaEl)
          bodyLink.appendChild(bodyInertialEl)
          bodyVisuals.forEach((vis, i) => addVisualElement(urdfDoc, bodyLink, vis, i))
          addResolvedCollisionSourceElements(urdfDoc, bodyLink, splitVisual.bodyCollision, bodyVisuals)

          // Mount joint: fixed, parent → body
          if (useCompoundServoCarrier) {
            const carrierLink = urdfDoc.createElement('link')
            carrierLink.setAttribute('name', carrierLinkName)
            const carrierInertialEl = urdfDoc.createElement('inertial')
            const carrierMassEl = urdfDoc.createElement('mass'); carrierMassEl.setAttribute('value', carrierMass.toFixed(4))
            const carrierInertiaEl = urdfDoc.createElement('inertia')
            carrierInertiaEl.setAttribute('ixx', carrierInertia.ixx.toFixed(6)); carrierInertiaEl.setAttribute('iyy', carrierInertia.iyy.toFixed(6)); carrierInertiaEl.setAttribute('izz', carrierInertia.izz.toFixed(6))
            carrierInertiaEl.setAttribute('ixy', '0'); carrierInertiaEl.setAttribute('ixz', '0'); carrierInertiaEl.setAttribute('iyz', '0')
            carrierInertialEl.appendChild(carrierMassEl); carrierInertialEl.appendChild(carrierInertiaEl)
            carrierLink.appendChild(carrierInertialEl)
            const carrierReach = Math.hypot(...parseXyzString(placement.xyz))
            const carrierVisuals = servoCompoundCarrierVisuals(cxm, cym, czm, carrierReach)
            carrierVisuals.forEach((vis, i) => addVisualElement(urdfDoc, carrierLink, vis, i))
            carrierVisuals.forEach(vis => addCollisionElement(urdfDoc, carrierLink, vis))

            const carrierJoint = urdfDoc.createElement('joint'); carrierJoint.setAttribute('name', carrierJointName); carrierJoint.setAttribute('type', 'fixed')
            const carrierParentEl = urdfDoc.createElement('parent'); carrierParentEl.setAttribute('link', parentLinkName)
            const carrierChildEl = urdfDoc.createElement('child'); carrierChildEl.setAttribute('link', carrierLinkName)
            const carrierOrigin = urdfDoc.createElement('origin'); carrierOrigin.setAttribute('xyz', placement.xyz); carrierOrigin.setAttribute('rpy', servoBodyMountRpy)
            carrierJoint.appendChild(carrierParentEl); carrierJoint.appendChild(carrierChildEl); carrierJoint.appendChild(carrierOrigin)
            robot.appendChild(carrierLink); robot.appendChild(carrierJoint)
          }

          const mountJoint = urdfDoc.createElement('joint'); mountJoint.setAttribute('name', mountJointName); mountJoint.setAttribute('type', 'fixed')
          const mountParentEl = urdfDoc.createElement('parent'); mountParentEl.setAttribute('link', useCompoundServoCarrier ? carrierLinkName : parentLinkName)
          const mountChildEl = urdfDoc.createElement('child'); mountChildEl.setAttribute('link', bodyLinkName)
          const mountOrigin = urdfDoc.createElement('origin')
          mountOrigin.setAttribute('xyz', useCompoundServoCarrier ? '0 0 0' : placement.xyz)
          mountOrigin.setAttribute('rpy', useCompoundServoCarrier ? '0 0 0' : servoBodyMountRpy)
          mountJoint.appendChild(mountParentEl); mountJoint.appendChild(mountChildEl); mountJoint.appendChild(mountOrigin)

          // Horn link
          const hornLink = urdfDoc.createElement('link'); hornLink.setAttribute('name', hornLinkName)
          const hornInertialEl = urdfDoc.createElement('inertial')
          const hornMassEl = urdfDoc.createElement('mass'); hornMassEl.setAttribute('value', hornMass.toFixed(4))
          const hornInertiaEl2 = urdfDoc.createElement('inertia')
          hornInertiaEl2.setAttribute('ixx', hornInertia.ixx.toFixed(6)); hornInertiaEl2.setAttribute('iyy', hornInertia.iyy.toFixed(6)); hornInertiaEl2.setAttribute('izz', hornInertia.izz.toFixed(6))
          hornInertiaEl2.setAttribute('ixy', '0'); hornInertiaEl2.setAttribute('ixz', '0'); hornInertiaEl2.setAttribute('iyz', '0')
          hornInertialEl.appendChild(hornMassEl); hornInertialEl.appendChild(hornInertiaEl2)
          hornLink.appendChild(hornInertialEl)
          hornVisuals.forEach((vis, i) => addVisualElement(urdfDoc, hornLink, vis, i))
          addResolvedCollisionSourceElements(urdfDoc, hornLink, splitVisual.hornCollision, hornVisuals)

          // Revolute joint: body → horn at horn origin
          const revJoint = urdfDoc.createElement('joint'); revJoint.setAttribute('name', jointName); revJoint.setAttribute('type', 'revolute')
          const revParentEl = urdfDoc.createElement('parent'); revParentEl.setAttribute('link', bodyLinkName)
          const revChildEl = urdfDoc.createElement('child'); revChildEl.setAttribute('link', hornLinkName)
          const revOrigin = urdfDoc.createElement('origin'); revOrigin.setAttribute('xyz', `0 0 ${hornOriginZ}`); revOrigin.setAttribute('rpy', servoHornZeroRpy)
          const revAxis = urdfDoc.createElement('axis'); revAxis.setAttribute('xyz', '0 0 1')
          const revLimit = urdfDoc.createElement('limit')
          const [rLo2, rHi2] = resolveJointLimitsRad(preset)
          revLimit.setAttribute('lower', rLo2.toFixed(5)); revLimit.setAttribute('upper', rHi2.toFixed(5))
          const me = preset.mechanical_electrical || {}
          const maxTorque = (me.max_torque_nm as number) ?? (me.holding_torque_nm as number) ?? 10
          revLimit.setAttribute('effort', String(maxTorque)); revLimit.setAttribute('velocity', '3.14')
          revJoint.appendChild(revParentEl); revJoint.appendChild(revChildEl); revJoint.appendChild(revOrigin); revJoint.appendChild(revAxis); revJoint.appendChild(revLimit)

          robot.appendChild(bodyLink); robot.appendChild(mountJoint)
          robot.appendChild(hornLink); robot.appendChild(revJoint)
        } else {
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
          addResolvedCollisionElements(urdfDoc, link, resolvedVisual)

          const joint = urdfDoc.createElement('joint')
          joint.setAttribute('name', jointName)
          joint.setAttribute('type', jointType)

          const parentEl = urdfDoc.createElement('parent'); parentEl.setAttribute('link', parentLinkName)
          const childEl = urdfDoc.createElement('child'); childEl.setAttribute('link', childName)
          const origin = urdfDoc.createElement('origin')
          origin.setAttribute('xyz', placement.xyz)
          origin.setAttribute('rpy', finalRpy)
          const axis = urdfDoc.createElement('axis'); axis.setAttribute('xyz', jointAxis)
          joint.appendChild(parentEl); joint.appendChild(childEl); joint.appendChild(origin); joint.appendChild(axis)

          if (jointType === 'revolute' || jointType === 'prismatic') {
            const limit = urdfDoc.createElement('limit')
            const [pLo, pHi] = resolveJointLimitsRad(preset)
            limit.setAttribute('lower', pLo.toFixed(5)); limit.setAttribute('upper', pHi.toFixed(5))
            const me = preset.mechanical_electrical || {}
            const maxTorque = (me.max_torque_nm as number) ?? (me.holding_torque_nm as number) ?? 10
            limit.setAttribute('effort', String(maxTorque)); limit.setAttribute('velocity', '3.14')
            joint.appendChild(limit)
          }

          robot.appendChild(link); robot.appendChild(joint)
        }
        return true
      })

      if (changed) {
        // Route children to the horn link for actuated components
        nameMap.set(comp.link_name, cIsActuated ? `${childName}_horn` : childName)
        const parentWorld = parentWorldTransform ?? new THREE.Matrix4()
        if (cIsActuated) {
          const bodyWorld = new THREE.Matrix4().multiplyMatrices(
            parentWorld,
            transformFromXyzRpy(placement.xyz, servoBodyMountRpy),
          )
          if (useCompoundServoCarrier) {
            linkWorldTransforms.set(`${childName}_compound_carrier`, bodyWorld)
          }
          const hornWorld = new THREE.Matrix4().multiplyMatrices(
            bodyWorld,
            transformFromXyzRpy(`0 0 ${servoHornOriginZ.toFixed(6)}`, servoHornZeroRpy),
          )
          linkWorldTransforms.set(`${childName}_body`, bodyWorld)
          linkWorldTransforms.set(`${childName}_horn`, hornWorld)
        } else {
          const childWorld = new THREE.Matrix4().multiplyMatrices(
            parentWorld,
            transformFromXyzRpy(placement.xyz, finalRpy),
          )
          linkWorldTransforms.set(childName, childWorld)
        }
        placedCount++
        placementEntries.push({
          linkName: childName,
          parentLinkName,
          xyz: placement.xyz,
          rpy: cIsActuated ? servoBodyMountRpy : finalRpy,
        })
        // Reparse so next component sees updated geometry. Skipped in bulk mode
        // — parentBoundsFromLink reads the resolver, not the rendered-mesh cache,
        // so the in-progress URDF buffer is sufficient.
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
      placed_via_connector: viaConnectorMap.get(c.link_name) || c.placed_via_connector,
    }))
    const remappedBase = nameMap.get(graph.base_link) || graph.base_link
    _lastAssemblyGraph = {
      base_link: remappedBase,
      ground_offset: graph.ground_offset,
      // Preserve archetype mode across the resolve→persist round-trip so a
      // novel-mode design survives reload, undo/redo, and subsequent
      // modify_topology turns. Without this, every successful resolve
      // silently dropped the mode and the next render used the dog template.
      _archetype_mode: graph._archetype_mode,
      components: remappedComponents,
    }
    _graphSource = 'ai'  // engine owns this URDF — reconcile may write back
    _persistGraph(_lastAssemblyGraph)
    console.log(`[assembly] Stored assembly graph (${remappedComponents.length} components, URDF names) for modify_topology`)

    // Render-time alignment: shift pivots so visible mesh faces meet. Must run
    // BEFORE groundAssembly because groundRobot measures post-reconcile world
    // extents. Safe when meshes haven't loaded yet — the EPS guard no-ops any
    // link whose parent/child AABB is unavailable or already aligned, and the
    // debounced onMeshLoaded path re-fires reconcile once GLBs settle.
    let icpEntriesForSummary: EngineIcpEntry[] = []
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

      // Step 2 — runtime ICP contact cleanup (docs/ENGINE_NEXT_STEPS.md).
      // Runs AFTER reconcile so the scene is already flush where it can be
      // via pure placement math. Then we raycast each mated pair's face and
      // sink the child slightly into the parent to hide chamfers / beveled
      // edges that flat placement can't close. Persist the same way reconcile
      // does so the debounced reparse doesn't wipe the adjustments.
      try {
        const cleanupRes = runContactCleanupPass()
        icpEntriesForSummary = cleanupRes.icpEntries
        if (cleanupRes.adjustedCount > 0) {
          persistReconcileShiftsToUrdf(ctx.getParsedRobot().linkGroups)
          rebuildMountNodes()
          const totalMm = cleanupRes.shifts.reduce((a, s) => a + s.dMm, 0)
          console.log(`[icp] Done: ${cleanupRes.adjustedCount} link(s) nudged, total nudge = ${totalMm.toFixed(2)}mm`)
        } else {
          console.log(`[icp] Done: 0 link(s) nudged (all mates flush within ${NUDGE_MIN_MM}mm tolerance)`)
        }
      } catch (e) {
        console.warn('[assembly] runContactCleanupPass failed:', e)
      }
    } catch (e) {
      console.warn('[assembly] reconcileNodePlacement failed:', e)
    }

    // Ground the robot so it sits on the floor plane (Y=0 in Three.js)
    try { ctx.groundAssembly?.() } catch (e) { console.warn('[assembly] groundAssembly failed:', e) }

    ctx.showToast(`Assembled ${placedCount} components`, 'success')

    const engineSummary: EngineSummary = {
      placements: placementEntries,
      icpGaps: icpEntriesForSummary,
    }
    return {
      urdf: ctx.getUrdfText(),
      topologyWarnings: topologyWarnings.length > 0 ? topologyWarnings : undefined,
      engineSummary,
    }
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

  function applyTopologyOps(
    graph: AssemblyGraph,
    operations: TopologyOp[],
    archetypeOverride?: 'standard' | 'novel',
  ): AssemblyGraph {
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
          attach_connector: op.attach_connector,
          mate_connector: op.mate_connector,
          mate_type: op.mate_type,
          placement_offset_mm: op.placement_offset_mm,
          splay_angle_deg: op.splay_angle_deg,
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
        if (op.attach_connector !== undefined) existing.attach_connector = op.attach_connector
        if (op.mate_connector !== undefined) existing.mate_connector = op.mate_connector
        if (op.mate_type !== undefined) existing.mate_type = op.mate_type
        if (op.placement_offset_mm !== undefined) existing.placement_offset_mm = op.placement_offset_mm
        if (op.splay_angle_deg !== undefined) existing.splay_angle_deg = op.splay_angle_deg
        console.log(`[topology] Modified ${op.link_name}: ${JSON.stringify(op)}`)
      }
    }

    // Preserve `_archetype_mode` across topology edits — without this, every
    // modify_topology turn silently reverts a novel design to standard mode
    // and the placement compiler reapplies the dog/arm/wheeled template.
    // Caller-provided override (Claude's per-turn declaration) wins; otherwise
    // inherit the existing graph's declaration.
    const inheritedMode = graph._archetype_mode
    const finalMode = archetypeOverride ?? inheritedMode
    return {
      base_link: graph.base_link,
      ground_offset: true,
      _archetype_mode: finalMode,
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
      // Skip entirely for user-owned URDFs (file load, checkpoint restore,
      // reverse-parse). The user's saved URDF is the source of truth — neither
      // the visual scene nor the editor text should be second-guessed by the
      // engine on reload. AI flow keeps reconcile because the engine just
      // generated the URDF and owns its placement.
      if (_graphSource !== 'ai') {
        return { adjustedCount: 0, residualMaxMm: 0, shifts: [] }
      }
      const parsed = ctx.getParsedRobot()
      const res = reconcileNodePlacement({
        graph: _lastAssemblyGraph,
        linkGroups: parsed.linkGroups,
        joints: parsed.joints,
      })
      // I1-bis: mirror the first-pass persistence so the debounced
      // onMeshLoaded path (main.ts) doesn't leave shifts only in the live
      // scene — without this the next reparse rebuilds from the un-baked
      // URDF and replays identical shifts on every mesh-settle cycle.
      // Must also rebuildMountNodes to match the first-pass pair at line
      // ~3940: mount nodes were placed against the pre-reconcile pivots,
      // and persisting without rebuilding leaves attachment rings on the
      // old pose while the URDF and scene have moved on.
      //
      // Provenance gate: only the AI flow owns the URDF. For load-from-disk
      // (graphSource='restored'), reconcile may shift pivots in the live
      // scene for visual alignment, but MUST NOT write back — otherwise the
      // editor drifts away from the on-disk text by sub-mm AABB-measurement
      // float noise. Mount nodes still rebuild so attachment rings stay
      // visually correct.
      if (res.adjustedCount > 0) {
        if (_graphSource === 'ai') {
          persistReconcileShiftsToUrdf(parsed.linkGroups)
        }
        rebuildMountNodes()
      }
      return res
    },
    getLastAssemblyGraph: () => _lastAssemblyGraph ? cloneAssemblyGraph(_lastAssemblyGraph) : null,
    setLastAssemblyGraph: (graph: AssemblyGraph | null) => {
      _lastAssemblyGraph = graph ? cloneAssemblyGraph(graph) : null
      _graphSource = graph ? 'restored' : null  // user owns this URDF — don't mutate it
      _persistGraph(_lastAssemblyGraph)
    },
    refreshAssemblyGraphForActiveFile: () => {
      try {
        const fileName = ctx.getActiveFileName?.() || 'robot.urdf'
        const stored = JSON.parse(localStorage.getItem(GRAPH_STORAGE_KEY) || '{}')
        if (stored[fileName]) {
          _lastAssemblyGraph = stored[fileName]
          _graphSource = 'restored'  // user-owned: file just loaded from disk/cache
          console.log(`[assembly] Loaded stored graph for "${fileName}" (${_lastAssemblyGraph!.components.length} components)`)
          return true
        }
        // No stored graph — fall back to reverse-parsing the current URDF.
        // Lossy: drops orientation, elevation_angle, length_mm, attach_rpy,
        // attach_connector. For bake those losses are mostly tolerable
        // (cluster planning needs joint_type + component_id, both preserved).
        // See graphPreservationCorpus.ts §"urdfToAssemblyGraph known limitations".
        const urdfText = ctx.getUrdfText()
        if (urdfText && urdfText.trim().length > 0) {
          const parsed = urdfToAssemblyGraph(urdfText)
          if (parsed && parsed.components.length > 0) {
            _lastAssemblyGraph = parsed
            _graphSource = 'restored'  // user-owned: third-party URDF loaded from disk
            console.log(`[assembly] Reverse-parsed graph from URDF for "${fileName}" (${parsed.components.length} components, lossy fallback)`)
            return true
          }
        }
        // Reverse-parse also failed (empty URDF, parse error). Clear in-memory
        // so callers fail fast instead of using a previous robot's graph.
        if (_lastAssemblyGraph !== null) {
          console.log(`[assembly] Cleared in-memory graph (no stored or reverse-parsable graph for "${fileName}")`)
          _lastAssemblyGraph = null
          _graphSource = null
        }
        return false
      } catch {
        // localStorage parse error — clear so callers don't act on stale data.
        _lastAssemblyGraph = null
        _graphSource = null
        return false
      }
    },
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
    refreshMountNodeTransforms: () => refreshNodeWorldTransforms(),
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

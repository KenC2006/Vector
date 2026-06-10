// Phase 3b — Shared placement compiler.
//
// `compileAssembly(semanticGraph, options) → CompiledGraph` is the single
// owner of placement math, mate-connector resolution, servo body/horn split +
// carrier synthesis, mirrored hardware sign mapping, foot-pad leveling,
// semantic rest-pose expansion, drivetrain rolling-axis remap, and root
// grounding offset.
//
// Today this module only ships the contract + a stub. Sub-phases 3b.1–3b.4
// move the placement code currently embedded in urdfAssembly.ts into the
// transforms.ts/joints.ts/face.ts/mate.ts/servoSplit.ts/grounding.ts
// siblings, then implement compileAssembly on top of them. Sub-phase 3b.5
// adds a Node-subprocess CLI (cli.ts) so the Python AI loop can call this
// compiler over JSON-lines without re-implementing the geometry rules.

import * as THREE from 'three'
import type { AssemblyGraph, AssemblyComponent } from '../urdfGraphEquivalence.ts'
import type { ComponentBoundsMm, ComponentInstanceSpec } from '../componentSpec.ts'
import { DiagnosticOwner, type Diagnostic } from '../compilerDiagnostics.ts'
import { isDrivetrainComponentId, isTireComponentId } from '../componentResolver.ts'
import type { MateConnector } from '../mateConnectors.ts'
import {
  parseRpyString,
  parseXyzString,
  transformFromXyzRpy,
  axisNameFromComponentAxis,
  servoMountRpyForParentWorld,
  servoPlanarMountRpyForParentWorld,
  worldLevelRpyForParent,
  worldOffsetFromParent,
  formatRpyTuple,
  servoLocalRestRpyFromJointRpy,
  servoAxisSignFromParentWorld,
} from './transforms.ts'
import {
  isSplitServoComponentId,
  isDistalBeamComponentId,
} from './componentNaming.ts'
import {
  axisNameToTuple,
  normalizeJointType,
} from './joints.ts'
import {
  computeFacePlacement as _pureComputeFacePlacement,
  parseOrientation,
  type ParentBoundsM,
} from './face.ts'
import {
  faceUVHalfExtents,
} from './multiChild.ts'
import {
  computeMatePlacement as _pureComputeMatePlacement,
  hasMateConnectorFields,
} from './mate.ts'
import { resolvePrimitiveAnchorPose, type LinkPrimitive } from '../linkGeometry.ts'
import {
  servoDrivenStructuralLimbPlacement,
  servoDrivenChildPlacement,
} from './servoSplit.ts'

/** Minimal data the compiler needs back from a per-component lookup. The
 * browser path wraps `findPreset` + `resolveComponentVisual`; the Node path
 * (Phase 3b.5) reads from a JSON catalog. Bounds are in meters (post-rpy,
 * parametric-spliced) — same convention as `ResolvedComponentRecord.bounds`. */
export interface ComponentResolution {
  componentId: string
  bounds: ComponentBoundsMm
  /** Connectors used by the **mate** placement path — typically the preset's
   * authored connectors merged with auto-generated face-default connectors
   * (top/bottom/front/back/left/right). Mate uses these to resolve
   * inferred-from-attach-face lookups. */
  connectors?: MateConnector[]
  /** Connectors used by the **face** placement path's connector-snap fallback.
   * Should be the RAW authored preset.connectors (no auto-defaults), so face
   * doesn't snap to a generated connector that the legacy assembler ignored.
   * When omitted, defaults to `connectors`. */
  presetConnectors?: MateConnector[]
  /** Drivetrain hub motors carry an assembled tire — the swap radius (meters)
   * used for `effectiveCym` when bottom-mounting. From
   * `preset.mounting_logic.assembled_outer_radius_mm / 1000`. Slice 4.I
   * (drivetrain_remap). */
  assembledOuterRadiusM?: number
  /** Per-instance length override for parametric extrusions (raw mm, NOT
   * divided). Set when the preset is `isParametricSpec` AND the
   * AssemblyComponent provided `length_mm`. Face placement uses this to
   * compute parentBodyHZ from the body length rather than the AABB which
   * includes pivot-boss / axle-cap protrusions. Slice 4.J (parametric_splice). */
  parametricLengthMm?: number
  /** Joint angle limits in radians (lo, hi). From `resolveJointLimitsRad`.
   * Slice 4.J (port_resolution). Used for revolute/prismatic joints; ignored
   * for fixed/continuous. */
  jointLimitsRad?: [number, number]
  /** Joint effort in N·m (URDF `<limit effort="…"/>`). From
   * `preset.mechanical_electrical.max_torque_nm` ?? `holding_torque_nm` ?? 10. */
  maxTorqueNm?: number
  /** Resolved component mass in kg (`resolveComponentMassKg`: authored mass_kg
   * → parametric per-100mm × length → fallback). Copied onto CompiledLink so
   * headless consumers (eval harness, URDF emitters) don't re-derive it. */
  massKg?: number
}

export type ComponentResolver = (
  componentId: string,
  instance: AssemblyComponent,
) => ComponentResolution | null

/** Joint type union — mirror of the file-local alias in urdfAssembly.ts. */
export type AssemblyJointType = 'fixed' | 'revolute' | 'continuous' | 'prismatic'

/** A single physical link emitted by the compiler.
 *
 * Servos that split into `_body`/`_horn` (and optionally `_compound_carrier`)
 * appear as one CompiledLink with `physicalLinks.length > 1` and
 * `childAttachTarget` set to the link any downstream child should hang off.
 * Consumers must never re-derive the body/horn naming from the logical name
 * with string suffixes — read `physicalLinks` and `childAttachTarget`. */
export interface CompiledLink {
  /** The logical link name from the semantic AssemblyGraph. Children targeting
   * this link reference `logicalName` even when the compiler split it. */
  logicalName: string
  componentId: string
  /** Physical link names emitted to URDF. Single-link components: just
   * `[logicalName]`. Servos: `[<name>_body, <name>_horn]` (+ optional
   * `<name>_compound_carrier`). Order is parent-frame-first. */
  physicalLinks: string[]
  /** The physical link a child component should attach to when its
   * AssemblyComponent.attach_to equals `logicalName`. For servos this is the
   * horn so children rotate with the actuator output. */
  childAttachTarget: string
  /** Resolved bounds (post-rpy, parametric-spliced). Copied from the
   * ResolvedComponentRecord — consumers must not re-derive geometry. */
  bounds: ComponentBoundsMm
  /** Local pose under the parent's `childAttachTarget` frame, URDF coords
   * (meters / radians). Root link: zero. */
  localXyz: [number, number, number]
  localRpy: [number, number, number]
  /** World pose under the compiler's deterministic root frame (root identity).
   * Used for invariant checks (foot leveling, mirror symmetry, parity tests).
   * Exporters should prefer local pose + parent traversal. */
  worldXyz: [number, number, number]
  worldRpy: [number, number, number]
  /** Connector chosen on each side, if any. Null for face-only placements. */
  parentConnector: string | null
  childConnector: string | null
  /** Joints synthesized for this link's physical links. Empty for root.
   *
   * Single-link components: one joint (parent.childAttachTarget → physicalName).
   * Servos: 2 joints — `_mount` (fixed, parent → body) + actuated revolute
   * (body → horn). Compound servos add a 3rd `_compound_carrier` (fixed,
   * parent → carrier, mount routes off carrier). Order is parent-frame-first
   * so consumers can append to URDF in iteration order. */
  joints: Array<{
    name: string
    type: AssemblyJointType
    axis: [number, number, number]
    limits?: [number, number]
    effort?: number
    velocity?: number
    /** Parent physical link the joint hangs off (after servo split routing). */
    parentLink: string
    /** Child physical link the joint connects to (the horn for servo splits). */
    childLink: string
    /** Local pose of the child link in the parent frame (URDF coords). */
    originXyz: [number, number, number]
    originRpy: [number, number, number]
  }>
  /** Marker for compiler-generated links that don't correspond to a logical
   * component on their own — body/horn halves of a split servo, the optional
   * compound carrier visual. Null when this is the user-named link. */
  syntheticRole: 'body' | 'horn' | 'carrier' | null
  /** Per-physical-link world poses (URDF coords). Same length as
   * `physicalLinks`, in the same order. For non-servos: length-1 = childWorld.
   * For servos: carrier+body share bodyWorld, horn gets hornWorld. The 4.K
   * cutover seeds `linkWorldTransforms` from this so reconcile + ICP find
   * every emitted link. */
  physicalWorldXyz: Array<[number, number, number]>
  physicalWorldRpy: Array<[number, number, number]>
  /** True when placement came from a connector mate (mate path matched, or
   * face's connector-snap fallback engaged). Drives `viaConnectorMap` in
   * reconcileNodePlacement so it skips re-flushing connector-aligned pairs. */
  placedViaConnector: boolean
  /** Resolved mass in kg for the logical component (0 when the resolver
   * didn't supply one). Servo splits report the whole component's mass here;
   * consumers needing the body/horn split apply their own ratio. */
  massKg: number
}

/** Placement-class taxonomy used by the Phase 3b.4 incremental rollout. Each
 * slice (4.B–4.J) implements one class and removes it from
 * CompiledGraph.skippedClasses. The shadow-compile parity harness only
 * compares links whose class has been implemented — anything else is silently
 * passed through. After 4.K cutover, `skippedClasses` is always []. */
export type PlacementClass =
  | 'root'
  | 'face_simple'
  | 'face_multi_child'
  | 'mate_connector'
  | 'servo_split'
  | 'mirrored_hardware'
  | 'foot_leveling'
  | 'ground_offset'
  | 'drivetrain_remap'
  | 'archetype_axis_normalize'
  | 'parametric_splice'
  | 'port_resolution'

export const ALL_PLACEMENT_CLASSES: readonly PlacementClass[] = [
  'root',
  'face_simple',
  'face_multi_child',
  'mate_connector',
  'servo_split',
  'mirrored_hardware',
  'foot_leveling',
  'ground_offset',
  'drivetrain_remap',
  'archetype_axis_normalize',
  'parametric_splice',
  'port_resolution',
] as const

export interface CompiledGraph {
  baseLink: string
  /** Topological order — parents before children. Single canonical traversal
   * for exporters and invariant checks. */
  links: CompiledLink[]
  /** Logical-name → physical-link routing table. Exporters look here when
   * resolving an `attach_to` from the semantic graph instead of synthesizing
   * string suffixes. */
  attachIndex: Record<string, string>
  /** Diagnostics emitted by the compiler. Owner-tagged via
   * compilerDiagnostics — semantic-graph violations that survived
   * normalize_and_validate are AI_TOPOLOGY; everything else is
   * PLACEMENT_COMPILER. */
  diagnostics: Diagnostic[]
  /** Hash of (semantic graph + resolver fingerprint + compiler version). Equal
   * fingerprints must produce equal graphs — used by the cross-runtime parity
   * tests and the AI-determinism corpus. */
  fingerprint: string
  /** Placement classes the compiler has not yet implemented in this build.
   * Always [] after Phase 3b.4.K cutover; the parity harness uses it to
   * decide which links to skip during incremental rollout. */
  skippedClasses: PlacementClass[]
}

export interface CompileOptions {
  /** Per-instance overrides keyed by logical link name. Currently only
   * `length_mm` for parametric extrusions; matches ComponentInstanceSpec. */
  instanceOverrides?: Record<string, ComponentInstanceSpec>
  /** When true (Python AI subprocess path), skip any work that would need a
   * THREE.Object3D scene graph; emit transforms + descriptors only. */
  headless?: boolean
  /** Looks up bounds + componentId for an AssemblyComponent. Required as soon
   * as `compileAssembly` emits any links; absent during the 4.A skeleton.
   * Browser callers wrap their preset catalog + resolveComponentVisual; the
   * Node CLI (3b.5) wraps the JSON catalog directly. */
  resolveComponent?: ComponentResolver
  /** Mirrors the assembler's `useMateConnectors()` flag — set false to bypass
   * connector-aware placement entirely (debug / regression bisect). Default
   * true matches the assembler's `VECTOR_USE_MATE_CONNECTORS !== false`. */
  useMateConnectors?: boolean
}

/** Compiler version string baked into the fingerprint. Bump when output for a
 * fixed input legitimately changes; the cross-runtime parity test will then
 * re-pin its golden fixtures. */
export const COMPILER_VERSION = '0.0.13-physical-world'

/** Z position of the servo horn rotation origin, as a fraction of the servo's
 * total height. Mirror of `SERVO_HORN_ORIGIN_Z_RATIO` in componentMeshes.ts.
 * Inlined here because the compiler module must stay free of three.js scene-
 * graph deps so it can run under `--experimental-strip-types` in the Node
 * corpus and the Python AI subprocess (slice 3b.5). */
const SERVO_HORN_ORIGIN_Z_RATIO = 0.44

/** Component-id prefixes that mark a component as "passive hardware" — no
 * splay applied during multi-child distribution. Mirrors urdfAssembly.ts
 * line ~4024 verbatim so noSplay computes bit-identically. */
function isPassiveHardware(componentId: string): boolean {
  return componentId.startsWith('structural_bracket')
    || componentId.startsWith('structural_joint_plate')
    || componentId.startsWith('structural_sheet')
    || componentId.startsWith('structural_servo_coupler')
    || componentId.startsWith('power_')
    || componentId.startsWith('sensor_')
    || componentId.startsWith('compute_')
}

/** Cross-cutting deferred-class bypasses — apply to ALL placement paths
 * (face/mate AND servo_split). Slices 4.G–4.J each peel one of these off as
 * they implement the corresponding class. */
function deferredByGlobalBypass(
  c: AssemblyComponent,
  _parent: AssemblyComponent | undefined,
): boolean {
  if (!c.attach_to) return true
  // All previously deferred classes (parametric_splice / attach_rpy override /
  // distal-beam-bottom flip / drivetrain_remap / elevation_angle / parametric
  // parent) are now handled inline in the main loop.
  return false
}

function isServoSplitCase(
  c: AssemblyComponent,
  parent: AssemblyComponent | undefined,
): boolean {
  return isSplitServoComponentId(c.component_id)
    || (parent != null && isSplitServoComponentId(parent.component_id))
}

/** Eligibility for face/mate placement (slices 4.C/4.D/4.E). Servos and
 * servo-children go through `eligibleForServoSplit` instead. */
function eligibleForFaceOrMatePlacement(
  c: AssemblyComponent,
  parent: AssemblyComponent | undefined,
): boolean {
  if (deferredByGlobalBypass(c, parent)) return false
  if (isServoSplitCase(c, parent)) return false               // servo_split path
  // Drivetrain wheels are continuous joints but their LINK pose is independent
  // of joint type; allow them through. Joint URDF emission stays deferred to
  // slice 4.J. Other revolute/prismatic placements remain deferred.
  const isDrivetrain = isDrivetrainComponentId(c.component_id)
    || isTireComponentId(c.component_id)
    || (parent != null && isDrivetrainComponentId(parent.component_id))
  if (!isDrivetrain && (c.joint_type ?? 'fixed') !== 'fixed') return false
  return true
}

/** Eligibility for the servo split path (slice 4.F). Covers both `cIsActuated`
 * (current child IS a servo) and `parentIsServo` (current child mounts on a
 * servo horn). */
function eligibleForServoSplit(
  c: AssemblyComponent,
  parent: AssemblyComponent | undefined,
): boolean {
  if (deferredByGlobalBypass(c, parent)) return false
  return isServoSplitCase(c, parent)
}

/**
 * Phase 3b.4 — incremental placement compiler.
 *
 * Each rollout slice removes one class from `skippedClasses` and emits the
 * corresponding CompiledLinks. The shadow-compile harness in
 * resolveAssemblyGraph compares observed vs. compiled per-link and only for
 * classes the compiler has implemented, so partial coverage never fires
 * false alarms.
 *
 * Currently implemented: `root`, `face_simple`. Pending: face_multi_child,
 * mate_connector, servo_split, mirrored_hardware, foot_leveling,
 * ground_offset, drivetrain_remap, archetype_axis_normalize,
 * parametric_splice, port_resolution.
 */
export function compileAssembly(
  semanticGraph: AssemblyGraph,
  options?: CompileOptions,
): CompiledGraph {
  const links: CompiledLink[] = []
  const attachIndex: Record<string, string> = {}
  const diagnostics: Diagnostic[] = []
  const implemented = new Set<PlacementClass>([
    'root', 'face_simple', 'face_multi_child', 'mate_connector', 'servo_split',
    'foot_leveling', 'ground_offset',
    // mirrored_hardware: covered by `servoAxisSignFromParentWorld` inside the
    // servo branch — Y-axis servos on a parent at world -Y get axisSign=-1, so
    // the body-mount rpy mirrors automatically. No extra dispatch needed.
    'mirrored_hardware',
    // archetype_axis_normalize: handled upstream by `normalizeAssembly`
    // (archetypeNormalizer) before resolveAssemblyGraph calls the compiler.
    // The compiler always receives pre-normalized joint axes.
    'archetype_axis_normalize',
    // drivetrain_remap: face placement gets rolling-hardware hints +
    // effectiveCym swap (slice 4.I). Joint-axis "y → 0 0 1" remap for
    // bottom-mounted drivetrain stays in slice 4.J (joint emission).
    'drivetrain_remap',
    // parametric_splice (slice 4.J½): per-instance length_mm flows through
    // resolver bounds (buildVisualPreset applies the override) and via the
    // `parametricLengthMm` field on ComponentResolution into face's parentBodyHZ.
    'parametric_splice',
    // port_resolution (slice 4.J): joint{} synthesized for every non-root link
    // — type from comp.joint_type (already port-matched by validate_topology
    // upstream), axis from joint_axis with bottom-drivetrain remap, limits
    // from resolver.jointLimitsRad. Servos emit 2-3 joints (carrier? + mount
    // + revolute body→horn).
    'port_resolution',
  ])
  const useMateConnectorsFlag = options?.useMateConnectors ?? true
  const skippedClasses = (): PlacementClass[] => ALL_PLACEMENT_CLASSES.filter(c => !implemented.has(c))

  const root = semanticGraph.components.find(c => !c.attach_to)
  if (!root) {
    return { baseLink: semanticGraph.base_link, links, attachIndex, diagnostics, fingerprint: 'empty', skippedClasses: skippedClasses() }
  }

  const resolver = options?.resolveComponent
  if (!resolver) {
    diagnostics.push({
      code: 'compiler.missing_resolver', severity: 'error',
      owner: DiagnosticOwner.PlacementCompiler,
      message: 'compileAssembly: resolveComponent is required when the graph has components',
    })
    return { baseLink: semanticGraph.base_link, links, attachIndex, diagnostics, fingerprint: 'error', skippedClasses: skippedClasses() }
  }

  const rootResolved = resolver(root.component_id, root)
  if (!rootResolved) {
    diagnostics.push({
      code: 'compiler.unknown_component', severity: 'error',
      owner: DiagnosticOwner.PlacementCompiler,
      message: `compileAssembly: resolver returned null for root component "${root.component_id}"`,
    })
    return { baseLink: semanticGraph.base_link, links, attachIndex, diagnostics, fingerprint: 'error', skippedClasses: skippedClasses() }
  }

  // ── Root emission (slice 4.B) ─────────────────────────────────────────────
  const rootLink: CompiledLink = {
    logicalName: root.link_name,
    componentId: root.component_id,
    physicalLinks: [root.link_name],
    childAttachTarget: root.link_name,
    bounds: rootResolved.bounds,
    localXyz: [0, 0, 0], localRpy: [0, 0, 0],
    worldXyz: [0, 0, 0], worldRpy: [0, 0, 0],
    parentConnector: null, childConnector: null,
    joints: [], syntheticRole: null,
    physicalWorldXyz: [[0, 0, 0]],
    physicalWorldRpy: [[0, 0, 0]],
    placedViaConnector: false,
    massKg: rootResolved.massKg ?? 0,
  }
  links.push(rootLink)
  attachIndex[root.link_name] = root.link_name

  // ── Slice 4.C — face_simple traversal ─────────────────────────────────────
  const componentByName = new Map<string, AssemblyComponent>(
    semanticGraph.components.map(c => [c.link_name, c])
  )
  const compiledByLogical = new Map<string, CompiledLink>([[root.link_name, rootLink]])
  const worldByLogical = new Map<string, THREE.Matrix4>([[root.link_name, new THREE.Matrix4()]])

  // Pre-pass face counts + per-face child UV-projected sizes, mirroring
  // assembler ~3851. faceChildSizes drives `_computeMultiChildOffsets` so
  // siblings on the same face don't overlap. Built in input-array order;
  // the assembler's topological loop visits same-face siblings in that same
  // order (their parent is processed once, then findIndex picks remaining
  // children in array order).
  const faceChildCounts = new Map<string, number>()
  const faceChildSizes = new Map<string, Array<{ hu: number; hv: number }>>()
  for (const c of semanticGraph.components) {
    if (!c.attach_to) continue
    const face = c.attach_face || 'top'
    const key = `${c.attach_to}:${face}`
    faceChildCounts.set(key, (faceChildCounts.get(key) ?? 0) + 1)
    const cr = resolver(c.component_id, c)
    if (!cr) continue
    const cb = cr.bounds
    const childBoundsForUV = {
      hx: cb.half[0], hy: cb.half[1], hz: cb.half[2],
      cx: cb.center[0], cy: cb.center[1], cz: cb.center[2],
    }
    const { hu, hv } = faceUVHalfExtents(childBoundsForUV, face)
    if (!faceChildSizes.has(key)) faceChildSizes.set(key, [])
    faceChildSizes.get(key)!.push({ hu, hv })
  }
  // Per-face running index, incremented as each child is placed.
  const faceChildIndex = new Map<string, number>()

  // armDepth tracking — mirror of assembler's `armDepth` Map at line ~4215.
  // Tracks consecutive Y-axis revolute "arm" joints down a chain so the
  // shoulder/elbow get their default rest pose (depth 1 → +π/4, depth 2 → -π/2).
  const armDepth = new Map<string, number>()

  // Topological iteration mirroring assembler line ~3888 (BFS-ish: pick first
  // remaining whose parent is already processed).
  const remaining = semanticGraph.components.filter(c => c.attach_to !== null)
  const processed = new Set<string>([root.link_name])
  // Mirrors assembler `placedCount`: 1 after root, ticks for every child the
  // assembler would have placed (preset resolved). Used for auto-naming so
  // `${preset.id}_${counter}` matches the URDF link names the assembler emits.
  let placedCount = 1
  let safety = remaining.length * 2 + 2

  const _pos = new THREE.Vector3()
  const _quat = new THREE.Quaternion()
  const _scl = new THREE.Vector3()
  const _euler = new THREE.Euler()

  while (remaining.length > 0 && safety-- > 0) {
    const idx = remaining.findIndex(c => processed.has(c.attach_to!))
    if (idx === -1) break
    const c = remaining.splice(idx, 1)[0]
    processed.add(c.link_name)

    const childResolved = resolver(c.component_id, c)
    if (!childResolved) continue   // unknown preset — assembler also skips, no count tick

    // The assembler's placedCount ticks after a successful placement (line
    // 4461). For correctness of the auto-name we tick BEFORE we know if the
    // current slice can emit — non-eligible children still consume an index
    // because the assembler placed them, just via a path this slice doesn't
    // implement yet.
    placedCount++

    const parentComp = componentByName.get(c.attach_to!)
    const isServo = isServoSplitCase(c, parentComp)
    if (!isServo && !eligibleForFaceOrMatePlacement(c, parentComp)) continue
    if (isServo && !eligibleForServoSplit(c, parentComp)) continue

    const parentCompiled = compiledByLogical.get(c.attach_to!)
    if (!parentCompiled) continue   // parent class not implemented yet
    const parentResolved = parentComp ? resolver(parentComp.component_id, parentComp) : null
    if (!parentResolved) continue
    const parentWorld = worldByLogical.get(c.attach_to!)!

    const pb = parentResolved.bounds
    const parentBoundsM: ParentBoundsM = {
      hx: pb.half[0], hy: pb.half[1], hz: pb.half[2],
      cx: pb.center[0], cy: pb.center[1], cz: pb.center[2],
    }
    const cb = childResolved.bounds
    const childX = cb.half[0] * 2
    const childY = cb.half[1] * 2
    const childZ = cb.half[2] * 2
    const childCenterOffset = { cx: cb.center[0], cy: cb.center[1], cz: cb.center[2] }
    const sortedDims = [childX, childY, childZ].slice().sort((a, b) => a - b)
    const isElongated = sortedDims[2] > sortedDims[0] * 2.5 && sortedDims[1] < sortedDims[0] * 2.0
    const orientation = c.orientation || 'auto'

    const faceKey = `${c.attach_to}:${c.attach_face || 'top'}`
    const totalOnFace = faceChildCounts.get(faceKey) || 1
    const childIdx = faceChildIndex.get(faceKey) || 0
    faceChildIndex.set(faceKey, childIdx + 1)

    // noSplay mirrors assembler line ~4031: rolling hardware (wheels, drivetrain,
    // swerves, anything carrying a tire), passive hardware (brackets/sheets/
    // sensors/etc.), and split servos all suppress the multi-child outward tilt.
    const _childIsTireForSplay = isTireComponentId(c.component_id)
    const _childIsDriveForSplay = isDrivetrainComponentId(c.component_id)
    const _hasTireChild = semanticGraph.components.some(
      sc => sc.attach_to === c.link_name && isTireComponentId(sc.component_id)
    )
    const _isRollingHardware = _childIsDriveForSplay || _childIsTireForSplay
      || c.component_id.startsWith('mobility_swerve_') || _hasTireChild
    const noSplay = _isRollingHardware
      || isPassiveHardware(c.component_id)
      || isSplitServoComponentId(c.component_id)
    const parentConnectors = parentResolved.connectors ?? []
    const childConnectors = childResolved.connectors ?? []
    // Face placement uses raw preset.connectors (no auto-defaults) so it
    // matches the legacy assembler — it shouldn't snap to top/bottom/etc
    // unless the preset explicitly authored those.
    const parentPresetConnectors = parentResolved.presetConnectors ?? parentResolved.connectors ?? []
    const childPresetConnectors = childResolved.presetConnectors ?? childResolved.connectors ?? []

    const cIsActuated = isSplitServoComponentId(c.component_id)
    const parentIsServo = !!(parentComp && isSplitServoComponentId(parentComp.component_id))

    // ── Placement (xyz, rpy) ──────────────────────────────────────────────
    // Four sources, in priority order:
    // 0. Raw authoring: Claude wrote `xyz` / `rpy` on the component —
    //    bypass everything below and use them verbatim.
    // 1. Servo-driven: when parent is a split servo, use the closed-form
    //    limb / child placement helpers (replaces face/mate entirely).
    // 2. Mate connector: closed-form connector mate.
    // 3. Bbox face placement: fallback / standard path.
    let placement: { xyz: string; rpy: string } | null = null
    let placedViaConnectorFlag = false

    {
      const rawXyz = Array.isArray(c.xyz) && c.xyz.length === 3 ? c.xyz : null
      const rawRpy = Array.isArray(c.rpy) && c.rpy.length === 3 ? c.rpy : null
      if (rawXyz || rawRpy) {
        // Compute defaults for the unauthored axis so a partially-authored
        // component still produces a valid joint origin. If only `xyz` is
        // provided, keep rpy at zero (face-up); if only `rpy` is provided,
        // keep xyz at origin. Authoring both is the typical case.
        const x = rawXyz ? Number(rawXyz[0]) || 0 : 0
        const y = rawXyz ? Number(rawXyz[1]) || 0 : 0
        const z = rawXyz ? Number(rawXyz[2]) || 0 : 0
        const r = rawRpy ? Number(rawRpy[0]) || 0 : 0
        const p = rawRpy ? Number(rawRpy[1]) || 0 : 0
        const yw = rawRpy ? Number(rawRpy[2]) || 0 : 0
        placement = {
          xyz: `${x.toFixed(4)} ${y.toFixed(4)} ${z.toFixed(4)}`,
          rpy: `${r.toFixed(4)} ${p.toFixed(4)} ${yw.toFixed(4)}`,
        }
        // Mark as connector-placed so reconcile doesn't snap the child back
        // to bbox-min — Claude's authored position is authoritative.
        placedViaConnectorFlag = true
      }
    }

    if (parentIsServo && placement === null) {
      const parentServoAxis = axisNameFromComponentAxis(parentComp!.joint_axis)
      const grandParentComp = parentComp!.attach_to
        ? componentByName.get(parentComp!.attach_to)
        : undefined
      const parentRestRpy = Array.isArray(parentComp!.attach_rpy) ? parentComp!.attach_rpy : undefined
      const parentRestPitch = parentRestRpy ? Number(parentRestRpy[1]) || 0 : 0
      const invertRadialSide = parentServoAxis === 'y'
        && (
          (
            !!grandParentComp
            && isSplitServoComponentId(grandParentComp.component_id)
            && axisNameFromComponentAxis(grandParentComp.joint_axis) === 'x'
          )
          || parentRestPitch < -0.001
        )
      const childBodyHX = Math.max(childX / 2 - Math.abs(cb.center[0]), 0)
      const childBodyHY = Math.max(childY / 2 - Math.abs(cb.center[1]), 0)
      const childBodyHZ = Math.max(childZ / 2 - Math.abs(cb.center[2]), 0)
      placement = c.component_id === 'structural_limb_link_slim'
        ? servoDrivenStructuralLimbPlacement(parentServoAxis, c.attach_face, childBodyHY, childBodyHZ, parentWorld)
        : servoDrivenChildPlacement(parentServoAxis, c.attach_face, childBodyHX, childBodyHY, childBodyHZ, invertRadialSide, cIsActuated)
      if (!placement) continue
      // Mirror inline-path behavior: servo-driven children are connector-placed
      // (the servo horn IS the connector). Reconcile must not re-flush them.
      placedViaConnectorFlag = true
    } else if (placement === null) {
      // Primitive-anchor placement (WS5): child mounts at a named anchor on a
      // named primitive of the parent's link_geometry. Resolve the anchor to
      // a synthetic connector and route through the ordinary fastened-mate
      // solver. Unresolvable names are unreachable here — the validator
      // hard-errors first — but keep a defensive diagnostic, never a silent
      // fallthrough to AABB-face placement.
      if (typeof c.attach_anchor === 'string' && typeof c.attach_primitive === 'string' && parentComp) {
        const anchorConn = resolvePrimitiveAnchorPose(
          parentComp.link_geometry as LinkPrimitive[] | undefined,
          c.attach_primitive,
          c.attach_anchor,
        )
        if (anchorConn) {
          const anchored: AssemblyComponent = {
            ...c,
            attach_connector: anchorConn.id,
            mate_connector: c.mate_connector ?? 'bottom',
            mate_type: c.mate_type ?? 'fastened',
          }
          const anchorYawRad = parseOrientation(c.orientation).yawDeg * Math.PI / 180
          const mateResult = _pureComputeMatePlacement(
            anchored, [...parentConnectors, anchorConn], childConnectors,
            { hxMm: parentBoundsM.hx * 1000, hyMm: parentBoundsM.hy * 1000, hzMm: parentBoundsM.hz * 1000 },
            undefined,
            { useMateConnectors: true, rotationRad: anchorYawRad },
          )
          if (!('miss' in mateResult)) {
            placement = mateResult
            placedViaConnectorFlag = true
          }
        }
        if (placement === null) {
          diagnostics.push({
            code: 'compiler.bad_primitive_anchor', severity: 'error',
            owner: DiagnosticOwner.PlacementCompiler,
            message:
              `${c.link_name}: attach_primitive="${c.attach_primitive}" / ` +
              `attach_anchor="${c.attach_anchor}" did not resolve on ${parentComp.link_name} ` +
              `(validator should have caught this upstream)`,
          })
          continue
        }
      }
      // Tier-B short-circuited above (raw xyz/rpy authored) — skip the mate
      // and face paths entirely so they don't overwrite the authored values.
      // Tire-on-drivetrain has exactly one correct placement (motor body's
      // outboard end), and the face short-circuit at face.ts:92 produces it.
      // The mate-connector path here would honor whatever attach_connector the
      // AI emits — and the AI consistently picks the motor's "bottom" because
      // it thinks "top" was consumed by the baseplate bolt-down, dropping the
      // wheel inboard. Skip the mate path entirely for this case so the face
      // short-circuit always wins regardless of which mate fields the AI sets.
      const tireOnDrivetrain =
        isTireComponentId(c.component_id)
        && !!parentComp && isDrivetrainComponentId(parentComp.component_id)
      if (placement === null && !tireOnDrivetrain && hasMateConnectorFields(c)) {
        const mateMulti = totalOnFace > 1
          ? { total: totalOnFace, index: childIdx, face: c.attach_face || 'top', childSizes: faceChildSizes.get(faceKey) }
          : undefined
        const mateResult = _pureComputeMatePlacement(
          c, parentConnectors, childConnectors,
          { hxMm: parentBoundsM.hx * 1000, hyMm: parentBoundsM.hy * 1000, hzMm: parentBoundsM.hz * 1000 },
          mateMulti,
          { useMateConnectors: useMateConnectorsFlag },
        )
        if (!('miss' in mateResult)) {
          placement = mateResult
          placedViaConnectorFlag = true
        }
      }
      if (placement === null) {
        // ── Slice 4.I — drivetrain_remap ───────────────────────────────────
        // Mirror of assembler ~4009-4040: rolling-hardware children (wheels,
        // tires, swerves) need the rolling bottom pose, parent-is-drivetrain
        // affects axle inference, and bottom-mounted hub motors swap their
        // y half-extent for the assembled tire's outer radius so the tire
        // clears the parent face instead of clipping through it.
        const childIsTire = isTireComponentId(c.component_id)
        const childIsDrivetrain = isDrivetrainComponentId(c.component_id)
        const parentIsDrivetrain = !!(parentComp && isDrivetrainComponentId(parentComp.component_id))
        const childUsesRollingBottomPose = childIsDrivetrain || childIsTire
          || c.component_id.startsWith('mobility_swerve_')
        let effectiveChildY = childY
        if (childIsDrivetrain && c.attach_face === 'bottom'
            && typeof childResolved.assembledOuterRadiusM === 'number') {
          effectiveChildY = childResolved.assembledOuterRadiusM * 2
        }
        placement = _pureComputeFacePlacement(
          parentBoundsM, parentComp!.link_name,
          childX, effectiveChildY, childZ,
          c.attach_face,
          isElongated, childIdx, totalOnFace,
          orientation, noSplay,
          c.component_id, c.elevation_angle ?? 0,
          faceChildSizes.get(faceKey),
          childCenterOffset,
          parentPresetConnectors.length > 0 ? parentPresetConnectors : undefined,
          undefined,
          childPresetConnectors.length > 0 ? childPresetConnectors : undefined,
          { parentIsDrivetrain, childIsTire, childIsDrivetrain, childUsesRollingBottomPose },
          parentResolved.parametricLengthMm,
        )
      }
    }

    const physicalName = `${childResolved.componentId}_${placedCount}`

    // ── Post-placement adjustments (slices 4.J.2 / 4.J.3 / 4.J.4) ─────────
    // Order mirrors assembler ~4197-4270:
    // 1. Distal-beam-bottom flip — child mounts under a distal limb tip
    // whose local-Z points the wrong way in world space; flip xyz[2].
    // 2. Arm rest pose — Y-axis revolute "arm" chain auto-bends:
    // shoulder (depth 1) +π/4, elbow (depth 2) -π/2.
    // 3. Explicit attach_rpy override — replaces auto rest pose.
    // Side-axis servo dz nudge (cIsActuated only) runs INSIDE the servo
    // emit branch, after these — same order as assembler line ~4277.
    const servoAxisName = axisNameFromComponentAxis(c.joint_axis)
    // Rest poses MIRROR with the hardware (live-eval finding, 2026-06): under
    // auto-distribution the model cannot know which world side a leg lands
    // on, so it naturally authors ONE value per joint role (both knees
    // [0,0.8,0]) and expects a symmetric stance. Verbatim application made
    // ±Y pairs bend opposite world directions and multi-legged robots
    // collapsed in the settle test. The engine therefore multiplies the
    // authored rest angle by the same geometric sign that mirrors the horn
    // shaft. Deliberate per-leg asymmetry remains available via raw `rpy`.
    const servoAxisSign = cIsActuated && placement
      ? servoAxisSignFromParentWorld(parentWorld, servoAxisName, parseXyzString(placement.xyz))
      : 1

    // Distal-beam 'bottom' flip. servoDrivenStructuralLimbPlacement mounts the
    // beam with local +Z as the radial-outward (free-tip) end and -Z toward
    // the joint, so 'bottom' on a servo-mounted beam points at the joint, not
    // the leg tip — flip. The world-Z normal check covers the non-servo case
    // where -Z would land above the origin (foot wants the floor side).
    // placed_via_connector is set so reconcileAlignment.ts (~164) doesn't snap
    // the child back to parentAABB.min on every pass.
    if (parentComp && isDistalBeamComponentId(parentComp.component_id)
        && c.attach_face === 'bottom') {
      const xyz = parseXyzString(placement.xyz)
      const normalWorld = worldOffsetFromParent(parentWorld, [0, 0, xyz[2]])
      const grandparent = parentComp.attach_to
        ? componentByName.get(parentComp.attach_to)
        : undefined
      const beamMountedOnServo = !!(grandparent && isSplitServoComponentId(grandparent.component_id))
      if (normalWorld.z > 0.0001 || beamMountedOnServo) {
        xyz[2] = -xyz[2]
        placement = { xyz: xyz.map(v => Number(v || 0).toFixed(4)).join(' '), rpy: placement.rpy }
        placedViaConnectorFlag = true
      }
    }

    const placementRpyAtRest = placement!.rpy
    let finalRpy = placementRpyAtRest
    let servoHornZeroRpy = '0 0 0'
    const isArmJoint = (c.joint_type ?? 'fixed') === 'revolute'
      && c.joint_axis?.toLowerCase() === 'y'
      && c.attach_face === 'top'
    const parentDepth = armDepth.get(c.attach_to!) ?? 0
    if (isArmJoint) {
      const depth = parentDepth + 1
      armDepth.set(c.link_name, depth)
      // Auto rest-pose disabled. Previously shoulder (depth 1) was pre-rotated
      // +π/4 and elbow (depth 2) -π/2, which left arms locked in an L-pose at
      // zero servo input rather than vertical. The prompt now instructs arms
      // to stand vertical at rest; dog/humanoid still get their crouch via
      // explicit `attach_rpy` from the AI (handled in the explicitRpyApplied
      // branch below).
    } else {
      armDepth.set(c.link_name, c.attach_face === 'top' ? parentDepth : 0)
    }

    const explicitRpy = c.attach_rpy
    const explicitRpyApplied = Array.isArray(explicitRpy) && explicitRpy.length === 3
      && explicitRpy.some(v => Math.abs(v) > 0.001)
    if (explicitRpyApplied) {
      const explicitRpyStr = formatRpyTuple([
        Number(explicitRpy![0]) || 0,
        Number(explicitRpy![1]) || 0,
        Number(explicitRpy![2]) || 0,
      ])
      if (cIsActuated) {
        const explicitTuple = parseRpyString(explicitRpyStr)
        const restPoseSign = servoAxisSign
        if (servoAxisName === 'y' && parentComp
            && isDistalBeamComponentId(parentComp.component_id)
            && c.attach_face === 'bottom') {
          const bend = Math.abs(explicitTuple[1] || 0)
          servoHornZeroRpy = formatRpyTuple([0, 0, bend * restPoseSign])
        } else {
          servoHornZeroRpy = servoLocalRestRpyFromJointRpy(explicitTuple, servoAxisName, restPoseSign)
        }
      } else {
        finalRpy = explicitRpyStr
      }
    }

    // ── Emission ──────────────────────────────────────────────────────────
    if (cIsActuated) {
      // Reconcile's face-flush math assumes the child link's three.js parent
      // matches its logical assembly parent. For split-servo emission the
      // logical "child" remaps to the _horn link, whose three.js parent is
      // the _body (one or two joints below the logical parent). Letting
      // reconcile process the horn against the logical parent's face normal
      // shifts the rev pivot to garbage. The placement compiler already owns
      // the body/horn poses, so flag the servo as connector-placed and
      // reconcile will skip it.
      placedViaConnectorFlag = true
      const useCompoundServoCarrier = parentIsServo

      // Side-axis servo clearance nudge (assembler line ~4277, generalized).
      // The face placement assumed the body's extent along the face normal was
      // childBody[face-axis]; after the shaft-align rotation, a different body
      // axis now lies along that world axis. Nudge xyz[face-axis] so the body
      // still seats flush.
      // axis-x shaft-align Ry(π/2): world(X,Y,Z) extents from body(Z,Y,X)
      // axis-y shaft-align Rx(-π/2): world(X,Y,Z) extents from body(X,Z,Y)
      if (servoAxisName !== 'z') {
        const face = c.attach_face || 'top'
        const faceNormalAxis: 0 | 1 | 2 | null =
          face === 'top' || face === 'bottom' ? 2
          : face === 'front' || face === 'back' ? 0
          : face === 'left' || face === 'right' ? 1 : null
        if (faceNormalAxis !== null) {
          const faceOutward = face === 'top' || face === 'front' || face === 'right' ? 1 : -1
          const xMap: [0, 1, 2] = [2, 1, 0] as unknown as [0, 1, 2]
          const yMap: [0, 1, 2] = [0, 2, 1] as unknown as [0, 1, 2]
          const bodyAxisForWorld: 0 | 1 | 2 = servoAxisName === 'x'
            ? xMap[faceNormalAxis]
            : yMap[faceNormalAxis]
          const baseHalf = Math.max(cb.half[faceNormalAxis] - Math.abs(cb.center[faceNormalAxis]), 0)
          const rotatedHalf = Math.max(cb.half[bodyAxisForWorld] - Math.abs(cb.center[bodyAxisForWorld]), 0)
          const delta = rotatedHalf - baseHalf
          if (Math.abs(delta) > 1e-6) {
            const xyzParts = placement.xyz.split(/\s+/).map(Number)
            xyzParts[faceNormalAxis] = (xyzParts[faceNormalAxis] ?? 0) + faceOutward * delta
            placement = { xyz: xyzParts.map(v => Number(v || 0).toFixed(4)).join(' '), rpy: placement.rpy }
          }
        }
      }

      const servoBodyMountRpy = servoAxisName === 'z'
        ? servoPlanarMountRpyForParentWorld(parentWorld, c.attach_face, finalRpy)
        : servoMountRpyForParentWorld(parentWorld, servoAxisName, parseXyzString(placement.xyz))
      const hornOriginZ = childZ * SERVO_HORN_ORIGIN_Z_RATIO

      const bodyName = `${physicalName}_body`
      const hornName = `${physicalName}_horn`
      const carrierName = `${physicalName}_compound_carrier`

      const bodyWorld = parentWorld.clone().multiply(transformFromXyzRpy(placement.xyz, servoBodyMountRpy))
      const hornWorld = bodyWorld.clone().multiply(transformFromXyzRpy(`0 0 ${hornOriginZ.toFixed(6)}`, servoHornZeroRpy))
      bodyWorld.decompose(_pos, _quat, _scl)
      _euler.setFromQuaternion(_quat, 'XYZ')
      const bodyWorldXyz: [number, number, number] = [_pos.x, _pos.y, _pos.z]
      const bodyWorldRpy: [number, number, number] = [_euler.x, _euler.y, _euler.z]
      hornWorld.decompose(_pos, _quat, _scl)
      _euler.setFromQuaternion(_quat, 'XYZ')
      const hornWorldXyz: [number, number, number] = [_pos.x, _pos.y, _pos.z]
      const hornWorldRpy: [number, number, number] = [_euler.x, _euler.y, _euler.z]

      const physicalLinks = useCompoundServoCarrier
        ? [carrierName, bodyName, hornName]
        : [bodyName, hornName]
      // Carrier shares bodyWorld (it's a fixed wrapper at the same pose); horn
      // is on its own. Order matches `physicalLinks` above.
      const physicalWorldXyz: Array<[number, number, number]> = useCompoundServoCarrier
        ? [bodyWorldXyz, bodyWorldXyz, hornWorldXyz]
        : [bodyWorldXyz, hornWorldXyz]
      const physicalWorldRpy: Array<[number, number, number]> = useCompoundServoCarrier
        ? [bodyWorldRpy, bodyWorldRpy, hornWorldRpy]
        : [bodyWorldRpy, hornWorldRpy]

      // ── Slice 4.J — servo joint synthesis ──────────────────────────────
      // Mirror of assembler ~4301-4390: 2-3 joints per servo.
      // 1. (compound only) `${jointName}_compound_carrier` fixed,
      // parent → carrier @ (placement.xyz, servoBodyMountRpy)
      // 2. `${jointName}_mount` fixed, parent-or-carrier → body
      // @ ('0 0 0' if compound else placement.xyz / servoBodyMountRpy)
      // 3. `${jointName}` revolute, body → horn
      // @ ('0 0 hornOriginZ', identity rpy), axis (0 0 1), limits
      const jointBaseName = `joint_${c.component_id}_${placedCount}`
      const carrierJointName = `${jointBaseName}_compound_carrier`
      const mountJointName = `${jointBaseName}_mount`
      const placementXyz = parseXyzString(placement.xyz)
      const placementRpy = parseRpyString(servoBodyMountRpy)
      const parentLinkPhys = parentCompiled.childAttachTarget
      const limitsRad = childResolved.jointLimitsRad ?? [-Math.PI / 2, Math.PI / 2]
      const torque = childResolved.maxTorqueNm ?? 10

      const servoJoints: CompiledLink['joints'] = []
      if (useCompoundServoCarrier) {
        servoJoints.push({
          name: carrierJointName, type: 'fixed', axis: [0, 0, 1],
          parentLink: parentLinkPhys, childLink: carrierName,
          originXyz: placementXyz, originRpy: placementRpy,
        })
        servoJoints.push({
          name: mountJointName, type: 'fixed', axis: [0, 0, 1],
          parentLink: carrierName, childLink: bodyName,
          originXyz: [0, 0, 0], originRpy: [0, 0, 0],
        })
      } else {
        servoJoints.push({
          name: mountJointName, type: 'fixed', axis: [0, 0, 1],
          parentLink: parentLinkPhys, childLink: bodyName,
          originXyz: placementXyz, originRpy: placementRpy,
        })
      }
      servoJoints.push({
        name: jointBaseName, type: 'revolute', axis: [0, 0, 1],
        limits: limitsRad, effort: torque, velocity: 3.14,
        parentLink: bodyName, childLink: hornName,
        originXyz: [0, 0, hornOriginZ], originRpy: parseRpyString(servoHornZeroRpy),
      })

      const link: CompiledLink = {
        logicalName: c.link_name,
        componentId: c.component_id,
        physicalLinks,
        childAttachTarget: hornName,
        bounds: childResolved.bounds,
        // Local pose: body-mount pose (matches what the assembler stores in
        // its placementEntry under the base auto-name). The harness keys
        // entries by the post-nameMap physical name (horn) and gets null,
        // so this value isn't actually compared today — it's emitted for
        // downstream consumers (URDF exporter cutover in slice 4.K).
        localXyz: placementXyz,
        localRpy: placementRpy,
        worldXyz: hornWorldXyz,
        worldRpy: hornWorldRpy,
        parentConnector: null, childConnector: null,
        joints: servoJoints,
        syntheticRole: null,
        physicalWorldXyz,
        physicalWorldRpy,
        placedViaConnector: placedViaConnectorFlag,
        massKg: childResolved.massKg ?? 0,
      }
      links.push(link)
      compiledByLogical.set(c.link_name, link)
      // Downstream children attach to the horn; their parentWorld must be
      // hornWorld so servoDrivenChildPlacement gets the rotated frame.
      worldByLogical.set(c.link_name, hornWorld)
      attachIndex[c.link_name] = hornName
    } else {
      // ── Slice 4.H — foot leveling ──────────────────────────────────────
      // Rubber foot pads override the face-derived rpy with a world-level
      // rpy so the contact patch stays parallel to the ground regardless of
      // parent leg orientation. Mirror of assembler line ~4268. Skipped when
      // attach_rpy was authored — explicit override wins.
      if (c.component_id === 'mobility_rubber_foot_pad' && !explicitRpyApplied) {
        finalRpy = worldLevelRpyForParent(parentWorld)
      }

      const localXyz = parseXyzString(placement.xyz)
      const localRpy = parseRpyString(finalRpy)
      const localMat = transformFromXyzRpy(placement.xyz, finalRpy)
      const childWorld = parentWorld.clone().multiply(localMat)
      childWorld.decompose(_pos, _quat, _scl)
      _euler.setFromQuaternion(_quat, 'XYZ')

      // ── Slice 4.J — port_resolution / joint synthesis ──────────────────
      // Single-link emission gets one joint (parent.childAttachTarget →
      // physicalName). Joint type comes from comp.joint_type (port-matched
      // upstream by validate_topology). Axis is the named-axis tuple, with
      // the bottom-mounted-drivetrain "y → 0 0 1" remap mirrored from
      // assembler line ~4117.
      const jointName = `joint_${c.component_id}_${placedCount}`
      const jointType = normalizeJointType(c.joint_type)
      let axisTuple = axisNameToTuple(c.joint_axis)
      if (_childIsDriveForSplay && c.attach_face === 'bottom'
          && c.joint_axis?.toLowerCase() === 'y') {
        axisTuple = [0, 0, 1]
      }
      const needsLimits = jointType === 'revolute' || jointType === 'prismatic'
      const joint = {
        name: jointName,
        type: jointType,
        axis: axisTuple,
        limits: needsLimits ? childResolved.jointLimitsRad : undefined,
        effort: needsLimits ? (childResolved.maxTorqueNm ?? 10) : undefined,
        velocity: needsLimits ? 3.14 : undefined,
        parentLink: parentCompiled.childAttachTarget,
        childLink: physicalName,
        originXyz: localXyz,
        originRpy: localRpy,
      }

      const childWorldXyz: [number, number, number] = [_pos.x, _pos.y, _pos.z]
      const childWorldRpy: [number, number, number] = [_euler.x, _euler.y, _euler.z]
      const link: CompiledLink = {
        logicalName: c.link_name,
        componentId: c.component_id,
        physicalLinks: [physicalName],
        childAttachTarget: physicalName,
        bounds: childResolved.bounds,
        localXyz, localRpy,
        worldXyz: childWorldXyz,
        worldRpy: childWorldRpy,
        parentConnector: null, childConnector: null,
        joints: [joint],
        syntheticRole: null,
        physicalWorldXyz: [childWorldXyz],
        physicalWorldRpy: [childWorldRpy],
        placedViaConnector: placedViaConnectorFlag,
        massKg: childResolved.massKg ?? 0,
      }
      links.push(link)
      compiledByLogical.set(c.link_name, link)
      worldByLogical.set(c.link_name, childWorld)
      attachIndex[c.link_name] = physicalName
    }
  }

  return {
    baseLink: root.link_name,
    links,
    attachIndex,
    diagnostics,
    fingerprint: `${root.component_id}@${COMPILER_VERSION}/${links.length}links`,
    skippedClasses: skippedClasses(),
  }
}

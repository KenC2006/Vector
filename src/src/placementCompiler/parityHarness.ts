// Phase 3b.4.A — shadow-compile parity harness.
//
// Two responsibilities:
//   1. captureObservedGraph(): turn the runtime artifacts emitted by
//      resolveAssemblyGraph (placementEntries, linkWorldTransforms, nameMap,
//      post-mutation components list) into an ObservedGraph keyed by logical
//      link name.
//   2. compareGraphs(): diff observed vs. compiled, per logical link, with
//      tolerance. Links whose CompiledLink is absent are silently skipped —
//      that's how the rollout works: compileAssembly only emits links for
//      classes it has implemented, and the harness only compares those.
//
// Pure module. No DOM. Runs both in-browser (live shadow check) and in Node
// corpus tests via `--experimental-strip-types`.

import * as THREE from 'three'
import type { AssemblyComponent } from '../urdfGraphEquivalence.ts'
import type { CompiledGraph, CompiledLink } from './index.ts'

/** Shape of one urdfAssembly EnginePlacementEntry. Inlined here instead of
 *  importing to keep this module free of urdfAssembly dependencies (avoids a
 *  cycle and lets the corpus build it from synthetic data). */
export interface PlacementEntryLike {
  /** Physical link name (post-nameMap remap). */
  linkName: string
  /** Physical parent link name. */
  parentLinkName: string
  /** Space-separated meters, as written to URDF. */
  xyz: string
  /** Space-separated radians, as written to URDF. */
  rpy: string
}

export interface ObservedLink {
  logicalName: string
  physicalLinkName: string
  componentId: string
  parentLogicalName: string | null
  /** Local pose under parent (URDF coords, meters/radians). Null for root. */
  localXyz: [number, number, number] | null
  localRpy: [number, number, number] | null
  /** World pose decomposed from the runtime linkWorldTransforms map. */
  worldXyz: [number, number, number]
  worldRpy: [number, number, number]
}

export interface ObservedGraph {
  baseLink: string
  links: ObservedLink[]
}

export interface ParityDiff {
  logicalName: string
  /** e.g. "localXyz[0]", "worldRpy[2]", "physicalLinkName". */
  field: string
  observed: unknown
  compiled: unknown
  /** Numeric magnitude when both sides are numeric; omitted for string diffs. */
  delta?: number
}

export interface ParityReport {
  totalObserved: number
  totalCompiled: number
  /** Number of observed links whose compiled counterpart matched within tolerance. */
  matched: number
  /** Number of observed links the compiler skipped (no CompiledLink emitted). */
  unmatched: number
  /** Snapshot of CompiledGraph.skippedClasses at compare time — informational. */
  skippedClasses: string[]
  diffs: ParityDiff[]
}

export interface ParityTolerance {
  xyzM: number
  rpyRad: number
}

/** URDF formats xyz/rpy with 6 decimals; tighter tolerance is just floating
 *  noise. 1e-6 m == 1 µm, 1e-6 rad == 0.00006°. */
export const DEFAULT_TOLERANCE: ParityTolerance = { xyzM: 1e-6, rpyRad: 1e-6 }

// ── captureObservedGraph ────────────────────────────────────────────────────

export interface CaptureArgs {
  baseLinkLogical: string
  rootPhysicalName: string
  rootComponentId: string
  /** Post-mutation components list (after archetype normalize, autorepair,
   *  joint-axis normalization). Caller is `resolveAssemblyGraph`. */
  components: AssemblyComponent[]
  placementEntries: PlacementEntryLike[]
  linkWorldTransforms: Map<string, THREE.Matrix4>
  /** Logical → physical link-name remap from the runtime placement loop. */
  nameMap: Map<string, string>
}

export function captureObservedGraph(args: CaptureArgs): ObservedGraph {
  const { baseLinkLogical, rootPhysicalName, rootComponentId, components,
    placementEntries, linkWorldTransforms, nameMap } = args

  const physicalByLogical = (logical: string): string => nameMap.get(logical) ?? logical

  // Index placement entries by physical link name. computeFacePlacement /
  // computeMatePlacement push one entry per child as it lands, keyed by the
  // post-remap physical name.
  const placementByPhysical = new Map<string, PlacementEntryLike>()
  for (const e of placementEntries) placementByPhysical.set(e.linkName, e)

  const _pos = new THREE.Vector3()
  const _quat = new THREE.Quaternion()
  const _scl = new THREE.Vector3()
  const _euler = new THREE.Euler()
  function decomposeWorld(physName: string): { xyz: [number, number, number]; rpy: [number, number, number] } {
    const m = linkWorldTransforms.get(physName)
    if (!m) return { xyz: [0, 0, 0], rpy: [0, 0, 0] }
    m.decompose(_pos, _quat, _scl)
    _euler.setFromQuaternion(_quat, 'XYZ')
    return { xyz: [_pos.x, _pos.y, _pos.z], rpy: [_euler.x, _euler.y, _euler.z] }
  }

  function parseTriple(s: string): [number, number, number] {
    const parts = s.trim().split(/\s+/).map(Number)
    return [parts[0] ?? 0, parts[1] ?? 0, parts[2] ?? 0]
  }

  const links: ObservedLink[] = []

  // Root: no parent, no local pose. World pose comes from the identity matrix
  // resolveAssemblyGraph seeds linkWorldTransforms with at the start of the
  // try block — should always be [0,0,0] / [0,0,0].
  const rootWorld = decomposeWorld(rootPhysicalName)
  links.push({
    logicalName: baseLinkLogical,
    physicalLinkName: rootPhysicalName,
    componentId: rootComponentId,
    parentLogicalName: null,
    localXyz: null,
    localRpy: null,
    worldXyz: rootWorld.xyz,
    worldRpy: rootWorld.rpy,
  })

  for (const c of components) {
    if (!c.attach_to) continue
    const phys = physicalByLogical(c.link_name)
    const entry = placementByPhysical.get(phys)
    const local = entry ? { xyz: parseTriple(entry.xyz), rpy: parseTriple(entry.rpy) } : null
    const world = decomposeWorld(phys)
    links.push({
      logicalName: c.link_name,
      physicalLinkName: phys,
      componentId: c.component_id,
      parentLogicalName: c.attach_to,
      localXyz: local?.xyz ?? null,
      localRpy: local?.rpy ?? null,
      worldXyz: world.xyz,
      worldRpy: world.rpy,
    })
  }

  return { baseLink: baseLinkLogical, links }
}

// ── compareGraphs ───────────────────────────────────────────────────────────

export function compareGraphs(
  observed: ObservedGraph,
  compiled: CompiledGraph,
  tolerance: ParityTolerance = DEFAULT_TOLERANCE,
): ParityReport {
  const diffs: ParityDiff[] = []
  const compiledByLogical = new Map<string, CompiledLink>()
  for (const link of compiled.links) compiledByLogical.set(link.logicalName, link)

  let matched = 0
  let unmatched = 0

  for (const o of observed.links) {
    const c = compiledByLogical.get(o.logicalName)
    if (!c) {
      // Compiler hasn't implemented this link's class yet — skip silently.
      // Slices 4.B–4.J grow coverage one class at a time; intermediate
      // states must not flood the console with "missing" warnings.
      unmatched++
      continue
    }

    let linkOK = true

    // Compare against `childAttachTarget` (the routing target, == _horn for
    // servos) instead of `physicalLinks[0]` (== _body for servos): the
    // runtime `nameMap` records the routing target, and the observed graph
    // resolves logical names through it. For non-servos these are equal.
    if (c.childAttachTarget !== o.physicalLinkName) {
      diffs.push({
        logicalName: o.logicalName,
        field: 'physicalLinkName',
        observed: o.physicalLinkName,
        compiled: c.childAttachTarget,
      })
      linkOK = false
    }

    const tripleDiff = (
      field: string,
      obs: [number, number, number],
      com: [number, number, number],
      tol: number,
    ) => {
      for (let i = 0; i < 3; i++) {
        const d = Math.abs(obs[i] - com[i])
        if (d > tol) {
          diffs.push({
            logicalName: o.logicalName,
            field: `${field}[${i}]`,
            observed: obs[i],
            compiled: com[i],
            delta: d,
          })
          linkOK = false
        }
      }
    }

    if (o.localXyz) tripleDiff('localXyz', o.localXyz, c.localXyz, tolerance.xyzM)
    if (o.localRpy) tripleDiff('localRpy', o.localRpy, c.localRpy, tolerance.rpyRad)
    tripleDiff('worldXyz', o.worldXyz, c.worldXyz, tolerance.xyzM)
    tripleDiff('worldRpy', o.worldRpy, c.worldRpy, tolerance.rpyRad)

    if (linkOK) matched++
  }

  return {
    totalObserved: observed.links.length,
    totalCompiled: compiled.links.length,
    matched,
    unmatched,
    skippedClasses: [...compiled.skippedClasses],
    diffs,
  }
}

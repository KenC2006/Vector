// Runtime ICP-along-normal nudge — closes mating-face gaps on imported
// components that don't have `engagement_depth_mm` authored on their
// connector.
//
// Pure module — no DOM, no Tauri, no richVisuals imports — so it runs
// from Node via the alignmentCorpus harness. Consumes THREE.Object3D
// trees that the caller has already loaded (GLB or parametric). The
// scene-aware path also accepts a `Set<Object3D>` of descendant pivot
// groups to exclude; callers (urdfAssembly.ts's post-reconcile pass)
// pass the full scene's pivot set so raycasts against a parent's scene
// link group don't accidentally intersect the child subtrees.
//
// Algorithm (1-D signed distance along the parent connector normal):
//   1. Tangent basis u, v spans the plane perpendicular to parent axis.
//   2. Sample a unit disc in tangent space — SAMPLE_COUNT points whose
//      (uFrac, vFrac) fall inside r ≤ 1 (square lattice clipped to disc).
//   3. For each (uFrac, vFrac), raycast the PARENT from outside along
//      -parent_axis to find the parent surface axial height p. Then
//      raycast the CHILD from outside along -child_axis (child outward,
//      = -parent_axis under the antiparallel-mate convention) to find
//      the child surface axial height c. BOTH raycasts use the SAME
//      (uFrac, vFrac) against the SAME tangent basis (uP, vP), which
//      lands them at the same WORLD point on the mate face — the plane
//      perpendicular to parent.axis is the same plane as the one perp
//      to child.axis, so one world-space basis serves both samplings.
//      (A previous version built a separate child basis and negated
//      uFrac/vFrac; that compensation was asymmetric — u flipped, v did
//      not — and put child samples at DIFFERENT world points than the
//      parent samples for asymmetric meshes. That was the 10-14mm drift
//      the user hit on servo-bottom / extrusion-top mates in Session 3.)
//   4. Per-pair signed axial gap g = -p - c. Positive = visible gap at
//      this sample; zero = touching; negative = interpenetrating.
//   5. Take the NUDGE_PERCENTILE-th percentile of g. This picks the
//      nudge that closes the gap for most of the face while letting
//      the small interpenetrating minority (shaft tip protruding, etc.)
//      hide behind the child once the nudge is applied.
//   6. Clamp to [0, NUDGE_CAP_MM].
//
// The algorithm deliberately returns 0 when the measured 90th-percentile
// is at or below zero — no "gap to close" in that case. Non-zero nudges
// are bounded above by NUDGE_CAP_MM to protect against bad inputs
// (missing geometry, mis-oriented connectors).

import * as THREE from 'three'

// ── Tunables (exported so callers can override per-mate if needed) ─────────

/** Target number of disc samples per face. Actual sampled pairs may be
 *  lower after disc clipping + raycast misses. Higher = more stable
 *  percentile, slower. 64 gives sub-millisecond runtime on modern GPUs
 *  and enough resolution for a 2-3mm chamfer signal. */
export const SAMPLE_COUNT = 64

/** Tangent-plane sample radius as a fraction of the mesh's smaller
 *  perpendicular extent. The spec calls for 0.8 × face half-extent;
 *  half × 0.8 = 0.4 × full extent. */
export const SAMPLE_RADIUS_FACTOR = 0.4

/** Percentile of the per-pair gap distribution used as the nudge. 0.9
 *  matches the "hide 10% of parent surface behind child" heuristic. */
export const NUDGE_PERCENTILE = 0.9

/** Conservative default cap on the returned nudge (meters). Bounds damage
 *  from bad inputs on low-confidence measurements — 3mm is well below any
 *  structural clearance we care about. */
export const NUDGE_CAP_MM = 3

/** Raised cap used when the per-sample gap distribution is both (a) tight
 *  (spread between p10 and p90 below CONFIDENT_SPREAD_MAX_MM) and (b) at
 *  least CONFIDENT_PAIRED_MIN_FRAC of the sample count landed as paired
 *  hits. Those conditions imply a FLAT, REAL offset between the two faces
 *  (not noise, not a bimodal mix of features), so the algorithm is allowed
 *  to close it even if the magnitude exceeds the conservative cap. Targets
 *  the extrusion-on-servo-bottom case where the servo body sits 4-7mm above
 *  its authored bottom connector, producing a real 4-7mm gap that would
 *  otherwise get clipped. */
export const NUDGE_CAP_CONFIDENT_MM = 15

/** Max allowed (p90 - p10) gap spread (in mm) for the "confident" adaptive
 *  cap to fire. Below this, per-sample gaps agree closely enough to treat
 *  the measurement as a real flat offset. */
export const CONFIDENT_SPREAD_MAX_MM = 3

/** Min allowed paired-sample fraction (paired / sample_count) for the
 *  confident cap. Low fractions often reflect a small child face where
 *  most rays miss — statistic may still be valid but we don't bump the
 *  cap until we have enough samples to trust it. 0.3 = need at least ~24
 *  of the ~80 default samples paired. */
export const CONFIDENT_PAIRED_MIN_FRAC = 0.3

/** Secondary gate for small-but-uniform distributions. When `|p90 − p50|`
 *  is below this and we have at least `UNIFORM_PAIRED_MIN` paired samples,
 *  the signal is unambiguous even if pair coverage is low — fires the
 *  same confident cap. Targets the thigh/shin-extrusion-on-servo and
 *  footpad-on-extrusion cases where the child footprint (20×20 / 25×25)
 *  is much smaller than the sample disc (radius ~14.4 / 8 mm on the
 *  parent), so paired coverage sits at 10-25% even though every paired
 *  sample reports the same gap depth. Without this, those mates stay
 *  clamped at the default 3mm cap even when the real gap is 4-5 mm and
 *  uniformly measured. */
export const UNIFORM_MEDIAN_SPREAD_MAX_MM = 1
export const UNIFORM_PAIRED_MIN = 6

/** Minimum raw nudge (mm) below which we return 0. Filters out sub-
 *  millimetre noise from raycast quantization / sample aliasing so
 *  clean flat-on-flat mates stay true no-ops. */
export const NUDGE_MIN_MM = 0.05

export interface NudgeOptions {
  percentile?: number
  maxNudgeM?: number
  sampleCount?: number
  faceRadiusM?: number
  /** Descendant Object3D's that the mesh walk + raycast must skip.
   *  Used by the scene-level pass in urdfAssembly.ts so raycasts against
   *  a parent link group don't walk into pivot subtrees (which belong to
   *  other links). Left empty for the Node corpus fixtures — they build
   *  synthetic scenes with no child subtrees. */
  excludeParent?: Set<THREE.Object3D>
  excludeChild?: Set<THREE.Object3D>
  /** Out-parameter the caller fills with diagnostics. Simpler than a
   *  tuple return — lets `nudgeAlongNormal` keep returning just the
   *  scalar while still exposing the histogram data to callers that
   *  need to log it (urdfAssembly.ts trace). */
  diagnostics?: NudgeDiagnostics
}

export interface NudgeDiagnostics {
  sampleCount: number
  parentHits: number
  childHits: number
  pairedCount: number
  faceRadiusM: number
  gapMinMm?: number
  gapP10Mm?: number
  gapP50Mm?: number
  gapP90Mm?: number
  gapMaxMm?: number
  nudgeMm: number
  reason: string
}

// ── Helpers ────────────────────────────────────────────────────────────────

function tangentBasis(axis: THREE.Vector3): { u: THREE.Vector3; v: THREE.Vector3 } {
  const n = axis.clone().normalize()
  const ax = Math.abs(n.x), ay = Math.abs(n.y), az = Math.abs(n.z)
  let helper: THREE.Vector3
  if (ax <= ay && ax <= az) helper = new THREE.Vector3(1, 0, 0)
  else if (ay <= az) helper = new THREE.Vector3(0, 1, 0)
  else helper = new THREE.Vector3(0, 0, 1)
  const u = new THREE.Vector3().crossVectors(n, helper).normalize()
  const v = new THREE.Vector3().crossVectors(n, u).normalize()
  return { u, v }
}

/** Square-lattice unit-disc sample grid. Uniform in area; matches the
 *  spec's "~64 points" target (actual count ≈ gridN² × π/4). */
function discSamples(sampleCount: number): Array<[number, number]> {
  const gridN = Math.max(3, Math.ceil(Math.sqrt(sampleCount * 4 / Math.PI)))
  const out: Array<[number, number]> = []
  for (let i = 0; i < gridN; i++) {
    for (let j = 0; j < gridN; j++) {
      const uFrac = (i + 0.5) / gridN * 2 - 1
      const vFrac = (j + 0.5) / gridN * 2 - 1
      if (uFrac * uFrac + vFrac * vFrac <= 1) out.push([uFrac, vFrac])
    }
  }
  return out
}

/** Collect meshes descendent of `root` but NOT descendent of any object
 *  in `exclude`. Uses a manual walk so exclude works on subtree roots,
 *  not individual objects — exactly matches reconcileAlignment.ts's
 *  linkLocalAABBExcludingPivots pattern. */
function collectMeshes(root: THREE.Object3D, exclude?: Set<THREE.Object3D>): THREE.Mesh[] {
  const out: THREE.Mesh[] = []
  const walk = (obj: THREE.Object3D) => {
    if (obj !== root && exclude?.has(obj)) return
    const mesh = obj as THREE.Mesh
    if ((mesh as any).isMesh && mesh.geometry) out.push(mesh)
    for (const c of obj.children) walk(c)
  }
  walk(root)
  return out
}

/** AABB of a mesh list. Box3.setFromObject(root) would include excluded
 *  subtrees, so we compute from the filtered mesh list directly. Returns
 *  null when the list is empty (no renderable geometry). */
function meshListAABB(meshes: THREE.Mesh[], rootInv: THREE.Matrix4 | null): THREE.Box3 | null {
  const box = new THREE.Box3()
  let hasGeom = false
  const tmpBox = new THREE.Box3()
  const tmpMat = new THREE.Matrix4()
  for (const m of meshes) {
    if (!m.geometry.boundingBox) m.geometry.computeBoundingBox()
    const bb = m.geometry.boundingBox
    if (!bb || bb.isEmpty()) continue
    m.updateMatrixWorld(true)
    if (rootInv) tmpMat.multiplyMatrices(rootInv, m.matrixWorld)
    else tmpMat.copy(m.matrixWorld)
    tmpBox.copy(bb).applyMatrix4(tmpMat)
    box.union(tmpBox)
    hasGeom = true
  }
  return hasGeom ? box : null
}

function defaultFaceRadius(aabb: THREE.Box3, axis: THREE.Vector3): number {
  const size = new THREE.Vector3()
  aabb.getSize(size)
  const absAxis = new THREE.Vector3(Math.abs(axis.x), Math.abs(axis.y), Math.abs(axis.z))
  const perp: number[] = []
  if (absAxis.x < 0.99) perp.push(size.x)
  if (absAxis.y < 0.99) perp.push(size.y)
  if (absAxis.z < 0.99) perp.push(size.z)
  const minPerp = perp.length > 0 ? Math.min(...perp) : Math.max(size.x, size.y, size.z, 0.02)
  return SAMPLE_RADIUS_FACTOR * minPerp
}

function sampleAxialAt(
  meshes: THREE.Mesh[],
  connectorOrigin: THREE.Vector3,
  outwardAxis: THREE.Vector3,
  u: THREE.Vector3, v: THREE.Vector3,
  uOffM: number, vOffM: number,
  rayStartOffset: number,
  raycaster: THREE.Raycaster,
): number | null {
  const rayOrigin = new THREE.Vector3()
    .copy(connectorOrigin)
    .addScaledVector(u, uOffM)
    .addScaledVector(v, vOffM)
    .addScaledVector(outwardAxis, rayStartOffset)
  raycaster.set(rayOrigin, outwardAxis.clone().negate())
  const hits = raycaster.intersectObjects(meshes, false)
  if (hits.length === 0) return null
  return hits[0].point.clone().sub(connectorOrigin).dot(outwardAxis)
}

function percentile(sorted: number[], p: number): number {
  if (sorted.length === 0) return NaN
  const idx = Math.min(sorted.length - 1, Math.max(0, Math.floor(sorted.length * p)))
  return sorted[idx]
}

// ── Core measurement ────────────────────────────────────────────────────────

/**
 * Returns the axial translation (meters) the child should move along the
 * parent connector axis (INTO the parent) to close the measured gap.
 *
 * The returned number is clamped into [0, NUDGE_CAP_MM / 1000]. Zero means
 * "no action required" (flat-on-flat, already interpenetrating, or too few
 * successful samples).
 *
 * When `opts.diagnostics` is provided, the function mutates it with the
 * sample-count / percentile / reason data so the caller can log it.
 */
export function nudgeAlongNormal(
  parentMesh: THREE.Object3D,
  childMesh: THREE.Object3D,
  parentConnectorOriginM: [number, number, number],
  parentConnectorAxisUnit: [number, number, number],
  childConnectorOriginM: [number, number, number],
  opts: NudgeOptions = {},
): number {
  const percentileOpt = opts.percentile ?? NUDGE_PERCENTILE
  // If caller passes `maxNudgeM` explicitly, that's the absolute cap — no
  // adaptive upgrade. If omitted, cap is chosen AFTER measurement based on
  // the spread / paired-fraction confidence gate. Default path lets clear
  // wide-gap cases like extrusion-on-servo-bottom close fully without
  // sacrificing the 3mm guard on noisy measurements.
  const callerOverrodeCap = opts.maxNudgeM !== undefined
  const callerMaxNudgeM = opts.maxNudgeM ?? NUDGE_CAP_MM / 1000
  const sampleCount = opts.sampleCount ?? SAMPLE_COUNT
  const diag: NudgeDiagnostics = opts.diagnostics ?? {
    sampleCount: 0, parentHits: 0, childHits: 0, pairedCount: 0, faceRadiusM: 0,
    nudgeMm: 0, reason: '',
  }

  const parentOrig = new THREE.Vector3(...parentConnectorOriginM)
  const parentAxis = new THREE.Vector3(...parentConnectorAxisUnit).normalize()
  const childOrig = new THREE.Vector3(...childConnectorOriginM)
  // Mate convention: child's local outward axis is antiparallel to the
  // parent's outward axis (verified in mateConnectors.ts's default connector
  // table and for every authored connector pair in generic_presets.json).
  const childAxis = parentAxis.clone().negate()

  // Single tangent basis shared between parent and child sampling — the
  // plane perpendicular to parent.axis = the plane perpendicular to
  // child.axis (since they're antiparallel), so the same (u, v) vectors
  // span both. Sharing the basis guarantees parent-and-child samples
  // land at the SAME world point, which is what the "matched pair" gap
  // derivation in the header assumes.
  const { u, v } = tangentBasis(parentAxis)

  // Build mesh lists with pivot-subtree exclusion applied.
  const parentMeshes = collectMeshes(parentMesh, opts.excludeParent)
  const childMeshes = collectMeshes(childMesh, opts.excludeChild)
  if (parentMeshes.length === 0 || childMeshes.length === 0) {
    diag.reason = 'no renderable meshes after exclude'
    diag.nudgeMm = 0
    return 0
  }

  // AABB must be in the SAME FRAME as `parentAxis` / `childAxis` so
  // defaultFaceRadius's perp-extent logic picks the right pair of sizes.
  // Callers from the scene-level pass pass axes in WORLD, so we measure
  // AABB in WORLD too (pass null to meshListAABB so it uses each mesh's
  // own matrixWorld without reprojection). Fixture callers at identity
  // transform get the same values either way.
  parentMesh.updateMatrixWorld(true)
  childMesh.updateMatrixWorld(true)
  const parentAabb = meshListAABB(parentMeshes, null)
  const childAabb = meshListAABB(childMeshes, null)
  if (!parentAabb || !childAabb) {
    diag.reason = 'empty AABB'
    diag.nudgeMm = 0
    return 0
  }

  const faceRadius = opts.faceRadiusM ?? defaultFaceRadius(parentAabb, parentAxis)
  diag.faceRadiusM = faceRadius
  if (faceRadius <= 0) {
    diag.reason = 'zero face radius'
    diag.nudgeMm = 0
    return 0
  }

  const parentSz = new THREE.Vector3(); parentAabb.getSize(parentSz)
  const childSz = new THREE.Vector3(); childAabb.getSize(childSz)
  const parentRayOffset = Math.max(parentSz.x, parentSz.y, parentSz.z, 0.01) + 0.01
  const childRayOffset = Math.max(childSz.x, childSz.y, childSz.z, 0.01) + 0.01

  // Raycaster is reused across the sample loop — allocation-free.
  const raycaster = new THREE.Raycaster()
  raycaster.far = Math.max(parentRayOffset, childRayOffset) * 4

  const samples = discSamples(sampleCount)
  diag.sampleCount = samples.length
  const gapsM: number[] = []
  let parentHits = 0, childHits = 0
  for (const [uFrac, vFrac] of samples) {
    const uOff = uFrac * faceRadius
    const vOff = vFrac * faceRadius
    const axialP = sampleAxialAt(
      parentMeshes, parentOrig, parentAxis,
      u, v, uOff, vOff,
      parentRayOffset, raycaster,
    )
    if (axialP !== null) parentHits++
    // Child uses the SAME tangent basis + SAME (uOff, vOff) as parent —
    // see header note on world-correspondence. The child raycast starts
    // from the OPPOSITE side of the mate plane from the parent raycast
    // (rayStartOffset is applied along childAxis, which is −parentAxis),
    // so both rays converge on the same world point on the mate face
    // from their respective sides.
    const axialC = sampleAxialAt(
      childMeshes, childOrig, childAxis,
      u, v, uOff, vOff,
      childRayOffset, raycaster,
    )
    if (axialC !== null) childHits++
    if (axialP === null || axialC === null) continue
    gapsM.push(-axialP - axialC)
  }
  diag.parentHits = parentHits
  diag.childHits = childHits
  diag.pairedCount = gapsM.length

  if (gapsM.length < 4) {
    diag.reason = 'too few paired samples'
    diag.nudgeMm = 0
    return 0
  }

  gapsM.sort((a, b) => a - b)
  diag.gapMinMm = gapsM[0] * 1000
  diag.gapP10Mm = percentile(gapsM, 0.1) * 1000
  diag.gapP50Mm = percentile(gapsM, 0.5) * 1000
  diag.gapP90Mm = percentile(gapsM, 0.9) * 1000
  diag.gapMaxMm = gapsM[gapsM.length - 1] * 1000

  const raw = percentile(gapsM, percentileOpt)
  if (!isFinite(raw)) { diag.reason = 'non-finite percentile'; diag.nudgeMm = 0; return 0 }
  if (raw <= NUDGE_MIN_MM / 1000) {
    diag.reason = `p${Math.round(percentileOpt * 100)}=${(raw * 1000).toFixed(2)}mm below min threshold`
    diag.nudgeMm = 0
    return 0
  }
  // Adaptive cap: the measurement is "confident" under either of two
  // conditions:
  //   (A) p10-p90 spread is tight AND paired coverage is high enough —
  //       classic case where many samples all agree.
  //   (B) p50-p90 spread is very tight AND we have ≥ UNIFORM_PAIRED_MIN
  //       paired samples — handles the small-child case (extrusion on
  //       servo bottom, footpad on extrusion bottom) where pair coverage
  //       sits at 10-25% because the child is smaller than the sample
  //       disc, but every paired sample reports the same gap depth.
  // Either fires the raised NUDGE_CAP_CONFIDENT_MM cap. If the caller
  // explicitly passed maxNudgeM, honor it — callers can force the
  // conservative cap for a specific mate if they want.
  const spreadMm = ((diag.gapP90Mm ?? 0) - (diag.gapP10Mm ?? 0))
  const medianSpreadMm = ((diag.gapP90Mm ?? 0) - (diag.gapP50Mm ?? 0))
  const pairedFrac = gapsM.length / Math.max(1, samples.length)
  const isTight =
    spreadMm < CONFIDENT_SPREAD_MAX_MM &&
    pairedFrac >= CONFIDENT_PAIRED_MIN_FRAC
  const isUniform =
    Math.abs(medianSpreadMm) < UNIFORM_MEDIAN_SPREAD_MAX_MM &&
    gapsM.length >= UNIFORM_PAIRED_MIN
  const isConfident = isTight || isUniform
  const effectiveCapM = callerOverrodeCap
    ? callerMaxNudgeM
    : (isConfident ? NUDGE_CAP_CONFIDENT_MM / 1000 : NUDGE_CAP_MM / 1000)
  const clamped = Math.min(raw, effectiveCapM)
  diag.nudgeMm = clamped * 1000
  const capLabel = callerOverrodeCap
    ? `caller-cap=${(callerMaxNudgeM * 1000).toFixed(1)}mm`
    : isTight
      ? `confident-cap=${NUDGE_CAP_CONFIDENT_MM}mm (tight: spread=${spreadMm.toFixed(2)}mm paired=${Math.round(pairedFrac * 100)}%)`
      : isUniform
        ? `confident-cap=${NUDGE_CAP_CONFIDENT_MM}mm (uniform: p90-p50=${medianSpreadMm.toFixed(2)}mm paired=${gapsM.length})`
        : `default-cap=${NUDGE_CAP_MM}mm`
  diag.reason = clamped < raw
    ? `p${Math.round(percentileOpt * 100)}=${(raw * 1000).toFixed(2)}mm clamped to ${(effectiveCapM * 1000).toFixed(1)}mm [${capLabel}]`
    : `p${Math.round(percentileOpt * 100)}=${(raw * 1000).toFixed(2)}mm [${capLabel}]`
  return clamped
}

// ── Gate ────────────────────────────────────────────────────────────────────

/**
 * Returns true when the runtime nudge should run for this mate. Skip when
 * the PARENT connector authors engagement_depth_mm — Step 1's curated
 * value is the preferred path.
 */
export function shouldApplyRuntimeNudge(parentEngagementDepthMm: number | undefined): boolean {
  if (parentEngagementDepthMm === undefined) return true
  if (!isFinite(parentEngagementDepthMm)) return true
  return parentEngagementDepthMm <= 0
}

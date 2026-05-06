// Message protocol between the main thread and the bake WebWorker.
//
// Shared types only — no DOM, no replicad imports. The main-thread and worker
// files both consume this; keeping it isolated avoids bundling OCCT WASM into
// the main-thread entry.

export interface MeshOpts {
  tolerance?: number
  angularTolerance?: number
}

export interface SerializedMesh {
  // Positions in millimeters (OCCT native units). Caller scales to meters.
  positions: Float32Array
  normals: Float32Array
  indices: Uint32Array
}

export interface SerializedEdges {
  // Flat xyz vertices of line segments (pairs).
  positions: Float32Array
}

/** One side of a mate. Connector origin + axis are in the part's native STEP
 *  frame (millimeters), matching `generic_presets.json → connectors[]`. */
export interface FusePart {
  stepUrl: string
  connectorOriginMm: [number, number, number]
  connectorAxisXyz: [number, number, number]
}

export interface FuseSpec {
  parent: FusePart
  child: FusePart
  /** Fillet radius in mm. Zero disables fillet (useful for debugging fuse alone).
   *  OCCT rejects radii larger than the smallest adjacent edge; Phase 4 adds a
   *  shrink-and-retry loop. Phase 2 hard-fails on fillet-reject. */
  filletRadiusMm?: number
  /** Half-thickness of the edge-filter slab around the mate plane (mm). Edges
   *  within ±slab of the mate plane are fillet candidates. Default 0.5 mm —
   *  just enough to capture the intersection ring without catching unrelated
   *  nearby detail. */
  filletSlabMm?: number
  /** Hint to Replicad/OCCT's BRepAlgoAPI_Fuse about coincident-face handling.
   *  `sameFace` is fastest when the parent/child share a flush planar mate
   *  (our canonical case); `none` disables optimisation — use when fuse
   *  rejects or looks wrong. */
  fuseOptimisation?: 'none' | 'commonFace' | 'sameFace'
  /** Debug escape hatch. When true, skip fuse+fillet entirely and return the
   *  concatenated parent+translated-child triangles. Lets callers test
   *  import + placement without paying for boolean ops. */
  skipFuse?: boolean
  /** Call `Shape.simplify()` on each imported shape before fusing. Removes
   *  redundant topology from over-modeled STEP sources (the coupler's splined
   *  bore adds ~900 edges that OCCT's boolean hates). Defaults to true. */
  simplifyImports?: boolean
}

/** Parametric disc-with-bore child — a stand-in for over-modeled vendor STEP
 *  (e.g. the 974-edge coupler whose fuse dominates bake time). Built from
 *  Replicad primitives inside the worker, no STEP import. Outer diameter /
 *  bore diameter / thickness in mm; frame is centered at origin with the
 *  disc's axis along +Z. `connectorOriginMm` + `connectorAxisXyz` are the
 *  child-side mate point, in the disc's local frame. */
export interface ParametricDiscChild {
  kind: 'disc'
  odMm: number
  idMm: number          // 0 for a solid disc
  thicknessMm: number
  connectorOriginMm: [number, number, number]
  connectorAxisXyz: [number, number, number]
}

export interface FuseParamSpec {
  parent: FusePart
  child: ParametricDiscChild
  filletRadiusMm?: number
  filletSlabMm?: number
  fuseOptimisation?: 'none' | 'commonFace' | 'sameFace'
  skipFuse?: boolean
  simplifyImports?: boolean
  /** How far to sink the child INTO the parent (along the parent connector
   *  axis, in mm) before fusing. Without this, solids placed flush at a
   *  shared face produce a Compound, not a merged Solid — the fuse appears
   *  to succeed but the mesh shows two parts stuck together. Default 0.3 mm
   *  gives OCCT enough real interpenetration to compute a clean merged
   *  topology without visibly sinking the child. */
  interpenetrationMm?: number
}

// ── Phase 3: cluster bake (fold many parts into one fused mesh) ──

/** One member of a cluster. Each part lives in the cluster root's frame
 *  (mm), so the worker doesn't need to do kinematic traversal — the main
 *  thread already computed each member's transform when it walked the
 *  Three.js scene graph. */
export type ClusterPart =
  | {
      kind: 'step'
      stepUrl: string
      /** Translation in cluster-root frame, mm. */
      translateMm: [number, number, number]
      /** Extrinsic XYZ Euler rotation applied BEFORE translation, radians. */
      rotateRadXyz: [number, number, number]
      /** Per-preset rotation override applied to geometry BEFORE centering +
       *  cluster transform. Matches `richVisuals/meshOverrides.ts` ROTATION_
       *  OVERRIDES — undefined = identity. */
      rotationOverrideRadXyz?: [number, number, number]
      /** Center the part on its bbox before applying the cluster transform.
       *  Matches the final step in `applyMeshToLink`. Default true — matches
       *  per-preset rendering; set false only if the part's native STEP frame
       *  is already at the link origin (rare). */
      centerOnBbox?: boolean
      /** For bake hygiene — strips redundant topology. Default true. */
      simplify?: boolean
      /** Preset's authored bounding box (mm). When provided AND the per-axis
       *  ratios from STEP-actual → authored agree within ±15%, the worker
       *  applies a single uniform scale (geometric-mean of the three ratios)
       *  to bring the rendered silhouette to authored size. Wrong-shape STEPs
       *  (per-axis variance > 1.15×) fall through unscaled — see
       *  `docs/STEP_BBOX_AUDIT.md` for the wrong-shape vs. uniform-scale
       *  split, and the bbox-fit handoff doc for why anisotropic per-axis
       *  residuals are intentionally NOT applied. */
      authoredBboxMm?: [number, number, number]
    }
  | {
      kind: 'disc'
      odMm: number
      idMm: number
      thicknessMm: number
      translateMm: [number, number, number]
      rotateRadXyz: [number, number, number]
    }
  | {
      /** Parametric axis-aligned box. Used for extrusions and other
       *  parametric presets whose STEP is authored at a fixed length but
       *  the URDF renders at a variable size (e.g. structural_extrusion_2020
       *  has a 20×20 cross-section with length_mm on the AssemblyComponent).
       *  Per-preset renders these as a <box> geometry already; baking them
       *  from STEP would pull the wrong length. */
      kind: 'box'
      sizeMm: [number, number, number]
      translateMm: [number, number, number]
      rotateRadXyz: [number, number, number]
    }
  | {
      /** Parametric T-slot aluminum extrusion. Main bar with a wide groove
       *  cut into each face and a narrow center ridge left inside each
       *  groove — matches the real 80/20-style profile readable as an
       *  extrusion at a glance. Length along +Z, cross section in X/Y. */
      kind: 'extrusion'
      crossSectionMm: [number, number]
      lengthMm: number
      /** Groove width across the face (tangent direction). */
      slotWidthMm: number
      /** How deep the groove cuts into the bar from each face. */
      slotDepthMm: number
      /** Thin center ridge left inside the groove — its top sits slightly
       *  below the bar's outer face. Reads as the classic T-slot "divider". */
      ridgeWidthMm: number
      ridgeHeightMm: number
      /** When provided, override `lengthMm` with this value. Cross-section +
       *  slot + ridge dims stay parametric (preset-authored). Set from the
       *  live Three.js scene's link-LOCAL Z extent so a URDF whose
       *  `length_mm` was dropped on reverse-parse still bakes at the right
       *  length — without it, the spec falls back to `lengthMm` which is
       *  100mm on a stale graph. Extrusion length is the only STEP-/preset-
       *  vs-URDF dim mismatch the bake fixes; everything else (servo bbox
       *  etc.) renders at native CAD dims so the visual stays faithful. */
      targetLengthMm?: number
      translateMm: [number, number, number]
      rotateRadXyz: [number, number, number]
    }

/** One fuse boundary between parts[i-1] and parts[i] (zero-indexed from 1).
 *  Describes the mate plane so the fillet knows where to round. */
export interface ClusterJoint {
  /** Lateral radius of the inBox fillet-edge filter (mm). Should cover the
   *  child's footprint with some margin; tighter means fewer incidental
   *  edges get rounded. */
  filletBoxHalfSideMm: number
  /** Mate plane center in cluster-root frame (mm). Typically the parent's
   *  connector origin after its world transform. */
  planeCenterMm: [number, number, number]
  /** Unit axis perpendicular to the mate plane (cluster-root frame). */
  axisXyz: [number, number, number]
  /** Slab half-thickness around the mate plane (mm). Default 0.15. */
  slabMm?: number
  /** Attempted fillet radius (mm). Worker shrinks and retries internally on
   *  OCCT reject. Zero disables fillet for this boundary. */
  filletRadiusMm: number
  /** Whether to skip the fuse boolean for this boundary (falls back to
   *  appending the child mesh as a sibling — compound rendering). */
  skipFuse?: boolean
  /** Fuse optimisation hint passed to OCCT. */
  fuseOptimisation?: 'none' | 'commonFace' | 'sameFace'
  /** Debug label — logged so failures can be traced to a specific URDF joint. */
  debugLabel?: string
}

export interface BakeClusterSpec {
  /** First part is the cluster root; its transform is ignored (the baked
   *  mesh is returned in the root's frame). Each subsequent part is fused
   *  with the accumulated shape via joints[i-1]. */
  parts: ClusterPart[]
  joints: ClusterJoint[]
  /** When true, skip fuse+fillet entirely and return one tessellated mesh
   *  per part. Lets the main thread apply per-part materials (preserving
   *  per-component colors). Trade-off: tangent contacts may z-fight where
   *  fuse would have merged, but we keep authored colors. */
  perPartMeshes?: boolean
}

/** Single placed part's tessellation, used when `perPartMeshes` is on. */
export interface PartMesh {
  partIdx: number
  mesh: SerializedMesh
  edges?: SerializedEdges
}

export type BakeCommand =
  | { kind: 'bakeSingle'; id: string; stepUrl: string; opts?: MeshOpts }
  | { kind: 'bakeFuseTwo'; id: string; fuse: FuseSpec; opts?: MeshOpts }
  | { kind: 'bakeFuseParametric'; id: string; fuse: FuseParamSpec; opts?: MeshOpts }
  | { kind: 'bakeCluster'; id: string; cluster: BakeClusterSpec; opts?: MeshOpts }

export type BakeResponse =
  | { kind: 'ready'; id: string }
  | { kind: 'result'; id: string; ok: true; mesh: SerializedMesh; edges?: SerializedEdges; diagnostics: BakeDiagnostics }
  | { kind: 'result'; id: string; ok: true; parts: PartMesh[]; diagnostics: BakeDiagnostics }
  | { kind: 'result'; id: string; ok: false; phase: string; message: string; diagnostics?: BakeDiagnostics }

export interface BakeDiagnostics {
  importMs: number
  meshMs: number
  edgeMs: number
  triangleCount: number
  vertexCount: number
  fuseMs?: number
  filletMs?: number
  filletRadiusMm?: number
  filletFellBackToFuseOnly?: boolean
}

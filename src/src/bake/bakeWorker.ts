/// <reference lib="webworker" />
// B-Rep bake WebWorker — runs Replicad/OpenCascade WASM off the main thread.
//
// Phase 1 scope: a single `bakeSingle` op that fetches a STEP file, imports it
// via Replicad, tessellates to a triangle mesh, and posts the buffers back.
// Phases 2+ will extend this file with fuse/fillet ops; the worker host only
// needs updating when new message kinds are added.
//
// Important: OCCT WASM is ~10 MB. It loads lazily on the first bake command.
// The main thread should spawn the worker early so this cost is paid in the
// background rather than at user-click time.

import initOpenCascade from 'replicad-opencascadejs'
import opencascadeWasmUrl from 'replicad-opencascadejs/src/replicad_single.wasm?url'
import type { OpenCascadeInstance } from 'replicad-opencascadejs'
import { setOC, importSTEP, makeCylinder, makeBaseBox, cast } from 'replicad'
import type { AnyShape, Shape3D, PlaneName } from 'replicad'
import type {
  BakeCommand, BakeResponse, SerializedMesh, SerializedEdges, BakeDiagnostics, MeshOpts,
  FuseSpec, FuseParamSpec, ParametricDiscChild, BakeClusterSpec, ClusterPart, PartMesh,
} from './types.ts'

// DedicatedWorkerGlobalScope narrows `self` so TS picks the worker postMessage
// overload (value, transfer[]) instead of the window one.
declare const self: DedicatedWorkerGlobalScope

// Top-level OCCT init — single in-flight promise shared by all bake requests.
// `initOpenCascade()` accepts a Module-overrides object; the declared TS type
// is no-arg (the replicad-opencascadejs .d.ts only exports the default zero-arg
// signature) so we cast.
let occtReady: Promise<OpenCascadeInstance> | null = null
function ensureOcct(): Promise<OpenCascadeInstance> {
  if (occtReady) return occtReady
  occtReady = (async () => {
    const init = initOpenCascade as unknown as (args?: { locateFile?: (path: string) => string }) => Promise<OpenCascadeInstance>
    const OC = await init({
      locateFile: (path: string) => {
        // replicad_single.js asks for its own .wasm by the filename only. Vite's
        // ?url import resolves to a hashed asset URL for the bundled build and
        // a dev-server URL during `vite dev`; both cases return the right path.
        if (path.endsWith('.wasm')) return opencascadeWasmUrl
        return path
      },
    })
    // Cast — replicad's setOC expects its own OpenCascadeInstance type which is
    // structurally the same as what initOpenCascade returns but the type export
    // graph doesn't narrow automatically.
    setOC(OC as never)
    return OC
  })()
  return occtReady
}

// Co-located .binbrep variant (OCCT native binary B-Rep) takes priority over
// the .step source — same basename, different extension. Parses ~50x faster
// than STEP through the same OCCT instance and the files are 50-80% smaller;
// see scripts/convert_steps_to_brep.sh for the offline conversion. Falls
// through to STEP when no .binbrep sibling exists so files that haven't been
// converted yet still bake.
interface LoadedShape { shape: AnyShape; bytes: number; source: 'binbrep' | 'step' }
async function loadShapeFromUrl(stepUrl: string): Promise<LoadedShape> {
  const oc = await ensureOcct()
  const binbrepUrl = stepUrl.replace(/\.(step|stp)$/i, '.binbrep')
  if (binbrepUrl !== stepUrl) {
    const res = await fetch(binbrepUrl)
    if (res.ok) {
      const bytes = new Uint8Array(await res.arrayBuffer())
      return { shape: importBinBRep(bytes, oc), bytes: bytes.length, source: 'binbrep' }
    }
    // Non-200 → fall through to STEP. The file probably just hasn't been
    // converted yet; do not throw.
  }
  const res = await fetch(stepUrl)
  if (!res.ok) throw new Error(`HTTP ${res.status} for ${stepUrl}`)
  const blob = await res.blob()
  const shape = await importSTEP(blob) as AnyShape
  return { shape, bytes: blob.size, source: 'step' }
}

// BinTools.Read takes a file path, so we stage the blob in OCCT's MEMFS, read
// it, then unlink. Path is uniquified so parallel cluster-part loads can't
// collide on the same temp name.
function importBinBRep(bytes: Uint8Array, oc: OpenCascadeInstance): AnyShape {
  const tmpPath = `/tmp/bake_${Date.now()}_${Math.random().toString(36).slice(2)}.binbrep`
  oc.FS.writeFile(tmpPath, bytes)
  try {
    const shape = new oc.TopoDS_Shape()
    const range = new oc.Message_ProgressRange_1()
    const ok = oc.BinTools.Read_2(shape, tmpPath, range)
    if (!ok) throw new Error(`BinTools.Read_2 returned false for ${tmpPath}`)
    return cast(shape) as AnyShape
  } finally {
    try { oc.FS.unlink(tmpPath) } catch { /* MEMFS unlink should not fail; swallow if it does */ }
  }
}

async function bakeSinglePreset(stepUrl: string, opts: MeshOpts | undefined): Promise<BakeResponse> {
  const id = stepUrl
  const diagnostics: BakeDiagnostics = {
    importMs: 0, meshMs: 0, edgeMs: 0, triangleCount: 0, vertexCount: 0,
  }
  try {
    const t0 = performance.now()
    let shape: AnyShape
    try {
      const loaded = await loadShapeFromUrl(stepUrl)
      shape = loaded.shape
    } catch (e) {
      const msg = e instanceof Error ? e.message : String(e)
      // STEP fallback fetch failure (binbrep miss is silent — see loadShapeFromUrl).
      if (msg.startsWith('HTTP ')) {
        return {
          kind: 'result', id, ok: false,
          phase: 'fetch',
          message: `Failed to fetch ${stepUrl}: ${msg}`,
        }
      }
      throw e
    }
    diagnostics.importMs = performance.now() - t0

    const t1 = performance.now()
    const tolerance = opts?.tolerance ?? 0.05          // mm — OCCT native units
    const angularTolerance = opts?.angularTolerance ?? 20
    const meshData = shape.mesh({ tolerance, angularTolerance })
    diagnostics.meshMs = performance.now() - t1

    const t2 = performance.now()
    const edgeData = shape.meshEdges({ tolerance, angularTolerance })
    diagnostics.edgeMs = performance.now() - t2

    // Convert number[] → typed arrays so they can be transferred.
    const positions = new Float32Array(meshData.vertices)
    const normals = new Float32Array(meshData.normals)
    const indices = new Uint32Array(meshData.triangles)
    const edgePositions = new Float32Array(edgeData.lines)
    diagnostics.triangleCount = indices.length / 3
    diagnostics.vertexCount = positions.length / 3

    const mesh: SerializedMesh = { positions, normals, indices }
    const edges: SerializedEdges = { positions: edgePositions }
    return {
      kind: 'result', id, ok: true, mesh, edges, diagnostics,
    }
  } catch (e) {
    const message = e instanceof Error ? `${e.name}: ${e.message}` : String(e)
    return {
      kind: 'result', id, ok: false,
      phase: 'occt',
      message,
      diagnostics,
    }
  }
}

// ── Phase 2: fuse + fillet a single mate ──

function approxEq(a: number, b: number, tol = 1e-4): boolean {
  return Math.abs(a - b) <= tol
}

function antiparallel(a: [number, number, number], b: [number, number, number]): boolean {
  // Expect unit-ish vectors in the generic_presets.json; |a+b| ≈ 0 when
  // antiparallel, |a-b| ≈ 0 when parallel. We only mate antiparallel today.
  const sx = a[0] + b[0], sy = a[1] + b[1], sz = a[2] + b[2]
  return approxEq(sx, 0) && approxEq(sy, 0) && approxEq(sz, 0)
}

function vecLen(v: [number, number, number]): number {
  return Math.hypot(v[0], v[1], v[2])
}

/** One of replicad's three standard plane-name aliases that matches the
 *  face-normal carried by this connector axis. The mate plane is perpendicular
 *  to the axis, so axis +Z → 'XY', axis +X → 'YZ', axis +Y → 'XZ'. Returns
 *  null for oblique axes — Phase 2 scope is axis-aligned mates only, Phase 4
 *  relaxes this. */
function planeNameForAxis(axis: [number, number, number]): { name: PlaneName; projectIdx: 0 | 1 | 2 } | null {
  const len = vecLen(axis)
  if (len < 1e-4) return null
  const n: [number, number, number] = [axis[0] / len, axis[1] / len, axis[2] / len]
  const ax = Math.abs(n[0]), ay = Math.abs(n[1]), az = Math.abs(n[2])
  if (az > 0.999) return { name: 'XY', projectIdx: 2 }
  if (ax > 0.999) return { name: 'YZ', projectIdx: 0 }
  if (ay > 0.999) return { name: 'XZ', projectIdx: 1 }
  return null
}

async function bakeFuseTwo(spec: FuseSpec, opts: MeshOpts | undefined): Promise<BakeResponse> {
  const diagnostics: BakeDiagnostics = {
    importMs: 0, meshMs: 0, edgeMs: 0, triangleCount: 0, vertexCount: 0,
    fuseMs: 0, filletMs: 0,
    filletRadiusMm: spec.filletRadiusMm ?? 0,
    filletFellBackToFuseOnly: false,
  }
  try {
    await ensureOcct()

    // Antiparallel check — authored connector pairs in the catalog follow
    // this convention. Non-antiparallel would need an extra rotation step
    // we don't compute in Phase 2.
    if (!antiparallel(spec.parent.connectorAxisXyz, spec.child.connectorAxisXyz)) {
      return {
        kind: 'result', id: '', ok: false,
        phase: 'mate-convention',
        message: `connector axes are not antiparallel — parent=${spec.parent.connectorAxisXyz.join(',')} child=${spec.child.connectorAxisXyz.join(',')}`,
        diagnostics,
      }
    }

    const planeInfo = planeNameForAxis(spec.parent.connectorAxisXyz)
    if (!planeInfo) {
      return {
        kind: 'result', id: '', ok: false,
        phase: 'mate-axis',
        message: `Phase 2 only supports axis-aligned mates; got axis ${spec.parent.connectorAxisXyz.join(',')}`,
        diagnostics,
      }
    }

    const t0 = performance.now()
    const [parentLoaded, childLoaded] = await Promise.all([
      loadShapeFromUrl(spec.parent.stepUrl),
      loadShapeFromUrl(spec.child.stepUrl),
    ])
    console.log(`[bake/worker] loaded parent (${parentLoaded.source}) ${parentLoaded.bytes}B, child (${childLoaded.source}) ${childLoaded.bytes}B in ${(performance.now()-t0).toFixed(0)}ms`)
    let parent = parentLoaded.shape
    console.log(`[bake/worker] parent faces=${parent.faces.length} edges=${parent.edges.length}`)
    let childRaw = childLoaded.shape
    console.log(`[bake/worker] child  faces=${childRaw.faces.length} edges=${childRaw.edges.length}`)

    // Simplify strips redundant topology (coupler has 974 edges from its
    // 25-tooth splined bore; simplify typically drops it to ~80-200). Skip
    // only when explicitly opted out for debug comparison.
    if (spec.simplifyImports !== false) {
      const tSim = performance.now()
      parent = (parent as unknown as { simplify: () => AnyShape }).simplify() as AnyShape
      childRaw = (childRaw as unknown as { simplify: () => AnyShape }).simplify() as AnyShape
      console.log(`[bake/worker] simplify() ${(performance.now()-tSim).toFixed(0)}ms parent faces=${parent.faces.length} edges=${parent.edges.length} child faces=${childRaw.faces.length} edges=${childRaw.edges.length}`)
    }
    diagnostics.importMs = performance.now() - t0

    // Translate the child so its connector origin coincides with the parent's
    // in world coords. Parent sits at identity — its connector is already at
    // parent.connectorOriginMm in world frame.
    const [pX, pY, pZ] = spec.parent.connectorOriginMm
    const [cX, cY, cZ] = spec.child.connectorOriginMm
    const child = childRaw.translate(pX - cX, pY - cY, pZ - cZ) as AnyShape

    // Resolve the combined shape. `skipFuse` is a debug escape hatch — meshes
    // both parts as a single compound without boolean fusing. Useful to prove
    // placement+mesh work when fuse times out on multi-solid STEPs.
    let out: Shape3D
    if (spec.skipFuse) {
      console.log(`[bake/worker] skipFuse=true — meshing parent+translated-child without boolean fuse`)
      // We can still mesh each separately and concatenate buffers below, but
      // the happy path mesh/edges code expects a single Shape3D. Fall through
      // with parent as the "out" shape and append child's mesh afterwards.
      out = parent as unknown as Shape3D
      diagnostics.filletFellBackToFuseOnly = true
    } else {
      console.log(`[bake/worker] fuse(optimisation=${spec.fuseOptimisation ?? 'none'}) starting...`)
      const t1 = performance.now()
      let fused: Shape3D
      try {
        fused = (parent as unknown as Shape3D).fuse(
          child as unknown as Shape3D,
          { optimisation: spec.fuseOptimisation ?? 'none' },
        )
      } catch (e) {
        const message = e instanceof Error ? `${e.name}: ${e.message}` : String(e)
        console.error(`[bake/worker] fuse threw after ${(performance.now()-t1).toFixed(0)}ms: ${message}`)
        return {
          kind: 'result', id: '', ok: false,
          phase: 'fuse', message, diagnostics,
        }
      }
      diagnostics.fuseMs = performance.now() - t1
      console.log(`[bake/worker] fuse done in ${diagnostics.fuseMs.toFixed(0)}ms faces=${fused.faces.length} edges=${fused.edges.length}`)
      out = fused

      // Fillet at the mate plane. Use `EdgeFinder.inPlane(planeName, offset)` —
      // the offset on an axis-aligned plane is the connector's coordinate along
      // that axis (planeInfo.projectIdx).
      const filletRadius = spec.filletRadiusMm ?? 0
      const mateOffset = spec.parent.connectorOriginMm[planeInfo.projectIdx]
      if (filletRadius > 0) {
        const t2 = performance.now()
        try {
          out = fused.fillet(filletRadius, e => e.inPlane(planeInfo.name, mateOffset)) as Shape3D
          diagnostics.filletMs = performance.now() - t2
          console.log(`[bake/worker] fillet done in ${diagnostics.filletMs.toFixed(0)}ms`)
        } catch (e) {
          // Fillet reject (radius too large for adjacent geometry, non-manifold
          // seam) — Phase 4 adds shrink-and-retry. Phase 2 degrades to fuse-only
          // so we still see the fused geometry instead of a hard failure.
          diagnostics.filletFellBackToFuseOnly = true
          const message = e instanceof Error ? `${e.name}: ${e.message}` : String(e)
          console.warn(`[bake/worker] fillet failed (radius=${filletRadius}mm at ${planeInfo.name}@${mateOffset}): ${message} — falling back to fuse-only`)
        }
      }
    }

    const tolerance = opts?.tolerance ?? 0.05
    const angularTolerance = opts?.angularTolerance ?? 20

    const t3 = performance.now()
    // In skipFuse mode we need to mesh parent AND child separately then
    // concatenate — `out` is set to `parent` above, and the translated child
    // still needs its own mesh call.
    let positions: Float32Array
    let normals: Float32Array
    let indices: Uint32Array
    let edgePositions: Float32Array
    if (spec.skipFuse) {
      const parentMeshData = (parent as unknown as Shape3D).mesh({ tolerance, angularTolerance })
      const childMeshData = (child as unknown as Shape3D).mesh({ tolerance, angularTolerance })
      const parentEdgeData = (parent as unknown as Shape3D).meshEdges({ tolerance, angularTolerance })
      const childEdgeData = (child as unknown as Shape3D).meshEdges({ tolerance, angularTolerance })
      // Concatenate. Indices on the child must be offset by parent.vertices.length / 3.
      const parentVertCount = parentMeshData.vertices.length / 3
      positions = new Float32Array(parentMeshData.vertices.length + childMeshData.vertices.length)
      positions.set(parentMeshData.vertices as unknown as ArrayLike<number>, 0)
      positions.set(childMeshData.vertices as unknown as ArrayLike<number>, parentMeshData.vertices.length)
      normals = new Float32Array(parentMeshData.normals.length + childMeshData.normals.length)
      normals.set(parentMeshData.normals as unknown as ArrayLike<number>, 0)
      normals.set(childMeshData.normals as unknown as ArrayLike<number>, parentMeshData.normals.length)
      indices = new Uint32Array(parentMeshData.triangles.length + childMeshData.triangles.length)
      indices.set(parentMeshData.triangles as unknown as ArrayLike<number>, 0)
      for (let i = 0; i < childMeshData.triangles.length; i++) {
        indices[parentMeshData.triangles.length + i] = childMeshData.triangles[i] + parentVertCount
      }
      edgePositions = new Float32Array(parentEdgeData.lines.length + childEdgeData.lines.length)
      edgePositions.set(parentEdgeData.lines as unknown as ArrayLike<number>, 0)
      edgePositions.set(childEdgeData.lines as unknown as ArrayLike<number>, parentEdgeData.lines.length)
    } else {
      const meshData = out.mesh({ tolerance, angularTolerance })
      const edgeData = out.meshEdges({ tolerance, angularTolerance })
      positions = new Float32Array(meshData.vertices)
      normals = new Float32Array(meshData.normals)
      indices = new Uint32Array(meshData.triangles)
      edgePositions = new Float32Array(edgeData.lines)
    }
    diagnostics.meshMs = performance.now() - t3

    diagnostics.edgeMs = 0 // folded into meshMs for the concatenated path
    diagnostics.triangleCount = indices.length / 3
    diagnostics.vertexCount = positions.length / 3
    console.log(`[bake/worker] mesh+edges done in ${diagnostics.meshMs.toFixed(0)}ms — tris=${diagnostics.triangleCount}`)

    const mesh: SerializedMesh = { positions, normals, indices }
    const edges: SerializedEdges = { positions: edgePositions }
    return { kind: 'result', id: '', ok: true, mesh, edges, diagnostics }
  } catch (e) {
    const message = e instanceof Error ? `${e.name}: ${e.message}` : String(e)
    return {
      kind: 'result', id: '', ok: false,
      phase: 'fuse-op', message, diagnostics,
    }
  }
}

/** Build a parametric disc-with-bore shape from Replicad primitives. Uses
 *  makeCylinder directly — no sketching layer, minimal topology. Returned
 *  shape is axis-aligned with local +Z and centered at origin. */
function buildParametricDisc(spec: ParametricDiscChild): Shape3D {
  const half = spec.thicknessMm / 2
  // makeCylinder(radius, height) — height extends along local +Z from origin.
  // Translate down by half so the disc is centered on z=0.
  const outer = makeCylinder(spec.odMm / 2, spec.thicknessMm).translateZ(-half) as unknown as Shape3D
  if (spec.idMm <= 0) return outer
  const bore = makeCylinder(spec.idMm / 2, spec.thicknessMm).translateZ(-half) as unknown as Shape3D
  return outer.cut(bore) as unknown as Shape3D
}

async function bakeFuseParametric(spec: FuseParamSpec, opts: MeshOpts | undefined): Promise<BakeResponse> {
  const diagnostics: BakeDiagnostics = {
    importMs: 0, meshMs: 0, edgeMs: 0, triangleCount: 0, vertexCount: 0,
    fuseMs: 0, filletMs: 0,
    filletRadiusMm: spec.filletRadiusMm ?? 0,
    filletFellBackToFuseOnly: false,
  }
  try {
    await ensureOcct()

    if (!antiparallel(spec.parent.connectorAxisXyz, spec.child.connectorAxisXyz)) {
      return {
        kind: 'result', id: '', ok: false,
        phase: 'mate-convention',
        message: `connector axes are not antiparallel — parent=${spec.parent.connectorAxisXyz.join(',')} child=${spec.child.connectorAxisXyz.join(',')}`,
        diagnostics,
      }
    }
    const planeInfo = planeNameForAxis(spec.parent.connectorAxisXyz)
    if (!planeInfo) {
      return {
        kind: 'result', id: '', ok: false,
        phase: 'mate-axis',
        message: `Phase 2 only supports axis-aligned mates; got axis ${spec.parent.connectorAxisXyz.join(',')}`,
        diagnostics,
      }
    }

    // Import the parent STEP (or its .binbrep sibling, when present).
    const t0 = performance.now()
    const parentLoaded = await loadShapeFromUrl(spec.parent.stepUrl)
    let parent = parentLoaded.shape
    console.log(`[bake/worker:param] loaded parent (${parentLoaded.source}) ${parentLoaded.bytes}B in ${(performance.now()-t0).toFixed(0)}ms faces=${parent.faces.length} edges=${parent.edges.length}`)

    if (spec.simplifyImports !== false) {
      const tSim = performance.now()
      parent = (parent as unknown as { simplify: () => AnyShape }).simplify() as AnyShape
      console.log(`[bake/worker:param] parent simplify() ${(performance.now()-tSim).toFixed(0)}ms → faces=${parent.faces.length} edges=${parent.edges.length}`)
    }
    diagnostics.importMs = performance.now() - t0

    // Build parametric child.
    const tBuild = performance.now()
    const childLocal = buildParametricDisc(spec.child)
    console.log(`[bake/worker:param] buildDisc() ${(performance.now()-tBuild).toFixed(0)}ms faces=${childLocal.faces.length} edges=${childLocal.edges.length}`)

    // Place child so its connector origin coincides with parent's in world,
    // then sink it slightly along the parent axis (away from the child's
    // outward direction, i.e. INTO the parent). Without this overlap, OCCT's
    // fuse on flush-tangent solids produces a Compound, not a merged Solid
    // — fillet then has nothing to do and the mesh looks un-joined.
    const [pX, pY, pZ] = spec.parent.connectorOriginMm
    const [cX, cY, cZ] = spec.child.connectorOriginMm
    const dx = pX - cX
    const dy = pY - cY
    const dz = pZ - cZ
    // Embed along parent's outward axis (= opposite of child axis) → child
    // moves INTO parent. Using child axis directly means we multiply by +1
    // to get the direction away from child = into parent.
    const embed = spec.interpenetrationMm ?? 0.3
    const [cax, cay, caz] = spec.child.connectorAxisXyz
    const clen = Math.hypot(cax, cay, caz) || 1
    const nx = cax / clen, ny = cay / clen, nz = caz / clen
    // Child connector axis points OUTWARD FROM child. To sink child INTO
    // parent we move child along child_axis by +embed (child's own outward
    // becomes the interpenetration direction when placed against parent).
    const child = childLocal.translate(dx + nx * embed, dy + ny * embed, dz + nz * embed) as unknown as Shape3D
    console.log(`[bake/worker:param] child placement: translate=(${dx.toFixed(2)},${dy.toFixed(2)},${dz.toFixed(2)})mm + embed=${embed}mm along child-axis (${nx.toFixed(2)},${ny.toFixed(2)},${nz.toFixed(2)})`)

    let out: Shape3D
    if (spec.skipFuse) {
      console.log(`[bake/worker:param] skipFuse=true`)
      out = parent as unknown as Shape3D
    } else {
      console.log(`[bake/worker:param] fuse(opt=${spec.fuseOptimisation ?? 'none'}) starting...`)
      const t1 = performance.now()
      let fused: Shape3D
      try {
        fused = (parent as unknown as Shape3D).fuse(child, { optimisation: spec.fuseOptimisation ?? 'none' })
      } catch (e) {
        const message = e instanceof Error ? `${e.name}: ${e.message}` : String(e)
        console.error(`[bake/worker:param] fuse threw after ${(performance.now()-t1).toFixed(0)}ms: ${message}`)
        return { kind: 'result', id: '', ok: false, phase: 'fuse', message, diagnostics }
      }
      diagnostics.fuseMs = performance.now() - t1
      console.log(`[bake/worker:param] fuse done in ${diagnostics.fuseMs.toFixed(0)}ms faces=${fused.faces.length} edges=${fused.edges.length}`)
      out = fused

      const filletRadius = spec.filletRadiusMm ?? 0
      if (filletRadius > 0) {
        // Filter to edges at the mate plane using a thin inBox slab around it
        // — DON'T use EdgeFinder.inPlane: the Replicad docs note it matches
        // LINES only, not circular edges, so disc-on-plate seams (circles
        // lying in the plane) return zero matches and the fillet fails with
        // "no edge was selected". inBox works on any edge geometry.
        const [pcx, pcy, pcz] = spec.parent.connectorOriginMm
        const LAT = Math.max(spec.child.odMm, 16) + 4  // disc OD + margin
        const SLAB = 0.15  // thin z-slab to isolate the mate ring from other
                          // horizontal-ish edges that happen to skirt the plane
        // Build box corners — the mate axis's projectIdx tells us which
        // component is "narrow"; the other two get the LAT extent.
        const mkCorner = (sign: -1 | 1): [number, number, number] => {
          const out: [number, number, number] = [pcx + sign * LAT, pcy + sign * LAT, pcz + sign * LAT]
          out[planeInfo.projectIdx] = (planeInfo.projectIdx === 0 ? pcx : planeInfo.projectIdx === 1 ? pcy : pcz) + sign * SLAB
          return out
        }
        const corner1 = mkCorner(-1)
        const corner2 = mkCorner(1)
        // Radius shrink-and-retry (Phase 4 has the full loop; this early
        // version gets us through the Phase 2 smoke test).
        const radiiToTry = [filletRadius, filletRadius * 0.5, filletRadius * 0.25]
        let filletApplied = false
        const t2 = performance.now()
        for (const r of radiiToTry) {
          try {
            out = fused.fillet(r, e => e.inBox(corner1, corner2)) as Shape3D
            diagnostics.filletMs = performance.now() - t2
            diagnostics.filletRadiusMm = r
            console.log(`[bake/worker:param] fillet done r=${r.toFixed(3)}mm in ${diagnostics.filletMs.toFixed(0)}ms box=(${corner1.map(n=>n.toFixed(1)).join(',')})→(${corner2.map(n=>n.toFixed(1)).join(',')})`)
            filletApplied = true
            break
          } catch (e) {
            const message = e instanceof Error ? `${e.name}: ${e.message}` : String(e)
            console.warn(`[bake/worker:param] fillet r=${r.toFixed(3)}mm failed: ${message} — retrying smaller`)
          }
        }
        if (!filletApplied) {
          diagnostics.filletFellBackToFuseOnly = true
          console.warn(`[bake/worker:param] all fillet radii failed — falling back to fuse-only`)
        }
      }
    }

    const t3 = performance.now()
    const tolerance = opts?.tolerance ?? 0.05
    const angularTolerance = opts?.angularTolerance ?? 20
    const meshData = out.mesh({ tolerance, angularTolerance })
    const edgeData = out.meshEdges({ tolerance, angularTolerance })
    diagnostics.meshMs = performance.now() - t3

    const positions = new Float32Array(meshData.vertices)
    const normals = new Float32Array(meshData.normals)
    const indices = new Uint32Array(meshData.triangles)
    const edgePositions = new Float32Array(edgeData.lines)
    diagnostics.triangleCount = indices.length / 3
    diagnostics.vertexCount = positions.length / 3

    const mesh: SerializedMesh = { positions, normals, indices }
    const edges: SerializedEdges = { positions: edgePositions }
    return { kind: 'result', id: '', ok: true, mesh, edges, diagnostics }
  } catch (e) {
    const message = e instanceof Error ? `${e.name}: ${e.message}` : String(e)
    return { kind: 'result', id: '', ok: false, phase: 'fuse-param-op', message, diagnostics }
  }
}

// ── Phase 3: cluster bake (left-fold fuse over N parts) ──

async function loadClusterPart(part: ClusterPart): Promise<Shape3D> {
  if (part.kind === 'step') {
    const loaded = await loadShapeFromUrl(part.stepUrl)
    let shape = loaded.shape
    if (part.simplify !== false) {
      shape = (shape as unknown as { simplify: () => AnyShape }).simplify() as AnyShape
    }
    let shape3d = shape as unknown as Shape3D
    // Apply per-preset rotation override (matches richVisuals applyMeshToLink
    // step 2). Converts the STEP's native axes to the preset's authored
    // convention — e.g. coupler disc has thickness on Y in its STEP but the
    // preset puts thickness on Z, so rotate X by π/2.
    if (part.rotationOverrideRadXyz) {
      // THREE.Euler 'XYZ' → apply Z, Y, X (see applyRotThenTrans comment).
      const [rx, ry, rz] = part.rotationOverrideRadXyz
      const R2D = 180 / Math.PI
      if (Math.abs(rz) > 1e-6) shape3d = shape3d.rotate(rz * R2D, [0, 0, 0], [0, 0, 1]) as Shape3D
      if (Math.abs(ry) > 1e-6) shape3d = shape3d.rotate(ry * R2D, [0, 0, 0], [0, 1, 0]) as Shape3D
      if (Math.abs(rx) > 1e-6) shape3d = shape3d.rotate(rx * R2D, [0, 0, 0], [1, 0, 0]) as Shape3D
    }
    // Uniform-only auto-scale. Fires only when the per-axis ratios from
    // STEP→authored agree within ±15% — i.e. same shape, wrong size (the
    // 21 entries in `docs/STEP_BBOX_AUDIT.md` "Uniform-scale mismatch"
    // section). Wrong-shape STEPs (variance > 1.15×, e.g. the high-torque
    // servo's 33×54×58 vs authored 47×36×26) fall through unscaled — see
    // the bbox-fit handoff doc for why anisotropic per-axis residuals are
    // intentionally NOT applied (they squashed real CAD silhouettes into
    // pucks). Replicad's `Shape.scale` is uniform-only by API; we pick
    // the geometric-mean factor to minimise the max-axis error.
    if (part.authoredBboxMm) {
      const bb = shape3d.boundingBox as unknown as { width: number; height: number; depth: number }
      const actual: [number, number, number] = [bb.width, bb.height, bb.depth]
      const authored = part.authoredBboxMm
      if (actual[0] > 0.1 && actual[1] > 0.1 && actual[2] > 0.1) {
        const ratios: [number, number, number] = [
          authored[0] / actual[0],
          authored[1] / actual[1],
          authored[2] / actual[2],
        ]
        const variance = Math.max(...ratios) / Math.min(...ratios)
        const fileLabel = part.stepUrl.split('/').pop() ?? part.stepUrl
        if (variance < 1.15) {
          const factor = Math.cbrt(ratios[0] * ratios[1] * ratios[2])
          if (Math.abs(factor - 1) > 0.02) {
            shape3d = (shape3d as unknown as { scale: (s: number) => Shape3D }).scale(factor) as Shape3D
            console.log(`[bake/scale] ${fileLabel} factor=${factor.toFixed(3)} ratios=[${ratios.map(r => r.toFixed(2)).join(',')}] variance=${variance.toFixed(3)}`)
          }
        } else {
          console.log(`[bake/scale] ${fileLabel} SKIP wrong-shape ratios=[${ratios.map(r => r.toFixed(2)).join(',')}] variance=${variance.toFixed(3)}`)
        }
      }
    }
    // Center on bbox (matches richVisuals step 4). Without this, the baked
    // mesh sits at the STEP's native origin, while per-preset GLBs sit at
    // the link's origin — producing ~20-50mm offsets per part that add up
    // to clearly misplaced baked clusters.
    //
    // replicad's BoundingBox.center returns a tuple [cx, cy, cz] (NOT an
    // object with .x/.y/.z). Grep the package source (dist/replicad.js
    // `get center()` around line 544) to confirm.
    if (part.centerOnBbox !== false) {
      const bb = shape3d.boundingBox
      const c = (bb as unknown as { center: [number, number, number] }).center
      const [cx, cy, cz] = c
      if (Math.abs(cx) > 1e-6 || Math.abs(cy) > 1e-6 || Math.abs(cz) > 1e-6) {
        shape3d = shape3d.translate(-cx, -cy, -cz) as Shape3D
      }
    }
    return shape3d
  }
  if (part.kind === 'box') {
    // makeBaseBox(W, H, D) is centered in X/Y (sketch plane centered) but
    // extrudes from Z=0 up to Z=D — i.e. bottom face at Z=0, top at Z=D.
    // Per Replicad source: Sketcher().movePointerTo([-W/2, H/2]).hLine(W)
    // ....extrude(D). So only Z needs recentering to get a fully-centered box.
    const { sizeMm } = part
    const box = makeBaseBox(sizeMm[0], sizeMm[1], sizeMm[2]) as unknown as Shape3D
    return box.translate(0, 0, -sizeMm[2] / 2) as Shape3D
  }
  if (part.kind === 'extrusion') {
    // Real 2020/4040 T-slot aluminum extrusion: main bar with a wide groove
    // cut into each face, and a narrow center ridge left standing inside
    // each groove. The ridge's top sits a bit below the bar's outer face
    // so the groove remains visible on either side of it.
    //
    // Build process per face:
    //   1. Start with the main cx × cy × L bar
    //   2. CUT a (sW × sD × L) slot into each face — this is the wide
    //      groove the bolts would slide into on a real extrusion
    //   3. FUSE a (rW × rH × L) ridge back into the center of each slot
    //      — the ridge extends from the slot floor outward by rH, leaving
    //      (sD − rH) of recess visible on both sides of it
    const [cx, cy] = part.crossSectionMm
    // targetLengthMm wins over lengthMm — see ClusterPart docs. Reverse-
    // parsed URDFs drop length_mm, so the live-scene bbox length is the
    // only authoritative source on a reload.
    const L = part.targetLengthMm ?? part.lengthMm
    const sW = part.slotWidthMm
    const sD = part.slotDepthMm
    const rW = part.ridgeWidthMm
    const rH = part.ridgeHeightMm
    const mkCenteredAt = (xS: number, yS: number, zS: number, cX: number, cY: number, cZ: number): Shape3D => {
      // makeBaseBox is centered in X/Y but extrudes Z from 0 to zS. Apply
      // -zS/2 to center it fully, then shift to target center.
      const b = (makeBaseBox(xS, yS, zS) as unknown as Shape3D).translate(0, 0, -zS / 2) as Shape3D
      if (Math.abs(cX) < 1e-6 && Math.abs(cY) < 1e-6 && Math.abs(cZ) < 1e-6) return b
      return b.translate(cX, cY, cZ) as Shape3D
    }
    let out = mkCenteredAt(cx, cy, L, 0, 0, 0)
    const slotLen = L * 1.01
    // Cut the four grooves first, then fuse the four ridges. Doing cuts
    // together and fuses together keeps each boolean's input small and
    // matches the geometric intent.
    const grooves: Array<{ x: number; y: number; z: number; cX: number; cY: number }> = [
      { x: sW, y: sD, z: slotLen, cX: 0,             cY:  cy / 2 - sD / 2 },
      { x: sW, y: sD, z: slotLen, cX: 0,             cY: -cy / 2 + sD / 2 },
      { x: sD, y: sW, z: slotLen, cX:  cx / 2 - sD / 2, cY: 0 },
      { x: sD, y: sW, z: slotLen, cX: -cx / 2 + sD / 2, cY: 0 },
    ]
    for (const g of grooves) {
      const cutShape = mkCenteredAt(g.x, g.y, g.z, g.cX, g.cY, 0)
      try {
        out = out.cut(cutShape, { optimisation: 'none' })
      } catch (e) {
        const msg = e instanceof Error ? e.message : String(e)
        console.warn(`[bake/worker:extrusion] groove cut failed: ${msg} — continuing with remaining faces`)
      }
    }
    // Ridge position: centered laterally in its groove, standing from the
    // groove floor outward by rH. Each ridge is slightly shorter on Z than
    // the bar so its ends align with the outer bar faces, not past them.
    const ridgeLen = L * 0.9999
    const ridges: Array<{ x: number; y: number; z: number; cX: number; cY: number }> = [
      // +Y face: groove floor sits at y = cy/2 - sD. Ridge extends from
      // that floor up to y = cy/2 - sD + rH. Center at y = cy/2 - sD + rH/2.
      { x: rW, y: rH, z: ridgeLen, cX: 0,                       cY:  cy / 2 - sD + rH / 2 },
      { x: rW, y: rH, z: ridgeLen, cX: 0,                       cY: -cy / 2 + sD - rH / 2 },
      { x: rH, y: rW, z: ridgeLen, cX:  cx / 2 - sD + rH / 2,   cY: 0 },
      { x: rH, y: rW, z: ridgeLen, cX: -cx / 2 + sD - rH / 2,   cY: 0 },
    ]
    for (const r of ridges) {
      const ridgeShape = mkCenteredAt(r.x, r.y, r.z, r.cX, r.cY, 0)
      try {
        out = out.fuse(ridgeShape, { optimisation: 'none' })
      } catch (e) {
        const msg = e instanceof Error ? e.message : String(e)
        console.warn(`[bake/worker:extrusion] ridge fuse failed: ${msg} — continuing with remaining faces`)
      }
    }
    return out
  }
  // disc — parametric discs are built aligned with preset Z convention, so
  // no rotation override / centering (makeCylinder centers on the thickness
  // axis naturally in buildParametricDisc).
  return buildParametricDisc({
    kind: 'disc',
    odMm: part.odMm, idMm: part.idMm, thicknessMm: part.thicknessMm,
    connectorOriginMm: [0, 0, 0],
    connectorAxisXyz: [0, 0, 1],
  })
}

function applyRotThenTrans(
  shape: Shape3D,
  rotateRadXyz: [number, number, number],
  translateMm: [number, number, number],
): Shape3D {
  // THREE.Euler 'XYZ' decomposition yields angles (x, y, z) such that the
  // rotation matrix is M = Rx · Ry · Rz (applied to a vector as v' = M·v).
  // To reproduce this via sequential rotate() calls, we must apply Rz first,
  // then Ry, then Rx — because successive rotate(θ, axis) on a replicad shape
  // composes as v' = R_last · ... · R_first · v. Applying X first (my prior
  // code) gives Rz·Ry·Rx = Mᵀ for orthogonal rotations, i.e. the inverse —
  // which visibly flips compound-rotation children in the scene.
  const [rx, ry, rz] = rotateRadXyz
  let out = shape
  const RAD2DEG = 180 / Math.PI
  if (Math.abs(rz) > 1e-6) out = out.rotate(rz * RAD2DEG, [0, 0, 0], [0, 0, 1]) as Shape3D
  if (Math.abs(ry) > 1e-6) out = out.rotate(ry * RAD2DEG, [0, 0, 0], [0, 1, 0]) as Shape3D
  if (Math.abs(rx) > 1e-6) out = out.rotate(rx * RAD2DEG, [0, 0, 0], [1, 0, 0]) as Shape3D
  const [tx, ty, tz] = translateMm
  if (Math.abs(tx) > 1e-6 || Math.abs(ty) > 1e-6 || Math.abs(tz) > 1e-6) {
    out = out.translate(tx, ty, tz) as Shape3D
  }
  return out
}

async function bakeClusterOp(spec: BakeClusterSpec, opts: MeshOpts | undefined): Promise<BakeResponse> {
  const diagnostics: BakeDiagnostics = {
    importMs: 0, meshMs: 0, edgeMs: 0, triangleCount: 0, vertexCount: 0,
    fuseMs: 0, filletMs: 0,
    filletRadiusMm: 0, filletFellBackToFuseOnly: false,
  }
  try {
    await ensureOcct()
    if (spec.parts.length === 0) {
      return { kind: 'result', id: '', ok: false, phase: 'cluster-empty', message: 'no parts' }
    }
    if (spec.parts.length !== spec.joints.length + 1) {
      return {
        kind: 'result', id: '', ok: false, phase: 'cluster-shape',
        message: `mismatched parts/joints: parts=${spec.parts.length} joints=${spec.joints.length} (expected joints = parts-1)`,
      }
    }

    // Import + place each part in cluster-root frame.
    const tImp = performance.now()
    const placed: Shape3D[] = []
    for (let i = 0; i < spec.parts.length; i++) {
      const part = spec.parts[i]
      const raw = await loadClusterPart(part)
      // Root part gets no transform; subsequent parts are already given
      // transforms in the root's frame by the caller.
      const pose = i === 0
        ? raw
        : applyRotThenTrans(raw, part.rotateRadXyz, part.translateMm)
      placed.push(pose)
      console.log(`[bake/worker:cluster] part[${i}] kind=${part.kind} faces=${pose.faces.length} edges=${pose.edges.length}`)
    }
    diagnostics.importMs = performance.now() - tImp

    // Per-part mesh path: skip fuse+fillet, tessellate each placed part on
    // its own. Caller gets an array of meshes + can apply per-part colors.
    if (spec.perPartMeshes) {
      const tolerance = opts?.tolerance ?? 0.05
      const angularTolerance = opts?.angularTolerance ?? 20
      const tM = performance.now()
      const parts: PartMesh[] = []
      let triTotal = 0, vertTotal = 0
      for (let i = 0; i < placed.length; i++) {
        const shape = placed[i]
        const meshData = shape.mesh({ tolerance, angularTolerance })
        const edgeData = shape.meshEdges({ tolerance, angularTolerance })
        const positions = new Float32Array(meshData.vertices)
        const normals = new Float32Array(meshData.normals)
        const indices = new Uint32Array(meshData.triangles)
        const edgePositions = new Float32Array(edgeData.lines)
        triTotal += indices.length / 3
        vertTotal += positions.length / 3
        parts.push({
          partIdx: i,
          mesh: { positions, normals, indices },
          edges: { positions: edgePositions },
        })
      }
      diagnostics.meshMs = performance.now() - tM
      diagnostics.triangleCount = triTotal
      diagnostics.vertexCount = vertTotal
      console.log(`[bake/worker:cluster] perPartMeshes: ${placed.length} parts tessellated in ${diagnostics.meshMs.toFixed(0)}ms total tris=${triTotal}`)
      return { kind: 'result', id: '', ok: true, parts, diagnostics }
    }

    // Fold. Each step: fuse placed[0...i] with placed[i+1], then fillet at
    // joints[i]'s mate plane.
    let accum: Shape3D = placed[0]
    let anyFilletApplied = false
    let allFilletFailed = true

    for (let j = 0; j < spec.joints.length; j++) {
      const child = placed[j + 1]
      const joint = spec.joints[j]
      const label = joint.debugLabel ?? `joint[${j}]`

      if (joint.skipFuse) {
        console.log(`[bake/worker:cluster] ${label} skipFuse=true — leaving as sibling`)
        // We can't literally skip the fuse AND keep both parts in a single
        // Shape3D. For the Phase 3 fallback path, we still fuse but without
        // fillet; the mesh output shows the two as one solid even if OCCT
        // couldn't merge them. Keeps return type simple.
      }

      const tF = performance.now()
      let fused: Shape3D
      try {
        fused = accum.fuse(child, { optimisation: joint.fuseOptimisation ?? 'sameFace' })
      } catch (e) {
        const message = e instanceof Error ? `${e.name}: ${e.message}` : String(e)
        console.error(`[bake/worker:cluster] ${label} fuse failed after ${(performance.now()-tF).toFixed(0)}ms: ${message}`)
        // Couldn't fuse — abort whole cluster. Caller falls back to per-preset.
        return { kind: 'result', id: '', ok: false, phase: 'cluster-fuse', message: `${label}: ${message}`, diagnostics }
      }
      const fuseMs = performance.now() - tF
      diagnostics.fuseMs = (diagnostics.fuseMs ?? 0) + fuseMs
      console.log(`[bake/worker:cluster] ${label} fuse ${fuseMs.toFixed(0)}ms faces=${fused.faces.length} edges=${fused.edges.length}`)
      accum = fused

      // Fillet the mate ring. Box filter, shrink-and-retry.
      if (joint.filletRadiusMm > 0) {
        const [cx, cy, cz] = joint.planeCenterMm
        const slab = joint.slabMm ?? 0.15
        const lat = joint.filletBoxHalfSideMm
        const [ax, ay, az] = joint.axisXyz
        const nX = Math.abs(ax), nY = Math.abs(ay), nZ = Math.abs(az)
        // Figure out which axis to "slab" — the dominant axis component.
        const ext: [number, number, number] = [lat, lat, lat]
        if (nZ >= nX && nZ >= nY) ext[2] = slab
        else if (nY >= nX) ext[1] = slab
        else ext[0] = slab
        const corner1: [number, number, number] = [cx - ext[0], cy - ext[1], cz - ext[2]]
        const corner2: [number, number, number] = [cx + ext[0], cy + ext[1], cz + ext[2]]
        const radii = [joint.filletRadiusMm, joint.filletRadiusMm * 0.5, joint.filletRadiusMm * 0.25, joint.filletRadiusMm * 0.125]
        let filleted = false
        const tR = performance.now()
        for (const r of radii) {
          try {
            accum = accum.fillet(r, e => e.inBox(corner1, corner2)) as Shape3D
            diagnostics.filletMs = (diagnostics.filletMs ?? 0) + (performance.now() - tR)
            diagnostics.filletRadiusMm = r
            console.log(`[bake/worker:cluster] ${label} fillet r=${r.toFixed(3)}mm ok`)
            filleted = true
            anyFilletApplied = true
            break
          } catch (e) {
            const message = e instanceof Error ? `${e.name}: ${e.message}` : String(e)
            console.warn(`[bake/worker:cluster] ${label} fillet r=${r.toFixed(3)}mm failed: ${message}`)
          }
        }
        if (!filleted) {
          // Fillet rejected all radii. Fall back to CHAMFER — a flat bevel
          // is mathematically simpler than a curved fillet (OCCT's chamfer
          // algorithm tolerates geometry that rejects fillet), and reads
          // visually almost identical at bake-quality zoom. Same shrink
          // schedule as above.
          for (const r of radii) {
            try {
              accum = accum.chamfer(r, e => e.inBox(corner1, corner2)) as Shape3D
              diagnostics.filletMs = (diagnostics.filletMs ?? 0) + (performance.now() - tR)
              diagnostics.filletRadiusMm = r
              console.log(`[bake/worker:cluster] ${label} chamfer r=${r.toFixed(3)}mm ok (fillet fallback)`)
              filleted = true
              anyFilletApplied = true
              allFilletFailed = false
              break
            } catch (e) {
              const message = e instanceof Error ? `${e.name}: ${e.message}` : String(e)
              console.warn(`[bake/worker:cluster] ${label} chamfer r=${r.toFixed(3)}mm failed: ${message}`)
            }
          }
          if (!filleted) {
            console.warn(`[bake/worker:cluster] ${label} fillet AND chamfer failed — seam stays sharp`)
          }
        } else {
          allFilletFailed = false
        }
      } else {
        allFilletFailed = false // no fillet requested ≠ failed
      }
    }
    diagnostics.filletFellBackToFuseOnly = spec.joints.length > 0 && !anyFilletApplied && allFilletFailed

    const tolerance = opts?.tolerance ?? 0.05
    const angularTolerance = opts?.angularTolerance ?? 20
    const tM = performance.now()
    const meshData = accum.mesh({ tolerance, angularTolerance })
    const edgeData = accum.meshEdges({ tolerance, angularTolerance })
    diagnostics.meshMs = performance.now() - tM

    const positions = new Float32Array(meshData.vertices)
    const normals = new Float32Array(meshData.normals)
    const indices = new Uint32Array(meshData.triangles)
    const edgePositions = new Float32Array(edgeData.lines)
    diagnostics.triangleCount = indices.length / 3
    diagnostics.vertexCount = positions.length / 3

    const mesh: SerializedMesh = { positions, normals, indices }
    const edges: SerializedEdges = { positions: edgePositions }
    return { kind: 'result', id: '', ok: true, mesh, edges, diagnostics }
  } catch (e) {
    const message = e instanceof Error ? `${e.name}: ${e.message}` : String(e)
    return { kind: 'result', id: '', ok: false, phase: 'cluster-op', message, diagnostics }
  }
}

function transferablesFor(res: BakeResponse): Transferable[] {
  if (res.kind !== 'result' || !res.ok) return []
  const t: Transferable[] = []
  if ('parts' in res) {
    for (const p of res.parts) {
      t.push(p.mesh.positions.buffer, p.mesh.normals.buffer, p.mesh.indices.buffer)
      if (p.edges) t.push(p.edges.positions.buffer)
    }
    return t
  }
  t.push(res.mesh.positions.buffer, res.mesh.normals.buffer, res.mesh.indices.buffer)
  if (res.edges) t.push(res.edges.positions.buffer)
  return t
}

self.addEventListener('message', (ev: MessageEvent<BakeCommand>) => {
  const cmd = ev.data
  void (async () => {
    try {
      if (cmd.kind === 'bakeSingle') {
        const res = await bakeSinglePreset(cmd.stepUrl, cmd.opts)
        // Override id so response matches the request even when the caller
        // passes a synthetic id (multiple requests for the same STEP).
        const withId: BakeResponse = { ...res, id: cmd.id }
        self.postMessage(withId, transferablesFor(withId))
      } else if (cmd.kind === 'bakeFuseTwo') {
        const res = await bakeFuseTwo(cmd.fuse, cmd.opts)
        const withId: BakeResponse = { ...res, id: cmd.id }
        self.postMessage(withId, transferablesFor(withId))
      } else if (cmd.kind === 'bakeFuseParametric') {
        const res = await bakeFuseParametric(cmd.fuse, cmd.opts)
        const withId: BakeResponse = { ...res, id: cmd.id }
        self.postMessage(withId, transferablesFor(withId))
      } else if (cmd.kind === 'bakeCluster') {
        const res = await bakeClusterOp(cmd.cluster, cmd.opts)
        const withId: BakeResponse = { ...res, id: cmd.id }
        self.postMessage(withId, transferablesFor(withId))
      } else {
        const bad: BakeResponse = {
          kind: 'result', id: (cmd as { id?: string }).id ?? 'unknown', ok: false,
          phase: 'dispatch',
          message: `unsupported command kind: ${(cmd as { kind?: string }).kind ?? '<missing>'}`,
        }
        self.postMessage(bad)
      }
    } catch (e) {
      const message = e instanceof Error ? `${e.name}: ${e.message}` : String(e)
      self.postMessage({
        kind: 'result', id: cmd.id ?? 'unknown', ok: false,
        phase: 'dispatch', message,
      } satisfies BakeResponse)
    }
  })()
})

// Announce ready after the module body finishes evaluating. Lets the main
// thread stop early if the worker failed to construct.
self.postMessage({ kind: 'ready', id: 'init' } satisfies BakeResponse)

// Main-thread client for the B-Rep bake WebWorker.
//
// Phase 1 scope: spawn the worker, request a single-preset bake, return the
// meshed buffers. Phases 2+ extend with fuse/fillet and assembly bake commands.
//
// The worker is constructed lazily on the first bake call — this defers the
// 10 MB OCCT WASM download until the user actually asks for a bake. Callers
// that know a bake is imminent (e.g. accept-click handler after resolve)
// should call `primeBakeWorker()` to warm it up.

import type { BakeCommand, BakeResponse, MeshOpts, SerializedMesh, SerializedEdges, BakeDiagnostics, FuseSpec, FuseParamSpec, BakeClusterSpec, PartMesh } from './types.ts'

export interface BakeSingleResult {
  ok: true
  mesh: SerializedMesh
  edges?: SerializedEdges
  diagnostics: BakeDiagnostics
}
export interface BakeMultiPartResult {
  ok: true
  parts: PartMesh[]
  diagnostics: BakeDiagnostics
}
export interface BakeFailure {
  ok: false
  phase: string
  message: string
}
export type BakeOutcome = BakeSingleResult | BakeMultiPartResult | BakeFailure

type Pending = {
  resolve: (r: BakeOutcome) => void
}

let worker: Worker | null = null
let readyPromise: Promise<Worker> | null = null
const pending = new Map<string, Pending>()
let nextId = 1

function createWorker(): Promise<Worker> {
  if (readyPromise) return readyPromise
  readyPromise = new Promise<Worker>((resolve, reject) => {
    let w: Worker
    try {
      w = new Worker(
        new URL('./bakeWorker.ts', import.meta.url),
        { type: 'module', name: 'vector-bake' },
      )
    } catch (e) {
      reject(e)
      return
    }
    const bootTimeout = setTimeout(() => {
      reject(new Error('bake worker failed to signal ready within 15s'))
    }, 15_000)
    w.addEventListener('message', (ev: MessageEvent<BakeResponse>) => {
      const res = ev.data
      if (res.kind === 'ready') {
        clearTimeout(bootTimeout)
        worker = w
        resolve(w)
        return
      }
      if (res.kind === 'result') {
        const pend = pending.get(res.id)
        if (!pend) return
        pending.delete(res.id)
        if (res.ok) {
          if ('parts' in res) {
            pend.resolve({ ok: true, parts: res.parts, diagnostics: res.diagnostics })
          } else {
            pend.resolve({ ok: true, mesh: res.mesh, edges: res.edges, diagnostics: res.diagnostics })
          }
        } else {
          pend.resolve({ ok: false, phase: res.phase, message: res.message })
        }
      }
    })
    w.addEventListener('error', ev => {
      // Worker-level script error — reject the boot promise if we haven't
      // resolved yet, and reject every in-flight bake.
      const message = ev.message || 'worker error'
      clearTimeout(bootTimeout)
      reject(new Error(message))
      for (const [, p] of pending) p.resolve({ ok: false, phase: 'worker-error', message })
      pending.clear()
      worker = null
      readyPromise = null
    })
  })
  return readyPromise
}

/** Spawn the worker early. Call on app boot (or the first user-triggered
 *  event that hints a bake is coming) so the WASM download finishes before
 *  the user clicks Accept. No-op on subsequent calls. */
export function primeBakeWorker(): void {
  if (!worker && !readyPromise) {
    void createWorker().catch(e => {
      console.warn('[bake] worker prime failed:', e)
    })
  }
}

export interface BakeCallOpts extends MeshOpts {
  /** Abort the bake after this many ms by terminating + respawning the worker.
   *  Emscripten/OCCT computations can't be cancelled any other way — once the
   *  C++ loop starts, the only recourse is to kill the thread. Default 30s —
   *  well above the ~75ms bracket bake / ~2s warmup, below what a stuck worker
   *  should be allowed to consume. */
  timeoutMs?: number
}

const DEFAULT_TIMEOUT_MS = 30_000

export async function bakeSinglePreset(stepUrl: string, opts?: BakeCallOpts): Promise<BakeSingleResult | BakeFailure> {
  const timeoutMs = opts?.timeoutMs ?? DEFAULT_TIMEOUT_MS
  const w = await createWorker().catch(e => {
    return { ok: false as const, phase: 'boot', message: String(e) } satisfies BakeFailure
  })
  if (!(w instanceof Worker)) return w
  return await new Promise<BakeSingleResult | BakeFailure>(resolve => {
    const id = `bake-${nextId++}`
    const timer = setTimeout(() => {
      if (!pending.has(id)) return
      pending.delete(id)
      // OCCT compute is uncancellable — kill + respawn the worker so the next
      // bake call gets a fresh one. The 10 MB WASM re-downloads from cache, so
      // respawn is ~1s, not ~10s.
      console.warn(`[bake] bake-${id} timed out after ${timeoutMs}ms — terminating worker`)
      if (worker) {
        worker.terminate()
        worker = null
      }
      readyPromise = null
      for (const [, p] of pending) p.resolve({ ok: false, phase: 'timeout-peer', message: 'worker terminated by peer timeout' })
      pending.clear()
      resolve({ ok: false, phase: 'timeout', message: `bake exceeded ${timeoutMs}ms — STEP likely too complex or malformed` })
    }, timeoutMs)
    pending.set(id, {
      resolve: out => {
        clearTimeout(timer)
        resolve(out as BakeSingleResult | BakeFailure)
      },
    })
    const cmd: BakeCommand = {
      kind: 'bakeSingle', id, stepUrl,
      opts: { tolerance: opts?.tolerance, angularTolerance: opts?.angularTolerance },
    }
    w.postMessage(cmd)
  })
}

/** Fuse + fillet two STEP parts along their authored connectors and return
 *  the baked mesh. The mate convention matches the catalog: parent connector
 *  axis and child connector axis are antiparallel, child translates so its
 *  connector origin coincides with the parent's in world frame. */
export async function bakeFuseTwo(spec: FuseSpec, opts?: BakeCallOpts): Promise<BakeSingleResult | BakeFailure> {
  const timeoutMs = opts?.timeoutMs ?? DEFAULT_TIMEOUT_MS
  const w = await createWorker().catch(e => {
    return { ok: false as const, phase: 'boot', message: String(e) } satisfies BakeFailure
  })
  if (!(w instanceof Worker)) return w
  return await new Promise<BakeSingleResult | BakeFailure>(resolve => {
    const id = `fuse-${nextId++}`
    const timer = setTimeout(() => {
      if (!pending.has(id)) return
      pending.delete(id)
      console.warn(`[bake] ${id} timed out after ${timeoutMs}ms — terminating worker`)
      if (worker) {
        worker.terminate()
        worker = null
      }
      readyPromise = null
      for (const [, p] of pending) p.resolve({ ok: false, phase: 'timeout-peer', message: 'worker terminated by peer timeout' })
      pending.clear()
      resolve({ ok: false, phase: 'timeout', message: `fuse+fillet exceeded ${timeoutMs}ms` })
    }, timeoutMs)
    pending.set(id, {
      resolve: out => {
        clearTimeout(timer)
        resolve(out as BakeSingleResult | BakeFailure)
      },
    })
    const cmd: BakeCommand = {
      kind: 'bakeFuseTwo', id, fuse: spec,
      opts: { tolerance: opts?.tolerance, angularTolerance: opts?.angularTolerance },
    }
    w.postMessage(cmd)
  })
}

/** Fuse + fillet a STEP parent with a parametric disc child. Used when the
 *  authored STEP for a standard fitting (coupler, bearing, plate) is so
 *  over-modeled that `fuse()` times out. Per-preset bake-source overrides
 *  land in Phase 3; this is the first exit-hatch. */
export async function bakeFuseParametric(spec: FuseParamSpec, opts?: BakeCallOpts): Promise<BakeSingleResult | BakeFailure> {
  const timeoutMs = opts?.timeoutMs ?? DEFAULT_TIMEOUT_MS
  const w = await createWorker().catch(e => {
    return { ok: false as const, phase: 'boot', message: String(e) } satisfies BakeFailure
  })
  if (!(w instanceof Worker)) return w
  return await new Promise<BakeSingleResult | BakeFailure>(resolve => {
    const id = `fusep-${nextId++}`
    const timer = setTimeout(() => {
      if (!pending.has(id)) return
      pending.delete(id)
      console.warn(`[bake] ${id} timed out after ${timeoutMs}ms — terminating worker`)
      if (worker) {
        worker.terminate()
        worker = null
      }
      readyPromise = null
      for (const [, p] of pending) p.resolve({ ok: false, phase: 'timeout-peer', message: 'worker terminated by peer timeout' })
      pending.clear()
      resolve({ ok: false, phase: 'timeout', message: `fuse-parametric exceeded ${timeoutMs}ms` })
    }, timeoutMs)
    pending.set(id, {
      resolve: out => {
        clearTimeout(timer)
        resolve(out as BakeSingleResult | BakeFailure)
      },
    })
    const cmd: BakeCommand = {
      kind: 'bakeFuseParametric', id, fuse: spec,
      opts: { tolerance: opts?.tolerance, angularTolerance: opts?.angularTolerance },
    }
    w.postMessage(cmd)
  })
}

/** Bake a whole cluster (fixed-joined component chain) into a single mesh.
 *  See `BakeClusterSpec` for the shape. Worker does a left-fold fuse + per-
 *  joint fillet. Fails atomically: any fuse reject aborts the whole cluster. */
export async function bakeClusterRequest(spec: BakeClusterSpec, opts?: BakeCallOpts): Promise<BakeOutcome> {
  const timeoutMs = opts?.timeoutMs ?? DEFAULT_TIMEOUT_MS
  const w = await createWorker().catch(e => {
    return { ok: false as const, phase: 'boot', message: String(e) } satisfies BakeFailure
  })
  if (!(w instanceof Worker)) return w
  return await new Promise<BakeOutcome>(resolve => {
    const id = `cluster-${nextId++}`
    const timer = setTimeout(() => {
      if (!pending.has(id)) return
      pending.delete(id)
      console.warn(`[bake] ${id} timed out after ${timeoutMs}ms — terminating worker`)
      if (worker) {
        worker.terminate()
        worker = null
      }
      readyPromise = null
      for (const [, p] of pending) p.resolve({ ok: false, phase: 'timeout-peer', message: 'worker terminated by peer timeout' })
      pending.clear()
      resolve({ ok: false, phase: 'timeout', message: `cluster bake exceeded ${timeoutMs}ms` })
    }, timeoutMs)
    pending.set(id, {
      resolve: out => {
        clearTimeout(timer)
        resolve(out)
      },
    })
    const cmd: BakeCommand = {
      kind: 'bakeCluster', id, cluster: spec,
      opts: { tolerance: opts?.tolerance, angularTolerance: opts?.angularTolerance },
    }
    w.postMessage(cmd)
  })
}

/** Tear down the worker. For dev reload / testing. */
export function disposeBakeWorker(): void {
  if (worker) {
    worker.terminate()
    worker = null
  }
  readyPromise = null
  for (const [, p] of pending) p.resolve({ ok: false, phase: 'disposed', message: 'worker disposed' })
  pending.clear()
}

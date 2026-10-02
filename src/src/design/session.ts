/**
 * The editable design behind the URDF on screen.
 *
 * `openSession(urdf)` asks the core for the design (the embedded one when the
 * URDF is untouched designer output, otherwise one rebuilt from the URDF's
 * geometry) and compiles it once to learn every part's pose. Edits produce a
 * new design, `compileDesign` turns it into URDF, and that URDF replaces the
 * editor text — so the URDF is always exactly what the design says.
 */
import { invoke } from '@tauri-apps/api/core'
import type { CompiledDesign, Design, Mat3, Vec3 } from './types'
import { geomFromCompiled, mulV, sub, transpose, type PartGeom } from './math.ts'
import { findPart, mirrorName } from './edits.ts'
import { loadCatalog } from './catalog.ts'

export interface DesignSession {
  urdf: string
  design: Design
  compiled: CompiledDesign
  /** True when the design was rebuilt from the URDF (hand-written or hand-edited). */
  imported: boolean
  notes: string[]
  linkToPart: Map<string, string>
  geoms: Map<string, PartGeom>
  /** Viewer frame (URDF world = the root link frame) in design-world terms. */
  root: { R: Mat3; p: Vec3 }
}

export type CompileResult = CompiledDesign | { error: string }

export async function compileDesign(design: Design, check = false): Promise<CompileResult> {
  await loadCatalog()
  return invoke('design_compile', { design, check }) as Promise<CompileResult>
}

let cached: { urdf: string; promise: Promise<DesignSession> } | null = null

export function invalidateSession(): void {
  cached = null
}

export function openSession(urdf: string): Promise<DesignSession> {
  if (cached && cached.urdf === urdf) return cached.promise
  const promise = (async () => {
    await loadCatalog()
    const imp = await invoke('design_import', { urdfContent: urdf }) as
      { design: Design; notes: string[]; imported: boolean } | { error: string }
    if ('error' in imp) throw new Error(imp.error)
    const compiled = await compileDesign(imp.design)
    if ('error' in compiled) throw new Error(compiled.error)
    return buildSession(urdf, imp.design, compiled, imp.imported, imp.notes)
  })()
  cached = { urdf, promise }
  promise.catch(() => { if (cached?.promise === promise) cached = null })
  return promise
}

/** Seed the cache after a compile so the next openSession on that URDF is free. */
export function adoptCompiled(design: Design, compiled: CompiledDesign): DesignSession {
  const s = buildSession(compiled.urdf, design, compiled, false, [])
  cached = { urdf: compiled.urdf, promise: Promise.resolve(s) }
  return s
}

function buildSession(urdf: string, design: Design, compiled: CompiledDesign, imported: boolean, notes: string[]): DesignSession {
  const linkToPart = new Map<string, string>()
  const geoms = new Map<string, PartGeom>()
  for (const [name, cp] of Object.entries(compiled.parts)) {
    linkToPart.set(cp.link, name)
    let spec = findPart(design, name)
    if (!spec && cp.mirror_of) {
      const src = findPart(design, cp.mirror_of)
      if (src) spec = { ...src, name, mirror: false }
    }
    if (spec) geoms.set(name, geomFromCompiled(name, spec, cp))
  }
  const rp = compiled.parts[compiled.root]
  return {
    urdf, design, compiled, imported, notes, linkToPart, geoms,
    root: { R: rp.R, p: rp.frame_p },
  }
}

// ── design world <-> viewer (URDF world, metres) ────────────────────────────

export function designToViewerPoint(s: DesignSession, p: Vec3): Vec3 {
  const v = mulV(transpose(s.root.R), sub(p, s.root.p))
  return [v[0] / 1000, v[1] / 1000, v[2] / 1000]
}

export function designToViewerDir(s: DesignSession, d: Vec3): Vec3 {
  return mulV(transpose(s.root.R), d)
}

export function viewerToDesignDir(s: DesignSession, d: Vec3): Vec3 {
  return mulV(s.root.R, d)
}

export function viewerToDesignPoint(s: DesignSession, p: Vec3): Vec3 {
  const d = mulV(s.root.R, [p[0] * 1000, p[1] * 1000, p[2] * 1000])
  return [d[0] + s.root.p[0], d[1] + s.root.p[1], d[2] + s.root.p[2]]
}

export { mirrorName }

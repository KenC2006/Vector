/**
 * Pure edits of a design (no I/O). Each returns a new design; the caller
 * compiles it (session.ts) and the compiler reports anything invalid.
 */
import type { AtSpec, Design, DesignPart, JointSpec, Vec3 } from './types'

export function cloneDesign(d: Design): Design {
  return JSON.parse(JSON.stringify(d)) as Design
}

// ── mirror naming (port of core/designer/geometry.mirror_name) ─────────────

const MIRROR_TOKENS: Record<string, string> = {
  left: 'right', l: 'r', fl: 'fr', rl: 'rr', ml: 'mr', bl: 'br', lf: 'rf', lr: 'rr', lh: 'rh',
}
for (const [k, v] of Object.entries({ ...MIRROR_TOKENS })) MIRROR_TOKENS[v] = k
MIRROR_TOKENS.rr = 'rl'

export function mirrorName(name: string): string {
  const tokens = name.split('_')
  for (let i = 0; i < tokens.length; i++) {
    const t = tokens[i]
    const swap = MIRROR_TOKENS[t.toLowerCase()]
    if (swap) {
      tokens[i] = t[0] && t[0] === t[0].toUpperCase() && t[0] !== t[0].toLowerCase()
        ? swap[0].toUpperCase() + swap.slice(1) : swap
      return tokens.join('_')
    }
    for (const [a, b] of [['Left', 'Right'], ['Right', 'Left'], ['left', 'right'], ['right', 'left']]) {
      if (t.includes(a)) {
        tokens[i] = t.replace(a, b)
        return tokens.join('_')
      }
    }
  }
  return `${name}_mirror`
}

/** Every part name in the compiled robot, including mirrored copies. */
export function allPartNames(d: Design): Set<string> {
  const out = new Set<string>()
  for (const p of d.parts) {
    out.add(p.name)
    if (p.mirror) out.add(mirrorName(p.name))
  }
  return out
}

export function uniqueName(d: Design, base: string): string {
  const clean = base.replace(/[^A-Za-z0-9_]/g, '_').replace(/^[^A-Za-z]+/, '') || 'part'
  const used = allPartNames(d)
  let n = 1
  while (used.has(`${clean}_${n}`) || used.has(mirrorName(`${clean}_${n}`))) n++
  return `${clean}_${n}`
}

// ── references ──────────────────────────────────────────────────────────────

function refPart(ref: string): string {
  const i = ref.indexOf('.')
  return i < 0 ? ref : ref.slice(0, i)
}

function atRef(at: AtSpec | undefined): string | null {
  if (at === undefined || Array.isArray(at)) return null
  return typeof at === 'string' ? at : at.ref
}

/** Names of the parts a spec refers to (parent, `at`, joint pivot/axis). */
export function referencedParts(p: DesignPart): Set<string> {
  const out = new Set<string>()
  if (p.parent) out.add(p.parent)
  const a = atRef(p.at)
  if (a) out.add(refPart(a))
  const j = typeof p.joint === 'object' ? p.joint : null
  if (j) {
    const pv = atRef(j.pivot)
    if (pv) out.add(refPart(pv))
    if (typeof j.axis === 'string' && j.axis.includes('.')) out.add(refPart(j.axis))
  }
  return out
}

/** A part plus everything that (transitively) depends on it. */
export function dependents(d: Design, name: string): string[] {
  const out = [name]
  const seen = new Set(out)
  let grew = true
  while (grew) {
    grew = false
    for (const p of d.parts) {
      if (seen.has(p.name)) continue
      for (const r of referencedParts(p)) {
        if (seen.has(r)) { seen.add(p.name); out.push(p.name); grew = true; break }
      }
    }
  }
  return out
}

export function findPart(d: Design, name: string): DesignPart | undefined {
  return d.parts.find(p => p.name === name)
}

/** The authored part behind a compiled part (mirrored copies map to their source). */
export function sourcePartName(d: Design, name: string): string {
  if (findPart(d, name)) return name
  const src = d.parts.find(p => p.mirror && mirrorName(p.name) === name)
  return src ? src.name : name
}

// ── edits ───────────────────────────────────────────────────────────────────

export function addPart(d: Design, part: DesignPart): Design {
  const out = cloneDesign(d)
  out.parts.push(part)
  return out
}

export function updatePart(d: Design, name: string, patch: Partial<DesignPart>): Design {
  const out = cloneDesign(d)
  const p = findPart(out, name)
  if (!p) return out
  for (const [k, v] of Object.entries(patch)) {
    if (v === undefined) delete (p as unknown as Record<string, unknown>)[k]
    else (p as unknown as Record<string, unknown>)[k] = v
  }
  return out
}

export function replacePart(d: Design, name: string, part: DesignPart): Design {
  const out = cloneDesign(d)
  const i = out.parts.findIndex(p => p.name === name)
  if (i >= 0) out.parts[i] = part
  return out
}

/** Remove a part and everything that depends on it. */
export function removePart(d: Design, name: string): { design: Design; removed: string[] } {
  const removed = dependents(d, name)
  const gone = new Set(removed)
  const out = cloneDesign(d)
  out.parts = out.parts.filter(p => !gone.has(p.name))
  return { design: out, removed }
}

function renameRef(ref: string, from: string, to: string): string {
  const i = ref.indexOf('.')
  const head = i < 0 ? ref : ref.slice(0, i)
  return head === from ? to + (i < 0 ? '' : ref.slice(i)) : ref
}

function renameAt(at: AtSpec | undefined, from: string, to: string): AtSpec | undefined {
  if (at === undefined || Array.isArray(at)) return at
  if (typeof at === 'string') return renameRef(at, from, to)
  return { ...at, ref: renameRef(at.ref, from, to) }
}

export function renamePart(d: Design, from: string, to: string): Design {
  const out = cloneDesign(d)
  for (const p of out.parts) {
    if (p.name === from) p.name = to
    if (p.parent === from) p.parent = to
    p.at = renameAt(p.at, from, to)
    if (typeof p.joint === 'object') {
      const j = p.joint as JointSpec
      j.pivot = renameAt(j.pivot, from, to)
      if (typeof j.axis === 'string' && j.axis.includes('.')) j.axis = renameRef(j.axis, from, to)
    }
  }
  return out
}

/** `part.anchor` -> the parts mounted on it (via `at`). */
export function anchorOccupancy(d: Design): Map<string, string[]> {
  const out = new Map<string, string[]>()
  for (const p of d.parts) {
    const a = atRef(p.at)
    if (!a || !a.includes('.')) continue
    const list = out.get(a) ?? []
    list.push(p.name)
    out.set(a, list)
  }
  return out
}

/** `at` normalised to {ref, offset} or a point. */
export function atParts(at: AtSpec | undefined): { ref: string | null; offset: Vec3; point: Vec3 | null } {
  if (at === undefined) return { ref: null, offset: [0, 0, 0], point: [0, 0, 0] }
  if (Array.isArray(at)) return { ref: null, offset: [0, 0, 0], point: [...at] as Vec3 }
  if (typeof at === 'string') return { ref: at, offset: [0, 0, 0], point: null }
  return { ref: at.ref, offset: at.offset ? [...at.offset] as Vec3 : [0, 0, 0], point: null }
}

/** Move a part by a world-frame delta (mm) without changing what it's attached to. */
export function translateAt(at: AtSpec | undefined, delta: Vec3): AtSpec {
  const a = atParts(at)
  const r = (v: number) => Math.round(v * 1000) / 1000
  if (a.point) return [r(a.point[0] + delta[0]), r(a.point[1] + delta[1]), r(a.point[2] + delta[2])]
  const off: Vec3 = [r(a.offset[0] + delta[0]), r(a.offset[1] + delta[1]), r(a.offset[2] + delta[2])]
  return off.every(v => v === 0) ? a.ref! : { ref: a.ref!, offset: off }
}

export function jointOf(p: DesignPart): JointSpec {
  if (!p.joint) return { type: 'fixed' }
  return typeof p.joint === 'string' ? { type: p.joint } : p.joint
}

/** Stable reorder so every part comes after the parts it references (the
 *  compiler resolves references in order). Throws on a reference cycle. */
export function orderParts(d: Design): Design {
  const out = cloneDesign(d)
  const byName = new Map(out.parts.map(p => [p.name, p]))
  const sourceOf = (n: string) => byName.has(n) ? n : (out.parts.find(p => p.mirror && mirrorName(p.name) === n)?.name ?? n)
  const placed = new Set<string>()
  const result: DesignPart[] = []
  const visiting = new Set<string>()
  const visit = (p: DesignPart) => {
    if (placed.has(p.name)) return
    if (visiting.has(p.name)) throw new Error(`${p.name}: circular attachment`)
    visiting.add(p.name)
    for (const r of referencedParts(p)) {
      const dep = byName.get(sourceOf(r))
      if (dep && dep !== p) visit(dep)
    }
    visiting.delete(p.name)
    placed.add(p.name)
    result.push(p)
  }
  for (const p of out.parts) visit(p)
  out.parts = result
  return out
}

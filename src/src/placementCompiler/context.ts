// Compilation-scoped context for the placement compiler.
//
// Lets us thread a single graph-level flag (archetype mode) into deeply-
// nested placement helpers without changing every signature. The alternative
// — adding `archetypeMode?: string` to ten function signatures (multiChild,
// face, mate, …) — produced a noisy diff for what is logically one bit of
// state per compile.
//
// Lifecycle: callers MUST set the mode at the start of compilation and clear
// it at the end. Reading stale state from a previous compile would silently
// corrupt the next one. The setter+resetter pair in urdfAssembly is the
// only authorized usage.
//
// Scope is intentionally narrow — only flags that gate "should the
// archetype-template layout fire?" belong here. General compilation context
// (presets, resolver, etc.) is still threaded as parameters because it's
// per-call data, not per-graph.

let _archetypeMode: 'standard' | 'novel' | null = null

export function setArchetypeMode(mode: 'standard' | 'novel' | null | undefined): void {
  _archetypeMode = mode === 'standard' || mode === 'novel' ? mode : null
}

export function getArchetypeMode(): 'standard' | 'novel' | null {
  return _archetypeMode
}

export function isNovelMode(): boolean {
  return _archetypeMode === 'novel'
}

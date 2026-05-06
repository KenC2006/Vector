// Phase 7 author tooling: per-component mesh-rotation triage inspector.
//
// Sits on top of propose-mesh-rotations.mjs. For every baselined component it
// shows the current state of the ROTATION_OVERRIDES / EXPLICIT_SCALE_POLICY
// registries against the proposed fix, and tracks per-component decisions in
// scripts/mesh-rotation-decisions.json so the same item is not re-triaged
// every run.
//
// Without this, executing MESH_BBOX_PARITY_FIX.md means staring at a 200-line
// proposals dump and hand-editing meshOverrides.ts for every component — easy
// to lose your place, easy to apply the wrong rpy, no audit trail.
//
// Read-only by default. Writes meshOverrides.ts only when --write is passed.
//
// Usage:
//   cd src && node ../scripts/triage-mesh-rotations.mjs
//       Print summary of pending / applied / deferred / shape-mismatch.
//   cd src && node ../scripts/triage-mesh-rotations.mjs --pending
//       Show only items needing a decision.
//   cd src && node ../scripts/triage-mesh-rotations.mjs --apply <id> [--write]
//       Apply rotation_only proposal for one component. --write edits the file;
//       without it prints the diff for review.
//   cd src && node ../scripts/triage-mesh-rotations.mjs --apply-all-rotate [--write]
//       Apply rotation_only proposal for every PARTIAL/ROTATE bucket item that
//       does not already have an override.
//   cd src && node ../scripts/triage-mesh-rotations.mjs --defer <id> --reason "..."
//       Record a decision to leave this component as-is (e.g. shared-GLB,
//       waiting on re-author). Future runs filter it out of the pending list.
//   cd src && node ../scripts/triage-mesh-rotations.mjs --status <id>
//       Inspect one component in detail.

import fs from 'node:fs/promises'
import { existsSync } from 'node:fs'
import path from 'node:path'
import { spawnSync } from 'node:child_process'
import { fileURLToPath } from 'node:url'

const here = path.dirname(fileURLToPath(import.meta.url))
const repoRoot = path.resolve(here, '..')
const PROPOSALS_SCRIPT = path.join(here, 'propose-mesh-rotations.mjs')
const OVERRIDES_FILE = path.join(repoRoot, 'src', 'src', 'richVisuals', 'meshOverrides.ts')
const DECISIONS_FILE = path.join(here, 'mesh-rotation-decisions.json')

function parseArgs(argv) {
  const args = { mode: 'summary', write: false, id: null, reason: null }
  for (let i = 2; i < argv.length; i++) {
    const a = argv[i]
    if (a === '--pending') args.mode = 'pending'
    else if (a === '--apply') { args.mode = 'apply'; args.id = argv[++i] }
    else if (a === '--apply-all-rotate') args.mode = 'apply-all-rotate'
    else if (a === '--defer') { args.mode = 'defer'; args.id = argv[++i] }
    else if (a === '--status') { args.mode = 'status'; args.id = argv[++i] }
    else if (a === '--reason') args.reason = argv[++i]
    else if (a === '--write') args.write = true
    else if (a === '--help' || a === '-h') { args.mode = 'help' }
    else { console.error(`unknown arg: ${a}`); process.exit(2) }
  }
  return args
}

async function loadProposals() {
  const r = spawnSync(process.execPath, [PROPOSALS_SCRIPT, '--json'], {
    encoding: 'utf8', cwd: path.join(repoRoot, 'src'),
  })
  if (r.status !== 0) {
    console.error('propose-mesh-rotations.mjs failed:')
    console.error(r.stderr || r.stdout)
    process.exit(1)
  }
  return JSON.parse(r.stdout)
}

async function loadDecisions() {
  if (!existsSync(DECISIONS_FILE)) return { decisions: {} }
  return JSON.parse(await fs.readFile(DECISIONS_FILE, 'utf8'))
}

async function saveDecisions(d) {
  await fs.writeFile(DECISIONS_FILE, JSON.stringify(d, null, 2) + '\n', 'utf8')
}

function parseRegistry(source, name) {
  // Find `export const NAME: ... = {` block, return Map of id → raw line.
  const re = new RegExp(`export const ${name}\\b[^=]*=\\s*\\{`, 'm')
  const m = re.exec(source)
  if (!m) return { ids: new Set(), open: -1, close: -1 }
  const open = m.index + m[0].length
  let depth = 1, i = open
  while (i < source.length && depth > 0) {
    const c = source[i]
    if (c === '{') depth++
    else if (c === '}') depth--
    i++
  }
  const close = i - 1
  const body = source.slice(open, close)
  const ids = new Set()
  const idRe = /['"]([a-z0-9_]+)['"]\s*:/gi
  for (const idm of body.matchAll(idRe)) ids.add(idm[1])
  return { ids, open, close }
}

async function loadOverrideState() {
  const src = await fs.readFile(OVERRIDES_FILE, 'utf8')
  return {
    src,
    rotation: parseRegistry(src, 'ROTATION_OVERRIDES'),
    scale: parseRegistry(src, 'EXPLICIT_SCALE_POLICY'),
  }
}

function classify(p, state, decisions) {
  const hasRotation = state.rotation.ids.has(p.id)
  const hasScale = state.scale.ids.has(p.id)
  const decided = decisions.decisions[p.id]
  if (decided?.action === 'defer') return 'DEFERRED'
  if (p.status === 'NO-DATA') return 'NO-DATA'
  if (hasRotation || hasScale) return 'APPLIED'
  if (p.status === 'SHAPE-MISMATCH') return 'SHAPE-MISMATCH'
  if (p.status === 'NEAR') return 'NEAR'
  return 'PENDING'
}

function fmtBucket(label, items) {
  if (!items.length) return ''
  const lines = [`── ${label} (${items.length}) ──`]
  for (const p of items) {
    const tag = p.shared_with?.length ? ` [SHARED:${p.shared_with.length}]` : ''
    const r = p.rotation_only
    const rs = p.rotation_plus_scale
    const fix = r ? `rpy ${r.rpy} → ${(r.divergence*100).toFixed(1)}%` : ''
    const fix2 = rs ? `  | +scale ${rs.scale} → ${(rs.divergence*100).toFixed(1)}%` : ''
    lines.push(`  ${p.id}${tag}`)
    if (fix) lines.push(`     ${fix}${fix2}`)
  }
  return lines.join('\n') + '\n'
}

function injectIntoRegistry(source, name, snippet) {
  const reg = parseRegistry(source, name)
  if (reg.close < 0) throw new Error(`registry ${name} not found in meshOverrides.ts`)
  // Insert snippet just before the closing brace, preserving indentation.
  const before = source.slice(0, reg.close)
  const after = source.slice(reg.close)
  // Trim trailing whitespace/newlines from `before` so we don't accumulate blanks.
  const trimmed = before.replace(/\s*$/, '\n')
  return trimmed + snippet + after
}

function applyRotationSnippet(p, mode) {
  // mode: 'rotation_only' | 'rotation_plus_scale'
  const r = mode === 'rotation_only' ? p.rotation_only : p.rotation_plus_scale
  const tag = mode === 'rotation_only'
    ? `rotation_only ${r.perm}, ${(r.divergence*100).toFixed(1)}%`
    : `rotation+uniform-scale ${r.perm}, ${(r.divergence*100).toFixed(1)}%`
  return [
    `  // Auto-applied via triage-mesh-rotations (${tag}).`,
    `  // preset ${JSON.stringify(p.bbox_mm)} vs GLB ${JSON.stringify(p.raw_extent_mm)}`,
    `  '${p.id}': ${r.rpy},`,
    '',
  ].join('\n')
}

function applyScaleSnippet(p) {
  const rs = p.rotation_plus_scale
  return [
    `  // Auto-applied via triage-mesh-rotations (uniform scale ${rs.scale} → ${(rs.divergence*100).toFixed(1)}%).`,
    `  '${p.id}': 'uniform',`,
    '',
  ].join('\n')
}

async function cmdSummary(proposals, state, decisions, opts) {
  const buckets = { PENDING: [], NEAR: [], APPLIED: [], DEFERRED: [], 'SHAPE-MISMATCH': [], 'NO-DATA': [] }
  for (const p of proposals) buckets[classify(p, state, decisions)].push(p)
  console.log(`Mesh-rotation triage  —  ${proposals.length} baselined components\n`)
  console.log(`  PENDING        ${buckets.PENDING.length}   ≤5% — auto-applyable`)
  console.log(`  NEAR           ${buckets.NEAR.length}   5–10% — needs visual review then --apply or --defer`)
  console.log(`  APPLIED        ${buckets.APPLIED.length}   already in meshOverrides.ts`)
  console.log(`  DEFERRED       ${buckets.DEFERRED.length}   marked for later (decisions file)`)
  console.log(`  SHAPE-MISMATCH ${buckets['SHAPE-MISMATCH'].length}   >10% — re-author needed`)
  if (buckets['NO-DATA'].length) console.log(`  NO-DATA        ${buckets['NO-DATA'].length}`)
  console.log()
  if (opts.mode === 'pending') {
    process.stdout.write(fmtBucket('PENDING', buckets.PENDING))
    process.stdout.write(fmtBucket('NEAR', buckets.NEAR))
  } else {
    for (const k of ['PENDING', 'NEAR', 'SHAPE-MISMATCH', 'DEFERRED', 'APPLIED']) {
      process.stdout.write(fmtBucket(k, buckets[k]))
    }
  }
}

async function cmdStatus(proposals, state, decisions, opts) {
  const p = proposals.find(x => x.id === opts.id)
  if (!p) { console.error(`unknown id: ${opts.id}`); process.exit(2) }
  const cls = classify(p, state, decisions)
  console.log(`${p.id}  [${cls}]`)
  console.log(`  visual_file ${p.visual_file}`)
  console.log(`  bbox_mm     ${JSON.stringify(p.bbox_mm)}`)
  console.log(`  raw_extent  ${JSON.stringify(p.raw_extent_mm)}`)
  console.log(`  baseline_div ${(p.baseline_div*100).toFixed(1)}%`)
  if (p.rotation_only) {
    const r = p.rotation_only
    console.log(`  rotation_only  ${r.perm}  rpy ${r.rpy}  → ${(r.divergence*100).toFixed(1)}%`)
  }
  if (p.rotation_plus_scale) {
    const rs = p.rotation_plus_scale
    console.log(`  rotation+scale ${rs.perm}  scale ${rs.scale}  → ${(rs.divergence*100).toFixed(1)}%`)
  }
  if (p.shared_with?.length) console.log(`  SHARED GLB with: ${p.shared_with.join(', ')}`)
  if (state.rotation.ids.has(p.id)) console.log(`  → already in ROTATION_OVERRIDES`)
  if (state.scale.ids.has(p.id)) console.log(`  → already in EXPLICIT_SCALE_POLICY`)
  const decided = decisions.decisions[p.id]
  if (decided) console.log(`  → decision: ${decided.action} (${decided.reason || 'no reason'}) at ${decided.at}`)
}

async function cmdApply(proposals, state, opts) {
  const p = proposals.find(x => x.id === opts.id)
  if (!p) { console.error(`unknown id: ${opts.id}`); process.exit(2) }
  if (state.rotation.ids.has(p.id)) {
    console.error(`${p.id}: already in ROTATION_OVERRIDES — edit by hand if you need to change it.`)
    process.exit(1)
  }
  if (!p.rotation_only) { console.error(`${p.id}: no rotation proposal available.`); process.exit(1) }

  // Pick best applicable strategy:
  //   rotation_only ≤ 5%       → just write ROTATION_OVERRIDES
  //   rotation+scale ≤ 10%     → write ROTATION_OVERRIDES + EXPLICIT_SCALE_POLICY:'uniform'
  //   otherwise                → refuse (SHAPE-MISMATCH; re-author)
  let strategy
  if (p.rotation_only.divergence <= 0.05) strategy = 'rotation_only'
  else if (p.rotation_plus_scale && p.rotation_plus_scale.divergence <= 0.10) strategy = 'rotation_plus_scale'
  else {
    console.error(`${p.id}: SHAPE-MISMATCH — rotation_only ${(p.rotation_only.divergence*100).toFixed(1)}%, rotation+scale ${(p.rotation_plus_scale?.divergence*100 || Infinity).toFixed(1)}%; refusing.`)
    console.error(`  Try: --defer ${p.id} --reason "..."  to mark this for re-author backlog.`)
    process.exit(1)
  }

  const rotSnip = applyRotationSnippet(p, strategy)
  console.log('--- proposed insertion into ROTATION_OVERRIDES ---')
  process.stdout.write(rotSnip)
  let scaleSnip = ''
  if (strategy === 'rotation_plus_scale') {
    scaleSnip = applyScaleSnippet(p)
    console.log('--- proposed insertion into EXPLICIT_SCALE_POLICY ---')
    process.stdout.write(scaleSnip)
  }
  console.log('--------------------------------------------------')
  if (!opts.write) {
    console.log('(dry-run — re-run with --write to apply)')
    return
  }
  let src = injectIntoRegistry(state.src, 'ROTATION_OVERRIDES', rotSnip)
  if (scaleSnip) src = injectIntoRegistry(src, 'EXPLICIT_SCALE_POLICY', scaleSnip)
  await fs.writeFile(OVERRIDES_FILE, src, 'utf8')
  console.log(`wrote ${OVERRIDES_FILE}`)
}

async function cmdApplyAllRotate(proposals, state, opts) {
  const candidates = proposals.filter(p => {
    if (state.rotation.ids.has(p.id)) return false
    if (p.status !== 'ROTATE') return false
    if (!p.rotation_only || p.rotation_only.divergence > 0.05) return false
    return true
  })
  if (!candidates.length) { console.log('no ROTATE-bucket components pending.'); return }
  console.log(`Will apply rotation_only override for ${candidates.length} components:\n`)
  let blob = ''
  for (const p of candidates) {
    console.log(`  ${p.id}  ${p.rotation_only.rpy}  (${(p.rotation_only.divergence*100).toFixed(1)}%)`)
    blob += applyRotationSnippet(p)
  }
  console.log()
  if (!opts.write) {
    console.log('(dry-run — re-run with --write to apply)')
    return
  }
  const updated = injectIntoRegistry(state.src, 'ROTATION_OVERRIDES', blob)
  await fs.writeFile(OVERRIDES_FILE, updated, 'utf8')
  console.log(`wrote ${OVERRIDES_FILE}  (${candidates.length} entries)`)
}

async function cmdDefer(proposals, decisions, opts) {
  const p = proposals.find(x => x.id === opts.id)
  if (!p) { console.error(`unknown id: ${opts.id}`); process.exit(2) }
  if (!opts.reason) { console.error('--defer requires --reason "<text>"'); process.exit(2) }
  decisions.decisions[p.id] = { action: 'defer', reason: opts.reason, at: new Date().toISOString() }
  await saveDecisions(decisions)
  console.log(`deferred ${p.id}: ${opts.reason}`)
}

function help() {
  console.log(`triage-mesh-rotations  —  Phase 7 author tooling

  --pending                       only pending items
  --status <id>                   inspect one component
  --apply <id> [--write]          apply rotation_only proposal
  --apply-all-rotate [--write]    apply all ROTATE-bucket proposals
  --defer <id> --reason "..."     record decision to leave as-is
  (no flags)                      summary report
`)
}

async function main() {
  const opts = parseArgs(process.argv)
  if (opts.mode === 'help') { help(); return }
  const proposals = await loadProposals()
  const state = await loadOverrideState()
  const decisions = await loadDecisions()
  switch (opts.mode) {
    case 'summary':
    case 'pending':       return cmdSummary(proposals, state, decisions, opts)
    case 'status':        return cmdStatus(proposals, state, decisions, opts)
    case 'apply':         return cmdApply(proposals, state, opts)
    case 'apply-all-rotate': return cmdApplyAllRotate(proposals, state, opts)
    case 'defer':         return cmdDefer(proposals, decisions, opts)
    default: help()
  }
}

main().catch(e => { console.error('fatal:', e.stack || e.message); process.exit(1) })

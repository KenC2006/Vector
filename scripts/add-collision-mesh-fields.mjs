// One-shot data migration: add `collision_mesh` field to every preset in
// generic_presets.json that has a mesh override. Keeps the existing
// hand-formatted JSON layout — does targeted text insertion instead of
// parse → stringify (which would clobber single-line connector entries).
//
// Re-running is idempotent: skips presets that already have a
// collision_mesh entry, and skips the OBJ-existence check per preset
// based on the {base}_collision.obj derived from meshOverrides.
//
// Usage:
//   node scripts/add-collision-mesh-fields.mjs
//
// Source of truth for the preset_id → STEP mapping is meshOverrides in
// src/src/richVisuals/visualOverrides.json. The collision OBJ filename is
// derived as {strip-step-extension}_collision.obj.

import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const here = path.dirname(fileURLToPath(import.meta.url))
const repoRoot = path.resolve(here, '..')
const PRESET_JSON = path.join(repoRoot, 'core', 'presets', 'generic_presets.json')
const VISUAL_OVERRIDES_JSON = path.join(repoRoot, 'src', 'src', 'richVisuals', 'visualOverrides.json')
const COLLISION_DIR = path.join(repoRoot, 'src', 'public', 'meshes', 'collision')

function collisionFromStep(stepName) {
  return stepName.replace(/\.(step|stp)$/i, '') + '_collision.obj'
}

async function main() {
  const presetText = await fs.readFile(PRESET_JSON, 'utf8')
  const overrides = JSON.parse(await fs.readFile(VISUAL_OVERRIDES_JSON, 'utf8')).meshOverrides

  const presetIds = Object.keys(overrides).sort()
  console.log(`meshOverrides has ${presetIds.length} entries`)

  const inserts = []
  const skipped = []

  for (const pid of presetIds) {
    const stepName = overrides[pid]
    const collisionName = collisionFromStep(stepName)
    const collisionPath = path.join(COLLISION_DIR, collisionName)
    try {
      await fs.access(collisionPath)
    } catch {
      skipped.push({ pid, reason: `missing ${collisionName}` })
      continue
    }
    inserts.push({ pid, collisionName })
  }

  let edited = presetText
  let added = 0
  let already = 0
  let notFound = 0

  for (const { pid, collisionName } of inserts) {
    // Locate the component block for this id and inject collision_mesh
    // as the last field inside its "physical": { ... } object.
    const idAnchor = `"id": "${pid}"`
    const idIdx = edited.indexOf(idAnchor)
    if (idIdx === -1) {
      notFound++
      console.warn(`  preset id not in JSON: ${pid}`)
      continue
    }

    // Find the "physical" block that follows this id (within ~3KB,
    // bounded by the next top-level component).
    const after = edited.slice(idIdx)
    const physMatch = after.match(/"physical"\s*:\s*\{/)
    if (!physMatch) {
      notFound++
      console.warn(`  no "physical" after ${pid}`)
      continue
    }
    const physOpenAbs = idIdx + physMatch.index + physMatch[0].length
    // Walk to matching close brace
    let depth = 1
    let j = physOpenAbs
    while (j < edited.length && depth > 0) {
      const c = edited[j]
      if (c === '{') depth++
      else if (c === '}') depth--
      if (depth === 0) break
      j++
    }
    if (depth !== 0) {
      notFound++
      console.warn(`  unbalanced "physical" for ${pid}`)
      continue
    }
    const physCloseAbs = j // points at matching '}'
    const block = edited.slice(physOpenAbs, physCloseAbs)

    if (block.includes('"collision_mesh"')) {
      already++
      continue
    }

    // Determine indent from the line containing the closing brace —
    // typically the field indent is +2 spaces from the brace's indent.
    const lineStart = edited.lastIndexOf('\n', physCloseAbs) + 1
    const closeIndent = edited.slice(lineStart, physCloseAbs).match(/^(\s*)/)[1]
    const fieldIndent = closeIndent + '  '

    // Trim trailing whitespace before close brace, then insert
    // ",\n<fieldIndent>"collision_mesh": "..."\n<closeIndent>".
    // The block ends like:  …\n<fieldIndent>"inertia_primitive": "box"\n<closeIndent>
    // We need to add a comma to the previous final field and append our line.
    const trimmedBlock = block.replace(/\s+$/, '')
    const newBlock =
      trimmedBlock +
      ',\n' + fieldIndent + `"collision_mesh": "${collisionName}"` +
      '\n' + closeIndent

    edited = edited.slice(0, physOpenAbs) + newBlock + edited.slice(physCloseAbs)
    added++
  }

  // Sanity: round-trip parse to verify validity.
  try {
    JSON.parse(edited)
  } catch (e) {
    throw new Error(`edited JSON failed to parse: ${e.message}`)
  }

  await fs.writeFile(PRESET_JSON, edited, 'utf8')

  console.log('')
  console.log(`Added collision_mesh: ${added}`)
  console.log(`Already present:     ${already}`)
  console.log(`Preset id not found: ${notFound}`)
  console.log(`Skipped (no OBJ):    ${skipped.length}`)
  for (const s of skipped) console.log(`  ${s.pid}: ${s.reason}`)
}

main().catch(e => {
  console.error('fatal:', e.stack || e.message)
  process.exit(1)
})

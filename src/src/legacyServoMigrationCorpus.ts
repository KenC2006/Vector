// Fixture test for docs/SERVO_SPLIT_PLAN.md §legacy-migration. Runs
// migrateLegacyServos against fixtures/legacy_servos/dog2.urdf and asserts
// the DOF-preserving output contract.
//
// Run: cd src && node --experimental-strip-types src/legacyServoMigrationCorpus.ts

import * as fs from 'node:fs'
import * as path from 'node:path'
import { fileURLToPath } from 'node:url'
import { DOMParser, XMLSerializer } from '@xmldom/xmldom'
import { migrateLegacyServos } from './legacyServoMigration.ts'
import type { LegacyServoMigrationPreset } from './legacyServoMigration.ts'

const here = path.dirname(fileURLToPath(import.meta.url))
const fixturePath = path.resolve(here, '..', '..', 'fixtures', 'legacy_servos', 'dog2.urdf')
const presetsPath = path.resolve(here, '..', 'public', 'generic_presets.json')

const urdfText = fs.readFileSync(fixturePath, 'utf-8')
const presetsRaw = JSON.parse(fs.readFileSync(presetsPath, 'utf-8')) as {
  categories: Record<string, { components: LegacyServoMigrationPreset[] }>
}

const byId = new Map<string, LegacyServoMigrationPreset>()
for (const cat of Object.values(presetsRaw.categories)) {
  for (const c of cat.components) byId.set(c.id, c)
}

const ctx = { findPreset: (id: string) => byId.get(id) ?? null }

// Pre-migration baseline.
const preDoc = new DOMParser().parseFromString(urdfText, 'text/xml') as unknown as Document
const preLinks = Array.from(preDoc.getElementsByTagName('link'))
const preJoints = Array.from(preDoc.getElementsByTagName('joint'))
const preServoLinks = preLinks.filter(l => {
  const n = (l as unknown as Element).getAttribute('name') ?? ''
  return /^(actuator_servo_|actuator_continuous_)/.test(n) && !/_body$|_output$/.test(n)
})
const preRevolute = preJoints.filter(j => (j as unknown as Element).getAttribute('type') === 'revolute' || (j as unknown as Element).getAttribute('type') === 'continuous')

console.log(`Pre-migration: ${preLinks.length} links, ${preJoints.length} joints, ${preServoLinks.length} legacy servos, ${preRevolute.length} actuated joints`)

// Migrate (mutates a freshly parsed doc).
const doc = new DOMParser().parseFromString(urdfText, 'text/xml') as unknown as Document
const result = migrateLegacyServos(doc, ctx)
console.log(`Migration: migrated=${result.migrated}, skipped=${result.skipped}`)

const postLinks = Array.from(doc.getElementsByTagName('link'))
const postJoints = Array.from(doc.getElementsByTagName('joint'))
const postBodyLinks = postLinks.filter(l => /(_body)$/.test((l as unknown as Element).getAttribute('name') ?? ''))
const postOutputLinks = postLinks.filter(l => /(_output)$/.test((l as unknown as Element).getAttribute('name') ?? ''))
const postRevolute = postJoints.filter(j => (j as unknown as Element).getAttribute('type') === 'revolute')
const postInternalJoints = postJoints.filter(j => /_joint$/.test((j as unknown as Element).getAttribute('name') ?? ''))
const postMountJoints = postJoints.filter(j => /_mount$/.test((j as unknown as Element).getAttribute('name') ?? ''))

console.log(`Post-migration: ${postLinks.length} links, ${postJoints.length} joints`)
console.log(`  body=${postBodyLinks.length}, output=${postOutputLinks.length}, internal revolute=${postInternalJoints.length}, mount=${postMountJoints.length}, total revolute=${postRevolute.length}`)

interface Check { name: string; ok: boolean; detail?: string }
const checks: Check[] = []

checks.push({
  name: 'migrated count matches pre-servo count',
  ok: result.migrated === preServoLinks.length,
  detail: `expected ${preServoLinks.length}, got ${result.migrated}`,
})
checks.push({
  name: 'body links = servo count',
  ok: postBodyLinks.length === preServoLinks.length,
  detail: `expected ${preServoLinks.length}, got ${postBodyLinks.length}`,
})
checks.push({
  name: 'output links = servo count',
  ok: postOutputLinks.length === preServoLinks.length,
  detail: `expected ${preServoLinks.length}, got ${postOutputLinks.length}`,
})
checks.push({
  name: 'total revolute count = pre actuated count (DOF preserved)',
  ok: postRevolute.length === preRevolute.length,
  detail: `expected ${preRevolute.length}, got ${postRevolute.length}`,
})
checks.push({
  name: 'all revolute joints have `_joint` name suffix',
  ok: postRevolute.every(j => /_joint$/.test((j as unknown as Element).getAttribute('name') ?? '')),
  detail: `non-conforming: ${postRevolute.filter(j => !/_joint$/.test((j as unknown as Element).getAttribute('name') ?? '')).map(j => (j as unknown as Element).getAttribute('name')).join(', ')}`,
})
checks.push({
  name: 'each internal revolute has body parent → output child',
  ok: postInternalJoints.every(j => {
    const p = (j as unknown as Element).getElementsByTagName('parent')[0] as unknown as Element | undefined
    const c = (j as unknown as Element).getElementsByTagName('child')[0] as unknown as Element | undefined
    const pn = p?.getAttribute('link') ?? ''
    const cn = c?.getAttribute('link') ?? ''
    return /_body$/.test(pn) && /_output$/.test(cn)
  }),
  detail: 'one or more internal joints have mismatched parent/child suffixes',
})
checks.push({
  name: 'each mount joint child ends in `_body`',
  ok: postMountJoints.every(j => {
    const c = (j as unknown as Element).getElementsByTagName('child')[0] as unknown as Element | undefined
    return /_body$/.test(c?.getAttribute('link') ?? '')
  }),
})
checks.push({
  name: 'migration is idempotent (second pass migrates 0)',
  ok: (() => {
    const r2 = migrateLegacyServos(doc, ctx)
    return r2.migrated === 0
  })(),
})

// Serialize migrated XML to validate round-trip parseability.
const serialized = new XMLSerializer().serializeToString(doc as unknown as Node)
checks.push({
  name: 'serialized XML reparses cleanly',
  ok: (() => {
    try {
      const reparsed = new DOMParser().parseFromString(serialized, 'text/xml')
      return (reparsed as unknown as Document).getElementsByTagName('robot').length === 1
    } catch {
      return false
    }
  })(),
})

let passed = 0, failed = 0
for (const c of checks) {
  if (c.ok) { passed++; console.log(`  ✓ ${c.name}`) }
  else { failed++; console.log(`  ✗ ${c.name}${c.detail ? ` — ${c.detail}` : ''}`) }
}
console.log(`\n${passed} passed, ${failed} failed`)
if (failed > 0) process.exit(1)

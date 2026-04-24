// docs/SERVO_SPLIT_PLAN.md §Step 8.1 — CLI that produces a migrated copy of
// fixtures/legacy_servos/dog2.urdf for the Python urdf_to_mjcf smoke test.
//
// Run: cd src && node --experimental-strip-types src/migrate_dog2_cli.ts > \
//        ../fixtures/legacy_servos/dog2_migrated.urdf

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

const doc = new DOMParser().parseFromString(urdfText, 'text/xml') as unknown as Document
const result = migrateLegacyServos(doc, ctx)
console.error(`migrated=${result.migrated} skipped=${result.skipped}`)

process.stdout.write(new XMLSerializer().serializeToString(doc as unknown as Node))

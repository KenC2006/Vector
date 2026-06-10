// Primitive-anchor parity corpus (TS side). Mirrors
// core/sim/tests/test_anchor_parity.py over the shared fixture file
// scripts/primitive-anchor-corpus.json so the anchor positions the AI sees in
// its spatial context (Python) can never drift from what the placement
// compiler resolves at build time (TS).
//
// Run: cd src && npm run test:anchor-parity

import * as fs from 'node:fs'
import * as path from 'node:path'
import { fileURLToPath } from 'node:url'
import { resolvePrimitiveAnchorPose, anchorNamesForPrimitive, type LinkPrimitive } from './linkGeometry.ts'

interface AnchorCase {
  name: string
  primitive: LinkPrimitive & { name: string }
  anchor: string
  origin_xyz_mm?: number[]
  axis_xyz?: number[]
}

const here = path.dirname(fileURLToPath(import.meta.url))
const corpusPath = path.resolve(here, '..', '..', 'scripts', 'primitive-anchor-corpus.json')
const corpus = JSON.parse(fs.readFileSync(corpusPath, 'utf-8')) as {
  cases: AnchorCase[]
  invalid_cases: AnchorCase[]
}

const EPS = 1e-6
let failed = 0

function close(a: number[], b: number[]): boolean {
  return a.length === b.length && a.every((v, i) => Math.abs(v - b[i]) < EPS)
}

for (const c of corpus.cases) {
  const resolved = resolvePrimitiveAnchorPose([c.primitive], c.primitive.name, c.anchor)
  if (!resolved) {
    failed++
    console.log(`  FAIL  ${c.name}: expected a pose, got null`)
    continue
  }
  if (!close([...resolved.origin_xyz_mm], c.origin_xyz_mm!)) {
    failed++
    console.log(`  FAIL  ${c.name}: origin ${JSON.stringify(resolved.origin_xyz_mm)} != ${JSON.stringify(c.origin_xyz_mm)}`)
    continue
  }
  if (!close([...resolved.axis_xyz], c.axis_xyz!)) {
    failed++
    console.log(`  FAIL  ${c.name}: axis ${JSON.stringify(resolved.axis_xyz)} != ${JSON.stringify(c.axis_xyz)}`)
    continue
  }
  console.log(`  PASS  ${c.name}`)
}

for (const c of corpus.invalid_cases) {
  const resolved = resolvePrimitiveAnchorPose([c.primitive], c.primitive.name, c.anchor)
  if (resolved !== null) {
    failed++
    console.log(`  FAIL  ${c.name}: expected null, got ${JSON.stringify(resolved)}`)
    continue
  }
  const valid = anchorNamesForPrimitive(c.primitive)
  if (valid.includes(c.anchor)) {
    failed++
    console.log(`  FAIL  ${c.name}: anchorNamesForPrimitive still lists "${c.anchor}"`)
    continue
  }
  console.log(`  PASS  ${c.name}`)
}

const total = corpus.cases.length + corpus.invalid_cases.length
console.log(`\nprimitive-anchor parity (TS): ${total - failed}/${total} passed`)
process.exit(failed === 0 ? 0 : 1)

// Phase 3b.5 — headless compile-assembly CLI.
//
// Bridge between the Python AI loop and the TypeScript placement compiler.
// Reads one JSON request from stdin, writes the CompiledGraph to stdout.
//
// Request shape:
//   { "graph": AssemblyGraph,
//     "useMateConnectors"?: boolean,
//     "instanceOverrides"?: Record<string, ComponentInstanceSpec> }
//
// Response shape: CompiledGraph (see placementCompiler/index.ts)
//
// Exits 0 with stdout JSON on success; exits 1 with stderr JSON on failure.

import { readFileSync } from 'node:fs'
import { resolve as resolvePath, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'

import { compileAssembly } from './placementCompiler/index.ts'
import { resolveJointLimitsRad } from './placementCompiler/joints.ts'
import { resolveComponent as resolveSpec, resolveComponentMassKg, isParametricSpec } from './componentResolver.ts'
import { hasLinkGeometry } from './linkGeometry.ts'
import { setMeshExtentsCatalog } from './meshExtents.ts'
import type { ComponentResolver } from './placementCompiler/index.ts'
import type { MeasuredMeshCatalog } from './meshExtents.ts'
import type { MateConnector } from './mateConnectors.ts'

// The compiler logs progress via console.log (e.g. face.ts placement traces).
// stdout is reserved for the JSON response, so route every console channel
// to stderr where the Python client can capture it as a diagnostic stream.
const _stderr = (s: string) => (process as unknown as { stderr: { write(s: string): void } }).stderr.write(s + '\n')
console.log = (...args: unknown[]) => _stderr(args.map(a => typeof a === 'string' ? a : JSON.stringify(a)).join(' '))
console.warn = console.log
console.info = console.log
console.debug = console.log

const __dirname = dirname(fileURLToPath(import.meta.url))
const projRoot = resolvePath(__dirname, '..', '..')

const presetCatalogPath = resolvePath(projRoot, 'core', 'presets', 'generic_presets.json')
const presetCatalog = JSON.parse(readFileSync(presetCatalogPath, 'utf8'))

try {
  const meshExtentsPath = resolvePath(projRoot, 'src', 'public', 'meshExtents.generated.json')
  const meshExtents = JSON.parse(readFileSync(meshExtentsPath, 'utf8')) as MeasuredMeshCatalog
  setMeshExtentsCatalog(meshExtents)
} catch {
  // mesh extents are optional — the resolver degrades to bbox-only collision.
}

const presetById = new Map<string, Record<string, unknown>>()
for (const cat of Object.values(presetCatalog.categories ?? {}) as Array<{ components?: Array<Record<string, unknown>> }>) {
  for (const c of cat.components ?? []) {
    const id = c.id as string | undefined
    if (id) presetById.set(id, c)
  }
}

const resolver: ComponentResolver = (componentId, instance) => {
  const preset = presetById.get(componentId)
  if (!preset) return null
  const resolved = resolveSpec({ spec: preset as Parameters<typeof resolveSpec>[0]['spec'], instance })
  const me = (preset.mechanical_electrical ?? {}) as Record<string, unknown>
  const mounting = (preset.mounting_logic ?? {}) as Record<string, unknown>
  const torque =
    typeof me.max_torque_nm === 'number' ? (me.max_torque_nm as number)
    : typeof me.holding_torque_nm === 'number' ? (me.holding_torque_nm as number)
    : undefined
  const outerRadiusMm = typeof mounting.assembled_outer_radius_mm === 'number'
    ? (mounting.assembled_outer_radius_mm as number)
    : undefined
  // componentResolver returns bounds in millimeters; the placement compiler
  // expects meters (matches the browser path which feeds resolveComponentVisual
  // bounds, those are post-conversion meters).
  const halfMm = resolved.bounds.half
  return {
    componentId,
    bounds: {
      half: [halfMm[0] / 1000, halfMm[1] / 1000, halfMm[2] / 1000],
      center: [resolved.bounds.center[0] / 1000, resolved.bounds.center[1] / 1000, resolved.bounds.center[2] / 1000],
      shape: resolved.bounds.shape,
    },
    connectors: resolved.connectors,
    // Authored shells: the face path's connector-snap must target the union
    // faces, not the donor preset's authored surfaces.
    presetConnectors: hasLinkGeometry(instance)
      ? resolved.connectors
      : (preset.connectors as MateConnector[] | undefined),
    assembledOuterRadiusM: outerRadiusMm !== undefined ? outerRadiusMm / 1000 : undefined,
    parametricLengthMm: instance?.length_mm && isParametricSpec(preset as Parameters<typeof resolveSpec>[0]['spec'])
      ? instance.length_mm
      : undefined,
    jointLimitsRad: resolveJointLimitsRad(preset),
    maxTorqueNm: torque,
    massKg: resolveComponentMassKg(preset as Parameters<typeof resolveSpec>[0]['spec'], instance),
  }
}

// @types/node isn't pulled in by the browser tsconfig — narrow this to the
// stdio bits we need without dragging the full node typings into compile.
type NodeProcess = {
  stdin: { setEncoding(enc: string): void; on(ev: string, cb: (chunk?: unknown) => void): void }
  stdout: { write(s: string): void }
  stderr: { write(s: string): void }
  exit(code?: number): never
}
const proc = process as unknown as NodeProcess

function readAllStdin(): Promise<string> {
  return new Promise((res, rej) => {
    let data = ''
    proc.stdin.setEncoding('utf8')
    proc.stdin.on('data', chunk => { data += String(chunk) })
    proc.stdin.on('end', () => res(data))
    proc.stdin.on('error', rej)
  })
}

async function main() {
  const raw = await readAllStdin()
  const req = JSON.parse(raw)
  const compiled = compileAssembly(req.graph, {
    useMateConnectors: req.useMateConnectors ?? true,
    headless: true,
    instanceOverrides: req.instanceOverrides,
    resolveComponent: resolver,
  })
  proc.stdout.write(JSON.stringify(compiled))
}

main().catch(e => {
  proc.stderr.write(JSON.stringify({ error: String((e as Error)?.stack ?? e) }))
  proc.exit(1)
})

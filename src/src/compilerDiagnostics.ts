// Structured diagnostic taxonomy — TypeScript twin of
// core/ai/compiler_diagnostics.py.
//
// Phase 3: every layer that emits
// validation feedback should classify it by the layer that owns the fix, so
// the AI redesign loop never sees compiler/exporter bugs and the developer
// console never has to guess which subsystem is upset.
//
// This module is the TS-side router. The frontend chat panel and any
// validator that wants to raise a tagged diagnostic can import from here
// instead of returning bare strings.

// Using a const-object + literal-union pattern instead of `const enum` because
// the project compiles under `erasableSyntaxOnly`, which forbids enums.
export const DiagnosticOwner = {
  AiTopology: 'ai_topology',
  ComponentSpec: 'component_spec',
  PlacementCompiler: 'placement_compiler',
  Exporter: 'exporter',
} as const
export type DiagnosticOwner = typeof DiagnosticOwner[keyof typeof DiagnosticOwner]

export type DiagnosticSeverity = 'info' | 'warning' | 'error' | 'repaired'

export interface Diagnostic {
  code: string
  severity: DiagnosticSeverity
  owner: DiagnosticOwner
  message: string
  component?: string
  repair?: string
}

// Codes are kept in lockstep with core/ai/compiler_diagnostics.py.
const OWNER_BY_CODE: Record<string, DiagnosticOwner> = {
  // semantic graph
  semantic_graph_forbidden_field: DiagnosticOwner.AiTopology,
  semantic_graph_foot_pad_attach_rpy: DiagnosticOwner.AiTopology,
  semantic_graph_split_servo_link_name: DiagnosticOwner.AiTopology,
  semantic_graph_mirrored_sign: DiagnosticOwner.AiTopology,

  // component spec / resolver
  missing_bbox: DiagnosticOwner.ComponentSpec,
  mesh_vs_bbox_divergence: DiagnosticOwner.ComponentSpec,
  connector_outside_bounds: DiagnosticOwner.ComponentSpec,

  // placement
  placement_overlap: DiagnosticOwner.PlacementCompiler,
  joint_origin_unresolved: DiagnosticOwner.PlacementCompiler,

  // exporter
  urdf_mjcf_transform_mismatch: DiagnosticOwner.Exporter,
  rpy_quaternion_drift: DiagnosticOwner.Exporter,
}

export function ownerForCode(code: string): DiagnosticOwner {
  return OWNER_BY_CODE[code] ?? DiagnosticOwner.PlacementCompiler
}

export function makeDiagnostic(input: Omit<Diagnostic, 'owner'> & { owner?: DiagnosticOwner }): Diagnostic {
  return { ...input, owner: input.owner ?? ownerForCode(input.code) }
}

export function routeDiagnostics(diags: Iterable<Diagnostic>): Record<DiagnosticOwner, Diagnostic[]> {
  const bucket: Record<DiagnosticOwner, Diagnostic[]> = {
    [DiagnosticOwner.AiTopology]: [],
    [DiagnosticOwner.ComponentSpec]: [],
    [DiagnosticOwner.PlacementCompiler]: [],
    [DiagnosticOwner.Exporter]: [],
  }
  for (const d of diags) bucket[d.owner].push(d)
  return bucket
}

// Adapter so existing callers that still return string[] can be folded into
// the routed view without rewriting every callsite. Strings are placed into
// the placement bucket — the safest default for unclassified noise.
export function liftStrings(
  strings: readonly string[],
  severity: DiagnosticSeverity = 'warning',
  owner: DiagnosticOwner = DiagnosticOwner.PlacementCompiler,
): Diagnostic[] {
  return strings.map((message, i) => ({
    code: `legacy_string_${i}`,
    severity,
    owner,
    message,
  }))
}

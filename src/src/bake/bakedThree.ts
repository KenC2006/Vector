// Converters from the worker's SerializedMesh/Edges to THREE.js objects.
// Kept separate from bakeClient.ts so non-rendering callers (tests, worker
// hosts) don't pull in the full three.js graph.

import * as THREE from 'three'
import { shouldCastShadow } from '../richVisuals/meshOverrides.ts'
import type { SerializedMesh, SerializedEdges } from './types.ts'

const MM_TO_M = 0.001

export interface BakedThreeOptions {
  /** Multiplier applied to the raw buffer positions. Defaults to mm→m. */
  unitScale?: number
  /** Material for the filled faces. Callers in the render path will pass
   *  the same tinted material the per-preset GLB path uses. */
  material?: THREE.Material
  /** When true, also build an EdgesGeometry LineSegments overlay from the
   *  worker's meshEdges output. Off for Phase 1 smoke; on for Phase 5. */
  includeEdges?: boolean
  /** Line material for the edge overlay. Default: flat black 1px. */
  edgeMaterial?: THREE.LineBasicMaterial
  /** Component id (or link name) used to honor the per-preset shadow-cast
   *  policy in `richVisuals/meshOverrides.shouldCastShadow`. Grippers and
   *  similar detail-rich end-effectors opt out so their hard shadow on the
   *  parent extrusion doesn't read as a duplicate mesh. Omitted = always
   *  cast. */
  componentId?: string
}

/** Build a THREE.Mesh (and optional edges overlay group) from the worker's
 *  serialized buffers. The returned group is parented only to the caller;
 *  no scene side effects. */
export function buildBakedMesh(
  mesh: SerializedMesh,
  edges: SerializedEdges | undefined,
  opts: BakedThreeOptions = {},
): THREE.Group {
  const scale = opts.unitScale ?? MM_TO_M

  const geom = new THREE.BufferGeometry()
  // Replicad tessellates in OCCT's native millimeters; Three.js scene is in
  // meters. Rescale the position buffer in place (cheaper than setting
  // mesh.scale — a scaled matrix re-transforms normals on every frame,
  // which is wasteful for a static baked asset).
  const positions = new Float32Array(mesh.positions.length)
  for (let i = 0; i < positions.length; i++) positions[i] = mesh.positions[i] * scale
  geom.setAttribute('position', new THREE.BufferAttribute(positions, 3))
  geom.setAttribute('normal', new THREE.BufferAttribute(mesh.normals, 3))
  geom.setIndex(new THREE.BufferAttribute(mesh.indices, 1))
  geom.computeBoundingBox()
  geom.computeBoundingSphere()

  const material = opts.material ?? new THREE.MeshStandardMaterial({
    color: 0x888888, roughness: 0.4, metalness: 0.3,
  })
  const faceMesh = new THREE.Mesh(geom, material)
  faceMesh.castShadow = opts.componentId ? shouldCastShadow(opts.componentId) : true
  faceMesh.receiveShadow = true

  const group = new THREE.Group()
  group.add(faceMesh)

  // Effector-class components have detail-rich CAD geometry (gears, finger
  // teeth, linkages) — rendered as semi-transparent line overlays the edges
  // read as a separate floating wireframe ghost of the part. Skip them for
  // the same components that opt out of shadow casting; the same intricate-
  // detail reasoning applies.
  const includeEdges = opts.includeEdges && (!opts.componentId || shouldCastShadow(opts.componentId))
  if (includeEdges && edges && edges.positions.length >= 6) {
    const edgeGeom = new THREE.BufferGeometry()
    const edgePositions = new Float32Array(edges.positions.length)
    for (let i = 0; i < edgePositions.length; i++) edgePositions[i] = edges.positions[i] * scale
    edgeGeom.setAttribute('position', new THREE.BufferAttribute(edgePositions, 3))
    const edgeMat = opts.edgeMaterial ?? new THREE.LineBasicMaterial({ color: 0x202020 })
    const lines = new THREE.LineSegments(edgeGeom, edgeMat)
    lines.name = 'baked-edges'
    group.add(lines)
  }

  return group
}

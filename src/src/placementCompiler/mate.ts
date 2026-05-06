// Phase 3b — closed-form mate-connector placement extracted from
// urdfAssembly.ts:computeMatePlacement.
//
// The function is pure: takes the assembly component, both connector lists,
// the parent's bbox half-extents, optional multi-child distribution context,
// and a few flags. Returns null to signal "fall through to legacy bbox path"
// — the caller uses that as before. The DEV-only "ENGINE-REGRESSION" throw
// on a connector miss stays in the urdfAssembly wrapper because it reads
// `import.meta.env`; the pure function never throws.

import * as THREE from 'three'
import {
  resolveMate,
  findConnector,
  childConnectorIdForAttachFace,
  faceUVToWorldOffset,
  type MateConnector,
  type MateType,
} from '../mateConnectors.ts'
import { quatToRpy } from '../rotationIO.ts'
import type { AssemblyComponent } from '../urdfGraphEquivalence.ts'
import { _computeMultiChildOffsets } from './multiChild.ts'

export interface MatePlacementMultiChild {
  total: number
  index: number
  face: string
  childSizes?: Array<{ hu: number; hv: number }>
  insetOverride?: number
}

export type MateConnectorMissReason =
  | 'feature_disabled'
  | 'no_mate_fields'
  | 'connector_not_found_explicit'
  | 'connector_not_found_inferred'
  | 'unknown_mate_type'

export interface MatePlacementResult {
  xyz: string
  rpy: string
}

export interface MatePlacementMiss {
  miss: MateConnectorMissReason
  details?: string
}

export function hasMateConnectorFields(c: AssemblyComponent): boolean {
  return !!(c.attach_connector || c.mate_connector || c.mate_type)
}

/** Pure mate-connector resolver. Returns either a placement, or a typed miss
 *  reason. The caller is responsible for any logging/dev-throw policy. */
export function computeMatePlacement(
  comp: AssemblyComponent,
  parentConnectors: MateConnector[],
  childConnectors: MateConnector[],
  parentPresetBboxMm: { hxMm: number; hyMm: number; hzMm: number },
  multiChild: MatePlacementMultiChild | undefined,
  options: { useMateConnectors: boolean },
): MatePlacementResult | MatePlacementMiss {
  if (!options.useMateConnectors) return { miss: 'feature_disabled' }
  if (!hasMateConnectorFields(comp)) return { miss: 'no_mate_fields' }

  const parentConnectorId = comp.attach_connector ?? comp.attach_face ?? 'top'
  const inferredChildId = comp.attach_face ? childConnectorIdForAttachFace(comp.attach_face) : null
  const childConnectorId = comp.mate_connector ?? inferredChildId ?? 'bottom'

  const parentConn = findConnector(parentConnectors, parentConnectorId)
  const childConn  = findConnector(childConnectors,  childConnectorId)
  if (!parentConn || !childConn) {
    const details = `${comp.link_name}: parent="${parentConnectorId}" (${parentConn ? 'ok' : 'MISS'}), ` +
      `child="${childConnectorId}" (${childConn ? 'ok' : 'MISS'})`
    const explicit = (!!comp.attach_connector && !parentConn) || (!!comp.mate_connector && !childConn)
    return {
      miss: explicit ? 'connector_not_found_explicit' : 'connector_not_found_inferred',
      details,
    }
  }

  const mateType: MateType = ((comp.mate_type as MateType) ?? 'fastened')
  if (mateType !== 'fastened' && mateType !== 'planar' && mateType !== 'concentric') {
    return { miss: 'unknown_mate_type', details: `mate_type="${mateType}" for ${comp.link_name}` }
  }

  const childLocal = resolveMate(new THREE.Matrix4(), parentConn, childConn, mateType, {})
  const pos = new THREE.Vector3()
  const quat = new THREE.Quaternion()
  const scl = new THREE.Vector3()
  childLocal.decompose(pos, quat, scl)

  // Multi-child tangential distribution. Skipped for concentric mates because
  // SHAFT_FANOUT validator enforces single-child-per-shaft, so the case
  // doesn't arise there; spreading would also be wrong (a shaft-in-hole mate
  // is supposed to be concentric).
  if (multiChild && multiChild.total > 1 && mateType !== 'concentric') {
    const parentMeters = {
      hx: parentPresetBboxMm.hxMm / 1000,
      hy: parentPresetBboxMm.hyMm / 1000,
      hz: parentPresetBboxMm.hzMm / 1000,
    }
    const faceConnector = parentConn.id === multiChild.face
      ? parentConn
      : (findConnector(parentConnectors, multiChild.face) ?? null)
    const offsets = _computeMultiChildOffsets(
      multiChild.total,
      multiChild.index,
      parentMeters,
      multiChild.face,
      multiChild.insetOverride,
      multiChild.childSizes,
      faceConnector,
    )
    const { dx, dy, dz } = faceUVToWorldOffset(multiChild.face, offsets.u, offsets.v)
    pos.x += dx
    pos.y += dy
    pos.z += dz
  }

  const [r, p, y] = quatToRpy(quat)
  return {
    xyz: `${pos.x.toFixed(4)} ${pos.y.toFixed(4)} ${pos.z.toFixed(4)}`,
    rpy: `${r.toFixed(4)} ${p.toFixed(4)} ${y.toFixed(4)}`,
  }
}

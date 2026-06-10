// Component visual parity corpus: exercises the resolver contract shared by
// carry previews and committed rich-visual rendering.
//
// Run: cd src && npm run test:visual-parity
// Direct: node --experimental-strip-types src/componentVisualParityCorpus.ts

import * as THREE from 'three'
import { resolveComponentVisual, resolveSplitServoVisual, visualBoundsFromDescriptors } from './componentVisualResolver.ts'
import type { ComponentVisualPresetLike, ResolvedComponentVisual } from './componentVisualResolver.ts'
import type { UrdfVisualDesc } from './componentMeshes.ts'
import { composeGhostWorldForConnectorSnap } from './carrySnapMath.ts'
import { resolveFaceToPort } from './attachmentNodes.ts'
import { componentVisualWorldQuat, renderVisualQuaternionForSource } from './richVisuals/index.ts'
import { URDF_TO_SCENE_Q } from './coordinates.ts'
import { setCachedMeshGroup, markMeshLoadInProgress, clearMeshLoadInProgress } from './richVisuals/meshCache.ts'
import { resolveComponent } from './componentResolver.ts'
import { setMeshExtentsCatalog } from './meshExtents.ts'

interface Case {
  name: string
  passed: boolean
  reason?: string
}

function preset(
  id: string,
  boundingBoxMm: [number, number, number],
  collisionMesh?: string,
): ComponentVisualPresetLike {
  return {
    id,
    physical: {
      bounding_box_mm: boundingBoxMm,
      ...(collisionMesh ? { collision_mesh: collisionMesh } : {}),
    },
    mechanical_electrical: {},
  }
}

function makeRawMeshGroup(size: [number, number, number]): THREE.Group {
  const group = new THREE.Group()
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(size[0], size[1], size[2]),
    new THREE.MeshStandardMaterial({ color: 0x888888 }),
  )
  group.add(mesh)
  return group
}

function groupSize(group: THREE.Group | undefined): THREE.Vector3 | null {
  if (!group) return null
  group.updateMatrixWorld(true)
  const box = new THREE.Box3().setFromObject(group)
  if (box.isEmpty()) return null
  return box.getSize(new THREE.Vector3())
}

function renderedRichSize(group: THREE.Group | undefined): THREE.Vector3 | null {
  if (!group) return null
  const wrapper = new THREE.Group()
  const clone = group.clone(true)
  clone.quaternion.copy(renderVisualQuaternionForSource('rich'))
  wrapper.add(clone)
  return groupSize(wrapper)
}

/** Authored-frame unification: previewGroups are now wrapped to Z-up at the
 *  resolver. The per-generator dimensional tests below were authored in the
 *  generator's native Y-up frame ("bbox.z drives visual height Y"), which is
 *  the most natural way to express authoring intent. This helper unwraps the
 *  unification rotation so those tests continue to read their intended axes
 *  without needing per-assertion Y↔Z swaps. */
function generatorNativeSize(group: THREE.Group | undefined): THREE.Vector3 | null {
  if (!group) return null
  const wrapper = new THREE.Group()
  const clone = group.clone(true)
  clone.quaternion.setFromEuler(new THREE.Euler(-Math.PI / 2, 0, 0, 'XYZ'))
  wrapper.add(clone)
  return groupSize(wrapper)
}

function approx(a: number, b: number, tol = 1e-6): boolean {
  return Math.abs(a - b) <= tol
}

function sameSize(a: THREE.Vector3 | null, b: THREE.Vector3 | null, tol = 1e-6): boolean {
  if (!a || !b) return a === b
  return approx(a.x, b.x, tol) && approx(a.y, b.y, tol) && approx(a.z, b.z, tol)
}

function sameBounds(a: ResolvedComponentVisual, b: ResolvedComponentVisual, tol = 1e-9): boolean {
  return approx(a.bounds.hx, b.bounds.hx, tol)
    && approx(a.bounds.hy, b.bounds.hy, tol)
    && approx(a.bounds.hz, b.bounds.hz, tol)
    && approx(a.bounds.cx, b.bounds.cx, tol)
    && approx(a.bounds.cy, b.bounds.cy, tol)
    && approx(a.bounds.cz, b.bounds.cz, tol)
    && a.bounds.shape === b.bounds.shape
}

function assertUnifiedFrameAdapter(): Case {
  // Authored-frame unification: every previewGroup is Z-up regardless of
  // source. The render adapter is identity (URDF link is Z-up), the carry
  // adapter is -90° X (scene is Y-up). No per-source branching.
  const NAME = 'frame adapter: unified Z-up authored, render=identity, carry=-90°X'
  const renderUp = new THREE.Vector3(0, 0, 1).applyQuaternion(componentVisualWorldQuat('z_up', 'urdf_z_up'))
  if (!approx(renderUp.x, 0) || !approx(renderUp.y, 0) || !approx(renderUp.z, 1)) {
    return { name: NAME, passed: false, reason: `render target should preserve +Z, got ${renderUp.toArray()}` }
  }
  const carryUp = new THREE.Vector3(0, 0, 1).applyQuaternion(componentVisualWorldQuat('z_up', 'scene_y_up'))
  if (!approx(carryUp.x, 0) || !approx(carryUp.y, 1) || !approx(carryUp.z, 0)) {
    return { name: NAME, passed: false, reason: `carry target should map +Z to +Y, got ${carryUp.toArray()}` }
  }
  return { name: NAME, passed: true }
}

// Parent-frame transforms that the live runtime applies above each preview
// group. Carry parents into the scene (Y-up) directly; render parents into
// the URDF link group, which is itself rotated Rx(-π/2) by the URDF-to-scene
// adapter. Encoding both transforms here lets us compare the actual
// world-space orientation of the two paths, not just the local quaternion.
const URDF_TO_SCENE_PARENT = new THREE.Quaternion().setFromEuler(
  new THREE.Euler(-Math.PI / 2, 0, 0, 'XYZ'),
)

function carryWorldSize(resolved: ResolvedComponentVisual): THREE.Vector3 | null {
  if (!resolved.previewGroup) return null
  const wrapper = new THREE.Group()
  const clone = resolved.previewGroup.clone(true)
  clone.quaternion.copy(componentVisualWorldQuat(resolved.authoredFrame, 'scene_y_up'))
  wrapper.add(clone)
  return groupSize(wrapper)
}

function renderWorldSize(resolved: ResolvedComponentVisual): THREE.Vector3 | null {
  if (!resolved.previewGroup) return null
  const sceneParent = new THREE.Group()
  sceneParent.quaternion.copy(URDF_TO_SCENE_PARENT)
  const linkGroup = new THREE.Group()
  sceneParent.add(linkGroup)
  const clone = resolved.previewGroup.clone(true)
  clone.quaternion.copy(componentVisualWorldQuat(resolved.authoredFrame, 'urdf_z_up'))
  linkGroup.add(clone)
  return groupSize(sceneParent)
}

/** For each preset, assert the carry-path and render-path world bounding-box
 *  axis lengths agree. This is the per-component property the plan promises:
 *  hover preview and committed render are oriented identically by construction.
 *  Without it, a generator authored against the wrong frame can drift one path
 *  from the other and the only signal is a user noticing a flat wheel. */
function assertCarryRenderWorldParity(
  name: string,
  component: ComponentVisualPresetLike,
  category: string,
): Case {
  const resolved = resolveComponentVisual({ preset: component, category })
  if (!resolved.previewGroup) return { name, passed: true }
  const carry = carryWorldSize(resolved)
  const render = renderWorldSize(resolved)
  if (!sameSize(carry, render, 1e-5)) {
    return {
      name,
      passed: false,
      reason: `carry world size ${carry?.toArray()} vs render world size ${render?.toArray()} (authoredFrame=${resolved.authoredFrame})`,
    }
  }
  return { name, passed: true }
}

function assertResolvedParity(
  name: string,
  component: ComponentVisualPresetLike,
  category: string,
  expected: { source: string; status?: string; hasPreviewGroup: boolean },
): Case {
  const carry = resolveComponentVisual({ preset: component, category })
  const render = resolveComponentVisual({
    preset: component,
    category,
    linkName: `${component.id}_1`,
    castShadow: true,
    receiveShadow: true,
  })

  if (carry.source !== render.source) {
    return { name, passed: false, reason: `source mismatch: carry=${carry.source}, render=${render.source}` }
  }
  if (carry.source !== expected.source) {
    return { name, passed: false, reason: `expected source ${expected.source}, got ${carry.source}` }
  }
  if (expected.status && carry.status !== expected.status) {
    return { name, passed: false, reason: `expected carry status ${expected.status}, got ${carry.status}` }
  }
  if (expected.status && render.status !== expected.status) {
    return { name, passed: false, reason: `expected render status ${expected.status}, got ${render.status}` }
  }
  if (!!carry.previewGroup !== expected.hasPreviewGroup || !!render.previewGroup !== expected.hasPreviewGroup) {
    return {
      name,
      passed: false,
      reason: `preview presence mismatch: carry=${!!carry.previewGroup}, render=${!!render.previewGroup}`,
    }
  }
  if (!sameBounds(carry, render)) {
    return { name, passed: false, reason: `bounds mismatch: ${JSON.stringify(carry.bounds)} vs ${JSON.stringify(render.bounds)}` }
  }
  if (!sameSize(groupSize(carry.previewGroup), groupSize(render.previewGroup))) {
    return {
      name,
      passed: false,
      reason: `preview size mismatch: carry=${groupSize(carry.previewGroup)?.toArray()} render=${groupSize(render.previewGroup)?.toArray()}`,
    }
  }
  return { name, passed: true }
}

function assertResolvedConnectorsAndPorts(): Case {
  // WS4 contract: ports derive FROM connectors. The shaft class is authored
  // connector data (cls: 'shaft'), and face resolution picks the functional
  // port geometrically — the servo's `top` FACE resolves to the shaft_out
  // port even though a planar `top` mount_face port coexists at the same z.
  const component = preset('actuator_servo_standard', [40, 20, 37])
  component.mounting_logic = { output: 'axial_shaft' }
  component.connectors = [{
    id: 'shaft_out',
    origin_xyz_mm: [0, 0, 19],
    axis_xyz: [0, 0, 1],
    type: 'cylindrical',
    cls: 'shaft',
    single: true,
    diameter_mm: 6,
  }]

  const resolved = resolveComponentVisual({ preset: component, category: 'actuators' })
  const topConnector = resolved.connectors.find(c => c.id === 'top')
  const shaftConnector = resolved.connectors.find(c => c.id === 'shaft_out')
  const shaftPort = resolved.ports.find(p => p.nodeId === 'shaft_out')

  if (!topConnector || !shaftConnector) {
    return {
      name: 'resolver contract: connectors and ports are emitted',
      passed: false,
      reason: `missing connector(s): ids=${resolved.connectors.map(c => c.id).join(',')}`,
    }
  }
  if (!shaftPort || shaftPort.cls !== 'shaft' || !shaftPort.single) {
    return {
      name: 'resolver contract: connectors and ports are emitted',
      passed: false,
      reason: `expected a single-use shaft port derived from shaft_out, got ${JSON.stringify(shaftPort)}`,
    }
  }
  const topFacePort = resolveFaceToPort('top', resolved.ports)
  if (topFacePort?.nodeId !== 'shaft_out') {
    return {
      name: 'resolver contract: connectors and ports are emitted',
      passed: false,
      reason: `expected face 'top' to resolve to the shaft_out port, got ${topFacePort?.nodeId ?? 'none'}`,
    }
  }
  if (resolved.connectors.length < 7 || shaftConnector.origin_xyz_mm[2] !== 19) {
    return {
      name: 'resolver contract: connectors and ports are emitted',
      passed: false,
      reason: `expected six defaults plus authored shaft_out, got ${resolved.connectors.length}`,
    }
  }
  return { name: 'resolver contract: connectors and ports are emitted', passed: true }
}

function assertPrimitiveBoundsBakeRpy(): Case {
  const rotated: UrdfVisualDesc[] = [{
    origin_xyz: [0, 0, 0],
    origin_rpy: [0, 0, Math.PI / 2],
    geometry: { type: 'box', size: [0.1, 0.02, 0.03] },
    color_rgba: [1, 1, 1, 1],
  }]
  const bounds = visualBoundsFromDescriptors(rotated)
  if (!bounds) {
    return { name: 'bounds: primitive descriptor RPY is baked into AABB', passed: false, reason: 'missing bounds' }
  }
  if (!approx(bounds.hx, 0.01) || !approx(bounds.hy, 0.05) || !approx(bounds.hz, 0.015)) {
    return {
      name: 'bounds: primitive descriptor RPY is baked into AABB',
      passed: false,
      reason: `expected half-extents [0.01,0.05,0.015], got [${bounds.hx},${bounds.hy},${bounds.hz}]`,
    }
  }
  return { name: 'bounds: primitive descriptor RPY is baked into AABB', passed: true }
}

function assertTargetEnvelopeFields(): Case {
  const fixed = resolveComponentVisual({
    preset: {
      id: 'target_bbox_box',
      physical: { bbox_mm: [12, 14, 16] },
      mechanical_electrical: {},
    },
    category: 'misc',
  })
  if (!approx(fixed.bounds.hx, 0.006) || !approx(fixed.bounds.hy, 0.007) || !approx(fixed.bounds.hz, 0.008)) {
    return {
      name: 'resolver contract: target bbox and parametric fields resolve',
      passed: false,
      reason: `expected bbox half extents [0.006,0.007,0.008], got [${fixed.bounds.hx},${fixed.bounds.hy},${fixed.bounds.hz}]`,
    }
  }

  const parametric = resolveComponentVisual({
    preset: {
      id: 'target_parametric_rod',
      physical: {
        inertia_primitive: 'cylinder',
        parametric: {
          axis: 'z',
          cross_section_mm: [8, 8],
        },
      },
      mechanical_electrical: {},
    },
    category: 'structural',
    instance: { length_mm: 250 },
  })
  const topConnector = parametric.connectors.find(c => c.id === 'top')
  if (!approx(parametric.bounds.hx, 0.004) || !approx(parametric.bounds.hy, 0.004) || !approx(parametric.bounds.hz, 0.125)) {
    return {
      name: 'resolver contract: target bbox and parametric fields resolve',
      passed: false,
      reason: `expected parametric half extents [0.004,0.004,0.125], got [${parametric.bounds.hx},${parametric.bounds.hy},${parametric.bounds.hz}]`,
    }
  }
  if (!topConnector || topConnector.origin_xyz_mm[2] !== 125) {
    return {
      name: 'resolver contract: target bbox and parametric fields resolve',
      passed: false,
      reason: `expected parametric top connector z=125, got ${topConnector?.origin_xyz_mm[2] ?? 'missing'}`,
    }
  }
  return { name: 'resolver contract: target bbox and parametric fields resolve', passed: true }
}

function assertCarryConnectorSnapMath(): Case {
  const sourceLocal = {
    position: new THREE.Vector3(0.01, -0.02, 0.005),
    axis: new THREE.Vector3(0, -1, 0),
  }
  const targetWorld = {
    position: new THREE.Vector3(1, 2, 3),
    axis: new THREE.Vector3(0, 1, 0),
  }
  const ghostWorld = composeGhostWorldForConnectorSnap(sourceLocal, targetWorld)
  const snappedSource = sourceLocal.position.clone().applyMatrix4(ghostWorld)
  const snappedAxis = sourceLocal.axis.clone().transformDirection(ghostWorld)
  if (snappedSource.distanceTo(targetWorld.position) > 1e-9) {
    return {
      name: 'carry snap: connector frame places ghost point and axis',
      passed: false,
      reason: `expected source point at target, got ${snappedSource.toArray()}`,
    }
  }
  if (snappedAxis.dot(targetWorld.axis) > -0.999999) {
    return {
      name: 'carry snap: connector frame places ghost point and axis',
      passed: false,
      reason: `expected antiparallel axes, got source=${snappedAxis.toArray()} target=${targetWorld.axis.toArray()}`,
    }
  }
  return { name: 'carry snap: connector frame places ghost point and axis', passed: true }
}

function assertPowerGeneratorAxisAdapter(): Case {
  const lipo = preset('power_lipo_3s_2200', [105, 34, 24])
  markMeshLoadInProgress(lipo.id)
  const resolved = resolveComponentVisual({ preset: lipo, category: 'power' })
  clearMeshLoadInProgress(lipo.id)

  const size = generatorNativeSize(resolved.previewGroup)
  if (!size) {
    return { name: 'power adapter: rich fallback has measurable bounds', passed: false, reason: 'missing preview group size' }
  }
  if (size.y >= size.z) {
    return {
      name: 'power adapter: bbox z maps to generator height',
      passed: false,
      reason: `expected generated height Y to be less than generated depth Z, got ${size.toArray()}`,
    }
  }
  return { name: 'power adapter: bbox z maps to generator height', passed: true }
}

function assertComputeGeneratorAxisAdapter(): Case {
  const hub = preset('compute_usb_hub', [40, 30, 8])
  const resolved = resolveComponentVisual({ preset: hub, category: 'compute' })

  const size = generatorNativeSize(resolved.previewGroup)
  if (resolved.source !== 'rich') {
    return {
      name: 'compute adapter: rich generator remains fallback source',
      passed: false,
      reason: `expected rich source, got ${resolved.source}`,
    }
  }
  if (!size) {
    return { name: 'compute adapter: rich generator has measurable bounds', passed: false, reason: 'missing preview group size' }
  }
  if (size.y >= size.z) {
    return {
      name: 'compute adapter: bbox z maps to generator height',
      passed: false,
      reason: `expected generated height Y to be less than generated depth Z, got ${size.toArray()}`,
    }
  }
  return { name: 'compute adapter: bbox z maps to generator height', passed: true }
}

function assertMobilityUprightAxisAdapter(): Case {
  const caster = preset('mobility_caster_wheel', [50, 50, 65])
  markMeshLoadInProgress(caster.id)
  const resolved = resolveComponentVisual({ preset: caster, category: 'mobility' })
  clearMeshLoadInProgress(caster.id)

  const size = generatorNativeSize(resolved.previewGroup)
  if (resolved.source !== 'rich') {
    return {
      name: 'mobility adapter: upright fallback remains rich source',
      passed: false,
      reason: `expected rich source, got ${resolved.source}`,
    }
  }
  if (!size) {
    return { name: 'mobility adapter: upright component has measurable bounds', passed: false, reason: 'missing preview group size' }
  }
  if (size.y < 0.055) {
    return {
      name: 'mobility adapter: upright bbox z maps to generator height',
      passed: false,
      reason: `expected caster generated height Y to follow bbox Z, got ${size.toArray()}`,
    }
  }
  return { name: 'mobility adapter: upright bbox z maps to generator height', passed: true }
}

function assertMobilityWheelAxisAdapter(): Case {
  const driven = preset('mobility_wheel_driven', [100, 100, 30])
  markMeshLoadInProgress(driven.id)
  const drivenResolved = resolveComponentVisual({ preset: driven, category: 'mobility' })
  clearMeshLoadInProgress(driven.id)

  const drivenSize = generatorNativeSize(drivenResolved.previewGroup)
  if (drivenResolved.source !== 'rich') {
    return {
      name: 'mobility adapter: wheel fallback remains rich source',
      passed: false,
      reason: `expected rich source, got ${drivenResolved.source}`,
    }
  }
  if (!drivenSize) {
    return { name: 'mobility adapter: wheel has measurable bounds', passed: false, reason: 'missing driven wheel size' }
  }
  if (drivenSize.z > 0.033) {
    return {
      name: 'mobility adapter: wheel bbox z maps to axle thickness',
      passed: false,
      reason: `expected driven wheel axle thickness Z to come from bbox Z, got ${drivenSize.toArray()}`,
    }
  }

  const mecanum = preset('mobility_mecanum_wheel', [100, 100, 48])
  markMeshLoadInProgress(mecanum.id)
  const resolved = resolveComponentVisual({ preset: mecanum, category: 'mobility' })
  clearMeshLoadInProgress(mecanum.id)

  const size = generatorNativeSize(resolved.previewGroup)
  if (resolved.source !== 'rich') {
    return {
      name: 'mobility adapter: wheel fallback remains rich source',
      passed: false,
      reason: `expected rich source, got ${resolved.source}`,
    }
  }
  if (!size) {
    return { name: 'mobility adapter: wheel has measurable bounds', passed: false, reason: 'missing preview group size' }
  }
  if (size.z < 0.04 || size.z > 0.055) {
    return {
      name: 'mobility adapter: wheel bbox z maps to axle thickness',
      passed: false,
      reason: `expected mecanum axle thickness Z near bbox Z, got ${size.toArray()}`,
    }
  }
  return { name: 'mobility adapter: wheel bbox z maps to axle thickness', passed: true }
}

function assertMobilityTrackAxisAdapter(): Case {
  const track = preset('mobility_track_tread_system', [200, 50, 60])
  const resolved = resolveComponentVisual({ preset: track, category: 'mobility' })

  const size = generatorNativeSize(resolved.previewGroup)
  if (resolved.source !== 'rich') {
    return {
      name: 'mobility adapter: track fallback remains rich source',
      passed: false,
      reason: `expected rich source, got ${resolved.source}`,
    }
  }
  if (!size) {
    return { name: 'mobility adapter: track has measurable bounds', passed: false, reason: 'missing preview group size' }
  }
  if (size.x < 0.15 || size.y < 0.045 || size.z < 0.035 || size.z > 0.065) {
    return {
      name: 'mobility adapter: track uses bbox length/depth/height',
      passed: false,
      reason: `expected track dimensions to follow bbox, got ${size.toArray()}`,
    }
  }
  return { name: 'mobility adapter: track uses bbox length/depth/height', passed: true }
}

function assertMotorGeneratorAxisAdapter(): Case {
  const gearMotor = preset('motor_gear_medium_37mm', [37, 37, 70])
  markMeshLoadInProgress(gearMotor.id)
  const resolved = resolveComponentVisual({ preset: gearMotor, category: 'motors' })
  clearMeshLoadInProgress(gearMotor.id)

  const size = generatorNativeSize(resolved.previewGroup)
  if (resolved.source !== 'rich') {
    return {
      name: 'motor adapter: gear motor fallback remains rich source',
      passed: false,
      reason: `expected rich source, got ${resolved.source}`,
    }
  }
  if (!size) {
    return { name: 'motor adapter: gear motor has measurable bounds', passed: false, reason: 'missing preview group size' }
  }
  if (size.y < 0.065 || size.z > 0.05) {
    return {
      name: 'motor adapter: gear motor bbox z maps to axial length',
      passed: false,
      reason: `expected gear motor axial length Y and compact Z, got ${size.toArray()}`,
    }
  }
  return { name: 'motor adapter: gear motor bbox z maps to axial length', passed: true }
}

function assertActuatorGeneratorAxisAdapter(): Case {
  const servo = preset('actuator_servo_standard', [40, 20, 37])
  markMeshLoadInProgress(servo.id)
  const servoResolved = resolveComponentVisual({ preset: servo, category: 'actuators' })
  clearMeshLoadInProgress(servo.id)

  const servoSize = generatorNativeSize(servoResolved.previewGroup)
  if (servoResolved.source !== 'rich') {
    return {
      name: 'actuator adapter: normalized bbox axes stay direct',
      passed: false,
      reason: `expected servo rich source, got ${servoResolved.source}`,
    }
  }
  if (!servoSize || servoSize.y < 0.03 || servoSize.z > 0.03) {
    return {
      name: 'actuator adapter: normalized bbox axes stay direct',
      passed: false,
      reason: `expected servo bbox Z to drive height Y and bbox Y to drive depth Z, got ${servoSize?.toArray()}`,
    }
  }

  const bldc = preset('actuator_bldc_small', [76, 76, 48])
  const bldcResolved = resolveComponentVisual({ preset: bldc, category: 'actuators' })
  const bldcSize = generatorNativeSize(bldcResolved.previewGroup)
  if (!bldcSize || bldcSize.y < 0.05 || bldcSize.x < 0.07 || bldcSize.z < 0.07) {
    return {
      name: 'actuator adapter: normalized bbox axes stay direct',
      passed: false,
      reason: `expected BLDC diameter on X/Z and height on Y, got ${bldcSize?.toArray()}`,
    }
  }

  const stepper = preset('actuator_stepper_nema17', [42.3, 42.3, 48])
  markMeshLoadInProgress(stepper.id)
  const stepperResolved = resolveComponentVisual({ preset: stepper, category: 'actuators' })
  clearMeshLoadInProgress(stepper.id)
  const stepperSize = generatorNativeSize(stepperResolved.previewGroup)
  if (!stepperSize || stepperSize.y < 0.055 || stepperSize.x < 0.04 || stepperSize.z < 0.04) {
    return {
      name: 'actuator adapter: normalized bbox axes stay direct',
      passed: false,
      reason: `expected stepper axial height on Y and square face on X/Z, got ${stepperSize?.toArray()}`,
    }
  }

  const linear = preset('actuator_linear_small', [16, 16, 130])
  markMeshLoadInProgress(linear.id)
  const linearResolved = resolveComponentVisual({ preset: linear, category: 'actuators' })
  clearMeshLoadInProgress(linear.id)
  const linearSize = generatorNativeSize(linearResolved.previewGroup)
  if (!linearSize || linearSize.z < 0.17 || linearSize.y > 0.03) {
    return {
      name: 'actuator adapter: normalized bbox axes stay direct',
      passed: false,
      reason: `expected linear actuator length on Z and compact height on Y, got ${linearSize?.toArray()}`,
    }
  }

  return { name: 'actuator adapter: normalized bbox axes stay direct', passed: true }
}

function assertSplitServoResolverContract(): Case {
  const servo = preset('actuator_servo_standard', [40, 20, 37], 'servo_standard_collision.obj')
  const split = resolveSplitServoVisual({ preset: servo, category: 'actuators' })

  if (split.bodyCollision.source !== 'authored_mesh' || split.bodyCollision.meshFile !== 'servo_standard_collision.obj') {
    return {
      name: 'split servo resolver: body and horn collision contract is explicit',
      passed: false,
      reason: `expected body authored collision mesh, got ${JSON.stringify(split.bodyCollision)}`,
    }
  }
  if (split.hornCollision.source !== 'urdf_primitives') {
    return {
      name: 'split servo resolver: body and horn collision contract is explicit',
      passed: false,
      reason: `expected horn primitive collision, got ${JSON.stringify(split.hornCollision)}`,
    }
  }
  if (split.bodyVisuals.length === 0 || split.hornVisuals.length === 0 || split.hornOriginZ <= 0) {
    return {
      name: 'split servo resolver: body and horn collision contract is explicit',
      passed: false,
      reason: `expected body/horn visuals and positive horn origin, got body=${split.bodyVisuals.length} horn=${split.hornVisuals.length} origin=${split.hornOriginZ}`,
    }
  }

  const sideYoke = resolveSplitServoVisual({ preset: servo, category: 'actuators', includeSideYoke: true })
  if (sideYoke.bodyVisuals.length <= split.bodyVisuals.length || sideYoke.hornVisuals.length <= split.hornVisuals.length) {
    return {
      name: 'split servo resolver: body and horn collision contract is explicit',
      passed: false,
      reason: 'expected side-yoke split visual to add body and horn adapter primitives',
    }
  }

  return { name: 'split servo resolver: body and horn collision contract is explicit', passed: true }
}

function assertTransmissionLongFootprintAxisAdapter(): Case {
  const rack = preset('transmission_rack_pinion_set', [20, 200, 12])
  const rackResolved = resolveComponentVisual({ preset: rack, category: 'transmission' })
  const rackSize = generatorNativeSize(rackResolved.previewGroup)
  if (!rackSize || rackSize.z < 0.15 || rackSize.y > 0.03) {
    return {
      name: 'transmission adapter: long footprint uses bbox depth',
      passed: false,
      reason: `expected rack long axis on Z and compact height on Y, got ${rackSize?.toArray()}`,
    }
  }

  return { name: 'transmission adapter: long footprint uses bbox depth', passed: true }
}

function assertTransmissionFlatHardwareAxisAdapter(): Case {
  const gearPair = preset('transmission_spur_gear_pair', [42, 42, 10])
  const gearResolved = resolveComponentVisual({ preset: gearPair, category: 'transmission' })
  const gearSize = generatorNativeSize(gearResolved.previewGroup)
  if (!gearSize || gearSize.y > 0.012 || gearSize.x < 0.035 || gearSize.z < 0.025) {
    return {
      name: 'transmission adapter: gears and chains stay flat',
      passed: false,
      reason: `expected gear pair footprint on X/Z and compact thickness on Y, got ${gearSize?.toArray()}`,
    }
  }

  const chain = preset('transmission_chain_sprocket_set', [60, 100, 8])
  const chainResolved = resolveComponentVisual({ preset: chain, category: 'transmission' })
  const chainSize = generatorNativeSize(chainResolved.previewGroup)
  if (!chainSize || chainSize.z < 0.09 || chainSize.y > 0.012) {
    return {
      name: 'transmission adapter: gears and chains stay flat',
      passed: false,
      reason: `expected chain long axis on Z and compact thickness on Y, got ${chainSize?.toArray()}`,
    }
  }

  const bearing = preset('transmission_bearing_deep_groove', [22, 22, 7])
  markMeshLoadInProgress(bearing.id)
  const bearingResolved = resolveComponentVisual({ preset: bearing, category: 'transmission' })
  clearMeshLoadInProgress(bearing.id)
  const bearingSize = generatorNativeSize(bearingResolved.previewGroup)
  if (!bearingSize || bearingSize.y > 0.009 || bearingSize.x < 0.02 || bearingSize.z < 0.02) {
    return {
      name: 'transmission adapter: gears and chains stay flat',
      passed: false,
      reason: `expected bearing diameter on X/Z and compact width on Y, got ${bearingSize?.toArray()}`,
    }
  }

  const slewing = preset('transmission_slewing_ring_bearing', [200, 200, 25])
  const slewingResolved = resolveComponentVisual({ preset: slewing, category: 'transmission' })
  const slewingSize = generatorNativeSize(slewingResolved.previewGroup)
  if (!slewingSize || slewingSize.y > 0.03 || slewingSize.x < 0.19 || slewingSize.z < 0.19) {
    return {
      name: 'transmission adapter: gears and chains stay flat',
      passed: false,
      reason: `expected slewing ring diameter on X/Z and compact width on Y, got ${slewingSize?.toArray()}`,
    }
  }

  return { name: 'transmission adapter: gears and chains stay flat', passed: true }
}

function assertTransmissionAngleGearAxisAdapter(): Case {
  const bevel = preset('transmission_bevel_gear_pair', [35, 35, 35])
  const bevelResolved = resolveComponentVisual({ preset: bevel, category: 'transmission' })
  const bevelSize = generatorNativeSize(bevelResolved.previewGroup)
  if (!bevelSize || bevelSize.x < 0.02 || bevelSize.y < 0.02 || bevelSize.z < 0.02) {
    return {
      name: 'transmission adapter: bevel and worm gears keep 3D footprint',
      passed: false,
      reason: `expected bevel gear pair to occupy X/Y/Z footprint, got ${bevelSize?.toArray()}`,
    }
  }
  if (bevelSize.x > 0.04 || bevelSize.y > 0.04 || bevelSize.z > 0.04) {
    return {
      name: 'transmission adapter: bevel and worm gears keep 3D footprint',
      passed: false,
      reason: `expected bevel gear pair to stay inside bbox scale, got ${bevelSize.toArray()}`,
    }
  }

  const worm = preset('transmission_worm_gear_set', [40, 40, 30])
  const wormResolved = resolveComponentVisual({ preset: worm, category: 'transmission' })
  const wormSize = generatorNativeSize(wormResolved.previewGroup)
  if (!wormSize || wormSize.z < 0.022 || wormSize.y > 0.035 || wormSize.x > 0.04) {
    return {
      name: 'transmission adapter: bevel and worm gears keep 3D footprint',
      passed: false,
      reason: `expected worm gear set to keep worm length on Z and compact Y, got ${wormSize?.toArray()}`,
    }
  }

  return { name: 'transmission adapter: bevel and worm gears keep 3D footprint', passed: true }
}

function assertTransmissionShaftHardwareAxisAdapter(): Case {
  const leadscrew = preset('transmission_leadscrew_8mm', [8, 8, 200])
  const leadscrewResolved = resolveComponentVisual({ preset: leadscrew, category: 'transmission' })
  const leadscrewSize = generatorNativeSize(leadscrewResolved.previewGroup)
  if (!leadscrewSize || leadscrewSize.z < 0.18 || leadscrewSize.y > 0.02) {
    return {
      name: 'transmission adapter: shaft hardware keeps axial length on Z',
      passed: false,
      reason: `expected leadscrew axial length on Z and compact Y, got ${leadscrewSize?.toArray()}`,
    }
  }

  const coupling = preset('transmission_flexible_coupling_jaw', [20, 20, 30])
  markMeshLoadInProgress(coupling.id)
  const couplingResolved = resolveComponentVisual({ preset: coupling, category: 'transmission' })
  clearMeshLoadInProgress(coupling.id)
  const couplingSize = generatorNativeSize(couplingResolved.previewGroup)
  if (!couplingSize || couplingSize.z < 0.028 || couplingSize.y > 0.023) {
    return {
      name: 'transmission adapter: shaft hardware keeps axial length on Z',
      passed: false,
      reason: `expected coupling axial length on Z and compact Y, got ${couplingSize?.toArray()}`,
    }
  }

  const uJoint = preset('transmission_universal_joint', [18, 18, 35])
  const uJointResolved = resolveComponentVisual({ preset: uJoint, category: 'transmission' })
  const uJointSize = generatorNativeSize(uJointResolved.previewGroup)
  if (!uJointSize || uJointSize.z < 0.015 || uJointSize.y > 0.025) {
    return {
      name: 'transmission adapter: shaft hardware keeps axial length on Z',
      passed: false,
      reason: `expected universal-joint axial detail on Z and compact Y, got ${uJointSize?.toArray()}`,
    }
  }

  const planetary = preset('transmission_planetary_gearbox', [42.3, 42.3, 38])
  markMeshLoadInProgress(planetary.id)
  const planetaryResolved = resolveComponentVisual({ preset: planetary, category: 'transmission' })
  clearMeshLoadInProgress(planetary.id)
  const planetarySize = generatorNativeSize(planetaryResolved.previewGroup)
  if (!planetarySize || planetarySize.z < 0.035 || planetarySize.y > 0.045) {
    return {
      name: 'transmission adapter: shaft hardware keeps axial length on Z',
      passed: false,
      reason: `expected planetary gearbox axial length on Z and square face on X/Y, got ${planetarySize?.toArray()}`,
    }
  }

  return { name: 'transmission adapter: shaft hardware keeps axial length on Z', passed: true }
}

function assertEndEffectorAxisAdapter(): Case {
  const gripper = preset('effector_parallel_gripper_small', [65, 45, 90])
  markMeshLoadInProgress(gripper.id)
  const gripperResolved = resolveComponentVisual({ preset: gripper, category: 'end_effectors' })
  clearMeshLoadInProgress(gripper.id)

  const gripperSize = generatorNativeSize(gripperResolved.previewGroup)
  if (gripperResolved.source !== 'rich') {
    return {
      name: 'end effector adapter: bbox z maps to vertical height',
      passed: false,
      reason: `expected gripper rich source, got ${gripperResolved.source}`,
    }
  }
  if (!gripperSize || gripperSize.y < 0.085 || gripperSize.z > 0.045) {
    return {
      name: 'end effector adapter: bbox z maps to vertical height',
      passed: false,
      reason: `expected gripper vertical Y from bbox Z and compact Z depth from bbox Y, got ${gripperSize?.toArray()}`,
    }
  }

  const suction = preset('effector_suction_cup', [40, 25, 80])
  markMeshLoadInProgress(suction.id)
  const suctionResolved = resolveComponentVisual({ preset: suction, category: 'end_effectors' })
  clearMeshLoadInProgress(suction.id)

  const suctionSize = generatorNativeSize(suctionResolved.previewGroup)
  if (!suctionSize || suctionSize.y < 0.055 || suctionSize.z > 0.04) {
    return {
      name: 'end effector adapter: bbox z maps to vertical height',
      passed: false,
      reason: `expected suction cup vertical Y from bbox Z, got ${suctionSize?.toArray()}`,
    }
  }

  const holder = preset('effector_screwdriver_holder', [25, 25, 60])
  markMeshLoadInProgress(holder.id)
  const holderResolved = resolveComponentVisual({ preset: holder, category: 'end_effectors' })
  clearMeshLoadInProgress(holder.id)

  const holderSize = generatorNativeSize(holderResolved.previewGroup)
  if (!holderSize || holderSize.y < 0.03 || holderSize.z > 0.03) {
    return {
      name: 'end effector adapter: bbox z maps to vertical height',
      passed: false,
      reason: `expected holder vertical Y from bbox Z, got ${holderSize?.toArray()}`,
    }
  }

  return { name: 'end effector adapter: bbox z maps to vertical height', passed: true }
}

function assertStructuralAxisAdapter(): Case {
  const extrusion = preset('structural_extrusion_2020', [20, 20, 200])
  markMeshLoadInProgress(extrusion.id)
  const extrusionResolved = resolveComponentVisual({ preset: extrusion, category: 'structural' })
  clearMeshLoadInProgress(extrusion.id)

  const extrusionSize = generatorNativeSize(extrusionResolved.previewGroup)
  if (extrusionResolved.source !== 'rich') {
    return {
      name: 'structural adapter: plates stay thin and profiles stay long',
      passed: false,
      reason: `expected extrusion rich source, got ${extrusionResolved.source}`,
    }
  }
  if (!extrusionSize || extrusionSize.z < 0.18 || extrusionSize.y > 0.025) {
    return {
      name: 'structural adapter: plates stay thin and profiles stay long',
      passed: false,
      reason: `expected extrusion length on Z and compact cross-section on Y, got ${extrusionSize?.toArray()}`,
    }
  }

  const angle = preset('structural_angle_aluminum_25x25', [25, 25, 200])
  const angleResolved = resolveComponentVisual({ preset: angle, category: 'structural' })
  const angleSize = generatorNativeSize(angleResolved.previewGroup)
  if (!angleSize || angleSize.z < 0.18 || angleSize.y > 0.035) {
    return {
      name: 'structural adapter: plates stay thin and profiles stay long',
      passed: false,
      reason: `expected angle stock length on Z and compact cross-section on Y, got ${angleSize?.toArray()}`,
    }
  }

  const baseplate = preset('structural_baseplate', [200, 150, 8])
  const baseplateResolved = resolveComponentVisual({ preset: baseplate, category: 'structural' })
  const baseplateSize = generatorNativeSize(baseplateResolved.previewGroup)
  if (!baseplateSize || baseplateSize.y > 0.018 || baseplateSize.z < 0.14) {
    return {
      name: 'structural adapter: plates stay thin and profiles stay long',
      passed: false,
      reason: `expected baseplate thickness on Y and footprint depth on Z, got ${baseplateSize?.toArray()}`,
    }
  }

  const slimLimb = preset('structural_limb_link_slim', [14, 6, 100])
  const slimLimbResolved = resolveComponentVisual({ preset: slimLimb, category: 'structural' })
  const slimCarrySize = generatorNativeSize(slimLimbResolved.previewGroup)
  const slimRenderSize = renderedRichSize(slimLimbResolved.previewGroup)
  if (!slimCarrySize || !slimRenderSize || slimCarrySize.y < 0.09 || slimRenderSize.z < 0.09 || slimRenderSize.y > 0.012) {
    return {
      name: 'structural adapter: plates stay thin and profiles stay long',
      passed: false,
      reason: `expected slim limb to author length on rich Y and render length on URDF Z, got carry=${slimCarrySize?.toArray()} render=${slimRenderSize?.toArray()}`,
    }
  }

  const rail = preset('structural_linear_rail_mgn12', [12, 8, 200])
  markMeshLoadInProgress(rail.id)
  const railResolved = resolveComponentVisual({ preset: rail, category: 'structural' })
  clearMeshLoadInProgress(rail.id)
  const railSize = generatorNativeSize(railResolved.previewGroup)
  if (!railSize || railSize.z < 0.18 || railSize.y > 0.015) {
    return {
      name: 'structural adapter: plates stay thin and profiles stay long',
      passed: false,
      reason: `expected linear rail length on Z and compact height on Y, got ${railSize?.toArray()}`,
    }
  }

  const crossPlate = preset('structural_cross_plate', [60, 60, 3])
  const crossPlateResolved = resolveComponentVisual({ preset: crossPlate, category: 'structural' })
  const crossPlateSize = generatorNativeSize(crossPlateResolved.previewGroup)
  if (!crossPlateSize || crossPlateSize.y > 0.006 || crossPlateSize.z < 0.05) {
    return {
      name: 'structural adapter: plates stay thin and profiles stay long',
      passed: false,
      reason: `expected cross plate thickness on Y and footprint on Z, got ${crossPlateSize?.toArray()}`,
    }
  }

  return { name: 'structural adapter: plates stay thin and profiles stay long', passed: true }
}

function assertStructuralHardwareAxisAdapter(): Case {
  const lBracket = preset('structural_bracket_l_60', [60, 30, 45])
  const lBracketResolved = resolveComponentVisual({ preset: lBracket, category: 'structural' })
  const lBracketSize = generatorNativeSize(lBracketResolved.previewGroup)
  if (!lBracketSize || lBracketSize.x < 0.055 || lBracketSize.y < 0.04 || lBracketSize.z < 0.025) {
    return {
      name: 'structural adapter: brackets and supports use normalized axes',
      passed: false,
      reason: `expected L bracket width X, height Y, depth Z from bbox, got ${lBracketSize?.toArray()}`,
    }
  }

  const uBracket = preset('structural_bracket_u_50', [50, 30, 40])
  const uBracketResolved = resolveComponentVisual({ preset: uBracket, category: 'structural' })
  const uBracketSize = generatorNativeSize(uBracketResolved.previewGroup)
  if (!uBracketSize || uBracketSize.x < 0.045 || uBracketSize.y < 0.035 || uBracketSize.z < 0.025) {
    return {
      name: 'structural adapter: brackets and supports use normalized axes',
      passed: false,
      reason: `expected U bracket width X, height Y, depth Z from bbox, got ${uBracketSize?.toArray()}`,
    }
  }

  const pillowBlock = preset('structural_pillow_block', [60, 30, 40])
  const pillowResolved = resolveComponentVisual({ preset: pillowBlock, category: 'structural' })
  const pillowSize = generatorNativeSize(pillowResolved.previewGroup)
  if (!pillowSize || pillowSize.y < 0.028 || pillowSize.z < 0.025) {
    return {
      name: 'structural adapter: brackets and supports use normalized axes',
      passed: false,
      reason: `expected pillow block height Y and bearing depth Z from bbox, got ${pillowSize?.toArray()}`,
    }
  }

  const carriage = preset('structural_linear_rail_carriage', [45, 35, 12])
  const carriageResolved = resolveComponentVisual({ preset: carriage, category: 'structural' })
  const carriageSize = generatorNativeSize(carriageResolved.previewGroup)
  if (!carriageSize || carriageSize.y > 0.016 || carriageSize.z < 0.03) {
    return {
      name: 'structural adapter: brackets and supports use normalized axes',
      passed: false,
      reason: `expected rail carriage low height Y and longer depth Z from bbox, got ${carriageSize?.toArray()}`,
    }
  }

  const roundTube = preset('structural_cf_tube_round', [12, 12, 200])
  const roundTubeResolved = resolveComponentVisual({ preset: roundTube, category: 'structural' })
  const roundTubeSize = generatorNativeSize(roundTubeResolved.previewGroup)
  if (!roundTubeSize || roundTubeSize.z < 0.18 || roundTubeSize.y > 0.02) {
    return {
      name: 'structural adapter: brackets and supports use normalized axes',
      passed: false,
      reason: `expected round tube length on Z and compact cross-section on Y, got ${roundTubeSize?.toArray()}`,
    }
  }

  const gusset = preset('structural_gusset_plate', [60, 40, 4])
  const gussetResolved = resolveComponentVisual({ preset: gusset, category: 'structural' })
  const gussetSize = generatorNativeSize(gussetResolved.previewGroup)
  if (!gussetSize || gussetSize.y > 0.008 || gussetSize.z < 0.035) {
    return {
      name: 'structural adapter: brackets and supports use normalized axes',
      passed: false,
      reason: `expected gusset thickness on Y and footprint depth on Z, got ${gussetSize?.toArray()}`,
    }
  }

  const standoff = preset('structural_standoff_m3', [5.5, 5.5, 15])
  const standoffResolved = resolveComponentVisual({ preset: standoff, category: 'structural' })
  const standoffSize = generatorNativeSize(standoffResolved.previewGroup)
  if (!standoffSize || standoffSize.y < 0.014 || standoffSize.x > 0.008 || standoffSize.z > 0.008) {
    return {
      name: 'structural adapter: brackets and supports use normalized axes',
      passed: false,
      reason: `expected standoff length on Y with compact hex footprint, got ${standoffSize?.toArray()}`,
    }
  }

  return { name: 'structural adapter: brackets and supports use normalized axes', passed: true }
}

function assertSensorBoxAxisAdapter(): Case {
  const ultrasonic = preset('sensor_ultrasonic', [45, 20, 15])
  markMeshLoadInProgress(ultrasonic.id)
  const resolved = resolveComponentVisual({ preset: ultrasonic, category: 'sensors' })
  clearMeshLoadInProgress(ultrasonic.id)

  const size = generatorNativeSize(resolved.previewGroup)
  if (resolved.source !== 'rich') {
    return {
      name: 'sensor adapter: box fallback remains rich source',
      passed: false,
      reason: `expected rich source, got ${resolved.source}`,
    }
  }
  if (!size) {
    return { name: 'sensor adapter: box sensor has measurable bounds', passed: false, reason: 'missing preview group size' }
  }
  if (size.y > 0.018 || size.z < 0.018) {
    return {
      name: 'sensor adapter: box bbox z maps to height and bbox y maps to depth',
      passed: false,
      reason: `expected ultrasonic height Y and depth Z to follow bbox, got ${size.toArray()}`,
    }
  }

  const limitSwitch = preset('sensor_limit_switch', [20, 10, 6])
  markMeshLoadInProgress(limitSwitch.id)
  const switchResolved = resolveComponentVisual({ preset: limitSwitch, category: 'sensors' })
  clearMeshLoadInProgress(limitSwitch.id)
  const switchSize = generatorNativeSize(switchResolved.previewGroup)
  if (!switchSize || switchSize.y > 0.008 || switchSize.z < 0.009) {
    return {
      name: 'sensor adapter: box bbox z maps to height and bbox y maps to depth',
      passed: false,
      reason: `expected limit switch height Y and depth Z to follow bbox, got ${switchSize?.toArray()}`,
    }
  }

  const thermal = preset('sensor_thermal_ir_camera', [30, 22, 10])
  const thermalResolved = resolveComponentVisual({ preset: thermal, category: 'sensors' })
  const thermalSize = generatorNativeSize(thermalResolved.previewGroup)
  if (!thermalSize || thermalSize.y > 0.012 || thermalSize.z < 0.019) {
    return {
      name: 'sensor adapter: box bbox z maps to height and bbox y maps to depth',
      passed: false,
      reason: `expected thermal camera height Y and depth Z to follow bbox, got ${thermalSize?.toArray()}`,
    }
  }

  const bumper = preset('sensor_contact_bumper', [28, 16, 10])
  const bumperResolved = resolveComponentVisual({ preset: bumper, category: 'sensors' })
  const bumperSize = generatorNativeSize(bumperResolved.previewGroup)
  if (!bumperSize || bumperSize.y > 0.014 || bumperSize.z < 0.014) {
    return {
      name: 'sensor adapter: box bbox z maps to height and bbox y maps to depth',
      passed: false,
      reason: `expected contact bumper height Y and depth Z to follow bbox, got ${bumperSize?.toArray()}`,
    }
  }

  return { name: 'sensor adapter: box bbox z maps to height and bbox y maps to depth', passed: true }
}

function assertSensorPcbAxisAdapter(): Case {
  const tof = preset('sensor_tof', [13, 18, 2])
  markMeshLoadInProgress(tof.id)
  const tofResolved = resolveComponentVisual({ preset: tof, category: 'sensors' })
  clearMeshLoadInProgress(tof.id)

  const tofSize = generatorNativeSize(tofResolved.previewGroup)
  if (tofResolved.source !== 'rich') {
    return {
      name: 'sensor adapter: PCB bbox z maps to board thickness',
      passed: false,
      reason: `expected rich source, got ${tofResolved.source}`,
    }
  }
  if (!tofSize) {
    return { name: 'sensor adapter: PCB bbox z maps to board thickness', passed: false, reason: 'missing ToF size' }
  }
  if (tofSize.y > 0.005 || tofSize.z < 0.015) {
    return {
      name: 'sensor adapter: PCB bbox z maps to board thickness',
      passed: false,
      reason: `expected ToF board thickness on Y and board depth on Z, got ${tofSize.toArray()}`,
    }
  }

  const current = preset('sensor_current', [30, 20, 12])
  const currentResolved = resolveComponentVisual({ preset: current, category: 'sensors' })
  const currentSize = generatorNativeSize(currentResolved.previewGroup)
  if (!currentSize || currentSize.y > 0.015 || currentSize.z < 0.018) {
    return {
      name: 'sensor adapter: PCB bbox z maps to board thickness',
      passed: false,
      reason: `expected current sensor thickness on Y and board depth on Z, got ${currentSize?.toArray()}`,
    }
  }

  const color = preset('sensor_color_light', [18, 12, 3])
  const colorResolved = resolveComponentVisual({ preset: color, category: 'sensors' })
  const colorSize = generatorNativeSize(colorResolved.previewGroup)
  if (!colorSize || colorSize.y > 0.006 || colorSize.z < 0.01) {
    return {
      name: 'sensor adapter: PCB bbox z maps to board thickness',
      passed: false,
      reason: `expected color sensor thickness on Y and board depth on Z, got ${colorSize?.toArray()}`,
    }
  }

  const barometer = preset('sensor_barometric_pressure', [16, 12, 3])
  const barometerResolved = resolveComponentVisual({ preset: barometer, category: 'sensors' })
  const barometerSize = generatorNativeSize(barometerResolved.previewGroup)
  if (!barometerSize || barometerSize.y > 0.006 || barometerSize.z < 0.01) {
    return {
      name: 'sensor adapter: PCB bbox z maps to board thickness',
      passed: false,
      reason: `expected barometric sensor thickness on Y and board depth on Z, got ${barometerSize?.toArray()}`,
    }
  }

  return { name: 'sensor adapter: PCB bbox z maps to board thickness', passed: true }
}

function assertSensorMechanicalAxisAdapter(): Case {
  const depthCamera = preset('sensor_depth_camera_small', [85, 25, 20])
  markMeshLoadInProgress(depthCamera.id)
  const depthResolved = resolveComponentVisual({ preset: depthCamera, category: 'sensors' })
  clearMeshLoadInProgress(depthCamera.id)

  const depthSize = generatorNativeSize(depthResolved.previewGroup)
  if (depthResolved.source !== 'rich') {
    return {
      name: 'sensor adapter: mechanical sensors use normalized axes',
      passed: false,
      reason: `expected depth-camera rich source, got ${depthResolved.source}`,
    }
  }
  if (!depthSize || depthSize.y > 0.026 || depthSize.z < 0.026) {
    return {
      name: 'sensor adapter: mechanical sensors use normalized axes',
      passed: false,
      reason: `expected depth-camera height Y and depth Z to follow bbox, got ${depthSize?.toArray()}`,
    }
  }

  const loadCell = preset('sensor_load_cell', [55, 18, 8])
  markMeshLoadInProgress(loadCell.id)
  const loadCellResolved = resolveComponentVisual({ preset: loadCell, category: 'sensors' })
  clearMeshLoadInProgress(loadCell.id)

  const loadCellSize = generatorNativeSize(loadCellResolved.previewGroup)
  if (!loadCellSize || loadCellSize.y > 0.011 || loadCellSize.z < 0.016) {
    return {
      name: 'sensor adapter: mechanical sensors use normalized axes',
      passed: false,
      reason: `expected load-cell thickness Y and beam depth Z to follow bbox, got ${loadCellSize?.toArray()}`,
    }
  }

  const encoder = preset('sensor_joint_encoder_absolute', [22, 22, 10])
  markMeshLoadInProgress(encoder.id)
  const encoderResolved = resolveComponentVisual({ preset: encoder, category: 'sensors' })
  clearMeshLoadInProgress(encoder.id)

  const encoderSize = generatorNativeSize(encoderResolved.previewGroup)
  if (!encoderSize || encoderSize.y < 0.004 || encoderSize.y > 0.014) {
    return {
      name: 'sensor adapter: mechanical sensors use normalized axes',
      passed: false,
      reason: `expected encoder axial height Y to follow bbox Z, got ${encoderSize?.toArray()}`,
    }
  }

  return { name: 'sensor adapter: mechanical sensors use normalized axes', passed: true }
}

function assertLogicalAndVisualBoundsAgree(): Case {
  // Components whose rich generators or primitive descriptors honour the
  // declared bbox envelope. compute_usb_hub and a handful of other generators
  // overshoot their bbox today; Phase 5 mesh-extent CI is the gate that
  // catches those, not this Phase 1 parity check.
  const cases: Array<{ id: string; bbox: [number, number, number]; cat: string }> = [
    { id: 'actuator_servo_standard', bbox: [40, 20, 37], cat: 'actuators' },
    { id: 'structural_baseplate', bbox: [200, 150, 8], cat: 'structural' },
    { id: 'unregistered_plain_box', bbox: [30, 20, 10], cat: 'misc' },
  ]
  for (const c of cases) {
    const p = preset(c.id, c.bbox)
    const visual = resolveComponentVisual({ preset: p, category: c.cat })
    const logical = resolveComponent({ spec: p, category: c.cat })
    const lhx = logical.bounds.half[0] / 1000
    const lhy = logical.bounds.half[1] / 1000
    const lhz = logical.bounds.half[2] / 1000
    // Logical bounds is the bbox envelope; visual bounds is the AABB of the
    // primitive descriptors after RPY-bake. Generators may overshoot the
    // envelope slightly — Phase 5 is the gate that fails CI at >5%. Match
    // that threshold here so the parity check enforces the same contract.
    const tol = Math.max(lhx, lhy, lhz) * 0.05
    if (!approx(visual.bounds.hx, lhx, tol) || !approx(visual.bounds.hy, lhy, tol) || !approx(visual.bounds.hz, lhz, tol)) {
      return {
        name: 'bounds parity: logical resolver and visual resolver agree on envelope',
        passed: false,
        reason: `${c.id}: visual=[${visual.bounds.hx},${visual.bounds.hy},${visual.bounds.hz}] vs logical=[${lhx},${lhy},${lhz}] (tol=${tol})`,
      }
    }
  }
  return { name: 'bounds parity: logical resolver and visual resolver agree on envelope', passed: true }
}

function assertCollisionUsesMeasuredMeshExtent(): Case {
  // Bbox-as-source-of-truth: collision.bounds derives from the spec bbox, not
  // from the measured OBJ extent. The OBJ stays as a build-time measurement
  // (and could legitimately differ from the spec) but no longer feeds runtime
  // collision geometry. This test confirms the new contract — the measured
  // mesh entry is intentionally provided here AND ignored.
  setMeshExtentsCatalog({
    components: {
      compute_mcu_small: {
        declared_bbox_mm: [51, 21, 5],
        collision: {
          file: 'compute_mcu_collision.obj',
          extent_mm: [49.8, 20.4, 4.6],
          center_mm: [0.1, 0, -0.05],
          vertex_count: 12,
        },
      },
    },
  })
  try {
    const withMesh = preset('compute_mcu_small', [51, 21, 5], 'compute_mcu_collision.obj')
    const visual = resolveComponentVisual({ preset: withMesh, category: 'compute' })
    const logical = resolveComponent({ spec: withMesh, category: 'compute' })

    if (visual.collision.source !== 'authored_mesh' || logical.collision.source !== 'authored_mesh') {
      return {
        name: 'collision parity: bbox populates collision.bounds (not measured OBJ)',
        passed: false,
        reason: `expected authored_mesh on both, got visual=${visual.collision.source} logical=${logical.collision.source}`,
      }
    }
    // Visual side is in meters: bbox [51,21,5] mm → halves [0.0255, 0.0105, 0.0025] m
    if (!approx(visual.collision.bounds.hx, 0.0255, 1e-4) || !approx(visual.collision.bounds.hy, 0.0105, 1e-4)) {
      return {
        name: 'collision parity: bbox populates collision.bounds (not measured OBJ)',
        passed: false,
        reason: `visual collision half from bbox expected [0.0255,0.0105,0.0025], got [${visual.collision.bounds.hx},${visual.collision.bounds.hy},${visual.collision.bounds.hz}]`,
      }
    }
    // Logical side is in mm.
    const lhx = logical.collision.bounds.half[0]
    const lhy = logical.collision.bounds.half[1]
    if (!approx(lhx, 25.5, 1e-2) || !approx(lhy, 10.5, 1e-2)) {
      return {
        name: 'collision parity: bbox populates collision.bounds (not measured OBJ)',
        passed: false,
        reason: `logical collision half (mm) expected [25.5,10.5,2.5], got [${lhx},${lhy},${logical.collision.bounds.half[2]}]`,
      }
    }
    return { name: 'collision parity: bbox populates collision.bounds (not measured OBJ)', passed: true }
  } finally {
    setMeshExtentsCatalog(null)
  }
}

function assertCollisionResolverContract(): Case {
  const withMesh = preset('compute_mcu_small', [51, 21, 5], 'compute_mcu_collision.obj')
  const meshResolved = resolveComponentVisual({ preset: withMesh, category: 'compute' })
  if (meshResolved.collision.source !== 'authored_mesh' || meshResolved.collision.meshFile !== 'compute_mcu_collision.obj') {
    return {
      name: 'collision resolver: authored collision mesh is explicit',
      passed: false,
      reason: `expected authored mesh, got ${JSON.stringify(meshResolved.collision)}`,
    }
  }

  const fallback = preset('unregistered_collision_box', [30, 20, 10])
  const fallbackResolved = resolveComponentVisual({ preset: fallback, category: 'misc' })
  if (fallbackResolved.collision.source !== 'urdf_primitives') {
    return {
      name: 'collision resolver: primitive collision fallback is explicit',
      passed: false,
      reason: `expected urdf_primitives, got ${JSON.stringify(fallbackResolved.collision)}`,
    }
  }

  return { name: 'collision resolver: collision source is explicit', passed: true }
}

/** Simulate the live commit→reparse→render round-trip and assert the placed
 *  visual lands in the same world orientation as the carry ghost.
 *
 *  This is the gap that let the basis-conjugation bug ship: every other parity
 *  test compares carry world AABB *size* (rotation-invariant for boxes), or
 *  resolver outputs (no commit math at all). None of them simulate the matrix
 *  product `parentWorldInv * ghostWorld` that `commitCarry` actually persists,
 *  so a missing right-side basis swap was invisible to CI.
 *
 *  The test marker is a small box offset along the authored +Z axis: rotation-
 *  asymmetric, so a 90°X bake (the "flat tire" failure mode) shifts the marker
 *  off-axis in scene world. Tolerance is sub-µm; any frame mismatch surfaces
 *  immediately. */
function assertCarryCommitRenderRoundTrip(
  name: string,
  authoredFrame: 'z_up' | 'y_up',
  carryQuat: THREE.Quaternion,
): Case {
  // Marker offset chosen to be sensitive to any 90° rotation about any axis.
  const MARKER_LOCAL = new THREE.Vector3(0.011, 0.022, 0.033)
  const MARKER_OFFSET_TOL = 1e-9

  // Shared previewGroup factory: same content for both carry and render paths,
  // so any divergence is purely a frame-math bug.
  const makePreview = () => {
    const group = new THREE.Group()
    const marker = new THREE.Object3D()
    marker.position.copy(MARKER_LOCAL)
    marker.name = 'marker'
    group.add(marker)
    return group
  }

  // Carry path: scene → carryGroup → ghost(componentVisualWorldQuat scene_y_up) → preview → marker.
  const sceneRoot = new THREE.Group()
  const carryGroup = new THREE.Group()
  carryGroup.quaternion.copy(carryQuat)
  carryGroup.position.set(0.45, 0.18, -0.27)
  sceneRoot.add(carryGroup)
  const ghost = makePreview()
  ghost.quaternion.copy(componentVisualWorldQuat(authoredFrame, 'scene_y_up'))
  carryGroup.add(ghost)
  sceneRoot.updateMatrixWorld(true)
  const carryMarker = ghost.getObjectByName('marker')!
  const carryMarkerWorld = new THREE.Vector3().setFromMatrixPosition(carryMarker.matrixWorld)

  // Commit path (the FIXED version): childLocal = parentInv * ghostWorld * URDF_TO_SCENE_Q.
  // Set up a non-trivial parent so the test exercises the full conjugation,
  // not just the worldGroup-only case where parent_local = identity.
  const renderRoot = new THREE.Group()
  const worldGroup = new THREE.Group()
  worldGroup.rotation.x = -Math.PI / 2 // matches main.ts:1144
  renderRoot.add(worldGroup)
  const parentLink = new THREE.Group()
  parentLink.position.set(0.05, -0.04, 0.12)
  parentLink.quaternion.setFromEuler(new THREE.Euler(0.3, -0.5, 0.2, 'XYZ'))
  worldGroup.add(parentLink)
  renderRoot.updateMatrixWorld(true)

  const URDF_TO_SCENE_M = new THREE.Matrix4().makeRotationFromQuaternion(URDF_TO_SCENE_Q)
  const ghostWorld = carryGroup.matrixWorld.clone()
  const parentInv = parentLink.matrixWorld.clone().invert()
  const childLocal = parentInv.clone().multiply(ghostWorld).multiply(URDF_TO_SCENE_M)

  // Reparse + applyRichVisuals: linkGroup parented under parent in URDF Z-up,
  // richGroup quaternion = componentVisualWorldQuat(_, urdf_z_up) = identity.
  const linkGroup = new THREE.Group()
  childLocal.decompose(linkGroup.position, linkGroup.quaternion, linkGroup.scale)
  parentLink.add(linkGroup)
  const richGroup = makePreview()
  richGroup.quaternion.copy(componentVisualWorldQuat(authoredFrame, 'urdf_z_up'))
  linkGroup.add(richGroup)
  renderRoot.updateMatrixWorld(true)
  const renderMarker = richGroup.getObjectByName('marker')!
  const renderMarkerWorld = new THREE.Vector3().setFromMatrixPosition(renderMarker.matrixWorld)

  const drift = renderMarkerWorld.distanceTo(carryMarkerWorld)
  if (drift > MARKER_OFFSET_TOL) {
    return {
      name,
      passed: false,
      reason: `carry marker ${carryMarkerWorld.toArray().map(n => n.toFixed(6)).join(',')} ` +
        `vs render marker ${renderMarkerWorld.toArray().map(n => n.toFixed(6)).join(',')} ` +
        `(drift=${(drift * 1000).toFixed(4)}mm)`,
    }
  }
  return { name, passed: true }
}

function main(): void {
  const results: Case[] = []

  results.push(assertUnifiedFrameAdapter())
  // Carry-commit-render round-trip (Phase 3 basis-conjugation regression).
  // Several carryGroup orientations: identity, R-key snaps (90°/45° on each
  // axis), and a non-axis-aligned tilt. All must round-trip exactly.
  for (const [label, q] of [
    ['identity', new THREE.Quaternion()],
    ['snap +90Y', new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI / 2)],
    ['snap +45Z', new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 4)],
    ['snap -90X', new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI / 2)],
    ['tilt 0.3 0.5 0.2', new THREE.Quaternion().setFromEuler(new THREE.Euler(0.3, 0.5, 0.2, 'XYZ'))],
  ] as const) {
    results.push(assertCarryCommitRenderRoundTrip(`carry-commit-render round-trip (z_up authored, ${label})`, 'z_up', q))
  }
  results.push(assertResolvedConnectorsAndPorts())
  results.push(assertPrimitiveBoundsBakeRpy())
  results.push(assertTargetEnvelopeFields())
  results.push(assertCarryConnectorSnapMath())
  results.push(assertMobilityWheelAxisAdapter())
  results.push(assertMobilityTrackAxisAdapter())
  results.push(assertMotorGeneratorAxisAdapter())
  results.push(assertActuatorGeneratorAxisAdapter())
  results.push(assertSplitServoResolverContract())
  results.push(assertTransmissionLongFootprintAxisAdapter())
  results.push(assertTransmissionFlatHardwareAxisAdapter())
  results.push(assertTransmissionAngleGearAxisAdapter())
  results.push(assertTransmissionShaftHardwareAxisAdapter())
  results.push(assertEndEffectorAxisAdapter())
  results.push(assertStructuralAxisAdapter())
  results.push(assertStructuralHardwareAxisAdapter())
  results.push(assertSensorBoxAxisAdapter())
  results.push(assertSensorPcbAxisAdapter())
  results.push(assertSensorMechanicalAxisAdapter())

  const wheel = preset('mobility_wheel_driven', [100, 100, 48])
  setCachedMeshGroup(wheel.id, makeRawMeshGroup([0.1, 0.05, 0.1]))
  results.push(assertResolvedParity(
    'cached mesh: carry and render both choose prepared mesh',
    wheel,
    'mobility',
    { source: 'mesh', status: 'ready', hasPreviewGroup: true },
  ))

  const lipo = preset('power_lipo_3s_2200', [105, 34, 24])
  markMeshLoadInProgress(lipo.id)
  results.push(assertResolvedParity(
    'loading mesh: carry and render both choose rich fallback',
    lipo,
    'power',
    { source: 'rich', status: 'loading', hasPreviewGroup: true },
  ))
  clearMeshLoadInProgress(lipo.id)
  results.push(assertPowerGeneratorAxisAdapter())
  results.push(assertComputeGeneratorAxisAdapter())
  results.push(assertMobilityUprightAxisAdapter())
  results.push(assertCollisionResolverContract())
  results.push(assertLogicalAndVisualBoundsAgree())
  results.push(assertCollisionUsesMeasuredMeshExtent())

  const sbc = preset('compute_sbc_small', [85, 56, 17])
  results.push(assertResolvedParity(
    'uncached mesh override: carry and render both choose rich fallback',
    sbc,
    'compute',
    { source: 'rich', status: 'fallback', hasPreviewGroup: true },
  ))

  const baseplate = preset('structural_baseplate', [200, 150, 8])
  results.push(assertResolvedParity(
    'no mesh override: carry and render both choose rich generator',
    baseplate,
    'structural',
    { source: 'rich', status: 'ready', hasPreviewGroup: true },
  ))

  const plainBox = preset('unregistered_plain_box', [30, 20, 10])
  results.push(assertResolvedParity(
    'no generator: carry and render both choose URDF primitives',
    plainBox,
    'misc',
    { source: 'urdf_primitives', status: 'ready', hasPreviewGroup: false },
  ))

  // Per-component carry/render world-orientation parity. Covers the
  // categories most prone to frame drift: rich-source generators authored in
  // Y-up (must round-trip through the adapter twice and land identical) and
  // mesh-source components authored in URDF Z-up (carry now flips to scene
  // Y-up via the same adapter). Wheels, motors, and cylinder-axis sensors
  // are the historical residue this is meant to catch.
  const parityCases: Array<{ id: string; bbox: [number, number, number]; cat: string; cached?: [number, number, number] }> = [
    { id: 'mobility_wheel_driven', bbox: [100, 100, 30], cat: 'mobility' },
    { id: 'mobility_mecanum_wheel', bbox: [100, 100, 48], cat: 'mobility' },
    { id: 'mobility_caster_wheel', bbox: [50, 50, 65], cat: 'mobility' },
    { id: 'motor_gear_medium_37mm', bbox: [37, 37, 70], cat: 'motors' },
    { id: 'actuator_servo_standard', bbox: [40, 20, 37], cat: 'actuators' },
    { id: 'actuator_bldc_small', bbox: [76, 76, 48], cat: 'actuators' },
    { id: 'actuator_stepper_nema17', bbox: [42.3, 42.3, 48], cat: 'actuators' },
    { id: 'sensor_ultrasonic', bbox: [45, 20, 15], cat: 'sensors' },
    { id: 'sensor_tof', bbox: [13, 18, 2], cat: 'sensors' },
    { id: 'effector_parallel_gripper_small', bbox: [65, 45, 90], cat: 'end_effectors' },
    { id: 'transmission_leadscrew_8mm', bbox: [8, 8, 200], cat: 'transmission' },
    { id: 'transmission_bevel_gear_pair', bbox: [35, 35, 35], cat: 'transmission' },
    { id: 'structural_extrusion_2020', bbox: [20, 20, 200], cat: 'structural' },
    { id: 'structural_baseplate', bbox: [200, 150, 8], cat: 'structural' },
    { id: 'compute_usb_hub', bbox: [40, 30, 8], cat: 'compute' },
    { id: 'power_lipo_3s_2200', bbox: [105, 34, 24], cat: 'power' },
    // Mesh-source case: cache a raw GLB-like group so the resolver emits source='mesh'
    // and exercises authoredFrame='z_up'. The cached wheel-driven mesh would have
    // originally hit the "flat tire" bug through the visualQuat snapshot.
    { id: 'mobility_wheel_driven', bbox: [100, 100, 30], cat: 'mobility', cached: [0.1, 0.05, 0.1] },
  ]
  for (const c of parityCases) {
    if (c.cached) {
      setCachedMeshGroup(c.id, makeRawMeshGroup(c.cached))
    } else {
      markMeshLoadInProgress(c.id)
    }
    results.push(assertCarryRenderWorldParity(`carry/render world parity: ${c.id}${c.cached ? ' (cached mesh)' : ''}`, preset(c.id, c.bbox), c.cat))
    if (!c.cached) clearMeshLoadInProgress(c.id)
  }

  const failed = results.filter(r => !r.passed)
  for (const r of results) {
    const tag = r.passed ? 'PASS' : 'FAIL'
    console.log(`[${tag}] ${r.name}`)
    if (!r.passed && r.reason) console.log(`       ${r.reason}`)
  }
  console.log(`visual parity corpus: ${results.length - failed.length}/${results.length} passed`)
  if (failed.length > 0) process.exit(1)
}

main()

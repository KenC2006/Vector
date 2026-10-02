// Component visual parity corpus: exercises the resolver contract shared by
// the assembly editor's placement ghost and committed rich-visual rendering.
//
// Run: cd src && npm run test:visual-parity
// Direct: node --experimental-strip-types src/componentVisualParityCorpus.ts

import * as THREE from 'three'
import * as fs from 'node:fs'
import * as path from 'node:path'
import { fileURLToPath } from 'node:url'
import { resolveComponentVisual } from './componentVisualResolver.ts'
import type { ComponentVisualPresetLike } from './componentVisualResolver.ts'
import { applyRichVisuals, componentVisualWorldQuat } from './richVisuals/index.ts'
import { nurbsTorus } from './richVisuals/nurbs.ts'
import { setCachedMeshGroup, markMeshLoadInProgress, clearMeshLoadInProgress } from './richVisuals/meshCache.ts'
import {
  EXPLICIT_SCALE_POLICY,
  getMeshOverrideUrl,
  MESH_OVERRIDES,
  PROCEDURAL_VISUAL_ONLY,
  ROTATION_OVERRIDES,
  SHAFT_OVERLAYS,
} from './richVisuals/meshOverrides.ts'

interface Case {
  name: string
  passed: boolean
  reason?: string
}

function preset(id: string, boundingBoxMm: [number, number, number]): ComponentVisualPresetLike {
  return { id, physical: { bounding_box_mm: boundingBoxMm }, mechanical_electrical: {} }
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

function assertResolvedParity(
  name: string,
  component: ComponentVisualPresetLike,
  category: string,
  expected: { source: string; status?: string; hasPreviewGroup: boolean },
): Case {
  const ghost = resolveComponentVisual({ preset: component, category })
  const render = resolveComponentVisual({
    preset: component,
    category,
    linkName: `${component.id}_1`,
    castShadow: true,
    receiveShadow: true,
  })

  if (ghost.source !== render.source) {
    return { name, passed: false, reason: `source mismatch: ghost=${ghost.source}, render=${render.source}` }
  }
  if (ghost.source !== expected.source) {
    return { name, passed: false, reason: `expected source ${expected.source}, got ${ghost.source}` }
  }
  if (expected.status && ghost.status !== expected.status) {
    return { name, passed: false, reason: `expected ghost status ${expected.status}, got ${ghost.status}` }
  }
  if (expected.status && render.status !== expected.status) {
    return { name, passed: false, reason: `expected render status ${expected.status}, got ${render.status}` }
  }
  if (!!ghost.previewGroup !== expected.hasPreviewGroup || !!render.previewGroup !== expected.hasPreviewGroup) {
    return {
      name,
      passed: false,
      reason: `preview presence mismatch: ghost=${!!ghost.previewGroup}, render=${!!render.previewGroup}`,
    }
  }
  if (!sameSize(groupSize(ghost.previewGroup), groupSize(render.previewGroup))) {
    return {
      name,
      passed: false,
      reason: `preview size mismatch: ghost=${groupSize(ghost.previewGroup)?.toArray()} render=${groupSize(render.previewGroup)?.toArray()}`,
    }
  }
  return { name, passed: true }
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

/**
 * Catalog-frame contract: the resolved visual, as rendered in the link's URDF
 * frame, must fill the preset bbox on each axis (within 25%, or 3mm for thin
 * dimensions). Connectors, placement and collision are all authored against
 * that bbox, so a visual lying along the wrong axis or drawn 2x too tall is a
 * placement bug on screen even when every number downstream is right.
 */
function assertCatalogFrame(name: string, cases: Array<[string, [number, number, number], string, boolean?]>): Case {
  for (const [id, bb, category, loading] of cases) {
    const p = preset(id, bb)
    if (loading) markMeshLoadInProgress(p.id)
    const resolved = resolveComponentVisual({ preset: p, category })
    if (loading) clearMeshLoadInProgress(p.id)
    const size = groupSize(resolved.previewGroup)
    if (!size) return { name, passed: false, reason: `${id}: no measurable preview group` }
    const got = size.toArray().map(v => v * 1000)
    for (let i = 0; i < 3; i++) {
      if (Math.abs(got[i] - bb[i]) > Math.max(0.25 * bb[i], 3)) {
        return { name, passed: false, reason: `${id}: rendered ${got.map(v => v.toFixed(1)).join('x')}mm vs bbox ${bb.join('x')}mm` }
      }
    }
  }
  return { name, passed: true }
}

function assertMobilityWheelAxisAdapter(): Case {
  return assertCatalogFrame('mobility adapter: wheel bbox z maps to axle thickness', [
    ['mobility_wheel_driven', [100, 100, 30], 'mobility', true],
    ['mobility_mecanum_wheel', [100, 100, 48], 'mobility', true],
  ])
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
  return assertCatalogFrame('actuator adapter: normalized bbox axes stay direct', [
    ['actuator_servo_standard', [40, 20, 37], 'actuators', true],
    ['actuator_bldc_small', [76, 76, 48], 'actuators'],
    ['actuator_stepper_nema17', [42.3, 42.3, 48], 'actuators', true],
    ['actuator_linear_small', [16, 16, 130], 'actuators', true],
  ])
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
  if (!wormSize || wormSize.z < 0.022 || wormSize.y > 0.035 || wormSize.x > 0.04 + 1e-6) {
    return {
      name: 'transmission adapter: bevel and worm gears keep 3D footprint',
      passed: false,
      reason: `expected worm gear set to keep worm length on Z and compact Y, got ${wormSize?.toArray()}`,
    }
  }

  return { name: 'transmission adapter: bevel and worm gears keep 3D footprint', passed: true }
}

function assertTransmissionShaftHardwareAxisAdapter(): Case {
  return assertCatalogFrame('transmission adapter: shaft hardware keeps axial length on Z', [
    ['transmission_leadscrew_8mm', [8, 8, 200], 'transmission'],
    ['transmission_flexible_coupling_jaw', [20, 20, 30], 'transmission', true],
    ['transmission_universal_joint', [18, 18, 35], 'transmission'],
    ['transmission_planetary_gearbox', [42.3, 42.3, 38], 'transmission', true],
  ])
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
  return assertCatalogFrame('structural adapter: plates stay thin and profiles stay long', [
    ['structural_extrusion_2020', [20, 20, 200], 'structural', true],
    ['structural_angle_aluminum_25x25', [25, 25, 200], 'structural'],
    ['structural_baseplate', [200, 150, 8], 'structural'],
    ['structural_limb_link_slim', [14, 6, 100], 'structural'],
    ['structural_linear_rail_mgn12', [12, 8, 200], 'structural', true],
    ['structural_cross_plate', [60, 60, 3], 'structural'],
  ])
}

function assertStructuralHardwareAxisAdapter(): Case {
  return assertCatalogFrame('structural adapter: brackets and supports use normalized axes', [
    ['structural_bracket_l_60', [60, 30, 45], 'structural'],
    ['structural_bracket_u_50', [50, 30, 40], 'structural'],
    ['structural_pillow_block', [60, 30, 40], 'structural'],
    ['structural_linear_rail_carriage', [45, 35, 12], 'structural'],
    ['structural_cf_tube_round', [12, 12, 200], 'structural'],
    ['structural_gusset_plate', [60, 40, 4], 'structural'],
    ['structural_standoff_m3', [5.5, 5.5, 15], 'structural'],
  ])
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

const SRC_ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..')

function loadCatalog(): Array<{ preset: ComponentVisualPresetLike; category: string }> {
  const data = JSON.parse(fs.readFileSync(path.resolve(SRC_ROOT, '..', 'core', 'presets', 'generic_presets.json'), 'utf-8')) as {
    categories: Record<string, { components: ComponentVisualPresetLike[] }>
  }
  const out: Array<{ preset: ComponentVisualPresetLike; category: string }> = []
  for (const [category, cat] of Object.entries(data.categories)) {
    for (const preset of cat.components) out.push({ preset, category })
  }
  return out
}

function catalogPreset(id: string): { preset: ComponentVisualPresetLike; category: string } {
  const hit = loadCatalog().find(c => c.preset.id === id)
  if (!hit) throw new Error(`catalog id ${id} not found`)
  return hit
}

function worldMeshes(group: THREE.Group): THREE.Mesh[] {
  group.updateMatrixWorld(true)
  const out: THREE.Mesh[] = []
  group.traverse(o => { if (o instanceof THREE.Mesh) out.push(o) })
  return out
}

/** XY footprint (m^2) of the vertices within the lowest / highest 15% of Z. */
function endFootprints(group: THREE.Group): { bottom: number; top: number } {
  const pts: THREE.Vector3[] = []
  for (const m of worldMeshes(group)) {
    const pos = m.geometry.getAttribute('position')
    for (let i = 0; i < pos.count; i++) pts.push(new THREE.Vector3().fromBufferAttribute(pos, i).applyMatrix4(m.matrixWorld))
  }
  const zs = pts.map(p => p.z)
  const lo = Math.min(...zs), hi = Math.max(...zs), band = 0.15 * (hi - lo)
  const area = (inBand: (z: number) => boolean) => {
    const box = new THREE.Box3()
    for (const p of pts) if (inBand(p.z)) box.expandByPoint(p)
    return box.isEmpty() ? 0 : (box.max.x - box.min.x) * (box.max.y - box.min.y)
  }
  return { bottom: area(z => z <= lo + band), top: area(z => z >= hi - band) }
}

/** Z of the mounting flange (flangePlate: the only untinted anodized-aluminum disc). */
function flangeCenterZ(group: THREE.Group): number | null {
  const box = new THREE.Box3()
  for (const m of worldMeshes(group)) {
    const mat = m.material as THREE.MeshStandardMaterial
    if (mat.color?.getHex() === 0x8899aa) box.expandByObject(m)
  }
  return box.isEmpty() ? null : box.getCenter(new THREE.Vector3()).z
}

/** Procedural generators must come out the way up the catalog connectors say
 *  (these used to be patched with a pi flip table in the resolver): boards
 *  carry their parts on +Z with the PCB at the bottom, tool heads mount on
 *  -Z (mount_back), the ball transfer unit mounts on +Z (mount_top). */
function assertProceduralOrientation(): Case {
  const name = 'procedural orientation: generators match catalog connectors'
  const boardsUp = [
    'compute_mcu_small', 'compute_sbc_small', 'compute_sbc_gpu', 'compute_motor_driver_dual',
    'compute_foc_controller', 'compute_fpga_dev_board', 'compute_can_transceiver', 'compute_usb_hub',
    'compute_wireless_24ghz', 'compute_lora_radio', 'compute_gps_gnss',
    'sensor_imu_6dof', 'sensor_tof', 'sensor_current', 'sensor_voltage_divider',
  ]
  for (const id of boardsUp) {
    const { preset: p, category } = catalogPreset(id)
    const g = resolveComponentVisual({ preset: p, category }).previewGroup
    if (!g) return { name, passed: false, reason: `${id}: no rich preview` }
    const f = endFootprints(g)
    if (!(f.bottom > f.top)) return { name, passed: false, reason: `${id}: board footprint bottom=${f.bottom} top=${f.top} (parts should face +Z)` }
  }
  const flangeSide: Array<[string, -1 | 1]> = [
    ['effector_magnetic_tool', -1], ['effector_tool_changer', -1], ['effector_soft_gripper', -1],
    ['effector_vacuum_pad_array', -1], ['effector_welding_torch_holder', -1], ['effector_screwdriver_holder', -1],
    ['effector_3finger_adaptive', -1], ['effector_suction_cup', -1], ['mobility_ball_transfer_unit', 1],
  ]
  for (const [id, sign] of flangeSide) {
    const { preset: p, category } = catalogPreset(id)
    const g = resolveComponentVisual({ preset: p, category }).previewGroup
    const z = g ? flangeCenterZ(g) : null
    if (z === null) return { name, passed: false, reason: `${id}: no mounting flange found` }
    if (Math.sign(z) !== sign) return { name, passed: false, reason: `${id}: flange at z=${(z * 1000).toFixed(1)}mm, expected ${sign < 0 ? '-Z' : '+Z'}` }
  }
  // Color sensor: like every board, mount_back is -Z and the optics face +Z,
  // so the PCB (the widest slab) sits at the -Z mount side.
  const { preset: color, category } = catalogPreset('sensor_color_light')
  const cg = resolveComponentVisual({ preset: color, category }).previewGroup!
  const pcb = worldMeshes(cg).find(m => m.geometry.type === 'ExtrudeGeometry')
  const pcbZ = pcb ? new THREE.Box3().setFromObject(pcb).getCenter(new THREE.Vector3()).z : null
  if (pcbZ === null || pcbZ >= 0) return { name, passed: false, reason: `sensor_color_light: PCB at z=${pcbZ}, expected the -Z mount side` }
  return { name, passed: true }
}

/** Mesh override tables: every mapped GLB ships in public/, rejected meshes
 *  are never mapped, and per-mesh tweaks only name mesh-rendered parts. */
function assertMeshOverrideTables(): Case {
  const name = 'mesh overrides: tables are consistent and every GLB ships'
  for (const id of Object.keys(MESH_OVERRIDES)) {
    if (PROCEDURAL_VISUAL_ONLY.has(id)) return { name, passed: false, reason: `${id} is both mesh-rendered and procedural-only` }
    const url = getMeshOverrideUrl(id)!
    if (!fs.existsSync(path.resolve(SRC_ROOT, 'public', url.slice(1)))) return { name, passed: false, reason: `${id}: missing ${url}` }
  }
  for (const [table, ids] of [
    ['rotationOverrides', Object.keys(ROTATION_OVERRIDES)],
    ['scalePolicy', Object.keys(EXPLICIT_SCALE_POLICY)],
    ['shaftOverlays', Object.keys(SHAFT_OVERLAYS)],
  ] as const) {
    for (const id of ids) {
      if (!MESH_OVERRIDES[id]) return { name, passed: false, reason: `${table}.${id} names a part that has no mesh` }
    }
  }
  return { name, passed: true }
}

/** applyRichVisuals places the replacement visual at the link's URDF
 *  <visual><origin>: xyz always, rpy when the link has a single visual (a
 *  multi-visual link is a legacy primitive decomposition whose per-primitive
 *  rpy must not rotate the whole part). */
function assertRichVisualHonoursVisualOrigin(): Case {
  const name = 'rich visuals: honour the URDF visual origin (xyz + single-visual rpy)'
  const run = (visualCount: number) => {
    const linkGroup = new THREE.Group()
    const geometryGroup = new THREE.Group()
    linkGroup.add(geometryGroup)
    for (let i = 0; i < visualCount; i++) {
      const visual = new THREE.Group()
      visual.position.set(0.01, 0.02, 0.03)
      visual.quaternion.setFromEuler(new THREE.Euler(0, 0, Math.PI / 2))
      visual.add(new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.15, 0.005), new THREE.MeshStandardMaterial()))
      geometryGroup.add(visual)
    }
    applyRichVisuals(
      { group: new THREE.Group(), linkGroups: new Map([['structural_baseplate_1', linkGroup]]) },
      undefined,
      () => [200, 150, 5],
    )
    return geometryGroup.children
  }
  const single = run(1)
  const expected = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0, Math.PI / 2))
  if (single.length !== 1 || single[0].position.distanceTo(new THREE.Vector3(0.01, 0.02, 0.03)) > 1e-9 || single[0].quaternion.angleTo(expected) > 1e-9) {
    return { name, passed: false, reason: `single visual: got pos ${single[0]?.position.toArray()} quat ${single[0]?.quaternion.toArray()}` }
  }
  const multi = run(2)
  if (multi.length !== 1 || multi[0].position.distanceTo(new THREE.Vector3(0.01, 0.02, 0.03)) > 1e-9 || multi[0].quaternion.angleTo(new THREE.Quaternion()) > 1e-9) {
    return { name, passed: false, reason: `multi visual: got pos ${multi[0]?.position.toArray()} quat ${multi[0]?.quaternion.toArray()}` }
  }
  return { name, passed: true }
}

/** Every catalog part resolves to a preview that fills its bbox (within the
 *  25% alignAxesToBbox tolerance, 3mm for thin axes), authored Z-up, and the
 *  renderer's frame adapter leaves it untouched. */
function assertCatalogPreviewsFillBbox(): Case {
  const name = 'catalog previews: every part fills its bbox in the link frame'
  const identity = componentVisualWorldQuat('z_up', 'urdf_z_up')
  if (identity.angleTo(new THREE.Quaternion()) > 1e-12) return { name, passed: false, reason: 'urdf_z_up adapter is not identity' }
  for (const { preset: p, category } of loadCatalog()) {
    const bb = (p.physical.bounding_box_mm ?? p.physical.bbox_mm) as [number, number, number] | undefined
    if (!bb) continue // cut-to-length parts: size comes from the instance
    const resolved = resolveComponentVisual({ preset: p, category })
    if (resolved.authoredFrame !== 'z_up') return { name, passed: false, reason: `${p.id}: authoredFrame ${resolved.authoredFrame}` }
    const size = groupSize(resolved.previewGroup)
    if (!size) return { name, passed: false, reason: `${p.id}: no preview (source=${resolved.source})` }
    const got = size.toArray().map(v => v * 1000)
    for (let i = 0; i < 3; i++) {
      if (Math.abs(got[i] - bb[i]) > Math.max(0.25 * bb[i], 3)) {
        return { name, passed: false, reason: `${p.id}: rendered ${got.map(v => v.toFixed(1)).join('x')}mm vs bbox ${bb.join('x')}mm` }
      }
    }
  }
  return { name, passed: true }
}

/** nurbsTorus uses the TorusGeometry frame (ring in XY around Z), so the
 *  generators' Rx(pi/2) lays decorative rings around a Y-up body. */
function assertTorusFrame(): Case {
  const name = 'nurbsTorus: ring lies in XY like TorusGeometry'
  const g = nurbsTorus(0.02, 0.002)
  g.computeBoundingBox()
  const s = g.boundingBox!.getSize(new THREE.Vector3())
  if (!approx(s.x, 0.044, 1e-3) || !approx(s.y, 0.044, 1e-3) || !approx(s.z, 0.004, 1e-4)) {
    return { name, passed: false, reason: `torus extents ${s.toArray()}` }
  }
  return { name, passed: true }
}

function main(): void {
  const results: Case[] = []

  results.push(assertMobilityWheelAxisAdapter())
  results.push(assertMobilityTrackAxisAdapter())
  results.push(assertMotorGeneratorAxisAdapter())
  results.push(assertActuatorGeneratorAxisAdapter())
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
    'cached mesh: editor ghost and renderer both choose prepared mesh',
    wheel,
    'mobility',
    { source: 'mesh', status: 'ready', hasPreviewGroup: true },
  ))

  const lipo = preset('power_lipo_3s_2200', [105, 34, 24])
  markMeshLoadInProgress(lipo.id)
  results.push(assertResolvedParity(
    'loading mesh: editor ghost and renderer both choose rich fallback',
    lipo,
    'power',
    { source: 'rich', status: 'loading', hasPreviewGroup: true },
  ))
  clearMeshLoadInProgress(lipo.id)
  results.push(assertPowerGeneratorAxisAdapter())
  results.push(assertComputeGeneratorAxisAdapter())
  results.push(assertMobilityUprightAxisAdapter())
  results.push(assertProceduralOrientation())
  results.push(assertCatalogPreviewsFillBbox())
  results.push(assertTorusFrame())
  results.push(assertMeshOverrideTables())
  results.push(assertRichVisualHonoursVisualOrigin())

  const sbc = preset('compute_sbc_small', [85, 56, 17])
  results.push(assertResolvedParity(
    'uncached mesh override: editor ghost and renderer both choose rich fallback',
    sbc,
    'compute',
    { source: 'rich', status: 'fallback', hasPreviewGroup: true },
  ))

  const baseplate = preset('structural_baseplate', [200, 150, 8])
  results.push(assertResolvedParity(
    'no mesh override: editor ghost and renderer both choose rich generator',
    baseplate,
    'structural',
    { source: 'rich', status: 'ready', hasPreviewGroup: true },
  ))

  const plainBox = preset('unregistered_plain_box', [30, 20, 10])
  results.push(assertResolvedParity(
    'no generator: editor ghost and renderer both fall back to primitives',
    plainBox,
    'misc',
    { source: 'urdf_primitives', status: 'ready', hasPreviewGroup: false },
  ))

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

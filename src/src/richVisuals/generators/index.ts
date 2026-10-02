/**
 * Generator registry — maps component ID patterns to rich visual generators.
 */
import * as THREE from 'three'
import { generateRichActuator } from './actuators'
import { generateRichMotor } from './motors'
import { generateRichSensor } from './sensors'
import { generateRichCompute } from './compute'
import { generateRichPower } from './power'
import { generateRichStructural } from './structural'
import { generateRichTransmission } from './transmission'
import { generateRichEndEffector } from './endEffectors'
import { generateRichMobility } from './mobility'
import { findHardwareGenerator } from './hardware'

export interface ComponentVisualDims {
  x: number  // width in meters
  y: number  // depth in meters
  z: number  // height in meters
}

export interface GeneratorDims {
  x: number  // width in meters
  y: number  // height in meters
  z: number  // depth in meters
}

export type RichGenerator = (id: string, dims: ComponentVisualDims, color?: [number, number, number]) => THREE.Group
export type LegacyRichGenerator = (id: string, dims: GeneratorDims, color?: [number, number, number]) => THREE.Group
export type LegacyZUpRichGenerator = (id: string, dims: ComponentVisualDims, color?: [number, number, number]) => THREE.Group

interface RegistryEntry {
  pattern: RegExp
  generator: RichGenerator
}

function urdfDimsToLegacyGeneratorDims(dims: ComponentVisualDims): GeneratorDims {
  return { x: dims.x, y: dims.z, z: dims.y }
}

function adaptLegacyGenerator(generator: LegacyRichGenerator): RichGenerator {
  return (id, dims, color) => generator(id, urdfDimsToLegacyGeneratorDims(dims), color)
}

function centeredWrapper(child: THREE.Group): THREE.Group {
  const wrapper = new THREE.Group()
  wrapper.add(child)
  child.updateMatrixWorld(true)
  const box = new THREE.Box3().setFromObject(child)
  if (!box.isEmpty()) {
    const center = box.getCenter(new THREE.Vector3())
    child.position.sub(center)
  }
  return wrapper
}

function adaptLegacyZUpGenerator(generator: LegacyZUpRichGenerator): RichGenerator {
  return (id, dims, color) => {
    const child = generator(id, dims, color)
    // Z-up -> the registry's Y-up convention (the resolver wraps Y back to
    // Z). +PI/2 here sent +Z to -Y, i.e. every board came out upside down.
    child.rotation.x = -Math.PI / 2
    return centeredWrapper(child)
  }
}

const registry: RegistryEntry[] = [
  // Actuators
  { pattern: /^actuator_servo|^actuator_continuous|^actuator_high_speed/, generator: generateRichActuator },
  { pattern: /^actuator_bldc/, generator: generateRichActuator },
  { pattern: /^actuator_stepper/, generator: generateRichActuator },
  { pattern: /^actuator_linear|^actuator_micro_linear/, generator: generateRichActuator },

  // Motors
  { pattern: /^motor_/, generator: generateRichMotor },

  // Sensors
  { pattern: /^sensor_/, generator: generateRichSensor },

  // Compute
  { pattern: /^compute_/, generator: adaptLegacyZUpGenerator(generateRichCompute) },

  // Power
  { pattern: /^power_/, generator: adaptLegacyGenerator(generateRichPower) },

  // Structural
  { pattern: /^structural_/, generator: generateRichStructural },

  // Transmission
  { pattern: /^transmission_/, generator: generateRichTransmission },

  // End Effectors
  { pattern: /^effector_/, generator: generateRichEndEffector },

  // Mobility
  { pattern: /^mobility_/, generator: generateRichMobility },
]

/**
 * Find a rich visual generator for a given component ID.
 * Returns null if no generator matches (link keeps its URDF primitives).
 */
export function findRichGenerator(componentId: string): RichGenerator | null {
  const hardware = findHardwareGenerator(componentId)
  if (hardware) return hardware
  for (const entry of registry) {
    if (entry.pattern.test(componentId)) {
      return entry.generator
    }
  }
  return null
}

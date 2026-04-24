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

export interface GeneratorDims {
  x: number  // width in meters
  y: number  // height in meters
  z: number  // depth in meters
}

export type RichGenerator = (
  id: string,
  dims: GeneratorDims,
  color?: [number, number, number],
  subLink?: 'body' | 'output',
) => THREE.Group

interface RegistryEntry {
  pattern: RegExp
  generator: RichGenerator
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
  { pattern: /^compute_/, generator: generateRichCompute },

  // Power
  { pattern: /^power_/, generator: generateRichPower },

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
  for (const entry of registry) {
    if (entry.pattern.test(componentId)) {
      return entry.generator
    }
  }
  return null
}

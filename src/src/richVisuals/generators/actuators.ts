/**
 * Rich visual generators for actuator components.
 * Servos, BLDC outrunners, steppers, linear actuators.
 */
import * as THREE from 'three'
import type { GeneratorDims } from './index'
import { getMaterial, getTintedMaterial } from '../materials'
import { chamferedBox, chamferedCylinder, boltCircle, mountingHole, screwHead, labelRecess, flangePlate } from '../primitives'

const CAT_COLOR: [number, number, number] = [0.90, 0.49, 0.13]  // orange

function catMetal(strength = 0.3) {
  return getTintedMaterial('anodized_aluminum', ...CAT_COLOR, strength)
}

// ── Servo ─────────────────────────────────────────────────────────────────────

function generateServo(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  const isHeavy = id.includes('heavy')
  const chamfer = Math.min(w, d) * 0.06

  // Main body
  const body = new THREE.Mesh(chamferedBox(w, d, h * 0.72, chamfer), catMetal(0.35))
  g.add(body)

  // Mounting ears (flanges on sides, midway up)
  const earW = w * 1.12
  const earH = h * 0.1
  const earD = d
  const ear = new THREE.Mesh(chamferedBox(earW, earD, earH, chamfer * 0.5), catMetal(0.25))
  ear.position.y = h * 0.32
  g.add(ear)

  // Mounting holes on ears (2 per side)
  const holeR = Math.min(w, d) * 0.035
  for (const sx of [-1, 1]) {
    for (const sz of [-1, 1]) {
      const hole = mountingHole(holeR, earH * 1.1)
      hole.position.set(sx * earW * 0.42, h * 0.32, sz * d * 0.3)
      g.add(hole)
    }
  }

  // Output horn (top face)
  const hornR = Math.min(w, d) * 0.32
  const hornH = h * 0.08
  const horn = new THREE.Mesh(
    chamferedCylinder(hornR, hornH, hornH * 0.2, 32),
    getMaterial('glossy_plastic', 0xeeeeee),
  )
  horn.position.y = h * 0.4
  g.add(horn)

  // Horn bolt circle
  const hornBolts = boltCircle(hornR * 0.7, holeR * 0.6, isHeavy ? 6 : 4, hornH * 1.05)
  hornBolts.position.y = h * 0.4
  g.add(hornBolts)

  // Center screw on horn
  const screw = screwHead(holeR * 1.2, hornH * 0.5)
  screw.position.y = h * 0.45
  g.add(screw)

  // Output shaft nub
  const shaftR = Math.min(w, d) * 0.06
  const shaft = new THREE.Mesh(
    chamferedCylinder(shaftR, hornH * 1.5, shaftR * 0.15),
    getMaterial('brushed_steel'),
  )
  shaft.position.y = h * 0.48
  g.add(shaft)

  // Cable exit (bottom rear)
  const cableR = Math.min(w, d) * 0.05
  const cable = new THREE.Mesh(
    new THREE.CylinderGeometry(cableR, cableR, d * 0.15, 12),
    getMaterial('matte_plastic'),
  )
  cable.rotation.x = Math.PI / 2
  cable.position.set(0, -h * 0.32, -d * 0.45)
  g.add(cable)

  // Label recess on front face
  const label = labelRecess(w * 0.6, h * 0.35, chamfer * 0.5)
  label.position.set(0, 0, d * 0.37)
  g.add(label)

  return g
}

// ── BLDC Outrunner ───────────────────────────────────────────────────────────

function generateBLDC(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h } = dims
  const r = w / 2

  const chamfer = r * 0.04

  // Main housing cylinder
  const housing = new THREE.Mesh(
    chamferedCylinder(r, h * 0.8, chamfer, 48),
    catMetal(0.4),
  )
  g.add(housing)

  // Top ring accent
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(r * 0.95, r * 0.03, 8, 48),
    getMaterial('dark_chrome'),
  )
  ring.position.y = h * 0.35
  ring.rotation.x = Math.PI / 2
  g.add(ring)

  // Bottom ring
  const ring2 = ring.clone()
  ring2.position.y = -h * 0.35
  g.add(ring2)

  // Output shaft
  const shaftR = r * 0.12
  const shaftH = h * 0.4
  const shaft = new THREE.Mesh(
    chamferedCylinder(shaftR, shaftH, shaftR * 0.15),
    getMaterial('brushed_steel'),
  )
  shaft.position.y = (h * 0.8 + shaftH) / 2
  g.add(shaft)

  // Rear mounting flange
  const flange = flangePlate(r * 0.85, h * 0.06, 6, r * 0.65, r * 0.04)
  flange.position.y = -h * 0.43
  g.add(flange)

  // Cable exit
  const cable = new THREE.Mesh(
    new THREE.CylinderGeometry(r * 0.04, r * 0.04, r * 0.2, 8),
    getMaterial('matte_plastic'),
  )
  cable.rotation.z = Math.PI / 2
  cable.position.set(r * 0.7, -h * 0.25, 0)
  g.add(cable)

  // Winding peek (copper ring visible through vent slots)
  const winding = new THREE.Mesh(
    new THREE.TorusGeometry(r * 0.6, r * 0.08, 6, 48),
    getMaterial('copper_trace'),
  )
  winding.rotation.x = Math.PI / 2
  winding.position.y = -h * 0.15
  g.add(winding)

  return g
}

// ── NEMA Stepper ─────────────────────────────────────────────────────────────

function generateStepper(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: faceW, z: h } = dims

  const bodyR = faceW * 0.45
  const chamfer = faceW * 0.02

  // Square faceplate
  const faceH = h * 0.06
  const face = new THREE.Mesh(
    chamferedBox(faceW, faceW, faceH, chamfer),
    catMetal(0.3),
  )
  face.position.y = (h + faceH) / 2
  g.add(face)

  // Faceplate mounting holes (4 corners)
  const holeR = faceW * 0.025
  const holeSpacing = faceW * 0.37
  for (const sx of [-1, 1]) {
    for (const sz of [-1, 1]) {
      const hole = mountingHole(holeR, faceH * 1.1)
      hole.position.set(sx * holeSpacing, (h + faceH) / 2, sz * holeSpacing)
      g.add(hole)
    }
  }

  // Round body (black motor casing)
  const motorBody = new THREE.Mesh(
    chamferedCylinder(bodyR, h, chamfer, 48),
    getMaterial('matte_plastic'),
  )
  g.add(motorBody)

  // Output shaft
  const shaftR = faceW * 0.04
  const shaftH = h * 0.25
  const shaft = new THREE.Mesh(
    chamferedCylinder(shaftR, shaftH, shaftR * 0.15),
    getMaterial('brushed_steel'),
  )
  shaft.position.y = (h + faceH + shaftH) / 2
  g.add(shaft)

  // Rear connector bump
  const connW = faceW * 0.35
  const connH = faceW * 0.2
  const connD = faceH
  const conn = new THREE.Mesh(
    chamferedBox(connW, connH, connD, chamfer * 0.5),
    getMaterial('matte_plastic', 0x333333),
  )
  conn.position.set(0, -h / 2 - connD / 2, -faceW * 0.25)
  g.add(conn)

  return g
}

// ── Linear Actuator ──────────────────────────────────────────────────────────

function generateLinearActuator(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims

  const chamfer = Math.min(w, h) * 0.08

  // Main body
  const body = new THREE.Mesh(
    chamferedBox(w, h, length, chamfer),
    catMetal(0.3),
  )
  g.add(body)

  // Extending rod
  const rodR = Math.min(w, h) * 0.18
  const rodLen = length * 0.35
  const rod = new THREE.Mesh(
    chamferedCylinder(rodR, rodLen, rodR * 0.1),
    getMaterial('brushed_steel'),
  )
  rod.position.z = (length + rodLen) / 2
  rod.rotation.x = Math.PI / 2
  g.add(rod)

  // Rod end clevis
  const clevisW = w * 0.5
  const clevisH = h * 1.1
  const clevisD = h * 0.25
  const clevis = new THREE.Mesh(
    chamferedBox(clevisW, clevisH, clevisD, chamfer * 0.3),
    getMaterial('brushed_steel'),
  )
  clevis.position.z = length / 2 + rodLen
  g.add(clevis)

  // Clevis pin hole
  const pinHole = mountingHole(rodR * 0.5, clevisW * 1.1)
  pinHole.rotation.z = Math.PI / 2
  pinHole.position.z = length / 2 + rodLen
  g.add(pinHole)

  // Rear clevis mount
  const rearClevis = new THREE.Mesh(
    chamferedBox(clevisW, clevisH, clevisD, chamfer * 0.3),
    catMetal(0.25),
  )
  rearClevis.position.z = -(length + clevisD) / 2
  g.add(rearClevis)

  // Label on body
  const label = labelRecess(w * 0.5, length * 0.3, chamfer * 0.3)
  label.position.set(0, h * 0.42, 0)
  label.rotation.x = Math.PI / 2
  g.add(label)

  return g
}

// ── Dispatcher ───────────────────────────────────────────────────────────────

export function generateRichActuator(id: string, dims: GeneratorDims): THREE.Group {
  if (id.includes('bldc')) return generateBLDC(id, dims)
  if (id.includes('stepper') || id.includes('nema')) return generateStepper(id, dims)
  if (id.includes('linear')) return generateLinearActuator(id, dims)
  // Default: servo (covers servo_micro, servo_standard, servo_high_torque, etc.)
  return generateServo(id, dims)
}

/**
 * Rich visual generators for actuator components.
 * Servos, BLDC outrunners, steppers, linear actuators.
 *
 * Uses profile-based geometry (servoBody, revolvedMotor, chamferedBox/Cylinder)
 * for Fusion-quality mechanical part visuals.
 */
import * as THREE from 'three'
import type { GeneratorDims } from './index'
import { getMaterial, getTintedMaterial } from '../materials'
import {
  servoBody, revolvedMotor, chamferedBox, chamferedCylinder,
  boltCircle, mountingHole, screwHead, labelRecess, flangePlate,
  cablePort, knurledRing,
} from '../primitives'
import { nurbsFilletBox, nurbsCylinder, nurbsServoHorn, nurbsMotorHousing, nurbsTorus } from '../nurbs'

const CAT_COLOR: [number, number, number] = [0.90, 0.49, 0.13]  // orange

function catMetal(strength = 0.3) {
  return getTintedMaterial('anodized_aluminum', ...CAT_COLOR, strength)
}

// ── Servo ─────────────────────────────────────────────────────────────────────

function generateServo(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h, y: d } = dims

  const isHeavy = id.includes('heavy') || id.includes('high_torque')

  // Servo housing — NURBS filleted box (smooth mathematically exact edges)
  const housing = new THREE.Mesh(
    nurbsFilletBox(w, h * 0.72, d, Math.min(w, d) * 0.06, 16),
    getMaterial('matte_plastic'),
  )
  g.add(housing)

  // Mounting ears — NURBS filleted
  const earW = w * 1.15
  const earH = h * 0.1
  const ear = new THREE.Mesh(
    nurbsFilletBox(earW, earH, d, Math.min(earW, d) * 0.04, 8),
    getMaterial('matte_plastic', 0x1a1a1a),
  )
  ear.position.y = h * 0.30
  g.add(ear)

  // Ear mounting holes
  const holeR = Math.min(w, d) * 0.03
  for (const sx of [-1, 1]) {
    for (const sz of [-1, 1]) {
      const hole = mountingHole(holeR, earH * 1.2)
      hole.position.set(sx * earW * 0.42, h * 0.30, sz * d * 0.28)
      g.add(hole)
    }
  }

  // Output horn — NURBS smooth disc with lip profile
  const hornR = Math.min(w, d) * 0.3
  const hornH = h * 0.07
  const horn = new THREE.Mesh(
    nurbsServoHorn(hornR, hornH, Math.min(w, d) * 0.04, 48),
    getMaterial('glossy_plastic', 0xeeeeee),
  )
  horn.position.y = h * 0.42
  g.add(horn)

  // Horn bolt circle
  const hornBolts = boltCircle(hornR * 0.68, holeR * 0.55, isHeavy ? 6 : 4, hornH * 1.1)
  hornBolts.position.y = h * 0.42
  g.add(hornBolts)

  // Center screw
  const screw = screwHead(holeR * 1.3, hornH * 0.6)
  screw.position.y = h * 0.46
  g.add(screw)

  // Output shaft — NURBS smooth cylinder
  const shaftR = Math.min(w, d) * 0.055
  const shaft = new THREE.Mesh(
    nurbsCylinder(shaftR, hornH * 1.8, shaftR * 0.15, 32),
    getMaterial('brushed_steel'),
  )
  shaft.position.y = h * 0.48
  g.add(shaft)

  // Shaft bearing ring — NURBS torus
  const bearingRing = new THREE.Mesh(
    nurbsTorus(shaftR * 2.2, shaftR * 0.4, 48, 16),
    getMaterial('brushed_steel'),
  )
  bearingRing.rotation.x = Math.PI / 2
  bearingRing.position.y = h * 0.38
  g.add(bearingRing)

  // Cable exit
  const cable = cablePort(Math.min(w, d) * 0.06, Math.min(w, d) * 0.02)
  cable.rotation.set(0, Math.PI, 0)
  cable.position.set(0, -h * 0.35, -d * 0.42)
  g.add(cable)

  // Cable strain relief — NURBS cylinder
  const strain = new THREE.Mesh(
    nurbsCylinder(Math.min(w, d) * 0.04, d * 0.12, Math.min(w, d) * 0.005, 16),
    getMaterial('rubber_black'),
  )
  strain.rotation.x = Math.PI / 2
  strain.position.set(0, -h * 0.35, -d * 0.5)
  g.add(strain)

  // Label recess
  const label = labelRecess(w * 0.55, h * 0.3, Math.min(w, d) * 0.025)
  label.position.set(0, -h * 0.05, d * 0.38)
  g.add(label)

  // Side ribs — NURBS filleted
  for (const sx of [-1, 1]) {
    const rib = new THREE.Mesh(
      nurbsFilletBox(w * 0.02, h * 0.5, d * 0.7, w * 0.003, 4),
      getMaterial('matte_plastic', 0x1a1a1a),
    )
    rib.position.set(sx * w * 0.48, -h * 0.05, 0)
    g.add(rib)
  }

  // Ventilation slots
  for (let i = -2; i <= 2; i++) {
    const slot = new THREE.Mesh(
      new THREE.BoxGeometry(w * 0.06, h * 0.015, d * 0.5),
      getMaterial('dark_chrome'),
    )
    slot.position.set(i * w * 0.12, -h * 0.36, 0)
    g.add(slot)
  }

  return g
}

// ── BLDC Outrunner ───────────────────────────────────────────────────────────

function generateBLDC(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h } = dims
  const r = w / 2

  // Main motor body — NURBS revolved profile (body + cap + shaft in one smooth surface)
  const shaftR = r * 0.11
  const shaftH = h * 0.35
  const motorBody = new THREE.Mesh(
    nurbsMotorHousing(r, h * 0.72, shaftR, shaftH, r * 0.88, h * 0.06, 64),
    catMetal(0.4),
  )
  g.add(motorBody)

  // Winding peek — NURBS torus
  const winding = new THREE.Mesh(
    nurbsTorus(r * 0.58, r * 0.09, 48, 16),
    getMaterial('copper_trace'),
  )
  winding.rotation.x = Math.PI / 2
  winding.position.y = -h * 0.1
  g.add(winding)

  // Top accent ring — NURBS torus
  const topRing = new THREE.Mesh(
    nurbsTorus(r * 0.96, r * 0.025, 48, 8),
    getMaterial('dark_chrome'),
  )
  topRing.rotation.x = Math.PI / 2
  topRing.position.y = h * 0.34
  g.add(topRing)

  // Bottom accent ring — NURBS torus
  const botRing = new THREE.Mesh(
    nurbsTorus(r * 0.96, r * 0.025, 48, 8),
    getMaterial('dark_chrome'),
  )
  botRing.rotation.x = Math.PI / 2
  botRing.position.y = -h * 0.34
  g.add(botRing)

  // Mounting flange (rear) — flangePlate
  const flange = flangePlate(r * 0.88, h * 0.06, 6, r * 0.66, r * 0.035)
  flange.position.y = -h * 0.45
  g.add(flange)

  // Cable exit
  const cable = cablePort(r * 0.05, r * 0.02)
  cable.rotation.set(0, 0, Math.PI / 2)
  cable.position.set(r * 0.75, -h * 0.28, 0)
  g.add(cable)

  // Cable strain relief wire
  const wire = new THREE.Mesh(
    chamferedCylinder(r * 0.035, r * 0.18, r * 0.005),
    getMaterial('rubber_black'),
  )
  wire.rotation.z = Math.PI / 2
  wire.position.set(r * 0.88, -h * 0.28, 0)
  g.add(wire)

  // Vent slots around body
  const ventCount = 8
  for (let i = 0; i < ventCount; i++) {
    const a = (i / ventCount) * Math.PI * 2
    const vent = new THREE.Mesh(
      new THREE.BoxGeometry(r * 0.04, h * 0.25, r * 0.008),
      getMaterial('dark_chrome'),
    )
    vent.position.set(Math.cos(a) * r * 0.98, 0, Math.sin(a) * r * 0.98)
    vent.rotation.y = -a
    g.add(vent)
  }

  // Shaft knurl detail
  const knurl = knurledRing(shaftR * 1.05, shaftH * 0.3, 12)
  knurl.position.y = h * 0.36 + shaftH * 0.6
  g.add(knurl)

  // Label recess
  const label = labelRecess(r * 0.7, h * 0.25, r * 0.015)
  label.position.set(0, 0, r * 0.94)
  g.add(label)

  return g
}

// ── NEMA Stepper ─────────────────────────────────────────────────────────────

function generateStepper(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: faceW, z: h } = dims

  const bodyR = faceW * 0.44
  const chamfer = faceW * 0.02

  // Square faceplate — chamferedBox
  const faceH = h * 0.07
  const face = new THREE.Mesh(
    chamferedBox(faceW, faceW, faceH, chamfer * 1.5),
    catMetal(0.3),
  )
  face.position.y = (h + faceH) / 2
  g.add(face)

  // 4 corner mounting holes on faceplate
  const holeR = faceW * 0.025
  const holeSpacing = faceW * 0.38
  for (const sx of [-1, 1]) {
    for (const sz of [-1, 1]) {
      const hole = mountingHole(holeR, faceH * 1.2)
      hole.position.set(sx * holeSpacing, (h + faceH) / 2, sz * holeSpacing)
      g.add(hole)
    }
  }

  // Round body — revolvedMotor for motor casing + cap
  const shaftR = faceW * 0.04
  const shaftH = h * 0.22
  const motor = new THREE.Mesh(
    revolvedMotor(bodyR, h * 0.85, shaftR, 0, 64),
    getMaterial('matte_plastic'),
  )
  g.add(motor)

  // Output shaft extends from faceplate
  const shaft = new THREE.Mesh(
    chamferedCylinder(shaftR, shaftH, shaftR * 0.15),
    getMaterial('brushed_steel'),
  )
  shaft.position.y = (h + faceH + shaftH) / 2
  g.add(shaft)

  // D-shaft flat (subtle dark strip on shaft)
  const dFlat = new THREE.Mesh(
    new THREE.BoxGeometry(shaftR * 0.5, shaftH * 0.9, shaftR * 0.15),
    getMaterial('dark_chrome'),
  )
  dFlat.position.set(shaftR * 0.75, (h + faceH + shaftH) / 2, 0)
  g.add(dFlat)

  // Rear faceplate
  const rearFace = new THREE.Mesh(
    chamferedBox(faceW, faceW, faceH * 0.7, chamfer * 1.5),
    catMetal(0.25),
  )
  rearFace.position.y = -(h + faceH * 0.7) / 2
  g.add(rearFace)

  // Rear connector bump
  const connW = faceW * 0.35
  const connH = faceW * 0.2
  const connD = faceH * 1.2
  const conn = new THREE.Mesh(
    chamferedBox(connW, connH, connD, chamfer * 0.5),
    getMaterial('matte_plastic', 0x333333),
  )
  conn.position.set(0, -(h / 2 + faceH * 0.7 + connD / 2), -faceW * 0.25)
  g.add(conn)

  // Body accent rings (4 thin rings along body)
  for (let i = 1; i <= 4; i++) {
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(bodyR * 1.01, bodyR * 0.008, 6, 48),
      getMaterial('dark_chrome'),
    )
    ring.rotation.x = Math.PI / 2
    ring.position.y = -h * 0.35 + i * h * 0.18
    g.add(ring)
  }

  // Label recess
  const label = labelRecess(bodyR * 1.2, h * 0.3, chamfer * 0.4)
  label.position.set(0, 0, bodyR * 0.93)
  g.add(label)

  // Corner chamfer details on faceplate
  const cornerR = faceW * 0.03
  for (const sx of [-1, 1]) {
    for (const sz of [-1, 1]) {
      const corner = new THREE.Mesh(
        chamferedCylinder(cornerR, faceH * 0.9, cornerR * 0.2, 16),
        getMaterial('brushed_steel'),
      )
      corner.position.set(sx * faceW * 0.46, (h + faceH) / 2, sz * faceW * 0.46)
      g.add(corner)
    }
  }

  return g
}

// ── Linear Actuator ──────────────────────────────────────────────────────────

function generateLinearActuator(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims

  const chamfer = Math.min(w, h) * 0.08

  // Main body — chamferedBox
  const body = new THREE.Mesh(
    chamferedBox(w, h, length, chamfer),
    catMetal(0.3),
  )
  g.add(body)

  // Body side ribs for realism
  for (const sx of [-1, 1]) {
    for (let i = 0; i < 3; i++) {
      const rib = new THREE.Mesh(
        new THREE.BoxGeometry(w * 0.015, h * 0.6, length * 0.06),
        getMaterial('dark_chrome'),
      )
      rib.position.set(sx * w * 0.48, 0, -length * 0.2 + i * length * 0.2)
      g.add(rib)
    }
  }

  // Extending rod — chamferedCylinder
  const rodR = Math.min(w, h) * 0.17
  const rodLen = length * 0.38
  const rod = new THREE.Mesh(
    chamferedCylinder(rodR, rodLen, rodR * 0.08),
    getMaterial('brushed_steel'),
  )
  rod.position.z = (length + rodLen) / 2
  rod.rotation.x = Math.PI / 2
  g.add(rod)

  // Rod wiper seal ring
  const sealRing = new THREE.Mesh(
    new THREE.TorusGeometry(rodR * 1.3, rodR * 0.15, 8, 32),
    getMaterial('rubber_black'),
  )
  sealRing.position.set(0, 0, length * 0.5)
  g.add(sealRing)

  // Rod end clevis (front)
  const clevisW = w * 0.48
  const clevisH = h * 1.15
  const clevisD = h * 0.22
  const clevis = new THREE.Mesh(
    chamferedBox(clevisW, clevisH, clevisD, chamfer * 0.25),
    getMaterial('brushed_steel'),
  )
  clevis.position.z = length / 2 + rodLen
  g.add(clevis)

  // Clevis pin hole
  const pinHole = mountingHole(rodR * 0.45, clevisW * 1.15)
  pinHole.rotation.z = Math.PI / 2
  pinHole.position.z = length / 2 + rodLen
  g.add(pinHole)

  // Clevis fork gap (dark slot through middle)
  const gapSlot = new THREE.Mesh(
    new THREE.BoxGeometry(clevisW * 1.05, clevisH * 0.35, clevisD * 0.5),
    getMaterial('dark_chrome'),
  )
  gapSlot.position.z = length / 2 + rodLen
  g.add(gapSlot)

  // Rear clevis mount
  const rearClevis = new THREE.Mesh(
    chamferedBox(clevisW, clevisH, clevisD, chamfer * 0.25),
    catMetal(0.25),
  )
  rearClevis.position.z = -(length + clevisD) / 2
  g.add(rearClevis)

  // Rear clevis pin hole
  const rearPinHole = mountingHole(rodR * 0.45, clevisW * 1.15)
  rearPinHole.rotation.z = Math.PI / 2
  rearPinHole.position.z = -(length + clevisD) / 2
  g.add(rearPinHole)

  // Rear clevis fork gap
  const rearGap = new THREE.Mesh(
    new THREE.BoxGeometry(clevisW * 1.05, clevisH * 0.35, clevisD * 0.5),
    getMaterial('dark_chrome'),
  )
  rearGap.position.z = -(length + clevisD) / 2
  g.add(rearGap)

  // Label on body top
  const label = labelRecess(w * 0.5, length * 0.25, chamfer * 0.3)
  label.position.set(0, h * 0.43, 0)
  label.rotation.x = Math.PI / 2
  g.add(label)

  // Cable exit
  const cable = cablePort(Math.min(w, h) * 0.05, Math.min(w, h) * 0.018)
  cable.position.set(0, -h * 0.4, -length * 0.3)
  g.add(cable)

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

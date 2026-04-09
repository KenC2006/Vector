/**
 * Rich visual generators for mobility components.
 * Wheels, casters, mecanum, omni, tracks, swerve drives, ball transfers, feet.
 */
import * as THREE from 'three'
import type { GeneratorDims } from './index'
import { getMaterial, getTintedMaterial } from '../materials'
import {
  chamferedBox, chamferedCylinder, flangePlate, labelRecess,
} from '../primitives'

const CAT_COLOR: [number, number, number] = [0.20, 0.29, 0.37]  // dark slate

function catMetal(strength = 0.3) {
  return getTintedMaterial('anodized_aluminum', ...CAT_COLOR, strength)
}

// ── Driven Wheel ────────────────────────────────────────────────────────────

function generateDrivenWheel(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.04

  // Tire (sideways chamferedCylinder)
  const tireR = Math.max(w, d) * 0.45
  const tireH = h * 0.35
  const tire = new THREE.Mesh(
    chamferedCylinder(tireR, tireH, chamfer, 36),
    getMaterial('rubber_black'),
  )
  tire.rotation.x = Math.PI / 2
  g.add(tire)

  // Hub (smaller, anodized aluminum)
  const hubR = tireR * 0.55
  const hubH = tireH * 0.6
  const hub = new THREE.Mesh(
    chamferedCylinder(hubR, hubH, chamfer * 0.5, 32),
    catMetal(0.35),
  )
  hub.rotation.x = Math.PI / 2
  g.add(hub)

  // Axle bore (dark inset)
  const axleR = hubR * 0.3
  const axle = new THREE.Mesh(
    chamferedCylinder(axleR, hubH * 1.1, axleR * 0.1, 16),
    getMaterial('dark_chrome'),
  )
  axle.rotation.x = Math.PI / 2
  g.add(axle)

  // Tread lines (thin rings on tire surface)
  const treadMat = getMaterial('matte_plastic', 0x1a1a1a)
  const treadCount = 6
  for (let i = 0; i < treadCount; i++) {
    const t = (i / (treadCount - 1)) - 0.5
    const tread = new THREE.Mesh(
      new THREE.TorusGeometry(tireR * 0.98, tireR * 0.015, 6, 32),
      treadMat,
    )
    tread.position.z = t * tireH * 0.8
    g.add(tread)
  }

  // Hub spokes (cross pattern)
  const spokeW = hubR * 0.12
  const spokeLen = hubR * 1.6
  const spokeD = hubH * 0.15
  for (let i = 0; i < 5; i++) {
    const angle = (i / 5) * Math.PI * 2
    const spoke = new THREE.Mesh(
      chamferedBox(spokeW, spokeLen, spokeD, chamfer * 0.2),
      catMetal(0.25),
    )
    spoke.rotation.z = angle
    g.add(spoke)
  }

  return g
}

// ── Caster Wheel ────────────────────────────────────────────────────────────

function generateCasterWheel(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.04

  // Top mounting plate
  const plateR = Math.min(w, d) * 0.3
  const plateH = h * 0.08
  const plate = new THREE.Mesh(
    chamferedCylinder(plateR, plateH, chamfer * 0.3, 24),
    catMetal(0.3),
  )
  plate.position.y = h * 0.4
  g.add(plate)

  // Swivel stem
  const stemR = plateR * 0.25
  const stemH = h * 0.2
  const stem = new THREE.Mesh(
    chamferedCylinder(stemR, stemH, stemR * 0.15, 16),
    getMaterial('brushed_steel'),
  )
  stem.position.y = h * 0.25
  g.add(stem)

  // Fork sides (2 thin chamferedBoxes)
  const forkW = w * 0.06
  const forkH = h * 0.4
  const forkD = d * 0.35
  const forkSpacing = d * 0.2
  for (const sz of [-1, 1]) {
    const fork = new THREE.Mesh(
      chamferedBox(forkW, forkD, forkH, chamfer * 0.3),
      catMetal(0.25),
    )
    fork.position.set(0, h * 0.05, sz * forkSpacing)
    g.add(fork)
  }

  // Fork top bridge
  const bridge = new THREE.Mesh(
    chamferedBox(forkW, forkSpacing * 2.2, h * 0.06, chamfer * 0.2),
    catMetal(0.25),
  )
  bridge.position.y = h * 0.15
  g.add(bridge)

  // Wheel (rubber_black, sideways)
  const wheelR = Math.min(w, d) * 0.25
  const wheelH = d * 0.15
  const wheel = new THREE.Mesh(
    chamferedCylinder(wheelR, wheelH, chamfer * 0.3, 24),
    getMaterial('rubber_black'),
  )
  wheel.rotation.x = Math.PI / 2
  wheel.position.y = -h * 0.25
  g.add(wheel)

  // Axle through wheel
  const axleR = wheelR * 0.15
  const axle = new THREE.Mesh(
    chamferedCylinder(axleR, forkSpacing * 2.5, axleR * 0.2, 8),
    getMaterial('brushed_steel'),
  )
  axle.rotation.x = Math.PI / 2
  axle.position.y = -h * 0.25
  g.add(axle)

  return g
}

// ── Mecanum Wheel ───────────────────────────────────────────────────────────

function generateMecanumWheel(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.03

  // Side plates (2 chamferedCylinders)
  const plateR = Math.max(w, d) * 0.42
  const plateH = h * 0.06
  for (const s of [-1, 1]) {
    const plate = new THREE.Mesh(
      chamferedCylinder(plateR, plateH, chamfer * 0.3, 36),
      catMetal(0.35),
    )
    plate.rotation.x = Math.PI / 2
    plate.position.z = s * h * 0.15
    g.add(plate)
  }

  // Hub
  const hubR = plateR * 0.3
  const hubH = h * 0.35
  const hub = new THREE.Mesh(
    chamferedCylinder(hubR, hubH, chamfer * 0.4, 24),
    catMetal(0.3),
  )
  hub.rotation.x = Math.PI / 2
  g.add(hub)

  // Axle bore
  const axleR = hubR * 0.35
  const axle = new THREE.Mesh(
    chamferedCylinder(axleR, hubH * 1.2, axleR * 0.1, 12),
    getMaterial('dark_chrome'),
  )
  axle.rotation.x = Math.PI / 2
  g.add(axle)

  // 9 angled rollers at 45 degrees
  const rollerR = plateR * 0.09
  const rollerH = h * 0.22
  const rollerCircleR = plateR * 0.72
  for (let i = 0; i < 9; i++) {
    const angle = (i / 9) * Math.PI * 2
    const rx = Math.cos(angle) * rollerCircleR
    const ry = Math.sin(angle) * rollerCircleR
    const roller = new THREE.Mesh(
      chamferedCylinder(rollerR, rollerH, rollerR * 0.15, 10),
      getMaterial('rubber_black'),
    )
    // 45-degree tilt in the axial direction
    roller.position.set(rx, ry, 0)
    roller.rotation.set(0, 0, angle)
    roller.rotateX(Math.PI / 4)
    g.add(roller)
  }

  return g
}

// ── Omni Wheel ──────────────────────────────────────────────────────────────

function generateOmniWheel(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.03

  // Hub
  const hubR = Math.max(w, d) * 0.2
  const hubH = h * 0.3
  const hub = new THREE.Mesh(
    chamferedCylinder(hubR, hubH, chamfer * 0.4, 24),
    catMetal(0.35),
  )
  hub.rotation.x = Math.PI / 2
  g.add(hub)

  // Axle bore
  const axleR = hubR * 0.35
  const axle = new THREE.Mesh(
    chamferedCylinder(axleR, hubH * 1.1, axleR * 0.1, 12),
    getMaterial('dark_chrome'),
  )
  axle.rotation.x = Math.PI / 2
  g.add(axle)

  // 10 perpendicular rollers around circumference
  const wheelR = Math.max(w, d) * 0.42
  const rollerR = wheelR * 0.1
  const rollerH = h * 0.25
  for (let i = 0; i < 10; i++) {
    const angle = (i / 10) * Math.PI * 2
    const rx = Math.cos(angle) * wheelR
    const ry = Math.sin(angle) * wheelR
    const roller = new THREE.Mesh(
      chamferedCylinder(rollerR, rollerH, rollerR * 0.12, 10),
      getMaterial('rubber_black'),
    )
    roller.position.set(rx, ry, 0)
    // Perpendicular to wheel plane (along Z, the axle direction)
    roller.rotation.x = Math.PI / 2
    g.add(roller)
  }

  // Side ring outlines (thin torus on each side)
  for (const s of [-1, 1]) {
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(wheelR, wheelR * 0.02, 6, 36),
      catMetal(0.25),
    )
    ring.position.z = s * hubH * 0.35
    g.add(ring)
  }

  return g
}

// ── Track Tread System ──────────────────────────────────────────────────────

function generateTrackSystem(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.04

  const sprocketR = Math.min(h, d) * 0.3
  const sprocketH = w * 0.12
  const trackLen = d * 0.8
  const halfLen = trackLen * 0.5

  // 2 sprocket wheels
  for (const sx of [-1, 1]) {
    const sprocket = new THREE.Mesh(
      chamferedCylinder(sprocketR, sprocketH, chamfer * 0.3, 24),
      catMetal(0.35),
    )
    sprocket.rotation.x = Math.PI / 2
    sprocket.position.set(0, 0, sx * halfLen)
    g.add(sprocket)

    // Sprocket teeth (small boxes around circumference)
    const toothCount = 12
    const toothW = sprocketR * 0.1
    const toothH = sprocketR * 0.12
    const toothD = sprocketH * 0.8
    for (let i = 0; i < toothCount; i++) {
      const angle = (i / toothCount) * Math.PI * 2
      const tx = Math.cos(angle) * sprocketR * 1.05
      const ty = Math.sin(angle) * sprocketR * 1.05
      const tooth = new THREE.Mesh(
        chamferedBox(toothD, toothW, toothH, chamfer * 0.1),
        catMetal(0.3),
      )
      tooth.position.set(tx, ty, sx * halfLen)
      tooth.rotation.z = angle
      g.add(tooth)
    }
  }

  // Tread belt (top and bottom runs)
  const beltW = w * 0.25
  const beltH = h * 0.06
  for (const sy of [-1, 1]) {
    const belt = new THREE.Mesh(
      chamferedBox(beltW, trackLen * 1.1, beltH, chamfer * 0.2),
      getMaterial('rubber_black'),
    )
    belt.position.y = sy * sprocketR
    g.add(belt)
  }

  // Side armor plates
  const armorW = w * 0.04
  const armorH = sprocketR * 2.4
  const armorD = trackLen * 1.15
  for (const sx of [-1, 1]) {
    const armor = new THREE.Mesh(
      chamferedBox(armorW, armorD, armorH, chamfer * 0.3),
      catMetal(0.2),
    )
    armor.position.x = sx * beltW * 0.55
    g.add(armor)
  }

  return g
}

// ── Swerve Drive Module ─────────────────────────────────────────────────────

function generateSwerveDrive(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.05

  // Steering housing
  const housingW = w * 0.5
  const housingH = h * 0.35
  const housingD = d * 0.5
  const housing = new THREE.Mesh(
    chamferedBox(housingW, housingD, housingH, chamfer),
    catMetal(0.35),
  )
  housing.position.y = h * 0.15
  g.add(housing)

  // Steering motor on top (small cylinder)
  const motorR = Math.min(w, d) * 0.12
  const motorH = h * 0.2
  const motor = new THREE.Mesh(
    chamferedCylinder(motorR, motorH, chamfer * 0.4, 20),
    getMaterial('matte_plastic'),
  )
  motor.position.y = h * 0.15 + housingH * 0.5 + motorH * 0.5
  g.add(motor)

  // Fork sides
  const forkW = w * 0.06
  const forkH = h * 0.3
  const forkD = d * 0.3
  const forkSpacing = d * 0.2
  for (const sz of [-1, 1]) {
    const fork = new THREE.Mesh(
      chamferedBox(forkW, forkD, forkH, chamfer * 0.3),
      catMetal(0.25),
    )
    fork.position.set(0, -h * 0.1, sz * forkSpacing)
    g.add(fork)
  }

  // Wheel
  const wheelR = Math.min(w, d) * 0.25
  const wheelH = d * 0.14
  const wheel = new THREE.Mesh(
    chamferedCylinder(wheelR, wheelH, chamfer * 0.3, 24),
    getMaterial('rubber_black'),
  )
  wheel.rotation.x = Math.PI / 2
  wheel.position.y = -h * 0.3
  g.add(wheel)

  // Mounting flange on top
  const flange = flangePlate(Math.min(w, d) * 0.22, h * 0.05, 4, Math.min(w, d) * 0.16, Math.min(w, d) * 0.02)
  flange.position.y = h * 0.15 + housingH * 0.5 + motorH + h * 0.025
  g.add(flange)

  // Label
  const label = labelRecess(housingW * 0.6, housingH * 0.4, chamfer * 0.3)
  label.position.set(0, h * 0.15, housingD * 0.42)
  g.add(label)

  return g
}

// ── Ball Transfer Unit ──────────────────────────────────────────────────────

function generateBallTransfer(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.04

  // Housing cylinder
  const housingR = Math.min(w, d) * 0.35
  const housingH = h * 0.5
  const housing = new THREE.Mesh(
    chamferedCylinder(housingR, housingH, chamfer, 28),
    catMetal(0.35),
  )
  housing.position.y = -h * 0.1
  g.add(housing)

  // Lip ring at top
  const lipR = housingR * 1.1
  const lipH = housingH * 0.12
  const lip = new THREE.Mesh(
    chamferedCylinder(lipR, lipH, chamfer * 0.3, 28),
    catMetal(0.25),
  )
  lip.position.y = housingH * 0.35
  g.add(lip)

  // Ball on top (glossy plastic)
  const ballR = housingR * 0.6
  const ball = new THREE.Mesh(
    new THREE.SphereGeometry(ballR, 24, 24),
    getMaterial('glossy_plastic', 0x999999),
  )
  ball.position.y = housingH * 0.4 + ballR * 0.5
  g.add(ball)

  // Mounting flange at bottom
  const flange = flangePlate(housingR * 1.2, h * 0.06, 4, housingR * 0.9, housingR * 0.06)
  flange.position.y = -h * 0.1 - housingH * 0.5 - h * 0.03
  g.add(flange)

  // Inner race ring (visible above lip)
  const raceRing = new THREE.Mesh(
    new THREE.TorusGeometry(housingR * 0.55, housingR * 0.04, 6, 24),
    getMaterial('brushed_steel'),
  )
  raceRing.rotation.x = Math.PI / 2
  raceRing.position.y = housingH * 0.42
  g.add(raceRing)

  return g
}

// ── Rubber Foot Pad ─────────────────────────────────────────────────────────

function generateRubberFoot(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.06

  // Squat rubber cylinder
  const bodyR = Math.min(w, d) * 0.35
  const bodyH = h * 0.45
  const body = new THREE.Mesh(
    chamferedCylinder(bodyR, bodyH, chamfer, 28),
    getMaterial('rubber_black'),
  )
  g.add(body)

  // Wider base flange
  const baseR = bodyR * 1.35
  const baseH = h * 0.15
  const base = new THREE.Mesh(
    chamferedCylinder(baseR, baseH, chamfer * 0.5, 28),
    getMaterial('rubber_black'),
  )
  base.position.y = -bodyH * 0.5 - baseH * 0.3
  g.add(base)

  // Center mounting bolt inset
  const boltR = bodyR * 0.15
  const bolt = new THREE.Mesh(
    chamferedCylinder(boltR, bodyH * 1.2, boltR * 0.1, 12),
    getMaterial('brushed_steel'),
  )
  g.add(bolt)

  // Tread pattern on bottom (concentric rings)
  const treadMat = getMaterial('matte_plastic', 0x0a0a0a)
  for (let i = 1; i <= 3; i++) {
    const ringR = baseR * (i / 4)
    const tread = new THREE.Mesh(
      new THREE.TorusGeometry(ringR, baseR * 0.02, 4, 24),
      treadMat,
    )
    tread.rotation.x = Math.PI / 2
    tread.position.y = -bodyH * 0.5 - baseH * 0.5
    g.add(tread)
  }

  // Washer on top
  const washerR = bodyR * 0.5
  const washerH = h * 0.04
  const washer = new THREE.Mesh(
    chamferedCylinder(washerR, washerH, washerH * 0.2, 16),
    getMaterial('brushed_steel'),
  )
  washer.position.y = bodyH * 0.5 + washerH * 0.5
  g.add(washer)

  return g
}

// ── Dispatcher ──────────────────────────────────────────────────────────────

export function generateRichMobility(id: string, dims: GeneratorDims): THREE.Group {
  if (id.includes('caster')) return generateCasterWheel(id, dims)
  if (id.includes('mecanum')) return generateMecanumWheel(id, dims)
  if (id.includes('omni')) return generateOmniWheel(id, dims)
  if (id.includes('track') || id.includes('tread')) return generateTrackSystem(id, dims)
  if (id.includes('swerve')) return generateSwerveDrive(id, dims)
  if (id.includes('ball_transfer')) return generateBallTransfer(id, dims)
  if (id.includes('foot') || id.includes('rubber_foot')) return generateRubberFoot(id, dims)
  if (id.includes('wheel') || id.includes('driven')) return generateDrivenWheel(id, dims)
  // Default: driven wheel
  return generateDrivenWheel(id, dims)
}

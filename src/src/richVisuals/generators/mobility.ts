/**
 * Rich visual generators for mobility components.
 * Fusion-quality visuals using profile-based geometry.
 * Wheels, casters, mecanum, omni, tracks, swerve drives, ball transfers, feet.
 */
import * as THREE from 'three'
import type { GeneratorDims } from './index'
import { getMaterial, getTintedMaterial } from '../materials'
import {
  flangePlate, labelRecess,
} from '../primitives'
import { nurbsFilletBox, nurbsCylinder, nurbsTorus } from '../nurbs'

const DEFAULT_COLOR: [number, number, number] = [0.20, 0.29, 0.37]  // dark slate
let CAT_COLOR: [number, number, number] = DEFAULT_COLOR

function catMetal(strength = 0.3) {
  return getTintedMaterial('anodized_aluminum', ...CAT_COLOR, strength)
}

// ── Driven Wheel ────────────────────────────────────────────────────────────

function generateDrivenWheel(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  const tireR = Math.max(w, d) * 0.45
  const tireWidth = h * 0.35
  const hubR = tireR * 0.52

  // Tire — LatheGeometry revolved profile (hub -> sidewall curve -> tread flat -> other sidewall)
  // Profile is half-section from center axis outward, revolved around Z
  const hw = tireWidth / 2
  const sidewallBulge = tireR * 0.06  // sidewall convex outward
  const treadFlat = tireR * 0.03      // tread crown radius drop
  const steps = 10

  const tirePts: THREE.Vector2[] = []
  // Inner bore bottom
  tirePts.push(new THREE.Vector2(hubR * 1.05, -hw))
  // Lower sidewall — curves outward from hub to tread
  for (let i = 0; i <= steps; i++) {
    const t = i / steps
    const angle = t * Math.PI * 0.5
    const r = hubR * 1.05 + (tireR - hubR * 1.05 - treadFlat) * Math.sin(angle)
    const bulge = sidewallBulge * Math.sin(t * Math.PI)
    const y = -hw + (hw - treadFlat) * t
    tirePts.push(new THREE.Vector2(r + bulge, y))
  }
  // Tread crown (slightly rounded flat)
  tirePts.push(new THREE.Vector2(tireR, -treadFlat))
  tirePts.push(new THREE.Vector2(tireR + treadFlat * 0.3, 0))
  tirePts.push(new THREE.Vector2(tireR, treadFlat))
  // Upper sidewall — mirror curves back to hub
  for (let i = 0; i <= steps; i++) {
    const t = i / steps
    const angle = (1 - t) * Math.PI * 0.5
    const r = hubR * 1.05 + (tireR - hubR * 1.05 - treadFlat) * Math.sin(angle)
    const bulge = sidewallBulge * Math.sin((1 - t) * Math.PI)
    const y = treadFlat + (hw - treadFlat) * t
    tirePts.push(new THREE.Vector2(r + bulge, y))
  }
  // Inner bore top
  tirePts.push(new THREE.Vector2(hubR * 1.05, hw))
  // Close inner wall
  tirePts.push(new THREE.Vector2(hubR * 1.05, -hw))

  const tireGeom = new THREE.LatheGeometry(tirePts, 48)
  const tire = new THREE.Mesh(tireGeom, getMaterial('rubber_black'))
  // Orient sideways (wheel spins around X axis)
  tire.rotation.x = Math.PI / 2
  g.add(tire)

  // Hub — separate anodized_aluminum chamferedCylinder
  const hubWidth = tireWidth * 0.55
  const hub = new THREE.Mesh(
    nurbsCylinder(hubR, hubWidth, hubR * 0.04, 32),
    catMetal(0.35),
  )
  hub.rotation.x = Math.PI / 2
  g.add(hub)

  // Hub spokes (5 cross pattern)
  const spokeW = hubR * 0.12
  const spokeLen = hubR * 1.6
  const spokeD = hubWidth * 0.15
  for (let i = 0; i < 5; i++) {
    const angle = (i / 5) * Math.PI * 2
    const spoke = new THREE.Mesh(
      nurbsFilletBox(spokeW, spokeLen, spokeD, hubR * 0.01, 12),
      catMetal(0.25),
    )
    spoke.rotation.z = angle
    g.add(spoke)
  }

  // Axle bore (dark inset)
  const axleR = hubR * 0.28
  const axle = new THREE.Mesh(
    nurbsCylinder(axleR, hubWidth * 1.1, axleR * 0.1, 16),
    getMaterial('dark_chrome'),
  )
  axle.rotation.x = Math.PI / 2
  g.add(axle)

  return g
}

// ── Mecanum Wheel ───────────────────────────────────────────────────────────

function generateMecanumWheel(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.03

  const plateR = Math.max(w, d) * 0.42
  const hubH = h * 0.35

  // 2 side plates
  const plateH = h * 0.055
  for (const s of [-1, 1]) {
    const plate = new THREE.Mesh(
      nurbsCylinder(plateR, plateH, chamfer * 0.3, 36),
      catMetal(0.35),
    )
    plate.rotation.x = Math.PI / 2
    plate.position.z = s * hubH * 0.42
    g.add(plate)
  }

  // Hub
  const hubR = plateR * 0.3
  const hub = new THREE.Mesh(
    nurbsCylinder(hubR, hubH, chamfer * 0.4, 24),
    catMetal(0.3),
  )
  hub.rotation.x = Math.PI / 2
  g.add(hub)

  // Axle bore
  const axleR = hubR * 0.35
  const axle = new THREE.Mesh(
    nurbsCylinder(axleR, hubH * 1.2, axleR * 0.1, 12),
    getMaterial('dark_chrome'),
  )
  axle.rotation.x = Math.PI / 2
  g.add(axle)

  // 9 angled rollers at 45 degrees (rubber_black chamferedCylinders)
  const rollerR = plateR * 0.09
  const rollerH = h * 0.24
  const rollerCircleR = plateR * 0.72
  for (let i = 0; i < 9; i++) {
    const angle = (i / 9) * Math.PI * 2
    const rx = Math.cos(angle) * rollerCircleR
    const ry = Math.sin(angle) * rollerCircleR
    const roller = new THREE.Mesh(
      nurbsCylinder(rollerR, rollerH, rollerR * 0.15, 10),
      getMaterial('rubber_black'),
    )
    // Position on rim, then tilt 45 degrees in axial direction
    roller.position.set(rx, ry, 0)
    roller.rotation.set(0, 0, angle)
    roller.rotateX(Math.PI / 4)
    g.add(roller)
  }

  return g
}

// ── Caster Wheel ────────────────────────────────────────────────────────────

function generateCasterWheel(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.04

  // Top mounting plate — chamferedCylinder
  const plateR = Math.min(w, d) * 0.3
  const plateH = h * 0.08
  const plate = new THREE.Mesh(
    nurbsCylinder(plateR, plateH, chamfer * 0.3, 24),
    catMetal(0.3),
  )
  plate.position.y = h * 0.4
  g.add(plate)

  // Swivel stem
  const stemR = plateR * 0.25
  const stemH = h * 0.2
  const stem = new THREE.Mesh(
    nurbsCylinder(stemR, stemH, stemR * 0.15, 16),
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
      nurbsFilletBox(forkW, forkD, forkH, chamfer * 0.3, 16),
      catMetal(0.25),
    )
    fork.position.set(0, h * 0.05, sz * forkSpacing)
    g.add(fork)
  }

  // Fork top bridge
  const bridge = new THREE.Mesh(
    nurbsFilletBox(forkW, forkSpacing * 2.2, h * 0.06, chamfer * 0.2, 12),
    catMetal(0.25),
  )
  bridge.position.y = h * 0.15
  g.add(bridge)

  // Wheel — LatheGeometry tire profile (not just a flat cylinder)
  const wheelR = Math.min(w, d) * 0.25
  const wheelWidth = d * 0.13
  const whw = wheelWidth / 2
  const wheelPts: THREE.Vector2[] = []
  const innerR = wheelR * 0.45
  // Inner bore bottom
  wheelPts.push(new THREE.Vector2(innerR, -whw))
  // Lower sidewall curve
  wheelPts.push(new THREE.Vector2(wheelR * 0.7, -whw))
  wheelPts.push(new THREE.Vector2(wheelR * 0.92, -whw * 0.7))
  wheelPts.push(new THREE.Vector2(wheelR, -whw * 0.3))
  // Tread
  wheelPts.push(new THREE.Vector2(wheelR * 1.01, 0))
  // Upper sidewall
  wheelPts.push(new THREE.Vector2(wheelR, whw * 0.3))
  wheelPts.push(new THREE.Vector2(wheelR * 0.92, whw * 0.7))
  wheelPts.push(new THREE.Vector2(wheelR * 0.7, whw))
  // Inner bore top
  wheelPts.push(new THREE.Vector2(innerR, whw))
  wheelPts.push(new THREE.Vector2(innerR, -whw))

  const wheelGeom = new THREE.LatheGeometry(wheelPts, 32)
  const wheel = new THREE.Mesh(wheelGeom, getMaterial('rubber_black'))
  wheel.rotation.x = Math.PI / 2
  wheel.position.y = -h * 0.25
  g.add(wheel)

  // Axle through wheel
  const axleR = wheelR * 0.15
  const axle2 = new THREE.Mesh(
    nurbsCylinder(axleR, forkSpacing * 2.5, axleR * 0.2, 8),
    getMaterial('brushed_steel'),
  )
  axle2.rotation.x = Math.PI / 2
  axle2.position.y = -h * 0.25
  g.add(axle2)

  return g
}

// ── Track/Tread System ──────────────────────────────────────────────────────

function generateTrackSystem(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.04

  const sprocketR = Math.min(h, d) * 0.3
  const sprocketH = w * 0.12
  const trackLen = d * 0.8
  const halfLen = trackLen * 0.5

  // 2 sprocket wheels with gear teeth
  for (const sx of [-1, 1]) {
    const sprocket = new THREE.Mesh(
      nurbsCylinder(sprocketR, sprocketH, chamfer * 0.3, 24),
      catMetal(0.35),
    )
    sprocket.rotation.x = Math.PI / 2
    sprocket.position.set(0, 0, sx * halfLen)
    g.add(sprocket)

    // Gear teeth around sprocket circumference
    const toothCount = 12
    const toothW = sprocketR * 0.1
    const toothH2 = sprocketR * 0.12
    const toothD = sprocketH * 0.8
    for (let i = 0; i < toothCount; i++) {
      const angle = (i / toothCount) * Math.PI * 2
      const tx = Math.cos(angle) * sprocketR * 1.05
      const ty = Math.sin(angle) * sprocketR * 1.05
      const tooth = new THREE.Mesh(
        nurbsFilletBox(toothD, toothW, toothH2, chamfer * 0.1, 12),
        catMetal(0.3),
      )
      tooth.position.set(tx, ty, sx * halfLen)
      tooth.rotation.z = angle
      g.add(tooth)
    }
  }

  // Flat belt — top and bottom runs (chamferedBoxes)
  const beltW = w * 0.25
  const beltH = h * 0.06
  for (const sy of [-1, 1]) {
    const belt = new THREE.Mesh(
      nurbsFilletBox(beltW, trackLen * 1.1, beltH, chamfer * 0.2, 16),
      getMaterial('rubber_black'),
    )
    belt.position.y = sy * sprocketR
    g.add(belt)
  }

  // Side armor plates
  const armorW = w * 0.04
  const armorH = sprocketR * 2.4
  const armorD = trackLen * 1.15
  for (const sx2 of [-1, 1]) {
    const armor = new THREE.Mesh(
      nurbsFilletBox(armorW, armorD, armorH, chamfer * 0.3, 16),
      catMetal(0.2),
    )
    armor.position.x = sx2 * beltW * 0.55
    g.add(armor)
  }

  return g
}

// ── Swerve Drive Module ─────────────────────────────────────────────────────

function generateSwerveDrive(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.05

  // Housing — chamferedBox
  const housingW = w * 0.5
  const housingH = h * 0.35
  const housingD = d * 0.5
  const housing = new THREE.Mesh(
    nurbsFilletBox(housingW, housingD, housingH, chamfer, 16),
    catMetal(0.35),
  )
  housing.position.y = h * 0.15
  g.add(housing)

  // Steering motor on top — chamferedCylinder
  const motorR = Math.min(w, d) * 0.12
  const motorH = h * 0.2
  const motor = new THREE.Mesh(
    nurbsCylinder(motorR, motorH, chamfer * 0.4, 20),
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
      nurbsFilletBox(forkW, forkD, forkH, chamfer * 0.3, 16),
      catMetal(0.25),
    )
    fork.position.set(0, -h * 0.1, sz * forkSpacing)
    g.add(fork)
  }

  // Wheel
  const wheelR = Math.min(w, d) * 0.25
  const wheelH2 = d * 0.14
  const wheel = new THREE.Mesh(
    nurbsCylinder(wheelR, wheelH2, chamfer * 0.3, 24),
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

// ── Omni Wheel ──────────────────────────────────────────────────────────────

function generateOmniWheel(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.03

  const wheelR = Math.max(w, d) * 0.42

  // Hub
  const hubR = wheelR * 0.48
  const hubH = h * 0.3
  const hub = new THREE.Mesh(
    nurbsCylinder(hubR, hubH, chamfer * 0.4, 24),
    catMetal(0.35),
  )
  hub.rotation.x = Math.PI / 2
  g.add(hub)

  // Axle bore
  const axleR = hubR * 0.35
  const axle = new THREE.Mesh(
    nurbsCylinder(axleR, hubH * 1.1, axleR * 0.1, 12),
    getMaterial('dark_chrome'),
  )
  axle.rotation.x = Math.PI / 2
  g.add(axle)

  // 10 small perpendicular chamferedCylinder rollers around circumference
  const rollerR = wheelR * 0.1
  const rollerH = h * 0.26
  for (let i = 0; i < 10; i++) {
    const angle = (i / 10) * Math.PI * 2
    const rx = Math.cos(angle) * wheelR
    const ry = Math.sin(angle) * wheelR
    const roller = new THREE.Mesh(
      nurbsCylinder(rollerR, rollerH, rollerR * 0.12, 10),
      getMaterial('rubber_black'),
    )
    roller.position.set(rx, ry, 0)
    // Perpendicular to wheel plane (along Z, the axle direction)
    roller.rotation.x = Math.PI / 2
    g.add(roller)
  }

  // Side ring outlines
  for (const s of [-1, 1]) {
    const ring = new THREE.Mesh(
      nurbsTorus(wheelR, wheelR * 0.02, 36, 6),
      catMetal(0.25),
    )
    ring.position.z = s * hubH * 0.35
    g.add(ring)
  }

  return g
}

// ── Ball Transfer Unit ──────────────────────────────────────────────────────

function generateBallTransfer(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.04

  // Housing cylinder — chamferedCylinder
  const housingR = Math.min(w, d) * 0.35
  const housingH = h * 0.5
  const housing = new THREE.Mesh(
    nurbsCylinder(housingR, housingH, chamfer, 28),
    catMetal(0.35),
  )
  housing.position.y = -h * 0.1
  g.add(housing)

  // Lip ring at top
  const lipR = housingR * 1.1
  const lipH = housingH * 0.12
  const lip = new THREE.Mesh(
    nurbsCylinder(lipR, lipH, chamfer * 0.3, 28),
    catMetal(0.25),
  )
  lip.position.y = housingH * 0.35
  g.add(lip)

  // Ball — sphere with glossy_plastic
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

  // Inner race ring
  const raceRing = new THREE.Mesh(
    nurbsTorus(housingR * 0.55, housingR * 0.04, 24, 6),
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
  // bounding_box_mm is in URDF Z-up convention: x,y = horizontal diameter, z = disc height.
  // In GeneratorDims: w=x=diameter, h=y=diameter, d=z=height.
  const discR = Math.min(w, h) * 0.5   // horizontal radius from x/y (both = diameter)
  const discH = d                       // vertical thickness from z
  const chamfer = discH * 0.08

  // Rubber body — chamferedCylinder (rubber_black)
  const bodyR = discR * 0.7
  const bodyH = discH * 0.45
  const body = new THREE.Mesh(
    nurbsCylinder(bodyR, bodyH, chamfer, 28),
    getMaterial('rubber_black'),
  )
  g.add(body)

  // Wider base flange — nurbsCylinder (rubber_black)
  const baseR = discR * 0.95
  const baseH = discH * 0.15
  const base = new THREE.Mesh(
    nurbsCylinder(baseR, baseH, chamfer * 0.5, 28),
    getMaterial('rubber_black'),
  )
  base.position.y = -bodyH * 0.5 - baseH * 0.3
  g.add(base)

  // Center mounting bolt
  const boltR = bodyR * 0.15
  const bolt = new THREE.Mesh(
    nurbsCylinder(boltR, bodyH * 1.2, boltR * 0.1, 12),
    getMaterial('brushed_steel'),
  )
  g.add(bolt)

  // Tread pattern on bottom (concentric rings)
  const treadMat = getMaterial('matte_plastic', 0x0a0a0a)
  for (let i = 1; i <= 3; i++) {
    const ringR = baseR * (i / 4)
    const tread = new THREE.Mesh(
      nurbsTorus(ringR, baseR * 0.02, 24, 4),
      treadMat,
    )
    tread.rotation.x = Math.PI / 2
    tread.position.y = -bodyH * 0.5 - baseH * 0.5
    g.add(tread)
  }

  // Washer on top
  const washerR = bodyR * 0.5
  const washerH = discH * 0.06
  const washer = new THREE.Mesh(
    nurbsCylinder(washerR, washerH, washerH * 0.2, 16),
    getMaterial('brushed_steel'),
  )
  washer.position.y = bodyH * 0.5 + washerH * 0.5
  g.add(washer)

  return g
}

// ── Dispatcher ──────────────────────────────────────────────────────────────

export function generateRichMobility(id: string, dims: GeneratorDims, color?: [number, number, number]): THREE.Group {
  CAT_COLOR = color ?? DEFAULT_COLOR
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

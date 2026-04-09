/**
 * Rich visual generators for motor components.
 * DC motors, gear motors, hub motors, harmonic drives, pancake, brushless inrunner.
 */
import * as THREE from 'three'
import type { GeneratorDims } from './index'
import { getMaterial, getTintedMaterial } from '../materials'
import {
  chamferedBox, chamferedCylinder, boltCircle, mountingHole,
  labelRecess, flangePlate,
} from '../primitives'

const CAT_COLOR: [number, number, number] = [0.91, 0.30, 0.24] // red-orange

function catMetal(strength = 0.3) {
  return getTintedMaterial('anodized_aluminum', ...CAT_COLOR, strength)
}

function catPlastic(strength = 0.25) {
  return getTintedMaterial('matte_plastic', ...CAT_COLOR, strength)
}

// ── DC Motor ────────────────────────────────────────────────────────────────

function generateDCMotor(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2
  const chamfer = r * 0.04

  const isLarge = id.includes('large') || id.includes('775')
  const isMedium = id.includes('medium') || id.includes('540')

  // Main cylindrical body
  const bodyH = d * 0.75
  const body = new THREE.Mesh(
    chamferedCylinder(r, bodyH, chamfer, 36),
    catPlastic(0.2),
  )
  g.add(body)

  // Rear end cap (slightly wider ring)
  const capH = d * 0.08
  const cap = new THREE.Mesh(
    chamferedCylinder(r * 1.02, capH, chamfer * 0.5, 36),
    getMaterial('matte_plastic'),
  )
  cap.position.y = -(bodyH + capH) / 2
  g.add(cap)

  // Rear terminal bumps (2 metal tabs)
  const termW = r * 0.12
  const termH = d * 0.06
  const termD = r * 0.08
  for (const sx of [-1, 1]) {
    const term = new THREE.Mesh(
      chamferedBox(termW, termD, termH, termW * 0.1),
      getMaterial('copper_trace'),
    )
    term.position.set(sx * r * 0.35, -(bodyH / 2 + capH + termH / 2), 0)
    g.add(term)
  }

  // Front bearing plate
  const plateH = d * 0.04
  const plate = new THREE.Mesh(
    chamferedCylinder(r * 0.95, plateH, chamfer * 0.3, 36),
    getMaterial('brushed_steel'),
  )
  plate.position.y = (bodyH + plateH) / 2
  g.add(plate)

  // Output shaft
  const shaftR = isLarge ? r * 0.1 : (isMedium ? r * 0.09 : r * 0.07)
  const shaftH = d * 0.3
  const shaft = new THREE.Mesh(
    chamferedCylinder(shaftR, shaftH, shaftR * 0.15),
    getMaterial('brushed_steel'),
  )
  shaft.position.y = bodyH / 2 + plateH + shaftH / 2
  g.add(shaft)

  // Body band ring (decorative)
  const bandR = r * 1.005
  const band = new THREE.Mesh(
    new THREE.TorusGeometry(bandR, r * 0.015, 8, 36),
    getMaterial('dark_chrome'),
  )
  band.rotation.x = Math.PI / 2
  band.position.y = bodyH * 0.15
  g.add(band)

  // Label recess on body
  const label = labelRecess(r * 1.0, bodyH * 0.35, chamfer * 0.3)
  label.position.set(0, 0, r * 0.92)
  g.add(label)

  // Mounting holes on front plate (2 holes, for larger motors)
  if (isLarge || isMedium) {
    const holeR = r * 0.035
    for (const sx of [-1, 1]) {
      const hole = mountingHole(holeR, plateH * 1.1)
      hole.position.set(sx * r * 0.55, (bodyH + plateH) / 2, 0)
      g.add(hole)
    }
  }

  return g
}

// ── Gear Motor ──────────────────────────────────────────────────────────────

function generateGearMotor(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const motorR = w * 0.4
  const chamfer = w * 0.02

  const isHeavy = id.includes('heavy')

  // Motor cylinder (rear half)
  const motorH = d * 0.5
  const motorBody = new THREE.Mesh(
    chamferedCylinder(motorR, motorH, chamfer, 36),
    catPlastic(0.2),
  )
  motorBody.position.y = -d * 0.15
  g.add(motorBody)

  // Motor rear cap
  const capH = d * 0.04
  const motorCap = new THREE.Mesh(
    chamferedCylinder(motorR * 0.95, capH, chamfer * 0.3, 36),
    getMaterial('matte_plastic'),
  )
  motorCap.position.y = -d * 0.15 - (motorH + capH) / 2
  g.add(motorCap)

  // Terminal bumps
  for (const sx of [-1, 1]) {
    const term = new THREE.Mesh(
      chamferedBox(w * 0.04, w * 0.03, d * 0.03, w * 0.005),
      getMaterial('copper_trace'),
    )
    term.position.set(sx * motorR * 0.4, -d * 0.15 - motorH / 2 - capH - d * 0.015, 0)
    g.add(term)
  }

  // Gearbox housing (front)
  const gbW = w * 0.85
  const gbH = h * 0.85
  const gbD = d * 0.35
  const gearbox = new THREE.Mesh(
    chamferedBox(gbW, gbH, gbD, chamfer * 1.5),
    catMetal(0.35),
  )
  gearbox.position.y = d * 0.2
  g.add(gearbox)

  // Gearbox mounting holes (4 corners)
  const holeR = w * 0.02
  for (const sx of [-1, 1]) {
    for (const sz of [-1, 1]) {
      const hole = mountingHole(holeR, gbD * 0.5)
      hole.position.set(sx * gbW * 0.38, d * 0.2 + gbD * 0.26, sz * gbH * 0.38)
      g.add(hole)
    }
  }

  // Output shaft
  const shaftR = isHeavy ? w * 0.07 : w * 0.05
  const shaftH = d * 0.2
  const shaft = new THREE.Mesh(
    chamferedCylinder(shaftR, shaftH, shaftR * 0.15),
    getMaterial('brushed_steel'),
  )
  shaft.position.y = d * 0.2 + gbD / 2 + shaftH / 2
  g.add(shaft)

  // Gearbox face plate accent
  const facePlate = new THREE.Mesh(
    chamferedCylinder(w * 0.2, gbD * 0.08, chamfer * 0.3, 24),
    getMaterial('brushed_steel'),
  )
  facePlate.position.y = d * 0.2 + gbD * 0.48
  g.add(facePlate)

  // Label on gearbox side
  const label = labelRecess(gbW * 0.5, gbD * 0.5, chamfer * 0.3)
  label.position.set(0, d * 0.2, gbH * 0.44)
  g.add(label)

  return g
}

// ── Coreless Motor ──────────────────────────────────────────────────────────

function generateCoreless(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2
  const chamfer = r * 0.05

  // Tiny cylindrical body
  const bodyH = d * 0.7
  const body = new THREE.Mesh(
    chamferedCylinder(r, bodyH, chamfer, 24),
    catMetal(0.3),
  )
  g.add(body)

  // Rear cap
  const cap = new THREE.Mesh(
    chamferedCylinder(r * 0.9, d * 0.08, chamfer * 0.3, 24),
    getMaterial('matte_plastic'),
  )
  cap.position.y = -(bodyH + d * 0.08) / 2
  g.add(cap)

  // Terminal wires (2 thin cylinders)
  for (const sx of [-1, 1]) {
    const wire = new THREE.Mesh(
      chamferedCylinder(r * 0.04, d * 0.15, r * 0.005),
      getMaterial('copper_trace'),
    )
    wire.position.set(sx * r * 0.3, -(bodyH / 2 + d * 0.08 + d * 0.075), 0)
    g.add(wire)
  }

  // Output shaft
  const shaftR = r * 0.08
  const shaftH = d * 0.35
  const shaft = new THREE.Mesh(
    chamferedCylinder(shaftR, shaftH, shaftR * 0.1),
    getMaterial('brushed_steel'),
  )
  shaft.position.y = bodyH / 2 + shaftH / 2
  g.add(shaft)

  // Body accent ring
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(r * 0.98, r * 0.015, 6, 24),
    getMaterial('dark_chrome'),
  )
  ring.rotation.x = Math.PI / 2
  g.add(ring)

  // Label
  const label = labelRecess(r * 0.7, bodyH * 0.3, chamfer * 0.2)
  label.position.set(0, 0, r * 0.93)
  g.add(label)

  return g
}

// ── Worm Gear Motor ─────────────────────────────────────────────────────────

function generateWormGear(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const motorR = w * 0.35
  const chamfer = w * 0.02

  // Motor cylinder (along Y axis)
  const motorH = d * 0.55
  const motorBody = new THREE.Mesh(
    chamferedCylinder(motorR, motorH, chamfer, 32),
    catPlastic(0.2),
  )
  g.add(motorBody)

  // Motor rear cap
  const capH = d * 0.04
  const mCap = new THREE.Mesh(
    chamferedCylinder(motorR * 0.92, capH, chamfer * 0.3, 32),
    getMaterial('matte_plastic'),
  )
  mCap.position.y = -(motorH + capH) / 2
  g.add(mCap)

  // Perpendicular gearbox housing
  const gbW = w * 0.6
  const gbH = h * 0.5
  const gbD = d * 0.4
  const gearbox = new THREE.Mesh(
    chamferedBox(gbW, gbH, gbD, chamfer * 1.2),
    catMetal(0.35),
  )
  gearbox.position.set(w * 0.3, motorH * 0.15, 0)
  g.add(gearbox)

  // Perpendicular output shaft (along X)
  const shaftR = w * 0.04
  const shaftH = d * 0.2
  const shaft = new THREE.Mesh(
    chamferedCylinder(shaftR, shaftH, shaftR * 0.15),
    getMaterial('brushed_steel'),
  )
  shaft.rotation.z = Math.PI / 2
  shaft.position.set(w * 0.3 + gbW / 2 + shaftH / 2, motorH * 0.15, 0)
  g.add(shaft)

  // Mounting holes on gearbox face
  const holeR = w * 0.018
  for (const sz of [-1, 1]) {
    const hole = mountingHole(holeR, gbH * 0.3)
    hole.rotation.z = Math.PI / 2
    hole.position.set(w * 0.3 + gbW * 0.48, motorH * 0.15, sz * gbD * 0.3)
    g.add(hole)
  }

  // Terminal bumps
  for (const sx of [-1, 1]) {
    const term = new THREE.Mesh(
      chamferedBox(w * 0.03, w * 0.025, d * 0.025, w * 0.004),
      getMaterial('copper_trace'),
    )
    term.position.set(sx * motorR * 0.35, -(motorH / 2 + capH + d * 0.012), 0)
    g.add(term)
  }

  // Label on gearbox
  const label = labelRecess(gbW * 0.5, gbD * 0.4, chamfer * 0.3)
  label.position.set(w * 0.3, motorH * 0.15 + gbH * 0.42, 0)
  label.rotation.x = Math.PI / 2
  g.add(label)

  return g
}

// ── Hub Motor ───────────────────────────────────────────────────────────────

function generateHubMotor(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const outerR = w / 2
  const chamfer = outerR * 0.03
  const is120 = id.includes('120')

  // Outer ring (stator shell)
  const shellH = d * 0.6
  const shell = new THREE.Mesh(
    chamferedCylinder(outerR, shellH, chamfer, 48),
    catMetal(0.4),
  )
  g.add(shell)

  // Inner hub
  const hubR = outerR * 0.35
  const hubH = shellH * 1.1
  const hub = new THREE.Mesh(
    chamferedCylinder(hubR, hubH, chamfer * 0.5, 32),
    getMaterial('brushed_steel'),
  )
  g.add(hub)

  // Axle
  const axleR = outerR * 0.08
  const axleH = d * 1.2
  const axle = new THREE.Mesh(
    chamferedCylinder(axleR, axleH, axleR * 0.1),
    getMaterial('brushed_steel'),
  )
  g.add(axle)

  // Spokes (radial connecting pieces)
  const spokeCount = is120 ? 8 : 6
  const spokeLen = outerR - hubR
  const spokeW = outerR * 0.06
  const spokeH = shellH * 0.4
  for (let i = 0; i < spokeCount; i++) {
    const a = (i / spokeCount) * Math.PI * 2
    const spoke = new THREE.Mesh(
      chamferedBox(spokeLen * 0.8, spokeW, spokeH, spokeW * 0.15),
      catMetal(0.25),
    )
    const midR = (outerR + hubR) / 2
    spoke.position.set(Math.cos(a) * midR, 0, Math.sin(a) * midR)
    spoke.rotation.y = -a
    g.add(spoke)
  }

  // Side cover plates
  for (const sy of [-1, 1]) {
    const cover = new THREE.Mesh(
      chamferedCylinder(outerR * 0.92, d * 0.03, chamfer * 0.3, 48),
      getMaterial('dark_chrome'),
    )
    cover.position.y = sy * shellH * 0.52
    g.add(cover)
  }

  // Bolt circle on side
  const bolts = boltCircle(outerR * 0.7, outerR * 0.025, is120 ? 8 : 6, d * 0.04)
  bolts.position.y = shellH * 0.52
  g.add(bolts)

  return g
}

// ── Harmonic Drive ──────────────────────────────────────────────────────────

function generateHarmonicDrive(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2
  const chamfer = r * 0.03
  const isLarge = id.includes('large')

  // Lower housing cylinder
  const lowerH = d * 0.45
  const lower = new THREE.Mesh(
    chamferedCylinder(r, lowerH, chamfer, 48),
    catMetal(0.35),
  )
  lower.position.y = -d * 0.15
  g.add(lower)

  // Upper housing cylinder (slightly smaller)
  const upperH = d * 0.35
  const upper = new THREE.Mesh(
    chamferedCylinder(r * 0.88, upperH, chamfer, 48),
    catMetal(0.25),
  )
  upper.position.y = d * 0.2
  g.add(upper)

  // Accent ring between sections
  const midRing = new THREE.Mesh(
    new THREE.TorusGeometry(r * 0.94, r * 0.02, 8, 48),
    getMaterial('dark_chrome'),
  )
  midRing.rotation.x = Math.PI / 2
  midRing.position.y = d * 0.02
  g.add(midRing)

  // Output flange
  const flangeH = d * 0.06
  const flange = flangePlate(r * 0.8, flangeH, isLarge ? 8 : 6, r * 0.6, r * 0.025)
  flange.position.y = d * 0.2 + upperH / 2 + flangeH / 2
  g.add(flange)

  // Output shaft bore
  const boreR = r * 0.15
  const bore = new THREE.Mesh(
    chamferedCylinder(boreR, flangeH * 2, boreR * 0.1),
    getMaterial('dark_chrome'),
  )
  bore.position.y = d * 0.2 + upperH / 2 + flangeH / 2
  g.add(bore)

  // Bottom mounting bolt circle
  const bottomBolts = boltCircle(r * 0.75, r * 0.025, isLarge ? 8 : 6, d * 0.04)
  bottomBolts.position.y = -d * 0.15 - lowerH / 2
  g.add(bottomBolts)

  // Label recess
  const label = labelRecess(r * 0.8, lowerH * 0.4, chamfer * 0.3)
  label.position.set(0, -d * 0.15, r * 0.96)
  g.add(label)

  // Cable exit
  const cableR = r * 0.04
  const cable = new THREE.Mesh(
    chamferedCylinder(cableR, r * 0.15, cableR * 0.1),
    getMaterial('matte_plastic'),
  )
  cable.rotation.z = Math.PI / 2
  cable.position.set(r * 0.85, -d * 0.3, 0)
  g.add(cable)

  return g
}

// ── Pancake Motor ───────────────────────────────────────────────────────────

function generatePancake(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2
  const chamfer = r * 0.04

  // Very flat main body
  const bodyH = d * 0.5
  const body = new THREE.Mesh(
    chamferedCylinder(r, bodyH, chamfer, 48),
    catMetal(0.35),
  )
  g.add(body)

  // Top cover plate
  const coverH = d * 0.05
  const cover = new THREE.Mesh(
    chamferedCylinder(r * 0.95, coverH, chamfer * 0.3, 48),
    getMaterial('brushed_steel'),
  )
  cover.position.y = (bodyH + coverH) / 2
  g.add(cover)

  // Bottom plate
  const botCover = new THREE.Mesh(
    chamferedCylinder(r * 0.95, coverH, chamfer * 0.3, 48),
    getMaterial('brushed_steel'),
  )
  botCover.position.y = -(bodyH + coverH) / 2
  g.add(botCover)

  // Output shaft
  const shaftR = r * 0.08
  const shaftH = d * 0.4
  const shaft = new THREE.Mesh(
    chamferedCylinder(shaftR, shaftH, shaftR * 0.15),
    getMaterial('brushed_steel'),
  )
  shaft.position.y = bodyH / 2 + coverH + shaftH / 2
  g.add(shaft)

  // Mounting bolt circle
  const bolts = boltCircle(r * 0.75, r * 0.025, 6, coverH * 1.1)
  bolts.position.y = (bodyH + coverH) / 2
  g.add(bolts)

  // Decorative body ring
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(r * 1.0, r * 0.015, 6, 48),
    getMaterial('dark_chrome'),
  )
  ring.rotation.x = Math.PI / 2
  g.add(ring)

  // Cable exit
  const cable = new THREE.Mesh(
    chamferedCylinder(r * 0.035, r * 0.12, r * 0.005),
    getMaterial('matte_plastic'),
  )
  cable.rotation.z = Math.PI / 2
  cable.position.set(r * 0.8, -bodyH * 0.2, 0)
  g.add(cable)

  // Label
  const label = labelRecess(r * 0.8, bodyH * 0.5, chamfer * 0.3)
  label.position.set(0, 0, r * 0.93)
  g.add(label)

  return g
}

// ── Brushless Inrunner ──────────────────────────────────────────────────────

function generateBrushlessInrunner(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2
  const chamfer = r * 0.04

  // Main cylindrical body
  const bodyH = d * 0.72
  const body = new THREE.Mesh(
    chamferedCylinder(r, bodyH, chamfer, 40),
    catMetal(0.35),
  )
  g.add(body)

  // Cooling fin ring (multiple thin torus rings along body)
  const finCount = 6
  const finSpacing = bodyH / (finCount + 1)
  for (let i = 1; i <= finCount; i++) {
    const fin = new THREE.Mesh(
      new THREE.TorusGeometry(r * 1.04, r * 0.012, 6, 40),
      getMaterial('anodized_aluminum', 0x444444),
    )
    fin.rotation.x = Math.PI / 2
    fin.position.y = -bodyH / 2 + i * finSpacing
    g.add(fin)
  }

  // Front bearing housing
  const bearingH = d * 0.06
  const bearing = new THREE.Mesh(
    chamferedCylinder(r * 0.8, bearingH, chamfer * 0.3, 32),
    getMaterial('brushed_steel'),
  )
  bearing.position.y = (bodyH + bearingH) / 2
  g.add(bearing)

  // Output shaft
  const shaftR = r * 0.1
  const shaftH = d * 0.35
  const shaft = new THREE.Mesh(
    chamferedCylinder(shaftR, shaftH, shaftR * 0.15),
    getMaterial('brushed_steel'),
  )
  shaft.position.y = bodyH / 2 + bearingH + shaftH / 2
  g.add(shaft)

  // Rear end cap
  const rearCapH = d * 0.06
  const rearCap = new THREE.Mesh(
    chamferedCylinder(r * 0.9, rearCapH, chamfer * 0.3, 32),
    getMaterial('dark_chrome'),
  )
  rearCap.position.y = -(bodyH + rearCapH) / 2
  g.add(rearCap)

  // 3 wire exits (rear)
  const wireColors = [0xcc2222, 0x2222cc, 0x22aa22]
  for (let i = 0; i < 3; i++) {
    const a = ((i / 3) * Math.PI * 2) - Math.PI / 2
    const wire = new THREE.Mesh(
      chamferedCylinder(r * 0.035, d * 0.12, r * 0.005),
      getMaterial('glossy_plastic', wireColors[i]),
    )
    wire.position.set(
      Math.cos(a) * r * 0.5,
      -(bodyH / 2 + rearCapH + d * 0.06),
      Math.sin(a) * r * 0.5,
    )
    g.add(wire)
  }

  // Label recess
  const label = labelRecess(r * 0.9, bodyH * 0.3, chamfer * 0.3)
  label.position.set(0, 0, r * 0.95)
  g.add(label)

  return g
}

// ── Dispatcher ──────────────────────────────────────────────────────────────

export function generateRichMotor(id: string, dims: GeneratorDims): THREE.Group {
  if (id.includes('hub')) return generateHubMotor(id, dims)
  if (id.includes('harmonic')) return generateHarmonicDrive(id, dims)
  if (id.includes('pancake')) return generatePancake(id, dims)
  if (id.includes('brushless') || id.includes('inrunner')) return generateBrushlessInrunner(id, dims)
  if (id.includes('worm')) return generateWormGear(id, dims)
  if (id.includes('coreless')) return generateCoreless(id, dims)
  if (id.includes('gear')) return generateGearMotor(id, dims)
  // Default: DC motor (covers dc_small, dc_medium, dc_large, 130, 540, 775)
  return generateDCMotor(id, dims)
}

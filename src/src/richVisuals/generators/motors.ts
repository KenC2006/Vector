/**
 * Rich visual generators for motor components.
 * DC motors, gear motors, hub motors, harmonic drives, pancake, brushless inrunner.
 *
 * Uses profile-based geometry (revolvedMotor, LatheGeometry, chamferedBox/Cylinder)
 * for Fusion-quality mechanical part visuals.
 */
import * as THREE from 'three'
import type { GeneratorDims } from './index'
import { getMaterial, getTintedMaterial } from '../materials'
import {
  revolvedMotor, chamferedBox, chamferedCylinder,
  boltCircle, mountingHole, labelRecess, flangePlate, cablePort,
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

  const isLarge = id.includes('large') || id.includes('775')
  const isMedium = id.includes('medium') || id.includes('540')

  // Main motor body — single revolved profile (body + rear cap + shaft)
  const shaftR = isLarge ? r * 0.1 : (isMedium ? r * 0.09 : r * 0.07)
  const shaftH = d * 0.3
  const motorMesh = new THREE.Mesh(
    revolvedMotor(r, d * 0.72, shaftR, shaftH, 48),
    catPlastic(0.2),
  )
  g.add(motorMesh)

  // Terminal bumps (2 metal tabs on rear)
  const termW = r * 0.12
  const termH = d * 0.055
  const termD = r * 0.08
  for (const sx of [-1, 1]) {
    const term = new THREE.Mesh(
      chamferedBox(termW, termD, termH, termW * 0.08),
      getMaterial('copper_trace'),
    )
    term.position.set(sx * r * 0.35, -(d * 0.36 + d * 0.05 + termH / 2), 0)
    g.add(term)
  }

  // Body band ring (decorative)
  const bandR = r * 1.005
  const band = new THREE.Mesh(
    new THREE.TorusGeometry(bandR, r * 0.015, 8, 36),
    getMaterial('dark_chrome'),
  )
  band.rotation.x = Math.PI / 2
  band.position.y = d * 0.12
  g.add(band)

  // Second band near bottom
  const band2 = new THREE.Mesh(
    new THREE.TorusGeometry(bandR, r * 0.012, 8, 36),
    getMaterial('dark_chrome'),
  )
  band2.rotation.x = Math.PI / 2
  band2.position.y = -d * 0.2
  g.add(band2)

  // Commutator detail on rear cap
  const commR = r * 0.3
  const commH = d * 0.04
  const comm = new THREE.Mesh(
    chamferedCylinder(commR, commH, commR * 0.05, 24),
    getMaterial('copper_trace'),
  )
  comm.position.y = -(d * 0.36 + d * 0.05 + commH / 2)
  g.add(comm)

  // Label recess on body
  const label = labelRecess(r * 1.0, d * 0.32, r * 0.015)
  label.position.set(0, 0, r * 0.93)
  g.add(label)

  // Mounting holes on front bearing plate (larger motors)
  if (isLarge || isMedium) {
    const holeR = r * 0.035
    for (const sx of [-1, 1]) {
      const hole = mountingHole(holeR, d * 0.05)
      hole.position.set(sx * r * 0.55, d * 0.36, 0)
      g.add(hole)
    }
  }

  // Shaft keyway detail
  const keyway = new THREE.Mesh(
    new THREE.BoxGeometry(shaftR * 0.4, shaftH * 0.8, shaftR * 0.12),
    getMaterial('dark_chrome'),
  )
  keyway.position.set(shaftR * 0.8, d * 0.36 + shaftH * 0.5, 0)
  g.add(keyway)

  // Brush access dimple
  const dimple = new THREE.Mesh(
    chamferedCylinder(r * 0.06, d * 0.015, r * 0.01, 16),
    getMaterial('dark_chrome'),
  )
  dimple.position.set(r * 0.6, -d * 0.25, r * 0.6)
  g.add(dimple)

  return g
}

// ── Gear Motor ──────────────────────────────────────────────────────────────

function generateGearMotor(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const motorR = w * 0.38
  const chamfer = w * 0.02

  const isHeavy = id.includes('heavy')

  // Motor cylinder — revolvedMotor (includes rear cap shape)
  const motorShaftR = motorR * 0.1
  const motorBody = new THREE.Mesh(
    revolvedMotor(motorR, d * 0.45, motorShaftR, 0, 36),
    catPlastic(0.2),
  )
  motorBody.position.y = -d * 0.15
  g.add(motorBody)

  // Terminal bumps on motor rear
  for (const sx of [-1, 1]) {
    const term = new THREE.Mesh(
      chamferedBox(w * 0.04, w * 0.03, d * 0.028, w * 0.005),
      getMaterial('copper_trace'),
    )
    term.position.set(sx * motorR * 0.4, -d * 0.15 - d * 0.225 - d * 0.06, 0)
    g.add(term)
  }

  // Motor body band
  const bandRing = new THREE.Mesh(
    new THREE.TorusGeometry(motorR * 1.005, motorR * 0.012, 6, 36),
    getMaterial('dark_chrome'),
  )
  bandRing.rotation.x = Math.PI / 2
  bandRing.position.y = -d * 0.08
  g.add(bandRing)

  // Gearbox housing — chamferedBox
  const gbW = w * 0.85
  const gbH = h * 0.85
  const gbD = d * 0.35
  const gearbox = new THREE.Mesh(
    chamferedBox(gbW, gbH, gbD, chamfer * 1.8),
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
  const shaftH = d * 0.22
  const shaft = new THREE.Mesh(
    chamferedCylinder(shaftR, shaftH, shaftR * 0.12),
    getMaterial('brushed_steel'),
  )
  shaft.position.y = d * 0.2 + gbD / 2 + shaftH / 2
  g.add(shaft)

  // Shaft D-flat
  const dFlat = new THREE.Mesh(
    new THREE.BoxGeometry(shaftR * 0.4, shaftH * 0.85, shaftR * 0.12),
    getMaterial('dark_chrome'),
  )
  dFlat.position.set(shaftR * 0.8, d * 0.2 + gbD / 2 + shaftH / 2, 0)
  g.add(dFlat)

  // Gearbox face plate accent (bearing boss)
  const facePlate = new THREE.Mesh(
    chamferedCylinder(w * 0.18, gbD * 0.1, chamfer * 0.3, 24),
    getMaterial('brushed_steel'),
  )
  facePlate.position.y = d * 0.2 + gbD * 0.48
  g.add(facePlate)

  // Gearbox parting line ring
  const partLine = new THREE.Mesh(
    new THREE.BoxGeometry(gbW * 0.98, gbH * 0.98, gbD * 0.01),
    getMaterial('dark_chrome'),
  )
  partLine.position.set(0, d * 0.2, 0)
  partLine.rotation.x = Math.PI / 2
  g.add(partLine)

  // Label on gearbox side
  const label = labelRecess(gbW * 0.5, gbD * 0.45, chamfer * 0.3)
  label.position.set(0, d * 0.2, gbH * 0.44)
  g.add(label)

  return g
}

// ── Coreless Motor ──────────────────────────────────────────────────────────

function generateCoreless(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2

  // Tiny body — revolvedMotor
  const shaftR = r * 0.08
  const shaftH = d * 0.35
  const motor = new THREE.Mesh(
    revolvedMotor(r, d * 0.65, shaftR, shaftH, 24),
    catMetal(0.3),
  )
  g.add(motor)

  // Terminal wires (2 thin cylinders)
  for (const sx of [-1, 1]) {
    const wire = new THREE.Mesh(
      chamferedCylinder(r * 0.04, d * 0.15, r * 0.005),
      getMaterial('copper_trace'),
    )
    wire.position.set(sx * r * 0.3, -(d * 0.325 + d * 0.065 + d * 0.075), 0)
    g.add(wire)
  }

  // Body accent ring
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(r * 0.98, r * 0.015, 6, 24),
    getMaterial('dark_chrome'),
  )
  ring.rotation.x = Math.PI / 2
  g.add(ring)

  // Label
  const label = labelRecess(r * 0.7, d * 0.25, r * 0.01)
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

  // Motor cylinder — revolvedMotor
  const motor = new THREE.Mesh(
    revolvedMotor(motorR, d * 0.5, motorR * 0.08, 0, 32),
    catPlastic(0.2),
  )
  g.add(motor)

  // Terminal bumps
  for (const sx of [-1, 1]) {
    const term = new THREE.Mesh(
      chamferedBox(w * 0.03, w * 0.025, d * 0.025, w * 0.004),
      getMaterial('copper_trace'),
    )
    term.position.set(sx * motorR * 0.35, -(d * 0.25 + d * 0.04 + d * 0.012), 0)
    g.add(term)
  }

  // Perpendicular gearbox housing — chamferedBox
  const gbW = w * 0.6
  const gbH = h * 0.5
  const gbD = d * 0.4
  const gearbox = new THREE.Mesh(
    chamferedBox(gbW, gbH, gbD, chamfer * 1.2),
    catMetal(0.35),
  )
  gearbox.position.set(w * 0.3, d * 0.275 * 0.55, 0)
  g.add(gearbox)

  // Perpendicular output shaft (along X)
  const shaftR = w * 0.04
  const shaftH = d * 0.2
  const shaft = new THREE.Mesh(
    chamferedCylinder(shaftR, shaftH, shaftR * 0.15),
    getMaterial('brushed_steel'),
  )
  shaft.rotation.z = Math.PI / 2
  shaft.position.set(w * 0.3 + gbW / 2 + shaftH / 2, d * 0.275 * 0.55, 0)
  g.add(shaft)

  // Mounting holes on gearbox face
  const holeR = w * 0.018
  for (const sz of [-1, 1]) {
    const hole = mountingHole(holeR, gbH * 0.3)
    hole.rotation.z = Math.PI / 2
    hole.position.set(w * 0.3 + gbW * 0.48, d * 0.275 * 0.55, sz * gbD * 0.3)
    g.add(hole)
  }

  // Label on gearbox
  const label = labelRecess(gbW * 0.5, gbD * 0.4, chamfer * 0.3)
  label.position.set(w * 0.3, d * 0.275 * 0.55 + gbH * 0.42, 0)
  label.rotation.x = Math.PI / 2
  g.add(label)

  return g
}

// ── Hub Motor ───────────────────────────────────────────────────────────────

function generateHubMotor(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const outerR = w / 2
  const is120 = id.includes('120')

  // Hub motor donut profile — LatheGeometry with proper toroidal cross-section
  const shellH = d * 0.6
  const hubR = outerR * 0.32
  const pts: THREE.Vector2[] = []
  const hw = shellH / 2

  // Inner hub bore up
  pts.push(new THREE.Vector2(hubR * 0.5, -hw))
  pts.push(new THREE.Vector2(hubR * 0.5, hw))
  // Inner hub wall
  pts.push(new THREE.Vector2(hubR, hw))
  // Spoke web top face
  pts.push(new THREE.Vector2(hubR, hw * 0.85))
  // Bridge to outer ring (spoke-web profile)
  const midR = (hubR + outerR) / 2
  pts.push(new THREE.Vector2(midR, hw * 0.5))
  // Outer ring top chamfer
  pts.push(new THREE.Vector2(outerR * 0.92, hw * 0.7))
  pts.push(new THREE.Vector2(outerR, hw * 0.5))
  // Outer ring body
  pts.push(new THREE.Vector2(outerR, -hw * 0.5))
  // Outer ring bottom chamfer
  pts.push(new THREE.Vector2(outerR * 0.92, -hw * 0.7))
  // Bridge back
  pts.push(new THREE.Vector2(midR, -hw * 0.5))
  pts.push(new THREE.Vector2(hubR, -hw * 0.85))
  pts.push(new THREE.Vector2(hubR, -hw))
  // Close at inner bore
  pts.push(new THREE.Vector2(hubR * 0.5, -hw))

  const hubGeom = new THREE.LatheGeometry(pts, 64)
  const hubMesh = new THREE.Mesh(hubGeom, catMetal(0.4))
  g.add(hubMesh)

  // Axle through center
  const axleR = outerR * 0.08
  const axleH = d * 1.2
  const axle = new THREE.Mesh(
    chamferedCylinder(axleR, axleH, axleR * 0.1),
    getMaterial('brushed_steel'),
  )
  g.add(axle)

  // Side cover plates
  for (const sy of [-1, 1]) {
    const cover = new THREE.Mesh(
      chamferedCylinder(outerR * 0.93, d * 0.025, outerR * 0.01, 48),
      getMaterial('dark_chrome'),
    )
    cover.position.y = sy * shellH * 0.52
    g.add(cover)
  }

  // Bolt circle on side
  const bolts = boltCircle(outerR * 0.7, outerR * 0.025, is120 ? 8 : 6, d * 0.035)
  bolts.position.y = shellH * 0.52
  g.add(bolts)

  // Stator winding peek (copper ring visible between spokes)
  const windingRing = new THREE.Mesh(
    new THREE.TorusGeometry(midR, outerR * 0.04, 8, 48),
    getMaterial('copper_trace'),
  )
  windingRing.rotation.x = Math.PI / 2
  g.add(windingRing)

  // Hall sensor cable exit
  const cable = cablePort(outerR * 0.04, outerR * 0.015)
  cable.position.set(hubR * 0.8, -shellH * 0.45, 0)
  g.add(cable)

  // Magnet indicators (dark marks on outer ring)
  const magnetCount = is120 ? 16 : 12
  for (let i = 0; i < magnetCount; i++) {
    const a = (i / magnetCount) * Math.PI * 2
    const mark = new THREE.Mesh(
      new THREE.BoxGeometry(outerR * 0.03, shellH * 0.3, outerR * 0.006),
      getMaterial('dark_chrome'),
    )
    mark.position.set(Math.cos(a) * outerR * 0.99, 0, Math.sin(a) * outerR * 0.99)
    mark.rotation.y = -a
    g.add(mark)
  }

  // Label recess on outer ring face
  const label = labelRecess(outerR * 0.4, shellH * 0.35, outerR * 0.008)
  label.position.set(0, 0, outerR * 0.97)
  g.add(label)

  return g
}

// ── Harmonic Drive ──────────────────────────────────────────────────────────

function generateHarmonicDrive(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2
  const isLarge = id.includes('large')

  // Harmonic drive stepped profile — LatheGeometry (body -> flange -> output)
  const totalH = d * 0.85
  const hh = totalH / 2
  const bodyR = r
  const flangeR = r * 0.92
  const outputR = r * 0.72
  const boreR = r * 0.14

  const pts: THREE.Vector2[] = []
  // Bottom center to body
  pts.push(new THREE.Vector2(boreR, -hh))
  pts.push(new THREE.Vector2(bodyR, -hh))
  // Body cylinder
  pts.push(new THREE.Vector2(bodyR, -hh * 0.1))
  // Step down to flange
  pts.push(new THREE.Vector2(flangeR, -hh * 0.1))
  pts.push(new THREE.Vector2(flangeR, hh * 0.15))
  // Step down to output section
  pts.push(new THREE.Vector2(outputR, hh * 0.15))
  pts.push(new THREE.Vector2(outputR, hh * 0.85))
  // Output top chamfer
  pts.push(new THREE.Vector2(outputR * 0.95, hh))
  // Top face back to bore
  pts.push(new THREE.Vector2(boreR, hh))
  pts.push(new THREE.Vector2(boreR, -hh))

  const hdGeom = new THREE.LatheGeometry(pts, 64)
  const hdMesh = new THREE.Mesh(hdGeom, catMetal(0.35))
  g.add(hdMesh)

  // Accent ring at body-to-flange step
  const stepRing = new THREE.Mesh(
    new THREE.TorusGeometry(flangeR, r * 0.018, 8, 48),
    getMaterial('dark_chrome'),
  )
  stepRing.rotation.x = Math.PI / 2
  stepRing.position.y = -hh * 0.05
  g.add(stepRing)

  // Accent ring at flange-to-output step
  const stepRing2 = new THREE.Mesh(
    new THREE.TorusGeometry(outputR, r * 0.015, 8, 48),
    getMaterial('dark_chrome'),
  )
  stepRing2.rotation.x = Math.PI / 2
  stepRing2.position.y = hh * 0.15
  g.add(stepRing2)

  // Output flange plate with bolt holes
  const outFlange = flangePlate(outputR * 0.95, d * 0.05, isLarge ? 8 : 6, outputR * 0.7, r * 0.022)
  outFlange.position.y = hh
  g.add(outFlange)

  // Bottom mounting bolt circle
  const bottomBolts = boltCircle(r * 0.78, r * 0.025, isLarge ? 8 : 6, d * 0.04)
  bottomBolts.position.y = -hh
  g.add(bottomBolts)

  // Output bore (dark center hole)
  const bore = new THREE.Mesh(
    chamferedCylinder(boreR, totalH * 1.05, boreR * 0.08),
    getMaterial('dark_chrome'),
  )
  g.add(bore)

  // Label recess on body section
  const label = labelRecess(r * 0.75, d * 0.3, r * 0.015)
  label.position.set(0, -hh * 0.5, r * 0.97)
  g.add(label)

  // Cable exit
  const cable = cablePort(r * 0.04, r * 0.015)
  cable.rotation.set(0, 0, Math.PI / 2)
  cable.position.set(r * 0.88, -hh * 0.6, 0)
  g.add(cable)

  // Strain gauge wire
  const wire = new THREE.Mesh(
    chamferedCylinder(r * 0.03, r * 0.12, r * 0.004),
    getMaterial('rubber_black'),
  )
  wire.rotation.z = Math.PI / 2
  wire.position.set(r * 0.98, -hh * 0.6, 0)
  g.add(wire)

  return g
}

// ── Pancake Motor ───────────────────────────────────────────────────────────

function generatePancake(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2

  // Flat motor body — revolvedMotor (very short body)
  const shaftR = r * 0.08
  const shaftH = d * 0.38
  const motor = new THREE.Mesh(
    revolvedMotor(r, d * 0.42, shaftR, shaftH, 48),
    catMetal(0.35),
  )
  g.add(motor)

  // Extra top cover plate
  const coverH = d * 0.04
  const cover = new THREE.Mesh(
    chamferedCylinder(r * 0.96, coverH, r * 0.01, 48),
    getMaterial('brushed_steel'),
  )
  cover.position.y = d * 0.21 + coverH / 2
  g.add(cover)

  // Mounting bolt circle
  const bolts = boltCircle(r * 0.75, r * 0.025, 6, coverH * 1.1)
  bolts.position.y = d * 0.21 + coverH
  g.add(bolts)

  // Decorative body ring
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(r * 1.0, r * 0.015, 6, 48),
    getMaterial('dark_chrome'),
  )
  ring.rotation.x = Math.PI / 2
  g.add(ring)

  // Cable exit
  const cable = cablePort(r * 0.035, r * 0.012)
  cable.rotation.set(0, 0, Math.PI / 2)
  cable.position.set(r * 0.82, -d * 0.15, 0)
  g.add(cable)

  // Label
  const label = labelRecess(r * 0.8, d * 0.2, r * 0.012)
  label.position.set(0, 0, r * 0.93)
  g.add(label)

  return g
}

// ── Brushless Inrunner ──────────────────────────────────────────────────────

function generateBrushlessInrunner(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2

  // Main body — revolvedMotor
  const shaftR = r * 0.1
  const shaftH = d * 0.33
  const motor = new THREE.Mesh(
    revolvedMotor(r, d * 0.68, shaftR, shaftH, 40),
    catMetal(0.35),
  )
  g.add(motor)

  // Cooling fin rings along body
  const finCount = 6
  const bodyH = d * 0.68
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

  // 3 wire exits (rear, color-coded)
  const wireColors = [0xcc2222, 0x2222cc, 0x22aa22]
  const rearCapY = -(bodyH / 2 + d * 0.068)
  for (let i = 0; i < 3; i++) {
    const a = ((i / 3) * Math.PI * 2) - Math.PI / 2
    const wire = new THREE.Mesh(
      chamferedCylinder(r * 0.035, d * 0.12, r * 0.005),
      getMaterial('glossy_plastic', wireColors[i]),
    )
    wire.position.set(
      Math.cos(a) * r * 0.5,
      rearCapY - d * 0.06,
      Math.sin(a) * r * 0.5,
    )
    g.add(wire)
  }

  // Label recess
  const label = labelRecess(r * 0.9, bodyH * 0.28, r * 0.012)
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

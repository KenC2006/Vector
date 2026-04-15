/**
 * Rich visual generators for sensor components.
 * Depth cameras, LiDAR, IMU, ultrasonic, ToF, force-torque, encoders, etc.
 *
 * Uses profile-based geometry (LatheGeometry, chamferedBox/Cylinder, pcbBoard)
 * for Fusion-quality mechanical part visuals.
 */
import * as THREE from 'three'
import type { GeneratorDims } from './index'
import { getMaterial, getTintedMaterial } from '../materials'
import {
  boltCircle, mountingHole,
  screwHead, labelRecess, knurledRing, pcbBoard,
  connectorBlock, cablePort,
} from '../primitives'
import { nurbsFilletBox, nurbsCylinder, nurbsTorus } from '../nurbs'

const CAT_COLOR: [number, number, number] = [0.20, 0.60, 0.86] // blue

function catMetal(strength = 0.3) {
  return getTintedMaterial('anodized_aluminum', ...CAT_COLOR, strength)
}

function catPlastic(strength = 0.25) {
  return getTintedMaterial('matte_plastic', ...CAT_COLOR, strength)
}

// ── Depth Camera ────────────────────────────────────────────────────────────

function generateDepthCamera(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, h) * 0.05
  void id

  // Main housing — chamferedBox (wide thin box)
  const body = new THREE.Mesh(
    nurbsFilletBox(w, h, d, chamfer, 16),
    getMaterial('matte_plastic'),
  )
  g.add(body)

  // Front face plate (recessed dark panel)
  const faceH = h * 0.72
  const faceW = w * 0.88
  const face = new THREE.Mesh(
    nurbsFilletBox(faceW, faceH, d * 0.06, chamfer * 0.3, 12),
    getMaterial('dark_chrome'),
  )
  face.position.z = d * 0.43
  g.add(face)

  // 3 lenses recessed into front face (NOT stuck on)
  const lensR = h * 0.17
  const lensDepth = d * 0.08
  const lensPositions = [-w * 0.28, 0, w * 0.28]
  const lensRadii = [lensR, h * 0.12, lensR]  // center is smaller (RGB)
  const lensMats = [
    getMaterial('dark_chrome'),
    getMaterial('glossy_plastic', 0x112244),
    getMaterial('dark_chrome'),
  ]

  for (let i = 0; i < 3; i++) {
    const lr = lensRadii[i]

    // Lens recess ring (outer housing, slightly larger, dark)
    const housing = new THREE.Mesh(
      nurbsCylinder(lr * 1.15, lensDepth * 1.2, lr * 0.04, 32),
      getMaterial('matte_plastic', 0x111111),
    )
    housing.rotation.x = Math.PI / 2
    housing.position.set(lensPositions[i], 0, d * 0.42)
    g.add(housing)

    // Actual lens element (recessed inside housing)
    const lens = new THREE.Mesh(
      nurbsCylinder(lr, lensDepth * 0.5, lr * 0.06, 32),
      lensMats[i],
    )
    lens.rotation.x = Math.PI / 2
    lens.position.set(lensPositions[i], 0, d * 0.44)
    g.add(lens)

    // Accent ring around each lens
    const ring = new THREE.Mesh(
      nurbsTorus(lr * 1.18, lensR * 0.05, 24, 6),
      getMaterial('dark_chrome'),
    )
    ring.position.set(lensPositions[i], 0, d * 0.41)
    g.add(ring)
  }

  // Status LED (tiny sphere)
  const led = new THREE.Mesh(
    new THREE.SphereGeometry(h * 0.035, 8, 8),
    getMaterial('glossy_plastic', 0x00ff44),
  )
  led.position.set(w * 0.36, h * 0.26, d * 0.48)
  g.add(led)

  // Bottom mount tab
  const tabW = w * 0.25
  const tabH = h * 0.14
  const tabD = d * 0.6
  const tab = new THREE.Mesh(
    nurbsFilletBox(tabW, tabH, tabD, chamfer * 0.3, 12),
    catMetal(0.25),
  )
  tab.position.set(0, -(h + tabH) / 2, 0)
  g.add(tab)

  // Mounting hole in tab
  const hole = mountingHole(tabW * 0.15, tabH * 1.1)
  hole.position.set(0, -(h + tabH) / 2, 0)
  g.add(hole)

  // Rear connector (USB)
  const conn = connectorBlock(w * 0.12, h * 0.3, d * 0.15, 0x333333)
  conn.position.set(0, 0, -d * 0.45)
  conn.rotation.y = Math.PI
  g.add(conn)

  // Label recess on top
  const label = labelRecess(w * 0.4, d * 0.35, chamfer * 0.3)
  label.position.set(0, h * 0.45, 0)
  label.rotation.x = Math.PI / 2
  g.add(label)

  // Ventilation slots on bottom
  for (let i = -2; i <= 2; i++) {
    const slot = new THREE.Mesh(
      new THREE.BoxGeometry(w * 0.04, h * 0.01, d * 0.4),
      getMaterial('dark_chrome'),
    )
    slot.position.set(i * w * 0.1, -h * 0.47, 0)
    g.add(slot)
  }

  return g
}

// ── LiDAR 2D ────────────────────────────────────────────────────────────────

function generateLidar2D(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2
  void id

  // LiDAR puck — LatheGeometry with smooth base->body->dome profile
  const totalH = d * 0.82
  const hh = totalH / 2
  const baseR = r * 1.05
  const pts: THREE.Vector2[] = []

  // Base (wider, flat bottom)
  pts.push(new THREE.Vector2(0, -hh))
  pts.push(new THREE.Vector2(baseR, -hh))
  pts.push(new THREE.Vector2(baseR, -hh + d * 0.12))
  // Step in to body
  pts.push(new THREE.Vector2(r, -hh + d * 0.12))
  // Body cylinder
  pts.push(new THREE.Vector2(r, hh * 0.4))
  // Dome transition (smooth curve via a few points)
  const domeStartY = hh * 0.4
  const domeSteps = 8
  for (let i = 0; i <= domeSteps; i++) {
    const t = i / domeSteps
    const angle = t * Math.PI / 2
    const dr = r * 0.85 * Math.cos(angle)
    const dy = domeStartY + (hh - domeStartY) * Math.sin(angle)
    pts.push(new THREE.Vector2(dr, dy))
  }
  pts.push(new THREE.Vector2(0, hh))

  const puckGeom = new THREE.LatheGeometry(pts, 48)
  const puck = new THREE.Mesh(puckGeom, catMetal(0.35))
  g.add(puck)

  // Dark sensor band (separate torus around body mid-height)
  const bandR = r * 1.01
  const band = new THREE.Mesh(
    nurbsTorus(bandR, d * 0.055, 48, 8),
    getMaterial('dark_chrome'),
  )
  band.rotation.x = Math.PI / 2
  band.position.y = -hh * 0.1
  g.add(band)

  // Second thinner band above
  const band2 = new THREE.Mesh(
    nurbsTorus(bandR * 0.98, d * 0.025, 48, 6),
    getMaterial('dark_chrome'),
  )
  band2.rotation.x = Math.PI / 2
  band2.position.y = hh * 0.15
  g.add(band2)

  // Mounting holes in base (4 positions)
  const holeR = r * 0.04
  for (let i = 0; i < 4; i++) {
    const a = (i / 4) * Math.PI * 2 + Math.PI / 4
    const hole = mountingHole(holeR, d * 0.13)
    hole.position.set(Math.cos(a) * r * 0.82, -hh, Math.sin(a) * r * 0.82)
    g.add(hole)
  }

  // Cable exit (rear)
  const cable = cablePort(r * 0.06, r * 0.02)
  cable.rotation.set(0, 0, Math.PI / 2)
  cable.position.set(r * 0.92, -d * 0.22, 0)
  g.add(cable)

  // Cable strain relief
  const strain = new THREE.Mesh(
    nurbsCylinder(r * 0.05, r * 0.12, r * 0.006),
    getMaterial('rubber_black'),
  )
  strain.rotation.z = Math.PI / 2
  strain.position.set(r * 1.02, -d * 0.22, 0)
  g.add(strain)

  // Label
  const label = labelRecess(r * 0.7, d * 0.3, r * 0.012)
  label.position.set(0, -hh * 0.4, r * 0.96)
  g.add(label)

  // Status indicator LED
  const led = new THREE.Mesh(
    new THREE.SphereGeometry(r * 0.03, 8, 8),
    getMaterial('glossy_plastic', 0x00ff44),
  )
  led.position.set(r * 0.5, hh * 0.2, r * 0.7)
  g.add(led)

  return g
}

// ── LiDAR 3D ────────────────────────────────────────────────────────────────

function generateLidar3D(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2
  void id

  // 3D LiDAR puck — LatheGeometry with base->body->dome all in one shape
  const totalH = d * 0.88
  const hh = totalH / 2
  const baseR = r * 1.05
  const pts: THREE.Vector2[] = []

  // Base
  pts.push(new THREE.Vector2(0, -hh))
  pts.push(new THREE.Vector2(baseR, -hh))
  pts.push(new THREE.Vector2(baseR, -hh + d * 0.1))
  // Step in to body
  pts.push(new THREE.Vector2(r, -hh + d * 0.1))
  // Body
  pts.push(new THREE.Vector2(r, hh * 0.25))
  // Dome (hemisphere transition, taller than 2D)
  const domeStartY = hh * 0.25
  const domeR = r * 0.82
  const steps = 10
  for (let i = 0; i <= steps; i++) {
    const t = i / steps
    const angle = t * Math.PI / 2
    const dr = domeR * Math.cos(angle)
    const dy = domeStartY + (hh - domeStartY) * Math.sin(angle)
    pts.push(new THREE.Vector2(dr, dy))
  }
  pts.push(new THREE.Vector2(0, hh))

  const puckGeom = new THREE.LatheGeometry(pts, 48)
  const puck = new THREE.Mesh(puckGeom, catMetal(0.35))
  g.add(puck)

  // Dark sensor band
  const band = new THREE.Mesh(
    nurbsTorus(r * 1.01, d * 0.045, 48, 8),
    getMaterial('dark_chrome'),
  )
  band.rotation.x = Math.PI / 2
  band.position.y = -hh * 0.15
  g.add(band)

  // Dome-to-body accent ring
  const domeRing = new THREE.Mesh(
    nurbsTorus(domeR, r * 0.015, 36, 6),
    getMaterial('dark_chrome'),
  )
  domeRing.rotation.x = Math.PI / 2
  domeRing.position.y = domeStartY
  g.add(domeRing)

  // Dome is translucent-looking (glossy plastic overlay)
  const domeOverlay = new THREE.Mesh(
    new THREE.SphereGeometry(domeR * 0.98, 32, 16, 0, Math.PI * 2, 0, Math.PI / 2),
    getMaterial('glossy_plastic', 0x1a1a33),
  )
  domeOverlay.position.y = domeStartY
  g.add(domeOverlay)

  // Mounting holes in base
  const holeR = r * 0.035
  for (let i = 0; i < 4; i++) {
    const a = (i / 4) * Math.PI * 2 + Math.PI / 4
    const hole = mountingHole(holeR, d * 0.11)
    hole.position.set(Math.cos(a) * r * 0.84, -hh, Math.sin(a) * r * 0.84)
    g.add(hole)
  }

  // Connector (rear)
  const conn = connectorBlock(w * 0.12, d * 0.14, w * 0.1, 0x333333)
  conn.position.set(r * 0.87, -d * 0.18, 0)
  conn.rotation.y = Math.PI / 2
  g.add(conn)

  // Label
  const label = labelRecess(r * 0.7, d * 0.25, r * 0.012)
  label.position.set(0, -hh * 0.45, r * 0.97)
  g.add(label)

  // Status LED
  const led = new THREE.Mesh(
    new THREE.SphereGeometry(r * 0.025, 8, 8),
    getMaterial('glossy_plastic', 0x00ff44),
  )
  led.position.set(-r * 0.5, -hh * 0.3, r * 0.8)
  g.add(led)

  return g
}

// ── IMU ─────────────────────────────────────────────────────────────────────

function generateIMU(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  void id

  // PCB board base
  const pcb = pcbBoard(w, h, d * 0.3)
  g.add(pcb)

  // Main IC chip (dark QFN package)
  const chipW = w * 0.35
  const chipH = h * 0.35
  const chipD = d * 0.18
  const chip = new THREE.Mesh(
    nurbsFilletBox(chipW, chipH, chipD, chipW * 0.04, 12),
    getMaterial('matte_plastic'),
  )
  chip.position.set(0, 0, d * 0.24)
  g.add(chip)

  // Chip marking dot (orientation indicator)
  const dot = new THREE.Mesh(
    new THREE.SphereGeometry(chipW * 0.08, 8, 8),
    getMaterial('glossy_plastic', 0xeeeeee),
  )
  dot.position.set(-chipW * 0.35, -chipH * 0.3, d * 0.34)
  g.add(dot)

  // Bypass capacitors (tiny SMD parts)
  for (const sx of [-1, 1]) {
    const cap = new THREE.Mesh(
      nurbsFilletBox(w * 0.08, h * 0.05, d * 0.06, w * 0.004, 12),
      getMaterial('matte_plastic', 0x443322),
    )
    cap.position.set(sx * w * 0.3, h * 0.15, d * 0.2)
    g.add(cap)
  }

  // Crystal oscillator
  const crystal = new THREE.Mesh(
    nurbsFilletBox(w * 0.1, h * 0.08, d * 0.06, w * 0.005, 12),
    getMaterial('brushed_steel'),
  )
  crystal.position.set(w * 0.2, -h * 0.15, d * 0.2)
  g.add(crystal)

  // Pin header (connector along one edge)
  const headerW = w * 0.6
  const headerH = h * 0.08
  const headerD = d * 0.35
  const header = new THREE.Mesh(
    nurbsFilletBox(headerW, headerH, headerD, headerH * 0.1, 12),
    getMaterial('matte_plastic'),
  )
  header.position.set(0, -h * 0.4, d * 0.05)
  g.add(header)

  // Pin header individual pins
  const pinCount = 6
  const pinSpacing = headerW / (pinCount + 1)
  for (let i = 1; i <= pinCount; i++) {
    const pin = new THREE.Mesh(
      new THREE.CylinderGeometry(w * 0.008, w * 0.008, d * 0.15, 6),
      getMaterial('copper_trace'),
    )
    pin.position.set(-headerW / 2 + i * pinSpacing, -h * 0.4, -d * 0.08)
    g.add(pin)
  }

  // Mounting holes (2 corners)
  const holeR = w * 0.03
  for (const sx of [-1, 1]) {
    const hole = mountingHole(holeR, d * 0.35)
    hole.rotation.x = Math.PI / 2
    hole.position.set(sx * w * 0.4, h * 0.35, 0)
    g.add(hole)
  }

  return g
}

// ── Ultrasonic ──────────────────────────────────────────────────────────────

function generateUltrasonic(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, h) * 0.06
  void id

  // Main body — chamferedBox
  const body = new THREE.Mesh(
    nurbsFilletBox(w, h, d, chamfer, 16),
    catMetal(0.3),
  )
  g.add(body)

  // Two "eye" transducer cylinders — recessed into front face
  const eyeR = Math.min(w, h) * 0.22
  const eyeDepth = d * 0.14
  for (const sx of [-1, 1]) {
    // Recess cavity (dark hole in face)
    const recess = new THREE.Mesh(
      nurbsCylinder(eyeR * 1.15, eyeDepth * 0.4, eyeR * 0.02, 32),
      getMaterial('dark_chrome'),
    )
    recess.rotation.x = Math.PI / 2
    recess.position.set(sx * w * 0.22, 0, d * 0.42)
    g.add(recess)

    // Outer housing ring
    const housing = new THREE.Mesh(
      nurbsCylinder(eyeR * 1.1, eyeDepth, eyeR * 0.05, 32),
      getMaterial('glossy_plastic', 0x888888),
    )
    housing.rotation.x = Math.PI / 2
    housing.position.set(sx * w * 0.22, 0, d * 0.44)
    g.add(housing)

    // Inner transducer cone (lighter, recessed)
    const inner = new THREE.Mesh(
      nurbsCylinder(eyeR * 0.78, eyeDepth * 0.5, eyeR * 0.03, 32),
      getMaterial('glossy_plastic', 0xcccccc),
    )
    inner.rotation.x = Math.PI / 2
    inner.position.set(sx * w * 0.22, 0, d * 0.46)
    g.add(inner)

    // Accent ring around transducer
    const ring = new THREE.Mesh(
      nurbsTorus(eyeR * 1.12, eyeR * 0.04, 24, 6),
      getMaterial('dark_chrome'),
    )
    ring.position.set(sx * w * 0.22, 0, d * 0.42)
    g.add(ring)
  }

  // PCB visible through back (partial board)
  const pcbMesh = new THREE.Mesh(
    nurbsFilletBox(w * 0.85, h * 0.75, d * 0.04, chamfer * 0.2, 12),
    getMaterial('pcb_green'),
  )
  pcbMesh.position.z = -d * 0.38
  g.add(pcbMesh)

  // Rear pin header
  const header = new THREE.Mesh(
    nurbsFilletBox(w * 0.55, h * 0.1, d * 0.15, chamfer * 0.2, 12),
    getMaterial('matte_plastic'),
  )
  header.position.set(0, -h * 0.38, -d * 0.38)
  g.add(header)

  // Mounting holes
  const holeR = Math.min(w, h) * 0.03
  for (const sx of [-1, 1]) {
    const hole = mountingHole(holeR, h * 0.2)
    hole.position.set(sx * w * 0.4, h * 0.35, 0)
    g.add(hole)
  }

  // Label
  const label = labelRecess(w * 0.45, h * 0.18, chamfer * 0.3)
  label.position.set(0, h * 0.28, d * 0.42)
  g.add(label)

  return g
}

// ── ToF Sensor ──────────────────────────────────────────────────────────────

function generateToF(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  void id

  // PCB base
  const pcb = pcbBoard(w, h, d * 0.3)
  g.add(pcb)

  // Sensor IC (small dark chip)
  const chipW = w * 0.3
  const chipD = d * 0.15
  const chip = new THREE.Mesh(
    nurbsFilletBox(chipW, chipW, chipD, chipW * 0.04, 12),
    getMaterial('matte_plastic'),
  )
  chip.position.set(0, 0, d * 0.22)
  g.add(chip)

  // Lens aperture (dark cylinder inset in chip)
  const lensR = chipW * 0.28
  const lens = new THREE.Mesh(
    nurbsCylinder(lensR, chipD * 0.7, lensR * 0.05, 16),
    getMaterial('dark_chrome'),
  )
  lens.rotation.x = Math.PI / 2
  lens.position.set(0, 0, d * 0.31)
  g.add(lens)

  // VCSEL emitter (tiny dot next to lens)
  const vcsel = new THREE.Mesh(
    new THREE.SphereGeometry(chipW * 0.06, 8, 8),
    getMaterial('glossy_plastic', 0x880000),
  )
  vcsel.position.set(chipW * 0.3, 0, d * 0.32)
  g.add(vcsel)

  // Bypass caps
  for (const sx of [-1, 1]) {
    const cap = new THREE.Mesh(
      nurbsFilletBox(w * 0.07, h * 0.04, d * 0.05, w * 0.004, 12),
      getMaterial('matte_plastic', 0x443322),
    )
    cap.position.set(sx * w * 0.25, h * 0.2, d * 0.18)
    g.add(cap)
  }

  // Voltage regulator
  const vreg = new THREE.Mesh(
    nurbsFilletBox(w * 0.1, h * 0.06, d * 0.08, w * 0.005, 12),
    getMaterial('matte_plastic'),
  )
  vreg.position.set(-w * 0.2, -h * 0.15, d * 0.2)
  g.add(vreg)

  // Pin header
  const header = new THREE.Mesh(
    nurbsFilletBox(w * 0.4, h * 0.06, d * 0.25, w * 0.005, 12),
    getMaterial('matte_plastic'),
  )
  header.position.set(0, -h * 0.38, d * 0.02)
  g.add(header)

  // Individual pins
  for (let i = 0; i < 4; i++) {
    const pin = new THREE.Mesh(
      new THREE.CylinderGeometry(w * 0.007, w * 0.007, d * 0.12, 6),
      getMaterial('copper_trace'),
    )
    pin.position.set(-w * 0.12 + i * w * 0.08, -h * 0.38, -d * 0.08)
    g.add(pin)
  }

  return g
}

// ── Force Torque 6-axis ─────────────────────────────────────────────────────

function generateForceTorque(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2
  void id

  // Main cylindrical flange — chamferedCylinder
  const bodyH = d * 0.6
  const body = new THREE.Mesh(
    nurbsCylinder(r, bodyH, r * 0.03, 64),
    catMetal(0.35),
  )
  g.add(body)

  // Top bolt circle
  const topBolts = boltCircle(r * 0.76, r * 0.028, 8, d * 0.04)
  topBolts.position.y = bodyH / 2
  g.add(topBolts)

  // Bottom bolt circle
  const botBolts = boltCircle(r * 0.76, r * 0.028, 8, d * 0.04)
  botBolts.position.y = -bodyH / 2
  g.add(botBolts)

  // Top flange face — flangePlate overlay for visual depth
  const topFlange = new THREE.Mesh(
    nurbsCylinder(r * 0.98, bodyH * 0.12, r * 0.02, 64),
    getMaterial('brushed_steel'),
  )
  topFlange.position.y = bodyH * 0.44
  g.add(topFlange)

  // Bottom flange face
  const botFlange = new THREE.Mesh(
    nurbsCylinder(r * 0.98, bodyH * 0.12, r * 0.02, 64),
    getMaterial('brushed_steel'),
  )
  botFlange.position.y = -bodyH * 0.44
  g.add(botFlange)

  // Inner sensing ring (contrasting material)
  const innerR = r * 0.44
  const innerH = bodyH * 0.48
  const inner = new THREE.Mesh(
    nurbsCylinder(innerR, innerH, r * 0.015, 32),
    getMaterial('brushed_steel'),
  )
  inner.position.y = bodyH * 0.05
  g.add(inner)

  // Accent ring (top face)
  const ring = new THREE.Mesh(
    nurbsTorus(r * 0.6, r * 0.015, 48, 6),
    getMaterial('dark_chrome'),
  )
  ring.rotation.x = Math.PI / 2
  ring.position.y = bodyH / 2
  g.add(ring)

  // Mid-body parting line ring
  const midRing = new THREE.Mesh(
    nurbsTorus(r * 1.005, r * 0.008, 48, 6),
    getMaterial('dark_chrome'),
  )
  midRing.rotation.x = Math.PI / 2
  g.add(midRing)

  // Cable exit
  const cable = cablePort(r * 0.05, r * 0.018)
  cable.rotation.set(0, 0, Math.PI / 2)
  cable.position.set(r * 0.92, 0, 0)
  g.add(cable)

  // Strain relief
  const strain = new THREE.Mesh(
    nurbsCylinder(r * 0.04, r * 0.12, r * 0.005),
    getMaterial('rubber_black'),
  )
  strain.rotation.z = Math.PI / 2
  strain.position.set(r * 1.02, 0, 0)
  g.add(strain)

  // Label
  const label = labelRecess(r * 0.6, bodyH * 0.28, r * 0.012)
  label.position.set(0, 0, r * 0.97)
  g.add(label)

  return g
}

// ── Joint Encoder ───────────────────────────────────────────────────────────

function generateEncoder(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2
  const chamfer = r * 0.04
  void id

  // Flat cylindrical body
  const bodyH = d * 0.5
  const body = new THREE.Mesh(
    nurbsCylinder(r, bodyH, chamfer, 36),
    catMetal(0.3),
  )
  g.add(body)

  // Shaft bore hole (dark center)
  const boreR = r * 0.2
  const bore = new THREE.Mesh(
    nurbsCylinder(boreR, bodyH * 1.1, boreR * 0.05, 16),
    getMaterial('dark_chrome'),
  )
  g.add(bore)

  // Top face accent ring
  const ring = new THREE.Mesh(
    nurbsTorus(r * 0.6, r * 0.012, 36, 6),
    getMaterial('dark_chrome'),
  )
  ring.rotation.x = Math.PI / 2
  ring.position.y = bodyH / 2
  g.add(ring)

  // Mounting screw on side
  const screw = screwHead(r * 0.06, d * 0.04)
  screw.rotation.z = Math.PI / 2
  screw.position.set(r * 0.85, 0, 0)
  g.add(screw)

  // Cable exit
  const cable = cablePort(r * 0.04, r * 0.015)
  cable.rotation.set(0, 0, Math.PI / 2)
  cable.position.set(r * 0.82, -bodyH * 0.2, r * 0.4)
  g.add(cable)

  // Label
  const label = labelRecess(r * 0.6, bodyH * 0.4, chamfer * 0.3)
  label.position.set(0, bodyH * 0.2, r * 0.95)
  g.add(label)

  return g
}

// ── Thermal Camera ──────────────────────────────────────────────────────────

function generateThermalCamera(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, h) * 0.06
  void id

  // Main body — chamferedBox
  const body = new THREE.Mesh(
    nurbsFilletBox(w, h, d, chamfer, 16),
    getMaterial('matte_plastic'),
  )
  g.add(body)

  // Single lens on front (recessed)
  const lensR = Math.min(w, h) * 0.25
  const lensDepth = d * 0.1

  // Lens housing recess
  const housingRecess = new THREE.Mesh(
    nurbsCylinder(lensR * 1.2, lensDepth * 0.5, lensR * 0.03, 32),
    getMaterial('dark_chrome'),
  )
  housingRecess.rotation.x = Math.PI / 2
  housingRecess.position.set(0, 0, d * 0.42)
  g.add(housingRecess)

  // Lens housing
  const housing = new THREE.Mesh(
    nurbsCylinder(lensR * 1.15, lensDepth, lensR * 0.05, 32),
    getMaterial('matte_plastic'),
  )
  housing.rotation.x = Math.PI / 2
  housing.position.set(0, 0, d * 0.45)
  g.add(housing)

  // Lens element (dark germanium look)
  const lens = new THREE.Mesh(
    nurbsCylinder(lensR, lensDepth * 0.5, lensR * 0.03, 32),
    getMaterial('glossy_plastic', 0x110022),
  )
  lens.rotation.x = Math.PI / 2
  lens.position.set(0, 0, d * 0.48)
  g.add(lens)

  // Lens ring accent
  const ring = new THREE.Mesh(
    nurbsTorus(lensR * 1.12, lensR * 0.05, 24, 6),
    getMaterial('dark_chrome'),
  )
  ring.position.set(0, 0, d * 0.44)
  g.add(ring)

  // Rear connector
  const conn = connectorBlock(w * 0.15, h * 0.25, d * 0.12, 0x333333)
  conn.position.set(0, 0, -d * 0.45)
  conn.rotation.y = Math.PI
  g.add(conn)

  // Status LED
  const led = new THREE.Mesh(
    new THREE.SphereGeometry(Math.min(w, h) * 0.03, 8, 8),
    getMaterial('glossy_plastic', 0xff3300),
  )
  led.position.set(w * 0.3, h * 0.3, d * 0.48)
  g.add(led)

  // Mounting tab
  const tabW = w * 0.2
  const tabH = h * 0.12
  const tab = new THREE.Mesh(
    nurbsFilletBox(tabW, tabH, d * 0.5, chamfer * 0.3, 12),
    catMetal(0.2),
  )
  tab.position.set(0, -(h + tabH) / 2, 0)
  g.add(tab)

  // Label
  const label = labelRecess(w * 0.4, h * 0.22, chamfer * 0.3)
  label.position.set(0, h * 0.32, d * 0.42)
  g.add(label)

  return g
}

// ── Contact / Limit Switch ──────────────────────────────────────────────────

function generateSwitch(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, h) * 0.05
  void id

  // Main body (small box)
  const body = new THREE.Mesh(
    nurbsFilletBox(w, h, d, chamfer, 16),
    catPlastic(0.2),
  )
  g.add(body)

  // Lever arm
  const leverW = w * 1.5
  const leverH = h * 0.06
  const leverD = d * 0.3
  const lever = new THREE.Mesh(
    nurbsFilletBox(leverW, leverD, leverH, leverH * 0.15, 12),
    getMaterial('brushed_steel'),
  )
  lever.position.set(w * 0.3, h * 0.35, 0)
  g.add(lever)

  // Lever pivot
  const pivotR = Math.min(w, h) * 0.06
  const pivot = new THREE.Mesh(
    nurbsCylinder(pivotR, d * 0.35, pivotR * 0.1, 12),
    getMaterial('dark_chrome'),
  )
  pivot.rotation.x = Math.PI / 2
  pivot.position.set(-w * 0.35, h * 0.35, 0)
  g.add(pivot)

  // Lever roller tip
  const rollerR = Math.min(w, h) * 0.08
  const roller = new THREE.Mesh(
    nurbsCylinder(rollerR, d * 0.25, rollerR * 0.1, 12),
    getMaterial('dark_chrome'),
  )
  roller.rotation.x = Math.PI / 2
  roller.position.set(w * 0.3 + leverW * 0.45, h * 0.35, 0)
  g.add(roller)

  // Terminal pins (bottom)
  for (let i = -1; i <= 1; i++) {
    const pin = new THREE.Mesh(
      nurbsCylinder(w * 0.02, h * 0.2, w * 0.003),
      getMaterial('copper_trace'),
    )
    pin.position.set(i * w * 0.2, -(h + h * 0.2) / 2, 0)
    g.add(pin)
  }

  // Mounting holes
  const holeR = w * 0.03
  for (const sx of [-1, 1]) {
    const hole = mountingHole(holeR, d * 0.3)
    hole.rotation.x = Math.PI / 2
    hole.position.set(sx * w * 0.38, 0, 0)
    g.add(hole)
  }

  return g
}

// ── Rotary Potentiometer ────────────────────────────────────────────────────

function generatePotentiometer(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2
  const chamfer = r * 0.04
  void id

  // Main cylindrical body
  const bodyH = d * 0.55
  const body = new THREE.Mesh(
    nurbsCylinder(r, bodyH, chamfer, 32),
    catMetal(0.3),
  )
  g.add(body)

  // Output shaft
  const shaftR = r * 0.1
  const shaftH = d * 0.2
  const shaft = new THREE.Mesh(
    nurbsCylinder(shaftR, shaftH, shaftR * 0.1),
    getMaterial('brushed_steel'),
  )
  shaft.position.y = (bodyH + shaftH) / 2
  g.add(shaft)

  // Knurled ring on top (adjustment ring)
  const kRing = knurledRing(r * 0.7, d * 0.12, 20)
  kRing.position.y = bodyH / 2 + d * 0.06
  g.add(kRing)

  // Terminal pins (3 pins at bottom)
  for (let i = -1; i <= 1; i++) {
    const pin = new THREE.Mesh(
      nurbsCylinder(r * 0.04, d * 0.2, r * 0.005),
      getMaterial('copper_trace'),
    )
    pin.position.set(i * r * 0.4, -(bodyH + d * 0.2) / 2, 0)
    g.add(pin)
  }

  // Mounting tab
  const tabW = w * 0.15
  const tabH = r * 0.06
  const tabD = d * 0.2
  const tab = new THREE.Mesh(
    nurbsFilletBox(tabW, tabD, tabH, tabH * 0.1, 12),
    getMaterial('brushed_steel'),
  )
  tab.position.set(r * 0.85, bodyH * 0.2, 0)
  g.add(tab)

  // Label
  const label = labelRecess(r * 0.5, bodyH * 0.3, chamfer * 0.3)
  label.position.set(0, 0, r * 0.95)
  g.add(label)

  return g
}

// ── Load Cell ───────────────────────────────────────────────────────────────

function generateLoadCell(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, h) * 0.04
  void id

  // Flat beam body
  const body = new THREE.Mesh(
    nurbsFilletBox(w, h, d, chamfer, 16),
    catMetal(0.3),
  )
  g.add(body)

  // Strain gauge (thin copper strip on top)
  const gaugeW = w * 0.5
  const gaugeH = h * 0.6
  const gaugeD = d * 0.02
  const gauge = new THREE.Mesh(
    nurbsFilletBox(gaugeW, gaugeH, gaugeD, gaugeW * 0.02, 12),
    getMaterial('copper_trace'),
  )
  gauge.position.set(0, 0, d * 0.48)
  g.add(gauge)

  // Strain gauge serpentine pattern
  const lineCount = 5
  const lineSpacing = gaugeH / (lineCount + 1)
  for (let i = 1; i <= lineCount; i++) {
    const line = new THREE.Mesh(
      nurbsFilletBox(gaugeW * 0.8, gaugeH * 0.02, gaugeD * 0.5, gaugeW * 0.005, 12),
      getMaterial('copper_trace'),
    )
    line.position.set(0, -gaugeH / 2 + i * lineSpacing, d * 0.5)
    g.add(line)
  }

  // Mounting holes at both ends
  const holeR = h * 0.1
  for (const sx of [-1, 1]) {
    const hole = mountingHole(holeR, d * 0.5)
    hole.rotation.x = Math.PI / 2
    hole.position.set(sx * w * 0.4, 0, 0)
    g.add(hole)
  }

  // Wire exit
  const cable = cablePort(h * 0.06, h * 0.02)
  cable.position.set(0, -h * 0.35, 0)
  g.add(cable)

  return g
}

// ── Current / Voltage Sensor ────────────────────────────────────────────────

function generateCurrentVoltage(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  void id

  // PCB
  const pcb = pcbBoard(w, h, d * 0.25)
  g.add(pcb)

  // Main IC
  const chipW = w * 0.25
  const chip = new THREE.Mesh(
    nurbsFilletBox(chipW, chipW, d * 0.12, chipW * 0.04, 12),
    getMaterial('matte_plastic'),
  )
  chip.position.set(0, 0, d * 0.2)
  g.add(chip)

  // Terminal blocks (2 on one edge)
  for (const sx of [-1, 1]) {
    const term = connectorBlock(w * 0.18, h * 0.25, d * 0.2, 0x0044aa)
    term.position.set(sx * w * 0.3, -h * 0.3, 0)
    g.add(term)
  }

  // Small capacitors
  for (const sx of [-1, 1]) {
    const cap = new THREE.Mesh(
      nurbsCylinder(w * 0.04, d * 0.15, w * 0.005, 12),
      getMaterial('matte_plastic', 0x222244),
    )
    cap.position.set(sx * w * 0.15, h * 0.2, d * 0.18)
    g.add(cap)
  }

  // Mounting holes
  const holeR = w * 0.025
  for (const sx of [-1, 1]) {
    const hole = mountingHole(holeR, d * 0.3)
    hole.rotation.x = Math.PI / 2
    hole.position.set(sx * w * 0.42, h * 0.35, 0)
    g.add(hole)
  }

  return g
}

// ── Color / Light Sensor ────────────────────────────────────────────────────

function generateColorLight(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  void id

  // Tiny PCB
  const pcb = pcbBoard(w, h, d * 0.3)
  g.add(pcb)

  // Lens dome (half sphere)
  const domeR = Math.min(w, h) * 0.2
  const dome = new THREE.Mesh(
    new THREE.SphereGeometry(domeR, 16, 8, 0, Math.PI * 2, 0, Math.PI / 2),
    getMaterial('glossy_plastic', 0xccccdd),
  )
  dome.position.set(0, 0, d * 0.2)
  g.add(dome)

  // LED emitters (small colored dots around dome)
  const ledColors = [0xff0000, 0x00ff00, 0x0000ff, 0xffffff]
  for (let i = 0; i < ledColors.length; i++) {
    const a = (i / ledColors.length) * Math.PI * 2
    const led = new THREE.Mesh(
      new THREE.SphereGeometry(domeR * 0.15, 6, 6),
      getMaterial('glossy_plastic', ledColors[i]),
    )
    led.position.set(
      Math.cos(a) * domeR * 1.8,
      Math.sin(a) * domeR * 1.8,
      d * 0.18,
    )
    g.add(led)
  }

  // IC chip
  const chip = new THREE.Mesh(
    nurbsFilletBox(w * 0.2, h * 0.2, d * 0.08, w * 0.01, 12),
    getMaterial('matte_plastic'),
  )
  chip.position.set(w * 0.2, -h * 0.15, d * 0.18)
  g.add(chip)

  // Pin header
  const header = new THREE.Mesh(
    nurbsFilletBox(w * 0.4, h * 0.06, d * 0.2, w * 0.005, 12),
    getMaterial('matte_plastic'),
  )
  header.position.set(0, -h * 0.4, d * 0.05)
  g.add(header)

  return g
}

// ── Barometer ───────────────────────────────────────────────────────────────

function generateBarometer(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  void id

  // Tiny PCB
  const pcb = pcbBoard(w, h, d * 0.3)
  g.add(pcb)

  // Pressure sensor dome (small dome)
  const domeR = Math.min(w, h) * 0.15
  const dome = new THREE.Mesh(
    new THREE.SphereGeometry(domeR, 12, 8, 0, Math.PI * 2, 0, Math.PI / 2),
    getMaterial('matte_plastic', 0x555555),
  )
  dome.position.set(0, 0, d * 0.2)
  g.add(dome)

  // Main IC
  const chipW = w * 0.25
  const chip = new THREE.Mesh(
    nurbsFilletBox(chipW, chipW, d * 0.1, chipW * 0.03, 12),
    getMaterial('matte_plastic'),
  )
  chip.position.set(w * 0.15, -h * 0.1, d * 0.18)
  g.add(chip)

  // Bypass cap
  const cap = new THREE.Mesh(
    nurbsFilletBox(w * 0.06, h * 0.04, d * 0.05, w * 0.005, 12),
    getMaterial('matte_plastic', 0x443322),
  )
  cap.position.set(-w * 0.2, h * 0.15, d * 0.18)
  g.add(cap)

  // Pin header
  const header = new THREE.Mesh(
    nurbsFilletBox(w * 0.35, h * 0.06, d * 0.2, w * 0.005, 12),
    getMaterial('matte_plastic'),
  )
  header.position.set(0, -h * 0.38, d * 0.05)
  g.add(header)

  // Mounting holes
  const holeR = w * 0.025
  for (const sx of [-1, 1]) {
    const hole = mountingHole(holeR, d * 0.3)
    hole.rotation.x = Math.PI / 2
    hole.position.set(sx * w * 0.4, h * 0.35, 0)
    g.add(hole)
  }

  return g
}

// ── Dispatcher ──────────────────────────────────────────────────────────────

export function generateRichSensor(id: string, dims: GeneratorDims): THREE.Group {
  if (id.includes('depth_camera')) return generateDepthCamera(id, dims)
  if (id.includes('lidar_3d')) return generateLidar3D(id, dims)
  if (id.includes('lidar_2d') || id.includes('lidar')) return generateLidar2D(id, dims)
  if (id.includes('imu')) return generateIMU(id, dims)
  if (id.includes('ultrasonic')) return generateUltrasonic(id, dims)
  if (id.includes('tof')) return generateToF(id, dims)
  if (id.includes('force_torque')) return generateForceTorque(id, dims)
  if (id.includes('encoder')) return generateEncoder(id, dims)
  if (id.includes('thermal_camera')) return generateThermalCamera(id, dims)
  if (id.includes('contact_switch') || id.includes('limit_switch')) return generateSwitch(id, dims)
  if (id.includes('potentiometer')) return generatePotentiometer(id, dims)
  if (id.includes('load_cell')) return generateLoadCell(id, dims)
  if (id.includes('current') || id.includes('voltage')) return generateCurrentVoltage(id, dims)
  if (id.includes('color') || id.includes('light')) return generateColorLight(id, dims)
  if (id.includes('barometer')) return generateBarometer(id, dims)
  // Fallback: generic small sensor box
  return generateIMU(id, dims)
}

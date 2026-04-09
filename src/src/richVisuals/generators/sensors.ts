/**
 * Rich visual generators for sensor components.
 * Depth cameras, LiDAR, IMU, ultrasonic, ToF, force-torque, encoders, etc.
 */
import * as THREE from 'three'
import type { GeneratorDims } from './index'
import { getMaterial, getTintedMaterial } from '../materials'
import {
  chamferedBox, chamferedCylinder, boltCircle, mountingHole,
  screwHead, labelRecess, knurledRing, pcbBoard,
  connectorBlock,
} from '../primitives'

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
  void id // used for future wide-angle variant logic

  // Main body (wide thin box)
  const body = new THREE.Mesh(
    chamferedBox(w, h, d, chamfer),
    catMetal(0.35),
  )
  g.add(body)

  // Front face plate (slightly recessed dark panel)
  const faceH = h * 0.7
  const faceW = w * 0.85
  const face = new THREE.Mesh(
    chamferedBox(faceW, faceH, d * 0.08, chamfer * 0.3),
    getMaterial('matte_plastic'),
  )
  face.position.z = d * 0.42
  g.add(face)

  // Left IR projector lens (dark circle)
  const lensR = h * 0.18
  const lensDepth = d * 0.06
  const leftLens = new THREE.Mesh(
    chamferedCylinder(lensR, lensDepth, lensR * 0.1, 24),
    getMaterial('dark_chrome'),
  )
  leftLens.rotation.x = Math.PI / 2
  leftLens.position.set(-w * 0.28, 0, d * 0.47)
  g.add(leftLens)

  // Center RGB lens (smaller)
  const rgbR = h * 0.12
  const rgbLens = new THREE.Mesh(
    chamferedCylinder(rgbR, lensDepth, rgbR * 0.08, 24),
    getMaterial('glossy_plastic', 0x112244),
  )
  rgbLens.rotation.x = Math.PI / 2
  rgbLens.position.set(0, 0, d * 0.47)
  g.add(rgbLens)

  // Right IR receiver lens (dark circle)
  const rightLens = new THREE.Mesh(
    chamferedCylinder(lensR, lensDepth, lensR * 0.1, 24),
    getMaterial('dark_chrome'),
  )
  rightLens.rotation.x = Math.PI / 2
  rightLens.position.set(w * 0.28, 0, d * 0.47)
  g.add(rightLens)

  // Lens rings (decorative accent around each lens)
  for (const xPos of [-w * 0.28, 0, w * 0.28]) {
    const r = xPos === 0 ? rgbR * 1.2 : lensR * 1.15
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(r, lensR * 0.06, 6, 24),
      getMaterial('dark_chrome'),
    )
    ring.position.set(xPos, 0, d * 0.46)
    g.add(ring)
  }

  // Status LED (tiny sphere)
  const led = new THREE.Mesh(
    new THREE.SphereGeometry(h * 0.04, 8, 8),
    getMaterial('glossy_plastic', 0x00ff44),
  )
  led.position.set(w * 0.35, h * 0.25, d * 0.48)
  g.add(led)

  // Bottom mount tab
  const tabW = w * 0.25
  const tabH = h * 0.15
  const tabD = d * 0.6
  const tab = new THREE.Mesh(
    chamferedBox(tabW, tabH, tabD, chamfer * 0.3),
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
  const label = labelRecess(w * 0.4, d * 0.4, chamfer * 0.3)
  label.position.set(0, h * 0.45, 0)
  label.rotation.x = Math.PI / 2
  g.add(label)

  return g
}

// ── LiDAR 2D ────────────────────────────────────────────────────────────────

function generateLidar2D(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2
  const chamfer = r * 0.04

  // Base plate
  const baseH = d * 0.15
  const base = new THREE.Mesh(
    chamferedCylinder(r * 1.05, baseH, chamfer, 36),
    getMaterial('matte_plastic'),
  )
  base.position.y = -(d - baseH) / 2
  g.add(base)

  // Main puck body
  const puckH = d * 0.6
  const puck = new THREE.Mesh(
    chamferedCylinder(r, puckH, chamfer, 36),
    catMetal(0.35),
  )
  g.add(puck)

  // Dark sensor band (torus ring around middle)
  const bandR = r * 1.01
  const band = new THREE.Mesh(
    new THREE.TorusGeometry(bandR, d * 0.06, 8, 48),
    getMaterial('dark_chrome'),
  )
  band.rotation.x = Math.PI / 2
  band.position.y = d * 0.05
  g.add(band)

  // Top cap
  const capH = d * 0.08
  const cap = new THREE.Mesh(
    chamferedCylinder(r * 0.85, capH, chamfer * 0.5, 36),
    catMetal(0.2),
  )
  cap.position.y = (puckH + capH) / 2
  g.add(cap)

  // Mounting holes in base (4 positions)
  const holeR = r * 0.04
  for (let i = 0; i < 4; i++) {
    const a = (i / 4) * Math.PI * 2 + Math.PI / 4
    const hole = mountingHole(holeR, baseH * 1.1)
    hole.position.set(Math.cos(a) * r * 0.8, -(d - baseH) / 2, Math.sin(a) * r * 0.8)
    g.add(hole)
  }

  // Cable exit (rear)
  const cable = new THREE.Mesh(
    chamferedCylinder(r * 0.06, r * 0.15, r * 0.008),
    getMaterial('matte_plastic'),
  )
  cable.rotation.z = Math.PI / 2
  cable.position.set(r * 0.9, -d * 0.2, 0)
  g.add(cable)

  // Label
  const label = labelRecess(r * 0.7, puckH * 0.35, chamfer * 0.3)
  label.position.set(0, -d * 0.1, r * 0.95)
  g.add(label)

  return g
}

// ── LiDAR 3D ────────────────────────────────────────────────────────────────

function generateLidar3D(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2
  const chamfer = r * 0.04

  // Base plate
  const baseH = d * 0.12
  const base = new THREE.Mesh(
    chamferedCylinder(r * 1.05, baseH, chamfer, 36),
    getMaterial('matte_plastic'),
  )
  base.position.y = -(d - baseH) / 2
  g.add(base)

  // Main body (taller puck)
  const bodyH = d * 0.55
  const body = new THREE.Mesh(
    chamferedCylinder(r, bodyH, chamfer, 48),
    catMetal(0.35),
  )
  g.add(body)

  // Sensor band (dark ring)
  const band = new THREE.Mesh(
    new THREE.TorusGeometry(r * 1.01, d * 0.05, 8, 48),
    getMaterial('dark_chrome'),
  )
  band.rotation.x = Math.PI / 2
  band.position.y = d * 0.05
  g.add(band)

  // Dome top (hemisphere)
  const domeR = r * 0.8
  const dome = new THREE.Mesh(
    new THREE.SphereGeometry(domeR, 32, 16, 0, Math.PI * 2, 0, Math.PI / 2),
    getMaterial('glossy_plastic', 0x222233),
  )
  dome.position.y = bodyH / 2
  g.add(dome)

  // Top accent ring where dome meets body
  const topRing = new THREE.Mesh(
    new THREE.TorusGeometry(domeR, r * 0.015, 6, 36),
    getMaterial('dark_chrome'),
  )
  topRing.rotation.x = Math.PI / 2
  topRing.position.y = bodyH / 2
  g.add(topRing)

  // Mounting holes
  const holeR = r * 0.035
  for (let i = 0; i < 4; i++) {
    const a = (i / 4) * Math.PI * 2 + Math.PI / 4
    const hole = mountingHole(holeR, baseH * 1.1)
    hole.position.set(Math.cos(a) * r * 0.82, -(d - baseH) / 2, Math.sin(a) * r * 0.82)
    g.add(hole)
  }

  // Connector (rear)
  const conn = connectorBlock(w * 0.12, d * 0.15, w * 0.1, 0x333333)
  conn.position.set(r * 0.85, -d * 0.15, 0)
  conn.rotation.y = Math.PI / 2
  g.add(conn)

  // Label
  const label = labelRecess(r * 0.7, bodyH * 0.3, chamfer * 0.3)
  label.position.set(0, -d * 0.1, r * 0.96)
  g.add(label)

  return g
}

// ── IMU ─────────────────────────────────────────────────────────────────────

function generateIMU(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  // PCB board
  const pcb = pcbBoard(w, h, d * 0.3)
  g.add(pcb)

  // Main IC chip (dark box on top)
  const chipW = w * 0.35
  const chipH = h * 0.35
  const chipD = d * 0.2
  const chip = new THREE.Mesh(
    chamferedBox(chipW, chipH, chipD, chipW * 0.05),
    getMaterial('matte_plastic'),
  )
  chip.position.set(0, 0, d * 0.25)
  g.add(chip)

  // Chip orientation dot
  const dot = new THREE.Mesh(
    new THREE.SphereGeometry(chipW * 0.08, 8, 8),
    getMaterial('glossy_plastic', 0xeeeeee),
  )
  dot.position.set(-chipW * 0.35, -chipH * 0.3, d * 0.36)
  g.add(dot)

  // Bypass capacitors (2 tiny boxes)
  for (const sx of [-1, 1]) {
    const cap = new THREE.Mesh(
      chamferedBox(w * 0.08, h * 0.05, d * 0.06, w * 0.005),
      getMaterial('matte_plastic', 0x443322),
    )
    cap.position.set(sx * w * 0.3, h * 0.15, d * 0.2)
    g.add(cap)
  }

  // Pin header (connector along one edge)
  const headerW = w * 0.6
  const headerH = h * 0.08
  const headerD = d * 0.35
  const header = new THREE.Mesh(
    chamferedBox(headerW, headerH, headerD, headerH * 0.1),
    getMaterial('matte_plastic'),
  )
  header.position.set(0, -h * 0.4, d * 0.05)
  g.add(header)

  // Mounting holes (2 corners)
  const holeR = w * 0.03
  for (const sx of [-1, 1]) {
    const hole = mountingHole(holeR, d * 0.4)
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

  // Main body
  const body = new THREE.Mesh(
    chamferedBox(w, h, d, chamfer),
    catMetal(0.3),
  )
  g.add(body)

  // Two "eye" transducer cylinders on front face
  const eyeR = Math.min(w, h) * 0.22
  const eyeDepth = d * 0.12
  for (const sx of [-1, 1]) {
    // Outer housing ring
    const housing = new THREE.Mesh(
      chamferedCylinder(eyeR * 1.1, eyeDepth, eyeR * 0.05, 24),
      getMaterial('glossy_plastic', 0x888888),
    )
    housing.rotation.x = Math.PI / 2
    housing.position.set(sx * w * 0.22, 0, d * 0.45)
    g.add(housing)

    // Inner mesh/cone (lighter)
    const inner = new THREE.Mesh(
      chamferedCylinder(eyeR * 0.8, eyeDepth * 0.6, eyeR * 0.03, 24),
      getMaterial('glossy_plastic', 0xcccccc),
    )
    inner.rotation.x = Math.PI / 2
    inner.position.set(sx * w * 0.22, 0, d * 0.5)
    g.add(inner)
  }

  // Rear pin header
  const header = new THREE.Mesh(
    chamferedBox(w * 0.6, h * 0.1, d * 0.15, chamfer * 0.2),
    getMaterial('matte_plastic'),
  )
  header.position.set(0, -h * 0.38, -d * 0.35)
  g.add(header)

  // Mounting holes
  const holeR = Math.min(w, h) * 0.03
  for (const sx of [-1, 1]) {
    const hole = mountingHole(holeR, h * 0.2)
    hole.position.set(sx * w * 0.4, h * 0.35, 0)
    g.add(hole)
  }

  // Label
  const label = labelRecess(w * 0.5, h * 0.2, chamfer * 0.3)
  label.position.set(0, h * 0.25, d * 0.42)
  g.add(label)

  return g
}

// ── ToF Sensor ──────────────────────────────────────────────────────────────

function generateToF(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  // PCB
  const pcb = pcbBoard(w, h, d * 0.3)
  g.add(pcb)

  // Sensor IC
  const chipW = w * 0.3
  const chipD = d * 0.15
  const chip = new THREE.Mesh(
    chamferedBox(chipW, chipW, chipD, chipW * 0.04),
    getMaterial('matte_plastic'),
  )
  chip.position.set(0, 0, d * 0.22)
  g.add(chip)

  // Lens aperture hole (dark cylinder inset)
  const lensR = chipW * 0.3
  const lens = new THREE.Mesh(
    chamferedCylinder(lensR, chipD * 0.8, lensR * 0.05, 16),
    getMaterial('dark_chrome'),
  )
  lens.rotation.x = Math.PI / 2
  lens.position.set(0, 0, d * 0.32)
  g.add(lens)

  // Bypass caps
  for (const sx of [-1, 1]) {
    const cap = new THREE.Mesh(
      chamferedBox(w * 0.07, h * 0.04, d * 0.05, w * 0.005),
      getMaterial('matte_plastic', 0x443322),
    )
    cap.position.set(sx * w * 0.25, h * 0.2, d * 0.18)
    g.add(cap)
  }

  // Pin header
  const header = new THREE.Mesh(
    chamferedBox(w * 0.4, h * 0.06, d * 0.25, w * 0.005),
    getMaterial('matte_plastic'),
  )
  header.position.set(0, -h * 0.38, d * 0.02)
  g.add(header)

  return g
}

// ── Force Torque 6-axis ─────────────────────────────────────────────────────

function generateForceTorque(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2
  const chamfer = r * 0.03

  // Main cylindrical flange
  const bodyH = d * 0.6
  const body = new THREE.Mesh(
    chamferedCylinder(r, bodyH, chamfer, 48),
    catMetal(0.35),
  )
  g.add(body)

  // Top bolt circle
  const topBolts = boltCircle(r * 0.75, r * 0.03, 8, d * 0.04)
  topBolts.position.y = bodyH / 2
  g.add(topBolts)

  // Bottom bolt circle
  const botBolts = boltCircle(r * 0.75, r * 0.03, 8, d * 0.04)
  botBolts.position.y = -bodyH / 2
  g.add(botBolts)

  // Inner sensing ring (contrasting material)
  const innerR = r * 0.45
  const innerH = bodyH * 0.5
  const inner = new THREE.Mesh(
    chamferedCylinder(innerR, innerH, chamfer * 0.5, 32),
    getMaterial('brushed_steel'),
  )
  inner.position.y = bodyH * 0.05
  g.add(inner)

  // Accent ring (top face)
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(r * 0.6, r * 0.015, 6, 48),
    getMaterial('dark_chrome'),
  )
  ring.rotation.x = Math.PI / 2
  ring.position.y = bodyH / 2
  g.add(ring)

  // Cable exit
  const cable = new THREE.Mesh(
    chamferedCylinder(r * 0.05, r * 0.15, r * 0.007),
    getMaterial('matte_plastic'),
  )
  cable.rotation.z = Math.PI / 2
  cable.position.set(r * 0.9, 0, 0)
  g.add(cable)

  // Label
  const label = labelRecess(r * 0.6, bodyH * 0.3, chamfer * 0.3)
  label.position.set(0, 0, r * 0.96)
  g.add(label)

  return g
}

// ── Joint Encoder ───────────────────────────────────────────────────────────

function generateEncoder(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: d } = dims
  const r = w / 2
  const chamfer = r * 0.04

  // Flat cylindrical body
  const bodyH = d * 0.5
  const body = new THREE.Mesh(
    chamferedCylinder(r, bodyH, chamfer, 36),
    catMetal(0.3),
  )
  g.add(body)

  // Shaft bore hole (dark center)
  const boreR = r * 0.2
  const bore = new THREE.Mesh(
    chamferedCylinder(boreR, bodyH * 1.1, boreR * 0.05, 16),
    getMaterial('dark_chrome'),
  )
  g.add(bore)

  // Top face accent ring
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(r * 0.6, r * 0.012, 6, 36),
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
  const cable = new THREE.Mesh(
    chamferedCylinder(r * 0.04, r * 0.12, r * 0.005),
    getMaterial('matte_plastic'),
  )
  cable.rotation.z = Math.PI / 2
  cable.position.set(r * 0.8, -bodyH * 0.2, r * 0.4)
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

  // Main body
  const body = new THREE.Mesh(
    chamferedBox(w, h, d, chamfer),
    catMetal(0.3),
  )
  g.add(body)

  // Single lens on front
  const lensR = Math.min(w, h) * 0.25
  const lensDepth = d * 0.1

  // Lens housing
  const housing = new THREE.Mesh(
    chamferedCylinder(lensR * 1.15, lensDepth, lensR * 0.05, 24),
    getMaterial('matte_plastic'),
  )
  housing.rotation.x = Math.PI / 2
  housing.position.set(0, 0, d * 0.45)
  g.add(housing)

  // Lens element (dark glossy)
  const lens = new THREE.Mesh(
    chamferedCylinder(lensR, lensDepth * 0.5, lensR * 0.03, 24),
    getMaterial('glossy_plastic', 0x110022),
  )
  lens.rotation.x = Math.PI / 2
  lens.position.set(0, 0, d * 0.5)
  g.add(lens)

  // Lens ring accent
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(lensR * 1.1, lensR * 0.06, 6, 24),
    getMaterial('dark_chrome'),
  )
  ring.position.set(0, 0, d * 0.45)
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
    chamferedBox(tabW, tabH, d * 0.5, chamfer * 0.3),
    catMetal(0.2),
  )
  tab.position.set(0, -(h + tabH) / 2, 0)
  g.add(tab)

  // Label
  const label = labelRecess(w * 0.4, h * 0.25, chamfer * 0.3)
  label.position.set(0, h * 0.3, d * 0.42)
  g.add(label)

  return g
}

// ── Contact / Limit Switch ──────────────────────────────────────────────────

function generateSwitch(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, h) * 0.05

  // Main body (small box)
  const body = new THREE.Mesh(
    chamferedBox(w, h, d, chamfer),
    catPlastic(0.2),
  )
  g.add(body)

  // Lever arm (thin elongated box)
  const leverW = w * 1.5
  const leverH = h * 0.06
  const leverD = d * 0.3
  const lever = new THREE.Mesh(
    chamferedBox(leverW, leverD, leverH, leverH * 0.15),
    getMaterial('brushed_steel'),
  )
  lever.position.set(w * 0.3, h * 0.35, 0)
  g.add(lever)

  // Lever pivot (small cylinder at hinge point)
  const pivotR = Math.min(w, h) * 0.06
  const pivot = new THREE.Mesh(
    chamferedCylinder(pivotR, d * 0.35, pivotR * 0.1, 12),
    getMaterial('dark_chrome'),
  )
  pivot.rotation.x = Math.PI / 2
  pivot.position.set(-w * 0.35, h * 0.35, 0)
  g.add(pivot)

  // Lever roller tip
  const rollerR = Math.min(w, h) * 0.08
  const roller = new THREE.Mesh(
    chamferedCylinder(rollerR, d * 0.25, rollerR * 0.1, 12),
    getMaterial('dark_chrome'),
  )
  roller.rotation.x = Math.PI / 2
  roller.position.set(w * 0.3 + leverW * 0.45, h * 0.35, 0)
  g.add(roller)

  // Terminal pins (bottom)
  for (let i = -1; i <= 1; i++) {
    const pin = new THREE.Mesh(
      chamferedCylinder(w * 0.02, h * 0.2, w * 0.003),
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

  // Main cylindrical body
  const bodyH = d * 0.55
  const body = new THREE.Mesh(
    chamferedCylinder(r, bodyH, chamfer, 32),
    catMetal(0.3),
  )
  g.add(body)

  // Output shaft
  const shaftR = r * 0.1
  const shaftH = d * 0.2
  const shaft = new THREE.Mesh(
    chamferedCylinder(shaftR, shaftH, shaftR * 0.1),
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
      chamferedCylinder(r * 0.04, d * 0.2, r * 0.005),
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
    chamferedBox(tabW, tabD, tabH, tabH * 0.1),
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

  // Flat beam body
  const body = new THREE.Mesh(
    chamferedBox(w, h, d, chamfer),
    catMetal(0.3),
  )
  g.add(body)

  // Strain gauge (thin copper strip on top)
  const gaugeW = w * 0.5
  const gaugeH = h * 0.6
  const gaugeD = d * 0.02
  const gauge = new THREE.Mesh(
    chamferedBox(gaugeW, gaugeH, gaugeD, gaugeW * 0.02),
    getMaterial('copper_trace'),
  )
  gauge.position.set(0, 0, d * 0.48)
  g.add(gauge)

  // Strain gauge serpentine pattern (thin lines)
  const lineCount = 5
  const lineSpacing = gaugeH / (lineCount + 1)
  for (let i = 1; i <= lineCount; i++) {
    const line = new THREE.Mesh(
      chamferedBox(gaugeW * 0.8, gaugeH * 0.02, gaugeD * 0.5, gaugeW * 0.005),
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
  const cable = new THREE.Mesh(
    chamferedCylinder(h * 0.06, w * 0.1, h * 0.008),
    getMaterial('matte_plastic'),
  )
  cable.rotation.z = Math.PI / 2
  cable.position.set(0, -h * 0.35, 0)
  g.add(cable)

  return g
}

// ── Current / Voltage Sensor ────────────────────────────────────────────────

function generateCurrentVoltage(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  // PCB
  const pcb = pcbBoard(w, h, d * 0.25)
  g.add(pcb)

  // Main IC
  const chipW = w * 0.25
  const chip = new THREE.Mesh(
    chamferedBox(chipW, chipW, d * 0.12, chipW * 0.04),
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
      chamferedCylinder(w * 0.04, d * 0.15, w * 0.005, 12),
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
    chamferedBox(w * 0.2, h * 0.2, d * 0.08, w * 0.01),
    getMaterial('matte_plastic'),
  )
  chip.position.set(w * 0.2, -h * 0.15, d * 0.18)
  g.add(chip)

  // Pin header
  const header = new THREE.Mesh(
    chamferedBox(w * 0.4, h * 0.06, d * 0.2, w * 0.005),
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
    chamferedBox(chipW, chipW, d * 0.1, chipW * 0.03),
    getMaterial('matte_plastic'),
  )
  chip.position.set(w * 0.15, -h * 0.1, d * 0.18)
  g.add(chip)

  // Bypass cap
  const cap = new THREE.Mesh(
    chamferedBox(w * 0.06, h * 0.04, d * 0.05, w * 0.005),
    getMaterial('matte_plastic', 0x443322),
  )
  cap.position.set(-w * 0.2, h * 0.15, d * 0.18)
  g.add(cap)

  // Pin header
  const header = new THREE.Mesh(
    chamferedBox(w * 0.35, h * 0.06, d * 0.2, w * 0.005),
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

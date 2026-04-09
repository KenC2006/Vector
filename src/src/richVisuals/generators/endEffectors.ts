/**
 * Rich visual generators for end-effector components.
 * Grippers, suction tools, magnetic tools, tool changers, holders.
 */
import * as THREE from 'three'
import type { GeneratorDims } from './index'
import { getMaterial, getTintedMaterial } from '../materials'
import {
  chamferedBox, chamferedCylinder, boltCircle, screwHead,
  flangePlate, labelRecess, connectorBlock,
} from '../primitives'

const CAT_COLOR: [number, number, number] = [0.10, 0.74, 0.61]  // teal

function catMetal(strength = 0.3) {
  return getTintedMaterial('anodized_aluminum', ...CAT_COLOR, strength)
}

// ── Parallel Gripper ────────────────────────────────────────────────────────

function generateParallelGripper(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const isLarge = id.includes('large')
  const chamfer = Math.min(w, d) * 0.06

  // Base housing
  const bodyW = w * 0.7
  const bodyH = h * 0.45
  const bodyD = d * 0.8
  const body = new THREE.Mesh(chamferedBox(bodyW, bodyD, bodyH, chamfer), catMetal(0.35))
  g.add(body)

  // Rail strip on front face
  const railW = bodyW * 0.9
  const railH = h * 0.04
  const railD = bodyD * 0.12
  const rail = new THREE.Mesh(
    chamferedBox(railW, railD, railH, chamfer * 0.3),
    getMaterial('brushed_steel'),
  )
  rail.position.set(0, -bodyH * 0.15, bodyD * 0.45)
  g.add(rail)

  // Two parallel fingers
  const fingerW = w * 0.12
  const fingerH = h * 0.5
  const fingerD = d * 0.6
  const fingerSpacing = w * 0.28
  for (const sx of [-1, 1]) {
    // Finger body
    const finger = new THREE.Mesh(
      chamferedBox(fingerW, fingerD, fingerH, chamfer * 0.4),
      catMetal(0.25),
    )
    finger.position.set(sx * fingerSpacing, -bodyH * 0.5 - fingerH * 0.5 + fingerH * 0.05, 0)
    g.add(finger)

    // Rubber finger pad on inner face
    const padW = fingerW * 0.35
    const padH = fingerH * 0.7
    const padD = fingerD * 0.85
    const pad = new THREE.Mesh(
      chamferedBox(padW, padD, padH, chamfer * 0.15),
      getMaterial('rubber_black'),
    )
    pad.position.set(
      sx * fingerSpacing - sx * fingerW * 0.35,
      -bodyH * 0.5 - fingerH * 0.5 + fingerH * 0.05,
      0,
    )
    g.add(pad)
  }

  // Mounting flange on top
  const flangeR = Math.min(w, d) * 0.28
  const flangeH = h * 0.08
  const flange = flangePlate(flangeR, flangeH, isLarge ? 6 : 4, flangeR * 0.7, flangeR * 0.06)
  flange.position.y = bodyH * 0.5 + flangeH * 0.5
  g.add(flange)

  // Label on body
  const label = labelRecess(bodyW * 0.55, bodyH * 0.35, chamfer * 0.4)
  label.position.set(0, 0, bodyD * 0.38)
  g.add(label)

  return g
}

// ── 3-Finger Adaptive ───────────────────────────────────────────────────────

function generate3FingerAdaptive(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.05

  // Cylindrical base
  const baseR = Math.min(w, d) * 0.38
  const baseH = h * 0.35
  const base = new THREE.Mesh(chamferedCylinder(baseR, baseH, chamfer, 36), catMetal(0.4))
  g.add(base)

  // Mounting flange on top
  const flange = flangePlate(baseR * 0.9, h * 0.06, 6, baseR * 0.65, baseR * 0.05)
  flange.position.y = baseH * 0.5 + h * 0.03
  g.add(flange)

  // 3 fingers at 120-degree spacing
  const fingerW = w * 0.1
  const fingerH = h * 0.5
  const fingerD = d * 0.12
  const fingerRadius = baseR * 0.7
  for (let i = 0; i < 3; i++) {
    const angle = (i / 3) * Math.PI * 2
    const fx = Math.cos(angle) * fingerRadius
    const fz = Math.sin(angle) * fingerRadius

    // Finger
    const finger = new THREE.Mesh(
      chamferedBox(fingerW, fingerD, fingerH, chamfer * 0.3),
      catMetal(0.25),
    )
    finger.position.set(fx, -baseH * 0.5 - fingerH * 0.5 + fingerH * 0.05, fz)
    finger.rotation.y = -angle
    g.add(finger)

    // Sphere fingertip
    const tipR = fingerW * 0.55
    const tip = new THREE.Mesh(
      new THREE.SphereGeometry(tipR, 16, 16),
      getMaterial('rubber_black'),
    )
    tip.position.set(fx, -baseH * 0.5 - fingerH + tipR * 0.3, fz)
    g.add(tip)
  }

  // Label on base
  const label = labelRecess(baseR * 1.0, baseH * 0.4, chamfer * 0.3)
  label.position.set(0, 0, baseR * 0.92)
  g.add(label)

  return g
}

// ── Suction Cup ─────────────────────────────────────────────────────────────

function generateSuctionCup(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.05

  // Body tube
  const bodyR = Math.min(w, d) * 0.22
  const bodyH = h * 0.5
  const body = new THREE.Mesh(chamferedCylinder(bodyR, bodyH, chamfer, 32), catMetal(0.35))
  body.position.y = h * 0.1
  g.add(body)

  // Bell-shaped cup (wider cylinder at bottom)
  const cupR = Math.min(w, d) * 0.38
  const cupH = h * 0.2
  const cup = new THREE.Mesh(
    chamferedCylinder(cupR, cupH, chamfer * 0.5, 32),
    getMaterial('rubber_black'),
  )
  cup.position.y = -h * 0.25
  g.add(cup)

  // Inner cup recess (darker)
  const innerR = cupR * 0.7
  const innerH = cupH * 0.5
  const inner = new THREE.Mesh(
    chamferedCylinder(innerR, innerH, chamfer * 0.2, 24),
    getMaterial('matte_plastic', 0x0a0a0a),
  )
  inner.position.y = -h * 0.28
  g.add(inner)

  // Air fitting on top (small cylinder)
  const fittingR = bodyR * 0.35
  const fittingH = h * 0.12
  const fitting = new THREE.Mesh(
    chamferedCylinder(fittingR, fittingH, fittingR * 0.15, 16),
    getMaterial('brushed_steel'),
  )
  fitting.position.y = h * 0.1 + bodyH * 0.5 + fittingH * 0.5
  g.add(fitting)

  // Mounting flange
  const flangeR = bodyR * 1.3
  const flange = flangePlate(flangeR, h * 0.05, 4, flangeR * 0.7, flangeR * 0.06)
  flange.position.y = h * 0.1 + bodyH * 0.5 + fittingH + h * 0.025
  g.add(flange)

  return g
}

// ── Vacuum Pad Array ────────────────────────────────────────────────────────

function generateVacuumPadArray(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.05

  // Flat plate
  const plateH = h * 0.25
  const plate = new THREE.Mesh(chamferedBox(w * 0.85, d * 0.85, plateH, chamfer), catMetal(0.3))
  g.add(plate)

  // 6 suction pad cylinders (3x2 grid)
  const padR = Math.min(w, d) * 0.1
  const padH = h * 0.3
  const cols = 3, rows = 2
  const spacingX = w * 0.25
  const spacingZ = d * 0.25
  for (let col = 0; col < cols; col++) {
    for (let row = 0; row < rows; row++) {
      const px = (col - (cols - 1) / 2) * spacingX
      const pz = (row - (rows - 1) / 2) * spacingZ
      const pad = new THREE.Mesh(
        chamferedCylinder(padR, padH, padR * 0.15, 20),
        getMaterial('rubber_black'),
      )
      pad.position.set(px, -plateH * 0.5 - padH * 0.5, pz)
      g.add(pad)
    }
  }

  // Air manifold (tube across top)
  const manifoldR = Math.min(w, d) * 0.04
  const manifold = new THREE.Mesh(
    chamferedCylinder(manifoldR, w * 0.7, manifoldR * 0.2, 12),
    getMaterial('brushed_steel'),
  )
  manifold.rotation.z = Math.PI / 2
  manifold.position.y = plateH * 0.5 + manifoldR
  g.add(manifold)

  // Mounting flange
  const flange = flangePlate(Math.min(w, d) * 0.2, h * 0.06, 4, Math.min(w, d) * 0.14, Math.min(w, d) * 0.02)
  flange.position.y = plateH * 0.5 + manifoldR * 3
  g.add(flange)

  return g
}

// ── Magnetic Tool ───────────────────────────────────────────────────────────

function generateMagneticTool(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.05

  // Cylindrical body
  const bodyR = Math.min(w, d) * 0.3
  const bodyH = h * 0.6
  const body = new THREE.Mesh(chamferedCylinder(bodyR, bodyH, chamfer, 32), catMetal(0.35))
  g.add(body)

  // Pole face (flat dark disc at bottom)
  const poleR = bodyR * 1.05
  const poleH = h * 0.06
  const pole = new THREE.Mesh(
    chamferedCylinder(poleR, poleH, poleH * 0.15, 32),
    getMaterial('dark_chrome'),
  )
  pole.position.y = -bodyH * 0.5 - poleH * 0.5
  g.add(pole)

  // Accent ring around pole
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(poleR * 0.85, poleR * 0.04, 8, 32),
    getMaterial('copper_trace'),
  )
  ring.rotation.x = Math.PI / 2
  ring.position.y = -bodyH * 0.5
  g.add(ring)

  // Cable exit (top)
  const cableR = bodyR * 0.15
  const cableH = h * 0.15
  const cable = new THREE.Mesh(
    chamferedCylinder(cableR, cableH, cableR * 0.2, 12),
    getMaterial('matte_plastic'),
  )
  cable.position.y = bodyH * 0.5 + cableH * 0.5
  g.add(cable)

  // Mounting flange
  const flange = flangePlate(bodyR * 0.9, h * 0.06, 4, bodyR * 0.65, bodyR * 0.05)
  flange.position.y = bodyH * 0.5 + cableH + h * 0.03
  g.add(flange)

  // Label
  const label = labelRecess(bodyR * 1.2, bodyH * 0.3, chamfer * 0.3)
  label.position.set(0, 0, bodyR * 0.92)
  g.add(label)

  return g
}

// ── Tool Changer ────────────────────────────────────────────────────────────

function generateToolChanger(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.04

  // Flat cylindrical plate
  const plateR = Math.min(w, d) * 0.42
  const plateH = h * 0.3
  const plate = new THREE.Mesh(chamferedCylinder(plateR, plateH, chamfer, 48), catMetal(0.35))
  g.add(plate)

  // Center pilot ring (raised cylinder)
  const pilotR = plateR * 0.35
  const pilotH = h * 0.2
  const pilot = new THREE.Mesh(
    chamferedCylinder(pilotR, pilotH, chamfer * 0.5, 32),
    getMaterial('brushed_steel'),
  )
  pilot.position.y = -plateH * 0.5 - pilotH * 0.5
  g.add(pilot)

  // 3 locating pins (dark_chrome cylinders at 120 degrees)
  const pinR = plateR * 0.06
  const pinH = h * 0.15
  const pinCircleR = plateR * 0.65
  for (let i = 0; i < 3; i++) {
    const angle = (i / 3) * Math.PI * 2
    const pin = new THREE.Mesh(
      chamferedCylinder(pinR, pinH, pinR * 0.2, 12),
      getMaterial('dark_chrome'),
    )
    pin.position.set(
      Math.cos(angle) * pinCircleR,
      -plateH * 0.5 - pinH * 0.5,
      Math.sin(angle) * pinCircleR,
    )
    g.add(pin)
  }

  // Bolt circle on top face
  const bolts = boltCircle(plateR * 0.75, plateR * 0.04, 6, plateH * 1.05)
  bolts.position.y = 0
  g.add(bolts)

  // Mounting flange on top
  const flange = flangePlate(plateR * 0.85, h * 0.07, 6, plateR * 0.6, plateR * 0.04)
  flange.position.y = plateH * 0.5 + h * 0.035
  g.add(flange)

  // Electrical connector block
  const conn = connectorBlock(plateR * 0.3, plateR * 0.2, plateR * 0.15, 0x222222)
  conn.position.set(plateR * 0.6, 0, 0)
  g.add(conn)

  return g
}

// ── Soft Gripper ────────────────────────────────────────────────────────────

function generateSoftGripper(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.05

  // Cylindrical base
  const baseR = Math.min(w, d) * 0.3
  const baseH = h * 0.25
  const base = new THREE.Mesh(chamferedCylinder(baseR, baseH, chamfer, 32), catMetal(0.35))
  g.add(base)

  // 4 tapered soft fingers at 90-degree spacing
  const fingerCount = 4
  const fingerRadius = baseR * 0.85
  for (let i = 0; i < fingerCount; i++) {
    const angle = (i / fingerCount) * Math.PI * 2
    const fx = Math.cos(angle) * fingerRadius
    const fz = Math.sin(angle) * fingerRadius

    // Tapered finger (thicker at base, thinner at tip)
    const fingerBaseR = Math.min(w, d) * 0.08
    const fingerTipR = fingerBaseR * 0.5
    const fingerH = h * 0.55
    const finger = new THREE.Mesh(
      chamferedCylinder(fingerBaseR, fingerH, fingerBaseR * 0.1, 16),
      getMaterial('rubber_black'),
    )
    finger.position.set(fx, -baseH * 0.5 - fingerH * 0.5, fz)
    g.add(finger)

    // Thinner tip section
    const tipSectionH = fingerH * 0.4
    const tipSection = new THREE.Mesh(
      chamferedCylinder(fingerTipR, tipSectionH, fingerTipR * 0.1, 12),
      getMaterial('rubber_black'),
    )
    tipSection.position.set(fx, -baseH * 0.5 - fingerH - tipSectionH * 0.4, fz)
    g.add(tipSection)

    // Sphere tip
    const tipR = fingerTipR * 0.9
    const tip = new THREE.Mesh(
      new THREE.SphereGeometry(tipR, 12, 12),
      getMaterial('rubber_black'),
    )
    tip.position.set(fx, -baseH * 0.5 - fingerH - tipSectionH * 0.7, fz)
    g.add(tip)
  }

  // Mounting flange on top
  const flange = flangePlate(baseR * 0.85, h * 0.06, 4, baseR * 0.6, baseR * 0.05)
  flange.position.y = baseH * 0.5 + h * 0.03
  g.add(flange)

  return g
}

// ── Tool Holder (welding torch, pen marker, screwdriver bit) ────────────────

function generateToolHolder(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.05

  // Clamp ring
  const ringR = Math.min(w, d) * 0.32
  const ringH = h * 0.2
  const ring = new THREE.Mesh(
    chamferedCylinder(ringR, ringH, chamfer * 0.5, 32),
    catMetal(0.3),
  )
  ring.position.y = -h * 0.1
  g.add(ring)

  // Body tube
  const tubeR = ringR * 0.85
  const tubeH = h * 0.45
  const tube = new THREE.Mesh(
    chamferedCylinder(tubeR, tubeH, chamfer * 0.4, 28),
    catMetal(0.35),
  )
  tube.position.y = h * 0.1
  g.add(tube)

  // Inner bore (dark inset)
  const boreR = tubeR * 0.55
  const bore = new THREE.Mesh(
    chamferedCylinder(boreR, tubeH * 0.3, boreR * 0.1, 16),
    getMaterial('dark_chrome'),
  )
  bore.position.y = -h * 0.15
  g.add(bore)

  // Mounting flange on top
  const flange = flangePlate(ringR * 0.9, h * 0.07, 4, ringR * 0.65, ringR * 0.05)
  flange.position.y = h * 0.1 + tubeH * 0.5 + h * 0.035
  g.add(flange)

  // Clamping screw
  const screw = screwHead(ringR * 0.12, ringH * 0.4)
  screw.position.set(ringR * 0.95, -h * 0.1, 0)
  screw.rotation.z = Math.PI / 2
  g.add(screw)

  // Cable exit (only for welding torch)
  if (id.includes('welding')) {
    const cableR = tubeR * 0.12
    const cable = new THREE.Mesh(
      new THREE.CylinderGeometry(cableR, cableR, tubeR * 0.4, 8),
      getMaterial('matte_plastic'),
    )
    cable.rotation.z = Math.PI / 2
    cable.position.set(tubeR * 0.8, h * 0.2, 0)
    g.add(cable)
  }

  // Label
  const label = labelRecess(ringR * 1.0, ringH * 0.6, chamfer * 0.3)
  label.position.set(0, -h * 0.1, ringR * 0.93)
  g.add(label)

  return g
}

// ── Dispatcher ──────────────────────────────────────────────────────────────

export function generateRichEndEffector(id: string, dims: GeneratorDims): THREE.Group {
  if (id.includes('parallel_gripper')) return generateParallelGripper(id, dims)
  if (id.includes('3finger') || id.includes('three_finger')) return generate3FingerAdaptive(id, dims)
  if (id.includes('suction_cup')) return generateSuctionCup(id, dims)
  if (id.includes('vacuum_pad')) return generateVacuumPadArray(id, dims)
  if (id.includes('magnetic')) return generateMagneticTool(id, dims)
  if (id.includes('tool_changer')) return generateToolChanger(id, dims)
  if (id.includes('soft_gripper') || id.includes('compliant')) return generateSoftGripper(id, dims)
  if (id.includes('holder') || id.includes('torch')) return generateToolHolder(id, dims)
  // Default: parallel gripper
  return generateParallelGripper(id, dims)
}

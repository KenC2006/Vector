/**
 * Rich visual generators for end-effector components.
 * Fusion-quality visuals using profile-based geometry.
 * Grippers, suction tools, tool changers, soft grippers, holders.
 */
import * as THREE from 'three'
import type { ComponentVisualDims } from './index'
import { getMaterial, getTintedMaterial } from '../materials'
import {
  boltCircle, screwHead,
  flangePlate, knurledRing, labelRecess, connectorBlock, cablePort,
} from '../primitives'
import { nurbsFilletBox, nurbsCylinder, nurbsTorus } from '../nurbs'

const DEFAULT_COLOR: [number, number, number] = [0.10, 0.74, 0.61]  // teal
let CAT_COLOR: [number, number, number] = DEFAULT_COLOR

function catMetal(strength = 0.3) {
  return getTintedMaterial('anodized_aluminum', ...CAT_COLOR, strength)
}

// ── Parallel Gripper ────────────────────────────────────────────────────────

function generateParallelGripper(id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const isLarge = id.includes('large')
  const chamfer = Math.min(w, d) * 0.06

  // Base housing — chamferedBox
  const bodyW = w * 0.7
  const bodyH = h * 0.4
  const bodyD = d * 0.75
  const body = new THREE.Mesh(nurbsFilletBox(bodyW, bodyH, bodyD, chamfer, 16), catMetal(0.35))
  g.add(body)

  // Rail detail — thin raised chamferedBox across front face (fingers slide on this)
  const railW = bodyW * 0.92
  const railThick = h * 0.035
  const railDepth = bodyD * 0.1
  const rail = new THREE.Mesh(
    nurbsFilletBox(railW, railThick, railDepth, chamfer * 0.2, 12),
    getMaterial('brushed_steel'),
  )
  rail.position.set(0, -bodyH * 0.22, bodyD * 0.43)
  g.add(rail)

  // Second rail line (parallel guide)
  const rail2 = new THREE.Mesh(
    nurbsFilletBox(railW, railThick * 0.7, railDepth * 0.6, chamfer * 0.15, 12),
    getMaterial('brushed_steel'),
  )
  rail2.position.set(0, -bodyH * 0.08, bodyD * 0.43)
  g.add(rail2)

  // Two parallel finger assemblies
  const fingerW = w * 0.13
  const fingerH = h * 0.5
  const fingerD = d * 0.55
  const fingerSpacing = w * 0.28
  for (const sx of [-1, 1]) {
    // Finger slider block (rides on rail)
    const sliderW = fingerW * 1.3
    const sliderH = bodyH * 0.25
    const sliderD = bodyD * 0.2
    const slider = new THREE.Mesh(
      nurbsFilletBox(sliderW, sliderH, sliderD, chamfer * 0.2, 12),
      catMetal(0.2),
    )
    slider.position.set(sx * fingerSpacing, -bodyH * 0.15, bodyD * 0.42)
    g.add(slider)

    // Finger body
    const finger = new THREE.Mesh(
      nurbsFilletBox(fingerW, fingerH, fingerD, chamfer * 0.4, 16),
      catMetal(0.25),
    )
    finger.position.set(sx * fingerSpacing, -bodyH * 0.5 - fingerH * 0.5 + fingerH * 0.05, 0)
    g.add(finger)

    // Rubber finger pad on inner face — thin box
    const padW = fingerW * 0.3
    const padH = fingerH * 0.75
    const padD = fingerD * 0.88
    const pad = new THREE.Mesh(
      nurbsFilletBox(padW, padH, padD, chamfer * 0.1, 12),
      getMaterial('rubber_black'),
    )
    pad.position.set(
      sx * fingerSpacing - sx * fingerW * 0.38,
      -bodyH * 0.5 - fingerH * 0.5 + fingerH * 0.05,
      0,
    )
    g.add(pad)
  }

  // Mounting flange on top — chamferedCylinder plate
  const flangeR = Math.min(w, d) * 0.28
  const flangeH = h * 0.08
  const flange = flangePlate(flangeR, flangeH, isLarge ? 6 : 4, flangeR * 0.7, flangeR * 0.06)
  flange.position.y = bodyH * 0.5 + flangeH * 0.5
  g.add(flange)

  // Label on body front
  const label = labelRecess(bodyW * 0.5, bodyH * 0.3, chamfer * 0.4)
  label.position.set(0, bodyH * 0.08, bodyD * 0.39)
  g.add(label)

  // Cable port on back
  const cp = cablePort(Math.min(w, d) * 0.04, Math.min(w, d) * 0.012)
  cp.position.set(0, bodyH * 0.2, -bodyD * 0.4)
  cp.rotation.y = Math.PI
  g.add(cp)

  return g
}

// ── 3-Finger Adaptive ───────────────────────────────────────────────────────

function generate3FingerAdaptive(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const chamfer = Math.min(w, d) * 0.05

  // Cylindrical base
  const baseR = Math.min(w, d) * 0.38
  const baseH = h * 0.32
  const base = new THREE.Mesh(nurbsCylinder(baseR, baseH, chamfer, 36), catMetal(0.4))
  g.add(base)

  // Mounting flange on top
  const flange = flangePlate(baseR * 0.9, h * 0.06, 6, baseR * 0.65, baseR * 0.05)
  flange.position.y = baseH * 0.5 + h * 0.03
  g.add(flange)

  // 3 finger assemblies at 120-degree spacing
  const fingerW = w * 0.1
  const fingerH = h * 0.48
  const fingerD = d * 0.12
  const fingerRadius = baseR * 0.72
  for (let i = 0; i < 3; i++) {
    const angle = (i / 3) * Math.PI * 2
    const fx = Math.cos(angle) * fingerRadius
    const fz = Math.sin(angle) * fingerRadius

    // Finger body — chamferedBox
    const finger = new THREE.Mesh(
      nurbsFilletBox(fingerW, fingerH, fingerD, chamfer * 0.3, 16),
      catMetal(0.25),
    )
    finger.position.set(fx, -baseH * 0.5 - fingerH * 0.5 + fingerH * 0.05, fz)
    finger.rotation.y = -angle
    g.add(finger)

    // Knurled ring fingertip
    const tipR = fingerW * 0.5
    const tipH = fingerD * 0.9
    const tip = knurledRing(tipR, tipH, 12)
    tip.position.set(
      Math.cos(angle) * fingerRadius,
      -baseH * 0.5 - fingerH + tipR * 0.6,
      Math.sin(angle) * fingerRadius,
    )
    tip.rotation.y = -angle
    g.add(tip)
  }

  // Label on base
  const label = labelRecess(baseR * 1.0, baseH * 0.4, chamfer * 0.3)
  label.position.set(0, 0, baseR * 0.92)
  g.add(label)

  return g
}

// ── Suction Cup ─────────────────────────────────────────────────────────────

function generateSuctionCup(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  // Revolved bell shape via LatheGeometry (narrow tube -> wide bell -> thin lip)
  const tubeR = Math.min(w, d) * 0.12
  const bellR = Math.min(w, d) * 0.38
  const totalH = h * 0.7
  const tubeH = totalH * 0.45
  const bellH = totalH * 0.35
  const lipH = totalH * 0.08

  const pts: THREE.Vector2[] = []
  // Top center (narrow tube start)
  pts.push(new THREE.Vector2(0, totalH * 0.5))
  pts.push(new THREE.Vector2(tubeR, totalH * 0.5))
  // Tube body down
  pts.push(new THREE.Vector2(tubeR, totalH * 0.5 - tubeH))
  // Transition to bell — smooth curve outward
  const bellTop = totalH * 0.5 - tubeH
  const bellBot = bellTop - bellH
  const curveSteps = 12
  for (let i = 0; i <= curveSteps; i++) {
    const t = i / curveSteps
    const angle = t * Math.PI * 0.5
    const r = tubeR + (bellR - tubeR) * Math.sin(angle)
    const y = bellTop - bellH * t
    pts.push(new THREE.Vector2(r, y))
  }
  // Lip (thin rolled edge at bottom)
  const lipBot = bellBot - lipH
  pts.push(new THREE.Vector2(bellR * 1.02, lipBot + lipH * 0.6))
  pts.push(new THREE.Vector2(bellR * 0.95, lipBot))
  pts.push(new THREE.Vector2(bellR * 0.82, lipBot + lipH * 0.2))
  // Inner wall curves back up
  pts.push(new THREE.Vector2(bellR * 0.65, bellBot))
  pts.push(new THREE.Vector2(tubeR * 0.8, bellTop + bellH * 0.1))
  // Close at center bottom
  pts.push(new THREE.Vector2(0, bellTop + bellH * 0.1))

  const cupGeom = new THREE.LatheGeometry(pts, 48)
  const cup = new THREE.Mesh(cupGeom, getMaterial('rubber_black'))
  g.add(cup)

  // Air fitting on top (small brushed steel cylinder)
  const fittingR = tubeR * 0.6
  const fittingH = h * 0.1
  const fitting = new THREE.Mesh(
    nurbsCylinder(fittingR, fittingH, fittingR * 0.15, 16),
    getMaterial('brushed_steel'),
  )
  fitting.position.y = totalH * 0.5 + fittingH * 0.5
  g.add(fitting)

  // Mounting flange
  const flangeR = tubeR * 2.2
  const flange = flangePlate(flangeR, h * 0.05, 4, flangeR * 0.7, flangeR * 0.06)
  flange.position.y = totalH * 0.5 + fittingH + h * 0.025
  g.add(flange)

  return g
}

// ── Vacuum Pad Array ────────────────────────────────────────────────────────

function generateVacuumPadArray(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const chamfer = Math.min(w, d) * 0.05

  // Flat plate
  const plateH = h * 0.25
  const plate = new THREE.Mesh(nurbsFilletBox(w * 0.85, plateH, d * 0.85, chamfer, 16), catMetal(0.3))
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
        nurbsCylinder(padR, padH, padR * 0.15, 20),
        getMaterial('rubber_black'),
      )
      pad.position.set(px, -plateH * 0.5 - padH * 0.5, pz)
      g.add(pad)
    }
  }

  // Air manifold tube
  const manifoldR = Math.min(w, d) * 0.04
  const manifold = new THREE.Mesh(
    nurbsCylinder(manifoldR, w * 0.7, manifoldR * 0.2, 12),
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

function generateMagneticTool(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const chamfer = Math.min(w, d) * 0.05

  // Cylindrical body
  const bodyR = Math.min(w, d) * 0.3
  const bodyH = h * 0.6
  const body = new THREE.Mesh(nurbsCylinder(bodyR, bodyH, chamfer, 32), catMetal(0.35))
  g.add(body)

  // Pole face disc at bottom
  const poleR = bodyR * 1.05
  const poleH = h * 0.06
  const pole = new THREE.Mesh(
    nurbsCylinder(poleR, poleH, poleH * 0.15, 32),
    getMaterial('dark_chrome'),
  )
  pole.position.y = -bodyH * 0.5 - poleH * 0.5
  g.add(pole)

  // Accent ring
  const ring = new THREE.Mesh(
    nurbsTorus(poleR * 0.85, poleR * 0.04, 32, 8),
    getMaterial('copper_trace'),
  )
  ring.rotation.x = Math.PI / 2
  ring.position.y = -bodyH * 0.5
  g.add(ring)

  // Cable exit (top)
  const cableR = bodyR * 0.15
  const cableH = h * 0.15
  const cable = new THREE.Mesh(
    nurbsCylinder(cableR, cableH, cableR * 0.2, 12),
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

function generateToolChanger(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const chamfer = Math.min(w, d) * 0.04

  // Flat cylindrical plate
  const plateR = Math.min(w, d) * 0.42
  const plateH = h * 0.3
  const plate = new THREE.Mesh(nurbsCylinder(plateR, plateH, chamfer, 48), catMetal(0.35))
  g.add(plate)

  // Center pilot — raised chamferedCylinder
  const pilotR = plateR * 0.35
  const pilotH = h * 0.22
  const pilot = new THREE.Mesh(
    nurbsCylinder(pilotR, pilotH, chamfer * 0.5, 32),
    getMaterial('brushed_steel'),
  )
  pilot.position.y = -plateH * 0.5 - pilotH * 0.5
  g.add(pilot)

  // 3 locating pin cylinders at 120 degrees
  const pinR = plateR * 0.065
  const pinH = h * 0.16
  const pinCircleR = plateR * 0.65
  for (let i = 0; i < 3; i++) {
    const angle = (i / 3) * Math.PI * 2
    const pin = new THREE.Mesh(
      nurbsCylinder(pinR, pinH, pinR * 0.2, 12),
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

function generateSoftGripper(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const chamfer = Math.min(w, d) * 0.05

  // Cylindrical base
  const baseR = Math.min(w, d) * 0.3
  const baseH = h * 0.25
  const base = new THREE.Mesh(nurbsCylinder(baseR, baseH, chamfer, 32), catMetal(0.35))
  g.add(base)

  // 4 tapered soft fingers at 90-degree spacing
  // Each finger is 3 stacked chamferedCylinders getting narrower (rubber_black)
  const fingerCount = 4
  const fingerCircleR = baseR * 0.82
  for (let i = 0; i < fingerCount; i++) {
    const angle = (i / fingerCount) * Math.PI * 2
    const fx = Math.cos(angle) * fingerCircleR
    const fz = Math.sin(angle) * fingerCircleR

    // Segment radii (taper down)
    const segCount = 3
    const baseSegR = Math.min(w, d) * 0.08
    const segH = h * 0.18
    let currentY = -baseH * 0.5
    for (let s = 0; s < segCount; s++) {
      const taper = 1.0 - s * 0.25  // 1.0, 0.75, 0.5
      const segR = baseSegR * taper
      const seg = new THREE.Mesh(
        nurbsCylinder(segR, segH, segR * 0.12, 16),
        getMaterial('rubber_black'),
      )
      seg.position.set(fx, currentY - segH * 0.5, fz)
      g.add(seg)
      currentY -= segH * 0.92  // slight overlap
    }
  }

  // Mounting flange on top
  const flange = flangePlate(baseR * 0.85, h * 0.06, 4, baseR * 0.6, baseR * 0.05)
  flange.position.y = baseH * 0.5 + h * 0.03
  g.add(flange)

  // Cable port on side
  const cp = cablePort(Math.min(w, d) * 0.035, Math.min(w, d) * 0.01)
  cp.position.set(baseR * 0.9, baseH * 0.1, 0)
  cp.rotation.z = Math.PI / 2
  g.add(cp)

  return g
}

// ── Tool Holder (welding torch, pen marker, screwdriver bit) ────────────────

function generateToolHolder(id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const chamfer = Math.min(w, d) * 0.05

  // Clamp ring — chamferedCylinder
  const ringR = Math.min(w, d) * 0.32
  const ringH = h * 0.18
  const ring = new THREE.Mesh(
    nurbsCylinder(ringR, ringH, chamfer * 0.5, 32),
    catMetal(0.3),
  )
  ring.position.y = -h * 0.08
  g.add(ring)

  // Body tube — chamferedCylinder
  const tubeR = ringR * 0.82
  const tubeH = h * 0.45
  const tube = new THREE.Mesh(
    nurbsCylinder(tubeR, tubeH, chamfer * 0.4, 28),
    catMetal(0.35),
  )
  tube.position.y = h * 0.1
  g.add(tube)

  // Clamping screw on clamp ring
  const screw = screwHead(ringR * 0.12, ringH * 0.4)
  screw.position.set(ringR * 0.95, -h * 0.08, 0)
  screw.rotation.z = Math.PI / 2
  g.add(screw)

  // Inner bore (dark inset)
  const boreR = tubeR * 0.55
  const bore = new THREE.Mesh(
    nurbsCylinder(boreR, tubeH * 0.3, boreR * 0.1, 16),
    getMaterial('dark_chrome'),
  )
  bore.position.y = -h * 0.15
  g.add(bore)

  // Mounting flange on top
  const flange = flangePlate(ringR * 0.9, h * 0.07, 4, ringR * 0.65, ringR * 0.05)
  flange.position.y = h * 0.1 + tubeH * 0.5 + h * 0.035
  g.add(flange)

  // Cable exit (only for welding torch)
  if (id.includes('welding')) {
    const cableR2 = tubeR * 0.12
    const cable = new THREE.Mesh(
      new THREE.CylinderGeometry(cableR2, cableR2, tubeR * 0.4, 8),
      getMaterial('matte_plastic'),
    )
    cable.rotation.z = Math.PI / 2
    cable.position.set(tubeR * 0.8, h * 0.2, 0)
    g.add(cable)
  }

  // Label
  const label = labelRecess(ringR * 1.0, ringH * 0.6, chamfer * 0.3)
  label.position.set(0, -h * 0.08, ringR * 0.93)
  g.add(label)

  return g
}

// ── Dispatcher ──────────────────────────────────────────────────────────────

export function generateRichEndEffector(id: string, dims: ComponentVisualDims, color?: [number, number, number]): THREE.Group {
  CAT_COLOR = color ?? DEFAULT_COLOR
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

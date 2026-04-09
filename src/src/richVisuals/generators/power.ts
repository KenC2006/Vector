/**
 * Rich visual generators for power components.
 * Batteries, converters, distribution, solar, capacitors, switches.
 */
import * as THREE from 'three'
import type { GeneratorDims } from './index'
import { getMaterial, getTintedMaterial } from '../materials'
import {
  chamferedBox, chamferedCylinder, pcbBoard, connectorBlock,
  labelRecess, screwHead,
} from '../primitives'

const CAT_COLOR: [number, number, number] = [0.95, 0.77, 0.06]  // yellow

function catMetal(strength = 0.3) {
  return getTintedMaterial('anodized_aluminum', ...CAT_COLOR, strength)
}

// ── LiPo Battery ────────────────────────────────────────────────────────────

function generateLipo(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.04

  // Main body (glossy plastic, blue-ish tint)
  const body = new THREE.Mesh(
    chamferedBox(w * 0.9, d * 0.85, h * 0.85, chamfer),
    getMaterial('glossy_plastic', 0x2255aa),
  )
  g.add(body)

  // Label strip (slightly raised darker box on top)
  const labelW = w * 0.75
  const labelH = h * 0.03
  const labelD = d * 0.55
  const label = new THREE.Mesh(
    chamferedBox(labelW, labelD, labelH, chamfer * 0.2),
    getMaterial('matte_plastic', 0x111133),
  )
  label.position.y = h * 0.43
  g.add(label)

  // XT60 connector (small yellow box on one end)
  const xt60W = w * 0.12
  const xt60H = h * 0.15
  const xt60D = d * 0.12
  const xt60 = new THREE.Mesh(
    chamferedBox(xt60W, xt60D, xt60H, chamfer * 0.15),
    getMaterial('glossy_plastic', 0xddaa00),
  )
  xt60.position.set(w * 0.45, 0, d * 0.35)
  g.add(xt60)

  // XT60 pin holes
  for (const sy of [-1, 1]) {
    const pin = new THREE.Mesh(
      chamferedCylinder(xt60W * 0.15, xt60D * 0.4, xt60W * 0.02, 8),
      getMaterial('dark_chrome'),
    )
    pin.rotation.x = Math.PI / 2
    pin.position.set(w * 0.45, sy * xt60H * 0.15, d * 0.42)
    g.add(pin)
  }

  // Balance connector (thin white box on same end)
  const balW = w * 0.08
  const balH = h * 0.08
  const balD = d * 0.2
  const bal = new THREE.Mesh(
    chamferedBox(balW, balD, balH, chamfer * 0.1),
    getMaterial('glossy_plastic', 0xeeeeee),
  )
  bal.position.set(w * 0.45, h * 0.2, -d * 0.15)
  g.add(bal)

  // Warning stripe (thin raised strip, yellow/black feel)
  const stripeW = w * 0.6
  const stripeH = h * 0.015
  const stripeD = d * 0.08
  const stripe = new THREE.Mesh(
    chamferedBox(stripeW, stripeD, stripeH, chamfer * 0.05),
    getMaterial('glossy_plastic', 0xeecc00),
  )
  stripe.position.set(0, -h * 0.35, d * 0.38)
  g.add(stripe)

  // Cell count indicator text area
  const cellLabel = labelRecess(w * 0.2, h * 0.12, chamfer * 0.2)
  cellLabel.position.set(-w * 0.25, h * 0.15, d * 0.38)
  g.add(cellLabel)

  return g
}

// ── Cell Holder / Battery Pack ──────────────────────────────────────────────

function generateCellHolder(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.04

  // Main housing
  const body = new THREE.Mesh(
    chamferedBox(w * 0.9, d * 0.85, h * 0.8, chamfer),
    getMaterial('matte_plastic', 0x222222),
  )
  g.add(body)

  // Cell divider lines (thin raised strips)
  const is4s2p = id.includes('4s2p')
  const cols = is4s2p ? 4 : 3
  const dividerH = h * 0.78
  const dividerD = d * 0.82
  for (let i = 1; i < cols; i++) {
    const t = (i / cols) - 0.5
    const divider = new THREE.Mesh(
      chamferedBox(w * 0.008, dividerD, dividerH, chamfer * 0.05),
      getMaterial('matte_plastic', 0x333333),
    )
    divider.position.set(t * w * 0.85, h * 0.02, 0)
    g.add(divider)
  }

  // Horizontal divider for 2p configs
  if (is4s2p) {
    const hDiv = new THREE.Mesh(
      chamferedBox(w * 0.87, w * 0.008, dividerH, chamfer * 0.05),
      getMaterial('matte_plastic', 0x333333),
    )
    hDiv.position.y = h * 0.02
    g.add(hDiv)
  }

  // Terminal contacts on each end
  for (const sx of [-1, 1]) {
    const terminal = new THREE.Mesh(
      chamferedBox(w * 0.06, d * 0.3, h * 0.08, chamfer * 0.1),
      getMaterial('copper_trace'),
    )
    terminal.position.set(sx * w * 0.42, h * 0.35, 0)
    g.add(terminal)
  }

  // Wire exit
  const wire = new THREE.Mesh(
    new THREE.CylinderGeometry(w * 0.015, w * 0.015, d * 0.15, 8),
    getMaterial('matte_plastic', 0xcc0000),
  )
  wire.rotation.x = Math.PI / 2
  wire.position.set(w * 0.35, h * 0.35, d * 0.45)
  g.add(wire)

  // Label
  const label = labelRecess(w * 0.4, h * 0.15, chamfer * 0.2)
  label.position.set(0, 0, d * 0.39)
  g.add(label)

  return g
}

// ── Supercapacitor ──────────────────────────────────────────────────────────

function generateSupercapacitor(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.04

  // Tall cylindrical body
  const bodyR = Math.min(w, d) * 0.35
  const bodyH = h * 0.8
  const body = new THREE.Mesh(
    chamferedCylinder(bodyR, bodyH, chamfer, 32),
    getMaterial('glossy_plastic', 0x333355),
  )
  g.add(body)

  // Sleeve label (slightly wider ring)
  const sleeveR = bodyR * 1.02
  const sleeveH = bodyH * 0.6
  const sleeve = new THREE.Mesh(
    chamferedCylinder(sleeveR, sleeveH, chamfer * 0.3, 32),
    getMaterial('glossy_plastic', 0x222244),
  )
  sleeve.position.y = -bodyH * 0.05
  g.add(sleeve)

  // Terminal posts on top (2 small cylinders, copper)
  const postR = bodyR * 0.1
  const postH = h * 0.1
  for (const sx of [-1, 1]) {
    const post = new THREE.Mesh(
      chamferedCylinder(postR, postH, postR * 0.15, 10),
      getMaterial('copper_trace'),
    )
    post.position.set(sx * bodyR * 0.4, bodyH * 0.5 + postH * 0.5, 0)
    g.add(post)
  }

  // Vent groove on top
  const vent = new THREE.Mesh(
    new THREE.TorusGeometry(bodyR * 0.5, bodyR * 0.02, 4, 24),
    getMaterial('dark_chrome'),
  )
  vent.rotation.x = Math.PI / 2
  vent.position.y = bodyH * 0.49
  g.add(vent)

  // Polarity marking stripe
  const stripe = new THREE.Mesh(
    chamferedBox(bodyR * 0.08, bodyH * 0.7, bodyR * 0.02, chamfer * 0.05),
    getMaterial('glossy_plastic', 0xcccccc),
  )
  stripe.position.set(-bodyR * 0.85, 0, 0)
  g.add(stripe)

  return g
}

// ── Solar Panel ─────────────────────────────────────────────────────────────

function generateSolarPanel(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.03

  // Thin flat body (dark blue/purple)
  const panelH = h * 0.12
  const panel = new THREE.Mesh(
    chamferedBox(w * 0.92, d * 0.92, panelH, chamfer),
    getMaterial('glossy_plastic', 0x1a1a55),
  )
  g.add(panel)

  // Cell grid lines (horizontal)
  const gridMat = getMaterial('matte_plastic', 0x111133)
  const hLines = 5
  for (let i = 1; i < hLines; i++) {
    const t = (i / hLines) - 0.5
    const line = new THREE.Mesh(
      chamferedBox(w * 0.88, d * 0.005, panelH * 0.2, chamfer * 0.05),
      gridMat,
    )
    line.position.set(0, panelH * 0.45, t * d * 0.85)
    g.add(line)
  }

  // Cell grid lines (vertical)
  const vLines = 4
  for (let i = 1; i < vLines; i++) {
    const t = (i / vLines) - 0.5
    const line = new THREE.Mesh(
      chamferedBox(w * 0.005, d * 0.88, panelH * 0.2, chamfer * 0.05),
      gridMat,
    )
    line.position.set(t * w * 0.85, panelH * 0.45, 0)
    g.add(line)
  }

  // Aluminum frame edges (4 strips)
  const frameThick = Math.min(w, d) * 0.025
  const frameMat = catMetal(0.2)
  // Left & right
  for (const sx of [-1, 1]) {
    const frame = new THREE.Mesh(
      chamferedBox(frameThick, d * 0.95, panelH * 1.1, chamfer * 0.1),
      frameMat,
    )
    frame.position.x = sx * w * 0.47
    g.add(frame)
  }
  // Front & back
  for (const sz of [-1, 1]) {
    const frame = new THREE.Mesh(
      chamferedBox(w * 0.95, frameThick, panelH * 1.1, chamfer * 0.1),
      frameMat,
    )
    frame.position.z = sz * d * 0.47
    g.add(frame)
  }

  // Junction box on back
  const jboxW = w * 0.15
  const jboxH = h * 0.08
  const jboxD = d * 0.1
  const jbox = new THREE.Mesh(
    chamferedBox(jboxW, jboxD, jboxH, chamfer * 0.15),
    getMaterial('matte_plastic'),
  )
  jbox.position.set(0, -panelH * 0.5 - jboxH * 0.4, 0)
  g.add(jbox)

  return g
}

// ── Buck Converter ──────────────────────────────────────────────────────────

function generateBuckConverter(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.03

  // PCB board
  const pcbH = h * 0.08
  const pcb = pcbBoard(w * 0.85, d * 0.8, pcbH)
  g.add(pcb)

  // Main IC chip (dark rectangle)
  const icW = w * 0.18
  const icH = h * 0.06
  const icD = d * 0.18
  const ic = new THREE.Mesh(
    chamferedBox(icW, icD, icH, chamfer * 0.1),
    getMaterial('matte_plastic', 0x111111),
  )
  ic.position.set(-w * 0.1, pcbH * 0.5 + icH * 0.5, -d * 0.1)
  g.add(ic)

  // IC marking dot
  const dot = new THREE.Mesh(
    new THREE.SphereGeometry(icW * 0.08, 8, 8),
    getMaterial('glossy_plastic', 0xeeeeee),
  )
  dot.position.set(-w * 0.1 - icW * 0.3, pcbH * 0.5 + icH, -d * 0.1 - icD * 0.3)
  g.add(dot)

  // Inductor (small chamferedCylinder)
  const indR = Math.min(w, d) * 0.1
  const indH = h * 0.1
  const inductor = new THREE.Mesh(
    chamferedCylinder(indR, indH, indR * 0.15, 16),
    getMaterial('matte_plastic', 0x333333),
  )
  inductor.position.set(w * 0.15, pcbH * 0.5 + indH * 0.5, d * 0.1)
  g.add(inductor)

  // Capacitors (tiny cylinders, 2 of them)
  const capR = Math.min(w, d) * 0.05
  const capH = h * 0.12
  for (let i = 0; i < 2; i++) {
    const cap = new THREE.Mesh(
      chamferedCylinder(capR, capH, capR * 0.12, 10),
      getMaterial('matte_plastic', 0x222222),
    )
    cap.position.set(w * 0.25 - i * w * 0.15, pcbH * 0.5 + capH * 0.5, -d * 0.25)
    g.add(cap)

    // Cap top marking
    const mark = new THREE.Mesh(
      chamferedCylinder(capR * 0.8, capH * 0.05, capR * 0.05, 8),
      getMaterial('glossy_plastic', 0x888888),
    )
    mark.position.set(w * 0.25 - i * w * 0.15, pcbH * 0.5 + capH + capH * 0.02, -d * 0.25)
    g.add(mark)
  }

  // Input/output pads
  for (const sx of [-1, 1]) {
    const pad = new THREE.Mesh(
      chamferedBox(w * 0.06, d * 0.08, pcbH * 0.3, chamfer * 0.02),
      getMaterial('copper_trace'),
    )
    pad.position.set(sx * w * 0.38, 0, d * 0.3)
    g.add(pad)
  }

  return g
}

// ── Power Distribution Unit ─────────────────────────────────────────────────

function generatePDU(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.05

  // Main housing
  const body = new THREE.Mesh(
    chamferedBox(w * 0.9, d * 0.85, h * 0.7, chamfer),
    catMetal(0.3),
  )
  g.add(body)

  // Terminal blocks (row of small connectorBlocks on top)
  const termCount = 6
  const termW = (w * 0.8) / termCount
  const termH = h * 0.12
  const termD = d * 0.2
  for (let i = 0; i < termCount; i++) {
    const tx = (i - (termCount - 1) / 2) * termW
    const term = connectorBlock(termW * 0.8, termD, termH, 0x111111)
    term.position.set(tx, h * 0.35 + termH * 0.3, d * 0.2)
    g.add(term)
  }

  // Fuse indicators (small colored cylinders)
  const fuseCount = 4
  const fuseR = Math.min(w, d) * 0.025
  const fuseH = h * 0.06
  for (let i = 0; i < fuseCount; i++) {
    const fx = (i - (fuseCount - 1) / 2) * w * 0.18
    const fuse = new THREE.Mesh(
      chamferedCylinder(fuseR, fuseH, fuseR * 0.15, 8),
      getMaterial('glossy_plastic', 0x33cc33),
    )
    fuse.position.set(fx, h * 0.35 + fuseH * 0.3, -d * 0.2)
    g.add(fuse)
  }

  // Main input connector
  const input = connectorBlock(w * 0.15, d * 0.15, h * 0.12, 0xddaa00)
  input.position.set(-w * 0.35, 0, d * 0.38)
  g.add(input)

  // Label
  const label = labelRecess(w * 0.5, h * 0.25, chamfer * 0.3)
  label.position.set(0, 0, d * 0.38)
  g.add(label)

  // Mounting ears
  for (const sx of [-1, 1]) {
    const ear = new THREE.Mesh(
      chamferedBox(w * 0.08, d * 0.15, h * 0.6, chamfer * 0.2),
      catMetal(0.2),
    )
    ear.position.set(sx * w * 0.48, 0, 0)
    g.add(ear)
  }

  return g
}

// ── USB-C PD Trigger ────────────────────────────────────────────────────────

function generateUSBCPD(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.03

  // PCB board
  const pcbH = h * 0.08
  const pcb = pcbBoard(w * 0.85, d * 0.75, pcbH)
  g.add(pcb)

  // USB-C connector (connectorBlock on one end)
  const usbW = w * 0.18
  const usbH = h * 0.08
  const usbD = d * 0.12
  const usb = connectorBlock(usbW, usbD, usbH, 0x444444)
  usb.position.set(w * 0.35, pcbH * 0.3, 0)
  g.add(usb)

  // Status LED (tiny green sphere)
  const ledR = Math.min(w, d) * 0.025
  const led = new THREE.Mesh(
    new THREE.SphereGeometry(ledR, 10, 10),
    getMaterial('glossy_plastic', 0x33ff33),
  )
  led.position.set(-w * 0.15, pcbH * 0.5 + ledR, d * 0.15)
  g.add(led)

  // Small IC chip
  const icW = w * 0.12
  const icH = h * 0.04
  const icD = d * 0.12
  const ic = new THREE.Mesh(
    chamferedBox(icW, icD, icH, chamfer * 0.05),
    getMaterial('matte_plastic', 0x111111),
  )
  ic.position.set(0, pcbH * 0.5 + icH * 0.5, -d * 0.1)
  g.add(ic)

  // Output pads
  for (const sx of [-1, 1]) {
    const pad = new THREE.Mesh(
      chamferedBox(w * 0.05, d * 0.06, pcbH * 0.3, chamfer * 0.02),
      getMaterial('copper_trace'),
    )
    pad.position.set(-w * 0.3, 0, sx * d * 0.2)
    g.add(pad)
  }

  // Tiny capacitor
  const capR = Math.min(w, d) * 0.03
  const capH = h * 0.06
  const cap = new THREE.Mesh(
    chamferedCylinder(capR, capH, capR * 0.1, 8),
    getMaterial('matte_plastic', 0x222222),
  )
  cap.position.set(w * 0.1, pcbH * 0.5 + capH * 0.5, d * 0.2)
  g.add(cap)

  return g
}

// ── E-Stop Switch ───────────────────────────────────────────────────────────

function generateEStop(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.05

  // Base box (yellow tint)
  const baseW = w * 0.7
  const baseH = h * 0.35
  const baseD = d * 0.7
  const base = new THREE.Mesh(
    chamferedBox(baseW, baseD, baseH, chamfer),
    getMaterial('glossy_plastic', 0xddaa00),
  )
  base.position.y = -h * 0.15
  g.add(base)

  // Yellow guard ring
  const guardR = Math.min(w, d) * 0.38
  const guardH = h * 0.1
  const guard = new THREE.Mesh(
    chamferedCylinder(guardR, guardH, chamfer * 0.3, 32),
    getMaterial('glossy_plastic', 0xddaa00),
  )
  guard.position.y = h * 0.08
  g.add(guard)

  // Guard ring inner cutout (dark inset)
  const guardInner = new THREE.Mesh(
    chamferedCylinder(guardR * 0.8, guardH * 0.6, chamfer * 0.1, 28),
    getMaterial('dark_chrome'),
  )
  guardInner.position.y = h * 0.1
  g.add(guardInner)

  // Red mushroom button (large chamferedCylinder)
  const buttonR = guardR * 0.7
  const buttonH = h * 0.25
  const button = new THREE.Mesh(
    chamferedCylinder(buttonR, buttonH, buttonR * 0.15, 32),
    getMaterial('glossy_plastic', 0xdd2222),
  )
  button.position.y = h * 0.22
  g.add(button)

  // Button dome top (slightly wider at top)
  const domeR = buttonR * 1.1
  const domeH = h * 0.06
  const dome = new THREE.Mesh(
    chamferedCylinder(domeR, domeH, domeR * 0.2, 32),
    getMaterial('glossy_plastic', 0xcc1111),
  )
  dome.position.y = h * 0.22 + buttonH * 0.5 + domeH * 0.3
  g.add(dome)

  // Contact block on bottom
  const contactW = baseW * 0.5
  const contactH = h * 0.15
  const contactD = baseD * 0.4
  const contact = new THREE.Mesh(
    chamferedBox(contactW, contactD, contactH, chamfer * 0.3),
    getMaterial('matte_plastic'),
  )
  contact.position.y = -h * 0.15 - baseH * 0.5 - contactH * 0.4
  g.add(contact)

  // Terminal screws
  for (const sx of [-1, 1]) {
    const screw = screwHead(Math.min(w, d) * 0.03, h * 0.03)
    screw.position.set(sx * contactW * 0.3, -h * 0.15 - baseH * 0.5 - contactH * 0.7, 0)
    g.add(screw)
  }

  // Warning label on base
  const label = labelRecess(baseW * 0.5, baseH * 0.35, chamfer * 0.2)
  label.position.set(0, -h * 0.15, baseD * 0.42)
  g.add(label)

  return g
}

// ── Dispatcher ──────────────────────────────────────────────────────────────

export function generateRichPower(id: string, dims: GeneratorDims): THREE.Group {
  if (id.includes('lipo') || id.includes('lipo_')) return generateLipo(id, dims)
  if (id.includes('18650') || id.includes('battery_pack') || id.includes('cell_holder')) return generateCellHolder(id, dims)
  if (id.includes('supercap')) return generateSupercapacitor(id, dims)
  if (id.includes('solar')) return generateSolarPanel(id, dims)
  if (id.includes('buck') || id.includes('converter')) return generateBuckConverter(id, dims)
  if (id.includes('distribution')) return generatePDU(id, dims)
  if (id.includes('usb_c') || id.includes('pd_trigger')) return generateUSBCPD(id, dims)
  if (id.includes('estop') || id.includes('e_stop')) return generateEStop(id, dims)
  // Default: lipo
  return generateLipo(id, dims)
}

/**
 * Rich visual generators for compute / electronics components.
 * Uses pcbBoard(), connectorBlock(), heatsinkFins(), chamferedBox chip packages,
 * and chamferedCylinder antenna stubs for Fusion-quality board visuals.
 */
import * as THREE from 'three'
import type { GeneratorDims } from './index'
import { getMaterial } from '../materials'
import {
  chamferedBox, chamferedCylinder, mountingHole,
  labelRecess, pcbBoard, connectorBlock, heatsinkFins,
} from '../primitives'
import { nurbsFilletBox, nurbsCylinder, nurbsMotorHousing, nurbsTorus } from '../nurbs'

// Category color: [0.18, 0.80, 0.44] (green) — applied via pcbBoard's green material

// ── MCU Small ───────────────────────────────────────────────────────────────

function generateMCU(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  // PCB base
  const pcb = pcbBoard(w, h, d * 0.25)
  g.add(pcb)

  // Main chip package (dark QFP)
  const chipW = w * 0.3
  const chipH = h * 0.3
  const chipD = d * 0.1
  const chip = new THREE.Mesh(
    nurbsFilletBox(chipW, chipH, chipD, chipW * 0.04, 12),
    getMaterial('matte_plastic'),
  )
  chip.position.set(0, 0, d * 0.2)
  g.add(chip)

  // Chip orientation dot
  const dot = new THREE.Mesh(
    new THREE.SphereGeometry(chipW * 0.08, 8, 8),
    getMaterial('glossy_plastic', 0xeeeeee),
  )
  dot.position.set(-chipW * 0.35, -chipH * 0.3, d * 0.26)
  g.add(dot)

  // Crystal oscillator (tiny chamferedCylinder)
  const crystalR = w * 0.035
  const crystal = new THREE.Mesh(
    nurbsCylinder(crystalR, d * 0.06, crystalR * 0.1, 12),
    getMaterial('brushed_steel'),
  )
  crystal.position.set(w * 0.25, h * 0.15, d * 0.18)
  g.add(crystal)

  // GPIO pin header (thin tall chamferedBox)
  const headerW = w * 0.7
  const headerH = h * 0.06
  const headerD = d * 0.4
  const header = new THREE.Mesh(
    nurbsFilletBox(headerW, headerH, headerD, headerH * 0.1, 12),
    getMaterial('matte_plastic'),
  )
  header.position.set(0, -h * 0.35, d * 0.12)
  g.add(header)

  // Individual pin bumps on header
  const pinCount = 10
  const pinSpacing = headerW / (pinCount + 1)
  for (let i = 1; i <= pinCount; i++) {
    const pin = new THREE.Mesh(
      nurbsCylinder(headerH * 0.3, headerD * 0.7, headerH * 0.03),
      getMaterial('copper_trace'),
    )
    pin.position.set(-headerW / 2 + i * pinSpacing, -h * 0.35, d * 0.12)
    g.add(pin)
  }

  // USB micro connector
  const usb = connectorBlock(w * 0.1, d * 0.08, h * 0.1, 0x333333)
  usb.rotation.x = Math.PI / 2
  usb.position.set(-w * 0.35, h * 0.0, -d * 0.06)
  g.add(usb)

  // Bypass capacitors (small chamferedBox)
  for (const pos of [[w * 0.15, h * 0.2], [-w * 0.15, h * 0.2]]) {
    const cap = new THREE.Mesh(
      nurbsFilletBox(w * 0.05, h * 0.03, d * 0.04, w * 0.004, 12),
      getMaterial('matte_plastic', 0x443322),
    )
    cap.position.set(pos[0], pos[1], d * 0.17)
    g.add(cap)
  }

  // Mounting holes (4 corners)
  const holeR = w * 0.02
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      const hole = mountingHole(holeR, d * 0.3)
      hole.rotation.x = Math.PI / 2
      hole.position.set(sx * w * 0.43, sy * h * 0.43, 0)
      g.add(hole)
    }
  }

  return g
}

// ── SBC Small ───────────────────────────────────────────────────────────────

function generateSBCSmall(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  // PCB base
  const pcb = pcbBoard(w, h, d * 0.2)
  g.add(pcb)

  // SoC chip package (chamferedBox)
  const chipW = w * 0.2
  const chip = new THREE.Mesh(
    nurbsFilletBox(chipW, chipW, d * 0.08, chipW * 0.03, 12),
    getMaterial('matte_plastic'),
  )
  chip.position.set(-w * 0.1, h * 0.05, d * 0.15)
  g.add(chip)

  // RAM chip (chamferedBox)
  const ram = new THREE.Mesh(
    nurbsFilletBox(w * 0.15, w * 0.08, d * 0.05, w * 0.008, 12),
    getMaterial('matte_plastic'),
  )
  ram.position.set(w * 0.15, h * 0.1, d * 0.14)
  g.add(ram)

  // USB connectorBlock ports (2 stacked)
  for (let i = 0; i < 2; i++) {
    const usb = connectorBlock(w * 0.1, d * 0.12, h * 0.15, 0x333333)
    usb.rotation.x = Math.PI / 2
    usb.position.set(w * 0.35, h * 0.15 - i * h * 0.25, 0)
    g.add(usb)
  }

  // Ethernet connectorBlock port (taller)
  const eth = connectorBlock(w * 0.12, d * 0.14, h * 0.15, 0x222222)
  eth.rotation.x = Math.PI / 2
  eth.position.set(w * 0.15, -h * 0.3, 0)
  g.add(eth)

  // GPIO header (thin tall chamferedBox)
  const headerW = w * 0.65
  const headerH = h * 0.05
  const headerD = d * 0.35
  const header = new THREE.Mesh(
    nurbsFilletBox(headerW, headerH, headerD, headerH * 0.1, 12),
    getMaterial('matte_plastic'),
  )
  header.position.set(-w * 0.05, h * 0.38, d * 0.12)
  g.add(header)

  // Pin rows on header
  for (const row of [-1, 1]) {
    const pinCount = 8
    const pinSpacing = headerW / (pinCount + 1)
    for (let i = 1; i <= pinCount; i++) {
      const pin = new THREE.Mesh(
        nurbsCylinder(headerH * 0.25, headerD * 0.6, headerH * 0.02),
        getMaterial('copper_trace'),
      )
      pin.position.set(
        -w * 0.05 - headerW / 2 + i * pinSpacing,
        h * 0.38 + row * headerH * 0.6,
        d * 0.12,
      )
      g.add(pin)
    }
  }

  // SD card slot
  const sd = new THREE.Mesh(
    nurbsFilletBox(w * 0.12, h * 0.02, d * 0.08, w * 0.005, 12),
    getMaterial('brushed_steel'),
  )
  sd.position.set(-w * 0.35, -h * 0.1, d * 0.05)
  g.add(sd)

  // Status LEDs
  for (let i = 0; i < 2; i++) {
    const led = new THREE.Mesh(
      new THREE.SphereGeometry(w * 0.012, 6, 6),
      getMaterial('glossy_plastic', i === 0 ? 0x00ff44 : 0xff4400),
    )
    led.position.set(-w * 0.3, -h * 0.3 + i * h * 0.08, d * 0.13)
    g.add(led)
  }

  // Mounting holes (4 corners)
  const holeR = w * 0.02
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      const hole = mountingHole(holeR, d * 0.25)
      hole.rotation.x = Math.PI / 2
      hole.position.set(sx * w * 0.43, sy * h * 0.43, 0)
      g.add(hole)
    }
  }

  return g
}

// ── SBC GPU ─────────────────────────────────────────────────────────────────

function generateSBCGPU(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  // PCB base
  const pcb = pcbBoard(w, h, d * 0.18)
  g.add(pcb)

  // heatsinkFins on top (covers chip area)
  const hsW = w * 0.45
  const hsH = h * 0.45
  const hsD = d * 0.5
  const hs = heatsinkFins(hsW, hsH, hsD, 8, hsW * 0.015)
  hs.position.set(-w * 0.05, 0, d * 0.1 + hsD / 2)
  hs.rotation.x = Math.PI / 2
  g.add(hs)

  // SoC chip under heatsink
  const chip = new THREE.Mesh(
    nurbsFilletBox(w * 0.2, h * 0.2, d * 0.06, w * 0.008, 12),
    getMaterial('matte_plastic'),
  )
  chip.position.set(-w * 0.05, 0, d * 0.12)
  g.add(chip)

  // USB connectorBlock ports (2 on right edge)
  for (let i = 0; i < 2; i++) {
    const usb = connectorBlock(w * 0.1, d * 0.1, h * 0.12, 0x333333)
    usb.rotation.x = Math.PI / 2
    usb.position.set(w * 0.38, h * 0.15 - i * h * 0.25, 0)
    g.add(usb)
  }

  // Ethernet connectorBlock port
  const eth = connectorBlock(w * 0.12, d * 0.12, h * 0.14, 0x222222)
  eth.rotation.x = Math.PI / 2
  eth.position.set(w * 0.2, -h * 0.32, 0)
  g.add(eth)

  // Power barrel jack
  const barrel = new THREE.Mesh(
    nurbsCylinder(w * 0.04, d * 0.1, w * 0.005, 12),
    getMaterial('matte_plastic'),
  )
  barrel.rotation.x = Math.PI / 2
  barrel.position.set(-w * 0.38, -h * 0.3, d * 0.06)
  g.add(barrel)

  // GPIO header (thin tall chamferedBox)
  const headerW = w * 0.55
  const headerH = h * 0.04
  const headerD = d * 0.3
  const header = new THREE.Mesh(
    nurbsFilletBox(headerW, headerH, headerD, headerH * 0.1, 12),
    getMaterial('matte_plastic'),
  )
  header.position.set(0, h * 0.38, d * 0.1)
  g.add(header)

  // Capacitors
  for (const pos of [[w * 0.2, h * 0.15], [-w * 0.3, h * 0.2]]) {
    const cap = new THREE.Mesh(
      nurbsCylinder(w * 0.025, d * 0.15, w * 0.003, 12),
      getMaterial('matte_plastic', 0x222244),
    )
    cap.position.set(pos[0], pos[1], d * 0.18)
    g.add(cap)
  }

  // Mounting holes
  const holeR = w * 0.018
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      const hole = mountingHole(holeR, d * 0.25)
      hole.rotation.x = Math.PI / 2
      hole.position.set(sx * w * 0.44, sy * h * 0.44, 0)
      g.add(hole)
    }
  }

  return g
}

// ── Motor Driver Dual ───────────────────────────────────────────────────────

function generateMotorDriver(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  // PCB base
  const pcb = pcbBoard(w, h, d * 0.2)
  g.add(pcb)

  // 2 heatsinkFins blocks (H-bridge drivers)
  for (const sx of [-1, 1]) {
    const hsW = w * 0.2
    const hsH = h * 0.3
    const hsD = d * 0.4
    const hs = heatsinkFins(hsW, hsH, hsD, 5, hsW * 0.02)
    hs.position.set(sx * w * 0.18, -h * 0.05, d * 0.1 + hsD / 2)
    hs.rotation.x = Math.PI / 2
    g.add(hs)
  }

  // Terminal blocks (motor outputs) — connectorBlock
  for (const sx of [-1, 1]) {
    const term = connectorBlock(w * 0.12, d * 0.12, h * 0.12, 0x00aa44)
    term.rotation.x = Math.PI / 2
    term.position.set(sx * w * 0.35, -h * 0.35, 0)
    g.add(term)
  }

  // Power input terminal — connectorBlock
  const powerTerm = connectorBlock(w * 0.15, d * 0.1, h * 0.1, 0xaa0000)
  powerTerm.rotation.x = Math.PI / 2
  powerTerm.position.set(0, h * 0.35, 0)
  g.add(powerTerm)

  // Signal header
  const header = new THREE.Mesh(
    nurbsFilletBox(w * 0.4, h * 0.04, d * 0.2, w * 0.004, 12),
    getMaterial('matte_plastic'),
  )
  header.position.set(0, h * 0.15, d * 0.12)
  g.add(header)

  // Capacitors (bulk decoupling)
  for (const sx of [-1, 1]) {
    const cap = new THREE.Mesh(
      nurbsCylinder(w * 0.04, d * 0.25, w * 0.005, 12),
      getMaterial('matte_plastic', 0x111133),
    )
    cap.position.set(sx * w * 0.05, h * 0.2, d * 0.2)
    g.add(cap)
  }

  // Status LED
  const led = new THREE.Mesh(
    new THREE.SphereGeometry(w * 0.012, 6, 6),
    getMaterial('glossy_plastic', 0x00ff44),
  )
  led.position.set(w * 0.3, h * 0.2, d * 0.12)
  g.add(led)

  // Mounting holes
  const holeR = w * 0.02
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      const hole = mountingHole(holeR, d * 0.25)
      hole.rotation.x = Math.PI / 2
      hole.position.set(sx * w * 0.43, sy * h * 0.43, 0)
      g.add(hole)
    }
  }

  return g
}

// ── FOC Controller ──────────────────────────────────────────────────────────

function generateFOCController(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  // PCB base
  const pcb = pcbBoard(w, h, d * 0.2)
  g.add(pcb)

  // Main heatsinkFins
  const hsW = w * 0.35
  const hsH = h * 0.4
  const hsD = d * 0.45
  const hs = heatsinkFins(hsW, hsH, hsD, 6, hsW * 0.018)
  hs.position.set(-w * 0.1, 0, d * 0.1 + hsD / 2)
  hs.rotation.x = Math.PI / 2
  g.add(hs)

  // FOC driver IC (chamferedBox under heatsink)
  const chip = new THREE.Mesh(
    nurbsFilletBox(w * 0.18, h * 0.18, d * 0.05, w * 0.006, 12),
    getMaterial('matte_plastic'),
  )
  chip.position.set(-w * 0.1, 0, d * 0.12)
  g.add(chip)

  // CAN bus connectorBlock
  const can = connectorBlock(w * 0.08, d * 0.08, h * 0.1, 0x00aa00)
  can.rotation.x = Math.PI / 2
  can.position.set(w * 0.35, h * 0.1, 0)
  g.add(can)

  // Motor phase output terminals (3-phase)
  for (let i = -1; i <= 1; i++) {
    const term = connectorBlock(w * 0.08, d * 0.08, h * 0.08, 0x0044aa)
    term.rotation.x = Math.PI / 2
    term.position.set(i * w * 0.15, -h * 0.35, 0)
    g.add(term)
  }

  // Power input terminal
  const powerIn = connectorBlock(w * 0.1, d * 0.08, h * 0.08, 0xaa0000)
  powerIn.rotation.x = Math.PI / 2
  powerIn.position.set(w * 0.35, -h * 0.15, 0)
  g.add(powerIn)

  // Bulk capacitors
  for (const sx of [-1, 1]) {
    const cap = new THREE.Mesh(
      nurbsCylinder(w * 0.03, d * 0.2, w * 0.004, 12),
      getMaterial('matte_plastic', 0x111133),
    )
    cap.position.set(w * 0.2 + sx * w * 0.08, h * 0.15, d * 0.18)
    g.add(cap)
  }

  // Status LEDs
  for (let i = 0; i < 3; i++) {
    const led = new THREE.Mesh(
      new THREE.SphereGeometry(w * 0.01, 6, 6),
      getMaterial('glossy_plastic', [0x00ff44, 0xffaa00, 0xff0000][i]),
    )
    led.position.set(w * 0.25 + i * w * 0.05, h * 0.3, d * 0.12)
    g.add(led)
  }

  // Mounting holes
  const holeR = w * 0.018
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      const hole = mountingHole(holeR, d * 0.25)
      hole.rotation.x = Math.PI / 2
      hole.position.set(sx * w * 0.43, sy * h * 0.43, 0)
      g.add(hole)
    }
  }

  return g
}

// ── FPGA Dev Board ──────────────────────────────────────────────────────────

function generateFPGA(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  // PCB base
  const pcb = pcbBoard(w, h, d * 0.18)
  g.add(pcb)

  // Large BGA chip (chamferedBox)
  const chipW = w * 0.3
  const chip = new THREE.Mesh(
    nurbsFilletBox(chipW, chipW, d * 0.06, chipW * 0.03, 12),
    getMaterial('matte_plastic'),
  )
  chip.position.set(0, 0, d * 0.13)
  g.add(chip)

  // Chip label marking
  const chipLabel = labelRecess(chipW * 0.6, chipW * 0.3, d * 0.01)
  chipLabel.position.set(0, 0, d * 0.17)
  g.add(chipLabel)

  // heatsinkFins on chip
  const hsW = chipW * 1.1
  const hsD = d * 0.3
  const hs = heatsinkFins(hsW, hsW, hsD, 6, hsW * 0.015)
  hs.position.set(0, 0, d * 0.1 + d * 0.06 + hsD / 2)
  hs.rotation.x = Math.PI / 2
  g.add(hs)

  // Configuration flash chip
  const flash = new THREE.Mesh(
    nurbsFilletBox(w * 0.1, h * 0.06, d * 0.04, w * 0.005, 12),
    getMaterial('matte_plastic'),
  )
  flash.position.set(w * 0.25, h * 0.15, d * 0.12)
  g.add(flash)

  // JTAG header
  const jtag = new THREE.Mesh(
    nurbsFilletBox(w * 0.12, h * 0.08, d * 0.2, w * 0.005, 12),
    getMaterial('matte_plastic'),
  )
  jtag.position.set(-w * 0.3, h * 0.25, d * 0.1)
  g.add(jtag)

  // IO headers (2 long rows)
  for (const sy of [-1, 1]) {
    const header = new THREE.Mesh(
      nurbsFilletBox(w * 0.6, h * 0.04, d * 0.25, w * 0.003, 12),
      getMaterial('matte_plastic'),
    )
    header.position.set(0, sy * h * 0.38, d * 0.1)
    g.add(header)
  }

  // Voltage regulators
  for (let i = 0; i < 2; i++) {
    const vreg = new THREE.Mesh(
      nurbsFilletBox(w * 0.06, h * 0.04, d * 0.08, w * 0.003, 12),
      getMaterial('matte_plastic'),
    )
    vreg.position.set(w * 0.3, -h * 0.05 + i * h * 0.12, d * 0.12)
    g.add(vreg)
  }

  // Status LEDs
  for (let i = 0; i < 4; i++) {
    const led = new THREE.Mesh(
      new THREE.SphereGeometry(w * 0.008, 6, 6),
      getMaterial('glossy_plastic', [0x00ff44, 0xffaa00, 0xff0000, 0x0088ff][i]),
    )
    led.position.set(-w * 0.35, -h * 0.2 + i * h * 0.08, d * 0.12)
    g.add(led)
  }

  // Mounting holes
  const holeR = w * 0.018
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      const hole = mountingHole(holeR, d * 0.25)
      hole.rotation.x = Math.PI / 2
      hole.position.set(sx * w * 0.44, sy * h * 0.44, 0)
      g.add(hole)
    }
  }

  return g
}

// ── Generic PCB Module ──────────────────────────────────────────────────────
// Covers CAN transceiver, USB hub, wireless, LoRa, GPS

function generatePCBModule(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const hasAntenna = id.includes('wireless') || id.includes('lora') || id.includes('gps')

  // PCB base
  const pcb = pcbBoard(w, h, d * 0.25)
  g.add(pcb)

  // Main IC (chamferedBox)
  const chipW = w * 0.2
  const chip = new THREE.Mesh(
    nurbsFilletBox(chipW, chipW * 0.8, d * 0.08, chipW * 0.03, 12),
    getMaterial('matte_plastic'),
  )
  chip.position.set(0, h * 0.05, d * 0.18)
  g.add(chip)

  // Passive component cluster (chamferedBox)
  const passiveCount = 4
  for (let i = 0; i < passiveCount; i++) {
    const passive = new THREE.Mesh(
      nurbsFilletBox(w * 0.06, h * 0.035, d * 0.04, w * 0.004, 12),
      getMaterial('matte_plastic', 0x443322),
    )
    passive.position.set(
      w * 0.15 - i * w * 0.1,
      h * 0.25,
      d * 0.17,
    )
    g.add(passive)
  }

  // Connector (CAN/USB/SMA depending on type) — connectorBlock
  if (id.includes('can')) {
    const conn = connectorBlock(w * 0.08, d * 0.08, h * 0.08, 0x00aa00)
    conn.rotation.x = Math.PI / 2
    conn.position.set(w * 0.35, 0, 0)
    g.add(conn)
  } else if (id.includes('usb_hub')) {
    for (let i = 0; i < 3; i++) {
      const usb = connectorBlock(w * 0.08, d * 0.07, h * 0.08, 0x333333)
      usb.rotation.x = Math.PI / 2
      usb.position.set(w * 0.35, h * 0.2 - i * h * 0.2, 0)
      g.add(usb)
    }
  } else {
    const conn = connectorBlock(w * 0.1, d * 0.08, h * 0.1, 0x333333)
    conn.rotation.x = Math.PI / 2
    conn.position.set(w * 0.35, -h * 0.1, 0)
    g.add(conn)
  }

  // Antenna stub (thin chamferedCylinder) for wireless/LoRa/GPS
  if (hasAntenna) {
    // SMA connector base
    const smaBase = new THREE.Mesh(
      nurbsCylinder(w * 0.03, d * 0.05, w * 0.004, 12),
      getMaterial('copper_trace'),
    )
    smaBase.position.set(-w * 0.3, h * 0.3, d * 0.16)
    g.add(smaBase)

    // Antenna rod (thin chamferedCylinder)
    const antennaR = w * 0.015
    const antennaH = d * 0.8
    const antenna = new THREE.Mesh(
      nurbsCylinder(antennaR, antennaH, antennaR * 0.1),
      getMaterial('matte_plastic'),
    )
    antenna.position.set(-w * 0.3, h * 0.3, d * 0.16 + d * 0.025 + antennaH / 2)
    g.add(antenna)

    // Antenna tip
    const tip = new THREE.Mesh(
      nurbsCylinder(antennaR * 1.5, antennaR * 3, antennaR * 0.2),
      getMaterial('matte_plastic'),
    )
    tip.position.set(-w * 0.3, h * 0.3, d * 0.16 + d * 0.025 + antennaH + antennaR * 1.5)
    g.add(tip)

    // GPS: patch antenna plate
    if (id.includes('gps')) {
      const patch = new THREE.Mesh(
        nurbsFilletBox(w * 0.25, h * 0.25, d * 0.04, w * 0.01, 12),
        getMaterial('glossy_plastic', 0xeeeeee),
      )
      patch.position.set(w * 0.1, -h * 0.1, d * 0.2)
      g.add(patch)
    }
  }

  // Pin header
  const header = new THREE.Mesh(
    nurbsFilletBox(w * 0.5, h * 0.04, d * 0.2, w * 0.003, 12),
    getMaterial('matte_plastic'),
  )
  header.position.set(0, -h * 0.38, d * 0.08)
  g.add(header)

  // Status LED
  const led = new THREE.Mesh(
    new THREE.SphereGeometry(w * 0.01, 6, 6),
    getMaterial('glossy_plastic', 0x00ff44),
  )
  led.position.set(w * 0.25, -h * 0.25, d * 0.14)
  g.add(led)

  // Mounting holes
  const holeR = w * 0.02
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      const hole = mountingHole(holeR, d * 0.3)
      hole.rotation.x = Math.PI / 2
      hole.position.set(sx * w * 0.42, sy * h * 0.42, 0)
      g.add(hole)
    }
  }

  return g
}

// ── Dispatcher ──────────────────────────────────────────────────────────────

export function generateRichCompute(id: string, dims: GeneratorDims): THREE.Group {
  if (id.includes('mcu')) return generateMCU(id, dims)
  if (id.includes('sbc_gpu') || id.includes('sbc_large')) return generateSBCGPU(id, dims)
  if (id.includes('sbc')) return generateSBCSmall(id, dims)
  if (id.includes('motor_driver')) return generateMotorDriver(id, dims)
  if (id.includes('foc')) return generateFOCController(id, dims)
  if (id.includes('fpga')) return generateFPGA(id, dims)
  // CAN transceiver, USB hub, wireless, LoRa, GPS all use PCB module
  return generatePCBModule(id, dims)
}

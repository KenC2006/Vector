/**
 * Rich visual generators for transmission components.
 * Belts, leadscrews, bearings, gearboxes, gears, couplings, chains.
 */
import * as THREE from 'three'
import type { GeneratorDims } from './index'
import { getMaterial, getTintedMaterial } from '../materials'
import {
  chamferedBox, chamferedCylinder, mountingHole, screwHead,
  boltCircle, knurledRing,
} from '../primitives'

const CAT_COLOR: [number, number, number] = [0.56, 0.27, 0.68]  // purple

function catMetal(base: string = 'anodized_aluminum', strength = 0.3) {
  return getTintedMaterial(base, ...CAT_COLOR, strength)
}

// ── Helpers ─────────────────────────────────────────────────────────────────

/** Create a gear profile via LatheGeometry with triangular teeth. */
function gearCylinder(
  outerR: number, innerR: number, height: number,
  toothCount: number, segments = 64,
): THREE.BufferGeometry {
  const hh = height / 2
  const points: THREE.Vector2[] = []

  // Bottom center to bottom edge
  points.push(new THREE.Vector2(0, -hh))
  points.push(new THREE.Vector2(innerR * 0.5, -hh))
  points.push(new THREE.Vector2(innerR, -hh))

  // Tooth profile along bottom fillet and outer wall
  const teethR = (outerR + innerR) / 2
  const toothDepth = (outerR - innerR) / 2

  // For LatheGeometry, the profile is radial (r,y), revolved around Y.
  // We create a slight bottom chamfer, then the main tooth radius wall, then top chamfer.
  const chamfer = height * 0.05
  points.push(new THREE.Vector2(innerR, -hh + chamfer))

  // Create teeth as radial bumps — LatheGeometry revolves the profile,
  // so we fake teeth by using a wavy radial profile at the outer edge.
  // Since LatheGeometry samples evenly, we create a wave with toothCount peaks.
  const wallSegments = toothCount * 4
  for (let i = 0; i <= wallSegments; i++) {
    const t = i / wallSegments
    const y = -hh + chamfer + t * (height - chamfer * 2)
    // Alternate between tooth peak and valley
    const phase = (i / 4) * Math.PI * 2
    const toothOffset = Math.cos(phase) * toothDepth * 0.3
    points.push(new THREE.Vector2(teethR + toothOffset, y))
  }

  points.push(new THREE.Vector2(innerR, hh - chamfer))
  points.push(new THREE.Vector2(innerR, hh))
  points.push(new THREE.Vector2(0, hh))

  return new THREE.LatheGeometry(points, segments)
}

// ── Timing Belt GT2 ─────────────────────────────────────────────────────────

function generateTimingBelt(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  const pulleyR = Math.min(w, d) * 0.2
  const pulleyH = h * 0.3
  const chamfer = pulleyR * 0.08
  const spacing = w * 0.35

  // Two pulleys
  for (const sx of [-1, 1]) {
    const pulley = new THREE.Mesh(
      chamferedCylinder(pulleyR, pulleyH, chamfer, 32),
      catMetal(),
    )
    pulley.position.set(sx * spacing, 0, 0)
    g.add(pulley)

    // Pulley flanges (thin wider discs top/bottom)
    for (const sy of [-1, 1]) {
      const flange = new THREE.Mesh(
        chamferedCylinder(pulleyR * 1.15, pulleyH * 0.08, chamfer * 0.5, 32),
        getMaterial('brushed_steel'),
      )
      flange.position.set(sx * spacing, sy * pulleyH * 0.45, 0)
      g.add(flange)
    }

    // Shaft bore
    const bore = new THREE.Mesh(
      new THREE.CylinderGeometry(pulleyR * 0.25, pulleyR * 0.25, pulleyH * 1.2, 16),
      getMaterial('dark_chrome'),
    )
    bore.position.set(sx * spacing, 0, 0)
    g.add(bore)
  }

  // Belt (thin flat box connecting pulleys)
  const beltW = spacing * 2 + pulleyR * 2
  const beltThick = pulleyH * 0.06
  const beltMat = getMaterial('rubber_black')

  // Top belt run
  const topBelt = new THREE.Mesh(
    new THREE.BoxGeometry(beltW, beltThick, pulleyH * 0.8),
    beltMat,
  )
  topBelt.position.y = pulleyR
  g.add(topBelt)

  // Bottom belt run
  const botBelt = new THREE.Mesh(
    new THREE.BoxGeometry(beltW, beltThick, pulleyH * 0.8),
    beltMat,
  )
  botBelt.position.y = -pulleyR
  g.add(botBelt)

  return g
}

// ── Leadscrew ───────────────────────────────────────────────────────────────

function generateLeadscrew(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const isBall = id.includes('ballscrew')
  const shaftR = isBall ? Math.min(w, h) * 0.12 : Math.min(w, h) * 0.08
  const chamfer = shaftR * 0.1

  // Long screw shaft
  const shaft = new THREE.Mesh(
    chamferedCylinder(shaftR, length * 0.9, chamfer, 24),
    getMaterial('brushed_steel'),
  )
  shaft.rotation.x = Math.PI / 2
  g.add(shaft)

  // Thread representation (knurled pattern along shaft)
  const threadRing = knurledRing(shaftR * 1.05, length * 0.85, Math.round(length / (shaftR * 1.5)))
  threadRing.rotation.x = Math.PI / 2
  g.add(threadRing)

  // Nut block
  const nutW = Math.min(w, h) * 0.5
  const nutH = Math.min(w, h) * 0.5
  const nutD = length * 0.12
  const nutMat = isBall ? catMetal('anodized_aluminum', 0.4) : catMetal()

  const nut = new THREE.Mesh(
    chamferedBox(nutW, nutH, nutD, nutW * 0.06),
    nutMat,
  )
  nut.position.z = length * 0.1
  g.add(nut)

  // Nut mounting holes
  const holeR = nutW * 0.06
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      const hole = mountingHole(holeR, nutD * 1.1)
      hole.rotation.x = Math.PI / 2
      hole.position.set(sx * nutW * 0.35, sy * nutH * 0.35, length * 0.1)
      g.add(hole)
    }
  }

  // Bearing block at end
  const blockW = Math.min(w, h) * 0.6
  const blockH = Math.min(w, h) * 0.5
  const blockD = length * 0.08

  const bearing = new THREE.Mesh(
    chamferedBox(blockW, blockH, blockD, blockW * 0.04),
    catMetal(),
  )
  bearing.position.z = -length * 0.42
  g.add(bearing)

  // Bearing bore
  const bore = new THREE.Mesh(
    new THREE.CylinderGeometry(shaftR * 1.5, shaftR * 1.5, blockD * 1.1, 20),
    getMaterial('dark_chrome'),
  )
  bore.rotation.x = Math.PI / 2
  bore.position.z = -length * 0.42
  g.add(bore)

  return g
}

// ── Bearing ─────────────────────────────────────────────────────────────────

function generateBearing(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h } = dims
  const outerR = w / 2
  const chamfer = outerR * 0.04
  const isLarge = id.includes('large')

  // Outer race
  const outerRace = new THREE.Mesh(
    chamferedCylinder(outerR, h, chamfer, 48),
    catMetal('brushed_steel', 0.2),
  )
  g.add(outerRace)

  // Inner race
  const innerR = outerR * (isLarge ? 0.55 : 0.5)
  const innerRace = new THREE.Mesh(
    chamferedCylinder(innerR, h * 1.01, chamfer * 0.5, 32),
    getMaterial('dark_chrome'),
  )
  g.add(innerRace)

  // Ball representation (knurled ring at midpoint between races)
  const ballR = (outerR + innerR) / 2
  const ballRing = knurledRing(ballR, h * 0.6, isLarge ? 20 : 14)
  g.add(ballRing)

  // Bore (dark center)
  const boreR = innerR * 0.65
  const bore = new THREE.Mesh(
    new THREE.CylinderGeometry(boreR, boreR, h * 1.05, 20),
    getMaterial('dark_chrome'),
  )
  g.add(bore)

  return g
}

// ── Flanged Bushing ─────────────────────────────────────────────────────────

function generateBushing(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h } = dims
  const r = w / 2
  const chamfer = r * 0.05

  // Main bushing body
  const body = new THREE.Mesh(
    chamferedCylinder(r, h, chamfer, 32),
    catMetal('brushed_steel', 0.25),
  )
  g.add(body)

  // Wider flange lip at one end
  const flangeR = r * 1.3
  const flangeH = h * 0.12
  const flange = new THREE.Mesh(
    chamferedCylinder(flangeR, flangeH, chamfer * 0.5, 32),
    catMetal('brushed_steel', 0.25),
  )
  flange.position.y = (h + flangeH) / 2
  g.add(flange)

  // Bore
  const boreR = r * 0.55
  const bore = new THREE.Mesh(
    new THREE.CylinderGeometry(boreR, boreR, h * 1.3, 20),
    getMaterial('dark_chrome'),
  )
  g.add(bore)

  return g
}

// ── Planetary Gearbox ───────────────────────────────────────────────────────

function generatePlanetaryGearbox(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h } = dims
  const r = w / 2
  const chamfer = r * 0.04

  // Main body cylinder
  const bodyH = h * 0.75
  const body = new THREE.Mesh(
    chamferedCylinder(r, bodyH, chamfer, 48),
    catMetal(),
  )
  g.add(body)

  // Output flange (slightly wider disc)
  const flangeR = r * 1.05
  const flangeH = h * 0.1
  const flange = new THREE.Mesh(
    chamferedCylinder(flangeR, flangeH, chamfer, 48),
    catMetal('anodized_aluminum', 0.35),
  )
  flange.position.y = (bodyH + flangeH) / 2
  g.add(flange)

  // Bolt circle on output flange
  const bolts = boltCircle(r * 0.75, r * 0.05, 6, flangeH * 1.1)
  bolts.position.y = (bodyH + flangeH) / 2
  g.add(bolts)

  // Output shaft
  const shaftR = r * 0.15
  const shaftH = h * 0.2
  const shaft = new THREE.Mesh(
    chamferedCylinder(shaftR, shaftH, shaftR * 0.15, 24),
    getMaterial('brushed_steel'),
  )
  shaft.position.y = (bodyH + flangeH * 2 + shaftH) / 2
  g.add(shaft)

  // Rear input boss
  const bossR = r * 0.6
  const bossH = h * 0.1
  const boss = new THREE.Mesh(
    chamferedCylinder(bossR, bossH, chamfer * 0.5, 32),
    catMetal(),
  )
  boss.position.y = -(bodyH + bossH) / 2
  g.add(boss)

  // Body rings (decorative separation lines)
  const ringMat = getMaterial('dark_chrome')
  for (const sy of [-0.2, 0.15]) {
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(r * 1.01, r * 0.015, 8, 48),
      ringMat,
    )
    ring.rotation.x = Math.PI / 2
    ring.position.y = bodyH * sy
    g.add(ring)
  }

  return g
}

// ── Spur/Bevel Gear Pair ────────────────────────────────────────────────────

function generateGearPair(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const isBevel = id.includes('bevel')

  const gear1R = Math.min(w, d) * 0.35
  const gear2R = gear1R * 0.65
  const gearH = h * 0.25
  const toothCount1 = 16
  const toothCount2 = 12

  // Gear 1 (larger)
  const gear1Geom = gearCylinder(gear1R, gear1R * 0.85, gearH, toothCount1)
  const gear1 = new THREE.Mesh(gear1Geom, catMetal('brushed_steel', 0.3))
  g.add(gear1)

  // Gear 1 shaft bore
  const bore1 = new THREE.Mesh(
    new THREE.CylinderGeometry(gear1R * 0.15, gear1R * 0.15, gearH * 1.1, 16),
    getMaterial('dark_chrome'),
  )
  g.add(bore1)

  // Gear 2 (smaller, offset and meshing)
  const gear2Geom = gearCylinder(gear2R, gear2R * 0.82, gearH, toothCount2)
  const gear2 = new THREE.Mesh(gear2Geom, catMetal('brushed_steel', 0.25))

  if (isBevel) {
    // Bevel: perpendicular arrangement
    gear2.rotation.x = Math.PI / 2
    gear2.position.set(gear1R + gear2R * 0.7, 0, 0)
  } else {
    // Spur: parallel, side by side
    gear2.position.set(gear1R + gear2R * 0.9, 0, 0)
  }
  g.add(gear2)

  // Gear 2 shaft bore
  const bore2 = new THREE.Mesh(
    new THREE.CylinderGeometry(gear2R * 0.15, gear2R * 0.15, gearH * 1.1, 16),
    getMaterial('dark_chrome'),
  )
  bore2.position.copy(gear2.position)
  if (isBevel) bore2.rotation.x = Math.PI / 2
  g.add(bore2)

  return g
}

// ── Worm Gear Set ───────────────────────────────────────────────────────────

function generateWormGearSet(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  const wormR = Math.min(w, d) * 0.1
  const wormLen = Math.max(w, d) * 0.6
  const wheelR = Math.min(w, d) * 0.3
  const gearH = h * 0.2

  // Worm shaft
  const wormShaft = new THREE.Mesh(
    chamferedCylinder(wormR, wormLen, wormR * 0.1, 24),
    getMaterial('brushed_steel'),
  )
  wormShaft.rotation.x = Math.PI / 2
  g.add(wormShaft)

  // Spiral ridge on worm (represented by a helix of small boxes)
  const spiralMat = getMaterial('brushed_steel')
  const spiralCount = Math.round(wormLen / (wormR * 0.8))
  for (let i = 0; i < spiralCount; i++) {
    const t = (i / spiralCount) - 0.5
    const angle = t * Math.PI * 2 * 4
    const ridge = new THREE.Mesh(
      new THREE.BoxGeometry(wormR * 0.3, wormR * 0.3, wormR * 0.15),
      spiralMat,
    )
    ridge.position.set(
      Math.cos(angle) * wormR * 1.05,
      Math.sin(angle) * wormR * 1.05,
      t * wormLen,
    )
    ridge.rotation.z = angle
    g.add(ridge)
  }

  // Worm wheel (perpendicular)
  const wheelGeom = gearCylinder(wheelR, wheelR * 0.85, gearH, 20)
  const wheel = new THREE.Mesh(wheelGeom, catMetal('brushed_steel', 0.3))
  wheel.position.set(0, -(wormR + wheelR * 0.85), 0)
  g.add(wheel)

  // Wheel bore
  const wheelBore = new THREE.Mesh(
    new THREE.CylinderGeometry(wheelR * 0.15, wheelR * 0.15, gearH * 1.1, 16),
    getMaterial('dark_chrome'),
  )
  wheelBore.position.copy(wheel.position)
  g.add(wheelBore)

  return g
}

// ── Chain & Sprocket Set ────────────────────────────────────────────────────

function generateChainSprocket(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  const sprocketR = Math.min(w, d) * 0.25
  const sprocketH = h * 0.15
  const spacing = w * 0.3
  const toothCount = 14

  // Two sprockets
  for (const sx of [-1, 1]) {
    // Sprocket disc
    const sprocket = new THREE.Mesh(
      chamferedCylinder(sprocketR, sprocketH, sprocketR * 0.04, 32),
      catMetal('brushed_steel', 0.3),
    )
    sprocket.position.set(sx * spacing, 0, 0)
    g.add(sprocket)

    // Radial tooth bumps
    const bumpMat = getMaterial('brushed_steel')
    for (let i = 0; i < toothCount; i++) {
      const a = (i / toothCount) * Math.PI * 2
      const bump = new THREE.Mesh(
        new THREE.BoxGeometry(sprocketR * 0.15, sprocketH * 0.9, sprocketR * 0.12),
        bumpMat,
      )
      bump.position.set(
        sx * spacing + Math.cos(a) * sprocketR * 1.05,
        0,
        Math.sin(a) * sprocketR * 1.05,
      )
      bump.rotation.y = -a
      g.add(bump)
    }

    // Shaft bore
    const bore = new THREE.Mesh(
      new THREE.CylinderGeometry(sprocketR * 0.2, sprocketR * 0.2, sprocketH * 1.2, 16),
      getMaterial('dark_chrome'),
    )
    bore.position.set(sx * spacing, 0, 0)
    g.add(bore)
  }

  // Chain runs (two thin boxes connecting sprockets)
  const chainLen = spacing * 2 + sprocketR * 2
  const chainThick = sprocketH * 0.15
  const chainMat = getMaterial('dark_chrome')

  for (const sz of [-1, 1]) {
    const chain = new THREE.Mesh(
      new THREE.BoxGeometry(chainLen, chainThick, sprocketH * 0.4),
      chainMat,
    )
    chain.position.z = sz * sprocketR
    g.add(chain)
  }

  return g
}

// ── Universal Joint ─────────────────────────────────────────────────────────

function generateUniversalJoint(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  const yokeW = Math.min(w, d) * 0.3
  const yokeH = Math.min(w, d) * 0.5
  const yokeThick = Math.min(w, d) * 0.08
  const chamfer = yokeThick * 0.3
  const mat = catMetal('brushed_steel', 0.25)

  // Yoke 1 (U-shape in XY plane)
  for (const sx of [-1, 1]) {
    const fork = new THREE.Mesh(
      chamferedBox(yokeThick, yokeThick, yokeH, chamfer),
      mat,
    )
    fork.position.set(sx * yokeW, 0, yokeH * 0.25)
    g.add(fork)
  }
  const yoke1Bar = new THREE.Mesh(
    chamferedBox(yokeW * 2 + yokeThick, yokeThick, yokeThick, chamfer),
    mat,
  )
  yoke1Bar.position.z = yokeH * 0.5
  g.add(yoke1Bar)

  // Yoke 2 (U-shape in YZ plane, perpendicular)
  for (const sz of [-1, 1]) {
    const fork = new THREE.Mesh(
      chamferedBox(yokeThick, yokeThick, yokeH, chamfer),
      mat,
    )
    fork.position.set(0, sz * yokeW, -yokeH * 0.25)
    g.add(fork)
  }
  const yoke2Bar = new THREE.Mesh(
    chamferedBox(yokeThick, yokeW * 2 + yokeThick, yokeThick, chamfer),
    mat,
  )
  yoke2Bar.position.z = -yokeH * 0.5
  g.add(yoke2Bar)

  // Center cross (sphere + 4 short cylinders)
  const crossR = yokeW * 0.15
  const crossSphere = new THREE.Mesh(
    new THREE.SphereGeometry(crossR, 16, 16),
    getMaterial('brushed_steel'),
  )
  g.add(crossSphere)

  const pinR = crossR * 0.5
  const pinLen = yokeW * 0.8
  const pinMat = getMaterial('brushed_steel')

  // X-axis pins
  for (const sx of [-1, 1]) {
    const pin = new THREE.Mesh(
      new THREE.CylinderGeometry(pinR, pinR, pinLen, 12),
      pinMat,
    )
    pin.rotation.z = Math.PI / 2
    pin.position.x = sx * pinLen * 0.3
    g.add(pin)
  }

  // Y-axis pins
  for (const sy of [-1, 1]) {
    const pin = new THREE.Mesh(
      new THREE.CylinderGeometry(pinR, pinR, pinLen, 12),
      pinMat,
    )
    pin.position.y = sy * pinLen * 0.3
    g.add(pin)
  }

  return g
}

// ── Flexible / Rigid Coupling ───────────────────────────────────────────────

function generateCoupling(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h } = dims
  const r = w / 2
  const chamfer = r * 0.05
  const isRigid = id.includes('rigid')

  // Main coupling body
  const body = new THREE.Mesh(
    chamferedCylinder(r, h, chamfer, 32),
    isRigid ? getMaterial('brushed_steel') : catMetal(),
  )
  g.add(body)

  // Split line (dark groove)
  const splitMat = getMaterial('dark_chrome')
  if (!isRigid) {
    // Flexible: helical split pattern (simplified as horizontal lines)
    for (let i = 0; i < 3; i++) {
      const y = -h * 0.3 + i * h * 0.3
      const split = new THREE.Mesh(
        new THREE.TorusGeometry(r * 1.01, r * 0.015, 6, 32),
        splitMat,
      )
      split.rotation.x = Math.PI / 2
      split.position.y = y
      g.add(split)
    }
  } else {
    // Rigid: single split line
    const split = new THREE.Mesh(
      new THREE.BoxGeometry(r * 0.02, h * 1.01, r * 2.1),
      splitMat,
    )
    g.add(split)
  }

  // Clamping screw
  const screw = screwHead(r * 0.1, h * 0.15)
  screw.position.set(r * 0.95, h * 0.2, 0)
  screw.rotation.z = Math.PI / 2
  g.add(screw)

  // Bore hints at each end
  const boreR = r * 0.35
  for (const sy of [-1, 1]) {
    const bore = new THREE.Mesh(
      new THREE.CylinderGeometry(boreR, boreR, h * 0.1, 16),
      getMaterial('dark_chrome'),
    )
    bore.position.y = sy * h * 0.5
    g.add(bore)
  }

  return g
}

// ── Rack & Pinion ───────────────────────────────────────────────────────────

function generateRackPinion(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims

  const rackW = w * 0.8
  const rackH = h * 0.2
  const rackD = d * 0.3
  const chamfer = Math.min(rackH, rackD) * 0.06
  const mat = getMaterial('brushed_steel')

  // Rack bar
  const rack = new THREE.Mesh(chamferedBox(rackW, rackD, rackH, chamfer), mat)
  rack.position.y = -h * 0.25
  g.add(rack)

  // Raised tooth strip on top of rack
  const toothStripH = rackH * 0.25
  const toothStrip = new THREE.Mesh(
    chamferedBox(rackW * 0.95, rackD * 0.6, toothStripH, chamfer * 0.3),
    catMetal('brushed_steel', 0.2),
  )
  toothStrip.position.y = -h * 0.25 + (rackH + toothStripH) / 2
  g.add(toothStrip)

  // Individual tooth bumps
  const toothCount = Math.max(8, Math.round(rackW / (rackH * 0.5)))
  const toothW = rackW * 0.7 / toothCount
  const toothMat = catMetal('brushed_steel', 0.15)
  for (let i = 0; i < toothCount; i++) {
    const px = -rackW * 0.35 + (i + 0.5) * (rackW * 0.7 / toothCount)
    const tooth = new THREE.Mesh(
      new THREE.BoxGeometry(toothW * 0.5, rackD * 0.4, toothStripH * 0.8),
      toothMat,
    )
    tooth.position.set(px, -h * 0.25 + rackH / 2 + toothStripH, 0)
    g.add(tooth)
  }

  // Pinion gear
  const pinionR = h * 0.2
  const pinionH = rackD * 0.8
  const pinionGeom = gearCylinder(pinionR, pinionR * 0.85, pinionH, 12)
  const pinion = new THREE.Mesh(pinionGeom, catMetal('brushed_steel', 0.3))
  pinion.position.set(0, -h * 0.25 + rackH / 2 + toothStripH + pinionR * 0.85, 0)
  g.add(pinion)

  // Pinion bore
  const bore = new THREE.Mesh(
    new THREE.CylinderGeometry(pinionR * 0.15, pinionR * 0.15, pinionH * 1.1, 16),
    getMaterial('dark_chrome'),
  )
  bore.position.copy(pinion.position)
  g.add(bore)

  return g
}

// ── Crossed Roller Bearing / Slewing Ring ───────────────────────────────────

function generateSlewingRing(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h } = dims
  const outerR = w / 2
  const chamfer = outerR * 0.03

  // Outer ring
  const outerRing = new THREE.Mesh(
    chamferedCylinder(outerR, h, chamfer, 48),
    catMetal('brushed_steel', 0.2),
  )
  g.add(outerRing)

  // Inner ring
  const innerR = outerR * 0.7
  const innerRing = new THREE.Mesh(
    chamferedCylinder(innerR, h * 1.01, chamfer * 0.5, 48),
    getMaterial('brushed_steel'),
  )
  g.add(innerRing)

  // Rolling element track (visible ring between races)
  const trackR = (outerR + innerR) / 2
  const track = new THREE.Mesh(
    new THREE.TorusGeometry(trackR, h * 0.15, 8, 48),
    getMaterial('dark_chrome'),
  )
  track.rotation.x = Math.PI / 2
  g.add(track)

  // Bore
  const boreR = innerR * 0.75
  const bore = new THREE.Mesh(
    new THREE.CylinderGeometry(boreR, boreR, h * 1.05, 32),
    getMaterial('dark_chrome'),
  )
  g.add(bore)

  // Bolt holes on outer ring top face
  const bolts = boltCircle(outerR * 0.85, outerR * 0.03, 12, h * 0.5)
  bolts.position.y = h * 0.25
  g.add(bolts)

  return g
}

// ── Dispatcher ──────────────────────────────────────────────────────────────

export function generateRichTransmission(id: string, dims: GeneratorDims): THREE.Group {
  if (id.includes('timing_belt'))
    return generateTimingBelt(id, dims)
  if (id.includes('leadscrew') || id.includes('ballscrew'))
    return generateLeadscrew(id, dims)
  if (id.includes('bearing_deep_groove') || id.includes('bearing_large'))
    return generateBearing(id, dims)
  if (id.includes('bushing'))
    return generateBushing(id, dims)
  if (id.includes('planetary'))
    return generatePlanetaryGearbox(id, dims)
  if (id.includes('spur_gear') || id.includes('bevel_gear'))
    return generateGearPair(id, dims)
  if (id.includes('worm_gear'))
    return generateWormGearSet(id, dims)
  if (id.includes('chain_sprocket'))
    return generateChainSprocket(id, dims)
  if (id.includes('universal_joint'))
    return generateUniversalJoint(id, dims)
  if (id.includes('flexible_coupling') || id.includes('rigid_coupling'))
    return generateCoupling(id, dims)
  if (id.includes('rack_pinion'))
    return generateRackPinion(id, dims)
  if (id.includes('crossed_roller') || id.includes('slewing_ring'))
    return generateSlewingRing(id, dims)

  // Fallback: generic transmission cylinder
  const g = new THREE.Group()
  const r = Math.min(dims.x, dims.z) / 2
  g.add(new THREE.Mesh(chamferedCylinder(r, dims.y, r * 0.04, 32), catMetal()))
  return g
}

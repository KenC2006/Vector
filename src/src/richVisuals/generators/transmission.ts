/**
 * Rich visual generators for transmission components.
 * Uses profile-based geometry: LatheGeometry bearings, gearCylinder teeth,
 * flangePlate bolt circles, chamferedCylinder shafts.
 */
import * as THREE from 'three'
import type { ComponentVisualDims } from './index'
import { getMaterial, getTintedMaterial } from '../materials'
import {
  mountingHole, screwHead,
  boltCircle, knurledRing, flangePlate,
  gearCylinder, bearingProfile, bearingOuterProfile,
} from '../primitives'
import { nurbsFilletBox, nurbsCylinder, nurbsTorus } from '../nurbs'

const DEFAULT_COLOR: [number, number, number] = [0.56, 0.27, 0.68]  // purple
let CAT_COLOR: [number, number, number] = DEFAULT_COLOR

function catMetal(base: string = 'anodized_aluminum', strength = 0.3) {
  return getTintedMaterial(base, ...CAT_COLOR, strength)
}

// ── Bearing ─────────────────────────────────────────────────────────────────

function generateBearing(id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h } = dims
  const outerR = w / 2
  const isLarge = id.includes('large')
  const innerR = outerR * (isLarge ? 0.55 : 0.5)
  const width = h

  // Inner race + ball track groove — revolved bearing cross-section
  const innerProfile = bearingProfile(outerR, innerR, width)
  const innerGeom = new THREE.LatheGeometry(innerProfile, 48)
  const innerRace = new THREE.Mesh(innerGeom, catMetal('brushed_steel', 0.2))
  g.add(innerRace)

  // Outer race — revolved outer profile
  const outerProfile = bearingOuterProfile(outerR, innerR, width)
  const outerGeom = new THREE.LatheGeometry(outerProfile, 48)
  const outerRace = new THREE.Mesh(outerGeom, getMaterial('brushed_steel'))
  g.add(outerRace)

  // Bore (dark center)
  const boreR = innerR * 0.65
  const bore = new THREE.Mesh(
    new THREE.CylinderGeometry(boreR, boreR, width * 1.05, 20),
    getMaterial('dark_chrome'),
  )
  g.add(bore)

  return g
}

// ── Flanged Bushing ─────────────────────────────────────────────────────────

function generateBushing(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h } = dims
  const r = w / 2
  const chamfer = r * 0.05

  const body = new THREE.Mesh(
    nurbsCylinder(r, h, chamfer, 32),
    catMetal('brushed_steel', 0.25),
  )
  g.add(body)

  // Wider flange lip at one end
  const flangeR = r * 1.3
  const flangeH = h * 0.12
  const flange = new THREE.Mesh(
    nurbsCylinder(flangeR, flangeH, chamfer * 0.5, 32),
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

// ── Spur/Bevel Gear Pair ────────────────────────────────────────────────────

function generateGearPair(id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const isBevel = id.includes('bevel')

  const gear1RootR = Math.min(w, d) * (isBevel ? 0.2 : 0.3)
  const gear1TipR = gear1RootR * 1.15
  const gear2RootR = gear1RootR * (isBevel ? 0.75 : 0.6)
  const gear2TipR = gear2RootR * 1.15
  const gearH = h * 0.8
  const toothCount1 = 20
  const toothCount2 = 14

  // Gear 1 (larger) — actual tooth profile via gearCylinder
  const gear1Group = gearCylinder(gear1RootR, gear1TipR, gearH, toothCount1)
  g.add(gear1Group)

  // Gear 1 shaft bore
  const bore1 = new THREE.Mesh(
    new THREE.CylinderGeometry(gear1RootR * 0.15, gear1RootR * 0.15, gearH * 1.1, 16),
    getMaterial('dark_chrome'),
  )
  g.add(bore1)

  // Gear 2 (smaller) — actual tooth profile
  const gear2Group = gearCylinder(gear2RootR, gear2TipR, gearH, toothCount2)

  if (isBevel) {
    gear2Group.rotation.x = Math.PI / 2
    gear2Group.position.set(gear1TipR + gear2TipR * 0.45, 0, 0)
  } else {
    gear2Group.position.set(gear1TipR + gear2TipR * 0.9, 0, 0)
  }
  g.add(gear2Group)

  // Gear 2 shaft bore
  const bore2 = new THREE.Mesh(
    new THREE.CylinderGeometry(gear2RootR * 0.15, gear2RootR * 0.15, gearH * 1.1, 16),
    getMaterial('dark_chrome'),
  )
  bore2.position.copy(gear2Group.position)
  if (isBevel) bore2.rotation.x = Math.PI / 2
  g.add(bore2)

  return g
}

// ── Planetary Gearbox ───────────────────────────────────────────────────────

function generatePlanetaryGearbox(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h } = dims
  const r = w / 2
  const chamfer = r * 0.04

  // Main body cylinder
  const bodyH = h * 0.75
  const body = new THREE.Mesh(
    nurbsCylinder(r, bodyH, chamfer, 48),
    catMetal(),
  )
  body.rotation.x = Math.PI / 2
  g.add(body)

  // Output flange plate with bolt circle
  const flangeR = r * 1.05
  const flangeH = h * 0.1
  const flange = flangePlate(flangeR, flangeH, 6, r * 0.75, r * 0.05)
  flange.rotation.x = Math.PI / 2
  flange.position.z = (bodyH + flangeH) / 2
  g.add(flange)

  // Output shaft
  const shaftR = r * 0.15
  const shaftH = h * 0.2
  const shaft = new THREE.Mesh(
    nurbsCylinder(shaftR, shaftH, shaftR * 0.15, 24),
    getMaterial('brushed_steel'),
  )
  shaft.rotation.x = Math.PI / 2
  shaft.position.z = (bodyH + flangeH * 2 + shaftH) / 2
  g.add(shaft)

  // Rear input boss
  const bossR = r * 0.6
  const bossH = h * 0.1
  const boss = new THREE.Mesh(
    nurbsCylinder(bossR, bossH, chamfer * 0.5, 32),
    catMetal(),
  )
  boss.rotation.x = Math.PI / 2
  boss.position.z = -(bodyH + bossH) / 2
  g.add(boss)

  // Body rings (decorative separation lines)
  const ringMat = getMaterial('dark_chrome')
  for (const sy of [-0.2, 0.15]) {
    const ring = new THREE.Mesh(
      nurbsTorus(r * 1.01, r * 0.015, 48, 8),
      ringMat,
    )
    ring.position.z = bodyH * sy
    g.add(ring)
  }

  return g
}

// ── Leadscrew ───────────────────────────────────────────────────────────────

function generateLeadscrew(id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const isBall = id.includes('ballscrew')
  const shaftR = isBall ? Math.min(w, h) * 0.12 : Math.min(w, h) * 0.08

  // Chamfered shaft
  const shaft = new THREE.Mesh(
    nurbsCylinder(shaftR, length * 0.9, shaftR * 0.1, 24),
    getMaterial('brushed_steel'),
  )
  shaft.rotation.x = Math.PI / 2
  g.add(shaft)

  // Thread representation
  const threadRing = knurledRing(shaftR * 1.05, length * 0.85, Math.round(length / (shaftR * 1.5)))
  threadRing.rotation.x = Math.PI / 2
  g.add(threadRing)

  // Nut block (chamferedBox)
  const nutW = Math.min(w, h) * 0.5
  const nutH = Math.min(w, h) * 0.5
  const nutD = length * 0.12
  const nutMat = isBall ? catMetal('anodized_aluminum', 0.4) : catMetal()

  const nut = new THREE.Mesh(
    nurbsFilletBox(nutW, nutH, nutD, nutW * 0.06, 12),
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
    nurbsFilletBox(blockW, blockH, blockD, blockW * 0.04, 12),
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

// ── Worm Gear Set ───────────────────────────────────────────────────────────

function generateWormGearSet(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims

  const wormR = Math.min(w, d) * 0.1
  const wormLen = Math.max(w, d) * 0.6
  const wheelRootR = Math.min(w, d) * 0.25
  const wheelTipR = wheelRootR * 1.15
  const gearH = h * 0.35

  // Worm shaft
  const wormShaft = new THREE.Mesh(
    nurbsCylinder(wormR, wormLen, wormR * 0.1, 24),
    getMaterial('brushed_steel'),
  )
  wormShaft.rotation.x = Math.PI / 2
  g.add(wormShaft)

  // Spiral ridge on worm
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

  // Worm wheel — gearCylinder with actual tooth profiles
  const wheelGroup = gearCylinder(wheelRootR, wheelTipR, gearH, 20)
  wheelGroup.position.set(0, -(wormR + wheelRootR * 0.85), 0)
  g.add(wheelGroup)

  // Wheel bore
  const wheelBore = new THREE.Mesh(
    new THREE.CylinderGeometry(wheelRootR * 0.15, wheelRootR * 0.15, gearH * 1.1, 16),
    getMaterial('dark_chrome'),
  )
  wheelBore.position.copy(wheelGroup.position)
  g.add(wheelBore)

  return g
}

// ── Chain & Sprocket Set ────────────────────────────────────────────────────

function generateChainSprocket(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: length, z: h } = dims

  const sprocketRootR = Math.min(w, length) * 0.22
  const sprocketTipR = Math.min(w, length) * 0.27
  const sprocketH = h * 0.65
  const spacing = length * 0.35

  // Two sprockets using gearCylinder for tooth profiles
  for (const sz of [-1, 1]) {
    const sprocket = gearCylinder(sprocketRootR, sprocketTipR, sprocketH, 14)
    sprocket.position.set(0, 0, sz * spacing)
    g.add(sprocket)

    // Shaft bore
    const bore = new THREE.Mesh(
      new THREE.CylinderGeometry(sprocketRootR * 0.2, sprocketRootR * 0.2, sprocketH * 1.2, 16),
      getMaterial('dark_chrome'),
    )
    bore.position.set(0, 0, sz * spacing)
    g.add(bore)
  }

  // Chain runs (two thin boxes connecting sprockets)
  const chainLen = spacing * 2 + sprocketTipR * 2
  const chainThick = sprocketH * 0.15
  const chainMat = getMaterial('dark_chrome')

  for (const sx of [-1, 1]) {
    const chain = new THREE.Mesh(
      new THREE.BoxGeometry(chainThick, chainThick, chainLen),
      chainMat,
    )
    chain.position.x = sx * sprocketRootR
    g.add(chain)
  }

  return g
}

// ── Universal Joint ─────────────────────────────────────────────────────────

function generateUniversalJoint(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: length } = dims

  const diameter = Math.min(w, d)
  const yokeW = diameter * 0.3
  const yokeH = length * 0.65
  const yokeThick = diameter * 0.08
  const chamfer = yokeThick * 0.3
  const mat = catMetal('brushed_steel', 0.25)

  // Yoke 1 (U-shape in XY plane) — nurbsFilletBox forks
  for (const sx of [-1, 1]) {
    const fork = new THREE.Mesh(
      nurbsFilletBox(yokeThick, yokeThick, yokeH, chamfer, 12),
      mat,
    )
    fork.position.set(sx * yokeW, 0, yokeH * 0.25)
    g.add(fork)
  }
  const yoke1Bar = new THREE.Mesh(
    nurbsFilletBox(yokeW * 2 + yokeThick, yokeThick, yokeThick, chamfer, 12),
    mat,
  )
  yoke1Bar.position.z = yokeH * 0.5
  g.add(yoke1Bar)

  // Yoke 2 (U-shape in YZ plane, perpendicular) — nurbsFilletBox forks
  for (const sz of [-1, 1]) {
    const fork = new THREE.Mesh(
      nurbsFilletBox(yokeThick, yokeThick, yokeH, chamfer, 12),
      mat,
    )
    fork.position.set(0, sz * yokeW, -yokeH * 0.25)
    g.add(fork)
  }
  const yoke2Bar = new THREE.Mesh(
    nurbsFilletBox(yokeThick, yokeW * 2 + yokeThick, yokeThick, chamfer, 12),
    mat,
  )
  yoke2Bar.position.z = -yokeH * 0.5
  g.add(yoke2Bar)

  // Center cross — sphere + 4 short chamferedCylinder pins
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
      nurbsCylinder(pinR, pinLen, pinR * 0.1, 12),
      pinMat,
    )
    pin.rotation.z = Math.PI / 2
    pin.position.x = sx * pinLen * 0.3
    g.add(pin)
  }

  // Y-axis pins
  for (const sy of [-1, 1]) {
    const pin = new THREE.Mesh(
      nurbsCylinder(pinR, pinLen, pinR * 0.1, 12),
      pinMat,
    )
    pin.position.y = sy * pinLen * 0.3
    g.add(pin)
  }

  return g
}

// ── Flexible / Rigid Coupling ───────────────────────────────────────────────

function generateCoupling(id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h } = dims
  const r = w / 2
  const chamfer = r * 0.05
  const isRigid = id.includes('rigid')

  // Main coupling body — nurbsCylinder
  const body = new THREE.Mesh(
    nurbsCylinder(r, h, chamfer, 32),
    isRigid ? getMaterial('brushed_steel') : catMetal(),
  )
  body.rotation.x = Math.PI / 2
  g.add(body)

  // Split line detail
  const splitMat = getMaterial('dark_chrome')
  if (!isRigid) {
    // Flexible: helical split pattern (ring grooves)
    for (let i = 0; i < 3; i++) {
      const z = -h * 0.3 + i * h * 0.3
      const split = new THREE.Mesh(
        nurbsTorus(r * 1.01, r * 0.015, 32, 6),
        splitMat,
      )
      split.position.z = z
      g.add(split)
    }
  } else {
    // Rigid: single split line
    const split = new THREE.Mesh(
      new THREE.BoxGeometry(r * 0.02, r * 2.1, h * 1.01),
      splitMat,
    )
    g.add(split)
  }

  // Clamping screw
  const screw = screwHead(r * 0.1, h * 0.15)
  screw.position.set(r * 0.95, 0, h * 0.2)
  screw.rotation.z = Math.PI / 2
  g.add(screw)

  // Bore hints at each end
  const boreR = r * 0.35
  for (const sz of [-1, 1]) {
    const bore = new THREE.Mesh(
      new THREE.CylinderGeometry(boreR, boreR, h * 0.1, 16),
      getMaterial('dark_chrome'),
    )
    bore.rotation.x = Math.PI / 2
    bore.position.z = sz * h * 0.5
    g.add(bore)
  }

  return g
}

// ── Rack & Pinion ───────────────────────────────────────────────────────────

function generateRackPinion(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: width, y: length, z: height } = dims

  const rackLength = length * 0.8
  const rackH = height * 0.22
  const rackD = width * 0.7
  const chamfer = Math.min(rackH, rackD) * 0.06

  // Rack bar
  const rack = new THREE.Mesh(nurbsFilletBox(rackD, rackH, rackLength, chamfer, 16), getMaterial('brushed_steel'))
  rack.position.y = -height * 0.25
  g.add(rack)

  // Tooth strip on top of rack
  const toothStripH = rackH * 0.25
  const toothStrip = new THREE.Mesh(
    nurbsFilletBox(rackD * 0.8, toothStripH, rackLength * 0.95, chamfer * 0.3, 12),
    catMetal('brushed_steel', 0.2),
  )
  toothStrip.position.y = -height * 0.25 + (rackH + toothStripH) / 2
  g.add(toothStrip)

  // Individual tooth bumps
  const toothCount = Math.max(8, Math.round(rackLength / (rackH * 0.5)))
  const toothMat = catMetal('brushed_steel', 0.15)
  for (let i = 0; i < toothCount; i++) {
    const toothD = rackLength * 0.7 / toothCount
    const pz = -rackLength * 0.35 + (i + 0.5) * (rackLength * 0.7 / toothCount)
    const tooth = new THREE.Mesh(
      new THREE.BoxGeometry(rackD * 0.5, toothStripH * 0.8, toothD * 0.5),
      toothMat,
    )
    tooth.position.set(0, -height * 0.25 + rackH / 2 + toothStripH, pz)
    g.add(tooth)
  }

  // Pinion gear — gearCylinder with actual teeth
  const pinionRootR = height * 0.6
  const pinionTipR = pinionRootR * 1.15
  const pinionH = rackD * 0.8

  const pinion = gearCylinder(pinionRootR, pinionTipR, pinionH, 12)
  pinion.position.set(0, -height * 0.25 + rackH / 2 + toothStripH + pinionRootR * 0.85, 0)
  g.add(pinion)

  // Pinion bore
  const bore = new THREE.Mesh(
    new THREE.CylinderGeometry(pinionRootR * 0.15, pinionRootR * 0.15, pinionH * 1.1, 16),
    getMaterial('dark_chrome'),
  )
  bore.position.copy(pinion.position)
  g.add(bore)

  return g
}

// ── Crossed Roller Bearing / Slewing Ring ───────────────────────────────────

function generateSlewingRing(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h } = dims
  const outerR = w / 2
  const chamfer = outerR * 0.03

  // Outer ring
  const outerRing = new THREE.Mesh(
    nurbsCylinder(outerR, h, chamfer, 48),
    catMetal('brushed_steel', 0.2),
  )
  g.add(outerRing)

  // Inner ring
  const innerR = outerR * 0.7
  const innerRing = new THREE.Mesh(
    nurbsCylinder(innerR, h * 1.01, chamfer * 0.5, 48),
    getMaterial('brushed_steel'),
  )
  g.add(innerRing)

  // Rolling element track
  const trackR = (outerR + innerR) / 2
  const track = new THREE.Mesh(
    nurbsTorus(trackR, h * 0.15, 48, 8),
    getMaterial('dark_chrome'),
  )
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

export function generateRichTransmission(id: string, dims: ComponentVisualDims, color?: [number, number, number]): THREE.Group {
  CAT_COLOR = color ?? DEFAULT_COLOR
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
  g.add(new THREE.Mesh(nurbsCylinder(r, dims.y, r * 0.04, 32), catMetal()))
  return g
}

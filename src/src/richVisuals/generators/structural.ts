/**
 * Rich visual generators for structural components.
 * Uses profile-based geometry (tSlotExtrusion, iBeamExtrusion, etc.) for
 * Fusion-quality visuals instead of composing primitive boxes with dark grooves.
 */
import * as THREE from 'three'
import type { ComponentVisualDims } from './index'
import { getMaterial, getTintedMaterial } from '../materials'
import {
  mountingHole, screwHead,
  labelRecess, knurledRing,
  tSlotExtrusion, iBeamExtrusion, cChannelExtrusion, lBracketExtrusion,
} from '../primitives'
import { nurbsFilletBox, nurbsCylinder } from '../nurbs'

const DEFAULT_COLOR: [number, number, number] = [0.66, 0.70, 0.72]  // silver-grey
let CAT_COLOR: [number, number, number] = DEFAULT_COLOR

function catMetal(base: string = 'anodized_aluminum', strength = 0.3) {
  return getTintedMaterial(base, ...CAT_COLOR, strength)
}

function isSteel(id: string): boolean {
  return id.includes('steel')
}

function baseMat(id: string) {
  return isSteel(id) ? getMaterial('brushed_steel') : catMetal('anodized_aluminum', 0.25)
}

// ── T-Slot Extrusion 2020/4040 ─────────────────────────────────────────────

function generateExtrusion(id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: length } = dims
  const is4040 = id.includes('4040')
  const size = is4040 ? Math.min(w, d) : Math.min(w, d)

  // Single extruded T-slot profile — the biggest visual upgrade
  const body = new THREE.Mesh(
    tSlotExtrusion(size, length),
    catMetal('anodized_aluminum', 0.2),
  )
  body.rotation.x = -Math.PI / 2
  g.add(body)

  // For 4040: four T-slot extrusions in a 2x2 grid
  if (is4040) {
    // Remove the single extrusion and replace with 2x2 grid
    g.remove(body)
    const halfSize = size * 0.5
    const offset = size * 0.25
    for (const sx of [-1, 1]) {
      for (const sz of [-1, 1]) {
        const quad = new THREE.Mesh(
          tSlotExtrusion(halfSize, length),
          catMetal('anodized_aluminum', 0.2),
        )
        quad.rotation.x = -Math.PI / 2
        quad.position.set(sx * offset, 0, sz * offset)
        g.add(quad)
      }
    }
  }

  return g
}

// ── I-Beam ──────────────────────────────────────────────────────────────────

function generateIBeam(id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const mat = baseMat(id)

  const flangeT = h * 0.12
  const webT = w * 0.2

  // Single extruded I-profile
  const beam = new THREE.Mesh(
    iBeamExtrusion(w, h, webT, flangeT, length),
    mat,
  )
  beam.rotation.x = -Math.PI / 2
  g.add(beam)

  return g
}

// ── C-Channel ───────────────────────────────────────────────────────────────

function generateCChannel(id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const mat = baseMat(id)

  const thickness = Math.min(w, h) * 0.12

  // Single extruded C-profile
  const channel = new THREE.Mesh(
    cChannelExtrusion(w, h, thickness, length),
    mat,
  )
  channel.rotation.x = -Math.PI / 2
  g.add(channel)

  return g
}

// ── Angle Stock (L-profile) ─────────────────────────────────────────────────

function generateAngle(id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const mat = baseMat(id)
  const thick = Math.min(w, h) * 0.15

  // Extruded L-profile
  const angle = new THREE.Mesh(
    lBracketExtrusion(h, w, thick, length),
    mat,
  )
  g.add(angle)

  return g
}

// ── L-Bracket ───────────────────────────────────────────────────────────────

function generateLBracket(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const mat = catMetal()
  const thick = Math.min(w, h, d) * 0.18

  // Extruded L-bracket profile
  const bracket = new THREE.Mesh(
    lBracketExtrusion(h, w, thick, d),
    mat,
  )
  g.add(bracket)

  // Mounting holes on horizontal face
  const holeR = Math.min(w, d) * 0.04
  for (const sx of [-1, 1]) {
    const hole = mountingHole(holeR, thick * 1.1)
    hole.position.set(sx * w * 0.28, -h / 2 + thick / 2, 0)
    g.add(hole)
  }

  // Mounting holes on vertical face
  for (const sx of [-1, 1]) {
    const hole = mountingHole(holeR, thick * 1.1)
    hole.rotation.x = Math.PI / 2
    hole.position.set(sx * w * 0.28, h * 0.25, -d / 2 + thick / 2)
    g.add(hole)
  }

  return g
}

// ── U-Bracket ───────────────────────────────────────────────────────────────

function generateUBracket(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const chamfer = Math.min(w, h, d) * 0.04
  const mat = catMetal()
  const thick = Math.min(w, h, d) * 0.15

  // Base plate
  const base = new THREE.Mesh(nurbsFilletBox(w, thick, d, chamfer, 16), mat)
  base.position.y = -h / 2 + thick / 2
  g.add(base)

  // Two side walls
  for (const sx of [-1, 1]) {
    const wall = new THREE.Mesh(nurbsFilletBox(thick, h - thick, d * 0.8, chamfer, 12), mat)
    wall.position.set(sx * (w / 2 - thick / 2), thick / 2, 0)
    g.add(wall)

    // Mounting hole on each side wall
    const hole = mountingHole(Math.min(w, d) * 0.04, thick * 1.1)
    hole.rotation.z = Math.PI / 2
    hole.position.set(sx * (w / 2 - thick / 2), h * 0.15, 0)
    g.add(hole)
  }

  // Mounting holes on base
  const holeR = Math.min(w, d) * 0.04
  for (const sx of [-1, 1]) {
    const hole = mountingHole(holeR, thick * 1.1)
    hole.position.set(sx * w * 0.25, -h / 2 + thick / 2, 0)
    g.add(hole)
  }

  return g
}

// ── T-Bracket ───────────────────────────────────────────────────────────────

function generateTBracket(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const chamfer = Math.min(w, h, d) * 0.04
  const mat = catMetal()
  const thick = Math.min(w, h, d) * 0.15

  // Horizontal bar
  const hBar = new THREE.Mesh(nurbsFilletBox(w, thick, d, chamfer, 16), mat)
  hBar.position.y = -h / 2 + thick / 2
  g.add(hBar)

  // Vertical stem
  const stem = new THREE.Mesh(nurbsFilletBox(thick * 1.5, h - thick, d * 0.8, chamfer, 12), mat)
  stem.position.y = thick / 2
  g.add(stem)

  // Mounting holes
  const holeR = Math.min(w, d) * 0.035
  for (const sx of [-1, 1]) {
    const hole = mountingHole(holeR, thick * 1.1)
    hole.position.set(sx * w * 0.35, -h / 2 + thick / 2, 0)
    g.add(hole)
  }
  const stemHole = mountingHole(holeR, thick * 1.6)
  stemHole.rotation.x = Math.PI / 2
  stemHole.position.set(0, h * 0.2, 0)
  g.add(stemHole)

  return g
}

// ── Joint Plate ─────────────────────────────────────────────────────────────

function generateJointPlate(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const thick = h
  const chamfer = Math.min(Math.min(w, d) * 0.03, thick * 0.4)
  const mat = catMetal()

  const plate = new THREE.Mesh(nurbsFilletBox(w, thick, d, chamfer, 16), mat)
  g.add(plate)

  // Grid of mounting holes
  const holeR = Math.min(w, d) * 0.03
  const cols = Math.max(2, Math.round(w / (Math.min(w, d) * 0.3)))
  const rows = Math.max(2, Math.round(d / (Math.min(w, d) * 0.3)))
  for (let ix = 0; ix < cols; ix++) {
    for (let iy = 0; iy < rows; iy++) {
      const px = -w * 0.4 + (ix / (cols - 1)) * w * 0.8
      const pz = -d * 0.4 + (iy / (rows - 1)) * d * 0.8
      const hole = mountingHole(holeR, thick * 1.1)
      hole.position.set(px, 0, pz)
      g.add(hole)
    }
  }

  return g
}

// ── Baseplate ───────────────────────────────────────────────────────────────

function generateBaseplate(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const thick = h
  const chamfer = Math.min(Math.min(w, d) * 0.04, thick * 0.4)
  const mat = catMetal('anodized_aluminum', 0.2)

  const plate = new THREE.Mesh(nurbsFilletBox(w, thick, d, chamfer, 16), mat)
  g.add(plate)

  // Mounting hole grid pattern
  const holeR = Math.min(w, d) * 0.035
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      const hole = mountingHole(holeR, thick * 1.1)
      hole.position.set(sx * w * 0.38, 0, sy * d * 0.38)
      g.add(hole)
    }
  }
  const centerHole = mountingHole(holeR * 1.2, thick * 1.1)
  g.add(centerHole)

  // Label sits on the top face (+Y) without changing the plate footprint.
  const label = labelRecess(w * 0.4, chamfer * 0.3, d * 0.15)
  label.position.set(0, thick * 0.51, -d * 0.25)
  g.add(label)

  return g
}

// ── Carbon Fiber Round Tube ─────────────────────────────────────────────────

function generateCFTubeRound(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: length } = dims
  const r = w / 2
  const chamfer = r * 0.05

  // Dark matte plastic material for carbon fiber look
  const tube = new THREE.Mesh(
    nurbsCylinder(r, length, chamfer, 32),
    getMaterial('matte_plastic', 0x1a1a1a),
  )
  tube.rotation.x = Math.PI / 2
  g.add(tube)

  // Inner bore (visible at ends)
  const boreR = r * 0.75
  const bore = new THREE.Mesh(
    new THREE.CylinderGeometry(boreR, boreR, length * 1.01, 24),
    getMaterial('dark_chrome'),
  )
  bore.rotation.x = Math.PI / 2
  g.add(bore)

  return g
}

// ── Carbon Fiber Square Tube ────────────────────────────────────────────────

function generateCFTubeSquare(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const chamfer = Math.min(w, h) * 0.06

  // Dark matte plastic material for carbon fiber look
  const tube = new THREE.Mesh(
    nurbsFilletBox(w, h, length, chamfer, 16),
    getMaterial('matte_plastic', 0x1a1a1a),
  )
  g.add(tube)

  // Inner bore representation (dark inset on each end)
  const inset = Math.min(w, h) * 0.15
  const boreMat = getMaterial('dark_chrome')
  for (const sz of [-1, 1]) {
    const bore = new THREE.Mesh(
      new THREE.BoxGeometry(w - inset * 2, h - inset * 2, length * 0.02),
      boreMat,
    )
    bore.position.z = sz * length * 0.505
    g.add(bore)
  }

  return g
}

// ── Flat / Round Bar / Threaded Rod ─────────────────────────────────────────

function generateBar(id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const chamfer = Math.min(w, h) * 0.06

  if (id.includes('round_bar') || id.includes('threaded_rod')) {
    const r = Math.min(w, h) / 2
    const mat = id.includes('threaded') ? getMaterial('brushed_steel') : baseMat(id)
    const bar = new THREE.Mesh(nurbsCylinder(r, length, chamfer, 24), mat)
    bar.rotation.x = Math.PI / 2
    g.add(bar)

    // Threaded rod: add knurl-style grooves
    if (id.includes('threaded')) {
      const ring = knurledRing(r * 1.02, length * 0.95, Math.round(length / (r * 0.3)))
      ring.rotation.x = Math.PI / 2
      g.add(ring)
    }
  } else {
    // Flat bar
    const mat = baseMat(id)
    const bar = new THREE.Mesh(nurbsFilletBox(w, h, length, chamfer, 16), mat)
    g.add(bar)
  }

  return g
}

// ── Sheet Metal ─────────────────────────────────────────────────────────────

function generateSheetMetal(id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const thick = Math.max(h, Math.min(w, d) * 0.02)
  const chamfer = thick * 0.3

  const sheet = new THREE.Mesh(
    nurbsFilletBox(w, thick, d, chamfer, 12),
    baseMat(id),
  )
  g.add(sheet)

  return g
}

// ── Hex Standoff ────────────────────────────────────────────────────────────

function generateHexStandoff(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: length } = dims
  const r = w / 2
  const chamfer = r * 0.08

  // Hex body (6-segment cylinder for hex shape)
  const hex = new THREE.Mesh(
    nurbsCylinder(r, length, chamfer, 6),
    catMetal(),
  )
  g.add(hex)

  // Threaded stubs on each end
  const stubR = r * 0.45
  const stubH = length * 0.15
  const stubMat = getMaterial('brushed_steel')
  for (const sy of [-1, 1]) {
    const stub = new THREE.Mesh(
      nurbsCylinder(stubR, stubH, stubR * 0.1, 16),
      stubMat,
    )
    stub.position.y = sy * (length + stubH) / 2
    g.add(stub)
  }

  return g
}

// ── Gusset (Triangular Plate) ───────────────────────────────────────────────

function generateGusset(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const thick = h

  const shape = new THREE.Shape()
  shape.moveTo(-w / 2, -d / 2)
  shape.lineTo(w / 2, -d / 2)
  shape.lineTo(-w / 2, d / 2)
  shape.closePath()

  const geom = new THREE.ExtrudeGeometry(shape, {
    depth: thick,
    bevelEnabled: true,
    bevelThickness: thick * 0.1,
    bevelSize: thick * 0.1,
    bevelSegments: 2,
  })
  geom.translate(0, 0, -thick / 2)
  geom.rotateX(Math.PI / 2)

  const mesh = new THREE.Mesh(geom, catMetal())
  g.add(mesh)

  // Mounting holes at each corner
  const holeR = Math.min(w, d) * 0.035
  const hole1 = mountingHole(holeR, thick * 1.1)
  hole1.position.set(-w * 0.35, 0, -d * 0.35)
  hole1.rotation.x = Math.PI / 2
  g.add(hole1)

  const hole2 = mountingHole(holeR, thick * 1.1)
  hole2.position.set(w * 0.35, 0, -d * 0.35)
  hole2.rotation.x = Math.PI / 2
  g.add(hole2)

  const hole3 = mountingHole(holeR, thick * 1.1)
  hole3.position.set(-w * 0.35, 0, d * 0.2)
  hole3.rotation.x = Math.PI / 2
  g.add(hole3)

  return g
}

// ── Corner Cube ─────────────────────────────────────────────────────────────

function generateCornerCube(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const s = Math.min(w, h, d)
  const chamfer = s * 0.05

  const cube = new THREE.Mesh(nurbsFilletBox(s, s, s, chamfer, 16), catMetal())
  g.add(cube)

  // Mounting holes on 3 faces
  const holeR = s * 0.06
  const hTop = mountingHole(holeR, s * 0.4)
  hTop.position.y = s * 0.35
  g.add(hTop)

  const hFront = mountingHole(holeR, s * 0.4)
  hFront.rotation.x = Math.PI / 2
  hFront.position.z = s * 0.35
  g.add(hFront)

  const hRight = mountingHole(holeR, s * 0.4)
  hRight.rotation.z = Math.PI / 2
  hRight.position.x = s * 0.35
  g.add(hRight)

  return g
}

// ── Cross Plate ─────────────────────────────────────────────────────────────

function generateCrossPlate(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const thick = h
  const chamfer = thick * 0.3
  const armW = Math.min(w, d) * 0.35
  const mat = catMetal()

  const hArm = new THREE.Mesh(nurbsFilletBox(w, thick, armW, chamfer, 16), mat)
  g.add(hArm)

  const vArm = new THREE.Mesh(nurbsFilletBox(armW, thick, d, chamfer, 16), mat)
  g.add(vArm)

  const holeR = armW * 0.12
  for (const [px, pz] of [[w * 0.4, 0], [-w * 0.4, 0], [0, d * 0.4], [0, -d * 0.4]] as [number, number][]) {
    const hole = mountingHole(holeR, thick * 1.1)
    hole.position.set(px, 0, pz)
    g.add(hole)
  }

  return g
}

// ── Pillow Block ────────────────────────────────────────────────────────────

function generatePillowBlock(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const chamfer = Math.min(w, d) * 0.04
  const mat = catMetal()

  const baseH = h * 0.35
  const base = new THREE.Mesh(nurbsFilletBox(w, baseH, d, chamfer, 16), mat)
  base.position.y = -h / 2 + baseH / 2
  g.add(base)

  const housingR = Math.min(w, d) * 0.35
  const housingH = d * 0.8
  const housing = new THREE.Mesh(
    nurbsCylinder(housingR, housingH, chamfer, 32),
    mat,
  )
  housing.rotation.x = Math.PI / 2
  housing.position.y = h * 0.05
  g.add(housing)

  const boreR = housingR * 0.5
  const bore = new THREE.Mesh(
    new THREE.CylinderGeometry(boreR, boreR, housingH * 1.1, 16),
    getMaterial('dark_chrome'),
  )
  bore.rotation.x = Math.PI / 2
  bore.position.y = h * 0.05
  g.add(bore)

  const holeR = Math.min(w, d) * 0.035
  for (const sx of [-1, 1]) {
    const hole = mountingHole(holeR, baseH * 1.1)
    hole.position.set(sx * w * 0.35, -h / 2 + baseH / 2, 0)
    g.add(hole)
  }

  return g
}

// ── Shaft Collar ────────────────────────────────────────────────────────────

function generateShaftCollar(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h } = dims
  const r = w / 2
  const chamfer = r * 0.06

  const ring = new THREE.Mesh(
    nurbsCylinder(r, h, chamfer, 32),
    catMetal(),
  )
  g.add(ring)

  const boreR = r * 0.55
  const bore = new THREE.Mesh(
    new THREE.CylinderGeometry(boreR, boreR, h * 1.01, 24),
    getMaterial('dark_chrome'),
  )
  g.add(bore)

  const split = new THREE.Mesh(
    new THREE.BoxGeometry(r * 0.03, h * 1.02, r * 1.1),
    getMaterial('dark_chrome'),
  )
  split.position.x = r * 0.8
  g.add(split)

  const screw = screwHead(r * 0.12, h * 0.25)
  screw.position.set(r * 0.9, h * 0.3, 0)
  g.add(screw)

  return g
}

// ── Linear Rail ─────────────────────────────────────────────────────────────

function generateLinearRail(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const chamfer = Math.min(w, h) * 0.03
  const mat = getMaterial('brushed_steel')

  // Rail base
  const railH = h * 0.35
  const rail = new THREE.Mesh(nurbsFilletBox(w, railH, length, chamfer, 16), mat)
  rail.position.y = -h / 2 + railH / 2
  g.add(rail)

  // Raised guide rail on top
  const guideW = w * 0.4
  const guideH = h * 0.2
  const guide = new THREE.Mesh(
    nurbsFilletBox(guideW, guideH, length * 0.98, chamfer * 0.5, 12),
    mat,
  )
  guide.position.y = -h / 2 + railH + guideH / 2
  g.add(guide)

  // Mounting holes along rail
  const holeR = w * 0.06
  const holeCount = Math.max(2, Math.round(length / (w * 1.5)))
  for (let i = 0; i < holeCount; i++) {
    const pz = -length * 0.4 + (i / (holeCount - 1)) * length * 0.8
    const hole = mountingHole(holeR, railH * 1.1)
    hole.rotation.x = Math.PI / 2
    hole.position.set(0, -h / 2 + railH / 2, pz)
    g.add(hole)
  }

  // Carriage block
  const carrW = w * 1.4
  const carrH = h * 0.45
  const carrD = length * 0.2
  const carriage = new THREE.Mesh(
    nurbsFilletBox(carrW, carrH, carrD, chamfer, 16),
    catMetal(),
  )
  carriage.position.y = -h / 2 + railH + guideH + carrH / 2
  g.add(carriage)

  // Carriage mounting holes (4)
  for (const sx of [-1, 1]) {
    for (const sz of [-1, 1]) {
      const hole = mountingHole(holeR, carrH * 1.1)
      hole.position.set(
        sx * carrW * 0.32,
        -h / 2 + railH + guideH + carrH / 2,
        sz * carrD * 0.3,
      )
      g.add(hole)
    }
  }

  return g
}

// ── Linear Rail Carriage (standalone) ───────────────────────────────────────

function generateLinearRailCarriage(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: d, z: h } = dims
  const chamfer = Math.min(w, h, d) * 0.04

  const body = new THREE.Mesh(nurbsFilletBox(w, h, d, chamfer, 16), catMetal())
  g.add(body)

  const holeR = Math.min(w, d) * 0.05
  for (const sx of [-1, 1]) {
    for (const sz of [-1, 1]) {
      const hole = mountingHole(holeR, h * 1.1)
      hole.position.set(sx * w * 0.3, 0, sz * d * 0.3)
      hole.rotation.x = Math.PI / 2
      g.add(hole)
    }
  }

  const groove = new THREE.Mesh(
    new THREE.BoxGeometry(w * 0.35, h * 0.3, d * 0.95),
    getMaterial('dark_chrome'),
  )
  groove.position.y = -h * 0.4
  g.add(groove)

  return g
}

// ── DIN Rail ────────────────────────────────────────────────────────────────

function generateDINRail(_id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const chamfer = Math.min(w, h) * 0.02
  const mat = getMaterial('brushed_steel')

  const bodyH = h * 0.6
  const body = new THREE.Mesh(nurbsFilletBox(w, bodyH, length, chamfer, 16), mat)
  g.add(body)

  const lipW = w * 1.3
  const lipH = h * 0.15
  const lip = new THREE.Mesh(nurbsFilletBox(lipW, lipH, length, chamfer * 0.5, 12), mat)
  lip.position.y = -(bodyH + lipH) / 2
  g.add(lip)

  for (const sx of [-1, 1]) {
    const returnEdge = new THREE.Mesh(
      nurbsFilletBox(w * 0.08, lipH * 1.5, length * 0.98, chamfer * 0.3, 12),
      mat,
    )
    returnEdge.position.set(sx * lipW * 0.45, -(bodyH / 2 + lipH + lipH * 0.25), 0)
    g.add(returnEdge)
  }

  return g
}

// ── Dispatcher ──────────────────────────────────────────────────────────────

function generateSlimLimbLink(id: string, dims: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const w = Math.max(dims.x, 0.010)
  const t = Math.max(dims.y, 0.004)
  const length = Math.max(dims.z, w * 2.2)
  const mat = baseMat(id)
  const dark = getMaterial('dark_chrome')
  const lugRadius = w * 0.5
  const lugY = Math.max(0, length / 2 - lugRadius)

  const spine = new THREE.Mesh(
    nurbsFilletBox(w * 0.46, length, t, Math.min(w, t) * 0.18, 14),
    mat,
  )
  g.add(spine)

  for (const sy of [-1, 1]) {
    const lug = new THREE.Mesh(
      nurbsCylinder(lugRadius, t, Math.min(w, t) * 0.08, 32),
      mat,
    )
    lug.rotation.x = Math.PI / 2
    lug.position.y = sy * lugY
    g.add(lug)

    const bore = new THREE.Mesh(
      nurbsCylinder(Math.max(w * 0.15, 0.0018), t * 1.08, 0, 24),
      dark,
    )
    bore.rotation.x = Math.PI / 2
    bore.position.y = sy * lugY
    g.add(bore)
  }

  return g
}

export function generateRichStructural(id: string, dims: ComponentVisualDims, color?: [number, number, number]): THREE.Group {
  CAT_COLOR = color ?? DEFAULT_COLOR
  if (id.includes('limb_link_slim'))
    return generateSlimLimbLink(id, dims)
  if (id.includes('extrusion') || id.includes('2020') || id.includes('4040'))
    return generateExtrusion(id, dims)
  if (id.includes('ibeam') || id.includes('i_beam'))
    return generateIBeam(id, dims)
  if (id.includes('cchannel') || id.includes('c_channel'))
    return generateCChannel(id, dims)
  if (id.includes('angle_stock') || id.includes('angle_al') || id.includes('angle_steel'))
    return generateAngle(id, dims)
  if (id.includes('bracket_l'))
    return generateLBracket(id, dims)
  if (id.includes('bracket_u'))
    return generateUBracket(id, dims)
  if (id.includes('bracket_t') || id.includes('t_bracket'))
    return generateTBracket(id, dims)
  if (id.includes('joint_plate'))
    return generateJointPlate(id, dims)
  if (id.includes('baseplate'))
    return generateBaseplate(id, dims)
  if (id.includes('cf_tube_round'))
    return generateCFTubeRound(id, dims)
  if (id.includes('cf_tube_square'))
    return generateCFTubeSquare(id, dims)
  if (id.includes('flat_bar') || id.includes('round_bar') || id.includes('threaded_rod'))
    return generateBar(id, dims)
  if (id.includes('sheet_metal') || id.startsWith('structural_sheet_'))
    return generateSheetMetal(id, dims)
  if (id.includes('hex_standoff') || id.includes('standoff'))
    return generateHexStandoff(id, dims)
  if (id.includes('gusset'))
    return generateGusset(id, dims)
  if (id.includes('corner_cube'))
    return generateCornerCube(id, dims)
  if (id.includes('cross_plate'))
    return generateCrossPlate(id, dims)
  if (id.includes('pillow_block'))
    return generatePillowBlock(id, dims)
  if (id.includes('shaft_collar'))
    return generateShaftCollar(id, dims)
  if (id.includes('linear_rail_carriage'))
    return generateLinearRailCarriage(id, dims)
  if (id.includes('linear_rail'))
    return generateLinearRail(id, dims)
  if (id.includes('din_rail'))
    return generateDINRail(id, dims)

  // Fallback: generic structural box
  const g = new THREE.Group()
  const fillet = Math.min(dims.x, dims.y, dims.z) * 0.05
  g.add(new THREE.Mesh(nurbsFilletBox(dims.x, dims.y, dims.z, fillet, 16), catMetal()))
  return g
}

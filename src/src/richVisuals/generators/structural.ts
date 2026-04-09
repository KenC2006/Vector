/**
 * Rich visual generators for structural components.
 * Extrusions, beams, channels, brackets, plates, bars, rails.
 */
import * as THREE from 'three'
import type { GeneratorDims } from './index'
import { getMaterial, getTintedMaterial } from '../materials'
import {
  chamferedBox, chamferedCylinder, mountingHole, screwHead,
  labelRecess, knurledRing,
} from '../primitives'

const CAT_COLOR: [number, number, number] = [0.66, 0.70, 0.72]  // silver-grey

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

function generateExtrusion(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const is4040 = id.includes('4040')
  const chamfer = Math.min(w, d) * 0.04

  // Main body
  const body = new THREE.Mesh(chamferedBox(w, d, h, chamfer), catMetal('anodized_aluminum', 0.2))
  g.add(body)

  // Channel grooves on each face (dark inset boxes)
  const grooveDepth = Math.min(w, d) * 0.12
  const grooveWidth = Math.min(w, d) * 0.3
  const grooveMat = getMaterial('dark_chrome')

  // Front/back grooves (Z faces)
  for (const sz of [-1, 1]) {
    const groove = new THREE.Mesh(
      new THREE.BoxGeometry(grooveWidth, grooveDepth, h * 0.98),
      grooveMat,
    )
    groove.position.set(0, sz * d / 2, 0)
    g.add(groove)
  }

  // Left/right grooves (X faces)
  for (const sx of [-1, 1]) {
    const groove = new THREE.Mesh(
      new THREE.BoxGeometry(grooveDepth, grooveWidth, h * 0.98),
      grooveMat,
    )
    groove.position.set(sx * w / 2, 0, 0)
    g.add(groove)
  }

  // If 4040, add extra grooves (2 per face)
  if (is4040) {
    const offset = Math.min(w, d) * 0.22
    for (const sz of [-1, 1]) {
      for (const ox of [-1, 1]) {
        const groove = new THREE.Mesh(
          new THREE.BoxGeometry(grooveWidth * 0.45, grooveDepth, h * 0.98),
          grooveMat,
        )
        groove.position.set(ox * offset, sz * d / 2, 0)
        g.add(groove)
      }
    }
    for (const sx of [-1, 1]) {
      for (const oy of [-1, 1]) {
        const groove = new THREE.Mesh(
          new THREE.BoxGeometry(grooveDepth, grooveWidth * 0.45, h * 0.98),
          grooveMat,
        )
        groove.position.set(sx * w / 2, oy * offset, 0)
        g.add(groove)
      }
    }
  }

  // Center bore hole (dark cylinder running through length)
  const boreR = Math.min(w, d) * 0.12
  const bore = new THREE.Mesh(
    new THREE.CylinderGeometry(boreR, boreR, h * 1.01, 16),
    grooveMat,
  )
  bore.rotation.x = Math.PI / 2
  g.add(bore)

  return g
}

// ── I-Beam ──────────────────────────────────────────────────────────────────

function generateIBeam(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const chamfer = Math.min(w, h) * 0.03
  const mat = baseMat(id)

  const flangeH = h * 0.12
  const webThick = w * 0.2

  // Web (center vertical plate)
  const web = new THREE.Mesh(chamferedBox(webThick, length, h - flangeH * 2, chamfer * 0.5), mat)
  g.add(web)

  // Top flange
  const topFlange = new THREE.Mesh(chamferedBox(w, length, flangeH, chamfer), mat)
  topFlange.position.y = (h - flangeH) / 2
  g.add(topFlange)

  // Bottom flange
  const botFlange = new THREE.Mesh(chamferedBox(w, length, flangeH, chamfer), mat)
  botFlange.position.y = -(h - flangeH) / 2
  g.add(botFlange)

  return g
}

// ── C-Channel ───────────────────────────────────────────────────────────────

function generateCChannel(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const chamfer = Math.min(w, h) * 0.03
  const mat = baseMat(id)

  const flangeH = h * 0.12
  const webThick = w * 0.15

  // Back web
  const web = new THREE.Mesh(chamferedBox(webThick, length, h, chamfer * 0.5), mat)
  web.position.x = -w / 2 + webThick / 2
  g.add(web)

  // Top flange
  const topFlange = new THREE.Mesh(chamferedBox(w, length, flangeH, chamfer), mat)
  topFlange.position.y = (h - flangeH) / 2
  g.add(topFlange)

  // Bottom flange
  const botFlange = new THREE.Mesh(chamferedBox(w, length, flangeH, chamfer), mat)
  botFlange.position.y = -(h - flangeH) / 2
  g.add(botFlange)

  return g
}

// ── Angle Stock (L-profile) ─────────────────────────────────────────────────

function generateAngle(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const chamfer = Math.min(w, h) * 0.03
  const mat = baseMat(id)

  const thick = Math.min(w, h) * 0.15

  // Vertical leg
  const vLeg = new THREE.Mesh(chamferedBox(thick, length, h, chamfer), mat)
  vLeg.position.x = -w / 2 + thick / 2
  g.add(vLeg)

  // Horizontal leg
  const hLeg = new THREE.Mesh(chamferedBox(w, length, thick, chamfer), mat)
  hLeg.position.y = -h / 2 + thick / 2
  g.add(hLeg)

  return g
}

// ── L-Bracket ───────────────────────────────────────────────────────────────

function generateLBracket(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, h, d) * 0.04
  const mat = catMetal()
  const thick = Math.min(w, h, d) * 0.18

  // Vertical face
  const vFace = new THREE.Mesh(chamferedBox(w, thick, h * 0.6, chamfer), mat)
  vFace.position.set(0, -d / 2 + thick / 2, h * 0.15)
  g.add(vFace)

  // Horizontal face
  const hFace = new THREE.Mesh(chamferedBox(w, d, thick, chamfer), mat)
  hFace.position.y = -h / 2 + thick / 2
  g.add(hFace)

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
    hole.position.set(sx * w * 0.28, -d / 2 + thick / 2, h * 0.25)
    g.add(hole)
  }

  return g
}

// ── U-Bracket ───────────────────────────────────────────────────────────────

function generateUBracket(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, h, d) * 0.04
  const mat = catMetal()
  const thick = Math.min(w, h, d) * 0.15

  // Base plate
  const base = new THREE.Mesh(chamferedBox(w, d, thick, chamfer), mat)
  base.position.y = -h / 2 + thick / 2
  g.add(base)

  // Two side walls
  for (const sx of [-1, 1]) {
    const wall = new THREE.Mesh(chamferedBox(thick, d * 0.8, h - thick, chamfer), mat)
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

function generateTBracket(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, h, d) * 0.04
  const mat = catMetal()
  const thick = Math.min(w, h, d) * 0.15

  // Horizontal bar
  const hBar = new THREE.Mesh(chamferedBox(w, d, thick, chamfer), mat)
  hBar.position.y = -h / 2 + thick / 2
  g.add(hBar)

  // Vertical stem
  const stem = new THREE.Mesh(chamferedBox(thick * 1.5, d * 0.8, h - thick, chamfer), mat)
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

function generateJointPlate(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const thick = Math.max(d, Math.min(w, h) * 0.08)
  const chamfer = Math.min(w, h) * 0.03
  const mat = catMetal()

  const plate = new THREE.Mesh(chamferedBox(w, h, thick, chamfer), mat)
  g.add(plate)

  // Grid of mounting holes
  const holeR = Math.min(w, h) * 0.03
  const cols = Math.max(2, Math.round(w / (Math.min(w, h) * 0.3)))
  const rows = Math.max(2, Math.round(h / (Math.min(w, h) * 0.3)))
  for (let ix = 0; ix < cols; ix++) {
    for (let iy = 0; iy < rows; iy++) {
      const px = -w * 0.4 + (ix / (cols - 1)) * w * 0.8
      const py = -h * 0.4 + (iy / (rows - 1)) * h * 0.8
      const hole = mountingHole(holeR, thick * 1.1)
      hole.position.set(px, py, 0)
      hole.rotation.x = Math.PI / 2
      g.add(hole)
    }
  }

  return g
}

// ── Baseplate ───────────────────────────────────────────────────────────────

function generateBaseplate(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const thick = Math.max(d, Math.min(w, h) * 0.12)
  const chamfer = Math.min(w, h) * 0.04
  const mat = catMetal('anodized_aluminum', 0.2)

  const plate = new THREE.Mesh(chamferedBox(w, h, thick, chamfer), mat)
  g.add(plate)

  // Mounting hole pattern (4 corners + center)
  const holeR = Math.min(w, h) * 0.035
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      const hole = mountingHole(holeR, thick * 1.1)
      hole.position.set(sx * w * 0.38, sy * h * 0.38, 0)
      hole.rotation.x = Math.PI / 2
      g.add(hole)
    }
  }
  const centerHole = mountingHole(holeR * 1.2, thick * 1.1)
  centerHole.rotation.x = Math.PI / 2
  g.add(centerHole)

  // Label
  const label = labelRecess(w * 0.4, h * 0.15, chamfer * 0.3)
  label.position.set(0, -h * 0.25, thick * 0.51)
  g.add(label)

  return g
}

// ── Carbon Fiber Round Tube ─────────────────────────────────────────────────

function generateCFTubeRound(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h } = dims
  const r = w / 2
  const chamfer = r * 0.05

  const tube = new THREE.Mesh(
    chamferedCylinder(r, h, chamfer, 32),
    getMaterial('matte_plastic', 0x1a1a1a),
  )
  g.add(tube)

  // Inner bore (slightly smaller dark cylinder visible at ends)
  const boreR = r * 0.75
  const bore = new THREE.Mesh(
    new THREE.CylinderGeometry(boreR, boreR, h * 1.01, 24),
    getMaterial('dark_chrome'),
  )
  g.add(bore)

  return g
}

// ── Carbon Fiber Square Tube ────────────────────────────────────────────────

function generateCFTubeSquare(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const chamfer = Math.min(w, h) * 0.06

  const tube = new THREE.Mesh(
    chamferedBox(w, h, length, chamfer),
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

function generateBar(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const chamfer = Math.min(w, h) * 0.06

  if (id.includes('round_bar') || id.includes('threaded_rod')) {
    const r = Math.min(w, h) / 2
    const mat = id.includes('threaded') ? getMaterial('brushed_steel') : baseMat(id)
    const bar = new THREE.Mesh(chamferedCylinder(r, length, chamfer, 24), mat)
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
    const bar = new THREE.Mesh(chamferedBox(w, h, length, chamfer), mat)
    g.add(bar)
  }

  return g
}

// ── Sheet Metal ─────────────────────────────────────────────────────────────

function generateSheetMetal(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const thick = Math.max(d, Math.min(w, h) * 0.02)
  const chamfer = thick * 0.3

  const sheet = new THREE.Mesh(
    chamferedBox(w, h, thick, chamfer),
    baseMat(id),
  )
  g.add(sheet)

  return g
}

// ── Hex Standoff ────────────────────────────────────────────────────────────

function generateHexStandoff(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const r = w / 2
  const chamfer = r * 0.08

  // Hex body (6-segment cylinder)
  const hex = new THREE.Mesh(
    chamferedCylinder(r, length, chamfer, 6),
    catMetal(),
  )
  g.add(hex)

  // Threaded stubs on each end
  const stubR = r * 0.45
  const stubH = length * 0.15
  const stubMat = getMaterial('brushed_steel')
  for (const sy of [-1, 1]) {
    const stub = new THREE.Mesh(
      chamferedCylinder(stubR, stubH, stubR * 0.1, 16),
      stubMat,
    )
    stub.position.y = sy * (length + stubH) / 2
    g.add(stub)
  }

  return g
}

// ── Gusset (Triangular Plate) ───────────────────────────────────────────────

function generateGusset(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const thick = Math.max(d, Math.min(w, h) * 0.08)

  const shape = new THREE.Shape()
  shape.moveTo(-w / 2, -h / 2)
  shape.lineTo(w / 2, -h / 2)
  shape.lineTo(-w / 2, h / 2)
  shape.closePath()

  const geom = new THREE.ExtrudeGeometry(shape, {
    depth: thick,
    bevelEnabled: true,
    bevelThickness: thick * 0.1,
    bevelSize: thick * 0.1,
    bevelSegments: 2,
  })
  geom.translate(0, 0, -thick / 2)

  const mesh = new THREE.Mesh(geom, catMetal())
  g.add(mesh)

  // Mounting holes at each corner
  const holeR = Math.min(w, h) * 0.035
  const hole1 = mountingHole(holeR, thick * 1.1)
  hole1.position.set(-w * 0.35, -h * 0.35, 0)
  hole1.rotation.x = Math.PI / 2
  g.add(hole1)

  const hole2 = mountingHole(holeR, thick * 1.1)
  hole2.position.set(w * 0.35, -h * 0.35, 0)
  hole2.rotation.x = Math.PI / 2
  g.add(hole2)

  const hole3 = mountingHole(holeR, thick * 1.1)
  hole3.position.set(-w * 0.35, h * 0.2, 0)
  hole3.rotation.x = Math.PI / 2
  g.add(hole3)

  return g
}

// ── Corner Cube ─────────────────────────────────────────────────────────────

function generateCornerCube(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const s = Math.min(w, h, d)
  const chamfer = s * 0.05

  const cube = new THREE.Mesh(chamferedBox(s, s, s, chamfer), catMetal())
  g.add(cube)

  // Mounting holes on 3 faces
  const holeR = s * 0.06
  // Top face
  const hTop = mountingHole(holeR, s * 0.4)
  hTop.position.y = s * 0.35
  g.add(hTop)

  // Front face
  const hFront = mountingHole(holeR, s * 0.4)
  hFront.rotation.x = Math.PI / 2
  hFront.position.z = s * 0.35
  g.add(hFront)

  // Right face
  const hRight = mountingHole(holeR, s * 0.4)
  hRight.rotation.z = Math.PI / 2
  hRight.position.x = s * 0.35
  g.add(hRight)

  return g
}

// ── Cross Plate ─────────────────────────────────────────────────────────────

function generateCrossPlate(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const thick = Math.max(d, Math.min(w, h) * 0.08)
  const chamfer = thick * 0.3
  const armW = Math.min(w, h) * 0.35
  const mat = catMetal()

  // Horizontal arm
  const hArm = new THREE.Mesh(chamferedBox(w, armW, thick, chamfer), mat)
  g.add(hArm)

  // Vertical arm
  const vArm = new THREE.Mesh(chamferedBox(armW, h, thick, chamfer), mat)
  g.add(vArm)

  // Mounting holes at 4 ends
  const holeR = armW * 0.12
  for (const [px, py] of [[w * 0.4, 0], [-w * 0.4, 0], [0, h * 0.4], [0, -h * 0.4]] as [number, number][]) {
    const hole = mountingHole(holeR, thick * 1.1)
    hole.rotation.x = Math.PI / 2
    hole.position.set(px, py, 0)
    g.add(hole)
  }

  return g
}

// ── Pillow Block ────────────────────────────────────────────────────────────

function generatePillowBlock(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, d) * 0.04
  const mat = catMetal()

  // Base plate
  const baseH = h * 0.35
  const base = new THREE.Mesh(chamferedBox(w, d, baseH, chamfer), mat)
  base.position.y = -h / 2 + baseH / 2
  g.add(base)

  // Cylindrical bearing housing
  const housingR = Math.min(w, d) * 0.35
  const housingH = d * 0.8
  const housing = new THREE.Mesh(
    chamferedCylinder(housingR, housingH, chamfer, 32),
    mat,
  )
  housing.rotation.x = Math.PI / 2
  housing.position.y = h * 0.05
  g.add(housing)

  // Bore hole (dark inset)
  const boreR = housingR * 0.5
  const bore = new THREE.Mesh(
    new THREE.CylinderGeometry(boreR, boreR, housingH * 1.1, 16),
    getMaterial('dark_chrome'),
  )
  bore.rotation.x = Math.PI / 2
  bore.position.y = h * 0.05
  g.add(bore)

  // Mounting holes on base
  const holeR = Math.min(w, d) * 0.035
  for (const sx of [-1, 1]) {
    const hole = mountingHole(holeR, baseH * 1.1)
    hole.position.set(sx * w * 0.35, -h / 2 + baseH / 2, 0)
    g.add(hole)
  }

  return g
}

// ── Shaft Collar ────────────────────────────────────────────────────────────

function generateShaftCollar(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, z: h } = dims
  const r = w / 2
  const chamfer = r * 0.06

  // Main ring
  const ring = new THREE.Mesh(
    chamferedCylinder(r, h, chamfer, 32),
    catMetal(),
  )
  g.add(ring)

  // Bore
  const boreR = r * 0.55
  const bore = new THREE.Mesh(
    new THREE.CylinderGeometry(boreR, boreR, h * 1.01, 24),
    getMaterial('dark_chrome'),
  )
  g.add(bore)

  // Clamping split (thin dark line)
  const split = new THREE.Mesh(
    new THREE.BoxGeometry(r * 0.03, h * 1.02, r * 1.1),
    getMaterial('dark_chrome'),
  )
  split.position.x = r * 0.8
  g.add(split)

  // Clamping screw
  const screw = screwHead(r * 0.12, h * 0.25)
  screw.position.set(r * 0.9, h * 0.3, 0)
  g.add(screw)

  return g
}

// ── Linear Rail ─────────────────────────────────────────────────────────────

function generateLinearRail(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const chamfer = Math.min(w, h) * 0.03
  const mat = getMaterial('brushed_steel')

  // Rail base (long thin box)
  const railH = h * 0.35
  const rail = new THREE.Mesh(chamferedBox(w, length, railH, chamfer), mat)
  rail.position.y = -h / 2 + railH / 2
  g.add(rail)

  // Raised guide rail on top
  const guideW = w * 0.4
  const guideH = h * 0.2
  const guide = new THREE.Mesh(
    chamferedBox(guideW, length * 0.98, guideH, chamfer * 0.5),
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
    chamferedBox(carrW, carrD, carrH, chamfer),
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

function generateLinearRailCarriage(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: d } = dims
  const chamfer = Math.min(w, h, d) * 0.04

  const body = new THREE.Mesh(chamferedBox(w, d, h, chamfer), catMetal())
  g.add(body)

  // 4 mounting holes
  const holeR = Math.min(w, d) * 0.05
  for (const sx of [-1, 1]) {
    for (const sz of [-1, 1]) {
      const hole = mountingHole(holeR, h * 1.1)
      hole.position.set(sx * w * 0.3, sz * d * 0.3, 0)
      hole.rotation.x = Math.PI / 2
      g.add(hole)
    }
  }

  // Guide groove on bottom (dark inset)
  const groove = new THREE.Mesh(
    new THREE.BoxGeometry(w * 0.35, d * 0.95, h * 0.3),
    getMaterial('dark_chrome'),
  )
  groove.position.y = 0
  groove.position.z = -h * 0.4
  g.add(groove)

  return g
}

// ── DIN Rail ────────────────────────────────────────────────────────────────

function generateDINRail(id: string, dims: GeneratorDims): THREE.Group {
  const g = new THREE.Group()
  const { x: w, y: h, z: length } = dims
  const chamfer = Math.min(w, h) * 0.02
  const mat = getMaterial('brushed_steel')

  // Main hat body
  const bodyH = h * 0.6
  const body = new THREE.Mesh(chamferedBox(w, length, bodyH, chamfer), mat)
  g.add(body)

  // Bottom lip edges (wider flanges at bottom)
  const lipW = w * 1.3
  const lipH = h * 0.15
  const lip = new THREE.Mesh(chamferedBox(lipW, length, lipH, chamfer * 0.5), mat)
  lip.position.y = -(bodyH + lipH) / 2
  g.add(lip)

  // Lip return edges (bent inward at bottom)
  for (const sx of [-1, 1]) {
    const returnEdge = new THREE.Mesh(
      chamferedBox(w * 0.08, length * 0.98, lipH * 1.5, chamfer * 0.3),
      mat,
    )
    returnEdge.position.set(sx * lipW * 0.45, -(bodyH / 2 + lipH + lipH * 0.25), 0)
    g.add(returnEdge)
  }

  return g
}

// ── Dispatcher ──────────────────────────────────────────────────────────────

export function generateRichStructural(id: string, dims: GeneratorDims): THREE.Group {
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
  if (id.includes('sheet_metal'))
    return generateSheetMetal(id, dims)
  if (id.includes('hex_standoff'))
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
  const chamfer = Math.min(dims.x, dims.y, dims.z) * 0.04
  g.add(new THREE.Mesh(chamferedBox(dims.x, dims.y, dims.z, chamfer), catMetal()))
  return g
}

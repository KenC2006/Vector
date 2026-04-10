/**
 * Parametric geometry builders for Fusion-quality component visuals.
 *
 * Uses profile-based geometry (LatheGeometry, ExtrudeGeometry) instead of
 * composing primitive boxes/cylinders. Most mechanical parts are either:
 *   - A 2D profile revolved (motors, bearings, shafts, knobs)
 *   - A 2D cross-section extruded (brackets, extrusions, housings)
 *
 * All functions return THREE.BufferGeometry or THREE.Group.
 */
import * as THREE from 'three'
import { getMaterial } from './materials'

// ── Constants ────────────────────────────────────────────────────────────────

const SEGMENTS_HIGH = 64    // smooth curves
const SEGMENTS_MED = 32     // default
const SEGMENTS_LOW = 16     // small detail parts
const BEVEL_SEGS = 4        // bevel quality

// ── Shape Helpers ────────────────────────────────────────────────────────────

/** Rounded rectangle shape for extrusion — the foundation of most housings. */
export function roundedRectShape(w: number, h: number, r: number): THREE.Shape {
  const shape = new THREE.Shape()
  const hw = w / 2, hh = h / 2
  r = Math.min(r, hw * 0.49, hh * 0.49)
  if (r < 0.0001) {
    shape.moveTo(-hw, -hh)
    shape.lineTo(hw, -hh)
    shape.lineTo(hw, hh)
    shape.lineTo(-hw, hh)
    shape.closePath()
    return shape
  }
  shape.moveTo(-hw + r, -hh)
  shape.lineTo(hw - r, -hh)
  shape.quadraticCurveTo(hw, -hh, hw, -hh + r)
  shape.lineTo(hw, hh - r)
  shape.quadraticCurveTo(hw, hh, hw - r, hh)
  shape.lineTo(-hw + r, hh)
  shape.quadraticCurveTo(-hw, hh, -hw, hh - r)
  shape.lineTo(-hw, -hh + r)
  shape.quadraticCurveTo(-hw, -hh, -hw + r, -hh)
  return shape
}

/** L-shaped profile for brackets. */
export function lShape(legA: number, legB: number, thickness: number): THREE.Shape {
  const shape = new THREE.Shape()
  shape.moveTo(0, 0)
  shape.lineTo(legB, 0)
  shape.lineTo(legB, thickness)
  shape.lineTo(thickness, thickness)
  shape.lineTo(thickness, legA)
  shape.lineTo(0, legA)
  shape.closePath()
  return shape
}

/** U-channel profile. */
export function uShape(w: number, h: number, thickness: number): THREE.Shape {
  const shape = new THREE.Shape()
  shape.moveTo(0, 0)
  shape.lineTo(w, 0)
  shape.lineTo(w, h)
  shape.lineTo(w - thickness, h)
  shape.lineTo(w - thickness, thickness)
  shape.lineTo(thickness, thickness)
  shape.lineTo(thickness, h)
  shape.lineTo(0, h)
  shape.closePath()
  return shape
}

/** I-beam profile. */
export function iBeamShape(flangeW: number, totalH: number, webT: number, flangeT: number): THREE.Shape {
  const shape = new THREE.Shape()
  const hw = flangeW / 2
  const hh = totalH / 2
  // Bottom flange
  shape.moveTo(-hw, -hh)
  shape.lineTo(hw, -hh)
  shape.lineTo(hw, -hh + flangeT)
  shape.lineTo(webT / 2, -hh + flangeT)
  // Web
  shape.lineTo(webT / 2, hh - flangeT)
  // Top flange
  shape.lineTo(hw, hh - flangeT)
  shape.lineTo(hw, hh)
  shape.lineTo(-hw, hh)
  shape.lineTo(-hw, hh - flangeT)
  shape.lineTo(-webT / 2, hh - flangeT)
  // Web left
  shape.lineTo(-webT / 2, -hh + flangeT)
  shape.lineTo(-hw, -hh + flangeT)
  shape.closePath()
  return shape
}

/** C-channel profile. */
export function cChannelShape(flangeW: number, totalH: number, thickness: number): THREE.Shape {
  const shape = new THREE.Shape()
  shape.moveTo(0, 0)
  shape.lineTo(flangeW, 0)
  shape.lineTo(flangeW, thickness)
  shape.lineTo(thickness, thickness)
  shape.lineTo(thickness, totalH - thickness)
  shape.lineTo(flangeW, totalH - thickness)
  shape.lineTo(flangeW, totalH)
  shape.lineTo(0, totalH)
  shape.closePath()
  return shape
}

/** T-slot extrusion cross-section profile — the signature CAD shape. */
export function tSlotShape(size: number): THREE.Shape {
  const s = size
  const slot = s * 0.25       // slot opening width
  const slotD = s * 0.2       // slot depth
  const core = s * 0.35       // core size
  const wall = (s - core) / 2 - slotD

  const shape = new THREE.Shape()
  const hs = s / 2
  const hc = core / 2
  const hsl = slot / 2

  // Build one quadrant and mirror — bottom-right first
  // Bottom face: slot opening
  shape.moveTo(-hsl, -hs)
  shape.lineTo(-hsl, -hs + wall)
  shape.lineTo(-hc, -hs + wall)
  shape.lineTo(-hc, -hc)
  // Left face slot
  shape.lineTo(-hs + wall, -hc)
  shape.lineTo(-hs + wall, -hsl)
  shape.lineTo(-hs, -hsl)
  shape.lineTo(-hs, hsl)
  shape.lineTo(-hs + wall, hsl)
  shape.lineTo(-hs + wall, hc)
  // Top
  shape.lineTo(-hc, hc)
  shape.lineTo(-hc, hs - wall)
  shape.lineTo(-hsl, hs - wall)
  shape.lineTo(-hsl, hs)
  shape.lineTo(hsl, hs)
  shape.lineTo(hsl, hs - wall)
  shape.lineTo(hc, hs - wall)
  shape.lineTo(hc, hc)
  // Right
  shape.lineTo(hs - wall, hc)
  shape.lineTo(hs - wall, hsl)
  shape.lineTo(hs, hsl)
  shape.lineTo(hs, -hsl)
  shape.lineTo(hs - wall, -hsl)
  shape.lineTo(hs - wall, -hc)
  // Bottom right
  shape.lineTo(hc, -hc)
  shape.lineTo(hc, -hs + wall)
  shape.lineTo(hsl, -hs + wall)
  shape.lineTo(hsl, -hs)
  shape.closePath()

  return shape
}

/** Servo body profile — rounded rect with mounting ear tabs. */
export function servoBodyShape(w: number, h: number, earW: number, earH: number, earY: number, r: number): THREE.Shape {
  const shape = new THREE.Shape()
  const hw = w / 2, hh = h / 2
  const hew = earW / 2
  const earBot = earY - earH / 2
  const earTop = earY + earH / 2
  r = Math.min(r, hw * 0.4, hh * 0.4)
  const er = r * 0.5

  // Start bottom-left, go clockwise
  shape.moveTo(-hw + r, -hh)
  shape.lineTo(hw - r, -hh)
  shape.quadraticCurveTo(hw, -hh, hw, -hh + r)
  // Right side up to ear
  shape.lineTo(hw, earBot)
  // Right ear
  shape.lineTo(hew, earBot)
  shape.lineTo(hew, earBot + er)
  shape.quadraticCurveTo(hew, earTop, hew - er, earTop)
  shape.lineTo(hw, earTop)
  // Continue to top
  shape.lineTo(hw, hh - r)
  shape.quadraticCurveTo(hw, hh, hw - r, hh)
  shape.lineTo(-hw + r, hh)
  shape.quadraticCurveTo(-hw, hh, -hw, hh - r)
  // Left side down to ear
  shape.lineTo(-hw, earTop)
  // Left ear
  shape.lineTo(-hew + er, earTop)
  shape.quadraticCurveTo(-hew, earTop, -hew, earBot + er)
  shape.lineTo(-hew, earBot)
  shape.lineTo(-hw, earBot)
  // Continue to bottom
  shape.lineTo(-hw, -hh + r)
  shape.quadraticCurveTo(-hw, -hh, -hw + r, -hh)
  return shape
}

// ── Lathe Profile Helpers ────────────────────────────────────────────────────

/** Build a lathe profile (array of Vector2 points) for revolving. */
export function motorProfile(bodyR: number, bodyH: number, shaftR: number, shaftH: number, capR?: number, capH?: number): THREE.Vector2[] {
  const pts: THREE.Vector2[] = []
  const hh = bodyH / 2
  const cr = capR ?? bodyR * 0.85
  const ch = capH ?? bodyH * 0.1

  // Bottom center to cap
  pts.push(new THREE.Vector2(0, -hh - ch))
  pts.push(new THREE.Vector2(cr, -hh - ch))
  pts.push(new THREE.Vector2(cr, -hh))
  // Body
  pts.push(new THREE.Vector2(bodyR, -hh))
  pts.push(new THREE.Vector2(bodyR, hh))
  // Top face to shaft
  pts.push(new THREE.Vector2(shaftR + 0.001, hh))
  pts.push(new THREE.Vector2(shaftR + 0.001, hh + shaftH))
  pts.push(new THREE.Vector2(shaftR, hh + shaftH))
  pts.push(new THREE.Vector2(shaftR, hh))
  // Back to center
  pts.push(new THREE.Vector2(0, hh))

  return pts
}

/** Bearing cross-section profile for lathe. */
export function bearingProfile(outerR: number, innerR: number, width: number, chamfer?: number): THREE.Vector2[] {
  const hw = width / 2
  const c = chamfer ?? width * 0.08
  const pts: THREE.Vector2[] = []

  // Inner bore up
  pts.push(new THREE.Vector2(innerR, -hw))
  pts.push(new THREE.Vector2(innerR, hw))
  // Inner race top chamfer
  pts.push(new THREE.Vector2(innerR + c, hw))
  // Ball track (slight groove)
  const midR = (innerR + outerR) / 2
  pts.push(new THREE.Vector2(midR, hw * 0.7))
  pts.push(new THREE.Vector2(midR, -hw * 0.7))
  pts.push(new THREE.Vector2(innerR + c, -hw))
  pts.push(new THREE.Vector2(innerR, -hw))

  return pts
}

export function bearingOuterProfile(outerR: number, innerR: number, width: number, chamfer?: number): THREE.Vector2[] {
  const hw = width / 2
  const c = chamfer ?? width * 0.08
  const midR = (innerR + outerR) / 2
  const pts: THREE.Vector2[] = []

  pts.push(new THREE.Vector2(midR, -hw * 0.7))
  pts.push(new THREE.Vector2(midR, hw * 0.7))
  pts.push(new THREE.Vector2(outerR - c, hw))
  pts.push(new THREE.Vector2(outerR, hw))
  pts.push(new THREE.Vector2(outerR, -hw))
  pts.push(new THREE.Vector2(outerR - c, -hw))

  return pts
}

/** Gear tooth profile for a single tooth (to be arrayed around circumference). */
export function gearToothProfile(rootR: number, tipR: number, toothWidth: number): THREE.Vector2[] {
  const hw = toothWidth / 2
  return [
    new THREE.Vector2(rootR, -hw * 1.2),
    new THREE.Vector2(tipR, -hw * 0.7),
    new THREE.Vector2(tipR, hw * 0.7),
    new THREE.Vector2(rootR, hw * 1.2),
  ]
}

// ── Geometry Builders ────────────────────────────────────────────────────────

/** Chamfered box using ExtrudeGeometry with rounded rect profile. */
export function chamferedBox(w: number, h: number, d: number, chamfer?: number): THREE.BufferGeometry {
  const r = Math.min(chamfer ?? Math.min(w, h) * 0.06, w / 4, h / 4, d / 4)
  if (r < 0.0002) return new THREE.BoxGeometry(w, h, d)

  const shape = roundedRectShape(w, h, r)
  const geom = new THREE.ExtrudeGeometry(shape, {
    depth: d,
    bevelEnabled: true,
    bevelThickness: r * 0.5,
    bevelSize: r * 0.5,
    bevelSegments: BEVEL_SEGS,
    curveSegments: 6,
  })
  geom.translate(0, 0, -d / 2)
  return geom
}

/** Chamfered cylinder using LatheGeometry with fillet profile. */
export function chamferedCylinder(radius: number, height: number, chamfer?: number, segments?: number): THREE.BufferGeometry {
  const r = Math.min(chamfer ?? radius * 0.04, radius / 3, height / 4)
  const segs = segments ?? SEGMENTS_HIGH

  if (r < 0.0002) return new THREE.CylinderGeometry(radius, radius, height, segs)

  const hh = height / 2
  const steps = 5
  const pts: THREE.Vector2[] = []

  pts.push(new THREE.Vector2(0, -hh))
  pts.push(new THREE.Vector2(radius - r, -hh))
  for (let i = 0; i <= steps; i++) {
    const a = Math.PI / 2 * (1 - i / steps)
    pts.push(new THREE.Vector2(radius - r + r * Math.cos(a), -hh + r - r * Math.sin(a)))
  }
  pts.push(new THREE.Vector2(radius, hh - r))
  for (let i = 0; i <= steps; i++) {
    const a = Math.PI / 2 * (i / steps)
    pts.push(new THREE.Vector2(radius - r + r * Math.cos(a), hh - r + r * Math.sin(a)))
  }
  pts.push(new THREE.Vector2(0, hh))

  return new THREE.LatheGeometry(pts, segs)
}

/** Revolved motor/actuator body from a 2D profile. */
export function revolvedMotor(bodyR: number, bodyH: number, shaftR: number, shaftH: number, segments?: number): THREE.BufferGeometry {
  const pts = motorProfile(bodyR, bodyH, shaftR, shaftH)
  return new THREE.LatheGeometry(pts, segments ?? SEGMENTS_HIGH)
}

/** Gear cylinder with tooth profile around the circumference. */
export function gearCylinder(rootR: number, tipR: number, faceWidth: number, toothCount: number): THREE.Group {
  const group = new THREE.Group()
  const mat = getMaterial('brushed_steel')

  // Base cylinder (root circle)
  const base = new THREE.Mesh(
    chamferedCylinder(rootR, faceWidth, rootR * 0.03),
    mat,
  )
  group.add(base)

  // Teeth
  const toothW = (2 * Math.PI * rootR) / (toothCount * 2.5)
  const toothH = tipR - rootR
  const toothGeom = chamferedBox(toothW, toothH, faceWidth * 0.85, toothW * 0.1)
  for (let i = 0; i < toothCount; i++) {
    const a = (i / toothCount) * Math.PI * 2
    const tooth = new THREE.Mesh(toothGeom, mat)
    const midR = rootR + toothH / 2
    tooth.position.set(Math.cos(a) * midR, 0, Math.sin(a) * midR)
    tooth.rotation.y = -a
    tooth.rotation.x = Math.PI / 2
    group.add(tooth)
  }

  return group
}

/** Extruded T-slot extrusion profile. */
export function tSlotExtrusion(size: number, length: number): THREE.BufferGeometry {
  const shape = tSlotShape(size)
  const geom = new THREE.ExtrudeGeometry(shape, {
    depth: length,
    bevelEnabled: false,
    curveSegments: 2,
  })
  geom.translate(0, 0, -length / 2)
  // Rotate so extrusion runs along Y axis
  geom.rotateX(Math.PI / 2)
  return geom
}

/** Extruded I-beam. */
export function iBeamExtrusion(flangeW: number, totalH: number, webT: number, flangeT: number, length: number): THREE.BufferGeometry {
  const shape = iBeamShape(flangeW, totalH, webT, flangeT)
  const geom = new THREE.ExtrudeGeometry(shape, {
    depth: length,
    bevelEnabled: false,
  })
  geom.translate(0, 0, -length / 2)
  geom.rotateX(Math.PI / 2)
  return geom
}

/** Extruded C-channel. */
export function cChannelExtrusion(flangeW: number, totalH: number, thickness: number, length: number): THREE.BufferGeometry {
  const shape = cChannelShape(flangeW, totalH, thickness)
  const geom = new THREE.ExtrudeGeometry(shape, {
    depth: length,
    bevelEnabled: false,
  })
  geom.translate(-flangeW / 2, -totalH / 2, -length / 2)
  geom.rotateX(Math.PI / 2)
  return geom
}

/** Extruded L-bracket. */
export function lBracketExtrusion(legA: number, legB: number, thickness: number, depth: number): THREE.BufferGeometry {
  const shape = lShape(legA, legB, thickness)
  const geom = new THREE.ExtrudeGeometry(shape, {
    depth,
    bevelEnabled: true,
    bevelThickness: thickness * 0.1,
    bevelSize: thickness * 0.1,
    bevelSegments: 2,
  })
  geom.translate(-legB / 2, -legA / 2, -depth / 2)
  return geom
}

/** Servo body — extruded profile with integrated mounting ears. */
export function servoBody(w: number, h: number, d: number, earW: number, earH: number, earY: number): THREE.BufferGeometry {
  const r = Math.min(w, h) * 0.06
  const shape = servoBodyShape(w, d, earW, earH, earY, r)
  const geom = new THREE.ExtrudeGeometry(shape, {
    depth: h,
    bevelEnabled: true,
    bevelThickness: r * 0.4,
    bevelSize: r * 0.4,
    bevelSegments: BEVEL_SEGS,
    curveSegments: 6,
  })
  geom.translate(0, 0, -h / 2)
  // Rotate so the extrusion depth is along Z, ears face up (Y)
  geom.rotateX(Math.PI / 2)
  return geom
}

// ── Detail Builders ──────────────────────────────────────────────────────────

/** Bolt circle — ring of mounting hole insets. */
export function boltCircle(circleRadius: number, holeRadius: number, count: number, depth: number): THREE.Group {
  const group = new THREE.Group()
  const mat = getMaterial('dark_chrome')
  const geom = new THREE.CylinderGeometry(holeRadius, holeRadius, depth, SEGMENTS_LOW)
  for (let i = 0; i < count; i++) {
    const a = (i / count) * Math.PI * 2
    const mesh = new THREE.Mesh(geom, mat)
    mesh.position.set(Math.cos(a) * circleRadius, 0, Math.sin(a) * circleRadius)
    group.add(mesh)
  }
  return group
}

/** Dark inset cylinder simulating a drilled hole. */
export function mountingHole(radius: number, depth: number): THREE.Mesh {
  const geom = new THREE.CylinderGeometry(radius, radius, depth, SEGMENTS_LOW)
  return new THREE.Mesh(geom, getMaterial('dark_chrome'))
}

/** Hex socket screw head. */
export function screwHead(radius: number, headHeight: number): THREE.Group {
  const group = new THREE.Group()
  const mat = getMaterial('dark_chrome')
  const head = new THREE.Mesh(chamferedCylinder(radius, headHeight, radius * 0.15, 6), mat)
  group.add(head)
  // Hex socket
  const socket = new THREE.Mesh(
    new THREE.CylinderGeometry(radius * 0.45, radius * 0.45, headHeight * 0.55, 6),
    getMaterial('matte_plastic'),
  )
  socket.position.y = headHeight * 0.23
  group.add(socket)
  return group
}

/** Heatsink fin array. */
export function heatsinkFins(w: number, h: number, d: number, finCount: number, finThickness?: number): THREE.Group {
  const group = new THREE.Group()
  const mat = getMaterial('anodized_aluminum', 0x333333)
  const ft = finThickness ?? 0.001
  const baseH = d * 0.15
  group.add(new THREE.Mesh(new THREE.BoxGeometry(w, baseH, h), mat))
  const finH = d - baseH
  const spacing = w / (finCount + 1)
  const finGeom = new THREE.BoxGeometry(ft, finH, h * 0.9)
  for (let i = 1; i <= finCount; i++) {
    const fin = new THREE.Mesh(finGeom, mat)
    fin.position.set(-w / 2 + i * spacing, baseH / 2 + finH / 2, 0)
    group.add(fin)
  }
  return group
}

/** Cable port — torus arc. */
export function cablePort(radius: number, tubeRadius: number): THREE.Mesh {
  return new THREE.Mesh(
    new THREE.TorusGeometry(radius, tubeRadius, 8, SEGMENTS_LOW, Math.PI),
    getMaterial('matte_plastic'),
  )
}

/** PCB board with copper traces. */
export function pcbBoard(w: number, h: number, thickness: number): THREE.Group {
  const group = new THREE.Group()
  const board = new THREE.Mesh(chamferedBox(w, h, thickness, Math.min(w, h) * 0.03), getMaterial('pcb_green'))
  group.add(board)
  const traceMat = getMaterial('copper_trace')
  const traceH = thickness * 0.3
  const traceGeom = new THREE.BoxGeometry(w * 0.9, traceH, thickness * 0.08)
  const t1 = new THREE.Mesh(traceGeom, traceMat)
  t1.position.set(0, h / 2 - traceH, thickness * 0.42)
  group.add(t1)
  const t2 = new THREE.Mesh(traceGeom, traceMat)
  t2.position.set(0, -h / 2 + traceH, thickness * 0.42)
  group.add(t2)
  return group
}

/** Connector block (USB, Ethernet, etc.) */
export function connectorBlock(w: number, h: number, d: number, portColor?: number): THREE.Group {
  const group = new THREE.Group()
  group.add(new THREE.Mesh(chamferedBox(w, h, d, Math.min(w, h, d) * 0.08), getMaterial('anodized_aluminum', 0x666666)))
  const port = new THREE.Mesh(
    new THREE.BoxGeometry(w * 0.75, h * 0.6, d * 0.3),
    getMaterial('matte_plastic', portColor ?? 0x222222),
  )
  port.position.z = d * 0.36
  group.add(port)
  return group
}

/** Flat disc flange with bolt holes. */
export function flangePlate(radius: number, thickness: number, boltCount: number, boltCircleR: number, boltR: number): THREE.Group {
  const group = new THREE.Group()
  group.add(new THREE.Mesh(chamferedCylinder(radius, thickness, thickness * 0.2), getMaterial('anodized_aluminum')))
  group.add(boltCircle(boltCircleR, boltR, boltCount, thickness * 1.05))
  return group
}

/** Dark recessed label area. */
export function labelRecess(w: number, h: number, depth: number): THREE.Mesh {
  return new THREE.Mesh(
    new THREE.BoxGeometry(w, h, depth),
    getMaterial('matte_plastic', 0x1a1a1a),
  )
}

/** Knurled cylinder ring. */
export function knurledRing(radius: number, height: number, grooveCount?: number): THREE.Group {
  const group = new THREE.Group()
  const gc = grooveCount ?? 24
  const mat = getMaterial('brushed_steel')
  group.add(new THREE.Mesh(chamferedCylinder(radius, height, radius * 0.03), mat))
  const grooveMat = getMaterial('dark_chrome')
  const grooveGeom = new THREE.BoxGeometry(radius * 0.04, height * 0.85, radius * 0.02)
  for (let i = 0; i < gc; i++) {
    const a = (i / gc) * Math.PI * 2
    const groove = new THREE.Mesh(grooveGeom, grooveMat)
    groove.position.set(Math.cos(a) * radius * 1.01, 0, Math.sin(a) * radius * 1.01)
    groove.rotation.y = -a
    group.add(groove)
  }
  return group
}

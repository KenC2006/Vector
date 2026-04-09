/**
 * Reusable parametric geometry builders for rich component visuals.
 *
 * All functions return THREE.BufferGeometry (or THREE.Group for composites).
 * No CSG booleans — detail is additive (dark insets simulate holes, raised
 * geometry simulates features). Keeps generation under 1ms per component.
 */
import * as THREE from 'three'
import { getMaterial } from './materials'

// ── Helpers ──────────────────────────────────────────────────────────────────

function roundedRectShape(w: number, h: number, r: number): THREE.Shape {
  const shape = new THREE.Shape()
  const hw = w / 2, hh = h / 2
  r = Math.min(r, hw, hh)
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

// ── Chamfered Box ────────────────────────────────────────────────────────────
// The single biggest visual upgrade — transforms every box from game-engine to CAD.

export function chamferedBox(
  w: number, h: number, d: number,
  chamfer = 0.001,
): THREE.BufferGeometry {
  const r = Math.min(chamfer, w / 4, h / 4, d / 4)
  if (r < 0.0002) return new THREE.BoxGeometry(w, h, d)

  const shape = roundedRectShape(w, h, r)
  const geom = new THREE.ExtrudeGeometry(shape, {
    depth: d,
    bevelEnabled: true,
    bevelThickness: r,
    bevelSize: r,
    bevelSegments: 3,
    curveSegments: 4,
  })
  // Center the extrusion (ExtrudeGeometry starts at z=0, extrudes along +z)
  geom.translate(0, 0, -d / 2)
  return geom
}

// ── Chamfered Cylinder ───────────────────────────────────────────────────────
// Smooth-edged cylinder via LatheGeometry with fillet profile.

export function chamferedCylinder(
  radius: number, height: number,
  chamfer = 0.001,
  segments = 32,
): THREE.BufferGeometry {
  const r = Math.min(chamfer, radius / 3, height / 4)
  if (r < 0.0002) return new THREE.CylinderGeometry(radius, radius, height, segments)

  const hh = height / 2
  // Profile: bottom-center up to top-center with fillet corners
  const points: THREE.Vector2[] = []
  const steps = 4

  // Bottom flat center to bottom edge
  points.push(new THREE.Vector2(0, -hh))
  points.push(new THREE.Vector2(radius - r, -hh))
  // Bottom fillet
  for (let i = 0; i <= steps; i++) {
    const a = Math.PI / 2 * (1 - i / steps)
    points.push(new THREE.Vector2(
      radius - r + r * Math.cos(a),
      -hh + r - r * Math.sin(a),
    ))
  }
  // Straight side
  points.push(new THREE.Vector2(radius, hh - r))
  // Top fillet
  for (let i = 0; i <= steps; i++) {
    const a = Math.PI / 2 * (i / steps)
    points.push(new THREE.Vector2(
      radius - r + r * Math.cos(a),
      hh - r + r * Math.sin(a),
    ))
  }
  // Top flat to center
  points.push(new THREE.Vector2(0, hh))

  const geom = new THREE.LatheGeometry(points, segments)
  // LatheGeometry is Y-axis aligned which matches Three.js cylinder convention
  return geom
}

// ── Bolt Circle ──────────────────────────────────────────────────────────────
// Ring of mounting hole insets for flanged connections.

export function boltCircle(
  circleRadius: number,
  holeRadius: number,
  count: number,
  depth: number,
): THREE.Group {
  const group = new THREE.Group()
  const mat = getMaterial('dark_chrome')
  const geom = new THREE.CylinderGeometry(holeRadius, holeRadius, depth, 12)

  for (let i = 0; i < count; i++) {
    const a = (i / count) * Math.PI * 2
    const mesh = new THREE.Mesh(geom, mat)
    mesh.position.set(
      Math.cos(a) * circleRadius,
      0,
      Math.sin(a) * circleRadius,
    )
    group.add(mesh)
  }
  return group
}

// ── Mounting Hole ────────────────────────────────────────────────────────────
// Dark inset cylinder simulating a drilled hole.

export function mountingHole(
  radius: number,
  depth: number,
): THREE.Mesh {
  const geom = new THREE.CylinderGeometry(radius, radius, depth, 12)
  const mat = getMaterial('dark_chrome')
  return new THREE.Mesh(geom, mat)
}

// ── Screw Head ───────────────────────────────────────────────────────────────
// Hex socket or Phillips screw head.

export function screwHead(
  radius: number,
  headHeight: number,
  socketType: 'hex' | 'phillips' = 'hex',
): THREE.Group {
  const group = new THREE.Group()
  const mat = getMaterial('dark_chrome')

  // Head cylinder
  const headGeom = chamferedCylinder(radius, headHeight, radius * 0.15, 6)
  const head = new THREE.Mesh(headGeom, mat)
  group.add(head)

  // Socket detail
  if (socketType === 'hex') {
    const socketGeom = new THREE.CylinderGeometry(radius * 0.5, radius * 0.5, headHeight * 0.6, 6)
    const socket = new THREE.Mesh(socketGeom, getMaterial('matte_plastic'))
    socket.position.y = headHeight * 0.21
    group.add(socket)
  } else {
    // Phillips cross
    const slotGeom = new THREE.BoxGeometry(radius * 1.2, headHeight * 0.3, radius * 0.15)
    const slot1 = new THREE.Mesh(slotGeom, getMaterial('matte_plastic'))
    slot1.position.y = headHeight * 0.36
    group.add(slot1)
    const slot2 = new THREE.Mesh(slotGeom, getMaterial('matte_plastic'))
    slot2.position.y = headHeight * 0.36
    slot2.rotation.y = Math.PI / 2
    group.add(slot2)
  }

  return group
}

// ── Heatsink Fins ────────────────────────────────────────────────────────────
// Extruded fin array for compute board heatsinks.

export function heatsinkFins(
  w: number, h: number, d: number,
  finCount: number,
  finThickness = 0.001,
): THREE.Group {
  const group = new THREE.Group()
  const mat = getMaterial('anodized_aluminum', 0x333333)

  // Base plate
  const baseH = d * 0.15
  const base = new THREE.Mesh(
    new THREE.BoxGeometry(w, baseH, h),
    mat,
  )
  base.position.y = -d / 2 + baseH / 2
  group.add(base)

  // Fins
  const finH = d - baseH
  const spacing = w / (finCount + 1)
  const finGeom = new THREE.BoxGeometry(finThickness, finH, h * 0.9)
  for (let i = 1; i <= finCount; i++) {
    const fin = new THREE.Mesh(finGeom, mat)
    fin.position.set(-w / 2 + i * spacing, -d / 2 + baseH + finH / 2, 0)
    group.add(fin)
  }

  return group
}

// ── Cable Port ───────────────────────────────────────────────────────────────
// Torus section for connector housings.

export function cablePort(
  radius: number,
  tubeRadius: number,
): THREE.Mesh {
  const geom = new THREE.TorusGeometry(radius, tubeRadius, 8, 16, Math.PI)
  const mat = getMaterial('matte_plastic')
  return new THREE.Mesh(geom, mat)
}

// ── PCB Board ────────────────────────────────────────────────────────────────
// Chamfered green board with copper edge traces and component pads.

export function pcbBoard(
  w: number, h: number, thickness: number,
): THREE.Group {
  const group = new THREE.Group()

  // Main board
  const boardGeom = chamferedBox(w, h, thickness, Math.min(w, h) * 0.03)
  const board = new THREE.Mesh(boardGeom, getMaterial('pcb_green'))
  group.add(board)

  // Copper edge traces (thin strips on long edges)
  const traceThick = thickness * 0.3
  const traceMat = getMaterial('copper_trace')
  const traceGeomH = new THREE.BoxGeometry(w * 0.95, traceThick, thickness * 0.1)
  const traceTop = new THREE.Mesh(traceGeomH, traceMat)
  traceTop.position.set(0, h / 2 - traceThick / 2, thickness * 0.45)
  group.add(traceTop)
  const traceBot = new THREE.Mesh(traceGeomH, traceMat)
  traceBot.position.set(0, -h / 2 + traceThick / 2, thickness * 0.45)
  group.add(traceBot)

  return group
}

// ── Knurled Ring ─────────────────────────────────────────────────────────────
// Cylinder with radial grooves (for grips, adjustment rings).

export function knurledRing(
  radius: number, height: number,
  grooveCount = 24,
): THREE.Group {
  const group = new THREE.Group()
  const mat = getMaterial('brushed_steel')

  // Base cylinder
  const baseGeom = new THREE.CylinderGeometry(radius, radius, height, 32)
  group.add(new THREE.Mesh(baseGeom, mat))

  // Groove lines (thin dark cylinders on surface)
  const grooveMat = getMaterial('dark_chrome')
  const grooveGeom = new THREE.BoxGeometry(radius * 0.04, height * 0.9, radius * 0.02)
  for (let i = 0; i < grooveCount; i++) {
    const a = (i / grooveCount) * Math.PI * 2
    const groove = new THREE.Mesh(grooveGeom, grooveMat)
    groove.position.set(
      Math.cos(a) * radius * 1.01,
      0,
      Math.sin(a) * radius * 1.01,
    )
    groove.rotation.y = -a
    group.add(groove)
  }

  return group
}

// ── Connector Block ──────────────────────────────────────────────────────────
// Generic rectangular connector (USB, Ethernet, etc.)

export function connectorBlock(
  w: number, h: number, d: number,
  portColor = 0x222222,
): THREE.Group {
  const group = new THREE.Group()

  // Shell
  const shell = new THREE.Mesh(
    new THREE.BoxGeometry(w, h, d),
    getMaterial('anodized_aluminum', 0x666666),
  )
  group.add(shell)

  // Port opening (dark inset)
  const port = new THREE.Mesh(
    new THREE.BoxGeometry(w * 0.75, h * 0.6, d * 0.3),
    getMaterial('matte_plastic', portColor),
  )
  port.position.z = d * 0.36
  group.add(port)

  return group
}

// ── Flange Plate ─────────────────────────────────────────────────────────────
// Flat disc with bolt holes (for actuator face mounts).

export function flangePlate(
  radius: number,
  thickness: number,
  boltCount: number,
  boltCircleR: number,
  boltR: number,
): THREE.Group {
  const group = new THREE.Group()

  // Plate disc
  const plateGeom = chamferedCylinder(radius, thickness, thickness * 0.2, 32)
  group.add(new THREE.Mesh(plateGeom, getMaterial('anodized_aluminum')))

  // Bolt holes
  const holes = boltCircle(boltCircleR, boltR, boltCount, thickness * 1.05)
  group.add(holes)

  return group
}

// ── Label Recess ─────────────────────────────────────────────────────────────
// Slightly recessed darker area simulating a label or nameplate.

export function labelRecess(
  w: number, h: number, depth: number,
): THREE.Mesh {
  const geom = new THREE.BoxGeometry(w, h, depth)
  return new THREE.Mesh(geom, getMaterial('matte_plastic', 0x1a1a1a))
}

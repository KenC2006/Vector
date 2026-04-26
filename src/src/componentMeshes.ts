/**
 * Parametric mesh generators for the Vector Component Preset Library.
 *
 * Each component type gets a distinctive silhouette built from URDF-legal
 * primitives (box, cylinder, sphere).  A single link can carry multiple
 * <visual> elements — this module returns an array of them so the 3-D
 * viewport shows a recognisable part while URDF stays standards-compliant.
 */

// ── Types ────────────────────────────────────────────────────────────────────

export interface UrdfVisualDesc {
  origin_xyz: [number, number, number]
  origin_rpy: [number, number, number]
  geometry:
    | { type: 'box'; size: [number, number, number] }
    | { type: 'cylinder'; radius: number; length: number }
    | { type: 'sphere'; radius: number }
  color_rgba: [number, number, number, number]
}

// ── Category colour palette ──────────────────────────────────────────────────

import { getComponentColor } from './richVisuals/materials'

export const CATEGORY_COLORS: Record<string, [number, number, number, number]> = {
  actuators:      [0.90, 0.49, 0.13, 1],  // orange
  motors:         [0.91, 0.30, 0.24, 1],  // red-orange
  sensors:        [0.20, 0.60, 0.86, 1],  // blue
  compute:        [0.18, 0.80, 0.44, 1],  // green
  power:          [0.95, 0.77, 0.06, 1],  // yellow
  structural:     [0.66, 0.70, 0.72, 1],  // silver-grey
  transmission:   [0.56, 0.27, 0.68, 1],  // purple
  end_effectors:  [0.10, 0.74, 0.61, 1],  // teal
  mobility:       [0.20, 0.29, 0.37, 1],  // dark slate
}

// Per-shape-generator component ID — set by generateVisuals before dispatching
let _currentCompId: string | null = null

function catColor(category: string, darken = 0): [number, number, number, number] {
  // Use per-component color when available (set by generateVisuals)
  let c: [number, number, number, number]
  if (_currentCompId) {
    const cc = getComponentColor(_currentCompId)
    c = [cc.tint[0], cc.tint[1], cc.tint[2], 1]
  } else {
    c = CATEGORY_COLORS[category] ?? [0.6, 0.6, 0.6, 1]
  }
  if (darken === 0) return c
  const f = 1 - darken * 0.25
  return [c[0] * f, c[1] * f, c[2] * f, c[3]]
}

// ── Primitive helpers (all dims in metres) ───────────────────────────────────

function box(
  sx: number, sy: number, sz: number,
  ox = 0, oy = 0, oz = 0,
  color: [number, number, number, number],
  rr = 0, rp = 0, ry = 0,
): UrdfVisualDesc {
  return {
    origin_xyz: [ox, oy, oz],
    origin_rpy: [rr, rp, ry],
    geometry: { type: 'box', size: [sx, sy, sz] },
    color_rgba: color,
  }
}

function cyl(
  radius: number, length: number,
  ox = 0, oy = 0, oz = 0,
  color: [number, number, number, number],
  rr = 0, rp = 0, ry = 0,
): UrdfVisualDesc {
  return {
    origin_xyz: [ox, oy, oz],
    origin_rpy: [rr, rp, ry],
    geometry: { type: 'cylinder', radius, length },
    color_rgba: color,
  }
}

function sphere(
  radius: number,
  ox = 0, oy = 0, oz = 0,
  color: [number, number, number, number],
): UrdfVisualDesc {
  return {
    origin_xyz: [ox, oy, oz],
    origin_rpy: [0, 0, 0],
    geometry: { type: 'sphere', radius },
    color_rgba: color,
  }
}

// mm → m helper
function mm(v: number): number { return v / 1000 }

// ── Shape generators by component class ──────────────────────────────────────

// Servo: rectangular body + mounting ears + circular horn on output face
/** Z-ratio of the horn joint origin above the servo body frame (URDF Z-up). */
export const SERVO_HORN_ORIGIN_Z_RATIO = 0.44

export function servoBodyShape(
  w: number, h: number, d: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const earH = h * 0.12
  const earW = w * 1.15
  return [
    box(w, d, h * 0.76, 0, 0, 0, c),
    box(earW, d, earH, 0, 0, h * 0.32, c2),
  ]
}

/**
 * Visual primitives for the servo horn link.
 * All Z positions are relative to the joint origin (at SERVO_HORN_ORIGIN_Z_RATIO * h
 * above the body frame), so they are already in the horn link's local frame.
 */
export function servoHornShape(
  w: number, h: number, d: number,
  cat: string,
): UrdfVisualDesc[] {
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  const hornR = Math.min(w, d) * 0.35
  const hornH = h * 0.12
  return [
    cyl(hornR, hornH, 0, 0, 0, c3),
    cyl(hornR * 0.25, hornH * 0.8, 0, 0, h * 0.08, c2),
  ]
}

export function servoSideYokeShape(
  w: number, h: number, d: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat, 1)
  const c2 = catColor(cat, 2)
  const plateT = Math.max(Math.min(w, d) * 0.08, mm(2))
  const sideGap = d / 2 + plateT * 1.4
  return [
    box(w * 1.18, plateT, h * 1.08, 0, sideGap, 0, c),
    box(w * 1.18, plateT, h * 1.08, 0, -sideGap, 0, c),
    box(w * 1.18, d + plateT * 3, plateT, 0, 0, -h * 0.54, c2),
  ]
}

export function servoHornBeamAdapterShape(
  w: number, h: number, d: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat, 1)
  const c2 = catColor(cat, 2)
  const plateT = Math.max(Math.min(w, d) * 0.08, mm(2.5))
  return [
    box(w * 0.74, d * 0.46, plateT, 0, 0, plateT * 0.45, c),
    box(w * 0.28, d * 0.92, plateT * 0.75, 0, 0, plateT * 1.15, c2),
  ]
}

export function limbLinkSlimShape(
  w: number, h: number, d: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const dark: [number, number, number, number] = [0.08, 0.08, 0.08, 1]
  const padR = Math.max(w * 0.52, mm(5))
  const padZ = Math.max(h / 2 - padR, 0)
  const webW = Math.max(w * 0.48, mm(5))
  const webLen = Math.max(h - padR * 1.7, mm(8))
  return [
    box(webW, d, webLen, 0, 0, 0, c),
    cyl(padR, d, 0, 0, padZ, c2, Math.PI / 2, 0, 0),
    cyl(padR, d, 0, 0, -padZ, c2, Math.PI / 2, 0, 0),
    cyl(padR * 0.32, d * 1.05, 0, 0, padZ, dark, Math.PI / 2, 0, 0),
    cyl(padR * 0.32, d * 1.05, 0, 0, -padZ, dark, Math.PI / 2, 0, 0),
  ]
}

function servoShape(
  w: number, h: number, d: number,      // bounding box in m
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  const earH = h * 0.12
  const earW = w * 1.15
  const hornR = Math.min(w, d) * 0.35
  const hornH = h * 0.12
  return [
    // Main body — height along Z (URDF up), depth along Y
    box(w, d, h * 0.76, 0, 0, 0, c),
    // Mounting ears (midway up the body)
    box(earW, d, earH, 0, 0, h * 0.32, c2),
    // Output horn (top)
    cyl(hornR, hornH, 0, 0, h * 0.44, c3),
    // Shaft nub
    cyl(hornR * 0.25, hornH * 0.8, 0, 0, h * 0.52, c2),
  ]
}

// DC / brushed motor: cylindrical body + rear cap + front shaft
function dcMotorShape(
  bodyR: number, bodyH: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const shaftR = bodyR * 0.12
  const shaftH = bodyH * 0.35
  const capH = bodyH * 0.12
  return [
    // Main body cylinder (axis along Z in URDF cylinder convention)
    cyl(bodyR, bodyH, 0, 0, 0, c),
    // Rear cap
    cyl(bodyR * 0.85, capH, 0, 0, -(bodyH + capH) / 2, c2),
    // Front shaft
    cyl(shaftR, shaftH, 0, 0, (bodyH + shaftH) / 2, c2),
    // Terminal bumps on rear
    box(bodyR * 0.3, bodyR * 0.5, capH, 0, bodyR * 0.5, -(bodyH + capH) / 2, c2),
  ]
}

// Gear motor: DC motor body + gearbox housing on front + output shaft
function gearMotorShape(
  bodyR: number, bodyH: number,
  gearboxW: number, gearboxH: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  const shaftR = gearboxW * 0.1
  const shaftH = gearboxH * 0.5
  return [
    // Motor body
    cyl(bodyR, bodyH, 0, 0, -gearboxH / 2, c),
    // Gearbox (box)
    box(gearboxW, gearboxW, gearboxH, 0, 0, (bodyH) / 2, c2),
    // Output shaft
    cyl(shaftR, shaftH, 0, 0, (bodyH + gearboxH + shaftH) / 2, c3),
  ]
}

// NEMA stepper: square faceplate + round body + shaft
function stepperShape(
  faceW: number, bodyR: number, bodyH: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  const shaftR = mm(2.5)
  const shaftH = bodyH * 0.3
  const faceH = bodyH * 0.08
  return [
    // Square faceplate
    box(faceW, faceW, faceH, 0, 0, (bodyH + faceH) / 2, c2),
    // Round body
    cyl(bodyR, bodyH, 0, 0, 0, c),
    // Shaft
    cyl(shaftR, shaftH, 0, 0, (bodyH + faceH + shaftH) / 2, c3),
    // Rear connector bump
    box(faceW * 0.4, faceW * 0.25, faceH, 0, -faceW * 0.3, -(bodyH + faceH) / 2, c2),
  ]
}

// Linear actuator: long body box + extending rod cylinder
function linearActuatorShape(
  w: number, h: number, length: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const rodR = Math.min(w, h) * 0.2
  const rodLen = length * 0.4
  return [
    // Main body
    box(w, h, length, 0, 0, 0, c),
    // Extending rod
    cyl(rodR, rodLen, 0, 0, (length + rodLen) / 2, c2),
    // Rear clevis mount
    box(w * 0.6, h * 1.2, h * 0.3, 0, 0, -(length + h * 0.3) / 2, c2),
  ]
}

// BLDC outrunner: wide flat cylinder + shaft + mounting face
function bldcShape(
  r: number, h: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  const shaftR = r * 0.12
  const shaftH = h * 0.5
  return [
    // Main housing
    cyl(r, h, 0, 0, 0, c),
    // Top ring detail
    cyl(r * 1.02, h * 0.08, 0, 0, h * 0.46, c2),
    // Output shaft
    cyl(shaftR, shaftH, 0, 0, (h + shaftH) / 2, c3),
    // Rear mounting face
    cyl(r * 0.85, h * 0.06, 0, 0, -(h + h * 0.06) / 2, c2),
    // Cable exit
    box(r * 0.15, r * 0.15, h * 0.2, r * 0.6, 0, -h * 0.3, c3),
  ]
}

// Hub motor: large flat donut shape
function hubMotorShape(
  r: number, h: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  return [
    // Outer ring
    cyl(r, h, 0, 0, 0, c),
    // Inner hub (darker, smaller)
    cyl(r * 0.4, h * 1.05, 0, 0, 0, c2),
    // Axle
    cyl(r * 0.1, h * 1.5, 0, 0, 0, catColor(cat, 2)),
  ]
}

// Harmonic drive: stacked cylinders
function harmonicDriveShape(
  r: number, h: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  return [
    // Main body
    cyl(r, h * 0.6, 0, 0, 0, c),
    // Output flange (slightly wider)
    cyl(r * 1.08, h * 0.15, 0, 0, h * 0.375, c2),
    // Input side
    cyl(r * 0.9, h * 0.25, 0, 0, -h * 0.425, c2),
    // Output shaft
    cyl(r * 0.15, h * 0.2, 0, 0, h * 0.55, c3),
  ]
}

// Depth camera: wide thin box + lens circles
function depthCameraShape(
  w: number, h: number, d: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 2)
  const lensR = Math.min(h, d) * 0.25
  const lensD = h * 0.08
  return [
    // Main housing
    box(w, h, d, 0, 0, 0, c),
    // Left lens
    cyl(lensR, lensD, -w * 0.28, 0, d / 2, c2, Math.PI / 2, 0, 0),
    // Right lens
    cyl(lensR, lensD, w * 0.28, 0, d / 2, c2, Math.PI / 2, 0, 0),
    // IR emitter (center, smaller)
    cyl(lensR * 0.5, lensD, 0, 0, d / 2, c2, Math.PI / 2, 0, 0),
    // Mounting tab (bottom)
    box(w * 0.15, h * 0.3, d * 0.3, 0, -h * 0.6, 0, catColor(cat, 1)),
  ]
}

// LiDAR puck
function lidarPuckShape(
  r: number, h: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  return [
    // Body
    cyl(r, h * 0.7, 0, 0, 0, c),
    // Top dome (slightly smaller)
    cyl(r * 0.92, h * 0.3, 0, 0, h * 0.35, c2),
    // Sensor window band (middle ring — slightly wider)
    cyl(r * 1.03, h * 0.15, 0, 0, h * 0.1, [0.15, 0.15, 0.15, 0.6]),
    // Base plate
    cyl(r * 1.1, h * 0.06, 0, 0, -h * 0.38, catColor(cat, 2)),
  ]
}

// IMU / tiny PCB
function pcbShape(
  w: number, h: number, d: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 2)
  const chipH = d * 0.4
  return [
    // PCB board
    box(w, h, d, 0, 0, 0, [0.1, 0.35, 0.1, 1]),
    // Main chip
    box(w * 0.4, h * 0.4, chipH, 0, 0, (d + chipH) / 2, c2),
    // Connector
    box(w * 0.25, h * 0.15, d * 0.5, w * 0.3, -h * 0.35, 0, c),
  ]
}

// Ultrasonic sensor: box with two "eyes"
function ultrasonicShape(
  w: number, h: number, d: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 2)
  const eyeR = Math.min(w, h) * 0.22
  return [
    box(w, h, d, 0, 0, 0, c),
    // Left eye (transducer)
    cyl(eyeR, d * 0.15, -w * 0.2, 0, d / 2, c2, Math.PI / 2, 0, 0),
    // Right eye (transducer)
    cyl(eyeR, d * 0.15, w * 0.2, 0, d / 2, c2, Math.PI / 2, 0, 0),
  ]
}

// Force/torque sensor: flat cylinder with bolt ring
function ftSensorShape(
  r: number, h: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const boltR = r * 0.06
  const n = 6
  const visuals: UrdfVisualDesc[] = [
    cyl(r, h, 0, 0, 0, c),
    cyl(r * 0.3, h * 1.1, 0, 0, 0, c2),
  ]
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2
    visuals.push(cyl(boltR, h * 1.05, Math.cos(a) * r * 0.8, Math.sin(a) * r * 0.8, 0, c2))
  }
  return visuals
}

// SBC board: PCB + heatsink + ports
function sbcShape(
  w: number, h: number, d: number,
  hasHeatsink: boolean,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const pcbColor: [number, number, number, number] = [0.1, 0.35, 0.1, 1]
  const portColor = catColor(cat, 2)
  const result: UrdfVisualDesc[] = [
    // PCB
    box(w, h, d * 0.3, 0, 0, 0, pcbColor),
    // USB ports (side)
    box(w * 0.08, h * 0.12, d * 0.35, w * 0.46, -h * 0.25, d * 0.05, portColor),
    box(w * 0.08, h * 0.12, d * 0.35, w * 0.46, -h * 0.05, d * 0.05, portColor),
    // Ethernet (side)
    box(w * 0.08, h * 0.15, d * 0.4, w * 0.46, h * 0.2, d * 0.05, portColor),
    // GPIO header
    box(w * 0.65, h * 0.04, d * 0.25, 0, h * 0.4, d * 0.15, c),
  ]
  if (hasHeatsink) {
    // Heatsink block with fin lines
    result.push(box(w * 0.5, h * 0.5, d * 0.6, -w * 0.1, 0, d * 0.45, [0.3, 0.3, 0.3, 1]))
    // Fin lines (3 slots)
    for (let i = -1; i <= 1; i++) {
      result.push(box(w * 0.52, h * 0.02, d * 0.62, -w * 0.1, i * h * 0.15, d * 0.45, [0.15, 0.15, 0.15, 1]))
    }
  }
  return result
}

// Battery: rounded-ish box with connector tab + label area
function batteryShape(
  w: number, h: number, d: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  return [
    // Main body
    box(w, h, d, 0, 0, 0, c),
    // Label strip (top face, slightly raised)
    box(w * 0.8, h * 0.6, d * 0.02, 0, 0, d / 2, c2),
    // Wire connector (end)
    box(w * 0.2, h * 0.15, d * 0.08, w * 0.35, -h * 0.35, d / 2, [0.8, 0.1, 0.1, 1]),
    // Balance connector
    box(w * 0.12, h * 0.08, d * 0.06, -w * 0.25, -h * 0.35, d / 2, [1, 1, 1, 1]),
  ]
}

// Extrusion profile: main square + T-slot channels (4 grooves)
function extrusionShape(
  profile: number, length: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const slotW = profile * 0.3
  const slotD = profile * 0.2
  return [
    // Main body
    box(profile, profile, length, 0, 0, 0, c),
    // T-slot channels (grooves shown as darker insets on each face)
    box(slotW, slotD, length * 1.001, 0, profile / 2, 0, c2),   // top
    box(slotW, slotD, length * 1.001, 0, -profile / 2, 0, c2),  // bottom
    box(slotD, slotW, length * 1.001, profile / 2, 0, 0, c2),   // right
    box(slotD, slotW, length * 1.001, -profile / 2, 0, 0, c2),  // left
  ]
}

// I-beam: web + 2 flanges
function ibeamShape(
  flangeW: number, totalH: number, length: number, webT: number, flangeT: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const webH = totalH - 2 * flangeT
  return [
    // Web (center vertical plate)
    box(webT, webH, length, 0, 0, 0, c),
    // Top flange
    box(flangeW, flangeT, length, 0, (webH + flangeT) / 2, 0, c2),
    // Bottom flange
    box(flangeW, flangeT, length, 0, -(webH + flangeT) / 2, 0, c2),
  ]
}

// C-channel: web + 2 flanges on same side
function channelShape(
  flangeW: number, totalH: number, length: number, t: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const webH = totalH
  return [
    // Web (back plate)
    box(t, webH, length, -flangeW / 2, 0, 0, c),
    // Top flange
    box(flangeW, t, length, 0, (webH - t) / 2, 0, c2),
    // Bottom flange
    box(flangeW, t, length, 0, -(webH - t) / 2, 0, c2),
  ]
}

// Angle stock: L-profile
function angleShape(
  legA: number, legB: number, length: number, t: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  return [
    // Vertical leg
    box(t, legA, length, -(legB - t) / 2, (legA - t) / 2, 0, c),
    // Horizontal leg
    box(legB, t, length, 0, 0, 0, c2),
  ]
}

// Bracket L: 90-degree bracket
function bracketLShape(
  size: number, thickness: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  return [
    box(size, thickness, size, 0, 0, 0, c),
    box(thickness, size, size, -(size - thickness) / 2, (size - thickness) / 2, 0, c2),
  ]
}

// Bracket U: U-channel for servo mounting
function bracketUShape(
  w: number, h: number, d: number, t: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  return [
    // Base plate
    box(w, t, d, 0, 0, 0, c),
    // Left wall
    box(t, h, d, -(w - t) / 2, (h + t) / 2, 0, c2),
    // Right wall
    box(t, h, d, (w - t) / 2, (h + t) / 2, 0, c2),
  ]
}

// Plate: flat box (joint plate, baseplate, gusset, sheet metal)
function plateShape(
  w: number, h: number, t: number,
  cat: string,
  holes = false,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const result: UrdfVisualDesc[] = [box(w, h, t, 0, 0, 0, c)]
  if (holes) {
    // Indicate mounting holes with darker circles
    const holeR = Math.min(w, h) * 0.04
    const c2 = catColor(cat, 2)
    result.push(cyl(holeR, t * 1.01, -w * 0.35, -h * 0.35, 0, c2))
    result.push(cyl(holeR, t * 1.01, w * 0.35, -h * 0.35, 0, c2))
    result.push(cyl(holeR, t * 1.01, -w * 0.35, h * 0.35, 0, c2))
    result.push(cyl(holeR, t * 1.01, w * 0.35, h * 0.35, 0, c2))
  }
  return result
}

// Bearing: outer ring + inner ring (torus-like via stacked cylinders)
function bearingShape(
  od: number, bore: number, width: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 2)
  return [
    // Outer race
    cyl(od / 2, width, 0, 0, 0, c),
    // Inner race (darker, visible from side)
    cyl(bore / 2 + (od / 2 - bore / 2) * 0.15, width * 1.01, 0, 0, 0, c2),
    // Inner bore
    cyl(bore / 2, width * 1.02, 0, 0, 0, [0.12, 0.12, 0.12, 1]),
  ]
}

// Parallel gripper: base + 2 fingers
function parallelGripperShape(
  w: number, h: number, d: number,
  fingerW: number, fingerH: number,
  opening: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  return [
    // Base housing
    box(w, h * 0.5, d, 0, 0, 0, c),
    // Rail (front face)
    box(w * 0.85, h * 0.08, d * 0.12, 0, -h * 0.29, d * 0.44, c2),
    // Left finger
    box(fingerW, fingerH, d * 0.8, -opening / 2, -h * 0.25 - fingerH / 2, d * 0.1, c3),
    // Right finger
    box(fingerW, fingerH, d * 0.8, opening / 2, -h * 0.25 - fingerH / 2, d * 0.1, c3),
    // Mounting flange (top)
    cyl(w * 0.3, h * 0.06, 0, h * 0.28, 0, c2),
  ]
}

// 3-finger gripper: cylindrical base + 3 fingers at 120°
function threeFingerGripperShape(
  r: number, h: number,
  fingerW: number, fingerH: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  const result: UrdfVisualDesc[] = [
    cyl(r, h * 0.4, 0, 0, 0, c),
    cyl(r * 0.85, h * 0.06, 0, 0, h * 0.23, c2),
  ]
  for (let i = 0; i < 3; i++) {
    const a = (i / 3) * Math.PI * 2 - Math.PI / 2
    const fx = Math.cos(a) * r * 0.6
    const fz = Math.sin(a) * r * 0.6
    result.push(box(fingerW, fingerH, fingerW, fx, -h * 0.2 - fingerH / 2, fz, c3))
    // Fingertip
    result.push(sphere(fingerW * 0.6, fx, -h * 0.2 - fingerH, fz, c2))
  }
  return result
}

// Suction cup: cylinder body + bell-shaped cup
function suctionCupShape(
  cupR: number, bodyH: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  return [
    // Body tube
    cyl(cupR * 0.35, bodyH * 0.6, 0, 0, bodyH * 0.2, c),
    // Cup lip (wider cylinder)
    cyl(cupR, bodyH * 0.15, 0, 0, -bodyH * 0.25, c2),
    // Cup inner
    cyl(cupR * 0.8, bodyH * 0.2, 0, 0, -bodyH * 0.15, catColor(cat, 2)),
    // Air fitting on top
    cyl(cupR * 0.15, bodyH * 0.15, 0, 0, bodyH * 0.55, c),
  ]
}

// Tool changer: flat disc with locking features
function toolChangerShape(
  r: number, h: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  const result: UrdfVisualDesc[] = [
    // Main plate
    cyl(r, h * 0.5, 0, 0, 0, c),
    // Center pilot ring
    cyl(r * 0.35, h * 0.7, 0, 0, 0, c2),
    // Locating pins (3)
  ]
  for (let i = 0; i < 3; i++) {
    const a = (i / 3) * Math.PI * 2
    result.push(cyl(r * 0.06, h * 0.8, Math.cos(a) * r * 0.65, Math.sin(a) * r * 0.65, 0, c3))
  }
  return result
}

// Wheel: flat cylinder + hub
function wheelShape(
  r: number, w: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  // Wheel is oriented sideways (rotate 90° around X to spin on Y axis)
  return [
    // Tire
    cyl(r, w, 0, 0, 0, c, Math.PI / 2, 0, 0),
    // Hub
    cyl(r * 0.35, w * 1.05, 0, 0, 0, c2, Math.PI / 2, 0, 0),
    // Axle bore
    cyl(r * 0.08, w * 1.1, 0, 0, 0, [0.1, 0.1, 0.1, 1], Math.PI / 2, 0, 0),
  ]
}

// Mecanum wheel: cylinder + angled rollers
function mecanumWheelShape(
  r: number, w: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  const rollerR = r * 0.12
  const result: UrdfVisualDesc[] = [
    // Side plates
    cyl(r, w * 0.1, 0, 0, -w * 0.4, c2, Math.PI / 2, 0, 0),
    cyl(r, w * 0.1, 0, 0, w * 0.4, c2, Math.PI / 2, 0, 0),
    // Hub
    cyl(r * 0.3, w * 1.05, 0, 0, 0, c, Math.PI / 2, 0, 0),
  ]
  // Rollers around circumference (angled 45°)
  const n = 9
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2
    const rx = Math.cos(a) * r * 0.78
    const ry = Math.sin(a) * r * 0.78
    result.push(cyl(rollerR, w * 0.7, rx, ry, 0, c3, Math.PI / 2, Math.PI / 4, a))
  }
  return result
}

// Caster wheel: fork bracket + swivel + wheel
function casterShape(
  wheelR: number, totalH: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const forkW = wheelR * 1.6
  const forkH = totalH - wheelR
  return [
    // Top mounting plate
    cyl(forkW * 0.5, totalH * 0.08, 0, 0, totalH * 0.46, c2),
    // Swivel stem
    cyl(forkW * 0.12, forkH * 0.3, 0, 0, totalH * 0.3, c),
    // Fork sides
    box(forkW * 0.06, forkH * 0.5, wheelR * 0.5, -forkW * 0.35, 0, 0, c),
    box(forkW * 0.06, forkH * 0.5, wheelR * 0.5, forkW * 0.35, 0, 0, c),
    // Wheel
    cyl(wheelR, wheelR * 0.35, 0, -forkH * 0.25, 0, c2, Math.PI / 2, 0, 0),
  ]
}

// Track / tread system: two drive wheels + flat belt
function trackShape(
  w: number, h: number, length: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const sprocketR = h * 0.4
  return [
    // Tread (long flat box)
    box(w, h * 0.15, length, 0, -h * 0.3, 0, c2),
    // Top tread return
    box(w, h * 0.08, length * 0.8, 0, h * 0.3, 0, c2),
    // Front sprocket
    cyl(sprocketR, w * 1.05, 0, 0, length * 0.4, c, 0, Math.PI / 2, 0),
    // Rear sprocket
    cyl(sprocketR, w * 1.05, 0, 0, -length * 0.4, c, 0, Math.PI / 2, 0),
    // Side armor/guard
    box(w * 0.05, h, length, w * 0.45, 0, 0, catColor(cat, 2)),
    box(w * 0.05, h, length, -w * 0.45, 0, 0, catColor(cat, 2)),
  ]
}

// Gear pair: two meshing cylinders
function gearPairShape(
  r1: number, r2: number, faceWidth: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  return [
    // Drive gear
    cyl(r1, faceWidth, 0, 0, 0, c),
    cyl(r1 * 0.2, faceWidth * 1.3, 0, 0, 0, c3),
    // Driven gear (offset by r1+r2)
    cyl(r2, faceWidth, r1 + r2, 0, 0, c2),
    cyl(r2 * 0.2, faceWidth * 1.3, r1 + r2, 0, 0, c3),
  ]
}

// Timing belt + pulleys
function beltDriveShape(
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const pR = mm(10)
  const beltW = mm(6)
  const spacing = mm(60)
  return [
    // Driver pulley
    cyl(pR, beltW, 0, 0, 0, c),
    cyl(pR * 0.25, beltW * 1.5, 0, 0, 0, catColor(cat, 2)),
    // Driven pulley
    cyl(pR * 1.5, beltW, spacing, 0, 0, c2),
    cyl(pR * 0.25, beltW * 1.5, spacing, 0, 0, catColor(cat, 2)),
    // Belt (flat box connecting them)
    box(spacing, mm(1), beltW, spacing / 2, pR * 1.2, 0, [0.15, 0.15, 0.15, 1]),
    box(spacing, mm(1), beltW, spacing / 2, -pR * 1.2, 0, [0.15, 0.15, 0.15, 1]),
  ]
}

// Leadscrew: long thin cylinder + nut
function leadscrewShape(
  r: number, length: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const nutW = r * 4
  return [
    cyl(r, length, 0, 0, 0, c),
    // Nut
    box(nutW, nutW, nutW * 0.8, 0, 0, 0, c2),
    // Bearing block (end)
    box(nutW * 1.2, nutW * 1.2, nutW * 0.5, 0, 0, (length - nutW * 0.5) / 2, catColor(cat, 2)),
  ]
}

// Coupling: short cylinder with split line
function couplingShape(
  r: number, h: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  return [
    cyl(r, h * 0.45, 0, 0, h * 0.14, c),
    cyl(r, h * 0.45, 0, 0, -h * 0.14, c2),
    // Split ring
    cyl(r * 1.05, h * 0.04, 0, 0, 0, catColor(cat, 2)),
  ]
}

// Rack and pinion
function rackPinionShape(
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  const rackL = mm(80)
  const rackH = mm(8)
  const rackW = mm(12)
  const pinionR = mm(8)
  return [
    // Rack bar
    box(rackW, rackH, rackL, 0, 0, 0, c),
    // Teeth line (raised strip)
    box(rackW * 1.01, rackH * 0.25, rackL, 0, rackH * 0.6, 0, c2),
    // Pinion gear
    cyl(pinionR, rackW * 1.1, 0, rackH / 2 + pinionR, 0, c3, 0, Math.PI / 2, 0),
    // Pinion shaft
    cyl(pinionR * 0.25, rackW * 2, 0, rackH / 2 + pinionR, 0, c2, 0, Math.PI / 2, 0),
  ]
}

// Universal joint: two yokes + cross
function ujointShape(
  r: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  return [
    // Input yoke
    cyl(r * 0.3, r * 1.2, 0, 0, -r * 0.5, c),
    box(r * 0.8, r * 0.15, r * 0.5, 0, 0, -r * 0.15, c2),
    // Cross
    sphere(r * 0.2, 0, 0, 0, c3),
    // Output yoke
    cyl(r * 0.3, r * 1.2, 0, 0, r * 0.5, c),
    box(r * 0.15, r * 0.8, r * 0.5, 0, 0, r * 0.15, c2),
  ]
}

// Standoff: hex-ish cylinder
function standoffShape(
  r: number, h: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 2)
  return [
    cyl(r, h, 0, 0, 0, c),
    // Thread stubs
    cyl(r * 0.4, h * 0.15, 0, 0, (h + h * 0.15) / 2, c2),
    cyl(r * 0.4, h * 0.15, 0, 0, -(h + h * 0.15) / 2, c2),
  ]
}

// Linear rail + carriage
function linearRailShape(
  w: number, h: number, length: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  return [
    // Rail
    box(w, h * 0.4, length, 0, 0, 0, c),
    // Carriage block
    box(w * 1.8, h, w * 2.5, 0, h * 0.5, 0, c2),
    // Carriage mounting holes
    cyl(w * 0.08, h * 1.01, -w * 0.5, h * 0.5, -w * 0.8, c3),
    cyl(w * 0.08, h * 1.01, w * 0.5, h * 0.5, -w * 0.8, c3),
    cyl(w * 0.08, h * 1.01, -w * 0.5, h * 0.5, w * 0.8, c3),
    cyl(w * 0.08, h * 1.01, w * 0.5, h * 0.5, w * 0.8, c3),
  ]
}

// Pillow block: bearing housing on a base
function pillowBlockShape(
  boreR: number, blockW: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const blockH = blockW * 0.6
  const baseH = blockH * 0.25
  return [
    // Base
    box(blockW * 1.4, baseH, blockW, 0, 0, 0, c),
    // Housing (cylinder)
    cyl(blockW * 0.35, blockW, 0, blockH * 0.4, 0, c2, 0, Math.PI / 2, 0),
    // Bore
    cyl(boreR, blockW * 1.01, 0, blockH * 0.4, 0, catColor(cat, 2), 0, Math.PI / 2, 0),
    // Mounting feet bolt holes
    cyl(blockW * 0.06, baseH * 1.01, -blockW * 0.55, 0, 0, catColor(cat, 2)),
    cyl(blockW * 0.06, baseH * 1.01, blockW * 0.55, 0, 0, catColor(cat, 2)),
  ]
}

// Swerve drive module: box housing + wheel
function swerveDriveShape(
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  const modW = mm(60)
  const modH = mm(80)
  const wheelR = mm(40)
  const wheelW = mm(25)
  return [
    // Steering housing
    box(modW, modH * 0.5, modW, 0, modH * 0.25, 0, c),
    // Steering motor (top cylinder)
    cyl(modW * 0.3, modH * 0.2, 0, modH * 0.6, 0, c2),
    // Fork
    box(modW * 0.08, modH * 0.4, wheelW * 1.2, -modW * 0.35, -modH * 0.1, 0, c),
    box(modW * 0.08, modH * 0.4, wheelW * 1.2, modW * 0.35, -modH * 0.1, 0, c),
    // Wheel
    cyl(wheelR, wheelW, 0, -modH * 0.3, 0, c3, Math.PI / 2, 0, 0),
  ]
}

// Ball transfer unit: sphere on cylinder housing
function ballTransferShape(
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const ballR = mm(8)
  return [
    cyl(ballR * 1.3, ballR * 1.5, 0, 0, 0, c),
    sphere(ballR, 0, 0, ballR * 1.0, catColor(cat, 1)),
  ]
}

// Rubber foot/pad
function rubberFootShape(
  cat: string,
): UrdfVisualDesc[] {
  return [
    cyl(mm(10), mm(5), 0, 0, 0, [0.2, 0.2, 0.2, 1]),
    cyl(mm(12), mm(2), 0, 0, -mm(2.5), catColor(cat, 1)),
  ]
}

// Worm gear set
function wormGearShape(
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  const wormR = mm(6)
  const wormL = mm(30)
  const gearR = mm(15)
  const faceW = mm(8)
  return [
    // Worm shaft
    cyl(wormR, wormL, 0, 0, 0, c),
    // Worm thread spiral (raised ring)
    cyl(wormR * 1.2, wormL * 0.9, 0, 0, 0, c2),
    // Worm gear (perpendicular)
    cyl(gearR, faceW, 0, -(wormR + gearR * 0.9), 0, c3, Math.PI / 2, 0, 0),
    cyl(gearR * 0.2, faceW * 1.3, 0, -(wormR + gearR * 0.9), 0, c, Math.PI / 2, 0, 0),
  ]
}

// Slewing ring / crossed roller bearing
function slewingRingShape(
  od: number, h: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  return [
    cyl(od / 2, h, 0, 0, 0, c),
    cyl(od / 2 * 0.65, h * 1.01, 0, 0, 0, c2),
    cyl(od / 2 * 0.5, h * 1.02, 0, 0, 0, [0.12, 0.12, 0.12, 1]),
  ]
}

// Magnetic tool: cylindrical electromagnet
function magnetToolShape(
  r: number, h: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  return [
    cyl(r, h, 0, 0, 0, c),
    // Pole face
    cyl(r * 0.9, h * 0.1, 0, 0, -h * 0.45, [0.4, 0.4, 0.45, 1]),
    // Cable exit
    cyl(r * 0.15, h * 0.2, 0, r * 0.7, h * 0.4, c2),
  ]
}

// E-stop switch
function estopShape(
  cat: string,
): UrdfVisualDesc[] {
  const base = mm(20)
  return [
    box(base, base, mm(15), 0, 0, 0, catColor(cat)),
    // Red mushroom button
    cyl(mm(12), mm(8), 0, 0, mm(11), [0.9, 0.1, 0.1, 1]),
    // Yellow ring
    cyl(mm(14), mm(3), 0, 0, mm(7), [0.9, 0.85, 0.1, 1]),
  ]
}

// Omni wheel
function omniWheelShape(
  r: number, w: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  const result: UrdfVisualDesc[] = [
    // Hub plates
    cyl(r * 0.4, w, 0, 0, 0, c, Math.PI / 2, 0, 0),
  ]
  // Rollers around circumference (perpendicular)
  const n = 10
  const rollerR = r * 0.15
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2
    const rx = Math.cos(a) * r * 0.75
    const ry = Math.sin(a) * r * 0.75
    result.push(cyl(rollerR, w * 0.8, rx, ry, 0, i % 2 === 0 ? c2 : c3, 0, 0, a + Math.PI / 2))
  }
  return result
}

// Soft / compliant gripper
function softGripperShape(
  r: number, h: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const fingerR = r * 0.15
  const fingerH = h * 0.6
  const result: UrdfVisualDesc[] = [
    cyl(r * 0.5, h * 0.3, 0, 0, 0, c),
  ]
  // 4 soft fingers — tapered via stacked spheres
  for (let i = 0; i < 4; i++) {
    const a = (i / 4) * Math.PI * 2
    const fx = Math.cos(a) * r * 0.4
    const fz = Math.sin(a) * r * 0.4
    result.push(cyl(fingerR, fingerH, fx, -fingerH / 2, fz, c2))
    result.push(sphere(fingerR * 1.3, fx, -fingerH, fz, c2))
  }
  return result
}

// Vacuum pad array
function vacuumPadArrayShape(
  w: number, h: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const padR = Math.min(w, h) * 0.12
  const result: UrdfVisualDesc[] = [
    box(w, h, mm(8), 0, 0, 0, c),
  ]
  // 3x2 grid of suction pads
  for (let ix = -1; ix <= 1; ix++) {
    for (let iy = -0.5; iy <= 0.5; iy++) {
      result.push(cyl(padR, mm(5), ix * w * 0.3, iy * h * 0.4, -mm(6), c2))
    }
  }
  return result
}

// Holder shape (pen, welding torch, screwdriver bit)
function holderShape(
  r: number, h: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  return [
    // Clamp ring
    cyl(r * 1.3, h * 0.25, 0, 0, h * 0.2, c),
    // Body tube
    cyl(r, h * 0.5, 0, 0, -h * 0.1, c2),
    // Mounting flange
    box(r * 3, r * 0.4, r * 3, 0, 0, h * 0.4, c),
    // Clamping screw
    cyl(r * 0.15, r * 1.5, r * 1.2, 0, h * 0.2, catColor(cat, 2)),
  ]
}

// Corner cube connector (for extrusions)
function cornerCubeShape(
  size: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  return [box(size, size, size, 0, 0, 0, c)]
}

// DIN rail
function dinRailShape(
  w: number, h: number, length: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  return [
    // Main rail body
    box(w, h, length, 0, 0, 0, c),
    // Lip edges
    box(w * 1.1, h * 0.2, length, 0, h * 0.4, 0, c2),
    box(w * 1.1, h * 0.2, length, 0, -h * 0.4, 0, c2),
  ]
}

// Chain and sprocket
function chainSprocketShape(
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  const c3 = catColor(cat, 2)
  const sR = mm(12)
  const spacing = mm(60)
  return [
    cyl(sR, mm(6), 0, 0, 0, c),
    cyl(sR * 0.25, mm(8), 0, 0, 0, c3),
    cyl(sR * 0.7, mm(6), spacing, 0, 0, c2),
    cyl(sR * 0.7 * 0.25, mm(8), spacing, 0, 0, c3),
    // Chain (two strands)
    box(spacing, mm(1), mm(3), spacing / 2, sR * 0.9, 0, [0.2, 0.2, 0.2, 1]),
    box(spacing, mm(1), mm(3), spacing / 2, -sR * 0.9, 0, [0.2, 0.2, 0.2, 1]),
  ]
}

// Solar panel
function solarPanelShape(
  w: number, h: number,
  cat: string,
): UrdfVisualDesc[] {
  return [
    box(w, h, mm(3), 0, 0, 0, [0.1, 0.1, 0.3, 1]),
    // Cell grid lines
    box(w * 0.01, h * 0.9, mm(3.1), -w * 0.25, 0, 0, [0.5, 0.5, 0.6, 1]),
    box(w * 0.01, h * 0.9, mm(3.1), 0, 0, 0, [0.5, 0.5, 0.6, 1]),
    box(w * 0.01, h * 0.9, mm(3.1), w * 0.25, 0, 0, [0.5, 0.5, 0.6, 1]),
    box(w * 0.9, h * 0.01, mm(3.1), 0, -h * 0.25, 0, [0.5, 0.5, 0.6, 1]),
    box(w * 0.9, h * 0.01, mm(3.1), 0, h * 0.25, 0, [0.5, 0.5, 0.6, 1]),
    // Frame
    box(w * 1.02, h * 0.03, mm(4), 0, -h * 0.48, 0, catColor(cat)),
    box(w * 1.02, h * 0.03, mm(4), 0, h * 0.48, 0, catColor(cat)),
  ]
}

// Shaft collar / clamp
function shaftCollarShape(
  od: number, bore: number, h: number,
  cat: string,
): UrdfVisualDesc[] {
  const c = catColor(cat)
  const c2 = catColor(cat, 1)
  return [
    cyl(od / 2, h, 0, 0, 0, c),
    cyl(bore / 2, h * 1.01, 0, 0, 0, [0.12, 0.12, 0.12, 1]),
    // Clamping screw
    cyl(od * 0.08, od * 0.3, od / 2 + od * 0.1, 0, 0, c2, 0, Math.PI / 2, 0),
  ]
}

// ── Master dispatcher ────────────────────────────────────────────────────────

export function generateVisuals(comp: {
  id: string
  physical: { mass_kg?: number; mass_kg_per_100mm?: number; bounding_box_mm?: number[]; cross_section_mm?: number[]; inertia_primitive?: string; outer_diameter_mm?: number; inner_diameter_mm?: number; wall_thickness_mm?: number }
  mechanical_electrical: Record<string, unknown>
}, category: string): UrdfVisualDesc[] {
  _currentCompId = comp.id
  try {
  const p = comp.physical
  const bb = p.bounding_box_mm ?? p.cross_section_mm ?? [40, 40, 40]
  const xm = mm(bb[0] ?? 40)
  const ym = mm(bb[1] ?? 40)
  const zm = mm(bb[2] ?? 40)
  const id = comp.id

  // ── Actuators ──────────────────────────────────────────────────────────────
  if (id.startsWith('actuator_servo') || id.startsWith('actuator_continuous') || id.startsWith('actuator_high_speed') || id.startsWith('actuator_micro_linear')) {
    if (id.includes('linear')) return linearActuatorShape(xm, ym, zm, category)
    return servoShape(xm, zm, ym, category)
  }
  if (id.startsWith('actuator_bldc')) return bldcShape(xm / 2, zm, category)
  if (id.startsWith('actuator_stepper')) return stepperShape(xm, xm * 0.45, zm, category)
  if (id.startsWith('actuator_linear')) return linearActuatorShape(xm, ym, zm, category)

  // ── Motors ─────────────────────────────────────────────────────────────────
  if (id.startsWith('motor_harmonic')) return harmonicDriveShape(xm / 2, zm, category)
  if (id.startsWith('motor_hub')) return hubMotorShape(xm / 2, zm, category)
  if (id.startsWith('motor_gear')) return gearMotorShape(xm / 2, zm, xm * 0.7, zm * 0.4, category)
  if (id.startsWith('motor_worm')) return gearMotorShape(xm / 2, zm, xm * 0.6, zm * 0.5, category)
  if (id.startsWith('motor_dc') || id.startsWith('motor_coreless') || id.startsWith('motor_pancake') || id.startsWith('motor_brushless')) {
    return dcMotorShape(xm / 2, zm, category)
  }

  // ── Sensors ────────────────────────────────────────────────────────────────
  if (id.includes('depth_camera') || id.includes('thermal_camera') || id.includes('color_sensor')) {
    return depthCameraShape(xm, ym, zm, category)
  }
  if (id.includes('lidar')) return lidarPuckShape(xm / 2, zm, category)
  if (id.includes('ultrasonic')) return ultrasonicShape(xm, ym, zm, category)
  if (id.includes('force_torque')) return ftSensorShape(xm / 2, zm, category)
  if (id.includes('imu') || id.includes('tof') || id.includes('barometer') || id.includes('current_sensor') || id.includes('voltage')) {
    return pcbShape(xm, ym, zm, category)
  }
  if (id.includes('encoder') || id.includes('potentiometer')) {
    return bearingShape(xm, xm * 0.3, zm, category)
  }
  if (id.includes('switch') || id.includes('bumper') || id.includes('limit') || id.includes('load_cell')) {
    const c = catColor(category)
    return [box(xm, ym, zm, 0, 0, 0, c), box(xm * 0.3, ym * 0.5, zm * 0.3, xm * 0.4, 0, 0, catColor(category, 1))]
  }

  // ── Compute ────────────────────────────────────────────────────────────────
  if (id.includes('sbc_gpu') || id.includes('fpga')) return sbcShape(xm, ym, zm, true, category)
  if (id.includes('sbc') || id.includes('mcu')) return sbcShape(xm, ym, zm, false, category)
  if (id.includes('motor_driver') || id.includes('foc') || id.includes('can_') || id.includes('usb_hub') || id.includes('wireless') || id.includes('lora') || id.includes('gps')) {
    return pcbShape(xm, ym, zm, category)
  }

  // ── Power ──────────────────────────────────────────────────────────────────
  if (id.includes('lipo') || id.includes('18650') || id.includes('supercap')) return batteryShape(xm, ym, zm, category)
  if (id.includes('solar')) return solarPanelShape(xm, ym, category)
  if (id.includes('estop') || id.includes('e_stop')) return estopShape(category)
  if (id.includes('buck') || id.includes('pdu') || id.includes('usb_c_pd')) return pcbShape(xm, ym, zm, category)

  // ── Structural ─────────────────────────────────────────────────────────────
  if (id.includes('limb_link_slim')) return limbLinkSlimShape(xm, zm, ym, category)
  if (id.includes('extrusion')) {
    const profile = bb[0] ?? 20
    return extrusionShape(mm(profile), zm, category)
  }
  if (id.includes('ibeam')) {
    const flangeW = mm(bb[0] ?? 30)
    const totalH = mm(bb[1] ?? 20)
    return ibeamShape(flangeW, totalH, zm, flangeW * 0.12, totalH * 0.15, category)
  }
  if (id.includes('cchannel')) {
    return channelShape(xm, ym, zm, Math.min(xm, ym) * 0.15, category)
  }
  if (id.includes('angle_stock') || id.includes('angle_al') || id.includes('angle_steel')) {
    return angleShape(xm, ym, zm, Math.min(xm, ym) * 0.12, category)
  }
  if (id.includes('servo_side_yoke')) return servoSideYokeShape(xm, zm, ym, category)
  if (id.includes('servo_horn_beam_adapter')) return servoHornBeamAdapterShape(xm, zm, ym, category)
  if (id.includes('bracket_l')) return bracketLShape(xm, xm * 0.08, category)
  if (id.includes('bracket_u')) return bracketUShape(xm, ym, zm, xm * 0.06, category)
  if (id.includes('bracket_t') || id.includes('t_bracket')) {
    const c = catColor(category)
    return [box(xm, ym * 0.15, zm, 0, 0, 0, c), box(xm * 0.15, ym, zm, 0, ym * 0.42, 0, catColor(category, 1))]
  }
  if (id.includes('corner_cube')) return cornerCubeShape(xm, category)
  if (id.includes('cross_plate')) return plateShape(xm, ym, zm, category, true)
  if (id.includes('baseplate') || id.includes('joint_plate') || id.includes('sheet_metal') || id.includes('gusset')) {
    return plateShape(xm, ym, zm, category, true)
  }
  if (id.includes('flat_bar') || id.includes('round_bar') || id.includes('threaded_rod')) {
    if (p.inertia_primitive === 'cylinder' || id.includes('round') || id.includes('threaded')) {
      return [cyl(xm / 2, zm, 0, 0, 0, catColor(category))]
    }
    return [box(xm, ym, zm, 0, 0, 0, catColor(category))]
  }
  if (id.includes('cf_tube')) {
    if (id.includes('round')) return [cyl(mm(p.outer_diameter_mm ?? 12) / 2, zm, 0, 0, 0, catColor(category))]
    return [box(xm, ym, zm, 0, 0, 0, catColor(category))]
  }
  if (id.includes('standoff') || id.includes('hex_standoff')) return standoffShape(xm / 2, zm, category)
  if (id.includes('linear_rail_carriage')) {
    const c = catColor(category)
    return [box(xm, ym, zm, 0, 0, 0, c)]
  }
  if (id.includes('linear_rail')) return linearRailShape(xm, ym, zm, category)
  if (id.includes('din_rail')) return dinRailShape(xm, ym, zm, category)
  if (id.includes('pillow_block')) return pillowBlockShape(mm(4), xm, category)
  if (id.includes('shaft_collar')) return shaftCollarShape(xm, mm(p.inner_diameter_mm ?? 8) ?? xm * 0.5, zm, category)

  // ── Transmission ───────────────────────────────────────────────────────────
  if (id.includes('timing_belt')) return beltDriveShape(category)
  if (id.includes('leadscrew') || id.includes('ballscrew')) return leadscrewShape(mm(p.outer_diameter_mm ?? bb[0] ?? 8) / 2, zm, category)
  if (id.includes('bearing_deep') || id.includes('bearing_large') || id.includes('crossed_roller')) {
    const od = mm((comp.mechanical_electrical.outer_diameter_mm as number) ?? bb[0] ?? 22)
    const bore = mm((comp.mechanical_electrical.bore_mm as number) ?? 8)
    const w = mm((comp.mechanical_electrical.width_mm as number) ?? bb[2] ?? 7)
    return bearingShape(od, bore, w, category)
  }
  if (id.includes('slewing_ring')) return slewingRingShape(xm, zm, category)
  if (id.includes('bushing')) return bearingShape(xm, xm * 0.55, zm, category)
  if (id.includes('planetary')) return harmonicDriveShape(xm / 2, zm, category)
  if (id.includes('spur_gear') || id.includes('bevel_gear')) return gearPairShape(mm(12), mm(8), mm(6), category)
  if (id.includes('worm_gear_set')) return wormGearShape(category)
  if (id.includes('chain_sprocket')) return chainSprocketShape(category)
  if (id.includes('universal_joint') || id.includes('u_joint')) return ujointShape(xm / 2, category)
  if (id.includes('flexible_coupling') || id.includes('rigid_coupling')) return couplingShape(xm / 2, zm, category)
  if (id.includes('rack_pinion')) return rackPinionShape(category)

  // ── End Effectors ──────────────────────────────────────────────────────────
  if (id.includes('parallel_gripper')) {
    const opening = mm((comp.mechanical_electrical.max_opening_mm as number) ?? 40)
    return parallelGripperShape(xm, ym, zm, xm * 0.12, ym * 0.4, opening, category)
  }
  if (id.includes('3finger') || id.includes('adaptive')) return threeFingerGripperShape(xm / 2, ym, xm * 0.1, ym * 0.35, category)
  if (id.includes('soft_gripper') || id.includes('compliant')) return softGripperShape(xm / 2, ym, category)
  if (id.includes('suction')) return suctionCupShape(xm / 2, ym, category)
  if (id.includes('vacuum_pad')) return vacuumPadArrayShape(xm, ym, category)
  if (id.includes('magnetic_tool') || id.includes('electromagnetic')) return magnetToolShape(xm / 2, ym, category)
  if (id.includes('tool_changer')) return toolChangerShape(xm / 2, ym, category)
  if (id.includes('holder') || id.includes('welding') || id.includes('pen_marker') || id.includes('screwdriver')) return holderShape(xm / 2, ym, category)

  // ── Mobility ───────────────────────────────────────────────────────────────
  if (id.includes('mecanum')) return mecanumWheelShape(xm / 2, zm, category)
  if (id.includes('omni_wheel')) return omniWheelShape(xm / 2, zm, category)
  if (id.includes('caster')) return casterShape(xm / 2, ym, category)
  if (id.includes('wheel_driven') || id.includes('driven_wheel')) return wheelShape(xm / 2, zm, category)
  if (id.includes('track') || id.includes('tread')) return trackShape(xm, ym, zm, category)
  if (id.includes('swerve')) return swerveDriveShape(category)
  if (id.includes('ball_transfer')) return ballTransferShape(category)
  if (id.includes('rubber_foot') || id.includes('rubber_pad')) return rubberFootShape(category)

  // ── Fallback: basic primitive ──────────────────────────────────────────────
  const shape = p.inertia_primitive || 'box'
  const c = catColor(category)
  if (shape === 'cylinder') return [cyl(Math.max(xm, ym) / 2, zm, 0, 0, 0, c)]
  if (shape === 'sphere') return [sphere(Math.max(xm, ym, zm) / 2, 0, 0, 0, c)]
  return [box(xm, ym, zm, 0, 0, 0, c)]
  } finally {
    _currentCompId = null
  }
}

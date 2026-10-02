/**
 * Procedural models for hardware that has no usable CAD mesh (or whose mesh
 * was the wrong part/proportions): motors and actuators, drive units, brackets,
 * gears and couplings, sensors and power modules.
 *
 * Authored directly in the catalog's URDF frame (Z-up, output shaft / jaw
 * opening along +Z, centered on the bbox) and handed back rotated into the
 * Y-up convention the rich-visual registry expects.
 */
import * as THREE from 'three'
import type { ComponentVisualDims } from './index'
import { getMaterial } from '../materials'

const alu = () => getMaterial('anodized_aluminum', 0xb4bcc6)
const darkAlu = () => getMaterial('anodized_aluminum', 0x3a3f46)
const steel = () => getMaterial('brushed_steel', 0xa0a4ab)
const black = () => getMaterial('matte_plastic', 0x1c1d20)
const rubber = () => getMaterial('rubber_black', 0x141414)
const accent = () => getMaterial('anodized_aluminum', 0xc0392b)

/** Cylinder along Z (three.js cylinders run along Y). */
function zCyl(r: number, h: number, z: number, mat: THREE.Material, seg = 32, x = 0, y = 0): THREE.Mesh {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, seg), mat)
  m.rotation.x = Math.PI / 2
  m.position.set(x, y, z)
  return m
}

/** Cylinder along Y. */
function yCyl(r: number, h: number, x: number, z: number, mat: THREE.Material, seg = 32, y = 0): THREE.Mesh {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, seg), mat)
  m.position.set(x, y, z)
  return m
}

/** Cylinder along X. */
function xCyl(r: number, h: number, x: number, y: number, z: number, mat: THREE.Material, seg = 32): THREE.Mesh {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, seg), mat)
  m.rotation.z = Math.PI / 2
  m.position.set(x, y, z)
  return m
}

function box(sx: number, sy: number, sz: number, x: number, y: number, z: number, mat: THREE.Material): THREE.Mesh {
  const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), mat)
  m.position.set(x, y, z)
  return m
}

function toYUp(zUp: THREE.Group): THREE.Group {
  zUp.rotation.x = -Math.PI / 2
  const g = new THREE.Group()
  g.add(zUp)
  return g
}

function bldcOutrunner(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const R = Math.min(d.x, d.y) / 2, H = d.z, z0 = -H / 2
  g.add(zCyl(R * 0.82, H * 0.16, z0 + H * 0.08, black()))                 // stator mount
  g.add(zCyl(R, H * 0.62, z0 + H * 0.16 + H * 0.31, alu(), 48))            // rotor bell
  for (let i = 0; i < 12; i++) {                                           // bell vents
    const a = (i / 12) * Math.PI * 2
    const v = box(R * 0.12, R * 0.05, H * 0.34, Math.cos(a) * R * 0.99, Math.sin(a) * R * 0.99, z0 + H * 0.47, black())
    v.rotation.z = a
    g.add(v)
  }
  g.add(zCyl(R * 0.7, H * 0.08, z0 + H * 0.82, accent(), 48))              // output flange
  g.add(zCyl(R * 0.16, H * 0.14, z0 + H * 0.93, steel(), 20))               // shaft
  return toYUp(g)
}

function hubMotor(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const R = Math.min(d.x, d.y) / 2, H = d.z, z0 = -H / 2
  g.add(zCyl(R * 0.97, H * 0.7, z0 + H * 0.45, darkAlu(), 48))             // drum
  g.add(zCyl(R, H * 0.1, z0 + H * 0.05, alu(), 48))                        // inner rim
  g.add(zCyl(R, H * 0.1, z0 + H * 0.85, alu(), 48))                        // outer rim
  for (let i = 0; i < 6; i++) {                                            // wheel bolts
    const a = (i / 6) * Math.PI * 2
    g.add(zCyl(R * 0.06, H * 0.08, z0 + H * 0.94, steel(), 12, Math.cos(a) * R * 0.6, Math.sin(a) * R * 0.6))
  }
  g.add(zCyl(R * 0.14, H * 0.1, z0 + H * 0.95, steel(), 20))               // axle nut
  return toYUp(g)
}

function gearmotorDrive(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const W = Math.min(d.x, d.y), H = d.z, z0 = -H / 2
  g.add(zCyl(W * 0.33, H * 0.46, z0 + H * 0.23, black(), 32))              // motor can
  g.add(zCyl(W * 0.4, H * 0.24, z0 + H * 0.58, steel(), 32))               // gearbox
  g.add(box(W, W, H * 0.05, 0, 0, z0 + H * 0.725, alu()))                   // mounting flange
  g.add(zCyl(W * 0.07, H * 0.12, z0 + H * 0.81, steel(), 16))              // shaft
  g.add(zCyl(W * 0.17, H * 0.12, z0 + H * 0.93, alu(), 24))                // coupler
  return toYUp(g)
}

function stubAxle(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const W = Math.min(d.x, d.y), H = d.z, z0 = -H / 2
  g.add(box(W, W, H * 0.18, 0, 0, z0 + H * 0.09, alu()))                    // flange
  g.add(zCyl(W * 0.3, H * 0.5, z0 + H * 0.43, darkAlu(), 32))              // bearing housing
  g.add(zCyl(W * 0.1, H * 0.32, z0 + H * 0.84, steel(), 16))               // axle
  return toYUp(g)
}

function casterFork(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const W = Math.min(d.x, d.y), H = d.z, z0 = -H / 2
  g.add(box(W, W, H * 0.06, 0, 0, H / 2 - H * 0.03, alu()))                 // top plate
  g.add(zCyl(W * 0.3, H * 0.1, H / 2 - H * 0.11, steel(), 32))             // swivel bearing
  g.add(box(W * 0.5, W * 0.7, H * 0.08, -W * 0.1, 0, H / 2 - H * 0.2, darkAlu()))  // fork crown
  for (const s of [-1, 1]) g.add(box(W * 0.35, W * 0.06, H * 0.75, -W * 0.15, s * W * 0.32, z0 + H * 0.4, darkAlu()))
  const pin = new THREE.Mesh(new THREE.CylinderGeometry(W * 0.05, W * 0.05, W * 0.72, 12), steel())
  pin.position.set(-W * 0.2, 0, z0 + H * 0.12)
  g.add(pin)
  return toYUp(g)
}

function steeringKnuckle(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const W = Math.min(d.x, d.y), H = d.z, z0 = -H / 2
  const kp = new THREE.Mesh(new THREE.CylinderGeometry(W * 0.12, W * 0.12, W * 0.9, 16), steel())  // kingpin along Y
  kp.position.set(-W * 0.25, 0, z0 + H * 0.3)
  g.add(kp)
  g.add(box(W * 0.45, W * 0.45, H * 0.45, 0, 0, z0 + H * 0.3, darkAlu()))  // knuckle body
  g.add(box(W * 0.5, W * 0.15, H * 0.12, W * 0.25, 0, z0 + H * 0.12, alu())) // steering arm
  g.add(zCyl(W * 0.26, H * 0.12, z0 + H * 0.58, alu(), 32))                // hub flange
  g.add(zCyl(W * 0.1, H * 0.3, z0 + H * 0.8, steel(), 16))                 // spindle
  return toYUp(g)
}

/** Track module: belt loop along X around two sprockets, width along Y. */
function trackModule(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const L = d.x, W = d.y, H = d.z
  const r = H / 2, t = H * 0.12, straight = L - 2 * r
  for (const s of [-1, 1]) g.add(box(straight, W, t, 0, 0, s * (r - t / 2), rubber()))  // belt runs
  for (const s of [-1, 1]) {                                                // wrap around sprockets
    const outer = new THREE.Mesh(new THREE.CylinderGeometry(r, r, W, 40, 1, true, 0, Math.PI), rubber())
    outer.rotation.y = s > 0 ? 0 : Math.PI
    outer.position.set(s * straight / 2, 0, 0)
    const holder = new THREE.Group()
    holder.add(outer)
    g.add(holder)
    g.add(yCyl(r - t, W * 0.7, s * straight / 2, 0, darkAlu(), 24))        // sprocket / idler
    g.add(yCyl(r * 0.25, W * 0.9, s * straight / 2, 0, steel(), 12))       // hub
  }
  const lugs = Math.max(6, Math.round(straight / (H * 0.35)))
  for (let i = 0; i < lugs; i++) {                                          // tread lugs
    const x = -straight / 2 + (i + 0.5) * straight / lugs
    for (const s of [-1, 1]) g.add(box(t * 0.9, W, t * 0.5, x, 0, s * (r + t * 0.25 - t * 0.5), rubber()))
  }
  const roadR = (r - t) * 0.55
  for (const f of [-0.25, 0, 0.25]) g.add(yCyl(roadR, W * 0.6, f * straight, -r + t + roadR, alu(), 20))  // road wheels
  g.add(box(straight, W * 0.2, H * 0.55, 0, W * 0.4, 0, darkAlu()))        // inner side plate
  return toYUp(g)
}

/** Parallel gripper: mount flange at -Z, palm, two jaws opening toward +Z. */
function parallelGripper(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const X = d.x, Y = d.y, H = d.z, z0 = -H / 2
  g.add(zCyl(Math.min(X, Y) * 0.32, H * 0.1, z0 + H * 0.05, steel(), 32))       // mount flange
  g.add(box(X, Y * 0.55, H * 0.38, 0, 0, z0 + H * 0.29, darkAlu()))                // palm / actuator body
  g.add(box(X * 0.9, Y * 0.2, H * 0.06, 0, 0, z0 + H * 0.51, steel()))             // jaw rail
  for (const s of [-1, 1]) {
    g.add(box(X * 0.14, Y * 0.5, H * 0.46, s * X * 0.3, 0, z0 + H * 0.77, alu()))  // jaw
    g.add(box(X * 0.04, Y * 0.44, H * 0.3, s * X * 0.21, 0, z0 + H * 0.82, black())) // finger pad
  }
  return toYUp(g)
}

/** Electric linear actuator: rear clevis at -Z, motor/gear housing, body tube, polished rod and rod-end eye reaching +Z (rod_out). */
function linearActuator(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const R = Math.min(d.x, d.y) / 2, H = d.z, z0 = -H / 2
  const eye = Math.min(R * 0.9, H * 0.05)
  const bodyLen = H * 0.58 - 2 * eye
  g.add(yCyl(eye * 0.8, R * 0.9, 0, z0 + eye, steel(), 20))                          // rear clevis pin boss
  g.add(box(R * 1.1, R * 0.9, eye * 1.4, 0, 0, z0 + eye * 1.7, darkAlu()))            // rear clevis block
  g.add(zCyl(R, bodyLen, z0 + eye * 2.4 + bodyLen / 2, darkAlu(), 40))                // outer tube
  g.add(zCyl(R * 1.02, bodyLen * 0.28, z0 + eye * 2.4 + bodyLen * 0.14, black(), 40)) // gear housing band
  g.add(zCyl(R * 1.04, H * 0.012, z0 + eye * 2.4 + bodyLen, black(), 40))            // front seal cap
  const rodStart = z0 + eye * 2.4 + bodyLen
  const rodLen = H / 2 - rodStart - eye * 2
  g.add(zCyl(R * 0.36, rodLen, rodStart + rodLen / 2, steel(), 24))                  // extension rod
  g.add(zCyl(R * 0.5, eye * 1.2, H / 2 - eye * 1.4, steel(), 24))                    // rod end
  g.add(yCyl(eye * 0.8, R * 0.7, 0, H / 2 - eye * 0.8, steel(), 20))                 // rod-end eye
  return toYUp(g)
}

/** Strain-wave (harmonic) drive actuator: flat housing with cooling fins, output flange with bolt circle on +Z. */
function harmonicDrive(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const R = Math.min(d.x, d.y) / 2, H = d.z, z0 = -H / 2
  g.add(zCyl(R * 0.92, H * 0.18, z0 + H * 0.09, black(), 48))                         // rear motor cap
  g.add(zCyl(R, H * 0.5, z0 + H * 0.43, darkAlu(), 64))                               // housing
  for (let i = 0; i < 4; i++) g.add(zCyl(R * 1.015, H * 0.03, z0 + H * (0.24 + i * 0.1), alu(), 64))  // fins
  g.add(zCyl(R * 0.97, H * 0.1, z0 + H * 0.73, alu(), 64))                             // crossed-roller ring
  g.add(zCyl(R * 0.62, H * 0.18, z0 + H * 0.87, steel(), 48))                          // output flange
  for (let i = 0; i < 8; i++) {                                                       // flange bolts
    const a = (i / 8) * Math.PI * 2
    g.add(zCyl(R * 0.045, H * 0.03, H / 2 - H * 0.01, black(), 10, Math.cos(a) * R * 0.48, Math.sin(a) * R * 0.48))
  }
  g.add(zCyl(R * 0.16, H * 0.03, H / 2 - H * 0.01, black(), 24))                      // hollow bore
  return toYUp(g)
}

/** Brushless inrunner: plain can, rear end bell with leads, front bell, shaft on +Z. */
function inrunnerMotor(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const R = Math.min(d.x, d.y) / 2, H = d.z, z0 = -H / 2
  const shaft = H * 0.14
  g.add(zCyl(R * 0.9, H * 0.08, z0 + H * 0.04, black(), 40))                          // rear bell
  g.add(zCyl(R, H * 0.66, z0 + H * 0.41, steel(), 48))                                // can
  g.add(zCyl(R * 1.005, H * 0.18, z0 + H * 0.4, getMaterial('anodized_aluminum', 0x2d5fa0), 48)) // label band
  g.add(zCyl(R * 0.94, H * 0.1, z0 + H * 0.79, alu(), 40))                             // front bell
  g.add(zCyl(R * 0.3, H * 0.02, z0 + H * 0.85, black(), 24))                           // bearing seal
  g.add(zCyl(R * 0.14, shaft, H / 2 - shaft / 2, steel(), 16))                         // shaft
  for (const [x, c] of [[-0.3, 0x111111], [0, 0xc0392b], [0.3, 0xf1c40f]] as const)   // phase leads
    g.add(zCyl(R * 0.07, H * 0.06, z0 - H * 0.0 + H * 0.03, getMaterial('matte_plastic', c), 8, x * R, -R * 0.55))
  return toYUp(g)
}

/** N20-style micro gearmotor: can motor below, square gearbox above, D-shaft on +Z. */
function microGearmotor(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const X = d.x, Y = d.y, H = d.z, z0 = -H / 2
  const shaft = H * 0.2, gear = H * 0.2, motor = H - shaft - gear
  g.add(box(X * 0.9, Y * 0.62, H * 0.03, 0, 0, z0 + H * 0.015, black()))               // end cap / terminals
  for (const s of [-1, 1]) g.add(box(X * 0.08, Y * 0.08, H * 0.05, s * X * 0.25, 0, z0 - H * 0.0 + H * 0.01, getMaterial('brushed_steel', 0xc9a227)))
  const can = zCyl(Math.min(X, Y) * 0.5, motor, z0 + motor / 2 + H * 0.02, steel(), 32)
  can.scale.set(X / Math.min(X, Y), Y / Math.min(X, Y), 1)                            // flattened can
  g.add(can)
  g.add(box(X, Y, gear, 0, 0, z0 + motor + gear / 2, getMaterial('brushed_steel', 0xc9a227)))  // brass gearbox
  g.add(box(X * 0.96, Y * 0.96, gear * 0.08, 0, 0, z0 + motor + gear * 0.35, black()))        // plate seam
  g.add(zCyl(Math.min(X, Y) * 0.15, shaft, H / 2 - shaft / 2, steel(), 16))           // D-shaft
  return toYUp(g)
}

const pcb = () => getMaterial('matte_plastic', 0x1f6f3f)
const gold = () => getMaterial('brushed_steel', 0xc9a227)

/** Absolute magnetic encoder: flat round housing, through-bore on the axis, cable gland. */
function jointEncoder(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const R = Math.min(d.x, d.y) / 2, H = d.z, z0 = -H / 2
  g.add(zCyl(R, H * 0.8, z0 + H * 0.4, darkAlu(), 48))                                 // housing
  g.add(zCyl(R * 0.8, H * 0.2, z0 + H * 0.9, alu(), 48))                               // top cover
  g.add(zCyl(R * 0.3, H * 0.22, z0 + H * 0.9, black(), 24))                            // bore / magnet hub
  g.add(zCyl(R * 0.12, H * 0.24, z0 + H * 0.9, steel(), 16))                           // shaft end
  g.add(xCyl(H * 0.18, R * 0.2, R * 0.9, 0, z0 + H * 0.4, black(), 12))                // cable gland
  return toYUp(g)
}

/** Panel potentiometer: square-ish body, threaded bushing and knurled shaft on +Z, solder lugs. */
function potentiometer(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const R = Math.min(d.x, d.y) / 2, H = d.z, z0 = -H / 2
  g.add(zCyl(R, H * 0.45, z0 + H * 0.225, getMaterial('brushed_steel', 0x8f959c), 40)) // can
  g.add(zCyl(R * 0.42, H * 0.18, z0 + H * 0.54, gold(), 24))                           // threaded bushing
  g.add(zCyl(R * 0.2, H * 0.37, H / 2 - H * 0.185, steel(), 20))                        // shaft
  for (const x of [-0.5, 0, 0.5]) g.add(box(R * 0.14, R * 0.05, H * 0.2, x * R, -R * 1.02, z0 + H * 0.1, gold()))  // lugs
  return toYUp(g)
}

/** Servo side yoke: U cradle — base plate at -Z, two slotted cheeks at ±X, open toward +Z. */
function servoYoke(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const X = d.x, Y = d.y, H = d.z, z0 = -H / 2
  const t = Math.max(0.002, Math.min(X, Y, H) * 0.08)
  g.add(box(X, Y, t, 0, 0, z0 + t / 2, alu()))                                          // base
  for (const s of [-1, 1]) {
    g.add(box(t, Y, H - t, s * (X / 2 - t / 2), 0, z0 + t + (H - t) / 2, alu()))       // cheek
    g.add(xCyl(Y * 0.12, t * 1.1, s * (X / 2 - t / 2), 0, z0 + H * 0.62, black(), 20))  // bearing boss
  }
  for (const sx of [-1, 1]) for (const sy of [-1, 1])
    g.add(zCyl(t * 0.6, t * 1.1, z0 + t / 2, steel(), 12, sx * X * 0.28, sy * Y * 0.3)) // base bolts
  return toYUp(g)
}

/** Servo horn beam adapter: round spline hub at -Z (bolts to the horn), beam clamp plate on +Z. */
function hornAdapter(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const X = d.x, Y = d.y, H = d.z, z0 = -H / 2
  const plate = H * 0.4
  g.add(zCyl(Math.min(X, Y) * 0.42, H - plate, z0 + (H - plate) / 2, alu(), 40))      // hub
  g.add(zCyl(Math.min(X, Y) * 0.12, (H - plate) * 1.02, z0 + (H - plate) / 2, black(), 20)) // spline bore
  g.add(box(X, Y, plate, 0, 0, H / 2 - plate / 2, alu()))                              // clamp plate
  for (const sx of [-1, 1]) for (const sy of [-1, 1])
    g.add(zCyl(Math.min(X, Y) * 0.06, plate * 1.05, H / 2 - plate / 2, steel(), 12, sx * X * 0.36, sy * Y * 0.3))
  return toYUp(g)
}

/** Supercapacitor bank: row of cans standing on a balancing PCB. */
function supercapModule(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const X = d.x, Y = d.y, H = d.z, z0 = -H / 2
  const t = Math.min(0.0016, H * 0.1)
  g.add(box(X, Y, t, 0, 0, z0 + t / 2, pcb()))
  const n = Math.max(2, Math.round(X / Y * 1.8))
  const cols = Math.ceil(n / 2), r = Math.min(X / cols, Y / 2) * 0.46
  for (let i = 0; i < n; i++) {
    const cx = -X / 2 + (Math.floor(i / 2) + 0.5) * (X / cols), cy = (i % 2 ? -1 : 1) * Y * 0.25
    g.add(zCyl(r, H - t, z0 + t + (H - t) / 2, getMaterial('matte_plastic', 0x2d5fa0), 32, cx, cy))      // can sleeve
    g.add(zCyl(r * 0.96, H * 0.02, H / 2 - H * 0.005, getMaterial('brushed_steel', 0xcfd3d8), 32, cx, cy)) // top
  }
  return toYUp(g)
}

/** 18650 pack: cells lying along X in shrink wrap, nickel strips, balance lead. */
function cellPack(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const X = d.x, Y = d.y, H = d.z
  const ny = Math.max(1, Math.round(Y / 0.02)), nz = Math.max(1, Math.round(H / 0.02))   // ~18650 cells (dims in m)
  const r = Math.min(Y / ny, H / nz) / 2 * 0.97
  for (let iy = 0; iy < ny; iy++) for (let iz = 0; iz < nz; iz++) {
    const cy = -Y / 2 + (iy + 0.5) * (Y / ny), cz = -H / 2 + (iz + 0.5) * (H / nz)
    g.add(xCyl(r, X * 0.94, 0, cy, cz, getMaterial('matte_plastic', 0x2d6fb5), 24))
  }
  for (const s of [-1, 1]) g.add(box(X * 0.03, Y * 0.95, H * 0.95, s * X * 0.485, 0, 0, getMaterial('brushed_steel', 0xcfd3d8)))  // nickel strips
  g.add(box(X * 0.12, Y * 0.2, H * 0.1, X * 0.3, 0, H / 2 - H * 0.05, getMaterial('matte_plastic', 0xf5f5f5)))   // balance connector
  return toYUp(g)
}

/** L bracket filling its bbox: plate along the -Z face, wall up the -X face, holes through both. */
function lBracket(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const X = d.x, Y = d.y, H = d.z
  const t = Math.min(X, Y, H) * 0.08
  g.add(box(X, Y, t, 0, 0, -H / 2 + t / 2, alu()))                                     // plate
  g.add(box(t, Y, H - t, -X / 2 + t / 2, 0, t / 2, alu()))                            // wall
  const gus = new THREE.Mesh(new THREE.BoxGeometry(t * 0.8, t * 0.8, Math.hypot(X, H) * 0.35), alu())
  gus.rotation.y = Math.PI / 4
  for (const s of [-1, 1]) {                                                         // side gussets
    const m = gus.clone()
    m.position.set(-X / 2 + X * 0.18, s * (Y / 2 - t * 0.4), -H / 2 + H * 0.18)
    g.add(m)
  }
  const hr = Math.min(X, Y) * 0.06
  for (const s of [-1, 1]) {
    g.add(zCyl(hr, t * 1.05, -H / 2 + t / 2, black(), 16, X * 0.18, s * Y * 0.25))     // plate holes
    g.add(xCyl(hr, t * 1.05, -X / 2 + t / 2, s * Y * 0.25, H * 0.18, black(), 16))     // wall holes
  }
  return toYUp(g)
}

/** Spur-gear outline in XY (trapezoid teeth), extruded along Z and centred. */
function gearGeometry(r: number, teeth: number, thick: number, bore = 0): THREE.BufferGeometry {
  const shape = new THREE.Shape()
  const add = 1.1 * (2 * Math.PI * r / teeth) / Math.PI, root = r - add, tip = r + add * 0.25
  const da = (Math.PI * 2) / teeth
  for (let i = 0; i < teeth; i++) {
    const a = i * da
    const pts: Array<[number, number]> = [[root, a], [root, a + da * 0.2], [tip, a + da * 0.35], [tip, a + da * 0.55], [root, a + da * 0.7]]
    pts.forEach(([rr, aa], k) => {
      const x = rr * Math.cos(aa), y = rr * Math.sin(aa)
      if (i === 0 && k === 0) shape.moveTo(x, y)
      else shape.lineTo(x, y)
    })
  }
  shape.closePath()
  if (bore > 0) {
    const hole = new THREE.Path()
    hole.absarc(0, 0, bore, 0, Math.PI * 2, true)
    shape.holes.push(hole)
  }
  const geo = new THREE.ExtrudeGeometry(shape, { depth: thick, bevelEnabled: false, curveSegments: 8 })
  geo.translate(0, 0, -thick / 2)
  return geo
}

function gear(r: number, teeth: number, thick: number, z: number, mat: THREE.Material, x = 0, y = 0): THREE.Mesh {
  const m = new THREE.Mesh(gearGeometry(r, teeth, thick, r * 0.18), mat)
  m.position.set(x, y, z)
  return m
}

/** Meshing spur gear pair side by side along X, axes on Z. */
function spurPair(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const X = d.x, Y = d.y, H = d.z
  const big = Math.min(Y / 2, X * 0.3)
  const R1 = big * 0.9, R2 = (X / 2 - big) * 0.9
  const x1 = -X / 2 + big, x2 = x1 + R1 + R2
  g.add(gear(R1, Math.max(12, Math.round(R1 * 1000 / 1.2)), H * 0.7, 0, steel(), x1))
  g.add(gear(R2, Math.max(8, Math.round(R2 * 1000 / 1.2)), H * 0.7, 0, alu(), x2))
  g.add(zCyl(R1 * 0.28, H, 0, darkAlu(), 20, x1))                                      // hubs
  g.add(zCyl(R2 * 0.28, H, 0, darkAlu(), 20, x2))
  return toYUp(g)
}

/** Bevel gear pair: one gear on Z, its mate on X, meeting at 90°. */
function bevelPair(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const S = Math.min(d.x, d.y, d.z), r = S * 0.34, t = S * 0.18
  // Built along Y (three.js cylinder axis), then turned into place.
  const cone = (mat: THREE.Material): THREE.Group => {
    const c = new THREE.Group()
    c.add(new THREE.Mesh(new THREE.CylinderGeometry(r * 0.55, r, t, 40), mat))
    const ring = new THREE.Mesh(gearGeometry(r * 0.95, 20, t * 0.45), mat)
    ring.rotation.x = Math.PI / 2
    ring.position.y = -t * 0.2
    c.add(ring)
    const hub = new THREE.Mesh(new THREE.CylinderGeometry(r * 0.3, r * 0.3, S * 0.3, 20), darkAlu())
    hub.position.y = -S * 0.15
    c.add(hub)
    return c
  }
  const a = cone(steel())
  a.rotation.x = -Math.PI / 2                                                          // axis Z, narrow end down
  a.position.set(-S * 0.08, 0, S * 0.2)
  g.add(a)
  const b = cone(alu())
  b.rotation.z = Math.PI / 2                                                           // axis X, narrow end toward -X
  b.position.set(S * 0.2, 0, -S * 0.08)
  g.add(b)
  return toYUp(g)
}

/** Worm set: worm wheel on Z, worm screw along Y tangent to it. */
function wormSet(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const X = d.x, Y = d.y, H = d.z
  const R = Math.min(X, Y) * 0.33, rw = Math.min(H * 0.2, R * 0.35)
  const cx = -X / 2 + R * 1.05, wz = -H * 0.15
  g.add(gear(R, 30, H * 0.3, wz, steel(), cx))
  g.add(zCyl(R * 0.25, H * 0.6, wz + H * 0.1, darkAlu(), 20, cx))                      // wheel hub / shaft
  const wx = cx + R + rw * 0.8
  g.add(yCyl(rw, Y * 0.8, wx, wz, alu(), 24))                                          // worm core
  for (let i = 0; i < 9; i++) {
    const ring = new THREE.Mesh(new THREE.TorusGeometry(rw * 1.05, rw * 0.2, 8, 24), alu())
    ring.rotation.x = Math.PI / 2
    ring.rotation.z = 0.12
    ring.position.set(wx, -Y * 0.3 + i * Y * 0.075, wz)
    g.add(ring)                                                                        // thread flights
  }
  g.add(yCyl(rw * 0.35, Y, wx, wz, steel(), 12))                                       // worm shaft
  return toYUp(g)
}

/** Universal joint: yoke on each end (shaft_in -Z, shaft_out +Z) around a spider cross. */
function uJoint(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const R = Math.min(d.x, d.y) / 2, H = d.z
  const hub = H * 0.28
  g.add(zCyl(R * 0.7, hub, -H / 2 + hub / 2, steel(), 24))                             // lower hub
  g.add(zCyl(R * 0.7, hub, H / 2 - hub / 2, steel(), 24))                              // upper hub
  for (const s of [-1, 1]) {
    g.add(box(R * 0.35, R * 0.5, H * 0.34, s * R * 0.78, 0, -H * 0.1, steel()))        // lower yoke ears (X)
    g.add(box(R * 0.5, R * 0.35, H * 0.34, 0, s * R * 0.78, H * 0.1, steel()))         // upper yoke ears (Y)
  }
  g.add(xCyl(R * 0.2, R * 1.9, 0, 0, 0, darkAlu(), 16))                                // spider pin X
  g.add(yCyl(R * 0.2, R * 1.9, 0, 0, darkAlu(), 16))                                   // spider pin Y
  g.add(zCyl(R * 0.32, R * 0.45, 0, darkAlu(), 16))                                    // spider centre
  return toYUp(g)
}

/** Lead / ball screw: threaded rod along Z with a nut riding on it. */
function screwAssembly(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const R = Math.min(d.x, d.y) / 2, H = d.z
  g.add(zCyl(R * 0.8, H, 0, steel(), 24))                                              // core
  const pitch = Math.max(R * 0.5, 0.002)
  const n = Math.min(300, Math.floor(H / pitch))
  for (let i = 0; i < n; i++) {
    const ring = new THREE.Mesh(new THREE.TorusGeometry(R * 0.84, R * 0.16, 6, 20), steel())
    ring.position.z = -H / 2 + (i + 0.5) * pitch
    g.add(ring)                                                                         // thread crests
  }
  g.add(zCyl(R * 1.2, Math.min(H * 0.18, R * 3), H * 0.1, getMaterial('brushed_steel', 0xc9a227), 24))  // nut
  return toYUp(g)
}

/** T bracket: flat T-shaped plate (bar along X at +Y, stem down the middle), with holes. */
function tPlate(d: ComponentVisualDims): THREE.Group {
  const g = new THREE.Group()
  const X = d.x, Y = d.y, H = d.z
  const bar = Y * 0.34, stem = X * 0.36
  g.add(box(X, bar, H, 0, Y / 2 - bar / 2, 0, alu()))
  g.add(box(stem, Y - bar, H, 0, -bar / 2, 0, alu()))
  const hr = Math.min(stem, bar) * 0.16
  for (const x of [-X * 0.35, 0, X * 0.35]) g.add(zCyl(hr, H * 1.05, 0, black(), 14, x, Y / 2 - bar / 2))
  for (const y of [-Y * 0.3, -Y * 0.05]) g.add(zCyl(hr, H * 1.05, 0, black(), 14, 0, y))
  return toYUp(g)
}

const GENERATORS: Array<[RegExp, (d: ComponentVisualDims) => THREE.Group]> = [
  [/^actuator_bldc|^motor_pancake/, bldcOutrunner],
  [/^motor_brushless_inrunner/, inrunnerMotor],
  [/^actuator_linear_(small|heavy)$/, linearActuator],
  [/^motor_harmonic_drive/, harmonicDrive],
  [/^motor_gear_small_n20$/, microGearmotor],
  [/^sensor_joint_encoder/, jointEncoder],
  [/^sensor_rotary_potentiometer/, potentiometer],
  [/^structural_servo_side_yoke_mount$/, servoYoke],
  [/^structural_servo_horn_beam_adapter$/, hornAdapter],
  [/^structural_bracket_l$/, lBracket],
  [/^structural_t_bracket$/, tPlate],
  [/^power_supercapacitor_module$/, supercapModule],
  [/^power_18650_4s2p_pack$/, cellPack],
  [/^transmission_spur_gear_pair$/, spurPair],
  [/^transmission_bevel_gear_pair$/, bevelPair],
  [/^transmission_worm_gear_set$/, wormSet],
  [/^transmission_universal_joint$/, uJoint],
  [/^transmission_(lead|ball)screw/, screwAssembly],
  [/^motor_hub|^drivetrain_hub_motor/, hubMotor],
  [/^drivetrain_geared_dc/, gearmotorDrive],
  [/^drivetrain_stub_axle/, stubAxle],
  [/^drivetrain_caster_swivel/, casterFork],
  [/^drivetrain_steering_knuckle/, steeringKnuckle],
  [/^mobility_track_tread/, trackModule],
  [/^effector_parallel_gripper/, parallelGripper],
]

export function findHardwareGenerator(id: string): ((id: string, dims: ComponentVisualDims) => THREE.Group) | null {
  for (const [re, gen] of GENERATORS) if (re.test(id)) return (_id, dims) => gen(dims)
  return null
}

import * as THREE from 'three'

// ── Shared materials ─────────────────────────────────────────────────────────

const aluminum    = new THREE.MeshStandardMaterial({ color: 0xc4c8dc, roughness: 0.25, metalness: 0.80 })
const steel       = new THREE.MeshStandardMaterial({ color: 0x5a6478, roughness: 0.38, metalness: 0.85 })
const servoBody   = new THREE.MeshStandardMaterial({ color: 0x364566, roughness: 0.48, metalness: 0.65 })
const servoHorn   = new THREE.MeshStandardMaterial({ color: 0xff8c55, roughness: 0.38, metalness: 0.25, emissive: 0x331500, emissiveIntensity: 0.4 })
const sensorShell = new THREE.MeshStandardMaterial({ color: 0x1e3050, roughness: 0.52, metalness: 0.30 })
const lensMat     = new THREE.MeshStandardMaterial({ color: 0x3399ff, roughness: 0.06, metalness: 0.05, transparent: true, opacity: 0.80 })
const pcb         = new THREE.MeshStandardMaterial({ color: 0x1a3a1a, roughness: 0.65, metalness: 0.15 })
const rubber      = new THREE.MeshStandardMaterial({ color: 0x282828, roughness: 0.95, metalness: 0.00 })
const battery     = new THREE.MeshStandardMaterial({ color: 0x1e2d4a, roughness: 0.48, metalness: 0.20 })
const accent      = new THREE.MeshStandardMaterial({ color: 0x4da6ff, roughness: 0.30, metalness: 0.45, emissive: 0x002244, emissiveIntensity: 0.5 })
const gripMat     = new THREE.MeshStandardMaterial({ color: 0x3377ee, roughness: 0.32, metalness: 0.55 })

function cloneMat(m: THREE.MeshStandardMaterial): THREE.MeshStandardMaterial { return m.clone() as THREE.MeshStandardMaterial }

// ── Types ────────────────────────────────────────────────────────────────────

export type ParamValues = Record<string, number | string>

export type InterfaceType =
  | 'servo_output_horn'
  | 'servo_mount_m3'
  | 'tube_end'
  | 'flat_m3_4bolt'
  | 'flat_surface'
  | 'bearing_shaft'
  | 'gripper_mount'
  | 'universal'

const COMPAT: Record<InterfaceType, InterfaceType[]> = {
  servo_output_horn: ['servo_mount_m3', 'flat_surface', 'bearing_shaft'],
  servo_mount_m3:    ['servo_output_horn', 'flat_m3_4bolt', 'flat_surface'],
  tube_end:          ['tube_end', 'flat_m3_4bolt', 'flat_surface'],
  flat_m3_4bolt:     ['flat_m3_4bolt', 'flat_surface', 'servo_mount_m3', 'tube_end'],
  flat_surface:      ['flat_surface', 'flat_m3_4bolt', 'servo_mount_m3', 'tube_end'],
  bearing_shaft:     ['servo_output_horn'],
  gripper_mount:     ['flat_m3_4bolt', 'flat_surface'],
  universal:         ['servo_output_horn','servo_mount_m3','tube_end','flat_m3_4bolt','flat_surface','bearing_shaft','gripper_mount','universal'],
}

export function interfacesCompatible(a: InterfaceType, b: InterfaceType): boolean {
  return COMPAT[a].includes(b) || COMPAT[b].includes(a)
}

export interface PartInterface {
  id: string
  label: string
  type: InterfaceType
  sex: 'output' | 'input' | 'bidirectional'
  localPosition(p: ParamValues): THREE.Vector3
  localNormal(p: ParamValues): THREE.Vector3
  localAxis(p: ParamValues): THREE.Vector3     // joint rotation/translation axis
  defaultJointType: 'fixed' | 'revolute' | 'prismatic'
}

export interface PartParameter {
  id: string
  label: string
  type: 'length' | 'radius' | 'mass' | 'enum'
  min?: number; max?: number; step?: number; unit?: string
  options?: string[]
  default: number | string
  affectsGeometry: boolean
}

export interface URDFGeomSpec {
  type: 'box' | 'cylinder' | 'sphere'
  size?: [number, number, number]
  radius?: number
  length?: number
}

export interface RobotPartDefinition {
  id: string
  name: string
  category: 'actuator' | 'structural' | 'sensor' | 'electrical' | 'end_effector'
  description: string
  parameters: PartParameter[]
  interfaces: PartInterface[]
  buildMesh(p: ParamValues): THREE.Group
  urdfVisual(p: ParamValues): URDFGeomSpec
  mass(p: ParamValues): number
  /** Inertia tensor (kg·m²) — computed analytically */
  inertia(p: ParamValues): { ixx:number; iyy:number; izz:number; ixy:number; ixz:number; iyz:number }
}

// ── Inertia helpers ───────────────────────────────────────────────────────────

function boxInertia(m: number, w: number, h: number, d: number) {
  return { ixx: m/12*(h*h+d*d), iyy: m/12*(w*w+d*d), izz: m/12*(w*w+h*h), ixy:0, ixz:0, iyz:0 }
}
function cylInertia(m: number, r: number, l: number) {
  const lat = m/12*(3*r*r+l*l)
  return { ixx: lat, iyy: m/2*r*r, izz: lat, ixy:0, ixz:0, iyz:0 }
}
export function sphInertia(m: number, r: number) {
  const v = 0.4*m*r*r
  return { ixx:v, iyy:v, izz:v, ixy:0, ixz:0, iyz:0 }
}

// ── Geometry helpers ─────────────────────────────────────────────────────────

function mesh(geo: THREE.BufferGeometry, mat: THREE.MeshStandardMaterial, shadow = true): THREE.Mesh {
  const m = new THREE.Mesh(geo, mat)
  m.castShadow = shadow; m.receiveShadow = shadow
  return m
}

const V = (x: number, y: number, z: number) => new THREE.Vector3(x, y, z)

// ── Servo variant data ────────────────────────────────────────────────────────

const SERVO_VARIANTS: Record<string, { w:number; h:number; d:number; flange_h:number; flange_d:number; mass:number; torque:number; vel:number }> = {
  'XH430': { w:0.028, h:0.046, d:0.034, flange_h:0.040, flange_d:0.048, mass:0.082, torque:2.5,  vel:5.0 },
  'XM540': { w:0.033, h:0.054, d:0.042, flange_h:0.048, flange_d:0.056, mass:0.165, torque:10.6, vel:4.8 },
  'AK80-9':{ w:0.076, h:0.038, d:0.076, flange_h:0.032, flange_d:0.084, mass:0.485, torque:18.0, vel:6.3 },
  'AK60-6':{ w:0.064, h:0.032, d:0.064, flange_h:0.028, flange_d:0.072, mass:0.350, torque:6.0,  vel:7.0 },
}
function getServoVariant(p: ParamValues) {
  return SERVO_VARIANTS[(p.variant as string) ?? 'XM540'] ?? SERVO_VARIANTS['XM540']
}

// ═══════════════════════════════════════════════════════════════════════════════
// PART DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════════

export const PART_LIBRARY: RobotPartDefinition[] = [

  // ── 1. Servo Motor ──────────────────────────────────────────────────────────
  {
    id: 'servo',
    name: 'Servo Motor',
    category: 'actuator',
    description: 'Dynamixel-compatible servo with output horn and side mounting flanges.',
    parameters: [
      { id: 'variant', label: 'Variant', type: 'enum', options: ['XH430','XM540','AK80-9','AK60-6'], default: 'XM540', affectsGeometry: true },
    ],
    interfaces: [
      {
        id: 'output_horn', label: 'Output Horn', type: 'servo_output_horn', sex: 'output',
        localPosition: p => { const v = getServoVariant(p); return V(0, v.h/2 + 0.004, 0) },
        localNormal:   _ => V(0, 1, 0),
        localAxis:     _ => V(0, 1, 0),
        defaultJointType: 'revolute',
      },
      {
        id: 'mount_left', label: 'Left Flange', type: 'servo_mount_m3', sex: 'bidirectional',
        localPosition: p => { const v = getServoVariant(p); return V(-(v.w/2 + 0.005), 0, 0) },
        localNormal:   _ => V(-1, 0, 0),
        localAxis:     _ => V(0, 0, 1),
        defaultJointType: 'fixed',
      },
      {
        id: 'mount_right', label: 'Right Flange', type: 'servo_mount_m3', sex: 'bidirectional',
        localPosition: p => { const v = getServoVariant(p); return V(v.w/2 + 0.005, 0, 0) },
        localNormal:   _ => V(1, 0, 0),
        localAxis:     _ => V(0, 0, 1),
        defaultJointType: 'fixed',
      },
      {
        id: 'mount_bottom', label: 'Bottom', type: 'flat_surface', sex: 'input',
        localPosition: p => { const v = getServoVariant(p); return V(0, -v.h/2, 0) },
        localNormal:   _ => V(0, -1, 0),
        localAxis:     _ => V(0, 1, 0),
        defaultJointType: 'fixed',
      },
    ],
    buildMesh(p) {
      const v = getServoVariant(p)
      const g = new THREE.Group()

      // Main body
      g.add(mesh(new THREE.BoxGeometry(v.w, v.h, v.d), servoBody))

      // Left flange
      const fl = mesh(new THREE.BoxGeometry(0.008, v.flange_h, v.flange_d), aluminum)
      fl.position.set(-(v.w/2 + 0.004), (v.flange_h - v.h)/2, 0)
      g.add(fl)

      // Right flange
      const fr = fl.clone(); fr.position.set(v.w/2 + 0.004, (v.flange_h - v.h)/2, 0); g.add(fr)

      // Output horn
      const horn = mesh(new THREE.CylinderGeometry(0.009, 0.009, 0.007, 14), servoHorn)
      horn.position.y = v.h/2 + 0.0035
      g.add(horn)

      // Horn spline ring
      const spline = mesh(new THREE.TorusGeometry(0.007, 0.0015, 5, 14), steel)
      spline.rotation.x = Math.PI/2; spline.position.y = v.h/2 + 0.006; g.add(spline)

      // Cable port
      const port = mesh(new THREE.CylinderGeometry(0.003, 0.003, 0.004, 8), steel)
      port.rotation.z = Math.PI/2; port.position.set(0, -v.h/2 + 0.008, v.d/2 - 0.002); g.add(port)

      // Bolt holes visual (indented circles on flanges)
      const boltGeo = new THREE.CylinderGeometry(0.0015, 0.0015, 0.002, 8)
      const boltMat = cloneMat(steel); boltMat.color.setHex(0x0a0a0e)
      const boltPositions = [[-v.flange_h/2*0.5], [v.flange_h/2*0.5]]
      for (const [yOff] of boltPositions) {
        const bL = mesh(boltGeo, boltMat)
        bL.rotation.z = Math.PI/2
        bL.position.set(-(v.w/2 + 0.008), (v.flange_h - v.h)/2 + yOff, 0)
        g.add(bL)
        const bR = bL.clone(); bR.position.x = v.w/2 + 0.008; g.add(bR)
      }

      return g
    },
    urdfVisual(p) {
      const v = getServoVariant(p)
      return { type: 'box', size: [v.w + 0.016, v.h, Math.max(v.d, v.flange_d)] }
    },
    mass(p) { return getServoVariant(p).mass },
    inertia(p) { const v = getServoVariant(p); return boxInertia(v.mass, v.w+0.016, v.h, v.flange_d) },
  },

  // ── 2. Structural Tube ──────────────────────────────────────────────────────
  {
    id: 'tube',
    name: 'Structural Tube',
    category: 'structural',
    description: 'Aluminum/carbon extrusion tube. Long axis is Y.',
    parameters: [
      { id: 'length', label: 'Length', type: 'length', min:0.04, max:0.6, step:0.005, unit:'m', default:0.2,  affectsGeometry: true },
      { id: 'od',     label: 'Outer Ø', type: 'radius', min:0.006, max:0.030, step:0.001, unit:'m', default:0.012, affectsGeometry: true },
      { id: 'mass',   label: 'Mass',    type: 'mass',   min:0.005, max:0.5, step:0.005, unit:'kg', default:0.04, affectsGeometry: false },
    ],
    interfaces: [
      {
        id: 'end_top', label: 'Top End', type: 'tube_end', sex: 'bidirectional',
        localPosition: p => V(0, (p.length as number)/2, 0),
        localNormal:   _ => V(0, 1, 0),
        localAxis:     _ => V(1, 0, 0),
        defaultJointType: 'fixed',
      },
      {
        id: 'end_bottom', label: 'Bottom End', type: 'tube_end', sex: 'bidirectional',
        localPosition: p => V(0, -(p.length as number)/2, 0),
        localNormal:   _ => V(0, -1, 0),
        localAxis:     _ => V(1, 0, 0),
        defaultJointType: 'fixed',
      },
    ],
    buildMesh(p) {
      const L = p.length as number, r = p.od as number
      const g = new THREE.Group()
      g.add(mesh(new THREE.CylinderGeometry(r, r, L, 14), aluminum))
      const ring = mesh(new THREE.TorusGeometry(r, 0.0015, 6, 14), steel)
      ring.rotation.x = Math.PI/2
      const rt = ring.clone(); rt.position.y =  L/2; g.add(rt)
      const rb = ring.clone(); rb.position.y = -L/2; g.add(rb)
      return g
    },
    urdfVisual(p) { return { type: 'cylinder', radius: p.od as number, length: p.length as number } },
    mass(p) { return p.mass as number },
    inertia(p) { return cylInertia(p.mass as number, p.od as number, p.length as number) },
  },

  // ── 3. U-Bracket ────────────────────────────────────────────────────────────
  {
    id: 'u_bracket',
    name: 'U-Bracket',
    category: 'structural',
    description: 'Aluminum U-bracket for holding a servo motor between two arms.',
    parameters: [
      { id: 'inner_w',  label: 'Inner Width',  type: 'length', min:0.020, max:0.080, step:0.002, unit:'m', default:0.040, affectsGeometry:true },
      { id: 'height',   label: 'Arm Height',   type: 'length', min:0.020, max:0.100, step:0.002, unit:'m', default:0.050, affectsGeometry:true },
      { id: 'depth',    label: 'Depth',        type: 'length', min:0.020, max:0.060, step:0.002, unit:'m', default:0.036, affectsGeometry:true },
      { id: 'thick',    label: 'Thickness',    type: 'length', min:0.002, max:0.008, step:0.001, unit:'m', default:0.004, affectsGeometry:true },
    ],
    interfaces: [
      {
        id: 'base',       label: 'Base (outer bottom)', type: 'flat_m3_4bolt', sex: 'input',
        localPosition: p => V(0, -(p.height as number)/2 - (p.thick as number), 0),
        localNormal:   _ => V(0, -1, 0),
        localAxis:     _ => V(1, 0, 0),
        defaultJointType: 'fixed',
      },
      {
        id: 'servo_left',  label: 'Left Servo Mount', type: 'servo_mount_m3', sex: 'input',
        localPosition: p => V(-(p.inner_w as number)/2, 0, 0),
        localNormal:   _ => V(-1, 0, 0),
        localAxis:     _ => V(0, 1, 0),
        defaultJointType: 'revolute',
      },
      {
        id: 'servo_right', label: 'Right Servo Mount', type: 'servo_mount_m3', sex: 'input',
        localPosition: p => V((p.inner_w as number)/2, 0, 0),
        localNormal:   _ => V(1, 0, 0),
        localAxis:     _ => V(0, 1, 0),
        defaultJointType: 'revolute',
      },
    ],
    buildMesh(p) {
      const iw = p.inner_w as number, ih = p.height as number
      const dp = p.depth as number,   th = p.thick as number
      const g = new THREE.Group()
      const mat = aluminum

      // Bottom plate
      const bot = mesh(new THREE.BoxGeometry(iw + 2*th, th, dp), mat)
      bot.position.y = -ih/2 - th/2; g.add(bot)

      // Left wall
      const lw = mesh(new THREE.BoxGeometry(th, ih, dp), mat)
      lw.position.x = -(iw/2 + th/2); g.add(lw)

      // Right wall
      const rw = lw.clone(); rw.position.x = iw/2 + th/2; g.add(rw)

      // Corner fillets (small cylinders at inner corners)
      const fillet = mesh(new THREE.CylinderGeometry(th*0.8, th*0.8, dp, 8), mat)
      const fL = fillet.clone()
      fL.position.set(-(iw/2), -ih/2, 0); g.add(fL)
      const fR = fillet.clone()
      fR.position.set( iw/2, -ih/2, 0); g.add(fR)

      return g
    },
    urdfVisual(p) {
      const iw = p.inner_w as number, ih = p.height as number
      const dp = p.depth as number,   th = p.thick as number
      return { type: 'box', size: [iw + 2*th, ih + th, dp] }
    },
    mass(_p) { return 0.045 },
    inertia(p) {
      const iw = p.inner_w as number + 2*(p.thick as number)
      return boxInertia(0.045, iw, (p.height as number) + (p.thick as number), p.depth as number)
    },
  },

  // ── 4. L-Bracket ────────────────────────────────────────────────────────────
  {
    id: 'l_bracket',
    name: 'L-Bracket',
    category: 'structural',
    description: 'Aluminum L-bracket. Horizontal arm faces +Z, vertical arm faces +Y.',
    parameters: [
      { id: 'arm_a',  label: 'Arm A (vert)', type: 'length', min:0.015, max:0.100, step:0.005, unit:'m', default:0.040, affectsGeometry:true },
      { id: 'arm_b',  label: 'Arm B (horiz)',type: 'length', min:0.015, max:0.100, step:0.005, unit:'m', default:0.040, affectsGeometry:true },
      { id: 'width',  label: 'Width',        type: 'length', min:0.010, max:0.060, step:0.005, unit:'m', default:0.030, affectsGeometry:true },
      { id: 'thick',  label: 'Thickness',    type: 'length', min:0.002, max:0.008, step:0.001, unit:'m', default:0.003, affectsGeometry:true },
    ],
    interfaces: [
      {
        id: 'face_vertical',   label: 'Vertical Face', type: 'flat_m3_4bolt', sex: 'bidirectional',
        localPosition: p => V(0, (p.arm_a as number), 0),
        localNormal:   _ => V(0, 1, 0),
        localAxis:     _ => V(1, 0, 0),
        defaultJointType: 'fixed',
      },
      {
        id: 'face_horizontal', label: 'Horizontal Face', type: 'flat_m3_4bolt', sex: 'bidirectional',
        localPosition: p => V(0, 0, (p.arm_b as number)),
        localNormal:   _ => V(0, 0, 1),
        localAxis:     _ => V(0, 1, 0),
        defaultJointType: 'fixed',
      },
    ],
    buildMesh(p) {
      const armA = p.arm_a as number, armB = p.arm_b as number
      const w = p.width as number,    th = p.thick as number
      const g = new THREE.Group()

      // Vertical arm (goes up in Y)
      const va = mesh(new THREE.BoxGeometry(w, armA, th), aluminum)
      va.position.set(0, armA/2, th/2); g.add(va)

      // Horizontal arm (goes out in Z)
      const ha = mesh(new THREE.BoxGeometry(w, th, armB), aluminum)
      ha.position.set(0, th/2, armB/2 + th); g.add(ha)

      return g
    },
    urdfVisual(p) {
      return { type: 'box', size: [p.width as number, (p.arm_a as number) + (p.thick as number), (p.arm_b as number) + (p.thick as number)] }
    },
    mass(_p) { return 0.025 },
    inertia(p) { return boxInertia(0.025, p.width as number, (p.arm_a as number) + (p.thick as number), (p.arm_b as number) + (p.thick as number)) },
  },

  // ── 5. Flat Plate ────────────────────────────────────────────────────────────
  {
    id: 'flat_plate',
    name: 'Flat Plate',
    category: 'structural',
    description: 'Generic aluminum mounting plate.',
    parameters: [
      { id: 'width',  label: 'Width',     type: 'length', min:0.020, max:0.300, step:0.010, unit:'m', default:0.100, affectsGeometry:true },
      { id: 'depth',  label: 'Depth',     type: 'length', min:0.020, max:0.300, step:0.010, unit:'m', default:0.100, affectsGeometry:true },
      { id: 'thick',  label: 'Thickness', type: 'length', min:0.002, max:0.010, step:0.001, unit:'m', default:0.004, affectsGeometry:true },
    ],
    interfaces: [
      {
        id: 'top',    label: 'Top Face',    type: 'flat_surface', sex: 'bidirectional',
        localPosition: p => V(0,  (p.thick as number)/2, 0),
        localNormal:   _ => V(0,  1, 0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed',
      },
      {
        id: 'bottom', label: 'Bottom Face', type: 'flat_surface', sex: 'bidirectional',
        localPosition: p => V(0, -(p.thick as number)/2, 0),
        localNormal:   _ => V(0, -1, 0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed',
      },
    ],
    buildMesh(p) {
      const w = p.width as number, d = p.depth as number, th = p.thick as number
      const g = new THREE.Group()
      g.add(mesh(new THREE.BoxGeometry(w, th, d), aluminum))

      // Corner bolt holes
      const boltR = Math.min(w, d) * 0.04
      const bGeo = new THREE.CylinderGeometry(boltR, boltR, th * 1.1, 8)
      const bMat = cloneMat(steel); bMat.color.setHex(0x080810)
      const offX = w/2 - 0.008, offZ = d/2 - 0.008
      for (const [sx, sz] of [[-1,-1],[-1,1],[1,-1],[1,1]]) {
        const b = mesh(bGeo, bMat)
        b.position.set(sx*offX, 0, sz*offZ); g.add(b)
      }
      return g
    },
    urdfVisual(p) { return { type: 'box', size: [p.width as number, p.thick as number, p.depth as number] } },
    mass(p) { const w=p.width as number,d=p.depth as number,th=p.thick as number; return 2700*w*d*th*0.85 },
    inertia(p) { const m=this.mass(p); return boxInertia(m, p.width as number, p.thick as number, p.depth as number) },
  },

  // ── 6. Cross Hub ─────────────────────────────────────────────────────────────
  {
    id: 'cross_hub',
    name: 'Cross Hub',
    category: 'structural',
    description: 'Central hub with 4 tube sockets radiating at 90° and a top shaft interface.',
    parameters: [
      { id: 'hub_r',    label: 'Hub Radius',    type: 'radius', min:0.012, max:0.050, step:0.002, unit:'m', default:0.020, affectsGeometry:true },
      { id: 'hub_h',    label: 'Hub Height',    type: 'length', min:0.010, max:0.040, step:0.002, unit:'m', default:0.020, affectsGeometry:true },
      { id: 'arm_len',  label: 'Arm Length',    type: 'length', min:0.010, max:0.060, step:0.002, unit:'m', default:0.024, affectsGeometry:true },
      { id: 'arm_r',    label: 'Arm Radius',    type: 'radius', min:0.004, max:0.016, step:0.001, unit:'m', default:0.008, affectsGeometry:true },
    ],
    interfaces: [
      { id:'top',    label:'Top Shaft',   type:'bearing_shaft',  sex:'output',       localPosition: p=>V(0,(p.hub_h as number)/2,0),                       localNormal:_=>V(0,1,0),  localAxis:_=>V(0,1,0), defaultJointType:'revolute' },
      { id:'px',     label:'+X Socket',   type:'tube_end',       sex:'bidirectional',localPosition: p=>V((p.hub_r as number)+(p.arm_len as number),0,0),     localNormal:_=>V(1,0,0),  localAxis:_=>V(0,1,0), defaultJointType:'fixed' },
      { id:'nx',     label:'-X Socket',   type:'tube_end',       sex:'bidirectional',localPosition: p=>V(-((p.hub_r as number)+(p.arm_len as number)),0,0),  localNormal:_=>V(-1,0,0), localAxis:_=>V(0,1,0), defaultJointType:'fixed' },
      { id:'pz',     label:'+Z Socket',   type:'tube_end',       sex:'bidirectional',localPosition: p=>V(0,0,(p.hub_r as number)+(p.arm_len as number)),     localNormal:_=>V(0,0,1),  localAxis:_=>V(0,1,0), defaultJointType:'fixed' },
      { id:'nz',     label:'-Z Socket',   type:'tube_end',       sex:'bidirectional',localPosition: p=>V(0,0,-((p.hub_r as number)+(p.arm_len as number))),  localNormal:_=>V(0,0,-1), localAxis:_=>V(0,1,0), defaultJointType:'fixed' },
      { id:'bottom', label:'Bottom',      type:'flat_surface',   sex:'input',        localPosition: p=>V(0,-(p.hub_h as number)/2,0),                        localNormal:_=>V(0,-1,0), localAxis:_=>V(1,0,0), defaultJointType:'fixed' },
    ],
    buildMesh(p) {
      const hr = p.hub_r as number, hh = p.hub_h as number
      const al = p.arm_len as number, ar = p.arm_r as number
      const g = new THREE.Group()

      // Center cylinder
      g.add(mesh(new THREE.CylinderGeometry(hr, hr, hh, 16), aluminum))

      // 4 arms
      for (const [ax, az] of [[1,0],[-1,0],[0,1],[0,-1]]) {
        const arm = mesh(new THREE.CylinderGeometry(ar, ar, al, 10), aluminum)
        arm.rotation.z = ax !== 0 ? Math.PI/2 : 0
        arm.rotation.x = az !== 0 ? Math.PI/2 : 0
        arm.position.set(ax*(hr+al/2), 0, az*(hr+al/2))
        g.add(arm)
        // Socket ring
        const ring = mesh(new THREE.TorusGeometry(ar, 0.0015, 6, 12), steel)
        if (ax !== 0) ring.rotation.z = Math.PI/2; else ring.rotation.x = Math.PI/2
        ring.position.set(ax*(hr+al), 0, az*(hr+al))
        g.add(ring)
      }

      // Top shaft stub
      const shaft = mesh(new THREE.CylinderGeometry(hr*0.3, hr*0.3, hh*0.4, 12), steel)
      shaft.position.y = hh/2 + hh*0.2; g.add(shaft)

      return g
    },
    urdfVisual(p) { return { type: 'cylinder', radius: (p.hub_r as number)+(p.arm_len as number), length: p.hub_h as number } },
    mass(_p) { return 0.06 },
    inertia(p) { return cylInertia(0.06, (p.hub_r as number)+(p.arm_len as number), p.hub_h as number) },
  },

  // ── 7. RGB Camera ───────────────────────────────────────────────────────────
  {
    id: 'camera_rgb',
    name: 'RGB Camera',
    category: 'sensor',
    description: 'Compact RGB camera module. Lens faces +Z.',
    parameters: [],
    interfaces: [
      {
        id: 'mount_back',   label: 'Back Mount',   type: 'flat_surface', sex: 'input',
        localPosition: _ => V(0, 0, -0.012),
        localNormal:   _ => V(0, 0, -1), localAxis: _ => V(0,1,0), defaultJointType: 'fixed',
      },
      {
        id: 'mount_bottom', label: 'Bottom Mount', type: 'flat_m3_4bolt', sex: 'input',
        localPosition: _ => V(0, -0.016, 0),
        localNormal:   _ => V(0, -1, 0),  localAxis: _ => V(1,0,0), defaultJointType: 'fixed',
      },
    ],
    buildMesh(_p) {
      const g = new THREE.Group()
      // Body
      g.add(mesh(new THREE.BoxGeometry(0.050, 0.030, 0.022), sensorShell))
      // Lens barrel
      const barrel = mesh(new THREE.CylinderGeometry(0.009, 0.009, 0.006, 12), steel)
      barrel.rotation.x = Math.PI/2; barrel.position.z = 0.013; g.add(barrel)
      // Lens glass
      const glass = mesh(new THREE.CylinderGeometry(0.007, 0.007, 0.003, 12), lensMat)
      glass.rotation.x = Math.PI/2; glass.position.z = 0.015; g.add(glass)
      // Lens ring
      const ring = mesh(new THREE.TorusGeometry(0.009, 0.0015, 6, 12), steel)
      ring.rotation.x = Math.PI/2; ring.position.z = 0.012; g.add(ring)
      // Status LED
      const led = mesh(new THREE.SphereGeometry(0.002, 6, 6), accent)
      led.position.set(0.020, 0.010, 0.010); g.add(led)
      return g
    },
    urdfVisual(_p) { return { type: 'box', size: [0.050, 0.030, 0.028] } },
    mass(_p) { return 0.050 },
    inertia(_p) { return boxInertia(0.050, 0.050, 0.030, 0.028) },
  },

  // ── 8. Depth Camera (D435-style) ────────────────────────────────────────────
  {
    id: 'camera_depth',
    name: 'Depth Camera',
    category: 'sensor',
    description: 'Stereo depth camera (RealSense D435 style). Lenses face +Z.',
    parameters: [],
    interfaces: [
      {
        id: 'mount_back',   label: 'Back Mount',   type: 'flat_surface', sex: 'input',
        localPosition: _ => V(0, 0, -0.013),
        localNormal:   _ => V(0, 0, -1), localAxis: _ => V(0,1,0), defaultJointType: 'fixed',
      },
    ],
    buildMesh(_p) {
      const g = new THREE.Group()
      g.add(mesh(new THREE.BoxGeometry(0.090, 0.025, 0.025), sensorShell))
      // 3 lenses: IR left, IR right, RGB center
      const lensPositions: [number, number][] = [[-0.030, 0], [0, 0], [0.030, 0]]
      for (const [lx, ly] of lensPositions) {
        const b = mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.004, 12), steel)
        b.rotation.x = Math.PI/2; b.position.set(lx, ly, 0.013); g.add(b)
        const gl = mesh(new THREE.CylinderGeometry(0.005, 0.005, 0.002, 12), lensMat)
        gl.rotation.x = Math.PI/2; gl.position.set(lx, ly, 0.014); g.add(gl)
      }
      // IR emitter dot
      const ir = mesh(new THREE.SphereGeometry(0.003, 6, 6), cloneMat(servoHorn))
      ir.position.set(-0.040, 0, 0.010); g.add(ir)
      // USB-C port
      const port = mesh(new THREE.BoxGeometry(0.009, 0.004, 0.003), steel)
      port.position.set(0.038, -0.010, -0.012); g.add(port)
      return g
    },
    urdfVisual(_p) { return { type: 'box', size: [0.090, 0.025, 0.030] } },
    mass(_p) { return 0.072 },
    inertia(_p) { return boxInertia(0.072, 0.090, 0.025, 0.030) },
  },

  // ── 9. IMU ──────────────────────────────────────────────────────────────────
  {
    id: 'imu',
    name: 'IMU',
    category: 'sensor',
    description: 'Inertial measurement unit PCB. Mounts flat.',
    parameters: [],
    interfaces: [
      {
        id: 'mount', label: 'Mount', type: 'flat_surface', sex: 'input',
        localPosition: _ => V(0, -0.003, 0),
        localNormal:   _ => V(0, -1, 0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed',
      },
    ],
    buildMesh(_p) {
      const g = new THREE.Group()
      // PCB
      g.add(mesh(new THREE.BoxGeometry(0.028, 0.004, 0.028), pcb))
      // Chip
      const chip = mesh(new THREE.BoxGeometry(0.006, 0.002, 0.006), cloneMat(steel))
      chip.position.y = 0.002; g.add(chip)
      // Connector
      const conn = mesh(new THREE.BoxGeometry(0.010, 0.003, 0.004), steel)
      conn.position.set(0, 0.003, 0.015); g.add(conn)
      return g
    },
    urdfVisual(_p) { return { type: 'box', size: [0.028, 0.006, 0.028] } },
    mass(_p) { return 0.008 },
    inertia(_p) { return boxInertia(0.008, 0.028, 0.006, 0.028) },
  },

  // ── 10. LiDAR ───────────────────────────────────────────────────────────────
  {
    id: 'lidar',
    name: 'LiDAR',
    category: 'sensor',
    description: '360° spinning LiDAR sensor. Scan plane is XZ.',
    parameters: [
      { id: 'variant', label: 'Variant', type: 'enum', options: ['VLP-16','RPLIDAR A2','Livox Mid-70'], default:'RPLIDAR A2', affectsGeometry:true },
    ],
    interfaces: [
      {
        id: 'base_mount', label: 'Base Mount', type: 'flat_m3_4bolt', sex: 'input',
        localPosition: _ => V(0, -0.038, 0),
        localNormal:   _ => V(0, -1, 0), localAxis: _ => V(0,1,0), defaultJointType: 'fixed',
      },
    ],
    buildMesh(p) {
      const isVlp = (p.variant as string).includes('VLP')
      const br = isVlp ? 0.051 : 0.038, bh = isVlp ? 0.072 : 0.042
      const sr = br * 0.6,              sh = bh * 0.35
      const g = new THREE.Group()
      // Base cylinder
      g.add(mesh(new THREE.CylinderGeometry(br, br, bh, 18), sensorShell))
      // Spinning drum
      const drum = mesh(new THREE.CylinderGeometry(sr, sr, sh, 18), cloneMat(steel))
      drum.position.y = bh/2 + sh/2 - 0.002; g.add(drum)
      // Ring detail
      const ring = mesh(new THREE.TorusGeometry(br, 0.002, 6, 18), steel)
      ring.rotation.x = Math.PI/2; ring.position.y = bh*0.2; g.add(ring)
      // Emitter window (laser strip)
      const win = mesh(new THREE.BoxGeometry(0.003, sh*0.8, sr*2), lensMat)
      win.position.set(sr, bh/2 + sh/2 - 0.002, 0); g.add(win)
      return g
    },
    urdfVisual(p) {
      const isVlp = (p.variant as string).includes('VLP')
      return { type: 'cylinder', radius: isVlp ? 0.051 : 0.038, length: isVlp ? 0.072 : 0.042 }
    },
    mass(p) { return (p.variant as string).includes('VLP') ? 0.830 : 0.170 },
    inertia(p) {
      const m = this.mass(p), isVlp = (p.variant as string).includes('VLP')
      return cylInertia(m, isVlp ? 0.051 : 0.038, isVlp ? 0.072 : 0.042)
    },
  },

  // ── 11. F/T Sensor ──────────────────────────────────────────────────────────
  {
    id: 'fts',
    name: 'Force/Torque Sensor',
    category: 'sensor',
    description: 'Flanged disk sensor. Passes forces/torques through 6 DOF.',
    parameters: [
      { id: 'od', label: 'Outer Ø', type: 'radius', min:0.030, max:0.080, step:0.002, unit:'m', default:0.038, affectsGeometry:true },
    ],
    interfaces: [
      {
        id: 'top',    label: 'Top Face',    type: 'flat_m3_4bolt', sex: 'bidirectional',
        localPosition: _ => V(0,  0.014, 0),
        localNormal:   _ => V(0,  1, 0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed',
      },
      {
        id: 'bottom', label: 'Bottom Face', type: 'flat_m3_4bolt', sex: 'bidirectional',
        localPosition: _ => V(0, -0.014, 0),
        localNormal:   _ => V(0, -1, 0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed',
      },
    ],
    buildMesh(p) {
      const r = p.od as number
      const g = new THREE.Group()
      // Main disk
      g.add(mesh(new THREE.CylinderGeometry(r, r, 0.026, 16), sensorShell))
      // Outer flange ring
      const flange = mesh(new THREE.TorusGeometry(r + 0.003, 0.003, 8, 16), steel)
      flange.rotation.x = Math.PI/2; g.add(flange)
      // Cable connector
      const conn = mesh(new THREE.BoxGeometry(0.012, 0.008, 0.006), steel)
      conn.position.set(r + 0.002, 0, 0); g.add(conn)
      return g
    },
    urdfVisual(p) { return { type: 'cylinder', radius: (p.od as number) + 0.006, length: 0.026 } },
    mass(_p) { return 0.280 },
    inertia(p) { return cylInertia(0.280, (p.od as number) + 0.006, 0.026) },
  },

  // ── 12. Parallel Jaw Gripper ─────────────────────────────────────────────────
  {
    id: 'gripper_parallel',
    name: 'Parallel Jaw Gripper',
    category: 'end_effector',
    description: 'Two-finger parallel jaw gripper. Fingers open/close along X.',
    parameters: [
      { id: 'opening', label: 'Max Opening', type: 'length', min:0.030, max:0.120, step:0.005, unit:'m', default:0.070, affectsGeometry:true },
      { id: 'finger_l',label: 'Finger Length',type:'length', min:0.030, max:0.120, step:0.005, unit:'m', default:0.060, affectsGeometry:true },
    ],
    interfaces: [
      {
        id: 'mount', label: 'Wrist Mount', type: 'flat_m3_4bolt', sex: 'input',
        localPosition: _ => V(0, 0.020, 0),
        localNormal:   _ => V(0, 1, 0), localAxis: _ => V(0,1,0), defaultJointType: 'fixed',
      },
    ],
    buildMesh(p) {
      const op = p.opening as number, fl = p.finger_l as number
      const g = new THREE.Group()
      const bw = op + 0.016, bh = 0.020, bd = 0.032

      // Body
      g.add(mesh(new THREE.BoxGeometry(bw, bh, bd), steel))

      // Finger material
      const fMat = gripMat
      // Left finger
      const lf = mesh(new THREE.BoxGeometry(0.008, fl, 0.016), fMat)
      lf.position.set(-(op/2 + 0.004), -(fl/2 + bh/2), 0); g.add(lf)
      // Right finger
      const rf = lf.clone(); rf.position.x = op/2 + 0.004; g.add(rf)

      // Finger tips (rubber pads)
      const pad = mesh(new THREE.BoxGeometry(0.006, 0.008, 0.018), rubber)
      const padL = pad.clone(); padL.position.set(-(op/2+0.004), -(fl+bh/2), 0); g.add(padL)
      const padR = pad.clone(); padR.position.set( op/2+0.004, -(fl+bh/2), 0); g.add(padR)

      // Guide rail
      const rail = mesh(new THREE.CylinderGeometry(0.002, 0.002, op, 8), steel)
      rail.rotation.z = Math.PI/2; rail.position.y = -bh*0.2; g.add(rail)

      return g
    },
    urdfVisual(p) {
      const op = p.opening as number, fl = p.finger_l as number
      return { type: 'box', size: [op + 0.016, fl + 0.020, 0.032] }
    },
    mass(_p) { return 0.250 },
    inertia(p) {
      const op=p.opening as number, fl=p.finger_l as number
      return boxInertia(0.250, op+0.016, fl+0.020, 0.032)
    },
  },

  // ── 13. Battery Pack ─────────────────────────────────────────────────────────
  {
    id: 'battery',
    name: 'Battery Pack',
    category: 'electrical',
    description: 'LiPo battery pack. Mounts flat on any surface.',
    parameters: [
      { id: 'cells',    label: 'Cell Count', type: 'enum',   options:['2S','3S','4S','6S'],  default:'4S', affectsGeometry:true },
      { id: 'capacity', label: 'Capacity',   type: 'enum',   options:['1300mAh','2200mAh','5000mAh','10000mAh'], default:'5000mAh', affectsGeometry:false },
    ],
    interfaces: [
      {
        id: 'mount', label: 'Mount', type: 'flat_surface', sex: 'input',
        localPosition: _ => V(0, -0.022, 0),
        localNormal:   _ => V(0, -1, 0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed',
      },
    ],
    buildMesh(p) {
      const s = p.cells as string
      const w = s==='2S'?0.06:s==='3S'?0.07:s==='4S'?0.10:0.13
      const g = new THREE.Group()
      // Body
      g.add(mesh(new THREE.BoxGeometry(w, 0.035, 0.040), battery))
      // Voltage label band
      const band = mesh(new THREE.BoxGeometry(w*0.6, 0.035, 0.002), cloneMat(servoHorn))
      ;(band.material as THREE.MeshStandardMaterial).color.setHex(0xffcc00); band.position.z = 0.021; g.add(band)
      // Connector
      const conn = mesh(new THREE.BoxGeometry(0.010, 0.010, 0.012), steel)
      conn.position.set(w/2 - 0.010, 0.005, 0); g.add(conn)
      // Warning stripe
      const stripe = mesh(new THREE.BoxGeometry(w*0.3, 0.005, 0.042), cloneMat(servoHorn))
      ;(stripe.material as THREE.MeshStandardMaterial).color.setHex(0xff3300); stripe.position.set(-w*0.2, 0.018, 0); g.add(stripe)
      return g
    },
    urdfVisual(p) {
      const s=p.cells as string, w=s==='2S'?0.06:s==='3S'?0.07:s==='4S'?0.10:0.13
      return { type: 'box', size: [w, 0.035, 0.040] }
    },
    mass(p) {
      const cap = p.capacity as string
      return cap.includes('1300')?0.120:cap.includes('2200')?0.180:cap.includes('5000')?0.350:0.620
    },
    inertia(p) {
      const s=p.cells as string, w=s==='2S'?0.06:s==='3S'?0.07:s==='4S'?0.10:0.13
      return boxInertia(this.mass(p), w, 0.035, 0.040)
    },
  },

  // ── 14. Controller PCB ───────────────────────────────────────────────────────
  {
    id: 'controller_pcb',
    name: 'Controller PCB',
    category: 'electrical',
    description: 'Embedded controller board (Raspberry Pi / Jetson / STM32 style).',
    parameters: [
      { id: 'variant', label: 'Type', type: 'enum', options: ['Raspberry Pi 4','Jetson Nano','STM32 Nucleo','ESP32'], default:'Raspberry Pi 4', affectsGeometry:true },
    ],
    interfaces: [
      {
        id: 'mount', label: 'PCB Mount', type: 'flat_surface', sex: 'input',
        localPosition: _ => V(0, -0.002, 0),
        localNormal:   _ => V(0, -1, 0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed',
      },
    ],
    buildMesh(p) {
      const v = p.variant as string
      const w = v.includes('Jetson')?0.100:v.includes('STM')?0.070:v.includes('ESP')?0.055:0.085
      const d = v.includes('Jetson')?0.080:v.includes('STM')?0.070:v.includes('ESP')?0.028:0.056
      const g = new THREE.Group()
      g.add(mesh(new THREE.BoxGeometry(w, 0.004, d), pcb))
      // Chip
      const chip = mesh(new THREE.BoxGeometry(0.016, 0.002, 0.016), cloneMat(steel))
      chip.position.set(-0.010, 0.003, -0.010); g.add(chip)
      // USB ports
      for (let i = 0; i < 2; i++) {
        const usb = mesh(new THREE.BoxGeometry(0.014, 0.007, 0.004), steel)
        usb.position.set(i*0.018 - 0.018, 0.006, d/2); g.add(usb)
      }
      // GPIO header
      const gpio = mesh(new THREE.BoxGeometry(0.005, 0.006, 0.040), cloneMat(steel))
      ;(gpio.material as THREE.MeshStandardMaterial).color.setHex(0x222222); gpio.position.set(w/2-0.006, 0.005, 0); g.add(gpio)
      return g
    },
    urdfVisual(p) {
      const v=p.variant as string
      const w=v.includes('Jetson')?0.100:v.includes('STM')?0.070:v.includes('ESP')?0.055:0.085
      const d=v.includes('Jetson')?0.080:v.includes('STM')?0.070:v.includes('ESP')?0.028:0.056
      return { type: 'box', size: [w, 0.010, d] }
    },
    mass(p) {
      const v=p.variant as string
      return v.includes('Jetson')?0.136:v.includes('STM')?0.045:v.includes('ESP')?0.010:0.046
    },
    inertia(p) {
      const v=p.variant as string
      const w=v.includes('Jetson')?0.100:v.includes('STM')?0.070:v.includes('ESP')?0.055:0.085
      const d=v.includes('Jetson')?0.080:v.includes('STM')?0.070:v.includes('ESP')?0.028:0.056
      return boxInertia(this.mass(p), w, 0.010, d)
    },
  },

] // end PART_LIBRARY

// ── Lookup helpers ────────────────────────────────────────────────────────────

export function getPartDef(id: string): RobotPartDefinition | undefined {
  return PART_LIBRARY.find(p => p.id === id)
}

export function defaultParams(def: RobotPartDefinition): ParamValues {
  const p: ParamValues = {}
  for (const param of def.parameters) p[param.id] = param.default
  return p
}

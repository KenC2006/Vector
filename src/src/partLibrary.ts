import * as THREE from 'three'

export type ParamValues = Record<string, number | string>

export type InterfaceType =
  | 'plane_struct'
  | 'hinge_axle'
  | 'link_end'
  | 'foot_mount'
  | 'universal_mount'

const COMPAT: Record<InterfaceType, InterfaceType[]> = {
  plane_struct:    ['plane_struct', 'universal_mount'],
  hinge_axle:      ['hinge_axle', 'universal_mount'],
  link_end:        ['link_end', 'universal_mount'],
  foot_mount:      ['foot_mount', 'universal_mount'],
  universal_mount: ['plane_struct', 'hinge_axle', 'link_end', 'foot_mount', 'universal_mount'],
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
  localAxis(p: ParamValues): THREE.Vector3
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

export type PartFamily = 'structure' | 'joints' | 'links' | 'feet' | 'mounts'

export interface RobotPartDefinition {
  id: string
  name: string
  category: PartFamily
  description: string
  parameters: PartParameter[]
  interfaces: PartInterface[]
  buildMesh(p: ParamValues): THREE.Group
  urdfVisual(p: ParamValues): URDFGeomSpec
  mass(p: ParamValues): number
  inertia(p: ParamValues): { ixx:number; iyy:number; izz:number; ixy:number; ixz:number; iyz:number }
}

const V = (x: number, y: number, z: number) => new THREE.Vector3(x, y, z)

const matAl = new THREE.MeshStandardMaterial({ color: 0xc4c9d6, roughness: 0.32, metalness: 0.72 })
const matSt = new THREE.MeshStandardMaterial({ color: 0x58627a, roughness: 0.36, metalness: 0.82 })
const matAc = new THREE.MeshStandardMaterial({ color: 0x66b5ff, roughness: 0.3, metalness: 0.4 })
const matRb = new THREE.MeshStandardMaterial({ color: 0x292929, roughness: 0.95, metalness: 0.02 })

function mesh(geo: THREE.BufferGeometry, mat: THREE.MeshStandardMaterial): THREE.Mesh {
  const m = new THREE.Mesh(geo, mat.clone())
  m.castShadow = true
  m.receiveShadow = true
  return m
}

function boxInertia(m: number, w: number, h: number, d: number) {
  return { ixx: m/12*(h*h+d*d), iyy: m/12*(w*w+d*d), izz: m/12*(w*w+h*h), ixy:0, ixz:0, iyz:0 }
}
function cylInertia(m: number, r: number, l: number) {
  const lat = m/12*(3*r*r+l*l)
  return { ixx: lat, iyy: m/2*r*r, izz: lat, ixy:0, ixz:0, iyz:0 }
}

export const PART_LIBRARY: RobotPartDefinition[] = [
  {
    id: 'struct.beam.straight',
    name: 'Straight Beam',
    category: 'structure',
    description: 'Primary structural member. Long axis is Y.',
    parameters: [
      { id: 'length', label: 'Length', type: 'length', min: 0.05, max: 0.5, step: 0.005, unit: 'm', default: 0.18, affectsGeometry: true },
      { id: 'width',  label: 'Width',  type: 'length', min: 0.01, max: 0.05, step: 0.001, unit: 'm', default: 0.02, affectsGeometry: true },
      { id: 'depth',  label: 'Depth',  type: 'length', min: 0.008, max: 0.04, step: 0.001, unit: 'm', default: 0.016, affectsGeometry: true },
    ],
    interfaces: [
      { id: 'top', label: 'Top End', type: 'plane_struct', sex: 'bidirectional', localPosition: p => V(0, (p.length as number)/2, 0), localNormal: _ => V(0,1,0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed' },
      { id: 'bottom', label: 'Bottom End', type: 'plane_struct', sex: 'bidirectional', localPosition: p => V(0, -(p.length as number)/2, 0), localNormal: _ => V(0,-1,0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed' },
    ],
    buildMesh(p) {
      const g = new THREE.Group()
      const l = p.length as number, w = p.width as number, d = p.depth as number
      g.add(mesh(new THREE.BoxGeometry(w, l, d), matAl))
      return g
    },
    urdfVisual(p) { return { type: 'box', size: [p.width as number, p.length as number, p.depth as number] } },
    mass(p) { return 2700 * (p.width as number) * (p.length as number) * (p.depth as number) * 0.55 },
    inertia(p) { return boxInertia(this.mass(p), p.width as number, p.length as number, p.depth as number) },
  },
  {
    id: 'struct.plate.rect',
    name: 'Rect Plate',
    category: 'structure',
    description: 'General purpose flat plate.',
    parameters: [
      { id: 'width',  label: 'Width', type: 'length', min: 0.03, max: 0.3, step: 0.005, unit: 'm', default: 0.1, affectsGeometry: true },
      { id: 'depth',  label: 'Depth', type: 'length', min: 0.03, max: 0.3, step: 0.005, unit: 'm', default: 0.08, affectsGeometry: true },
      { id: 'thick',  label: 'Thickness', type: 'length', min: 0.002, max: 0.01, step: 0.001, unit: 'm', default: 0.004, affectsGeometry: true },
    ],
    interfaces: [
      { id: 'top', label: 'Top Face', type: 'plane_struct', sex: 'bidirectional', localPosition: p => V(0, (p.thick as number)/2, 0), localNormal: _ => V(0,1,0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed' },
      { id: 'bottom', label: 'Bottom Face', type: 'plane_struct', sex: 'bidirectional', localPosition: p => V(0, -(p.thick as number)/2, 0), localNormal: _ => V(0,-1,0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed' },
    ],
    buildMesh(p) {
      const g = new THREE.Group()
      g.add(mesh(new THREE.BoxGeometry(p.width as number, p.thick as number, p.depth as number), matAl))
      return g
    },
    urdfVisual(p) { return { type: 'box', size: [p.width as number, p.thick as number, p.depth as number] } },
    mass(p) { return 2700 * (p.width as number) * (p.thick as number) * (p.depth as number) * 0.6 },
    inertia(p) { return boxInertia(this.mass(p), p.width as number, p.thick as number, p.depth as number) },
  },
  {
    id: 'struct.bracket.L90',
    name: 'L-Bracket 90',
    category: 'structure',
    description: 'Orthogonal structural corner bracket.',
    parameters: [
      { id: 'armA', label: 'Arm A', type: 'length', min: 0.02, max: 0.12, step: 0.005, unit: 'm', default: 0.05, affectsGeometry: true },
      { id: 'armB', label: 'Arm B', type: 'length', min: 0.02, max: 0.12, step: 0.005, unit: 'm', default: 0.05, affectsGeometry: true },
      { id: 'width', label: 'Width', type: 'length', min: 0.01, max: 0.06, step: 0.002, unit: 'm', default: 0.025, affectsGeometry: true },
      { id: 'thick', label: 'Thickness', type: 'length', min: 0.002, max: 0.01, step: 0.001, unit: 'm', default: 0.004, affectsGeometry: true },
    ],
    interfaces: [
      { id: 'faceA', label: 'Face A', type: 'plane_struct', sex: 'bidirectional', localPosition: p => V(0, p.armA as number, 0), localNormal: _ => V(0,1,0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed' },
      { id: 'faceB', label: 'Face B', type: 'plane_struct', sex: 'bidirectional', localPosition: p => V(0, 0, p.armB as number), localNormal: _ => V(0,0,1), localAxis: _ => V(0,1,0), defaultJointType: 'fixed' },
    ],
    buildMesh(p) {
      const g = new THREE.Group()
      const a = p.armA as number, b = p.armB as number, w = p.width as number, t = p.thick as number
      const va = mesh(new THREE.BoxGeometry(w, a, t), matAl); va.position.set(0, a/2, t/2); g.add(va)
      const ha = mesh(new THREE.BoxGeometry(w, t, b), matAl); ha.position.set(0, t/2, b/2 + t); g.add(ha)
      return g
    },
    urdfVisual(p) { return { type: 'box', size: [p.width as number, (p.armA as number)+(p.thick as number), (p.armB as number)+(p.thick as number)] } },
    mass(_p) { return 0.03 },
    inertia(p) { return boxInertia(0.03, p.width as number, (p.armA as number)+(p.thick as number), (p.armB as number)+(p.thick as number)) },
  },
  {
    id: 'joint.hinge.block',
    name: 'Hinge Block',
    category: 'joints',
    description: 'Compact revolute hinge module with structural faces.',
    parameters: [
      { id: 'bodyW', label: 'Body Width', type: 'length', min: 0.02, max: 0.08, step: 0.002, unit: 'm', default: 0.04, affectsGeometry: true },
      { id: 'bodyH', label: 'Body Height', type: 'length', min: 0.02, max: 0.08, step: 0.002, unit: 'm', default: 0.035, affectsGeometry: true },
      { id: 'bodyD', label: 'Body Depth', type: 'length', min: 0.015, max: 0.06, step: 0.002, unit: 'm', default: 0.025, affectsGeometry: true },
    ],
    interfaces: [
      { id: 'in_face', label: 'Input Face', type: 'plane_struct', sex: 'input', localPosition: p => V(0, -(p.bodyH as number)/2, 0), localNormal: _ => V(0,-1,0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed' },
      { id: 'axle_out', label: 'Hinge Axle', type: 'hinge_axle', sex: 'output', localPosition: p => V(0, (p.bodyH as number)/2 + 0.008, 0), localNormal: _ => V(0,1,0), localAxis: _ => V(0,1,0), defaultJointType: 'revolute' },
    ],
    buildMesh(p) {
      const g = new THREE.Group()
      const w = p.bodyW as number, h = p.bodyH as number, d = p.bodyD as number
      g.add(mesh(new THREE.BoxGeometry(w, h, d), matSt))
      const axle = mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.014, 12), matAc)
      axle.position.y = h/2 + 0.007
      g.add(axle)
      return g
    },
    urdfVisual(p) { return { type: 'box', size: [p.bodyW as number, (p.bodyH as number) + 0.014, p.bodyD as number] } },
    mass(_p) { return 0.08 },
    inertia(p) { return boxInertia(0.08, p.bodyW as number, (p.bodyH as number) + 0.014, p.bodyD as number) },
  },
  {
    id: 'joint.coupler.inline',
    name: 'Inline Coupler',
    category: 'joints',
    description: 'Straight rotational coupler linking two axle interfaces.',
    parameters: [
      { id: 'length', label: 'Length', type: 'length', min: 0.015, max: 0.09, step: 0.002, unit: 'm', default: 0.035, affectsGeometry: true },
      { id: 'radius', label: 'Radius', type: 'radius', min: 0.004, max: 0.02, step: 0.001, unit: 'm', default: 0.008, affectsGeometry: true },
    ],
    interfaces: [
      { id: 'a', label: 'Axle A', type: 'hinge_axle', sex: 'bidirectional', localPosition: p => V(0, (p.length as number)/2, 0), localNormal: _ => V(0,1,0), localAxis: _ => V(0,1,0), defaultJointType: 'revolute' },
      { id: 'b', label: 'Axle B', type: 'hinge_axle', sex: 'bidirectional', localPosition: p => V(0, -(p.length as number)/2, 0), localNormal: _ => V(0,-1,0), localAxis: _ => V(0,1,0), defaultJointType: 'revolute' },
    ],
    buildMesh(p) {
      const g = new THREE.Group()
      g.add(mesh(new THREE.CylinderGeometry(p.radius as number, p.radius as number, p.length as number, 12), matSt))
      return g
    },
    urdfVisual(p) { return { type: 'cylinder', radius: p.radius as number, length: p.length as number } },
    mass(p) { return 7800 * Math.PI * (p.radius as number) ** 2 * (p.length as number) * 0.35 },
    inertia(p) { return cylInertia(this.mass(p), p.radius as number, p.length as number) },
  },
  {
    id: 'link.arm.single',
    name: 'Single Arm Link',
    category: 'links',
    description: 'Single-branch linkage arm with two end mounts.',
    parameters: [
      { id: 'length', label: 'Length', type: 'length', min: 0.04, max: 0.3, step: 0.005, unit: 'm', default: 0.12, affectsGeometry: true },
      { id: 'radius', label: 'Radius', type: 'radius', min: 0.004, max: 0.02, step: 0.001, unit: 'm', default: 0.008, affectsGeometry: true },
    ],
    interfaces: [
      { id: 'root', label: 'Root End', type: 'link_end', sex: 'bidirectional', localPosition: p => V(0, (p.length as number)/2, 0), localNormal: _ => V(0,1,0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed' },
      { id: 'tip', label: 'Tip End', type: 'link_end', sex: 'bidirectional', localPosition: p => V(0, -(p.length as number)/2, 0), localNormal: _ => V(0,-1,0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed' },
    ],
    buildMesh(p) {
      const g = new THREE.Group()
      const l = p.length as number, r = p.radius as number
      g.add(mesh(new THREE.CylinderGeometry(r, r, l, 12), matAl))
      return g
    },
    urdfVisual(p) { return { type: 'cylinder', radius: p.radius as number, length: p.length as number } },
    mass(p) { return 2700 * Math.PI * (p.radius as number) ** 2 * (p.length as number) * 0.55 },
    inertia(p) { return cylInertia(this.mass(p), p.radius as number, p.length as number) },
  },
  {
    id: 'link.arm.double',
    name: 'Double Arm Link',
    category: 'links',
    description: 'Forked dual-rail link with paired end mounts.',
    parameters: [
      { id: 'length', label: 'Length', type: 'length', min: 0.05, max: 0.35, step: 0.005, unit: 'm', default: 0.14, affectsGeometry: true },
      { id: 'gap', label: 'Rail Gap', type: 'length', min: 0.01, max: 0.06, step: 0.002, unit: 'm', default: 0.025, affectsGeometry: true },
      { id: 'radius', label: 'Rail Radius', type: 'radius', min: 0.003, max: 0.012, step: 0.001, unit: 'm', default: 0.006, affectsGeometry: true },
    ],
    interfaces: [
      { id: 'root', label: 'Root End', type: 'link_end', sex: 'bidirectional', localPosition: p => V(0, (p.length as number)/2, 0), localNormal: _ => V(0,1,0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed' },
      { id: 'tip', label: 'Tip End', type: 'link_end', sex: 'bidirectional', localPosition: p => V(0, -(p.length as number)/2, 0), localNormal: _ => V(0,-1,0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed' },
    ],
    buildMesh(p) {
      const g = new THREE.Group()
      const l = p.length as number, r = p.radius as number, gap = p.gap as number
      const left = mesh(new THREE.CylinderGeometry(r, r, l, 10), matAl)
      left.position.x = -gap/2
      const right = mesh(new THREE.CylinderGeometry(r, r, l, 10), matAl)
      right.position.x = gap/2
      g.add(left, right)
      const bridgeTop = mesh(new THREE.BoxGeometry(gap + 2*r, 0.008, 0.008), matSt)
      bridgeTop.position.y = l/2 - 0.004
      const bridgeBottom = bridgeTop.clone()
      bridgeBottom.position.y = -l/2 + 0.004
      g.add(bridgeTop, bridgeBottom)
      return g
    },
    urdfVisual(p) { return { type: 'box', size: [(p.gap as number) + 2*(p.radius as number), p.length as number, 2*(p.radius as number)] } },
    mass(_p) { return 0.065 },
    inertia(p) { return boxInertia(0.065, (p.gap as number) + 2*(p.radius as number), p.length as number, 2*(p.radius as number)) },
  },
  {
    id: 'foot.pad.basic',
    name: 'Foot Pad',
    category: 'feet',
    description: 'Rubberized ground-contact pad.',
    parameters: [
      { id: 'radius', label: 'Pad Radius', type: 'radius', min: 0.01, max: 0.05, step: 0.002, unit: 'm', default: 0.022, affectsGeometry: true },
      { id: 'height', label: 'Pad Height', type: 'length', min: 0.005, max: 0.03, step: 0.001, unit: 'm', default: 0.012, affectsGeometry: true },
    ],
    interfaces: [
      { id: 'mount', label: 'Pad Mount', type: 'foot_mount', sex: 'input', localPosition: p => V(0, (p.height as number)/2, 0), localNormal: _ => V(0,1,0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed' },
    ],
    buildMesh(p) {
      const g = new THREE.Group()
      g.add(mesh(new THREE.CylinderGeometry(p.radius as number, (p.radius as number)*0.8, p.height as number, 18), matRb))
      return g
    },
    urdfVisual(p) { return { type: 'cylinder', radius: p.radius as number, length: p.height as number } },
    mass(p) { return 1100 * Math.PI * (p.radius as number) ** 2 * (p.height as number) * 0.85 },
    inertia(p) { return cylInertia(this.mass(p), p.radius as number, p.height as number) },
  },
  {
    id: 'mount.universal.flat',
    name: 'Universal Flat Mount',
    category: 'mounts',
    description: 'Adapter plate that can mate with any node class.',
    parameters: [
      { id: 'size', label: 'Plate Size', type: 'length', min: 0.02, max: 0.12, step: 0.002, unit: 'm', default: 0.04, affectsGeometry: true },
      { id: 'thick', label: 'Thickness', type: 'length', min: 0.002, max: 0.01, step: 0.001, unit: 'm', default: 0.004, affectsGeometry: true },
    ],
    interfaces: [
      { id: 'top', label: 'Top Mount', type: 'universal_mount', sex: 'bidirectional', localPosition: p => V(0, (p.thick as number)/2, 0), localNormal: _ => V(0,1,0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed' },
      { id: 'bottom', label: 'Bottom Mount', type: 'universal_mount', sex: 'bidirectional', localPosition: p => V(0, -(p.thick as number)/2, 0), localNormal: _ => V(0,-1,0), localAxis: _ => V(1,0,0), defaultJointType: 'fixed' },
    ],
    buildMesh(p) {
      const g = new THREE.Group()
      const s = p.size as number, t = p.thick as number
      g.add(mesh(new THREE.BoxGeometry(s, t, s), matAc))
      return g
    },
    urdfVisual(p) { return { type: 'box', size: [p.size as number, p.thick as number, p.size as number] } },
    mass(p) { return 2700 * (p.size as number) * (p.thick as number) * (p.size as number) * 0.75 },
    inertia(p) { return boxInertia(this.mass(p), p.size as number, p.thick as number, p.size as number) },
  },
]

export function getPartDef(id: string): RobotPartDefinition | undefined {
  return PART_LIBRARY.find(p => p.id === id)
}

export function defaultParams(def: RobotPartDefinition): ParamValues {
  const p: ParamValues = {}
  for (const param of def.parameters) p[param.id] = param.default
  return p
}

import * as THREE from 'three'
import type { AssemblyGraph } from './assemblyGraph'
import { getPartDef } from './partLibrary'
import type { URDFGeomSpec } from './partLibrary'
import { computeConnectionTransform } from './assemblyRenderer'

// ── Helpers ───────────────────────────────────────────────────────────────────

function fmt(n: number, dp = 6) { return n.toFixed(dp) }
function v3(v: THREE.Vector3 | [number,number,number]) {
  if (Array.isArray(v)) return `${fmt(v[0])} ${fmt(v[1])} ${fmt(v[2])}`
  return `${fmt(v.x)} ${fmt(v.y)} ${fmt(v.z)}`
}
function rpy(m: THREE.Matrix4) {
  const euler = new THREE.Euler().setFromRotationMatrix(m, 'XYZ')
  return `${fmt(euler.x)} ${fmt(euler.y)} ${fmt(euler.z)}`
}

function geomElement(spec: URDFGeomSpec): string {
  switch (spec.type) {
    case 'box':
      return `<box size="${fmt(spec.size![0])} ${fmt(spec.size![1])} ${fmt(spec.size![2])}"/>`
    case 'cylinder':
      return `<cylinder radius="${fmt(spec.radius!)}" length="${fmt(spec.length!)}"/>`
    case 'sphere':
      return `<sphere radius="${fmt(spec.radius!)}"/>`
  }
}

function inertiaElement(i: { ixx:number; iyy:number; izz:number; ixy:number; ixz:number; iyz:number }): string {
  return `<inertia ixx="${fmt(i.ixx)}" ixy="${fmt(i.ixy)}" ixz="${fmt(i.ixz)}" iyy="${fmt(i.iyy)}" iyz="${fmt(i.iyz)}" izz="${fmt(i.izz)}"/>`
}

// ── Generator ─────────────────────────────────────────────────────────────────

export interface URDFGenOptions {
  robotName?:          string
  addGravityLink?:     boolean   // add a fixed world→base joint
  selfCollision?:      boolean
}

export function generateURDF(graph: AssemblyGraph, opts: URDFGenOptions = {}): string {
  if (graph.isEmpty()) return `<?xml version="1.0"?>\n<robot name="${opts.robotName ?? 'empty_robot'}">\n</robot>`

  const robotName = opts.robotName ?? 'assembled_robot'
  const links: string[]  = []
  const joints: string[] = []


  graph.walk((inst, parentConn, _depth) => {
    const def = getPartDef(inst.definitionId)
    if (!def) return

    const linkName = `${def.id}_${inst.instanceId}`
    const mass     = def.mass(inst.params)
    const inertia  = def.inertia(inst.params)
    const geomSpec = def.urdfVisual(inst.params)

    // ── Link ────────────────────────────────────────────────────────────────
    links.push(`  <link name="${linkName}">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="${fmt(mass)}"/>
      ${inertiaElement(inertia)}
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>${geomElement(geomSpec)}</geometry>
      <material name="${def.category}_mat">
        <color rgba="${categoryColor(def.category)}"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>${geomElement(geomSpec)}</geometry>
    </collision>
  </link>`)

    // ── Joint ────────────────────────────────────────────────────────────────
    if (parentConn) {
      const parentInst = graph.getInstance(parentConn.parentInstanceId)!
      const parentDef  = getPartDef(parentInst.definitionId)!
      const parentLinkName = `${parentDef.id}_${parentInst.instanceId}`

      // Local transform of child relative to parent (connection transform)
      const localMat = computeConnectionTransform(
        parentDef, parentInst.params, parentConn.parentInterfaceId,
        def,       inst.params,       parentConn.childInterfaceId,
      )

      const localPos = new THREE.Vector3().setFromMatrixPosition(localMat)
      const j = parentConn.joint

      const axisXYZ = j.axis.map(a => fmt(a, 4)).join(' ')

      let limitEl = ''
      if (j.type === 'revolute' || j.type === 'prismatic') {
        limitEl = `\n      <limit lower="${fmt(j.lower ?? -3.14159)}" upper="${fmt(j.upper ?? 3.14159)}" effort="${fmt(j.effort ?? 10)}" velocity="${fmt(j.velocity ?? 2)}"/>`
        if (j.damping !== undefined || j.friction !== undefined) {
          limitEl += `\n      <dynamics damping="${fmt(j.damping ?? 0)}" friction="${fmt(j.friction ?? 0)}"/>`
        }
      }

      joints.push(`  <joint name="joint_${parentConn.connectionId}" type="${j.type}">
    <parent link="${parentLinkName}"/>
    <child link="${linkName}"/>
    <origin xyz="${v3(localPos)}" rpy="${rpy(localMat)}"/>
    <axis xyz="${axisXYZ}"/>${limitEl}
  </joint>`)
    } else if (opts.addGravityLink) {
      // Attach root to world via fixed joint
      joints.push(`  <link name="world"/>
  <joint name="world_fixed" type="fixed">
    <parent link="world"/>
    <child link="${linkName}"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>`)
    }
  })

  const body = [...links, ...joints].join('\n\n')
  return [
    '<?xml version="1.0"?>',
    `<robot name="${robotName}" xmlns:xacro="http://www.ros.org/wiki/xacro">`,
    '',
    body,
    '',
    '</robot>',
  ].join('\n')
}

function categoryColor(cat: string): string {
  switch (cat) {
    case 'structure': return '0.63 0.65 0.73 1'
    case 'joints':    return '0.82 0.47 0.24 1'
    case 'links':     return '0.31 0.66 0.95 1'
    case 'feet':      return '0.28 0.66 0.32 1'
    case 'mounts':    return '0.66 0.52 0.88 1'
    default:            return '0.50 0.50 0.50 1'
  }
}

// ── MJCF generator (flat bodies, MuJoCo world-frame) ─────────────────────────

export function generateMJCF(graph: AssemblyGraph, robotName = 'assembled_robot'): string {
  if (graph.isEmpty()) return `<mujoco model="${robotName}"/>`

  // Collect flat list of (body, localMat) pairs — MuJoCo allows flat worldbody
  const bodyLines: string[] = []

  graph.walk((inst, parentConn, _depth) => {
    const def = getPartDef(inst.definitionId)
    if (!def) return
    const bodyName = `${def.id}_${inst.instanceId}`
    const geomStr  = mjcfGeom(def.urdfVisual(inst.params))
    const mass     = def.mass(inst.params)

    let pos = '0 0 0', euler = '0 0 0'
    if (parentConn) {
      const parentInst = graph.getInstance(parentConn.parentInstanceId)!
      const parentDef  = getPartDef(parentInst.definitionId)!
      const localMat   = computeConnectionTransform(
        parentDef, parentInst.params, parentConn.parentInterfaceId,
        def,       inst.params,       parentConn.childInterfaceId,
      )
      const lp = new THREE.Vector3().setFromMatrixPosition(localMat)
      const le = new THREE.Euler().setFromRotationMatrix(localMat, 'XYZ')
      pos   = v3(lp)
      euler = [le.x, le.y, le.z].map(n => fmt(n, 4)).join(' ')
    }

    const j = parentConn?.joint
    const jointEl = j && j.type !== 'fixed'
      ? `    <joint name="${parentConn!.connectionId}" type="${j.type === 'revolute' ? 'hinge' : 'slide'}" axis="${j.axis.join(' ')}" range="${fmt(j.lower??-3.14)} ${fmt(j.upper??3.14)}" damping="${fmt(j.damping??0)}" frictionloss="${fmt(j.friction??0)}"/>`
      : ''

    bodyLines.push(
      `    <body name="${bodyName}" pos="${pos}" euler="${euler}">`,
      ...(jointEl ? [jointEl] : []),
      `      <geom ${geomStr} mass="${fmt(mass)}" contype="1" conaffinity="1"/>`,
      `    </body>`,
    )
  })

  return [
    `<mujoco model="${robotName}">`,
    '  <compiler angle="radian" inertiafromgeom="auto"/>',
    '  <option gravity="0 0 -9.81"/>',
    '  <worldbody>',
    ...bodyLines,
    '  </worldbody>',
    '</mujoco>',
  ].join('\n')
}

function mjcfGeom(spec: URDFGeomSpec): string {
  switch (spec.type) {
    case 'box':      return `type="box" size="${spec.size!.map(s=>fmt(s/2,4)).join(' ')}"`
    case 'cylinder': return `type="cylinder" size="${fmt(spec.radius!,4)} ${fmt((spec.length!)/2,4)}"`
    case 'sphere':   return `type="sphere" size="${fmt(spec.radius!,4)}"`
  }
}

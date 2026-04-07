import * as THREE from 'three'
import type { AssemblyGraph, PartInstance, ConnectionEdge } from './assemblyGraph'
import { getPartDef, interfacesCompatible } from './partLibrary'
import type { ParamValues } from './partLibrary'

// ── Interface ring colours ────────────────────────────────────────────────────

const COL_AVAILABLE = 0x44aaff
const COL_COMPAT    = 0x44ffaa
const COL_OCCUPIED  = 0x555566

// ── Ring geometry helper ──────────────────────────────────────────────────────

function makeRingMesh(color: number): THREE.Mesh {
  const geo = new THREE.TorusGeometry(0.018, 0.003, 6, 20)
  const mat = new THREE.MeshStandardMaterial({
    color, emissive: color, emissiveIntensity: 0.6,
    roughness: 0.3, metalness: 0.0,
    transparent: true, opacity: 0.9, depthWrite: false,
  })
  const m = new THREE.Mesh(geo, mat)
  m.userData.isRing = true
  return m
}

function orientRingToNormal(ring: THREE.Mesh, normal: THREE.Vector3) {
  const up = new THREE.Vector3(0, 1, 0)
  const n  = normal.clone().normalize()
  if (Math.abs(n.dot(up)) > 0.999) {
    ring.rotation.x = n.y > 0 ? 0 : Math.PI
  } else {
    const q = new THREE.Quaternion().setFromUnitVectors(up, n)
    ring.quaternion.copy(q)
  }
}

// ── Connection transform ──────────────────────────────────────────────────────

export function computeConnectionTransform(
  parentDef: ReturnType<typeof getPartDef>,
  parentParams: ParamValues,
  parentIfaceId: string,
  childDef: ReturnType<typeof getPartDef>,
  childParams: ParamValues,
  childIfaceId: string,
): THREE.Matrix4 {
  if (!parentDef || !childDef) return new THREE.Matrix4()

  const pIface = parentDef.interfaces.find(i => i.id === parentIfaceId)
  const cIface = childDef.interfaces.find(i => i.id === childIfaceId)
  if (!pIface || !cIface) return new THREE.Matrix4()

  const pPos    = pIface.localPosition(parentParams)
  const pNormal = pIface.localNormal(parentParams).normalize()

  const cPos    = cIface.localPosition(childParams)
  const cNormal = cIface.localNormal(childParams).normalize()

  // Align child interface normal to opposite of parent interface normal
  const targetNormal = pNormal.clone().negate()
  const sourceNormal = cNormal.clone()

  let rot: THREE.Quaternion
  const dot = sourceNormal.dot(targetNormal)
  if (dot < -0.9999) {
    const perp = new THREE.Vector3(1, 0, 0)
    if (Math.abs(sourceNormal.dot(perp)) > 0.9) perp.set(0, 1, 0)
    const axis = perp.cross(sourceNormal).normalize()
    rot = new THREE.Quaternion().setFromAxisAngle(axis, Math.PI)
  } else {
    rot = new THREE.Quaternion().setFromUnitVectors(sourceNormal, targetNormal)
  }

  // Child position in parent space: parent interface pos – rotated child interface pos
  const rotatedChildPos = cPos.clone().applyQuaternion(rot)
  const childOrigin     = pPos.clone().sub(rotatedChildPos)

  return new THREE.Matrix4().compose(childOrigin, rot, new THREE.Vector3(1, 1, 1))
}

// ── HitResult ─────────────────────────────────────────────────────────────────

export type HitResult =
  | { type: 'instance';  instanceId: string; interfaceId?: undefined }
  | { type: 'interface'; instanceId: string; interfaceId: string }

// ── AssemblyRenderer ──────────────────────────────────────────────────────────

export class AssemblyRenderer {
  readonly group = new THREE.Group()

  private graph: AssemblyGraph | null = null
  private unsubscribe: (() => void) | null = null

  private meshMap = new Map<string, THREE.Group>()   // instanceId → part group
  private ringMap = new Map<string, THREE.Mesh>()    // "instanceId:ifaceId" → ring mesh
  private connectionLines: THREE.Line[] = []

  private selectedInstanceId: string | null = null
  private pendingPartDefId:   string | null = null

  private ghostGroup: THREE.Group | null = null

  private readonly _connLineMat = new THREE.LineBasicMaterial({ color: 0x4488aa, transparent: true, opacity: 0.5 })

  constructor(scene: THREE.Scene) {
    scene.add(this.group)
  }

  // ── Bind / Unbind ─────────────────────────────────────────────────────────

  bind(graph: AssemblyGraph) {
    this.unbind()
    this.graph = graph
    this.unsubscribe = graph.on(ev => {
      switch (ev.type) {
        case 'instance_added':   this.rebuildAll();               break
        case 'instance_removed': this.rebuildAll();               break
        case 'params_changed':   this.rebuildInstance(ev.instanceId); break
        case 'joint_changed':    this.applyJointValue(ev.connectionId); break
        case 'cleared':          this.clear();                    break
        case 'restored':         this.rebuildAll();               break
      }
    })
  }

  unbind() {
    this.unsubscribe?.()
    this.unsubscribe = null
    this.graph = null
  }

  // ── Full rebuild ──────────────────────────────────────────────────────────

  rebuildAll() {
    if (!this.graph) return

    while (this.group.children.length) this.group.remove(this.group.children[0])
    this.meshMap.clear()
    this.ringMap.clear()
    this.connectionLines = []

    this.graph.walk((inst, parentConn) => {
      this.addInstanceMesh(inst, parentConn)
    })

    this._drawConnectionLines()

    // Restore rings
    if (this.pendingPartDefId) {
      this._showAllCompatibleRings(this.pendingPartDefId)
    } else if (this.selectedInstanceId && this.meshMap.has(this.selectedInstanceId)) {
      this._showInterfacesInner(this.selectedInstanceId)
    }

    // Restore selection highlight
    if (this.selectedInstanceId) {
      this._applyHighlight(this.selectedInstanceId)
    }
  }

  // ── Per-instance mesh ─────────────────────────────────────────────────────

  private addInstanceMesh(inst: PartInstance, parentConn: ConnectionEdge | undefined) {
    const def = getPartDef(inst.definitionId)
    if (!def) return

    const partGroup = def.buildMesh(inst.params)
    partGroup.name = `part_${inst.instanceId}`
    partGroup.userData.instanceId = inst.instanceId

    // Clone every mesh material so instances never share material objects.
    // Shared materials cause selection / visibility changes to bleed across parts.
    partGroup.traverse(obj => {
      if (obj instanceof THREE.Mesh) {
        obj.userData.instanceId = inst.instanceId
        if (obj.material) obj.material = (obj.material as THREE.Material).clone()
      }
    })

    const worldMat = this._computeWorldMatrix(inst, parentConn)
    partGroup.position.setFromMatrixPosition(worldMat)
    partGroup.quaternion.setFromRotationMatrix(worldMat)

    this.group.add(partGroup)
    this.meshMap.set(inst.instanceId, partGroup)
  }

  private _computeWorldMatrix(inst: PartInstance, parentConn: ConnectionEdge | undefined): THREE.Matrix4 {
    if (!parentConn || !this.graph) return new THREE.Matrix4()

    const parentGroup = this.meshMap.get(parentConn.parentInstanceId)
    if (!parentGroup) return new THREE.Matrix4()

    const parentWorldMat = new THREE.Matrix4().compose(parentGroup.position, parentGroup.quaternion, parentGroup.scale)

    const parentInst   = this.graph.getInstance(parentConn.parentInstanceId)!
    const parentDef    = getPartDef(parentInst.definitionId)!
    const childDef     = getPartDef(inst.definitionId)!

    const localMat = computeConnectionTransform(
      parentDef, parentInst.params, parentConn.parentInterfaceId,
      childDef,  inst.params,       parentConn.childInterfaceId,
    )
    return parentWorldMat.clone().multiply(localMat)
  }

  // ── Param-only rebuild ────────────────────────────────────────────────────

  private rebuildInstance(instanceId: string) {
    const old = this.meshMap.get(instanceId)
    if (old) this.group.remove(old)
    this.meshMap.delete(instanceId)

    for (const key of [...this.ringMap.keys()]) {
      if (key.startsWith(instanceId + ':')) {
        this.ringMap.get(key)!.parent?.remove(this.ringMap.get(key)!)
        this.ringMap.delete(key)
      }
    }

    if (!this.graph) return
    const inst = this.graph.getInstance(instanceId)
    if (!inst) return
    const parentConn = this.graph.getParentConnection(instanceId)
    this.addInstanceMesh(inst, parentConn)

    if (instanceId === this.selectedInstanceId) {
      this._applyHighlight(instanceId)
      this._showInterfacesInner(instanceId)
    }

    for (const conn of this.graph.getChildConnections(instanceId)) {
      this.rebuildInstance(conn.childInstanceId)
    }
  }

  // ── Connection lines ──────────────────────────────────────────────────────

  private _drawConnectionLines() {
    if (!this.graph) return
    this.graph.walk((inst, parentConn) => {
      if (!parentConn) return
      const pGrp = this.meshMap.get(parentConn.parentInstanceId)
      const cGrp = this.meshMap.get(inst.instanceId)
      if (!pGrp || !cGrp) return
      const geo  = new THREE.BufferGeometry().setFromPoints([pGrp.position.clone(), cGrp.position.clone()])
      const line = new THREE.Line(geo, this._connLineMat)
      line.renderOrder = -1
      this.group.add(line)
      this.connectionLines.push(line)
    })
  }

  // ── Joint animation ───────────────────────────────────────────────────────

  private applyJointValue(connectionId: string) {
    if (!this.graph) return
    const conn = this.graph.getConnection(connectionId)
    if (!conn) return
    const childGroup = this.meshMap.get(conn.childInstanceId)
    if (!childGroup) return
    const joint = conn.joint
    if (joint.type === 'fixed' || joint.value === undefined) return
    const inst       = this.graph.getInstance(conn.childInstanceId)!
    const parentConn = this.graph.getParentConnection(conn.childInstanceId)
    const baseMat    = this._computeWorldMatrix(inst, parentConn)
    if (joint.type === 'revolute') {
      const axis   = new THREE.Vector3(...joint.axis).normalize()
      baseMat.multiply(new THREE.Matrix4().makeRotationAxis(axis, joint.value))
    } else if (joint.type === 'prismatic') {
      const axis   = new THREE.Vector3(...joint.axis).normalize()
      baseMat.multiply(new THREE.Matrix4().makeTranslation(axis.x * joint.value, axis.y * joint.value, axis.z * joint.value))
    }
    childGroup.position.setFromMatrixPosition(baseMat)
    childGroup.quaternion.setFromRotationMatrix(baseMat)
    for (const c of this.graph.getChildConnections(conn.childInstanceId)) {
      this.rebuildInstance(c.childInstanceId)
    }
  }

  // ── Interface rings ───────────────────────────────────────────────────────

  showInterfacesFor(instanceId: string) {
    this.hideAllRings()
    this._showInterfacesInner(instanceId)
  }

  hideAllRings() {
    for (const ring of this.ringMap.values()) ring.parent?.remove(ring)
    this.ringMap.clear()
  }

  setPendingPart(defId: string | null) {
    this.pendingPartDefId = defId
    if (defId) {
      this._showAllCompatibleRings(defId)
    } else if (this.selectedInstanceId) {
      this.showInterfacesFor(this.selectedInstanceId)
    } else {
      this.hideAllRings()
    }
  }

  private _showAllCompatibleRings(_defId: string) {
    this.hideAllRings()
    if (!this.graph) return
    this.graph.walk(inst => this._showInterfacesInner(inst.instanceId))
  }

  private _showInterfacesInner(instanceId: string) {
    if (!this.graph) return
    const inst = this.graph.getInstance(instanceId)
    if (!inst) return
    const def = getPartDef(inst.definitionId)
    if (!def) return
    const partGroup = this.meshMap.get(instanceId)
    if (!partGroup) return
    const occupied = this.graph.occupiedInterfaces(instanceId)

    for (const iface of def.interfaces) {
      const key   = `${instanceId}:${iface.id}`
      if (this.ringMap.has(key)) continue
      const isOcc = occupied.has(iface.id)
      let color   = isOcc ? COL_OCCUPIED : COL_AVAILABLE
      if (!isOcc && this.pendingPartDefId) {
        const pendingDef = getPartDef(this.pendingPartDefId)
        if (pendingDef) {
          const hasCompat = pendingDef.interfaces.some(pi => interfacesCompatible(pi.type, iface.type))
          color = hasCompat ? COL_COMPAT : COL_AVAILABLE
        }
      }
      const ring = makeRingMesh(color)
      ring.position.copy(iface.localPosition(inst.params))
      orientRingToNormal(ring, iface.localNormal(inst.params))
      ring.userData.instanceId  = instanceId
      ring.userData.interfaceId = iface.id
      ring.userData.isRing      = true
      partGroup.add(ring)
      this.ringMap.set(key, ring)
    }
  }

  // ── Selection highlight ───────────────────────────────────────────────────

  selectInstance(instanceId: string | null) {
    // Remove highlight from previously selected
    if (this.selectedInstanceId) {
      this._clearHighlight(this.selectedInstanceId)
    }

    this.hideAllRings()
    this.selectedInstanceId = instanceId

    if (!instanceId) {
      if (this.pendingPartDefId) this._showAllCompatibleRings(this.pendingPartDefId)
      return
    }

    this._applyHighlight(instanceId)

    if (this.pendingPartDefId) {
      this._showAllCompatibleRings(this.pendingPartDefId)
    } else {
      this._showInterfacesInner(instanceId)
    }
  }

  private _applyHighlight(instanceId: string) {
    const grp = this.meshMap.get(instanceId)
    if (!grp) return
    grp.traverse(o => {
      if (!(o instanceof THREE.Mesh) || o.userData.isRing) return
      const mat = o.material as THREE.MeshStandardMaterial
      if (!('_origEmissiveHex' in mat.userData)) {
        mat.userData._origEmissiveHex       = mat.emissive.getHex()
        mat.userData._origEmissiveIntensity = mat.emissiveIntensity
      }
      mat.emissive.setHex(0x224488)
      mat.emissiveIntensity = 0.9
    })
  }

  private _clearHighlight(instanceId: string) {
    const grp = this.meshMap.get(instanceId)
    if (!grp) return
    grp.traverse(o => {
      if (!(o instanceof THREE.Mesh) || o.userData.isRing) return
      const mat = o.material as THREE.MeshStandardMaterial
      mat.emissive.setHex(mat.userData._origEmissiveHex       ?? 0x000000)
      mat.emissiveIntensity =          mat.userData._origEmissiveIntensity ?? 0
    })
  }

  getSelectedInstanceId() { return this.selectedInstanceId }

  // ── Raycasting ────────────────────────────────────────────────────────────

  raycast(raycaster: THREE.Raycaster): HitResult | null {
    const targets: THREE.Object3D[] = []
    this.group.traverse(o => { if (o instanceof THREE.Mesh) targets.push(o) })
    const hits = raycaster.intersectObjects(targets, false)

    // Rings take priority
    for (const h of hits) {
      if (h.object.userData.isRing) {
        return { type: 'interface', instanceId: h.object.userData.instanceId, interfaceId: h.object.userData.interfaceId }
      }
    }
    for (const h of hits) {
      if (h.object.userData.instanceId) {
        return { type: 'instance', instanceId: h.object.userData.instanceId }
      }
    }
    return null
  }

  // ── Ghost preview ─────────────────────────────────────────────────────────

  showGhostAt(defId: string, params: ParamValues, parentInstanceId: string, parentIfaceId: string, childIfaceId: string) {
    this.clearGhost()
    if (!this.graph) return
    const def   = getPartDef(defId)
    const pInst = this.graph.getInstance(parentInstanceId)
    const pDef  = pInst ? getPartDef(pInst.definitionId) : null
    if (!def || !pInst || !pDef) return

    const ghost = def.buildMesh(params)
    ghost.traverse(o => {
      if (o instanceof THREE.Mesh) {
        const mat = (o.material as THREE.MeshStandardMaterial).clone()
        mat.transparent = true
        mat.opacity     = 0.40
        mat.depthWrite  = false
        o.material = mat
      }
    })

    const pGrp     = this.meshMap.get(parentInstanceId)
    if (!pGrp) return
    const pWorldMat = new THREE.Matrix4().compose(pGrp.position, pGrp.quaternion, pGrp.scale)
    const localMat  = computeConnectionTransform(pDef, pInst.params, parentIfaceId, def, params, childIfaceId)
    const worldMat  = pWorldMat.clone().multiply(localMat)

    ghost.position.setFromMatrixPosition(worldMat)
    ghost.quaternion.setFromRotationMatrix(worldMat)
    this.group.add(ghost)
    this.ghostGroup = ghost
  }

  clearGhost() {
    if (this.ghostGroup) { this.group.remove(this.ghostGroup); this.ghostGroup = null }
  }

  // ── Queries ───────────────────────────────────────────────────────────────

  getMeshGroup(instanceId: string): THREE.Group | undefined {
    return this.meshMap.get(instanceId)
  }

  getBoundingBox(instanceId: string): THREE.Box3 | null {
    const grp = this.meshMap.get(instanceId)
    if (!grp) return null
    const box = new THREE.Box3()
    grp.traverse(o => {
      if (o instanceof THREE.Mesh && !o.userData.isRing) box.union(new THREE.Box3().setFromObject(o))
    })
    return box.isEmpty() ? null : box
  }

  // ── Cleanup ───────────────────────────────────────────────────────────────

  clear() {
    while (this.group.children.length) this.group.remove(this.group.children[0])
    this.meshMap.clear()
    this.ringMap.clear()
    this.connectionLines = []
    this.ghostGroup = null
    this.selectedInstanceId = null
  }

  dispose() {
    this.unbind()
    this.clear()
    this.group.parent?.remove(this.group)
  }
}

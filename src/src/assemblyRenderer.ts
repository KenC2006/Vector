import * as THREE from 'three'
import type { AssemblyGraph, ConnectionEdge, PartInstance } from './assemblyGraph'
import { getPartDef, interfacesCompatible } from './partLibrary'
import type { ParamValues, RobotPartDefinition } from './partLibrary'

// ── Constants ─────────────────────────────────────────────────────────────────

const RING_RADIUS      = 0.012
const RING_TUBE        = 0.0018
const RING_SEG_MAJOR   = 18
const RING_SEG_MINOR   = 6

const COL_AVAILABLE = 0x00d4ff  // teal
const COL_OCCUPIED  = 0xff6633  // orange
const COL_SELECTED  = 0x44ff88  // green
const COL_COMPAT    = 0xffee00  // yellow — compatible with pending part

// ── Helpers ───────────────────────────────────────────────────────────────────

function makeMat(color: number, opacity = 1) {
  return new THREE.MeshStandardMaterial({
    color, roughness: 0.3, metalness: 0.6,
    transparent: opacity < 1, opacity,
    emissive: color, emissiveIntensity: 0.25,
  })
}

const ringGeo = new THREE.TorusGeometry(RING_RADIUS, RING_TUBE, RING_SEG_MINOR, RING_SEG_MAJOR)

function makeRingMesh(color: number): THREE.Mesh {
  const m = new THREE.Mesh(ringGeo, makeMat(color))
  m.castShadow = false; m.receiveShadow = false
  return m
}

/**
 * Orient a ring so its face (torus axis) aligns with the given normal vector.
 * A torus by default has its axis along +Y.
 */
function orientRingToNormal(ring: THREE.Object3D, normal: THREE.Vector3) {
  const up = new THREE.Vector3(0, 1, 0)
  const n  = normal.clone().normalize()
  if (Math.abs(n.dot(up)) > 0.9999) {
    ring.quaternion.setFromAxisAngle(new THREE.Vector3(1,0,0), n.y < 0 ? Math.PI : 0)
  } else {
    ring.quaternion.setFromUnitVectors(up, n)
  }
}

// ── Connection transform math ─────────────────────────────────────────────────

/**
 * Compute the transform of child relative to parent so that:
 *   childIface (on child, in child local space) snaps face-to-face with parentIface (on parent).
 *
 * Returns a THREE.Matrix4 representing the child's local transform IN PARENT SPACE.
 */
export function computeConnectionTransform(
  parentDef: RobotPartDefinition,
  parentParams: ParamValues,
  parentIfaceId: string,
  childDef: RobotPartDefinition,
  childParams: ParamValues,
  childIfaceId: string,
): THREE.Matrix4 {
  const pIface = parentDef.interfaces.find(i => i.id === parentIfaceId)!
  const cIface = childDef.interfaces.find(i => i.id === childIfaceId)!

  const pPos    = pIface.localPosition(parentParams)
  const pNormal = pIface.localNormal(parentParams).normalize()

  const cPos    = cIface.localPosition(childParams)
  const cNormal = cIface.localNormal(childParams).normalize()

  // Target: child's interface normal should face OPPOSITE to parent's normal
  const targetNormal = pNormal.clone().negate()

  // Quaternion rotating cNormal → targetNormal
  let q = new THREE.Quaternion()
  if (cNormal.dot(targetNormal) < -0.9999) {
    // 180° flip — pick an arbitrary perpendicular axis
    const perp = new THREE.Vector3(1, 0, 0)
    if (Math.abs(cNormal.dot(perp)) > 0.9) perp.set(0, 1, 0)
    perp.cross(cNormal).normalize()
    q.setFromAxisAngle(perp, Math.PI)
  } else {
    q.setFromUnitVectors(cNormal, targetNormal)
  }

  // Rotate child interface position
  const rotatedCPos = cPos.clone().applyQuaternion(q)

  // Child's position so that its interface aligns with parent's interface
  const childLocalPos = pPos.clone().sub(rotatedCPos)

  // Build matrix
  const mat = new THREE.Matrix4()
  mat.compose(childLocalPos, q, new THREE.Vector3(1, 1, 1))
  return mat
}

// ── AssemblyRenderer ──────────────────────────────────────────────────────────

export interface HitResult {
  type: 'instance' | 'interface'
  instanceId: string
  interfaceId?: string
}

export class AssemblyRenderer {
  /** Root THREE group added to the scene */
  readonly group = new THREE.Group()
  group_name = 'assembly_root'

  // instanceId → the part's mesh group (positioned in world space, child of this.group)
  private meshMap     = new Map<string, THREE.Group>()
  // `${instanceId}:${interfaceId}` → ring mesh
  private ringMap     = new Map<string, THREE.Mesh>()

  // Thin lines showing parent→child connections
  private connectionLines: THREE.Line[] = []
  private readonly _connLineMat = new THREE.LineBasicMaterial({
    color: 0x445566, transparent: true, opacity: 0.40, depthWrite: false,
  })

  private graph: AssemblyGraph | null = null
  private unsubscribe: (() => void) | null = null

  private selectedInstanceId: string | null = null
  private pendingPartDefId: string | null = null   // toolbox selection waiting for connection

  constructor(scene: THREE.Scene) {
    scene.add(this.group)
    this.group.name = 'assembly_root'
  }

  // ── Graph binding ─────────────────────────────────────────────────────────

  bind(graph: AssemblyGraph) {
    this.unsubscribe?.()
    this.graph = graph
    this.unsubscribe = graph.on(ev => {
      switch (ev.type) {
        case 'instance_added':   this.rebuildAll(); break
        case 'instance_removed': this.rebuildAll(); break
        case 'params_changed':   this.rebuildInstance(ev.instanceId); break
        case 'joint_changed':    this.applyJointValue(ev.connectionId); break
        case 'cleared':          this.clear(); break
        case 'restored':         this.rebuildAll(); break
      }
    })
    this.rebuildAll()
  }

  unbind() {
    this.unsubscribe?.()
    this.unsubscribe = null
    this.graph = null
  }

  // ── Full rebuild ──────────────────────────────────────────────────────────

  rebuildAll() {
    if (!this.graph) return

    // Clear scene children but keep the group itself
    while (this.group.children.length) this.group.remove(this.group.children[0])
    this.meshMap.clear()
    this.ringMap.clear()
    this.connectionLines = []

    this.graph.walk((inst, parentConn, _depth) => {
      this.addInstanceMesh(inst, parentConn)
    })

    // Draw parent→child lines
    this._drawConnectionLines()

    // Restore rings — all if pending, else just selected
    if (this.pendingPartDefId) {
      this._showAllCompatibleRings(this.pendingPartDefId)
    } else if (this.selectedInstanceId && this.meshMap.has(this.selectedInstanceId)) {
      this.showInterfacesFor(this.selectedInstanceId)
    }
  }

  private _drawConnectionLines() {
    if (!this.graph) return
    this.graph.walk((inst, parentConn) => {
      if (!parentConn) return
      const pGrp = this.meshMap.get(parentConn.parentInstanceId)
      const cGrp = this.meshMap.get(inst.instanceId)
      if (!pGrp || !cGrp) return
      const geo = new THREE.BufferGeometry().setFromPoints([
        pGrp.position.clone(), cGrp.position.clone(),
      ])
      const line = new THREE.Line(geo, this._connLineMat)
      line.renderOrder = -1
      this.group.add(line)
      this.connectionLines.push(line)
    })
  }

  // ── Per-instance mesh ─────────────────────────────────────────────────────

  private addInstanceMesh(inst: PartInstance, parentConn: ConnectionEdge | undefined) {
    const def = getPartDef(inst.definitionId)
    if (!def) return

    const partGroup = def.buildMesh(inst.params)
    partGroup.name = `part_${inst.instanceId}`
    partGroup.userData.instanceId = inst.instanceId

    // Clone every mesh material so instances never share material objects.
    // Without this, applyXRay / selectInstance mutations bleed across all parts
    // that use the same library-level singleton material (e.g. two servos both
    // using `servoBody`) and make the entire scene go transparent or black.
    partGroup.traverse(obj => {
      if (obj instanceof THREE.Mesh) {
        obj.userData.instanceId = inst.instanceId
        if (obj.material) {
          obj.material = (obj.material as THREE.Material).clone()
        }
      }
    })

    // Compute world transform
    const worldMat = this.computeWorldMatrix(inst, parentConn)
    const worldPos = new THREE.Vector3().setFromMatrixPosition(worldMat)

    // Apply user drag offset (world-space translation)
    if (inst.dragOffset) {
      worldPos.x += inst.dragOffset.x
      worldPos.y += inst.dragOffset.y
      worldPos.z += inst.dragOffset.z
    }

    // Apply user drag rotation (local-space delta quaternion)
    const baseQuat = new THREE.Quaternion().setFromRotationMatrix(worldMat)
    if (inst.dragRotation) {
      const delta = new THREE.Quaternion(inst.dragRotation.x, inst.dragRotation.y, inst.dragRotation.z, inst.dragRotation.w)
      baseQuat.multiply(delta)
    }

    partGroup.position.copy(worldPos)
    partGroup.quaternion.copy(baseQuat)

    this.group.add(partGroup)
    this.meshMap.set(inst.instanceId, partGroup)
  }

  private computeWorldMatrix(inst: PartInstance, parentConn: ConnectionEdge | undefined): THREE.Matrix4 {
    if (!parentConn) return new THREE.Matrix4() // root stays at identity

    const parentGroup = this.meshMap.get(parentConn.parentInstanceId)
    if (!parentGroup) return new THREE.Matrix4()

    const parentWorldMat = new THREE.Matrix4()
    parentWorldMat.compose(parentGroup.position, parentGroup.quaternion, parentGroup.scale)

    const parentDef   = getPartDef(this.graph!.getInstance(parentConn.parentInstanceId)!.definitionId)!
    const parentParams = this.graph!.getInstance(parentConn.parentInstanceId)!.params
    const childDef    = getPartDef(inst.definitionId)!

    const localMat = computeConnectionTransform(
      parentDef, parentParams, parentConn.parentInterfaceId,
      childDef,  inst.params,  parentConn.childInterfaceId,
    )

    return parentWorldMat.clone().multiply(localMat)
  }

  // ── Param-only rebuild ────────────────────────────────────────────────────

  private rebuildInstance(instanceId: string) {
    // Remove old mesh
    const old = this.meshMap.get(instanceId)
    if (old) this.group.remove(old)
    this.meshMap.delete(instanceId)

    // Remove old rings for this instance
    for (const key of [...this.ringMap.keys()]) {
      if (key.startsWith(instanceId + ':')) {
        const ring = this.ringMap.get(key)!
        ring.parent?.remove(ring)
        this.ringMap.delete(key)
      }
    }

    if (!this.graph) return
    const inst = this.graph.getInstance(instanceId)
    if (!inst) return
    const parentConn = this.graph.getParentConnection(instanceId)
    this.addInstanceMesh(inst, parentConn)

    // Rebuild subtree because child positions depend on parent params
    for (const conn of this.graph.getChildConnections(instanceId)) {
      this.rebuildInstance(conn.childInstanceId)
    }

    if (this.selectedInstanceId === instanceId) {
      this.showInterfacesFor(instanceId)
    }
  }

  // ── Joint value animation ─────────────────────────────────────────────────

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
    const baseMat    = this.computeWorldMatrix(inst, parentConn)

    if (joint.type === 'revolute') {
      const axis    = new THREE.Vector3(...joint.axis).normalize()
      const rotMat  = new THREE.Matrix4().makeRotationAxis(axis, joint.value)
      baseMat.multiply(rotMat)
    } else if (joint.type === 'prismatic') {
      const axis    = new THREE.Vector3(...joint.axis).normalize()
      const transMat = new THREE.Matrix4().makeTranslation(axis.x * joint.value, axis.y * joint.value, axis.z * joint.value)
      baseMat.multiply(transMat)
    }

    childGroup.position.setFromMatrixPosition(baseMat)
    childGroup.quaternion.setFromRotationMatrix(baseMat)

    // Children need to follow
    for (const c of this.graph.getChildConnections(conn.childInstanceId)) {
      this.rebuildInstance(c.childInstanceId)
    }
  }

  // ── Interface rings ───────────────────────────────────────────────────────

  showInterfacesFor(instanceId: string) {
    this.hideAllRings()
    this.selectedInstanceId = instanceId
    this._showInterfacesInner(instanceId)
  }

  hideAllRings() {
    for (const ring of this.ringMap.values()) ring.parent?.remove(ring)
    this.ringMap.clear()
  }

  setPendingPart(defId: string | null) {
    this.pendingPartDefId = defId
    if (defId) {
      // Show compatible rings on every part simultaneously
      this._showAllCompatibleRings(defId)
    } else if (this.selectedInstanceId) {
      this.showInterfacesFor(this.selectedInstanceId)
    } else {
      this.hideAllRings()
    }
  }

  /** Show interface rings on ALL assembly parts, coloured by compatibility with the pending part. */
  private _showAllCompatibleRings(_defId: string) {
    this.hideAllRings()
    if (!this.graph) return
    this.graph.walk((inst) => {
      this._showInterfacesInner(inst.instanceId)
    })
  }

  /** Inner ring-drawing shared by showInterfacesFor and _showAllCompatibleRings. */
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
      if (this.ringMap.has(key)) continue  // already drawn
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

  highlightRing(instanceId: string, interfaceId: string) {
    const key  = `${instanceId}:${interfaceId}`
    const ring = this.ringMap.get(key)
    if (ring) (ring.material as THREE.MeshStandardMaterial).color.setHex(COL_SELECTED)
  }

  unhighlightRing(instanceId: string, interfaceId: string) {
    const key  = `${instanceId}:${interfaceId}`
    const ring = this.ringMap.get(key)
    if (!ring) return
    const inst      = this.graph?.getInstance(instanceId)
    const occupied  = inst ? this.graph!.occupiedInterfaces(instanceId) : new Set<string>()
    const isOcc     = occupied.has(interfaceId)
    ;(ring.material as THREE.MeshStandardMaterial).color.setHex(isOcc ? COL_OCCUPIED : COL_AVAILABLE)
  }

  // ── Selection ─────────────────────────────────────────────────────────────

  selectInstance(instanceId: string | null) {
    // Clear emissive on previously selected
    if (this.selectedInstanceId) {
      const old = this.meshMap.get(this.selectedInstanceId)
      if (old) old.traverse(o => {
        if (o instanceof THREE.Mesh && !o.userData.isRing) {
          const mat = o.material as THREE.MeshStandardMaterial
          mat.emissive.setHex(mat.userData._baseEmissiveHex ?? 0x000000)
          mat.emissiveIntensity = mat.userData._baseEmissive ?? 0
        }
      })
    }

    this.hideAllRings()
    this.selectedInstanceId = instanceId

    if (!instanceId) {
      // Even with no selection, show rings on all parts if a pending part is active
      if (this.pendingPartDefId) this._showAllCompatibleRings(this.pendingPartDefId)
      return
    }

    const grp = this.meshMap.get(instanceId)
    if (grp) {
      grp.traverse(o => {
        if (o instanceof THREE.Mesh && !o.userData.isRing) {
          const mat = o.material as THREE.MeshStandardMaterial
          // Save original emissive state once (materials are cloned per-instance so this is safe)
          if (!('_baseEmissive' in mat.userData)) {
            mat.userData._baseEmissive    = mat.emissiveIntensity
            mat.userData._baseEmissiveHex = mat.emissive.getHex()
          }
          // Apply a visible blue-white selection tint
          mat.emissive.setHex(0x224488)
          mat.emissiveIntensity = 0.8
        }
      })
    }

    // Always show rings on ALL parts when a pending part is waiting for connection;
    // otherwise show rings only on the selected part.
    if (this.pendingPartDefId) {
      this._showAllCompatibleRings(this.pendingPartDefId)
    } else {
      this._showInterfacesInner(instanceId)
    }
  }

  getSelectedInstanceId() { return this.selectedInstanceId }

  // ── Raycasting ────────────────────────────────────────────────────────────

  /**
   * Returns the first hit object (instance or interface ring) under the raycaster.
   */
  raycast(raycaster: THREE.Raycaster): HitResult | null {
    const targets: THREE.Object3D[] = []
    this.group.traverse(o => { if (o instanceof THREE.Mesh) targets.push(o) })
    const hits = raycaster.intersectObjects(targets, false)

    // Rings get priority
    for (const hit of hits) {
      if (hit.object.userData.isRing) {
        return {
          type: 'interface',
          instanceId:  hit.object.userData.instanceId,
          interfaceId: hit.object.userData.interfaceId,
        }
      }
    }

    for (const hit of hits) {
      if (hit.object.userData.instanceId) {
        return { type: 'instance', instanceId: hit.object.userData.instanceId }
      }
    }

    return null
  }

  // ── Ghost preview ─────────────────────────────────────────────────────────

  private ghostGroup: THREE.Group | null = null

  showGhostAt(defId: string, params: ParamValues, parentInstanceId: string, parentIfaceId: string, childIfaceId: string) {
    this.clearGhost()
    if (!this.graph) return

    const def    = getPartDef(defId)
    const pInst  = this.graph.getInstance(parentInstanceId)
    const pDef   = pInst ? getPartDef(pInst.definitionId) : null
    if (!def || !pInst || !pDef) return

    const ghost = def.buildMesh(params)
    ghost.traverse(o => {
      if (o instanceof THREE.Mesh) {
        const mat = (o.material as THREE.MeshStandardMaterial).clone()
        mat.transparent = true
        mat.opacity = 0.45
        mat.depthWrite = false
        o.material = mat
      }
    })

    const pPartGroup = this.meshMap.get(parentInstanceId)
    if (!pPartGroup) return

    const pWorldMat = new THREE.Matrix4().compose(pPartGroup.position, pPartGroup.quaternion, pPartGroup.scale)
    const localMat  = computeConnectionTransform(pDef, pInst.params, parentIfaceId, def, params, childIfaceId)
    const worldMat  = pWorldMat.clone().multiply(localMat)

    ghost.position.setFromMatrixPosition(worldMat)
    ghost.quaternion.setFromRotationMatrix(worldMat)

    this.group.add(ghost)
    this.ghostGroup = ghost
  }

  clearGhost() {
    if (this.ghostGroup) {
      this.group.remove(this.ghostGroup)
      this.ghostGroup = null
    }
  }

  // ── Scene queries ─────────────────────────────────────────────────────────

  getMeshGroup(instanceId: string): THREE.Group | undefined {
    return this.meshMap.get(instanceId)
  }

  /**
   * Return the world position a part WOULD sit at from its connection transform alone,
   * ignoring any dragOffset. Used when computing the cumulative offset to commit.
   */
  getComputedWorldPosition(instanceId: string): THREE.Vector3 | null {
    if (!this.graph) return null
    const inst = this.graph.getInstance(instanceId)
    if (!inst) return null
    const parentConn = this.graph.getParentConnection(instanceId)
    const mat = this.computeWorldMatrix(inst, parentConn)
    return new THREE.Vector3().setFromMatrixPosition(mat)
  }

  /**
   * Return the full 4×4 world matrix from connection transform alone
   * (no dragOffset/dragRotation applied). Used to compute gizmo deltas.
   */
  getComputedWorldMatrix(instanceId: string): THREE.Matrix4 | null {
    if (!this.graph) return null
    const inst = this.graph.getInstance(instanceId)
    if (!inst) return null
    const parentConn = this.graph.getParentConnection(instanceId)
    return this.computeWorldMatrix(inst, parentConn)
  }

  getInterfaceWorldPosition(instanceId: string, interfaceId: string): THREE.Vector3 | null {
    if (!this.graph) return null
    const inst = this.graph.getInstance(instanceId)
    if (!inst) return null
    const def = getPartDef(inst.definitionId)
    if (!def) return null
    const iface = def.interfaces.find(i => i.id === interfaceId)
    if (!iface) return null

    const grp = this.meshMap.get(instanceId)
    if (!grp) return null

    const localPos = iface.localPosition(inst.params)
    return localPos.clone().applyMatrix4(
      new THREE.Matrix4().compose(grp.position, grp.quaternion, grp.scale)
    )
  }

  // ── X-Ray isolation ───────────────────────────────────────────────────────

  /** Saved state before x-ray was applied. Key = mesh uuid. */
  private xrayState = new Map<string, { transparent: boolean; opacity: number; depthWrite: boolean }>()

  /**
   * Fade every part except selectedId to near-transparent so the selected part
   * stands out clearly. Call clearXRay() to restore.
   */
  applyXRay(selectedId: string) {
    this.clearXRay()   // idempotent — reset before re-applying
    this.group.traverse(o => {
      if (!(o instanceof THREE.Mesh) || o.userData.isRing) return
      const isSel = o.userData.instanceId === selectedId
      if (isSel) return  // leave selected part untouched

      const mat = o.material as THREE.MeshStandardMaterial
      this.xrayState.set(o.uuid, {
        transparent: mat.transparent,
        opacity:     mat.opacity,
        depthWrite:  mat.depthWrite,
      })
      mat.transparent = true
      mat.opacity     = 0.40
      mat.depthWrite  = false
    })
  }

  /** Restore all part materials to their state before applyXRay. */
  clearXRay() {
    if (this.xrayState.size === 0) return
    this.group.traverse(o => {
      if (!(o instanceof THREE.Mesh) || o.userData.isRing) return
      const saved = this.xrayState.get(o.uuid)
      if (!saved) return
      const mat = o.material as THREE.MeshStandardMaterial
      mat.transparent = saved.transparent
      mat.opacity     = saved.opacity
      mat.depthWrite  = saved.depthWrite
    })
    this.xrayState.clear()
  }

  // ── Visibility ────────────────────────────────────────────────────────────

  private hiddenInstances = new Set<string>()

  setInstanceVisible(instanceId: string, visible: boolean) {
    const grp = this.meshMap.get(instanceId)
    if (!grp) return
    grp.visible = visible
    if (visible) this.hiddenInstances.delete(instanceId)
    else         this.hiddenInstances.add(instanceId)
  }

  isInstanceVisible(instanceId: string): boolean {
    return !this.hiddenInstances.has(instanceId)
  }

  // ── Bounding box ──────────────────────────────────────────────────────────

  /** World-space axis-aligned bounding box of the part's mesh group. */
  getBoundingBox(instanceId: string): THREE.Box3 | null {
    const grp = this.meshMap.get(instanceId)
    if (!grp) return null
    const box = new THREE.Box3()
    grp.traverse(o => {
      if (o instanceof THREE.Mesh && !o.userData.isRing) {
        const meshBox = new THREE.Box3().setFromObject(o)
        box.union(meshBox)
      }
    })
    return box.isEmpty() ? null : box
  }

  // ── Cleanup ───────────────────────────────────────────────────────────────

  clear() {
    while (this.group.children.length) this.group.remove(this.group.children[0])
    this.meshMap.clear()
    this.ringMap.clear()
    this.xrayState.clear()
    this.hiddenInstances.clear()
    this.ghostGroup = null
    this.selectedInstanceId = null
  }

  dispose() {
    this.unbind()
    this.clear()
    this.group.parent?.remove(this.group)
  }
}

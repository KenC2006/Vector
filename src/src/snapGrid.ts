import * as THREE from 'three'

// ── Snap sizes (metres) ───────────────────────────────────────────────────────

export const SNAP_SIZES = [0.05, 0.1, 0.25, 0.5] as const
export type SnapSize = typeof SNAP_SIZES[number]

// ── SnapGrid ──────────────────────────────────────────────────────────────────

export class SnapGrid {
  private readonly scene: THREE.Scene

  // Dot-cloud covering the ground plane
  private dotsPoints: THREE.Points | null = null

  // Single accent dot at the current drag target
  private highlightDot: THREE.Points | null = null

  snapSize: SnapSize  = 0.1
  snapEnabled         = true

  private _visible    = false
  private _lastCenter = new THREE.Vector3()

  // Cover a fixed world-space radius; fewer points at coarser resolutions
  private get _radius(): number {
    return Math.min(40, Math.ceil(2.8 / this.snapSize))
  }

  // ── Shared materials (created once) ──────────────────────────────────────

  private readonly _dotsMat = new THREE.PointsMaterial({
    color: 0x3a3a52,
    size: 0.006,
    sizeAttenuation: true,
    depthWrite: false,
    transparent: true,
    opacity: 0.9,
  })

  private readonly _hlMat = new THREE.PointsMaterial({
    color: 0x4a9eff,
    size: 0.020,
    sizeAttenuation: true,
    depthWrite: false,
    transparent: true,
    opacity: 1,
  })

  constructor(scene: THREE.Scene) {
    this.scene = scene
  }

  // ── Visibility ─────────────────────────────────────────────────────────────

  get visible() { return this._visible }

  setVisible(v: boolean) {
    this._visible = v
    if (this.dotsPoints) this.dotsPoints.visible = v
    if (!v) {
      this.clearHighlight()
    } else {
      this._rebuild()
    }
  }

  // ── Snap size ──────────────────────────────────────────────────────────────

  setSnapSize(size: SnapSize) {
    this.snapSize = size
    if (this._visible) this._rebuild()
  }

  // ── Keep grid centred on camera target ────────────────────────────────────

  updateAround(center: THREE.Vector3) {
    const sz = this.snapSize
    // Only rebuild if camera moved more than half a grid cell
    const moved =
      Math.abs(center.x - this._lastCenter.x) > sz * 0.5 ||
      Math.abs(center.z - this._lastCenter.z) > sz * 0.5

    if (moved && this._visible) this._rebuild(center)
    else this._lastCenter.copy(center)
  }

  // ── Snap function (public) ─────────────────────────────────────────────────

  snapToGrid(worldPos: THREE.Vector3): THREE.Vector3 {
    if (!this.snapEnabled) return worldPos.clone()
    const sz = this.snapSize
    return new THREE.Vector3(
      Math.round(worldPos.x / sz) * sz,
      worldPos.y,
      Math.round(worldPos.z / sz) * sz,
    )
  }

  // ── Drag highlight ─────────────────────────────────────────────────────────

  /** Show accent dot at the snapped target position. Returns the snapped position. */
  showHighlightAt(worldPos: THREE.Vector3): THREE.Vector3 {
    const snapped = this.snapToGrid(worldPos)
    const pos = new Float32Array([snapped.x, 0.004, snapped.z])
    const geo = new THREE.BufferGeometry()
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3))

    if (this.highlightDot) {
      this.scene.remove(this.highlightDot)
      this.highlightDot.geometry.dispose()
    }
    this.highlightDot = new THREE.Points(geo, this._hlMat)
    this.highlightDot.renderOrder = 2
    this.scene.add(this.highlightDot)
    return snapped
  }

  clearHighlight() {
    if (this.highlightDot) {
      this.scene.remove(this.highlightDot)
      this.highlightDot.geometry.dispose()
      this.highlightDot = null
    }
  }

  // ── Dispose ───────────────────────────────────────────────────────────────

  dispose() {
    this.setVisible(false)
    this._dotsMat.dispose()
    this._hlMat.dispose()
  }

  // ── Internal rebuild ──────────────────────────────────────────────────────

  private _rebuild(center?: THREE.Vector3) {
    if (center) this._lastCenter.copy(center)

    const sz = this.snapSize
    const r  = this._radius

    // Snap the origin so dots don't slide with the camera
    const cx = Math.round(this._lastCenter.x / sz) * sz
    const cz = Math.round(this._lastCenter.z / sz) * sz

    const side   = 2 * r + 1
    const count  = side * side
    const pos    = new Float32Array(count * 3)
    let   idx    = 0

    for (let i = -r; i <= r; i++) {
      for (let j = -r; j <= r; j++) {
        pos[idx++] = cx + i * sz
        pos[idx++] = 0.002            // just above the ground mesh
        pos[idx++] = cz + j * sz
      }
    }

    const geo = new THREE.BufferGeometry()
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3))

    if (this.dotsPoints) {
      this.scene.remove(this.dotsPoints)
      this.dotsPoints.geometry.dispose()
    }

    this.dotsPoints = new THREE.Points(geo, this._dotsMat)
    this.dotsPoints.renderOrder = -1
    this.dotsPoints.visible = this._visible
    this.scene.add(this.dotsPoints)
  }
}

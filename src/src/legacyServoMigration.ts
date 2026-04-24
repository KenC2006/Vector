// docs/SERVO_SPLIT_PLAN.md §legacy-migration — rewrite pre-split URDFs into
// the body+output form at load time so dog2.urdf and friends keep rendering
// without hand edits. Idempotent: if a `_body` sub-link already exists, skip.
//
// Heuristics:
// - A `<link>` is a migratable servo if its name matches a split-link preset.
// - The servo's INCOMING joint (parent → servo) becomes a `_mount` fixed
//   joint into `_body` (the DOF is now internal, not at the chassis boundary).
// - Each OUTGOING joint is routed by the sign of its origin Z:
//     origin z > 0   → child previously mated to `top` → re-parent to `_output`
//     origin z ≤ 0   → child previously mated to body/side → re-parent to `_body`
//   (URDF doesn't persist attach_face, so sign-of-Z is the cheapest proxy.)

import { findConnector as _findConnector } from './mateConnectors.ts'  // unused but keeps tree-shake parity with legacy imports
void _findConnector

export interface LegacyServoMigrationPreset {
  id: string
  physical?: { mass_kg?: number }
  mechanical_electrical?: Record<string, unknown>
  split_link?: {
    body_mass_frac: number
    output_origin_xyz_mm: [number, number, number]
    output_axis_xyz: [number, number, number]
    output_half_extents_mm: [number, number, number]
    bracket_tilt_threshold_deg: number
  }
}

export interface LegacyServoMigrationContext {
  findPreset(componentId: string): LegacyServoMigrationPreset | null
}

export interface LegacyServoMigrationResult {
  migrated: number
  skipped: number
  notes: string[]
}

const SERVO_LINK_RE = /^(actuator_servo_\w+|actuator_continuous_\w+)_(\d+)$/

function componentIdFromLinkName(linkName: string): string | null {
  const m = linkName.match(SERVO_LINK_RE)
  return m ? m[1] : null
}

function directChildByTag(parent: Element, tag: string): Element | null {
  for (let i = 0; i < parent.childNodes.length; i++) {
    const n = parent.childNodes[i]
    if (n.nodeType === 1 /* ELEMENT_NODE */ && (n as Element).tagName === tag) {
      return n as Element
    }
  }
  return null
}

function directChildrenByTag(parent: Element, tag: string): Element[] {
  const out: Element[] = []
  for (let i = 0; i < parent.childNodes.length; i++) {
    const n = parent.childNodes[i]
    if (n.nodeType === 1 && (n as Element).tagName === tag) out.push(n as Element)
  }
  return out
}

function getOriginXyz(jointEl: Element): [number, number, number] {
  const origin = directChildByTag(jointEl, 'origin')
  const xyz = (origin?.getAttribute('xyz') ?? '0 0 0').trim().split(/\s+/).map(Number)
  return [xyz[0] || 0, xyz[1] || 0, xyz[2] || 0]
}

function cloneElementRemovingChildren(doc: Document, tag: string): Element {
  return doc.createElement(tag)
}

/** Rewrite every legacy single-link servo in `doc` into body + output sub-
 *  links joined by an internal revolute. Safe to call repeatedly — links
 *  already ending in `_body` / `_output` (or whose `_body` counterpart
 *  exists) are skipped. */
export function migrateLegacyServos(
  doc: Document,
  ctx: LegacyServoMigrationContext,
): LegacyServoMigrationResult {
  const notes: string[] = []
  let migrated = 0
  let skipped = 0

  const robot = doc.getElementsByTagName('robot')[0]
  if (!robot) return { migrated: 0, skipped: 0, notes: ['no <robot> root'] }

  const allLinks = Array.from(doc.getElementsByTagName('link')) as Element[]
  const existingNames = new Set(allLinks.map(l => l.getAttribute('name') ?? ''))

  for (const linkEl of allLinks) {
    const name = linkEl.getAttribute('name') ?? ''
    if (!name) continue
    if (name.endsWith('_body') || name.endsWith('_output')) {
      skipped++
      continue
    }
    if (existingNames.has(`${name}_body`)) {
      skipped++
      continue
    }
    const compId = componentIdFromLinkName(name)
    if (!compId) continue
    const preset = ctx.findPreset(compId)
    if (!preset?.split_link) {
      skipped++
      notes.push(`${name}: preset ${compId} has no split_link — leaving as single link`)
      continue
    }

    const split = preset.split_link
    const totalMass = preset.physical?.mass_kg ?? 0.1
    const bodyMass = totalMass * split.body_mass_frac
    const outputMass = totalMass * Math.max(0, 1 - split.body_mass_frac)

    // Rename the original link to `${name}_body` and rescale mass + inertia.
    const bodyName = `${name}_body`
    const outputName = `${name}_output`
    const mountJointName = `${name}_mount`
    const internalJointName = `${name}_joint`

    linkEl.setAttribute('name', bodyName)
    const inertialEl = directChildByTag(linkEl, 'inertial')
    if (inertialEl) {
      const massEl = directChildByTag(inertialEl, 'mass')
      if (massEl) massEl.setAttribute('value', bodyMass.toFixed(4))
      // Inertia tensor scales linearly with mass (same geometry, mass × frac),
      // so carry the body_mass_frac through ixx/iyy/izz too. Without this the
      // body link ends up with ~11% inflated rotational inertia relative to
      // its own mass — which can surprise PD tuning downstream.
      const inertiaEl = directChildByTag(inertialEl, 'inertia')
      if (inertiaEl) {
        for (const attr of ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']) {
          const v = Number(inertiaEl.getAttribute(attr))
          if (Number.isFinite(v)) inertiaEl.setAttribute(attr, (v * split.body_mass_frac).toFixed(6))
        }
      }
    }

    // Build the new output link. Inertia: cylinder approximation of the horn.
    // `output_half_extents_mm` convention is [radius, radius, full_height]
    // (name is historical; values come from hornR = min(w,d)*0.3 and hornH =
    // h*0.07 in actuators.ts).
    const hornR = split.output_half_extents_mm[0] / 1000
    const hornH = split.output_half_extents_mm[2] / 1000
    const outIxx = outputMass / 12 * (3 * hornR * hornR + hornH * hornH)
    const outIzz = outputMass / 2 * hornR * hornR

    const outputLink = doc.createElement('link')
    outputLink.setAttribute('name', outputName)
    const oInertial = doc.createElement('inertial')
    const oMass = doc.createElement('mass'); oMass.setAttribute('value', outputMass.toFixed(4))
    const oInertia = doc.createElement('inertia')
    oInertia.setAttribute('ixx', outIxx.toFixed(6))
    oInertia.setAttribute('iyy', outIxx.toFixed(6))
    oInertia.setAttribute('izz', outIzz.toFixed(6))
    oInertia.setAttribute('ixy', '0'); oInertia.setAttribute('ixz', '0'); oInertia.setAttribute('iyz', '0')
    oInertial.appendChild(oMass); oInertial.appendChild(oInertia)
    outputLink.appendChild(oInertial)

    // Output visual: small cylinder centered at link origin (horn disc).
    const oVisual = doc.createElement('visual')
    const oOrigin = cloneElementRemovingChildren(doc, 'origin')
    oOrigin.setAttribute('xyz', '0 0 0'); oOrigin.setAttribute('rpy', '0 0 0')
    const oGeom = doc.createElement('geometry')
    const oCyl = doc.createElement('cylinder')
    oCyl.setAttribute('radius', hornR.toFixed(6)); oCyl.setAttribute('length', hornH.toFixed(6))
    oGeom.appendChild(oCyl)
    oVisual.appendChild(oOrigin); oVisual.appendChild(oGeom)
    outputLink.appendChild(oVisual)

    const oCollision = doc.createElement('collision')
    const oColOrigin = cloneElementRemovingChildren(doc, 'origin')
    oColOrigin.setAttribute('xyz', '0 0 0'); oColOrigin.setAttribute('rpy', '0 0 0')
    const oColGeom = doc.createElement('geometry')
    const oColCyl = doc.createElement('cylinder')
    oColCyl.setAttribute('radius', hornR.toFixed(6)); oColCyl.setAttribute('length', hornH.toFixed(6))
    oColGeom.appendChild(oColCyl)
    oCollision.appendChild(oColOrigin); oCollision.appendChild(oColGeom)
    outputLink.appendChild(oCollision)
    robot.appendChild(outputLink)

    // Rewrite joints.
    const jointEls = Array.from(doc.getElementsByTagName('joint')) as Element[]
    let savedEffort = '10'
    let savedVelocity = '3.14'

    for (const j of jointEls) {
      const parentAttr = directChildByTag(j, 'parent')?.getAttribute('link') ?? ''
      const childAttr = directChildByTag(j, 'child')?.getAttribute('link') ?? ''

      if (childAttr === name) {
        // Incoming joint: parent → servo. Becomes fixed mount into body.
        // Save effort/velocity to carry onto the new internal revolute.
        const limit = directChildByTag(j, 'limit')
        if (limit) {
          savedEffort = limit.getAttribute('effort') ?? savedEffort
          savedVelocity = limit.getAttribute('velocity') ?? savedVelocity
        }
        j.setAttribute('name', mountJointName)
        j.setAttribute('type', 'fixed')
        const childEl = directChildByTag(j, 'child')
        childEl?.setAttribute('link', bodyName)
        // Drop axis + limit (fixed joints don't need them).
        for (const ax of directChildrenByTag(j, 'axis')) j.removeChild(ax)
        for (const lim of directChildrenByTag(j, 'limit')) j.removeChild(lim)
      } else if (parentAttr === name) {
        // Outgoing joint: servo → external child. Route to body or output
        // based on joint origin Z sign. All external mates become fixed —
        // the DOF is now internal.
        const [, , oz] = getOriginXyz(j)
        const parentEl = directChildByTag(j, 'parent')
        if (oz > 0) {
          parentEl?.setAttribute('link', outputName)
        } else {
          parentEl?.setAttribute('link', bodyName)
        }
        j.setAttribute('type', 'fixed')
        for (const ax of directChildrenByTag(j, 'axis')) j.removeChild(ax)
        for (const lim of directChildrenByTag(j, 'limit')) j.removeChild(lim)
      }
    }

    // Insert the internal revolute.
    const internalJoint = doc.createElement('joint')
    internalJoint.setAttribute('name', internalJointName)
    internalJoint.setAttribute('type', 'revolute')
    const iParent = doc.createElement('parent'); iParent.setAttribute('link', bodyName)
    const iChild = doc.createElement('child'); iChild.setAttribute('link', outputName)
    const iOrigin = cloneElementRemovingChildren(doc, 'origin')
    const originXyz = split.output_origin_xyz_mm.map(v => (v / 1000).toFixed(6)).join(' ')
    iOrigin.setAttribute('xyz', originXyz); iOrigin.setAttribute('rpy', '0 0 0')
    const iAxis = doc.createElement('axis')
    iAxis.setAttribute('xyz', split.output_axis_xyz.join(' '))
    const iLimit = doc.createElement('limit')
    iLimit.setAttribute('lower', '-3.14159'); iLimit.setAttribute('upper', '3.14159')
    iLimit.setAttribute('effort', savedEffort); iLimit.setAttribute('velocity', savedVelocity)
    internalJoint.appendChild(iParent); internalJoint.appendChild(iChild); internalJoint.appendChild(iOrigin)
    internalJoint.appendChild(iAxis); internalJoint.appendChild(iLimit)
    robot.appendChild(internalJoint)

    migrated++
    existingNames.add(bodyName)
    existingNames.add(outputName)
  }

  return { migrated, skipped, notes }
}

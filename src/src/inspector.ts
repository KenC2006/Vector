import type { LinkDetail } from './robotData'
import { getParentJoint, getChildJoints } from './robotData'
import type { AttachmentNode } from './nodeManager'

let bodyEl: HTMLElement | null = null

export function initInspector() {
  bodyEl = document.querySelector('#panel-inspector .insp-body')
  hideInspector()
}

export function showInspector(linkName: string, detail: LinkDetail, nodes: AttachmentNode[]) {
  if (!bodyEl) return

  const parentJoint = getParentJoint(linkName)
  const childJoints = getChildJoints(linkName)
  const g = detail.geometry

  const geoDesc =
    g.type === 'box'
      ? `${g.params.width?.toFixed(3)} × ${g.params.height?.toFixed(3)} × ${g.params.depth?.toFixed(3)} m`
      : g.type === 'cylinder'
      ? `r=${g.params.radius?.toFixed(4)} m, l=${g.params.length?.toFixed(4)} m`
      : `r=${g.params.radius?.toFixed(4)} m`

  const ins = detail.inertia

  const parentJointHtml = parentJoint
    ? `
    <div class="insp-section-title">Parent Joint</div>
    <div class="insp-kv">
      <span class="insp-kv-name">${parentJoint.name}</span>
      <span class="insp-badge insp-badge-${parentJoint.type}">${parentJoint.type}</span>
    </div>
    ${parentJoint.limits ? `
    <div class="insp-row"><span class="insp-key">Axis</span><span class="insp-val">${parentJoint.axis}</span></div>
    <div class="insp-row"><span class="insp-key">Range</span><span class="insp-val">${parentJoint.limits.lower.toFixed(2)} → ${parentJoint.limits.upper.toFixed(2)} ${parentJoint.type === 'prismatic' ? 'm' : 'rad'}</span></div>
    <div class="insp-row"><span class="insp-key">Effort</span><span class="insp-val">${parentJoint.limits.effort} ${parentJoint.type === 'prismatic' ? 'N' : 'Nm'}</span></div>
    <div class="insp-row"><span class="insp-key">Velocity</span><span class="insp-val">${parentJoint.limits.velocity} ${parentJoint.type === 'prismatic' ? 'm/s' : 'rad/s'}</span></div>
    ` : `<div class="insp-row insp-dimmed">Fixed joint — no limits</div>`}
    ${parentJoint.dynamics ? `
    <div class="insp-row"><span class="insp-key">Damping</span><span class="insp-val">${parentJoint.dynamics.damping}</span></div>
    <div class="insp-row"><span class="insp-key">Friction</span><span class="insp-val">${parentJoint.dynamics.friction}</span></div>
    ` : ''}
    `
    : `<div class="insp-row insp-dimmed" style="padding-top:6px">Root link — no parent joint</div>`

  const childJointsHtml = childJoints.length > 0 ? `
    <div class="insp-section-title">Child Joints</div>
    ${childJoints.map(j => `
    <div class="insp-kv">
      <span class="insp-kv-name">${j.name}</span>
      <span class="insp-badge insp-badge-${j.type}">${j.type}</span>
    </div>
    <div class="insp-row insp-dimmed" style="padding-bottom:2px">→ ${j.childLink}</div>
    `).join('')}
  ` : ''

  const nodesHtml = `
    <div class="insp-section-title">Attachment Nodes <span class="insp-count">${nodes.length}</span></div>
    <div class="insp-nodes" id="insp-nodes-list">
      ${nodes.map(n => `
      <div class="insp-node-row${n.isOccupied ? ' occupied' : ''}" data-node-id="${n.id}">
        <span class="node-dot ${n.isOccupied ? 'occupied' : 'empty'}"></span>
        <span class="insp-node-label">${n.label}</span>
        <span class="insp-node-status">${n.isOccupied ? (n.attachedPartName ?? 'occupied') : 'empty'}</span>
        ${n.isOccupied ? `<button class="insp-node-detach" data-node-id="${n.id}" title="Remove part">✕</button>` : ''}
      </div>
      `).join('')}
    </div>
  `

  bodyEl.innerHTML = `
    <div class="insp-link-header">
      <span class="insp-link-name">${linkName}</span>
      <span class="insp-geo-badge">${g.type}</span>
    </div>
    <div class="insp-section-title">Link Properties</div>
    <div class="insp-row"><span class="insp-key">Mass</span><span class="insp-val">${detail.mass} kg</span></div>
    <div class="insp-row"><span class="insp-key">Geometry</span><span class="insp-val geo-val">${geoDesc}</span></div>
    <div class="insp-section-title">Inertia (kg·m²)</div>
    <div class="insp-inertia-grid">
      <span class="insp-key">Ixx</span><span class="insp-val">${ins.ixx.toFixed(5)}</span>
      <span class="insp-key">Iyy</span><span class="insp-val">${ins.iyy.toFixed(5)}</span>
      <span class="insp-key">Izz</span><span class="insp-val">${ins.izz.toFixed(5)}</span>
      <span class="insp-key">Ixy</span><span class="insp-val">${ins.ixy.toFixed(5)}</span>
      <span class="insp-key">Ixz</span><span class="insp-val">${ins.ixz.toFixed(5)}</span>
      <span class="insp-key">Iyz</span><span class="insp-val">${ins.iyz.toFixed(5)}</span>
    </div>
    ${parentJointHtml}
    ${childJointsHtml}
    ${nodesHtml}
  `
}

export function hideInspector() {
  if (!bodyEl) return
  bodyEl.innerHTML = '<div class="insp-empty">Click a robot part to inspect it</div>'
}

export function updateNodeRow(nodeId: string, isOccupied: boolean, partName?: string) {
  const row = document.querySelector(`.insp-node-row[data-node-id="${nodeId}"]`) as HTMLElement | null
  if (!row) return

  const dot = row.querySelector('.node-dot')
  const status = row.querySelector('.insp-node-status')

  if (isOccupied) {
    row.classList.add('occupied')
    dot?.classList.replace('empty', 'occupied')
    if (status) status.textContent = partName ?? 'occupied'
    if (!row.querySelector('.insp-node-detach')) {
      const btn = document.createElement('button')
      btn.className = 'insp-node-detach'
      btn.dataset.nodeId = nodeId
      btn.title = 'Remove part'
      btn.textContent = '✕'
      row.appendChild(btn)
    }
  } else {
    row.classList.remove('occupied')
    dot?.classList.replace('occupied', 'empty')
    if (status) status.textContent = 'empty'
    row.querySelector('.insp-node-detach')?.remove()
  }
}

export function highlightInspectorNodeRow(nodeId: string | null) {
  document.querySelectorAll('.insp-node-row').forEach(row => {
    row.classList.toggle('selected', (row as HTMLElement).dataset.nodeId === nodeId)
  })
}

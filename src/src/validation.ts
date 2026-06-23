import * as monaco from 'monaco-editor'
import type { KinematicLink, KinematicJoint } from './urdfParser'
import type { StructuredDiagnostic } from './topologyValidation'

export interface ValResult {
  name: string
  severity: string  // "pass" | "warn" | "error" | "info"
  message: string
  category: string
  line?: number
  column?: number
  /** Concrete suggested edit (from the assembly-soundness validator). Rendered
   *  as an amber "Fix:" line and folded into the Monaco marker text. */
  repair?: string
}

/** Top-level XML failures that make any deeper analysis meaningless — when one
 *  of these is present we show it alone and skip the per-link / backend passes. */
const PARSE_BLOCKERS = new Set(['XML Parse Error', 'Invalid root element', 'No links defined'])

/** Categories the Python backend owns when it's reachable. It computes richer
 *  versions of these (inertia triangle-inequality, effort limits, approximate
 *  overlaps), so its results replace the client-side equivalents. Structural
 *  and Assembly always stay client-side: they're line-accurate and need no
 *  round-trip. */
const BACKEND_CATEGORIES = new Set(['Physics', 'Actuators', 'Mesh', 'Spatial'])

/** 1-based line of the first `name="<name>"` occurrence in `content`, or
 *  undefined. Anchors a finding to its URDF source so Monaco markers and
 *  click-to-jump land on the right element. Falls back to the split-servo body
 *  link, whose logical name (without the `_body` suffix) is what the assembly
 *  graph reports. */
function lineOfName(content: string, name: string | undefined): number | undefined {
  if (!name) return undefined
  let idx = content.indexOf(`name="${name}"`)
  if (idx < 0) idx = content.indexOf(`name="${name}_body"`)
  if (idx < 0) return undefined
  return content.slice(0, idx).split('\n').length
}

/** Parse a URDF string and return structural validation errors. Exported for use outside initValidation. */
export function validateXMLStructure(content: string): ValResult[] {
  const errors: ValResult[] = []
  const parser = new DOMParser()
  const xmlDoc = parser.parseFromString(content, 'text/xml')

  if (xmlDoc.getElementsByTagName('parsererror').length > 0) {
    const parserError = xmlDoc.getElementsByTagName('parsererror')[0]
    errors.push({ name: 'XML Parse Error', severity: 'error', message: parserError.textContent || 'Unknown XML parse error', category: 'Structural' })
    return errors
  }
  if (xmlDoc.documentElement.tagName !== 'robot') {
    errors.push({ name: 'Invalid root element', severity: 'error', message: `Expected root element <robot>, got <${xmlDoc.documentElement.tagName}>`, category: 'Structural' })
    return errors
  }
  const robotName = xmlDoc.documentElement.getAttribute('name')
  if (!robotName) errors.push({ name: 'Robot missing name', severity: 'error', message: 'Root <robot> element must have a "name" attribute', category: 'Structural' })

  const links = xmlDoc.getElementsByTagName('link')
  if (links.length === 0) {
    errors.push({ name: 'No links defined', severity: 'error', message: 'URDF must contain at least one <link> element', category: 'Structural' })
    return errors
  }
  const linkNames = new Set<string>()
  for (let i = 0; i < links.length; i++) {
    const name = links[i].getAttribute('name')
    if (name) {
      if (linkNames.has(name)) errors.push({ name: 'Duplicate link name', severity: 'error', message: `Link "${name}" is defined multiple times`, category: 'Structural', line: lineOfName(content, name) })
      linkNames.add(name)
    }
  }
  const joints = xmlDoc.getElementsByTagName('joint')
  const jointNames = new Set<string>()
  const childLinkNames = new Set<string>()
  for (let i = 0; i < joints.length; i++) {
    const joint = joints[i]
    const jointName = joint.getAttribute('name')
    if (jointName) {
      if (jointNames.has(jointName)) errors.push({ name: 'Duplicate joint name', severity: 'error', message: `Joint "${jointName}" is defined multiple times`, category: 'Structural', line: lineOfName(content, jointName) })
      jointNames.add(jointName)
    }
    const parent = joint.querySelector('parent')
    const child = joint.querySelector('child')
    if (!parent || !child) { errors.push({ name: `Joint ${jointName || 'unknown'} missing parent/child`, severity: 'error', message: 'Joint must have both <parent> and <child> elements', category: 'Structural', line: lineOfName(content, jointName ?? undefined) }); continue }
    const parentLink = parent.getAttribute('link')
    const childLink = child.getAttribute('link')
    if (!parentLink || !linkNames.has(parentLink)) errors.push({ name: `Invalid parent link in joint ${jointName || 'unknown'}`, severity: 'error', message: `Parent link "${parentLink}" is not defined`, category: 'Structural', line: lineOfName(content, jointName ?? undefined) })
    if (!childLink || !linkNames.has(childLink)) errors.push({ name: `Invalid child link in joint ${jointName || 'unknown'}`, severity: 'error', message: `Child link "${childLink}" is not defined`, category: 'Structural', line: lineOfName(content, jointName ?? undefined) })
    if (childLink) childLinkNames.add(childLink)
  }

  const rootLinks = Array.from(linkNames)
    .filter(name => !name.includes('__mount__'))
    .filter(name => !childLinkNames.has(name))
  if (rootLinks.length !== 1) {
    errors.push({
      name: rootLinks.length === 0 ? 'No root link' : 'Multiple root links',
      severity: 'error',
      message: rootLinks.length === 0
        ? 'URDF must have exactly one root link for simulation'
        : `Simulation supports one connected robot tree; found ${rootLinks.length} root links: ${rootLinks.slice(0, 8).join(', ')}${rootLinks.length > 8 ? ', ...' : ''}`,
      category: 'Structural',
      line: rootLinks.length > 1 ? lineOfName(content, rootLinks[0]) : undefined,
    })
  }
  if (errors.length === 0) errors.push({ name: 'XML structure valid', severity: 'pass', message: `${links.length} links, ${joints.length} joints`, category: 'Structural' })
  return errors
}

/**
 * Per-link completeness checks: collision geometry, inertial mass, joint limits,
 * plus a total-mass and joint-count summary. Skips mount-node links
 * (containing __mount__). Returns one result per category (not per link) to keep
 * the list short; the first offending link's line anchors each finding.
 */
export function validateURDFPerLink(content: string): ValResult[] {
  const results: ValResult[] = []
  const parser = new DOMParser()
  const xmlDoc = parser.parseFromString(content, 'text/xml')
  if (xmlDoc.getElementsByTagName('parsererror').length > 0) return results

  const links = Array.from(xmlDoc.getElementsByTagName('link'))
    .filter(l => !l.getAttribute('name')?.includes('__mount__'))

  // Find root link (not referenced as a child in any joint)
  const childLinks = new Set(
    Array.from(xmlDoc.getElementsByTagName('joint'))
      .map(j => j.querySelector('child')?.getAttribute('link') ?? '')
      .filter(Boolean)
  )
  const isRoot = (name: string) => !childLinks.has(name)

  // ── Collision geometry check ──
  const noCollision: string[] = []
  for (const link of links) {
    const name = link.getAttribute('name') ?? ''
    if (isRoot(name)) continue
    if (!link.querySelector('collision')) noCollision.push(name)
  }
  if (noCollision.length === 0) {
    results.push({ name: 'Collision geometry', severity: 'pass', message: 'All non-root links have collision geometry', category: 'Mesh' })
  } else {
    results.push({ name: 'Missing collision geometry', severity: 'warn', message: `${noCollision.length} link(s) lack <collision>: ${noCollision.join(', ')}`, category: 'Mesh', line: lineOfName(content, noCollision[0]) })
  }

  // ── Inertial / mass check ──
  const noInertial: string[] = []
  const zeroMass: string[] = []
  let totalMass = 0
  for (const link of links) {
    const name = link.getAttribute('name') ?? ''
    const inertial = link.querySelector('inertial')
    const massEl = inertial?.querySelector('mass')
    const mass = parseFloat(massEl?.getAttribute('value') ?? '0')
    if (Number.isFinite(mass) && mass > 0) totalMass += mass
    if (isRoot(name)) continue
    if (!inertial) { noInertial.push(name); continue }
    if (!Number.isFinite(mass) || mass <= 0) zeroMass.push(name)
  }

  if (noInertial.length > 0) {
    results.push({ name: 'Missing inertial', severity: 'warn', message: `${noInertial.length} link(s) lack <inertial>: ${noInertial.join(', ')}`, category: 'Physics', line: lineOfName(content, noInertial[0]) })
  }
  if (zeroMass.length > 0) {
    results.push({ name: 'Zero/missing mass', severity: 'warn', message: `Zero or missing mass on: ${zeroMass.join(', ')}`, category: 'Physics', line: lineOfName(content, zeroMass[0]) })
  }
  if (noInertial.length === 0 && zeroMass.length === 0) {
    results.push({ name: 'Inertial properties', severity: 'pass', message: 'All non-root links have mass > 0', category: 'Physics' })
  }
  results.push({ name: 'Total mass', severity: 'info', message: `Total robot mass: ${totalMass.toFixed(3)} kg`, category: 'Physics' })

  // ── Joint limits check ──
  const joints = Array.from(xmlDoc.getElementsByTagName('joint'))
  const actuated = joints.filter(j => ['revolute', 'prismatic'].includes(j.getAttribute('type') ?? ''))
  const missingLimits: string[] = []
  let firstBadLimitName: string | undefined
  for (const joint of actuated) {
    const limit = joint.querySelector('limit')
    const lower = parseFloat(limit?.getAttribute('lower') ?? 'NaN')
    const upper = parseFloat(limit?.getAttribute('upper') ?? 'NaN')
    if (!limit || !Number.isFinite(lower) || !Number.isFinite(upper) || lower >= upper) {
      const jn = joint.getAttribute('name') ?? 'unnamed'
      missingLimits.push(jn)
      if (!firstBadLimitName) firstBadLimitName = jn
    }
  }
  if (missingLimits.length > 0) {
    results.push({ name: 'Joint limits', severity: 'warn', message: `Actuated joints with invalid limits: ${missingLimits.join(', ')}`, category: 'Actuators', line: lineOfName(content, firstBadLimitName) })
  } else if (actuated.length > 0) {
    results.push({ name: 'Joint limits', severity: 'pass', message: 'All actuated joints have valid limits', category: 'Actuators' })
  }
  const fixedCount = joints.length - actuated.length
  results.push({ name: 'Joint summary', severity: 'info', message: `${actuated.length} actuated, ${fixedCount} fixed joints`, category: 'Actuators' })

  return results
}

export function initValidation(deps: {
  invoke: (cmd: string, args?: Record<string, unknown>) => Promise<unknown>
  monacoEditor: monaco.editor.IStandaloneCodeEditor
  getKinematicGraph: () => Record<string, KinematicLink>
  getKinematicJoints: () => Record<string, KinematicJoint>
  /** Assembly-soundness findings (codes + suggested repairs) for the current
   *  in-memory graph — the same diagnostics the AI self-correction loop acts on.
   *  Empty when nothing has been assembled (e.g. a hand-loaded demo). */
  getAssemblyDiagnostics: () => StructuredDiagnostic[]
  showToast: (msg: string, type?: 'success' | 'warning' | 'error' | 'info') => void
}): {
  runLocalValidation: () => void
  runValidation: () => Promise<void>
  setValidationMarkers: (results: ValResult[]) => void
} {
  const { invoke, monacoEditor, getKinematicGraph, getKinematicJoints, getAssemblyDiagnostics } = deps

  // ── DOM elements ──
  const validationResults = document.getElementById('validation-results') as HTMLDivElement
  const validationSummary = document.getElementById('validation-summary') as HTMLDivElement
  const btnRevalidate = document.getElementById('btn-revalidate') as HTMLButtonElement

  function getContent(): string {
    return monacoEditor.getModel()?.getValue() ?? ''
  }

  // ── Validation Markers for Monaco ──

  function setValidationMarkers(results: ValResult[]) {
    const model = monacoEditor.getModel()
    if (!model) return

    const markers: monaco.editor.IMarkerData[] = []

    for (const r of results) {
      if (r.severity === 'pass' || r.severity === 'info') continue

      // Map validation severity to Monaco marker severity
      const markerSeverity = r.severity === 'error'
        ? monaco.MarkerSeverity.Error
        : monaco.MarkerSeverity.Warning

      // Use provided line/column or default to line 1
      const lineNumber = r.line || 1
      const column = r.column || 1
      const repair = r.repair ? ` — Fix: ${r.repair}` : ''

      markers.push({
        severity: markerSeverity,
        message: `[${r.category}] ${r.name}: ${r.message}${repair}`,
        startLineNumber: lineNumber,
        startColumn: column,
        endLineNumber: lineNumber,
        endColumn: Math.max(column + 1, column + (r.name.length || 10)),
        source: 'Vector Validator',
      })
    }

    monaco.editor.setModelMarkers(model, 'vector-validator', markers)
  }

  // ── Render validation results ──

  function renderValidationResults(results: ValResult[], summary: { pass: number; warn: number; error: number; info: number }) {
    // Render summary bar
    validationSummary.innerHTML = `
      <div class="vs-item vs-pass"><span class="vs-count">${summary.pass}</span> pass</div>
      <div class="vs-item vs-warn"><span class="vs-count">${summary.warn}</span> warn</div>
      <div class="vs-item vs-error"><span class="vs-count">${summary.error}</span> error</div>
    `

    // Update status bar error/warning counts
    const errorCountEl = document.getElementById('error-count')
    const warningCountEl = document.getElementById('warning-count')
    if (errorCountEl) errorCountEl.textContent = String(summary.error)
    if (warningCountEl) warningCountEl.textContent = String(summary.warn)

    // Group results by category. Assembly first — it's the soundness signal the
    // tab now exists to surface — then the rest in a stable order.
    const groups: Record<string, ValResult[]> = {}
    for (const r of results) {
      if (!groups[r.category]) groups[r.category] = []
      groups[r.category].push(r)
    }
    const ORDER = ['Assembly', 'Structural', 'Physics', 'Actuators', 'Mesh', 'Spatial']
    const categories = Object.keys(groups).sort((a, b) => {
      const ia = ORDER.indexOf(a), ib = ORDER.indexOf(b)
      return (ia < 0 ? ORDER.length : ia) - (ib < 0 ? ORDER.length : ib)
    })

    // Render groups
    validationResults.innerHTML = ''
    if (results.length === 0) {
      validationResults.innerHTML = '<div class="val-placeholder">No URDF to validate</div>'
      return
    }
    for (const category of categories) {
      const items = groups[category]
      const group = document.createElement('div')
      group.className = 'val-group'

      const title = document.createElement('div')
      title.className = 'vg-title'
      title.textContent = category
      group.appendChild(title)

      for (const item of items) {
        const el = document.createElement('div')
        el.className = `val-item ${item.severity}`

        const contentEl = document.createElement('div')
        contentEl.style.display = 'flex'
        contentEl.style.justifyContent = 'space-between'
        contentEl.style.alignItems = 'flex-start'
        contentEl.style.gap = '8px'

        const textEl = document.createElement('div')
        textEl.style.flex = '1'
        let inner = `<div>${item.name}</div><span class="val-detail">${escapeHtml(item.message)}</span>`
        if (item.repair) inner += `<div class="val-repair">Fix: ${escapeHtml(item.repair)}</div>`
        textEl.innerHTML = inner

        const lineEl = document.createElement('div')
        lineEl.style.fontSize = '11px'
        lineEl.style.opacity = '0.6'
        lineEl.style.whiteSpace = 'nowrap'
        lineEl.textContent = item.line ? `Ln ${item.line}` : ''

        contentEl.appendChild(textEl)
        if (item.line) contentEl.appendChild(lineEl)

        el.appendChild(contentEl)

        // Make clickable to jump to line
        if (item.line) {
          el.style.cursor = 'pointer'
          el.addEventListener('click', () => {
            monacoEditor.revealLineInCenter(item.line!)
            monacoEditor.setPosition({ lineNumber: item.line!, column: item.column || 1 })
            monacoEditor.focus()
          })
        }

        group.appendChild(el)
      }

      validationResults.appendChild(group)
    }
  }

  // ── Assembly soundness (topology validator) ──
  // The structured findings — stable code + message + suggested_repair — that
  // drive the AI self-correction loop, rendered so a human sees the same signal.
  function assemblyResults(content: string): ValResult[] {
    let diags: StructuredDiagnostic[] = []
    try { diags = getAssemblyDiagnostics() } catch { diags = [] }
    return diags.map(d => ({
      name: d.code,
      severity: d.severity === 'error' ? 'error' : 'warn',
      message: d.message,
      category: 'Assembly',
      line: lineOfName(content, d.link_name),
      repair: d.suggested_repair,
    }))
  }

  // ── Client-side validation (no backend round-trip) ──
  // Structural (line-accurate XML) + Assembly soundness + per-link completeness.
  // This is what auto-run uses, so it never blocks the AI completion mutex.
  function clientResults(content: string): ValResult[] {
    const structural = validateXMLStructure(content)
    if (structural.some(r => r.severity === 'error' && PARSE_BLOCKERS.has(r.name))) {
      return structural
    }
    return [...structural, ...assemblyResults(content), ...validateURDFPerLink(content)]
  }

  function finish(results: ValResult[]) {
    const summary = buildSummary(results)
    renderValidationResults(results, summary)
    setValidationMarkers(results)
  }

  // ── Run full validation (client-side + Python backend augmentation) ──

  async function runValidation() {
    btnRevalidate.disabled = true
    btnRevalidate.textContent = 'Validating…'

    try {
      const content = getContent()
      const client = clientResults(content)

      // Can't go deeper than the structural error if the XML won't parse.
      const blocked = client.some(r => r.severity === 'error' && PARSE_BLOCKERS.has(r.name))
      if (!blocked) {
        try {
          const result = await invoke('validate_urdf_content', { urdf_content: content })
          const backend = (result as any)?.results as ValResult[] | undefined
          const backendOwned = (backend ?? []).filter(r => BACKEND_CATEGORIES.has(r.category))
          if (backendOwned.length > 0) {
            // Backend ran and produced its richer Physics/Actuators/Mesh/Spatial
            // (inertia triangle-inequality, effort limits, overlaps). Swap those
            // in; keep the client's line-accurate Structural + Assembly, and
            // take ONLY the backend-owned categories so its own (redundant)
            // Structural pass doesn't double up with the client's.
            const kept = client.filter(r => !BACKEND_CATEGORIES.has(r.category))
            finish([...kept, ...backendOwned])
            return
          }
          // Backend reachable but produced nothing it owns (e.g. a "Dependencies
          // Missing" / Setup notice) — keep the full client set and surface any
          // actionable notice alongside it rather than dropping checks.
          const notices = (backend ?? []).filter(r => r.severity === 'error' || r.severity === 'warn')
          if (notices.length > 0) {
            finish([...client, ...notices])
            return
          }
        } catch (_e) {
          // Backend unavailable (interpreter without deps, core not running) —
          // the client set already stands on its own.
        }
      }
      finish(client)
    } catch (_e) {
      // Last resort: validate against the in-memory kinematic graph.
      finish(buildLocalResults())
    } finally {
      btnRevalidate.disabled = false
      btnRevalidate.textContent = 'Run Checks'
    }
  }

  // ── Shared summary builder ──

  function buildSummary(results: ValResult[]) {
    const summary = { pass: 0, warn: 0, error: 0, info: 0 }
    for (const r of results) {
      if (r.severity in summary) summary[r.severity as keyof typeof summary]++
    }
    return summary
  }

  // ── In-memory graph fallback (only when the editor text can't be read) ──

  function buildLocalResults(): ValResult[] {
    const kinematicGraph = getKinematicGraph()
    const kinematicJoints = getKinematicJoints()
    const results: ValResult[] = []

    // ── Structural checks ──
    const linkNames = Object.keys(kinematicGraph).filter(n => !n.includes('__mount__'))
    const rootLink = Object.values(kinematicGraph).find(l => !l.parent)?.name ?? 'base_link'
    const hasRoot = linkNames.includes(rootLink)

    results.push({
      name: 'Root link defined',
      severity: hasRoot ? 'pass' : 'error',
      message: hasRoot ? `Root link '${rootLink}' exists` : 'No root link found',
      category: 'Structural',
    })

    // Orphan check (exclude mount nodes)
    const orphans = linkNames.filter(n => n !== rootLink && !kinematicGraph[n].parent)
    results.push({
      name: 'No orphan links',
      severity: orphans.length === 0 ? 'pass' : 'error',
      message: orphans.length === 0 ? 'All links connected to tree' : `Orphans: ${orphans.join(', ')}`,
      category: 'Structural',
    })

    // Cycle check via DFS
    let hasCycle = false
    const visited = new Set<string>()
    function dfs(name: string, path: Set<string>) {
      if (path.has(name)) { hasCycle = true; return }
      if (visited.has(name)) return
      visited.add(name)
      path.add(name)
      for (const child of kinematicGraph[name]?.children || []) {
        if (!child.includes('__mount__')) dfs(child, path)
      }
      path.delete(name)
    }
    dfs(rootLink, new Set())

    results.push({
      name: 'Tree structure OK',
      severity: hasCycle ? 'error' : 'pass',
      message: hasCycle ? 'Cycle detected in kinematic tree' : 'Valid tree (no cycles)',
      category: 'Structural',
    })

    // ── Physics checks ──
    const zeroMassLinks = linkNames.filter(n => n !== rootLink && kinematicGraph[n].mass === 0)
    results.push({
      name: 'Link masses set',
      severity: zeroMassLinks.length > 0 ? 'warn' : 'pass',
      message: zeroMassLinks.length > 0
        ? `Zero mass on: ${zeroMassLinks.join(', ')}`
        : 'All non-root links have positive mass',
      category: 'Physics',
    })

    const totalMass = linkNames.reduce((sum, n) => sum + (kinematicGraph[n].mass || 0), 0)
    results.push({
      name: 'Total mass',
      severity: 'info',
      message: `Total robot mass: ${totalMass.toFixed(3)} kg`,
      category: 'Physics',
    })

    // ── Actuator checks ──
    const jointNamesList = Object.keys(kinematicJoints).filter(j => !kinematicJoints[j].childLink.includes('__mount__'))
    const actuated = jointNamesList.filter(j => kinematicJoints[j].type !== 'fixed')
    const fixed = jointNamesList.filter(j => kinematicJoints[j].type === 'fixed')

    results.push({
      name: 'Joint summary',
      severity: 'info',
      message: `${actuated.length} actuated, ${fixed.length} fixed joints`,
      category: 'Actuators',
    })

    return results
  }

  /** Client-side validation against the current editor text. Falls back to the
   *  in-memory graph only when the editor has no readable content yet. */
  function runLocalValidation() {
    const content = getContent()
    if (!content.trim()) {
      finish(buildLocalResults())
      return
    }
    finish(clientResults(content))
  }

  // ── Revalidate button click handler ──
  btnRevalidate.addEventListener('click', () => {
    runValidation()
  })

  // Auto-run validation on load
  runLocalValidation()

  return {
    runLocalValidation,
    runValidation,
    setValidationMarkers,
  }
}

/** Minimal HTML escaping for validator-supplied strings rendered via innerHTML. */
function escapeHtml(s: string): string {
  return s
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
}

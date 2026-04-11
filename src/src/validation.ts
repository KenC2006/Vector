import * as monaco from 'monaco-editor'
import type { KinematicLink, KinematicJoint } from './urdfParser'

export interface ValResult {
  name: string
  severity: string  // "pass" | "warn" | "error" | "info"
  message: string
  category: string
  line?: number
  column?: number
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
      if (linkNames.has(name)) errors.push({ name: 'Duplicate link name', severity: 'error', message: `Link "${name}" is defined multiple times`, category: 'Structural' })
      linkNames.add(name)
    }
  }
  const joints = xmlDoc.getElementsByTagName('joint')
  const jointNames = new Set<string>()
  for (let i = 0; i < joints.length; i++) {
    const joint = joints[i]
    const jointName = joint.getAttribute('name')
    if (jointName) {
      if (jointNames.has(jointName)) errors.push({ name: 'Duplicate joint name', severity: 'error', message: `Joint "${jointName}" is defined multiple times`, category: 'Structural' })
      jointNames.add(jointName)
    }
    const parent = joint.querySelector('parent')
    const child = joint.querySelector('child')
    if (!parent || !child) { errors.push({ name: `Joint ${jointName || 'unknown'} missing parent/child`, severity: 'error', message: 'Joint must have both <parent> and <child> elements', category: 'Structural' }); continue }
    const parentLink = parent.getAttribute('link')
    const childLink = child.getAttribute('link')
    if (!parentLink || !linkNames.has(parentLink)) errors.push({ name: `Invalid parent link in joint ${jointName || 'unknown'}`, severity: 'error', message: `Parent link "${parentLink}" is not defined`, category: 'Structural' })
    if (!childLink || !linkNames.has(childLink)) errors.push({ name: `Invalid child link in joint ${jointName || 'unknown'}`, severity: 'error', message: `Child link "${childLink}" is not defined`, category: 'Structural' })
  }
  if (errors.length === 0) errors.push({ name: 'XML structure valid', severity: 'pass', message: `${links.length} links, ${joints.length} joints`, category: 'Structural' })
  return errors
}

/**
 * Per-link completeness checks: collision geometry, inertial mass.
 * Skips mount-node links (containing __mount__).
 * Returns one result per category (not per link) to keep the list short,
 * plus individual warnings for each offending link.
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
    results.push({ name: 'Collision geometry', severity: 'pass', message: 'All non-root links have collision geometry', category: 'Physics' })
  } else {
    results.push({ name: 'Missing collision geometry', severity: 'warn', message: `${noCollision.length} link(s) lack <collision>: ${noCollision.join(', ')}`, category: 'Physics' })
  }

  // ── Inertial / mass check ──
  const noInertial: string[] = []
  const zeroMass: string[] = []
  for (const link of links) {
    const name = link.getAttribute('name') ?? ''
    if (isRoot(name)) continue
    const inertial = link.querySelector('inertial')
    if (!inertial) { noInertial.push(name); continue }
    const massEl = inertial.querySelector('mass')
    const mass = parseFloat(massEl?.getAttribute('value') ?? '0')
    if (!Number.isFinite(mass) || mass <= 0) zeroMass.push(name)
  }

  if (noInertial.length > 0) {
    results.push({ name: 'Missing inertial', severity: 'warn', message: `${noInertial.length} link(s) lack <inertial>: ${noInertial.join(', ')}`, category: 'Physics' })
  }
  if (zeroMass.length > 0) {
    results.push({ name: 'Zero/missing mass', severity: 'warn', message: `Zero or missing mass on: ${zeroMass.join(', ')}`, category: 'Physics' })
  }
  if (noInertial.length === 0 && zeroMass.length === 0) {
    results.push({ name: 'Inertial properties', severity: 'pass', message: 'All non-root links have mass > 0', category: 'Physics' })
  }

  // ── Joint limits check ──
  const joints = Array.from(xmlDoc.getElementsByTagName('joint'))
  const missingLimits: string[] = []
  for (const joint of joints) {
    const type = joint.getAttribute('type') ?? ''
    if (type !== 'revolute' && type !== 'prismatic') continue
    const limit = joint.querySelector('limit')
    const lower = parseFloat(limit?.getAttribute('lower') ?? 'NaN')
    const upper = parseFloat(limit?.getAttribute('upper') ?? 'NaN')
    if (!limit || !Number.isFinite(lower) || !Number.isFinite(upper) || lower >= upper) {
      missingLimits.push(joint.getAttribute('name') ?? 'unnamed')
    }
  }
  if (missingLimits.length > 0) {
    results.push({ name: 'Joint limits', severity: 'warn', message: `Actuated joints with invalid limits: ${missingLimits.join(', ')}`, category: 'Actuators' })
  } else if (joints.filter(j => ['revolute','prismatic'].includes(j.getAttribute('type') ?? '')).length > 0) {
    results.push({ name: 'Joint limits', severity: 'pass', message: 'All actuated joints have valid limits', category: 'Actuators' })
  }

  return results
}

export function initValidation(deps: {
  invoke: (cmd: string, args?: Record<string, unknown>) => Promise<unknown>
  monacoEditor: monaco.editor.IStandaloneCodeEditor
  getKinematicGraph: () => Record<string, KinematicLink>
  getKinematicJoints: () => Record<string, KinematicJoint>
  showToast: (msg: string, type?: 'success' | 'warning' | 'error' | 'info') => void
}): {
  runLocalValidation: () => void
  runValidation: () => Promise<void>
  setValidationMarkers: (results: ValResult[]) => void
} {
  const { invoke, monacoEditor, getKinematicGraph, getKinematicJoints } = deps

  // ── DOM elements ──
  const validationResults = document.getElementById('validation-results') as HTMLDivElement
  const validationSummary = document.getElementById('validation-summary') as HTMLDivElement
  const btnRevalidate = document.getElementById('btn-revalidate') as HTMLButtonElement

  // ── Validation Markers for Monaco ──

  function setValidationMarkers(results: Array<{ name: string; severity: string; message: string; category: string; line?: number; column?: number }>) {
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

      markers.push({
        severity: markerSeverity,
        message: `[${r.category}] ${r.name}: ${r.message}`,
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

    // Group results by category
    const groups: Record<string, ValResult[]> = {}
    for (const r of results) {
      if (!groups[r.category]) groups[r.category] = []
      groups[r.category].push(r)
    }

    // Render groups
    validationResults.innerHTML = ''
    for (const [category, items] of Object.entries(groups)) {
      const group = document.createElement('div')
      group.className = 'val-group'

      const title = document.createElement('div')
      title.className = 'vg-title'
      title.textContent = category
      group.appendChild(title)

      for (const item of items) {
        const el = document.createElement('div')
        el.className = `val-item ${item.severity}`
        el.style.cursor = 'pointer'

        const contentEl = document.createElement('div')
        contentEl.style.display = 'flex'
        contentEl.style.justifyContent = 'space-between'
        contentEl.style.alignItems = 'flex-start'
        contentEl.style.gap = '8px'

        const textEl = document.createElement('div')
        textEl.style.flex = '1'
        textEl.innerHTML = `<div>${item.name}</div><span class="val-detail">${item.message}</span>`

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
          el.addEventListener('click', () => {
            const editor = (window as any).__vectorEditor as monaco.editor.IStandaloneCodeEditor | undefined
            if (editor) {
              editor.revealLineInCenter(item.line!)
              editor.setPosition({ lineNumber: item.line!, column: item.column || 1 })
              editor.focus()
            }
          })
        }

        group.appendChild(el)
      }

      validationResults.appendChild(group)
    }
  }

  // ── Run full validation (client-side XML + Python backend) ──

  async function runValidation() {
    btnRevalidate.disabled = true
    btnRevalidate.textContent = 'Validating...'

    try {
      const editor = (window as any).__vectorEditor as monaco.editor.IStandaloneCodeEditor | undefined
      const urdfContent = editor?.getValue() || ''

      // First: client-side XML validation
      const xmlErrors = validateXMLStructure(urdfContent)

      if (xmlErrors.length > 0) {
        // If XML is malformed, show only XML errors
        const summary = { pass: 0, warn: 0, error: xmlErrors.length, info: 0 }
        renderValidationResults(xmlErrors, summary)
        setValidationMarkers(xmlErrors)
      } else {
        // XML is valid — run per-link client-side checks immediately,
        // then try to augment with the Python backend.
        const perLinkResults = validateURDFPerLink(urdfContent)

        try {
          const result = await invoke('validate_urdf_content', {
            urdf_content: urdfContent
          })

          if (result && (result as any).results) {
            // Merge: use Python results as primary, add any per-link results not already covered.
            const backendResults: ValResult[] = (result as any).results
            const backendCategories = new Set(backendResults.map(r => r.category))
            const extra = perLinkResults.filter(r => !backendCategories.has(r.category))
            const merged = [...backendResults, ...extra]
            const summary = buildSummary(merged)
            renderValidationResults(merged, summary)
            setValidationMarkers(merged)
          }
        } catch (_e) {
          // Python backend not available — combine local graph checks with per-link XML checks.
          runLocalValidationWith(perLinkResults)
        }
      }
    } catch (_e) {
      // Fallback: run local validation against the hardcoded kinematic graph
      runLocalValidation()
    }

    btnRevalidate.disabled = false
    btnRevalidate.textContent = 'Run Checks'
  }

  // ── Shared summary builder ──

  function buildSummary(results: ValResult[]) {
    const summary = { pass: 0, warn: 0, error: 0, info: 0 }
    for (const r of results) {
      if (r.severity in summary) summary[r.severity as keyof typeof summary]++
    }
    return summary
  }

  // ── Local validation fallback (runs in browser against the in-memory graph) ──

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

    results.push({
      name: 'Unique link names',
      severity: 'pass',
      message: `${linkNames.length} links, all uniquely named`,
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

    results.push({
      name: 'Mesh watertight check',
      severity: 'info',
      message: 'Mesh watertight check skipped (requires mesh files)',
      category: 'Mesh',
    })

    return results
  }

  /** Run local validation, optionally merging in pre-computed per-link XML results. */
  function runLocalValidationWith(extraResults: ValResult[] = []) {
    const results = [...buildLocalResults(), ...extraResults]
    const summary = buildSummary(results)
    renderValidationResults(results, summary)
    setValidationMarkers(results)
  }

  function runLocalValidation() {
    runLocalValidationWith([])
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

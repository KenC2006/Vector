import { readFile } from 'node:fs/promises'
import { fileURLToPath } from 'node:url'
import path from 'node:path'

const repoRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..')

const PRESET_FILES = [
  'core/presets/generic_presets.json',
]

const DEPRECATED_PHYSICAL_FIELDS = [
  'outer_diameter_mm',
  'inner_diameter_mm',
  'wall_thickness_mm',
  'diameter_mm',
  'thickness_mm',
]

const CONNECTOR_TYPES = new Set(['planar', 'cylindrical', 'point'])

const errors = []
const warnings = []
const catalogs = []

function isObject(value) {
  return !!value && typeof value === 'object' && !Array.isArray(value)
}

function isFiniteNumber(value) {
  return typeof value === 'number' && Number.isFinite(value)
}

function numberTuple(value, length) {
  return Array.isArray(value) && value.length === length && value.every(isFiniteNumber)
}

function warn(file, message) {
  warnings.push(`${file}: ${message}`)
}

function error(file, message) {
  errors.push(`${file}: ${message}`)
}

async function loadJson(relPath) {
  const absPath = path.join(repoRoot, relPath)
  try {
    return JSON.parse(await readFile(absPath, 'utf8'))
  } catch (err) {
    error(relPath, `failed to parse JSON: ${err.message}`)
    return null
  }
}

function validateConnector(file, categoryName, componentId, connector, index, bboxMm) {
  const prefix = `${categoryName}.${componentId}.connectors[${index}]`
  if (!isObject(connector)) {
    error(file, `${prefix} must be an object`)
    return
  }
  if (typeof connector.id !== 'string' || connector.id.trim() === '') {
    error(file, `${prefix}.id must be a non-empty string`)
  }
  if (!numberTuple(connector.origin_xyz_mm, 3)) {
    error(file, `${prefix}.origin_xyz_mm must be a 3-number tuple`)
  }
  if (!numberTuple(connector.axis_xyz, 3)) {
    error(file, `${prefix}.axis_xyz must be a 3-number tuple`)
  }
  if (!CONNECTOR_TYPES.has(connector.type)) {
    error(file, `${prefix}.type must be one of ${Array.from(CONNECTOR_TYPES).join(', ')}`)
  }
  if (connector.diameter_mm !== undefined && !isFiniteNumber(connector.diameter_mm)) {
    error(file, `${prefix}.diameter_mm must be a number when present`)
  }
  if (connector.engagement_depth_mm !== undefined && !isFiniteNumber(connector.engagement_depth_mm)) {
    error(file, `${prefix}.engagement_depth_mm must be a number when present`)
  }
  // Connector origin must lie within bbox half-extents (±0.5mm slack for
  // float noise). Connectors floating outside the AABB indicate a preset
  // bug — either bbox is undersized or the connector is mis-authored.
  if (bboxMm && numberTuple(connector.origin_xyz_mm, 3)) {
    const [hx, hy, hz] = [bboxMm[0] / 2, bboxMm[1] / 2, bboxMm[2] / 2]
    const [ox, oy, oz] = connector.origin_xyz_mm
    const TOL = 0.5
    if (Math.abs(ox) > hx + TOL || Math.abs(oy) > hy + TOL || Math.abs(oz) > hz + TOL) {
      error(file, `${prefix}.origin_xyz_mm [${ox}, ${oy}, ${oz}] falls outside bbox half-extents [${hx}, ${hy}, ${hz}]`)
    }
  }
}

function validateComponent(file, categoryName, component, seenIds) {
  if (!isObject(component)) {
    error(file, `${categoryName}.components entry must be an object`)
    return
  }
  if (typeof component.id !== 'string' || component.id.trim() === '') {
    error(file, `${categoryName}.components entry missing non-empty id`)
    return
  }
  if (seenIds.has(component.id)) {
    error(file, `duplicate component id '${component.id}'`)
  }
  seenIds.add(component.id)

  if (!isObject(component.physical)) {
    error(file, `${categoryName}.${component.id}.physical must be an object`)
    return
  }

  const physical = component.physical
  for (const field of DEPRECATED_PHYSICAL_FIELDS) {
    if (Object.prototype.hasOwnProperty.call(physical, field)) {
      warn(file, `${categoryName}.${component.id}.physical.${field} is deprecated and kept only for migration compatibility`)
    }
  }

  const hasCurrentBBox = numberTuple(physical.bounding_box_mm, 3)
  const hasTargetBBox = numberTuple(physical.bbox_mm, 3)
  const hasCurrentParametric = Array.isArray(physical.cross_section_mm)
    && (numberTuple(physical.cross_section_mm, 2) || numberTuple(physical.cross_section_mm, 3))
  const hasTargetParametric = isObject(physical.parametric)
    && ['x', 'y', 'z'].includes(physical.parametric.axis)
    && numberTuple(physical.parametric.cross_section_mm, 2)

  const hasLegacyDimensionalFallback =
    isFiniteNumber(physical.outer_diameter_mm)
    || isFiniteNumber(physical.inner_diameter_mm)
    || isFiniteNumber(physical.wall_thickness_mm)
    || isFiniteNumber(physical.diameter_mm)
    || isFiniteNumber(physical.thickness_mm)

  if (!hasCurrentBBox && !hasTargetBBox && !hasCurrentParametric && !hasTargetParametric) {
    if (hasLegacyDimensionalFallback) {
      warn(file, `${categoryName}.${component.id} has only legacy dimensional fields; add bounding_box_mm/bbox_mm or parametric before Phase 6`)
    } else {
      error(file, `${categoryName}.${component.id} must define bounding_box_mm/bbox_mm or cross_section_mm/parametric`)
    }
  }
  if (physical.bounding_box_mm !== undefined && !hasCurrentBBox) {
    error(file, `${categoryName}.${component.id}.physical.bounding_box_mm must be a 3-number tuple`)
  }
  if (physical.bbox_mm !== undefined && !hasTargetBBox) {
    error(file, `${categoryName}.${component.id}.physical.bbox_mm must be a 3-number tuple`)
  }
  if (physical.cross_section_mm !== undefined && !hasCurrentParametric) {
    error(file, `${categoryName}.${component.id}.physical.cross_section_mm must be a 2- or 3-number tuple`)
  }
  if (physical.parametric !== undefined && !hasTargetParametric) {
    error(file, `${categoryName}.${component.id}.physical.parametric must define axis x/y/z and cross_section_mm as a 2-number tuple`)
  }
  if (physical.mass_kg !== undefined && !isFiniteNumber(physical.mass_kg)) {
    error(file, `${categoryName}.${component.id}.physical.mass_kg must be a number when present`)
  }
  if (!isObject(component.mechanical_electrical)) {
    error(file, `${categoryName}.${component.id}.mechanical_electrical must be an object`)
  }
  if (component.mounting_logic !== undefined && !isObject(component.mounting_logic)) {
    error(file, `${categoryName}.${component.id}.mounting_logic must be an object when present`)
  }
  if (component.connectors !== undefined) {
    if (!Array.isArray(component.connectors)) {
      error(file, `${categoryName}.${component.id}.connectors must be an array when present`)
    } else {
      const bboxMm = hasCurrentBBox ? physical.bounding_box_mm : (hasTargetBBox ? physical.bbox_mm : null)
      component.connectors.forEach((connector, index) => validateConnector(file, categoryName, component.id, connector, index, bboxMm))
    }
  }
}

function validateCatalog(file, catalog) {
  if (!isObject(catalog)) {
    error(file, 'top-level JSON must be an object')
    return { ids: new Set() }
  }
  if (!isObject(catalog.categories)) {
    error(file, 'top-level categories must be an object')
    return { ids: new Set() }
  }

  const seenIds = new Set()
  for (const [categoryName, category] of Object.entries(catalog.categories)) {
    if (!isObject(category)) {
      error(file, `category '${categoryName}' must be an object`)
      continue
    }
    if (!Array.isArray(category.components)) {
      error(file, `category '${categoryName}'.components must be an array`)
      continue
    }
    for (const component of category.components) {
      validateComponent(file, categoryName, component, seenIds)
    }
  }
  return { ids: seenIds }
}

for (const file of PRESET_FILES) {
  const catalog = await loadJson(file)
  if (catalog) catalogs.push({ file, ...validateCatalog(file, catalog) })
}

if (catalogs.length === PRESET_FILES.length) {
  const [first, ...rest] = catalogs
  for (const catalog of rest) {
    const missing = Array.from(first.ids).filter(id => !catalog.ids.has(id))
    const extra = Array.from(catalog.ids).filter(id => !first.ids.has(id))
    if (missing.length || extra.length) {
      error(catalog.file, `component id set does not match ${first.file}; missing=${missing.length}, extra=${extra.length}`)
    }
  }
}

if (process.env.COMPONENT_SPEC_VERBOSE === '1') {
  for (const message of warnings) console.warn(`[component-spec warning] ${message}`)
} else if (warnings.length > 0) {
  console.warn(`[component-spec warning] ${warnings.length} migration warning(s); set COMPONENT_SPEC_VERBOSE=1 for full details`)
  const legacyOnly = warnings.filter(message => message.includes('has only legacy dimensional fields'))
  for (const message of legacyOnly) console.warn(`[component-spec warning] ${message}`)
}
for (const message of errors) console.error(`[component-spec error] ${message}`)

if (errors.length > 0) {
  console.error(`component spec validation failed: ${errors.length} error(s), ${warnings.length} warning(s)`)
  process.exit(1)
}

console.log(`component spec validation passed: ${warnings.length} warning(s)`)

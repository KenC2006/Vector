// Surgical connector inserter for core/presets/generic_presets.json.
// Reads CRLF text, finds preset by id, inserts a connectors[] block right
// after the closing "}" of sim_metadata (i.e. as the last preset field).
// Usage: node scripts/add_connectors.js <category-module>
const fs = require('fs');
const path = require('path');

const FILE = 'core/presets/generic_presets.json';

function fmtScalar(v) {
  if (typeof v === 'string') return JSON.stringify(v);
  if (typeof v === 'number') return String(v);
  if (typeof v === 'boolean') return String(v);
  if (v === null) return 'null';
  throw new Error('Unhandled scalar: ' + JSON.stringify(v));
}

function buildConnectorsBlock(connectors, eol) {
  // Mirror existing 2-space JSON.stringify formatting at 10/12/14 indent levels.
  const lines = [];
  lines.push('          "connectors": [');
  for (let i = 0; i < connectors.length; i++) {
    const conn = connectors[i];
    lines.push('            {');
    const keys = Object.keys(conn);
    for (let j = 0; j < keys.length; j++) {
      const k = keys[j];
      const v = conn[k];
      const last = j === keys.length - 1;
      let valStr;
      if (Array.isArray(v)) {
        const arrLines = ['['];
        for (let q = 0; q < v.length; q++) {
          arrLines.push('                ' + fmtScalar(v[q]) + (q === v.length - 1 ? '' : ','));
        }
        arrLines.push('              ]');
        valStr = arrLines.join(eol);
      } else {
        valStr = fmtScalar(v);
      }
      lines.push('              ' + JSON.stringify(k) + ': ' + valStr + (last ? '' : ','));
    }
    lines.push('            }' + (i === connectors.length - 1 ? '' : ','));
  }
  lines.push('          ]');
  return lines.join(eol);
}

function findPresetCloseBrace(raw, presetId) {
  const idMarker = '"id": ' + JSON.stringify(presetId);
  const idIdx = raw.indexOf(idMarker);
  if (idIdx === -1) throw new Error('id not found: ' + presetId);
  // Walk back to opening '{'
  let openIdx = idIdx;
  while (openIdx > 0 && raw[openIdx] !== '{') openIdx--;
  // Scan forward, balanced braces, ignoring those inside strings.
  let depth = 0;
  let inStr = false;
  let escape = false;
  for (let i = openIdx; i < raw.length; i++) {
    const c = raw[i];
    if (escape) { escape = false; continue; }
    if (c === '\\') { escape = true; continue; }
    if (c === '"') { inStr = !inStr; continue; }
    if (inStr) continue;
    if (c === '{') depth++;
    else if (c === '}') {
      depth--;
      if (depth === 0) return { openIdx, closeIdx: i };
    }
  }
  throw new Error('unbalanced braces for preset ' + presetId);
}

function insertConnectorsForPreset(raw, presetId, connectors) {
  // Detect EOL.
  const eol = raw.includes('\r\n') ? '\r\n' : '\n';

  // Skip if already present. Search within preset bounds.
  const { openIdx, closeIdx } = findPresetCloseBrace(raw, presetId);
  const slice = raw.slice(openIdx, closeIdx);
  if (/\"connectors\"\s*:/.test(slice)) {
    console.log('  skip (already authored):', presetId);
    return raw;
  }

  // Find sim_metadata closing brace inside the preset (the last "}" before
  // the preset's outer "}" at closeIdx). Walk back from closeIdx-1, skipping
  // whitespace, and require that what we find is "}".
  let i = closeIdx - 1;
  while (i > openIdx && /\s/.test(raw[i])) i--;
  if (raw[i] !== '}') throw new Error('expected sim_metadata } before preset close for ' + presetId + ', got ' + JSON.stringify(raw[i]));
  // i points to the inner "}". Insertion goes right after it.
  const before = raw.slice(0, i + 1);
  const after = raw.slice(i + 1);
  const block = buildConnectorsBlock(connectors, eol);
  // After-form: ",\r\n<block>\r\n        " then existing close brace.
  // The `after` slice already starts with eol + 8-space indent + "}". We
  // insert "," + eol + block before it, where block ends with the closing
  // "]" at 10-space indent. The line break after our block needs to lead
  // back into the existing 8-space "}" line, so prepend "," and the block.
  return before + ',' + eol + block + after;
}

function loadPresetMap() {
  const j = JSON.parse(fs.readFileSync(FILE, 'utf8'));
  const map = new Map();
  for (const [c, cat] of Object.entries(j.categories)) {
    if (cat && Array.isArray(cat.components)) {
      for (const p of cat.components) map.set(p.id, { category: c, preset: p });
    }
  }
  return map;
}

function applyPlan(plan) {
  const map = loadPresetMap();
  let raw = fs.readFileSync(FILE, 'utf8');
  for (const [id, connectors] of Object.entries(plan)) {
    if (!map.has(id)) {
      console.log('  MISSING preset id (skipped):', id);
      continue;
    }
    raw = insertConnectorsForPreset(raw, id, connectors);
    console.log('  applied:', id, `(${connectors.length} connectors)`);
  }
  fs.writeFileSync(FILE, raw);
  // Reparse to verify.
  JSON.parse(fs.readFileSync(FILE, 'utf8'));
  console.log('JSON parses OK.');
}

module.exports = { applyPlan };

if (require.main === module) {
  const which = process.argv[2];
  if (!which) { console.error('usage: node add_connectors.js <plan-module>'); process.exit(1); }
  const plan = require(path.resolve(which));
  applyPlan(plan);
}

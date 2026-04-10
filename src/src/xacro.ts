/**
 * Xacro preprocessor for URDF files.
 *
 * Implements the core xacro macro language features:
 * - Properties: <xacro:property name="x" value="1.0"/>
 * - Expressions: ${x * 2 + pi}
 * - Macros: <xacro:macro name="foo" params="a b c">...</xacro:macro>
 * - Includes: <xacro:include filename="other.xacro"/>
 * - Conditionals: <xacro:if value="${condition}">...</xacro:if>
 * - Arguments: <xacro:arg name="x" default="0"/>
 * - Insert blocks: <xacro:insert_block name="content"/>
 * - Block params: *block and **block in macro params
 *
 * Does NOT support:
 * - $(find package) ROS package resolution
 * - load_yaml()
 * - Full Python expression evaluation (uses safe math subset)
 * - Lazy evaluation
 */

// ── Safe math expression evaluator ──────────────────────────────────────────

const MATH_FUNCS: Record<string, (...args: number[]) => number> = {
  sin: Math.sin,
  cos: Math.cos,
  tan: Math.tan,
  asin: Math.asin,
  acos: Math.acos,
  atan: Math.atan,
  atan2: Math.atan2,
  abs: Math.abs,
  fabs: Math.abs,
  ceil: Math.ceil,
  floor: Math.floor,
  sqrt: Math.sqrt,
  pow: Math.pow,
  exp: Math.exp,
  log: Math.log,
  min: Math.min,
  max: Math.max,
  round: Math.round,
  radians: (deg: number) => deg * Math.PI / 180,
  degrees: (rad: number) => rad * 180 / Math.PI,
}

const MATH_CONSTS: Record<string, number> = {
  pi: Math.PI,
  PI: Math.PI,
  tau: Math.PI * 2,
  e: Math.E,
  true: 1,
  True: 1,
  false: 0,
  False: 0,
}

/**
 * Evaluate a math expression string with variables.
 * Supports: +, -, *, /, **, (), function calls, variable refs.
 * Safe — no eval(), no arbitrary code execution.
 */
function evalExpression(expr: string, vars: Record<string, string>): string {
  let resolved = expr.trim()

  // Replace variable references with their values
  // Sort by length descending to prevent partial matches
  const varNames = Object.keys(vars).sort((a, b) => b.length - a.length)
  for (const name of varNames) {
    const val = vars[name]
    // Replace whole-word occurrences
    const re = new RegExp(`\\b${escapeRegex(name)}\\b`, 'g')
    resolved = resolved.replace(re, val)
  }

  // Replace math constants
  for (const [name, val] of Object.entries(MATH_CONSTS)) {
    const re = new RegExp(`\\b${name}\\b`, 'g')
    resolved = resolved.replace(re, String(val))
  }

  // Replace function calls: func(args) → computed value
  let maxIter = 20
  while (maxIter-- > 0) {
    const funcMatch = resolved.match(/(\w+)\(([^()]*)\)/)
    if (!funcMatch) break
    const [fullMatch, funcName, argsStr] = funcMatch
    const func = MATH_FUNCS[funcName]
    if (func) {
      const args = argsStr.split(',').map(a => {
        try { return evalMathSafe(a.trim()) } catch { return 0 }
      })
      const result = func(...args)
      resolved = resolved.replace(fullMatch, String(result))
    } else {
      // Unknown function — try to evaluate the parenthesized expression
      try {
        const innerVal = evalMathSafe(argsStr)
        resolved = resolved.replace(`(${argsStr})`, String(innerVal))
      } catch {
        break
      }
    }
  }

  // Try to evaluate as a math expression
  try {
    const numResult = evalMathSafe(resolved)
    // Return as number string, trimming unnecessary decimals
    return cleanNumber(numResult)
  } catch {
    // Not a valid math expression — return as-is (might be a string)
    return resolved
  }
}

function cleanNumber(n: number): string {
  if (!Number.isFinite(n)) return '0'
  // Avoid scientific notation for small numbers
  const s = n.toPrecision(10)
  // Remove trailing zeros after decimal
  if (s.includes('.')) return s.replace(/\.?0+$/, '')
  return s
}

/**
 * Safe math evaluator — parses and evaluates arithmetic expressions.
 * Supports: +, -, *, /, **, unary -, parentheses, numbers.
 * No eval() or Function() — fully hand-parsed.
 */
function evalMathSafe(expr: string): number {
  const tokens = tokenize(expr.trim())
  let pos = 0

  function peek(): string { return tokens[pos] || '' }
  function consume(): string { return tokens[pos++] || '' }

  function parseExpr(): number {
    let left = parseTerm()
    while (peek() === '+' || peek() === '-') {
      const op = consume()
      const right = parseTerm()
      left = op === '+' ? left + right : left - right
    }
    return left
  }

  function parseTerm(): number {
    let left = parsePower()
    while (peek() === '*' || peek() === '/') {
      const op = consume()
      const right = parsePower()
      left = op === '*' ? left * right : left / right
    }
    return left
  }

  function parsePower(): number {
    let base = parseUnary()
    if (peek() === '**') {
      consume()
      const exp = parseUnary()
      base = Math.pow(base, exp)
    }
    return base
  }

  function parseUnary(): number {
    if (peek() === '-') {
      consume()
      return -parseAtom()
    }
    if (peek() === '+') {
      consume()
    }
    return parseAtom()
  }

  function parseAtom(): number {
    if (peek() === '(') {
      consume() // (
      const val = parseExpr()
      consume() // )
      return val
    }
    const tok = consume()
    const n = parseFloat(tok)
    if (isNaN(n)) throw new Error(`Invalid number: ${tok}`)
    return n
  }

  const result = parseExpr()
  return result
}

function tokenize(expr: string): string[] {
  const tokens: string[] = []
  let i = 0
  while (i < expr.length) {
    if (/\s/.test(expr[i])) { i++; continue }
    // Number (including decimals and scientific notation)
    if (/[\d.]/.test(expr[i]) || (expr[i] === '-' && (tokens.length === 0 || /[+\-*/(,]/.test(tokens[tokens.length - 1])))) {
      let num = ''
      if (expr[i] === '-') { num += '-'; i++ }
      while (i < expr.length && /[\d.eE]/.test(expr[i])) { num += expr[i++] }
      tokens.push(num)
    } else if (expr[i] === '*' && expr[i + 1] === '*') {
      tokens.push('**'); i += 2
    } else if ('+-*/()'.includes(expr[i])) {
      tokens.push(expr[i++])
    } else {
      // Variable or function name — shouldn't reach here after substitution
      let name = ''
      while (i < expr.length && /[\w.]/.test(expr[i])) { name += expr[i++] }
      if (name) tokens.push(name)
      else i++ // skip unknown char
    }
  }
  return tokens
}

function escapeRegex(s: string): string {
  return s.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')
}

// ── Xacro Processor ─────────────────────────────────────────────────────────

interface XacroMacro {
  name: string
  params: Array<{ name: string; defaultValue?: string; isBlock: boolean; isDoubleBlock: boolean }>
  body: string // raw XML string of macro body
}

interface XacroContext {
  properties: Record<string, string>
  macros: Record<string, XacroMacro>
  args: Record<string, string>
  fileLoader?: (filename: string) => Promise<string> | string
  basePath?: string
}

/**
 * Process a xacro XML string into plain URDF XML.
 *
 * @param xacroXml - The xacro XML source
 * @param options - Optional configuration
 * @returns Processed URDF XML string
 */
export async function processXacro(
  xacroXml: string,
  options?: {
    /** Function to load included files. Receives filename, returns XML string. */
    fileLoader?: (filename: string) => Promise<string> | string
    /** Base path for resolving relative includes */
    basePath?: string
    /** Command-line arguments (name=value pairs) */
    args?: Record<string, string>
  },
): Promise<string> {
  const ctx: XacroContext = {
    properties: {},
    macros: {},
    args: options?.args ?? {},
    fileLoader: options?.fileLoader,
    basePath: options?.basePath ?? '',
  }

  // Process the document
  let result = await processXacroString(xacroXml, ctx)

  // Clean up xacro namespace declarations
  result = result.replace(/\s*xmlns:xacro="[^"]*"/g, '')

  return result
}

async function processXacroString(xml: string, ctx: XacroContext): Promise<string> {
  // First pass: extract and process xacro:arg defaults
  xml = processArgs(xml, ctx)

  // Second pass: extract xacro:property definitions
  xml = extractProperties(xml, ctx)

  // Third pass: extract xacro:macro definitions
  xml = extractMacros(xml, ctx)

  // Fourth pass: process includes
  xml = await processIncludes(xml, ctx)

  // Re-extract after includes (included files may define properties/macros)
  xml = extractProperties(xml, ctx)
  xml = extractMacros(xml, ctx)

  // Fifth pass: expand macros (iterative until no more expansions)
  let maxIter = 50
  while (maxIter-- > 0) {
    const expanded = expandMacros(xml, ctx)
    if (expanded === xml) break
    xml = expanded
    // Re-extract properties that macros might have defined
    xml = extractProperties(xml, ctx)
  }

  // Sixth pass: process conditionals
  xml = processConditionals(xml, ctx)

  // Seventh pass: substitute all ${...} expressions
  xml = substituteExpressions(xml, ctx)

  // Eighth pass: process insert_block
  xml = processInsertBlocks(xml, ctx)

  return xml
}

// ── Pass 1: Args ─────────────────────────────────────────────────────────────

function processArgs(xml: string, ctx: XacroContext): string {
  // <xacro:arg name="x" default="0"/>
  const argRe = /<xacro:arg\s+name="([^"]+)"\s+default="([^"]*)"\s*\/>/g
  return xml.replace(argRe, (_match, name, defaultVal) => {
    if (!(name in ctx.args)) {
      ctx.args[name] = defaultVal
    }
    // Also make args available as properties for ${} substitution
    ctx.properties[name] = ctx.args[name]
    return '' // remove the xacro:arg tag
  })
}

// ── Pass 2: Properties ───────────────────────────────────────────────────────

function extractProperties(xml: string, ctx: XacroContext): string {
  // Self-closing: <xacro:property name="x" value="1.0"/>
  const propRe = /<xacro:property\s+name="([^"]+)"\s+value="([^"]*)"\s*\/>/g
  xml = xml.replace(propRe, (_match, name, value) => {
    // Evaluate expressions in the value
    ctx.properties[name] = substituteExpressionsInString(value, ctx)
    return ''
  })

  // Block properties: <xacro:property name="x">...content...</xacro:property>
  const blockPropRe = /<xacro:property\s+name="([^"]+)"\s*>([\s\S]*?)<\/xacro:property>/g
  xml = xml.replace(blockPropRe, (_match, name, content) => {
    ctx.properties[name] = content.trim()
    return ''
  })

  return xml
}

// ── Pass 3: Macros ───────────────────────────────────────────────────────────

function extractMacros(xml: string, ctx: XacroContext): string {
  // Match <xacro:macro name="..." params="...">...</xacro:macro>
  // Need to handle nested macros — use a simple depth counter
  const macroStartRe = /<xacro:macro\s+name="([^"]+)"(?:\s+params="([^"]*)")?\s*>/g
  let match
  const macrosToRemove: Array<{ start: number; end: number }> = []

  while ((match = macroStartRe.exec(xml)) !== null) {
    const macroName = match[1]
    const paramsStr = match[2] || ''
    const startIdx = match.index
    const bodyStart = startIdx + match[0].length

    // Find matching </xacro:macro> accounting for nesting
    let depth = 1
    let searchPos = bodyStart
    let endIdx = -1
    while (depth > 0 && searchPos < xml.length) {
      const nextOpen = xml.indexOf('<xacro:macro', searchPos)
      const nextClose = xml.indexOf('</xacro:macro>', searchPos)

      if (nextClose === -1) break

      if (nextOpen !== -1 && nextOpen < nextClose) {
        depth++
        searchPos = nextOpen + 12
      } else {
        depth--
        if (depth === 0) {
          endIdx = nextClose + '</xacro:macro>'.length
        }
        searchPos = nextClose + '</xacro:macro>'.length
      }
    }

    if (endIdx === -1) continue

    const body = xml.slice(bodyStart, endIdx - '</xacro:macro>'.length)

    // Parse params
    const params = parseParams(paramsStr)

    ctx.macros[macroName] = { name: macroName, params, body }
    macrosToRemove.push({ start: startIdx, end: endIdx })
  }

  // Remove macro definitions from XML (in reverse order to preserve indices)
  macrosToRemove.sort((a, b) => b.start - a.start)
  for (const { start, end } of macrosToRemove) {
    xml = xml.slice(0, start) + xml.slice(end)
  }

  return xml
}

function parseParams(paramsStr: string): XacroMacro['params'] {
  if (!paramsStr.trim()) return []
  return paramsStr.trim().split(/\s+/).map(p => {
    const isDoubleBlock = p.startsWith('**')
    const isBlock = !isDoubleBlock && p.startsWith('*')
    const cleanName = p.replace(/^\*{1,2}/, '')

    // Check for default value: name:=value
    const defaultMatch = cleanName.match(/^(\w+):=(.*)$/)
    if (defaultMatch) {
      return { name: defaultMatch[1], defaultValue: defaultMatch[2], isBlock, isDoubleBlock }
    }
    return { name: cleanName, isBlock, isDoubleBlock }
  })
}

// ── Pass 4: Includes ─────────────────────────────────────────────────────────

async function processIncludes(xml: string, ctx: XacroContext): Promise<string> {
  if (!ctx.fileLoader) return xml

  // <xacro:include filename="other.xacro"/>
  const includeRe = /<xacro:include\s+filename="([^"]+)"\s*\/>/g
  const matches = [...xml.matchAll(includeRe)]

  for (const match of matches.reverse()) {
    let filename = match[1]

    // Handle $(find package) — strip to just the filename
    filename = filename.replace(/\$\(find\s+\w+\)\/?/g, '')

    // Resolve relative path
    if (ctx.basePath && !filename.startsWith('/')) {
      filename = ctx.basePath + '/' + filename
    }

    try {
      let included = await ctx.fileLoader(filename)
      // Process the included file recursively
      const subCtx: XacroContext = {
        ...ctx,
        basePath: filename.replace(/[\\/][^\\/]+$/, ''),
      }
      included = await processXacroString(included, subCtx)
      // Merge properties and macros from included file
      Object.assign(ctx.properties, subCtx.properties)
      Object.assign(ctx.macros, subCtx.macros)

      // Strip the robot root tag from included content if present
      included = included.replace(/<\?xml[^?]*\?>\s*/g, '')
      included = included.replace(/<robot[^>]*>/g, '')
      included = included.replace(/<\/robot>\s*/g, '')

      xml = xml.slice(0, match.index!) + included + xml.slice(match.index! + match[0].length)
    } catch (e) {
      console.warn(`[xacro] Failed to include ${filename}:`, e)
      xml = xml.slice(0, match.index!) + `<!-- xacro include failed: ${filename} -->` + xml.slice(match.index! + match[0].length)
    }
  }

  return xml
}

// ── Pass 5: Macro expansion ──────────────────────────────────────────────────

function expandMacros(xml: string, ctx: XacroContext): string {
  for (const [macroName, macro] of Object.entries(ctx.macros)) {
    // Match self-closing: <xacro:macroName param1="val1" ... />
    const selfCloseRe = new RegExp(
      `<xacro:${escapeRegex(macroName)}((?:\\s+[\\w:]+="[^"]*")*)\\s*/>`,
      'g',
    )
    xml = xml.replace(selfCloseRe, (_match, attrsStr) => {
      return expandMacroCall(macro, attrsStr, '', ctx)
    })

    // Match with body: <xacro:macroName ...>...</xacro:macroName>
    const withBodyRe = new RegExp(
      `<xacro:${escapeRegex(macroName)}((?:\\s+[\\w:]+="[^"]*")*)\\s*>([\\s\\S]*?)</xacro:${escapeRegex(macroName)}>`,
      'g',
    )
    xml = xml.replace(withBodyRe, (_match, attrsStr, body) => {
      return expandMacroCall(macro, attrsStr, body, ctx)
    })
  }

  return xml
}

function expandMacroCall(macro: XacroMacro, attrsStr: string, body: string, ctx: XacroContext): string {
  // Parse attributes from the call
  const attrs: Record<string, string> = {}
  const attrRe = /([\w:]+)="([^"]*)"/g
  let attrMatch
  while ((attrMatch = attrRe.exec(attrsStr)) !== null) {
    attrs[attrMatch[1]] = attrMatch[2]
  }

  // Build local scope with macro parameters
  const localVars: Record<string, string> = { ...ctx.properties }

  // Extract block elements from the body (for * and ** params)
  const bodyElements = extractBlockElements(body)

  let blockIdx = 0
  for (const param of macro.params) {
    if (param.isBlock || param.isDoubleBlock) {
      // Block parameter — use positional block from body
      if (blockIdx < bodyElements.length) {
        if (param.isDoubleBlock) {
          // ** — strip the root tag, keep only children
          const inner = bodyElements[blockIdx].replace(/^<[^>]+>/, '').replace(/<\/[^>]+>$/, '')
          localVars[param.name] = inner.trim()
        } else {
          localVars[param.name] = bodyElements[blockIdx]
        }
        blockIdx++
      }
    } else if (param.name in attrs) {
      localVars[param.name] = substituteExpressionsInString(attrs[param.name], ctx)
    } else if (param.defaultValue !== undefined) {
      let def = param.defaultValue
      if (def === '^') {
        // Inherit from outer scope
        def = ctx.properties[param.name] ?? ''
      } else if (def.startsWith('^|')) {
        def = ctx.properties[param.name] ?? def.slice(2)
      }
      localVars[param.name] = substituteExpressionsInString(def, ctx)
    }
  }

  // Expand the macro body with local variables
  let expanded = macro.body

  // Substitute ${...} expressions with local scope
  const localCtx: XacroContext = { ...ctx, properties: localVars }
  expanded = substituteExpressions(expanded, localCtx)
  expanded = processInsertBlocks(expanded, localCtx)

  return expanded
}

function extractBlockElements(body: string): string[] {
  const elements: string[] = []
  const tagRe = /<(\w+)[\s>]/g
  let match
  let searchPos = 0

  while ((match = tagRe.exec(body.slice(searchPos))) !== null) {
    const tagName = match[1]
    const tagStart = searchPos + match.index

    // Find matching close tag
    const selfClose = body.indexOf('/>', tagStart)
    const closeTag = body.indexOf(`</${tagName}>`, tagStart)

    if (selfClose !== -1 && (closeTag === -1 || selfClose < closeTag) &&
      body.slice(tagStart, selfClose + 2).indexOf('>') > selfClose - tagStart - 2) {
      // Self-closing
      elements.push(body.slice(tagStart, selfClose + 2))
      searchPos = selfClose + 2
    } else if (closeTag !== -1) {
      elements.push(body.slice(tagStart, closeTag + `</${tagName}>`.length))
      searchPos = closeTag + `</${tagName}>`.length
    } else {
      searchPos = tagStart + match[0].length
    }

    tagRe.lastIndex = 0 // reset for next slice
  }

  return elements
}

// ── Pass 6: Conditionals ─────────────────────────────────────────────────────

function processConditionals(xml: string, ctx: XacroContext): string {
  let maxIter = 50
  while (maxIter-- > 0) {
    const ifMatch = xml.match(/<xacro:if\s+value="([^"]*)">([\s\S]*?)<\/xacro:if>/)
    const unlessMatch = xml.match(/<xacro:unless\s+value="([^"]*)">([\s\S]*?)<\/xacro:unless>/)

    if (!ifMatch && !unlessMatch) break

    if (ifMatch) {
      const condStr = substituteExpressionsInString(ifMatch[1], ctx)
      const condVal = isTruthy(condStr)
      xml = xml.replace(ifMatch[0], condVal ? ifMatch[2] : '')
    }

    if (unlessMatch) {
      const condStr = substituteExpressionsInString(unlessMatch[1], ctx)
      const condVal = isTruthy(condStr)
      xml = xml.replace(unlessMatch[0], condVal ? '' : unlessMatch[2])
    }
  }

  return xml
}

function isTruthy(s: string): boolean {
  const v = s.trim().toLowerCase()
  if (v === '' || v === '0' || v === 'false' || v === 'none') return false
  try {
    const n = parseFloat(v)
    if (n === 0) return false
  } catch { /* not a number */ }
  return true
}

// ── Pass 7: Expression substitution ──────────────────────────────────────────

function substituteExpressions(xml: string, ctx: XacroContext): string {
  // Replace ${...} expressions
  return xml.replace(/\$\{([^}]+)\}/g, (_match, expr) => {
    return evalExpression(expr, ctx.properties)
  })
}

function substituteExpressionsInString(s: string, ctx: XacroContext): string {
  // Also handle $(arg name) substitutions
  s = s.replace(/\$\(arg\s+(\w+)\)/g, (_match, name) => {
    return ctx.args[name] ?? ctx.properties[name] ?? ''
  })
  return s.replace(/\$\{([^}]+)\}/g, (_match, expr) => {
    return evalExpression(expr, ctx.properties)
  })
}

// ── Pass 8: Insert blocks ────────────────────────────────────────────────────

function processInsertBlocks(xml: string, ctx: XacroContext): string {
  return xml.replace(/<xacro:insert_block\s+name="([^"]+)"\s*\/>/g, (_match, name) => {
    return ctx.properties[name] ?? `<!-- insert_block: ${name} not found -->`
  })
}

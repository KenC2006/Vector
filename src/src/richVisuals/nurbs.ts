/**
 * NURBS-based geometry generation for Fusion-quality component visuals.
 *
 * Uses Three.js NURBSSurface to define mathematically smooth surfaces
 * from control points, then tessellates to BufferGeometry at high resolution.
 *
 * Key utilities:
 * - nurbsFilletBox: rectangular body with smooth filleted edges (like Fusion fillet)
 * - nurbsRevolved: NURBS profile revolved around Y axis
 * - nurbsCylinder: smooth cylinder with filleted top/bottom edges
 * - nurbsSurface: low-level surface patch from control point grid
 */
import * as THREE from 'three'
import { NURBSSurface } from 'three/addons/curves/NURBSSurface.js'
import { ParametricGeometry } from 'three/addons/geometries/ParametricGeometry.js'

// Default tessellation resolution
const DEFAULT_SLICES = 32
const DEFAULT_STACKS = 32

// ── Low-level NURBS surface → BufferGeometry ─────────────────────────────────

/**
 * Create a BufferGeometry from a NURBSSurface by evaluating it on a grid.
 */
export function nurbsSurfaceToGeometry(
  surface: NURBSSurface,
  slices = DEFAULT_SLICES,
  stacks = DEFAULT_STACKS,
): THREE.BufferGeometry {
  const geom = new ParametricGeometry(
    (u: number, v: number, target: THREE.Vector3) => {
      surface.getPoint(u, v, target)
    },
    slices,
    stacks,
  )
  geom.computeVertexNormals()
  return geom
}

// ── Knot vector helpers ──────────────────────────────────────────────────────

/** Uniform clamped knot vector for given degree and control point count. */
function clampedKnots(degree: number, cpCount: number): number[] {
  const knots: number[] = []
  const n = cpCount + degree + 1
  for (let i = 0; i < n; i++) {
    if (i <= degree) knots.push(0)
    else if (i >= n - degree - 1) knots.push(1)
    else knots.push((i - degree) / (cpCount - degree))
  }
  return knots
}

// ── NURBS Filleted Box ───────────────────────────────────────────────────────

/**
 * A rectangular box with smooth NURBS-filleted edges.
 * This is the equivalent of Fusion 360's "box + fillet" operation.
 *
 * Creates 6 surface patches (one per face) with smooth blending at edges.
 * For simplicity, we build it as a set of filleted cross-section profiles
 * swept along the depth axis.
 *
 * @param w - width (X)
 * @param h - height (Y)
 * @param d - depth (Z)
 * @param fillet - fillet radius for edges
 * @param resolution - tessellation resolution per face
 */
export function nurbsFilletBox(
  w: number, h: number, d: number,
  fillet = 0,
  resolution = 24,
): THREE.BufferGeometry {
  const r = Math.min(fillet, w / 3, h / 3, d / 3)

  if (r < 0.0001) {
    // No fillet — just a box
    return new THREE.BoxGeometry(w, h, d, 2, 2, 2)
  }

  // Build as cross-section profile swept along Z
  // Profile: rounded rectangle in XY plane
  // We use degree 2 NURBS to get exact circular arcs at corners

  const hw = w / 2, hh = h / 2, hd = d / 2

  // Generate a rounded rectangle cross-section as a NURBS curve
  // Then sweep it along Z with fillets at the Z edges too
  // For a full implementation, we'd need 12 edge fillet patches + 6 face patches
  // Simplified approach: use ExtrudeGeometry with a NURBS-quality rounded rect

  const segments = resolution
  const shape = new THREE.Shape()

  // Build rounded rect with many segments per corner for smoothness
  const cornerSegs = Math.max(8, Math.floor(resolution / 4))

  shape.moveTo(-hw + r, -hh)
  shape.lineTo(hw - r, -hh)
  // Bottom-right corner
  for (let i = 1; i <= cornerSegs; i++) {
    const a = Math.PI / 2 * (i / cornerSegs)
    shape.lineTo(hw - r + r * Math.sin(a), -hh + r - r * Math.cos(a))
  }
  shape.lineTo(hw, hh - r)
  // Top-right corner
  for (let i = 1; i <= cornerSegs; i++) {
    const a = Math.PI / 2 * (i / cornerSegs)
    shape.lineTo(hw - r + r * Math.cos(a), hh - r + r * Math.sin(a))
  }
  shape.lineTo(-hw + r, hh)
  // Top-left corner
  for (let i = 1; i <= cornerSegs; i++) {
    const a = Math.PI / 2 * (i / cornerSegs)
    shape.lineTo(-hw + r - r * Math.sin(a), hh - r + r * Math.cos(a))
  }
  shape.lineTo(-hw, -hh + r)
  // Bottom-left corner
  for (let i = 1; i <= cornerSegs; i++) {
    const a = Math.PI / 2 * (i / cornerSegs)
    shape.lineTo(-hw + r - r * Math.cos(a), -hh + r - r * Math.sin(a))
  }

  const geom = new THREE.ExtrudeGeometry(shape, {
    depth: d,
    bevelEnabled: true,
    bevelThickness: r,
    bevelSize: r,
    bevelSegments: cornerSegs,
    curveSegments: cornerSegs,
  })
  geom.translate(0, 0, -hd)
  geom.computeVertexNormals()

  return geom
}

// ── NURBS Revolved Profile ───────────────────────────────────────────────────

/**
 * Revolve a 2D NURBS profile curve around the Y axis.
 * The profile is defined as control points in the XY plane (x=radius, y=height).
 * Each control point is [radius, height, weight].
 *
 * This is the equivalent of Fusion's "Revolve" operation.
 */
export function nurbsRevolved(
  profilePoints: Array<[number, number, number?]>, // [radius, height, weight?]
  degree = 3,
  segments = 64,
): THREE.BufferGeometry {
  // Convert profile to NURBS curve control points
  const cpCount = profilePoints.length
  const knots = clampedKnots(degree, cpCount)

  // For revolution, we create a NURBS surface where:
  // - u parameter sweeps around the revolution (0 to 2π)
  // - v parameter goes along the profile
  // We need 9 control points per ring for a degree-2 rational circle

  const circleCP = 9 // 9 CPs for degree-2 circle
  const circleDegree = 2
  const circleKnots = [0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1]
  const w1 = Math.SQRT1_2 // weight for intermediate points (cos 45°)

  // Build control point grid: circleCP x profilePoints
  const controlPoints: THREE.Vector4[][] = []

  for (let i = 0; i < circleCP; i++) {
    controlPoints[i] = []
    // Angle for this circle control point
    const angles = [0, 0, Math.PI / 2, Math.PI / 2, Math.PI, Math.PI, 3 * Math.PI / 2, 3 * Math.PI / 2, 2 * Math.PI]
    const weights = [1, w1, 1, w1, 1, w1, 1, w1, 1]
    const a = angles[i]
    const cw = weights[i]

    for (let j = 0; j < cpCount; j++) {
      const [radius, height, pw] = profilePoints[j]
      const totalW = (pw ?? 1) * cw
      // Control point in homogeneous coordinates
      // x = radius * cos(a), y = height, z = radius * sin(a)
      controlPoints[i][j] = new THREE.Vector4(
        radius * Math.cos(a) * totalW,
        height * totalW,
        radius * Math.sin(a) * totalW,
        totalW,
      )
    }
  }

  const surface = new NURBSSurface(
    circleDegree,           // degree in u (around circle)
    Math.min(degree, cpCount - 1),  // degree in v (along profile)
    circleKnots,            // knots for circle
    knots,                  // knots for profile
    controlPoints,
  )

  return nurbsSurfaceToGeometry(surface, segments, Math.max(cpCount * 4, 16))
}

// ── NURBS Cylinder with Fillets ──────────────────────────────────────────────

/**
 * A cylinder with smooth filleted top and bottom edges.
 * Equivalent to Fusion's "cylinder + fillet edges".
 */
export function nurbsCylinder(
  radius: number,
  height: number,
  fillet = 0,
  segments = 64,
): THREE.BufferGeometry {
  const r = Math.min(fillet, radius / 2, height / 4)
  const hh = height / 2

  if (r < 0.0001) {
    // No fillet
    return nurbsRevolved([
      [0, -hh],
      [radius, -hh],
      [radius, hh],
      [0, hh],
    ], 1, segments)
  }

  // Profile with fillet arcs at top and bottom
  const profile: Array<[number, number, number?]> = [
    [0, -hh, 1],                           // center bottom
    [radius - r, -hh, 1],                  // bottom flat
    [radius, -hh, Math.SQRT1_2],           // fillet corner (weighted for arc)
    [radius, -hh + r, 1],                  // fillet end
    [radius, hh - r, 1],                   // straight side
    [radius, hh, Math.SQRT1_2],            // top fillet corner
    [radius - r, hh, 1],                   // top fillet end
    [0, hh, 1],                            // center top
  ]

  return nurbsRevolved(profile, 2, segments)
}

// ── NURBS Disc / Flange ──────────────────────────────────────────────────────

/**
 * A flat disc with filleted edges.
 */
export function nurbsDisc(
  radius: number,
  thickness: number,
  fillet = 0,
  segments = 64,
): THREE.BufferGeometry {
  return nurbsCylinder(radius, thickness, fillet || thickness * 0.2, segments)
}

// ── NURBS Dome ───────────────────────────────────────────────────────────────

/**
 * A hemisphere dome (half sphere) on top of a cylinder.
 */
export function nurbsDome(
  radius: number,
  domeHeight: number,
  segments = 48,
): THREE.BufferGeometry {
  const w = Math.SQRT1_2
  const profile: Array<[number, number, number?]> = [
    [0, 0, 1],                              // center bottom
    [radius, 0, 1],                         // base edge
    [radius, domeHeight, w],                // dome curve control
    [0, domeHeight, 1],                     // dome peak
  ]
  return nurbsRevolved(profile, 2, segments)
}

// ── NURBS Torus Section ──────────────────────────────────────────────────────

/**
 * A torus (donut) ring using NURBS. Produces smoother results than
 * Three.js TorusGeometry at the same vertex count.
 */
export function nurbsTorus(
  majorR: number,
  minorR: number,
  segments = 48,
  tubeSegments = 24,
): THREE.BufferGeometry {
  // Torus profile: circle at offset majorR
  const w = Math.SQRT1_2
  const profile: Array<[number, number, number?]> = [
    [majorR + minorR, 0, 1],
    [majorR + minorR, minorR, w],
    [majorR, minorR, 1],
    [majorR - minorR, minorR, w],
    [majorR - minorR, 0, 1],
    [majorR - minorR, -minorR, w],
    [majorR, -minorR, 1],
    [majorR + minorR, -minorR, w],
    [majorR + minorR, 0, 1],
  ]

  return nurbsRevolved(profile, 2, segments)
}

// ── Servo-Specific NURBS Parts ───────────────────────────────────────────────

/**
 * NURBS servo horn — smooth disc with lip edge and center bore.
 */
export function nurbsServoHorn(
  radius: number,
  thickness: number,
  boreRadius: number,
  segments = 48,
): THREE.BufferGeometry {
  const lipR = radius * 1.08
  const lipH = thickness * 0.3
  const hh = thickness / 2

  const profile: Array<[number, number, number?]> = [
    [boreRadius, -hh, 1],                   // bore inner bottom
    [radius * 0.9, -hh, 1],                 // bottom flat
    [radius, -hh, Math.SQRT1_2],            // bottom edge fillet
    [radius, -hh + thickness * 0.2, 1],     // side
    [lipR, -hh + thickness * 0.5, 1],       // lip flare out
    [lipR, hh - lipH, 1],                   // lip top
    [radius * 0.95, hh - lipH * 0.3, 1],   // lip return
    [radius * 0.85, hh, 1],                 // top surface
    [boreRadius * 1.5, hh, 1],              // center rise
    [boreRadius, hh - thickness * 0.1, 1],  // bore inner top
    [boreRadius, -hh, 1],                   // back to start
  ]

  return nurbsRevolved(profile, 2, segments)
}

/**
 * NURBS motor housing — smooth revolved body with cap and shaft.
 * Single surface patch for the entire motor.
 */
export function nurbsMotorHousing(
  bodyR: number,
  bodyH: number,
  shaftR: number,
  shaftH: number,
  capR?: number,
  capH?: number,
  segments = 64,
): THREE.BufferGeometry {
  const cr = capR ?? bodyR * 0.88
  const ch = capH ?? bodyH * 0.08
  const hh = bodyH / 2
  const fillet = bodyR * 0.03

  const profile: Array<[number, number, number?]> = [
    // Center bottom to cap
    [0, -hh - ch, 1],
    [cr, -hh - ch, 1],
    [cr, -hh, Math.SQRT1_2],               // cap-to-body fillet
    // Main body
    [bodyR, -hh + fillet, 1],
    [bodyR, hh - fillet, 1],
    // Top face transition
    [bodyR, hh, Math.SQRT1_2],              // body-to-top fillet
    [bodyR - fillet, hh, 1],
    // Top face to shaft
    [shaftR + fillet, hh, 1],
    [shaftR, hh, Math.SQRT1_2],             // shaft fillet
    // Shaft
    [shaftR, hh + shaftH * 0.9, 1],
    // Shaft tip chamfer
    [shaftR * 0.7, hh + shaftH, 1],
    [0, hh + shaftH, 1],
  ]

  return nurbsRevolved(profile, 2, segments)
}

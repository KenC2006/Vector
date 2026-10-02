/**
 * Catalog category colours (RGBA 0-1), used by the assembly editor for part
 * swatches and plain-box stand-ins.
 *
 * (This file used to hold the URDF primitive silhouettes the old placement
 * engine emitted per part; nothing consumes primitive descriptors any more:
 * catalog parts render through componentVisualResolver and the designer
 * compiler writes their collision boxes from the catalog bbox.)
 */
export const CATEGORY_COLORS: Record<string, [number, number, number, number]> = {
  actuators:      [0.90, 0.49, 0.13, 1],  // orange
  motors:         [0.91, 0.30, 0.24, 1],  // red-orange
  sensors:        [0.20, 0.60, 0.86, 1],  // blue
  compute:        [0.18, 0.80, 0.44, 1],  // green
  power:          [0.95, 0.77, 0.06, 1],  // yellow
  structural:     [0.66, 0.70, 0.72, 1],  // silver-grey
  transmission:   [0.56, 0.27, 0.68, 1],  // purple
  end_effectors:  [0.10, 0.74, 0.61, 1],  // teal
  mobility:       [0.20, 0.29, 0.37, 1],  // dark slate
}

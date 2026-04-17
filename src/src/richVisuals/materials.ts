/**
 * PBR Material cache for rich component visuals.
 *
 * 8 canonical material types with lazy caching and category-tint support.
 * All materials are MeshStandardMaterial so the existing highlight system
 * (emissive.setHex) continues to work.
 */
import * as THREE from 'three'

// ── Material definitions ─────────────────────────────────────────────────────

export interface MaterialDef {
  roughness: number
  metalness: number
  baseColor: number  // default color (can be overridden by tint)
}

export const MATERIAL_DEFS: Record<string, MaterialDef> = {
  anodized_aluminum: { roughness: 0.35, metalness: 0.85, baseColor: 0x8899aa },
  brushed_steel:     { roughness: 0.45, metalness: 0.90, baseColor: 0x888899 },
  matte_plastic:     { roughness: 0.70, metalness: 0.05, baseColor: 0x222222 },
  glossy_plastic:    { roughness: 0.25, metalness: 0.08, baseColor: 0x444444 },
  pcb_green:         { roughness: 0.60, metalness: 0.10, baseColor: 0x1a5c1a },
  rubber_black:      { roughness: 0.95, metalness: 0.02, baseColor: 0x111111 },
  copper_trace:      { roughness: 0.40, metalness: 0.75, baseColor: 0xb87333 },
  dark_chrome:       { roughness: 0.20, metalness: 0.95, baseColor: 0x333344 },
}

// ── Cache ────────────────────────────────────────────────────────────────────

const _cache = new Map<string, THREE.MeshStandardMaterial>()

function cacheKey(materialId: string, color?: number): string {
  return color != null ? `${materialId}_${color.toString(16)}` : materialId
}

/**
 * Get or create a PBR material.
 * @param id - one of the MATERIAL_DEFS keys
 * @param tintColor - optional color override (hex number like 0xff0000)
 */
export function getMaterial(id: string, tintColor?: number): THREE.MeshStandardMaterial {
  const key = cacheKey(id, tintColor)
  let mat = _cache.get(key)
  if (mat) return mat

  const def = MATERIAL_DEFS[id]
  if (!def) {
    // fallback
    mat = new THREE.MeshStandardMaterial({ color: tintColor ?? 0x888888, roughness: 0.5, metalness: 0.3 })
    _cache.set(key, mat)
    return mat
  }

  mat = new THREE.MeshStandardMaterial({
    color: tintColor ?? def.baseColor,
    roughness: def.roughness,
    metalness: def.metalness,
  })
  _cache.set(key, mat)
  return mat
}

/**
 * Get a tinted variant of a material — blends the tint with the base color.
 * Useful for category-coloring metal parts (e.g., orange-anodized servo body).
 */
export function getTintedMaterial(id: string, tintR: number, tintG: number, tintB: number, strength = 0.4): THREE.MeshStandardMaterial {
  const def = MATERIAL_DEFS[id]
  if (!def) return getMaterial(id)

  const base = new THREE.Color(def.baseColor)
  const tint = new THREE.Color(tintR, tintG, tintB)
  const blended = base.lerp(tint, strength)
  const hex = blended.getHex()
  return getMaterial(id, hex)
}

// ── Per-component realistic color map ────────────────────────────────────────
// Maps component ID → [r, g, b] tint (0–1 range) and primary material type.
// Colors reference real-world products (SG90, Dynamixel, Raspberry Pi, etc.)

export interface ComponentColorDef {
  material: string      // MATERIAL_DEFS key
  tint: [number, number, number]  // RGB 0–1
  strength?: number     // tint blend strength (default 0.4)
}

/** Convert a hex number (0xRRGGBB) to [r, g, b] in 0–1 range. */
function hexToRgb(hex: number): [number, number, number] {
  return [((hex >> 16) & 0xff) / 255, ((hex >> 8) & 0xff) / 255, (hex & 0xff) / 255]
}

const COMPONENT_COLORS: Record<string, ComponentColorDef> = {
  // ── Actuators ──
  actuator_servo_micro:              { material: 'glossy_plastic', tint: hexToRgb(0x1E6FBA), strength: 0.6 },   // SG90 blue
  actuator_servo_standard:           { material: 'matte_plastic',  tint: hexToRgb(0x2a3550), strength: 0.55 },  // MG996R dark navy
  actuator_servo_high_torque:        { material: 'matte_plastic',  tint: hexToRgb(0x303848), strength: 0.55 },  // Dynamixel dark steel-blue
  actuator_servo_heavy_duty:         { material: 'matte_plastic',  tint: hexToRgb(0x282838), strength: 0.55 },  // Dynamixel PRO deep
  actuator_bldc_small:               { material: 'anodized_aluminum', tint: hexToRgb(0xa0a0a0), strength: 0.3 }, // T-Motor silver
  actuator_bldc_large:               { material: 'anodized_aluminum', tint: hexToRgb(0x888888), strength: 0.3 }, // T-Motor dark silver
  actuator_stepper_nema17:           { material: 'matte_plastic',  tint: hexToRgb(0x1a1a1a), strength: 0.3 },   // NEMA17 black
  actuator_stepper_nema23:           { material: 'matte_plastic',  tint: hexToRgb(0x1a1a1a), strength: 0.3 },   // NEMA23 black
  actuator_linear_small:             { material: 'brushed_steel',  tint: hexToRgb(0xc0c0c0), strength: 0.2 },   // Actuonix silver
  actuator_linear_heavy:             { material: 'brushed_steel',  tint: hexToRgb(0xa0a0a0), strength: 0.2 },   // Industrial silver
  actuator_micro_linear_servo:       { material: 'matte_plastic',  tint: hexToRgb(0x1a1a1a), strength: 0.3 },   // Black housing
  actuator_continuous_rotation_servo: { material: 'matte_plastic', tint: hexToRgb(0x1a2a4a), strength: 0.4 },   // Parallax dark blue
  actuator_high_speed_mini_servo:    { material: 'glossy_plastic', tint: hexToRgb(0x2244aa), strength: 0.5 },   // Savox blue

  // ── Motors ──
  motor_dc_small_130:                { material: 'brushed_steel',  tint: hexToRgb(0xc0c0c0), strength: 0.2 },   // Silver can
  motor_dc_medium_540:               { material: 'brushed_steel',  tint: hexToRgb(0xaaaaaa), strength: 0.2 },   // Silver can
  motor_dc_large_775:                { material: 'brushed_steel',  tint: hexToRgb(0xb0a080), strength: 0.3 },   // Gold-ish can
  motor_gear_small_n20:              { material: 'anodized_aluminum', tint: hexToRgb(0xc8a830), strength: 0.4 }, // Brass gearbox
  motor_gear_medium_37mm:            { material: 'brushed_steel',  tint: hexToRgb(0x888888), strength: 0.2 },   // Silver body
  motor_gear_heavy_50mm:             { material: 'brushed_steel',  tint: hexToRgb(0x777777), strength: 0.2 },   // Silver body
  motor_coreless_dc:                 { material: 'brushed_steel',  tint: hexToRgb(0xc0c0c0), strength: 0.2 },   // Silver cylinder
  motor_worm_gear:                   { material: 'brushed_steel',  tint: hexToRgb(0xb0a080), strength: 0.3 },   // Brass worm
  motor_hub_80mm:                    { material: 'matte_plastic',  tint: hexToRgb(0x2a2a2a), strength: 0.3 },   // Black disc
  motor_hub_120mm:                   { material: 'matte_plastic',  tint: hexToRgb(0x2a2a2a), strength: 0.3 },   // Black disc
  motor_harmonic_drive_compact:      { material: 'matte_plastic',  tint: hexToRgb(0x222222), strength: 0.3 },   // Dark precision housing
  motor_harmonic_drive_large:        { material: 'matte_plastic',  tint: hexToRgb(0x222222), strength: 0.3 },   // Dark precision housing
  motor_pancake_dc:                  { material: 'brushed_steel',  tint: hexToRgb(0xc0c0c0), strength: 0.2 },   // Flat silver
  motor_brushless_inrunner_micro:    { material: 'anodized_aluminum', tint: hexToRgb(0xa8a8a8), strength: 0.2 },
  motor_brushless_inrunner_medium:   { material: 'anodized_aluminum', tint: hexToRgb(0xa8a8a8), strength: 0.2 },

  // ── Sensors ──
  sensor_depth_camera_small:         { material: 'glossy_plastic', tint: hexToRgb(0x607090), strength: 0.5 },   // RealSense blue-silver
  sensor_depth_camera_wide:          { material: 'glossy_plastic', tint: hexToRgb(0x2a2a40), strength: 0.5 },   // Orbbec dark blue
  sensor_lidar_2d:                   { material: 'matte_plastic',  tint: hexToRgb(0x205040), strength: 0.5 },   // RPLiDAR dark teal
  sensor_lidar_3d:                   { material: 'matte_plastic',  tint: hexToRgb(0x1a3050), strength: 0.55 },  // Ouster dark blue
  sensor_imu_6dof:                   { material: 'pcb_green',      tint: hexToRgb(0x7030a0), strength: 0.6 },   // Purple PCB
  sensor_imu_9dof:                   { material: 'pcb_green',      tint: hexToRgb(0x7030a0), strength: 0.6 },   // Purple PCB
  sensor_ultrasonic:                 { material: 'anodized_aluminum', tint: hexToRgb(0xcccccc), strength: 0.2 }, // HC-SR04 silver
  sensor_tof:                        { material: 'glossy_plastic', tint: hexToRgb(0x1a1a1a), strength: 0.3 },   // VL53L0X black
  sensor_force_torque_6axis:         { material: 'anodized_aluminum', tint: hexToRgb(0x999999), strength: 0.2 }, // ATI silver
  sensor_joint_encoder_absolute:     { material: 'matte_plastic',  tint: hexToRgb(0x1a1a1a), strength: 0.3 },   // Black disc
  sensor_thermal_ir_camera:          { material: 'matte_plastic',  tint: hexToRgb(0x1a1a1a), strength: 0.3 },   // FLIR black
  sensor_contact_bumper:             { material: 'matte_plastic',  tint: hexToRgb(0x881111), strength: 0.5 },   // Red bumper
  sensor_limit_switch:               { material: 'matte_plastic',  tint: hexToRgb(0x1a1a1a), strength: 0.3 },   // Black body
  sensor_rotary_potentiometer:       { material: 'pcb_green',      tint: hexToRgb(0x1a5c1a), strength: 0.3 },   // Green breakout
  sensor_load_cell:                  { material: 'anodized_aluminum', tint: hexToRgb(0xaaaaaa), strength: 0.2 }, // Aluminum bar
  sensor_current:                    { material: 'pcb_green',      tint: hexToRgb(0x1a5c1a), strength: 0.3 },   // Green PCB
  sensor_voltage_divider:            { material: 'pcb_green',      tint: hexToRgb(0x1a5c1a), strength: 0.3 },   // Green PCB
  sensor_color_light:                { material: 'pcb_green',      tint: hexToRgb(0x6b2d8b), strength: 0.5 },   // Purple Adafruit
  sensor_barometric_pressure:        { material: 'pcb_green',      tint: hexToRgb(0x6b2d8b), strength: 0.5 },   // Purple BMP280

  // ── Compute ──
  compute_mcu_small:                 { material: 'pcb_green',      tint: hexToRgb(0x006699), strength: 0.5 },   // Arduino blue
  compute_sbc_small:                 { material: 'pcb_green',      tint: hexToRgb(0x008844), strength: 0.6 },   // Raspberry Pi green
  compute_sbc_gpu:                   { material: 'pcb_green',      tint: hexToRgb(0x1a3a1a), strength: 0.4 },   // Jetson dark green
  compute_motor_driver_dual:         { material: 'pcb_green',      tint: hexToRgb(0x6a1a1a), strength: 0.5 },   // L298N red PCB
  compute_foc_controller:            { material: 'pcb_green',      tint: hexToRgb(0x1a2a5a), strength: 0.5 },   // ODrive blue-black
  compute_fpga_dev_board:            { material: 'pcb_green',      tint: hexToRgb(0x1a1a2a), strength: 0.5 },   // Dark PCB
  compute_can_transceiver:           { material: 'pcb_green',      tint: hexToRgb(0x006699), strength: 0.5 },   // Blue module
  compute_usb_hub:                   { material: 'pcb_green',      tint: hexToRgb(0x1a1a1a), strength: 0.4 },   // Black PCB
  compute_wireless_24ghz:            { material: 'pcb_green',      tint: hexToRgb(0x1a4a1a), strength: 0.4 },   // Green nRF24L01
  compute_lora_radio:                { material: 'pcb_green',      tint: hexToRgb(0x1a4a2a), strength: 0.4 },   // Green SX1276
  compute_gps_gnss:                  { material: 'pcb_green',      tint: hexToRgb(0x1a2a4a), strength: 0.5 },   // Blue u-blox

  // ── Power ──
  power_lipo_3s_2200:                { material: 'glossy_plastic', tint: hexToRgb(0x1a4080), strength: 0.65 },  // Blue shrinkwrap
  power_lipo_4s_5000:                { material: 'glossy_plastic', tint: hexToRgb(0xc8a830), strength: 0.5 },   // Yellow Tattu
  power_lipo_6s_10000:               { material: 'glossy_plastic', tint: hexToRgb(0x8a1a1a), strength: 0.5 },   // Red/black large
  power_buck_converter_5v:           { material: 'pcb_green',      tint: hexToRgb(0x006699), strength: 0.5 },   // Blue LM2596
  power_buck_converter_12v:          { material: 'pcb_green',      tint: hexToRgb(0x006699), strength: 0.5 },   // Blue module
  power_distribution_unit:           { material: 'pcb_green',      tint: hexToRgb(0x1a1a1a), strength: 0.4 },   // Black PDB
  power_18650_cell_holder_1s:        { material: 'matte_plastic',  tint: hexToRgb(0x2a4a2a), strength: 0.4 },   // Black+green cell
  power_18650_4s2p_pack:             { material: 'glossy_plastic', tint: hexToRgb(0x1a2a5a), strength: 0.5 },   // Blue wrap
  power_supercapacitor_module:       { material: 'glossy_plastic', tint: hexToRgb(0x1a2a4a), strength: 0.5 },   // Dark blue
  power_solar_panel_small:           { material: 'glossy_plastic', tint: hexToRgb(0x0a1a3a), strength: 0.6 },   // Navy PV cells
  power_usbc_pd_trigger:             { material: 'pcb_green',      tint: hexToRgb(0x1a4a1a), strength: 0.4 },   // Green PCB
  power_estop_switch:                { material: 'glossy_plastic', tint: hexToRgb(0xcc1111), strength: 0.6 },   // Red mushroom

  // ── Structural ──
  structural_extrusion_2020:         { material: 'anodized_aluminum', tint: hexToRgb(0xb0b8c0), strength: 0.35 }, // Silver V-slot (slight cool)
  structural_extrusion_4040:         { material: 'anodized_aluminum', tint: hexToRgb(0xb0b8c0), strength: 0.35 }, // Silver V-slot (slight cool)
  structural_cf_tube_round:          { material: 'dark_chrome',    tint: hexToRgb(0x1a1a1a), strength: 0.3 },   // Carbon fiber
  structural_cf_tube_square:         { material: 'dark_chrome',    tint: hexToRgb(0x1a1a1a), strength: 0.3 },   // Carbon fiber
  structural_bracket_l:              { material: 'anodized_aluminum', tint: hexToRgb(0xb0b0b0), strength: 0.2 },
  structural_bracket_u:              { material: 'anodized_aluminum', tint: hexToRgb(0xb0b0b0), strength: 0.2 },
  structural_joint_plate:            { material: 'anodized_aluminum', tint: hexToRgb(0xb0b0b0), strength: 0.2 },
  structural_baseplate:              { material: 'anodized_aluminum', tint: hexToRgb(0xd0d0d0), strength: 0.2 }, // Light silver
  structural_ibeam_steel_small:      { material: 'brushed_steel',  tint: hexToRgb(0x888899), strength: 0.2 },
  structural_ibeam_aluminum_small:   { material: 'anodized_aluminum', tint: hexToRgb(0xb0b0b0), strength: 0.2 },
  structural_sheet_aluminum_1mm:     { material: 'anodized_aluminum', tint: hexToRgb(0xcccccc), strength: 0.2 },
  structural_sheet_steel_1_5mm:      { material: 'brushed_steel',  tint: hexToRgb(0x888899), strength: 0.2 },
  structural_standoff_m3:            { material: 'anodized_aluminum', tint: hexToRgb(0xb0a060), strength: 0.4 }, // Brass
  structural_standoff_m4:            { material: 'anodized_aluminum', tint: hexToRgb(0xb0a060), strength: 0.4 }, // Brass
  structural_linear_rail_mgn12:      { material: 'brushed_steel',  tint: hexToRgb(0xaaaaaa), strength: 0.2 },
  structural_linear_rail_carriage_mgn12h: { material: 'brushed_steel', tint: hexToRgb(0xaaaaaa), strength: 0.2 },
  structural_din_rail_35mm:          { material: 'brushed_steel',  tint: hexToRgb(0xaaaaaa), strength: 0.2 },

  // ── Transmission ──
  transmission_timing_belt_gt2:      { material: 'rubber_black',   tint: hexToRgb(0x222222), strength: 0.2 },
  transmission_leadscrew_8mm:        { material: 'brushed_steel',  tint: hexToRgb(0x999999), strength: 0.2 },
  transmission_ballscrew_12mm:       { material: 'brushed_steel',  tint: hexToRgb(0xaaaaaa), strength: 0.2 },
  transmission_bearing_deep_groove:  { material: 'dark_chrome',    tint: hexToRgb(0xbbbbcc), strength: 0.2 },
  transmission_bearing_large:        { material: 'dark_chrome',    tint: hexToRgb(0xbbbbcc), strength: 0.2 },
  transmission_bushing_flanged:      { material: 'anodized_aluminum', tint: hexToRgb(0xd4af37), strength: 0.5 }, // Bronze
  transmission_planetary_gearbox:    { material: 'anodized_aluminum', tint: hexToRgb(0xc0c0c0), strength: 0.2 },
  transmission_spur_gear_pair:       { material: 'brushed_steel',  tint: hexToRgb(0xa0a0a0), strength: 0.2 },
  transmission_bevel_gear_pair:      { material: 'brushed_steel',  tint: hexToRgb(0xa0a0a0), strength: 0.2 },
  transmission_worm_gear_set:        { material: 'brushed_steel',  tint: hexToRgb(0xb0a080), strength: 0.3 },  // Bronze worm
  transmission_chain_sprocket_set:   { material: 'brushed_steel',  tint: hexToRgb(0x555566), strength: 0.3 },
  transmission_universal_joint:      { material: 'brushed_steel',  tint: hexToRgb(0x999999), strength: 0.2 },
  transmission_flexible_coupling_jaw: { material: 'anodized_aluminum', tint: hexToRgb(0x999999), strength: 0.2 },
  transmission_rigid_shaft_coupling: { material: 'anodized_aluminum', tint: hexToRgb(0x999999), strength: 0.2 },
  transmission_rack_pinion_set:      { material: 'brushed_steel',  tint: hexToRgb(0xa0a0a0), strength: 0.2 },
  transmission_crossed_roller_bearing: { material: 'brushed_steel', tint: hexToRgb(0x999999), strength: 0.2 },
  transmission_slewing_ring_bearing: { material: 'brushed_steel',  tint: hexToRgb(0x707070), strength: 0.2 },

  // ── End Effectors ──
  effector_parallel_gripper_small:   { material: 'anodized_aluminum', tint: hexToRgb(0x444455), strength: 0.3 },
  effector_parallel_gripper_large:   { material: 'anodized_aluminum', tint: hexToRgb(0x444455), strength: 0.3 },
  effector_3finger_adaptive:         { material: 'matte_plastic',  tint: hexToRgb(0x333333), strength: 0.3 },
  effector_suction_cup:              { material: 'rubber_black',   tint: hexToRgb(0xcc6600), strength: 0.5 },   // Orange silicone
  effector_magnetic_tool:            { material: 'brushed_steel',  tint: hexToRgb(0x999999), strength: 0.2 },
  effector_tool_changer:             { material: 'anodized_aluminum', tint: hexToRgb(0xaaaaaa), strength: 0.2 },
  effector_soft_gripper:             { material: 'glossy_plastic', tint: hexToRgb(0x4466aa), strength: 0.5 },   // Blue silicone
  effector_vacuum_pad_array:         { material: 'rubber_black',   tint: hexToRgb(0x1a1a1a), strength: 0.2 },
  effector_welding_torch_holder:     { material: 'anodized_aluminum', tint: hexToRgb(0xc0c0c0), strength: 0.2 },
  effector_pen_marker_holder:        { material: 'anodized_aluminum', tint: hexToRgb(0x3a3a3a), strength: 0.3 },
  effector_screwdriver_holder:       { material: 'anodized_aluminum', tint: hexToRgb(0x3a3a3a), strength: 0.3 },

  // ── Mobility ──
  mobility_wheel_driven:             { material: 'rubber_black',   tint: hexToRgb(0x1a1a1a), strength: 0.2 },
  mobility_caster_wheel:             { material: 'brushed_steel',  tint: hexToRgb(0x888888), strength: 0.2 },
  mobility_mecanum_wheel:            { material: 'rubber_black',   tint: hexToRgb(0x2a2a2a), strength: 0.2 },
  mobility_omni_wheel:               { material: 'rubber_black',   tint: hexToRgb(0x333333), strength: 0.2 },
  mobility_track_tread_system:       { material: 'rubber_black',   tint: hexToRgb(0x1a1a1a), strength: 0.2 },
  mobility_swerve_drive_module:      { material: 'anodized_aluminum', tint: hexToRgb(0x555555), strength: 0.3 },
  mobility_ball_transfer_unit:       { material: 'brushed_steel',  tint: hexToRgb(0xc0c0c0), strength: 0.2 },
  mobility_rubber_foot_pad:          { material: 'rubber_black',   tint: hexToRgb(0x111111), strength: 0.2 },
}

// Category-level fallback colors (used when exact component ID not found)
const CATEGORY_FALLBACK_COLORS: Record<string, ComponentColorDef> = {
  actuator:     { material: 'matte_plastic',     tint: [0.10, 0.10, 0.10], strength: 0.3 },
  motor:        { material: 'brushed_steel',     tint: [0.75, 0.75, 0.75], strength: 0.2 },
  sensor:       { material: 'matte_plastic',     tint: [0.13, 0.13, 0.13], strength: 0.3 },
  compute:      { material: 'pcb_green',         tint: [0.10, 0.36, 0.10], strength: 0.4 },
  power:        { material: 'glossy_plastic',    tint: [0.10, 0.23, 0.42], strength: 0.5 },
  structural:   { material: 'anodized_aluminum', tint: [0.75, 0.75, 0.75], strength: 0.2 },
  transmission: { material: 'brushed_steel',     tint: [0.63, 0.63, 0.70], strength: 0.2 },
  effector:     { material: 'anodized_aluminum', tint: [0.33, 0.40, 0.47], strength: 0.3 },
  mobility:     { material: 'rubber_black',      tint: [0.10, 0.10, 0.10], strength: 0.2 },
}

// Body-part token fallbacks (used when compId is category-less, e.g., "wheel_driven",
// "front_left_wheel", "drive_tire"). Any token in the compId that matches a key here
// yields the corresponding realistic color. Order matters in TOKEN_PRIORITY below.
const TOKEN_FALLBACK_COLORS: Record<string, ComponentColorDef> = {
  wheel:   { material: 'rubber_black',     tint: [0.10, 0.10, 0.10], strength: 0.2 },
  tire:    { material: 'rubber_black',     tint: [0.07, 0.07, 0.07], strength: 0.2 },
  tread:   { material: 'rubber_black',     tint: [0.10, 0.10, 0.10], strength: 0.2 },
  track:   { material: 'rubber_black',     tint: [0.10, 0.10, 0.10], strength: 0.2 },
  rubber:  { material: 'rubber_black',     tint: [0.07, 0.07, 0.07], strength: 0.2 },
  caster:  { material: 'brushed_steel',    tint: [0.53, 0.53, 0.53], strength: 0.2 },
  pad:     { material: 'rubber_black',     tint: [0.07, 0.07, 0.07], strength: 0.2 },
  bearing: { material: 'brushed_steel',    tint: [0.75, 0.75, 0.75], strength: 0.2 },
  shaft:   { material: 'brushed_steel',    tint: [0.70, 0.70, 0.70], strength: 0.2 },
  servo:   { material: 'matte_plastic',    tint: [0.10, 0.10, 0.10], strength: 0.3 },
  stepper: { material: 'matte_plastic',    tint: [0.10, 0.10, 0.10], strength: 0.3 },
  bldc:    { material: 'anodized_aluminum',tint: [0.63, 0.63, 0.63], strength: 0.3 },
  bracket: { material: 'anodized_aluminum',tint: [0.75, 0.75, 0.75], strength: 0.2 },
  frame:   { material: 'anodized_aluminum',tint: [0.55, 0.55, 0.55], strength: 0.2 },
  plate:   { material: 'anodized_aluminum',tint: [0.65, 0.65, 0.65], strength: 0.2 },
  battery: { material: 'glossy_plastic',   tint: [0.10, 0.23, 0.42], strength: 0.5 },
  pcb:     { material: 'pcb_green',        tint: [0.10, 0.36, 0.10], strength: 0.4 },
  gripper: { material: 'anodized_aluminum',tint: [0.33, 0.40, 0.47], strength: 0.3 },
  belt:    { material: 'rubber_black',     tint: [0.13, 0.13, 0.13], strength: 0.2 },
  pulley:  { material: 'anodized_aluminum',tint: [0.60, 0.60, 0.60], strength: 0.2 },
  gear:    { material: 'brushed_steel',    tint: [0.70, 0.70, 0.70], strength: 0.2 },
}

// When multiple tokens match, earlier entries win. Surface-defining parts (rubber,
// wheel) outrank structural hosts (bracket) so "wheel_bracket" reads as rubber.
const TOKEN_PRIORITY = [
  'tire', 'wheel', 'tread', 'track', 'rubber', 'belt', 'pad',
  'caster', 'bearing', 'shaft', 'pulley', 'gear',
  'servo', 'stepper', 'bldc',
  'pcb', 'battery',
  'gripper', 'bracket', 'plate', 'frame',
]

/**
 * Look up realistic color for a component. Falls back through:
 *   1. exact match, 2. category-prefixed match, 3. token match,
 *   4. category prefix fallback, 5. generic neutral.
 */
export function getComponentColor(componentId: string): ComponentColorDef {
  // 1. Exact match
  const exact = COMPONENT_COLORS[componentId]
  if (exact) return exact

  // 2. Category-prefixed match: "wheel_driven" → try "mobility_wheel_driven", etc.
  // Handles AI-generated IDs that drop the category prefix.
  for (const cat of Object.keys(CATEGORY_FALLBACK_COLORS)) {
    const prefixed = COMPONENT_COLORS[`${cat}_${componentId}`]
    if (prefixed) return prefixed
  }

  // 3. Token fallback: scan compId tokens for known body-part nouns.
  // This is what catches "wheel_driven", "front_left_wheel", "drive_tire".
  const tokens = componentId.split('_')
  const tokenSet = new Set(tokens)
  for (const priorityTok of TOKEN_PRIORITY) {
    if (tokenSet.has(priorityTok)) {
      return TOKEN_FALLBACK_COLORS[priorityTok]
    }
  }

  // 4. Category prefix fallback (first token)
  const catFallback = CATEGORY_FALLBACK_COLORS[tokens[0]]
  if (catFallback) return catFallback

  // 5. Generic fallback
  return { material: 'anodized_aluminum', tint: [0.53, 0.57, 0.60], strength: 0.3 }
}

/**
 * Dispose all cached materials (call on app shutdown or full scene reset).
 */
export function disposeAllMaterials(): void {
  for (const mat of _cache.values()) {
    mat.dispose()
  }
  _cache.clear()
}

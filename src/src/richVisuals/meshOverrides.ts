/**
 * Mapping from component preset IDs to real 3D mesh files (STEP/STP).
 *
 * When a mesh override is available, the rich visual system loads the
 * manufacturer STEP file via OpenCascade WASM instead of generating
 * parametric geometry. This gives Fusion 360-quality visuals.
 */

export const MESH_OVERRIDES: Record<string, string> = {
  // ── Actuators: Servos ──
  'actuator_servo_micro': 'servo_small.step',
  'actuator_servo_standard': 'servo_standard.step',
  'actuator_servo_high_torque': 'servo_high_torque.stp',
  'actuator_servo_heavy_duty': 'servo_high_torque.stp',
  'actuator_continuous_rotation_servo': 'servo_standard.step',
  'actuator_high_speed_mini_servo': 'servo_small.step',

  // ── Actuators: BLDC ──
  'actuator_bldc_small': 'bldc_outrunner.step',
  'actuator_bldc_large': 'bldc_outrunner.step',

  // ── Actuators: Steppers ──
  'actuator_stepper_nema17': 'stepper_nema17.step',
  'actuator_stepper_nema23': 'stepper_nema23.stp',

  // ── Actuators: Linear ──
  'actuator_linear_small': 'linear_actuator.stp',
  'actuator_linear_heavy': 'linear_actuator.stp',
  'actuator_micro_linear_servo': 'linear_actuator.stp',

  // ── Motors ──
  'motor_dc_small_130': 'motor_dc_small.stp',
  'motor_dc_medium_540': 'motor_dc_medium.stp',
  'motor_dc_large_775': 'motor_dc_large.step',
  'motor_gear_small_n20': 'motor_gear_small.stp',
  'motor_gear_medium_37mm': 'motor_gear_medium.step',
  'motor_gear_heavy_50mm': 'motor_gear_heavy.step',
  'motor_coreless_dc': 'motor_coreless.step',
  'motor_worm_gear': 'motor_worm_gear.step',
  'motor_hub_80mm': 'motor_hub.stp',
  'motor_hub_120mm': 'motor_hub.stp',
  'motor_harmonic_drive_compact': 'motor_harmonic_drive.stp',
  'motor_harmonic_drive_large': 'motor_harmonic_drive.stp',

  // ── Sensors ──
  'sensor_depth_camera_small': 'depth_camera.step',
  'sensor_depth_camera_wide': 'depth_camera.step',
  'sensor_lidar_2d': 'lidar_2d.step',
  'sensor_lidar_3d': 'lidar_3d.stp',
  'sensor_imu_6dof': 'imu_board.step',
  'sensor_imu_9dof': 'imu_board.step',
  'sensor_ultrasonic': 'sensor_ultrasonic.stp',
  'sensor_tof': 'sensor_tof.stp',
  'sensor_force_torque_6axis': 'sensor_force_torque.step',
  'sensor_joint_encoder_absolute': 'sensor_encoder.step',
  'sensor_thermal_camera': 'sensor_thermal_camera.step',
  'sensor_contact_switch': 'sensor_limit_switch.step',
  'sensor_limit_switch': 'sensor_limit_switch.step',
  'sensor_load_cell': 'sensor_load_cell.step',

  // ── Compute ──
  'compute_mcu_small': 'compute_mcu.step',
  'compute_sbc_small': 'sbc_small.step',
  'compute_sbc_gpu': 'sbc_gpu.stp',
  'compute_motor_driver_dual': 'compute_motor_driver.stp',
  'compute_foc_controller': 'compute_foc_controller.step',
  'compute_fpga_dev_board': 'compute_fpga.stp',
  'compute_can_transceiver': 'compute_can_transceiver.step',
  'compute_gps_gnss': 'compute_gps.step',

  // ── Power ──
  'power_lipo_3s_2200': 'battery_lipo.step',
  'power_lipo_4s_5000': 'battery_lipo.step',
  'power_lipo_6s_10000': 'battery_lipo.step',
  'power_buck_converter_5v': 'power_buck_converter.step',
  'power_buck_converter_12v': 'power_buck_converter.step',
  'power_distribution_unit': 'power_pdu.step',
  'power_18650_cell_holder': 'power_cell_holder.step',
  'power_18650_4s2p_battery': 'power_cell_holder.step',
  'power_supercapacitor': 'power_supercapacitor.stp',
  'power_solar_panel_small': 'power_solar_panel.step',
  'power_usb_c_pd_trigger': 'power_usb_c_pd.step',
  'power_estop_switch': 'power_estop.stp',

  // ── Structural ──
  'structural_extrusion_2020': 'extrusion_2020.step',
  'structural_extrusion_4040': 'extrusion_4040.step',
  'structural_bracket_l': 'structural_bracket_l.step',
  'structural_bracket_u': 'structural_bracket_u.step',
  'structural_servo_coupler_disc': 'servo_coupler_disc.step',
  'structural_hip_housing_2dof': 'hip_housing_2dof.stp',
  'structural_hex_standoff_m3': 'structural_standoff.step',
  'structural_hex_standoff_m4': 'structural_standoff.step',
  'structural_shaft_collar': 'structural_shaft_collar.step',
  'structural_linear_rail_mgn12': 'structural_linear_rail.step',
  'structural_pillow_block': 'structural_pillow_block.step',
  'structural_din_rail_35mm': 'structural_din_rail.step',

  // ── Transmission ──
  'transmission_timing_belt_gt2': 'pulley_gt2.step',
  'transmission_bearing_deep_groove': 'bearing_small.stp',
  'transmission_bearing_large': 'bearing_large.step',
  'transmission_planetary_gearbox': 'transmission_planetary_gearbox.step',
  'transmission_leadscrew_8mm': 'transmission_leadscrew.step',
  'transmission_flexible_coupling': 'transmission_coupling.step',
  'transmission_flexible_coupling_jaw': 'transmission_coupling.step',
  'transmission_rigid_shaft_coupling': 'transmission_rigid_coupling.step',
  'transmission_rack_pinion_set': 'transmission_rack_pinion.step',

  // ── End Effectors ──
  'effector_parallel_gripper_small': 'gripper_parallel.step',
  'effector_parallel_gripper_large': 'gripper_parallel.step',
  'effector_3finger_adaptive': 'effector_3finger.step',
  'effector_suction_cup': 'effector_suction_cup.step',
  'effector_pen_marker_holder': 'effector_pen_holder.step',

  // ── Mobility ──
  'mobility_wheel_driven': 'wheel_driven.step',
  'mobility_caster_wheel': 'wheel_caster.step',
  'mobility_mecanum_wheel': 'wheel_mecanum.stp',
  'mobility_omni_wheel': 'wheel_omni.step',
  'mobility_track_tread_system': 'mobility_track.step',
  'mobility_rubber_foot_pad': 'mobility_rubber_foot.step',
}

export type MeshVisualScalePolicy = 'none' | 'uniform' | 'per-axis'
export type MeshVisualUnits = 'm' | 'mm' | 'auto'

export interface MeshVisualMetadata {
  file: string
  rotation?: [number, number, number]
  units: MeshVisualUnits
  scalePolicy: MeshVisualScalePolicy
  targetFrame: 'urdf-z-up'
  shaftOverlay?: { shaft_length_mm: number; shaft_radius_mm: number }
  blacklisted?: boolean
}

/**
 * Get the mesh override URL for a component ID, or null if none.
 * Prefers pre-converted GLB files over raw STEP for fast loading.
 */
export function getMeshOverrideUrl(componentId: string): string | null {
  const metadata = getMeshVisualMetadata(componentId)
  if (!metadata) return null
  // Prefer GLB (pre-converted at build time) — falls back to STEP at runtime
  const baseName = metadata.file.replace(/\.(step|stp)$/i, '')
  return `/meshes/glb/${baseName}.glb`
}

/**
 * Get the raw STEP file URL (fallback when GLB is missing).
 */
export function getStepFallbackUrl(componentId: string): string | null {
  const metadata = getMeshVisualMetadata(componentId)
  if (!metadata) return null
  return `/meshes/components/${metadata.file}`
}

/**
 * Check if a component ID has a real mesh override available.
 */
export function hasMeshOverride(componentId: string): boolean {
  return !!getMeshVisualMetadata(componentId)
}

// Component IDs whose meshes are too large/slow to load at runtime - use
// parametric instead. Includes no GLB available, very large GLBs, or known bad
// source geometry.
export const SLOW_MESH_BLACKLIST = new Set([
  // No GLB (STEP files blacklisted from conversion: >25MB)
  'compute_sbc_gpu',                   // sbc_gpu.stp - 71MB STEP
  'mobility_track_tread_system',       // mobility_track.step - 50MB STEP
  // GLB still >10MB (too slow to fetch+parse at runtime)
  'actuator_bldc_small',               // bldc_outrunner.glb - 15MB
  'actuator_bldc_large',               // bldc_outrunner.glb - 15MB
  'motor_hub_80mm',                    // motor_hub.glb - 11MB
  'motor_hub_120mm',                   // motor_hub.glb - 11MB
  'compute_foc_controller',            // compute_foc_controller.glb - 11MB
  // Wrong STEP file or broken geometry
  'transmission_rack_pinion_set',      // STEP is industrial-scale (2.4m), not robotics
  'motor_harmonic_drive_compact',      // STEP is a disc servo, not a harmonic drive
  'motor_harmonic_drive_large',        // same mislabeled STEP
])

/**
 * Per-component Euler rotations (XYZ order, radians) applied to the loaded mesh
 * BEFORE per-axis scaling in applyMeshToLink. Use when the STEP/GLB axes don't
 * match the preset's bounding_box_mm convention — without this, per-axis scaling
 * stretches a mismatched axis into a sliver.
 *
 * Determine the rotation by comparing preset bbox (X,Y,Z) to GLB raw bbox:
 * pick the axis swap that makes GLB extents line up with preset extents.
 */
export const ROTATION_OVERRIDES: Record<string, [number, number, number]> = {
  // Disc thickness on Y in STEP, preset puts it on Z. Rotate X by 90°: Y→Z.
  // preset [32,32,8] vs GLB [31.96,8,31.98]
  'structural_servo_coupler_disc': [Math.PI / 2, 0, 0],

  // Bearing axial direction on X in STEP, preset puts it on Z. Rotate Y by 90°: X→Z.
  // preset [22,22,7] vs GLB [7,22,22]
  'transmission_bearing_deep_groove': [0, Math.PI / 2, 0],
  // preset [52,52,15] vs GLB [15,53,53]
  'transmission_bearing_large': [0, Math.PI / 2, 0],

  // SBC thickness on Y in STEP (20mm), preset puts it on Z (17mm). Rotate X by 90°.
  // preset [85,56,17] vs GLB [89,20,58]
  'compute_sbc_small': [Math.PI / 2, 0, 0],

  // Motor driver board thickness on Y in STEP (28mm), preset puts it on Z (15mm).
  // Rotate X by 90°. Without it: Y stretches 1.54x and Z squishes to 0.35x — a
  // wide-but-thin sliver. With it: the board renders as a normal PCB.
  // preset [43,43,15] vs GLB [43,28,43]
  'compute_motor_driver_dual': [Math.PI / 2, 0, 0],

  // Wheel axle on Y in STEP, preset puts it on Z. Rotate X by 90°: Y→Z.
  // preset [100,100,48] vs GLB [100,50,110]
  'mobility_mecanum_wheel': [Math.PI / 2, 0, 0],

  // Lidar housing height on Y in STEP, preset puts it on Z. Rotate X by 90°: Y→Z.
  // preset [70,70,41] vs GLB [75.64, 41.3, 75.7] → rotated [75.64, 75.7, 41.3].
  'sensor_lidar_2d': [Math.PI / 2, 0, 0],

  // Driven wheel GLB has hub face at +Z but wheels are placed along drivetrain
  // local +Z (outboard), so the hub ends up facing outboard instead of inboard.
  // Rx(π) flips the hub from +Z to −Z so it faces the motor correctly.
  // Axle symmetry is preserved; collision geometry (cylinder) is unaffected.
  'mobility_wheel_driven': [Math.PI, 0, 0],

  // LiPo GLB (detailed pack with XT60 + balance lead) has length on Z (~103mm)
  // and a near-square ~24×23mm cross-section. All three lipo presets put length
  // on X with a rectangular 34×24 / 42×30 / 65×42 cross-section. Rotation
  // [π/2, 0, π/2] permutes (X,Y,Z) → (Z,X,Y) so GLB Z becomes preset X (length),
  // GLB X becomes preset Y, GLB Y becomes preset Z. Per-axis scale then widens
  // the square GLB cross-section to the preset's rectangular footprint —
  // 1.4-1.75× Y stretch on 3s/4s, up to 2.7× on the 6s pack; acceptable next to
  // the much bigger servos/extrusions in the same scene.
  // preset [105,34,24] / [137,42,30] / [165,65,42] vs GLB [24,23,103]
  'power_lipo_3s_2200': [Math.PI / 2, 0, Math.PI / 2],
  'power_lipo_4s_5000': [Math.PI / 2, 0, Math.PI / 2],
  'power_lipo_6s_10000': [Math.PI / 2, 0, Math.PI / 2],

  // Authored GLB axes already match the preset envelope exactly.
  // preset [66,58,40] vs GLB [66,58,40]
  'structural_hip_housing_2dof': [0, 0, 0],

  // Servo GLBs are authored with body length on Y; preset frame puts length on X
  // (with shaft along +Z for axis-z servos). Without these, per-axis scaling
  // squishes/stretches the mesh and the rendered body's mount face lands far
  // from the URDF link origin → split servos appear detached from their parent.
  // preset [40,20,37] vs GLB [28.5,46.5,41] — Rz(π/2) maps GLB Y→preset X.
  'actuator_servo_standard': [0, 0, Math.PI / 2],
  'actuator_continuous_rotation_servo': [0, 0, Math.PI / 2],
  // preset [54,42,54] vs GLB [33.5,58.5,54.3] — Rz(π/2): GLB Y→X, X→-Y.
  'actuator_servo_heavy_duty': [0, 0, Math.PI / 2],
  // preset [46.5,36,34] vs GLB [33.5,58.5,54.3] — cycle XYZ→YZX so shaft (Y)
  // becomes preset Z and body length (Y) becomes preset X.
  'actuator_servo_high_torque': [Math.PI / 2, 0, Math.PI / 2],
  // preset [32,16,30] vs GLB [29,46.5,38.3] — Rz(π/2) maps GLB Y→preset X.
  'actuator_high_speed_mini_servo': [0, 0, Math.PI / 2],
  // preset [23,12.2,29] vs GLB [29,46.5,38.3] — cycle XYZ→ZXY so shaft lands
  // along +Z and body length on X.
  'actuator_servo_micro': [0, Math.PI / 2, Math.PI / 2],
}

/**
 * Components whose GLB models the entire servo (body + shaft) as a single
 * primitive — the per-axis scaling stretches the full mesh to fill bbox.z, so
 * there is no visible shaft column above the body. For face-mount mates, the
 * authored body_top connector wants a real body-vs-shaft distinction so the
 * coupler bore can hide the shaft.
 *
 * When a component has an entry here, applyMeshToLink:
 *   1. Scales the GLB Z to (bbox.z - shaft_length_mm) instead of bbox.z, so the
 *      body GLB occupies only the lower (bbox.z - shaft_length) of the link.
 *   2. Adds a procedural cylinder above the body — radius = shaft_radius_mm,
 *      height = shaft_length_mm, axis along URDF +Z — to fill the upper
 *      shaft_length region of the bbox.
 *
 * The total visible envelope (body + shaft) still fills the bbox; per-axis
 * scaling just splits its allocation between two meshes. The procedural shaft
 * is a smooth metallic cylinder, not a splined output horn — at robot-scale
 * zoom this is invisible; at extreme close-up it reads as a generic shaft.
 *
 * Only needed for GLBs that don't already split body and shaft as separate
 * primitives. Servo presets whose GLBs DO split (e.g. servo_standard.glb has
 * the shaft as a 19.5×19.5×4 primitive at +Z) don't need an entry here.
 */
export const SHAFT_OVERLAYS: Record<string, { shaft_length_mm: number; shaft_radius_mm: number }> = {
  // servo_high_torque.glb is one continuous primitive — no shaft separation.
  // Both presets that share this GLB get an overlay shaft sized to their
  // shaft_out connector's diameter_mm. Length = 5mm covers a typical Dynamixel
  // XM-class output horn protrusion (a real spline horn would be ~6-8mm).
  'actuator_servo_high_torque': { shaft_length_mm: 5, shaft_radius_mm: 4 },
  'actuator_servo_heavy_duty':  { shaft_length_mm: 5, shaft_radius_mm: 6 },
}

/**
 * Components whose GLB shape is close enough to the preset bbox (after rotation)
 * that uniform scaling preserves the model's proportions. Per-axis would deform
 * a near-correct mesh; uniform keeps it honest at the cost of a small bbox gap.
 *
 * Eligibility: post-rotation extent must be within ~5% of bbox per axis.
 */
export const EXPLICIT_SCALE_POLICY: Record<string, MeshVisualScalePolicy> = {
  // GLB raw [100.2, 49.99, 109.82] → rotX 90° → [100.2, 109.82, 49.99]
  // bbox [100, 100, 48]. Uniform scale 0.952 → divergence 4.6%.
  'mobility_mecanum_wheel': 'uniform',
  // GLB raw [75.64, 41.3, 75.7] → rotX 90° → [75.64, 75.7, 41.3]
  // bbox [70, 70, 41]. Uniform scale 0.957 → divergence 3.5%.
  'sensor_lidar_2d': 'uniform',
}

function scalePolicyForComponent(componentId: string): MeshVisualScalePolicy {
  const explicit = EXPLICIT_SCALE_POLICY[componentId]
  if (explicit) return explicit
  const perAxisBlacklist = ['gripper', 'effector', 'claw', 'suction']
  return perAxisBlacklist.some(k => componentId.includes(k)) ? 'none' : 'per-axis'
}

export const MESH_VISUAL_METADATA: Record<string, MeshVisualMetadata> = Object.fromEntries(
  Object.entries(MESH_OVERRIDES).map(([componentId, file]) => [
    componentId,
    {
      file,
      rotation: ROTATION_OVERRIDES[componentId],
      units: 'auto',
      scalePolicy: scalePolicyForComponent(componentId),
      targetFrame: 'urdf-z-up',
      shaftOverlay: SHAFT_OVERLAYS[componentId],
      blacklisted: SLOW_MESH_BLACKLIST.has(componentId),
    } satisfies MeshVisualMetadata,
  ]),
)

export function getMeshVisualMetadata(componentId: string): MeshVisualMetadata | null {
  return MESH_VISUAL_METADATA[componentId] ?? null
}

export function getShaftOverlay(componentId: string): { shaft_length_mm: number; shaft_radius_mm: number } | null {
  return getMeshVisualMetadata(componentId)?.shaftOverlay ?? null
}

/** Get the per-component rotation override (XYZ Euler radians), or null if none. */
export function getRotationOverride(componentId: string): [number, number, number] | null {
  return getMeshVisualMetadata(componentId)?.rotation ?? null
}


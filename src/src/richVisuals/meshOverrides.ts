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

/**
 * Get the mesh override URL for a component ID, or null if none.
 * Prefers pre-converted GLB files over raw STEP for fast loading.
 */
export function getMeshOverrideUrl(componentId: string): string | null {
  const filename = MESH_OVERRIDES[componentId]
  if (!filename) return null
  // Prefer GLB (pre-converted at build time) — falls back to STEP at runtime
  const baseName = filename.replace(/\.(step|stp)$/i, '')
  return `/meshes/glb/${baseName}.glb`
}

/**
 * Get the raw STEP file URL (fallback when GLB is missing).
 */
export function getStepFallbackUrl(componentId: string): string | null {
  const filename = MESH_OVERRIDES[componentId]
  if (!filename) return null
  return `/meshes/components/${filename}`
}

/**
 * Check if a component ID has a real mesh override available.
 */
export function hasMeshOverride(componentId: string): boolean {
  return componentId in MESH_OVERRIDES
}

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
  // preset [32,32,6] vs GLB [31.96,8,31.98]
  'structural_servo_coupler_disc': [Math.PI / 2, 0, 0],

  // Bearing axial direction on X in STEP, preset puts it on Z. Rotate Y by 90°: X→Z.
  // preset [22,22,7] vs GLB [7,22,22]
  'transmission_bearing_deep_groove': [0, Math.PI / 2, 0],
  // preset [52,52,15] vs GLB [15,53,53]
  'transmission_bearing_large': [0, Math.PI / 2, 0],

  // SBC thickness on Y in STEP (20mm), preset puts it on Z (17mm). Rotate X by 90°.
  // preset [85,56,17] vs GLB [89,20,58]
  'compute_sbc_small': [Math.PI / 2, 0, 0],

  // Wheel axle on Y in STEP, preset puts it on Z. Rotate X by 90°: Y→Z.
  // preset [100,100,48] vs GLB [100,50,110]
  'mobility_mecanum_wheel': [Math.PI / 2, 0, 0],
}

/** Get the per-component rotation override (XYZ Euler radians), or null if none. */
export function getRotationOverride(componentId: string): [number, number, number] | null {
  return ROTATION_OVERRIDES[componentId] ?? null
}


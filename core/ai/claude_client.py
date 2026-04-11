"""
Claude AI client for robot design editing.
Communicates with Claude API to generate robot model edits based on natural language requests.
"""
import json
import os
import re
import sys
import time
from collections import defaultdict

# Lazy import — anthropic may not be installed
_anthropic = None
_anthropic_error = None
try:
    import anthropic
    _anthropic = anthropic
except ImportError as e:
    _anthropic_error = str(e)

# ── Singleton client ──────────────────────────────────────────────────────────
_client = None

def _get_client():
    """Return a shared Anthropic client instance (reuses HTTP connection pool)."""
    global _client
    if _client is None:
        if _anthropic is None:
            raise ImportError(
                f"anthropic package not installed. Run: pip install anthropic\n"
                f"Error: {_anthropic_error}"
            )
        api_key = os.environ.get("ANTHROPIC_API_KEY")
        if not api_key:
            raise ValueError("ANTHROPIC_API_KEY environment variable not set")
        _client = _anthropic.Anthropic(api_key=api_key)
    return _client

# ── Conversation history store (keyed by session_id) ──────────────────────────
# Each entry is a list of {"role": "user"|"assistant", "content": str} dicts.
# Capped to last 20 messages to avoid unbounded token growth.
_conversation_history: dict[str, list] = defaultdict(list)
_MAX_HISTORY_MESSAGES = 20


# Component IDs that have verified GLB meshes available in the UI.
# Must match meshOverrides.ts minus SLOW_MESH_BLACKLIST in richVisuals/index.ts.
_ALLOWED_COMPONENT_IDS = {
    # Actuators
    'actuator_servo_micro', 'actuator_servo_standard', 'actuator_servo_high_torque',
    'actuator_servo_heavy_duty', 'actuator_stepper_nema17', 'actuator_stepper_nema23',
    'actuator_linear_small', 'actuator_linear_heavy', 'actuator_micro_linear_servo',
    'actuator_continuous_rotation_servo', 'actuator_high_speed_mini_servo',
    # Motors
    'motor_dc_small_130', 'motor_dc_medium_540', 'motor_dc_large_775',
    'motor_gear_small_n20', 'motor_gear_medium_37mm', 'motor_gear_heavy_50mm',
    'motor_coreless_dc', 'motor_worm_gear',
    # Sensors
    'sensor_depth_camera_small', 'sensor_depth_camera_wide',
    'sensor_lidar_2d', 'sensor_lidar_3d',
    'sensor_imu_6dof', 'sensor_imu_9dof',
    'sensor_ultrasonic', 'sensor_tof',
    'sensor_force_torque_6axis', 'sensor_joint_encoder_absolute',
    'sensor_limit_switch', 'sensor_load_cell',
    # Compute
    'compute_mcu_small', 'compute_sbc_small',
    'compute_motor_driver_dual', 'compute_fpga_dev_board',
    'compute_can_transceiver', 'compute_gps_gnss',
    # Power
    'power_lipo_3s_2200', 'power_lipo_4s_5000', 'power_lipo_6s_10000',
    'power_buck_converter_5v', 'power_buck_converter_12v',
    'power_distribution_unit', 'power_solar_panel_small', 'power_estop_switch',
    # Structural
    'structural_extrusion_2020', 'structural_extrusion_4040',
    'structural_bracket_l', 'structural_bracket_u',
    'structural_shaft_collar', 'structural_linear_rail_mgn12',
    'structural_din_rail_35mm',
    # Transmission
    'transmission_timing_belt_gt2', 'transmission_leadscrew_8mm',
    'transmission_bearing_deep_groove', 'transmission_bearing_large',
    'transmission_planetary_gearbox',
    'transmission_flexible_coupling_jaw', 'transmission_rigid_shaft_coupling',
    # End Effectors
    'effector_parallel_gripper_small', 'effector_parallel_gripper_large',
    'effector_3finger_adaptive', 'effector_suction_cup', 'effector_pen_marker_holder',
    # Mobility
    'mobility_wheel_driven', 'mobility_caster_wheel',
    'mobility_mecanum_wheel', 'mobility_omni_wheel', 'mobility_rubber_foot_pad',
}


def _build_component_catalog() -> str:
    """Build a compact summary of available preset components for the AI system prompt.
    Only includes components with verified GLB meshes shown in the UI."""
    try:
        from core.presets import list_components, get_all_categories, get_category
        lines = []
        for cat_name in get_all_categories():
            cat = get_category(cat_name)
            label = cat_name.replace("_", " ").title()
            comps = [c for c in cat["components"] if c["id"] in _ALLOWED_COMPONENT_IDS]
            if not comps:
                continue
            items = []
            for c in comps:
                phys = c["physical"]
                mass = phys.get("mass_kg") or phys.get("mass_kg_per_100mm")
                mass_str = f"{mass}kg" if mass and mass >= 1 else f"{round(mass*1000)}g" if mass else "?"
                bb = phys.get("bounding_box_mm", [])
                bb_str = f"{bb[0]}x{bb[1]}x{bb[2]}mm" if len(bb) >= 3 else ""
                shape = phys.get("inertia_primitive", "box")
                # Key spec
                me = c.get("mechanical_electrical", {})
                spec = ""
                if "max_torque_nm" in me: spec = f"{me['max_torque_nm']}Nm"
                elif "holding_torque_nm" in me: spec = f"{me['holding_torque_nm']}Nm"
                elif "max_force_n" in me: spec = f"{me['max_force_n']}N"
                elif "grip_force_n" in me: spec = f"{me['grip_force_n']}N"
                elif "capacity_mah" in me: spec = f"{me['capacity_mah']}mAh"
                elif "fov_h_deg" in me: spec = f"{me['fov_h_deg']}°FOV"
                elif "range_m" in me: spec = f"{me['range_m']}m"
                items.append(f"  - {c['id']}: {c['name']} [{mass_str}, {bb_str}, {shape}]{(' ' + spec) if spec else ''}")
            lines.append(f"\n{label} ({len(comps)}):")
            lines.extend(items)
        return "\n".join(lines)
    except Exception as e:
        return f"(Component catalog unavailable: {e})"

_COMPONENT_CATALOG = None

def _get_component_catalog() -> str:
    global _COMPONENT_CATALOG
    if _COMPONENT_CATALOG is None:
        _COMPONENT_CATALOG = _build_component_catalog()
    return _COMPONENT_CATALOG


def _build_spatial_context(kg_json: dict) -> str:
    """
    Build a spatial context string from the kinematic graph JSON that tells
    Claude the world-frame position and bounding box of every link, plus
    joint origin details. This helps Claude calculate correct offsets.
    """
    try:
        from model.kinematic_graph import KinematicGraph
        kg = KinematicGraph.from_json(kg_json)
        frames = kg.compute_world_frames()

        lines = ["## Spatial Layout (world-frame positions & sizes)"]
        lines.append("Each link's approximate world position and bounding box in meters:")
        lines.append("")

        for link_name in kg.get_links():
            frame = frames.get(link_name, {})
            xyz = frame.get("world_xyz", [0, 0, 0])
            bbox = frame.get("bbox_m")
            bbox_str = f", bbox={bbox[0]:.3f}x{bbox[1]:.3f}x{bbox[2]:.3f}m" if bbox else ""
            lines.append(f"  {link_name}: pos=[{xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}]{bbox_str}")

        # Add joint origin summary
        joints = kg_json.get("joints", [])
        if joints:
            lines.append("")
            lines.append("Joint origins (offset from parent link, in meters):")
            for j in joints:
                oxyz = j.get("origin_xyz", [0, 0, 0])
                lines.append(f"  {j['name']}: {j['parent_link']}->{j['child_link']} offset=[{oxyz[0]:.4f}, {oxyz[1]:.4f}, {oxyz[2]:.4f}]")

        return "\n".join(lines)
    except Exception as e:
        return f"(Spatial context unavailable: {e})"


def _build_mounting_context() -> str:
    """
    Build a mounting rules context from the preset library's mounting_logic
    to help Claude understand how components connect.
    """
    try:
        from core.presets import get_all_categories, get_category
        mounting_map = {}
        for cat_name in get_all_categories():
            cat = get_category(cat_name)
            for c in cat.get("components", []):
                if c["id"] not in _ALLOWED_COMPONENT_IDS:
                    continue
                ml = c.get("mounting_logic", {})
                if ml:
                    entry = f"primary={ml.get('primary','?')}, output={ml.get('output','?')}"
                    if ml.get("shaft_diameter_mm"):
                        entry += f", shaft={ml['shaft_diameter_mm']}mm"
                    if ml.get("bolt_pattern_mm"):
                        entry += f", bolts={ml['bolt_pattern_mm']}mm"
                    mounting_map[c["id"]] = entry

        lines = []
        for cid, info in sorted(mounting_map.items()):
            lines.append(f"  {cid}: {info}")
        return "\n".join(lines)
    except Exception as e:
        return f"(Mounting context unavailable: {e})"


_MOUNTING_CONTEXT = None

def _get_mounting_context() -> str:
    global _MOUNTING_CONTEXT
    if _MOUNTING_CONTEXT is None:
        _MOUNTING_CONTEXT = _build_mounting_context()
    return _MOUNTING_CONTEXT


SYSTEM_PROMPT = r"""You are a robot design assistant for Vector IDE.

CRITICAL: You must return ONLY valid JSON. No English text, no markdown, no code blocks. Just a JSON object.

You receive:
1. The current URDF XML of a robot
2. The kinematic graph (links, joints, masses, geometries)
3. A Robot Structure Summary (link names, joint types, kinematic chain)
4. A Spatial Layout showing world-frame positions and bounding boxes of all links
5. A natural language edit request from the user

You have conversation memory for multi-turn context.

## Component Preset Library

When adding components, use values from this library (never invent masses/dimensions).
Use the component ID as the link name prefix (e.g., "actuator_servo_high_torque_4").

Component rules:
- mass_kg -> <mass value="..."/>
- bounding_box_mm (div 1000) -> geometry size in meters
- inertia_primitive -> geometry type (box/cylinder/sphere)
- Actuators/motors -> joint type="revolute", effort = max_torque_nm
- Everything else -> joint type="fixed"
- Inertia: Box Ixx=m/12*(h^2+d^2), Cylinder Ixx=m/12*(3r^2+h^2), Sphere Ixx=2/5*m*r^2

Available components:
{COMPONENT_CATALOG}

## Mounting Rules

Components connect through compatible mounting interfaces. Use these rules to determine valid connections:

Compatible pairs:
- axial_shaft <-> hub_bore: actuator output shaft into transmission bore (coaxial)
- face_mount <-> face_mount: bolt-circle flanged connection (planar)
- side_rail_mount <-> rail_slot: slide onto extrusion T-slot
- bracket_mount <-> bracket_mount: L/U-bracket with through-holes
- tool_changer_master <-> tool_changer_slave: quick-change plate interface
- press_fit <-> bore: bearing/bushing pressed into housing

Component mounting interfaces:
{MOUNTING_CONTEXT}

## Spatial Placement Rules

CRITICAL: When adding components, you MUST calculate correct joint origin offsets.

1. **Check the Spatial Layout** -- it shows world-frame [x,y,z] position and bounding box of every existing link. Use this to avoid overlaps and calculate correct offsets.

2. **Joint origin = offset from parent link's frame to child link's frame.** To attach a child to the END of a parent:
   - If parent is a vertical link with bbox LxWxH: offset Z by H/2 (half the parent height) + child_H/2 (half the child height)
   - If parent is a horizontal link: offset X or Y by the appropriate half-extents
   - The child's geometry center will be at the joint origin relative to parent

3. **Stacking rule**: For a chain A->B->C, if A is at world [0,0,0] and is 0.1m tall, B's joint origin should be [0,0,0.05] to sit on top. If B is 0.08m tall, C's joint origin should be [0,0,0.08] (relative to B, which is B's full extent since B's frame is at B's center).

4. **Wheels/feet**: Place at Z offsets that put the contact point at Z=0 in world frame. Work backwards: if base is at height H, wheel joints need origin Z = -H.

5. **Side-mounted components** (sensors, cameras): Use X or Y offsets, not just Z. E.g., a camera on the left side: offset Y = parent_width/2 + camera_width/2.

6. **NEVER use origin xyz="0 0 0" for non-root joints** -- this stacks everything at the same point.

## Few-Shot Example (Option C)

CRITICAL: Study this example. You only specify TOPOLOGY -- the backend handles all positioning, rotation, and sizing.

User request: "Build a 2-DOF arm with a gripper on a baseplate"

Correct response (ENTIRE response is this single JSON object, nothing before or after):
{"explanation": "2-DOF arm: base plate on ground, shoulder servo yaw on top, upper arm extrusion horizontal, elbow servo pitch at end, gripper at tip", "assembly": {"base_link": "structural_baseplate_1", "ground_offset": true, "components": [
  {"link_name": "structural_baseplate_1", "component_id": "structural_baseplate", "attach_to": null, "attach_face": null, "joint_type": "fixed", "joint_axis": "z"},
  {"link_name": "actuator_servo_high_torque_1", "component_id": "actuator_servo_high_torque", "attach_to": "structural_baseplate_1", "attach_face": "top", "joint_type": "revolute", "joint_axis": "z"},
  {"link_name": "structural_extrusion_2020_1", "component_id": "structural_extrusion_2020", "attach_to": "actuator_servo_high_torque_1", "attach_face": "top", "joint_type": "fixed", "joint_axis": "z", "length_mm": 200},
  {"link_name": "actuator_servo_standard_1", "component_id": "actuator_servo_standard", "attach_to": "structural_extrusion_2020_1", "attach_face": "front", "joint_type": "revolute", "joint_axis": "y"},
  {"link_name": "effector_parallel_gripper_small_1", "component_id": "effector_parallel_gripper_small", "attach_to": "actuator_servo_standard_1", "attach_face": "front", "joint_type": "fixed", "joint_axis": "z"}
]}, "changes_summary": "5 components, 2 DOF"}

## Key Rules

1. **You only specify topology**: component_id, attach_to, attach_face, joint_type, joint_axis. NO coordinates, NO rpy values.
2. **joint_axis**: Just use "x", "y", or "z". The backend resolves to numeric vectors.
3. **attach_face**: "top", "bottom", "front", "back", "left", "right". The backend computes offsets and rotations automatically.
4. **Extrusions auto-rotate**: When an extrusion attaches to "top", the backend automatically rotates it horizontal. Use attach_face="front" for the next component to put it at the tip.
5. **length_mm**: Optional for extrusions. Default is 100mm. Use 150-300mm for arm segments.
6. **ground_offset**: Set to true in the assembly object so the robot sits on the ground plane.
7. **Arm pattern**: base -> servo(top, revolute z) -> extrusion(top, fixed) -> servo(front, revolute y) -> extrusion(top, fixed) -> gripper(front, fixed)
8. **Think through the kinematic chain before writing JSON**, but put your reasoning INSIDE the "explanation" field, NOT as separate text before the JSON. Your entire response must be a single JSON object.

## Output Format

Return ONLY this JSON structure (no other text). You have THREE options:

Option A -- For modifications to existing URDF (adding/removing/changing parts):
{"explanation": "...", "edits": [{"search": "exact text", "replace": "replacement"}, ...], "changes_summary": "..."}

Option B -- For creating a new robot from scratch or replacing the entire URDF:
{"explanation": "...", "full_urdf": "<?xml version=\"1.0\"?>\n<robot name=\"...\">...</robot>", "changes_summary": "..."}

Option C -- For multi-component assemblies. You specify ONLY topology -- the backend computes all positions and rotations:
{"explanation": "...", "assembly": {"base_link": "root_link_name", "ground_offset": true, "components": [{"link_name": "...", "component_id": "...", "attach_to": "parent_link", "attach_face": "top", "joint_type": "fixed", "joint_axis": "z"}, ...]}, "changes_summary": "..."}

Per-component fields:
- link_name: unique name (use component_id prefix + number, e.g. "actuator_servo_high_torque_1")
- component_id: ID from the component library above
- attach_to: parent link_name (null for root)
- attach_face: "top", "bottom", "front", "back", "left", "right"
- joint_type: "fixed", "revolute", or "prismatic"
- joint_axis: "x", "y", or "z" (the backend resolves to [1,0,0], [0,1,0], or [0,0,1])
- length_mm: (optional, extrusions only) Override default 100mm. Use 150-300mm for arm links.
DO NOT include coordinates, rpy values, or any numbers except length_mm. The backend handles ALL geometry.

CRITICAL: ALWAYS use Option C when the user wants to build, create, design, or make a robot — regardless of how many components. Option C uses the backend compute engine for correct placement. NEVER write raw URDF coordinates yourself.
Use Option A ONLY for small edits to an existing robot (e.g., "change the arm length", "remove the sensor").
Option B is DEPRECATED — do not use it. Use Option C instead.

Edit rules (Option A only):
- "search" must be an EXACT substring of the current URDF (verbatim, including whitespace)
- "replace" is what replaces it
- Edits are applied in order, each on the result of the previous
- Keep edits minimal -- only change what's needed

Rules:
- ALWAYS use component preset values for physical properties
- Maintain valid URDF XML structure
- Use SI units: meters, kilograms, radians
- If adding links, include inertial, visual, and collision elements
- Position robots so the ground contact points (feet, wheels, base) are at Z=0 and the body is ABOVE the ground. The grid plane is at Z=0 -- nothing should be below it.
- For legged robots: set joint origins so the legs are in a natural standing pose at rest (knees slightly bent, not straight). Use negative Z offsets from hip to knee to foot. The body should be at a realistic height above ground.

CRITICAL OUTPUT SIZE RULES:
- NO XML comments in URDF output
- NO collision elements (they will be auto-generated)
- Minimal whitespace -- no blank lines between elements
- Omit optional attributes that use default values
- Keep joint names short but link names MUST use the full component ID prefix (e.g., "actuator_servo_high_torque_1")
- For inertia, use simple diagonal values only (ixy=ixz=iyz=0 can be omitted)

REMINDER: Return ONLY JSON. No English preamble. Start your response with { and end with }.
"""

COMPLETION_SYSTEM_PROMPT = """You are a URDF/XML code completion engine for a robotics IDE.

You receive:
- ===CURSOR_PREFIX===: the last ~100 characters IMMEDIATELY before the cursor. THIS IS WHAT YOU ARE CONTINUING. Your output is concatenated directly after this text.
- ===CONTEXT===: broader surrounding code for reference (before and after cursor).
- ===ROBOT===: (optional) summary of the robot's links, joints, and kinematic chain.

Your job: output the raw XML that comes IMMEDIATELY after ===CURSOR_PREFIX===.

Rules:
- Output raw XML only. No English, no explanations, no markdown, no code fences.
- CRITICAL: Your very first character must be the correct next character after the prefix.
  Examples:
  - Prefix ends with `<link` -> your output starts with ` name="...">`  (SPACE then name)
  - Prefix ends with `<joint` -> your output starts with ` name="...">`  (SPACE then name)
  - Prefix ends with `name="gripper_` -> your output starts with `finger">`  (finish the value)
  - Prefix ends with `</collision>` on a blank line -> output the next element
- Produce VALID XML. The concatenation prefix + your_output must be well-formed.
- Complete up to ~15 lines. Stop at a natural boundary (closing tag, end of element).
- Do NOT repeat code that already exists after the cursor in ===CONTEXT===.
- Use link/joint names from ===ROBOT=== when available.
- Match the indentation style of the surrounding code."""


def _apply_edits(urdf: str, edits: list) -> str:
    """
    Apply a list of search/replace edit operations to a URDF string.

    Each edit is a dict with "search" and "replace" keys. Edits are applied
    sequentially -- each one modifies the URDF for the next.

    If a search string is not found, it tries whitespace-normalized matching
    as a fallback (handles minor indentation differences from the model).

    Raises ValueError if a search string cannot be found at all.
    """
    result = urdf

    for i, edit in enumerate(edits):
        if not isinstance(edit, dict):
            print(f"[ai_edit] Skipping non-dict edit at index {i}", file=sys.stderr)
            continue

        search = edit.get("search", "")
        replace = edit.get("replace", "")

        if not search:
            print(f"[ai_edit] Skipping edit {i} with empty search string", file=sys.stderr)
            continue

        # Try exact match first
        if search in result:
            result = result.replace(search, replace, 1)
            print(f"[ai_edit] Applied edit {i}: exact match ({len(search)}c -> {len(replace)}c)", file=sys.stderr)
            continue

        # Fallback: whitespace-normalized matching
        # Normalize both the search and every possible window of the URDF
        search_normalized = re.sub(r'[ \t]+', ' ', search.strip())
        lines = result.split('\n')

        # Try to find a contiguous block of lines that matches when normalized
        search_line_count = len(search.strip().split('\n'))
        matched = False

        for start in range(len(lines)):
            end = min(start + search_line_count + 2, len(lines))  # +2 for tolerance
            for e in range(start + 1, end + 1):
                candidate = '\n'.join(lines[start:e])
                candidate_normalized = re.sub(r'[ \t]+', ' ', candidate.strip())
                if candidate_normalized == search_normalized:
                    result = result.replace(candidate, replace, 1)
                    print(f"[ai_edit] Applied edit {i}: whitespace-normalized match (lines {start+1}-{e})", file=sys.stderr)
                    matched = True
                    break
            if matched:
                break

        if not matched:
            print(f"[ai_edit] WARNING: Could not find search text for edit {i}: {search[:80]!r}...", file=sys.stderr)

    return result


def _assemble_from_graph(assembly: dict) -> str:
    """
    Given an assembly graph (Option C output from Claude), compute joint origins
    from component bounding boxes and attach_face directions, then generate
    a complete URDF string.
    """
    try:
        from core.presets import get_component
    except ImportError:
        raise ValueError("Preset library not available for assembly")

    components = assembly.get("components", [])
    base_link = assembly.get("base_link", components[0]["link_name"] if components else "base_link")

    # Build a lookup: link_name -> component definition
    comp_lookup = {}
    for comp in components:
        cid = comp["component_id"]
        preset = get_component(cid)
        if not preset:
            print(f"[assembly] WARNING: Unknown component_id '{cid}', using defaults", file=sys.stderr)
            preset = {"id": cid, "physical": {"mass_kg": 0.1, "bounding_box_mm": [50, 50, 50], "inertia_primitive": "box"}}
        comp_lookup[comp["link_name"]] = {**comp, "preset": preset}
        # Debug: log what Claude sent for each component
        rpy = comp.get("attach_rpy", "MISSING")
        face = comp.get("attach_face", "MISSING")
        print(f"[assembly] {comp['link_name']}: component={cid}, face={face}, rpy={rpy}", file=sys.stderr)

    # Compute joint origins based on parent bbox and attach_face
    def _get_bbox_m(preset):
        phys = preset.get("physical", {})
        bb = phys.get("bounding_box_mm")
        if bb and len(bb) >= 3:
            return [b / 1000.0 for b in bb]
        # Handle extrusions: cross_section_mm + default 100mm length
        cs = phys.get("cross_section_mm")
        if cs and len(cs) >= 2:
            return [cs[0] / 1000.0, cs[1] / 1000.0, 0.1]  # 100mm default length
        return [0.05, 0.05, 0.05]

    def _is_elongated(bbox):
        """Check if a component is rod-shaped (e.g., extrusion, arm link).
        Returns True for rods (two short dims, one long), False for plates/cubes."""
        if not bbox or len(bbox) < 3:
            return False
        sorted_dims = sorted(bbox)
        # Rod: longest dim >> both short dims (both short dims are similar)
        # Plate: shortest dim << both long dims (two long dims are similar)
        is_long = sorted_dims[2] > sorted_dims[0] * 2.5
        short_dims_similar = sorted_dims[1] < sorted_dims[0] * 2.0
        return is_long and short_dims_similar

    def _compute_origin_and_rpy(parent_preset, child_preset, attach_face, explicit_rpy=None):
        """Compute joint origin xyz AND rpy based on parent/child bounding boxes and face.

        For elongated children (extrusions) attaching to 'top', auto-rotates them
        to extend horizontally along +X instead of stacking vertically.
        """
        import math
        p_bbox = _get_bbox_m(parent_preset)
        c_bbox = _get_bbox_m(child_preset)

        # Use explicit rpy if provided and non-zero
        if explicit_rpy and any(abs(v) > 0.001 for v in explicit_rpy):
            rpy = explicit_rpy
        else:
            rpy = [0, 0, 0]
            # Auto-rotate: elongated child on "top" face -> pitch 90° to extend along +X
            if attach_face in ("top", "coaxial") and _is_elongated(c_bbox):
                rpy = [0, math.pi/2, 0]  # pitch 90°
                print(f"[assembly] Auto-rotating elongated child to horizontal (pitch 90°)", file=sys.stderr)

        # Half-extents (geometry is always centered at link frame origin)
        px, py, pz = p_bbox[0]/2, p_bbox[1]/2, p_bbox[2]/2
        cx, cy, cz = c_bbox[0]/2, c_bbox[1]/2, c_bbox[2]/2

        child_is_rod = _is_elongated(c_bbox)
        is_rotated = abs(rpy[1] - math.pi/2) < 0.01

        if child_is_rod:
            # Rod geometry is offset in local +Z, so it extends forward from the joint.
            # The joint only needs to clear the rod's cross-section, not half its length.
            if is_rotated:
                cx_eff = cx  # cross-section in rotated X (was originally X)
                cz_eff = cx  # cross-section in Z (was originally X)
            else:
                cx_eff = cx
                cz_eff = cx  # cross-section, not half-length
        elif is_rotated:
            cx_eff, cz_eff = cz, cx  # swap Z and X extents
        else:
            cx_eff, cz_eff = cx, cz

        parent_is_rod = _is_elongated(p_bbox)

        # For rod parents that are rotated (horizontal arms), "front" means the tip
        # The rod extends from joint origin along rotated axis for its full length
        if parent_is_rod:
            rod_tip = p_bbox[2]  # full length (geometry offset means tip is at length from joint)
            p_front = rod_tip + cx_eff
        else:
            p_front = px + cx_eff

        face_offsets = {
            "top":     [0, 0, pz + cz_eff],
            "bottom":  [0, 0, -(pz + cz_eff)],
            "front":   [p_front, 0, 0],
            "back":    [-(p_front), 0, 0],
            "right":   [0, py + cy, 0],
            "left":    [0, -(py + cy), 0],
            "coaxial": [0, 0, pz + cz_eff],
        }
        xyz = face_offsets.get(attach_face, [0, 0, pz + cz_eff])
        return xyz, rpy

    # Generate URDF XML
    import xml.etree.ElementTree as ET
    import math

    robot = ET.Element("robot", name="assembled_robot")

    for comp in components:
        link_name = comp["link_name"]
        preset = comp_lookup[link_name]["preset"]
        phys = preset.get("physical", {})
        bbox_m = _get_bbox_m(preset)

        # Support custom length for extrusion-type components
        custom_length = comp.get("length_mm")
        if custom_length and phys.get("cross_section_mm"):
            bbox_m[2] = custom_length / 1000.0
            # Scale mass proportionally
            base_mass = phys.get("mass_kg_per_100mm", 0.054)
            mass = base_mass * (custom_length / 100.0)
        else:
            mass = phys.get("mass_kg") or phys.get("mass_kg_per_100mm") or 0.1

        shape = phys.get("inertia_primitive", "box")
        is_rod = _is_elongated(bbox_m)

        # For rods, offset visual in local +Z so geometry extends forward from the joint.
        # After the joint's rpy rotation (pitch 90°), local Z becomes world X,
        # so this offset correctly pushes the rod forward along the arm direction.
        local_visual_offset = f"0 0 {bbox_m[2]/2:.4f}" if is_rod else "0 0 0"

        # Link element
        link_el = ET.SubElement(robot, "link", name=link_name)

        # Inertial
        inertial = ET.SubElement(link_el, "inertial")
        ET.SubElement(inertial, "mass", value=f"{mass:.4f}")
        ET.SubElement(inertial, "origin", xyz=local_visual_offset, rpy="0 0 0")
        # Compute inertia from shape
        if shape == "cylinder":
            r = max(bbox_m[0], bbox_m[1]) / 2
            h = bbox_m[2]
            ixx = mass/12 * (3*r*r + h*h)
            izz = mass/2 * r*r
            ET.SubElement(inertial, "inertia", ixx=f"{ixx:.6f}", iyy=f"{ixx:.6f}", izz=f"{izz:.6f}", ixy="0", ixz="0", iyz="0")
        elif shape == "sphere":
            r = max(bbox_m) / 2
            ii = 2/5 * mass * r*r
            ET.SubElement(inertial, "inertia", ixx=f"{ii:.6f}", iyy=f"{ii:.6f}", izz=f"{ii:.6f}", ixy="0", ixz="0", iyz="0")
        else:  # box
            lx, ly, lz = bbox_m
            ixx = mass/12 * (ly*ly + lz*lz)
            iyy = mass/12 * (lx*lx + lz*lz)
            izz = mass/12 * (lx*lx + ly*ly)
            ET.SubElement(inertial, "inertia", ixx=f"{ixx:.6f}", iyy=f"{iyy:.6f}", izz=f"{izz:.6f}", ixy="0", ixz="0", iyz="0")

        # Visual
        visual = ET.SubElement(link_el, "visual")
        ET.SubElement(visual, "origin", xyz=local_visual_offset, rpy="0 0 0")
        geom = ET.SubElement(visual, "geometry")
        if shape == "cylinder":
            r = max(bbox_m[0], bbox_m[1]) / 2
            ET.SubElement(geom, "cylinder", radius=f"{r:.4f}", length=f"{bbox_m[2]:.4f}")
        elif shape == "sphere":
            r = max(bbox_m) / 2
            ET.SubElement(geom, "sphere", radius=f"{r:.4f}")
        else:
            ET.SubElement(geom, "box", size=f"{bbox_m[0]:.4f} {bbox_m[1]:.4f} {bbox_m[2]:.4f}")

        mat = ET.SubElement(visual, "material", name=f"mat_{link_name}")
        ET.SubElement(mat, "color", rgba="0.7 0.7 0.7 1")

        # Joint (skip for base link)
        attach_to = comp.get("attach_to")
        if attach_to and attach_to in comp_lookup:
            parent_preset = comp_lookup[attach_to]["preset"]
            child_preset = preset
            attach_face = comp.get("attach_face", "top")

            # Get explicit rpy from Claude (if provided)
            explicit_rpy = comp.get("attach_rpy", [0, 0, 0])
            if not isinstance(explicit_rpy, list) or len(explicit_rpy) != 3:
                explicit_rpy = [0, 0, 0]

            origin_xyz, computed_rpy = _compute_origin_and_rpy(
                parent_preset, child_preset, attach_face, explicit_rpy
            )
            rpy_str = f"{computed_rpy[0]:.4f} {computed_rpy[1]:.4f} {computed_rpy[2]:.4f}"

            joint_type = comp.get("joint_type", "fixed")
            raw_axis = comp.get("joint_axis", "z")
            # Resolve string axis to vector
            axis_map = {"x": [1,0,0], "y": [0,1,0], "z": [0,0,1]}
            if isinstance(raw_axis, str):
                joint_axis = axis_map.get(raw_axis.lower(), [0,0,1])
            elif isinstance(raw_axis, list) and len(raw_axis) == 3:
                joint_axis = raw_axis
            else:
                joint_axis = [0, 0, 1]
            joint_name = f"j_{link_name}"

            joint_el = ET.SubElement(robot, "joint", name=joint_name, type=joint_type)
            ET.SubElement(joint_el, "parent", link=attach_to)
            ET.SubElement(joint_el, "child", link=link_name)
            ET.SubElement(joint_el, "origin", xyz=f"{origin_xyz[0]:.4f} {origin_xyz[1]:.4f} {origin_xyz[2]:.4f}", rpy=rpy_str)
            ET.SubElement(joint_el, "axis", xyz=f"{joint_axis[0]} {joint_axis[1]} {joint_axis[2]}")

            if joint_type in ("revolute", "prismatic"):
                me = preset.get("mechanical_electrical", {})
                effort = me.get("max_torque_nm", 10.0)
                ET.SubElement(joint_el, "limit", lower="-3.14159", upper="3.14159",
                             effort=f"{effort}", velocity="1.0")

    # Apply ground offset: shift root link's visual up so bottom face is at Z=0
    if assembly.get("ground_offset", False) and base_link in comp_lookup:
        root_preset = comp_lookup[base_link]["preset"]
        root_bbox = _get_bbox_m(root_preset)
        half_h = root_bbox[2] / 2
        # Find the root link's visual origin and shift it up
        for link_el in robot.findall("link"):
            if link_el.get("name") == base_link:
                for visual in link_el.findall("visual"):
                    origin = visual.find("origin")
                    if origin is not None:
                        origin.set("xyz", f"0 0 {half_h:.4f}")
                for inertial in link_el.findall("inertial"):
                    origin = inertial.find("origin")
                    if origin is not None:
                        origin.set("xyz", f"0 0 {half_h:.4f}")
                break
        print(f"[assembly] Applied ground_offset: shifted root up by {half_h:.4f}m", file=sys.stderr)

    # Pretty-print
    def indent(elem, level=0):
        i = "\n" + level * "  "
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "  "
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for child in elem:
                indent(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i
        if not level:
            elem.tail = "\n"

    indent(robot)
    xml_str = '<?xml version="1.0"?>\n' + ET.tostring(robot, encoding="unicode")
    return xml_str


def _validate_and_log(urdf: str) -> str:
    """Run spatial validation on generated URDF and log warnings."""
    try:
        from model.urdf_parser import parse_urdf_string
        from validation.validator import validate_kinematic_graph
        kg = parse_urdf_string(urdf)
        results = validate_kinematic_graph(kg)
        warnings = [r for r in results if r.get("severity") in ("warn", "error")]
        if warnings:
            print(f"[ai_edit] Post-generation validation: {len(warnings)} issue(s):", file=sys.stderr)
            for w in warnings:
                print(f"  [{w['severity']}] {w['name']}: {w['message']}", file=sys.stderr)
        else:
            print(f"[ai_edit] Post-generation validation: all checks passed", file=sys.stderr)
    except Exception as e:
        print(f"[ai_edit] Post-generation validation skipped: {e}", file=sys.stderr)
    return urdf


def generate_edit(prompt: str, current_urdf: str, kinematic_graph_json: dict,
                   kinematic_context: str = None, session_id: str = "default") -> dict:
    """
    Call Claude API to generate a robot edit based on natural language.
    Maintains conversation history per session for multi-turn context.

    Args:
        prompt: User's natural language edit request
        current_urdf: Current URDF XML as string
        kinematic_graph_json: Kinematic graph as dict (from kg.to_json())
        kinematic_context: Optional structured text summary of robot structure from frontend
        session_id: Session identifier for conversation history tracking

    Returns:
        Dict with keys:
        - "explanation": str
        - "new_urdf": str
        - "stats": str

    Raises:
        ImportError: If anthropic package is not installed
        ValueError: If ANTHROPIC_API_KEY env var not set
        Exception: On API errors or JSON parsing issues
    """
    client = _get_client()

    # Build spatial context from kinematic graph
    spatial_context = _build_spatial_context(kinematic_graph_json) if kinematic_graph_json else ""

    # Build user message with available context
    user_message = f"""Current URDF:
```xml
{current_urdf}
```

Kinematic Graph:
```json
{json.dumps(kinematic_graph_json, indent=2)}
```"""

    if spatial_context:
        user_message += f"\n\n{spatial_context}"

    if kinematic_context:
        user_message += f"""

Robot Structure Summary:
{kinematic_context}"""

    user_message += f"""

User Request: {prompt}"""

    # Build messages array with conversation history
    history = _conversation_history[session_id]
    messages = list(history) + [{"role": "user", "content": user_message}]

    system_prompt = SYSTEM_PROMPT.replace("{COMPONENT_CATALOG}", _get_component_catalog())
    system_prompt = system_prompt.replace("{MOUNTING_CONTEXT}", _get_mounting_context())

    response = client.messages.create(
        model="claude-sonnet-4-20250514",
        max_tokens=64000,  # API hard limit (65536 rejected)
        system=system_prompt,
        messages=messages,
        timeout=180.0,
    )

    # Parse the response
    response_text = response.content[0].text

    # Store conversation turn in history (compact: just prompt + explanation, not full URDF)
    history.append({"role": "user", "content": f"[Edit request] {prompt}"})
    history.append({"role": "assistant", "content": response_text})

    # Trim history to cap
    while len(history) > _MAX_HISTORY_MESSAGES:
        history.pop(0)

    # Try to extract JSON from the response (handle markdown code blocks)
    result = _parse_json_response(response_text)

    # Check for Option C: assembly graph
    if "assembly" in result and result["assembly"]:
        print(f"[ai_edit] Claude assembly JSON: {json.dumps(result['assembly'], indent=2)}", file=sys.stderr)
        new_urdf = _assemble_from_graph(result["assembly"])
        print(f"[ai_edit] Using assembly graph ({len(result['assembly'].get('components', []))} components)", file=sys.stderr)
        new_urdf = _validate_and_log(new_urdf)
    # Check if response uses full_urdf (Option B: complete replacement)
    elif "full_urdf" in result and result["full_urdf"]:
        new_urdf = result["full_urdf"]
        print(f"[ai_edit] WARNING: Claude used Option B (raw URDF) instead of Option C", file=sys.stderr)
    else:
        # Apply search/replace edits (Option A: incremental)
        edits = result.get("edits", [])
        new_urdf = _apply_edits(current_urdf, edits)

    return {
        "explanation": result.get("explanation", "Changes applied"),
        "new_urdf": new_urdf,
        "stats": result.get("changes_summary", "Edit complete"),
    }


def generate_edit_streaming(prompt: str, current_urdf: str, kinematic_graph_json: dict,
                             kinematic_context: str = None, session_id: str = "default",
                             on_progress=None) -> dict:
    """
    Streaming version of generate_edit. Calls on_progress(stage, text) as tokens arrive.
    Stages: "thinking", "generating", "applying"
    """
    client = _get_client()

    # Build spatial context
    spatial_context = _build_spatial_context(kinematic_graph_json) if kinematic_graph_json else ""

    user_message = f"""Current URDF:
```xml
{current_urdf}
```

Kinematic Graph:
```json
{json.dumps(kinematic_graph_json, indent=2)}
```"""

    if spatial_context:
        user_message += f"\n\n{spatial_context}"

    if kinematic_context:
        user_message += f"""

Robot Structure Summary:
{kinematic_context}"""

    user_message += f"""

User Request: {prompt}"""

    history = _conversation_history[session_id]
    messages = list(history) + [{"role": "user", "content": user_message}]

    system_prompt = SYSTEM_PROMPT.replace("{COMPONENT_CATALOG}", _get_component_catalog())
    system_prompt = system_prompt.replace("{MOUNTING_CONTEXT}", _get_mounting_context())

    if on_progress:
        on_progress("thinking", "Analyzing model...")

    # Use streaming API
    accumulated_text = ""
    try:
        with client.messages.stream(
            model="claude-sonnet-4-20250514",
            max_tokens=64000,  # API hard limit (65536 rejected)
            system=system_prompt,
            messages=messages,
        ) as stream:
            token_count = 0
            sent_generating = False
            for text in stream.text_stream:
                accumulated_text += text
                token_count += 1

                if not sent_generating and token_count > 2:
                    if on_progress:
                        on_progress("generating", "Generating design...")
                    sent_generating = True

                # Stream partial explanation every ~8 tokens for smooth updates
                if on_progress and token_count % 8 == 0:
                    partial = _extract_partial_explanation(accumulated_text)
                    if partial:
                        on_progress("streaming", partial)

    except Exception as e:
        raise ValueError(f"Streaming API call failed: {e}")

    response_text = accumulated_text

    if on_progress:
        on_progress("applying", "Applying changes...")

    # Store conversation history
    history.append({"role": "user", "content": f"[Edit request] {prompt}"})
    history.append({"role": "assistant", "content": response_text})
    while len(history) > _MAX_HISTORY_MESSAGES:
        history.pop(0)

    # Parse and apply
    result = _parse_json_response(response_text)

    # Check for Option C: assembly graph
    if "assembly" in result and result["assembly"]:
        print(f"[ai_edit] Claude assembly JSON: {json.dumps(result['assembly'], indent=2)}", file=sys.stderr)
        new_urdf = _assemble_from_graph(result["assembly"])
        print(f"[ai_edit] Using assembly graph ({len(result['assembly'].get('components', []))} components)", file=sys.stderr)
        new_urdf = _validate_and_log(new_urdf)
    elif "full_urdf" in result and result["full_urdf"]:
        new_urdf = result["full_urdf"]
        print(f"[ai_edit] WARNING: Claude used Option B (raw URDF) instead of Option C", file=sys.stderr)
    else:
        edits = result.get("edits", [])
        new_urdf = _apply_edits(current_urdf, edits)

    return {
        "explanation": result.get("explanation", "Changes applied"),
        "new_urdf": new_urdf,
        "stats": result.get("changes_summary", "Edit complete"),
    }


def _extract_partial_explanation(text: str) -> str:
    """Try to extract the explanation field from partial JSON for live preview."""
    # Look for "explanation": "..." pattern
    match = re.search(r'"explanation"\s*:\s*"((?:[^"\\]|\\.)*)', text)
    if match:
        return match.group(1).replace('\\"', '"').replace('\\n', ' ')
    return ""


def clear_conversation(session_id: str = "default") -> None:
    """Clear conversation history for a session."""
    _conversation_history.pop(session_id, None)


def _parse_json_response(text: str) -> dict:
    """
    Parse JSON from Claude's response, handling markdown code blocks.

    Args:
        text: Response text that may contain JSON with or without markdown

    Returns:
        Parsed JSON dict

    Raises:
        ValueError: If JSON cannot be parsed
    """
    # Try direct JSON parsing first
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass

    # Try to extract JSON from markdown code blocks
    # Look for ```json ... ``` or ``` ... ```
    patterns = [
        r"```(?:json)?\s*\n?(.*?)\n?```",
        r"```(.*?)```",
    ]

    for pattern in patterns:
        matches = re.findall(pattern, text, re.DOTALL)
        for match in matches:
            try:
                return json.loads(match)
            except json.JSONDecodeError:
                continue

    # Try to find JSON object in the text (strip any preamble text before first {)
    first_brace = text.find('{')
    last_brace = text.rfind('}')
    if first_brace >= 0 and last_brace > first_brace:
        try:
            return json.loads(text[first_brace:last_brace + 1])
        except json.JSONDecodeError:
            pass

    # If all else fails, raise error with context
    raise ValueError(
        f"Failed to parse JSON from response. Response text:\n{text[:500]}"
    )


def _postprocess_completion(completion: str, cursor_prefix: str) -> str:
    """
    Fix common model mistakes in completions by examining what the prefix
    ends with and what the completion starts with.
    """
    if not completion or not cursor_prefix:
        return completion

    prefix = cursor_prefix.rstrip('\n')  # preserve spaces, strip trailing newlines

    # ── Fix 1: Missing space before XML attribute ──
    # If prefix ends with a tag name like `<link` or `<joint` and the completion
    # starts with an attribute like `name=` or `type=`, inject a space.
    # Also handles `</link` or `</joint` that shouldn't get attributes at all.
    if completion and completion[0] not in (' ', '\n', '\t', '>', '/', '"'):
        # Check if prefix ends with an XML tag name (opening or closing)
        tag_match = re.search(r'</?(\w+)$', prefix)
        if tag_match:
            # Check if completion starts with an attribute-like pattern
            if re.match(r'\w+=', completion) or re.match(r'\w+\s', completion):
                completion = ' ' + completion
                print(f"[ai_complete] Post-fix: injected space after tag name", file=sys.stderr)

    # ── Fix 2: Strip stray "> on its own or after closing tags ──
    # The model sometimes leaves orphaned "> from broken tag constructions
    completion = re.sub(r'(</\w+>)\s*">', r'\1', completion)
    # Also strip lines that are ONLY "> or "/>
    completion = re.sub(r'\n\s*">\s*$', '', completion)
    completion = re.sub(r'^\s*">\s*\n', '', completion)

    # ── Fix 3: Detect repeated/duplicate content ──
    # If the completion contains a block of lines that duplicates lines already
    # present earlier in the completion (the model looping), truncate at the repeat.
    lines = completion.split('\n')
    if len(lines) > 4:
        seen_stripped: list[str] = []
        for i, line in enumerate(lines):
            s = line.strip()
            if not s:
                seen_stripped.append(s)
                continue
            # Check if this line + next line match a pair we already saw
            # (single line matches are too aggressive -- common XML like </geometry> repeats legitimately)
            if i > 0 and i < len(lines) - 1:
                pair = (s, lines[i + 1].strip())
                for j in range(len(seen_stripped) - 1):
                    if (seen_stripped[j], seen_stripped[j + 1]) == pair and s not in ('', '<visual>', '<collision>', '<geometry>'):
                        print(f"[ai_complete] Post-fix: truncated at repeated content (line {i}): {s[:50]}", file=sys.stderr)
                        completion = '\n'.join(lines[:i])
                        return completion
            seen_stripped.append(s)

    # ── Fix 4: Validate tag nesting ──
    # If completion closes a tag that was never opened within the completion
    # AND is not the expected parent close, truncate there.
    lines = completion.split('\n')
    tag_stack: list[str] = []
    valid_lines: list[str] = []
    # Allow exactly one "parent close" (closing a tag from the prefix, like </link>)
    parent_closes_allowed = 1
    parent_closes_seen = 0

    for line in lines:
        stripped_line = line.strip()

        opens = re.findall(r'<(\w+)[\s>]', stripped_line)
        self_closing = re.findall(r'<(\w+)\s[^>]*/>', stripped_line)
        closes = re.findall(r'</(\w+)>', stripped_line)
        real_opens = [t for t in opens if t not in self_closing]

        bad_close = False
        for tag in closes:
            if tag in tag_stack:
                idx = len(tag_stack) - 1 - tag_stack[::-1].index(tag)
                tag_stack = tag_stack[:idx]
            elif not tag_stack:
                parent_closes_seen += 1
                if parent_closes_seen > parent_closes_allowed:
                    bad_close = True
            else:
                bad_close = True

        if bad_close:
            print(f"[ai_complete] Post-fix: truncated at bad nesting: {stripped_line[:50]}", file=sys.stderr)
            break

        for tag in real_opens:
            tag_stack.append(tag)
        valid_lines.append(line)

    if len(valid_lines) < len(lines):
        completion = '\n'.join(valid_lines)

    return completion


def _is_low_quality_completion(completion: str, cursor_prefix: str, context_after: str) -> bool:
    """
    Filter out low-value completions that would annoy more than help.
    Returns True if the completion should be suppressed.
    """
    stripped = completion.strip()

    # Too short to be useful (< 3 non-whitespace chars)
    if len(stripped) < 3:
        return True

    # Just closing a tag the user is already typing
    # e.g., prefix ends with `</link` and completion is just `>`
    if stripped in ('>', '/>', '">', '"/>'):
        return True

    # Completion is just a closing tag for something the user already has
    # e.g., prefix has `</collision>` and completion is `\n  </link>` -- that's just boilerplate closing
    if re.match(r'^\s*</\w+>\s*$', stripped):
        # Check if this closing tag already exists right after cursor
        close_tag = stripped.strip()
        after_stripped = context_after.lstrip()
        if after_stripped.startswith(close_tag):
            return True

    # Completion is identical to what's already after the cursor (pure duplication)
    if stripped and context_after.strip().startswith(stripped):
        return True

    return False


def _strip_suffix_overlap(completion: str, suffix: str) -> str:
    """
    Remove the longest tail of `completion` that matches a prefix of `suffix`.
    This prevents duplication when the model's output continues into code that
    already exists after the cursor.

    Uses line-level matching first (faster, handles the common case), then
    falls back to character-level for partial-line overlaps.
    """
    if not completion or not suffix:
        return completion

    # Normalize for comparison (strip trailing whitespace per line)
    comp_lines = completion.split('\n')
    suf_lines = suffix.split('\n')

    # Line-level overlap: find the longest tail of comp_lines that matches
    # the head of suf_lines (comparing stripped content to be whitespace-tolerant)
    best_overlap = 0
    for overlap_len in range(1, min(len(comp_lines), len(suf_lines)) + 1):
        tail = [l.strip() for l in comp_lines[-overlap_len:]]
        head = [l.strip() for l in suf_lines[:overlap_len]]
        if tail == head:
            best_overlap = overlap_len

    if best_overlap > 0:
        trimmed = '\n'.join(comp_lines[:-best_overlap])
        print(f"[ai_complete] Stripped {best_overlap} overlapping lines from completion", file=sys.stderr)
        return trimmed

    # Character-level fallback: find longest suffix of completion that matches
    # prefix of suffix text (for partial-line overlaps)
    max_check = min(len(completion), len(suffix), 200)
    for length in range(max_check, 0, -1):
        if completion[-length:] == suffix[:length]:
            print(f"[ai_complete] Stripped {length} overlapping chars from completion", file=sys.stderr)
            return completion[:-length]

    return completion


# ── Completion cache ──────────────────────────────────────────────────────────
import hashlib
_completion_cache: dict[str, str] = {}
_CACHE_MAX_SIZE = 50


def _cache_key(cursor_prefix: str, context_before: str, context_after: str) -> str:
    raw = f"{cursor_prefix}||{context_before[-200:]}||{context_after[:200]}"
    return hashlib.md5(raw.encode()).hexdigest()


def _find_parent_element(urdf_content: str, cursor_index: int) -> tuple[str, str]:
    try:
        before = urdf_content[:cursor_index]
        after = urdf_content[cursor_index:]
        open_stack: list[tuple[str, int]] = []
        for m in re.finditer(r'<(/?)(\w+)[\s>]', before):
            is_close = m.group(1) == '/'
            tag_name = m.group(2)
            if is_close:
                for i in range(len(open_stack) - 1, -1, -1):
                    if open_stack[i][0] == tag_name:
                        open_stack.pop(i)
                        break
            else:
                tag_start = m.start()
                tag_end_search = before[tag_start:]
                if '/>' in tag_end_search.split('>')[0] if '>' in tag_end_search else False:
                    continue
                open_stack.append((tag_name, tag_start))
        if open_stack:
            parent_tag, parent_pos = open_stack[-1]
            context_start = max(0, parent_pos - 200)
            context_before = urdf_content[context_start:cursor_index]
            close_pattern = f'</{parent_tag}>'
            close_match = re.search(re.escape(close_pattern), after)
            if close_match:
                context_end = cursor_index + close_match.end() + 200
                context_after = urdf_content[cursor_index:min(len(urdf_content), context_end)]
                return context_before, context_after
    except Exception as e:
        print(f"[ai_complete] XML-aware context failed, using flat window: {e}", file=sys.stderr)
    context_before = urdf_content[max(0, cursor_index - 2000):cursor_index]
    context_after = urdf_content[cursor_index:min(len(urdf_content), cursor_index + 1000)]
    return context_before, context_after


def generate_completion(
    urdf_content: str, cursor_line: int, cursor_column: int,
    prefix: str = "", kinematic_context: str = ""
) -> str:
    global _completion_cache
    client = _get_client()
    lines = urdf_content.split('\n')
    cursor_index = sum(len(line) + 1 for line in lines[:cursor_line - 1]) + (cursor_column - 1)
    context_before, context_after = _find_parent_element(urdf_content, cursor_index)
    broad_before = urdf_content[max(0, cursor_index - 4000):cursor_index]
    broad_after = urdf_content[cursor_index:min(len(urdf_content), cursor_index + 2000)]
    cursor_prefix = broad_before[-100:] if len(broad_before) > 100 else broad_before
    cache_k = _cache_key(cursor_prefix, context_before, context_after)
    if cache_k in _completion_cache:
        cached = _completion_cache[cache_k]
        print(f"[ai_complete] Cache hit ({len(cached)} chars)", file=sys.stderr)
        return cached
    user_message = f"""===CURSOR_PREFIX===\n{cursor_prefix}\n===END_CURSOR_PREFIX===\n\n===CONTEXT===\n...{context_before[-800:]}|CURSOR|{context_after[:800]}...\n===END_CONTEXT==="""
    if kinematic_context:
        user_message += f"\n\n===ROBOT===\n{kinematic_context}\n===END_ROBOT==="
    print(f"[ai_complete] Calling Claude Haiku at line {cursor_line}", file=sys.stderr)
    t0 = time.time()
    response = client.messages.create(
        model="claude-haiku-4-5-20251001", max_tokens=600, temperature=0,
        system=COMPLETION_SYSTEM_PROMPT,
        messages=[{"role": "user", "content": user_message}],
        stop_sequences=["</robot>", "<!--"], timeout=10.0,
    )
    elapsed = time.time() - t0
    print(f"[ai_complete] Claude responded in {elapsed:.1f}s", file=sys.stderr)
    completion_text = response.content[0].text
    if completion_text.startswith('```'):
        inner = completion_text.split('```')
        completion_text = inner[1] if len(inner) > 1 else completion_text
        if completion_text.startswith('xml\n'): completion_text = completion_text[4:]
        elif completion_text.startswith('xml'): completion_text = completion_text[3:]
    if completion_text.endswith('```'):
        completion_text = completion_text.rsplit('```', 1)[0]
    stripped = completion_text.strip()
    if stripped and stripped[0].isupper() and len(stripped) > 10:
        first_word = stripped.split()[0] if stripped.split() else ""
        if first_word in {"The","This","Here","I","You","To","In","It","Note","For","A","An"}:
            print(f"[ai_complete] Rejected prose response: {stripped[:50]}", file=sys.stderr)
            return ""
    completion_text = _postprocess_completion(completion_text, cursor_prefix)
    completion_text = _strip_suffix_overlap(completion_text, broad_after)
    if _is_low_quality_completion(completion_text, cursor_prefix, broad_after):
        print(f"[ai_complete] Filtered low-quality: {completion_text.strip()[:50]!r}", file=sys.stderr)
        return ""
    _completion_cache[cache_k] = completion_text
    if len(_completion_cache) > _CACHE_MAX_SIZE:
        oldest_key = next(iter(_completion_cache))
        del _completion_cache[oldest_key]
    return completion_text

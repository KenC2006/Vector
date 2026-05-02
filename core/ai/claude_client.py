"""
Claude AI client for robot design editing.
Communicates with Claude API to generate robot model edits based on natural language requests.
"""
import base64
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

# Lazy import — google-genai for Gemini visual validation
_genai = None
_genai_error = None
try:
    from google import genai as _genai_module
    from google.genai import types as _genai_types
    _genai = _genai_module
except ImportError as e:
    _genai_error = str(e)

# ── Singleton clients ─────────────────────────────────────────────────────────
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


_gemini_client = None

def _get_gemini_client():
    """Return a shared Gemini client instance for visual validation."""
    global _gemini_client
    if _gemini_client is None:
        if _genai is None:
            raise ImportError(
                f"google-genai package not installed. Run: pip install google-genai\n"
                f"Error: {_genai_error}"
            )
        api_key = os.environ.get("GEMINI_API_KEY")
        if not api_key:
            raise ValueError("GEMINI_API_KEY environment variable not set")
        _gemini_client = _genai.Client(api_key=api_key)
    return _gemini_client


# ── Prompt cache logging ─────────────────────────────────────────────────────
def _log_cache_usage(label: str, response) -> None:
    """Log prompt cache hit/miss stats for debugging rate limit and cost issues."""
    usage = response.usage
    cache_write = getattr(usage, 'cache_creation_input_tokens', 0) or 0
    cache_read = getattr(usage, 'cache_read_input_tokens', 0) or 0
    regular = getattr(usage, 'input_tokens', 0) or 0
    output = getattr(usage, 'output_tokens', 0) or 0
    print(
        f"[ai_cache] {label}: input={regular} cache_write={cache_write} "
        f"cache_read={cache_read} output={output} "
        f"({'HIT' if cache_read > 0 else 'MISS' if cache_write > 0 else 'NONE'})",
        file=sys.stderr,
    )


# ── Conversation history store (keyed by session_id) ──────────────────────────
# Each entry is a list of {"role": "user"|"assistant", "content": str} dicts.
# Capped to last 20 messages to avoid unbounded token growth.
#
# NOTE: images are never persisted here. The text-only summary produced by
# `_build_history_summary` is what gets stored; attached reference images live
# only in the turn that sent them. On history replay from localStorage the
# text summary is restored, but the images are intentionally dropped
# (they'd bloat localStorage 5MB+ per turn and don't survive a reload anyway).
_conversation_history: dict[str, list] = defaultdict(list)
_MAX_HISTORY_MESSAGES = 20


def _build_user_content(text: str, images: list | None):
    """
    Build a user message `content` field. Returns a plain string when no
    valid images are attached (preserves the existing wire format so
    conversation history replay stays unchanged); returns a list of content
    blocks ordered text-first, images-after when images are present.

    Anthropic Messages API shape per
    https://platform.claude.com/docs/en/build-with-claude/vision :
        {"type": "image",
         "source": {"type": "base64",
                    "media_type": "image/jpeg" | "image/png" | "image/gif" | "image/webp",
                    "data": "<raw base64, no data: prefix>"}}
    """
    if not images:
        return text
    image_blocks: list = []
    for img in images:
        media_type = img.get("media_type") if isinstance(img, dict) else None
        data = img.get("data") if isinstance(img, dict) else None
        if not media_type or not data:
            continue
        image_blocks.append({
            "type": "image",
            "source": {
                "type": "base64",
                "media_type": media_type,
                "data": data,
            },
        })
    if not image_blocks:
        # All entries were malformed — fall back to plain-string form so we
        # don't silently change the wire format when images == [].
        return text
    return [{"type": "text", "text": text}, *image_blocks]


# Component IDs the AI may use. Most have verified GLB meshes; drivetrain presets
# use parametric fallback rendering (box/cylinder from bounding_box_mm) — no GLB needed.
_ALLOWED_COMPONENT_IDS = {
    # Actuators — core servo range + continuous rotation + linear
    'actuator_servo_micro', 'actuator_servo_standard', 'actuator_servo_high_torque',
    'actuator_continuous_rotation_servo', 'actuator_linear_small',
    # Motors — small DC only (geared motor replaced by drivetrain presets for wheels)
    'motor_dc_small_130',
    # Drivetrain assemblies — hub motor, caster, passive axle, steering knuckle
    'drivetrain_hub_motor_80', 'drivetrain_caster_swivel',
    'drivetrain_stub_axle_passive', 'drivetrain_steering_knuckle',
    # Sensors — camera, lidar, IMU, range, force, encoder
    'sensor_depth_camera_small', 'sensor_lidar_2d',
    'sensor_imu_6dof', 'sensor_ultrasonic',
    'sensor_force_torque_6axis', 'sensor_joint_encoder_absolute',
    # Compute — MCU, SBC, motor driver
    'compute_mcu_small', 'compute_sbc_small', 'compute_motor_driver_dual',
    # Power — two battery sizes + regulation + distribution
    'power_lipo_3s_2200', 'power_lipo_4s_5000',
    'power_buck_converter_5v', 'power_distribution_unit',
    # Structural — baseplate (always root), extrusions, brackets, shaft collar
    'structural_baseplate', 'structural_baseplate_large',
    'structural_limb_link_slim', 'structural_extrusion_2020', 'structural_extrusion_4040',
    'structural_bracket_l', 'structural_bracket_u', 'structural_servo_side_yoke_mount',
    'structural_servo_horn_beam_adapter', 'structural_shaft_collar',
    # Transmission — belt, leadscrew, bearing, coupling
    'transmission_timing_belt_gt2', 'transmission_leadscrew_8mm',
    'transmission_bearing_deep_groove', 'transmission_flexible_coupling_jaw',
    # End Effectors — two grippers + suction
    'effector_parallel_gripper_small', 'effector_parallel_gripper_large', 'effector_suction_cup',
    # Mobility — wheel, caster, mecanum, foot pad
    'mobility_wheel_driven', 'mobility_caster_wheel',
    'mobility_mecanum_wheel', 'mobility_rubber_foot_pad',
}


def _is_tire_component_id(component_id: str) -> bool:
    return component_id.startswith((
        "mobility_wheel_",
        "mobility_mecanum_",
        "mobility_omni_",
        "mobility_caster_",
    ))


def _is_drivetrain_component_id(component_id: str) -> bool:
    return component_id.startswith("drivetrain_")


def _build_component_catalog() -> str:
    """Build a compact summary of available preset components for the AI system prompt.
    Only includes components with verified GLB meshes shown in the UI."""
    try:
        from core.presets import list_components, get_all_categories, get_category
        from core.ai.catalog_selector import render_connector_hint
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
                conn_hint = render_connector_hint(c)
                items.append(f"  - {c['id']}: {c['name']} [{mass_str}, {bb_str}, {shape}]{(' ' + spec) if spec else ''}{conn_hint}")
            lines.append(f"\n{label} ({len(comps)}):")
            lines.extend(items)
        return "\n".join(lines)
    except Exception as e:
        return f"(Component catalog unavailable: {e})"

_COMPONENT_CATALOG = None

def _get_component_catalog(
    user_prompt: str | None = None,
    kg_json: dict | None = None,
    tried_preset_ids=None,
) -> str:
    """Return the preset catalog text for the system prompt.

    Default path (VECTOR_DYNAMIC_CATALOG unset): the cached full dump,
    identical to pre-WS4 behavior. This keeps a clean A/B baseline for
    measuring the dynamic-catalog change in isolation after WS1/WS3 ship.

    When VECTOR_DYNAMIC_CATALOG is truthy: delegate to the scoped selector,
    which returns only presets relevant to this request + a core floor.
    """
    from core.ai.catalog_selector import dynamic_catalog_enabled, build_scoped_catalog
    if dynamic_catalog_enabled() and user_prompt is not None:
        try:
            return build_scoped_catalog(
                user_prompt=user_prompt,
                allowed_ids=_ALLOWED_COMPONENT_IDS,
                kg_json=kg_json,
                tried_preset_ids=tried_preset_ids,
            )
        except Exception as e:
            # Scoring failure must never block a request — fall through
            # to the full catalog, log so regressions are visible.
            print(f"[ai_catalog] scoped catalog failed, using full dump: {e}", file=sys.stderr)
    global _COMPONENT_CATALOG
    if _COMPONENT_CATALOG is None:
        _COMPONENT_CATALOG = _build_component_catalog()
    return _COMPONENT_CATALOG


def _tried_preset_ids(kg_json: dict | None) -> list[str]:
    """Extract the preset_ids currently attached in the graph.

    Used to surface "you already picked these" on retry — the scorer always
    includes them in the scoped catalog so Claude's next turn can either
    keep them or explicitly swap them out. Without this, the scorer might
    drop a preset Claude just used if retry tokens look unrelated to it.
    """
    if not kg_json or not isinstance(kg_json, dict):
        return []
    seen: list[str] = []
    unique: set[str] = set()
    candidates = []
    links = kg_json.get("links")
    if isinstance(links, list):
        candidates.extend(links)
    elif isinstance(links, dict):
        candidates.extend(links.values())
    comps = kg_json.get("components")
    if isinstance(comps, list):
        candidates.extend(comps)
    for entry in candidates:
        if not isinstance(entry, dict):
            continue
        cid = entry.get("component_id") or entry.get("preset_id")
        if isinstance(cid, str) and cid not in unique:
            unique.add(cid)
            seen.append(cid)
    return seen


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


SYSTEM_PROMPT = r"""You are a robot assembly agent for Vector IDE.

You design robots by specifying TOPOLOGY ONLY -- which components connect to which, and how. A backend placement engine handles all 3D positioning, rotation, scaling, and URDF generation. You never write coordinates or URDF XML.

## Coordinate System (URDF standard)

X = right, Y = forward, Z = up.
- "top" face = +Z direction (upward)
- "bottom" face = -Z direction (downward)
- "front" face = +X direction (forward)
- "back" face = -X direction (backward)
- "right" face = +Y direction
- "left" face = -Y direction

When you specify attach_face, you are choosing which DIRECTION from the parent the child extends.

## Available Components

{COMPONENT_CATALOG}

## Tools

You MUST respond by calling one of the provided tools:
- **design_robot**: For "build", "create", "design", or "make" requests — when building a robot from scratch or the user wants a complete redesign. Specify the full component topology.
- **modify_topology**: For iterative edits to an existing robot — "add a camera", "remove the tail", "make the arms longer", "add 2 more wheels", "replace the gripper with a suction cup". Specifies add/remove/modify operations on the existing component tree. The placement engine re-resolves the full assembly.

### When to use which tool:
- User describes a NEW robot or says "start over" → **design_robot**
- User wants to CHANGE an existing robot (add, remove, modify components) → **modify_topology**
- When in doubt: if the current URDF has real components (not just a base_link placeholder), prefer **modify_topology**.

## What the Backend Handles Automatically

- All xyz coordinates and rpy rotations
- Elongated parts use the orientation hint to determine rotation direction
- Multiple children on the same face are distributed to corners (e.g., 4 wheels on "bottom" go to 4 corners)
- Passive leg links on "bottom" can get automatic outward splay, but rotary servo bodies stay flat; joint_axis/rest pose controls leg angle. Wheels are excluded from splay and stay level.
- Ground offset so the robot sits on the floor
- Collision geometry, inertia computation, visual materials

## Rotation Controls

Beyond attach_face and joint_axis, you have two additional per-component rotation parameters:

### orientation
Controls how an elongated or directable component is rotated within its face:
- **"vertical"** (default): component extends along +Z (upward). Use for leg segments, vertical masts.
- **"horizontal"**: component extends along +X (forward). Use for tails, horizontal booms.
- **"auto"**: engine chooses based on component type.
- **Numeric string (degrees)**: yaw rotation around the face normal. E.g., `"45"` rotates 45° on a top-face component. Use to angle sensors, offset actuators, or fan out side mounts. Combine with "horizontal" for a horizontally-extended component at a specific yaw: specify e.g. `"horizontal+45"` — the engine applies 90° pitch then 45° yaw.

### elevation_angle
**Only applies to side faces (front, back, left, right).** Tilts the component up (+) or down (−) from the face normal, in degrees.
- Use for cameras/sensors that should angle toward the ground or sky.
- Use for arms that extend outward but pitch upward at rest.
- Example: a depth camera on the front face with elevation_angle=-20 angles 20° downward to see the floor.
- Range: typically −45 to +45. Applied as pitch (front/back) or roll (left/right).

### attach_rpy
**Optional 3-element [roll, pitch, yaw] in RADIANS** applied verbatim to the joint origin relative to the parent. Overrides the engine's default rotation — use for rest-pose joint angles (quadruped crouch, forward-splayed shoulder, etc.).
- Omit (or pass [0, 0, 0]) to let the engine auto-rotate. That's the default for almost every component.
- Example (Z-crouch hip pitch ≈ +30°): `attach_rpy=[0, 0.52, 0]` on the thigh-to-hip-pitch-servo link.
- Example (Z-crouch knee ≈ 60° magnitude): `attach_rpy=[0, 1.05, 0]` on the shin-to-knee-servo link. The assembler maps this magnitude onto the mirrored servo horn sign.
- Prefer this over `elevation_angle` when the face is top/bottom (elevation_angle only applies to side faces).
- **For rotary servos specifically:** `attach_rpy` is the horn's initial/rest offset, not a housing tilt. The servo housing stays bolted flat to its parent face; the horn and everything below it start at this offset and the controller drives relative to that zero. `attach_rpy=[0, 0.52, 0]` on hip_pitch means the thigh chain starts 30° forward while the servo body remains properly mounted.
- For rotary servos with `joint_axis="z"` mounted on a top/bottom face, the assembler keeps the horn shaft normal to the plate; bottom-mounted planar servos have the horn facing downward so the child sweeps in the XY plane.
- For rotary servos with `joint_axis="x"` or `"y"`, the assembler automatically adds the side-yoke holder geometry and rotates the servo body so its physical horn shaft lies on that red/blue axis. Do not compensate by adding extra coupler discs or tilting the servo housing yourself.

## Mate Connectors (optional — precision control for shaft mates and ambiguous surfaces)

Every part has 6 default face connectors — `top`, `bottom`, `front`, `back`, `left`, `right` — which is what `attach_face` picks. Some parts also author NAMED connectors visible as `conn=[…]` on the catalog line (e.g. `conn=[shaft_out(cyl 8mm)]` on a servo, `conn=[plate_top(plan), wall_inner(plan), wall_outer(plan)]` on an L-bracket).

**Critical — `attach_connector` vs `mate_connector` are NOT interchangeable:**
- `attach_connector` names a connector on the PARENT (the thing `attach_to` points at).
- `mate_connector` names a connector on this CHILD (the component you're adding).

If the ambiguous named connector (`plate_top`, `wall_inner`, `shaft_hole`, etc.) belongs to the component you're adding, it goes in `mate_connector`. If it belongs to the parent, it goes in `attach_connector`. Putting a child-side name into `attach_connector` makes the engine fail the lookup and fall back to default-face placement.

Two cases where named connectors beat `attach_face`:

1. **Concentric shaft mates** (servo/motor output → coupler/horn). When the user explicitly asks for a servo-shaft coupling, emit:
   - `attach_connector: "shaft_out"` (parent servo's shaft)
   - `mate_connector: "shaft_hole"` (child coupler's/horn's bore)
   - `mate_type: "concentric"` (shaft-in-hole, antiparallel axes — the engine aligns them)
   For ordinary servo→bracket/extrusion attachments, keep using `attach_face: "top"` — the engine auto-inserts the coupler/bracket and wires the concentric mate itself.

2. **Face-ambiguous parts** (L-bracket — has BOTH a horizontal plate and a vertical wall). When the L-bracket is the CHILD, use `mate_connector` to pick which bracket surface sits against the parent: `"plate_top"` (horizontal plate face up — mount on parent using the underside of the plate), `"wall_inner"` (concave inside face), `"wall_outer"` (convex back of wall). When the L-bracket is the PARENT and something mounts on it, use `attach_connector` with the same names.

Leave all three fields omitted for normal face-to-face mounts — `attach_face` is the right choice ~95% of the time.

Examples (note which side each named connector belongs to):
- Servo → coupler (concentric shaft mate). `shaft_out` lives on the parent servo, `shaft_hole` lives on the child coupler:
  Do not insert `structural_servo_coupler_disc` between a rotary servo and a limb/joint servo. Side-axis servos already get an internal yoke + horn-link adapter, and children attached to the servo are routed to that driven adapter.
- L-bracket mounted to baseplate's front face with its wall flush against the baseplate (plate sticks out forward as a shelf). `wall_outer` lives on the BRACKET — it's the child — so it goes in `mate_connector`, NOT `attach_connector`:
  `{"link_name": "structural_bracket_l_1", "component_id": "structural_bracket_l", "attach_to": "structural_baseplate_large_1", "attach_face": "front", "mate_connector": "wall_outer", "joint_type": "fixed", "joint_axis": "z"}`
- Camera mounted on that L-bracket's inside wall (bracket is now the PARENT, so `wall_inner` moves to `attach_connector`; camera's `mount_back` is the child-side name):
  `{"link_name": "sensor_depth_camera_small_1", "component_id": "sensor_depth_camera_small", "attach_to": "structural_bracket_l_1", "attach_connector": "wall_inner", "mate_connector": "mount_back", "mate_type": "fastened", "joint_type": "fixed", "joint_axis": "z"}`

## Topology Rules

1. Root is ALWAYS a baseplate. Pick `structural_baseplate` (200×150×5mm) for small rovers and tabletop arms; pick `structural_baseplate_large` (350×250×8mm) for quadrupeds, humanoid torsos, or any robot whose hip/shoulder span or payload mass outgrows the small plate. Never use an extrusion as root. **Do NOT downgrade a quadruped/humanoid from `structural_baseplate_large` to `structural_baseplate` on a redesign retry — the small plate is too narrow for the hip span. If a validator says "body is too wide, narrow to ~140mm", IGNORE IT: no preset in the palette is 140mm wide, and the hip/shoulder spacing needs the 250mm width. The large plate is the correct answer.**
2. Drivetrain motors (drivetrain_hub_motor_80, drivetrain_geared_dc_with_coupler) use joint_type="continuous" — unbounded spin, torque-controlled in sim. Servo actuators (actuator_servo_*, actuator_bldc_*, actuator_stepper_*) use joint_type="revolute" — bounded angle, PD-controlled. Everything else uses "fixed".
3. joint_axis by motion type — the axis is the rotation axis; the child sweeps in the plane PERPENDICULAR to it:
   - "y" — leg pitches forward/back, arm pitches up/down, knee bends, elbow bends, head nods
   - "x" — hip abducts laterally, shoulder rolls, wrist tilts side-to-side
   - "z" — base yaws in place, turret spins, hip/shoulder sweeps in the XY plane
   ❌ joint_axis="z" for a knee or elbow — that spins the limb around its long axis, not bends it.
   ❌ joint_axis="y" for hip abduction — abduction is lateral (frontal plane), use "x".
4. Multiple children on the same parent face are auto-distributed (wheels to corners, sensors to edges).
5. Include ALL components the user mentions. Do not skip or simplify.
6. For legs/downward extensions: use attach_face="bottom" so components extend DOWNWARD from their parent. Never use "top" for leg segments — "top" extends upward.
7. For arms: all links chain via "top" face going UPWARD. Do NOT use orientation="horizontal" — arm extrusions stand vertical at rest position, and joint servos control the angle. The shoulder servo pitches the upper arm, the elbow servo pitches the forearm.
8. For wheels: use a drivetrain assembly — baseplate → drivetrain_hub_motor_80 (bottom, **continuous** y) → mobility_wheel_driven (coaxial, fixed). The drivetrain IS the motor; it uses joint_type="continuous" (not "revolute") so the sim treats it as a torque motor, not a servo. Tires ALWAYS use attach_face="coaxial". The placement engine axially offsets the tire so its bore face seats against the motor body and auto-flips drivetrains on the -Y half of the baseplate so wheels end up outboard on both sides — you do not need to specify positions, orientations, or per-corner flips. Tires MUST NOT attach directly to the baseplate.
9. length_mm overrides parametric structural links (default 100mm). For sleek robot limbs, prefer `structural_limb_link_slim` over T-slot extrusion: use 80–120mm for leg segments, 150–300mm for arm links, and 50–80mm for short connectors. Use `structural_extrusion_2020/4040` for frames and chassis rails, not dog thighs/shins unless the user asks for bulky extrusion. For robot dogs/quadrupeds specifically, `structural_extrusion_2020` and `structural_extrusion_4040` are FORBIDDEN as thigh/shin/leg bones; use `structural_limb_link_slim`.
10. **Rotary servos drive exactly ONE child.** The backend splits each rotary servo into a fixed body (bolted to its parent) and a rotating horn (the output). For `joint_axis="x"` or `"y"`, it also inserts effective side-yoke plus slim horn-link adapter hardware so the physical horn shaft is on the red/blue hinge axis. Children you attach to a servo link are automatically routed to the horn/adapter — you do not need to name `_body` or `_horn` links yourself; just use the servo's `link_name` as `attach_to`. Attach exactly ONE child per servo; never fan out multiple children from the same servo.
11. **Sensors mount on STRUCTURAL links, not actuator shafts.** To mount a sensor near the end effector (e.g., "wrist camera"), attach it to the last extrusion in the chain, NOT to the wrist servo or the gripper. Example: `forearm_extrusion → wrist_servo → gripper`; the camera attaches to `forearm_extrusion` (front or top), not to `wrist_servo`.
12. **Electronics (battery, PDU, SBC, IMU, motor drivers) mount DIRECTLY on the baseplate's top face. NEVER route them through an intermediate structural_extrusion, regardless of count.** All three of these shapes are FORBIDDEN:
    - 4 vertical extrusions (one per electronic) → "ironing board on stilts"
    - 1 central vertical extrusion hosting multiple electronics → "torso tower" (validator will flag it AS WELL as the 4-standoff case)
    - An extrusion named `torso_extrusion_*` or `body_extrusion_*` used to "elevate" or "enclose" electronics
    The baseplate's top face distributes multiple children across its area automatically — four electronics on the top face become four compact pads at the corners, not any form of tower. If a validator tells you the body "needs to be a volumetric torso" or "boxier chassis", IGNORE IT — no preset in this palette implements a 3D body block, so pretending a vertical extrusion is one just produces a worse design.
13. **Cameras and lidars NEVER mount directly to a baseplate face.** Always interpose a `structural_bracket_l`: the bracket mates to the baseplate via `mate_connector: "wall_outer"` on the chosen face (front/back/left/right), then the sensor mates to the bracket via `attach_connector: "wall_inner"` (or `"plate_top"` for an upward-facing sensor) + `mate_connector: "mount_back"` + `mate_type: "fastened"`. See the L-bracket + camera example in the Mate Connectors section above for the exact fields. This rule is canonical — it holds across redesign cycles. If a validator says the camera is "floating", "not visibly mounted", or "offset from the baseplate edge", the fix is to ADD a bracket or reposition the existing bracket's mount face — NEVER move the sensor onto `attach_face: "top"` of the baseplate to "make it sit flat". A top-face baseplate mount with no bracket is the same bug in a different orientation.
14. **Match servo torque class to kinematic depth.** The further a joint is from the root, the less load it carries — scale down accordingly:
    - Hip/shoulder (root-adjacent, carries full limb weight): `actuator_servo_high_torque`
    - Knee/elbow (carries segment + distal chain): `actuator_servo_high_torque` or `actuator_servo_standard` depending on payload
    - Wrist/ankle/neck (effector-only load): `actuator_servo_standard`
    - Finger/fine manipulator: `actuator_servo_standard` or micro variant
    ❌ `actuator_servo_standard` at the hip/shoulder of a walking robot — it will stall under leg weight.
    ❌ `actuator_servo_high_torque` at a wrist for a lightweight gripper — unnecessary mass, no benefit.
15. **All series revolute joints (non-compound) require a structural limb link between them.** The limb link is the bone — its `length_mm` is the segment length. Prefer `structural_limb_link_slim` for thighs/shins/forearms; do not use clunky beams for sleek animals or humanoid limbs. Skipping the link produces zero-length limbs that collapse in sim.
    - ✅ `hip_pitch_servo → thigh_link_slim (100mm, fixed) → knee_servo → shin_link_slim (120mm, fixed) → foot`
    - ❌ `hip_pitch_servo → knee_servo` — zero-length thigh, robot collapses
    - ❌ `elbow_servo → wrist_servo` — zero-length forearm, arm folds flat
    The ONE exception: the compound 2-DOF hip/shoulder (two perpendicular-axis servos stacked directly). The engine auto-inserts a short bracket between them; you do not emit it.
16. `structural_limb_link_slim` mounts to servo horns on its broad flat face. Do NOT add `attach_rpy` or custom `orientation` to slim limb links to make them look flush; that rotates the bone itself and can make it attach edge-on. Emit slim links as fixed children on the correct face, with only `length_mm`. Put crouch/rest angles on the driving servo's `attach_rpy`, not on the passive limb link.

## Common Patterns (topology only -- no coordinates needed)

Arms: baseplate -> base_servo(top, revolute z) -> shoulder_servo(top, revolute y) -> upper_arm_extrusion(top, fixed, 200mm) -> elbow_servo(top, revolute y) -> forearm_extrusion(top, fixed, 150mm) -> wrist_servo(top, revolute y) -> gripper(top, fixed)
Note: arm extrusions go UPWARD from the base (vertical at rest). Joints control the angle. Do NOT use orientation="horizontal" for arm links.
Wrist camera: attach the camera to forearm_extrusion (front face), NOT to wrist_servo or gripper.

Wheeled base (differential drive / rover): baseplate -> 4x drivetrain_hub_motor_80(bottom, continuous y) -> 4x mobility_wheel_driven(coaxial, fixed). The drivetrain mounts on the baseplate bottom face; the tire attaches coaxially. The placement engine handles the axial offset (so the tire sits beside the motor, not inside it) and auto-flips drivetrains on one side of the baseplate so wheels land outboard on both sides — do not try to encode per-corner positions or rotations. Do NOT attach tires directly to the baseplate. Tires ALWAYS use attach_face="coaxial" when their parent is a drivetrain.

Caster: baseplate -> drivetrain_caster_swivel(bottom, fixed) -> mobility_wheel_driven(coaxial, fixed). Use for passive support points.

Rubber foot pad: `shin_link → mobility_rubber_foot_pad(bottom, fixed)`. ONE node — no parent drivetrain, no children. NEVER set attach_rpy on a foot pad; the assembly engine auto-levels it flat to the world floor. Do NOT use attach_face="coaxial".

Steered car (Ackermann): front pair uses baseplate -> drivetrain_steering_knuckle(bottom, revolute z) -> drivetrain_hub_motor_80(top, continuous y) -> mobility_wheel_driven(coaxial, fixed). Rear pair uses plain hub motors as above.

Mecanum base: baseplate -> 4x drivetrain_hub_motor_80(bottom, continuous y) -> 4x mobility_mecanum_wheel(coaxial, fixed), alternating handedness (FL/RR left-handed, FR/RL right-handed).

Vehicle vocabulary (all map to wheeled base above — DEFAULT 4 wheels):
- "car", "truck", "vehicle", "rover", "buggy", "cart" → 4 drivetrain_hub_motor_80 + 4 mobility_wheel_driven. NEVER emit a 2-wheel car; real cars have 4 wheels at corners. Only drop below 4 if the user explicitly says "two-wheeled" or "bike/motorcycle/unicycle".
- "6-wheeled rover" / "hexapod rover" / "Mars rover" → 6 wheels (backend distributes 2 rows × 3).
- "tank" / "tracked" → still use 4 hub motors + wheels (closest palette match); no track preset exists.
- Always add at least 1 sensor (camera on front face) and electronics (battery + SBC on top) for any vehicle request — a bare chassis with wheels is not a recognizable car.

Quadruped (canonical 12-DOF, Unitree Go1 / Boston Dynamics Spot style).

Anatomical joint order (IMPORTANT — joints drive the segment BELOW them, not above):
  body → hip_yaw → hip_pitch → THIGH → knee → SHIN → foot
                    ↑ compound hip ↑         ↑ knee joint drives shin, not thigh

  structural_baseplate_large  (use the large plate, 350×250×8mm — small plate is too narrow for a Go1-class hip span)
    -> 4x hip_yaw_servo (bottom, revolute z)                             — horn faces downward; swings the whole leg in the XY plane (compound hip axis 1)
      -> 4x hip_pitch_servo (bottom, revolute y, attach_rpy=[0, 0.52, 0]) — pitches THIGH forward ≈+30° for crouch (compound hip axis 2). PAIRED WITH KNEE attach_rpy — see crouch rule below.
        -> 4x thigh_link_slim (bottom, fixed, 100mm, vertical)            — sleek structural thigh bone (`structural_limb_link_slim`)
          -> 4x knee_servo (bottom, revolute y, attach_rpy=[0, 1.05, 0]) — pitches SHIN into the crouch using assembler-side mirrored horn signs. PAIRED WITH HIP_PITCH attach_rpy — see crouch rule below.
            -> 4x shin_link_slim (bottom, fixed, 120mm, vertical)         — sleek structural shin bone (`structural_limb_link_slim`)
              -> 4x mobility_rubber_foot_pad (bottom, fixed)               — ONE node, no attach_rpy, no children; auto-leveled by engine

- Total: 12 DOF (3 per leg × 4 legs). Each pitch servo drives the limb segment DIRECTLY BELOW it: hip_pitch rotates the thigh (and everything below), knee rotates the shin (and everything below). If you put the thigh between hip_yaw and hip_pitch, the hip_pitch rpy will bend the SHIN instead of the thigh — producing a broken scissor pose.
- The hip (yaw + pitch) is a compound 2-DOF joint at the body — the two servos stack directly. The port system auto-inserts a short bracket between them; you do not need to emit it. This is the ONE exception to "never stack servos directly."
- Structural limb links MUST appear between hip_pitch→knee (the thigh) and knee→foot (the shin). Use `structural_limb_link_slim` for dog legs; reserve T-slot extrusions for chassis/frame rails.
- For a rest "Z-shape crouch" / dog-stand stance, emit attach_rpy on hip_pitch (≈+0.52 rad / +30°) AND knee (≈+1.05 rad / 60° magnitude) **as a pair — both or neither, never one without the other.** Do not hand-mirror knee signs per side; the assembler converts knee bend magnitude into the correct local horn sign for mirrored left/right hardware. This is the #1 crouch-emission bug: emitting knee.attach_rpy while leaving hip_pitch.attach_rpy unset (or [0,0,0]) leaves thighs hanging vertical while shins rotate off them, producing a broken horizontal-splay pose where shins stick out sideways from the body instead of folding under it. If you set knee attach_rpy, you MUST also set hip_pitch attach_rpy with the matching crouch value. If you're not sure whether to emit them, emit BOTH — a full crouch is always better than a half-crouch.
- For a straight stance (neutral), omit attach_rpy from BOTH hip_pitch AND knee servos — never just one. If you find yourself setting attach_rpy on only one of the two, stop: that always produces a broken pose. The two values travel together.
- A simpler 8-DOF variant (no planar hip sweep) is acceptable if the user asks for "simple" or "cheap": baseplate -> 4x hip_pitch_servo -> thigh_link_slim -> knee_servo -> shin_link_slim -> mobility_rubber_foot_pad(bottom, fixed). Do not substitute `structural_extrusion_2020` for these dog leg links.

Head/camera for robot dogs: attach a fixed depth camera directly to the baseplate front face. Do NOT add a neck servo, head servo, head bracket, or head limb unless the user explicitly asks for an articulated head/neck. For humanoids or explicitly articulated heads only: baseplate -> neck_servo(front, revolute y) -> head_bracket(top, fixed) -> camera(front, fixed).

Do NOT add a tail to quadrupeds unless the user explicitly asks for one. A default robot dog should spend parts/mass on legs, body electronics, and an optional head/camera, not a cosmetic tail.

Sensor mount: any_structural_link -> sensor(top/front/left/right, fixed). Remember — sensors attach to structural links (extrusions, baseplates, brackets), never to servo shafts or effectors.
Angled sensor: any_link -> depth_camera(front, fixed, elevation_angle=-20) — tilts 20° downward to see the floor.
Rotated top sensor: any_link -> lidar(top, fixed, orientation="45") — yaws 45° on the top face.

## Forbidden Patterns (these WILL be rejected by the placement engine)

- ❌ Sensor attached to another sensor — sensors must attach to structural or actuator links
- ❌ End effector with children — effectors (grippers, suction cups) are ALWAYS terminal nodes
- ❌ Multiple children on a servo/motor shaft — each servo/motor output drives exactly ONE child
- ❌ Duplicate link_name values — every link_name must be unique
- ❌ Multiple root components — exactly one component has attach_to=null (the baseplate)
- ❌ Cycles in the topology — A→B→C→A is invalid; the topology must be a tree
- ❌ Extrusion as root — root is always structural_baseplate
- ❌ `structural_extrusion_2020` / `structural_extrusion_4040` as dog or quadruped thigh/shin bones — use `structural_limb_link_slim` for every leg segment.
- ❌ Electronics on an extrusion standoff above the baseplate — any of these shapes:
    · `baseplate → 4x vertical structural_extrusion_4040 → each hosts one of (battery, PDU, SBC, IMU)` (the 4-standoff case)
    · `baseplate → 1x vertical structural_extrusion_4040 → [battery, PDU, SBC, IMU all on its top face]` (the central-tower case — also wrong, don't interpret "no 4 standoffs" as "1 standoff is fine")
    · `baseplate → torso_extrusion → electronics` — any named-as-torso intermediate is still a tower
  Attach every electronic module DIRECTLY to `baseplate.top`. The placement engine spreads multiple children across the face automatically.
- ❌ Two `attach_to` entries pointing at the same servo link_name — the horn drives exactly ONE child. The servo backend has `_body` and `_horn` links internally, but you must never name them; use only the bare servo link_name. One servo → one child, always.
- ❌ Driven child (joint_type != "fixed") attached to a servo side face — side faces are the housing ears (bolted to parent structure), not the horn output. Driven children must use attach_face="top" or "bottom" to align with the horn output direction.
- ❌ Series revolute joints with no structural extrusion between them (except the compound hip/shoulder) — zero-length limbs collapse in sim.
- ❌ mobility_rubber_foot_pad with attach_rpy set — the assembly engine auto-levels foot pads to the world floor; setting attach_rpy bypasses leveling and leaves the pad sideways. Never set attach_rpy on a foot pad.
- ❌ mobility_rubber_foot_pad with children — foot pads are terminal leaf nodes. Never attach anything to a foot pad.
- ❌ mobility_rubber_foot_pad with a drivetrain parent — foot pads attach directly to structural links (shin, extrusion), not to hub motors or drivetrain components.

## Critical Rules

- ALWAYS use design_robot or modify_topology. NEVER write raw URDF.
- Use modify_topology when the user wants to change an existing robot. Use design_robot only for new builds or complete redesigns.
- Use component IDs exactly as listed in the library.
- The backend handles ALL geometry. You handle ALL design decisions.
"""

# ── Tool schemas for structured output ──────────────────────────────────────

DESIGN_ROBOT_TOOL = {
    "name": "design_robot",
    "description": "Design a new robot by specifying the full component topology. The placement engine handles all 3D positioning.",
    "input_schema": {
        "type": "object",
        "properties": {
            "explanation": {
                "type": "string",
                "description": "Describe what you're building and why",
            },
            "base_link": {
                "type": "string",
                "description": "Name of the root link (usually 'structural_baseplate_1')",
            },
            "components": {
                "type": "array",
                "description": "List of components in dependency order (root first)",
                "items": {
                    "type": "object",
                    "properties": {
                        "link_name": {"type": "string", "description": "Unique name: component_id + number (e.g., 'actuator_servo_high_torque_1')"},
                        "component_id": {"type": "string", "description": "Exact ID from the component library"},
                        "attach_to": {"type": ["string", "null"], "description": "Parent's link_name, or null for root"},
                        "attach_face": {"type": "string", "enum": ["top", "bottom", "front", "back", "left", "right"]},
                        "joint_type": {"type": "string", "enum": ["fixed", "revolute", "prismatic", "continuous"]},
                        "joint_axis": {"type": "string", "enum": ["x", "y", "z"]},
                        "length_mm": {"type": "number", "description": "Override length for parametric structural links (default 100mm). Use structural_limb_link_slim at 80-120 for leg segments; use 150-300 for arm links."},
                        "orientation": {"type": "string", "description": "Rotation within the face. Keywords: 'vertical' (default, extend +Z), 'horizontal' (extend +X), 'auto'. Or a numeric string in degrees for yaw around the face normal (e.g. '45', '-30'). Combine keyword+degrees as 'horizontal+45'."},
                        "elevation_angle": {"type": "number", "description": "Tilt in degrees for side-face attachments (front/back/left/right only). Positive=up, negative=down. E.g. -20 angles a front camera 20° downward. Ignored on top/bottom faces."},
                        "attach_rpy": {
                            "type": "array",
                            "items": {"type": "number"},
                            "minItems": 3,
                            "maxItems": 3,
                            "description": "Optional [roll, pitch, yaw] in RADIANS applied to the joint origin. Use for rest-pose joint angles (quadruped crouch, splayed shoulders). Example: [0, 0.52, 0] for +30° pitch, [0, -1.05, 0] for -60° pitch. Omit or pass [0,0,0] to let the engine auto-rotate.",
                        },
                        "attach_connector": {
                            "type": "string",
                            "description": "Optional named connector id on the PARENT — e.g. 'shaft_out' on a servo, 'plate_top'/'wall_inner'/'wall_outer' on an L-bracket. Use to disambiguate surfaces when the part authors named connectors (see `conn=[…]` in the catalog). Omit to fall back to `attach_face`.",
                        },
                        "mate_connector": {
                            "type": "string",
                            "description": "Optional named connector id on the CHILD — e.g. 'shaft_hole' on a coupler/horn, 'mount_back' on a camera. Pair with `attach_connector` to make the placement explicit. Omit to let the engine pick the opposite-face default.",
                        },
                        "mate_type": {
                            "type": "string",
                            "enum": ["fastened", "planar", "concentric"],
                            "description": "Mate semantics. 'fastened' = rigid face-to-face weld (default when connectors are named). 'concentric' = shaft-in-hole (servo shaft_out ↔ coupler shaft_hole); antiparallel axes, axial slide free. 'planar' = face-flush with in-plane offset. Omit unless emitting a concentric shaft mate.",
                        },
                    },
                    "required": ["link_name", "component_id", "attach_to", "attach_face", "joint_type", "joint_axis"],
                },
            },
            "changes_summary": {
                "type": "string",
                "description": "Brief summary: N components, M DOF",
            },
        },
        "required": ["explanation", "base_link", "components", "changes_summary"],
    },
}

MODIFY_TOPOLOGY_TOOL = {
    "name": "modify_topology",
    "description": "Modify an existing robot's topology by adding, removing, or changing components. Use for iterative edits like 'add a camera', 'remove the tail', 'make the arms longer'. The placement engine re-resolves the full assembly after applying changes.",
    "input_schema": {
        "type": "object",
        "properties": {
            "explanation": {
                "type": "string",
                "description": "Describe what you're changing and why",
            },
            "operations": {
                "type": "array",
                "description": "List of topology operations to apply in order",
                "items": {
                    "type": "object",
                    "properties": {
                        "op": {
                            "type": "string",
                            "enum": ["add", "remove", "modify"],
                            "description": "add: insert a new component (MUST include component_id, attach_to, attach_face, joint_type, joint_axis), remove: delete an existing component and all its children, modify: change properties of an existing component (only include fields to change)",
                        },
                        "link_name": {
                            "type": "string",
                            "description": "For remove/modify: the existing link_name to target. For add: the new unique link_name (use component_id + number, e.g. 'sensor_depth_camera_small_2').",
                        },
                        "component_id": {
                            "type": "string",
                            "description": "REQUIRED for add. Component ID from the library. For modify: new component_id (omit to keep current).",
                        },
                        "attach_to": {
                            "type": ["string", "null"],
                            "description": "REQUIRED for add. Parent's link_name. For modify: new parent (omit to keep current).",
                        },
                        "attach_face": {
                            "type": "string",
                            "enum": ["top", "bottom", "front", "back", "left", "right"],
                            "description": "REQUIRED for add. Face on parent to attach to. For modify: new face (omit to keep current).",
                        },
                        "joint_type": {
                            "type": "string",
                            "enum": ["fixed", "revolute", "prismatic"],
                        },
                        "joint_axis": {
                            "type": "string",
                            "enum": ["x", "y", "z"],
                        },
                        "length_mm": {
                            "type": "number",
                            "description": "Override length for extrusions.",
                        },
                        "orientation": {
                            "type": "string",
                            "description": "Rotation within the face (same as design_robot).",
                        },
                        "elevation_angle": {
                            "type": "number",
                            "description": "Tilt for side-face attachments.",
                        },
                        "attach_rpy": {
                            "type": "array",
                            "items": {"type": "number"},
                            "minItems": 3,
                            "maxItems": 3,
                            "description": "Optional [roll, pitch, yaw] in RADIANS. Same as design_robot — use for rest-pose joint angles like quadruped crouch.",
                        },
                        "attach_connector": {
                            "type": "string",
                            "description": "Optional parent-side connector id (e.g. 'shaft_out', 'plate_top'). Same semantics as design_robot.",
                        },
                        "mate_connector": {
                            "type": "string",
                            "description": "Optional child-side connector id (e.g. 'shaft_hole', 'mount_back'). Same semantics as design_robot.",
                        },
                        "mate_type": {
                            "type": "string",
                            "enum": ["fastened", "planar", "concentric"],
                            "description": "Mate type: 'fastened' | 'planar' | 'concentric'. Same semantics as design_robot.",
                        },
                    },
                    "required": ["op", "link_name"],
                },
            },
            "changes_summary": {
                "type": "string",
                "description": "Brief summary: what was added/removed/changed",
            },
        },
        "required": ["explanation", "operations", "changes_summary"],
    },
}

ROBOT_TOOLS = [DESIGN_ROBOT_TOOL, MODIFY_TOPOLOGY_TOOL]


# ── Workstream #2: Tool-Call Edit Surface ───────────────────────────────────
#
# EDIT_TOOLS = typed, link-level graph mutations driven by Anthropic's native
# tool-use protocol. Unlike `modify_topology` (which emits N operations in a
# single shot with validation deferred to the end), each EDIT_TOOLS call gets
# validated *inline* by the TS validator — invalid mutations are rejected
# before they enter the graph and the error feeds back to the same Claude turn
# for self-correction. The orchestrator is in `viewportChat.ts`; Python's only
# job here is defining the schemas and relaying a single turn.
#
# Coarse link-level granularity is deliberate: field-level tools inflate token
# count and turn the model into a key-value setter. Five tools cover ~90% of
# edit intents:
#   add_link          — new component at (parent, face)
#   attach_sensor     — sensor preset on a structural/actuator parent
#   replace_component — swap preset_id, preserve topology
#   set_joint         — change joint type/axis/rest-pose rpy in place
#   remove_link       — delete subtree or graft children up

ADD_LINK_TOOL = {
    "name": "add_link",
    "description": (
        "Add a new component to the existing robot. Use for structural parts, actuators, and "
        "effectors. For sensors, prefer attach_sensor (it enforces the fixed-joint convention). "
        "The mutation is validated immediately; if the attach would trip port-class incompatibility "
        "(shaft↔mount_face) or SHAFT_FANOUT/SENSOR_ON_ACTUATOR, you get a structured error and can "
        "retry in the same turn."
    ),
    "input_schema": {
        "type": "object",
        "properties": {
            "link_name": {"type": "string", "description": "Unique new link_name (convention: component_id + N, e.g. 'structural_bracket_u_2')."},
            "parent_link": {"type": "string", "description": "Existing link_name to attach to."},
            "preset_id": {"type": "string", "description": "Component ID from the library (e.g. 'actuator_servo_standard')."},
            "attach_face": {"type": "string", "enum": ["top", "bottom", "front", "back", "left", "right"]},
            "joint_type": {"type": "string", "enum": ["fixed", "revolute", "prismatic", "continuous"], "description": "Default: 'fixed'."},
            "joint_axis": {"type": "string", "enum": ["x", "y", "z"], "description": "Default: 'z'."},
            "length_mm": {"type": "number", "description": "Extrusion length override."},
            "orientation": {"type": "string", "description": "'vertical' (default) | 'horizontal' | 'auto' | numeric degrees for yaw."},
            "elevation_angle": {"type": "number", "description": "Side-face tilt in degrees (positive=up). Ignored on top/bottom."},
            "attach_rpy": {
                "type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
                "description": "Rest-pose [roll, pitch, yaw] in radians (e.g. quadruped crouch [0, 0.52, 0]). Omit for auto.",
            },
            "attach_connector": {
                "type": "string",
                "description": "Optional parent-side named connector (e.g. 'shaft_out', 'plate_top'). See the 'Mate Connectors' section of the main system prompt — pair with mate_connector + mate_type='concentric' for shaft mounts.",
            },
            "mate_connector": {
                "type": "string",
                "description": "Optional child-side named connector (e.g. 'shaft_hole', 'mount_back'). Omit to let the engine pick the default opposite face.",
            },
            "mate_type": {
                "type": "string",
                "enum": ["fastened", "planar", "concentric"],
                "description": "Mate type. Use 'concentric' for shaft-in-hole mates. Omit for ordinary face-to-face mounts.",
            },
        },
        "required": ["link_name", "parent_link", "preset_id", "attach_face"],
    },
}

ATTACH_SENSOR_TOOL = {
    "name": "attach_sensor",
    "description": (
        "Attach a sensor_* preset to a structural/actuator parent. Joint is forced to 'fixed' so "
        "the sensor frame stays stable as the robot articulates. Attaching a sensor directly to an "
        "actuator's shaft face returns SENSOR_ON_ACTUATOR — mount on a nearby structural extrusion."
    ),
    "input_schema": {
        "type": "object",
        "properties": {
            "link_name": {"type": "string", "description": "Unique new link_name (convention: sensor_<kind>_N)."},
            "parent_link": {"type": "string"},
            "preset_id": {"type": "string", "description": "Must start with 'sensor_' (e.g. 'sensor_depth_camera_small')."},
            "mount_face": {"type": "string", "enum": ["top", "bottom", "front", "back", "left", "right"]},
            "elevation_angle": {"type": "number"},
        },
        "required": ["link_name", "parent_link", "preset_id", "mount_face"],
    },
}

REPLACE_COMPONENT_TOOL = {
    "name": "replace_component",
    "description": (
        "Swap the preset on an existing link while preserving its attach_to / attach_face / children. "
        "Use for 'change the gripper to a suction cup' or 'make this servo the high-torque variant'. "
        "If the new preset's port class doesn't mate with the parent face, returns PORT_MISMATCH."
    ),
    "input_schema": {
        "type": "object",
        "properties": {
            "link_name": {"type": "string", "description": "The existing link to modify."},
            "new_preset_id": {"type": "string", "description": "Replacement component_id from the library."},
        },
        "required": ["link_name", "new_preset_id"],
    },
}

SET_JOINT_TOOL = {
    "name": "set_joint",
    "description": (
        "Change an existing link's joint type, axis, or rest-pose rpy without touching topology. "
        "Use for 'make the elbow revolute around y', 'angle the hip joint 30° forward at rest'."
    ),
    "input_schema": {
        "type": "object",
        "properties": {
            "link_name": {"type": "string"},
            "joint_type": {"type": "string", "enum": ["fixed", "revolute", "prismatic", "continuous"]},
            "joint_axis": {"type": "string", "enum": ["x", "y", "z"], "description": "Omit to keep current."},
            "attach_rpy": {
                "type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3,
                "description": "Omit to keep current. Use for rest-pose joint angles in radians.",
            },
        },
        "required": ["link_name", "joint_type"],
    },
}

REMOVE_LINK_TOOL = {
    "name": "remove_link",
    "description": (
        "Remove a link. By default the whole subtree goes with it (same as modify_topology's "
        "remove). Pass reparent_children=true to graft direct children onto the removed link's "
        "parent — useful when yanking a redundant structural intermediate."
    ),
    "input_schema": {
        "type": "object",
        "properties": {
            "link_name": {"type": "string"},
            "reparent_children": {
                "type": "boolean",
                "description": "Default: false (cascade delete). True: graft direct children onto the removed link's parent.",
            },
        },
        "required": ["link_name"],
    },
}

EDIT_TOOLS = [
    ADD_LINK_TOOL,
    ATTACH_SENSOR_TOOL,
    REPLACE_COMPONENT_TOOL,
    SET_JOINT_TOOL,
    REMOVE_LINK_TOOL,
]

# Canonical tool names — kept in one place so the frontend dispatcher and the
# Python turn-loop both agree on what counts as an "edit tool" vs. the
# legacy design_robot / modify_topology paths.
EDIT_TOOL_NAMES = {t["name"] for t in EDIT_TOOLS}

# Per-session buffer for the multi-turn tool-use message history. Separate from
# `_conversation_history` (which is the summarized long-term chat) because the
# tool loop needs the raw tool_use / tool_result blocks intact between turns —
# once the loop ends, a summary entry is pushed to _conversation_history so
# subsequent non-edit calls see "[Used add_link] …" instead of the raw blocks.
_edit_tool_sessions: dict[str, dict] = defaultdict(dict)

# Safety cap on tool-loop rounds. Ten covers realistic self-correction chains
# (bad attach → retry with bracket → validator warn → done) without letting a
# confused model run up a bill. viewportChat also caps this independently.
_MAX_EDIT_TOOL_ROUNDS = 10

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

    # Auto-prepend a baseplate if the root component isn't a plate-shaped component
    if components:
        root_comp = components[0]
        root_preset = get_component(root_comp.get("component_id", ""))
        if root_preset:
            root_bbox = root_preset.get("physical", {}).get("bounding_box_mm", [50, 50, 50])
            # Check if root is plate-shaped (one dim much smaller than others)
            if root_bbox and len(root_bbox) >= 3:
                sorted_bb = sorted(root_bbox)
                is_plate = sorted_bb[0] < sorted_bb[1] * 0.2  # thinnest dim < 20% of middle
            else:
                is_plate = False
            if not is_plate:
                print(f"[assembly] Root '{root_comp['component_id']}' is not a plate — auto-prepending baseplate", file=sys.stderr)
                bp_name = "structural_baseplate_auto"
                baseplate_comp = {
                    "link_name": bp_name,
                    "component_id": "structural_baseplate",
                    "attach_to": None,
                    "attach_face": None,
                    "joint_type": "fixed",
                    "joint_axis": "z",
                }
                # Re-parent the original root onto the baseplate
                root_comp["attach_to"] = bp_name
                root_comp["attach_face"] = root_comp.get("attach_face") or "top"
                components.insert(0, baseplate_comp)
                base_link = bp_name
                assembly["ground_offset"] = True

    def _component_is_split_servo_id(component_id: str) -> bool:
        return (
            component_id.startswith("actuator_servo")
            or component_id.startswith("actuator_continuous_rotation_servo")
            or component_id.startswith("actuator_high_speed")
        )

    # Old plans sometimes contain couplers/brackets between servo joints. With
    # split servos, those extra spacers rotate the next servo frame again or
    # make the next limb coaxial with the shaft. Remove them so children mount
    # to the driven horn adapter radially.
    for comp in list(components):
        parent = next((c for c in components if c.get("link_name") == comp.get("attach_to")), None)
        children = [c for c in components if c.get("attach_to") == comp.get("link_name")]
        cid = comp.get("component_id", "")
        is_servo_coupler = cid == "structural_servo_coupler_disc"
        is_servo_to_servo_bracket = (
            cid.startswith("structural_bracket_")
            and parent is not None
            and _component_is_split_servo_id(parent.get("component_id", ""))
            and any(_component_is_split_servo_id(c.get("component_id", "")) for c in children)
        )
        if not is_servo_coupler and not is_servo_to_servo_bracket:
            continue
        touches_servo = (
            (parent is not None and _component_is_split_servo_id(parent.get("component_id", ""))) or
            any(_component_is_split_servo_id(c.get("component_id", "")) for c in children)
        )
        if not touches_servo:
            continue
        for child in children:
            child["attach_to"] = comp.get("attach_to")
            if not child.get("attach_face") and comp.get("attach_face"):
                child["attach_face"] = comp.get("attach_face")
        components.remove(comp)
        print(f"[assembly] removed obsolete servo spacer {comp.get('link_name')}", file=sys.stderr)

    # Claude occasionally backslides to 2020/4040 T-slot for dog thigh/shin
    # bones even though the canonical quadruped pattern uses slim limb links.
    # Coerce only bottom-mounted actuator/foot chains so normal chassis rails
    # and frames still keep their extrusion presets.
    for comp in components:
        cid = comp.get("component_id", "")
        if not isinstance(cid, str) or not cid.startswith("structural_extrusion_"):
            continue
        parent = next((c for c in components if c.get("link_name") == comp.get("attach_to")), None)
        children = [c for c in components if c.get("attach_to") == comp.get("link_name")]
        parent_is_pitch_servo_leg = (
            parent is not None
            and _component_is_split_servo_id(parent.get("component_id", ""))
            and comp.get("attach_face") == "bottom"
        )
        child_is_leg_terminal = any(
            _component_is_split_servo_id(c.get("component_id", "")) or c.get("component_id") == "mobility_rubber_foot_pad"
            for c in children
        )
        if parent_is_pitch_servo_leg and child_is_leg_terminal:
            comp["component_id"] = "structural_limb_link_slim"
            print(f"[assembly] coerced dog leg beam {comp.get('link_name')} from {cid} to structural_limb_link_slim", file=sys.stderr)

    # The prompt forbids default cosmetic tails on quadrupeds, but the model can
    # still emit a small rear servo + limb chain. Strip non-functional rear
    # chains deterministically while preserving real legs, sensors, and payloads.
    def _remove_subtree(root_name: str) -> None:
        pending = [root_name]
        remove_names = set()
        while pending:
            name = pending.pop()
            if name in remove_names:
                continue
            remove_names.add(name)
            pending.extend(
                c.get("link_name")
                for c in components
                if c.get("attach_to") == name and c.get("link_name")
            )
        components[:] = [c for c in components if c.get("link_name") not in remove_names]

    def _subtree_component_ids(root_name: str) -> set:
        pending = [root_name]
        seen = set()
        ids = set()
        while pending:
            name = pending.pop()
            if name in seen:
                continue
            seen.add(name)
            comp = next((c for c in components if c.get("link_name") == name), None)
            if comp is not None:
                ids.add(comp.get("component_id", ""))
            pending.extend(
                c.get("link_name")
                for c in components
                if c.get("attach_to") == name and c.get("link_name")
            )
        return ids

    is_quadruped = sum(1 for c in components if c.get("component_id") == "mobility_rubber_foot_pad") >= 4
    if is_quadruped:
        base_names = {
            c.get("link_name")
            for c in components
            if c.get("component_id", "").startswith("structural_baseplate")
        }
        for comp in list(components):
            link_name = comp.get("link_name", "")
            cid = comp.get("component_id", "")
            if comp.get("attach_to") not in base_names:
                continue
            if comp.get("attach_face") != "back" and "tail" not in link_name.lower():
                continue
            subtree_ids = _subtree_component_ids(link_name)
            has_functional_terminal = any(
                sid == "mobility_rubber_foot_pad"
                or sid.startswith("sensor_")
                or sid.startswith("compute_")
                or sid.startswith("power_")
                for sid in subtree_ids
            )
            tail_like = (
                cid.startswith("actuator_servo")
                or cid.startswith("actuator_high_speed")
                or cid == "structural_limb_link_slim"
                or "tail" in link_name.lower()
            )
            if tail_like and not has_functional_terminal:
                _remove_subtree(link_name)
                print(f"[assembly] removed default quadruped tail chain {link_name}", file=sys.stderr)

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
    def _get_bbox_m(preset, comp=None):
        phys = preset.get("physical", {})
        bb = phys.get("bounding_box_mm")
        if bb and len(bb) >= 3:
            return [b / 1000.0 for b in bb]
        # Handle parametric structural links: cross_section_mm + per-component length.
        cs = phys.get("cross_section_mm")
        if cs and len(cs) >= 2:
            length_mm = 100
            if isinstance(comp, dict):
                length_mm = float(comp.get("length_mm") or 100)
            return [cs[0] / 1000.0, cs[1] / 1000.0, length_mm / 1000.0]
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

    def _is_distal_beam_component_id(component_id):
        return (
            component_id == "structural_limb_link_slim" or
            (isinstance(component_id, str) and component_id.startswith("structural_extrusion_"))
        )

    def _compute_origin_and_rpy(parent_preset, child_preset, attach_face, explicit_rpy=None, parent_comp=None, child_comp=None):
        """Compute joint origin xyz AND rpy based on parent/child bounding boxes and face.

        For elongated children (extrusions) attaching to 'top', auto-rotates them
        to extend horizontally along +X instead of stacking vertically.
        """
        import math
        p_bbox = _get_bbox_m(parent_preset, parent_comp)
        c_bbox = _get_bbox_m(child_preset, child_comp)

        # Hub-motor -> tire: axial mount along drivetrain-local +Z.
        # After the drivetrain's -pi/2 X-roll on the baseplate bottom,
        # drivetrain-local +Z maps to world +Y (outboard). Offsetting the tire
        # by (motor_hz + tire_half_axle) along +Z seats the tire's inboard bore
        # face flush against the motor's outboard end. Do NOT use p_bbox[1]/2
        # (the motor radius / local-Y half-extent) — that direction is world -Z
        # (downward) after the roll, which places the tire below the motor
        # rather than beside it. Outboard direction for 4-wheel vehicles is
        # handled by yawing drivetrains on the -Y half 180 deg (bottom-face branch).
        child_id = (child_preset or {}).get("id", "")
        parent_id = (parent_preset or {}).get("id", "")
        _is_tire_child = _is_tire_component_id(child_id)
        _is_drivetrain_parent = _is_drivetrain_component_id(parent_id)
        if _is_tire_child and _is_drivetrain_parent:
            motor_hz = p_bbox[2] / 2  # axle half-length along drivetrain local Z
            tire_half_axle = c_bbox[2] / 2  # tire half-width along axle
            dz = motor_hz + tire_half_axle
            print(f"[assembly] Axial hub mount: {child_id} on {parent_id}, dz={dz:.4f}", file=sys.stderr)
            return [0, 0, dz], [0, 0, 0]

        # Use explicit rpy if provided and non-zero
        if explicit_rpy and any(abs(v) > 0.001 for v in explicit_rpy):
            rpy = explicit_rpy
        else:
            rpy = [0, 0, 0]
            # Auto-rotate: elongated child on "top" face -> pitch 90° to extend along +X
            if attach_face in ("top", "coaxial") and _is_elongated(c_bbox):
                rpy = [0, math.pi/2, 0]  # pitch 90°
                print(f"[assembly] Auto-rotating elongated child to horizontal (pitch 90°)", file=sys.stderr)
            # Auto-roll: drivetrain assemblies on the "bottom" face need -90° roll
            # so their output shaft lies along Y (standard ROS convention).
            # Tires (mobility_wheel_*) attach fixed to the drivetrain and inherit
            # the orientation; they do not need their own roll correction.
            child_id = (child_preset or {}).get("id", "")
            _is_drivetrain = _is_drivetrain_component_id(child_id)
            if _is_drivetrain and attach_face == "bottom":
                rpy = [-math.pi / 2, 0, 0]
                print(f"[assembly] Auto-rolling {child_id} -90° for bottom-face drivetrain mount", file=sys.stderr)

        # Half-extents (geometry is always centered at link frame origin)
        px, py, pz = p_bbox[0]/2, p_bbox[1]/2, p_bbox[2]/2
        cx, cy, cz = c_bbox[0]/2, c_bbox[1]/2, c_bbox[2]/2

        child_is_rod = _is_elongated(c_bbox)
        # Rotation-aware extents: a ±90° pitch swings X onto Z; a ±90° roll
        # swings Y onto Z. Without this, sideways cylinders (wheels, rollers,
        # horizontal bearings) get placed using their pre-rotation thickness
        # instead of their post-rotation radius and clip into their parent.
        RIGHT = math.pi / 2
        is_pitch_rotated = abs(abs(rpy[1]) - RIGHT) < 0.1
        is_roll_rotated = abs(abs(rpy[0]) - RIGHT) < 0.1

        if child_is_rod:
            # Rod geometry is offset in local +Z, so it extends forward from the joint.
            # The joint only needs to clear the rod's cross-section, not half its length.
            cx_eff = cx
            cz_eff = cx  # cross-section, not half-length (true regardless of rotation)
        elif is_pitch_rotated:
            cx_eff, cz_eff = cz, cx  # ±90° pitch: old Z → X, old X → Z
        elif is_roll_rotated:
            cx_eff, cz_eff = cx, cy  # ±90° roll: old Y → Z (X unchanged)
            # Drivetrain hub motors carry an assembled tire; use tire outer radius for clearance
            if _is_drivetrain_component_id(child_id) and child_preset:
                ml = child_preset.get("mounting_logic", {})
                aor = ml.get("assembled_outer_radius_mm")
                if aor:
                    cz_eff = aor / 1000.0
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
            "coaxial": [0, 0, 0],
        }
        xyz = face_offsets.get(attach_face, [0, 0, pz + cz_eff])
        return xyz, rpy

    # Generate URDF XML
    import xml.etree.ElementTree as ET
    import math

    SERVO_HORN_Z_RATIO = 0.44  # horn joint sits 44% up the servo height (URDF Z-up)
    DEFAULT_REVOLUTE_LIMIT_RAD = math.pi / 2  # ±90° fallback for hobby servos when preset omits limits

    def _resolve_joint_limits_rad(preset: dict) -> tuple:
        """Return (lower, upper) in radians for a revolute/prismatic joint.

        Order of precedence:
        1. sim_metadata.mjcf_joint_limits_deg (authoring source of truth, in degrees)
        2. mechanical_electrical.angle_range_deg (legacy/alt name)
        3. ±DEFAULT_REVOLUTE_LIMIT_RAD fallback
        Robot-agnostic: any preset can opt in by adding the field; nothing else changes.
        """
        sim = preset.get("sim_metadata", {}) or {}
        me = preset.get("mechanical_electrical", {}) or {}
        deg = sim.get("mjcf_joint_limits_deg") or me.get("angle_range_deg")
        if isinstance(deg, list) and len(deg) == 2:
            try:
                lo = math.radians(float(deg[0]))
                hi = math.radians(float(deg[1]))
                if hi > lo:
                    return lo, hi
            except (TypeError, ValueError):
                pass
        return -DEFAULT_REVOLUTE_LIMIT_RAD, DEFAULT_REVOLUTE_LIMIT_RAD

    def _is_split_servo_component_id(component_id: str) -> bool:
        return (
            component_id.startswith("actuator_servo")
            or component_id.startswith("actuator_continuous_rotation_servo")
            or component_id.startswith("actuator_high_speed")
        )

    # Identify rotary servo components for split-link emit
    servo_link_names: set = set()
    servo_axis_names: dict = {}
    for comp in components:
        p = comp_lookup[comp["link_name"]]["preset"]
        pid = p.get("id", "")
        if _is_split_servo_component_id(pid):
            servo_link_names.add(comp["link_name"])

    def _effective_parent(raw_attach_to: str) -> str:
        """Return the actual URDF parent link name (remaps servo links to _horn)."""
        if raw_attach_to in servo_link_names:
            return raw_attach_to + "_horn"
        return raw_attach_to

    def _axis_name(raw_axis) -> str:
        if isinstance(raw_axis, str):
            v = raw_axis.lower()
            return v if v in ("x", "y", "z") else "z"
        if isinstance(raw_axis, list) and len(raw_axis) == 3:
            vals = [abs(float(v or 0)) for v in raw_axis]
            return ("x", "y", "z")[vals.index(max(vals))]
        return "z"

    for comp in components:
        link_name = comp.get("link_name")
        parent_name = comp.get("attach_to")
        parent_info = comp_lookup.get(parent_name) if parent_name else None
        parent_id = (parent_info or {}).get("component_id", "")
        has_servo_child = any(
            child.get("attach_to") == link_name and child.get("link_name") in servo_link_names
            for child in components
        )
        axis_name = _axis_name(comp.get("joint_axis", "z"))
        is_compound_hip_base_servo = (
            link_name in servo_link_names
            and axis_name != "z"
            and comp.get("attach_face") in ("top", "bottom")
            and parent_id.startswith("structural_baseplate")
            and has_servo_child
        )
        if is_compound_hip_base_servo:
            print(f"[assembly] planar hip servo axis normalized: {link_name} {comp.get('joint_axis', 'z')} -> z so horn faces down/up normal to the baseplate", file=sys.stderr)
            comp["joint_axis"] = "z"
            if link_name in comp_lookup:
                comp_lookup[link_name]["joint_axis"] = "z"

    for comp in components:
        if comp["link_name"] in servo_link_names:
            servo_axis_names[comp["link_name"]] = _axis_name(comp.get("joint_axis", "z"))

    def _servo_shaft_align_rpy(axis_name: str) -> list:
        if axis_name == "x":
            return [0, math.pi / 2, 0]
        if axis_name == "y":
            return [-math.pi / 2, 0, 0]
        return [0, 0, 0]

    def _matmul3(a: list, b: list) -> list:
        return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

    def _matvec3(m: list, v: list) -> list:
        return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]

    def _vadd3(a: list, b: list) -> list:
        return [a[i] + b[i] for i in range(3)]

    def _transpose3(a: list) -> list:
        return [[a[j][i] for j in range(3)] for i in range(3)]

    def _rpy_to_mat(rpy: list) -> list:
        roll, pitch, yaw = [float(v or 0) for v in rpy]
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        # Match the frontend assembler's Three.js Euler XYZ convention.
        return [
            [cp * cy, -cp * sy, sp],
            [sr * sp * cy + cr * sy, -sr * sp * sy + cr * cy, -sr * cp],
            [-cr * sp * cy + sr * sy, cr * sp * sy + sr * cy, cr * cp],
        ]

    def _mat_to_rpy(m: list) -> list:
        pitch = math.asin(max(-1.0, min(1.0, m[0][2])))
        cp = math.cos(pitch)
        if abs(cp) > 1e-8:
            roll = math.atan2(-m[1][2], m[2][2])
            yaw = math.atan2(-m[0][1], m[0][0])
        else:
            roll = math.atan2(m[2][1], m[1][1])
            yaw = 0.0
        return [roll, pitch, yaw]

    def _servo_desired_world_rot(axis_name: str, axis_sign: int = 1) -> list:
        if axis_name == "y" and axis_sign < 0:
            # Mirror the physical shaft onto world -Y while keeping local +Y as
            # the radial-down zero for leg chains.
            return [
                [-1, 0, 0],
                [0, 0, -1],
                [0, -1, 0],
            ]
        return _rpy_to_mat(_servo_shaft_align_rpy(axis_name))

    def _servo_mount_rpy_for_parent(parent_link_name: str, axis_name: str) -> list:
        if axis_name == "z":
            return None
        parent_rot = link_world_rot.get(parent_link_name, _rpy_to_mat([0, 0, 0]))
        parent_pos = link_world_pos.get(parent_link_name, [0, 0, 0])
        axis_sign = -1 if axis_name == "y" and parent_pos[1] < -1e-6 else 1
        desired_world = _servo_desired_world_rot(axis_name, axis_sign)
        return _mat_to_rpy(_matmul3(_transpose3(parent_rot), desired_world))

    def _servo_planar_mount_rpy_for_parent(parent_link_name: str, attach_face: str, computed_rpy: list) -> list:
        if attach_face not in ("top", "bottom"):
            return computed_rpy
        yaw = float(computed_rpy[2] or 0)
        desired_world = _rpy_to_mat([math.pi, 0, yaw] if attach_face == "bottom" else [0, 0, yaw])
        parent_rot = link_world_rot.get(parent_link_name, _rpy_to_mat([0, 0, 0]))
        return _mat_to_rpy(_matmul3(_transpose3(parent_rot), desired_world))

    def _world_level_rpy_for_parent(parent_link_name: str) -> list:
        parent_rot = link_world_rot.get(parent_link_name, _rpy_to_mat([0, 0, 0]))
        return _mat_to_rpy(_transpose3(parent_rot))

    def _servo_axis_sign_for_parent(parent_link_name: str, axis_name: str) -> int:
        parent_pos = link_world_pos.get(parent_link_name, [0, 0, 0])
        return -1 if axis_name == "y" and parent_pos[1] < -1e-6 else 1

    def _servo_rest_rpy(explicit_rpy: list, axis_name: str, axis_sign: int = 1) -> list:
        axis_idx = {"x": 0, "y": 1, "z": 2}.get(axis_name, 2)
        return [0, 0, -float(explicit_rpy[axis_idx] or 0) * axis_sign]

    def _servo_driven_child_origin_rpy(parent_axis_name: str, attach_face: str, child_bbox: list, invert_radial_side: bool = False, child_is_servo: bool = False, parent_link_name=None, child_component_id: str = ""):
        """Place a child relative to the servo horn output, not the housing face."""
        adapter_gap = 0.008
        child_half_len = child_bbox[2] / 2
        if child_component_id == "structural_limb_link_slim":
            radial_offset = adapter_gap + child_half_len
            side_clearance = child_bbox[1] / 2
            sign = -1 if attach_face == "top" else 1
            desired_world_z = 1 if attach_face == "top" else -1
            parent_rot_for_sign = link_world_rot.get(parent_link_name, _rpy_to_mat([0, 0, 0])) if parent_link_name else _rpy_to_mat([0, 0, 0])
            if parent_axis_name == "x":
                radial_world = _matvec3(parent_rot_for_sign, [sign, 0, 0])
                if radial_world[2] * desired_world_z < 0:
                    sign = -sign
                # Slim links are thin plates: length is local Z, broad face
                # normal is local Y.  Map local Z to the radial direction and
                # local Y onto the horn shaft normal so the plate seats on its
                # broad face instead of edge-on.
                pitch = math.pi / 2 if sign > 0 else -math.pi / 2
                return [sign * radial_offset, 0, side_clearance], [math.pi / 2, pitch, 0]
            if parent_axis_name == "y":
                radial_world = _matvec3(parent_rot_for_sign, [0, sign, 0])
                if radial_world[2] * desired_world_z < 0:
                    sign = -sign
                roll = -math.pi / 2 if sign > 0 else math.pi / 2
                return [0, sign * radial_offset, side_clearance], [roll, 0, 0]
            radial_world = _matvec3(parent_rot_for_sign, [0, 0, sign])
            if radial_world[2] * desired_world_z < 0:
                sign = -sign
            return [0, 0, sign * radial_offset], [0, 0, 0]
        if parent_axis_name == "z":
            radial_offset = adapter_gap + child_half_len
            return [0, 0, radial_offset], [0, 0, 0]
        radial_offset = max(adapter_gap + child_half_len, 0.060) if child_is_servo else adapter_gap + child_half_len
        sign = 1 if (child_is_servo and parent_axis_name == "x") else (-1 if attach_face == "top" else 1)
        if parent_axis_name == "x":
            return [sign * radial_offset, 0, 0], [0, sign * math.pi / 2, 0]
        roll = (sign if invert_radial_side else -sign) * math.pi / 2
        return [0, sign * radial_offset, 0], [roll, 0, 0]

    def _parent_is_servo_driven_radial_child(parent_name: str) -> bool:
        parent_info = comp_lookup.get(parent_name)
        if not parent_info:
            return False
        driver_name = parent_info.get("attach_to")
        return bool(
            driver_name in servo_link_names and
            servo_axis_names.get(driver_name, "z") != "z"
        )

    robot = ET.Element("robot", name="assembled_robot")
    link_world_rot = {}
    link_world_pos = {}

    for comp in components:
        link_name = comp["link_name"]
        is_actuated = link_name in servo_link_names
        preset = comp_lookup[link_name]["preset"]
        phys = preset.get("physical", {})
        bbox_m = _get_bbox_m(preset, comp)

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

        # For rods that have a parent (i.e., attached to something), offset visual in
        # local +Z so geometry extends forward from the joint. After the joint's rpy
        # rotation (pitch 90°), local Z becomes world X.
        # Root rods (no parent) should NOT be offset — they sit centered at origin.
        has_parent = comp.get("attach_to") is not None
        local_visual_offset = f"0 0 {bbox_m[2]/2:.4f}" if (is_rod and has_parent) else "0 0 0"

        def _make_inertial(parent_el, m, shape, bm, offset="0 0 0"):
            inertial = ET.SubElement(parent_el, "inertial")
            ET.SubElement(inertial, "mass", value=f"{m:.4f}")
            ET.SubElement(inertial, "origin", xyz=offset, rpy="0 0 0")
            if shape == "cylinder":
                r = max(bm[0], bm[1]) / 2
                h_dim = bm[2]
                ixx = m/12 * (3*r*r + h_dim*h_dim)
                izz = m/2 * r*r
                ET.SubElement(inertial, "inertia", ixx=f"{ixx:.6f}", iyy=f"{ixx:.6f}", izz=f"{izz:.6f}", ixy="0", ixz="0", iyz="0")
            elif shape == "sphere":
                r = max(bm) / 2
                ii = 2/5 * m * r*r
                ET.SubElement(inertial, "inertia", ixx=f"{ii:.6f}", iyy=f"{ii:.6f}", izz=f"{ii:.6f}", ixy="0", ixz="0", iyz="0")
            else:
                lx, ly, lz = bm
                ixx = m/12 * (ly*ly + lz*lz)
                iyy = m/12 * (lx*lx + lz*lz)
                izz_val = m/12 * (lx*lx + ly*ly)
                ET.SubElement(inertial, "inertia", ixx=f"{ixx:.6f}", iyy=f"{iyy:.6f}", izz=f"{izz_val:.6f}", ixy="0", ixz="0", iyz="0")

        def _make_visual_box(parent_el, bm, offset="0 0 0", rgba="0.7 0.7 0.7 1", mat_name="mat"):
            visual = ET.SubElement(parent_el, "visual")
            ET.SubElement(visual, "origin", xyz=offset, rpy="0 0 0")
            geom = ET.SubElement(visual, "geometry")
            ET.SubElement(geom, "box", size=f"{bm[0]:.4f} {bm[1]:.4f} {bm[2]:.4f}")
            mat = ET.SubElement(visual, "material", name=mat_name)
            ET.SubElement(mat, "color", rgba=rgba)

        if is_actuated:
            # Split servo: body link (fixed to parent) + horn link (revolute output)
            raw_axis = comp.get("joint_axis", "z")
            axis_name = _axis_name(raw_axis)
            use_side_yoke = axis_name != "z"
            horn_z = bbox_m[2] * SERVO_HORN_Z_RATIO
            body_name = f"{link_name}_body"
            horn_name = f"{link_name}_horn"
            carrier_name = f"{link_name}_compound_carrier"
            body_mass = mass * 0.95
            horn_mass = mass * 0.05

            # Body link — housing occupies full bbox centered at mount origin
            body_link = ET.SubElement(robot, "link", name=body_name)
            body_bm = [bbox_m[0], bbox_m[1], bbox_m[2] * 0.76]
            _make_inertial(body_link, body_mass, "box", bbox_m)
            _make_visual_box(body_link, body_bm, f"0 0 {-bbox_m[2]*0.12:.4f}", rgba="0.3 0.3 0.8 1", mat_name=f"mat_{body_name}")
            if use_side_yoke:
                plate_t = max(min(bbox_m[0], bbox_m[1]) * 0.08, 0.002)
                side_gap = bbox_m[1] / 2 + plate_t * 1.4
                _make_visual_box(body_link, [bbox_m[0] * 1.18, plate_t, bbox_m[2] * 1.08],
                                 f"0 {side_gap:.4f} 0", rgba="0.45 0.45 0.55 1", mat_name=f"mat_{body_name}_yoke_a")
                _make_visual_box(body_link, [bbox_m[0] * 1.18, plate_t, bbox_m[2] * 1.08],
                                 f"0 {-side_gap:.4f} 0", rgba="0.45 0.45 0.55 1", mat_name=f"mat_{body_name}_yoke_b")
                _make_visual_box(body_link, [bbox_m[0] * 1.18, bbox_m[1] + plate_t * 3, plate_t],
                                 f"0 0 {-bbox_m[2]*0.54:.4f}", rgba="0.35 0.35 0.42 1", mat_name=f"mat_{body_name}_yoke_bridge")

            # Horn link — small cylinder at origin of horn frame
            horn_link = ET.SubElement(robot, "link", name=horn_name)
            horn_r = bbox_m[0] * 0.28
            horn_h = bbox_m[2] * 0.08
            horn_bm = [horn_r*2, horn_r*2, horn_h]
            _make_inertial(horn_link, horn_mass, "cylinder", horn_bm)
            horn_vis = ET.SubElement(horn_link, "visual")
            ET.SubElement(horn_vis, "origin", xyz="0 0 0", rpy="0 0 0")
            horn_geom = ET.SubElement(horn_vis, "geometry")
            ET.SubElement(horn_geom, "cylinder", radius=f"{horn_r:.4f}", length=f"{horn_h:.4f}")
            horn_mat = ET.SubElement(horn_vis, "material", name=f"mat_{horn_name}")
            ET.SubElement(horn_mat, "color", rgba="0.8 0.8 0.8 1")
            if use_side_yoke:
                adapter_t = max(min(bbox_m[0], bbox_m[1]) * 0.08, 0.0025)
                _make_visual_box(horn_link, [bbox_m[0] * 0.74, bbox_m[1] * 0.46, adapter_t],
                                 f"0 0 {adapter_t*0.45:.4f}", rgba="0.55 0.55 0.62 1", mat_name=f"mat_{horn_name}_adapter_plate")
                _make_visual_box(horn_link, [bbox_m[0] * 0.28, bbox_m[1] * 0.92, adapter_t * 0.75],
                                 f"0 0 {adapter_t*1.15:.4f}", rgba="0.45 0.45 0.52 1", mat_name=f"mat_{horn_name}_adapter_lug")

            # Mount joint: parent → body (fixed, carries placement rpy/xyz)
            attach_to = comp.get("attach_to")
            if attach_to and attach_to in comp_lookup:
                parent_preset = comp_lookup[attach_to]["preset"]
                attach_face = comp.get("attach_face", "top")
                explicit_rpy = comp.get("attach_rpy", [0, 0, 0])
                if not isinstance(explicit_rpy, list) or len(explicit_rpy) != 3:
                    explicit_rpy = [0, 0, 0]
                explicit_rpy = [float(v or 0) for v in explicit_rpy]
                origin_xyz, computed_rpy = _compute_origin_and_rpy(
                    parent_preset, preset, attach_face, [0, 0, 0], comp_lookup.get(attach_to), comp
                )
                if use_side_yoke and attach_face in ("top", "bottom"):
                    base_half_z = bbox_m[2] / 2
                    rotated_half_z = bbox_m[0] / 2 if axis_name == "x" else bbox_m[1] / 2
                    dz = rotated_half_z - base_half_z
                    origin_xyz[2] += dz if attach_face == "top" else -dz
                parent_link_name = _effective_parent(attach_to)
                if attach_to in servo_link_names:
                    parent_axis_name = servo_axis_names.get(attach_to, "z")
                    parent_comp_for_axis = comp_lookup.get(attach_to)
                    grand_parent_name = parent_comp_for_axis.get("attach_to") if parent_comp_for_axis else None
                    grand_parent_axis = servo_axis_names.get(grand_parent_name, "z")
                    parent_rest = parent_comp_for_axis.get("attach_rpy") if parent_comp_for_axis else None
                    parent_rest_pitch = 0.0
                    if isinstance(parent_rest, list) and len(parent_rest) == 3:
                        try:
                            parent_rest_pitch = float(parent_rest[1] or 0)
                        except (TypeError, ValueError):
                            parent_rest_pitch = 0.0
                    invert_radial = parent_axis_name == "y" and (
                        (grand_parent_name in servo_link_names and grand_parent_axis == "x") or
                        parent_rest_pitch < -0.001
                    )
                    driven_pose = _servo_driven_child_origin_rpy(
                        parent_axis_name, attach_face, bbox_m, invert_radial, is_actuated,
                        parent_link_name, preset.get("id", ""),
                    )
                    if driven_pose:
                        origin_xyz, computed_rpy = driven_pose
                    else:
                        parent_horn_z = _get_bbox_m(comp_lookup[attach_to]["preset"], comp_lookup.get(attach_to))[2] * SERVO_HORN_Z_RATIO
                        origin_xyz = [origin_xyz[0], origin_xyz[1], origin_xyz[2] - parent_horn_z]
                align_rpy = _servo_shaft_align_rpy(axis_name)
                parent_rot = link_world_rot.get(parent_link_name, _rpy_to_mat([0, 0, 0]))
                parent_cid = parent_preset.get("id", "") if parent_preset else ""
                if _is_distal_beam_component_id(parent_cid) and attach_face == "bottom":
                    normal_world = _matvec3(parent_rot, [0, 0, origin_xyz[2]])
                    if normal_world[2] > 0.0001:
                        origin_xyz[2] = -origin_xyz[2]
                        print(f"[assembly] beam distal bottom corrected: {link_name} local_z flipped so child moves downward in world", file=sys.stderr)
                mounted_rpy = (
                    _servo_planar_mount_rpy_for_parent(parent_link_name, attach_face, computed_rpy)
                    if axis_name == "z"
                    else (_servo_mount_rpy_for_parent(parent_link_name, axis_name) or [
                        computed_rpy[i] + align_rpy[i] for i in range(3)
                    ])
                )
                rpy_str = f"{mounted_rpy[0]:.4f} {mounted_rpy[1]:.4f} {mounted_rpy[2]:.4f}"
                axis_sign = _servo_axis_sign_for_parent(parent_link_name, axis_name)
                if (
                    axis_name == "y"
                    and _is_distal_beam_component_id(parent_cid)
                    and attach_face == "bottom"
                ):
                    bend = abs(float(explicit_rpy[1] or 0))
                    # Mirrored Y-axis knee carriers need opposite local horn
                    # signs so both sides fold toward world -X in dog stance.
                    horn_zero = [0, 0, bend * axis_sign]
                else:
                    horn_zero = _servo_rest_rpy(explicit_rpy, axis_name, axis_sign)
                horn_zero_rpy = f"{horn_zero[0]:.4f} {horn_zero[1]:.4f} {horn_zero[2]:.4f}"
                use_compound_carrier = attach_to in servo_link_names
                if use_compound_carrier:
                    carrier_mass = max(mass * 0.18, 0.025)
                    carrier_link = ET.SubElement(robot, "link", name=carrier_name)
                    _make_inertial(carrier_link, carrier_mass, "box", [bbox_m[0] * 1.2, bbox_m[1] * 1.4, bbox_m[2] * 1.2])
                    carrier_plate_t = max(min(bbox_m[0], bbox_m[1]) * 0.075, 0.0025)
                    carrier_side_y = bbox_m[1] / 2 + carrier_plate_t * 4.2
                    _make_visual_box(
                        carrier_link,
                        [bbox_m[0] * 1.18, carrier_plate_t, bbox_m[2] * 1.18],
                        f"0 {carrier_side_y:.4f} 0",
                        rgba="0.42 0.46 0.50 1",
                        mat_name=f"mat_{carrier_name}_side_a",
                    )
                    _make_visual_box(
                        carrier_link,
                        [bbox_m[0] * 1.18, carrier_plate_t, bbox_m[2] * 1.18],
                        f"0 {-carrier_side_y:.4f} 0",
                        rgba="0.42 0.46 0.50 1",
                        mat_name=f"mat_{carrier_name}_side_b",
                    )
                    _make_visual_box(
                        carrier_link,
                        [carrier_plate_t * 1.2, bbox_m[1] + carrier_plate_t * 8.4, carrier_plate_t * 1.2],
                        f"{-bbox_m[0] * 0.42:.4f} 0 {-bbox_m[2] * 0.42:.4f}",
                        rgba="0.31 0.34 0.38 1",
                        mat_name=f"mat_{carrier_name}_tie",
                    )
                    carrier_reach = math.sqrt(sum(float(v or 0) * float(v or 0) for v in origin_xyz))
                    bridge_len = max(carrier_reach - bbox_m[1] * 0.25, 0)
                    if bridge_len > carrier_plate_t * 2:
                        for sx, suffix in ((0.38, "bridge_a"), (-0.38, "bridge_b")):
                            _make_visual_box(
                                carrier_link,
                                [carrier_plate_t * 1.8, bridge_len, carrier_plate_t * 1.8],
                                f"{bbox_m[0] * sx:.4f} {-bridge_len / 2:.4f} 0",
                                rgba="0.31 0.34 0.38 1",
                                mat_name=f"mat_{carrier_name}_{suffix}",
                            )
                    carrier_joint = ET.SubElement(robot, "joint", name=f"{link_name}_compound_carrier", type="fixed")
                    ET.SubElement(carrier_joint, "parent", link=parent_link_name)
                    ET.SubElement(carrier_joint, "child", link=carrier_name)
                    ET.SubElement(carrier_joint, "origin", xyz=f"{origin_xyz[0]:.4f} {origin_xyz[1]:.4f} {origin_xyz[2]:.4f}", rpy=rpy_str)
                mount_joint = ET.SubElement(robot, "joint", name=f"{link_name}_mount", type="fixed")
                ET.SubElement(mount_joint, "parent", link=carrier_name if use_compound_carrier else parent_link_name)
                ET.SubElement(mount_joint, "child", link=body_name)
                if use_compound_carrier:
                    ET.SubElement(mount_joint, "origin", xyz="0 0 0", rpy="0 0 0")
                else:
                    ET.SubElement(mount_joint, "origin", xyz=f"{origin_xyz[0]:.4f} {origin_xyz[1]:.4f} {origin_xyz[2]:.4f}", rpy=rpy_str)
                parent_rot = link_world_rot.get(parent_link_name, _rpy_to_mat([0, 0, 0]))
                parent_pos = link_world_pos.get(parent_link_name, [0, 0, 0])
                body_pos = _vadd3(parent_pos, _matvec3(parent_rot, origin_xyz))
                body_rot = _matmul3(parent_rot, _rpy_to_mat(mounted_rpy))
                if use_compound_carrier:
                    link_world_rot[carrier_name] = body_rot
                    link_world_pos[carrier_name] = body_pos
                horn_pos = _vadd3(body_pos, _matvec3(body_rot, [0, 0, horn_z]))
                horn_rot = _matmul3(body_rot, _rpy_to_mat(horn_zero))
                link_world_rot[body_name] = body_rot
                link_world_rot[horn_name] = horn_rot
                link_world_pos[body_name] = body_pos
                link_world_pos[horn_name] = horn_pos
            else:
                horn_zero_rpy = "0 0 0"
                link_world_rot[body_name] = _rpy_to_mat([0, 0, 0])
                link_world_rot[horn_name] = _rpy_to_mat([0, 0, 0])
                link_world_pos[body_name] = [0, 0, 0]
                link_world_pos[horn_name] = [0, 0, horn_z]

            # Revolute joint: body → horn at horn origin, axis from joint_axis
            joint_axis_vec = [0, 0, 1]
            me = preset.get("mechanical_electrical", {})
            effort = me.get("max_torque_nm", me.get("holding_torque_nm", 10.0))
            rev_joint = ET.SubElement(robot, "joint", name=link_name, type="revolute")
            ET.SubElement(rev_joint, "parent", link=body_name)
            ET.SubElement(rev_joint, "child", link=horn_name)
            ET.SubElement(rev_joint, "origin", xyz=f"0 0 {horn_z:.4f}", rpy=horn_zero_rpy)
            ET.SubElement(rev_joint, "axis", xyz=f"{joint_axis_vec[0]} {joint_axis_vec[1]} {joint_axis_vec[2]}")
            lo_rad, hi_rad = _resolve_joint_limits_rad(preset)
            ET.SubElement(rev_joint, "limit", lower=f"{lo_rad:.5f}", upper=f"{hi_rad:.5f}",
                         effort=f"{effort}", velocity="1.0")
            print(f"[assembly] servo split: {link_name} body+horn, requested_axis={axis_name}, local_axis={joint_axis_vec}, side_yoke={use_side_yoke}, horn_z={horn_z:.4f}", file=sys.stderr)

        else:
            # Non-servo component: emit single link + joint (original path)

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
                    parent_preset, child_preset, attach_face, explicit_rpy, comp_lookup.get(attach_to), comp
                )
                parent_link_name = _effective_parent(attach_to)
                # If parent is a servo, xyz is in body frame — correct to horn frame
                if attach_to in servo_link_names:
                    parent_axis_name = servo_axis_names.get(attach_to, "z")
                    parent_comp_for_axis = comp_lookup.get(attach_to)
                    grand_parent_name = parent_comp_for_axis.get("attach_to") if parent_comp_for_axis else None
                    grand_parent_axis = servo_axis_names.get(grand_parent_name, "z")
                    parent_rest = parent_comp_for_axis.get("attach_rpy") if parent_comp_for_axis else None
                    parent_rest_pitch = 0.0
                    if isinstance(parent_rest, list) and len(parent_rest) == 3:
                        try:
                            parent_rest_pitch = float(parent_rest[1] or 0)
                        except (TypeError, ValueError):
                            parent_rest_pitch = 0.0
                    invert_radial = parent_axis_name == "y" and (
                        (grand_parent_name in servo_link_names and grand_parent_axis == "x") or
                        parent_rest_pitch < -0.001
                    )
                    driven_pose = _servo_driven_child_origin_rpy(
                        parent_axis_name, attach_face, bbox_m, invert_radial, False,
                        parent_link_name, child_preset.get("id", ""),
                    )
                    if driven_pose:
                        origin_xyz, computed_rpy = driven_pose
                    else:
                        parent_horn_z = _get_bbox_m(comp_lookup[attach_to]["preset"], comp_lookup.get(attach_to))[2] * SERVO_HORN_Z_RATIO
                        origin_xyz = [origin_xyz[0], origin_xyz[1], origin_xyz[2] - parent_horn_z]

                parent_rot = link_world_rot.get(parent_link_name, _rpy_to_mat([0, 0, 0]))
                parent_cid = parent_preset.get("id", "") if parent_preset else ""
                if _is_distal_beam_component_id(parent_cid) and attach_face == "bottom":
                    normal_world = _matvec3(parent_rot, [0, 0, origin_xyz[2]])
                    if normal_world[2] > 0.0001:
                        origin_xyz[2] = -origin_xyz[2]
                        print(f"[assembly] beam distal bottom corrected: {link_name} local_z flipped so child moves downward in world", file=sys.stderr)
                explicit_nonzero = any(abs(float(v or 0)) > 0.001 for v in explicit_rpy)
                child_cid = child_preset.get("id", "") if child_preset else ""
                if child_cid == "mobility_rubber_foot_pad" and not explicit_nonzero:
                    computed_rpy = _world_level_rpy_for_parent(parent_link_name)
                    print(f"[assembly] level foot pad: {link_name} rpy={computed_rpy}", file=sys.stderr)

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
                # Drivetrain motors on bottom face get Rx(-90°); after that rotation,
                # local Z = world Y (the rolling axis). Remap "y" → [0,0,1].
                if _is_drivetrain_component_id(child_cid) and attach_face == "bottom" and raw_axis == "y":
                    joint_axis = [0, 0, 1]

                joint_name = f"j_{link_name}"

                joint_el = ET.SubElement(robot, "joint", name=joint_name, type=joint_type)
                ET.SubElement(joint_el, "parent", link=parent_link_name)
                ET.SubElement(joint_el, "child", link=link_name)
                ET.SubElement(joint_el, "origin", xyz=f"{origin_xyz[0]:.4f} {origin_xyz[1]:.4f} {origin_xyz[2]:.4f}", rpy=rpy_str)
                ET.SubElement(joint_el, "axis", xyz=f"{joint_axis[0]} {joint_axis[1]} {joint_axis[2]}")
                parent_pos = link_world_pos.get(parent_link_name, [0, 0, 0])
                link_world_rot[link_name] = _matmul3(parent_rot, _rpy_to_mat(computed_rpy))
                link_world_pos[link_name] = _vadd3(parent_pos, _matvec3(parent_rot, origin_xyz))

                if joint_type in ("revolute", "prismatic"):
                    me = preset.get("mechanical_electrical", {})
                    effort = me.get("max_torque_nm", 10.0)
                    lo_rad, hi_rad = _resolve_joint_limits_rad(preset)
                    ET.SubElement(joint_el, "limit", lower=f"{lo_rad:.5f}", upper=f"{hi_rad:.5f}",
                                 effort=f"{effort}", velocity="1.0")
                elif joint_type == "continuous":
                    # No angle limits, but carry effort so urdf_to_mjcf gets correct ctrlrange.
                    me = preset.get("mechanical_electrical", {})
                    sim = preset.get("sim_metadata", {})
                    effort = sim.get("peak_torque_nm",
                             me.get("stall_torque_nm",
                             me.get("peak_torque_nm", 10.0)))
                    ET.SubElement(joint_el, "limit", effort=f"{effort}", velocity="50.0")
            else:
                link_world_rot[link_name] = _rpy_to_mat([0, 0, 0])
                link_world_pos[link_name] = [0, 0, 0]

    # Apply ground offset: shift root link's visual up so bottom face is at Z=0
    if assembly.get("ground_offset", False) and base_link in comp_lookup:
        root_preset = comp_lookup[base_link]["preset"]
        root_bbox = _get_bbox_m(root_preset, comp_lookup.get(base_link))
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


# ── Tool-Use Assembly Agent ─────────────────────────────────────────────────

ASSEMBLY_TOOLS = [
    {
        "name": "add_component",
        "description": "Add a component to the robot assembly. The placement engine computes exact 3D position from the face specification. Returns the updated assembly state.",
        "input_schema": {
            "type": "object",
            "properties": {
                "component_id": {"type": "string", "description": "ID from the component library (e.g., 'actuator_servo_high_torque')"},
                "parent_link": {"type": "string", "description": "Name of the parent link to attach to. Use null for the root/first component.", "nullable": True},
                "attach_face": {"type": "string", "enum": ["top", "bottom", "front", "back", "left", "right"], "description": "Which face of the parent to attach to"},
                "joint_type": {"type": "string", "enum": ["fixed", "revolute", "prismatic"], "description": "Joint type connecting to parent"},
                "joint_axis": {"type": "string", "enum": ["x", "y", "z"], "description": "Rotation/translation axis. z=yaw/spin, y=pitch, x=roll"},
                "length_mm": {"type": "number", "description": "Optional: override length for parametric structural links (default 100mm). Prefer structural_limb_link_slim for sleek robot limbs."},
                "orientation": {"type": "string", "enum": ["horizontal", "vertical", "auto"], "description": "Orientation hint for elongated parts. 'horizontal' extends along +X, 'vertical' stays along +Z, 'auto' lets the engine decide."},
            },
            "required": ["component_id", "parent_link", "attach_face", "joint_type", "joint_axis"],
        },
    },
    {
        "name": "get_state",
        "description": "Get the current assembly state including all links, their world positions, and bounding boxes. Use this to verify placement before adding more components.",
        "input_schema": {"type": "object", "properties": {}},
    },
    {
        "name": "finish",
        "description": "Signal that the assembly is complete. Call this when all components have been placed.",
        "input_schema": {
            "type": "object",
            "properties": {
                "summary": {"type": "string", "description": "Brief description of the completed robot"},
            },
            "required": ["summary"],
        },
    },
]

ASSEMBLY_SYSTEM_PROMPT = r"""You are a robot assembly agent. You build robots by calling tools — one component at a time.

## How It Works

1. Call add_component for each part, starting with the base
2. After each call, you receive the updated assembly state (all links, positions, bounding boxes)
3. Use the state to verify placement before adding the next component
4. Call finish when done

## Available Components

{COMPONENT_CATALOG}

## Placement Rules (the engine handles these, but you should understand them)

- "top" face = +Z direction. "front" = +X. "bottom" = -Z.
- Elongated parts (extrusions) on "top" with orientation="horizontal" extend along +X
- Elongated parts with orientation="vertical" stay along +Z (for legs)
- Multiple children on the same face are auto-distributed to corners
- Ground plane is at Z=0

## Design Rules

- ALWAYS start with structural_baseplate as the first component (parent_link=null)
- Drivetrain motors (drivetrain_hub_motor_80, drivetrain_geared_dc_with_coupler) use joint_type="continuous" — unbounded spin, torque-controlled. Servo actuators use joint_type="revolute". Structural/sensors use "fixed".
- joint_axis: "z" for yaw/spin, "y" for pitch, "x" for roll
- For arms: servo(revolute z) -> extrusion(horizontal) -> servo(revolute y) -> extrusion(horizontal) -> gripper
- For legs: servo(revolute y) on bottom -> structural_limb_link_slim(vertical) -> servo(revolute y) -> structural_limb_link_slim(vertical)
- For slim limb links: do not set attach_rpy/orientation to make the link look flush. The engine mounts `structural_limb_link_slim` on its broad flat face; rest/crouch angles belong on the servo that drives the link.
- For wheels: NEVER attach a tire directly to the baseplate. Use a drivetrain assembly:
  baseplate -> drivetrain_hub_motor_80 (attach_face="bottom", continuous y) -> mobility_wheel_driven (attach_face="coaxial", fixed).
  The drivetrain IS the motor; the tire mounts coaxially on the hub (attach_face="coaxial"). The placement engine applies the axial offset and the side-flip automatically — emit the same (coaxial, fixed) annotation for every wheel regardless of corner.
  Casters: baseplate -> drivetrain_caster_swivel (bottom, fixed) -> mobility_wheel_driven (coaxial, fixed).
  Mecanum: baseplate -> drivetrain_hub_motor_80 (bottom, continuous y) -> mobility_mecanum_wheel (coaxial, fixed).
  Default 4 wheels for any "car/truck/vehicle/rover/buggy/cart" request.
- Use length_mm=150-250 for arm/leg extrusions

## Important

- Place components ONE AT A TIME. Check the state after each placement.
- If something looks wrong in the state (overlap, wrong position), you can adjust by adding a corrective component.
- Call finish when the robot is complete.
"""


def _execute_add_component(assembly_state: dict, tool_input: dict) -> dict:
    """Execute an add_component tool call, updating the assembly state."""
    try:
        from core.presets import get_component
    except ImportError:
        return {"error": "Preset library not available"}

    comp_id = tool_input["component_id"]
    parent_link = tool_input.get("parent_link")
    attach_face = tool_input.get("attach_face", "top")
    joint_type = tool_input.get("joint_type", "fixed")
    joint_axis_str = tool_input.get("joint_axis", "z")
    length_mm = tool_input.get("length_mm")
    orientation = tool_input.get("orientation", "auto")

    preset = get_component(comp_id)
    if not preset:
        return {"error": f"Unknown component_id: {comp_id}"}

    phys = preset.get("physical", {})
    bb = phys.get("bounding_box_mm")
    if bb and len(bb) >= 3:
        bbox_m = [b / 1000.0 for b in bb]
    else:
        cs = phys.get("cross_section_mm")
        if cs and len(cs) >= 2:
            bbox_m = [cs[0] / 1000.0, cs[1] / 1000.0, 0.1]
        else:
            bbox_m = [0.05, 0.05, 0.05]

    if length_mm and phys.get("cross_section_mm"):
        bbox_m[2] = length_mm / 1000.0

    mass = phys.get("mass_kg") or phys.get("mass_kg_per_100mm") or 0.1

    # Determine link name
    links = assembly_state.get("links", {})
    idx = len(links) + 1
    link_name = f"{comp_id}_{idx}"

    # Compute placement
    import math
    axis_map = {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]}
    joint_axis = axis_map.get(joint_axis_str, [0, 0, 1])

    gap = 0.005
    px, py, pz = 0.05, 0.05, 0.05  # default parent half-extents
    origin_xyz = [0, 0, 0]
    origin_rpy = [0, 0, 0]

    if parent_link and parent_link in links:
        parent_info = links[parent_link]
        p_bbox = parent_info.get("bbox_m", [0.1, 0.1, 0.1])
        px, py, pz = p_bbox[0] / 2, p_bbox[1] / 2, p_bbox[2] / 2
        parent_world = parent_info.get("world_xyz", [0, 0, 0])
    else:
        parent_world = [0, 0, 0]

    cx, cy, cz = bbox_m[0] / 2, bbox_m[1] / 2, bbox_m[2] / 2

    # Check if child is elongated (rod-shaped)
    sorted_dims = sorted(bbox_m)
    is_elongated = sorted_dims[2] > sorted_dims[0] * 2.5 and sorted_dims[1] < sorted_dims[0] * 2.0

    # Determine if we should rotate the elongated part
    should_rotate_horizontal = False
    if is_elongated:
        if orientation == "horizontal":
            should_rotate_horizontal = True
        elif orientation == "vertical":
            should_rotate_horizontal = False
        else:  # auto
            # Auto: horizontal for "top" face (arms), vertical for "bottom" face (legs)
            should_rotate_horizontal = attach_face in ("top", "front", "back")

    if should_rotate_horizontal:
        origin_rpy = [0, math.pi / 2, 0]
        # After rotation, Z extent maps to X, X extent maps to Z
        cz_eff = cx  # cross-section becomes Z extent
    else:
        cz_eff = cz

    # Count how many children are already on this face of this parent
    face_key = f"{parent_link}:{attach_face}"
    existing_on_face = assembly_state.get("face_counts", {}).get(face_key, 0)
    total_on_face = existing_on_face + 1  # including this one

    # Compute tangential offset for multi-child distribution
    tu, tv = 0, 0
    if existing_on_face > 0:
        # Simple offset: alternate sides
        inset = 0.7
        if existing_on_face == 1:
            tu = -inset * px if attach_face in ("top", "bottom") else -inset * py
        elif existing_on_face == 2:
            tv = inset * py if attach_face in ("top", "bottom") else inset * pz
        elif existing_on_face == 3:
            tu = -inset * px if attach_face in ("top", "bottom") else -inset * py
            tv = -inset * py if attach_face in ("top", "bottom") else -inset * pz

    # Face-based offset
    face_offsets = {
        "top":    [tu, tv, pz + cz_eff + gap],
        "bottom": [tu, tv, -(pz + cz_eff + gap)],
        "front":  [px + cx + gap, tu, tv],
        "back":   [-(px + cx + gap), tu, tv],
        "right":  [tu, py + cy + gap, tv],
        "left":   [tu, -(py + cy + gap), tv],
    }
    origin_xyz = face_offsets.get(attach_face, [0, 0, pz + cz_eff + gap])

    # Coaxial hub-motor mount: tire offsets along drivetrain-local +Z (the axle
    # direction) so its inboard bore face seats against the motor's outboard end.
    # After the drivetrain's -pi/2 X-roll, local +Z = world +Y (outboard).
    # Do NOT offset in local +Y — that is world -Z (downward) after the roll and
    # places the tire below the motor rather than beside it. Outboard direction
    # is handled by the 180° yaw flip on the -Y baseplate half (below).
    _is_tire = _is_tire_component_id(comp_id)
    parent_comp_id = links.get(parent_link, {}).get("component_id", "") if parent_link else ""
    _is_drivetrain_parent = _is_drivetrain_component_id(parent_comp_id)
    if attach_face == "coaxial" and _is_tire and _is_drivetrain_parent:
        # pz = motor axle half-length (local Z), cz = tire half-width along axle.
        origin_xyz = [0, 0, pz + cz]
        origin_rpy = [0, 0, 0]

    # Drivetrain side-flip: yaw 180° when on baseplate -Y half so the coaxial
    # wheel-offset ends up outboard on both sides of the chassis.
    if _is_drivetrain_component_id(comp_id) and attach_face == "bottom" and tv < 0:
        origin_rpy = [origin_rpy[0], origin_rpy[1], origin_rpy[2] + math.pi]

    # Compute world position
    world_xyz = [
        parent_world[0] + origin_xyz[0],
        parent_world[1] + origin_xyz[1],
        parent_world[2] + origin_xyz[2],
    ]

    # Update assembly state
    links[link_name] = {
        "component_id": comp_id,
        "parent": parent_link,
        "attach_face": attach_face,
        "joint_type": joint_type,
        "joint_axis": joint_axis,
        "joint_axis_name": joint_axis_str,
        "origin_xyz": [round(v, 4) for v in origin_xyz],
        "origin_rpy": [round(v, 4) for v in origin_rpy],
        "bbox_m": [round(v, 4) for v in bbox_m],
        "world_xyz": [round(v, 4) for v in world_xyz],
        "mass_kg": mass,
    }
    if length_mm is not None:
        links[link_name]["length_mm"] = length_mm
    if orientation:
        links[link_name]["orientation"] = orientation
    assembly_state["links"] = links

    # Update face counts
    face_counts = assembly_state.get("face_counts", {})
    face_counts[face_key] = existing_on_face + 1
    assembly_state["face_counts"] = face_counts

    # Build state summary for Claude
    state_lines = []
    for lname, linfo in links.items():
        pos = linfo["world_xyz"]
        bb = linfo["bbox_m"]
        state_lines.append(f"  {lname}: pos=[{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}] bbox={bb[0]:.3f}x{bb[1]:.3f}x{bb[2]:.3f}m parent={linfo.get('parent','none')}")

    return {
        "success": True,
        "link_name": link_name,
        "placed_at": {"xyz": origin_xyz, "rpy": origin_rpy, "world_xyz": world_xyz},
        "total_links": len(links),
        "assembly_state": "\n".join(state_lines),
    }


def _axis_name_from_vector(axis) -> str:
    """Best-effort conversion for legacy add_component state."""
    try:
        vals = [float(v) for v in axis]
    except Exception:
        return "z"
    if len(vals) < 3:
        return "z"
    best_i = max(range(3), key=lambda i: abs(vals[i]))
    return ("x", "y", "z")[best_i]


def _assembly_graph_from_state(assembly_state: dict) -> dict:
    """
    Convert the legacy step-by-step tool-agent state into the canonical
    AssemblyGraph shape consumed by the TypeScript resolver.
    """
    links = assembly_state.get("links", {})
    components = []
    base_link = None

    for link_name, info in links.items():
        parent = info.get("parent")
        if parent is None and base_link is None:
            base_link = link_name

        comp = {
            "link_name": link_name,
            "component_id": info.get("component_id", ""),
            "attach_to": parent,
            "attach_face": info.get("attach_face"),
            "joint_type": info.get("joint_type", "fixed"),
            "joint_axis": info.get("joint_axis_name") or _axis_name_from_vector(info.get("joint_axis")),
        }
        if info.get("length_mm") is not None:
            comp["length_mm"] = info["length_mm"]
        if info.get("orientation"):
            comp["orientation"] = info["orientation"]
        components.append(comp)

    return {
        "base_link": base_link or (components[0]["link_name"] if components else "structural_baseplate_1"),
        "ground_offset": True,
        "components": components,
    }


def _build_urdf_from_state(assembly_state: dict) -> str:
    """Generate URDF XML from the assembly state dict."""
    import xml.etree.ElementTree as ET
    import math

    try:
        from core.presets import get_component
    except ImportError:
        return ""

    links = assembly_state.get("links", {})
    robot = ET.Element("robot", name="assembled_robot")

    for link_name, info in links.items():
        preset = get_component(info["component_id"])
        if not preset:
            continue

        phys = preset.get("physical", {})
        bbox_m = info["bbox_m"]
        mass = info["mass_kg"]
        shape = phys.get("inertia_primitive", "box")

        # Link element
        link_el = ET.SubElement(robot, "link", name=link_name)
        inertial = ET.SubElement(link_el, "inertial")
        ET.SubElement(inertial, "mass", value=f"{mass:.4f}")
        ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")

        if shape == "cylinder":
            r = max(bbox_m[0], bbox_m[1]) / 2
            h = bbox_m[2]
            ixx = mass / 12 * (3 * r * r + h * h)
            izz = mass / 2 * r * r
            ET.SubElement(inertial, "inertia", ixx=f"{ixx:.6f}", iyy=f"{ixx:.6f}", izz=f"{izz:.6f}", ixy="0", ixz="0", iyz="0")
        else:
            lx, ly, lz = bbox_m
            ixx = mass / 12 * (ly * ly + lz * lz)
            iyy = mass / 12 * (lx * lx + lz * lz)
            izz = mass / 12 * (lx * lx + ly * ly)
            ET.SubElement(inertial, "inertia", ixx=f"{ixx:.6f}", iyy=f"{iyy:.6f}", izz=f"{izz:.6f}", ixy="0", ixz="0", iyz="0")

        visual = ET.SubElement(link_el, "visual")
        ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
        geom = ET.SubElement(visual, "geometry")
        if shape == "cylinder":
            r = max(bbox_m[0], bbox_m[1]) / 2
            ET.SubElement(geom, "cylinder", radius=f"{r:.4f}", length=f"{bbox_m[2]:.4f}")
        else:
            ET.SubElement(geom, "box", size=f"{bbox_m[0]:.4f} {bbox_m[1]:.4f} {bbox_m[2]:.4f}")
        mat = ET.SubElement(visual, "material", name=f"mat_{link_name}")
        ET.SubElement(mat, "color", rgba="0.7 0.7 0.7 1")

        # Joint (skip for root)
        parent = info.get("parent")
        if parent and parent in links:
            oxyz = info["origin_xyz"]
            orpy = info["origin_rpy"]
            joint_el = ET.SubElement(robot, "joint", name=f"j_{link_name}", type=info["joint_type"])
            ET.SubElement(joint_el, "parent", link=parent)
            ET.SubElement(joint_el, "child", link=link_name)
            ET.SubElement(joint_el, "origin",
                         xyz=f"{oxyz[0]:.4f} {oxyz[1]:.4f} {oxyz[2]:.4f}",
                         rpy=f"{orpy[0]:.4f} {orpy[1]:.4f} {orpy[2]:.4f}")
            ax = info["joint_axis"]
            ET.SubElement(joint_el, "axis", xyz=f"{ax[0]} {ax[1]} {ax[2]}")
            if info["joint_type"] in ("revolute", "prismatic"):
                me = preset.get("mechanical_electrical", {})
                effort = me.get("max_torque_nm") or me.get("holding_torque_nm") or 10
                ET.SubElement(joint_el, "limit", lower="-3.14159", upper="3.14159",
                             effort=f"{effort}", velocity="1.0")

    # Pretty print
    def indent(elem, level=0):
        i = "\n" + level * "  "
        if len(elem):
            if not elem.text or not elem.text.strip(): elem.text = i + "  "
            if not elem.tail or not elem.tail.strip(): elem.tail = i
            for child in elem: indent(child, level + 1)
            if not child.tail or not child.tail.strip(): child.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()): elem.tail = i
        if not level: elem.tail = "\n"

    indent(robot)
    return '<?xml version="1.0"?>\n' + ET.tostring(robot, encoding="unicode")


def generate_assembly_with_tools(prompt: str, session_id: str = "default",
                                  on_progress=None,
                                  model: str = "claude-sonnet-4-6",
                                  images: list | None = None) -> dict:
    """
    Use Claude's tool-use API to build a robot iteratively.
    Claude calls add_component one at a time, seeing the state after each placement.
    Returns a canonical AssemblyGraph; the frontend resolver builds the URDF.
    """
    client = _get_client()

    system_prompt = ASSEMBLY_SYSTEM_PROMPT.replace(
        "{COMPONENT_CATALOG}",
        _get_component_catalog(user_prompt=prompt),
    )

    first_user_content = _build_user_content(f"Build this robot: {prompt}", images)
    messages = [{"role": "user", "content": first_user_content}]
    assembly_state = {"links": {}, "face_counts": {}}

    max_rounds = 30  # safety limit
    round_num = 0

    if on_progress:
        on_progress("thinking", "Planning robot design...")

    while round_num < max_rounds:
        round_num += 1
        print(f"[tool-agent] Round {round_num}, {len(assembly_state['links'])} links placed", file=sys.stderr)

        response = client.messages.create(
            model=model,
            max_tokens=4096,
            system=[{
                "type": "text",
                "text": system_prompt,
                "cache_control": {"type": "ephemeral"},
            }],
            messages=messages,
            tools=ASSEMBLY_TOOLS,
            timeout=60.0,
        )
        _log_cache_usage("tool_agent", response)

        # Process response content blocks
        assistant_content = response.content
        messages.append({"role": "assistant", "content": assistant_content})

        # Check if there are any tool_use blocks
        tool_uses = [block for block in assistant_content if block.type == "tool_use"]

        if not tool_uses:
            # No tool calls — Claude is done (returned final text)
            text_blocks = [block.text for block in assistant_content if hasattr(block, "text")]
            explanation = " ".join(text_blocks) if text_blocks else "Assembly complete"
            break

        # Execute each tool call
        tool_results = []
        for tool_use in tool_uses:
            tool_name = tool_use.name
            tool_input = tool_use.input

            print(f"[tool-agent] Tool call: {tool_name}({json.dumps(tool_input, indent=None)})", file=sys.stderr)

            if tool_name == "add_component":
                result = _execute_add_component(assembly_state, tool_input)
                if on_progress:
                    n = len(assembly_state["links"])
                    on_progress("generating", f"Placing component {n}...")

            elif tool_name == "get_state":
                state_lines = []
                for lname, linfo in assembly_state.get("links", {}).items():
                    pos = linfo["world_xyz"]
                    bb = linfo["bbox_m"]
                    state_lines.append(f"  {lname}: pos=[{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}] bbox={bb[0]:.3f}x{bb[1]:.3f}x{bb[2]:.3f}m")
                result = {"total_links": len(assembly_state["links"]), "state": "\n".join(state_lines)}

            elif tool_name == "finish":
                result = {"success": True, "summary": tool_input.get("summary", "Complete")}
                tool_results.append({
                    "type": "tool_result",
                    "tool_use_id": tool_use.id,
                    "content": json.dumps(result),
                })
                messages.append({"role": "user", "content": tool_results})

                explanation = tool_input.get("summary", "Robot assembly complete")
                # Break out of the loop
                round_num = max_rounds  # force exit
                break
            else:
                result = {"error": f"Unknown tool: {tool_name}"}

            tool_results.append({
                "type": "tool_result",
                "tool_use_id": tool_use.id,
                "content": json.dumps(result),
            })

        if round_num >= max_rounds:
            break

        # Send tool results back to Claude
        messages.append({"role": "user", "content": tool_results})

    assembly_graph = _assembly_graph_from_state(assembly_state)
    n_links = len(assembly_state["links"])
    n_joints = sum(1 for l in assembly_state["links"].values() if l.get("parent"))

    if on_progress:
        on_progress("done", "Assembly complete")

    print(f"[tool-agent] Done: {n_links} links, {n_joints} joints, {round_num} rounds", file=sys.stderr)

    return {
        "explanation": explanation if 'explanation' in dir() else "Assembly complete",
        "assembly_graph": assembly_graph,
        "new_urdf": "",
        "stats": f"{n_links} components, {n_joints} joints, {round_num} rounds",
    }


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


def _extract_tool_result(response, current_urdf: str) -> dict:
    """Extract structured result from a Claude response with tool-use or text fallback."""
    # Check for tool_use blocks first (structured output)
    for block in response.content:
        if block.type == "tool_use":
            tool_input = block.input
            if block.name == "design_robot":
                assembly = {
                    "base_link": tool_input.get("base_link", "structural_baseplate_1"),
                    "ground_offset": True,
                    "components": tool_input.get("components", []),
                }
                n = len(assembly["components"])
                print(f"[ai_edit] Tool-use design_robot: {n} components (structured output)", file=sys.stderr)
                return {
                    "explanation": tool_input.get("explanation", "Assembly designed"),
                    "assembly_graph": assembly,
                    "new_urdf": current_urdf,
                    "stats": tool_input.get("changes_summary", f"{n} components"),
                }
            elif block.name == "modify_topology":
                operations = tool_input.get("operations", [])
                print(f"[ai_edit] Tool-use modify_topology: {len(operations)} operations (structured output)", file=sys.stderr)
                return {
                    "explanation": tool_input.get("explanation", "Topology modified"),
                    "topology_ops": operations,
                    "new_urdf": current_urdf,
                    "stats": tool_input.get("changes_summary", f"{len(operations)} topology changes"),
                }

    # Fallback: extract text and parse as JSON (backward compat)
    response_text = ""
    for block in response.content:
        if hasattr(block, "text"):
            response_text += block.text

    if not response_text:
        return {"explanation": "No response", "new_urdf": current_urdf, "stats": "No changes"}

    print(f"[ai_edit] Falling back to text JSON parsing (no tool_use block)", file=sys.stderr)
    result = _parse_json_response(response_text)

    if "assembly" in result and result["assembly"]:
        assembly = result["assembly"]
        return {
            "explanation": result.get("explanation", "Assembly designed"),
            "assembly_graph": assembly,
            "new_urdf": current_urdf,
            "stats": result.get("changes_summary", "Assembly ready"),
        }
    elif "full_urdf" in result and result["full_urdf"]:
        return {
            "explanation": result.get("explanation", "Changes applied"),
            "new_urdf": result["full_urdf"],
            "stats": result.get("changes_summary", "Edit complete"),
        }
    else:
        return {
            "explanation": result.get("explanation", "No changes"),
            "new_urdf": current_urdf,
            "stats": result.get("changes_summary", "No structured output"),
        }


def _build_edit_user_message(prompt: str, current_urdf: str, kinematic_graph_json: dict,
                              kinematic_context: str | None,
                              assembly_graph: dict | None) -> str:
    """Assemble the user turn for a Claude edit call.

    When `assembly_graph` is present (Workstream #1 canonical-graph preservation),
    it becomes the authoritative source of truth: Claude is told to reason and
    edit against this JSON, and the URDF is dropped. URDF is a lossy renderer
    serialization — including it when the graph is available wastes tokens and
    reintroduces the round-trip losses the workstream exists to fix (orientation,
    elevation_angle, length_mm, attach_rpy).

    On first-ever-turn / import-URDF / backward-compat paths, the graph is
    absent; we fall back to URDF + kinematic_graph_json as before.
    """
    spatial_context = _build_spatial_context(kinematic_graph_json) if kinematic_graph_json else ""

    if assembly_graph is not None:
        # Canonical-graph path. Keep kinematic_graph_json out too — it's also
        # derived from URDF and carries no fields the assembly graph doesn't.
        user_message = f"""Canonical AssemblyGraph (authoritative — reason and edit against this, not URDF):
```json
{json.dumps(assembly_graph, indent=2)}
```"""
        if spatial_context:
            user_message += f"\n\n{spatial_context}"
        if kinematic_context:
            user_message += f"\n\nRobot Structure Summary:\n{kinematic_context}"
        user_message += f"\n\nUser Request: {prompt}"
        return user_message

    # Legacy / fallback path: no canonical graph available.
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

    return user_message


def generate_edit(prompt: str, current_urdf: str, kinematic_graph_json: dict,
                   kinematic_context: str = None, session_id: str = "default",
                   model: str = "claude-sonnet-4-6",
                   images: list | None = None,
                   assembly_graph: dict | None = None) -> dict:
    """
    Call Claude API to generate a robot edit based on natural language.
    Maintains conversation history per session for multi-turn context.

    Args:
        prompt: User's natural language edit request
        current_urdf: Current URDF XML as string
        kinematic_graph_json: Kinematic graph as dict (from kg.to_json())
        kinematic_context: Optional structured text summary of robot structure from frontend
        session_id: Session identifier for conversation history tracking
        assembly_graph: Optional canonical AssemblyGraph dict (Workstream #1). When present,
            used as the authoritative source of truth; URDF is omitted from the prompt.

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

    user_message = _build_edit_user_message(
        prompt, current_urdf, kinematic_graph_json, kinematic_context, assembly_graph,
    )

    # Build messages array with conversation history
    history = _conversation_history[session_id]
    messages = list(history) + [{"role": "user", "content": _build_user_content(user_message, images)}]

    system_prompt = SYSTEM_PROMPT.replace(
        "{COMPONENT_CATALOG}",
        _get_component_catalog(
            user_prompt=prompt,
            kg_json=kinematic_graph_json,
            tried_preset_ids=_tried_preset_ids(kinematic_graph_json),
        ),
    )

    response = client.messages.create(
        model=model,
        max_tokens=64000,
        system=[{
            "type": "text",
            "text": system_prompt,
            "cache_control": {"type": "ephemeral"},
        }],
        messages=messages,
        tools=ROBOT_TOOLS,
        tool_choice={"type": "any"},  # Force tool use — guarantees structured output
        timeout=180.0,
    )
    # Log cache performance
    _log_cache_usage("generate_edit", response)

    # Store conversation turn in history
    history.append({"role": "user", "content": f"[Edit request] {prompt}"})

    # Handle tool-use response (structured output)
    result = _extract_tool_result(response, current_urdf)

    # Store richer assistant response with tool context
    history.append({"role": "assistant", "content": _build_history_summary(result)})
    while len(history) > _MAX_HISTORY_MESSAGES:
        history.pop(0)

    return result


def generate_edit_streaming(prompt: str, current_urdf: str, kinematic_graph_json: dict,
                             kinematic_context: str = None, session_id: str = "default",
                             on_progress=None,
                             model: str = "claude-sonnet-4-6",
                             images: list | None = None,
                             assembly_graph: dict | None = None) -> dict:
    """
    Streaming version of generate_edit. Calls on_progress(stage, text) as tokens arrive.
    Stages: "thinking", "generating", "applying"

    `assembly_graph`: see generate_edit docstring — Workstream #1 canonical graph.
    """
    client = _get_client()

    user_message = _build_edit_user_message(
        prompt, current_urdf, kinematic_graph_json, kinematic_context, assembly_graph,
    )

    history = _conversation_history[session_id]
    messages = list(history) + [{"role": "user", "content": _build_user_content(user_message, images)}]

    system_prompt = SYSTEM_PROMPT.replace(
        "{COMPONENT_CATALOG}",
        _get_component_catalog(
            user_prompt=prompt,
            kg_json=kinematic_graph_json,
            tried_preset_ids=_tried_preset_ids(kinematic_graph_json),
        ),
    )

    if on_progress:
        on_progress("thinking", "Analyzing model...")

    # Use streaming API with tool-use
    # Stream text for progress, then get final message with tool_use blocks
    try:
        with client.messages.stream(
            model=model,
            max_tokens=64000,
            system=[{
                "type": "text",
                "text": system_prompt,
                "cache_control": {"type": "ephemeral"},
            }],
            messages=messages,
            tools=ROBOT_TOOLS,
            tool_choice={"type": "any"},  # Force tool use — guarantees structured output
        ) as stream:
            sent_generating = False
            # Consume the stream (drives progress updates)
            for text in stream.text_stream:
                if not sent_generating:
                    if on_progress:
                        on_progress("generating", "Generating design...")
                    sent_generating = True

            # Get the complete response including tool_use blocks
            final_response = stream.get_final_message()
            _log_cache_usage("generate_edit_streaming", final_response)

    except Exception as e:
        raise ValueError(f"Streaming API call failed: {e}")

    if on_progress:
        on_progress("applying", "Applying changes...")

    # Store conversation history
    history.append({"role": "user", "content": f"[Edit request] {prompt}"})

    # Extract result from tool-use or text fallback
    result = _extract_tool_result(final_response, current_urdf)

    history.append({"role": "assistant", "content": _build_history_summary(result)})
    while len(history) > _MAX_HISTORY_MESSAGES:
        history.pop(0)

    return result


def generate_edit_turn(
    session_id: str,
    prompt: str | None = None,
    assembly_graph: dict | None = None,
    kinematic_context: str | None = None,
    tool_results: list | None = None,
    model: str = "claude-sonnet-4-6",
    images: list | None = None,
) -> dict:
    """Run ONE turn of the Workstream #2 tool-use edit loop.

    The frontend (`viewportChat.ts`) drives the loop — each call hits Claude
    once, dispatches any tool_use blocks locally with per-call validation, then
    comes back here with the tool_results to let Claude see the outcomes and
    either call another tool or stop with a final text response.

    First-turn shape:
        generate_edit_turn(session_id, prompt="add a depth camera to the head",
                           assembly_graph={...}, ...)
    Subsequent turns:
        generate_edit_turn(session_id, tool_results=[
            {"tool_use_id": "toolu_…", "ok": true,  "summary": "added …"},
            {"tool_use_id": "toolu_…", "ok": false, "code": "PORT_MISMATCH", ...},
        ])

    The per-session raw message buffer lives in `_edit_tool_sessions[session_id]` —
    distinct from `_conversation_history`, which only gets the summary once the
    loop ends (so later non-edit turns don't have to digest raw tool_use blocks).

    Returns: {
        "stop_reason": "tool_use" | "end_turn" | "max_tokens" | ...,
        "text": "…",                              # empty if no text blocks
        "tool_calls": [                           # empty when stop_reason != tool_use
          { "id": "toolu_…", "name": "add_link", "input": {…} }, ...
        ],
        "done": bool,                             # True when the caller should stop looping
    }
    """
    client = _get_client()

    session = _edit_tool_sessions[session_id]
    # Reset session on a new user prompt (first turn of a fresh edit request).
    # Re-using the old buffer across distinct edit requests would confuse Claude
    # (stale tool_use pairs in history) and inflate token count.
    if prompt is not None:
        # Seed with the long-term conversation history so the model keeps
        # multi-edit context ("now move it to the chest" after a prior add).
        # list() copies so downstream mutations on session["messages"] don't
        # touch the shared history deque.
        session["messages"] = list(_conversation_history[session_id])
        session["rounds"] = 0
        # Stash the user's prompt so the final-turn history summary can cite
        # what they actually asked ("add a depth camera") instead of recording
        # "(tool-loop continuation)" — which would happen otherwise because
        # `prompt` is None on continuation turns.
        session["initial_prompt"] = prompt
        # Track whether at least one tool call has fired so the `done` branch
        # can skip history noise when Claude responded without any mutation
        # (the caller then falls through to ai_edit, which records its own turn).
        session["any_tool_used"] = False
        # Snapshot the starting graph for summary/diagnostics. Only read, never
        # mutated here — the frontend owns the canonical graph.
        session["initial_graph"] = assembly_graph

    session["rounds"] = session.get("rounds", 0) + 1
    if session["rounds"] > _MAX_EDIT_TOOL_ROUNDS:
        _edit_tool_sessions.pop(session_id, None)
        return {
            "stop_reason": "max_rounds",
            "text": f"Hit the {_MAX_EDIT_TOOL_ROUNDS}-round safety limit. Stopping the edit loop.",
            "tool_calls": [],
            "done": True,
        }

    messages: list = session.get("messages", [])

    # Turn 1: build the initial user message from prompt + graph + context.
    # Turn N: caller passes tool_results, we marshal them into a user turn.
    if prompt is not None:
        # Keep the same graph-as-source-of-truth framing as _build_edit_user_message
        # — URDF is not passed because tool-call edits operate strictly on the graph.
        parts = []
        if assembly_graph is not None:
            parts.append(
                "Current AssemblyGraph (authoritative — reason and edit against this):\n"
                f"```json\n{json.dumps(assembly_graph, indent=2)}\n```"
            )
        if kinematic_context:
            parts.append(f"Robot Structure Summary:\n{kinematic_context}")
        parts.append(f"User Request: {prompt}")
        parts.append(
            "Use the add_link / attach_sensor / replace_component / set_joint / "
            "remove_link tools to mutate the graph. Each call is validated immediately — "
            "if you see a structured error (PORT_MISMATCH, SENSOR_ON_ACTUATOR, etc.), "
            "adjust and try again in the same turn. When the edit is complete, respond "
            "with a short plain-text confirmation and stop (no further tool calls)."
        )
        user_text = "\n\n".join(parts)
        messages.append({"role": "user", "content": _build_user_content(user_text, images)})
    elif tool_results is not None:
        # Stale-resend guard: a continuation can only be processed if the
        # session's last assistant message contains tool_use blocks with IDs
        # the frontend is replying to. When the loop already ended (session
        # cleared or empty), Anthropic rejects orphan tool_result messages
        # with a 400. Surface a structured error instead so the frontend can
        # restart the loop with a fresh prompt.
        if not messages:
            _edit_tool_sessions.pop(session_id, None)
            return {
                "stop_reason": "session_expired",
                "text": "Edit session expired — resend the request to start a new tool-use loop.",
                "tool_calls": [],
                "done": True,
            }
        # Marshal the frontend-side dispatch results into Anthropic tool_result blocks.
        # The "ok": true branch surfaces the mutation summary + any warnings; the
        # error branch surfaces the rule code + message + suggested_repair so Claude
        # can pattern-match on the code and self-correct.
        blocks = []
        for tr in tool_results:
            tool_use_id = tr.get("tool_use_id")
            if not tool_use_id:
                continue
            if tr.get("ok"):
                payload = {
                    "ok": True,
                    "summary": tr.get("summary", ""),
                    "warnings": tr.get("warnings", []),
                }
            else:
                payload = {
                    "ok": False,
                    "code": tr.get("code", "UNKNOWN"),
                    "message": tr.get("message", ""),
                    "suggested_repair": tr.get("suggested_repair", ""),
                }
            blocks.append({
                "type": "tool_result",
                "tool_use_id": tool_use_id,
                "content": json.dumps(payload),
                "is_error": not tr.get("ok"),
            })
        if blocks:
            messages.append({"role": "user", "content": blocks})
    else:
        raise ValueError("generate_edit_turn requires either prompt (first turn) or tool_results (subsequent turns)")

    system_prompt = SYSTEM_PROMPT.replace("{COMPONENT_CATALOG}", _get_component_catalog())

    response = client.messages.create(
        model=model,
        max_tokens=8192,  # per-turn cap — the full loop is the budget-heavy axis
        system=[{
            "type": "text",
            "text": system_prompt,
            "cache_control": {"type": "ephemeral"},
        }],
        messages=messages,
        tools=EDIT_TOOLS,
        # "auto" instead of "any": after the mutation succeeds, Claude should be
        # able to stop naturally with a text block instead of being forced to
        # keep calling tools. "any" forces tool use every turn — infinite loop.
        tool_choice={"type": "auto"},
        timeout=120.0,
    )
    _log_cache_usage("generate_edit_turn", response)

    # Persist the assistant response verbatim — Anthropic requires the raw blocks
    # (not a summary) to remain in history so subsequent tool_result messages
    # reference valid tool_use_ids.
    assistant_content = response.content
    messages.append({"role": "assistant", "content": assistant_content})
    session["messages"] = messages

    text_parts: list[str] = []
    tool_calls: list[dict] = []
    for block in assistant_content:
        if block.type == "text":
            text_parts.append(block.text)
        elif block.type == "tool_use":
            tool_calls.append({
                "id": block.id,
                "name": block.name,
                "input": block.input,
            })

    if tool_calls:
        session["any_tool_used"] = True

    stop_reason = getattr(response, "stop_reason", "end_turn") or "end_turn"
    done = stop_reason != "tool_use" or not tool_calls

    if done:
        # Loop finished — push a summary to the long-term history for context on
        # future turns, but ONLY when at least one tool actually fired. If Claude
        # answered with pure text (no mutation), the frontend falls through to
        # ai_edit, which records its own history entry; double-recording here
        # would fabricate a "[Used edit tools]" line that didn't correspond to
        # any real mutation.
        any_tool_used = bool(session.get("any_tool_used"))
        initial_prompt = session.get("initial_prompt") or prompt
        if any_tool_used:
            history = _conversation_history[session_id]
            summary_text = " ".join(t for t in text_parts if t).strip() or "Edit complete"
            history.append({"role": "user", "content": f"[Edit request] {initial_prompt or '(tool-loop continuation)'}"})
            history.append({"role": "assistant", "content": f"[Used edit tools] {summary_text}"})
            while len(history) > _MAX_HISTORY_MESSAGES:
                history.pop(0)
        _edit_tool_sessions.pop(session_id, None)

    return {
        "stop_reason": stop_reason,
        "text": " ".join(text_parts).strip(),
        "tool_calls": tool_calls,
        "done": done,
    }


def _build_history_summary(result: dict) -> str:
    """Build a richer history entry so Claude remembers what tool it used and what it built."""
    explanation = result.get("explanation", "Done")
    stats = result.get("stats") or ""

    if "assembly_graph" in result:
        graph = result["assembly_graph"]
        components = graph.get("components", [])
        comp_names = [c.get("link_name", "?") for c in components[:10]]
        comp_list = ", ".join(comp_names)
        if len(components) > 10:
            comp_list += f", ... ({len(components)} total)"
        return f"[Used design_robot] {explanation}. Components: {comp_list}. {stats}"
    elif "topology_ops" in result:
        ops = result["topology_ops"]
        op_summary = ", ".join(f"{o.get('op', '?')} {o.get('link_name', '?')}" for o in ops[:5])
        if len(ops) > 5:
            op_summary += f", ... ({len(ops)} total)"
        return f"[Used modify_topology] {explanation}. Operations: {op_summary}. {stats}"
    else:
        return f"[Response] {explanation}. {stats}"


def _extract_partial_explanation(text: str) -> str:
    """Try to extract the explanation field from partial JSON for live preview."""
    # Look for "explanation": "..." pattern
    match = re.search(r'"explanation"\s*:\s*"((?:[^"\\]|\\.)*)', text)
    if match:
        return match.group(1).replace('\\"', '"').replace('\\n', ' ')
    return ""


def set_conversation_history(session_id: str, messages: list) -> dict:
    """
    Set conversation history for a session from frontend localStorage data.
    Called on reconnect/session start to restore context lost on backend restart.

    Args:
        session_id: Session identifier matching the frontend chat ID
        messages: List of {role, content} dicts from frontend chat history.
                  Roles: 'user', 'assistant', 'system'. System messages are skipped.

    Returns:
        Dict with status and count of messages loaded.
    """
    history = _conversation_history[session_id]

    # Only restore if backend has no history for this session (i.e., it restarted).
    # If history is already populated, skip to preserve enriched _build_history_summary entries.
    if len(history) > 0:
        print(f"[ai_history] Session {session_id} already has {len(history)} messages — skipping resync", file=sys.stderr)
        return {"status": "skipped", "count": len(history)}

    for msg in messages:
        role = msg.get("role", "")
        content = msg.get("content", "")
        if not content or role == "system":
            continue
        if role not in ("user", "assistant"):
            continue
        # Match the format that generate_edit/generate_edit_streaming stores
        if role == "user":
            history.append({"role": "user", "content": f"[Edit request] {content}"})
        else:
            # Wrap assistant messages to indicate tool context (even if we can't
            # reconstruct the exact _build_history_summary format from plain text)
            if content.startswith("[Used "):
                history.append({"role": "assistant", "content": content})
            else:
                history.append({"role": "assistant", "content": f"[Previous response] {content}"})

    # Cap to max history size
    while len(history) > _MAX_HISTORY_MESSAGES:
        history.pop(0)

    # Ensure history ends on an assistant turn (Claude API requires role alternation)
    while history and history[-1]["role"] == "user":
        history.pop()

    count = len(history)
    print(f"[ai_history] Restored {count} messages for session {session_id}", file=sys.stderr)
    return {"status": "ok", "count": count}


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


# ── Assembly Validation (2nd-pass correction) ────────────────────────────────

VALIDATION_SYSTEM_PROMPT = r"""You are a CRITICAL robot assembly validator for Vector IDE. Your job is to find problems, not confirm things look good. Be harsh.

You receive a URDF and 3D viewport screenshots of the assembled robot. The screenshots show the ACTUAL rendered result.

## IMPORTANT: Two Types of Problems

The robot is built in two stages: (1) an AI designs the TOPOLOGY (which components connect to which), then (2) a placement engine computes all 3D positions and angles automatically. You must classify each problem by what caused it:

- **"topology"** = The AI chose wrong components, missed components, or connected things incorrectly. Examples: missing head on a dog, no gripper on an arm, using wrong component type, too few legs. THESE CAN BE FIXED by redesigning the topology.
- **"placement"** = The components are correct but the placement engine positioned them poorly. Examples: legs too close together, body not elevated enough, components overlapping due to small splay angles, parts appearing too small on the grid. THESE CANNOT BE FIXED by the AI — the placement engine handles all coordinates.

## Engine ground truth — do not contradict

When the prompt contains an "Engine ground-truth measurements" section with ICP-gap and/or placement tables, treat those numbers as authoritative. The placement engine already computed the real xyz/rpy applied and the real contact gap along each mate's normal. If a screenshot makes a joint LOOK like it is floating or detached, but the ICP row for that joint reports `gap_p50 < 2mm` at `confidence=high`, the part IS touching — the apparent gap is mesh visual offset or proportions, not placement. Do NOT emit `grounded`, `overlap`, or `direction` failures with numeric magnitudes that contradict a high-confidence gap < 2mm. If two siblings share the same xyz/rpy but one looks displaced, the displacement lives on an ancestor's `attach_rpy` — name the ancestor, do not call the child detached.

## Describe symptoms, do NOT prescribe geometry

Your job is to DESCRIBE what looks wrong. It is NOT to prescribe specific angles, sign flips, axis mirrors, or pose changes. Do NOT write `"mirror legs in pairs"`, `"rotate shoulder by 30°"`, `"flip the rear hip_pitch sign"`, or `"set attach_rpy=[0, π, 0]"`. The downstream topology AI will choose the corrective change — prescriptive geometry suggestions from the validator have historically steered it toward worse poses (e.g. mirroring a quadruped's hip_pitch produces a crossed-leg stance; real quadrupeds keep all four the same direction). Describe the SYMPTOM ("legs collide at the centerline", "torso leans forward", "camera points sideways instead of forward") and stop.

## Check for These Problems in the IMAGE

1. **shape_match**: Does it look like what was requested? (topology: wrong structure. placement: correct structure but poor positioning)
2. **direction**: Components going wrong way? (usually placement — the engine controls orientation)
3. **overlap**: Parts clipping through each other? (usually placement — spacing/offset issue)
4. **completeness**: Missing features the user asked for? (topology — AI forgot a component)
5. **proportions**: Segments wildly wrong sizes? (topology if wrong length_mm specified, placement if correct sizes but bad layout)
6. **grounded**: Floating or buried? (placement — the grounding function handles this)

## Be SPECIFIC in every `detail` string

Generic critiques ("proportions are wrong", "parts overlap") waste a redesign round. Every `detail` MUST:

1. **Name the specific link(s)** involved using URDF `link_name` values (e.g. `structural_baseplate_1`, `sensor_depth_camera_small_1`, `actuator_servo_standard_13_3`). Parse them from the URDF's `<link name="...">` attributes. Do NOT say "the baseplate" — say `structural_baseplate_1`.
2. **Quantify the problem** when possible: include a numeric delta with units — e.g. "baseplate should be ~8cm wider", "thigh_extrusion_2 is 100mm but should be ~180mm for Go1 proportions", "foot_1 is floating ~50mm above the ground plane". Prefer cm or mm. When a ratio is obvious, state it (e.g. "baseplate width 200mm is only 1/5 of robot height 1000mm — needs to be ≥1/3 for stability").
3. **For missing components**, say what's missing AND where it should attach — e.g. "no gripper at end of forearm_extrusion_1:top", not just "missing gripper".
4. **For overlaps/clipping**, name BOTH links involved — e.g. "wheel_front_left_1 clips through structural_baseplate_1 along -Z by ~15mm".
5. **In the top-level `notes` field**, write a 1–2 sentence summary that also names the top 1–3 offending `link_name`s. Example: "Baseplate `structural_baseplate_1` is too narrow for the 900mm-tall arm — widen it or add a pedestal. Also `sensor_depth_camera_small_1` is attached to `actuator_servo_standard_13_3` (a wrist servo shaft) instead of the forearm."

If you cannot determine exact numbers from the image, estimate with a `~` prefix ("~ widen by ~5cm"). Do NOT omit the estimate — even a rough number is more actionable than a vague word.

## Response Format

Return ONLY valid JSON with a structured checklist. Each check MUST include "fixable_by": "topology" or "placement":

{"ok": false, "checklist": [{"check": "shape_match", "pass": true, "detail": "looks like an arm with 3 revolute joints", "fixable_by": "topology"}, {"check": "direction", "pass": false, "detail": "forearm_extrusion_1 points ~30° into the ground plane (should be vertical or pitched up)", "fixable_by": "placement"}, {"check": "completeness", "pass": false, "detail": "no gripper at end of forearm_extrusion_1:top", "fixable_by": "topology"}, {"check": "proportions", "pass": false, "detail": "structural_baseplate_1 is ~200mm wide but total arm height is ~900mm — baseplate should be ~300mm (≥1/3 of height) for stability", "fixable_by": "topology"}, {"check": "grounded", "pass": true, "detail": "sitting on floor", "fixable_by": "placement"}, {"check": "overlap", "pass": true, "detail": "no clipping", "fixable_by": "placement"}], "notes": "Arm is missing a gripper at forearm_extrusion_1:top, and structural_baseplate_1 is underpowered (200mm wide vs 900mm arm height).", "needs_redesign": true}

Set "needs_redesign" to true ONLY if there are topology-fixable failures (missing/wrong components, bad connections). Do NOT set needs_redesign for placement-only issues — the AI cannot fix those, and the frontend will skip the redesign entirely when every failure is placement-fixable. If you flag a design as failing with only placement issues, return "needs_redesign": false and let the placement engine / grounding pass resolve it; triggering a redesign in that case just reshuffles a viable topology.

Set "ok" to true ONLY if ALL checks pass. The checklist must always have these 6 checks.

CRITICAL RULES:
- Do NOT include "edits" with modified xyz/rpy coordinates. You cannot do spatial math.
- Classify every failure as "topology" or "placement" — this determines whether a redesign is triggered.
- Every `detail` for a failing check must name at least one `link_name` from the URDF and include a numeric estimate where applicable.
- Your job is to DESCRIBE what's wrong and WHO can fix it (topology AI vs placement engine).

REMINDER: Return ONLY JSON. Start with { end with }.
"""


def validate_assembly(urdf_content: str, original_prompt: str,
                      session_id: str = "default",
                      screenshot_base64: str = None,
                      screenshots: list = None,
                      reference_images: list = None,
                      engine_summary: dict = None) -> dict:
    """
    Second-pass validation: send assembled URDF + 3 viewport screenshots to Gemini.
    Uses Gemini 3 Flash for visual validation (cheap, fast, good vision, separate rate limits).
    Falls back to Claude Sonnet if Gemini is unavailable.
    Returns dict with 'ok' bool, 'notes' str, 'checklist', and 'needs_redesign'.
    Cost: ~$0.0003 per call (Gemini 3 Flash with 3 images).

    reference_images: optional list of user-uploaded reference images in the
    shape [{"media_type": "image/png", "data": "<base64>"}], forwarded so
    Gemini can compare rendered output against the reference.

    engine_summary: optional engine ground-truth payload of the shape
    {"placements": [{linkName, parentLinkName, xyz, rpy}, ...],
     "icpGaps": [{linkName, parentConnector, childConnector, pairedCount,
                  sampleCount, gapP50Mm, gapP90Mm, gapMinMm, gapMaxMm,
                  nudgeMm, reason, confidence}, ...]}. When present, rendered
    into tables the validator is told not to contradict (Layer 1 of
    docs/VALIDATOR_MEASUREMENT_FEEDBACK.md). The same structure is passed to
    the critique classifier so it can drop validator-misreads that contradict
    a high-confidence ICP gap (Layer 2).
    """
    # Gemini required — no Claude fallback to avoid burning Anthropic tokens/rate limit
    if _genai is None:
        print(f"[ai_validate] google-genai not installed — skipping validation", file=sys.stderr)
        return {"ok": True, "notes": "Validation skipped: google-genai not installed"}
    if not os.environ.get("GEMINI_API_KEY"):
        print(f"[ai_validate] GEMINI_API_KEY not set — skipping validation", file=sys.stderr)
        return {"ok": True, "notes": "Validation skipped: GEMINI_API_KEY not set"}

    # Retry once on transient errors (503 overload, network timeouts)
    for attempt in range(2):
        try:
            raw = _validate_assembly_gemini(urdf_content, original_prompt, screenshot_base64, screenshots, reference_images, engine_summary)
            return _classify_and_enrich(raw, original_prompt, engine_summary)
        except Exception as e:
            err_str = str(e)
            is_transient = '503' in err_str or 'UNAVAILABLE' in err_str or 'timeout' in err_str.lower()
            if is_transient and attempt == 0:
                print(f"[ai_validate] Gemini transient error, retrying in 3s: {e}", file=sys.stderr)
                time.sleep(3)
                continue
            print(f"[ai_validate] Gemini validation failed: {e}", file=sys.stderr)
            return {"ok": True, "notes": f"Validation skipped: Gemini error — {e}"}


def _classify_and_enrich(valresult: dict, original_prompt: str, engine_summary: dict = None) -> dict:
    """
    Run the critique classifier over Gemini's checklist and attach drop flags
    per-item. Infeasible critiques (components not in the catalog) get
    classifier_drop=True + classifier_reason; the TS side uses these to skip
    redesigns that can't be satisfied.

    engine_summary (optional): Layer 2 pass-through so the classifier can drop
    placement-fixable critiques with mm magnitudes that contradict a
    high-confidence ICP gap entry.
    """
    checklist = valresult.get("checklist")
    if not checklist:
        return valresult
    try:
        from core.ai.critique_classifier import classify_checklist
        enriched = classify_checklist(checklist, original_prompt, engine_summary)
        valresult = {**valresult, "checklist": enriched}
    except Exception as e:
        print(f"[ai_validate] critique classifier failed, passing raw checklist: {e}", file=sys.stderr)
    return valresult


def _format_engine_summary_block(engine_summary: dict) -> str:
    """
    Render the engine ground-truth payload as two fixed-width tables for the
    validator prompt. Empty/None input returns ''. Kept compact — long tables
    balloon token counts and the validator only needs representative rows to
    sanity-check screenshot claims.
    """
    if not engine_summary or not isinstance(engine_summary, dict):
        return ""
    placements = engine_summary.get("placements") or []
    icp_gaps = engine_summary.get("icpGaps") or []
    if not placements and not icp_gaps:
        return ""

    lines: list = []
    lines.append("## Engine ground-truth measurements (authoritative)")
    lines.append("")
    lines.append(
        "The two tables below come directly from Vector's placement engine. "
        "Each row is what the engine ACTUALLY wrote/measured, not a screenshot "
        "inference. Treat them as ground truth and do not contradict them "
        "from pixel inspection alone."
    )
    lines.append("")

    if icp_gaps:
        lines.append("### ICP gap table (per mated pair)")
        lines.append("")
        lines.append(
            "`gap_p50/p90` are the median / 90th-percentile face-to-face gaps "
            "in mm along the contact normal. Negative = child slightly "
            "interpenetrates the parent (flush contact). `confidence=high` "
            "means paired-sample ratio ≥75% or the adaptive confident-cap "
            "fired; a high-confidence gap < 2mm means the part IS in contact "
            "regardless of what the screenshot looks like."
        )
        lines.append("")
        lines.append("| link | parent→child connector | gap_p50(mm) | gap_p90(mm) | nudge(mm) | paired | confidence |")
        lines.append("|---|---|---|---|---|---|---|")
        for row in icp_gaps[:200]:  # hard cap; >200 rows is a pathological graph
            p50 = row.get("gapP50Mm")
            p90 = row.get("gapP90Mm")
            paired = f"{row.get('pairedCount', 0)}/{row.get('sampleCount', 0)}"
            lines.append(
                f"| `{row.get('linkName','?')}` | `{row.get('parentConnector','?')}` → `{row.get('childConnector','?')}` | "
                f"{'' if p50 is None else f'{p50:.2f}'} | "
                f"{'' if p90 is None else f'{p90:.2f}'} | "
                f"{row.get('nudgeMm', 0):.2f} | {paired} | {row.get('confidence','?')} |"
            )
        lines.append("")

    if placements:
        lines.append("### Placement table (xyz/rpy as written to URDF, per child)")
        lines.append("")
        lines.append(
            "If two siblings share the same xyz and rpy but the screenshot "
            "shows one displaced, the displacement comes from rotation "
            "accumulated up the ancestor chain (attach_rpy on a parent). "
            "Call out the ancestor, do NOT claim the child is detached."
        )
        lines.append("")
        lines.append("| link | parent | xyz (m) | rpy (rad) |")
        lines.append("|---|---|---|---|")
        for row in placements[:200]:
            lines.append(
                f"| `{row.get('linkName','?')}` | `{row.get('parentLinkName','?')}` | "
                f"{row.get('xyz','?')} | {row.get('rpy','?')} |"
            )
        lines.append("")

    lines.append(
        "**Rules for using these tables:** If the engine reports an ICP gap < 2mm "
        "at high confidence for a joint but the screenshot looks like the child "
        "is \"floating\" by tens of mm, the screenshot is showing mesh visual "
        "offset / proportions, not a placement error. Do NOT emit a `grounded`, "
        "`overlap`, or `direction` failure that contradicts a high-confidence gap."
    )
    lines.append("")
    return "\n".join(lines)


def _validate_assembly_gemini(urdf_content: str, original_prompt: str,
                               screenshot_base64: str = None,
                               screenshots: list = None,
                               reference_images: list = None,
                               engine_summary: dict = None) -> dict:
    """Gemini 3 Flash visual validation. ~$0.0003 per call."""
    client = _get_gemini_client()

    # Build multimodal content parts
    parts = []
    parts.append(_genai_types.Part.from_text(text=VALIDATION_SYSTEM_PROMPT))

    # Reference images from the user's original Claude turn come first so the
    # reference establishes context before Gemini sees the rendered output
    # (G3 fix). Shape: [{"media_type": "image/png", "data": "<base64>"}].
    ref_count = 0
    if reference_images:
        for ref in reference_images:
            if not isinstance(ref, dict):
                continue
            data = ref.get("data")
            media_type = ref.get("media_type") or "image/png"
            if not data:
                continue
            parts.append(_genai_types.Part.from_text(text="**User reference image (target to match):**"))
            parts.append(_genai_types.Part.from_bytes(
                data=base64.b64decode(data),
                mime_type=media_type,
            ))
            ref_count += 1
        if ref_count:
            print(f"[ai_validate] [Gemini] Including {ref_count} reference image(s) from user", file=sys.stderr)

    view_labels = ["Low side view", "Three-quarter view", "Overhead view"]
    has_images = False

    if screenshots and len(screenshots) >= 3:
        for img, label in zip(screenshots[:3], view_labels):
            if img:
                parts.append(_genai_types.Part.from_text(text=f"**{label}:**"))
                parts.append(_genai_types.Part.from_bytes(
                    data=base64.b64decode(img),
                    mime_type='image/png',
                ))
                has_images = True
        total_kb = sum(len(s) for s in screenshots[:3]) // 1024
        print(f"[ai_validate] [Gemini] Including 3 viewport screenshots ({total_kb}KB total)", file=sys.stderr)
    elif screenshot_base64:
        parts.append(_genai_types.Part.from_bytes(
            data=base64.b64decode(screenshot_base64),
            mime_type='image/png',
        ))
        has_images = True
        print(f"[ai_validate] [Gemini] Including 1 viewport screenshot ({len(screenshot_base64) // 1024}KB)", file=sys.stderr)

    if has_images and ref_count:
        view_instruction = (
            "COMPARE THE RENDERED ROBOT AGAINST THE USER REFERENCE IMAGE ABOVE. "
            "Flag gaps between the reference and the output (missing parts, wrong counts, wrong proportions, missing features). "
            "Then examine all 3 rendered views for physical correctness. Be critical."
        )
    elif has_images:
        view_instruction = "EXAMINE ALL 3 VIEWS ABOVE (front-right, rear-left, top-down). Does the assembled robot actually look like what the user asked for? Be critical — check shape from every angle, proportions, direction of components, overlap, and completeness. Find problems."
    else:
        view_instruction = "Check the spatial layout for physical correctness based on the URDF joint origins. Be critical."

    engine_block = _format_engine_summary_block(engine_summary)
    if engine_block:
        print(
            f"[ai_validate] [Gemini] Including engine summary "
            f"(placements={len(engine_summary.get('placements') or [])}, "
            f"icpGaps={len(engine_summary.get('icpGaps') or [])})",
            file=sys.stderr,
        )
    prompt_text = f"""Original user request: "{original_prompt}"

Assembled URDF:
```xml
{urdf_content}
```

{engine_block}
{view_instruction}"""
    parts.append(_genai_types.Part.from_text(text=prompt_text))

    t0 = time.time()
    response = client.models.generate_content(
        model='gemini-3-flash-preview',
        contents=[_genai_types.Content(role="user", parts=parts)],
        config=_genai_types.GenerateContentConfig(
            response_mime_type='application/json',
            temperature=0.2,
        ),
    )
    elapsed = time.time() - t0
    response_text = response.text or ""
    if not response_text:
        return {"ok": True, "notes": "Gemini validation returned empty response"}
    print(f"[ai_validate] [Gemini] Responded in {elapsed:.1f}s: {response_text[:200]}", file=sys.stderr)
    return _process_validation_result(response_text)


def _process_validation_result(response_text: str) -> dict:
    """Shared parsing logic for validation responses from Gemini or Claude."""
    result = _parse_json_response(response_text)
    # Gemini sometimes wraps JSON responses in an array — unwrap it
    if isinstance(result, list):
        result = result[0] if result else {}

    checklist = result.get("checklist", [])
    if checklist:
        failed = [c for c in checklist if not c.get("pass", True)]
        passed = [c for c in checklist if c.get("pass", True)]
        topo_fails = [c for c in failed if c.get("fixable_by") == "topology"]
        placement_fails = [c for c in failed if c.get("fixable_by") == "placement"]
        print(f"[ai_validate] Checklist: {len(passed)} passed, {len(failed)} failed (topology={len(topo_fails)}, placement={len(placement_fails)})", file=sys.stderr)
        for c in failed:
            print(f"[ai_validate]   FAIL [{c.get('fixable_by', '?')}]: {c.get('check')}: {c.get('detail')}", file=sys.stderr)

    if not result.get("ok", True) or result.get("needs_redesign"):
        has_topo_failures = any(
            not c.get("pass", True) and c.get("fixable_by") == "topology"
            for c in checklist
        )
        needs_redesign = result.get("needs_redesign", False) and has_topo_failures
        if not has_topo_failures and result.get("needs_redesign"):
            print(f"[ai_validate] Overriding needs_redesign=false — all failures are placement-only", file=sys.stderr)
        return {
            "ok": False,
            "notes": result.get("notes", "Issues found"),
            "needs_redesign": needs_redesign,
            "checklist": checklist,
        }

    return {
        "ok": True,
        "notes": result.get("notes", "Assembly looks correct"),
        "checklist": checklist,
    }


# ── Sim Script Generator ──────────────────────────────────────────────────────

SIM_SCRIPT_SYSTEM_PROMPT = r"""You generate Python control scripts for a robot simulator.

## Contract

Your output MUST be a single Python module defining exactly this function:

    def step(t, state):
        # return {joint_name: actuator_command, ...}
        return {...}

- `t` is simulation time in seconds (float, starts at 0).
- `state` is a dict; you may ignore it. It is a read-only snapshot.
- Return a dict mapping joint names (strings, exactly as listed under JOINTS) to
  actuator commands. Revolute joints are target angles in radians. Prismatic
  joints are target positions in meters. Continuous joints are raw torque
  commands in N*m, not angle targets. Omitted joints hold their last command.
- For wheeled robots, use WHEEL DRIVE INFO when it is present. For straight
  forward motion command every drive wheel as `throttle * forward_sign`.
  Wheels on the same side must receive the same straight-drive command sign.
  Do not alternate wheel signs by numeric suffix, front/rear position, or guess.
- THROTTLE MAGNITUDE: use 10–20% of max_torque_Nm as your throttle value for
  smooth, stable motion. Example: if max_torque_Nm=3.5, use throttle=0.35–0.70.
  Using full or near-full torque causes wheel slip → oscillating contact forces
  → the robot bounces and jumps. Start low; ramp up slowly if needed.

## Locomotion reference

Use LEG DRIVE INFO when it is present. swing_sign and bend_sign are computed
from the URDF geometry — they tell you which direction is physically correct.
Never guess joint polarity from names or numeric suffixes.
Use the explicit leg role, not chain depth, to decide what each joint does:
- role=leg_swing: forward/back stride joint; command with swing_sign.
- role=leg_bend: stance/clearance joint; bend_sign moves the foot downward.
- role=leg_swing_bend: rare combined joint; apply both effects conservatively.
- role=leg_aux: auxiliary abduction/roll/yaw joint; hold at 0.0 by default.
Chain depth is only root-to-tip order. It is not a reliable hip/knee label.

### Archetype detection

Classify the robot from the joint list before writing code:
- LEG DRIVE INFO present, 4 leg groups (FR/FL/RR/RL) → QUADRUPED
- LEG DRIVE INFO present, 2 leg groups → BIPED
- LEG DRIVE INFO present, 6 leg groups → HEXAPOD
- WHEEL DRIVE INFO present → WHEELED
- joints contain shoulder/elbow/wrist with no leg groups → ARM
- joints contain finger/palm/thumb only → GRIPPER

### QUADRUPED — diagonal trot

Diagonal pairs: Group A = FR + RL, Group B = FL + RR.
Phase A = 0, Phase B = π. FREQ_HZ = 1.5, SETTLE_TIME = 0.8 s.

  w = 2 * math.pi * FREQ_HZ
  ramp = min(1.0, t / SETTLE_TIME)
  sin_a = math.sin(w * t)
  sin_b = math.sin(w * t + math.pi)

For each role=leg_swing joint:
  angle = swing_sign * ramp * HIP_AMP * sin_X       # HIP_AMP = 0.35 rad

For each role=leg_bend joint:
  angle = bend_sign * ramp * (KNEE_BIAS - KNEE_AMP * max(0.0, sin_X))
    # KNEE_BIAS = 0.45 rad (stance extension), KNEE_AMP = 0.30 rad (swing foot clearance)
    # max(0, sin) = rectified. Subtract the swing term because bend_sign points the foot downward.

For each role=leg_aux joint:
  angle = 0.0
  # Do not oscillate abduction/roll/yaw joints for a straight default gait.

The rectified knee is critical. Using raw sin for the knee produces a piston
motion that fights the ground. Always use max(0, sin_X) for the tuck component.

Hold all non-leg joints (neck, tail, spine) at 0.0. The default gait should
move straight forward: do not intentionally turn, yaw, sidestep, or crab-walk.

### BIPED — alternating step

Two leg groups. Phase L = 0, Phase R = π (or vice versa).
FREQ_HZ = 0.8, HIP_AMP = 0.25, KNEE_BIAS = 0.3, KNEE_AMP = 0.2, SETTLE_TIME = 1.0 s.
Same role=leg_swing / role=leg_bend pattern as quadruped. Optionally add a
small counter-rotation only to a clearly marked distal bend/ankle joint.

### HEXAPOD — alternating tripod

Two tripods: A = legs 0, 2, 4; B = legs 1, 3, 5 (by position order).
Phase A = 0, Phase B = π. FREQ_HZ = 1.2, HIP_AMP = 0.3, KNEE_AMP = 0.25,
KNEE_BIAS = 0.35, SETTLE_TIME = 0.6 s. Same role-based swing/bend pattern and
same rectified-knee pattern.

### WHEELED — differential drive

Use WHEEL DRIVE INFO. forward = throttle * forward_sign per wheel.
THROTTLE = 10–20 % of max_torque_Nm. Ramp throttle over SETTLE_TIME = 0.5 s.
Never oscillate individual wheel commands for straight motion.

### ARM — reach and return

Drive joints with slow sinusoids staggered by π/N phases so they move in
sequence. FREQ_HZ = 0.3, amplitude = 35 % of joint range. SETTLE_TIME = 1.0 s.

### GRIPPER — open/close cycle

angle = limit_max * 0.8 * 0.5 * (1 - math.cos(2 * math.pi * FREQ_HZ * t))
FREQ_HZ = 0.25. Ramp over 0.5 s.

## Sandbox — HARD RULES

The script runs in a restricted sandbox. Violations will be rejected.

- **No import statements.** None. `math` is already a global — use `math.sin`,
  `math.pi`, etc. directly, without `import math`.
- No use of: eval, exec, compile, open, __import__, getattr, setattr, delattr,
  globals, locals, vars, input, help.
- No dunder attribute access (anything starting with `__`).
- Only these builtins are available: abs, min, max, round, sum, len, range,
  enumerate, zip, map, filter, sorted, reversed, all, any, int, float, bool,
  str, list, tuple, dict, set, print, isinstance.
- Module-level code (constants, helper functions) is allowed and runs once.
- `step` is called every sim tick; keep it cheap. No unbounded loops.

## Output format

Return ONLY the Python code. No markdown fences, no prose before or after.
Do not include `import math`. Do not wrap in ```python. Just the code."""


def _format_sim_vec(vec) -> str:
    try:
        vals = [float(x) for x in vec[:3]]
    except Exception:
        vals = [0.0, 0.0, 0.0]
    return "(" + ", ".join(f"{v:+.3f}" for v in vals) + ")"


def _format_sim_joint_blocks(joint_names: list, joint_metadata: list | None) -> tuple[str, str]:
    if not joint_metadata:
        return "\n".join(f"  - {n}" for n in joint_names) or "  (none)", ""

    by_name = {
        item.get("name"): item
        for item in joint_metadata
        if isinstance(item, dict) and item.get("name")
    }
    joint_rows = []
    wheel_rows = []
    leg_rows = []

    for name in joint_names:
        meta = by_name.get(name, {})
        jtype = meta.get("type", "unknown")
        if jtype == "continuous":
            control = "torque_Nm"
        elif jtype == "prismatic":
            control = "position_m"
        elif jtype == "revolute":
            control = "position_rad"
        else:
            control = meta.get("control", "command")

        row = f"  - {name}: type={jtype}, control={control}"
        if meta.get("is_wheel_drive"):
            row += (
                f", wheel={meta.get('side', 'unknown')}/{meta.get('end', 'unknown')}, "
                f"forward_sign={int(meta.get('forward_sign', 1)):+d}"
            )
        if meta.get("is_leg"):
            depth = meta.get("leg_depth", 0)
            role = meta.get("leg_role") or (
                "swing_bend" if "swing_sign" in meta and "bend_sign" in meta else
                "swing" if "swing_sign" in meta else
                "bend" if "bend_sign" in meta else
                "aux"
            )
            row += f", leg={meta.get('leg_id', '?')}, chain_depth={depth}, role=leg_{role}"
            if "swing_sign" in meta:
                row += f", swing_sign={int(meta['swing_sign']):+d}"
            if "bend_sign" in meta:
                row += f", bend_sign={int(meta['bend_sign']):+d}"
        joint_rows.append(row)

        if meta.get("is_wheel_drive"):
            effort = float(meta.get("effort", 10.0) or 10.0)
            wheel_rows.append(
                "  - "
                f"joint={name}; side={meta.get('side', 'unknown')}; "
                f"side_sign={int(meta.get('side_sign', 0)):+d}; "
                f"end={meta.get('end', 'unknown')}; "
                f"forward_sign={int(meta.get('forward_sign', 1)):+d}; "
                f"axis_world={_format_sim_vec(meta.get('axis_world', [0, 0, 0]))}; "
                f"center={_format_sim_vec(meta.get('center', [0, 0, 0]))}; "
                f"max_torque_Nm={effort:.3f}"
            )

        if meta.get("is_leg"):
            depth = meta.get("leg_depth", 0)
            role = meta.get("leg_role") or (
                "swing_bend" if "swing_sign" in meta and "bend_sign" in meta else
                "swing" if "swing_sign" in meta else
                "bend" if "bend_sign" in meta else
                "aux"
            )
            leg_entry = (
                f"  - joint={name}; leg={meta.get('leg_id', '?')}; "
                f"chain_depth={depth}; role=leg_{role}"
            )
            if "swing_sign" in meta:
                leg_entry += f"; swing_sign={int(meta['swing_sign']):+d}"
            if "bend_sign" in meta:
                leg_entry += f"; bend_sign={int(meta['bend_sign']):+d}"
            leg_rows.append(leg_entry)

    wheel_block = ""
    if wheel_rows:
        wheel_block = (
            "\n\nWHEEL DRIVE INFO:\n"
            "Use these precomputed signs; do not infer drive direction from numeric suffixes.\n"
            "For straight forward motion, command each wheel joint as throttle * forward_sign.\n"
            "For straight backward motion, negate that same command. Front and rear wheels on\n"
            "the same side should not fight each other during straight drive.\n"
            + "\n".join(wheel_rows)
        )

    leg_block = ""
    if leg_rows:
        leg_block = (
            "\n\nLEG DRIVE INFO:\n"
            "swing_sign and bend_sign are geometrically computed from the URDF - do not guess.\n"
            "Use role=leg_swing for stride, role=leg_bend for crouch/tuck, and hold role=leg_aux at 0.0.\n"
            "swing angle = swing_sign * HIP_AMP * sin(phase)  (straight forward stride)\n"
            "bend angle  = bend_sign * (KNEE_BIAS - KNEE_AMP * max(0, sin(phase)))  (stance + foot clearance)\n"
            + "\n".join(leg_rows)
        )

    return "\n".join(joint_rows) or "  (none)", wheel_block + leg_block


def _leg_role(meta: dict) -> str:
    return (
        meta.get("leg_role")
        or ("swing_bend" if "swing_sign" in meta and "bend_sign" in meta else
            "swing" if "swing_sign" in meta else
            "bend" if "bend_sign" in meta else
            "aux")
    )


def _phase_map_for_legs(leg_ids: list[str]) -> dict[str, float]:
    unique = sorted(set(leg_ids))
    if {"FR", "FL", "RR", "RL"}.issubset(set(unique)):
        return {"FR": 0.0, "RL": 0.0, "FL": 3.141592653589793, "RR": 3.141592653589793}
    if len(unique) == 2:
        return {unique[0]: 0.0, unique[1]: 3.141592653589793}
    if len(unique) == 6:
        return {leg: (0.0 if i % 2 == 0 else 3.141592653589793) for i, leg in enumerate(unique)}
    return {leg: (0.0 if i % 2 == 0 else 3.141592653589793) for i, leg in enumerate(unique)}


def _default_terrain_profile(terrain_config: dict | None) -> dict:
    """
    Return deterministic gait/drive tuning for blank-prompt defaults.
    Flat keeps current behavior; rough/stairs become slower with more clearance.
    """
    terrain_type = str((terrain_config or {}).get("type", "flat")).lower()
    if terrain_type == "stairs":
        return {
            "freq_hz": 0.85,
            "hip_amp": 0.34,
            "knee_bias": 0.50,
            "knee_clearance": 0.34,
            "settle_time_leg": 1.2,
            "wheel_throttle_frac": 0.10,
            "settle_time_wheel": 0.8,
        }
    if terrain_type == "rough":
        return {
            "freq_hz": 1.00,
            "hip_amp": 0.30,
            "knee_bias": 0.46,
            "knee_clearance": 0.31,
            "settle_time_leg": 1.1,
            "wheel_throttle_frac": 0.12,
            "settle_time_wheel": 0.7,
        }
    return {
        "freq_hz": 1.2,
        "hip_amp": 0.26,
        "knee_bias": 0.42,
        "knee_clearance": 0.28,
        "settle_time_leg": 1.0,
        "wheel_throttle_frac": 0.15,
        "settle_time_wheel": 0.5,
    }


def _generate_default_sim_script(joint_names: list, joint_metadata: list | None, terrain_config: dict | None = None) -> str:
    if not joint_metadata:
        return ""
    profile = _default_terrain_profile(terrain_config)

    by_name = {
        item.get("name"): item
        for item in joint_metadata
        if isinstance(item, dict) and item.get("name")
    }

    wheel_joints = []
    leg_swing = []
    leg_bend = []
    leg_aux = []
    leg_ids = []
    hold_joints = []

    for name in joint_names:
        meta = by_name.get(name, {})
        if meta.get("is_wheel_drive"):
            effort = float(meta.get("effort", 10.0) or 10.0)
            wheel_joints.append({
                "joint": name,
                "sign": int(meta.get("forward_sign", 1) or 1),
                "effort": round(effort, 6),
            })
            continue

        if meta.get("is_leg"):
            leg_id = str(meta.get("leg_id", ""))
            if leg_id:
                leg_ids.append(leg_id)
            role = _leg_role(meta)
            if role in ("swing", "swing_bend") and "swing_sign" in meta:
                leg_swing.append({
                    "joint": name,
                    "leg": leg_id,
                    "sign": int(meta.get("swing_sign", 1) or 1),
                })
            if role in ("bend", "swing_bend") and "bend_sign" in meta:
                leg_bend.append({
                    "joint": name,
                    "leg": leg_id,
                    "sign": int(meta.get("bend_sign", 1) or 1),
                })
            if role == "aux":
                leg_aux.append(name)
            continue

        hold_joints.append(name)

    if leg_swing and leg_bend and len(set(leg_ids)) >= 2:
        phases = _phase_map_for_legs(leg_ids)
        return (
            "# Auto-generated default legged locomotion controller.\n"
            "# Positive X/orange axis is treated as forward. Auxiliary lateral joints stay neutral.\n"
            f"FREQ_HZ = {profile['freq_hz']:.2f}\n"
            f"HIP_AMP = {profile['hip_amp']:.2f}\n"
            f"KNEE_BIAS = {profile['knee_bias']:.2f}\n"
            f"KNEE_CLEARANCE = {profile['knee_clearance']:.2f}\n"
            f"SETTLE_TIME = {profile['settle_time_leg']:.1f}\n\n"
            f"LEG_PHASE = {phases!r}\n"
            f"SWING_JOINTS = {leg_swing!r}\n"
            f"BEND_JOINTS = {leg_bend!r}\n"
            f"AUX_JOINTS = {leg_aux!r}\n"
            f"HOLD_JOINTS = {hold_joints!r}\n\n"
            "def step(t, state):\n"
            "    ramp = min(1.0, t / SETTLE_TIME) if SETTLE_TIME > 0 else 1.0\n"
            "    w = 2.0 * math.pi * FREQ_HZ\n"
            "    cmds = {}\n"
            "    for name in HOLD_JOINTS:\n"
            "        cmds[name] = 0.0\n"
            "    for name in AUX_JOINTS:\n"
            "        cmds[name] = 0.0\n"
            "    for row in SWING_JOINTS:\n"
            "        phase = LEG_PHASE.get(row['leg'], 0.0)\n"
            "        s = math.sin(w * t + phase)\n"
            "        # Negate so hips push backward during stance → body travels forward.\n"
            "        cmds[row['joint']] = -row['sign'] * ramp * HIP_AMP * s\n"
            "    for row in BEND_JOINTS:\n"
            "        phase = LEG_PHASE.get(row['leg'], 0.0)\n"
            "        s = math.sin(w * t + phase)\n"
            "        # bend_sign points the foot downward; subtract the swing term for foot clearance.\n"
            "        cmds[row['joint']] = row['sign'] * ramp * (KNEE_BIAS - KNEE_CLEARANCE * max(0.0, s))\n"
            "    return cmds"
        )

    if wheel_joints:
        return (
            "# Auto-generated default wheeled locomotion controller.\n"
            f"SETTLE_TIME = {profile['settle_time_wheel']:.1f}\n"
            f"DRIVE_JOINTS = {wheel_joints!r}\n\n"
            "def step(t, state):\n"
            "    ramp = min(1.0, t / SETTLE_TIME) if SETTLE_TIME > 0 else 1.0\n"
            "    cmds = {}\n"
            "    for row in DRIVE_JOINTS:\n"
            f"        throttle = {profile['wheel_throttle_frac']:.2f} * row['effort']\n"
            "        cmds[row['joint']] = ramp * throttle * row['sign']\n"
            "    return cmds"
        )

    return ""


def generate_sim_script(
    prompt: str,
    joint_names: list,
    current_script: str = "",
    joint_limits: dict = None,
    joint_metadata: list = None,
    terrain_config: dict = None,
) -> str:
    """
    Generate a sim-sandbox Python script from a natural-language prompt.

    Args:
        prompt: user's natural-language description (e.g. "make it trot")
        joint_names: exact joint names the robot exposes
        current_script: if non-empty, treat prompt as a modification request
        joint_limits: optional {name: (lower, upper)} for revolute joints
        joint_metadata: optional per-joint type/axis/wheel direction metadata
        terrain_config: optional terrain settings from the active sim world

    Returns the raw Python source (no fences).
    """
    joints_block, wheel_block = _format_sim_joint_blocks(joint_names, joint_metadata)
    limits_block = ""
    if joint_limits:
        rows = []
        for n in joint_names:
            lim = joint_limits.get(n)
            if lim and all(x is not None for x in lim):
                rows.append(f"  - {n}: [{lim[0]:.3f}, {lim[1]:.3f}]")
        if rows:
            limits_block = "\n\nLIMITS (rad or m):\n" + "\n".join(rows)

    terrain_block = ""
    if isinstance(terrain_config, dict):
        terrain_type = str(terrain_config.get("type", "flat"))
        try:
            height = float(terrain_config.get("height", 0.0))
            scale = float(terrain_config.get("scale", 1.0))
            roughness = float(terrain_config.get("roughness", 0.0))
            friction = float(terrain_config.get("friction", 3.0))
            terrain_block = (
                "\n\nTERRAIN:\n"
                f"  - type: {terrain_type}\n"
                f"  - height_m: {height:.3f}\n"
                f"  - scale: {scale:.2f}\n"
                f"  - roughness: {roughness:.2f}\n"
                f"  - friction: {friction:.2f}\n"
                "Use terrain-aware motion when the request calls for locomotion: "
                "slower, higher-clearance steps for rough ground or stairs; smoother lower motion for flat ground."
            )
        except Exception:
            terrain_block = f"\n\nTERRAIN:\n  - type: {terrain_type}"

    p = prompt.strip()
    if not p and not current_script.strip():
        default_script = _generate_default_sim_script(joint_names, joint_metadata, terrain_config)
        if default_script:
            return default_script

    client = _get_client()

    if current_script.strip():
        request_line = (
            f"MODIFY REQUEST: {p}" if p else
            "MODIFY REQUEST: Improve the script — make the motion smoother, "
            "more natural, and better matched to the robot's morphology."
        )
        user_msg = (
            f"JOINTS:\n{joints_block}{limits_block}{wheel_block}{terrain_block}\n\n"
            f"CURRENT SCRIPT:\n{current_script}\n\n"
            f"{request_line}\n\n"
            f"Return the full modified script. Keep the same joint names, wheel signs, leg signs, and leg roles."
        )
    else:
        if p:
            request_line = f"REQUEST: {p}"
        else:
            request_line = (
                "REQUEST: Classify the robot using the Archetype detection rules in your "
                "instructions, then generate the matching default controller:\n"
                "  - QUADRUPED -> diagonal trot using LEG DRIVE INFO roles and signs\n"
                "  - BIPED     -> alternating step using LEG DRIVE INFO roles and signs\n"
                "  - HEXAPOD   -> alternating tripod using LEG DRIVE INFO roles and signs\n"
                "  - WHEELED   -> smooth forward drive using WHEEL DRIVE INFO forward_sign\n"
                "  - ARM       -> slow staggered reach-and-return sinusoids\n"
                "  - GRIPPER   -> slow open/close cycle\n"
                "  - other     -> gentle sinusoidal idle across all joints\n"
                "Always use the precomputed signs from LEG/WHEEL DRIVE INFO - never guess polarity.\n"
                "For legged robots with no user prompt, make straight forward locomotion: no turn, yaw, sidestep, or crab-walk.\n"
                "Hold role=leg_aux joints at 0.0 unless the user explicitly asks for lateral motion or balancing.\n"
                "Include a SETTLE_TIME ramp so motion eases in from zero."
            )
        user_msg = (
            f"JOINTS:\n{joints_block}{limits_block}{wheel_block}{terrain_block}\n\n"
            f"{request_line}\n\n"
            f"Write a sandbox-compliant script."
        )

    print(f"[ai_gen_sim_script] Calling Claude Sonnet: {prompt[:80]}", file=sys.stderr)
    t0 = time.time()
    response = client.messages.create(
        model="claude-sonnet-4-6",
        max_tokens=3000,
        temperature=0.2,
        system=SIM_SCRIPT_SYSTEM_PROMPT,
        messages=[{"role": "user", "content": user_msg}],
        timeout=60.0,
    )
    elapsed = time.time() - t0
    print(f"[ai_gen_sim_script] Responded in {elapsed:.1f}s", file=sys.stderr)

    text = response.content[0].text.strip()
    # Strip fences if the model ignored instructions.
    if text.startswith("```"):
        text = text.split("\n", 1)[1] if "\n" in text else text[3:]
        if text.endswith("```"):
            text = text.rsplit("```", 1)[0]
        text = text.strip()
    # Strip stray `import math` if it slipped through — math is pre-provided.
    lines = [ln for ln in text.split("\n")
             if not re.match(r"^\s*(import|from)\s+math(\s|$)", ln)]
    return "\n".join(lines).strip()

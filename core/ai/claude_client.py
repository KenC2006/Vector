"""
Claude AI client for robot design editing.
Communicates with Claude API to generate robot model edits based on natural language requests.
"""
import base64
import json
import math
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

# Component IDs the AI may use. Auto-derived from the full preset registry so
# Claude has access to every component the library ships — including parts
# without bespoke GLB meshes (they fall back to parametric box/cylinder
# rendering from bounding_box_mm). The previous hand-curated whitelist of ~40
# components starved novel-archetype designs of variety (e.g. crab legs always
# came back as identical limb_link chains because steppers, BLDCs, soft
# grippers, harmonic drives, swerve modules, CF tubes etc. were hidden).
def _compute_allowed_component_ids() -> set:
    try:
        from core.presets import list_components
        return {c["id"] for c in list_components()}
    except Exception:
        # Conservative floor if the registry can't load — minimum viable robot.
        return {
            'structural_baseplate', 'actuator_servo_standard',
            'structural_limb_link_slim', 'structural_bracket_l',
            'mobility_rubber_foot_pad',
        }

_ALLOWED_COMPONENT_IDS = _compute_allowed_component_ids()

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
                # Phase 3: resolver as source of truth for catalog dims.
                from core.presets import resolve_component_bounds_mm
                bb = resolve_component_bounds_mm(c)
                bb_str = f"{int(bb[0])}x{int(bb[1])}x{int(bb[2])}mm" if len(bb) >= 3 else ""
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

def _get_component_catalog() -> str:
    """Return the full preset catalog text for the system prompt (cached)."""
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

SYSTEM_PROMPT = r"""You are the robot design engine for Vector IDE.

You design robots by specifying TOPOLOGY and INTENT — which components connect to which, where, and in what pose. A deterministic placement compiler turns your specification into 3D positions and URDF. You never write URDF XML or world coordinates (except the explicit raw-placement escape hatch below).

## Coordinate system (URDF, Z-up)

X = forward, Y = left/right, Z = up. Face names on every component:
top = +Z, bottom = -Z, front = +X, back = -X, right = +Y, left = -Y.
`attach_face` picks the DIRECTION the child extends from its parent.

## Available components

{COMPONENT_CATALOG}

## Tools

- **design_robot** — new robot or full redesign. Emit the complete component tree.
- **modify_topology** — iterative edits ("add a camera", "longer arms"). Emit add/remove/modify ops; the engine re-resolves the whole assembly.
When in doubt: if the current robot has real components, prefer modify_topology.

## Placement methods — pick EXACTLY ONE per component

1. **Face mount** (`attach_face`, optionally `orientation`/`elevation_angle`) — the default, right for ~90% of mounts. Children land centered on the parent's face; multiple children on one face auto-distribute RADIALLY around the face center (4 children land on the corners), and bottom-face limbs get automatic outward splay aligned with their radial direction.
2. **Primitive anchor** (`attach_primitive` + `attach_anchor`) — for parents with a `link_geometry` body. Mounts on the NAMED primitive's real surface instead of the body's bounding box. THE method for placing limbs/sensors on custom bodies. Anchors: box `+x_face -x_face +y_face -y_face +z_face -z_face`; cylinder `+axis_end -axis_end tangent_+x tangent_-x tangent_+y tangent_-y tangent_+z tangent_-z`; sphere `+x_pole -x_pole +y_pole -y_pole +z_pole -z_pole`. Example: humanoid shoulder servo -> `attach_primitive: "shoulder_l", attach_anchor: "+axis_end"`.
3. **Named connector** (`attach_connector` on the PARENT / `mate_connector` on the CHILD / `mate_type`) — for authored connectors shown as `conn=[...]` in the catalog. Use `mate_type: "concentric"` for shaft-in-bore (servo `shaft_out` <-> coupler `shaft_hole`); use child-side names like `plate_top`/`wall_inner`/`wall_outer` to pick which L-bracket surface touches the parent. Putting a child-side name into `attach_connector` fails the lookup.
4. **Raw placement** (`xyz` meters + `rpy` radians, parent-relative) — bypasses everything; your values become the joint origin verbatim. The strongest lever; use when face/anchor mounting can't express the design (exact creature poses, asymmetric anatomy, sculpture).

## Orientation controls (face mounts)

- `orientation`: `"vertical"` (default — long axis up), `"horizontal"` (long axis flat: on top/bottom faces it lies along +X; on side faces it extends OUTWARD along the face normal — booms, tails), a number string like `"45"` (yaw around the face normal, works on every face), or `"horizontal+45"` (both).
- `elevation_angle` (degrees): side faces only, tilts up(+)/down(-) from the face normal — e.g. a front camera with `-20` looks at the floor. Not combined with `horizontal`.
- `attach_rpy` [roll, pitch, yaw] radians: the joint's REST POSE. For a servo, only the component about its `joint_axis` is used, applied as the horn's zero offset — the housing stays bolted flat. **Author ONE value per joint ROLE** (all knees `[0, 0.8, 0]`, all hip pitches `[0, -0.4, 0]`): the engine mirrors +/-Y-side pairs automatically so symmetric stances come out symmetric. You may still VARY values along the body (front legs vs rear legs, per-segment tail curl). For deliberately asymmetric per-leg poses, use raw `rpy` instead.
- `rpy` (raw) is the joint frame's MOUNTING orientation; `attach_rpy` is the rest angle within a normal mounting. Most designs only need `attach_rpy`.

## How rotary parts work

Every rotary actuator (servo, BLDC, stepper, DC/gear motor) authors its output shaft in the catalog — `shaft_out(cyl 5.9mm out:+Z@18mm)` means the shaft exits the +Z face 18mm above center. The engine compiles each one into a fixed body + spinning output link at that authored shaft position; your child mounts on the output and rotates with it.
- **Face-mounted** (the default): the engine rotates the whole actuator so its shaft lies along your `joint_axis`, mirrors +/-Y-side pairs so left/right limbs match, and seats the body flush. You pick the face and the semantic axis; the engine owns the body's orientation (`orientation` is ignored for x/y-axis actuators).
- **Anchored / mated / raw-`rpy`**: your frame is kept VERBATIM and the joint spins about that frame's +Z — the physical shaft. Point the anchor/frame where the shaft should point; `joint_axis` does not re-orient a socketed mount.
- **Wheels** lie coin-flat in their local frame (`hub_bore in:+Z`). On a drivetrain (`coaxial`) or as the child of ANY rotary actuator, the engine stands them upright on the shaft — a gearmotor (`continuous`, `y`) with a wheel child is a complete drive assembly.

## Custom bodies: `link_geometry`

The catalog covers FUNCTIONAL hardware; it has no torso/hull/carapace/segment presets. For body silhouettes, set `link_geometry` on a component — a union of primitives that REPLACES its visuals, bounds, collision, and mass (mounting faces and anchors derive from the real shapes):
- `{name:'chest', shape:'box', size_mm:[w,d,h], xyz_mm?, rpy?, color?}`
- `{name:'shoulder_l', shape:'cylinder', radius_mm, length_mm, xyz_mm?, rpy?, color?}` (axis local +Z before rpy)
- `{name:'head', shape:'sphere', radius_mm, xyz_mm?, color?}`
**Always `name` the primitives** — children mount on them via attach_primitive/attach_anchor. Division of labor: presets = mechanical function; link_geometry = body shape. Don't draw a torso as stacked extrusions, and don't route extrusions THROUGH a shell — the shell IS the structure (it carries derived mass and collision).

## Topology rules

1. Exactly one root (`attach_to: null`) — the component matching the robot's structural center. Flat chassis -> `structural_baseplate` (200x150) or `structural_baseplate_large` (350x250, for wide hip spans). Creatures may root on a link_geometry body, an extrusion spine segment, or a hub bracket.
2. Joint types: `continuous` = unbounded torque-controlled spin (drivetrain motors, gearmotor wheel drives, BLDC props); `revolute` = position-controlled joints (servos, BLDC/stepper articulation); everything else = `fixed`.
3. `joint_axis` is the ROTATION axis; the child sweeps perpendicular to it. "y" = pitch (knees, elbows, leg swing, nodding), "x" = roll (lateral abduction, wrist tilt), "z" = yaw (base spin, turret, hip sweep in XY). Never "z" for a knee.
4. **One drive child per rotary actuator.** The engine splits every rotary actuator (servo, BLDC, stepper, gearmotor) into a fixed body + rotating output (plus yoke hardware for x/y axes) and routes your child to the output. Use the bare link_name as `attach_to`; never name `_body`/`_horn` links; never fan out multiple children from one actuator. Drive children mount on its top/bottom (the shaft axis), not its side faces.
5. **Put a bone between revolute joints.** Series revolute joints need a structural link between them (its `length_mm` is the segment length) or the limb collapses to zero length in sim. Exception: a 2-DOF hip/shoulder made of two perpendicular-axis rotary actuators stacked directly — the engine inserts the carrier itself.
6. Wheels: `baseplate -> drivetrain_hub_motor_80 (bottom, continuous, y) -> mobility_wheel_driven (coaxial, fixed)`, or any rotary actuator (`continuous`, `y`) with the wheel as its child. Tires always `attach_face: "coaxial"`, no connector fields, never directly on the chassis. The engine handles outboard offsets and side flips.
7. Foot pads (`mobility_rubber_foot_pad`) are auto-leveled ground contacts: no `attach_rpy`, conventionally terminal (children would inherit the leveling).
8. Sensors mount on structural links, not actuator shafts (they'd spin/vibrate with the joint). For a wrist camera, use the forearm link near its tip.
9. Match servo torque to load: high_torque at root-adjacent joints carrying a limb, standard at distal joints, micro for fine appendages. Vary across limbs when their roles differ.
10. Vary components like real anatomy: different limb lengths, torque tiers, and terminals (grippers as pincers, foot pads for walkers, sensors as feelers). Do not copy one identical chain N times unless the design truly is uniform.
11. Bend limbs with `attach_rpy` on the joint servos; without it a servo->bone->servo->bone chain is a straight stick. Crouches pair a hip rest angle with a knee rest angle (both or neither).
12. **Always set `length_mm` on parametric links** (extrusions, limb links) — the 100mm default is rarely right. Legs: thighs 80-120, shins 100-130. Arms: stem 60-100, upper 180-220, forearm 130-170. Scale to the robot.
13. Electronics (battery, SBC, MCU, drivers, IMU) mount flat on TOP faces of the chassis/body — never below it, never on standoff towers.
14. Build only what the user asked for plus what's functionally required. No cosmetic tails, spoilers, or extra appendages unless requested.
15. Anchored sockets must point AWAY from the body: a cylinder's `+axis_end`/`-axis_end` follows its local +Z after its `rpy` — pick the end outside the body shell, and place socket primitives so they protrude past the body surface.

## Validation feedback

Your output is validated, not silently rewritten. **Errors** (UNKNOWN_COMPONENT, UNKNOWN_PARENT, DUPLICATE_LINK_NAME, MULTIPLE_ROOTS, CYCLE, BAD_PRIMITIVE_REF, BAD_ANCHOR, PRIMITIVE_ON_NON_CAD_BODY) block the build — fix and re-emit. **Warnings** (PORT_MISMATCH, BARE_TIRE, SHAFT_FANOUT, SENSOR_ON_ACTUATOR, EFFECTOR_HAS_CHILDREN, FOOT_PAD_HAS_CHILDREN, SERVO_SPACER, DIRECT_SERVO_STACK, ANCHOR_POINTS_INWARD, TIPPY_PROPORTIONS) come back with a `suggested_repair`: APPLY it on your next turn, or keep the design only when the warning describes a deliberate creative choice (a wheel as decoration, a feeler past a gripper).

## Critical rules

- ALWAYS respond with design_robot or modify_topology. Never raw URDF.
- Use component IDs exactly as listed. Include everything the user asked for.
- You own the design decisions; the engine owns the geometry.
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
                        "length_mm": {"type": "number", "description": "Override length for parametric structural links (default 100mm). For dog/quadruped legs: structural_limb_link_slim 80-120mm. For arm upper/forearm: structural_extrusion_2020 180-220mm upper, 130-170mm forearm."},
                        "orientation": {"type": "string", "description": "Rotation within the face. Keywords: 'vertical' (default, extend +Z), 'horizontal' (extend +X), 'auto'. Or a numeric string in degrees for yaw around the face normal (e.g. '45', '-30'). Combine keyword+degrees as 'horizontal+45'."},
                        "elevation_angle": {"type": "number", "description": "Tilt in degrees for side-face attachments (front/back/left/right only). Positive=up, negative=down. E.g. -20 angles a front camera 20° downward. Ignored on top/bottom faces."},
                        "attach_rpy": {
                            "type": "array",
                            "items": {"type": "number"},
                            "minItems": 3,
                            "maxItems": 3,
                            "description": "Optional [roll, pitch, yaw] in RADIANS applied to the joint origin. Use for rest-pose joint angles (quadruped crouch, splayed shoulders). Example: [0, 0.52, 0] for +30° pitch, [0, -1.05, 0] for -60° pitch. Omit or pass [0,0,0] to let the engine auto-rotate.",
                        },
                        "attach_primitive": {
                            "type": "string",
                            "description": "PRIMITIVE-ANCHOR PLACEMENT (pair with attach_anchor) — the `name` of a primitive on the PARENT's link_geometry to mount this child on. Mounts land on the primitive's REAL surface (a shoulder cylinder's end, a head sphere's pole), not the body's bounding box. Requires the parent's link_geometry entries to carry `name` fields. Validated hard: a name that doesn't exist returns BAD_PRIMITIVE_REF with the available list.",
                        },
                        "attach_anchor": {
                            "type": "string",
                            "description": "Anchor on the named primitive's surface. Box: '+x_face','-x_face','+y_face','-y_face','+z_face','-z_face'. Cylinder: '+axis_end','-axis_end' (along its rotated axis), 'tangent_+x','tangent_-x','tangent_+y','tangent_-y','tangent_+z','tangent_-z' (side-wall points, link-frame directions; tangents parallel to the cylinder axis are invalid). Sphere: '+x_pole','-x_pole','+y_pole','-y_pole','+z_pole','-z_pole'. Example: shoulder servo on a humanoid torso -> attach_primitive='shoulder_l', attach_anchor='+axis_end'.",
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
                        "xyz": {
                            "type": "array",
                            "items": {"type": "number"},
                            "minItems": 3,
                            "maxItems": 3,
                            "description": "RAW PLACEMENT — full [x, y, z] position of this component's joint origin relative to its parent, in METERS. When set, BYPASSES the deterministic face-placement / multi-child distribution / mate-connector resolver entirely — this xyz becomes the URDF joint origin verbatim. Use when the auto-derived placement can't express your design (asymmetric anatomy, exact creature poses, sculpture-style robots). Pair with `rpy` for rotation.",
                        },
                        "rpy": {
                            "type": "array",
                            "items": {"type": "number"},
                            "minItems": 3,
                            "maxItems": 3,
                            "description": "RAW ROTATION — full [roll, pitch, yaw] of this component's joint origin in RADIANS. When set, BYPASSES the auto-computed face-orientation / splay / servo-flip logic entirely — this rpy becomes the mounting frame verbatim, including for rotary actuators (whose joint then spins about this frame's +Z, i.e. the physical shaft; `joint_axis` does not re-orient a raw-placed actuator). Note: this is DIFFERENT from `attach_rpy` — `attach_rpy` is the joint's REST POSE (how the joint is rotated at zero state). `rpy` is the joint origin's mounting orientation. Most designs only need `attach_rpy`; reach for `rpy` only when the auto-orient is fighting your design.",
                        },
                        "link_geometry": {
                            "type": "array",
                            "description": (
                                "CUSTOM BODY SHELL — list of primitive shapes (box / cylinder / sphere) "
                                "that compose this link's rendered appearance. When set, REPLACES the preset's mesh/rendered visuals with a "
                                "free-form union of authored primitives. Bounds, collision envelope, mate-face connectors (top/bottom/front/back/left/right), "
                                "and inertia all auto-derive from the AABB of these primitives, so the resulting link behaves like any other "
                                "catalog component for placement and physics. \n\n"
                                "Use this for body shells, hulls, fairings, plates, domes, and any custom silhouette that no existing preset matches: "
                                "humanoid torso, drone airframe, tank hull, snake/scorpion body segment, robot head, dragon body, sculpture, etc. "
                                "The catalog already covers FUNCTIONAL hardware (servos, brackets, baseplates, limb links, sensors, batteries, wheels, "
                                "grippers); `link_geometry` covers BODY SHAPE. Reach for primitives when the existing extrusions/brackets/plates "
                                "would force you to draw a humanoid torso as stacked beams (it shouldn't be). \n\n"
                                "Each primitive entry is an object: \n"
                                "  - `shape`: 'box' | 'cylinder' | 'sphere' (required) \n"
                                "  - `size_mm`: [w, d, h] in MILLIMETRES for boxes (required for box) \n"
                                "  - `radius_mm`: scalar in MM (required for cylinder and sphere) \n"
                                "  - `length_mm`: scalar in MM along the cylinder's local +Z axis (required for cylinder) \n"
                                "  - `xyz_mm`: optional [x, y, z] OFFSET of this primitive's centre from the link origin, in MILLIMETRES. Default [0,0,0]. \n"
                                "  - `rpy`: optional [roll, pitch, yaw] rotation of this primitive in RADIANS. Default [0,0,0]. \n"
                                "  - `color`: optional [r, g, b] in [0,1]. Default structural-grey. \n\n"
                                "Example — humanoid torso (chest+head+shoulder-stubs+pelvis) all on one link: \n"
                                "  [ {shape:'box', size_mm:[200,100,300], xyz_mm:[0,0,150]}, \n"
                                "    {shape:'sphere', radius_mm:80, xyz_mm:[0,0,360]}, \n"
                                "    {shape:'cylinder', radius_mm:25, length_mm:60, xyz_mm:[100,0,260], rpy:[0,1.5708,0]}, \n"
                                "    {shape:'cylinder', radius_mm:25, length_mm:60, xyz_mm:[-100,0,260], rpy:[0,1.5708,0]}, \n"
                                "    {shape:'box', size_mm:[180,90,80], xyz_mm:[0,0,-40]} ] \n\n"
                                "Example — drone X-frame airframe (central hub + 4 outrigger arms): \n"
                                "  [ {shape:'box', size_mm:[80,80,30]}, \n"
                                "    {shape:'box', size_mm:[20,200,15], xyz_mm:[0,100,0], rpy:[0,0,0.785]}, \n"
                                "    {shape:'box', size_mm:[20,200,15], xyz_mm:[0,100,0], rpy:[0,0,-0.785]} ] \n\n"
                                "Use this for anything where the silhouette matters more than off-the-shelf hardware."
                            ),
                            "items": {
                                "type": "object",
                                "additionalProperties": True,
                            },
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
                        "attach_primitive": {
                            "type": "string",
                            "description": "PRIMITIVE-ANCHOR PLACEMENT (pair with attach_anchor) — the `name` of a primitive on the PARENT's link_geometry to mount this child on. Mounts land on the primitive's REAL surface (a shoulder cylinder's end, a head sphere's pole), not the body's bounding box. Requires the parent's link_geometry entries to carry `name` fields. Validated hard: a name that doesn't exist returns BAD_PRIMITIVE_REF with the available list.",
                        },
                        "attach_anchor": {
                            "type": "string",
                            "description": "Anchor on the named primitive's surface. Box: '+x_face','-x_face','+y_face','-y_face','+z_face','-z_face'. Cylinder: '+axis_end','-axis_end' (along its rotated axis), 'tangent_+x','tangent_-x','tangent_+y','tangent_-y','tangent_+z','tangent_-z' (side-wall points, link-frame directions; tangents parallel to the cylinder axis are invalid). Sphere: '+x_pole','-x_pole','+y_pole','-y_pole','+z_pole','-z_pole'. Example: shoulder servo on a humanoid torso -> attach_primitive='shoulder_l', attach_anchor='+axis_end'.",
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
                        "link_geometry": {
                            "type": "array",
                            "description": "CUSTOM BODY SHELL — same as design_robot.link_geometry. Replaces the preset's rendered visuals with a free-form union of box/cylinder/sphere primitives. Each entry: {shape: 'box'|'cylinder'|'sphere', size_mm | radius_mm + length_mm | radius_mm, xyz_mm?, rpy?, color?}. Use for body shells (torso, hull, fairing, segment) that no preset can express.",
                            "items": {"type": "object", "additionalProperties": True},
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
# add_link — new component at (parent, face)
# attach_sensor — sensor preset on a structural/actuator parent
# replace_component — swap preset_id, preserve topology
# set_joint — change joint type/axis/rest-pose rpy in place
# remove_link — delete subtree or graft children up

ADD_LINK_TOOL = {
    "name": "add_link",
    "description": (
        "Add a new component to the existing robot. Use for structural parts, actuators, and "
        "effectors. For sensors, prefer attach_sensor (it enforces the fixed-joint convention). "
        "The mutation is validated immediately; if the attach would trip port-class incompatibility "
        "(shaft↔mount_face) or SHAFT_FANOUT/SENSOR_ON_ACTUATOR, the mutation lands with a structured "
        "warning carrying a suggested_repair — apply it next call or justify the design. Hard errors "
        "(UNKNOWN_PARENT, DUPLICATE_LINK, BAD_PRIMITIVE_REF, ...) reject and can be retried in the same turn."
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
            "attach_primitive": {
                "type": "string",
                "description": "Primitive-anchor placement: name of a primitive on the parent's link_geometry (pair with attach_anchor).",
            },
            "attach_anchor": {
                "type": "string",
                "description": "Anchor on the named primitive: box +/-x|y|z_face, cylinder +/-axis_end / tangent_+/-x|y|z, sphere +/-x|y|z_pole.",
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
        "actuator's shaft face is accepted with a SENSOR_ON_ACTUATOR warning — prefer a nearby structural extrusion."
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
        "If the new preset's port class doesn't mate with the parent face, the swap lands with a PORT_MISMATCH warning + suggested repair."
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

# Phase 3b.7: _assemble_from_graph(assembly) was deleted (~926 LOC). It had
# zero callers — the actual hot path runs through _extract_tool_result →
# normalize_and_validate(), and the frontend is the single source of truth
# for graph→URDF compilation. Phase 3b.5 will reintroduce a Python entry
# point that calls the shared placementCompiler over a Node subprocess
# rather than re-implementing the geometry rules in Python.

# ───── Tool-Use Assembly Agent ─────────────────────────────────────────────────

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
- For arms: servo(revolute z) -> structural_extrusion_2020(top) -> servo(revolute y) -> structural_extrusion_2020(top) -> gripper. Arm extrusions are VERTICAL at rest (orientation default), not horizontal — joints control the angle.
- For legs: servo(revolute y) on bottom -> structural_limb_link_slim(vertical) -> servo(revolute y) -> structural_limb_link_slim(vertical)
- For slim limb links: do not set attach_rpy/orientation to make the link look flush. The engine mounts `structural_limb_link_slim` on its broad flat face; rest/crouch angles belong on the servo that drives the link.
- For wheels: NEVER attach a tire directly to the baseplate — it needs a spin axis. Two working patterns:
  baseplate -> drivetrain_hub_motor_80 (attach_face="bottom", continuous y) -> mobility_wheel_driven (attach_face="coaxial", fixed), or
  any rotary actuator (e.g. motor_gear_medium_37mm, continuous y) -> mobility_wheel_driven (coaxial, fixed) — the wheel mounts axially on its output shaft.
  The placement engine applies the axial offset and the side-flip automatically — emit the same (coaxial, fixed) annotation for every wheel regardless of corner. Do NOT set `attach_connector`, `mate_connector`, or `mate_type` on a tire; those fields make the engine try to honor your connector choice and the wrong choice puts the wheel inboard.
  Casters: baseplate -> drivetrain_caster_swivel (bottom, fixed) -> mobility_wheel_driven (coaxial, fixed).
  Mecanum: baseplate -> drivetrain_hub_motor_80 (bottom, continuous y) -> mobility_mecanum_wheel (coaxial, fixed).
  Default 4 wheels for any "car/truck/vehicle/rover/buggy/cart" request.
- Use length_mm=80-120 for dog/quadruped leg segments (structural_limb_link_slim) and 180-220 for arm upper-arm / 130-170 for arm forearm (structural_extrusion_2020)

## Important

- Place components ONE AT A TIME. Check the state after each placement.
- If something looks wrong in the state (overlap, wrong position), you can adjust by adding a corrective component.
- Call finish when the robot is complete.
"""

def _execute_add_component(assembly_state: dict, tool_input: dict) -> dict:
    """Append a component to the topology and recompile via the shared compiler.

    Phase 3b.6: placement geometry no longer lives in Python. The AI's
    `add_component` tool call carries pure topology (component_id, parent,
    attach_face, joint_type/axis, optional length_mm/orientation). Each call:

      1. Allocates a deterministic link name (`{component_id}_{N}`, matching
         the frontend's auto-naming counter).
      2. Records the topology entry in `assembly_state["links"]`.
      3. Calls the TypeScript placement compiler over a Node subprocess
         (see core.ai.compiler_client) on the full graph-so-far.
      4. Backfills every link's `origin_xyz` / `origin_rpy` / `world_xyz` /
         `bbox_m` from the compiler output. Servo body/horn splits, multi-
         child distribution, drivetrain side-flips, foot leveling — all
         emerge from the compiler. The AI sees true rendered positions
         instead of Python-heuristic approximations that diverge from the
         frontend.
    """
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
    mass = phys.get("mass_kg") or phys.get("mass_kg_per_100mm") or 0.1

    links = assembly_state.get("links", {})
    idx = len(links) + 1
    link_name = f"{comp_id}_{idx}"

    axis_map = {"x": [1, 0, 0], "y": [0, 1, 0], "z": [0, 0, 1]}
    joint_axis_vec = axis_map.get(joint_axis_str, [0, 0, 1])

    # Topology-only entry. Pose fields get filled in below from the compiler.
    entry = {
        "component_id": comp_id,
        "parent": parent_link,
        "attach_face": attach_face,
        "joint_type": joint_type,
        "joint_axis": joint_axis_vec,
        "joint_axis_name": joint_axis_str,
        "mass_kg": mass,
        # Pose fields populated post-compile:
        "origin_xyz": [0.0, 0.0, 0.0],
        "origin_rpy": [0.0, 0.0, 0.0],
        "world_xyz": [0.0, 0.0, 0.0],
        "bbox_m": [0.0, 0.0, 0.0],
    }
    if length_mm is not None:
        entry["length_mm"] = length_mm
    if orientation:
        entry["orientation"] = orientation
    links[link_name] = entry
    assembly_state["links"] = links

    from core.ai.compiler_client import compile_assembly, CompilerError
    graph = _assembly_graph_from_state(assembly_state)
    try:
        compiled = compile_assembly(graph)
    except CompilerError as e:
        # Roll back the speculative add so the AI's next turn doesn't see a
        # half-placed link.
        del links[link_name]
        return {"error": f"placement compiler failed: {e}"}

    by_logical = {l["logicalName"]: l for l in compiled.get("links", [])}
    for lname, linfo in links.items():
        cl = by_logical.get(lname)
        if not cl:
            continue
        local_xyz = cl.get("localXyz") or [0.0, 0.0, 0.0]
        local_rpy = cl.get("localRpy") or [0.0, 0.0, 0.0]
        world_xyz = cl.get("worldXyz") or local_xyz
        half = (cl.get("bounds") or {}).get("half") or [0.0, 0.0, 0.0]
        linfo["origin_xyz"] = [round(float(v), 4) for v in local_xyz]
        linfo["origin_rpy"] = [round(float(v), 4) for v in local_rpy]
        linfo["world_xyz"] = [round(float(v), 4) for v in world_xyz]
        linfo["bbox_m"] = [round(float(half[i]) * 2, 4) for i in range(3)]

    new_entry = links[link_name]
    state_lines = []
    for lname, linfo in links.items():
        pos = linfo["world_xyz"]
        bb = linfo["bbox_m"]
        state_lines.append(
            f"  {lname}: pos=[{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}] "
            f"bbox={bb[0]:.3f}x{bb[1]:.3f}x{bb[2]:.3f}m "
            f"parent={linfo.get('parent', 'none')}"
        )

    return {
        "success": True,
        "link_name": link_name,
        "placed_at": {
            "xyz": new_entry["origin_xyz"],
            "rpy": new_entry["origin_rpy"],
            "world_xyz": new_entry["world_xyz"],
        },
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
        _get_component_catalog(),
    )

    first_user_content = _build_user_content(f"Build this robot: {prompt}", images)
    messages = [{"role": "user", "content": first_user_content}]
    assembly_state = {"links": {}}

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
                # Phase 3: run the post-LLM contract pipeline (semantic-graph
                # validator + archetype normalizer + diagnostic router) on the
                # AI's raw output so the assembly_graph that flows downstream
                # is already cleaned, and AI-owned diagnostics are surfaced
                # for the next redesign turn.
                from core.ai.semantic_graph import normalize_and_validate
                normalize_and_validate(assembly)
                n = len(assembly["components"])
                print(f"[ai_edit] Tool-use design_robot: {n} components (structured output)", file=sys.stderr)
                ai_feedback = (assembly.get("_diagnostics") or {}).get("ai_feedback")
                explanation = tool_input.get("explanation", "Assembly designed")
                if ai_feedback:
                    explanation = f"{explanation}\n\n{ai_feedback}"
                return {
                    "explanation": explanation,
                    "assembly_graph": assembly,
                    "new_urdf": current_urdf,
                    "stats": tool_input.get("changes_summary", f"{n} components"),
                    "diagnostics": assembly.get("_diagnostics"),
                }
            elif block.name == "modify_topology":
                operations = tool_input.get("operations", [])
                print(f"[ai_edit] Tool-use modify_topology: {len(operations)} operations", file=sys.stderr)
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
        # Phase 3: same post-LLM contract pipeline as the tool-use path.
        from core.ai.semantic_graph import normalize_and_validate
        normalize_and_validate(assembly)
        ai_feedback = (assembly.get("_diagnostics") or {}).get("ai_feedback")
        explanation = result.get("explanation", "Assembly designed")
        if ai_feedback:
            explanation = f"{explanation}\n\n{ai_feedback}"
        return {
            "explanation": explanation,
            "assembly_graph": assembly,
            "new_urdf": current_urdf,
            "stats": result.get("changes_summary", "Assembly ready"),
            "diagnostics": assembly.get("_diagnostics"),
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

def _build_anchor_context(assembly_graph: dict | None) -> str:
    """Per-primitive anchor tables for every link_geometry body in the graph.

    Rendered from core.ai.primitive_anchors (parity-pinned against the TS
    resolver) so the positions Claude reasons about match what the placement
    compiler produces for attach_primitive/attach_anchor mounts."""
    if not isinstance(assembly_graph, dict):
        return ""
    try:
        from core.ai.primitive_anchors import render_anchor_table
    except Exception:
        return ""
    tables: list[str] = []
    for comp in assembly_graph.get("components") or []:
        if not isinstance(comp, dict):
            continue
        prims = comp.get("link_geometry")
        if not isinstance(prims, list) or not prims:
            continue
        table = render_anchor_table(str(comp.get("link_name", "?")), prims)
        if table:
            tables.append(table)
    if not tables:
        return ""
    return "## Primitive anchors (mount points on custom bodies)\n" + "\n".join(tables)


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
        anchor_context = _build_anchor_context(assembly_graph)
        if anchor_context:
            user_message += f"\n\n{anchor_context}"
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

    system_prompt = SYSTEM_PROMPT.replace("{COMPONENT_CATALOG}", _get_component_catalog())

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

    system_prompt = SYSTEM_PROMPT.replace("{COMPONENT_CATALOG}", _get_component_catalog())

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
        anchor_context = _build_anchor_context(assembly_graph)
        if anchor_context:
            parts.append(anchor_context)
        if kinematic_context:
            parts.append(f"Robot Structure Summary:\n{kinematic_context}")
        parts.append(f"User Request: {prompt}")
        parts.append(
            "Use the add_link / attach_sensor / replace_component / set_joint / "
            "remove_link tools to mutate the graph. Each call is validated immediately — "
            "hard errors reject the call; warnings (PORT_MISMATCH, SENSOR_ON_ACTUATOR, ...) land with a "
            "suggested_repair — apply it or proceed deliberately. When the edit is complete, respond "
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

    # Use the first-turn prompt for archetype classification so freedom mode
    # stays consistent across the whole tool-loop (cache stays warm and the
    # rules don't flip mid-loop).
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
    # e.g., prefix has `</collision>` and completion is `\n </link>` -- that's just boilerplate closing
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
). The same structure is passed to
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
- `state` is reserved for future use — its contents are currently undocumented
  and unstable. Write open-loop controllers parameterized only by `t`. Do not
  read from `state`.
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

If the user message contains "RECOMMENDED CONSTANTS", treat those values as
authoritative — they are tuned to this specific robot's geometry. The numeric
defaults later in this prompt are fallbacks for when no recommendations are
provided.

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
- ARM CHAIN INFO present → ARM (use chain_id, depth, role to drive in sequence)
- GRIPPER INFO present → GRIPPER (drive fingers in unison via gripper_root group)
- Multiple categories may be present at once (e.g. WHEELED + ARM); compose
  the per-archetype sections in one returned dict.

### Settle ramp (used by every archetype)

Every archetype eases motion in from zero with a ramp:

  ramp = min(1.0, t / SETTLE_TIME)

Multiply every commanded amplitude by `ramp` so the robot doesn't snap to its
target at t=0. The per-archetype SETTLE_TIME values below are the defaults.

### QUADRUPED — diagonal trot

Diagonal pairs: Group A = FR + RL, Group B = FL + RR.
Phase A = 0, Phase B = π. FREQ_HZ = 1.5, SETTLE_TIME = 0.8 s.

  w = 2 * math.pi * FREQ_HZ
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

Set all non-leg joints (neck, tail, spine) to 0.0 explicitly in the returned
dict — don't rely on omission. "Omitted joints hold their last command" applies
to the previous tick's command, which may be non-zero if the script was
hot-swapped mid-run. The default gait should move straight forward: do not
intentionally turn, yaw, sidestep, or crab-walk.

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

Use WHEEL DRIVE INFO. For straight forward motion:
  command = throttle * forward_sign     # per wheel
THROTTLE = 10–20 % of max_torque_Nm. Ramp throttle over SETTLE_TIME = 0.5 s.
Never oscillate individual wheel commands for straight motion.

For turning, scale opposite-side throttles using the per-wheel `side`:
  left_command  = throttle * forward_sign * (1 - turn_factor)
  right_command = throttle * forward_sign * (1 + turn_factor)
where turn_factor ∈ [-0.5, 0.5]; positive = turn right, negative = turn left.
Only command turning when the user explicitly asks; default is straight.

### ARM — reach and return

Drive joints with slow sinusoids staggered by π/N phases so they move in
sequence. FREQ_HZ = 0.3, SETTLE_TIME = 1.0 s. Read each joint's `(lower, upper)`
from the LIMITS block and command:

  center = 0.5 * (lower + upper)
  amplitude = 0.35 * (upper - lower)
  angle = center + ramp * amplitude * math.sin(w * t + phase_i)

Skip any joint missing from LIMITS (no safe range to clamp against).

### GRIPPER — open/close cycle

For each gripper joint (single-axis or per-finger), drive in unison so all
fingers open and close together:

  angle = upper * 0.8 * 0.5 * (1 - math.cos(2 * math.pi * FREQ_HZ * t))

FREQ_HZ = 0.25. SETTLE_TIME = 0.5 s. Use each joint's `upper` from LIMITS;
fall back to upper = 0.8 rad if LIMITS is missing. Multi-finger grippers with
independent joints should still receive the same command — independent
finger control requires explicit user instruction.

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
- The script must be deterministic. Same `(t, state)` input must produce the
  same output. No randomness, no time-of-day dependencies, no global mutable
  state outside module-level constants.

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
    arm_rows = []
    gripper_rows = []

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
        if meta.get("is_arm_chain"):
            row += (
                f", arm_chain={meta.get('arm_chain_id', '?')}, arm_depth={meta.get('arm_depth', 0)}"
                f", role=arm_{meta.get('arm_role', 'distal')}"
            )
        if meta.get("is_gripper"):
            row += (
                f", gripper={meta.get('gripper_root', '?')}, finger={meta.get('finger_id', 0)}"
                f"/{meta.get('finger_count', 1)}"
            )
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

        if meta.get("is_arm_chain"):
            arm_rows.append(
                f"  - joint={name}; chain={meta.get('arm_chain_id', '?')}; "
                f"depth={meta.get('arm_depth', 0)}; role=arm_{meta.get('arm_role', 'distal')}; "
                f"chain_size={meta.get('arm_chain_size', 1)}; type={jtype}"
            )

        if meta.get("is_gripper"):
            gripper_rows.append(
                f"  - joint={name}; gripper_root={meta.get('gripper_root', '?')}; "
                f"finger={meta.get('finger_id', 0)}/{meta.get('finger_count', 1)}; type={jtype}"
            )

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

    arm_block = ""
    if arm_rows:
        arm_block = (
            "\n\nARM CHAIN INFO:\n"
            "Each chain is a serial sequence of revolute/prismatic joints from a base outward.\n"
            "Roles by depth: arm_base (0), arm_shoulder (1), arm_elbow (2), arm_wrist (3), arm_distal (>=4).\n"
            "Drive each chain with sinusoids staggered by phase = pi * depth / chain_size for sequenced motion.\n"
            "Always center on (lower+upper)/2 from LIMITS and use amplitude = 0.35 * (upper - lower).\n"
            + "\n".join(arm_rows)
        )

    gripper_block = ""
    if gripper_rows:
        gripper_block = (
            "\n\nGRIPPER INFO:\n"
            "Fingers in the same gripper_root group should open/close in unison unless the user asks otherwise.\n"
            "Use a half-cosine open/close cycle scaled to each joint's LIMITS upper bound.\n"
            + "\n".join(gripper_rows)
        )

    return "\n".join(joint_rows) or "  (none)", wheel_block + leg_block + arm_block + gripper_block

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

def _physics_gait_profile(leg_geometry: dict | None, terrain_config: dict | None) -> dict | None:
    """
    Derive gait constants from URDF leg geometry instead of literature defaults.

    The pendulum-frequency formula sqrt(g/L) sets the natural locomotion rate;
    halving it gives a conservative trot. SETTLE_TIME scales with sqrt(L) so
    larger, heavier robots ease in more slowly. Hip and knee amplitudes stay
    in physically reasonable bands regardless of robot size: stride is set as
    a fraction of leg length, and knee retraction in radians is roughly the
    desired foot lift divided by shin length.

    Returns None when leg geometry is unavailable, so the caller falls back
    to the legacy terrain-only profile.
    """
    if not leg_geometry:
        return None
    L = float(leg_geometry.get("mean_leg_length_m", 0.0))
    if L < 0.03:
        return None

    g = 9.81
    w_pend = math.sqrt(g / max(0.05, L))
    # Trot frequency at the pendulum's nat freq (rad/s) over 2π. Matches the
    # legacy 1.2 Hz default at L≈0.18m; smaller legs run faster, larger slower.
    freq_hz = max(0.5, min(2.5, w_pend / (2 * math.pi)))

    # Stride = 55% of leg length → forward foot travel ≈ 0.55 L per cycle.
    hip_amp = math.atan2(0.275 * L, L)

    # KNEE_BIAS is a small offset from URDF-neutral that biases the knee
    # toward stance. Scales mildly with leg length: longer legs sag more.
    knee_bias = max(0.30, min(0.60, 0.40 + 0.5 * max(0.0, L - 0.20)))

    # KNEE_CLEARANCE is the swing-time retraction in radians. Foot lift ≈
    # clearance * shin_length. Targeting ~15% L of clearance gives ~0.30 rad
    # for a typical 2-link leg regardless of size.
    knee_clearance = 0.30

    # Inertia heuristic: settle time scales with sqrt(L) anchored at L=0.18m.
    settle_leg = max(0.5, min(1.6, 0.8 * math.sqrt(max(0.05, L) / 0.18)))

    terrain_type = str((terrain_config or {}).get("type", "flat")).lower()
    if terrain_type == "stairs":
        freq_hz *= 0.7
        knee_clearance += 0.10
        settle_leg += 0.4
    elif terrain_type == "rough":
        freq_hz *= 0.85
        knee_clearance += 0.05
        settle_leg += 0.3

    return {
        "freq_hz": round(freq_hz, 3),
        "hip_amp": round(hip_amp, 3),
        "knee_bias": round(knee_bias, 3),
        "knee_clearance": round(knee_clearance, 3),
        "settle_time_leg": round(settle_leg, 2),
        "wheel_throttle_frac": 0.15,
        "settle_time_wheel": 0.5,
        "_leg_length_m": round(L, 4),
        "_source": "physics",
    }

def _classify_joints_for_default(
    joint_names: list,
    by_name: dict,
    joint_limits: dict | None,
) -> dict:
    """Single pass that buckets joints by category for the default-script handlers."""
    out = {
        "wheel": [], "leg_swing": [], "leg_bend": [], "leg_aux": [], "leg_ids": [],
        "arm": [], "gripper": [], "hold": [],
    }
    for name in joint_names:
        meta = by_name.get(name, {})
        if meta.get("is_wheel_drive"):
            effort = float(meta.get("effort", 10.0) or 10.0)
            out["wheel"].append({
                "joint": name,
                "sign": int(meta.get("forward_sign", 1) or 1),
                "effort": round(effort, 6),
            })
            continue
        if meta.get("is_leg"):
            leg_id = str(meta.get("leg_id", ""))
            if leg_id:
                out["leg_ids"].append(leg_id)
            role = _leg_role(meta)
            if role in ("swing", "swing_bend") and "swing_sign" in meta:
                out["leg_swing"].append({
                    "joint": name, "leg": leg_id,
                    "sign": int(meta.get("swing_sign", 1) or 1),
                })
            if role in ("bend", "swing_bend") and "bend_sign" in meta:
                out["leg_bend"].append({
                    "joint": name, "leg": leg_id,
                    "sign": int(meta.get("bend_sign", 1) or 1),
                })
            if role == "aux":
                out["leg_aux"].append(name)
            continue
        if meta.get("is_gripper"):
            lim = (joint_limits or {}).get(name)
            upper = float(lim[1]) if lim and lim[1] is not None else 0.8
            out["gripper"].append({"joint": name, "upper": round(upper, 4)})
            continue
        if meta.get("is_arm_chain"):
            lim = (joint_limits or {}).get(name)
            if lim and all(x is not None for x in lim):
                lower = float(lim[0]); upper = float(lim[1])
            else:
                lower, upper = -0.8, 0.8
            out["arm"].append({
                "joint": name,
                "chain": str(meta.get("arm_chain_id", "A0")),
                "depth": int(meta.get("arm_depth", 0)),
                "size": int(meta.get("arm_chain_size", 1)),
                "lower": round(lower, 4),
                "upper": round(upper, 4),
            })
            continue
        out["hold"].append(name)
    return out

# Each handler returns either None (no joints in its category) or a dict:
#   {"name": str, "consts": [(NAME, repr-string), ...], "body": [str, ...]}
# where `body` lines are inserted into the shared step() body and may reference
# the constants by name. Handlers must NOT touch each other's joints.

def _legged_section(classified: dict, profile: dict) -> dict | None:
    if not (classified["leg_swing"] and classified["leg_bend"] and len(set(classified["leg_ids"])) >= 2):
        return None
    phases = _phase_map_for_legs(classified["leg_ids"])
    consts = [
        ("LEG_FREQ_HZ", f"{profile['freq_hz']:.2f}"),
        ("LEG_HIP_AMP", f"{profile['hip_amp']:.2f}"),
        ("LEG_KNEE_BIAS", f"{profile['knee_bias']:.2f}"),
        ("LEG_KNEE_CLEARANCE", f"{profile['knee_clearance']:.2f}"),
        ("LEG_SETTLE", f"{profile['settle_time_leg']:.1f}"),
        ("LEG_PHASE", repr(phases)),
        ("LEG_SWING_JOINTS", repr(classified["leg_swing"])),
        ("LEG_BEND_JOINTS", repr(classified["leg_bend"])),
        ("LEG_AUX_JOINTS", repr(classified["leg_aux"])),
    ]
    body = [
        "# --- legged locomotion ---",
        "leg_ramp = min(1.0, t / LEG_SETTLE) if LEG_SETTLE > 0 else 1.0",
        "leg_w = 2.0 * math.pi * LEG_FREQ_HZ",
        "for name in LEG_AUX_JOINTS:",
        "    cmds[name] = 0.0",
        "for row in LEG_SWING_JOINTS:",
        "    phase = LEG_PHASE.get(row['leg'], 0.0)",
        "    s = math.sin(leg_w * t + phase)",
        "    # Negate: hips push backward during stance -> body travels forward.",
        "    cmds[row['joint']] = -row['sign'] * leg_ramp * LEG_HIP_AMP * s",
        "for row in LEG_BEND_JOINTS:",
        "    phase = LEG_PHASE.get(row['leg'], 0.0)",
        "    s = math.sin(leg_w * t + phase)",
        "    cmds[row['joint']] = row['sign'] * leg_ramp * (LEG_KNEE_BIAS - LEG_KNEE_CLEARANCE * max(0.0, s))",
    ]
    return {"name": "legged", "consts": consts, "body": body}

def _wheeled_section(classified: dict, profile: dict) -> dict | None:
    if not classified["wheel"]:
        return None
    consts = [
        ("WHEEL_SETTLE", f"{profile['settle_time_wheel']:.1f}"),
        ("WHEEL_THROTTLE_FRAC", f"{profile['wheel_throttle_frac']:.2f}"),
        ("WHEEL_DRIVE_JOINTS", repr(classified["wheel"])),
    ]
    body = [
        "# --- wheeled drive ---",
        "wheel_ramp = min(1.0, t / WHEEL_SETTLE) if WHEEL_SETTLE > 0 else 1.0",
        "for row in WHEEL_DRIVE_JOINTS:",
        "    throttle = WHEEL_THROTTLE_FRAC * row['effort']",
        "    cmds[row['joint']] = wheel_ramp * throttle * row['sign']",
    ]
    return {"name": "wheeled", "consts": consts, "body": body}

def _arm_section(classified: dict, profile: dict) -> dict | None:
    if not classified["arm"]:
        return None
    arm_profile = profile.get("arm") or {}
    freq_hz = arm_profile.get("freq_hz", 0.30)
    amp_frac = arm_profile.get("amp_frac", 0.35)
    settle = arm_profile.get("settle_time", 1.0)
    consts = [
        ("ARM_FREQ_HZ", f"{freq_hz:.2f}"),
        ("ARM_AMP_FRAC", f"{amp_frac:.2f}"),
        ("ARM_SETTLE", f"{settle:.2f}"),
        ("ARM_JOINTS", repr(classified["arm"])),
    ]
    body = [
        "# --- arm reach ---",
        "arm_ramp = min(1.0, t / ARM_SETTLE) if ARM_SETTLE > 0 else 1.0",
        "arm_w = 2.0 * math.pi * ARM_FREQ_HZ",
        "for row in ARM_JOINTS:",
        "    center = 0.5 * (row['lower'] + row['upper'])",
        "    amp = ARM_AMP_FRAC * (row['upper'] - row['lower'])",
        "    phase = math.pi * row['depth'] / max(1, row['size'])",
        "    cmds[row['joint']] = center + arm_ramp * amp * math.sin(arm_w * t + phase)",
    ]
    return {"name": "arm", "consts": consts, "body": body}

def _gripper_section(classified: dict, profile: dict) -> dict | None:
    if not classified["gripper"]:
        return None
    consts = [
        ("GRIP_FREQ_HZ", "0.25"),
        ("GRIP_OPEN_FRAC", "0.80"),
        ("GRIP_SETTLE", "0.5"),
        ("GRIP_JOINTS", repr(classified["gripper"])),
    ]
    body = [
        "# --- gripper open/close ---",
        "grip_ramp = min(1.0, t / GRIP_SETTLE) if GRIP_SETTLE > 0 else 1.0",
        "grip_cycle = 0.5 * (1.0 - math.cos(2.0 * math.pi * GRIP_FREQ_HZ * t))",
        "for row in GRIP_JOINTS:",
        "    cmds[row['joint']] = grip_ramp * GRIP_OPEN_FRAC * row['upper'] * grip_cycle",
    ]
    return {"name": "gripper", "consts": consts, "body": body}

_DEFAULT_SECTION_HANDLERS = (_legged_section, _wheeled_section, _arm_section, _gripper_section)

def _assemble_default_script(sections: list, hold_joints: list) -> str:
    section_names = ", ".join(s["name"] for s in sections)
    lines = [
        f"# Auto-generated default controller — sections: {section_names}",
        "# Composed from per-archetype handlers; multiple sections coexist if the",
        "# robot mixes archetypes (e.g. wheeled rover with arm).",
    ]
    for sec in sections:
        for n, v in sec["consts"]:
            lines.append(f"{n} = {v}")
    lines.append(f"HOLD_JOINTS = {hold_joints!r}")
    lines.append("")
    lines.append("def step(t, state):")
    lines.append("    cmds = {}")
    lines.append("    for name in HOLD_JOINTS:")
    lines.append("        cmds[name] = 0.0")
    for sec in sections:
        for bl in sec["body"]:
            lines.append("    " + bl)
    lines.append("    return cmds")
    return "\n".join(lines)

def _generate_default_sim_script(
    joint_names: list,
    joint_metadata: list | None,
    terrain_config: dict | None = None,
    leg_geometry: dict | None = None,
    joint_limits: dict | None = None,
    arm_geometry: dict | None = None,
) -> str:
    if not joint_metadata:
        return ""
    profile = _physics_gait_profile(leg_geometry, terrain_config) or _default_terrain_profile(terrain_config)
    profile["arm"] = _arm_physics_profile(arm_geometry)
    by_name = {
        item.get("name"): item
        for item in joint_metadata
        if isinstance(item, dict) and item.get("name")
    }
    classified = _classify_joints_for_default(joint_names, by_name, joint_limits)
    sections = [s for s in (h(classified, profile) for h in _DEFAULT_SECTION_HANDLERS) if s is not None]
    if not sections:
        return ""
    return _assemble_default_script(sections, classified["hold"])

def _arm_physics_profile(arm_geometry: dict | None) -> dict | None:
    """
    Derive arm-controller constants from URDF chain geometry.

    Frequency drops with reach because longer arms have more rotational inertia
    at the shoulder; we approximate that with the inverted-pendulum frequency
    sqrt(g/R) at 25% safety margin (slower than legs because arms aren't
    self-stabilizing). Settle time scales with sqrt(R) so larger arms ease in
    proportionally. Amplitude fraction stays at 35% of joint range — physics
    here is mostly about timing, not range.
    """
    if not arm_geometry:
        return None
    R = float(arm_geometry.get("mean_reach_m", 0.0))
    if R < 0.05:
        return None
    g = 9.81
    w_pend = math.sqrt(g / max(0.05, R))
    freq_hz = max(0.10, min(0.80, 0.25 * w_pend / (2 * math.pi)))
    settle = max(0.6, min(2.5, 1.0 * math.sqrt(max(0.05, R) / 0.30)))
    return {
        "freq_hz": round(freq_hz, 3),
        "amp_frac": 0.35,
        "settle_time": round(settle, 2),
        "_reach_m": round(R, 4),
        "_source": "physics",
    }

def generate_sim_script(
    prompt: str,
    joint_names: list,
    current_script: str = "",
    joint_limits: dict = None,
    joint_metadata: list = None,
    terrain_config: dict = None,
    leg_geometry: dict = None,
    arm_geometry: dict = None,
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

    geometry_block = ""
    physics_profile = _physics_gait_profile(leg_geometry, terrain_config)
    if leg_geometry:
        L = float(leg_geometry.get("mean_leg_length_m", 0.0))
        body_h = float(leg_geometry.get("mean_body_height_m", 0.0))
        n_legs = int(leg_geometry.get("leg_count", 0))
        L_min = float(leg_geometry.get("min_leg_length_m", L))
        L_max = float(leg_geometry.get("max_leg_length_m", L))
        geometry_block = (
            "\n\nGAIT GEOMETRY (measured from URDF neutral pose):\n"
            f"  - leg_count: {n_legs}\n"
            f"  - mean_leg_length_m: {L:.3f}\n"
            f"  - leg_length_range_m: [{L_min:.3f}, {L_max:.3f}]\n"
            f"  - mean_standing_height_m: {body_h:.3f}\n"
        )
        if physics_profile:
            geometry_block += (
                "\nRECOMMENDED CONSTANTS (physics-derived, prefer these over the\n"
                "system-prompt defaults — they are tuned to this robot's actual size):\n"
                f"  - FREQ_HZ: {physics_profile['freq_hz']:.2f}     "
                f"# 0.5 * sqrt(g/L) / (2pi); pendulum natural rate at 50% margin\n"
                f"  - HIP_AMP: {physics_profile['hip_amp']:.2f}     "
                f"# atan(stride/2 / L) for stride = 0.55 * L\n"
                f"  - KNEE_BIAS: {physics_profile['knee_bias']:.2f}    "
                f"# stance offset, bend_sign-relative\n"
                f"  - KNEE_CLEARANCE: {physics_profile['knee_clearance']:.2f}  "
                f"# swing-time retraction, ~15% L foot lift\n"
                f"  - SETTLE_TIME: {physics_profile['settle_time_leg']:.2f}    "
                f"# scales with sqrt(L) to ease in proportionally\n"
            )

    arm_profile = _arm_physics_profile(arm_geometry)
    if arm_geometry:
        n_chains = int(arm_geometry.get("chain_count", 0))
        R = float(arm_geometry.get("mean_reach_m", 0.0))
        R_min = float(arm_geometry.get("min_reach_m", R))
        R_max = float(arm_geometry.get("max_reach_m", R))
        geometry_block += (
            "\n\nARM GEOMETRY (measured from URDF neutral pose):\n"
            f"  - chain_count: {n_chains}\n"
            f"  - mean_reach_m: {R:.3f}\n"
            f"  - reach_range_m: [{R_min:.3f}, {R_max:.3f}]\n"
        )
        if arm_profile:
            geometry_block += (
                "\nRECOMMENDED ARM CONSTANTS (physics-derived from chain reach):\n"
                f"  - ARM_FREQ_HZ: {arm_profile['freq_hz']:.2f}      "
                f"# 0.25 * sqrt(g/R) / (2pi); slower than free pendulum for stability\n"
                f"  - ARM_AMP_FRAC: {arm_profile['amp_frac']:.2f}     "
                f"# fraction of (upper-lower) joint range to sweep\n"
                f"  - ARM_SETTLE: {arm_profile['settle_time']:.2f}      "
                f"# sqrt(R) scaled ease-in\n"
            )

    p = prompt.strip()
    if not p and not current_script.strip():
        default_script = _generate_default_sim_script(joint_names, joint_metadata, terrain_config, leg_geometry, joint_limits, arm_geometry)
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
            f"JOINTS:\n{joints_block}{limits_block}{wheel_block}{terrain_block}{geometry_block}\n\n"
            f"CURRENT SCRIPT:\n{current_script}\n\n"
            f"{request_line}\n\n"
            f"Return the full modified script. Keep the same joint names, wheel signs, leg signs, and leg roles."
        )
    else:
        if p:
            request_line = f"REQUEST: {p}"
        else:
            request_line = (
                "REQUEST: Classify the robot using the Archetype detection rules in "
                "your system instructions and generate the matching default "
                "controller. For unmatched archetypes, emit a gentle sinusoidal "
                "idle across all joints. Straight forward locomotion only — no "
                "turn, yaw, sidestep, or crab-walk."
            )
        user_msg = (
            f"JOINTS:\n{joints_block}{limits_block}{wheel_block}{terrain_block}{geometry_block}\n\n"
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

"""
Robot designer agent: Claude writes an explicit-pose design (build_robot),
the compiler builds it, the critic measures it, and the model revises until
the geometry report is clean (bounded rounds). Returns the final URDF.
"""
from __future__ import annotations

import json
import os
import sys
import time
from typing import Any, Callable, Dict, List, Optional

from core.designer.compile import DesignError, compile_design
from core.designer.importer import ImportError_, import_urdf
from core.designer.critic import critique, format_report
from core.presets import get_all_categories, get_category, resolve_component_bounds_mm, is_parametric_spec

DESIGN_MODEL = os.environ.get("VECTOR_DESIGN_MODEL", "claude-opus-5")
DESIGN_EFFORT = os.environ.get("VECTOR_DESIGN_EFFORT", "medium")
MAX_ROUNDS = 4

BUILD_ROBOT_TOOL = {
    "name": "build_robot",
    "description": (
        "Build the robot from an explicit list of parts (see the system prompt for the part format). "
        "Always send the COMPLETE design. Returns a geometry report measured from the built robot."
    ),
    "input_schema": {
        "type": "object",
        "properties": {
            "name": {"type": "string", "description": "short robot name"},
            "summary": {"type": "string", "description": "one or two sentences on what was built"},
            "parts": {"type": "array", "items": {"type": "object"}, "description": "parts, root body first; parents before children"},
        },
        "required": ["name", "summary", "parts"],
    },
}


def _catalog_text() -> str:
    lines = []
    for cat in get_all_categories():
        comps = get_category(cat)["components"]
        lines.append(f"\n### {cat}")
        for c in comps:
            phys = c.get("physical", {})
            desc = (c.get("description") or "").split(". ")[0][:90]
            if is_parametric_spec(c):
                cs = resolve_component_bounds_mm(c, {"length_mm": 1})
                size = f"cut-to-length: cross-section {cs[0]:g}x{cs[1]:g}mm, length_mm along local Z"
                mass = phys.get("mass_kg_per_100mm")
                mass_s = f"{mass * 1000:.0f}g/100mm" if mass else ""
            else:
                bb = resolve_component_bounds_mm(c)
                size = f"{bb[0]:g}x{bb[1]:g}x{bb[2]:g}mm"
                m = phys.get("mass_kg")
                mass_s = (f"{m:g}kg" if m and m >= 1 else f"{m * 1000:.0f}g") if m else ""
            conns = []
            for k in c.get("connectors", []) or []:
                o, a = k["origin_xyz_mm"], k["axis_xyz"]
                conns.append(f"{k['id']}@({o[0]:g},{o[1]:g},{o[2]:g})->{_axis_word(a)}")
            conn_s = f" conn: {' '.join(conns)}" if conns else ""
            lines.append(f"- {c['id']}: {c['name']} — {size}, {mass_s}. {desc}.{conn_s}")
    return "\n".join(lines)


def _axis_word(a) -> str:
    names = ("x", "y", "z")
    i = max(range(3), key=lambda k: abs(a[k]))
    if abs(abs(a[i]) - 1) < 1e-6:
        return ("+" if a[i] > 0 else "-") + names[i]
    return f"[{a[0]:.2f},{a[1]:.2f},{a[2]:.2f}]"


SYSTEM_PROMPT = r"""You are Vector's robot designer. You design real, buildable robots from a catalog of hardware parts plus custom body shells, and you place every part EXPLICITLY. A small compiler turns your design into a URDF exactly as written — nothing is moved, splayed, mirrored, or re-oriented behind your back. The result is rendered with the real CAD model of each catalog part, so it must look like a believable machine a roboticist would build.

# Coordinates
World frame, millimetres, degrees: +X forward, +Y left, +Z up. Author the robot in its REST pose (standing, wheels down). The compiler lifts the finished robot so its lowest point sits on the ground (z=0) — author the body at roughly its real height.

# Parts
Each part is one object in `parts` (root first; every `parent`/`ref` must name an EARLIER part):
- `name` — unique snake_case (e.g. `hip_fl`, `thigh_fl`). Use `_l`/`_fl`/`_rl`/`left` naming on the left (+Y) side so mirroring renames correctly.
- EITHER `component` — an exact catalog id (cut-to-length parts also need `length_mm`)
  OR `shape` — a custom body shell: list of primitives, each `{name, shape:"box"|"cylinder"|"sphere", size_mm:[x,y,z] | radius_mm (+ length_mm for cylinders, axis = local Z), xyz_mm:[..] (offset from the part origin), rpy_deg:[r,p,y], color:[r,g,b] 0-1}`. Name every primitive.
- `parent` — the part it is bolted to or driven by (omit only on the single root body).
- `at` — WHERE it goes: an absolute point `[x,y,z]`, or `{"ref": "part.anchor", "offset": [dx,dy,dz]}` (offset in world mm, added after the anchor lookup). `"ref": "part"` alone = that part's center.
- `align` — WHICH point of this part lands at `at` (default `center`): a face `+x -x +y -y +z -z` (aliases front/back/left/right/top/bottom) or one of the part's catalog connectors (e.g. `mount_back`, `hub_bore`, `bottom`).
- Orientation, one of:
  - Flush mate (automatic): when BOTH `align` and the `at` anchor are faces/connectors, the part turns so its `align` face presses flat against the anchor face (normals opposed). Optional `x_axis` fixes the spin around that normal; `spin_deg` adds a twist.
  - Explicit: `z_axis` (and optionally `x_axis`) — world directions the part's local +Z/+X point to, as "+x", "-y", "up", or [x,y,z]. Default = identity (local axes = world axes). Explicit axes override the flush mate.
- `joint` — how it moves relative to its parent (default fixed): `{"type": "revolute"|"continuous"|"prismatic", "rest_deg": 0, "lower_deg": -90, "upper_deg": 90}`. The pivot and axis default to the anchor in `at` — so a part placed `at` a motor's `shaft_out` automatically rotates about that shaft. Override with `"pivot": point-or-ref` and `"axis": "+y" | "part.connector"`. `rest_deg` bends the joint (and everything beyond it) to its rest angle about the axis — author limbs STRAIGHT, then bend them with rest_deg (right-hand rule about the axis). `lower_deg`/`upper_deg` are measured the same way (0 = as authored, straight) and must contain rest_deg with room to move both ways — a walking leg needs roughly ±30° of travel around its rest angle at the hip and knee. A joint is DRIVEN by the actuator it hangs off (its parent); a joint whose parent is not an actuator is a free, unmotored pivot in the simulator (rocker-bogie, passive hinge) — set `"passive": false` only if you really mean it to be motorised through some other drive.
- Optional overrides (imported robots carry these; keep them unless asked to change them): `link` (URDF link name; `mirror_link` for the mirrored copy), `mass_kg`, and in `joint`: `name`, `effort`, `velocity`. A shell primitive may also be `{shape:"mesh", filename, scale}` (an external mesh file) and a shell may be empty (`"shape": []`, a bare frame).
- `mirror: true` — also create the mirror image across the XZ plane (y -> -y). Mirrored children of mirrored parents attach to the mirrored parent. Use it for every left/right pair; author the LEFT (+Y) side.

# Anchors
`part.+z`, `part.-y`, … = center of that face of the part's bounding box (outward normal). Catalog parts also expose their connectors (`servo.shaft_out`, `camera.mount_back`, `wheel.hub_bore`). Shells expose primitive faces: `torso.chest.+y` = the +Y face of primitive `chest` (in the shell's frame), and the whole shell's bounding faces `torso.+z`.

# Catalog frame conventions (every part's local frame, before you rotate it)
- Rotary actuators and motors: output shaft on local +Z (`shaft_out`); the body is the bounding box.
- Wheels/tires: a disc in local XY, axle along local Z (`hub_bore` on +Z). To stand a wheel up, its Z must point sideways (±Y).
- Cameras/depth cameras: look along local +X (`optical_front`), mount on -X (`mount_back`).
- Other sensors (ultrasonic, ToF, thermal, bumpers): aim the `sensing_face` connector where the sensor should look; `mount_back` is the opposite side.
- LiDARs: mount on the bottom (-Z), scan plane horizontal.
- Casters, ball transfers, feet: mount on +Z (`mount_top`), touch the ground at -Z (`contact_bottom`).
- Grippers/effectors: mount on -Z (`mount_back`), jaws open toward +Z.
- Cut-to-length beams, tubes, limb links, extrusions: length along local Z, centered.
- Boards, batteries: flat, thin along Z.

# How real mechanisms are assembled (idioms)
- Electronics on a deck: `{"name":"sbc","component":"compute_sbc_small","parent":"body","at":{"ref":"body.+z","offset":[40,0,0]},"align":"bottom"}`
- Servo on the body side, shaft pointing outward: `{"name":"hip_fl","component":"actuator_servo_high_torque","parent":"body","at":{"ref":"body.+y","offset":[120,0,-20]},"align":"bottom","mirror":true}` (its bottom face is flush on the body's side, shaft points +Y).
- A limb segment driven by that servo, hanging down from the shaft, just outboard of the horn: `{"name":"thigh_fl","component":"structural_limb_link_slim","length_mm":120,"parent":"hip_fl","at":{"ref":"hip_fl.shaft_out","offset":[0,4,0]},"align":"+z","z_axis":"+z","joint":{"type":"revolute","rest_deg":-30,"lower_deg":-70,"upper_deg":70},"mirror":true}` — `align:"+z"` puts the segment's top end on the shaft, so it hangs below the pivot.
- A knee servo bolted to the bottom of that segment, shaft along +Y: `{"name":"knee_fl","component":"actuator_servo_standard","parent":"thigh_fl","at":{"ref":"thigh_fl.-z"},"align":"center","z_axis":"+y","mirror":true}` then the next segment `at` `knee_fl.shaft_out` with its own revolute joint.
- Drive wheel: gearmotor inside/under the hull with its shaft poking out the side, wheel mated on the shaft: motor `{"at":{"ref":"hull.+y","offset":[x,-30,z]},"align":"shaft_out","z_axis":"+y"}`, wheel `{"parent":"motor","at":{"ref":"motor.shaft_out"},"align":"hub_bore","joint":{"type":"continuous"}}` (flush mate stands the wheel up on the shaft).
- A part may sit partly inside a shell (motors in a hull, a servo recessed in a hip housing) — that's fine and realistic.

# What makes it look real (you are judged on this)
- Legs that can walk in the simulator: stand with the knees BENT at rest (thigh rest ~-25 to -35°, knee ~+50 to +70°) — a dead-straight leg can't lift its foot. Bipeds additionally need a hip-roll joint and an ankle-pitch joint per leg to balance and shift weight.
- Proportions of the real thing: a small quadruped body is ~300-450mm long with legs long enough that the belly clears the ground by 150-250mm; an arm's links get shorter toward the tip; a rover's wheels are big relative to the hull and every wheel touches the ground.
- Legs hang UNDER/beside the body at its corners, knees bent in a natural stance, all feet on the same ground plane (after grounding every foot must touch z=0 — check the report).
- Every part touches the part it is bolted to. Nothing floats, nothing clips through something it isn't attached to.
- Size actuators for the load: every joint that carries body weight (hips and knees of a walker, an arm's shoulder) needs real torque margin. The report estimates each driven joint's load and flags OVERLOADED ones — pick the stronger actuator rather than the lighter one when in doubt.
- Custom shells give silhouette: a torso, hull, head, fairing. Keep shells simple (2-8 primitives), sized so the hardware mounted on them fits. Soften the silhouette where a real product would: cylinders for rounded shoulders, hips and wheel arches, a sphere or capsule-like head, a chamfer-like step between body and deck.
- Give the robot its own finish, the way a real team would: a palette that fits its purpose and character — e.g. white/light-grey panels with a blue accent for a lab or service robot, white and gold for a planetary rover, safety yellow and dark grey for an industrial arm, olive/sand for a field robot, glossy consumer colours for a toy or pet. Two or three colours, used consistently across its shells.
- Sensors look outward from sensible places: a depth camera on the front/head facing +X, LiDAR on top with a clear view, IMU near the center.
- Wire-less realism: batteries/computers go on a deck or inside/under a cover on the body, never on a moving limb.
- Build what the user asked for plus what it needs to function (power, compute, actuators for every joint). No decorative extras unless asked.

# Process
1. Think through the robot's layout: overall dimensions, where each subsystem goes, the kinematic chains, joint axes.
2. Call build_robot with the complete design. You get back a geometry report: every part's final center/size, ground contacts, stability, sensor directions, and a PROBLEMS list (floating parts, clipping, parts that should/shouldn't touch the ground, tipping).
3. If there are problems — or the numbers show something unrealistic — call build_robot again with the complete corrected design. When the report is clean and the design is right, reply with a short plain-text summary (no tool call).

# Catalog
{CATALOG}
"""


def _system_prompt() -> str:
    return SYSTEM_PROMPT.replace("{CATALOG}", _catalog_text())


def _get_client():
    from core.ai.client import get_client as get
    return get()


def design_robot(prompt: str, current_urdf: str = "", progress: Optional[Callable[[str, str], None]] = None,
                 images: Optional[List[Dict[str, str]]] = None, max_rounds: int = MAX_ROUNDS,
                 log: Optional[List[Dict[str, Any]]] = None) -> Dict[str, Any]:
    """Run the design loop. Returns {urdf, design, report, summary, rounds, issues}."""
    client = _get_client()
    system = [{"type": "text", "text": _system_prompt(), "cache_control": {"type": "ephemeral"}}]

    existing, notes = None, []
    if current_urdf and current_urdf.count("<link") > 1:
        try:
            imp = import_urdf(current_urdf)
            existing, notes = imp["design"], imp["notes"]
        except ImportError_ as e:
            raise RuntimeError(f"The current URDF can't be edited: {e}")
    text = prompt
    if existing:
        note_s = ("\n\nNotes on the current robot:\n" + "\n".join(f"- {n}" for n in notes)) if notes else ""
        text = (f"Current robot design (JSON):\n```json\n{json.dumps(existing)}\n```{note_s}\n\n"
                f"Change request: {prompt}\n\nKeep every part the request doesn't concern exactly as it is "
                f"(same names, placement and joints). Send the complete updated design with build_robot.")
    content: Any = text
    if images:
        content = [{"type": "text", "text": text}] + [
            {"type": "image", "source": {"type": "base64", "media_type": im["media_type"], "data": im["data"]}}
            for im in images if im.get("data") and im.get("media_type")]
    messages: List[Dict[str, Any]] = [{"role": "user", "content": content}]

    best = None  # (issue_count, round, asm, design, report_text, summary, critique)
    summary = ""
    last_built = 0
    for rnd in range(1, max_rounds + 2):
        if progress:
            progress("status", "Designing robot..." if rnd == 1 else f"Refining design (round {rnd})...")
        t0 = time.time()
        with client.messages.stream(
            model=DESIGN_MODEL,
            max_tokens=64000,
            system=system,
            tools=[BUILD_ROBOT_TOOL],
            messages=messages,
            output_config={"effort": DESIGN_EFFORT},
        ) as stream:
            resp = stream.get_final_message()
        print(f"[designer] round {rnd}: {time.time() - t0:.1f}s stop={resp.stop_reason} "
              f"in={resp.usage.input_tokens} out={resp.usage.output_tokens}", file=sys.stderr)
        messages.append({"role": "assistant", "content": resp.content})
        if log is not None:
            log.append({"round": rnd, "stop": resp.stop_reason, "content": [b.model_dump() for b in resp.content]})

        calls = [b for b in resp.content if b.type == "tool_use"]
        texts = "".join(b.text for b in resp.content if b.type == "text").strip()
        if resp.stop_reason == "refusal":
            raise RuntimeError("The design request was declined.")
        if not calls:
            summary = texts or summary
            break
        if rnd > max_rounds:
            break

        results = []
        for call in calls:
            design = call.input if isinstance(call.input, dict) else {}
            if resp.stop_reason == "max_tokens" or not isinstance(design.get("parts"), list):
                results.append({"type": "tool_result", "tool_use_id": call.id, "is_error": True,
                                "content": "The design was cut off or malformed. Send the complete design again, more compactly."})
                continue
            try:
                asm = compile_design(design)
            except DesignError as e:
                results.append({"type": "tool_result", "tool_use_id": call.id, "is_error": True,
                                "content": f"Design could not be built:\n{e}\nFix these and send the complete design again."})
                continue
            rep = critique(asm)
            report = format_report(asm, rep)
            summary = design.get("summary", summary)
            last_built = rnd
            n_issues = len(rep["issues"])
            if best is None or n_issues <= best[0]:
                best = (n_issues, rnd, asm, design, report, summary, rep)
            tail = ("\n\nFix every problem and call build_robot again with the complete design."
                    if n_issues else
                    "\n\nIf the design is right, reply with a short summary (no tool call). Otherwise send an improved complete design.")
            results.append({"type": "tool_result", "tool_use_id": call.id, "content": report + tail})
            if progress:
                progress("status", f"Built {len(asm.parts)} parts — {n_issues} issue(s) to fix" if n_issues else f"Built {len(asm.parts)} parts")
        messages.append({"role": "user", "content": results})
        # A clean build needs no extra round-trip just to hear "looks good".
        if best is not None and best[0] == 0 and best[1] == rnd and all(not r.get("is_error") for r in results):
            break

    if best is None:
        raise RuntimeError("The designer did not produce a buildable robot.")
    n_issues, rnd, asm, design, report, s, rep = best
    # The closing text describes the last build; if an earlier build was kept
    # (fewer problems), describe that one instead.
    return {
        "urdf": asm.to_urdf(design.get("name") or "robot"),
        "design": design,
        "report": report,
        "summary": (summary or s) if rnd == last_built else s,
        "issues": rep["issues"],
        "rounds": rnd,
    }

#!/usr/bin/env python3
"""
JSON-RPC 2.0 server over stdio.

Reads line-delimited JSON from stdin, writes responses to stdout.
Minimal implementation without external JSON-RPC libraries.
"""
import json
import sys
from typing import Any, Dict, Optional
import traceback
import os

# Add current directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Auto-load .env file from project root (if python-dotenv is installed)
try:
    from dotenv import load_dotenv
    # Walk up from core/ to project root to find .env
    _project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    _env_path = os.path.join(_project_root, '.env')
    if os.path.exists(_env_path):
        load_dotenv(_env_path)
        print(f"Loaded .env from {_env_path}", file=sys.stderr)
    else:
        # Also check current working directory
        if os.path.exists('.env'):
            load_dotenv('.env')
            print("Loaded .env from cwd", file=sys.stderr)
except ImportError:
    # dotenv not installed — env vars must be set manually
    pass

# Lazy imports — these modules may have missing dependencies (networkx, etc.)
_parse_urdf = None
_parse_urdf_string = None
_KinematicGraph = None
_serialize_to_urdf = None
_validate_kinematic_graph = None
_model_import_error = None

try:
    from model.urdf_parser import parse_urdf, parse_urdf_string
    from model.kinematic_graph import KinematicGraph
    from model.urdf_serializer import serialize_to_urdf
    from validation.validator import validate_kinematic_graph
    _parse_urdf = parse_urdf
    _parse_urdf_string = parse_urdf_string
    _KinematicGraph = KinematicGraph
    _serialize_to_urdf = serialize_to_urdf
    _validate_kinematic_graph = validate_kinematic_graph
except ImportError as e:
    _model_import_error = str(e)
    print(f"Warning: Model modules not fully available: {e}", file=sys.stderr)
    print(f"  Install missing deps: pip install networkx", file=sys.stderr)

# Lazy import MuJoCo — it may not be installed
_MuJoCoSimulator = None
_mujoco_import_error = None

try:
    from sim.mujoco_adapter import MuJoCoSimulator as _MuJoCoSimulator
except ImportError as e:
    _mujoco_import_error = str(e)
    print(f"Warning: MuJoCo not available: {e}", file=sys.stderr)

# Lazy import AI client — anthropic may not be installed
_generate_edit = None
_generate_edit_streaming = None
_generate_completion = None
_ai_import_error = None

_generate_assembly_with_tools = None
_validate_assembly = None
_set_conversation_history = None
_generate_sim_script = None
_generate_edit_turn = None
try:
    from ai.claude_client import generate_edit as _generate_edit, generate_edit_streaming as _generate_edit_streaming, generate_completion as _generate_completion, generate_assembly_with_tools as _generate_assembly_with_tools, validate_assembly as _validate_assembly, set_conversation_history as _set_conversation_history, generate_sim_script as _generate_sim_script, generate_edit_turn as _generate_edit_turn
except ImportError as e:
    _ai_import_error = str(e)
    print(f"Warning: AI client not available: {e}", file=sys.stderr)

# Always-available local completion engine (no external dependencies)
from ai.local_completions import generate_local_completion as _generate_local_completion


# ── sim_set_script sandbox ────────────────────────────────────────────────────
# Whitelisted builtins available to user-authored sim scripts. Anything not in
# this set (open, exec, eval, __import__, compile, getattr, setattr, ...) is
# unavailable, so a malicious script cannot reach the filesystem, the network,
# or arbitrary Python attributes.
import math as _math
import builtins as _builtins
_SAFE_BUILTINS = {
    name: getattr(_builtins, name)
    for name in (
        "abs", "min", "max", "round", "sum", "len", "range", "enumerate", "zip",
        "map", "filter", "sorted", "reversed", "all", "any",
        "int", "float", "bool", "str", "list", "tuple", "dict", "set",
        "print", "isinstance",
    )
}
_SCRIPT_BASE_GLOBALS: Dict[str, Any] = {
    "__builtins__": _SAFE_BUILTINS,
    "math": _math,
}


def _fresh_script_globals() -> Dict[str, Any]:
    """Return a clean globals sandbox for one script install/validation run."""
    return dict(_SCRIPT_BASE_GLOBALS)
# Names a script may not use, even though they aren't reachable through
# builtins — defense in depth against future _SAFE_BUILTINS additions.
_SCRIPT_NAME_DENY = frozenset({
    "eval", "exec", "compile", "open", "__import__", "getattr", "setattr",
    "delattr", "globals", "locals", "vars", "input", "help",
})


def _sim_xml_child(elem: Any, name: str) -> Optional[Any]:
    for child in list(elem):
        tag = str(getattr(child, "tag", "")).rsplit("}", 1)[-1]
        if tag == name:
            return child
    return None


def _sim_parse_vec(text: Optional[str], default: list) -> list:
    if not text:
        return list(default)
    try:
        vals = [float(x) for x in text.split()]
    except (TypeError, ValueError):
        return list(default)
    if len(vals) < 3:
        return list(default)
    return vals[:3]


def _sim_mat_mul(a: list, b: list) -> list:
    return [
        [
            a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
            for j in range(3)
        ]
        for i in range(3)
    ]


def _sim_mat_vec(m: list, v: list) -> list:
    return [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]


def _sim_rpy_matrix(rpy: list) -> list:
    roll, pitch, yaw = rpy
    cr, sr = _math.cos(roll), _math.sin(roll)
    cp, sp = _math.cos(pitch), _math.sin(pitch)
    cy, sy = _math.cos(yaw), _math.sin(yaw)
    rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    # Match the frontend assembler's Three.js Euler XYZ convention used for
    # generated URDF RPY values.
    return _sim_mat_mul(_sim_mat_mul(rx, ry), rz)


def _sim_vec_add(a: list, b: list) -> list:
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def _sim_dot(a: list, b: list) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sim_cross(a: list, b: list) -> list:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _sim_normalize(v: list) -> list:
    mag = _math.sqrt(max(_sim_dot(v, v), 0.0))
    if mag < 1e-9:
        return [0.0, 0.0, 0.0]
    return [v[0] / mag, v[1] / mag, v[2] / mag]


def _sim_round_vec(v: list) -> list:
    return [round(float(x), 6) for x in v]


def _sim_name_has_any(name: str, tokens: tuple) -> bool:
    lowered = (name or "").lower()
    return any(token in lowered for token in tokens)


def _extract_sim_joint_context(urdf_content: str) -> tuple:
    import xml.etree.ElementTree as _ET

    root = _ET.fromstring(urdf_content)
    links = set()
    child_link_set = set()
    joints = []
    joint_names = []
    joint_limits: Dict[str, tuple] = {}
    children_by_link: Dict[str, list] = {}

    for elem in root.iter():
        tag = str(elem.tag).rsplit("}", 1)[-1]
        if tag == "link":
            name = elem.get("name")
            if name:
                links.add(name)

    for elem in root.iter():
        tag = str(elem.tag).rsplit("}", 1)[-1]
        if tag != "joint":
            continue

        name = elem.get("name")
        jtype = elem.get("type")
        parent_el = _sim_xml_child(elem, "parent")
        child_el = _sim_xml_child(elem, "child")
        parent = parent_el.get("link") if parent_el is not None else ""
        child = child_el.get("link") if child_el is not None else ""
        if not name or not parent or not child:
            continue

        origin_el = _sim_xml_child(elem, "origin")
        axis_el = _sim_xml_child(elem, "axis")
        limit_el = _sim_xml_child(elem, "limit")
        xyz = _sim_parse_vec(origin_el.get("xyz") if origin_el is not None else None, [0, 0, 0])
        rpy = _sim_parse_vec(origin_el.get("rpy") if origin_el is not None else None, [0, 0, 0])
        axis = _sim_normalize(_sim_parse_vec(axis_el.get("xyz") if axis_el is not None else None, [0, 0, 1]))

        limits = {}
        if limit_el is not None:
            for key in ("lower", "upper", "effort", "velocity"):
                raw = limit_el.get(key)
                if raw is None:
                    continue
                try:
                    limits[key] = float(raw)
                except ValueError:
                    pass

        info = {
            "name": name,
            "type": jtype or "",
            "parent": parent,
            "child": child,
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis,
            "limits": limits,
        }
        joints.append(info)
        children_by_link.setdefault(parent, []).append(info)
        links.add(parent)
        links.add(child)
        child_link_set.add(child)

        if jtype not in ("fixed", None):
            joint_names.append(name)
            if "lower" in limits and "upper" in limits:
                joint_limits[name] = (limits["lower"], limits["upper"])

    ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    link_pose: Dict[str, tuple] = {}
    joint_pose: Dict[str, dict] = {}
    roots = sorted(links - child_link_set)
    primary_root = "base_link" if "base_link" in links else (roots[0] if roots else None)

    def visit(link: str, seen: set) -> None:
        if link in seen:
            return
        seen.add(link)
        parent_pos, parent_rot = link_pose.get(link, ([0.0, 0.0, 0.0], ident))
        for joint in children_by_link.get(link, []):
            origin_rot = _sim_rpy_matrix(joint["rpy"])
            child_pos = _sim_vec_add(parent_pos, _sim_mat_vec(parent_rot, joint["xyz"]))
            child_rot = _sim_mat_mul(parent_rot, origin_rot)
            link_pose[joint["child"]] = (child_pos, child_rot)
            joint_pose[joint["name"]] = {
                "center": child_pos,
                "axis_world": _sim_normalize(_sim_mat_vec(child_rot, joint["axis"])),
            }
            visit(joint["child"], seen)

    ordered_roots = []
    if primary_root:
        ordered_roots.append(primary_root)
    ordered_roots.extend(root for root in roots if root != primary_root)
    for root_link in ordered_roots:
        link_pose.setdefault(root_link, ([0.0, 0.0, 0.0], ident))
        visit(root_link, set())

    descendant_cache: Dict[str, set] = {}

    def descendants(link: str, seen: Optional[set] = None) -> set:
        if link in descendant_cache:
            return set(descendant_cache[link])
        if seen is None:
            seen = set()
        if link in seen:
            return set()
        seen.add(link)
        found = {link}
        for child_joint in children_by_link.get(link, []):
            found.update(descendants(child_joint["child"], seen))
        descendant_cache[link] = set(found)
        return found

    wheel_tokens = ("wheel", "tire", "mecanum")
    drivetrain_tokens = ("drivetrain", "hub_motor", "drive_motor")
    forward = [1.0, 0.0, 0.0]
    up = [0.0, 0.0, 1.0]
    joint_metadata = []

    for joint in joints:
        if joint["type"] in ("fixed", None):
            continue

        pose = joint_pose.get(joint["name"], {})
        center = pose.get("center", [0.0, 0.0, 0.0])
        axis_world = pose.get("axis_world", joint["axis"])
        desc = descendants(joint["child"])
        has_wheel_descendant = any(_sim_name_has_any(link, wheel_tokens) for link in desc)
        has_drivetrain_name = (
            _sim_name_has_any(joint["name"], drivetrain_tokens)
            or _sim_name_has_any(joint["parent"], drivetrain_tokens)
            or _sim_name_has_any(joint["child"], drivetrain_tokens)
        )
        is_wheel_drive = bool(has_wheel_descendant and (joint["type"] == "continuous" or has_drivetrain_name))

        rolling_dir = _sim_cross(axis_world, up)
        forward_alignment = _sim_dot(rolling_dir, forward)
        forward_sign = 1 if forward_alignment >= 0 else -1
        if abs(forward_alignment) < 1e-6:
            forward_sign = 1

        x, y, _z = center
        side = "center"
        side_sign = 0
        if y > 0.01:
            side = "left"
            side_sign = 1
        elif y < -0.01:
            side = "right"
            side_sign = -1

        end = "center"
        if x > 0.01:
            end = "front"
        elif x < -0.01:
            end = "rear"

        effort = joint["limits"].get("effort", 10.0)
        joint_metadata.append({
            "name": joint["name"],
            "type": joint["type"],
            "control": "torque_Nm" if joint["type"] == "continuous" else "position",
            "parent": joint["parent"],
            "child": joint["child"],
            "axis_world": _sim_round_vec(axis_world),
            "center": _sim_round_vec(center),
            "effort": round(float(effort), 6),
            "is_wheel_drive": is_wheel_drive,
            "side": side,
            "side_sign": side_sign,
            "end": end,
            "forward_sign": forward_sign,
            "forward_alignment": round(float(forward_alignment), 6),
        })

    # ── Leg kinematic analysis ──────────────────────────────────────────────
    # Annotate non-wheel revolute joints that form leg groups with:
    # is_leg, leg_id (FR/FL/RR/RL/...), leg_depth (0=rootward, increasing outward),
    # leg_role ("swing", "bend", "swing_bend", or "aux"),
    # swing_sign (+1 = positive angle swings foot forward, assigned to joint with largest fwd Jacobian),
    # bend_sign (+1 = positive angle bends knee into stance, foot moves -Z).
    # Signs are computed via the Jacobian: delta_tip = cross(axis_world, tip - center)

    # Compute revolute depth for each joint (# revolute ancestors from root).
    rev_depth_map: Dict[str, int] = {}
    _rd_seen: set = set()

    def _assign_rev_depths(link: str, rev_count: int) -> None:
        if link in _rd_seen:
            return
        _rd_seen.add(link)
        for cj in children_by_link.get(link, []):
            if cj["type"] not in ("fixed", None):
                rev_depth_map[cj["name"]] = rev_count
                _assign_rev_depths(cj["child"], rev_count + 1)
            else:
                _assign_rev_depths(cj["child"], rev_count)

    if primary_root:
        _assign_rev_depths(primary_root, 0)

    # Group non-wheel movable joints by body-relative position → leg_id.
    leg_groups: Dict[str, list] = {}
    for m in joint_metadata:
        if m["is_wheel_drive"]:
            continue
        if m["side"] == "center" and m["end"] == "center":
            continue  # body/neck/tail joints — skip
        leg_key = f"{m['end'][0].upper()}{m['side'][0].upper()}"  # FR, FL, RR, RL, CR, CL …
        leg_groups.setdefault(leg_key, []).append(m)

    # Only treat groups as legs if ≥2 joints/group and ≥2 distinct groups.
    valid_leg_groups = {k: v for k, v in leg_groups.items() if len(v) >= 2}
    if len(valid_leg_groups) >= 2:
        joint_info_by_name = {j["name"]: j for j in joints}
        # Robots are always assembled facing +X (orange axis).
        fwd_axis = 0

        def _tip_pos(child_link: str) -> list:
            """Walk to the leaf of a leg subtree, return its world position."""
            lnk = child_link
            _seen_t: set = set()
            while True:
                if lnk in _seen_t:
                    break
                _seen_t.add(lnk)
                cjs = children_by_link.get(lnk, [])
                if not cjs:
                    break
                lnk = cjs[0]["child"]
            pos, _ = link_pose.get(lnk, ([0.0, 0.0, 0.0], ident))
            return list(pos)

        for leg_id, members in valid_leg_groups.items():
            members.sort(key=lambda m: rev_depth_map.get(m["name"], 99))
            min_depth = rev_depth_map.get(members[0]["name"], 0)

            # First pass: annotate all members and compute Jacobian deltas.
            deltas: list = []
            for m in members:
                m["is_leg"] = True
                m["leg_id"] = leg_id
                m["leg_depth"] = rev_depth_map.get(m["name"], 0) - min_depth
                m["leg_role"] = "aux"
                ji = joint_info_by_name.get(m["name"])
                if ji is None:
                    deltas.append(None)
                    continue
                tip = _tip_pos(ji["child"])
                r = [tip[i] - m["center"][i] for i in range(3)]
                deltas.append(_sim_cross(m["axis_world"], r))

            # Second pass: assign swing_sign to the joint whose Jacobian has the
            # largest forward component (fwd_axis), and bend_sign to the joint
            # whose Jacobian has the largest downward component (-Z).
            # This correctly handles 3-DOF legs where leg_depth=0 may be an
            # abduction joint (side-to-side) rather than the forward-swing joint.
            valid_idx = [i for i, d in enumerate(deltas) if d is not None]
            if valid_idx:
                swing_idx = max(valid_idx, key=lambda i: abs(deltas[i][fwd_axis]))
                bend_idx  = max(valid_idx, key=lambda i: abs(deltas[i][2]))
                d = deltas[swing_idx]
                members[swing_idx]["swing_sign"] = 1 if d[fwd_axis] >= 0 else -1
                members[swing_idx]["leg_role"] = "swing"
                d = deltas[bend_idx]
                members[bend_idx]["bend_sign"]   = 1 if d[2] < 0 else -1
                members[bend_idx]["leg_role"] = (
                    "swing_bend" if bend_idx == swing_idx else "bend"
                )

    return joint_names, joint_limits, joint_metadata


class JSONRPCServer:
    """Simple JSON-RPC 2.0 server."""

    def __init__(self):
        """Initialize the server."""
        self.simulator = _MuJoCoSimulator() if _MuJoCoSimulator else None
        # Script runner state (Phase C)
        self.sim_script_fn = None   # compiled step(t, state) callable or None
        self.sim_script_error: Optional[str] = None
        # Per-server script globals sandbox; reset on clear/install to avoid stale
        # constants/helpers leaking across script sessions.
        self._script_globals: Dict[str, Any] = _fresh_script_globals()
        self.methods = {
            "parse_urdf": self.handle_parse_urdf,
            "ping": self.handle_ping,
            "sim_load": self.handle_sim_load,
            "sim_step": self.handle_sim_step,
            "sim_reset": self.handle_sim_reset,
            "sim_set_control": self.handle_sim_set_control,
            "sim_get_state": self.handle_sim_get_state,
            "sim_render": self.handle_sim_render,
            "sim_set_gravity": self.handle_sim_set_gravity,
            "sim_set_script": self.handle_sim_set_script,
            "validate_urdf": self.handle_validate_urdf,
            "validate_urdf_content": self.handle_validate_urdf_content,
            "ai_edit": self.handle_ai_edit,
            "ai_edit_turn": self.handle_ai_edit_turn,
            "ai_complete": self.handle_ai_complete,
            "ai_validate_assembly": self.handle_ai_validate_assembly,
            "ai_set_history": self.handle_ai_set_history,
            "ai_gen_sim_script": self.handle_ai_gen_sim_script,
        }

    def handle_parse_urdf(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Parse a URDF file and return the kinematic graph as JSON.

        Params:
            path (str): Path to the URDF file.

        Returns:
            Kinematic graph JSON.
        """
        if "path" not in params:
            raise ValueError("Missing required parameter: path")

        path = params["path"]
        if not isinstance(path, str):
            raise ValueError("Parameter 'path' must be a string")

        if _parse_urdf is None:
            raise ValueError(f"Model modules not available: {_model_import_error}")

        try:
            kg = _parse_urdf(path)
            return kg.to_json()
        except FileNotFoundError as e:
            raise ValueError(f"File not found: {e}")
        except Exception as e:
            raise ValueError(f"Failed to parse URDF: {e}")

    def handle_validate_urdf(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Parse a URDF file and run validation checks.

        Params:
            path (str): Path to the URDF file.

        Returns:
            Dict with 'results' list and 'summary' counts.
        """
        if "path" not in params:
            raise ValueError("Missing required parameter: path")

        path = params["path"]
        if not isinstance(path, str):
            raise ValueError("Parameter 'path' must be a string")

        if _parse_urdf is None or _validate_kinematic_graph is None:
            raise ValueError(f"Model modules not available: {_model_import_error}")

        try:
            kg = _parse_urdf(path)
            results = _validate_kinematic_graph(kg)

            # Build summary counts
            summary = {"pass": 0, "warn": 0, "error": 0, "info": 0}
            for r in results:
                sev = r.get("severity", "info")
                if sev in summary:
                    summary[sev] += 1

            return {"results": results, "summary": summary}
        except FileNotFoundError as e:
            raise ValueError(f"File not found: {e}")
        except Exception as e:
            raise ValueError(f"Validation failed: {e}")

    def handle_validate_urdf_content(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate URDF content from a string (not a file path).
        Used by the frontend editor to validate as-you-type.

        Params:
            urdf_content (str): URDF XML as a string.

        Returns:
            Dict with 'results' list and 'summary' counts.
        """
        if "urdf_content" not in params:
            raise ValueError("Missing required parameter: urdf_content")

        urdf_content = params["urdf_content"]
        if not isinstance(urdf_content, str):
            raise ValueError("Parameter 'urdf_content' must be a string")

        if _parse_urdf_string is None or _validate_kinematic_graph is None:
            return {
                "results": [{
                    "name": "Dependencies Missing",
                    "severity": "warn",
                    "message": f"Model modules not available: {_model_import_error}. Install: pip install networkx",
                    "category": "Setup"
                }],
                "summary": {"pass": 0, "warn": 1, "error": 0, "info": 0}
            }

        try:
            kg = _parse_urdf_string(urdf_content)
            results = _validate_kinematic_graph(kg)

            # Build summary counts
            summary = {"pass": 0, "warn": 0, "error": 0, "info": 0}
            for r in results:
                sev = r.get("severity", "info")
                if sev in summary:
                    summary[sev] += 1

            return {"results": results, "summary": summary}
        except Exception as e:
            # If parsing fails, return a parse error
            return {
                "results": [{
                    "name": "URDF Parse Error",
                    "severity": "error",
                    "message": str(e),
                    "category": "Structural"
                }],
                "summary": {"pass": 0, "warn": 0, "error": 1, "info": 0}
            }

    def handle_ping(self, params: Dict[str, Any]) -> str:
        """
        Health check method.

        Returns:
            "pong"
        """
        return "pong"

    def _require_simulator(self):
        """Check that the simulator is available."""
        if self.simulator is None:
            raise ValueError(
                f"MuJoCo not installed. Run: pip install mujoco pillow\n"
                f"Import error: {_mujoco_import_error}"
            )

    def handle_sim_load(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Load a URDF file into the simulator.

        Params:
            path (str): Path to the URDF file.

        Returns:
            Model info dict.
        """
        self._require_simulator()
        if "path" not in params:
            raise ValueError("Missing required parameter: path")

        path = params["path"]
        if not isinstance(path, str):
            raise ValueError("Parameter 'path' must be a string")

        free_base = bool(params.get("free_base", False))
        terrain_config = params.get("terrain_config", params.get("terrainConfig", None))
        if terrain_config is not None and not isinstance(terrain_config, dict):
            terrain_config = None
        seed = params.get("seed", None)
        # Clear any active script when loading a new model
        self.sim_script_fn = None
        self.sim_script_error = None
        self._script_globals = _fresh_script_globals()
        try:
            if seed is not None:
                try:
                    import numpy as np
                    np.random.seed(int(seed))
                except Exception:
                    pass
                try:
                    import mujoco as _mj
                    _mj.mj_setSeed(int(seed))
                except Exception:
                    pass
            return self.simulator.load_urdf(
                path,
                free_base=free_base,
                terrain_config=terrain_config,
            )
        except FileNotFoundError as e:
            raise ValueError(f"File not found: {e}")
        except Exception as e:
            raise ValueError(f"Failed to load URDF: {e}")

    def handle_sim_step(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Step the simulation forward.

        Params:
            n_steps (int, optional): Number of steps to advance. Default: 1.

        Returns:
            Current simulation state.
        """
        self._require_simulator()
        n_steps = params.get("n_steps", 1)
        if not isinstance(n_steps, int) or n_steps < 1:
            n_steps = 1

        try:
            # Script runner: observe state → compute controls → apply before advancing
            script_error: Optional[str] = None
            if self.sim_script_fn is not None:
                try:
                    current_state = self.simulator.get_state()
                    controls = self.sim_script_fn(current_state["time"], current_state)
                    if isinstance(controls, dict):
                        self.simulator.set_control(controls)
                except Exception as se:
                    script_error = str(se)
                    self.sim_script_error = script_error
                    # Fail-safe: clear controls immediately so stale commands do not
                    # keep driving the robot after a script runtime error.
                    try:
                        self.simulator.set_control({})
                    except Exception:
                        pass
                    # Disable script until explicitly re-applied/reset by user.
                    self.sim_script_fn = None

            self.simulator.step(n_steps)
            state = self.simulator.get_state()
            if script_error:
                state["script_error"] = script_error
            return state
        except Exception as e:
            raise ValueError(f"Failed to step simulation: {e}")

    def handle_sim_reset(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Reset the simulation to initial state.

        Params:
            (none)

        Returns:
            Current simulation state after reset.
        """
        self._require_simulator()
        try:
            self.simulator.reset()
            return self.simulator.get_state()
        except Exception as e:
            raise ValueError(f"Failed to reset simulation: {e}")

    def handle_sim_set_control(self, params: Dict[str, Any]) -> None:
        """
        Set actuator control values.

        Params:
            controls (dict): Mapping of joint name to control value.

        Returns:
            null
        """
        self._require_simulator()
        if "controls" not in params:
            raise ValueError("Missing required parameter: controls")

        controls = params["controls"]
        if not isinstance(controls, dict):
            raise ValueError("Parameter 'controls' must be a dict")

        try:
            self.simulator.set_control(controls)
        except Exception as e:
            raise ValueError(f"Failed to set controls: {e}")

    def handle_sim_set_gravity(self, params: Dict[str, Any]) -> None:
        """
        Set gravity vector.

        Params:
            gravity (list[float]): [gx, gy, gz] in m/s². Pass [0,0,0] for zero-G.
        """
        self._require_simulator()
        gravity = params.get("gravity", [0.0, 0.0, -9.81])
        if not isinstance(gravity, list) or len(gravity) != 3:
            raise ValueError("Parameter 'gravity' must be a list of 3 floats")
        try:
            self.simulator.set_gravity([float(g) for g in gravity])
        except Exception as e:
            raise ValueError(f"Failed to set gravity: {e}")

    def handle_sim_get_state(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Get current simulation state.

        Params:
            (none)

        Returns:
            Simulation state dict.
        """
        self._require_simulator()
        try:
            return self.simulator.get_state()
        except Exception as e:
            raise ValueError(f"Failed to get state: {e}")

    def handle_sim_set_script(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Compile and install a Python step-callback script.

        Params:
            code (str): Python source defining ``def step(t, state) -> dict``.
                        Pass empty string to clear.

        Returns:
            {"status": "ok" | "cleared" | "error", "message": str (on error)}
        """
        code = params.get("code", "").strip()
        if not code:
            self.sim_script_fn = None
            self.sim_script_error = None
            self._script_globals = _fresh_script_globals()
            return {"status": "cleared"}
        try:
            self._reject_unsafe_script(code)
            script_globals = _fresh_script_globals()
            exec(compile(code, "<sim_script>", "exec"), script_globals)
            fn = script_globals.pop("step", None)
            if fn is None or not callable(fn):
                raise ValueError("Script must define a callable 'step(t, state)' function")
            self.sim_script_fn = fn
            self.sim_script_error = None
            self._script_globals = script_globals
            return {"status": "ok"}
        except Exception as e:
            self.sim_script_fn = None
            self.sim_script_error = str(e)
            self._script_globals = _fresh_script_globals()
            return {"status": "error", "message": str(e)}

    @staticmethod
    def _reject_unsafe_script(code: str) -> None:
        """Static AST scan: reject imports, attribute access into dunders, exec/eval."""
        import ast
        tree = ast.parse(code, mode="exec")
        for node in ast.walk(tree):
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                raise ValueError("Imports are not allowed in sim scripts")
            if isinstance(node, ast.Attribute) and node.attr.startswith("__"):
                raise ValueError(f"Dunder attribute access not allowed: {node.attr}")
            if isinstance(node, ast.Name) and node.id in _SCRIPT_NAME_DENY:
                raise ValueError(f"Use of '{node.id}' is not allowed in sim scripts")

    def handle_sim_render(self, params: Dict[str, Any]) -> str:
        """
        Render a frame and return as base64-encoded PNG.

        Params:
            width (int, optional): Image width. Default: 640.
            height (int, optional): Image height. Default: 480.

        Returns:
            Base64-encoded PNG string.
        """
        self._require_simulator()
        width = params.get("width", 640)
        height = params.get("height", 480)

        if not isinstance(width, int) or width < 1:
            width = 640
        if not isinstance(height, int) or height < 1:
            height = 480

        try:
            return self.simulator.render_frame(width, height)
        except Exception as e:
            raise ValueError(f"Failed to render frame: {e}")

    def _emit_progress(self, stage: str, text: str) -> None:
        """Emit a JSON-RPC notification for AI progress (no id = notification)."""
        notification = {
            "jsonrpc": "2.0",
            "method": "ai_progress",
            "params": {"stage": stage, "text": text}
        }
        sys.stdout.write(json.dumps(notification, ensure_ascii=False) + "\n")
        sys.stdout.flush()

    def handle_ai_edit(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Use Claude AI to generate a robot model edit from natural language.
        Uses streaming API with progress notifications when available.
        """
        if _generate_edit is None:
            raise ValueError(
                f"Claude AI not installed. Run: pip install anthropic\n"
                f"Error: {_ai_import_error}"
            )

        if "prompt" not in params or "urdf_content" not in params:
            raise ValueError("Missing required parameters: prompt, urdf_content")

        prompt = params["prompt"]
        urdf_content = params["urdf_content"]
        kinematic_context = params.get("kinematic_context", None)
        session_id = params.get("session_id", "default")
        # Workstream #1: canonical AssemblyGraph from the frontend. When provided,
        # claude_client prefers it over URDF as the edit-retry source of truth.
        # Kept as a dict (not stringified) so generate_edit can json.dumps with its
        # own formatting and the roundtrip shape stays inspectable in logs.
        assembly_graph = params.get("assembly_graph", None)
        if assembly_graph is not None and not isinstance(assembly_graph, dict):
            print(f"[ai_edit] Ignoring non-dict assembly_graph: {type(assembly_graph).__name__}", file=sys.stderr)
            assembly_graph = None

        if not isinstance(prompt, str):
            raise ValueError("Parameter 'prompt' must be a string")
        if not isinstance(urdf_content, str):
            raise ValueError("Parameter 'urdf_content' must be a string")

        # Model whitelist — reject unknown IDs by falling back to Sonnet
        _ALLOWED_MODELS = {"claude-sonnet-4-6", "claude-opus-4-7"}
        model = params.get("model") or "claude-sonnet-4-6"
        if model not in _ALLOWED_MODELS:
            print(f"[ai_edit] Unknown model '{model}', falling back to claude-sonnet-4-6", file=sys.stderr)
            model = "claude-sonnet-4-6"

        # Normalize images: strip any data:image/...;base64, prefix the frontend
        # may have included (Anthropic SDK rejects it).
        raw_images = params.get("images") or []
        images: list = []
        if isinstance(raw_images, list):
            for img in raw_images:
                if not isinstance(img, dict):
                    continue
                media_type = img.get("media_type")
                data = img.get("data", "")
                if not isinstance(data, str) or not isinstance(media_type, str):
                    continue
                if data.startswith("data:"):
                    comma = data.find(",")
                    if comma != -1:
                        data = data[comma + 1:]
                images.append({"media_type": media_type, "data": data})

        try:
            # Tool-use assembly agent (disabled by default — too many API calls / expensive)
            # To enable: pass "use_tools": true in params
            if params.get("use_tools") and _generate_assembly_with_tools is not None:
                print(f"[ai_edit] Using tool-use assembly agent for: {prompt[:80]}", file=sys.stderr)
                result = _generate_assembly_with_tools(
                    prompt, session_id,
                    on_progress=self._emit_progress,
                    model=model,
                    images=images,
                )
                self._emit_progress("done", "Complete")
                response = {
                    "explanation": result.get("explanation", "Assembly complete"),
                    "new_urdf": result.get("new_urdf") or urdf_content,
                    "stats": result.get("stats", "Assembly complete"),
                }
                if "assembly_graph" in result:
                    response["assembly_graph"] = result["assembly_graph"]
                if "topology_ops" in result:
                    response["topology_ops"] = result["topology_ops"]
                return response

            # Standard edit path
            kg_json = {}
            try:
                if _parse_urdf_string is not None:
                    kg = _parse_urdf_string(urdf_content)
                    kg_json = kg.to_json()
            except Exception as parse_err:
                print(f"[ai_edit] URDF pre-parse skipped: {parse_err}", file=sys.stderr)

            if _generate_edit_streaming is not None:
                result = _generate_edit_streaming(
                    prompt, urdf_content, kg_json, kinematic_context, session_id,
                    on_progress=self._emit_progress,
                    model=model,
                    images=images,
                    assembly_graph=assembly_graph,
                )
            else:
                self._emit_progress("thinking", "Processing request...")
                result = _generate_edit(
                    prompt, urdf_content, kg_json, kinematic_context, session_id,
                    model=model,
                    images=images,
                    assembly_graph=assembly_graph,
                )

            self._emit_progress("done", "Complete")

            response = {
                "explanation": result.get("explanation", "Edit applied"),
                "new_urdf": result.get("new_urdf", urdf_content),
                "stats": result.get("stats", "Edit complete"),
            }
            if "assembly_graph" in result:
                response["assembly_graph"] = result["assembly_graph"]
            if "topology_ops" in result:
                response["topology_ops"] = result["topology_ops"]
            # Phase 3: surface owner-routed diagnostics to the frontend so the
            # chat panel can display AI-fixable issues (and developer logs can
            # show compiler/exporter ones) instead of swallowing them silently.
            if result.get("diagnostics"):
                response["diagnostics"] = result["diagnostics"]
            return response
        except Exception as e:
            raise ValueError(f"AI edit failed: {e}")

    def handle_ai_edit_turn(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Workstream #2 tool-call edit surface: one turn of the multi-round
        tool-use loop. The frontend drives the loop — it calls this once with
        `prompt` (first turn), then again for each round with `tool_results`
        from local dispatch until `done` comes back true.

        Params:
            session_id (str)
            prompt (str, optional)          — first-turn user message
            assembly_graph (dict, optional) — first-turn graph snapshot
            kinematic_context (str, optional)
            tool_results (list, optional)   — subsequent turns, one per tool_use_id
            model (str, optional)           — 'claude-sonnet-4-6' | 'claude-opus-4-7'
            images (list, optional)

        Returns: dict — see generate_edit_turn docstring for the shape.
        """
        if _generate_edit_turn is None:
            raise ValueError(
                f"Claude AI not installed. Run: pip install anthropic\n"
                f"Error: {_ai_import_error}"
            )

        session_id = params.get("session_id", "default")
        prompt = params.get("prompt")
        assembly_graph = params.get("assembly_graph")
        if assembly_graph is not None and not isinstance(assembly_graph, dict):
            assembly_graph = None
        kinematic_context = params.get("kinematic_context")
        tool_results = params.get("tool_results")

        _ALLOWED_MODELS = {"claude-sonnet-4-6", "claude-opus-4-7"}
        model = params.get("model") or "claude-sonnet-4-6"
        if model not in _ALLOWED_MODELS:
            model = "claude-sonnet-4-6"

        # Normalize images (same treatment as ai_edit — strip data: URL prefix).
        raw_images = params.get("images") or []
        images: list = []
        if isinstance(raw_images, list):
            for img in raw_images:
                if not isinstance(img, dict):
                    continue
                media_type = img.get("media_type")
                data = img.get("data", "")
                if not isinstance(data, str) or not isinstance(media_type, str):
                    continue
                if data.startswith("data:"):
                    comma = data.find(",")
                    if comma != -1:
                        data = data[comma + 1:]
                images.append({"media_type": media_type, "data": data})

        try:
            return _generate_edit_turn(
                session_id=session_id,
                prompt=prompt,
                assembly_graph=assembly_graph,
                kinematic_context=kinematic_context,
                tool_results=tool_results,
                model=model,
                images=images,
            )
        except Exception as e:
            raise ValueError(f"AI edit turn failed: {e}")

    def handle_ai_complete(self, params: Dict[str, Any]) -> str:
        """
        Generate inline completions for URDF/XML editing.
        Uses Claude API if available, falls back to local pattern-based completions.

        Params:
            urdf_content (str): Current URDF XML as string.
            cursor_line (int): Current cursor line (1-indexed).
            cursor_column (int): Current cursor column (1-indexed).
            prefix (str): Optional prefix context (e.g., recent characters typed).

        Returns:
            Completion text string (the text to insert at cursor).
        """
        if "urdf_content" not in params:
            raise ValueError("Missing required parameter: urdf_content")

        urdf_content = params["urdf_content"]
        cursor_line = params.get("cursor_line", 1)
        cursor_column = params.get("cursor_column", 1)
        prefix = params.get("prefix", "")
        kinematic_context = params.get("kinematic_context", "")

        if not isinstance(urdf_content, str):
            raise ValueError("Parameter 'urdf_content' must be a string")
        if not isinstance(cursor_line, int) or not isinstance(cursor_column, int):
            raise ValueError("Parameters 'cursor_line' and 'cursor_column' must be integers")

        print(f"[server] ai_complete: line={cursor_line}, col={cursor_column}", file=sys.stderr)

        # Try Claude API first, fall back to local completions
        if _generate_completion is not None:
            try:
                completion = _generate_completion(
                    urdf_content,
                    cursor_line,
                    cursor_column,
                    prefix,
                    kinematic_context
                )
                print(f"[server] ai_complete (claude): got {len(completion)} chars", file=sys.stderr)
                return completion
            except Exception as e:
                print(f"[server] Claude completion failed: {e}", file=sys.stderr)
                return ""

        # No Claude API available — return empty rather than low-quality local suggestions
        print(f"[server] ai_complete: Claude not available, returning empty", file=sys.stderr)
        return ""

    def handle_ai_validate_assembly(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Second-pass AI validation of an assembled URDF.
        Sends the URDF back to Claude (Haiku) for spatial correctness checks.
        Returns corrections if needed (~$0.01-0.02 per call).

        Params:
            urdf_content (str): The assembled URDF XML.
            original_prompt (str): The user's original build request.
            session_id (str, optional): Session identifier.

        Returns:
            Dict with 'ok' bool, 'notes' str, and optional 'corrected_urdf' str.
        """
        if _validate_assembly is None:
            raise ValueError(
                f"Claude AI not installed. Run: pip install anthropic\n"
                f"Error: {_ai_import_error}"
            )

        if "urdf_content" not in params or "original_prompt" not in params:
            raise ValueError("Missing required parameters: urdf_content, original_prompt")

        urdf_content = params["urdf_content"]
        original_prompt = params["original_prompt"]
        session_id = params.get("session_id", "default")
        screenshot_base64 = params.get("screenshot_base64")
        screenshots = params.get("screenshots")  # array of 3 base64 PNGs
        # User-uploaded reference images — [{media_type, data}, ...]. Threaded
        # through so Gemini compares the rendered output against the reference
        # the user originally gave Claude (G3 fix).
        reference_images = params.get("reference_images") or []
        # Engine-computed ground truth (
        # Layer 1). Shape: {placements: [...], icpGaps: [...]}. Forwarded to
        # the Gemini prompt so screenshot misreads can be refuted with the
        # actual xyz/rpy the placement engine wrote and the ICP gap it measured.
        engine_summary = params.get("engine_summary")

        try:
            self._emit_progress("validating", "Checking assembly with visual feedback...")
            result = _validate_assembly(urdf_content, original_prompt, session_id, screenshot_base64, screenshots, reference_images, engine_summary)
            self._emit_progress("done", "Validation complete")
            return result
        except Exception as e:
            raise ValueError(f"Assembly validation failed: {e}")

    def handle_ai_set_history(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Restore conversation history for a session from frontend localStorage.
        Called on reconnect to maintain context across backend restarts.

        Params:
            session_id (str): Session identifier.
            history (list): List of {role, content} message dicts.

        Returns:
            Dict with 'status' and 'count'.
        """
        if _set_conversation_history is None:
            raise ValueError(
                f"Claude AI not installed. Run: pip install anthropic\n"
                f"Error: {_ai_import_error}"
            )

        if "session_id" not in params or "history" not in params:
            raise ValueError("Missing required parameters: session_id, history")

        session_id = params["session_id"]
        history = params["history"]

        if not isinstance(session_id, str):
            raise ValueError("Parameter 'session_id' must be a string")
        if not isinstance(history, list):
            raise ValueError("Parameter 'history' must be a list")

        try:
            return _set_conversation_history(session_id, history)
        except Exception as e:
            raise ValueError(f"Failed to set history: {e}")

    def handle_ai_gen_sim_script(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Generate a sandbox-compliant sim control script via Claude.

        Params:
            prompt (str): Natural-language request.
            urdf_content (str): Current URDF; used to extract joint names/limits.
            current_script (str, optional): If provided, treat prompt as a
                modification of this script instead of generating fresh.

        Returns:
            {"status": "ok", "code": str, "joint_names": [...]} on success,
            {"status": "error", "message": str} on rejection or API failure.
        """
        if _generate_sim_script is None:
            raise ValueError(
                f"Claude AI not installed. Run: pip install anthropic\n"
                f"Error: {_ai_import_error}"
            )

        prompt = params.get("prompt", "") or ""
        urdf_content = params.get("urdf_content", "")
        current_script = params.get("current_script", "") or ""
        terrain_config = params.get("terrain_config", params.get("terrainConfig", None))
        if terrain_config is not None and not isinstance(terrain_config, dict):
            terrain_config = None
        if not isinstance(prompt, str):
            raise ValueError("Parameter 'prompt' must be a string")
        if not isinstance(urdf_content, str) or not urdf_content.strip():
            raise ValueError("Parameter 'urdf_content' must be a non-empty string")

        # Extract joint names, limits, and wheel direction metadata from the URDF.
        try:
            joint_names, joint_limits, joint_metadata = _extract_sim_joint_context(urdf_content)
        except Exception as e:
            return {"status": "error",
                    "message": f"Could not parse URDF joints: {e}"}

        if not joint_names:
            return {"status": "error",
                    "message": "No controllable joints found in URDF"}

        try:
            code = _generate_sim_script(
                prompt, joint_names, current_script, joint_limits, joint_metadata, terrain_config
            )
        except Exception as e:
            return {"status": "error", "message": f"AI call failed: {e}"}

        if not code.strip():
            return {"status": "error", "message": "AI returned empty code"}

        # Validate: must compile and pass sandbox scan.
        try:
            self._reject_unsafe_script(code)
            compiled = compile(code, "<ai_sim_script>", "exec")
            validation_globals = _fresh_script_globals()
            exec(compiled, validation_globals)
            fn = validation_globals.get("step")
            if fn is None or not callable(fn):
                raise ValueError("Generated script must define a callable 'step(t, state)' function")
        except SyntaxError as e:
            return {"status": "error",
                    "message": f"Generated script has syntax error: {e}",
                    "code": code}
        except ValueError as e:
            return {"status": "error",
                    "message": f"Generated script rejected by sandbox: {e}",
                    "code": code}

        return {"status": "ok", "code": code, "joint_names": joint_names}

    def process_request(self, request: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Process a single JSON-RPC 2.0 request.

        Args:
            request: JSON-RPC request dict.

        Returns:
            JSON-RPC response dict, or None if notification.
        """
        # Extract fields
        jsonrpc = request.get("jsonrpc")
        method = request.get("method")
        params = request.get("params", {})
        request_id = request.get("id")

        # Validate JSON-RPC version
        if jsonrpc != "2.0":
            return {
                "jsonrpc": "2.0",
                "error": {
                    "code": -32600,
                    "message": "Invalid Request",
                    "data": "jsonrpc must be 2.0"
                },
                "id": request_id,
            }

        # Check if method exists
        if method not in self.methods:
            return {
                "jsonrpc": "2.0",
                "error": {
                    "code": -32601,
                    "message": "Method not found",
                    "data": f"Unknown method: {method}"
                },
                "id": request_id,
            }

        # Call method
        try:
            handler = self.methods[method]
            result = handler(params)

            # If this is a notification (no id), return None
            if "id" not in request:
                return None

            return {
                "jsonrpc": "2.0",
                "result": result,
                "id": request_id,
            }
        except Exception as e:
            # If this is a notification, still log but don't return error
            if "id" not in request:
                print(f"Error in notification: {e}", file=sys.stderr)
                traceback.print_exc(file=sys.stderr)
                return None

            return {
                "jsonrpc": "2.0",
                "error": {
                    "code": -32603,
                    "message": "Internal error",
                    "data": str(e)
                },
                "id": request_id,
            }

    def run(self) -> None:
        """
        Main server loop.
        Reads line-delimited JSON from stdin, writes responses to stdout.
        """
        # Force UTF-8 on Windows (default is often cp1252)
        if sys.platform == 'win32':
            import io
            sys.stdin = io.TextIOWrapper(sys.stdin.buffer, encoding='utf-8')
            sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
            sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')

        while True:
            try:
                line = sys.stdin.readline()
                if not line:
                    # EOF
                    break

                line = line.strip()
                if not line:
                    continue

                try:
                    request = json.loads(line)
                except json.JSONDecodeError as e:
                    response = {
                        "jsonrpc": "2.0",
                        "error": {
                            "code": -32700,
                            "message": "Parse error",
                            "data": str(e)
                        },
                        "id": None,
                    }
                    sys.stdout.write(json.dumps(response, ensure_ascii=False) + "\n")
                    sys.stdout.flush()
                    continue

                # Process the request
                response = self.process_request(request)

                # Write response (skip for notifications)
                if response is not None:
                    sys.stdout.write(json.dumps(response, ensure_ascii=False) + "\n")
                    sys.stdout.flush()

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Server error: {e}", file=sys.stderr)
                traceback.print_exc(file=sys.stderr)
                break


if __name__ == "__main__":
    server = JSONRPCServer()
    server.run()

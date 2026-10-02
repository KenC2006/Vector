"""
Explicit-pose robot compiler.

A design is a list of parts. Every part states WHERE it is (a world point, or
a reference to another part's anchor plus an offset) and HOW it is oriented
(which world directions its local axes point, or "mate flush" against the
anchor it is placed on). Joints state their pivot and axis explicitly or
inherit them from the anchor the part hangs on (e.g. a servo's shaft_out).
Nothing is inferred beyond that, so the rendered robot is exactly what the
design says.

Units in the design: millimetres and degrees, world frame X forward, Y left,
Z up, authored in the robot's rest pose. Output: URDF (metres/radians).
"""
from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from core.presets import actuator_rating, get_component, resolve_component_bounds_mm, is_parametric_spec
from core.designer.geometry import (
    S, frame_from_axes, matrix_to_rpy, mirror_name, rot_axis_angle,
    rotation_between, rpy_to_matrix, unit, vec,
)

FACE_NORMALS = {
    "+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0),
    "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1),
}
FACE_ALIASES = {
    "front": "+x", "back": "-x", "left": "+y", "right": "-y", "top": "+z", "bottom": "-z",
}
SHELL_AREAL_DENSITY = 3.0  # kg/m^2: a hollow body skin, ~2.5mm printed plastic / thin sheet


class DesignError(ValueError):
    """A design the compiler cannot build. Message is shown to the model."""


@dataclass
class Box:
    """Oriented box in world mm: center, rotation (columns = local axes), half extents."""
    c: np.ndarray
    R: np.ndarray
    h: np.ndarray


@dataclass
class Part:
    name: str
    spec: Dict[str, Any]
    component: Optional[Dict[str, Any]]      # catalog preset, None for shells
    size: np.ndarray                          # local AABB full extents (mm)
    center_local: np.ndarray                  # local AABB center (shells can be offset)
    R: np.ndarray = field(default_factory=lambda: np.eye(3))
    p: np.ndarray = field(default_factory=lambda: np.zeros(3))
    parent: Optional[str] = None
    joint: Optional[Dict[str, Any]] = None    # resolved: type, pivot, axis, limits
    mirror_of: Optional[str] = None
    link: str = ""

    @property
    def is_shell(self) -> bool:
        return self.component is None

    @property
    def frame_p(self) -> np.ndarray:
        """URDF link frame origin: the joint pivot for moving parts (URDF puts
        a child link's frame on its joint), else the part origin. The frame's
        rotation is always the part's own."""
        if self.joint and "pivot" in self.joint:
            return self.joint["pivot"]
        return self.p

    @property
    def geom_offset(self) -> np.ndarray:
        """Part origin expressed in its link frame (mm)."""
        return self.R.T @ (self.p - self.frame_p)

    def world(self, local_pt: np.ndarray) -> np.ndarray:
        return self.p + self.R @ local_pt

    # ── anchors ──────────────────────────────────────────────────────────
    def anchor(self, key: str) -> tuple[np.ndarray, Optional[np.ndarray]]:
        """Local point + local outward normal (None for 'center')."""
        k = key.strip()
        k = FACE_ALIASES.get(k, k)
        if k in ("center", "origin", ""):
            return np.zeros(3), None
        if k in FACE_NORMALS:
            n = np.array(FACE_NORMALS[k], dtype=float)
            return self.center_local + n * self.size / 2.0, n
        if self.is_shell and "." in k:
            prim_name, face = k.split(".", 1)
            prim = next((q for q in self.spec.get("shape", []) if q.get("name") == prim_name), None)
            if prim is None:
                raise DesignError(f"{self.name}: no primitive named '{prim_name}' (have: {self._prim_names()})")
            return _primitive_anchor(prim, FACE_ALIASES.get(face, face), self.name)
        if self.is_shell:
            prim = next((q for q in self.spec.get("shape", []) if q.get("name") == k), None)
            if prim is not None:
                return _prim_center(prim), None
        if self.component is not None:
            for conn in self.component.get("connectors", []) or []:
                if conn.get("id") == k:
                    return np.asarray(conn["origin_xyz_mm"], float), unit(np.asarray(conn["axis_xyz"], float))
        opts = ["center", "+x", "-x", "+y", "-y", "+z", "-z"]
        if self.component is not None:
            opts += [c["id"] for c in self.component.get("connectors", []) or []]
        else:
            opts += [f"{n}.+z" for n in self._prim_names()[:3]] + ["<primitive>.<face>"]
        raise DesignError(f"{self.name}: unknown anchor '{key}'. Valid: {', '.join(opts)}")

    def _prim_names(self) -> List[str]:
        return [q.get("name") for q in self.spec.get("shape", []) if q.get("name")]

    def boxes(self) -> List[Box]:
        """World-space oriented boxes approximating the part's volume."""
        if not self.is_shell:
            return [Box(self.world(self.center_local), self.R, self.size / 2.0)]
        out = []
        for prim in self.spec.get("shape", []):
            if prim.get("shape") == "mesh" and "size_mm" not in prim:
                continue
            Rl = _prim_rot(prim)
            out.append(Box(self.world(_prim_center(prim)), self.R @ Rl, _prim_half(prim)))
        return out


# ── primitives (shell geometry) ──────────────────────────────────────────────

def _prim_center(prim) -> np.ndarray:
    return vec(prim.get("xyz_mm", [0, 0, 0]), "xyz_mm")


def _prim_rot(prim) -> np.ndarray:
    rpy = prim.get("rpy_deg", [0, 0, 0])
    return rpy_to_matrix([math.radians(float(a)) for a in vec(rpy, "rpy_deg")])


def _prim_half(prim) -> np.ndarray:
    shape = prim.get("shape")
    if shape == "mesh":
        # A mesh file carried through from an imported URDF. Its extent is not
        # known here; `size_mm` (when given) is the envelope used for geometry
        # checks, otherwise it counts as a point.
        return vec(prim.get("size_mm", [1, 1, 1]), "size_mm") / 2.0
    if shape == "box":
        return vec(prim.get("size_mm"), "size_mm") / 2.0
    if shape == "cylinder":
        r, L = float(prim["radius_mm"]), float(prim["length_mm"])
        return np.array([r, r, L / 2.0])
    if shape == "sphere":
        r = float(prim["radius_mm"])
        return np.array([r, r, r])
    raise DesignError(f"primitive shape must be box/cylinder/sphere/mesh, got {shape!r}")


def _prim_area_m2(prim) -> float:
    shape = prim.get("shape")
    if shape == "mesh" and "size_mm" not in prim:
        return 0.0
    if shape in ("box", "mesh"):
        x, y, z = vec(prim["size_mm"]) * 1e-3
        return 2 * (x * y + y * z + x * z)
    if shape == "cylinder":
        r, L = float(prim["radius_mm"]) * 1e-3, float(prim["length_mm"]) * 1e-3
        return 2 * math.pi * r * (r + L)
    return 4 * math.pi * (float(prim["radius_mm"]) * 1e-3) ** 2


def _primitive_anchor(prim, face: str, owner: str):
    if face in ("center", ""):
        return _prim_center(prim), None
    if face not in FACE_NORMALS:
        raise DesignError(f"{owner}: primitive face must be +x/-x/+y/-y/+z/-z/center, got '{face}'")
    n_local = np.array(FACE_NORMALS[face], dtype=float)
    Rl = _prim_rot(prim)
    return _prim_center(prim) + Rl @ (n_local * _prim_half(prim)), Rl @ n_local


def _shell_bounds(shape: List[Dict]) -> tuple[np.ndarray, np.ndarray]:
    if not shape:
        return np.zeros(3), np.zeros(3)   # frame-only part (an empty URDF link)
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for prim in shape:
        c, Rl, h = _prim_center(prim), _prim_rot(prim), _prim_half(prim)
        ext = np.abs(Rl) @ h
        lo, hi = np.minimum(lo, c - ext), np.maximum(hi, c + ext)
    return hi - lo, (hi + lo) / 2.0


# ── compile ─────────────────────────────────────────────────────────────────

_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def compile_design(design: Dict[str, Any]) -> "Assembly":
    parts_in = design.get("parts")
    if not isinstance(parts_in, list) or not parts_in:
        raise DesignError("design.parts must be a non-empty list")

    parts: Dict[str, Part] = {}
    order: List[str] = []
    errors: List[str] = []

    for i, spec in enumerate(parts_in):
        try:
            part = _resolve_part(spec, parts, i)
        except DesignError as e:
            errors.append(str(e))
            continue
        except (ValueError, KeyError, TypeError) as e:
            errors.append(f"{spec.get('name', f'parts[{i}]')}: {e}")
            continue
        parts[part.name] = part
        order.append(part.name)
    if errors:
        raise DesignError("\n".join(errors))

    roots = [n for n in order if parts[n].parent is None]
    if len(roots) != 1:
        raise DesignError(f"exactly one part must have no parent (the root body); found {roots}")

    _apply_rest_angles(parts, order)
    _apply_mirrors(parts, order)
    asm = Assembly(parts=parts, order=order, root=roots[0], design=design)
    asm.ground()
    asm.assign_link_names()
    return asm


def _resolve_part(spec: Dict[str, Any], parts: Dict[str, Part], idx: int) -> Part:
    name = spec.get("name")
    if not isinstance(name, str) or not _NAME_RE.match(name):
        raise DesignError(f"parts[{idx}]: name must be snake_case letters/digits/underscores, got {name!r}")
    if name in parts:
        raise DesignError(f"{name}: duplicate part name")

    comp = None
    if "shape" in spec:
        shape = spec["shape"]
        if not isinstance(shape, list):
            raise DesignError(f"{name}: shape must be a list of primitives")
        for q in shape:
            _prim_half(q)
        size, center_local = _shell_bounds(shape)
    else:
        cid = spec.get("component")
        comp = get_component(cid) if isinstance(cid, str) else None
        if comp is None:
            raise DesignError(f"{name}: unknown component '{cid}' (use an exact catalog id, or give a `shape` for a custom body)")
        inst = {"length_mm": spec["length_mm"]} if spec.get("length_mm") else None
        if is_parametric_spec(comp) and not inst:
            raise DesignError(f"{name}: {cid} is a cut-to-length part — set length_mm")
        size = np.asarray(resolve_component_bounds_mm(comp, inst), float)
        center_local = np.zeros(3)

    part = Part(name=name, spec=spec, component=comp, size=size, center_local=center_local)

    parent = spec.get("parent")
    if parent is not None:
        if parent not in parts:
            raise DesignError(f"{name}: parent '{parent}' must be an earlier part")
        part.parent = parent

    # ── where does `align` go, and what is it mated to? ─────────────────
    target_pt, target_n, ref_part, ref_anchor = _resolve_point(spec.get("at", [0, 0, 0]), parts, name)
    align_key = spec.get("align", "center")
    align_local, align_n = part.anchor(align_key)

    # ── orientation ──────────────────────────────────────────────────────
    explicit = spec.get("z_axis") is not None or (spec.get("x_axis") is not None and target_n is None)
    if explicit or align_n is None or target_n is None:
        R = frame_from_axes(spec.get("z_axis"), spec.get("x_axis"))
    else:
        # Flush mate: the aligned face/connector points back INTO the anchor.
        R = rotation_between(align_n, -target_n)
        if spec.get("x_axis") is not None:
            R = _spin_to_match_x(R, -target_n, unit(vec(spec["x_axis"], "x_axis")))
    spin = float(spec.get("spin_deg", 0) or 0)
    if spin:
        about = -target_n if (target_n is not None and not explicit) else R[:, 2]
        R = rot_axis_angle(about, math.radians(spin)) @ R
    part.R = R
    part.p = target_pt - R @ align_local

    # ── joint ────────────────────────────────────────────────────────────
    j = spec.get("joint")
    if parent is not None:
        part.joint = _resolve_joint(j, part, parts, ref_part, ref_anchor)
    elif j:
        raise DesignError(f"{name}: the root part cannot have a joint")

    if spec.get("mirror"):
        if parent is None:
            raise DesignError(f"{name}: the root cannot be mirrored")
        part.mirror_of = "__self__"
    return part


def _spin_to_match_x(R: np.ndarray, axis: np.ndarray, want_x: np.ndarray) -> np.ndarray:
    x = R[:, 0]
    x_p = x - axis * (x @ axis)
    w_p = want_x - axis * (want_x @ axis)
    if np.linalg.norm(x_p) < 1e-6 or np.linalg.norm(w_p) < 1e-6:
        return R
    x_p, w_p = unit(x_p), unit(w_p)
    ang = math.atan2(float(axis @ np.cross(x_p, w_p)), float(x_p @ w_p))
    return rot_axis_angle(axis, ang) @ R


def _resolve_point(at, parts: Dict[str, Part], owner: str):
    """-> (world point, world normal|None, ref part name|None, anchor|None)."""
    if isinstance(at, str):
        at = {"ref": at}
    if isinstance(at, dict):
        ref = at.get("ref")
        if not isinstance(ref, str):
            raise DesignError(f"{owner}: `at.ref` must be 'part' or 'part.anchor'")
        pname, _, anchor = ref.partition(".")
        if pname not in parts:
            raise DesignError(f"{owner}: at.ref part '{pname}' must be an earlier part")
        rp = parts[pname]
        loc, n = rp.anchor(anchor or "center")
        pt = rp.world(loc) + vec(at.get("offset", [0, 0, 0]), "offset")
        return pt, (rp.R @ n if n is not None else None), pname, (anchor or "center")
    return vec(at, "at"), None, None, None


def _resolve_joint(j, part: Part, parts: Dict[str, Part], ref_part, ref_anchor) -> Dict[str, Any]:
    if not j:
        return {"type": "fixed"}
    if isinstance(j, str):
        j = {"type": j}
    jtype = j.get("type", "fixed")
    if jtype not in ("fixed", "revolute", "continuous", "prismatic"):
        raise DesignError(f"{part.name}: joint.type must be fixed/revolute/continuous/prismatic")
    if jtype == "fixed":
        return {"type": "fixed", **({"name": str(j["name"])} if j.get("name") else {})}

    # Pivot + axis default to the anchor the part hangs on (a shaft connector).
    pivot = axis = None
    if ref_part is not None and ref_anchor not in (None, "center"):
        rp = parts[ref_part]
        loc, n = rp.anchor(ref_anchor)
        pivot = rp.world(loc)
        axis = rp.R @ n if n is not None else None
    if j.get("pivot") is not None:
        pivot, _, _, _ = _resolve_point(j["pivot"], parts, part.name)
    if j.get("axis") is not None:
        a = j["axis"]
        if isinstance(a, str) and "." in a:
            pname, _, anchor = a.partition(".")
            if pname not in parts:
                raise DesignError(f"{part.name}: joint.axis part '{pname}' unknown")
            _, n = parts[pname].anchor(anchor)
            if n is None:
                raise DesignError(f"{part.name}: joint.axis anchor '{a}' has no direction")
            axis = parts[pname].R @ n
        else:
            axis = vec(a, "joint.axis")
    if pivot is None:
        raise DesignError(f"{part.name}: {jtype} joint needs a pivot — mount the part `at` a connector (e.g. a servo's shaft_out) or set joint.pivot")
    if axis is None:
        raise DesignError(f"{part.name}: {jtype} joint needs an axis (e.g. \"+y\" or \"hip_servo.shaft_out\")")
    out = {"type": jtype, "pivot": np.asarray(pivot, float), "axis": unit(np.asarray(axis, float)),
           "rest": math.radians(float(j.get("rest_deg", 0) or 0))}
    if jtype == "revolute":
        # Limits are authored like rest_deg (0 = the straight, as-placed pose);
        # the URDF's zero is the rest pose, so shift them by the rest angle.
        lo, hi = float(j.get("lower_deg", -90)), float(j.get("upper_deg", 90))
        rest_deg = math.degrees(out["rest"])
        if "rest_deg" in j and not lo <= rest_deg <= hi:
            raise DesignError(f"{part.name}: rest_deg {rest_deg:g} lies outside its limits [{lo:g}, {hi:g}]")
        out["lower"] = math.radians(lo) - out["rest"]
        out["upper"] = math.radians(hi) - out["rest"]
    # A joint is driven by the actuator it hangs off, when that actuator moves
    # the way the joint does (a rotary motor turns, a linear actuator slides).
    # One whose parent is not such an actuator (a rocker on the hull, a free
    # hinge) is a passive pivot: no motor in the sim, only its limits.
    # `passive` overrides either way.
    rating = _drive_rating(parts.get(part.parent), jtype)
    if jtype == "prismatic":
        stroke = (rating or {}).get("stroke_mm")
        out["lower"] = float(j.get("lower_mm", 0)) / 1000.0
        out["upper"] = float(j.get("upper_mm", stroke if stroke else 50)) / 1000.0
        out["rest"] = 0.0
    out["effort"], out["velocity"] = (rating["effort"], rating["speed"]) if rating else (5.0, 2 * math.pi)
    if j.get("effort") is not None:
        out["effort"] = float(j["effort"])
    if j.get("velocity") is not None:
        out["velocity"] = float(j["velocity"])
    driven = rating is not None or jtype == "continuous"
    out["passive"] = bool(j.get("passive", not driven))
    if out["passive"]:
        out["effort"] = 0.0
    if j.get("name"):
        out["name"] = str(j["name"])
    return out


def _drive_rating(parent: Optional[Part], jtype: str) -> Optional[Dict[str, float]]:
    """Effort (N·m or N), speed (rad/s or m/s) and stroke of the actuator that
    drives a joint of this type, or None when the parent can't drive it."""
    if parent is None or parent.component is None:
        return None
    r = actuator_rating(parent.component)
    if r is None:
        return None
    kind, effort, speed = r
    if (kind == "linear") != (jtype == "prismatic"):
        return None
    act = parent.component.get("actuation") or {}
    return {"effort": effort, "speed": speed, "stroke_mm": act.get("stroke_mm")}


def _descendants(parts: Dict[str, Part], order: List[str], root: str) -> List[str]:
    out, frontier = [root], {root}
    for n in order:
        if parts[n].parent in frontier and n not in frontier:
            frontier.add(n)
            out.append(n)
    return out


def _apply_rest_angles(parts: Dict[str, Part], order: List[str]) -> None:
    """Bend joints to their rest angle: rotate the whole subtree about the pivot."""
    for n in order:
        j = parts[n].joint
        if not j or j["type"] not in ("revolute", "continuous") or not j.get("rest"):
            continue
        Rr = rot_axis_angle(j["axis"], j["rest"])
        pivot = j["pivot"].copy()
        for d in _descendants(parts, order, n):
            q = parts[d]
            q.p = pivot + Rr @ (q.p - pivot)
            q.R = Rr @ q.R
            if d != n and q.joint and "pivot" in q.joint:
                q.joint["pivot"] = pivot + Rr @ (q.joint["pivot"] - pivot)
                q.joint["axis"] = Rr @ q.joint["axis"]


def _apply_mirrors(parts: Dict[str, Part], order: List[str]) -> None:
    mirrored = [n for n in list(order) if parts[n].mirror_of == "__self__"]
    names = set(mirrored)
    for n in mirrored:
        src = parts[n]
        src.mirror_of = None
        m_name = mirror_name(n)
        if m_name in parts:
            raise DesignError(f"{n}: mirrored copy name '{m_name}' collides with an existing part — rename one of them")
        cp = Part(name=m_name, spec=src.spec, component=src.component, size=src.size.copy(),
                  center_local=src.center_local.copy())
        cp.p = S @ src.p
        cp.R = S @ src.R @ S
        cp.parent = mirror_name(src.parent) if src.parent in names else src.parent
        if src.joint:
            j = dict(src.joint)
            if j.get("name"):
                j["name"] = mirror_name(j["name"])
            if "pivot" in j:
                j["pivot"] = S @ src.joint["pivot"]
                # Axial vector: reflect then negate so +angle is the mirrored motion.
                j["axis"] = -(S @ src.joint["axis"])
            cp.joint = j
        cp.mirror_of = n
        parts[m_name] = cp
        order.insert(order.index(n) + 1, m_name)


# ── assembly / URDF ─────────────────────────────────────────────────────────

@dataclass
class Assembly:
    parts: Dict[str, Part]
    order: List[str]
    root: str
    design: Dict[str, Any]
    ground_shift: np.ndarray = field(default_factory=lambda: np.zeros(3))  # applied by ground()

    def all_boxes(self) -> Dict[str, List[Box]]:
        return {n: self.parts[n].boxes() for n in self.order}

    def ground(self) -> None:
        zs = [_box_zmin(b) for bs in self.all_boxes().values() for b in bs]
        if not zs:
            return
        zmin = min(zs)
        shift = np.array([0.0, 0.0, -zmin])
        self.ground_shift = shift
        for q in self.parts.values():
            q.p = q.p + shift
            if q.joint and "pivot" in q.joint:
                q.joint["pivot"] = q.joint["pivot"] + shift

    def assign_link_names(self) -> None:
        counters: Dict[str, int] = {}
        # Explicit link names are reserved first so generated ones step around them.
        def explicit_link(q: Part) -> Optional[str]:
            # A mirrored copy shares its source's spec: its name is `mirror_link`.
            return q.spec.get("mirror_link") if q.mirror_of else q.spec.get("link")
        reserved = {str(explicit_link(self.parts[n])) for n in self.order if explicit_link(self.parts[n])}
        used = set()
        for n in self.order:
            q = self.parts[n]
            explicit = explicit_link(q)
            if explicit:
                if not str(explicit).strip() or re.search(r'[\s"<>&]', str(explicit)):
                    raise DesignError(f"{n}: link name {explicit!r} must be non-empty with no spaces, quotes or <>&")
                q.link = str(explicit)
            elif q.is_shell:
                base = re.sub(r"_(\d+)$", r"_n\1", n)  # never look like '<preset>_<N>'
                q.link = base
            else:
                cid = q.component["id"]
                counters[cid] = counters.get(cid, 0) + 1
                while f"{cid}_{counters[cid]}" in reserved:
                    counters[cid] += 1
                q.link = f"{cid}_{counters[cid]}"
            if q.link in used:
                raise DesignError(f"link name collision on {q.link}")
            used.add(q.link)

    # ── mass ─────────────────────────────────────────────────────────────
    def mass_kg(self, q: Part) -> float:
        if q.spec.get("mass_kg") is not None:
            return max(float(q.spec["mass_kg"]), 1e-4)
        if q.is_shell and not q.spec.get("shape"):
            return 1e-3   # frame-only link
        if q.is_shell:
            return max(0.02, SHELL_AREAL_DENSITY * sum(_prim_area_m2(p) for p in q.spec["shape"]))
        phys = q.component.get("physical", {})
        if phys.get("mass_kg"):
            return float(phys["mass_kg"])
        per = phys.get("mass_kg_per_100mm")
        if per:
            return float(per) * float(q.size[2]) / 100.0
        return 0.05

    def com(self) -> np.ndarray:
        tot, acc = 0.0, np.zeros(3)
        for q in self.parts.values():
            m = self.mass_kg(q)
            tot += m
            acc += m * q.world(q.center_local)
        return acc / max(tot, 1e-9)

    def to_urdf(self, robot_name: str = "robot") -> str:
        L: List[str] = ['<?xml version="1.0"?>', f'<robot name="{_xml(robot_name)}">']
        payload = json.dumps(self.design, separators=(",", ":")).replace("--", "-\\u002d")
        L.append(f"  <!-- vector:design {payload} -->")
        L.append(f"  <!-- vector:parts {json.dumps(self.part_map(), separators=(',', ':'))} -->")
        for n in self.order:
            L.extend(self._link_xml(self.parts[n]))
        for n in self.order:
            q = self.parts[n]
            if q.parent is None:
                continue
            par = self.parts[q.parent]
            j = q.joint or {"type": "fixed"}
            rel = (par.R.T @ (q.frame_p - par.frame_p), par.R.T @ q.R)
            jname = j.get("name") or f"{n}_joint"
            if j["type"] == "fixed":
                L.extend(_joint_xml(jname, "fixed", par.link, q.link, rel))
            else:
                # The axis is expressed in the child (joint) frame.
                L.extend(_joint_xml(jname, j["type"], par.link, q.link, rel,
                                    axis=q.R.T @ j["axis"], limits=j))
        L.append("</robot>")
        text = "\n".join(L) + "\n"
        # Stamp the body so an importer can tell untouched output (the design is
        # authoritative) from a URDF that was edited by hand since.
        return text.replace("  <!-- vector:parts ", f"  <!-- vector:hash {urdf_body_hash(text)} -->\n  <!-- vector:parts ", 1)

    def part_map(self) -> Dict[str, Dict[str, Any]]:
        """link name -> the design part it came from and its catalog component
        (None for shells). The viewer and editors read identity from this
        instead of guessing it from link names."""
        out: Dict[str, Dict[str, Any]] = {}
        for n in self.order:
            q = self.parts[n]
            entry: Dict[str, Any] = {"part": n, "component": q.component["id"] if q.component else None}
            if q.mirror_of:
                entry["mirror_of"] = q.mirror_of
            if q.parent is not None:
                entry["joint"] = (q.joint or {}).get("name") or f"{n}_joint"
            out[q.link] = entry
        return out

    def _link_xml(self, q: Part) -> List[str]:
        m = self.mass_kg(q)
        off = q.geom_offset
        L = [f'  <link name="{q.link}">']
        if q.is_shell:
            prims = q.spec["shape"]
            if prims:
                L.append(_inertial_xml(m, (q.center_local + off) / 1000.0, q.size / 1000.0))
            for k, prim in enumerate(prims):
                # Visual and collision are the exact authored geometry (flush
                # faces between primitives are resolved by the renderer).
                geo = vgeo = _prim_geometry_xml(prim)
                xyz = (_prim_center(prim) + off) / 1000.0
                rpy = matrix_to_rpy(_prim_rot(prim))
                color = prim.get("color") or q.spec.get("color")
                if color is None and prim["shape"] != "mesh":
                    color = [0.62, 0.64, 0.68]
                mat = f'<material name="{q.link}_m{k}"><color rgba="{_f3(color)} 1"/></material>' if color else ''
                org = f'<origin xyz="{_f3(xyz)}" rpy="{_f3(rpy)}"/>'
                L.append(f'    <visual name="{_xml(prim.get("name", f"p{k}"))}">{org}{vgeo}{mat}</visual>')
                L.append(f'    <collision>{org}{geo}</collision>')
        else:
            sz = q.size / 1000.0
            L.append(_inertial_xml(m, off / 1000.0, sz))
            shape = (q.component.get("physical", {}) or {}).get("inertia_primitive", "box")
            if shape == "cylinder" and abs(sz[0] - sz[1]) < 1e-6:
                geo = f'<geometry><cylinder radius="{sz[0] / 2:.5f}" length="{sz[2]:.5f}"/></geometry>'
            else:
                geo = f'<geometry><box size="{_f3(sz)}"/></geometry>'
            org = f'<origin xyz="{_f3(off / 1000.0)}"/>' if np.linalg.norm(off) > 1e-6 else ''
            L.append(f'    <visual>{org}{geo}</visual>')
            L.append(f'    <collision>{org}{geo}</collision>')
        L.append('  </link>')
        return L


def _box_zmin(b: Box) -> float:
    return float(b.c[2] - np.abs(b.R[2, :]) @ b.h)


def _rel(par: Part, p_world: np.ndarray, R_world: np.ndarray):
    return (par.R.T @ (p_world - par.p), par.R.T @ R_world)


def _joint_xml(name, jtype, parent, child, rel, axis=None, limits=None) -> List[str]:
    xyz, R = rel
    rpy = matrix_to_rpy(R)
    L = [f'  <joint name="{_xml(name)}" type="{jtype}">',
         f'    <parent link="{parent}"/>', f'    <child link="{child}"/>',
         f'    <origin xyz="{_f3(xyz / 1000.0)}" rpy="{_f3(rpy)}"/>']
    if axis is not None:
        L.append(f'    <axis xyz="{_f3(axis)}"/>')
    if limits and jtype in ("revolute", "prismatic"):
        L.append(f'    <limit lower="{limits["lower"]:.4f}" upper="{limits["upper"]:.4f}" '
                 f'effort="{limits.get("effort", 5):.3f}" velocity="{limits.get("velocity", 3):.3f}"/>')
    elif limits and jtype == "continuous":
        L.append(f'    <limit effort="{limits.get("effort", 5):.3f}" velocity="{limits.get("velocity", 10):.3f}"/>')
    L.append('  </joint>')
    return L


def _inertial_xml(m: float, com_m: np.ndarray, size_m: np.ndarray) -> str:
    x, y, z = [max(float(v), 1e-3) for v in size_m]
    ixx, iyy, izz = m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12
    return (f'    <inertial><origin xyz="{_f3(com_m)}"/><mass value="{m:.4f}"/>'
            f'<inertia ixx="{ixx:.3e}" iyy="{iyy:.3e}" izz="{izz:.3e}" ixy="0" ixz="0" iyz="0"/></inertial>')


def _prim_geometry_xml(prim) -> str:
    shape = prim["shape"]
    if shape == "mesh":
        scale = prim.get("scale")
        sc = f' scale="{_f3(scale)}"' if scale else ''
        return f'<geometry><mesh filename="{_xml(prim["filename"])}"{sc}/></geometry>'

    if shape == "box":
        size = vec(prim["size_mm"])
        return f'<geometry><box size="{_f3(size / 1000.0)}"/></geometry>'
    r = float(prim["radius_mm"])
    if shape == "cylinder":
        length = float(prim["length_mm"])
        return f'<geometry><cylinder radius="{r / 1000:.5f}" length="{length / 1000:.5f}"/></geometry>'
    return f'<geometry><sphere radius="{r / 1000:.5f}"/></geometry>'


def _f3(v) -> str:
    return " ".join(f"{float(x):.6f}".rstrip("0").rstrip(".") if abs(float(x)) > 1e-9 else "0" for x in v)


def _xml(s: str) -> str:
    return str(s).replace("&", "&amp;").replace('"', "&quot;").replace("<", "&lt;")


_DESIGN_RE = re.compile(r"<!-- vector:design (.*?) -->", re.S)
_HASH_RE = re.compile(r"<!-- vector:hash ([0-9a-f]+) -->")


def urdf_body_hash(urdf: str) -> str:
    """Hash of a URDF's content ignoring comments and whitespace."""
    import hashlib
    body = re.sub(r"<!--.*?-->", "", urdf, flags=re.S)
    return hashlib.sha1(re.sub(r"\s+", "", body).encode("utf-8")).hexdigest()[:16]


def design_is_current(urdf: str) -> bool:
    """True when the URDF is exactly what its embedded design compiles to
    (modulo formatting) — i.e. nobody edited the text since."""
    m = _HASH_RE.search(urdf or "")
    if m:
        return m.group(1) == urdf_body_hash(urdf)
    # Older output without a stamp: recompile the embedded design and compare.
    design = extract_design(urdf)
    if not design:
        return False
    try:
        asm = compile_design(design)
    except (DesignError, ValueError, KeyError, TypeError):
        return False
    name = re.search(r"<robot\s[^>]*?name=\"([^\"]*)\"", urdf)
    return urdf_body_hash(asm.to_urdf(name.group(1) if name else "robot")) == urdf_body_hash(urdf)
_PARTS_RE = re.compile(r"<!-- vector:parts (.*?) -->", re.S)


def extract_part_map(urdf: str) -> Optional[Dict[str, Any]]:
    m = _PARTS_RE.search(urdf or "")
    if not m:
        return None
    try:
        return json.loads(m.group(1))
    except json.JSONDecodeError:
        return None


def extract_design(urdf: str) -> Optional[Dict[str, Any]]:
    """Recover the design JSON embedded by to_urdf (for iterative edits)."""
    m = _DESIGN_RE.search(urdf or "")
    if not m:
        return None
    try:
        return json.loads(m.group(1))
    except json.JSONDecodeError:
        return None

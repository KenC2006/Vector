"""
URDF -> design importer.

Turns any URDF into an explicit-pose design so that every editor (manual
placement, the inspector, the AI designer) works on the same model.

- Untouched designer output: the embedded design is returned as-is.
- Designer output that was edited by hand since: the embedded design is
  MERGED with the URDF. Parts whose links and joints still match what the
  design compiles to keep their spec (their mates, mirrors and references);
  parts the edit changed are replaced by their exact pose from the URDF;
  added/removed links are added/removed. So a hand edit is kept exactly and
  everything it didn't touch stays a real, re-editable mate.
- Any other URDF: every link becomes one part at its forward-kinematics pose
  (zero joint state) with explicit axes. Joints keep their names, type, world
  pivot/axis, limits and ratings; links keep their names and masses. A link
  whose name and geometry match a catalog part becomes that component;
  everything else becomes a shell of its visual primitives (meshes are kept
  by filename).

What does not survive: collision geometry that differs from the visuals,
full inertia tensors (recomputed from the envelope), mimic/safety/transmission
tags. `import_urdf` reports those in `notes`.
"""
from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from core.designer.compile import (
    DesignError, compile_design, design_is_current, extract_design, extract_part_map, _HASH_RE,
)
from core.designer.geometry import matrix_to_rpy, mirror_name, rot_axis_angle, rpy_to_matrix
from core.presets import get_component, is_parametric_spec, resolve_component_bounds_mm

_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
_INSTANCE_RE = re.compile(r"^(.+?)_(\d+)$")


class ImportError_(ValueError):
    pass


def _floats(s: Optional[str], n: int, default: float = 0.0) -> np.ndarray:
    if not s:
        return np.full(n, default)
    vals = [float(v) for v in s.split()]
    if len(vals) != n:
        raise ImportError_(f"expected {n} numbers, got {s!r}")
    return np.array(vals)


def _origin(el) -> Tuple[np.ndarray, np.ndarray]:
    o = el.find("origin") if el is not None else None
    if o is None:
        return np.zeros(3), np.eye(3)
    return _floats(o.get("xyz"), 3) * 1000.0, rpy_to_matrix(_floats(o.get("rpy"), 3))


def _r(v, nd=4) -> List[float]:
    out = []
    for x in np.asarray(v, float).ravel():
        x = round(float(x), nd)
        out.append(0.0 if x == 0 else x)
    return out


def _deg(R: np.ndarray) -> List[float]:
    return _r([math.degrees(a) for a in matrix_to_rpy(R)], 6)


def _geometry(vis) -> Optional[Dict[str, Any]]:
    g = vis.find("geometry")
    if g is None or len(g) == 0:
        return None
    k = g[0]
    if k.tag == "box":
        return {"shape": "box", "size_mm": _r(_floats(k.get("size"), 3) * 1000.0)}
    if k.tag == "cylinder":
        return {"shape": "cylinder", "radius_mm": _r([float(k.get("radius")) * 1000.0])[0],
                "length_mm": _r([float(k.get("length")) * 1000.0])[0]}
    if k.tag == "sphere":
        return {"shape": "sphere", "radius_mm": _r([float(k.get("radius")) * 1000.0])[0]}
    if k.tag == "mesh":
        out: Dict[str, Any] = {"shape": "mesh", "filename": k.get("filename", "")}
        if k.get("scale"):
            out["scale"] = _r(_floats(k.get("scale"), 3), 6)
        return out
    return None


def _color(vis, materials: Dict[str, List[float]]) -> Optional[List[float]]:
    m = vis.find("material")
    if m is None:
        return None
    c = m.find("color")
    if c is not None and c.get("rgba"):
        return _r(_floats(c.get("rgba"), 4)[:3], 3)
    return materials.get(m.get("name", ""))


def _catalog_match(link: str, visuals, part_map: Optional[Dict[str, Any]]) -> Optional[Tuple[Dict, Optional[float]]]:
    """(component, length_mm|None) when this link is a single catalog part."""
    if part_map is not None:
        cid = (part_map.get(link) or {}).get("component")
    else:
        m = _INSTANCE_RE.match(link)
        cid = m.group(1) if m else None
    if not cid:
        return None
    comp = get_component(cid)
    if comp is None or len(visuals) != 1:
        return None
    geo = _geometry(visuals[0])
    if geo is None or geo["shape"] not in ("box", "cylinder"):
        return None
    if geo["shape"] == "box":
        size = np.array(geo["size_mm"])
    else:
        d = 2 * geo["radius_mm"]
        size = np.array([d, d, geo["length_mm"]])
    length = None
    if is_parametric_spec(comp):
        length = float(size[2])
        expect = np.asarray(resolve_component_bounds_mm(comp, {"length_mm": length}), float)
    else:
        expect = np.asarray(resolve_component_bounds_mm(comp), float)
    # The geometry must be the catalog envelope (a renamed or resized link is
    # not that part any more).
    if np.abs(expect - size).max() > max(1.0, 0.02 * float(expect.max())):
        return None
    return comp, length


# ── parsed URDF ─────────────────────────────────────────────────────────────

@dataclass
class _Urdf:
    root: Any
    name: str
    links: Dict[str, Any]
    child_of: Dict[str, Any]                   # child link -> joint element
    order: List[str]                           # BFS from the root link
    world: Dict[str, Tuple[np.ndarray, np.ndarray]] = field(default_factory=dict)   # link frames (mm)
    materials: Dict[str, List[float]] = field(default_factory=dict)
    part_map: Optional[Dict[str, Any]] = None  # the URDF's vector:parts identity map


def _parse(urdf: str) -> _Urdf:
    try:
        root = ET.fromstring(urdf)
    except ET.ParseError as e:
        raise ImportError_(f"URDF is not valid XML: {e}")
    if root.tag != "robot":
        raise ImportError_("root element must be <robot>")
    links = {l.get("name"): l for l in root.findall("link")}
    if not links:
        raise ImportError_("URDF has no links")
    child_of: Dict[str, Any] = {}
    kids: Dict[str, List[Any]] = {}
    for j in root.findall("joint"):
        p, c = j.find("parent").get("link"), j.find("child").get("link")
        if p not in links or c not in links:
            raise ImportError_(f"joint {j.get('name')} references a missing link")
        if c in child_of:
            raise ImportError_(f"link {c} has two parent joints")
        child_of[c] = j
        kids.setdefault(p, []).append(j)
    roots = [n for n in links if n not in child_of]
    if len(roots) != 1:
        raise ImportError_(f"URDF must have exactly one root link, found {roots}")
    u = _Urdf(root=root, name=root.get("name") or "robot", links=links, child_of=child_of, order=[roots[0]])
    u.world[roots[0]] = (np.zeros(3), np.eye(3))
    queue = [roots[0]]
    while queue:
        par = queue.pop(0)
        pp, pR = u.world[par]
        for j in kids.get(par, []):
            c = j.find("child").get("link")
            xyz, R = _origin(j)
            u.world[c] = (pp + pR @ xyz, pR @ R)
            u.order.append(c)
            queue.append(c)
    for m in root.findall("material"):
        c = m.find("color")
        if c is not None and c.get("rgba"):
            u.materials[m.get("name", "")] = _r(_floats(c.get("rgba"), 4)[:3], 3)
    return u


def _link_spec(u: _Urdf, ln: str, part_map: Optional[Dict[str, Any]], notes: List[str],
               frame: Tuple[np.ndarray, np.ndarray] = (np.eye(3), np.zeros(3))) -> Dict[str, Any]:
    """Explicit-pose spec of one link. `name`/`parent` are link names (callers
    map them to part names). `frame` maps URDF-world to the design world."""
    Rf, pf = frame
    el = u.links[ln]
    pw, Rw = u.world[ln]
    pw, Rw = Rf @ pw + pf, Rf @ Rw
    visuals = el.findall("visual")
    spec: Dict[str, Any] = {"name": ln, "link": ln}
    match = _catalog_match(ln, visuals, part_map)
    if match:
        comp, length = match
        xyz, Rv = _origin(visuals[0])
        spec["component"] = comp["id"]
        if length is not None:
            spec["length_mm"] = round(length, 3)
        p_part, R_part = pw + Rw @ xyz, Rw @ Rv
    else:
        shape = []
        for k, vis in enumerate(visuals):
            geo = _geometry(vis)
            if geo is None:
                continue
            xyz, Rv = _origin(vis)
            prim = {"name": vis.get("name") or f"p{k}", **geo, "xyz_mm": _r(xyz, 3)}
            if np.abs(Rv - np.eye(3)).max() > 1e-9:
                prim["rpy_deg"] = _deg(Rv)
            col = _color(vis, u.materials)
            if col:
                prim["color"] = col
            shape.append(prim)
        spec["shape"] = shape
        p_part, R_part = pw, Rw
        if any(v.find("geometry/mesh") is not None for v in visuals):
            notes.append(f"{ln}: mesh visuals are kept by filename; geometry checks treat them as points.")
    spec["at"] = _r(p_part, 3)
    spec["z_axis"] = _r(R_part[:, 2], 6)
    spec["x_axis"] = _r(R_part[:, 0], 6)

    inert = el.find("inertial")
    if inert is not None and inert.find("mass") is not None:
        spec["mass_kg"] = float(inert.find("mass").get("value"))

    j = u.child_of.get(ln)
    if j is not None:
        spec["parent"] = j.find("parent").get("link")
        jt = j.get("type")
        jd: Dict[str, Any] = {"name": j.get("name")}
        if jt in ("floating", "planar"):
            notes.append(f"{j.get('name')}: {jt} joints are not supported; imported as fixed.")
            jt = "fixed"
        jd["type"] = jt
        if jt != "fixed":
            ax = _floats(j.find("axis").get("xyz") if j.find("axis") is not None else None, 3)
            if not np.any(ax):
                ax = np.array([1.0, 0, 0])
            jd["pivot"] = _r(pw, 3)
            jd["axis"] = _r(Rw @ (ax / np.linalg.norm(ax)), 6)
            jd.update(_limit_fields(j, jt, rest_deg=0.0))
            if "passive" not in jd:
                jd["passive"] = False
            if j.find("mimic") is not None:
                notes.append(f"{j.get('name')}: <mimic> dropped.")
        spec["joint"] = jd
    return spec


def _limit_fields(j, jt: str, rest_deg: float) -> Dict[str, Any]:
    """Limits/ratings of a URDF joint as design fields (revolute limits are
    authored relative to the straight pose, so the rest angle is added back)."""
    out: Dict[str, Any] = {}
    lim = j.find("limit")
    if lim is None:
        return out
    if jt == "revolute":
        out["lower_deg"] = round(math.degrees(float(lim.get("lower", 0))) + rest_deg, 3)
        out["upper_deg"] = round(math.degrees(float(lim.get("upper", 0))) + rest_deg, 3)
    elif jt == "prismatic":
        out["lower_mm"] = round(float(lim.get("lower", 0)) * 1000.0, 3)
        out["upper_mm"] = round(float(lim.get("upper", 0)) * 1000.0, 3)
    if lim.get("effort") is not None:
        out["effort"] = float(lim.get("effort"))
        if float(lim.get("effort")) <= 0:
            out["passive"] = True
    if lim.get("velocity") is not None:
        out["velocity"] = float(lim.get("velocity"))
    return out


def _plain_import(u: _Urdf, part_map: Optional[Dict[str, Any]], notes: List[str]) -> Dict[str, Any]:
    names: Dict[str, str] = {}
    used = set()
    for ln in u.order:
        base = ln if _NAME_RE.match(ln) else ("l_" + re.sub(r"[^A-Za-z0-9_]", "_", ln)).strip("_")
        n, k = base, 2
        while n in used:
            n, k = f"{base}_{k}", k + 1
        used.add(n)
        names[ln] = n
    parts = []
    for ln in u.order:
        spec = _link_spec(u, ln, part_map, notes)
        spec["name"] = names[ln]
        if "parent" in spec:
            spec["parent"] = names[spec["parent"]]
        parts.append(spec)
    return {"name": u.name, "summary": "Imported from URDF.", "parts": parts}


# ── merging a hand-edited URDF into its embedded design ─────────────────────

def _T(el) -> np.ndarray:
    xyz, R = _origin(el)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, xyz
    return T


def _geom_sig(vis) -> Tuple:
    g = vis.find("geometry")
    k = g[0] if g is not None and len(g) else None
    if k is None:
        return ("none",)
    if k.tag == "mesh":
        return ("mesh", k.get("filename"), tuple(_r(_floats(k.get("scale"), 3, 1.0), 6)))
    vals = [float(v) for a in ("size", "radius", "length") if k.get(a) for v in k.get(a).split()]
    return (k.tag, tuple(round(v, 6) for v in vals))


def _same_T(a: np.ndarray, b: np.ndarray) -> bool:
    return np.abs(a[:3, 3] - b[:3, 3]).max() < 0.01 and np.abs(a[:3, :3] - b[:3, :3]).max() < 1e-5


def _link_same(a, b) -> bool:
    va, vb = a.findall("visual"), b.findall("visual")
    return len(va) == len(vb) and all(
        _geom_sig(x) == _geom_sig(y) and _same_T(_T(x), _T(y)) for x, y in zip(va, vb))


def _joint_same(a, b) -> bool:
    if a is None or b is None:
        return a is None and b is None
    if a.get("type") != b.get("type") or a.find("parent").get("link") != b.find("parent").get("link"):
        return False
    if not _same_T(_T(a), _T(b)):
        return False
    if a.get("type") != "fixed":
        ax = lambda j: _floats(j.find("axis").get("xyz") if j.find("axis") is not None else "1 0 0", 3)
        if np.abs(ax(a) - ax(b)).max() > 1e-5:
            return False
    return True


def _refs(spec: Dict[str, Any]) -> List[str]:
    out = []
    at = spec.get("at")
    for r in ([at] if isinstance(at, str) else [at.get("ref")] if isinstance(at, dict) else []):
        if isinstance(r, str):
            out.append(r.split(".")[0])
    j = spec.get("joint") if isinstance(spec.get("joint"), dict) else {}
    pv = j.get("pivot")
    for r in ([pv] if isinstance(pv, str) else [pv.get("ref")] if isinstance(pv, dict) else []):
        if isinstance(r, str):
            out.append(r.split(".")[0])
    if isinstance(j.get("axis"), str) and "." in j["axis"]:
        out.append(j["axis"].split(".")[0])
    return out


def _merge(embedded: Dict[str, Any], u: _Urdf, stamped: bool, notes: List[str]) -> Optional[Dict[str, Any]]:
    try:
        asm = compile_design(embedded)
    except (DesignError, ValueError, KeyError, TypeError):
        return None
    ref = _parse(asm.to_urdf(u.name))
    link_of = {n: q.link for n, q in asm.parts.items()}
    part_of_link = {l: n for n, l in link_of.items()}

    def unchanged(part: str) -> bool:
        ln = link_of[part]
        if ln not in u.links:
            return False
        return _link_same(ref.links[ln], u.links[ln]) and _joint_same(ref.child_of.get(ln), u.child_of.get(ln))

    authored = {p["name"]: p for p in embedded["parts"]}
    mirror_source = {mirror_name(n): n for n, p in authored.items() if p.get("mirror")}
    changed_copy = {n for n, p in authored.items()
                    if p.get("mirror") and mirror_name(n) in link_of and not unchanged(mirror_name(n))}
    def world_delta(part: Optional[str]) -> str:
        """How a part's link frame moved in the world: 'same', 'moved' (pure
        translation) or 'turned'."""
        if part is None or part not in link_of or link_of[part] not in u.world:
            return "same"
        (pa, Ra), (pb, Rb) = ref.world[link_of[part]], u.world[link_of[part]]
        if np.abs(Ra - Rb).max() > 1e-5:
            return "turned"
        return "same" if np.abs(pa - pb).max() < 0.01 else "moved"

    def relative_only(spec: Dict[str, Any]) -> bool:
        """Placed purely from its parent's anchors, so it follows the parent
        through a translation."""
        par = spec.get("parent")
        at = spec.get("at")
        j = spec.get("joint") if isinstance(spec.get("joint"), dict) else {}
        return (all(r == par for r in _refs(spec)) and isinstance(at, (str, dict))
                and not isinstance(j.get("pivot"), list))

    keep = {n for n in authored if n in link_of and unchanged(n)}
    # A kept spec must still produce the part's (possibly moved) world pose:
    # everything it references other than its parent must be kept and unmoved,
    # and if the parent moved, the spec must follow it (only a translation,
    # placed purely from the parent's anchors).
    grew = True
    while grew:
        grew = False
        for n in list(keep):
            spec = authored[n]
            par = spec.get("parent")
            others = [r for r in _refs(spec) if r != par]
            ok = all(r in keep and world_delta(r) == "same" for r in others)
            d = world_delta(par)
            if d == "turned" or (d == "moved" and not relative_only(spec)):
                ok = False
            if not ok:
                keep.discard(n)
                grew = True
    # Un-mirror pairs where either side changed, and every mirrored part below them.
    unmirror = {n for n in authored if authored[n].get("mirror") and (n not in keep or n in changed_copy)}
    # ... and every mirrored part below them (their copies hang off the copy),
    # and every mirrored part above them (the materialised copy needs a real
    # parent to attach to).
    grew = True
    while grew:
        grew = False
        for n, p in authored.items():
            if p.get("mirror") and n not in unmirror and p.get("parent") in unmirror:
                unmirror.add(n)
                grew = True
        for n in list(unmirror):
            par = authored[n].get("parent")
            if par in authored and authored[par].get("mirror") and par not in unmirror:
                unmirror.add(par)
                grew = True

    # URDF world = the root link frame. Express URDF poses in the design world
    # as authored, i.e. before the compiler's grounding shift (which the merged
    # design gets again when it compiles).
    root = asm.parts[asm.root]
    frame = (root.R, root.frame_p - asm.ground_shift)
    kept_names: List[str] = []
    parts: List[Dict[str, Any]] = []

    def from_urdf(part_name: str, ln: str) -> Dict[str, Any]:
        spec = _link_spec(u, ln, u.part_map, notes, frame=frame)
        spec["name"] = part_name
        if "parent" in spec:
            spec["parent"] = part_of_link.get(spec["parent"], spec["parent"])
        # An unchanged custom body keeps its authored shape and origin (other
        # parts may be placed at its center or on its primitives' faces);
        # only its pose comes from the URDF.
        src_spec = authored.get(part_name) or authored.get(mirror_source.get(part_name, ""))
        if (src_spec is not None and "shape" in src_spec and "shape" in spec and part_name in asm.parts
                and _link_same(ref.links[ln], u.links[ln])):
            R = np.column_stack([np.asarray(spec["x_axis"], float),
                                 np.cross(spec["z_axis"], spec["x_axis"]),
                                 np.asarray(spec["z_axis"], float)])
            spec["at"] = _r(np.asarray(spec["at"], float) + R @ asm.parts[part_name].geom_offset, 3)
            spec["shape"] = [dict(q) for q in src_spec["shape"]]
        _unbend(spec, _bent_ancestors(spec.get("parent")))
        # A part that was authored bent keeps its rest angle: re-import it in
        # its straight pose so everything designed on it stays valid.
        orig = authored.get(part_name) or authored.get(mirror_source.get(part_name, ""))
        oj = orig.get("joint") if orig else None
        rest = float(oj.get("rest_deg", 0) or 0) if isinstance(oj, dict) else 0.0
        j = spec.get("joint")
        if rest and isinstance(j, dict) and j.get("type") in ("revolute", "continuous") and "pivot" in j:
            # (A mirrored copy's joint axis is already the reflected one; the
            # reflected rotation has the same angle about it.)
            _unbend(spec, [(np.asarray(j["pivot"], float), np.asarray(j["axis"], float), math.radians(rest))])
            j["pivot"] = _r(np.asarray(j["pivot"], float), 3)
            j["rest_deg"] = round(rest, 6)
            jel = u.child_of.get(ln)
            if jel is not None:
                j.update({k: v for k, v in _limit_fields(jel, j["type"], rest).items() if k.endswith("_deg")})
        return spec

    def _bent_ancestors(start: Optional[str]) -> List[Tuple[np.ndarray, np.ndarray, float]]:
        """(pivot, axis, rest) of ancestors with a rest angle, deepest first.
        Every one of them still bends its subtree in the merged design (kept
        specs keep their rest, re-imported ones are given it back), so a pose
        read from the bent URDF must be written in the straight pose."""
        out, n = [], start
        Rf, pf = frame
        while n is not None and n in asm.parts:
            j = asm.parts[n].joint or {}
            ln = link_of.get(n)
            jel = u.child_of.get(ln) if ln else None
            if j.get("rest") and "pivot" in j and jel is not None:
                # Where that joint is in the EDITED robot (its pivot is its
                # link frame origin), in the pre-grounding design frame.
                pw, Rw = u.world[ln]
                ax = _floats(jel.find("axis").get("xyz") if jel.find("axis") is not None else None, 3)
                ax = ax / np.linalg.norm(ax) if np.any(ax) else np.array([1.0, 0, 0])
                out.append((Rf @ pw + pf, Rf @ (Rw @ ax), float(j["rest"])))
            n = asm.parts[n].parent
        return out

    for p in embedded["parts"]:
        n = p["name"]
        ln = link_of.get(n)
        if ln is None or ln not in u.links:
            notes.append(f"{n}: removed (its link is gone from the URDF).")
            continue
        if n in keep:
            spec = dict(p)
            if stamped:
                _carry_numbers(spec, u, ln)
        else:
            spec = from_urdf(n, ln)
            notes.append(f"{n}: pose taken from the edited URDF.")
        if n in unmirror:
            spec.pop("mirror", None)
        spec["link"] = ln        # keep link names stable across the merge
        if spec.get("mirror") and link_of.get(mirror_name(n)):
            spec["mirror_link"] = link_of[mirror_name(n)]
        parts.append(spec)
        kept_names.append(n)
        if p.get("mirror") and n in unmirror:
            m = mirror_name(n)
            if link_of.get(m) in u.links:
                parts.append(from_urdf(m, link_of[m]))
    for ln in u.order:
        if ln not in part_of_link:
            spec = from_urdf(ln if _NAME_RE.match(ln) else "l_" + re.sub(r"[^A-Za-z0-9_]", "_", ln), ln)
            parts.append(spec)
            notes.append(f"{spec['name']}: added from the URDF.")
    design = dict(embedded)
    design["parts"] = _ordered(parts)
    return design


def _unbend(spec: Dict[str, Any], bends: List[Tuple[np.ndarray, np.ndarray, float]]) -> None:
    """Undo rest-angle rotations (deepest first) on an explicit-pose spec."""
    for pivot, axis, rest in bends:
        Rinv = rot_axis_angle(axis, -rest)
        p = np.asarray(spec["at"], float)
        spec["at"] = _r(pivot + Rinv @ (p - pivot), 3)
        spec["z_axis"] = _r(Rinv @ np.asarray(spec["z_axis"], float), 6)
        spec["x_axis"] = _r(Rinv @ np.asarray(spec["x_axis"], float), 6)
        j = spec.get("joint")
        if isinstance(j, dict) and isinstance(j.get("pivot"), list):
            jp = np.asarray(j["pivot"], float)
            j["pivot"] = _r(pivot + Rinv @ (jp - pivot), 3)
            j["axis"] = _r(Rinv @ np.asarray(j["axis"], float), 6)


def _carry_numbers(spec: Dict[str, Any], u: _Urdf, ln: str) -> None:
    """Hand-edited numbers on an otherwise unchanged part: mass, joint limits and ratings."""
    inert = u.links[ln].find("inertial")
    if inert is not None and inert.find("mass") is not None:
        spec["mass_kg"] = float(inert.find("mass").get("value"))
    j = u.child_of.get(ln)
    if j is None or j.get("type") == "fixed" or not isinstance(spec.get("joint"), (dict, str)):
        return
    jd = {"type": spec["joint"]} if isinstance(spec["joint"], str) else dict(spec["joint"])
    fields = _limit_fields(j, j.get("type"), float(jd.get("rest_deg", 0) or 0))
    jd.update(fields)
    spec["joint"] = jd


def _ordered(parts: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Parents and referenced parts before the parts that use them."""
    by = {p["name"]: p for p in parts}
    out, placed = [], set()
    pending = list(parts)
    while pending:
        progressed = False
        for p in list(pending):
            deps = [r for r in _refs(p) + ([p["parent"]] if p.get("parent") else []) if r in by and r != p["name"]]
            if all(d in placed for d in deps):
                out.append(p)
                placed.add(p["name"])
                pending.remove(p)
                progressed = True
        if not progressed:           # a cycle: keep the original order for the rest
            out.extend(pending)
            break
    return out


# ── entry point ─────────────────────────────────────────────────────────────

def import_urdf(urdf: str) -> Dict[str, Any]:
    """-> {"design", "notes": [str], "imported": bool}. `imported` is False only
    when the URDF is untouched designer output (the embedded design as-is)."""
    embedded = extract_design(urdf)
    if embedded and design_is_current(urdf):
        return {"design": embedded, "notes": [], "imported": False}
    u = _parse(urdf)
    u.part_map = part_map = extract_part_map(urdf)
    notes: List[str] = []
    if embedded:
        merged = _merge(embedded, u, stamped=bool(_HASH_RE.search(urdf)), notes=notes)
        if merged is not None:
            return {"design": merged, "notes": notes, "imported": True}
        notes.append("The embedded design no longer compiles; the design was rebuilt from the URDF.")
    design = _plain_import(u, part_map, notes)
    if u.root.findall("transmission"):
        notes.append("<transmission> elements dropped.")
    return {"design": design, "notes": notes, "imported": True}

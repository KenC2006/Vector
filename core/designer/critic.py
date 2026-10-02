"""
Geometry critic: measures a compiled assembly and reports problems in plain
numbers the designer model can act on — floating parts, parts clipping into
each other, what touches the ground, static stability, and where sensors
point. Boxes are the catalog bounding boxes / shell primitives, so distances
are approximate to a few millimetres.
"""
from __future__ import annotations

import itertools
from typing import Dict, List, Tuple

import numpy as np

from core.designer.compile import Assembly, Box, Part
from core.presets import actuator_rating
from core.presets.identity import GROUND_CONTACT_CLASSES, contact_class_of

GAP_TOL = 3.0          # mm — closer than this counts as touching
PEN_TOL = 4.0          # mm — deeper than this counts as clipping
GROUND_TOL = 3.0       # mm above z=0 still counts as ground contact
# Contact classes (catalog sim_metadata.contact_class) that must reach the
# ground when the robot stands on it. Suction pads MAY touch the ground
# (climbers) but an arm's suction tool need not.
MUST_TOUCH_GROUND = frozenset(("foot", "wheel", "track", "caster"))


def _contact_class(q: Part) -> str:
    return "none" if q.is_shell else contact_class_of(q.component)


def _surface_samples(b: Box, n: int = 5) -> np.ndarray:
    """Points on the 6 faces of an oriented box."""
    t = np.linspace(-1, 1, n)
    uu, vv = np.meshgrid(t, t)
    uu, vv = uu.ravel(), vv.ravel()
    pts = []
    for ax in range(3):
        o = [i for i in range(3) if i != ax]
        for s in (-1, 1):
            loc = np.zeros((uu.size, 3))
            loc[:, ax] = s
            loc[:, o[0]] = uu
            loc[:, o[1]] = vv
            pts.append(loc)
    loc = np.vstack(pts) * b.h
    return b.c + loc @ b.R.T


def _signed_dist(points: np.ndarray, b: Box) -> np.ndarray:
    """Signed distance from points to an oriented box (negative inside)."""
    loc = (points - b.c) @ b.R
    q = np.abs(loc) - b.h
    outside = np.linalg.norm(np.maximum(q, 0), axis=1)
    inside = np.minimum(np.max(q, axis=1), 0)
    return outside + inside


def box_gap_and_pen(a: List[Box], b: List[Box]) -> Tuple[float, float]:
    """(min gap mm >= 0, max penetration depth mm >= 0) between two box sets."""
    gap, pen = np.inf, 0.0
    for ba in a:
        sa = _surface_samples(ba)
        for bb in b:
            sb = _surface_samples(bb)
            d1, d2 = _signed_dist(sa, bb), _signed_dist(sb, ba)
            gap = min(gap, max(0.0, float(min(d1.min(), d2.min()))))
            pen = max(pen, float(max(-d1.min(), -d2.min(), 0.0)))
    return gap, pen


def critique(asm: Assembly) -> Dict:
    parts = asm.parts
    boxes = asm.all_boxes()
    issues: List[str] = []

    # ── attachment: every part must touch its kinematic parent ───────────
    touching: Dict[Tuple[str, str], Tuple[float, float]] = {}
    for a, b in itertools.combinations(asm.order, 2):
        touching[(a, b)] = touching[(b, a)] = box_gap_and_pen(boxes[a], boxes[b])

    for n in asm.order:
        q = parts[n]
        if q.parent is None or q.mirror_of or not boxes[n] or not boxes[q.parent]:
            continue
        gap, _ = touching[(n, q.parent)]
        if gap > GAP_TOL:
            near = sorted(((touching[(n, o)][0], o) for o in asm.order if o != n), key=lambda t: t[0])[:1]
            hint = f"; nearest part is {near[0][1]} ({near[0][0]:.0f}mm)" if near else ""
            issues.append(f"FLOATING: {n} does not touch its parent {q.parent} — gap {gap:.0f}mm{hint}.")

    # ── clipping between parts that are not parent/child ─────────────────
    related = {(n, parts[n].parent) for n in asm.order if parts[n].parent}
    related |= {(b, a) for a, b in related}
    # A mirrored copy clashes exactly where its source does, so every clash
    # has a mirror-image twin (A–B ↔ A'–B', and A–B' ↔ A'–B). Report each
    # {pair, mirror pair} class once — double counting skewed the issue
    # totals that pick the best design round.
    twin = {}
    for n in asm.order:
        if parts[n].mirror_of:
            twin[n], twin[parts[n].mirror_of] = parts[n].mirror_of, n
    reported = set()
    for a, b in itertools.combinations(asm.order, 2):
        if (a, b) in related or parts[a].mirror_of == b or parts[b].mirror_of == a:
            continue
        gap, pen = touching[(a, b)]
        if pen <= PEN_TOL:
            continue
        pair = frozenset((a, b))
        mirror = frozenset((twin.get(a, a), twin.get(b, b)))
        if pair in reported or mirror in reported:
            continue
        reported.add(pair)
        issues.append(f"CLIPPING: {a} and {b} intersect by {pen:.0f}mm.")

    # ── ground contact ───────────────────────────────────────────────────
    contacts = []
    for n in asm.order:
        if not boxes[n]:
            continue
        zmin = min(float(b.c[2] - np.abs(b.R[2, :]) @ b.h) for b in boxes[n])
        if zmin < GROUND_TOL:
            contacts.append(n)
    # Which parts are meant to touch the ground comes from the catalog
    # contact class (feet, wheels, tracks, casters, suction pads).
    ground_like = [n for n in asm.order if boxes[n] and _contact_class(parts[n]) in GROUND_CONTACT_CLASSES]
    for n in ground_like:
        if parts[n].mirror_of or _contact_class(parts[n]) not in MUST_TOUCH_GROUND:
            continue   # a mirrored copy sits at its source's height: report once
        zmin = min(float(b.c[2] - np.abs(b.R[2, :]) @ b.h) for b in boxes[n])
        if zmin > GROUND_TOL:
            issues.append(f"OFF_GROUND: {n} is {zmin:.0f}mm above the ground while other parts touch it.")
    if ground_like:
        bad = [n for n in contacts if n not in ground_like]
        if bad:
            issues.append("GROUND_CONTACT: " + ", ".join(bad) +
                          " touch(es) the ground — only feet/wheels/tracks should. Raise the body or lengthen the legs.")

    # ── static stability ─────────────────────────────────────────────────
    com = asm.com()
    pts = []
    for n in contacts:
        for b in boxes[n]:
            for s in itertools.product((-1, 1), repeat=3):
                c = b.c + b.R @ (np.array(s) * b.h)
                if c[2] < GROUND_TOL:
                    pts.append(c[:2])
    stability = "unknown"
    if len(pts) >= 3:
        margin = _hull_margin(np.array(pts), com[:2])
        stability = f"COM {'inside' if margin >= 0 else 'OUTSIDE'} support polygon (margin {margin:.0f}mm)"
        if margin < 0:
            issues.append(f"TIPS_OVER: centre of mass (x={com[0]:.0f}, y={com[1]:.0f}) is {-margin:.0f}mm outside the ground-contact polygon.")

    # ── actuator load: can every joint hold the standing pose? ───────────
    issues.extend(_load_issues(asm, boxes, contacts))

    # ── sensors: where do they look? ─────────────────────────────────────
    facing = []
    for n in asm.order:
        q = parts[n]
        if q.is_shell:
            continue
        for conn in q.component.get("connectors", []) or []:
            if conn["id"] in ("optical_front", "lens_front", "sensing_face", "gripper_open", "optical_top"):
                d = q.R @ np.asarray(conn["axis_xyz"], float)
                facing.append(f"{n}.{conn['id']} points {_dir_word(d)}")

    return {
        "issues": issues,
        "ground_contacts": contacts,
        "stability": stability,
        "com_mm": [round(float(v)) for v in com],
        "facing": facing,
        "size_mm": _overall_size(boxes),
    }


G = 9.81
LOAD_LIMIT = 0.6   # holding torque should leave 40% of the rating for motion


def _load_issues(asm: Assembly, boxes, contacts: List[str]) -> List[str]:
    """Static holding torque (revolute) or force (prismatic) per driven joint
    vs its actuator's catalog rating.

    Ground reactions: the minimum-norm vertical forces at the contact parts
    that balance total weight and its moment about the COM. Each joint then
    carries the gravity and ground-reaction moments of everything beyond it.
    """
    parts = asm.parts
    mass = {n: asm.mass_kg(parts[n]) for n in asm.order}
    com_of = {n: parts[n].world(parts[n].center_local) / 1000.0 for n in asm.order}
    W = G * sum(mass.values())
    feet = []
    for n in contacts:
        pts = []
        for b in boxes[n]:
            for sgn in itertools.product((-1, 1), repeat=3):
                c = b.c + b.R @ (np.array(sgn) * b.h)
                if c[2] < GROUND_TOL:
                    pts.append(c)
        if pts:
            feet.append((n, np.mean(pts, axis=0) / 1000.0))
    reaction = {}
    if feet:
        com = asm.com() / 1000.0
        A = np.array([[1.0] * len(feet), [f[1][0] for f in feet], [f[1][1] for f in feet]])
        rhs = np.array([W, W * com[0], W * com[1]])
        F = np.linalg.lstsq(A, rhs, rcond=None)[0]
        reaction = {n: (pt, max(0.0, float(f))) for (n, pt), f in zip(feet, F)}

    children = {}
    for n in asm.order:
        if parts[n].parent:
            children.setdefault(parts[n].parent, []).append(n)

    def subtree(n):
        out, stack = [], [n]
        while stack:
            k = stack.pop()
            out.append(k)
            stack.extend(children.get(k, []))
        return out

    # Walking puts a leg's share of the weight on half the feet (trot), and
    # the lever grows as the leg swings, so leg joints are sized for that.
    gait_force = W / max(1.0, len(feet) / 2.0)

    issues = []
    for n in asm.order:
        q = parts[n]
        j = q.joint or {}
        jtype = j.get("type")
        if q.mirror_of or jtype not in ("revolute", "prismatic") or j.get("passive"):
            continue
        rating = actuator_rating(parts[q.parent].component if not parts[q.parent].is_shell else None)
        want = "linear" if jtype == "prismatic" else "rotary"
        if rating is None or rating[0] != want:
            continue  # passive pivot (rocker, bogie, free hinge): nothing to overload
        pivot, axis = j["pivot"] / 1000.0, j["axis"]
        sub = subtree(n)
        sub_feet = [reaction[k][0] for k in sub if k in reaction]
        if jtype == "prismatic":
            # A slide carries the load component along its axis.
            if sub_feet:
                load = gait_force * abs(float(axis[2]))
                what = "to carry its share of the weight while walking"
            else:
                load = abs(sum(G * mass[k] for k in sub) * float(axis[2]))
                what = "to hold what it carries against gravity"
            unit = "N"
        else:
            if sub_feet:
                foot = sub_feet[0]
                d = foot - pivot
                lever = max(float(np.hypot(d[0], d[1])), 0.5 * float(np.linalg.norm(d)))
                load, what = gait_force * lever, "to carry its share of the weight while walking"
            else:
                load = abs(sum(float(axis @ np.cross(com_of[k] - pivot, [0.0, 0.0, -G * mass[k]]))
                               for k in sub))
                what = "to hold what it carries against gravity"
            unit = "N·m"
        effort = rating[1]
        if load > LOAD_LIMIT * effort:
            driver = parts[q.parent].component["id"] if parts[q.parent].component else q.parent
            issues.append(
                f"OVERLOADED: {n}'s joint needs ~{load:.1f} {unit} {what}, but its actuator {q.parent} ({driver}) "
                f"is rated {effort:.1f} {unit} (keep load under {int(LOAD_LIMIT * 100)}% of rating) — use a stronger "
                f"actuator, shorten the lever, or lighten the robot.")
    return issues


def _dir_word(d: np.ndarray) -> str:
    names = ["+x (forward)", "+y (left)", "+z (up)"]
    i = int(np.argmax(np.abs(d)))
    w = names[i] if d[i] > 0 else names[i].replace("+", "-").replace("forward", "backward").replace("left", "right").replace("up", "down")
    return f"{w} [{d[0]:.2f},{d[1]:.2f},{d[2]:.2f}]"


def _overall_size(boxes) -> List[int]:
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for bs in boxes.values():
        for b in bs:
            e = np.abs(b.R) @ b.h
            lo, hi = np.minimum(lo, b.c - e), np.maximum(hi, b.c + e)
    return [round(float(v)) for v in hi - lo]


def _hull_margin(pts: np.ndarray, p: np.ndarray) -> float:
    """Signed distance of p inside the convex hull of pts (negative = outside)."""
    pts = np.unique(np.round(pts, 3), axis=0)
    if len(pts) < 3:
        return -1.0
    hull = _convex_hull(pts)
    if len(hull) < 3:
        return -1.0
    margin = np.inf
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        e = b - a
        nrm = np.array([e[1], -e[0]]) / (np.linalg.norm(e) + 1e-12)  # outward for CCW hull
        margin = min(margin, -float((p - a) @ nrm))
    return margin


def _convex_hull(pts: np.ndarray) -> np.ndarray:
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def format_report(asm: Assembly, rep: Dict) -> str:
    lines = [f"Overall size (x,y,z): {rep['size_mm']} mm. Centre of mass: {rep['com_mm']} mm. {rep['stability']}."]
    lines.append("Ground contacts: " + (", ".join(rep["ground_contacts"]) or "none"))
    if rep["facing"]:
        lines.append("Sensors/tools: " + "; ".join(rep["facing"]))
    lines.append("")
    lines.append("Part placements (world mm, rest pose, after grounding):")
    for n in asm.order:
        q = asm.parts[n]
        if q.mirror_of:
            continue
        c = q.world(q.center_local)
        what = q.component["id"] if q.component else "shell"
        lines.append(f"  {n} [{what}] center=({c[0]:.0f},{c[1]:.0f},{c[2]:.0f}) size={[round(float(v)) for v in q.size]}"
                     f" local+z->{_dir_word(q.R[:, 2]).split(' [')[0]}")
    lines.append("")
    if rep["issues"]:
        lines.append(f"PROBLEMS ({len(rep['issues'])}):")
        lines.extend(f"  - {s}" for s in rep["issues"])
    else:
        lines.append("No geometric problems found.")
    return "\n".join(lines)

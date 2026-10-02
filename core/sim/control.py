"""
Controller runtime, robot brief and headless evaluation for sim scripts.

A sim script defines ``step(t, state) -> {joint: command}``. The runtime calls
it at a fixed control rate (independent of the UI frame rate) and hands it a
small, stable ``state`` dict:

    state = {
      "t": float,                        # sim time, s
      "dt": float,                       # control period, s
      "q":  {joint: position},           # rad or m (continuous joints: unwrapped angle)
      "qd": {joint: velocity},           # rad/s or m/s
      "effort": {joint: actuator force}, # N·m or N
      "base": {
        "pos": [x, y, z],                # m, world
        "rpy": [roll, pitch, yaw],       # rad, world (ZYX)
        "lin_vel": [vx, vy, vz],         # m/s in the heading frame (x = robot forward)
        "ang_vel": [wx, wy, wz],         # rad/s in the body frame
      },
      "contacts": [link, ...],           # robot links touching the ground now
      "cmd": {"active": bool, "vx": m/s, "vy": m/s, "yaw_rate": rad/s},  # operator teleop
    }

``evaluate_controller`` runs a script headless and reports what actually
happened (distance, heading, falls, tracking, saturation, ground contacts), so
the controller generator can test and revise before handing code to the user.
"""
from __future__ import annotations

import ast
import math
import os
import sys
import tempfile
from typing import Any, Callable, Dict, List, Optional

import numpy as np

from core.sim.mujoco_adapter import MuJoCoSimulator

CONTROL_DT = 0.005          # 200 Hz control loop
DEFAULT_CMD = {"active": False, "vx": 0.0, "vy": 0.0, "yaw_rate": 0.0}

# ── Sandbox ────────────────────────────────────────────────────────────────
import builtins as _builtins


def _script_print(*args, sep=" ", end="\n", **_ignored) -> None:
    """print() for sim scripts. stdout is the core's JSON-RPC channel, so a
    script's debug output goes to stderr (the app's log) and can never
    corrupt a response. file=/flush= are ignored."""
    try:
        sys.stderr.write("[sim script] " + sep.join(str(a) for a in args) + end)
        sys.stderr.flush()
    except Exception:
        pass


SAFE_BUILTINS = {
    name: getattr(_builtins, name)
    for name in (
        "abs", "min", "max", "round", "sum", "len", "range", "enumerate", "zip",
        "map", "filter", "sorted", "reversed", "all", "any",
        "int", "float", "bool", "str", "list", "tuple", "dict", "set",
        "isinstance",
    )
}
SAFE_BUILTINS["print"] = _script_print
NAME_DENY = frozenset({
    "eval", "exec", "compile", "open", "__import__", "getattr", "setattr",
    "delattr", "globals", "locals", "vars", "input", "help",
})


def fresh_script_globals() -> Dict[str, Any]:
    return {"__builtins__": SAFE_BUILTINS, "math": math}


def reject_unsafe_script(code: str) -> None:
    """Static AST scan: no imports, no dunder attribute access, no exec/eval."""
    tree = ast.parse(code, mode="exec")
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            raise ValueError("Imports are not allowed in sim scripts (math is already available)")
        if isinstance(node, ast.Attribute) and node.attr.startswith("__"):
            raise ValueError(f"Access to dunder attribute '{node.attr}' is not allowed")
        if isinstance(node, ast.Name) and (node.id in NAME_DENY or node.id.startswith("__")):
            raise ValueError(f"Use of '{node.id}' is not allowed in sim scripts")


def compile_script(code: str) -> Callable:
    reject_unsafe_script(code)
    g = fresh_script_globals()
    exec(compile(code, "<sim_script>", "exec"), g)
    fn = g.get("step")
    if fn is None or not callable(fn):
        raise ValueError("Script must define a callable 'step(t, state)' function")
    return fn


# ── Runtime ────────────────────────────────────────────────────────────────

def _quat_to_rpy(w: float, x: float, y: float, z: float) -> List[float]:
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [roll, pitch, yaw]


class ControllerRuntime:
    """Drives a loaded MuJoCoSimulator with a step(t, state) script at a fixed rate."""

    def __init__(self, sim: MuJoCoSimulator, control_dt: float = CONTROL_DT):
        self.sim = sim
        self.control_dt = control_dt
        self.fn: Optional[Callable] = None
        self.cmd: Dict[str, Any] = dict(DEFAULT_CMD)
        self._next_control_t = 0.0
        self._index()

    def _index(self) -> None:
        mj, m = self.sim.mujoco, self.sim.model
        self.joints: List[tuple] = []   # (name, qpos_adr, dof_adr)
        for i in range(m.njnt):
            if int(m.jnt_type[i]) in (int(mj.mjtJoint.mjJNT_FREE), int(mj.mjtJoint.mjJNT_BALL)):
                continue
            name = mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, i)
            if name:
                self.joints.append((name, int(m.jnt_qposadr[i]), int(m.jnt_dofadr[i])))
        self.free_adr = None
        for i in range(m.njnt):
            if int(m.jnt_type[i]) == int(mj.mjtJoint.mjJNT_FREE):
                self.free_adr = (int(m.jnt_qposadr[i]), int(m.jnt_dofadr[i]))
                break
        self.root_body = next((b for b in range(1, m.nbody) if int(m.body_parentid[b]) == 0), 1)
        self.act_of_joint = dict(self.sim._actuator_by_joint)
        self.body_names = [mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, b) or "" for b in range(m.nbody)]
        # Ground = geoms on the world body (floor plane, heightfield, stairs).
        self.ground_geoms = {g for g in range(m.ngeom) if int(m.geom_bodyid[g]) == 0}

    def set_script(self, fn: Optional[Callable]) -> None:
        self.fn = fn
        self._next_control_t = float(self.sim.data.time)

    def reset(self) -> None:
        self._next_control_t = 0.0

    def ground_contacts(self) -> List[str]:
        d, m = self.sim.data, self.sim.model
        out = set()
        for i in range(int(d.ncon)):
            c = d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if g1 in self.ground_geoms and g2 not in self.ground_geoms:
                out.add(self.body_names[int(m.geom_bodyid[g2])])
            elif g2 in self.ground_geoms and g1 not in self.ground_geoms:
                out.add(self.body_names[int(m.geom_bodyid[g1])])
        return sorted(out)

    def base_state(self) -> Dict[str, Any]:
        d = self.sim.data
        b = self.root_body
        pos = [float(v) for v in d.xpos[b]]
        w, x, y, z = (float(v) for v in d.xquat[b])
        rpy = _quat_to_rpy(w, x, y, z)
        if self.free_adr is not None:
            _, dadr = self.free_adr
            v_world = np.array(d.qvel[dadr:dadr + 3], dtype=float)
            w_body = [float(v) for v in d.qvel[dadr + 3:dadr + 6]]
        else:
            v_world = np.zeros(3)
            w_body = [0.0, 0.0, 0.0]
        cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
        lin = [float(cy * v_world[0] + sy * v_world[1]), float(-sy * v_world[0] + cy * v_world[1]), float(v_world[2])]
        return {"pos": pos, "rpy": rpy, "lin_vel": lin, "ang_vel": w_body}

    def controller_state(self) -> Dict[str, Any]:
        d = self.sim.data
        q, qd, eff = {}, {}, {}
        for name, qa, da in self.joints:
            q[name] = float(d.qpos[qa])
            qd[name] = float(d.qvel[da])
            a = self.act_of_joint.get(name)
            eff[name] = float(d.actuator_force[a]) if a is not None else 0.0
        return {
            "t": float(d.time), "dt": self.control_dt,
            "q": q, "qd": qd, "effort": eff,
            "base": self.base_state(),
            "contacts": self.ground_contacts(),
            "cmd": dict(self.cmd),
        }

    def control_tick(self) -> Optional[Dict[str, float]]:
        """Run the script once and apply its commands. Raises on script errors."""
        if self.fn is None:
            return None
        st = self.controller_state()
        out = self.fn(st["t"], st)
        if not isinstance(out, dict):
            raise ValueError(f"step() must return a dict, got {type(out).__name__}")
        ctrl = self.sim.data.ctrl
        m = self.sim.model
        for k, v in out.items():
            a = self.act_of_joint.get(k)
            if a is None:
                continue
            v = float(v)
            if not math.isfinite(v):
                raise ValueError(f"step() returned a non-finite command for {k!r}")
            if m.actuator_ctrllimited[a]:
                lo, hi = m.actuator_ctrlrange[a]
                v = min(max(v, float(lo)), float(hi))
            ctrl[a] = v
        return out

    def advance(self, n_steps: int) -> None:
        """Advance physics n_steps, running the controller every control_dt of sim time."""
        mj, m, d = self.sim.mujoco, self.sim.model, self.sim.data
        for _ in range(n_steps):
            if self.fn is not None and d.time + 1e-9 >= self._next_control_t:
                self.control_tick()
                self._next_control_t = d.time + self.control_dt
            mj.mj_step(m, d)
        if not np.isfinite(d.qpos).all():
            raise RuntimeError("Simulation diverged (non-finite state) — check link inertias/masses "
                               "or command magnitudes.")


# ── Loading helpers ──────────────────────────────────────────────────────────

def load_sim_from_text(urdf_text: str, free_base: bool = True,
                       terrain_config: Optional[Dict[str, Any]] = None,
                       base_dir: Optional[str] = None) -> MuJoCoSimulator:
    sim = MuJoCoSimulator()
    fd, path = tempfile.mkstemp(suffix=".urdf", dir=base_dir if base_dir and os.path.isdir(base_dir) else None)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(urdf_text)
        sim.load_urdf(path, free_base=free_base, terrain_config=terrain_config)
    finally:
        try:
            os.remove(path)
        except OSError:
            pass
    return sim


# ── Robot brief: what each joint actually does, measured ─────────────────────

def _subtree_bodies(m, root: int) -> List[int]:
    out = [root]
    for b in range(root + 1, m.nbody):
        p = int(m.body_parentid[b])
        if p in out:
            out.append(b)
    return out


def _lowest_point(sim: MuJoCoSimulator, bodies: List[int]) -> np.ndarray:
    m, d = sim.model, sim.data
    best, best_z = None, float("inf")
    for g in range(m.ngeom):
        if int(m.geom_bodyid[g]) not in bodies:
            continue
        z = sim._geom_min_z(g)
        if z < best_z:
            best_z = z
            best = np.array([float(d.geom_xpos[g][0]), float(d.geom_xpos[g][1]), z])
    return best if best is not None else np.array(d.xpos[bodies[-1]], dtype=float)


def _tip_body(m, bodies: List[int]) -> int:
    """Deepest body of a subtree (the end of its longest chain)."""
    depth = {bodies[0]: 0}
    for b in bodies[1:]:
        depth[b] = depth[int(m.body_parentid[b])] + 1
    return max(bodies, key=lambda b: (depth[b], b))


def robot_brief_data(urdf_text: str, free_base: bool = False) -> Dict[str, Any]:
    """Measure the robot in its zero pose: what touches the ground, and what each
    joint's positive direction does to the part it moves."""
    sim = load_sim_from_text(urdf_text, free_base=free_base)
    mj, m, d = sim.mujoco, sim.model, sim.data
    mj.mj_forward(m, d)
    names = [mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, b) or "" for b in range(m.nbody)]
    root = next((b for b in range(1, m.nbody) if int(m.body_parentid[b]) == 0), 1)
    # Only geoms that can touch the terrain count (visual-only envelopes don't).
    solid = [g for g in range(m.ngeom) if int(m.geom_bodyid[g]) != 0
             and (int(m.geom_contype[g]) or int(m.geom_conaffinity[g]))]
    zmin_all = min(sim._geom_min_z(g) for g in solid)
    zmax_all = max(float(d.geom_xpos[g][2]) + float(m.geom_rbound[g]) for g in range(m.ngeom)
                   if int(m.geom_bodyid[g]) != 0)
    height = max(zmax_all - zmin_all, 1e-3)
    ground_tol = max(0.01, 0.03 * height)

    def touches(bodies):
        return any(sim._geom_min_z(g) - zmin_all < ground_tol for g in solid
                   if int(m.geom_bodyid[g]) in bodies)

    joints = []
    for j in range(m.njnt):
        jt = int(m.jnt_type[j])
        if jt in (int(mj.mjtJoint.mjJNT_FREE), int(mj.mjtJoint.mjJNT_BALL)):
            continue
        name = mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, j) or ""
        if "_idler" in name:
            continue   # track rollers ganged to their drive joint
        child = int(m.jnt_bodyid[j])
        sub = _subtree_bodies(m, child)
        qa = int(m.jnt_qposadr[j])
        a = sim._actuator_by_joint.get(name)
        kind = "prismatic" if jt == int(mj.mjtJoint.mjJNT_SLIDE) else "revolute"
        act_name = mj.mj_id2name(m, mj.mjtObj.mjOBJ_ACTUATOR, a) if a is not None else ""
        if act_name.endswith("_vel"):
            kind = "continuous"
        limited = bool(m.jnt_limited[j])
        lo, hi = (float(m.jnt_range[j][0]), float(m.jnt_range[j][1])) if limited else (None, None)
        effort = float(m.actuator_forcerange[a][1]) if a is not None else 0.0
        vmax = float(m.actuator_ctrlrange[a][1]) if (a is not None and kind == "continuous") else None
        axis_w = d.xmat[child].reshape(3, 3) @ np.array(m.jnt_axis[j], dtype=float)
        pivot = np.array(d.xanchor[j], dtype=float)
        grounded = touches(sub)
        tip = _tip_body(m, sub)
        if a is None:
            kind = "passive"
        info = {
            "name": name, "type": kind, "parent": names[int(m.body_parentid[child])],
            "child": names[child], "moves": [names[b] for b in sub],
            "tip": names[tip], "limits": [lo, hi] if limited else None,
            "effort": effort, "max_velocity": vmax,
            "pivot": pivot.tolist(), "axis_world": axis_w.tolist(),
            "subtree_touches_ground": grounded,
        }
        if kind == "continuous":
            # A wheel: which way does +velocity roll the robot? The contact point
            # is below the wheel centre; the ground pushes the robot opposite to
            # the contact point's velocity.
            centre = np.array(d.xpos[child], dtype=float)
            contact = centre.copy()
            contact[2] = _lowest_point(sim, [child])[2]
            v_contact = np.cross(axis_w, contact - centre)
            drive = -v_contact
            info["is_wheel"] = grounded
            if np.linalg.norm(drive[:2]) > 1e-9:
                drive2 = drive[:2] / np.linalg.norm(drive[:2])
                info["plus_drives_robot"] = [float(drive2[0]), float(drive2[1])]
                info["wheel_radius"] = float(max(centre[2] - contact[2], 1e-3))
                info["side"] = "left" if centre[1] > 0.005 else ("right" if centre[1] < -0.005 else "center")
                info["pos"] = centre.tolist()
        else:
            # Measure the effect of a small positive move on the subtree tip and
            # (for ground-touching chains) on its lowest point = the foot.
            delta = 0.1 if kind == "revolute" else 0.01
            if limited:
                delta = min(delta, max(1e-3, (hi - lo) * 0.25))
            q0 = float(d.qpos[qa])
            probe = _lowest_point(sim, sub) if grounded else np.array(d.xpos[tip], dtype=float)
            d.qpos[qa] = q0 + delta
            mj.mj_kinematics(m, d)
            probe2 = _lowest_point(sim, sub) if grounded else np.array(d.xpos[tip], dtype=float)
            d.qpos[qa] = q0
            mj.mj_kinematics(m, d)
            dp = (probe2 - probe) / delta
            info["probe"] = "foot (lowest point)" if grounded else f"tip ({names[tip]})"
            info["probe_pos"] = probe.tolist()
            info["plus_moves_probe"] = dp.tolist()     # m per rad (or m per m)
        joints.append(info)

    wheels = [j for j in joints if j.get("is_wheel")]
    legs = [j for j in joints if j["type"] in ("revolute", "prismatic") and j["subtree_touches_ground"]]
    # Leg groups: one per ground-contact chain rooted directly on the base
    # side of the tree (the outermost joint whose subtree touches the ground).
    leg_roots = []
    for j in legs:
        if not any(j["child"] in other["moves"] and other is not j for other in legs):
            leg_roots.append(j)
    groups = []
    for r in leg_roots:
        members = [j["name"] for j in legs if j["child"] in r["moves"]]
        foot = r["probe_pos"]
        groups.append({"root_joint": r["name"], "joints": members, "foot": foot})
    mass = float(np.sum(m.body_mass[1:]))
    info = {
        "robot_mass_kg": mass,
        "height_m": height,
        "base_link": names[root],
        "joints": joints,
        "wheels": [j["name"] for j in wheels],
        "leg_groups": groups,
        "mobile": bool(wheels or len(groups) >= 2),
        "ground_links": sorted({names[int(m.geom_bodyid[g])] for g in solid
                                if sim._geom_min_z(g) - zmin_all < ground_tol}),
    }
    return info


def _dir_words(v: List[float], scale: float, unit: str) -> str:
    """'+12mm forward, -3mm down' for a 3-vector (m) scaled into mm per unit."""
    words = []
    for k, (pos, neg) in enumerate((("forward", "back"), ("left", "right"), ("up", "down"))):
        mm = v[k] * scale * 1000.0
        if abs(mm) >= 0.5:
            words.append(f"{abs(mm):.0f}mm {pos if mm > 0 else neg}")
    return ", ".join(words) if words else "barely moves"


def format_robot_brief(brief: Dict[str, Any]) -> str:
    L = [f"ROBOT (measured in the zero pose; world frame +X forward, +Y left, +Z up):",
         f"  mass {brief['robot_mass_kg']:.2f} kg, height {brief['height_m'] * 1000:.0f} mm, base link {brief['base_link']}",
         f"  touching the ground at rest: {', '.join(brief['ground_links']) or 'nothing'}"]
    if brief["wheels"]:
        L.append("  wheeled: yes")
    if brief["leg_groups"]:
        L.append(f"  legs: {len(brief['leg_groups'])}")
        for g in brief["leg_groups"]:
            f = g["foot"]
            L.append(f"    leg at foot ({f[0] * 1000:+.0f}, {f[1] * 1000:+.0f}) mm: joints {', '.join(g['joints'])}")
    L.append("")
    L.append("JOINTS (what a positive command does, measured):")
    for j in brief["joints"]:
        head = f"- {j['name']} [{j['type']}] moves {j['child']}"
        if len(j["moves"]) > 1:
            head += f" (+{len(j['moves']) - 1} parts below it)"
        if j["type"] == "passive":
            L.append(head + "; PASSIVE (no motor, cannot be commanded — e.g. a rocker/bogie pivot)")
            continue
        head += f"; effort {j['effort']:.2f}"
        if j["type"] == "continuous":
            head += f"; command = wheel speed rad/s, max {j['max_velocity']:.1f}"
            if j.get("is_wheel") and "plus_drives_robot" in j:
                dx, dy = j["plus_drives_robot"]
                where = "forward" if dx > 0.7 else ("backward" if dx < -0.7 else f"direction ({dx:+.2f},{dy:+.2f})")
                head += (f"; ground wheel on the {j['side']} side, r={j['wheel_radius'] * 1000:.0f}mm; "
                         f"positive speed drives the robot {where} "
                         f"(forward_sign {'+1' if dx > 0 else '-1'})")
            elif not j.get("is_wheel"):
                head += "; not touching the ground (spinner/roller)"
        else:
            if j["limits"]:
                lo, hi = j["limits"]
                u = "rad" if j["type"] == "revolute" else "m"
                head += f"; limits [{lo:+.2f}, {hi:+.2f}] {u}"
            unit = 0.1 if j["type"] == "revolute" else 0.01
            head += (f"; +{unit:g}{'rad' if j['type'] == 'revolute' else 'm'} moves the {j['probe']} "
                     f"{_dir_words(j['plus_moves_probe'], unit, '')}")
        L.append(head)
    return "\n".join(L)


# ── Headless evaluation ─────────────────────────────────────────────────────

def evaluate_controller(urdf_text: str, code: str, seconds: float = 6.0,
                        terrain_config: Optional[Dict[str, Any]] = None,
                        free_base: Optional[bool] = None,
                        cmd: Optional[Dict[str, float]] = None) -> Dict[str, Any]:
    """Run a controller headless and measure what the robot actually did."""
    try:
        fn = compile_script(code)
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "error": f"script rejected: {e}"}
    brief = robot_brief_data(urdf_text, free_base=False)
    if free_base is None:
        free_base = bool(brief["mobile"])
    sim = load_sim_from_text(urdf_text, free_base=free_base, terrain_config=terrain_config)
    rt = ControllerRuntime(sim)
    if cmd:
        rt.cmd.update(cmd)
    rt.set_script(fn)
    m, d = sim.model, sim.data
    base0 = rt.base_state()
    h0 = base0["pos"][2]
    names = [j[0] for j in rt.joints]
    n_ctrl = 0
    track_err = {n: 0.0 for n in names}
    sat = {n: 0 for n in names}
    cmd_range = {n: [float("inf"), float("-inf")] for n in names}
    q_range = {n: [float("inf"), float("-inf")] for n in names}
    contact_time: Dict[str, int] = {}
    max_tilt, fell_at, min_h = 0.0, None, h0
    unknown = set()
    error = None
    steps_per_ctrl = max(1, int(round(rt.control_dt / m.opt.timestep)))
    total_ctrl = int(seconds / rt.control_dt)
    samples = []
    # Speed-controlled joints (wheels, tracks) are judged on speed, not angle.
    speed_ctl = {n for n, a in rt.act_of_joint.items()
                 if (sim.mujoco.mj_id2name(m, sim.mujoco.mjtObj.mjOBJ_ACTUATOR, a) or "").endswith("_vel")}
    try:
        for k in range(total_ctrl):
            out = rt.control_tick() or {}
            for key in out:
                if key not in rt.act_of_joint:
                    unknown.add(str(key))
            for _ in range(steps_per_ctrl):
                sim.mujoco.mj_step(m, d)
            if not np.isfinite(d.qpos).all():
                raise RuntimeError("simulation diverged (non-finite state)")
            n_ctrl += 1
            st = rt.controller_state()
            for n, qa, _ in rt.joints:
                a = rt.act_of_joint.get(n)
                if a is None:
                    continue
                q = st["qd"][n] if n in speed_ctl else st["q"][n]
                q_range[n] = [min(q_range[n][0], q), max(q_range[n][1], q)]
                c = float(d.ctrl[a])
                cmd_range[n] = [min(cmd_range[n][0], c), max(cmd_range[n][1], c)]
                lim = float(m.actuator_forcerange[a][1]) if m.actuator_forcelimited[a] else 0.0
                if lim > 0 and abs(st["effort"][n]) >= 0.97 * lim:
                    sat[n] += 1
                track_err[n] += abs(c - q)
            for link in st["contacts"]:
                contact_time[link] = contact_time.get(link, 0) + 1
            r, p, _ = st["base"]["rpy"]
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(r) * math.cos(p)))))
            max_tilt = max(max_tilt, tilt)
            min_h = min(min_h, st["base"]["pos"][2])
            if fell_at is None and free_base and (tilt > 60 or st["base"]["pos"][2] < 0.35 * h0):
                fell_at = st["t"]
            if k % int(0.5 / rt.control_dt) == 0:
                samples.append((round(st["t"], 2), [round(v, 3) for v in st["base"]["pos"]],
                                round(math.degrees(st["base"]["rpy"][2]), 1)))
    except Exception as e:  # noqa: BLE001
        error = f"{type(e).__name__}: {e}"
    base1 = rt.base_state()
    t_run = float(d.time)
    yaw0 = base0["rpy"][2]
    dx, dy = base1["pos"][0] - base0["pos"][0], base1["pos"][1] - base0["pos"][1]
    fwd = math.cos(yaw0) * dx + math.sin(yaw0) * dy
    lat = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
    dyaw = math.degrees(math.atan2(math.sin(base1["rpy"][2] - yaw0), math.cos(base1["rpy"][2] - yaw0)))
    n = max(n_ctrl, 1)
    joints_rep = []
    for nm in names:
        if cmd_range[nm][0] == float("inf"):
            continue
        joints_rep.append({
            "joint": nm,
            "cmd_range": [round(v, 3) for v in cmd_range[nm]],
            "actual_range": [round(v, 3) for v in q_range[nm]],
            "mean_tracking_error": round(track_err[nm] / n, 3),
            "saturated_pct": round(100.0 * sat[nm] / n),
            "speed": nm in speed_ctl,
        })
    ground = set(brief["ground_links"])
    unexpected = {k: round(100.0 * v / n) for k, v in contact_time.items() if k not in ground and v / n > 0.05}
    return {
        "ok": error is None,
        "error": error,
        "free_base": free_base,
        "sim_seconds": round(t_run, 2),
        "forward_m": round(fwd, 3), "lateral_m": round(lat, 3),
        "avg_forward_speed_mps": round(fwd / max(t_run, 1e-6), 3),
        "yaw_change_deg": round(dyaw, 1),
        "max_tilt_deg": round(max_tilt, 1),
        "base_height_start_m": round(h0, 3), "base_height_min_m": round(min_h, 3),
        "base_height_end_m": round(base1["pos"][2], 3),
        "fell_at_s": fell_at,
        "unknown_joint_names": sorted(unknown),
        "ground_contact_pct": {k: round(100.0 * v / n) for k, v in sorted(contact_time.items())},
        "unexpected_ground_contact_pct": unexpected,
        "joints": joints_rep,
        "trajectory": samples,
    }


def format_evaluation(ev: Dict[str, Any]) -> str:
    if not ev.get("ok") and ev.get("error", "").startswith("script rejected"):
        return ev["error"]
    L = []
    if ev.get("error"):
        L.append(f"RUNTIME ERROR after {ev['sim_seconds']}s: {ev['error']}")
    L.append(f"Ran {ev['sim_seconds']}s ({'free base' if ev['free_base'] else 'fixed base'}).")
    if ev["free_base"]:
        L.append(f"Base moved {ev['forward_m'] * 100:+.1f} cm forward, {ev['lateral_m'] * 100:+.1f} cm left "
                 f"(avg {ev['avg_forward_speed_mps'] * 100:+.1f} cm/s); heading changed {ev['yaw_change_deg']:+.1f} deg.")
        L.append(f"Max tilt {ev['max_tilt_deg']:.0f} deg; base height {ev['base_height_start_m'] * 100:.1f} cm "
                 f"-> min {ev['base_height_min_m'] * 100:.1f} cm -> end {ev['base_height_end_m'] * 100:.1f} cm.")
        if ev["fell_at_s"] is not None:
            L.append(f"FELL OVER at t={ev['fell_at_s']:.2f}s.")
        L.append("Ground contact (% of time): " + ", ".join(f"{k} {v}%" for k, v in ev["ground_contact_pct"].items()))
        if ev["unexpected_ground_contact_pct"]:
            L.append("Parts dragging on the ground that don't touch it at rest: "
                     + ", ".join(f"{k} {v}%" for k, v in ev["unexpected_ground_contact_pct"].items()))
        L.append("Trajectory (t, base xyz m, yaw deg): " + "; ".join(
            f"{t}s {p} {y}" for t, p, y in ev["trajectory"][::2]))
    if ev["unknown_joint_names"]:
        L.append(f"Commands for unknown joints were ignored: {', '.join(ev['unknown_joint_names'])}")
    L.append("Joints (commanded range -> actual range, mean tracking error, % time at torque limit; "
             "wheels/tracks in rad/s):")
    for j in ev["joints"]:
        flag = "  <-- SATURATED" if j["saturated_pct"] >= 25 else ""
        L.append(f"  {j['joint']}{' (speed)' if j.get('speed') else ''}: cmd {j['cmd_range']} -> actual {j['actual_range']}, "
                 f"err {j['mean_tracking_error']}, sat {j['saturated_pct']}%{flag}")
    return "\n".join(L)

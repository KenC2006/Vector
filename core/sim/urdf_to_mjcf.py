"""
URDF to MJCF converter.

Converts a URDF file to MJCF XML format with actuators.
MuJoCo can load MJCF directly, and we inject actuators for all non-fixed joints.
"""
from typing import Dict, List, Tuple, Optional, Any
import os
from lxml import etree
import numpy as np

from core.presets import actuator_rating, resolve_component_bounds_m
from core.presets.identity import PartIdentity

# Resolve package:// URIs emitted by the frontend URDF builder.
# This file lives at <repo_root>/core/sim/urdf_to_mjcf.py, so the repo root
# is two directories up. The frontend maps package:// to src/public/.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PACKAGE_URL_PREFIX = "package://"
_PACKAGE_MESH_ROOTS = [
    os.path.join(_REPO_ROOT, "src", "public"),
]

# Collision bitmask layout — two geoms collide iff (a.contype & b.conaffinity) | (b.contype & a.conaffinity) != 0
CT_WORLD      = 1  # terrain / static environment
CT_ROBOT_ROLE = 2  # foot / wheel / gripper
CT_STRUCTURAL = 4  # chassis, links, sensors, brackets


def _resolve_mesh_filename(filename: str, urdf_dir: str) -> str:
    """
    Resolve a URDF mesh filename to an absolute filesystem path.

    Handles three cases:
      1. package://meshes/... — frontend-emitted URI, resolved under src/public/
      2. Absolute path — returned as-is.
      3. Relative path — resolved against the URDF file's directory.

    Returns an empty string if the path resolves outside the expected mesh root
    or if filename is empty.
    """
    if not filename:
        return ""
    if filename.startswith(_PACKAGE_URL_PREFIX):
        rel = filename[len(_PACKAGE_URL_PREFIX):]
        for root in _PACKAGE_MESH_ROOTS:
            candidate = os.path.normpath(os.path.join(root, rel))
            if candidate.startswith(os.path.normpath(root) + os.sep) or \
               candidate == os.path.normpath(root):
                return candidate
        return ""
    if os.path.isabs(filename):
        return filename
    return os.path.join(urdf_dir, filename)


def _load_urdf_xml(urdf_path: str) -> etree._Element:
    """Load URDF XML file and return root element."""
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(urdf_path, parser)
    return tree.getroot()


def _extract_link_data(link_elem: etree._Element, urdf_dir: str) -> Dict[str, Any]:
    """Extract relevant data from a URDF link element."""
    name = link_elem.get("name", "")

    mass = 0.0
    explicit_inertia: Optional[List[float]] = None  # [ixx, iyy, izz, ixy, ixz, iyz]

    inertial_elem = link_elem.find(".//inertial")
    if inertial_elem is not None:
        mass_elem = inertial_elem.find("mass")
        if mass_elem is not None:
            try:
                mass = float(mass_elem.get("value", 0.0))
            except (ValueError, TypeError):
                mass = 0.0

        # Use explicit URDF inertia tensor if provided and non-trivial
        inertia_sub = inertial_elem.find("inertia")
        if inertia_sub is not None:
            try:
                ixx = float(inertia_sub.get("ixx", 0))
                iyy = float(inertia_sub.get("iyy", 0))
                izz = float(inertia_sub.get("izz", 0))
                ixy = float(inertia_sub.get("ixy", 0))
                ixz = float(inertia_sub.get("ixz", 0))
                iyz = float(inertia_sub.get("iyz", 0))
                if ixx > 1e-12 and iyy > 1e-12 and izz > 1e-12:
                    explicit_inertia = [ixx, iyy, izz, ixy, ixz, iyz]
            except (ValueError, TypeError):
                pass

    # Collect ALL collision elements with their origins (Phase B: multi-primitive support)
    collision_list: List[Dict[str, Any]] = []
    for coll_elem in link_elem.findall(".//collision"):
        geom_el = coll_elem.find("geometry")
        if geom_el is None:
            continue
        geom = _extract_geometry(geom_el, urdf_dir)
        if geom is None:
            continue
        origin_el = coll_elem.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy_vals = [0.0, 0.0, 0.0]
        if origin_el is not None:
            try:
                xyz = [float(v) for v in origin_el.get("xyz", "0 0 0").split()]
            except (ValueError, TypeError):
                pass
            try:
                rpy_vals = [float(v) for v in origin_el.get("rpy", "0 0 0").split()]
            except (ValueError, TypeError):
                pass
        collision_list.append({"geometry": geom, "origin_xyz": xyz, "origin_rpy": rpy_vals})

    # Collect visual meshes for trimesh inertia estimation. Only meshes with a
    # zero <origin> are eligible — otherwise inertia about the mesh COM cannot be
    # written at the link origin without a parallel-axis shift, which would
    # silently produce a wrong tensor. We also capture the scale so volume-based
    # inertia is computed against the rendered geometry, not the unit-scale file.
    visual_mesh_files: List[Dict[str, Any]] = []
    for vis_elem in link_elem.findall(".//visual"):
        geom_el = vis_elem.find("geometry/mesh")
        if geom_el is None:
            continue
        fname = _resolve_mesh_filename(geom_el.get("filename", ""), urdf_dir)
        if not (fname and os.path.exists(fname)):
            continue
        origin_xyz = [0.0, 0.0, 0.0]
        origin_rpy = [0.0, 0.0, 0.0]
        origin_el = vis_elem.find("origin")
        if origin_el is not None:
            try:
                origin_xyz = [float(v) for v in origin_el.get("xyz", "0 0 0").split()]
            except (ValueError, TypeError):
                pass
            try:
                origin_rpy = [float(v) for v in origin_el.get("rpy", "0 0 0").split()]
            except (ValueError, TypeError):
                pass
        scale = [1.0, 1.0, 1.0]
        scale_str = geom_el.get("scale")
        if scale_str:
            try:
                parts = [float(v) for v in scale_str.split()]
                if len(parts) == 3:
                    scale = parts
            except (ValueError, TypeError):
                pass
        visual_mesh_files.append({
            "file": fname,
            "origin_xyz": origin_xyz,
            "origin_rpy": origin_rpy,
            "scale": scale,
        })

    # Fall back to visual geometry if no collision elements
    if not collision_list:
        visual_elem = link_elem.find(".//visual/geometry")
        if visual_elem is not None:
            geom = _extract_geometry(visual_elem, urdf_dir)
            if geom is not None:
                collision_list.append({
                    "geometry": geom,
                    "origin_xyz": [0.0, 0.0, 0.0],
                    "origin_rpy": [0.0, 0.0, 0.0],
                })

    return {
        "name": name,
        "mass": mass,
        "explicit_inertia": explicit_inertia,
        "collision_list": collision_list,
        "visual_mesh_files": visual_mesh_files,
    }


def _extract_geometry(geom_elem: etree._Element, urdf_dir: str) -> Optional[Dict[str, Any]]:
    """Extract geometry from a URDF geometry element."""

    # Box
    box_elem = geom_elem.find("box")
    if box_elem is not None:
        size_str = box_elem.get("size", "1 1 1")
        try:
            size = [float(x) for x in size_str.split()]
            return {
                "type": "box",
                "size": size,
            }
        except (ValueError, TypeError):
            pass

    # Cylinder
    cyl_elem = geom_elem.find("cylinder")
    if cyl_elem is not None:
        try:
            radius = float(cyl_elem.get("radius", 0.1))
            length = float(cyl_elem.get("length", 1.0))
            return {
                "type": "cylinder",
                "radius": radius,
                "length": length,
            }
        except (ValueError, TypeError):
            pass

    # Sphere
    sphere_elem = geom_elem.find("sphere")
    if sphere_elem is not None:
        try:
            radius = float(sphere_elem.get("radius", 0.1))
            return {
                "type": "sphere",
                "radius": radius,
            }
        except (ValueError, TypeError):
            pass

    # Mesh
    mesh_elem = geom_elem.find("mesh")
    if mesh_elem is not None:
        filename = _resolve_mesh_filename(mesh_elem.get("filename", ""), urdf_dir)
        # Parse optional <mesh scale="sx sy sz"> — URDF default is 1 1 1.
        scale = [1.0, 1.0, 1.0]
        scale_str = mesh_elem.get("scale")
        if scale_str:
            try:
                parts = [float(v) for v in scale_str.split()]
                if len(parts) == 3:
                    scale = parts
            except (ValueError, TypeError):
                pass
        if filename and os.path.exists(filename):
            # Generate stable asset name: strip extension, sanitize. Include scale
            # in the name so different scales of the same file get distinct meshes.
            raw_name = os.path.splitext(os.path.basename(filename))[0]
            base_mesh_name = "".join(c if c.isalnum() or c == "_" else "_" for c in raw_name)
            if scale != [1.0, 1.0, 1.0]:
                scale_tag = "_s" + "_".join(f"{s:g}".replace(".", "p").replace("-", "n") for s in scale)
                mesh_name = base_mesh_name + scale_tag
            else:
                mesh_name = base_mesh_name
            return {
                "type": "mesh",
                "filename": filename,
                "mesh_name": mesh_name,
                "scale": scale,
            }

    return None


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> List[float]:
    """Convert URDF roll-pitch-yaw to quaternion [w, x, y, z].

    URDF convention: fixed-axis X, then Y, then Z — R = Rz(yaw)·Ry(pitch)·Rx(roll),
    the same matrix as `_rotation_matrix_rpy` below and the frontend's
    rotationIO (three.js Euler order 'ZYX').
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [w, x, y, z]


# ── Contact / material helpers ────────────────────────────────────────────────

# Contact role -> MJCF default class. The role comes from the catalog
# (sim_metadata.contact_class via PartIdentity), never from link names:
# 'sole' used to match 'console' and a vacuum pad array got foot friction.
_CONTACT_GEOM_CLASS = {
    "foot": "foot",
    "wheel": "wheel",
    "caster": "caster",
    "gripper": "gripper",
    "suction": "suction",
}

# Contact classes: (sliding, torsional, rolling) friction on nominal ground,
# condim, solimp. Robot geoms carry priority 1 so MuJoCo uses THEIR friction
# for robot/terrain contacts (with equal priorities it takes the element-wise
# max, and the grippy default terrain overrode every class). The terrain
# slider scales the sliding coefficient; see _friction_attr.
_CONTACT_CLASS_PARAMS = {
    #            sliding torsion rolling   condim  solimp
    "default": ((1.0, 0.05, 0.001), 4, "0.9 0.95 0.001"),
    "foot":    ((3.0, 0.3, 0.03), 6, "0.95 0.99 0.001"),     # rubber pads: no pirouette
    "wheel":   ((1.2, 0.002, 0.0001), 6, "0.9 0.95 0.001"),  # grips laterally, rolls freely
    "caster":  ((0.05, 0.001, 0.0001), 3, "0.9 0.95 0.001"), # ball transfer: slides any way
    "gripper": ((2.0, 0.2, 0.02), 6, "0.9 0.95 0.001"),      # secure grasps
    "suction": ((1.0, 0.05, 0.005), 6, "0.9 0.95 0.001"),    # rubber cup lip
}

# Terrain friction the class coefficients are tuned for (the UI default).
NOMINAL_TERRAIN_FRICTION = 3.0

_EE_KEYWORDS_MJCF = frozenset(("ee", "end_effector", "end-effector", "tool", "tcp"))
_IMU_KEYWORDS     = frozenset(("imu",))

import re as _re
_TOKEN_SPLIT = _re.compile(r"[_\-\s]+")


def _name_tokens(name: str) -> set:
    return {t for t in _TOKEN_SPLIT.split(name.lower()) if t}


def _is_imu_link(link_name: str, ident: PartIdentity) -> bool:
    """An IMU: a catalog part with an accelerometer sensor type; links with no
    catalog identity (hand-written URDFs) fall back to an `imu` name token."""
    comp = ident.component(link_name)
    if comp is not None:
        return "accelerometer" in str((comp.get("sim_metadata") or {}).get("mjcf_sensor_type", ""))
    return bool(_name_tokens(link_name) & _IMU_KEYWORDS)


def _is_ee_link(link_name: str, ident: PartIdentity) -> bool:
    """An end effector: a catalog part from the end_effectors category; links
    with no catalog identity fall back to ee/tool/tcp name tokens."""
    comp = ident.component(link_name)
    if comp is not None:
        return comp["id"] in _END_EFFECTOR_IDS
    tokens = _name_tokens(link_name)
    return any(kw in tokens for kw in _EE_KEYWORDS_MJCF)


def _end_effector_ids() -> frozenset:
    try:
        from core.presets import get_category
        return frozenset(c["id"] for c in get_category("end_effectors")["components"])
    except Exception:
        return frozenset()


_END_EFFECTOR_IDS = _end_effector_ids()


def _friction_attr(contact_class: str, terrain_friction: float) -> str:
    """Friction triple for a robot geom class on terrain of the given grip."""
    (slide, torsion, roll), _, _ = _CONTACT_CLASS_PARAMS[contact_class]
    scale = terrain_friction / NOMINAL_TERRAIN_FRICTION
    return f"{slide * scale:.6g} {torsion:.6g} {roll:.6g}"


def _sim_wheel_cylinder_collision(component: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """
    Return the intended MuJoCo wheel cylinder collision for preset-backed wheels.

    The frontend can emit authored OBJ collision meshes for visual inspection,
    but rolling contact is much more stable as a primitive cylinder.  Presets
    opt into that with sim_metadata.mjcf_geom_type="cylinder".
    """
    if not component:
        return None
    if (component.get("sim_metadata") or {}).get("mjcf_geom_type") != "cylinder":
        return None
    # Route through the resolver instead of reading the raw preset bbox —
    # keeps wheel cylinder dims consistent with the placement envelope.
    try:
        x_m, y_m, z_m = resolve_component_bounds_m(component)
    except (TypeError, ValueError):
        return None
    radius = max(x_m, y_m) / 2.0
    length = z_m
    if radius <= 0 or length <= 0:
        return None
    return {
        "geometry": {
            "type": "cylinder",
            "radius": radius,
            "length": length,
        },
        "origin_xyz": [0.0, 0.0, 0.0],
        "origin_rpy": [0.0, 0.0, 0.0],
    }


# ── Inertia helpers ───────────────────────────────────────────────────────────

# Default density for robot parts with unknown material (kg/m³).
_PLASTIC_DENSITY = 1200.0

# Named material densities (kg/m³) that the UI can pass to override the default.
MATERIAL_DENSITIES: Dict[str, float] = {
    "pla":      1240.0,
    "abs":      1050.0,
    "petg":     1270.0,
    "aluminum": 2700.0,
    "aluminium": 2700.0,
    "steel":    7850.0,
    "carbon":   1600.0,  # CFRP laminate typical
    "titanium": 4500.0,
}


def _trimesh_inertia(
    mesh_file: str,
    mass: float,
    scale: Optional[List[float]] = None,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    Compute (inertia_tensor, com_position) from a mesh file using trimesh.

    Both are expressed in the mesh's local coordinate frame.  The inertia is
    taken about the mesh COM (not the mesh origin) — callers MUST pass `com`
    through to MJCF as the body's `inertial pos`, otherwise MuJoCo will treat
    the tensor as being about the link origin and the resulting spatial inertia
    will not be physically valid (non-PD when COM is offset).

    Returns None if trimesh is not installed or the mesh is degenerate.
    """
    try:
        import trimesh
    except ImportError:
        return None
    try:
        mesh = trimesh.load(mesh_file, force="mesh", process=False)
        if not isinstance(mesh, trimesh.Trimesh) or mesh.volume <= 0:
            return None
        if scale is not None and scale != [1.0, 1.0, 1.0]:
            mesh = mesh.copy()
            mesh.apply_scale(scale)
            if mesh.volume <= 0:
                return None
        # Scale inertia to the target mass (trimesh assumes density=1).
        density_scale = mass / (mesh.volume * 1.0)
        I = np.array(mesh.moment_inertia) * density_scale
        com = np.array(mesh.center_mass, dtype=float)
        # Reject anything non-finite or non-PD — degenerate meshes can sneak
        # through volume>0 and still produce a near-singular tensor that blows
        # up the composite mass matrix.
        if not np.all(np.isfinite(I)) or not np.all(np.isfinite(com)):
            return None
        eigvals = np.linalg.eigvalsh((I + I.T) * 0.5)
        if eigvals[0] <= 1e-12:
            return None
        return I, com
    except Exception:
        return None


def _primitive_volume(geom: Dict[str, Any]) -> float:
    """Return the volume of a collision primitive in m³."""
    g_type = geom.get("type", "box")
    if g_type == "box":
        s = geom.get("size", [0.1, 0.1, 0.1])
        return float(s[0]) * float(s[1]) * float(s[2])
    elif g_type == "cylinder":
        r = float(geom.get("radius", 0.05))
        h = float(geom.get("length", 0.1))
        return np.pi * r * r * h
    elif g_type == "sphere":
        r = float(geom.get("radius", 0.05))
        return (4.0 / 3.0) * np.pi * r * r * r
    else:
        return 1e-6  # mesh or unknown — tiny placeholder


def _inertia_at_centroid(geom: Dict[str, Any], mass: float) -> np.ndarray:
    """
    Return the 3×3 inertia tensor of a primitive at its own centroid, in its local frame.
    Cylinder convention: Z-aligned (matches both URDF and MuJoCo defaults).
    """
    g_type = geom.get("type", "box")
    if g_type == "box":
        s = geom.get("size", [0.1, 0.1, 0.1])
        x, y, z = float(s[0]), float(s[1]), float(s[2])
        Ixx = mass / 12.0 * (y**2 + z**2)
        Iyy = mass / 12.0 * (x**2 + z**2)
        Izz = mass / 12.0 * (x**2 + y**2)
    elif g_type == "cylinder":
        r = float(geom.get("radius", 0.05))
        h = float(geom.get("length", 0.1))
        Ixx = mass / 12.0 * (3.0 * r**2 + h**2)
        Iyy = Ixx
        Izz = 0.5 * mass * r**2
    elif g_type == "sphere":
        r = float(geom.get("radius", 0.05))
        v = 0.4 * mass * r**2
        Ixx = Iyy = Izz = v
    else:
        # Mesh / unknown: isotropic fallback
        v = max(0.001 * mass, 1e-9)
        Ixx = Iyy = Izz = v
    return np.diag([max(Ixx, 1e-9), max(Iyy, 1e-9), max(Izz, 1e-9)])


def _rotation_matrix_rpy(rpy: List[float]) -> np.ndarray:
    """3×3 rotation matrix from URDF RPY (extrinsic XYZ, i.e. R = Rz·Ry·Rx)."""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _parallel_axis(I: np.ndarray, mass: float, d: np.ndarray) -> np.ndarray:
    """
    Translate an inertia tensor from the primitive centroid to a reference point
    displaced by d (parallel-axis theorem): I' = I + m(|d|²E − d dᵀ).
    """
    d_sq = float(np.dot(d, d))
    return I + mass * (d_sq * np.eye(3) - np.outer(d, d))


def _compute_inertia_from_collision_list(
    collision_list: List[Dict[str, Any]],
    mass_total: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute the composite inertia tensor and COM for a link with multiple collision
    primitives.  Mass is distributed proportionally to primitive volume.

    Returns:
        (I_com, com_pos)
        I_com   — 3×3 inertia tensor expressed at the body COM in the link frame
        com_pos — COM position in the link frame (m)
    """
    if not collision_list:
        return np.diag([1e-6, 1e-6, 1e-6]), np.zeros(3)

    # Volume-weighted mass distribution
    volumes = [_primitive_volume(c["geometry"]) for c in collision_list]
    total_vol = sum(volumes) or float(len(collision_list))
    masses = [mass_total * v / total_vol for v in volumes]

    # Volume-weighted COM in link frame
    com = np.zeros(3)
    for coll, m in zip(collision_list, masses):
        com += m * np.array(coll["origin_xyz"], dtype=float)
    com /= mass_total

    # Sum inertia contributions, each expressed at the body COM
    I_total = np.zeros((3, 3))
    for coll, m in zip(collision_list, masses):
        d_i = np.array(coll["origin_xyz"], dtype=float)  # primitive centroid in link frame
        rpy = coll["origin_rpy"]

        # Inertia at primitive centroid in primitive's local frame
        I_local = _inertia_at_centroid(coll["geometry"], m)

        # Rotate into link frame axes
        if any(abs(v) > 1e-8 for v in rpy):
            R = _rotation_matrix_rpy(rpy)
            I_local = R @ I_local @ R.T

        # Translate from primitive centroid (d_i) to body COM (com)
        I_total += _parallel_axis(I_local, m, d_i - com)

    # Small numerical floor to guarantee positive-definiteness
    floor = max(1e-9, 1e-7 * mass_total)
    I_total += np.eye(3) * floor

    return I_total, com


def _create_body_element(
    link_data: Dict[str, Any],
    joint_elem: Optional[etree._Element] = None,
    parent_elem: Optional[etree._Element] = None,
    mesh_assets: Optional[Dict[str, str]] = None,
    contact_class: str = "none",
    component: Optional[Dict[str, Any]] = None,
    is_imu: bool = False,
) -> etree._Element:
    """Create a MuJoCo body element for a URDF link.

    contact_class is the link's catalog contact role (see PartIdentity);
    component is its catalog entry (None for custom bodies).
    """
    is_wheel = contact_class == "wheel"

    body = etree.Element("body")
    body.set("name", link_data["name"])

    # IMU site at body origin so the accelerometer/gyro sensors compile.
    # Without this, MJCF compile fails for any URDF with an imu* link.
    if is_imu:
        site = etree.SubElement(body, "site")
        site.set("name", link_data["name"])
        site.set("size", "0.005")

    # ── Position / orientation from the joint that connects to this body ───────
    if joint_elem is not None:
        origin_elem = joint_elem.find("origin")
        if origin_elem is not None:
            try:
                pos = [float(v) for v in origin_elem.get("xyz", "0 0 0").split()]
                body.set("pos", " ".join(str(v) for v in pos))
            except (ValueError, TypeError):
                pass
            try:
                rpy = [float(v) for v in origin_elem.get("rpy", "0 0 0").split()]
                quat = _rpy_to_quat(rpy[0], rpy[1], rpy[2])
                body.set("quat", " ".join(str(v) for v in quat))
            except (ValueError, TypeError):
                pass

    # ── Mass / inertia ─────────────────────────────────────────────────────────
    collision_list = link_data.get("collision_list", [])
    visual_mesh_files = link_data.get("visual_mesh_files", [])
    mass = link_data["mass"]
    explicit_inertia = link_data.get("explicit_inertia")

    # Auto-estimate mass from geometry volume if not given in the URDF.
    if mass < 1e-6 and collision_list:
        total_vol = sum(_primitive_volume(c["geometry"]) for c in collision_list)
        mass = max(total_vol * _PLASTIC_DENSITY, 0.001)  # floor at 1 g

    if mass > 1e-6:
        inertial = etree.SubElement(body, "inertial")
        inertial.set("mass", f"{mass:.6g}")

        if explicit_inertia is not None:
            # URDF provided a precise tensor (e.g. from CAD export) — use it as-is
            ixx, iyy, izz, ixy, ixz, iyz = explicit_inertia
            inertial.set("pos", "0 0 0")
            inertial.set("fullinertia",
                         f"{ixx:.6g} {iyy:.6g} {izz:.6g} {ixy:.6g} {ixz:.6g} {iyz:.6g}")
        else:
            # Try trimesh inertia from visual mesh when:
            # (a) no explicit inertia from URDF, AND
            # (b) visual geometry is a richer mesh (common for CAD robots).
            # This is the biggest single accuracy win for mesh robots.
            trimesh_result: Optional[Tuple[np.ndarray, np.ndarray]] = None
            for vm in visual_mesh_files:
                # Skip meshes with non-zero <origin> — the COM trimesh reports
                # is in the mesh frame; with a non-zero visual origin we'd need
                # to compose that transform too, which we don't currently track.
                if any(abs(v) > 1e-9 for v in vm["origin_xyz"]) or any(
                    abs(v) > 1e-9 for v in vm["origin_rpy"]
                ):
                    continue
                trimesh_result = _trimesh_inertia(vm["file"], mass, vm["scale"])
                if trimesh_result is not None:
                    break

            if trimesh_result is not None:
                # Trimesh inertia is expressed at the mesh COM — write that COM
                # as the body's inertial pos so the spatial inertia is valid.
                trimesh_I, trimesh_com = trimesh_result
                ixx = trimesh_I[0, 0]; iyy = trimesh_I[1, 1]; izz = trimesh_I[2, 2]
                ixy = trimesh_I[0, 1]; ixz = trimesh_I[0, 2]; iyz = trimesh_I[1, 2]
                cx, cy, cz = float(trimesh_com[0]), float(trimesh_com[1]), float(trimesh_com[2])
                inertial.set("pos", f"{cx:.6g} {cy:.6g} {cz:.6g}")
                inertial.set("fullinertia",
                             f"{ixx:.6g} {iyy:.6g} {izz:.6g} {ixy:.6g} {ixz:.6g} {iyz:.6g}")
            elif collision_list:
                # Compute shape-based inertia from all collision primitives
                I_com, com_pos = _compute_inertia_from_collision_list(collision_list, mass)
                px, py, pz = com_pos
                inertial.set("pos", f"{px:.6g} {py:.6g} {pz:.6g}")
                ixx, iyy, izz = I_com[0, 0], I_com[1, 1], I_com[2, 2]
                ixy, ixz, iyz = I_com[0, 1], I_com[0, 2], I_com[1, 2]
                max_diag = max(ixx, iyy, izz)
                if max(abs(ixy), abs(ixz), abs(iyz)) > 1e-4 * max_diag:
                    inertial.set("fullinertia",
                                 f"{ixx:.6g} {iyy:.6g} {izz:.6g} {ixy:.6g} {ixz:.6g} {iyz:.6g}")
                else:
                    inertial.set("diaginertia", f"{ixx:.6g} {iyy:.6g} {izz:.6g}")
            else:
                # No geometry — tiny isotropic fallback so MuJoCo doesn't reject the body
                d = max(1e-6 * mass, 1e-9)
                inertial.set("pos", "0 0 0")
                inertial.set("diaginertia", f"{d:.6g} {d:.6g} {d:.6g}")

    # ── Wheel simplification ──────────────────────────────────────────────────
    # The wheel's joint already carries rpy=[-π/2, 0, 0] to orient the axle
    # laterally. The collision cylinders from the visual shape carry origin_rpy
    # [π/2, 0, 0]. In MuJoCo these compose as body_quat * geom_quat, so the two
    # rotations cancel and the cylinder ends up standing upright (flat face on
    # floor) — that's the "square hitbox" that causes bouncy physics.
    # Fix: replace all wheel collision geoms with the single outermost (tire)
    # cylinder at identity orientation so only the body quat acts on it.
    if is_wheel:
        tire_coll = None
        for c in collision_list:
            g = c["geometry"]
            if g.get("type") == "cylinder":
                if tire_coll is None or g.get("radius", 0) > tire_coll["geometry"].get("radius", 0):
                    tire_coll = c
        if tire_coll is None:
            tire_coll = _sim_wheel_cylinder_collision(component)
        if tire_coll:
            collision_list = [{
                "geometry": tire_coll["geometry"],
                "origin_xyz": tire_coll["origin_xyz"],
                "origin_rpy": [0.0, 0.0, 0.0],
            }]

    # ── Collision geoms — one <geom> per collision primitive ───────────────────
    for coll in collision_list:
        geom = coll["geometry"]
        ox, oy, oz = coll["origin_xyz"]
        orpy = coll["origin_rpy"]

        geom_elem = etree.SubElement(body, "geom")
        geom_type = geom.get("type", "box")
        geom_elem.set("type", geom_type)
        geom_elem.set("material", "MatGray")
        # Contact class from the catalog role sets the friction parameters.
        # Role geoms (foot/wheel/gripper/...) collide with terrain only (CT_ROBOT_ROLE ↔ CT_WORLD).
        # Structural geoms collide with terrain only (CT_STRUCTURAL ↔ CT_WORLD) — no
        # structural-vs-structural pairs, avoiding the constraint-arena explosion that
        # occurs with 40+ link assemblies at t=0.
        geom_class = _CONTACT_GEOM_CLASS.get(contact_class)
        if geom_class:
            geom_elem.set("class", geom_class)
        else:
            geom_elem.set("contype", str(CT_STRUCTURAL))
            geom_elem.set("conaffinity", str(CT_WORLD))

        # Geom position offset
        if any(abs(v) > 1e-9 for v in (ox, oy, oz)):
            geom_elem.set("pos", f"{ox:.6g} {oy:.6g} {oz:.6g}")

        # Geom orientation (rpy → quaternion)
        if any(abs(v) > 1e-9 for v in orpy):
            qw, qx, qy, qz = _rpy_to_quat(orpy[0], orpy[1], orpy[2])
            geom_elem.set("quat", f"{qw:.6g} {qx:.6g} {qy:.6g} {qz:.6g}")

        # Geometry-type-specific size attributes
        if geom_type == "box":
            size = geom.get("size", [1, 1, 1])
            halfsize = [s / 2.0 for s in size]
            geom_elem.set("size", " ".join(f"{v:.6g}" for v in halfsize))

        elif geom_type == "cylinder":
            # MuJoCo cylinder: size = [radius, half-length]; default axis is Z
            radius = float(geom.get("radius", 0.1))
            length = float(geom.get("length", 1.0))
            geom_elem.set("size", f"{radius:.6g} {length / 2.0:.6g}")

        elif geom_type == "sphere":
            radius = float(geom.get("radius", 0.1))
            geom_elem.set("size", f"{radius:.6g}")

        elif geom_type == "mesh":
            filename = geom.get("filename", "")
            mesh_name = geom.get("mesh_name", "")
            scale = geom.get("scale", [1.0, 1.0, 1.0])
            if filename and mesh_name:
                if mesh_assets is not None:
                    mesh_assets[mesh_name] = (filename, scale)
                geom_elem.set("mesh", mesh_name)

    return body



def _add_track_rollers(body_elem: etree._Element, link_name: str,
                       incoming_joint: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Make a rigid track module drivable.

    A tracked module is one rigid link in the URDF, so MuJoCo had nothing to
    turn and tanks could not move. Model the belt as a row of rollers along the
    module's long axis, ganged by equality constraints and driven through one
    velocity-controlled joint (``<track joint>_drive``). The box stays as the
    visual/structural envelope but no longer touches the ground; the rollers do.
    """
    box = next((g for g in body_elem.findall("geom") if g.get("type") == "box"), None)
    if box is None:
        return None
    try:
        hx, hy, hz = (float(v) for v in box.get("size", "").split())
    except ValueError:
        return None
    cx, cy, cz = (float(v) for v in (box.get("pos") or "0 0 0").split())
    if box.get("quat"):
        return None  # rotated envelope: leave it rigid rather than guess axes
    long_ax = 0 if hx >= hy else 1
    half_len, half_w = (hx, hy) if long_ax == 0 else (hy, hx)
    r = hz
    n = max(3, int(round((2 * half_len - 2 * r) / (1.2 * r))) + 1)
    box.set("contype", "0")
    box.set("conaffinity", "0")
    base = (incoming_joint["name"] if incoming_joint else link_name)
    base = base[:-6] if base.endswith("_joint") else base
    drive = f"{base}_drive"
    names = []
    for k in range(n):
        u = -half_len + r + k * (2 * half_len - 2 * r) / max(n - 1, 1)
        pos = [cx, cy, cz]
        pos[long_ax] += u
        rb = etree.SubElement(body_elem, "body")
        rb.set("name", f"{link_name}_roller{k}")
        rb.set("pos", " ".join(f"{v:.6g}" for v in pos))
        jn = drive if k == 0 else f"{drive}_idler{k}"
        names.append(jn)
        j = etree.SubElement(rb, "joint")
        j.set("name", jn)
        j.set("type", "hinge")
        j.set("axis", "0 1 0" if long_ax == 0 else "1 0 0")
        j.set("damping", "0.01")
        j.set("armature", "0.0005")
        g = etree.SubElement(rb, "geom")
        g.set("type", "cylinder")
        g.set("class", "wheel")
        g.set("material", "MatGray")
        g.set("rgba", "0 0 0 0")
        g.set("mass", "0.02")
        # cylinder axis (local Z) along the roller axle
        g.set("quat", "0.707107 0.707107 0 0" if long_ax == 0 else "0.707107 0 0.707107 0")
        g.set("size", f"{r:.6g} {half_w:.6g}")
    return {"drive": drive, "joints": names, "link": link_name}

# Actuator defaults when neither the URDF nor the catalog rates a joint.
DEFAULT_EFFORT = 10.0            # N·m (revolute) / N (prismatic)
DEFAULT_VELOCITY_RAD_S = 10.0    # continuous joints without a speed rating
DEFAULT_TRACK_TORQUE_NM = 6.0
# Position-servo error at which the rated effort is reached (sets kp).
SERVO_SATURATION_RAD = 0.1       # ~6°
SERVO_SATURATION_M = 0.005       # 5 mm


def _joint_rating(joint: Dict[str, Any], ident: PartIdentity) -> Tuple[float, float]:
    """(effort, velocity) for a joint: the URDF <limit> when it states them,
    else the catalog rating of the actuator driving it (the joint's parent
    link, by the designer's convention), else the defaults. velocity 0 means
    "unrated"."""
    limits = joint.get("limits") or {}
    effort = limits.get("effort")
    velocity = float(limits.get("velocity") or 0.0)
    if effort is None or velocity <= 0:
        rating = actuator_rating(ident.component(joint["parent"]))
        want = "linear" if joint["type"] == "prismatic" else "rotary"
        if rating and rating[0] == want:
            if effort is None:
                effort = rating[1]
            if velocity <= 0:
                velocity = rating[2]
    return (DEFAULT_EFFORT if effort is None else float(effort)), velocity


def _terrain_float(config: Dict[str, Any], key: str, default: float, lo: float, hi: float) -> float:
    try:
        value = float(config.get(key, default))
    except (TypeError, ValueError):
        value = default
    return max(lo, min(hi, value))


def _terrain_int(config: Dict[str, Any], key: str, default: int, lo: int, hi: int) -> int:
    try:
        value = int(config.get(key, default))
    except (TypeError, ValueError):
        value = default
    return max(lo, min(hi, value))


def normalize_terrain_config(terrain_config: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    config = terrain_config if isinstance(terrain_config, dict) else {}
    terrain_type = str(config.get("type", "flat")).lower()
    if terrain_type not in {"flat", "rough", "stairs"}:
        terrain_type = "flat"
    return {
        "type": terrain_type,
        "seed": _terrain_int(config, "seed", 1, 0, 2_147_483_647),
        "height": _terrain_float(config, "height", 0.08, 0.0, 0.4),
        "scale": _terrain_float(config, "scale", 1.0, 0.25, 3.0),
        "roughness": _terrain_float(config, "roughness", 0.6, 0.0, 1.0),
        "friction": _terrain_float(config, "friction", 3.0, 0.05, 5.0),
    }


def _set_terrain_friction(geom: etree._Element, friction: float) -> None:
    # Robot geoms have priority 1 and terrain the default 0, so MuJoCo takes a
    # robot/terrain contact's friction from the ROBOT geom (its contact class,
    # already scaled by this terrain setting — see _friction_attr). With equal
    # priorities it would take the element-wise max, and this grippy terrain
    # overrode every class (wheels at 3.0 instead of 1.2; a floor rolling
    # coefficient once braked every wheel). The value here only matters for
    # contacts with other priority-0 geoms.
    geom.set("friction", f"{friction:.6g} 0.005 0.0001")
    geom.set("condim", "6")
    # Explicit: terrain geoms would otherwise inherit the robot default's priority 1.
    geom.set("priority", "0")


def _add_flat_floor(worldbody: etree._Element, friction: float, rgba: str = "0.5 0.5 0.5 1") -> etree._Element:
    floor_geom = etree.SubElement(worldbody, "geom")
    floor_geom.set("name", "floor")
    floor_geom.set("type", "plane")
    floor_geom.set("size", "0 0 0.05")
    floor_geom.set("rgba", rgba)
    floor_geom.set("contype", str(CT_WORLD))
    floor_geom.set("conaffinity", str(CT_WORLD | CT_ROBOT_ROLE | CT_STRUCTURAL))
    _set_terrain_friction(floor_geom, friction)
    return floor_geom


def _add_rough_terrain(asset: etree._Element, worldbody: etree._Element, config: Dict[str, Any]) -> None:
    n = 49
    height = max(config["height"], 0.01)
    scale = config["scale"]
    roughness = config["roughness"]
    rng = np.random.default_rng(config["seed"])
    data = rng.normal(0.0, 1.0, (n, n))

    # Smooth the raw noise so feet see rolling bumps rather than a sharp checkerboard.
    smooth_passes = max(1, int(round(2 + scale * 3)))
    for _ in range(smooth_passes):
        data = (
            data
            + np.roll(data, 1, axis=0)
            + np.roll(data, -1, axis=0)
            + np.roll(data, 1, axis=1)
            + np.roll(data, -1, axis=1)
        ) / 5.0

    data -= float(np.min(data))
    span = float(np.max(data))
    if span > 1e-9:
        data /= span
    data *= roughness

    coords = np.linspace(-1.0, 1.0, n)
    xx, yy = np.meshgrid(coords, coords, indexing="ij")
    rr = np.sqrt(xx * xx + yy * yy)
    # Keep the spawn pad flat and blend out gradually so the robot never starts
    # intersecting random terrain.
    blend = np.clip((rr - 0.16) / 0.18, 0.0, 1.0)
    data *= blend

    hfield = etree.SubElement(asset, "hfield")
    hfield.set("name", "terrain_hfield")
    hfield.set("nrow", str(n))
    hfield.set("ncol", str(n))
    hfield.set("size", f"{4.5 * scale:.6g} {4.5 * scale:.6g} {height:.6g} 0.02")
    hfield.set("elevation", " ".join(f"{v:.5f}" for v in data.reshape(-1)))

    geom = etree.SubElement(worldbody, "geom")
    geom.set("name", "floor")
    geom.set("type", "hfield")
    geom.set("hfield", "terrain_hfield")
    geom.set("rgba", "0.24 0.30 0.27 1")
    geom.set("contype", str(CT_WORLD))
    geom.set("conaffinity", str(CT_WORLD | CT_ROBOT_ROLE | CT_STRUCTURAL))
    _set_terrain_friction(geom, config["friction"])


def _add_stair_terrain(worldbody: etree._Element, config: Dict[str, Any]) -> None:
    friction = config["friction"]
    height = max(config["height"], 0.02)
    scale = config["scale"]
    depth = 0.34 * scale
    width = 2.4 * scale
    start_x = 0.9
    count = 7

    _add_flat_floor(worldbody, friction, "0.38 0.38 0.36 1")
    for i in range(count):
        step_height = height * (i + 1)
        step = etree.SubElement(worldbody, "geom")
        step.set("name", f"terrain_step_{i + 1}")
        step.set("type", "box")
        step.set("pos", f"{start_x + depth * (i + 0.5):.6g} 0 {step_height / 2.0:.6g}")
        step.set("size", f"{depth / 2.0:.6g} {width / 2.0:.6g} {step_height / 2.0:.6g}")
        step.set("rgba", "0.36 0.35 0.32 1")
        step.set("contype", str(CT_WORLD))
        step.set("conaffinity", str(CT_WORLD | CT_ROBOT_ROLE | CT_STRUCTURAL))
        _set_terrain_friction(step, friction)


def _add_terrain(asset: etree._Element, worldbody: etree._Element, terrain_config: Optional[Dict[str, Any]]) -> None:
    config = normalize_terrain_config(terrain_config)
    if config["type"] == "rough":
        _add_rough_terrain(asset, worldbody, config)
    elif config["type"] == "stairs":
        _add_stair_terrain(worldbody, config)
    else:
        _add_flat_floor(worldbody, config["friction"])


def urdf_to_mjcf(
    urdf_path: str,
    free_base: bool = False,
    terrain_config: Optional[Dict[str, Any]] = None,
) -> str:
    """
    Convert a URDF file to MJCF XML string.

    Args:
        urdf_path: Path to the URDF file.
        free_base: If True, add a <freejoint/> to the root body so it is
                   free-floating (useful for mobile robots and UAVs).
        terrain_config: Optional terrain settings. Supported types are
                        "flat", "rough", and "stairs".

    Returns:
        MJCF XML as a string.

    Raises:
        FileNotFoundError: If URDF file doesn't exist.
        ValueError: If URDF is invalid.
    """
    urdf_root = _load_urdf_xml(urdf_path)
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    # Which catalog part each link is (explicit vector:parts map, or the
    # <component_id>_<N> naming convention for catalog ids only).
    with open(urdf_path, encoding="utf-8", errors="replace") as f:
        ident = PartIdentity(f.read())
    terrain = normalize_terrain_config(terrain_config)

    # Extract basic info
    robot_name = urdf_root.get("name", "robot")

    # Parse links and joints
    links: Dict[str, Dict[str, Any]] = {}
    for link_elem in urdf_root.findall("link"):
        link_data = _extract_link_data(link_elem, urdf_dir)
        links[link_data["name"]] = link_data

    # Parse joints
    joints: List[Dict[str, Any]] = []
    for joint_elem in urdf_root.findall("joint"):
        joint_type = joint_elem.get("type", "fixed")
        joint_name = joint_elem.get("name", "")

        parent_elem = joint_elem.find("parent")
        child_elem = joint_elem.find("child")

        if parent_elem is None or child_elem is None:
            continue

        parent_link = parent_elem.get("link", "")
        child_link = child_elem.get("link", "")

        if not parent_link or not child_link:
            continue

        axis_elem = joint_elem.find("axis")
        axis = [0, 0, 1]
        if axis_elem is not None:
            axis_str = axis_elem.get("xyz", "0 0 1")
            try:
                axis = [float(x) for x in axis_str.split()]
            except (ValueError, TypeError):
                axis = [0, 0, 1]

        limits = None
        limit_elem = joint_elem.find("limit")
        if limit_elem is not None:
            try:
                lower = float(limit_elem.get("lower", "-3.14159"))
                upper = float(limit_elem.get("upper", "3.14159"))
                # None = not in the URDF: rated from the driving actuator below.
                effort = float(limit_elem.get("effort")) if limit_elem.get("effort") is not None else None
                velocity = float(limit_elem.get("velocity", "0.0"))
                limits = {
                    "lower": lower,
                    "upper": upper,
                    "effort": effort,
                    "velocity": velocity,
                }
            except (ValueError, TypeError):
                pass

        # Read URDF <dynamics> for damping and friction.
        # None means "not specified in URDF" so joint-building code can apply
        # physics-based defaults instead of blindly using 0.1 / 0.0.
        dynamics = {"damping": None, "friction": None}
        dynamics_elem = joint_elem.find("dynamics")
        if dynamics_elem is not None:
            try:
                d = dynamics_elem.get("damping")
                f = dynamics_elem.get("friction")
                if d is not None:
                    dynamics["damping"] = float(d)
                if f is not None:
                    dynamics["friction"] = float(f)
            except (ValueError, TypeError):
                pass

        joint = {
            "name": joint_name,
            "type": joint_type,
            "parent": parent_link,
            "child": child_link,
            "axis": axis,
            "elem": joint_elem,
            "limits": limits,
            "dynamics": dynamics,
        }
        joint["effort"], joint["velocity"] = _joint_rating(joint, ident)
        joints.append(joint)

    # Find root link (link with no parent)
    child_links = {j["child"] for j in joints}
    root_links = [link_name for link_name in links if link_name not in child_links]
    if len(root_links) != 1:
        if not root_links:
            raise ValueError("URDF must have exactly one root link for simulation")
        preview = ", ".join(root_links[:8])
        suffix = ", ..." if len(root_links) > 8 else ""
        raise ValueError(
            f"Simulation supports one connected robot tree; found "
            f"{len(root_links)} root links: {preview}{suffix}"
        )
    root_link = root_links[0]

    # Build MJCF
    mjcf_root = etree.Element("mujoco")
    mjcf_root.set("model", robot_name)

    # URDF angles are radians. MJCF defaults to degrees for joint ranges, so
    # without this every revolute limit (e.g. -0.87..0.79 rad) became a ±1°
    # clamp and legs/arms could barely move.
    compiler = etree.SubElement(mjcf_root, "compiler")
    compiler.set("angle", "radian")

    # Add option
    # timestep=0.001: tighter than default (0.002) — needed for stiff actuators.
    # integrator=implicitfast: A-stable, much better than Euler for stiff joints.
    # solver=Newton, iterations=100, tolerance=1e-10: tight convergence.
    # cone=elliptic + impratio=10: eliminates the "ice-skating" feel of pyramidal cone.
    # noslip_iterations=3: kills residual tangential drift at rest contacts.
    option = etree.SubElement(mjcf_root, "option")
    option.set("timestep", "0.001")
    option.set("gravity", "0 0 -9.81")
    option.set("integrator", "implicitfast")
    option.set("solver", "Newton")
    option.set("iterations", "100")
    option.set("tolerance", "1e-10")
    option.set("cone", "elliptic")
    option.set("impratio", "10")
    option.set("noslip_iterations", "3")
    # Energy tracking: lets us assert conservation in tests and display KE+PE in the UI.
    flag_elem = etree.SubElement(option, "flag")
    flag_elem.set("energy", "enable")

    # Quadrupeds with per-mesh convex-hull collision geoms blow past MuJoCo's
    # default constraint arena on the first step ("Insufficient arena memory…
    # above 14M bytes"). When that overflows, contacts are dropped and the
    # integrator produces NaN poses, making the robot vanish from the viewport.
    size_elem = etree.SubElement(mjcf_root, "size")
    size_elem.set("memory", "64M")

    # Add contact/solver defaults
    # Every robot geom has priority 1 (terrain 0), so robot/terrain contacts
    # use the robot geom's own class friction, scaled by the terrain grip
    # setting — see _CONTACT_CLASS_PARAMS. solref/solimp: slightly soft
    # contacts reduce bounce without penetration.
    terrain_friction = terrain["friction"]
    default_block = etree.SubElement(mjcf_root, "default")
    default_geom = etree.SubElement(default_block, "geom")
    _, condim, solimp = _CONTACT_CLASS_PARAMS["default"]
    default_geom.set("friction", _friction_attr("default", terrain_friction))
    default_geom.set("condim", str(condim))
    default_geom.set("solref", "0.005 1")
    default_geom.set("solimp", solimp)
    default_geom.set("priority", "1")
    # Role sub-classes (foot / wheel / caster / gripper / suction).
    for cls_name in sorted(set(_CONTACT_GEOM_CLASS.values())):
        _, condim, solimp = _CONTACT_CLASS_PARAMS[cls_name]
        cls_elem = etree.SubElement(default_block, "default")
        cls_elem.set("class", cls_name)
        cls_geom = etree.SubElement(cls_elem, "geom")
        cls_geom.set("friction", _friction_attr(cls_name, terrain_friction))
        cls_geom.set("condim", str(condim))
        cls_geom.set("solref", "0.005 1")
        cls_geom.set("solimp", solimp)
        cls_geom.set("contype", str(CT_ROBOT_ROLE))
        cls_geom.set("conaffinity", str(CT_WORLD))

    # Add visual settings
    visual = etree.SubElement(mjcf_root, "visual")
    global_elem = etree.SubElement(visual, "global")
    global_elem.set("offheight", "2400")
    global_elem.set("offwidth", "2400")

    # Add asset section (materials + meshes populated after body building)
    asset = etree.SubElement(mjcf_root, "asset")
    material = etree.SubElement(asset, "material")
    material.set("name", "MatGray")
    material.set("rgba", "0.7 0.7 0.7 1.0")

    # Build world body
    worldbody = etree.SubElement(mjcf_root, "worldbody")

    # Terrain lives in the world body so physics and contact reporting see the
    # same landscape the UI asks for. The default is the old flat floor.
    _add_terrain(asset, worldbody, terrain)

    # Track mesh assets that need declarations in <asset>: name → (file, [sx, sy, sz])
    mesh_assets: Dict[str, Tuple[str, List[float]]] = {}

    # Recursively add bodies
    track_drives: List[Dict[str, Any]] = []

    def add_body_recursive(parent_body_elem: etree._Element, link_name: str, visited: set):
        """Recursively add body elements for a link and its children."""
        if link_name in visited:
            return
        visited.add(link_name)

        if link_name not in links:
            return

        link_data = links[link_name]

        # Find incoming joint (parent -> child)
        incoming_joint = None
        for joint in joints:
            if joint["child"] == link_name:
                incoming_joint = joint
                break

        # Contact role from the catalog (sim_metadata.contact_class).
        contact_class = ident.contact_class(link_name)
        is_imu = _is_imu_link(link_name, ident)

        # Create body element (mesh_assets dict accumulates mesh file declarations)
        body_elem = _create_body_element(
            link_data,
            incoming_joint["elem"] if incoming_joint else None,
            mesh_assets=mesh_assets,
            contact_class=contact_class,
            component=ident.component(link_name),
            is_imu=is_imu,
        )
        parent_body_elem.append(body_elem)
        if contact_class == "track":
            track = _add_track_rollers(body_elem, link_name, incoming_joint)
            if track:
                track_drives.append(track)

        # Add joint element if incoming joint exists and is not fixed
        if incoming_joint and incoming_joint["type"] != "fixed":
            joint_elem = etree.SubElement(body_elem, "joint")
            joint_elem.set("name", incoming_joint["name"])

            # Convert URDF joint type to MuJoCo joint type
            urdf_joint_type = incoming_joint["type"]
            _URDF_TO_MJCF_JOINT = {
                "revolute": "hinge",
                "continuous": "hinge",  # continuous = unbounded hinge
                "prismatic": "slide",
            }
            mjcf_joint_type = _URDF_TO_MJCF_JOINT.get(urdf_joint_type, "ball")
            joint_elem.set("type", mjcf_joint_type)

            # Set axis
            axis = incoming_joint["axis"]
            joint_elem.set("axis", " ".join(str(x) for x in axis))

            # Set limits — continuous joints are unbounded, skip range
            if incoming_joint["limits"] and urdf_joint_type != "continuous":
                limits = incoming_joint["limits"]
                joint_elem.set("range", f"{limits['lower']} {limits['upper']}")
                if limits.get("velocity", 0.0) > 0:
                    eff = incoming_joint["effort"]
                    joint_elem.set("actuatorfrcrange", f"-{eff} {eff}")

            # ── Joint dynamics ────────────────────────────────────────────────
            dyn = incoming_joint.get("dynamics", {})
            is_prismatic = urdf_joint_type == "prismatic"
            effort = max(incoming_joint["effort"], 0.01)

            # Damping (viscous friction). Use URDF value when provided; otherwise
            # scale with sqrt(effort) so heavier joints settle at a similar rate
            # regardless of stiffness. Prismatic joints are 10× stiffer by default
            # because linear slides have higher viscous losses.
            urdf_damping = dyn.get("damping")
            if urdf_damping is not None:
                damping = urdf_damping
            elif is_prismatic:
                damping = max(0.5, 1.0 * (effort / 10.0) ** 0.5)
            else:
                damping = max(0.05, 0.1 * (effort / 10.0) ** 0.5)
            joint_elem.set("damping", f"{damping:.6g}")

            # Frictionloss (Coulomb / dry friction). URDF default is 0, which lets
            # joints drift indefinitely when control is released. A small non-zero
            # value prevents this without fighting the actuator.
            urdf_friction = dyn.get("friction")
            if urdf_friction is not None and urdf_friction > 1e-9:
                frictionloss = urdf_friction
            elif is_prismatic:
                frictionloss = 0.5   # N — typical linear-slide Coulomb friction
            else:
                frictionloss = 0.02  # Nm — typical revolute dry friction
            joint_elem.set("frictionloss", f"{frictionloss:.6g}")

            # Armature (reflected rotor inertia). Even a tiny value dramatically
            # stabilises PD control and prevents high-frequency jitter, at negligible
            # computational cost. Scale conservatively with effort so that a 1 Nm
            # servo and a 100 Nm drive don't share the same value.
            armature = (5e-4 if not is_prismatic else 1e-3) * max(1.0, (effort / 10.0) ** 0.5)
            joint_elem.set("armature", f"{armature:.6g}")

        # Add children
        for joint in joints:
            if joint["parent"] == link_name:
                add_body_recursive(body_elem, joint["child"], visited)

    # Start with root link
    if root_link:
        add_body_recursive(worldbody, root_link, set())
        # Free-floating base: inject a freejoint into the root body so it can
        # move freely in the world (mobile robots, UAVs, etc.)
        if free_base and len(worldbody) > 0:
            root_bodies = [c for c in worldbody if c.tag == "body"]
            if root_bodies:
                free_joint = etree.Element("freejoint")
                free_joint.set("name", "base_freejoint")
                root_bodies[0].insert(0, free_joint)

    # Back-fill mesh asset declarations now that all bodies have been built
    for mesh_name, (mesh_file, mesh_scale) in mesh_assets.items():
        mesh_decl = etree.SubElement(asset, "mesh")
        mesh_decl.set("name", mesh_name)
        mesh_decl.set("file", mesh_file)
        if mesh_scale != [1.0, 1.0, 1.0]:
            mesh_decl.set("scale", " ".join(f"{s:.6g}" for s in mesh_scale))

    # Self-collision excludes: suppress contacts between every adjacent body pair.
    # Touching neighbors (parent/child across a joint) almost always cause spurious
    # contact forces that destabilize the sim. Non-adjacent self-collision among
    # role-tagged geoms (foot/wheel/gripper) is still on so legs/feet can't tunnel
    # through each other; structural geoms use CT_STRUCTURAL contype so they collide with terrain only.
    if joints:
        contact_section = etree.SubElement(mjcf_root, "contact")
        seen_pairs: set = set()
        for joint in joints:
            p, c = joint["parent"], joint["child"]
            pair = (min(p, c), max(p, c))
            if pair not in seen_pairs:
                seen_pairs.add(pair)
                excl = etree.SubElement(contact_section, "exclude")
                excl.set("body1", p)
                excl.set("body2", c)

    # Add actuators section
    actuators = etree.SubElement(mjcf_root, "actuator")

    # Add actuators for all non-fixed joints.
    # Revolute and prismatic joints use position actuators (servo-like PD control).
    # Continuous joints have no angle limits, so they get velocity servos.
    for joint in joints:
        if joint["type"] in ("fixed",):
            continue

        if joint["effort"] <= 0.0:
            continue  # effort="0": a passive pivot (rocker, bogie, free hinge)
        effort = max(joint["effort"], 0.01)

        if joint["type"] == "continuous":
            # Unbounded rotation (wheels, rollers): a velocity servo, like a real
            # motor driver in speed mode. The command is a target speed in rad/s,
            # capped at the rated / URDF velocity; torque is capped at the effort.
            # Raw torque control made every wheeled robot slip and bounce unless
            # the script hand-tuned throttles.
            vmax = joint["velocity"] if joint["velocity"] > 0 else DEFAULT_VELOCITY_RAD_S
            # Full torque at ~15% speed error: stiff enough to hold speed under
            # load, soft enough not to chatter against ground contact.
            kv = max(effort / (0.15 * vmax), 1e-3)
            vel = etree.SubElement(actuators, "velocity")
            vel.set("name", f"{joint['name']}_vel")
            vel.set("joint", joint["name"])
            vel.set("kv", f"{kv:.6g}")
            vel.set("ctrllimited", "true")
            vel.set("ctrlrange", f"{-vmax:.6g} {vmax:.6g}")
            vel.set("forcelimited", "true")
            vel.set("forcerange", f"{-effort:.6g} {effort:.6g}")
        else:
            # Revolute / prismatic — position servo. A real servo saturates at
            # its rated torque a few degrees (or millimetres) off target, so
            # kp = effort / saturation error: a 0.18 N·m micro servo and a
            # 28 N·m one both reach full torque at the same error instead of
            # the micro servo being a bang-bang switch (the old kp floor of 50
            # saturated it 0.2° off target). Damping is critical (dampratio=1)
            # against the joint's reflected inertia, which MuJoCo computes at
            # compile time — no guessed kv.
            lower = joint["limits"]["lower"] if joint["limits"] else -3.14159
            upper = joint["limits"]["upper"] if joint["limits"] else 3.14159
            sat = SERVO_SATURATION_M if joint["type"] == "prismatic" else SERVO_SATURATION_RAD
            kp = effort / sat
            pos_act = etree.SubElement(actuators, "position")
            pos_act.set("name", f"{joint['name']}_pos")
            pos_act.set("joint", joint["name"])
            pos_act.set("kp", f"{kp:.6g}")
            pos_act.set("dampratio", "1")
            pos_act.set("ctrllimited", "true")
            pos_act.set("ctrlrange", f"{lower:.6f} {upper:.6f}")
            pos_act.set("forcelimited", "true")
            pos_act.set("forcerange", f"{-effort:.6g} {effort:.6g}")

    # Track modules: gang each track's rollers to its drive joint and drive it
    # like a wheel (velocity servo). Belt speed limit ~0.6 m/s.
    if track_drives:
        equality = etree.SubElement(mjcf_root, "equality")
        for tr in track_drives:
            for jn in tr["joints"][1:]:
                eq = etree.SubElement(equality, "joint")
                eq.set("joint1", jn)
                eq.set("joint2", tr["drive"])
                eq.set("polycoef", "0 1 0 0 0")
            # A track module carries its own drive when the catalog rates one.
            rating = actuator_rating(ident.component(tr["link"]))
            effort = rating[1] if rating and rating[0] == "rotary" else DEFAULT_TRACK_TORQUE_NM
            vmax = 15.0
            vel = etree.SubElement(actuators, "velocity")
            vel.set("name", f"{tr['drive']}_vel")
            vel.set("joint", tr["drive"])
            vel.set("kv", f"{effort / (0.15 * vmax):.6g}")
            vel.set("ctrllimited", "true")
            vel.set("ctrlrange", f"{-vmax:.6g} {vmax:.6g}")
            vel.set("forcelimited", "true")
            vel.set("forcerange", f"{-effort:.6g} {effort:.6g}")

    # Add sensor section — jointpos/jointvel/jointactuatorfrc per non-fixed joint,
    # plus end-effector pose sensors for any link tagged <vector:ee> or named *ee*/*end*.
    sensor_section = etree.SubElement(mjcf_root, "sensor")
    for joint in joints:
        if joint["type"] == "fixed":
            continue
        jname = joint["name"]
        jp = etree.SubElement(sensor_section, "jointpos")
        jp.set("name", f"{jname}_pos_sens")
        jp.set("joint", jname)
        jv = etree.SubElement(sensor_section, "jointvel")
        jv.set("name", f"{jname}_vel_sens")
        jv.set("joint", jname)
        jf = etree.SubElement(sensor_section, "jointactuatorfrc")
        jf.set("name", f"{jname}_frc_sens")
        jf.set("joint", jname)

    # End-effector sensors: token-aware match so 'knee' doesn't match 'ee'.
    for link_name in links:
        if _is_ee_link(link_name, ident):
            fp = etree.SubElement(sensor_section, "framepos")
            fp.set("name", f"{link_name}_pos_sens")
            fp.set("objtype", "body")
            fp.set("objname", link_name)
            fq = etree.SubElement(sensor_section, "framequat")
            fq.set("name", f"{link_name}_quat_sens")
            fq.set("objtype", "body")
            fq.set("objname", link_name)

    # IMU sensors: a <site> with the link name was emitted into each imu* body
    # by _create_body_element above, so the site reference here resolves.
    for link_name in links:
        if _is_imu_link(link_name, ident):
            acc = etree.SubElement(sensor_section, "accelerometer")
            acc.set("name", f"{link_name}_acc_sens")
            acc.set("site", link_name)
            gyro = etree.SubElement(sensor_section, "gyro")
            gyro.set("name", f"{link_name}_gyro_sens")
            gyro.set("site", link_name)

    # Convert to string
    xml_string = etree.tostring(
        mjcf_root,
        encoding="unicode",
        pretty_print=True,
    )

    return xml_string

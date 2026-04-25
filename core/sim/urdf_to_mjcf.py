"""
URDF to MJCF converter.

Converts a URDF file to MJCF XML format with actuators.
MuJoCo can load MJCF directly, and we inject actuators for all non-fixed joints.
"""
from typing import Dict, List, Tuple, Optional, Any
import os
from lxml import etree
import numpy as np

try:
    from presets import get_component as _get_preset_component
except Exception:  # pragma: no cover - depends on import entrypoint
    try:
        from core.presets import get_component as _get_preset_component
    except Exception:  # pragma: no cover
        _get_preset_component = None

# Resolve package:// URIs emitted by the frontend URDF builder.
# This file lives at <repo_root>/core/sim/urdf_to_mjcf.py, so the repo root
# is two directories up.  The frontend maps package:// to src/public/.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PACKAGE_URL_PREFIX = "package://"
_PACKAGE_MESH_ROOTS = [
    os.path.join(_REPO_ROOT, "src", "public"),
]


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

    # Collect visual meshes for trimesh inertia estimation.  Only meshes with a
    # zero <origin> are eligible — otherwise inertia about the mesh COM cannot be
    # written at the link origin without a parallel-axis shift, which would
    # silently produce a wrong tensor.  We also capture the scale so volume-based
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
    """Convert roll-pitch-yaw to quaternion [w, x, y, z]."""
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

# Link-name keywords that identify contact surface categories.
_FOOT_KEYWORDS    = frozenset(("foot", "toe", "pad", "paw", "tip", "sole"))
_GRIPPER_KEYWORDS = frozenset(("gripper", "finger", "claw", "thumb", "palm", "grasp"))
_IMU_KEYWORDS     = frozenset(("imu",))
_EE_KEYWORDS_MJCF = frozenset(("ee", "end_effector", "end-effector", "tool", "tcp"))

import re as _re
_TOKEN_SPLIT = _re.compile(r"[_\-\s]+")
_COMPONENT_INSTANCE_SUFFIX = _re.compile(r"^(.+)_\d+$")
_PRESET_CACHE: Dict[str, Optional[Dict[str, Any]]] = {}


def _name_tokens(name: str) -> set:
    return {t for t in _TOKEN_SPLIT.split(name.lower()) if t}


def _is_imu_link(link_name: str) -> bool:
    return bool(_name_tokens(link_name) & _IMU_KEYWORDS)


def _is_ee_link(link_name: str) -> bool:
    tokens = _name_tokens(link_name)
    return any(kw in tokens for kw in _EE_KEYWORDS_MJCF)


def _is_foot_link(link_name: str) -> bool:
    """Heuristic: does this link name suggest a foot / ground-contact surface?"""
    lower = link_name.lower()
    return any(kw in lower for kw in _FOOT_KEYWORDS)


def _component_id_from_link_name(link_name: str) -> str:
    """Strip the UI's trailing instance suffix from preset-backed link names."""
    match = _COMPONENT_INSTANCE_SUFFIX.match(link_name)
    return match.group(1) if match else link_name


def _preset_for_link(link_name: str) -> Optional[Dict[str, Any]]:
    if _get_preset_component is None:
        return None
    for component_id in (_component_id_from_link_name(link_name), link_name):
        if component_id not in _PRESET_CACHE:
            _PRESET_CACHE[component_id] = _get_preset_component(component_id)
        preset = _PRESET_CACHE[component_id]
        if preset:
            return preset
    return None


def _contact_class_for_link(link_name: str) -> str:
    preset = _preset_for_link(link_name)
    sim_metadata = preset.get("sim_metadata", {}) if preset else {}
    contact_class = sim_metadata.get("contact_class")
    return contact_class if isinstance(contact_class, str) else ""


def _is_gripper_link(link_name: str) -> bool:
    """Heuristic: does this link name suggest a gripper / finger contact?"""
    lower = link_name.lower()
    return any(kw in lower for kw in _GRIPPER_KEYWORDS)


def _link_has_cylinder_collision(link_data: Dict[str, Any]) -> bool:
    """Return True if the link's primary collision geometry is a cylinder."""
    for coll in link_data.get("collision_list", []):
        if coll["geometry"].get("type") == "cylinder":
            return True
    return False


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
    is_foot: bool = False,
    is_wheel: bool = False,
    is_gripper: bool = False,
    is_imu: bool = False,
) -> etree._Element:
    """Create a MuJoCo body element for a URDF link."""

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
    mass_auto_estimated = False

    # Auto-estimate mass from geometry volume if not given in the URDF.
    # Tag the link so the model-info warning can be surfaced in the UI.
    if mass < 1e-6 and collision_list:
        total_vol = sum(_primitive_volume(c["geometry"]) for c in collision_list)
        mass = max(total_vol * _PLASTIC_DENSITY, 0.001)  # floor at 1 g
        mass_auto_estimated = True

    if mass > 1e-6:
        inertial = etree.SubElement(body, "inertial")
        inertial.set("mass", f"{mass:.6g}")
        if mass_auto_estimated:
            inertial.set("user", "1")  # sentinel for post-processing / UI warning

        if explicit_inertia is not None:
            # URDF provided a precise tensor (e.g. from CAD export) — use it as-is
            ixx, iyy, izz, ixy, ixz, iyz = explicit_inertia
            inertial.set("pos", "0 0 0")
            inertial.set("fullinertia",
                         f"{ixx:.6g} {iyy:.6g} {izz:.6g} {ixy:.6g} {ixz:.6g} {iyz:.6g}")
        else:
            # Try trimesh inertia from visual mesh when:
            #   (a) no explicit inertia from URDF, AND
            #   (b) visual geometry is a richer mesh (common for CAD robots).
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
        # Assign contact class based on link role for appropriate friction parameters.
        # Only role-tagged geoms (feet, wheels, grippers) participate in collision —
        # frame/electronics/coupler geoms are visualized but excluded from the
        # broadphase (contype=0 conaffinity=0).  A 40+ link assembly otherwise emits
        # hundreds of spurious overlapping-frame contacts at t=0 (mounting cylinders
        # bolted into baseplates, etc.), exploding the constraint count past any
        # reasonable arena size and producing NaN poses on the first step.
        if is_foot:
            geom_elem.set("class", "foot")
        elif is_wheel:
            geom_elem.set("class", "wheel")
        elif is_gripper:
            geom_elem.set("class", "gripper")
        else:
            geom_elem.set("contype", "0")
            geom_elem.set("conaffinity", "0")

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
    geom.set("friction", f"{friction:.6g} {friction / 10.0:.6g} {friction / 100.0:.6g}")
    geom.set("condim", "6")


def _add_flat_floor(worldbody: etree._Element, friction: float, rgba: str = "0.5 0.5 0.5 1") -> etree._Element:
    floor_geom = etree.SubElement(worldbody, "geom")
    floor_geom.set("name", "floor")
    floor_geom.set("type", "plane")
    floor_geom.set("size", "0 0 0.05")
    floor_geom.set("rgba", rgba)
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
                effort = float(limit_elem.get("effort", "10.0"))
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

        joints.append({
            "name": joint_name,
            "type": joint_type,
            "parent": parent_link,
            "child": child_link,
            "axis": axis,
            "elem": joint_elem,
            "limits": limits,
            "dynamics": dynamics,
        })

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
    # condim=4: tangential + torsional friction (good for most links, avoids sliding)
    # solref/solimp: slightly soft contacts reduce bounce without penetration
    default_block = etree.SubElement(mjcf_root, "default")
    default_geom = etree.SubElement(default_block, "geom")
    default_geom.set("friction", "1.0 0.05 0.001")
    default_geom.set("condim", "4")
    default_geom.set("solref", "0.005 1")
    default_geom.set("solimp", "0.9 0.95 0.001")
    # Foot sub-class: full 6D friction (torsional + rolling) so feet don't pirouette
    foot_cls = etree.SubElement(default_block, "default")
    foot_cls.set("class", "foot")
    foot_geom = etree.SubElement(foot_cls, "geom")
    foot_geom.set("friction", "3.0 0.3 0.03")
    foot_geom.set("condim", "6")
    foot_geom.set("solref", "0.005 1")
    foot_geom.set("solimp", "0.95 0.99 0.001")
    # Wheel sub-class: high lateral friction, low torsional/rolling — prevents
    # lateral slip but allows rolling with minimal resistance.
    wheel_cls = etree.SubElement(default_block, "default")
    wheel_cls.set("class", "wheel")
    wheel_geom = etree.SubElement(wheel_cls, "geom")
    wheel_geom.set("friction", "1.2 0.002 0.0001")
    wheel_geom.set("condim", "6")
    wheel_geom.set("solref", "0.005 1")
    wheel_geom.set("solimp", "0.9 0.95 0.001")
    # Gripper/finger sub-class: high friction in all directions for secure grasping.
    gripper_cls = etree.SubElement(default_block, "default")
    gripper_cls.set("class", "gripper")
    gripper_geom = etree.SubElement(gripper_cls, "geom")
    gripper_geom.set("friction", "2.0 0.2 0.02")
    gripper_geom.set("condim", "6")
    gripper_geom.set("solref", "0.005 1")
    gripper_geom.set("solimp", "0.9 0.95 0.001")

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
    # same landscape the UI asks for.  The default is the old flat floor.
    _add_terrain(asset, worldbody, terrain_config)

    # Track mesh assets that need declarations in <asset>: name → (file, [sx, sy, sz])
    mesh_assets: Dict[str, Tuple[str, List[float]]] = {}

    # Recursively add bodies
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

        # Classify contact role: foot > wheel > gripper > default.
        # Wheel/tire contact is data-driven by preset sim_metadata.contact_class.
        contact_class = _contact_class_for_link(link_name)
        is_foot = _is_foot_link(link_name)
        is_wheel = contact_class == "wheel"
        is_gripper = _is_gripper_link(link_name)
        is_imu = _is_imu_link(link_name)

        # Create body element (mesh_assets dict accumulates mesh file declarations)
        body_elem = _create_body_element(
            link_data,
            incoming_joint["elem"] if incoming_joint else None,
            mesh_assets=mesh_assets,
            is_foot=is_foot,
            is_wheel=is_wheel and not is_foot,
            is_gripper=is_gripper and not is_foot and not is_wheel,
            is_imu=is_imu,
        )
        parent_body_elem.append(body_elem)

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
                    joint_elem.set("actuatorfrcrange", f"-{limits['effort']} {limits['effort']}")

            # ── Joint dynamics ────────────────────────────────────────────────
            dyn = incoming_joint.get("dynamics", {})
            is_prismatic = urdf_joint_type == "prismatic"
            effort = incoming_joint["limits"]["effort"] if incoming_joint["limits"] else 10.0

            # Damping (viscous friction).  Use URDF value when provided; otherwise
            # scale with sqrt(effort) so heavier joints settle at a similar rate
            # regardless of stiffness.  Prismatic joints are 10× stiffer by default
            # because linear slides have higher viscous losses.
            urdf_damping = dyn.get("damping")
            if urdf_damping is not None:
                damping = urdf_damping
            elif is_prismatic:
                damping = max(0.5, 1.0 * (effort / 10.0) ** 0.5)
            else:
                damping = max(0.05, 0.1 * (effort / 10.0) ** 0.5)
            joint_elem.set("damping", f"{damping:.6g}")

            # Frictionloss (Coulomb / dry friction).  URDF default is 0, which lets
            # joints drift indefinitely when control is released.  A small non-zero
            # value prevents this without fighting the actuator.
            urdf_friction = dyn.get("friction")
            if urdf_friction is not None and urdf_friction > 1e-9:
                frictionloss = urdf_friction
            elif is_prismatic:
                frictionloss = 0.5   # N — typical linear-slide Coulomb friction
            else:
                frictionloss = 0.02  # Nm — typical revolute dry friction
            joint_elem.set("frictionloss", f"{frictionloss:.6g}")

            # Armature (reflected rotor inertia).  Even a tiny value dramatically
            # stabilises PD control and prevents high-frequency jitter, at negligible
            # computational cost.  Scale conservatively with effort so that a 1 Nm
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
    # contact forces that destabilize the sim.  Non-adjacent self-collision among
    # role-tagged geoms (foot/wheel/gripper) is still on so legs/feet can't tunnel
    # through each other; frame geoms are filtered upstream via contype=0.
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
    # Continuous joints have no angle limits so fall back to raw torque motors.
    for joint in joints:
        if joint["type"] in ("fixed",):
            continue

        effort = 10.0
        if joint["limits"]:
            effort = max(joint["limits"].get("effort", 10.0), 0.01)

        if joint["type"] == "continuous":
            # Unbounded rotation — no position target makes sense, use torque motor
            motor = etree.SubElement(actuators, "motor")
            motor.set("name", f"{joint['name']}_motor")
            motor.set("joint", joint["name"])
            motor.set("ctrllimited", "true")
            motor.set("ctrlrange", f"{-effort} {effort}")
            motor.set("forcerange", f"{-effort} {effort}")
        else:
            # Revolute / prismatic — position-controlled servo
            # kp: position stiffness gain. Scaled with effort so a 10 Nm servo → kp=100.
            # kv: velocity (damping) gain. ~0.1×kp gives reasonable settling without oscillation.
            lower = joint["limits"]["lower"] if joint["limits"] else -3.14159
            upper = joint["limits"]["upper"] if joint["limits"] else 3.14159
            kp = max(effort * 10.0, 50.0)
            kv = max(effort * 1.0, 2.0)
            pos_act = etree.SubElement(actuators, "position")
            pos_act.set("name", f"{joint['name']}_pos")
            pos_act.set("joint", joint["name"])
            pos_act.set("kp", f"{kp:.4f}")
            pos_act.set("kv", f"{kv:.4f}")
            pos_act.set("ctrllimited", "true")
            pos_act.set("ctrlrange", f"{lower:.6f} {upper:.6f}")
            pos_act.set("forcerange", f"{-effort:.4f} {effort:.4f}")

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
        if _is_ee_link(link_name):
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
        if _is_imu_link(link_name):
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

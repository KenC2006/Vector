"""
URDF to MJCF converter.

Converts a URDF file to MJCF XML format with actuators.
MuJoCo can load MJCF directly, and we inject actuators for all non-fixed joints.
"""
from typing import Dict, List, Tuple, Optional, Any
import os
from lxml import etree
import numpy as np


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
        filename = mesh_elem.get("filename", "")
        # Resolve relative paths
        if filename and not os.path.isabs(filename):
            filename = os.path.join(urdf_dir, filename)
        if filename and os.path.exists(filename):
            # Generate stable asset name: strip extension, sanitize
            raw_name = os.path.splitext(os.path.basename(filename))[0]
            mesh_name = "".join(c if c.isalnum() or c == "_" else "_" for c in raw_name)
            return {
                "type": "mesh",
                "filename": filename,
                "mesh_name": mesh_name,
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

# Link-name keywords that identify foot contact surfaces.
_FOOT_KEYWORDS = frozenset(("foot", "toe", "pad", "paw", "tip", "sole"))


def _is_foot_link(link_name: str) -> bool:
    """Heuristic: does this link name suggest a foot / ground-contact surface?"""
    lower = link_name.lower()
    return any(kw in lower for kw in _FOOT_KEYWORDS)


# ── Inertia helpers ───────────────────────────────────────────────────────────

# Default density for robot parts with unknown material (kg/m³).
# ~1200 is typical ABS/PLA plastic; aluminium extrusion would be ~2700.
_PLASTIC_DENSITY = 1200.0


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
) -> etree._Element:
    """Create a MuJoCo body element for a URDF link."""

    body = etree.Element("body")
    body.set("name", link_data["name"])

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
    mass = link_data["mass"]
    explicit_inertia = link_data.get("explicit_inertia")

    # Auto-estimate mass from geometry volume if not given in the URDF
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
        elif collision_list:
            # Compute shape-based inertia from all collision primitives
            I_com, com_pos = _compute_inertia_from_collision_list(collision_list, mass)
            px, py, pz = com_pos
            inertial.set("pos", f"{px:.6g} {py:.6g} {pz:.6g}")
            ixx, iyy, izz = I_com[0, 0], I_com[1, 1], I_com[2, 2]
            ixy, ixz, iyz = I_com[0, 1], I_com[0, 2], I_com[1, 2]
            # Use fullinertia only when off-diagonal terms are non-negligible
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

    # ── Collision geoms — one <geom> per collision primitive ───────────────────
    for coll in collision_list:
        geom = coll["geometry"]
        ox, oy, oz = coll["origin_xyz"]
        orpy = coll["origin_rpy"]

        geom_elem = etree.SubElement(body, "geom")
        geom_type = geom.get("type", "box")
        geom_elem.set("type", geom_type)
        geom_elem.set("material", "MatGray")
        # Foot geoms inherit the "foot" default class (higher torsional friction)
        if is_foot:
            geom_elem.set("class", "foot")

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
            if filename and mesh_name:
                if mesh_assets is not None:
                    mesh_assets[mesh_name] = filename
                geom_elem.set("mesh", mesh_name)

    return body


def urdf_to_mjcf(urdf_path: str, free_base: bool = False) -> str:
    """
    Convert a URDF file to MJCF XML string.

    Args:
        urdf_path: Path to the URDF file.
        free_base: If True, add a <freejoint/> to the root body so it is
                   free-floating (useful for mobile robots and UAVs).

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

        # Read URDF <dynamics> for damping and friction
        dynamics = {"damping": 0.1, "friction": 0.0}
        dynamics_elem = joint_elem.find("dynamics")
        if dynamics_elem is not None:
            try:
                dynamics["damping"] = float(dynamics_elem.get("damping", "0.1"))
                dynamics["friction"] = float(dynamics_elem.get("friction", "0.0"))
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
    root_link = None
    for link_name in links:
        if link_name not in child_links:
            root_link = link_name
            break

    if root_link is None and links:
        root_link = list(links.keys())[0]

    # Build MJCF
    mjcf_root = etree.Element("mujoco")
    mjcf_root.set("model", robot_name)

    # Add option
    option = etree.SubElement(mjcf_root, "option")
    option.set("timestep", "0.002")
    option.set("gravity", "0 0 -9.81")

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
    foot_geom.set("friction", "1.5 0.1 0.01")
    foot_geom.set("condim", "6")
    foot_geom.set("solref", "0.005 1")
    foot_geom.set("solimp", "0.9 0.95 0.001")

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

    # Default ground plane (can be toggled off later)
    floor_geom = etree.SubElement(worldbody, "geom")
    floor_geom.set("name", "floor")
    floor_geom.set("type", "plane")
    floor_geom.set("size", "0 0 0.05")
    floor_geom.set("rgba", "0.5 0.5 0.5 1")
    # Explicit floor friction: high lateral (1.5), moderate torsional/rolling
    # condim=6 on floor so it can provide torsional reaction to foot condim=6
    floor_geom.set("friction", "1.5 0.1 0.01")
    floor_geom.set("condim", "6")

    # Track mesh assets that need declarations in <asset>
    mesh_assets: Dict[str, str] = {}

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

        # Create body element (mesh_assets dict accumulates mesh file declarations)
        body_elem = _create_body_element(
            link_data,
            incoming_joint["elem"] if incoming_joint else None,
            mesh_assets=mesh_assets,
            is_foot=_is_foot_link(link_name),
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

            # Damping and frictionloss from URDF <dynamics> tag
            dyn = incoming_joint.get("dynamics", {})
            damping = dyn.get("damping", 0.1)
            friction = dyn.get("friction", 0.0)
            joint_elem.set("damping", str(damping))
            if friction > 0:
                joint_elem.set("frictionloss", str(friction))

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
    for mesh_name, mesh_file in mesh_assets.items():
        mesh_decl = etree.SubElement(asset, "mesh")
        mesh_decl.set("name", mesh_name)
        mesh_decl.set("file", mesh_file)

    # Self-collision excludes: suppress contacts between every adjacent body pair.
    # Touching neighbors (parent/child across a joint) almost always cause spurious
    # contact forces that destabilize the sim.  Non-adjacent self-collision is left
    # ON intentionally — that's what prevents legs from passing through each other.
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

    # Convert to string
    xml_string = etree.tostring(
        mjcf_root,
        encoding="unicode",
        pretty_print=True,
    )

    return xml_string

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
    inertia_elem = link_elem.find(".//inertia")
    if inertia_elem is not None:
        mass_elem = link_elem.find(".//mass")
        if mass_elem is not None:
            try:
                mass = float(mass_elem.get("value", 0.0))
            except (ValueError, TypeError):
                mass = 0.0

    # Get collision geometry
    collision_geom = None
    collision_elem = link_elem.find(".//collision/geometry")
    if collision_elem is not None:
        collision_geom = _extract_geometry(collision_elem, urdf_dir)

    # Fall back to visual geometry if no collision
    if collision_geom is None:
        visual_elem = link_elem.find(".//visual/geometry")
        if visual_elem is not None:
            collision_geom = _extract_geometry(visual_elem, urdf_dir)

    # Get origin (position/rotation) of collision geometry
    origin_xyz = [0.0, 0.0, 0.0]
    origin_rpy = [0.0, 0.0, 0.0]
    origin_elem = link_elem.find(".//collision/origin")
    if origin_elem is not None:
        xyz_str = origin_elem.get("xyz", "0 0 0")
        rpy_str = origin_elem.get("rpy", "0 0 0")
        try:
            origin_xyz = [float(x) for x in xyz_str.split()]
        except (ValueError, TypeError):
            origin_xyz = [0.0, 0.0, 0.0]
        try:
            origin_rpy = [float(x) for x in rpy_str.split()]
        except (ValueError, TypeError):
            origin_rpy = [0.0, 0.0, 0.0]

    return {
        "name": name,
        "mass": mass,
        "collision_geometry": collision_geom,
        "origin_xyz": origin_xyz,
        "origin_rpy": origin_rpy,
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
            return {
                "type": "mesh",
                "filename": filename,
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


def _create_body_element(
    link_data: Dict[str, Any],
    joint_elem: Optional[etree._Element] = None,
    parent_elem: Optional[etree._Element] = None,
) -> etree._Element:
    """Create a body element for a link."""

    body = etree.Element("body")
    body.set("name", link_data["name"])

    # Set position if joint is provided
    if joint_elem is not None:
        origin_elem = joint_elem.find("origin")
        if origin_elem is not None:
            xyz_str = origin_elem.get("xyz", "0 0 0")
            rpy_str = origin_elem.get("rpy", "0 0 0")
            try:
                pos = [float(x) for x in xyz_str.split()]
                body.set("pos", " ".join(str(x) for x in pos))
            except (ValueError, TypeError):
                pass

            try:
                rpy = [float(x) for x in rpy_str.split()]
                quat = _rpy_to_quat(rpy[0], rpy[1], rpy[2])
                body.set("quat", " ".join(str(x) for x in quat))
            except (ValueError, TypeError):
                pass

    # Add geometry (inertial + collision)
    mass = link_data["mass"]
    # Auto-fix: if a body has collision geometry but zero/tiny mass, assign a minimal mass
    # so MuJoCo doesn't complain about non-positive-definite inertia
    if mass < 0.001 and link_data["collision_geometry"] is not None:
        mass = 0.01  # 10g default for massless links with geometry
    if mass > 0.001:
        inertial = etree.SubElement(body, "inertial")
        inertial.set("mass", str(mass))
        inertial.set("pos", "0 0 0")  # MuJoCo requires pos for inertial
        # Simplified isotropic inertia: I = 0.001 * m (reasonable for ~cm-scale bodies)
        diag = max(1e-6 * mass, 0.001 * mass)
        inertial.set("diaginertia", f"{diag} {diag} {diag}")

    # Add collision geometry
    if link_data["collision_geometry"]:
        geom = link_data["collision_geometry"]
        geom_elem = etree.SubElement(body, "geom")

        geom_type = geom.get("type", "box")
        geom_elem.set("type", geom_type)
        geom_elem.set("material", "MatGray")

        if geom_type == "box":
            size = geom.get("size", [1, 1, 1])
            # MuJoCo halfsize is half of box dimensions
            halfsize = [s / 2.0 for s in size]
            geom_elem.set("size", " ".join(str(x) for x in halfsize))

        elif geom_type == "cylinder":
            radius = geom.get("radius", 0.1)
            length = geom.get("length", 1.0)
            geom_elem.set("size", f"{radius} {length / 2.0}")
            geom_elem.set("fromto", f"0 0 {-length/2.0} 0 0 {length/2.0}")

        elif geom_type == "sphere":
            radius = geom.get("radius", 0.1)
            geom_elem.set("size", str(radius))

        elif geom_type == "mesh":
            filename = geom.get("filename", "")
            if filename:
                geom_elem.set("mesh", filename)

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

    # Add visual settings
    visual = etree.SubElement(mjcf_root, "visual")
    global_elem = etree.SubElement(visual, "global")
    global_elem.set("offheight", "2400")
    global_elem.set("offwidth", "2400")

    # Add asset (materials)
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

        # Create body element
        body_elem = _create_body_element(link_data, incoming_joint["elem"] if incoming_joint else None)
        parent_body_elem.append(body_elem)

        # Add joint element if incoming joint exists and is not fixed
        if incoming_joint and incoming_joint["type"] != "fixed":
            joint_elem = etree.SubElement(body_elem, "joint")
            joint_elem.set("name", incoming_joint["name"])

            # Convert URDF joint type to MuJoCo joint type
            urdf_joint_type = incoming_joint["type"]
            mjcf_joint_type = "hinge" if urdf_joint_type == "revolute" else "slide" if urdf_joint_type == "prismatic" else "ball"
            joint_elem.set("type", mjcf_joint_type)

            # Set axis
            axis = incoming_joint["axis"]
            joint_elem.set("axis", " ".join(str(x) for x in axis))

            # Set limits
            if incoming_joint["limits"]:
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
        if free_base and worldbody:
            root_bodies = [c for c in worldbody if c.tag == "body"]
            if root_bodies:
                free_joint = etree.Element("freejoint")
                free_joint.set("name", "base_freejoint")
                root_bodies[0].insert(0, free_joint)

    # Add actuators section
    actuators = etree.SubElement(mjcf_root, "actuator")

    # Add motors for all non-fixed joints
    for joint in joints:
        if joint["type"] not in ("fixed",):
            motor = etree.SubElement(actuators, "motor")
            motor.set("name", f"{joint['name']}_motor")
            motor.set("joint", joint["name"])
            motor.set("ctrllimited", "true")

            effort = 10.0
            if joint["limits"]:
                effort = joint["limits"].get("effort", 10.0)
                # Clamp to a sensible non-zero range so MuJoCo doesn't reject it
                effort = max(effort, 0.01)
            motor.set("ctrlrange", f"{-effort} {effort}")
            motor.set("forcerange", f"{-effort} {effort}")

    # Convert to string
    xml_string = etree.tostring(
        mjcf_root,
        encoding="unicode",
        pretty_print=True,
    )

    return xml_string

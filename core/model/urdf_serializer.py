"""
URDF serializer: converts a KinematicGraph back to URDF XML.
"""
from typing import Optional
import xml.etree.ElementTree as ET
from .kinematic_graph import KinematicGraph


def serialize_to_urdf(kg: KinematicGraph) -> str:
    """
    Serialize a KinematicGraph to valid URDF XML string.

    Args:
        kg: The KinematicGraph to serialize.

    Returns:
        Valid URDF XML as a string (well-indented).

    Raises:
        ValueError: If the graph is invalid.
    """
    if not kg.get_links():
        raise ValueError("Cannot serialize empty kinematic graph")

    # Create root robot element
    robot = ET.Element("robot")
    robot.set("name", "robot")

    # Serialize all links
    for link_name in kg.get_links():
        link_data = kg.get_link_data(link_name)
        if link_data is None:
            continue

        link_elem = ET.SubElement(robot, "link")
        link_elem.set("name", link_data.name)

        # Add inertial element if mass > 0 or inertia exists
        if link_data.mass > 0.0 or link_data.inertia is not None:
            inertial_elem = ET.SubElement(link_elem, "inertial")

            # Mass
            mass_elem = ET.SubElement(inertial_elem, "mass")
            mass_elem.set("value", str(link_data.mass))

            # Inertia (with reasonable defaults if not specified)
            inertia_elem = ET.SubElement(inertial_elem, "inertia")
            if link_data.inertia is not None:
                inertia_elem.set("ixx", str(link_data.inertia.ixx))
                inertia_elem.set("ixy", str(link_data.inertia.ixy))
                inertia_elem.set("ixz", str(link_data.inertia.ixz))
                inertia_elem.set("iyy", str(link_data.inertia.iyy))
                inertia_elem.set("iyz", str(link_data.inertia.iyz))
                inertia_elem.set("izz", str(link_data.inertia.izz))
            else:
                # Default inertia (small cube)
                default_i = 0.001
                inertia_elem.set("ixx", str(default_i))
                inertia_elem.set("ixy", "0")
                inertia_elem.set("ixz", "0")
                inertia_elem.set("iyy", str(default_i))
                inertia_elem.set("iyz", "0")
                inertia_elem.set("izz", str(default_i))

            # Origin for inertial (default to 0,0,0)
            origin_elem = ET.SubElement(inertial_elem, "origin")
            origin_elem.set("xyz", "0 0 0")
            origin_elem.set("rpy", "0 0 0")

        # Add visual element
        if link_data.visual_geometry is not None or link_data.visual_mesh is not None:
            visual_elem = ET.SubElement(link_elem, "visual")

            # Origin for visual
            if link_data.visual_origin is not None:
                origin_elem = ET.SubElement(visual_elem, "origin")
                xyz = link_data.visual_origin.get("xyz", [0, 0, 0])
                rpy = link_data.visual_origin.get("rpy", [0, 0, 0])
                origin_elem.set("xyz", " ".join(str(v) for v in xyz))
                origin_elem.set("rpy", " ".join(str(v) for v in rpy))

            # Geometry
            geometry_elem = ET.SubElement(visual_elem, "geometry")
            if link_data.visual_geometry is not None:
                _add_geometry_element(geometry_elem, link_data.visual_geometry)
            elif link_data.visual_mesh is not None:
                # Fall back to mesh if visual_geometry not set
                mesh_elem = ET.SubElement(geometry_elem, "mesh")
                mesh_elem.set("filename", link_data.visual_mesh)

            # Material
            if link_data.material is not None:
                material_elem = ET.SubElement(visual_elem, "material")
                material_name = link_data.material.get("name", "default")
                material_elem.set("name", material_name)
                if "color" in link_data.material:
                    color = link_data.material["color"]
                    color_elem = ET.SubElement(material_elem, "color")
                    color_elem.set("rgba", " ".join(str(c) for c in color))

        # Add collision element
        if link_data.collision_geometry is not None:
            collision_elem = ET.SubElement(link_elem, "collision")

            # Origin for collision
            if link_data.collision_origin is not None:
                origin_elem = ET.SubElement(collision_elem, "origin")
                xyz = link_data.collision_origin.get("xyz", [0, 0, 0])
                rpy = link_data.collision_origin.get("rpy", [0, 0, 0])
                origin_elem.set("xyz", " ".join(str(v) for v in xyz))
                origin_elem.set("rpy", " ".join(str(v) for v in rpy))
            else:
                # Default origin
                origin_elem = ET.SubElement(collision_elem, "origin")
                origin_elem.set("xyz", "0 0 0")
                origin_elem.set("rpy", "0 0 0")

            # Geometry
            geometry_elem = ET.SubElement(collision_elem, "geometry")
            _add_geometry_element(geometry_elem, link_data.collision_geometry)

    # Serialize all joints
    for u, v in kg.graph.edges():
        joint_data = kg.graph[u][v]["data"]

        joint_elem = ET.SubElement(robot, "joint")
        joint_elem.set("name", joint_data.name)
        joint_elem.set("type", joint_data.joint_type)

        # Parent and child
        parent_elem = ET.SubElement(joint_elem, "parent")
        parent_elem.set("link", joint_data.parent_link)

        child_elem = ET.SubElement(joint_elem, "child")
        child_elem.set("link", joint_data.child_link)

        # Origin
        origin_elem = ET.SubElement(joint_elem, "origin")
        origin_elem.set("xyz", " ".join(str(v) for v in joint_data.origin_xyz))
        origin_elem.set("rpy", " ".join(str(v) for v in joint_data.origin_rpy))

        # Axis
        axis_elem = ET.SubElement(joint_elem, "axis")
        axis_elem.set("xyz", " ".join(str(v) for v in joint_data.axis))

        # Limits (for non-fixed joints)
        if joint_data.joint_type != "fixed" and joint_data.limits is not None:
            limits_elem = ET.SubElement(joint_elem, "limit")
            limits_elem.set("lower", str(joint_data.limits.lower))
            limits_elem.set("upper", str(joint_data.limits.upper))
            if joint_data.limits.effort is not None:
                limits_elem.set("effort", str(joint_data.limits.effort))
            if joint_data.limits.velocity is not None:
                limits_elem.set("velocity", str(joint_data.limits.velocity))

        # Dynamics (damping, friction)
        if joint_data.dynamics is not None:
            dynamics_elem = ET.SubElement(joint_elem, "dynamics")
            if "damping" in joint_data.dynamics:
                dynamics_elem.set("damping", str(joint_data.dynamics["damping"]))
            if "friction" in joint_data.dynamics:
                dynamics_elem.set("friction", str(joint_data.dynamics["friction"]))

    # Convert to string with indentation
    _indent(robot)
    return ET.tostring(robot, encoding="unicode", method="xml")


def _add_geometry_element(parent: ET.Element, geometry_dict: dict) -> None:
    """Add a geometry subelement from a geometry dict."""
    geom_type = geometry_dict.get("type", "box")
    params = geometry_dict.get("params", {})

    if geom_type == "box":
        box_elem = ET.SubElement(parent, "box")
        size = params.get("size", [1, 1, 1])
        box_elem.set("size", " ".join(str(s) for s in size))

    elif geom_type == "cylinder":
        cyl_elem = ET.SubElement(parent, "cylinder")
        cyl_elem.set("radius", str(params.get("radius", 0.1)))
        cyl_elem.set("length", str(params.get("length", 1.0)))

    elif geom_type == "sphere":
        sphere_elem = ET.SubElement(parent, "sphere")
        sphere_elem.set("radius", str(params.get("radius", 0.1)))

    elif geom_type == "mesh":
        mesh_elem = ET.SubElement(parent, "mesh")
        mesh_elem.set("filename", str(params.get("filename", "")))
        if "scale" in params:
            scale = params["scale"]
            mesh_elem.set("scale", " ".join(str(s) for s in scale))


def _indent(elem: ET.Element, level: int = 0) -> None:
    """Add indentation to an ElementTree for pretty-printing."""
    indent_str = "\n" + ("  " * level)
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = indent_str + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = indent_str
        for child in elem:
            _indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = indent_str
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = indent_str

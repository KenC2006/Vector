"""
URDF parser to build a kinematic graph.

Two entry points with deliberately different dependencies:
  - parse_urdf(path)         — file-based, uses yourdfpy (also feeds the sim path).
  - parse_urdf_string(xml)   — string-based, pure-stdlib (xml.etree). Used by the
                               editor Validation tab and AI post-generation checks.
                               Keeping it dependency-light means validation works
                               even when yourdfpy isn't installed in the spawned
                               interpreter, and it validates the user's raw XML
                               directly rather than a library-normalized version.
"""
from typing import Optional, Dict, Tuple, Any, List
import os
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
from .kinematic_graph import KinematicGraph
from .types import LinkData, JointData, Inertia, Limits


def _extract_inertia(inertia_elem: Any) -> Optional[Inertia]:
    """Extract inertia from a yourdfpy inertia element (3x3 matrix)."""
    try:
        if inertia_elem is None:
            return None
        # yourdfpy represents inertia as a 3x3 numpy array
        # Convert matrix to URDF format: ixx, ixy, ixz, iyy, iyz, izz
        inertia_matrix = inertia_elem
        return Inertia(
            ixx=float(inertia_matrix[0, 0]),
            ixy=float(inertia_matrix[0, 1]),
            ixz=float(inertia_matrix[0, 2]),
            iyy=float(inertia_matrix[1, 1]),
            iyz=float(inertia_matrix[1, 2]),
            izz=float(inertia_matrix[2, 2]),
        )
    except (AttributeError, ValueError, TypeError, IndexError):
        return None


def _extract_geometry(geom_elem: Any) -> Optional[Dict[str, Any]]:
    """Extract geometry from a yourdfpy geometry element."""
    try:
        if geom_elem is None:
            return None

        # Check for box
        if hasattr(geom_elem, "box") and geom_elem.box is not None:
            size = getattr(geom_elem.box, "size", [1, 1, 1])
            try:
                size_list = list(size) if hasattr(size, "__iter__") else [1, 1, 1]
            except (TypeError, ValueError):
                size_list = [1, 1, 1]
            return {
                "type": "box",
                "params": {
                    "size": size_list
                }
            }

        # Check for cylinder
        if hasattr(geom_elem, "cylinder") and geom_elem.cylinder is not None:
            cylinder = geom_elem.cylinder
            radius = float(getattr(cylinder, "radius", 0.1))
            length = float(getattr(cylinder, "length", 1.0))
            return {
                "type": "cylinder",
                "params": {
                    "radius": radius,
                    "length": length,
                }
            }

        # Check for sphere
        if hasattr(geom_elem, "sphere") and geom_elem.sphere is not None:
            sphere = geom_elem.sphere
            radius = float(getattr(sphere, "radius", 0.1))
            return {
                "type": "sphere",
                "params": {
                    "radius": radius,
                }
            }

        # Check for mesh
        if hasattr(geom_elem, "mesh") and geom_elem.mesh is not None:
            filename = getattr(geom_elem.mesh, "filename", "")
            return {
                "type": "mesh",
                "params": {
                    "filename": str(filename),
                }
            }

        return None
    except (AttributeError, ValueError, TypeError):
        return None


def _extract_mesh_path(visual_elem: Any) -> Optional[str]:
    """Extract mesh filename from a visual element."""
    try:
        if visual_elem is None:
            return None
        if hasattr(visual_elem, "geometry") and visual_elem.geometry is not None:
            geom = visual_elem.geometry
            if hasattr(geom, "mesh") and geom.mesh is not None:
                return str(getattr(geom.mesh, "filename", ""))
        return None
    except (AttributeError, ValueError, TypeError):
        return None


def _extract_origin(origin_elem: Any) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """
    Extract origin (position and rotation) from an element.
    Returns (xyz, rpy) tuples.
    """
    try:
        xyz = (0.0, 0.0, 0.0)
        rpy = (0.0, 0.0, 0.0)

        if origin_elem is not None:
            xyz_val = getattr(origin_elem, "xyz", None)
            if xyz_val is not None:
                xyz = tuple(float(x) for x in xyz_val)

            rpy_val = getattr(origin_elem, "rpy", None)
            if rpy_val is not None:
                rpy = tuple(float(x) for x in rpy_val)

        return xyz, rpy
    except (AttributeError, ValueError, TypeError):
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)


def _matrix_to_rpy(matrix: Any) -> Tuple[float, float, float]:
    """
    Convert a 4x4 transformation matrix rotation part to roll, pitch, yaw (ZYX Euler angles).
    Returns (roll, pitch, yaw) in radians.
    """
    try:
        if matrix is None:
            return (0.0, 0.0, 0.0)

        # Extract 3x3 rotation matrix from 4x4 transformation
        R = matrix[:3, :3]

        # Use numpy to convert rotation matrix to Euler angles (ZYX)
        # sin(pitch) = -R[2, 0]
        sin_pitch = -R[2, 0]
        sin_pitch = np.clip(sin_pitch, -1.0, 1.0)

        pitch = np.arcsin(sin_pitch)

        # cos(pitch) calculations
        cos_pitch = np.cos(pitch)

        if abs(cos_pitch) > 1e-6:
            roll = np.arctan2(R[2, 1] / cos_pitch, R[2, 2] / cos_pitch)
            yaw = np.arctan2(R[1, 0] / cos_pitch, R[0, 0] / cos_pitch)
        else:
            # Gimbal lock case
            roll = 0.0
            yaw = np.arctan2(-R[0, 1], R[1, 1])

        return (float(roll), float(pitch), float(yaw))
    except (AttributeError, ValueError, TypeError, IndexError):
        return (0.0, 0.0, 0.0)


def _axis_in_parent_frame(
    axis: Tuple[float, float, float],
    rpy: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Rotate a URDF joint-frame axis into the parent frame using origin RPY."""
    try:
        roll, pitch, yaw = rpy
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
        out = (rz @ ry @ rx) @ np.array(axis, dtype=float)
        norm = float(np.linalg.norm(out))
        if norm > 1e-9:
            out = out / norm
        return (float(out[0]), float(out[1]), float(out[2]))
    except (ValueError, TypeError, IndexError):
        return axis


def _reconstitute_servo_splits(kg: KinematicGraph) -> None:
    """
    Merge split servo pairs back into single logical nodes.

    The URDF emitter writes each servo as:
      real_parent --(fixed, X_mount)--> X_body --(revolute, X)--> X_horn --> downstream

    This post-pass detects those patterns and collapses them into:
      real_parent --(revolute, X)--> X --> downstream

    Operates in-place on the KinematicGraph's underlying networkx DiGraph.
    """
    g = kg.graph

    # Collect mount joints: fixed joints whose name ends with '_mount'
    mount_edges = [
        (u, v, data)
        for u, v in list(g.edges())
        for data in [g[u][v]["data"]]
        if data.joint_type == "fixed" and data.name.endswith("_mount")
    ]

    for real_parent, body_link_name, mount_joint in mount_edges:
        if not body_link_name.endswith("_body"):
            continue

        base_link_name = body_link_name[: -len("_body")]
        horn_link_name = base_link_name + "_horn"

        if body_link_name not in g or horn_link_name not in g:
            continue

        # Find the revolute joint: body_link → horn_link
        if not g.has_edge(body_link_name, horn_link_name):
            continue
        revolute_joint: JointData = g[body_link_name][horn_link_name]["data"]
        if revolute_joint.joint_type == "fixed":
            continue

        body_data: LinkData = g.nodes[body_link_name]["data"]
        horn_data: LinkData = g.nodes[horn_link_name]["data"]

        # Merged link: sum mass, keep body inertia (horn is ~5%)
        merged_link = LinkData(
            name=base_link_name,
            mass=body_data.mass + horn_data.mass,
            inertia=body_data.inertia,
            visual_mesh=body_data.visual_mesh,
            visual_geometry=body_data.visual_geometry,
            visual_origin=body_data.visual_origin,
            material=body_data.material,
            collision_geometry=body_data.collision_geometry,
            collision_origin=body_data.collision_origin,
        )

        # Merged joint: revolute properties, but placement from the mount joint
        merged_joint = JointData(
            name=revolute_joint.name,
            joint_type=revolute_joint.joint_type,
            parent_link=real_parent,
            child_link=base_link_name,
            axis=_axis_in_parent_frame(revolute_joint.axis, mount_joint.origin_rpy),
            origin_xyz=mount_joint.origin_xyz,
            origin_rpy=mount_joint.origin_rpy,
            limits=revolute_joint.limits,
            dynamics=revolute_joint.dynamics,
        )

        # Preserve downstream edges (horn → children)
        downstream = [
            (child, g[horn_link_name][child]["data"])
            for child in list(g.successors(horn_link_name))
        ]

        # Patch graph: add merged node + edges, remove phantom nodes
        g.add_node(base_link_name, data=merged_link)
        g.add_edge(real_parent, base_link_name, data=merged_joint)
        for child_name, child_joint_data in downstream:
            # Update child joint's parent_link to point at base_link_name
            child_joint_data = JointData(
                name=child_joint_data.name,
                joint_type=child_joint_data.joint_type,
                parent_link=base_link_name,
                child_link=child_joint_data.child_link,
                axis=child_joint_data.axis,
                origin_xyz=child_joint_data.origin_xyz,
                origin_rpy=child_joint_data.origin_rpy,
                limits=child_joint_data.limits,
                dynamics=child_joint_data.dynamics,
            )
            g.add_edge(base_link_name, child_name, data=child_joint_data)

        # Remove body and horn (also removes all their edges)
        g.remove_node(body_link_name)
        g.remove_node(horn_link_name)

        # Update root if it somehow pointed at body (shouldn't happen, but guard)
        if kg.root_link == body_link_name:
            kg.root_link = base_link_name


def parse_urdf(file_path: str) -> KinematicGraph:
    """
    Parse a URDF file and build a kinematic graph.

    Args:
        file_path: Path to the URDF file.

    Returns:
        KinematicGraph: The parsed robot model.

    Raises:
        FileNotFoundError: If the file doesn't exist.
        ValueError: If the URDF is invalid.
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"URDF file not found: {file_path}")

    try:
        import yourdfpy
    except ImportError:
        raise ImportError("yourdfpy not installed. Run: pip install yourdfpy")

    # Parse the URDF
    try:
        urdf = yourdfpy.URDF.load(file_path)
    except Exception as e:
        raise ValueError(f"Failed to parse URDF: {e}")

    # Create kinematic graph
    kg = KinematicGraph()

    # Add all links
    if not urdf.link_map:
        raise ValueError("URDF has no links")

    for link_name, link in urdf.link_map.items():
        # Extract inertia
        inertia = None
        mass = 0.0
        if link.inertial is not None:
            inertial = link.inertial
            mass = float(getattr(inertial, "mass", 0.0))
            inertia = _extract_inertia(getattr(inertial, "inertia", None))

        # Extract visual geometry and origin
        visual_geom = None
        visual_mesh = None
        visual_ori = None
        material = None
        if link.visuals and len(link.visuals) > 0:
            visual = link.visuals[0]
            visual_geom = _extract_geometry(visual.geometry)
            visual_mesh = _extract_mesh_path(visual)

            # Extract visual origin
            if hasattr(visual, "origin") and visual.origin is not None:
                xyz, rpy = _extract_origin(visual.origin)
                visual_ori = {"xyz": list(xyz), "rpy": list(rpy)}

            # Extract material
            if hasattr(visual, "material") and visual.material is not None:
                mat = visual.material
                mat_dict = {"name": getattr(mat, "name", "default")}
                if hasattr(mat, "color"):
                    try:
                        color = getattr(mat, "color", None)
                        if color is not None:
                            mat_dict["color"] = list(color) if hasattr(color, "__iter__") else [0.8, 0.8, 0.8, 1.0]
                    except (TypeError, ValueError):
                        pass
                material = mat_dict

        # Extract collision geometry and origin
        collision_geom = None
        collision_ori = None
        if link.collisions and len(link.collisions) > 0:
            collision = link.collisions[0]
            collision_geom = _extract_geometry(collision.geometry)

            # Extract collision origin
            if hasattr(collision, "origin") and collision.origin is not None:
                xyz, rpy = _extract_origin(collision.origin)
                collision_ori = {"xyz": list(xyz), "rpy": list(rpy)}

        link_data = LinkData(
            name=link.name,
            mass=mass,
            inertia=inertia,
            visual_mesh=visual_mesh,
            visual_geometry=visual_geom,
            visual_origin=visual_ori,
            material=material,
            collision_geometry=collision_geom,
            collision_origin=collision_ori,
        )
        kg.add_link(link_data)

    # Set root link (base link)
    if urdf.base_link:
        # urdf.base_link is a string (link name)
        kg.set_root_link(urdf.base_link)
    elif urdf.link_map:
        kg.set_root_link(list(urdf.link_map.keys())[0])

    # Add all joints
    if not urdf.joint_map:
        # No joints is valid (single-link robot)
        return kg

    for joint_name, joint in urdf.joint_map.items():
        joint_type = str(joint.type)
        parent_name = str(joint.parent)
        child_name = str(joint.child)

        if not parent_name or not child_name:
            continue

        # Extract axis (yourdfpy provides axis as numpy array)
        axis = (0.0, 0.0, 1.0)
        if joint.axis is not None:
            try:
                axis = tuple(float(x) for x in joint.axis)
            except (ValueError, TypeError):
                axis = (0.0, 0.0, 1.0)

        # Extract origin from transformation matrix
        origin_xyz = (0.0, 0.0, 0.0)
        origin_rpy = (0.0, 0.0, 0.0)
        if joint.origin is not None:
            try:
                # origin is a 4x4 transformation matrix
                origin_xyz = tuple(float(x) for x in joint.origin[:3, 3])
                # Decompose rotation matrix to RPY
                origin_rpy = _matrix_to_rpy(joint.origin)
            except (ValueError, TypeError, IndexError):
                pass

        # Extract limits
        limits = None
        if joint.limit is not None:
            limit_obj = joint.limit
            limits = Limits(
                lower=float(limit_obj.lower) if hasattr(limit_obj, "lower") else 0.0,
                upper=float(limit_obj.upper) if hasattr(limit_obj, "upper") else 0.0,
                effort=float(limit_obj.effort) if hasattr(limit_obj, "effort") and limit_obj.effort else None,
                velocity=float(limit_obj.velocity) if hasattr(limit_obj, "velocity") and limit_obj.velocity else None,
            )

        # Extract dynamics
        dynamics = None
        if joint.dynamics is not None:
            dyn_obj = joint.dynamics
            dynamics = {
                "damping": float(dyn_obj.damping) if hasattr(dyn_obj, "damping") and dyn_obj.damping else 0.0,
                "friction": float(dyn_obj.friction) if hasattr(dyn_obj, "friction") and dyn_obj.friction else 0.0,
            }

        joint_data = JointData(
            name=joint.name,
            joint_type=joint_type,
            parent_link=parent_name,
            child_link=child_name,
            axis=axis,
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            limits=limits,
            dynamics=dynamics,
        )
        kg.add_joint(joint_data)

    _reconstitute_servo_splits(kg)
    return kg


# ── Stdlib URDF parsing (string path) ─────────────────────────────────────────

def _floats(text: Optional[str], n: int, default: Tuple[float, ...]) -> Tuple[float, ...]:
    """Parse a whitespace-separated float list (URDF's xyz/rpy/rgba/size form)."""
    if text:
        try:
            parts = [float(x) for x in text.split()]
            if len(parts) >= n:
                return tuple(parts[:n])
        except (ValueError, AttributeError):
            pass
    return default


def _attr_float(elem: Optional[ET.Element], key: str, default: Optional[float]) -> Optional[float]:
    if elem is None:
        return default
    raw = elem.get(key)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _geom_from_geometry(geometry: Optional[ET.Element]) -> Optional[Dict[str, Any]]:
    """Convert a <geometry> element into the project's geometry dict, or None."""
    if geometry is None:
        return None
    box = geometry.find("box")
    if box is not None:
        return {"type": "box", "params": {"size": list(_floats(box.get("size"), 3, (1.0, 1.0, 1.0)))}}
    cyl = geometry.find("cylinder")
    if cyl is not None:
        return {"type": "cylinder", "params": {
            "radius": _attr_float(cyl, "radius", 0.1),
            "length": _attr_float(cyl, "length", 1.0),
        }}
    sph = geometry.find("sphere")
    if sph is not None:
        return {"type": "sphere", "params": {"radius": _attr_float(sph, "radius", 0.1)}}
    mesh = geometry.find("mesh")
    if mesh is not None:
        return {"type": "mesh", "params": {"filename": mesh.get("filename", "")}}
    return None


def _origin_of(parent: Optional[ET.Element]) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Read the <origin xyz rpy/> child of `parent`. Returns ((x,y,z),(r,p,y))."""
    zero = (0.0, 0.0, 0.0)
    if parent is None:
        return zero, zero
    origin = parent.find("origin")
    if origin is None:
        return zero, zero
    return _floats(origin.get("xyz"), 3, zero), _floats(origin.get("rpy"), 3, zero)


def _link_from_element(link: ET.Element) -> Optional[LinkData]:
    name = link.get("name")
    if not name:
        return None

    mass = 0.0
    inertia: Optional[Inertia] = None
    inertial = link.find("inertial")
    if inertial is not None:
        mass = _attr_float(inertial.find("mass"), "value", 0.0) or 0.0
        in_el = inertial.find("inertia")
        if in_el is not None:
            inertia = Inertia(
                ixx=_attr_float(in_el, "ixx", 0.0) or 0.0,
                ixy=_attr_float(in_el, "ixy", 0.0) or 0.0,
                ixz=_attr_float(in_el, "ixz", 0.0) or 0.0,
                iyy=_attr_float(in_el, "iyy", 0.0) or 0.0,
                iyz=_attr_float(in_el, "iyz", 0.0) or 0.0,
                izz=_attr_float(in_el, "izz", 0.0) or 0.0,
            )

    # First <visual> only — matches the yourdfpy-backed parse_urdf behavior.
    visual_geom = visual_mesh = visual_ori = material = None
    visual = link.find("visual")
    if visual is not None:
        visual_geom = _geom_from_geometry(visual.find("geometry"))
        if visual_geom and visual_geom["type"] == "mesh":
            visual_mesh = visual_geom["params"].get("filename")
        xyz, rpy = _origin_of(visual)
        visual_ori = {"xyz": list(xyz), "rpy": list(rpy)}
        mat = visual.find("material")
        if mat is not None:
            mat_dict: Dict[str, Any] = {"name": mat.get("name", "default")}
            color = mat.find("color")
            if color is not None:
                mat_dict["color"] = list(_floats(color.get("rgba"), 4, (0.8, 0.8, 0.8, 1.0)))
            material = mat_dict

    collision_geom = collision_ori = None
    collision = link.find("collision")
    if collision is not None:
        collision_geom = _geom_from_geometry(collision.find("geometry"))
        xyz, rpy = _origin_of(collision)
        collision_ori = {"xyz": list(xyz), "rpy": list(rpy)}

    return LinkData(
        name=name,
        mass=mass,
        inertia=inertia,
        visual_mesh=visual_mesh,
        visual_geometry=visual_geom,
        visual_origin=visual_ori,
        material=material,
        collision_geometry=collision_geom,
        collision_origin=collision_ori,
    )


def _joint_from_element(joint: ET.Element) -> Optional[JointData]:
    parent_el = joint.find("parent")
    child_el = joint.find("child")
    parent_name = parent_el.get("link") if parent_el is not None else None
    child_name = child_el.get("link") if child_el is not None else None
    if not parent_name or not child_name:
        return None

    axis = (0.0, 0.0, 1.0)
    axis_el = joint.find("axis")
    if axis_el is not None:
        axis = _floats(axis_el.get("xyz"), 3, (0.0, 0.0, 1.0))

    xyz, rpy = _origin_of(joint)

    limits: Optional[Limits] = None
    limit_el = joint.find("limit")
    if limit_el is not None:
        lower = _attr_float(limit_el, "lower", None)
        upper = _attr_float(limit_el, "upper", None)
        effort = _attr_float(limit_el, "effort", None)
        velocity = _attr_float(limit_el, "velocity", None)
        limits = Limits(
            lower=lower if lower is not None else 0.0,
            upper=upper if upper is not None else 0.0,
            effort=effort if effort else None,
            velocity=velocity if velocity else None,
        )

    dynamics = None
    dyn_el = joint.find("dynamics")
    if dyn_el is not None:
        dynamics = {
            "damping": _attr_float(dyn_el, "damping", 0.0) or 0.0,
            "friction": _attr_float(dyn_el, "friction", 0.0) or 0.0,
        }

    return JointData(
        name=joint.get("name") or "unnamed",
        joint_type=joint.get("type", "fixed"),
        parent_link=parent_name,
        child_link=child_name,
        axis=tuple(axis),
        origin_xyz=tuple(xyz),
        origin_rpy=tuple(rpy),
        limits=limits,
        dynamics=dynamics,
    )


def parse_urdf_string(xml_content: str) -> KinematicGraph:
    """
    Parse a URDF from an XML string into a KinematicGraph using only the stdlib.

    Unlike parse_urdf (file + yourdfpy), this validates the raw author/compiler
    XML directly and carries no third-party dependency, so the editor's
    Validation tab keeps working regardless of the spawned interpreter's
    site-packages. Joints that reference an undefined link are dropped here so a
    malformed graph can't crash the parser — the structural checks in
    validation.validator (and the frontend's XML pre-pass) surface those.

    Raises:
        ValueError: If the XML is malformed or contains no links.
    """
    try:
        root = ET.fromstring(xml_content)
    except ET.ParseError as e:
        raise ValueError(f"Malformed URDF XML: {e}")

    if root.tag != "robot":
        raise ValueError(f"Expected root element <robot>, got <{root.tag}>")

    kg = KinematicGraph()

    for link_el in root.findall("link"):
        link_data = _link_from_element(link_el)
        if link_data is not None and link_data.name not in kg.graph:
            kg.add_link(link_data)

    if not kg.get_links():
        raise ValueError("URDF has no links")

    parsed_joints: List[JointData] = []
    child_links = set()
    for joint_el in root.findall("joint"):
        jd = _joint_from_element(joint_el)
        if jd is None:
            continue
        parsed_joints.append(jd)
        child_links.add(jd.child_link)

    # Root = the (real, non-mount) link that is never some joint's child.
    links = kg.get_links()
    roots = [
        name for name in links
        if name not in child_links and "__mount__" not in name
    ]
    kg.set_root_link(roots[0] if roots else links[0])

    for jd in parsed_joints:
        if jd.parent_link in kg.graph and jd.child_link in kg.graph:
            try:
                kg.add_joint(jd)
            except ValueError:
                continue

    _reconstitute_servo_splits(kg)
    return kg

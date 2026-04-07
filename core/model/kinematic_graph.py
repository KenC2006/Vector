"""
Kinematic graph representation using networkx.
The kinematic graph is the internal representation used by the AI layer.
Links are nodes, joints are edges.
"""
from typing import Dict, List, Optional, Any
import networkx as nx
from .types import LinkData, JointData, Inertia, Limits


class KinematicGraph:
    """
    Represents a robot's kinematic structure as a directed graph.

    Nodes: Links with LinkData (mass, inertia, meshes, collision geometry)
    Edges: Joints with JointData (type, axis, limits, parent/child)
    """

    def __init__(self):
        """Initialize an empty kinematic graph."""
        self.graph = nx.DiGraph()
        self.root_link: Optional[str] = None

    def add_link(self, link_data: LinkData) -> None:
        """Add a link to the graph."""
        self.graph.add_node(link_data.name, data=link_data)

    def add_joint(self, joint_data: JointData) -> None:
        """
        Add a joint to the graph.
        Creates an edge from parent_link to child_link.
        """
        # Ensure both links exist
        if joint_data.parent_link not in self.graph:
            raise ValueError(f"Parent link '{joint_data.parent_link}' not in graph")
        if joint_data.child_link not in self.graph:
            raise ValueError(f"Child link '{joint_data.child_link}' not in graph")

        # Add edge with joint data
        self.graph.add_edge(
            joint_data.parent_link,
            joint_data.child_link,
            data=joint_data
        )

    def set_root_link(self, link_name: str) -> None:
        """Set the root (base) link of the kinematic tree."""
        if link_name not in self.graph:
            raise ValueError(f"Link '{link_name}' not in graph")
        self.root_link = link_name

    def get_links(self) -> List[str]:
        """Return list of all link names."""
        return list(self.graph.nodes())

    def get_joints(self) -> List[str]:
        """Return list of all joint names."""
        return [
            self.graph[u][v]["data"].name
            for u, v in self.graph.edges()
        ]

    def get_link_data(self, link_name: str) -> Optional[LinkData]:
        """Get LinkData for a given link."""
        if link_name in self.graph:
            return self.graph.nodes[link_name].get("data")
        return None

    def get_joint_data(self, joint_name: str) -> Optional[JointData]:
        """Get JointData by joint name."""
        for u, v in self.graph.edges():
            joint_data = self.graph[u][v]["data"]
            if joint_data.name == joint_name:
                return joint_data
        return None

    def get_parent_link(self, link_name: str) -> Optional[str]:
        """Get the parent link of a given link (via incoming edge)."""
        incoming = list(self.graph.in_edges(link_name))
        if incoming:
            return incoming[0][0]  # First (and only) parent
        return None

    def get_children_links(self, link_name: str) -> List[str]:
        """Get all child links of a given link (via outgoing edges)."""
        return list(self.graph.successors(link_name))

    def get_subtree(self, link_name: str) -> "KinematicGraph":
        """
        Extract a subtree rooted at link_name as a new KinematicGraph.
        """
        if link_name not in self.graph:
            raise ValueError(f"Link '{link_name}' not in graph")

        # BFS to find all reachable nodes
        reachable = set(nx.descendants(self.graph, link_name))
        reachable.add(link_name)

        # Create subgraph
        subgraph_nx = self.graph.subgraph(reachable).copy()

        # Convert to new KinematicGraph
        new_kg = KinematicGraph()
        new_kg.graph = subgraph_nx
        new_kg.root_link = link_name
        return new_kg

    def to_json(self) -> Dict[str, Any]:
        """
        Serialize the entire graph to a JSON-serializable dict.
        This is what gets sent over IPC to the frontend.
        """
        links = []
        for link_name in self.graph.nodes():
            link_data = self.get_link_data(link_name)
            if link_data:
                links.append(link_data.to_dict())

        joints = []
        for u, v in self.graph.edges():
            joint_data = self.graph[u][v]["data"]
            joints.append(joint_data.to_dict())

        return {
            "root_link": self.root_link,
            "links": links,
            "joints": joints,
        }

    @classmethod
    def from_json(cls, data: Dict[str, Any]) -> "KinematicGraph":
        """
        Reconstruct a KinematicGraph from JSON data.
        """
        kg = cls()
        kg.root_link = data.get("root_link")

        # Reconstruct links
        for link_dict in data.get("links", []):
            inertia = None
            if link_dict.get("inertia"):
                inertia_data = link_dict["inertia"]
                inertia = Inertia(**inertia_data)

            link_data = LinkData(
                name=link_dict["name"],
                mass=link_dict.get("mass", 0.0),
                inertia=inertia,
                visual_mesh=link_dict.get("visual_mesh"),
                collision_geometry=link_dict.get("collision_geometry"),
            )
            kg.add_link(link_data)

        # Reconstruct joints
        for joint_dict in data.get("joints", []):
            limits = None
            if joint_dict.get("limits"):
                limits_data = joint_dict["limits"]
                limits = Limits(**limits_data)

            joint_data = JointData(
                name=joint_dict["name"],
                joint_type=joint_dict["joint_type"],
                parent_link=joint_dict["parent_link"],
                child_link=joint_dict["child_link"],
                axis=tuple(joint_dict.get("axis", [0.0, 0.0, 1.0])),
                origin_xyz=tuple(joint_dict.get("origin_xyz", [0.0, 0.0, 0.0])),
                origin_rpy=tuple(joint_dict.get("origin_rpy", [0.0, 0.0, 0.0])),
                limits=limits,
                dynamics=joint_dict.get("dynamics"),
            )
            kg.add_joint(joint_data)

        return kg

    def __repr__(self) -> str:
        return f"KinematicGraph(links={len(self.graph.nodes())}, joints={len(self.graph.edges())})"

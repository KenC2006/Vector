# Vector Core — Python AI Layer

This is the Python backend for Vector, the AI-native robotics IDE. It handles robot model parsing, kinematic graph representation, and serves as the AI processing layer via JSON-RPC over stdio.

## Quick Start

### Install dependencies

```bash
pip install -r requirements.txt
```

### Test the implementation

```bash
# Quick test: parse URDF and serialize to JSON
python test_hello.py

# JSON-RPC server test
python test_server.py
```

### Run the JSON-RPC server

The server reads line-delimited JSON from stdin and writes responses to stdout. Start it with:

```bash
python server.py
```

Then send JSON-RPC 2.0 requests like:

```json
{"jsonrpc": "2.0", "method": "ping", "params": {}, "id": 1}
{"jsonrpc": "2.0", "method": "parse_urdf", "params": {"path": "test_data/simple_arm.urdf"}, "id": 2}
```

## Architecture

### Directories

- **`model/`** — Robot model parsing and kinematic graph
  - `types.py` — Data structures (LinkData, JointData, Inertia, Limits)
  - `kinematic_graph.py` — KinematicGraph class using NetworkX
  - `urdf_parser.py` — URDF parsing using yourdfpy

- **`ai/`** — Claude API integration (future)

- **`presets/`** — Component preset library (future)

- **`validation/`** — Validation checks (future)

- **`cad/`** — CadQuery-based CAD generation (future)

- **`sim/`** — Simulation backend adapters (MuJoCo, Isaac, Genesis)

### Key Classes

#### `LinkData`
Represents a robot link with mass, inertia, visual mesh, and collision geometry.

```python
from model.types import LinkData, Inertia

link = LinkData(
    name="upper_arm",
    mass=0.8,
    inertia=Inertia(ixx=0.005, ixy=0.0, ixz=0.0, iyy=0.005, iyz=0.0, izz=0.001),
    collision_geometry={"type": "cylinder", "params": {"radius": 0.04, "length": 0.3}},
)
```

#### `JointData`
Represents a robot joint with type, axis, limits, and dynamics.

```python
from model.types import JointData, Limits

joint = JointData(
    name="elbow_joint",
    joint_type="revolute",
    parent_link="shoulder_link",
    child_link="upper_arm_link",
    axis=(0.0, 1.0, 0.0),
    limits=Limits(lower=-1.57, upper=1.57, effort=8.0, velocity=1.5),
)
```

#### `KinematicGraph`
Directed graph representation of the robot's kinematic structure.

```python
from model.kinematic_graph import KinematicGraph
from model.urdf_parser import parse_urdf

# Parse URDF file
kg = parse_urdf("robot.urdf")

# Serialize to JSON for IPC
kg_json = kg.to_json()

# Get links and joints
links = kg.get_links()
joints = kg.get_joints()

# Get subtree rooted at a link
subtree = kg.get_subtree("upper_arm")
```

### JSON Format

The kinematic graph serializes to JSON for transmission to the frontend:

```json
{
  "root_link": "base_link",
  "links": [
    {
      "name": "base_link",
      "mass": 1.0,
      "inertia": {
        "ixx": 0.01,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyy": 0.01,
        "iyz": 0.0,
        "izz": 0.01
      },
      "visual_mesh": null,
      "collision_geometry": {
        "type": "box",
        "params": {"size": [0.2, 0.2, 0.1]}
      }
    }
  ],
  "joints": [
    {
      "name": "shoulder_joint",
      "joint_type": "revolute",
      "parent_link": "base_link",
      "child_link": "shoulder_link",
      "axis": [0.0, 0.0, 1.0],
      "origin_xyz": [0.0, 0.0, 0.1],
      "origin_rpy": [0.0, 0.0, 0.0],
      "limits": {
        "lower": -3.14159,
        "upper": 3.14159,
        "effort": 10.0,
        "velocity": 1.0
      },
      "dynamics": {
        "damping": 0.1,
        "friction": 0.0
      }
    }
  ]
}
```

## JSON-RPC Methods

### `ping()`

Health check.

**Request:**
```json
{"jsonrpc": "2.0", "method": "ping", "params": {}, "id": 1}
```

**Response:**
```json
{"jsonrpc": "2.0", "result": "pong", "id": 1}
```

### `parse_urdf(path: str) -> dict`

Parse a URDF file and return the kinematic graph.

**Request:**
```json
{
  "jsonrpc": "2.0",
  "method": "parse_urdf",
  "params": {"path": "/path/to/robot.urdf"},
  "id": 2
}
```

**Response:**
```json
{
  "jsonrpc": "2.0",
  "result": {
    "root_link": "base_link",
    "links": [...],
    "joints": [...]
  },
  "id": 2
}
```

## Dependencies

- **yourdfpy** — URDF parsing and validation
- **networkx** — Kinematic graph representation
- **numpy** — Linear algebra for inertia tensors
- **lxml** — XML manipulation (for future diff support)

## Sample URDF

A sample 3-DOF robot arm is provided in `test_data/simple_arm.urdf`:

```
base_link → shoulder_joint → shoulder_link → elbow_joint → upper_arm_link
  ↓
  (1 DOF, revolute, Z-axis)

upper_arm_link → wrist_joint → forearm_link → tool_joint → end_effector
                (2 DOF, revolute, Z-axis)  (3 DOF, fixed)
```

All links include realistic mass, inertia, and collision geometry.

## Testing

Run the test suite:

```bash
# Quick parsing test
python test_hello.py

# JSON-RPC server test
python test_server.py
```

Both tests verify:
- URDF parsing and kinematic graph building
- JSON serialization and round-trip consistency
- JSON-RPC protocol compliance
- Error handling

## Future Work

- **AI/Claude integration** — Stateful edit commands from Claude
- **Validation agent** — Self-collision checks, inertia plausibility, etc.
- **Preset library** — Actuator, sensor, and archetype presets
- **CAD generation** — CadQuery export/import
- **Simulation backend** — MuJoCo, Isaac Sim, Genesis adapters
- **MJCF support** — MuJoCo XML format parsing and generation

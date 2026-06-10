"""Author 'new-style' demo graphs that exercise the refactored authoring
surface the way the rewritten SYSTEM_PROMPT teaches:

  - link_geometry bodies with NAMED primitives
  - children mounted via attach_primitive/attach_anchor (real surfaces)
  - per-side attach_rpy rest poses (verbatim, no mirroring)
  - side-face horizontal booms via the fixed orientation grammar

Scored next to the old-style baselines they correspond to, these demonstrate
what the refactor unlocked. Deterministic; writes eval/newstyle/*.graph.json.
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "newstyle")
os.makedirs(OUT, exist_ok=True)


def comp(link, cid, parent, face="top", jt="fixed", ja="z", **kw):
    c = {"link_name": link, "component_id": cid, "attach_to": parent,
         "attach_face": face, "joint_type": jt, "joint_axis": ja}
    c.update(kw)
    return c


graphs = {}

# -- humanoid: anchored shoulders/hips/head on a named-primitive torso --
torso_geom = [
    {"name": "chest", "shape": "box", "size_mm": [180, 100, 260], "xyz_mm": [0, 0, 130]},
    {"name": "head", "shape": "sphere", "radius_mm": 70, "xyz_mm": [0, 0, 330]},
    {"name": "shoulder_l", "shape": "cylinder", "radius_mm": 25, "length_mm": 60, "xyz_mm": [0, -115, 230], "rpy": [1.5708, 0, 0]},
    {"name": "shoulder_r", "shape": "cylinder", "radius_mm": 25, "length_mm": 60, "xyz_mm": [0, 115, 230], "rpy": [1.5708, 0, 0]},
]
hum = [comp("pelvis", "structural_baseplate", None),
       comp("torso", "structural_extrusion_4040", "pelvis", "top", "fixed", "z", length_mm=250, link_geometry=torso_geom)]
for side, anchor in [("l", "+axis_end"), ("r", "-axis_end")]:
    hum += [
        comp(f"shoulder_servo_{side}", "actuator_servo_high_torque", "torso",
             attach_primitive=f"shoulder_{side}", attach_anchor=anchor,
             face="left" if side == "l" else "right", jt="revolute", ja="y"),
        comp(f"upper_arm_{side}", "structural_extrusion_2020", f"shoulder_servo_{side}", "bottom", "fixed", "z", length_mm=180),
        comp(f"elbow_{side}", "actuator_servo_standard", f"upper_arm_{side}", "bottom", "revolute", "y"),
        comp(f"forearm_{side}", "structural_extrusion_2020", f"elbow_{side}", "bottom", "fixed", "z", length_mm=140),
        comp(f"wrist_{side}", "actuator_servo_standard", f"forearm_{side}", "bottom", "revolute", "y"),
        comp(f"hand_{side}", "effector_parallel_gripper_small", f"wrist_{side}", "bottom", "fixed", "z"),
    ]
for side in ["l", "r"]:
    hum += [
        comp(f"hip_{side}", "actuator_servo_high_torque", "pelvis", "bottom", "revolute", "y", attach_rpy=[0, 0.3, 0]),
        comp(f"thigh_{side}", "structural_extrusion_2020", f"hip_{side}", "bottom", "fixed", "z", length_mm=200),
        comp(f"knee_{side}", "actuator_servo_high_torque", f"thigh_{side}", "bottom", "revolute", "y", attach_rpy=[0, -0.6, 0]),
        comp(f"shin_{side}", "structural_extrusion_2020", f"knee_{side}", "bottom", "fixed", "z", length_mm=200),
        comp(f"ankle_{side}", "actuator_servo_standard", f"shin_{side}", "bottom", "revolute", "y", attach_rpy=[0, 0.3, 0]),
        comp(f"foot_{side}", "mobility_rubber_foot_pad", f"ankle_{side}", "bottom", "fixed", "z"),
    ]
hum += [comp("camera_1", "sensor_depth_camera_small", "torso",
             attach_primitive="head", attach_anchor="+x_pole", face="front")]
graphs["humanoid"] = {"base_link": "pelvis", "ground_offset": True, "components": hum}

# -- sculpture: scorpion with anchored legs/claws on named carapace sockets --
cara_geom = [
    {"name": "carapace", "shape": "box", "size_mm": [260, 180, 60], "xyz_mm": [0, 0, 30]},
    {"name": "head", "shape": "box", "size_mm": [80, 120, 50], "xyz_mm": [160, 0, 25]},
    {"name": "claw_socket_l", "shape": "cylinder", "radius_mm": 18, "length_mm": 40, "xyz_mm": [120, -95, 35], "rpy": [1.5708, 0, 0]},
    {"name": "claw_socket_r", "shape": "cylinder", "radius_mm": 18, "length_mm": 40, "xyz_mm": [120, 95, 35], "rpy": [1.5708, 0, 0]},
    {"name": "tail_socket", "shape": "cylinder", "radius_mm": 20, "length_mm": 30, "xyz_mm": [-120, 0, 60]},
]
sco = [comp("body", "structural_baseplate", None, link_geometry=cara_geom)]
# 6 walking legs on the carapace bottom (radial distribution + per-leg rest poses)
bends = [(-0.8, 0.9), (-0.9, 1.0), (-0.8, 0.9), (-0.8, 0.9), (-0.9, 1.0), (-0.8, 0.9)]
for i, (hip_b, knee_b) in enumerate(bends):
    sco += [
        comp(f"leg_hip_{i}", "actuator_servo_standard", "body", "bottom", "revolute", "z"),
        comp(f"leg_pitch_{i}", "actuator_servo_standard", f"leg_hip_{i}", "bottom", "revolute", "y", attach_rpy=[0, hip_b, 0]),
        comp(f"leg_seg_{i}", "structural_limb_link_slim", f"leg_pitch_{i}", "bottom", "fixed", "z", length_mm=90),
        comp(f"leg_knee_{i}", "actuator_servo_standard", f"leg_seg_{i}", "bottom", "revolute", "y", attach_rpy=[0, knee_b, 0]),
        comp(f"leg_shin_{i}", "structural_limb_link_slim", f"leg_knee_{i}", "bottom", "fixed", "z", length_mm=100),
        comp(f"leg_foot_{i}", "mobility_rubber_foot_pad", f"leg_shin_{i}", "bottom", "fixed", "z"),
    ]
# pincer arms anchored on the claw sockets
for side, anchor in [("l", "+axis_end"), ("r", "-axis_end")]:
    sco += [
        comp(f"claw_shoulder_{side}", "actuator_servo_standard", "body",
             attach_primitive=f"claw_socket_{side}", attach_anchor=anchor,
             face="left" if side == "l" else "right", jt="revolute", ja="z"),
        comp(f"claw_arm_{side}", "structural_limb_link_slim", f"claw_shoulder_{side}", "bottom", "fixed", "z", length_mm=80),
        comp(f"claw_{side}", "effector_parallel_gripper_small", f"claw_arm_{side}", "bottom", "fixed", "z"),
    ]
# curled tail anchored on the tail socket
sco += [comp("tail_servo_0", "actuator_servo_standard", "body",
             attach_primitive="tail_socket", attach_anchor="+axis_end",
             face="top", jt="revolute", ja="y", attach_rpy=[0, -0.45, 0])]
prev = "tail_servo_0"
for i in range(1, 4):
    sco += [comp(f"tail_seg_{i-1}", "structural_limb_link_slim", prev, "top", "fixed", "z", length_mm=70),
            comp(f"tail_servo_{i}", "actuator_servo_standard", f"tail_seg_{i-1}", "top", "revolute", "y", attach_rpy=[0, -0.45, 0])]
    prev = f"tail_servo_{i}"
sco += [comp("tail_tip", "structural_limb_link_slim", prev, "top", "fixed", "z", length_mm=70),
        comp("stinger", "sensor_ultrasonic", "tail_tip", "top", "fixed", "z")]
graphs["sculpture"] = {"base_link": "body", "ground_offset": True, "components": sco}

for pid, g in graphs.items():
    path = os.path.join(OUT, f"{pid}.graph.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(g, f, indent=1)
    print(pid, len(g["components"]), "components ->", path)

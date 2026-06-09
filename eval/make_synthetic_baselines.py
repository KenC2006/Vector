"""Author the synthetic baseline AssemblyGraphs for the eval harness.

These are hand-written stand-ins shaped like real AI output (same component
ids, attachment idioms, and authoring fields the design_robot tool emits).
They exist because baseline capture needs a valid ANTHROPIC_API_KEY; replace
them with real --live captures when one is available (see eval/baselines/README.md).
Deterministic by construction — safe to re-run.
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "baselines")
os.makedirs(OUT, exist_ok=True)


def comp(link, cid, parent, face="top", jt="fixed", ja="z", **kw):
    c = {"link_name": link, "component_id": cid, "attach_to": parent,
         "attach_face": face, "joint_type": jt, "joint_axis": ja}
    c.update(kw)
    return c


graphs = {}

# -- dog: canonical 12-DOF quadruped with crouch --
dog = [comp("base", "structural_baseplate_large", None)]
for leg in ["fl", "fr", "rl", "rr"]:
    dog += [
        comp(f"hip_yaw_{leg}", "actuator_servo_high_torque", "base", "bottom", "revolute", "z"),
        comp(f"hip_pitch_{leg}", "actuator_servo_high_torque", f"hip_yaw_{leg}", "bottom", "revolute", "y", attach_rpy=[0, 0.52, 0]),
        comp(f"thigh_{leg}", "structural_limb_link_slim", f"hip_pitch_{leg}", "bottom", "fixed", "z", length_mm=100),
        comp(f"knee_{leg}", "actuator_servo_high_torque", f"thigh_{leg}", "bottom", "revolute", "y", attach_rpy=[0, 1.05, 0]),
        comp(f"shin_{leg}", "structural_limb_link_slim", f"knee_{leg}", "bottom", "fixed", "z", length_mm=120),
        comp(f"foot_{leg}", "mobility_rubber_foot_pad", f"shin_{leg}", "bottom", "fixed", "z"),
    ]
dog += [comp("battery_1", "power_lipo_4s_5000", "base", "top"),
        comp("sbc_1", "compute_sbc_small", "base", "top")]
graphs["dog"] = {"base_link": "base", "ground_offset": True, "components": dog}

# -- humanoid: pelvis + link_geometry torso + arms + legs --
torso_geom = [
    {"name": "chest", "shape": "box", "size_mm": [180, 100, 260], "xyz_mm": [0, 0, 130]},
    {"name": "head", "shape": "sphere", "radius_mm": 70, "xyz_mm": [0, 0, 330]},
    {"name": "shoulder_l", "shape": "cylinder", "radius_mm": 25, "length_mm": 60, "xyz_mm": [0, -115, 230], "rpy": [1.5708, 0, 0]},
    {"name": "shoulder_r", "shape": "cylinder", "radius_mm": 25, "length_mm": 60, "xyz_mm": [0, 115, 230], "rpy": [1.5708, 0, 0]},
]
hum = [comp("pelvis", "structural_baseplate", None),
       comp("torso", "structural_extrusion_4040", "pelvis", "top", "fixed", "z", length_mm=250, link_geometry=torso_geom)]
for side, face in [("l", "left"), ("r", "right")]:
    hum += [
        comp(f"shoulder_servo_{side}", "actuator_servo_high_torque", "torso", face, "revolute", "y"),
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
        comp(f"knee_{side}", "actuator_servo_high_torque", f"thigh_{side}", "bottom", "revolute", "y", attach_rpy=[0, 0.6, 0]),
        comp(f"shin_{side}", "structural_extrusion_2020", f"knee_{side}", "bottom", "fixed", "z", length_mm=200),
        comp(f"ankle_{side}", "actuator_servo_standard", f"shin_{side}", "bottom", "revolute", "y"),
        comp(f"foot_{side}", "mobility_rubber_foot_pad", f"ankle_{side}", "bottom", "fixed", "z"),
    ]
hum += [comp("camera_1", "sensor_depth_camera_small", "torso", "front", "fixed", "z")]
graphs["humanoid"] = {"base_link": "pelvis", "ground_offset": True, "components": hum}

# -- hexapod: 6 legs, varied lengths --
hexp = [comp("body", "structural_baseplate", None)]
lens = [(90, 110), (100, 120), (90, 110), (90, 110), (100, 120), (90, 110)]
for i, (t, s) in enumerate(lens):
    hexp += [
        comp(f"hip_{i}", "actuator_servo_high_torque", "body", "bottom", "revolute", "z"),
        comp(f"hip_pitch_{i}", "actuator_servo_standard", f"hip_{i}", "bottom", "revolute", "y", attach_rpy=[0, 0.7, 0]),
        comp(f"thigh_{i}", "structural_limb_link_slim", f"hip_pitch_{i}", "bottom", "fixed", "z", length_mm=t),
        comp(f"knee_{i}", "actuator_servo_standard", f"thigh_{i}", "bottom", "revolute", "y", attach_rpy=[0, -1.1, 0]),
        comp(f"shin_{i}", "structural_limb_link_slim", f"knee_{i}", "bottom", "fixed", "z", length_mm=s),
        comp(f"foot_{i}", "mobility_rubber_foot_pad", f"shin_{i}", "bottom", "fixed", "z"),
    ]
hexp += [comp("antenna_l", "sensor_ultrasonic", "body", "front", "fixed", "z"),
         comp("battery_1", "power_lipo_3s_2200", "body", "top")]
graphs["hexapod"] = {"base_link": "body", "ground_offset": True, "components": hexp}

# -- rover: 4 hub motors + wheels + electronics --
rov = [comp("chassis", "structural_baseplate", None)]
for w in ["fl", "fr", "rl", "rr"]:
    rov += [comp(f"motor_{w}", "drivetrain_hub_motor_80", "chassis", "bottom", "continuous", "y"),
            comp(f"wheel_{w}", "mobility_wheel_driven", f"motor_{w}", "coaxial", "fixed", "z")]
rov += [comp("bracket_cam", "structural_bracket_l", "chassis", "front", "fixed", "z", mate_connector="wall_outer"),
        comp("camera_1", "sensor_depth_camera_small", "bracket_cam", "front", "fixed", "z",
             attach_connector="wall_inner", mate_connector="mount_back", mate_type="fastened"),
        comp("battery_1", "power_lipo_4s_5000", "chassis", "top"),
        comp("sbc_1", "compute_sbc_small", "chassis", "top")]
graphs["rover"] = {"base_link": "chassis", "ground_offset": True, "components": rov}

# -- arm: canonical tabletop arm --
arm = [comp("base", "structural_baseplate", None),
       comp("base_servo", "actuator_servo_high_torque", "base", "top", "revolute", "z"),
       comp("stem", "structural_extrusion_2020", "base_servo", "top", "fixed", "z", length_mm=80),
       comp("shoulder", "actuator_servo_high_torque", "stem", "top", "revolute", "y"),
       comp("upper_arm", "structural_extrusion_2020", "shoulder", "top", "fixed", "z", length_mm=200),
       comp("elbow", "actuator_servo_high_torque", "upper_arm", "top", "revolute", "y"),
       comp("forearm", "structural_extrusion_2020", "elbow", "top", "fixed", "z", length_mm=150),
       comp("wrist", "actuator_servo_standard", "forearm", "top", "revolute", "y"),
       comp("gripper", "effector_parallel_gripper_small", "wrist", "top", "fixed", "z")]
graphs["arm"] = {"base_link": "base", "ground_offset": True, "components": arm}

# -- sculpture: scorpion with link_geometry carapace + curled tail + claws --
cara_geom = [
    {"name": "carapace", "shape": "box", "size_mm": [260, 180, 60], "xyz_mm": [0, 0, 30]},
    {"name": "head", "shape": "box", "size_mm": [80, 120, 50], "xyz_mm": [160, 0, 25]},
]
sco = [comp("body", "structural_baseplate", None, link_geometry=cara_geom)]
for i in range(6):
    sco += [
        comp(f"leg_hip_{i}", "actuator_servo_standard", "body", "bottom", "revolute", "z"),
        comp(f"leg_seg_{i}", "structural_limb_link_slim", f"leg_hip_{i}", "bottom", "fixed", "z", length_mm=90),
        comp(f"leg_knee_{i}", "actuator_servo_standard", f"leg_seg_{i}", "bottom", "revolute", "y", attach_rpy=[0, -0.9, 0]),
        comp(f"leg_shin_{i}", "structural_limb_link_slim", f"leg_knee_{i}", "bottom", "fixed", "z", length_mm=100),
        comp(f"leg_foot_{i}", "mobility_rubber_foot_pad", f"leg_shin_{i}", "bottom", "fixed", "z"),
    ]
for side, face in [("l", "left"), ("r", "right")]:
    sco += [
        comp(f"claw_shoulder_{side}", "actuator_servo_standard", "body", face, "revolute", "z"),
        comp(f"claw_arm_{side}", "structural_limb_link_slim", f"claw_shoulder_{side}", "bottom", "fixed", "z", length_mm=80),
        comp(f"claw_{side}", "effector_parallel_gripper_small", f"claw_arm_{side}", "bottom", "fixed", "z"),
    ]
prev = "body"
for i in range(4):
    sco += [comp(f"tail_servo_{i}", "actuator_servo_standard", prev, "top", "revolute", "y", attach_rpy=[0, -0.45, 0]),
            comp(f"tail_seg_{i}", "structural_limb_link_slim", f"tail_servo_{i}", "top", "fixed", "z", length_mm=70)]
    prev = f"tail_seg_{i}"
sco += [comp("stinger", "sensor_ultrasonic", prev, "top", "fixed", "z")]
graphs["sculpture"] = {"base_link": "body", "ground_offset": True, "components": sco}

for pid, g in graphs.items():
    path = os.path.join(OUT, f"{pid}.graph.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(g, f, indent=1)
    print(pid, len(g["components"]), "components ->", path)

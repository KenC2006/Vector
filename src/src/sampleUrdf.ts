/** Default `robot.urdf` content on startup (Monaco + first scene parse). */
export const SAMPLE_URDF = `<?xml version="1.0"?>
<robot name="robot_dog">
  <material name="dark_gray">
    <color rgba="0.3 0.3 0.3 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.2 0.45 0.95 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.1 0.1 0.1 1.0"/>
  </material>

  <!-- Body/Torso -->
  <link name="body">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2.5"/>
      <inertia ixx="0.083" ixy="0.0" ixz="0.0" iyy="0.042" iyz="0.0" izz="0.083"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.2 0.1"/>
      </geometry>
      <material name="dark_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.2 0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Front Left Leg -->
  <link name="fl_hip">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.165"/>
      <inertia ixx="0.00034" ixy="0.0" ixz="0.0" iyy="0.00055" iyz="0.0" izz="0.00034"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.0465 0.036 0.034"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.0465 0.036 0.034"/>
      </geometry>
    </collision>
  </link>

  <joint name="fl_hip_joint" type="revolute">
    <parent link="body"/>
    <child link="fl_hip"/>
    <origin xyz="0.15 0.12 -0.05" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.6" velocity="2.0"/>
  </joint>

  <link name="fl_shoulder">
    <inertial>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <mass value="0.165"/>
      <inertia ixx="0.00055" ixy="0.0" ixz="0.0" iyy="0.00055" iyz="0.0" izz="0.00034"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.03 0.15"/>
      </geometry>
      <material name="dark_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.03 0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="fl_shoulder_joint" type="revolute">
    <parent link="fl_hip"/>
    <child link="fl_shoulder"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.09" upper="2.09" effort="10.6" velocity="2.0"/>
  </joint>

  <link name="fl_knee">
    <inertial>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <mass value="0.165"/>
      <inertia ixx="0.00055" ixy="0.0" ixz="0.0" iyy="0.00055" iyz="0.0" izz="0.00034"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.025 0.025 0.15"/>
      </geometry>
      <material name="dark_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.025 0.025 0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="fl_knee_joint" type="revolute">
    <parent link="fl_shoulder"/>
    <child link="fl_knee"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.09" upper="0.52" effort="10.6" velocity="2.0"/>
  </joint>

  <link name="fl_foot">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.02"/>
      <inertia ixx="0.000003" ixy="0.0" ixz="0.0" iyy="0.000003" iyz="0.0" izz="0.000003"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.015"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.015"/>
      </geometry>
    </collision>
  </link>

  <joint name="fl_foot_joint" type="fixed">
    <parent link="fl_knee"/>
    <child link="fl_foot"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
  </joint>

  <!-- Front Right Leg -->
  <link name="fr_hip">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.165"/>
      <inertia ixx="0.00034" ixy="0.0" ixz="0.0" iyy="0.00055" iyz="0.0" izz="0.00034"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.0465 0.036 0.034"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.0465 0.036 0.034"/>
      </geometry>
    </collision>
  </link>

  <joint name="fr_hip_joint" type="revolute">
    <parent link="body"/>
    <child link="fr_hip"/>
    <origin xyz="0.15 -0.12 -0.05" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.6" velocity="2.0"/>
  </joint>

  <link name="fr_shoulder">
    <inertial>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <mass value="0.165"/>
      <inertia ixx="0.00055" ixy="0.0" ixz="0.0" iyy="0.00055" iyz="0.0" izz="0.00034"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.03 0.15"/>
      </geometry>
      <material name="dark_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.03 0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="fr_shoulder_joint" type="revolute">
    <parent link="fr_hip"/>
    <child link="fr_shoulder"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.09" upper="2.09" effort="10.6" velocity="2.0"/>
  </joint>

  <link name="fr_knee">
    <inertial>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <mass value="0.165"/>
      <inertia ixx="0.00055" ixy="0.0" ixz="0.0" iyy="0.00055" iyz="0.0" izz="0.00034"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.025 0.025 0.15"/>
      </geometry>
      <material name="dark_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.025 0.025 0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="fr_knee_joint" type="revolute">
    <parent link="fr_shoulder"/>
    <child link="fr_knee"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.09" upper="0.52" effort="10.6" velocity="2.0"/>
  </joint>

  <link name="fr_foot">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.02"/>
      <inertia ixx="0.000003" ixy="0.0" ixz="0.0" iyy="0.000003" iyz="0.0" izz="0.000003"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.015"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.015"/>
      </geometry>
    </collision>
  </link>

  <joint name="fr_foot_joint" type="fixed">
    <parent link="fr_knee"/>
    <child link="fr_foot"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
  </joint>

  <!-- Back Left Leg -->
  <link name="bl_hip">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.165"/>
      <inertia ixx="0.00034" ixy="0.0" ixz="0.0" iyy="0.00055" iyz="0.0" izz="0.00034"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.0465 0.036 0.034"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.0465 0.036 0.034"/>
      </geometry>
    </collision>
  </link>

  <joint name="bl_hip_joint" type="revolute">
    <parent link="body"/>
    <child link="bl_hip"/>
    <origin xyz="-0.15 0.12 -0.05" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.6" velocity="2.0"/>
  </joint>

  <link name="bl_shoulder">
    <inertial>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <mass value="0.165"/>
      <inertia ixx="0.00055" ixy="0.0" ixz="0.0" iyy="0.00055" iyz="0.0" izz="0.00034"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.03 0.15"/>
      </geometry>
      <material name="dark_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.03 0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="bl_shoulder_joint" type="revolute">
    <parent link="bl_hip"/>
    <child link="bl_shoulder"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.09" upper="2.09" effort="10.6" velocity="2.0"/>
  </joint>

  <link name="bl_knee">
    <inertial>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <mass value="0.165"/>
      <inertia ixx="0.00055" ixy="0.0" ixz="0.0" iyy="0.00055" iyz="0.0" izz="0.00034"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.025 0.025 0.15"/>
      </geometry>
      <material name="dark_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.025 0.025 0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="bl_knee_joint" type="revolute">
    <parent link="bl_shoulder"/>
    <child link="bl_knee"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.09" upper="0.52" effort="10.6" velocity="2.0"/>
  </joint>

  <link name="bl_foot">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.02"/>
      <inertia ixx="0.000003" ixy="0.0" ixz="0.0" iyy="0.000003" iyz="0.0" izz="0.000003"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.015"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.015"/>
      </geometry>
    </collision>
  </link>

  <joint name="bl_foot_joint" type="fixed">
    <parent link="bl_knee"/>
    <child link="bl_foot"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
  </joint>

  <!-- Back Right Leg -->
  <link name="br_hip">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.165"/>
      <inertia ixx="0.00034" ixy="0.0" ixz="0.0" iyy="0.00055" iyz="0.0" izz="0.00034"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.0465 0.036 0.034"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.0465 0.036 0.034"/>
      </geometry>
    </collision>
  </link>

  <joint name="br_hip_joint" type="revolute">
    <parent link="body"/>
    <child link="br_hip"/>
    <origin xyz="-0.15 -0.12 -0.05" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.6" velocity="2.0"/>
  </joint>

  <link name="br_shoulder">
    <inertial>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <mass value="0.165"/>
      <inertia ixx="0.00055" ixy="0.0" ixz="0.0" iyy="0.00055" iyz="0.0" izz="0.00034"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.03 0.15"/>
      </geometry>
      <material name="dark_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.03 0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="br_shoulder_joint" type="revolute">
    <parent link="br_hip"/>
    <child link="br_shoulder"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.09" upper="2.09" effort="10.6" velocity="2.0"/>
  </joint>

  <link name="br_knee">
    <inertial>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <mass value="0.165"/>
      <inertia ixx="0.00055" ixy="0.0" ixz="0.0" iyy="0.00055" iyz="0.0" izz="0.00034"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.025 0.025 0.15"/>
      </geometry>
      <material name="dark_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.075" rpy="0 0 0"/>
      <geometry>
        <box size="0.025 0.025 0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="br_knee_joint" type="revolute">
    <parent link="br_shoulder"/>
    <child link="br_knee"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.09" upper="0.52" effort="10.6" velocity="2.0"/>
  </joint>

  <link name="br_foot">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.02"/>
      <inertia ixx="0.000003" ixy="0.0" ixz="0.0" iyy="0.000003" iyz="0.0" izz="0.000003"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.015"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.015"/>
      </geometry>
    </collision>
  </link>

  <joint name="br_foot_joint" type="fixed">
    <parent link="br_knee"/>
    <child link="br_foot"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
  </joint>

  <!-- IMU Sensor -->
  <link name="sensor_imu_9dof_1">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.005"/>
      <inertia ixx="0.000000005" ixy="0.0" ixz="0.0" iyy="0.000000003" iyz="0.0" izz="0.000000003"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.025 0.018 0.004"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.025 0.018 0.004"/>
      </geometry>
    </collision>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="body"/>
    <child link="sensor_imu_9dof_1"/>
    <origin xyz="0 0 0.052" rpy="0 0 0"/>
  </joint>

</robot>`

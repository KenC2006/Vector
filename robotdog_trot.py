"""
robotdog_trot.py — diagonal-trot gait for robotDog.urdf

Upload via the sim panel's Script Runner -> Upload .py button.
Enable Free Base in the Physics section first so the dog can fall + step.

NOTE: the sim script sandbox forbids `import` statements.
      `math` is already provided as a global — just use it directly.

Joint map (from robotDog.urdf revolute joints):
  leg 1: hip = _11 (high_torque),  knee = _13 (standard)
  leg 2: hip = _16,                knee = _18
  leg 3: hip = _21,                knee = _23
  leg 4: hip = _26,                knee = _28

Trot pairs diagonal legs (1+3) against (2+4).
Joints _6 and _9 (neck/tail) hold at 0.
"""


# --- Tuning ---------------------------------------------------------------
FREQ_HZ     = 1.5     # strides per second
HIP_AMP     = 0.35    # rad — hip swing amplitude
KNEE_BIAS   = 0.45    # rad — stance crouch (raise if the dog faceplants)
KNEE_AMP    = 0.35    # rad — knee tuck on swing-up
SETTLE_TIME = 0.8     # s — ramp the gait in from zero for a stable start


# --- Joint names ----------------------------------------------------------
HIP_A_1  = 'joint_actuator_servo_high_torque_11'
KNEE_A_1 = 'joint_actuator_servo_standard_13'
HIP_B_1  = 'joint_actuator_servo_high_torque_16'
KNEE_B_1 = 'joint_actuator_servo_standard_18'
HIP_A_2  = 'joint_actuator_servo_high_torque_21'
KNEE_A_2 = 'joint_actuator_servo_standard_23'
HIP_B_2  = 'joint_actuator_servo_high_torque_26'
KNEE_B_2 = 'joint_actuator_servo_standard_28'
NECK     = 'joint_actuator_servo_standard_6'
TAIL     = 'joint_actuator_servo_standard_9'


def step(t, state):
    # Ramp amplitude in from 0 over SETTLE_TIME so the controller
    # doesn't slam the legs at t=0.
    ramp = min(1.0, t / SETTLE_TIME) if SETTLE_TIME > 0 else 1.0

    w = 2.0 * math.pi * FREQ_HZ
    # Diagonal A: legs 1 & 3 in phase; Diagonal B: legs 2 & 4, opposite.
    sin_a = math.sin(w * t)
    sin_b = math.sin(w * t + math.pi)

    hip_a = ramp * HIP_AMP * sin_a
    hip_b = ramp * HIP_AMP * sin_b

    # Knees tuck only on the up-swing (rectified sine). Bias holds a stance.
    knee_a = KNEE_BIAS + ramp * KNEE_AMP * max(0.0, sin_a)
    knee_b = KNEE_BIAS + ramp * KNEE_AMP * max(0.0, sin_b)

    return {
        HIP_A_1:  hip_a,
        KNEE_A_1: knee_a,
        HIP_A_2:  hip_a,
        KNEE_A_2: knee_a,
        HIP_B_1:  hip_b,
        KNEE_B_1: knee_b,
        HIP_B_2:  hip_b,
        KNEE_B_2: knee_b,
        NECK:     0.0,
        TAIL:     0.0,
    }

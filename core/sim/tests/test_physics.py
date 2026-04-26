"""
Physics validation tests for Vector's MuJoCo simulation.

Each test targets one specific aspect of physical accuracy.  If any test
fails the sim cannot be considered physically faithful, regardless of how
good it looks.

Run with:
    python -m pytest core/sim/tests/test_physics.py -v

Tests:
    1. test_gravity_freefall     — z = z₀ - ½g t²          (1 mm tolerance)
    2. test_pendulum_period      — T = 2π √(L/g)            (0.5 % tolerance)
    3. test_energy_conservation  — undamped pendulum, 10 s   (1 % drift)
    4. test_contact_stability    — stacked-box tower, 5 s    (1 mm drift)
    5. test_actuator_settling    — PD arm with gravity, 5 s  (5 % of target)
"""
import math

import numpy as np
import pytest

try:
    import mujoco
    HAS_MUJOCO = True
except ImportError:
    HAS_MUJOCO = False

pytestmark = pytest.mark.skipif(not HAS_MUJOCO, reason="mujoco not installed")

# ---------------------------------------------------------------------------
# Shared MJCF option string — matches what urdf_to_mjcf.py emits after §1.
# ---------------------------------------------------------------------------
_OPTS = (
    'timestep="0.001" '
    'integrator="implicitfast" '
    'solver="Newton" '
    'iterations="100" '
    'tolerance="1e-10" '
    'cone="elliptic" '
    'impratio="10" '
    'noslip_iterations="3"'
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load(xml: str):
    """Parse MJCF xml string, return a fresh (model, data) pair."""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    return model, data


def _step(model, data, n: int) -> None:
    for _ in range(n):
        mujoco.mj_step(model, data)


# ---------------------------------------------------------------------------
# Test 1 — Gravity / free fall
# ---------------------------------------------------------------------------

def test_gravity_freefall():
    """
    A free-floating body released from rest must follow z = z₀ − ½·g·t²
    to within 1 mm at t = 0.1, 0.25, 0.5, and 1.0 s.

    This directly validates:
    - gravity magnitude (9.81 m/s²)
    - integrator accuracy for a purely translational DOF
    """
    g = 9.81
    z0 = 20.0  # high enough to avoid floor for 2 s

    xml = f"""
    <mujoco model="freefall">
      <option {_OPTS} gravity="0 0 -{g}"/>
      <worldbody>
        <body name="box" pos="0 0 {z0}">
          <freejoint/>
          <geom type="box" size="0.1 0.1 0.1" mass="1"/>
        </body>
      </worldbody>
    </mujoco>
    """

    for t_check in [0.1, 0.25, 0.5, 1.0]:
        model, data = _load(xml)
        n_steps = round(t_check / float(model.opt.timestep))
        _step(model, data, n_steps)

        # data.xpos[1] = world position of body 1 (box); body 0 is world.
        z_actual = float(data.xpos[1, 2])
        t_actual = float(data.time)
        z_expected = z0 - 0.5 * g * t_actual ** 2
        err_mm = abs(z_actual - z_expected) * 1000.0

        # Semi-implicit Euler has a systematic O(dt) bias: error ≈ g·t·dt/2.
        # At t=1 s and dt=0.001 s that is ~4.9 mm.  We allow 1.5× the theoretical
        # bias so the tolerance tightens automatically if dt is reduced later.
        bias_mm = 0.5 * g * t_actual * float(model.opt.timestep) * 1000.0 * 1.5
        assert err_mm < bias_mm, (
            f"t={t_actual:.3f} s: z_actual={z_actual:.6f} m  "
            f"z_expected={z_expected:.6f} m  "
            f"error={err_mm:.3f} mm  tolerance={bias_mm:.3f} mm"
        )


# ---------------------------------------------------------------------------
# Test 2 — Pendulum period
# ---------------------------------------------------------------------------

def test_pendulum_period():
    """
    A simple pendulum of length L = 1 m must oscillate with period
    T = 2π √(L/g) ≈ 2.006 s, measured to within 0.5 %.

    Small-angle initial displacement (0.1 rad) keeps non-linearity error
    below 0.06 %, leaving the tolerance budget entirely for numerical error.
    """
    L = 1.0
    g = 9.81
    theta0 = 0.1   # rad — small angle (non-linearity < 0.06 %)
    T_theory = 2.0 * math.pi * math.sqrt(L / g)   # ≈ 2.0065 s

    xml = f"""
    <mujoco model="pendulum">
      <option {_OPTS} gravity="0 0 -{g}"/>
      <worldbody>
        <body name="pivot" pos="0 0 0">
          <!-- hinge axis = Y so pendulum swings in the X-Z plane -->
          <joint name="hinge" type="hinge" axis="0 1 0"
                 damping="0" frictionloss="0" armature="0"/>
          <geom type="sphere" size="0.01" mass="0"/>
          <body name="bob" pos="0 0 -{L}">
            <geom type="sphere" size="0.05" mass="1"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """
    model, data = _load(xml)

    # Start at theta0 with zero velocity; recompute internal state.
    data.qpos[0] = theta0
    data.qvel[0] = 0.0
    mujoco.mj_forward(model, data)

    # Simulate ~3.5 periods and record zero-crossing times.
    dt = float(model.opt.timestep)
    n_steps = round(3.5 * T_theory / dt)
    crossings: list[float] = []
    prev = float(data.qpos[0])

    for _ in range(n_steps):
        _step(model, data, 1)
        curr = float(data.qpos[0])
        if prev * curr < 0.0:
            # Linear interpolation for the crossing instant.
            frac = abs(prev) / (abs(prev) + abs(curr))
            crossings.append(float(data.time) - dt + frac * dt)
        prev = curr

    assert len(crossings) >= 4, (
        f"Expected ≥ 4 zero crossings, got {len(crossings)}. "
        "Check simulation duration or initial conditions."
    )

    # Full period = gap between every other crossing (T/2 each).
    periods = [crossings[i + 2] - crossings[i] for i in range(len(crossings) - 2)]
    T_measured = float(np.mean(periods))
    err_pct = abs(T_measured - T_theory) / T_theory * 100.0

    assert err_pct < 0.5, (
        f"Pendulum period: measured {T_measured:.4f} s  "
        f"theory {T_theory:.4f} s  error {err_pct:.3f} % > 0.5 %"
    )


# ---------------------------------------------------------------------------
# Test 3 — Energy conservation
# ---------------------------------------------------------------------------

def test_energy_conservation():
    """
    Total mechanical energy of an undamped pendulum must not drift more
    than 1 % over 10 seconds of simulation.

    The energy flag is enabled so MuJoCo computes KE + PE at each step.
    A larger initial angle (0.5 rad) gives a higher energy signal and makes
    percentage drift easier to measure accurately.
    """
    L = 1.0
    g = 9.81
    theta0 = 0.5   # rad — large enough for a clear energy signal

    xml = f"""
    <mujoco model="energy_test">
      <option {_OPTS} gravity="0 0 -{g}">
        <flag energy="enable"/>
      </option>
      <worldbody>
        <body name="pivot" pos="0 0 0">
          <joint name="hinge" type="hinge" axis="0 1 0"
                 damping="0" frictionloss="0" armature="0"/>
          <geom type="sphere" size="0.01" mass="0"/>
          <body name="bob" pos="0 0 -{L}">
            <geom type="sphere" size="0.05" mass="1"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """
    model, data = _load(xml)

    data.qpos[0] = theta0
    data.qvel[0] = 0.0
    mujoco.mj_forward(model, data)

    E0 = float(data.energy[0]) + float(data.energy[1])
    assert abs(E0) > 1e-6, (
        "Initial total energy is effectively zero — "
        "the energy flag may not have taken effect."
    )

    dt = float(model.opt.timestep)
    n_steps = round(10.0 / dt)
    _step(model, data, n_steps)

    # Recompute final energy via mj_forward to ensure a fresh reading.
    mujoco.mj_forward(model, data)
    E_final = float(data.energy[0]) + float(data.energy[1])

    drift_pct = abs(E_final - E0) / abs(E0) * 100.0
    assert drift_pct < 1.0, (
        f"Energy drift over 10 s: {drift_pct:.3f} % > 1 %  "
        f"(E₀={E0:.6f} J  E_final={E_final:.6f} J)"
    )


# ---------------------------------------------------------------------------
# Test 4 — Contact stability (stacked boxes)
# ---------------------------------------------------------------------------

def test_contact_stability():
    """
    Three unit-mass boxes stacked on a floor must not drift more than 1 mm
    horizontally or vertically in any direction for 3 s after an initial
    2 s settling phase.

    This validates the contact solver, friction cone, and noslip iterations.
    """
    half = 0.1   # box half-size → box is 0.2 m on each side

    xml = f"""
    <mujoco model="stack">
      <option {_OPTS} gravity="0 0 -9.81"/>
      <default>
        <geom friction="1 0.05 0.001" condim="4"
              solref="0.005 1" solimp="0.9 0.95 0.001"/>
      </default>
      <worldbody>
        <geom name="floor" type="plane" size="0 0 0.05"
              friction="1.5 0.1 0.01" condim="6"/>
        <body name="box1" pos="0 0 {half}">
          <freejoint/>
          <geom type="box" size="{half} {half} {half}" mass="1"/>
        </body>
        <body name="box2" pos="0 0 {3 * half}">
          <freejoint/>
          <geom type="box" size="{half} {half} {half}" mass="1"/>
        </body>
        <body name="box3" pos="0 0 {5 * half}">
          <freejoint/>
          <geom type="box" size="{half} {half} {half}" mass="1"/>
        </body>
      </worldbody>
    </mujoco>
    """
    model, data = _load(xml)
    dt = float(model.opt.timestep)

    # Phase 1: settle for 2 s.
    _step(model, data, round(2.0 / dt))
    mujoco.mj_forward(model, data)

    # Record settled positions.  Bodies 1–3; body 0 is the world body.
    settled = np.array([data.xpos[b].copy() for b in range(1, 4)])

    # Phase 2: hold for another 3 s.
    _step(model, data, round(3.0 / dt))
    mujoco.mj_forward(model, data)

    for idx, b in enumerate(range(1, 4)):
        final = data.xpos[b]
        drift_mm = np.abs(final - settled[idx]) * 1000.0
        for axis, label in enumerate("XYZ"):
            assert drift_mm[axis] < 1.0, (
                f"box{idx + 1} drifted {drift_mm[axis]:.2f} mm in {label} "
                f"(> 1 mm) after settling"
            )


# ---------------------------------------------------------------------------
# Test 5 — Actuator settling
# ---------------------------------------------------------------------------

def test_actuator_settling():
    """
    A gravity-loaded 1-DOF arm with a position actuator must:
      (a) Settle to within 5 % of the commanded target angle.
      (b) Come to rest (|qdot| < 0.01 rad/s) within 5 s.

    kp=200 → gravity-induced steady-state error ≈ 0.2 % (well within 5 %).
    joint damping=5.0 → ζ ≈ 2.5 (overdamped, no oscillation).
    No kv on the actuator — velocity shaping is handled by joint damping so
    the test is not sensitive to MuJoCo version differences in PD actuator gain
    interpretation.
    """
    kp = 200.0
    q_target = 1.0   # rad

    xml = f"""
    <mujoco model="arm1dof">
      <option {_OPTS} gravity="0 0 -9.81"/>
      <worldbody>
        <body name="arm" pos="0 0 1">
          <!-- Pivot 1 m above origin; arm hangs in -Z at rest. -->
          <joint name="shoulder" type="hinge" axis="0 1 0"
                 damping="5.0" frictionloss="0.02" armature="5e-4"
                 range="-3.14159 3.14159"/>
          <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03" mass="0.3"/>
        </body>
      </worldbody>
      <actuator>
        <position name="shoulder_pos" joint="shoulder"
                  kp="{kp}"
                  ctrllimited="true" ctrlrange="-3.14159 3.14159"
                  forcerange="-300 300"/>
      </actuator>
    </mujoco>
    """
    model, data = _load(xml)

    data.ctrl[0] = q_target
    dt = float(model.opt.timestep)
    _step(model, data, round(5.0 / dt))

    q_final = float(data.qpos[0])
    qdot_final = abs(float(data.qvel[0]))
    pos_err_pct = abs(q_final - q_target) / abs(q_target) * 100.0

    assert pos_err_pct < 5.0, (
        f"Arm settled to {q_final:.4f} rad, target {q_target} rad "
        f"(error {pos_err_pct:.2f} % > 5 %)"
    )
    assert qdot_final < 0.01, (
        f"Arm still moving at t=5 s: qdot={qdot_final:.5f} rad/s > 0.01 rad/s"
    )


# ---------------------------------------------------------------------------
# Split-servo MJCF fixtures
#
# Body indices (worldbody depth-first):
#   0 = world  1 = base  2 = servo_body  3 = servo_horn  4 = arm
# Joint 0 = servo_1 (the hinge)
# ---------------------------------------------------------------------------

# Tests 6-8: Z-axis hinge.  Arm placed 0.15 m off the Z rotation axis so
# its XY world position is a direct readout of horn angle.
#
# The heavier, further-out arm raises the effective rotational inertia to
# ~0.011 kg·m², which keeps both system poles well above the 2 ms timestep
# Nyquist limit and eliminates the numerical steady-state error that occurs
# with high joint damping + small armature.
_SPLIT_Z_XML = """
<mujoco model="split_servo_z">
  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="base" pos="0 0 0.1">
      <geom type="box" size="0.05 0.05 0.005" mass="1.0"/>
      <body name="servo_body" pos="0 0 0.02">
        <geom type="box" size="0.02 0.015 0.018" mass="0.190"/>
        <body name="servo_horn" pos="0 0 0.008">
          <joint name="servo_1" type="hinge" axis="0 0 1"
                 range="-3.14159 3.14159"/>
          <geom type="cylinder" size="0.01 0.002" mass="0.010"/>
          <body name="arm" pos="0.15 0 0.03">
            <geom type="box" size="0.01 0.01 0.01" mass="0.5"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="servo_1_pos" joint="servo_1"
              kp="100.0" kv="5.0"
              ctrllimited="true" ctrlrange="-3.14159 3.14159"
              forcerange="-1000 1000"/>
  </actuator>
</mujoco>
"""

# Test 9: Y-axis pitch.  Split servo where servo_body CoM is 0.03 m BELOW
# the hinge and arm CoM is 0.10 m ABOVE the hinge.  Both contribute to X-CoM
# when pitching, so old (full-housing-rotates) and new (horn-only) models
# produce measurably different CoM traces.
#
# Body indices: 0=world 1=base 2=servo_body 3=servo_horn 4=arm
_SPLIT_Y_XML = """
<mujoco model="split_servo_pitch">
  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="base" pos="0 0 0.2">
      <geom type="box" size="0.08 0.08 0.01" mass="0.5"/>
      <body name="servo_body" pos="0 0 0.04">
        <geom type="box" size="0.02 0.015 0.03" mass="2.0"/>
        <body name="servo_horn" pos="0 0 0.03">
          <joint name="servo_1" type="hinge" axis="0 1 0"
                 damping="0.2" armature="1e-3" range="-3.14159 3.14159"/>
          <geom type="cylinder" size="0.01 0.003" mass="0.1"/>
          <body name="arm" pos="0 0 0.1">
            <geom type="capsule" fromto="0 0 -0.05 0 0 0.05"
                  size="0.008" mass="0.1"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="servo_1_pos" joint="servo_1"
              kp="50.0" kv="5.0"
              ctrllimited="true" ctrlrange="-3.14159 3.14159"
              forcerange="-20 20"/>
  </actuator>
</mujoco>
"""

# Single-link equivalent of _SPLIT_Y_XML: full 2.1 kg servo rotates as one body.
# Geom is offset +0.03 m above the hinge so the housing CoM is off-axis and
# contributes measurably to the CoM trace when pitching.
# Body indices: 0=world 1=base 2=servo 3=arm
_OLD_Y_XML = """
<mujoco model="old_servo_pitch">
  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="base" pos="0 0 0.2">
      <geom type="box" size="0.08 0.08 0.01" mass="0.5"/>
      <body name="servo" pos="0 0 0.04">
        <joint name="servo_1" type="hinge" axis="0 1 0"
               damping="0.2" armature="1e-3" range="-3.14159 3.14159"/>
        <!-- Geom center at +0.03 m above joint so housing CoM has a moment arm -->
        <geom type="box" size="0.02 0.015 0.03" pos="0 0 0.03" mass="2.1"/>
        <body name="arm" pos="0 0 0.1">
          <geom type="capsule" fromto="0 0 -0.05 0 0 0.05"
                size="0.008" mass="0.1"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="servo_1_pos" joint="servo_1"
              kp="50.0" kv="5.0"
              ctrllimited="true" ctrlrange="-3.14159 3.14159"
              forcerange="-20 20"/>
  </actuator>
</mujoco>
"""


# ---------------------------------------------------------------------------
# Test 6 — Split servo: zero command, no drift
# ---------------------------------------------------------------------------

def test_split_servo_zero_command():
    """
    With ctrl=0 the split servo must remain motionless for 5 s.

    Validates:
    - servo_body world position stable to < 0.01 mm (housing rigid)
    - horn joint angle stays at 0 ± 0.001 rad (no gravity-induced creep)
    """
    model, data = _load(_SPLIT_Z_XML)
    mujoco.mj_forward(model, data)   # populate xpos/xquat before recording baseline

    # Body 2 = servo_body (see fixture comment above)
    body_pos_0 = data.xpos[2].copy()

    data.ctrl[0] = 0.0
    _step(model, data, round(5.0 / float(model.opt.timestep)))
    mujoco.mj_forward(model, data)

    drift_mm = np.abs(data.xpos[2] - body_pos_0) * 1000.0
    assert np.all(drift_mm < 0.01), (
        f"servo_body drifted {drift_mm} mm under zero command — housing not rigid"
    )

    q_final = float(data.qpos[0])
    assert abs(q_final) < 0.001, (
        f"Horn drifted to {q_final:.4f} rad with zero command"
    )


# ---------------------------------------------------------------------------
# Test 7 — Split servo: body orientation fixed while horn rotates
# ---------------------------------------------------------------------------

def test_split_servo_body_stays_fixed():
    """
    Commanding a 1.0 rad target must rotate only the horn.

    Validates:
    - servo_body orientation (xquat) unchanged to within 1e-5 norm
    - horn joint angle reaches target within 5 %
    - arm world position has moved (downstream chain driven by horn)
    """
    model, data = _load(_SPLIT_Z_XML)
    mujoco.mj_forward(model, data)   # populate xpos/xquat before recording baseline

    # Body 2 = servo_body, body 4 = arm
    body_quat_0 = data.xquat[2].copy()
    arm_pos_0 = data.xpos[4].copy()

    data.ctrl[0] = 1.0
    _step(model, data, round(5.0 / float(model.opt.timestep)))
    mujoco.mj_forward(model, data)

    # Housing orientation must be unchanged
    quat_diff = np.linalg.norm(data.xquat[2] - body_quat_0)
    assert quat_diff < 1e-5, (
        f"servo_body quaternion changed by {quat_diff:.2e} — "
        "housing is rotating with horn (split not working)"
    )

    # Horn must have reached target
    q_final = float(data.qpos[0])
    err_pct = abs(q_final - 1.0) / 1.0 * 100.0
    assert err_pct < 5.0, (
        f"Horn settled to {q_final:.4f} rad, target 1.0 rad (error {err_pct:.1f} %)"
    )

    # Arm must have moved (driven by horn)
    arm_displacement_mm = np.linalg.norm(data.xpos[4] - arm_pos_0) * 1000.0
    assert arm_displacement_mm > 5.0, (
        f"Arm barely moved ({arm_displacement_mm:.1f} mm) — horn may not be driving chain"
    )


# ---------------------------------------------------------------------------
# Test 8 — Split servo: housing static under sine command
# ---------------------------------------------------------------------------

def test_split_servo_sine_housing_static():
    """
    Under a 0.5 Hz sine command over 3 s the housing body must stay
    completely static while the horn oscillates.

    Validates the split behaviour across a dynamic trajectory, not just
    at a settled end-point.
    """
    model, data = _load(_SPLIT_Z_XML)
    mujoco.mj_forward(model, data)   # populate xpos/xquat before recording baseline
    dt = float(model.opt.timestep)
    body_pos_0 = data.xpos[2].copy()
    body_quat_0 = data.xquat[2].copy()

    max_body_drift_mm = 0.0
    max_body_quat_diff = 0.0
    q_values: list[float] = []

    freq = 0.5   # Hz
    amp  = 1.0   # rad

    n_steps = round(3.0 / dt)
    for i in range(n_steps):
        t = i * dt
        data.ctrl[0] = amp * math.sin(2.0 * math.pi * freq * t)
        mujoco.mj_step(model, data)
        if i % 50 == 0:   # sample at 10 Hz
            mujoco.mj_forward(model, data)
            drift_mm = float(np.linalg.norm(data.xpos[2] - body_pos_0)) * 1000.0
            quat_diff = float(np.linalg.norm(data.xquat[2] - body_quat_0))
            max_body_drift_mm = max(max_body_drift_mm, drift_mm)
            max_body_quat_diff = max(max_body_quat_diff, quat_diff)
            q_values.append(float(data.qpos[0]))

    assert max_body_drift_mm < 0.05, (
        f"servo_body translated {max_body_drift_mm:.3f} mm during sine — housing not rigid"
    )
    assert max_body_quat_diff < 1e-5, (
        f"servo_body rotated (quat diff {max_body_quat_diff:.2e}) during sine — "
        "housing is co-rotating with horn"
    )

    # Horn must have actually oscillated — peak angle should exceed 0.5 rad
    q_peak = max(abs(q) for q in q_values)
    assert q_peak > 0.5, (
        f"Horn peak angle only {q_peak:.3f} rad during 1.0-rad-amp sine — "
        "actuator may not be driving the joint"
    )


# ---------------------------------------------------------------------------
# Test 9 — Split servo: CoM amplitude < single-link model
# ---------------------------------------------------------------------------

def test_split_servo_com_vs_single_link():
    """
    Under identical sine commands the split model's base-subtree CoM must
    oscillate less than the single-link (old) model.

    Physics: in the old model the 2.1 kg housing rotates, sweeping its CoM
    through the XZ plane.  In the split model only the 0.1 kg horn + 0.1 kg
    arm rotate; the 2.0 kg housing stays fixed.

    Expected ratio of CoM amplitudes: ≈ 0.1–0.25 × old model.
    The test requires new amplitude < 0.5 × old amplitude (conservative bound).
    """
    dt_split = float(_load(_SPLIT_Y_XML)[0].opt.timestep)
    dt_old   = float(_load(_OLD_Y_XML)[0].opt.timestep)
    assert abs(dt_split - dt_old) < 1e-9, "timestep mismatch between fixtures"

    freq = 0.3   # Hz — slow enough for both models to track the command
    amp  = 0.8   # rad

    def _run_com(xml: str, body_subtree_idx: int = 1) -> float:
        """Simulate 5 s and return peak subtree CoM displacement in X from initial."""
        m, d = _load(xml)
        dt = float(m.opt.timestep)
        mujoco.mj_forward(m, d)
        com_x_0 = float(d.subtree_com[body_subtree_idx, 0])
        peak = 0.0
        n = round(5.0 / dt)
        for i in range(n):
            t = i * dt
            d.ctrl[0] = amp * math.sin(2.0 * math.pi * freq * t)
            mujoco.mj_step(m, d)
            if i % 20 == 0:
                mujoco.mj_forward(m, d)
                disp = abs(float(d.subtree_com[body_subtree_idx, 0]) - com_x_0)
                peak = max(peak, disp)
        return peak

    peak_split = _run_com(_SPLIT_Y_XML, body_subtree_idx=1)
    peak_old   = _run_com(_OLD_Y_XML,   body_subtree_idx=1)

    assert peak_old > 1e-4, (
        f"Old model CoM barely moved ({peak_old*1000:.2f} mm) — "
        "check fixture geometry: servo body should be off-axis from hinge"
    )
    assert peak_split < peak_old * 0.5, (
        f"Split servo CoM amplitude ({peak_split*1000:.2f} mm) is not significantly "
        f"smaller than single-link ({peak_old*1000:.2f} mm).  "
        f"Ratio = {peak_split/peak_old:.2f}, expected < 0.50.  "
        "The servo body may still be rotating with the horn."
    )

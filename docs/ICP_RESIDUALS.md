# ICP Residuals — What Step 2 Polish Closed and What's Left

**Status:** Step 2 + polish (confident-cap) shipped on `engine-gaps-integration`. Remaining residuals are documented here as known acceptable, not as bugs to fix in the current engine sprint.

## What changed in the polish pass

Pre-polish (initial Step 2 ship):
- 20 nudges, total 60.00 mm
- All clamped at 3 mm `default-cap`

Post-polish (adaptive `confident-cap`):
- 33 nudges, total 100.45 mm
- 4 firings of `confident-cap=15mm` on hip abduction servos — each at 3.36 mm (was 3.00 mm clamped)
- Default 3 mm cap still applies when the percentile spread is wide or the paired ratio is low

Visible improvement: hip-joint chamfer gaps tighter than image 18; coupler now visibly recessed into servo top (intentional ~3 mm interpenetration that hides the chamfer transition).

## What's still residual after the polish

Three patterns in the [icp][trace] log that the current single-pass 1-D ICP cannot close further. None of these are tuning issues — they require different algorithmic approaches.

### A. Rotated mates (knee, hip pitch with `attach_rpy`)

```
[icp][trace] actuator_servo_high_torque_12 ... structural_servo_coupler_disc.bottom→actuator_servo_high_torque.top
  paired=76/80 | gap(mm) min=-9.60 p50=-0.08 p90=16.65 max=19.33 → nudge=3.00mm (default-cap)
```

26 mm percentile spread because the child's contact face is rotated relative to the parent's normal. Sampling 64 points and projecting onto the parent axis returns wildly different distances depending on which corner of the rotated face the sample landed on. The confident-cap correctly stays conservative (signal too noisy to trust an aggressive nudge).

**Why 1D ICP can't close this:** the algorithm only solves for an axial translation. The actual misalignment includes a rotational component the rotated child introduces. Closing this needs full 3D ICP (rotation + translation), which is ~10× more work and only relevant for these few rotated joints.

### B. Low pair coverage on small contact faces

```
[icp][trace] structural_extrusion_2020_10 ... actuator_servo_high_torque.bottom→structural_extrusion_2020.top
  r=14.4mm paired=8/80 pHit=80 cHit=8 | gap p50=4.12 → nudge=3.00mm (default-cap)
```

Only 10% of child samples found a parent surface within range. The 14.4 mm sample radius is wider than the 20 mm extrusion top, so 88% of child samples miss the parent. Tight percentile spread (p50=p90=4.12 mm) suggests the real gap is 4 mm flat, but the confident-cap threshold (≥75% paired) won't fire on 10% coverage.

**Why this can't close further with current sampling:** the radius is hardcoded relative to the parent's face extent (0.8× face_extent), not the smaller of the two faces. Adaptive radius (shrink to match the smaller contact face) would push paired ratio above 75%, allowing confident-cap to fire and close the remaining ~1 mm. Medium-effort fix; medium-impact result (1 mm visible improvement on extrusion edges).

### C. Low pair coverage on baseplate-top components

```
[icp][trace] power_lipo_4s_5000_2 ... structural_baseplate_large.top→power_lipo_4s_5000.bottom
  r=102.1mm paired=7/80 | gap p50=-2.65 p90=12.52 → nudge=3.00mm (default-cap)
```

Sample radius derived from the *baseplate*'s 175 mm half-extent → 102.1 mm. The lipo / SBC / IMU contact faces are 137 / 85 / 16 mm wide, so 92% of child samples sample empty parent space outside their actual contact area. Real gap is small (p50=-2.65 mm = mild overlap, expected), but the noisy spread keeps confident-cap from firing.

This is the same root cause as **B** — same fix would help.

## What we are NOT going to do (and why)

| Approach | Why not (now) |
|---|---|
| **Full 3D ICP** | Solves the rotated-mate case but is 10× the implementation cost. Only 4 mates per quadruped show the issue, and the visible artifact is small (~2 mm angular gap at knees). Re-evaluate if a future workload makes rotated mates a primary visual concern. |
| **Multi-pass iterative ICP** | Would close 5–6 mm of large gaps (3 mm + 3 mm) by running the algorithm twice. Most remaining gaps are <2 mm, so the marginal win is small. Adds bake/persist cycles that complicate reconcile. |
| **Adaptive sample radius** (B + C) | Real win: closes the residual ~1 mm on extrusion edges and possibly the lipo/SBC samples. ~1 hour to implement. **Reasonable next iteration if any of these residuals show up as user complaints.** Not blocking ship. |

## Residual quality bar

- Hip abduction joints (image 22): tight, intentional 3.36 mm interpenetration.
- Knee + hip pitch joints: ~2 mm visible angular gap, conservative-by-design.
- Extrusion-on-servo edges: ~1 mm uncovered.
- Baseplate-top components: visually flush (gaps in log are sample-coverage artifacts, not real gaps).
- Foot pads to extrusion ends: clean (reconcile handles those at -2.96 mm).

Net: vs the original Layer-5 quadruped (image 16), this state is significantly tighter. Vs Zoo.dev / curated CAD assemblies, it's still recognizably "AI-assembled with sub-mm-grade alignment but not perfect."

## When to revisit

- If a user's design uses many rotated mates (humanoid arms, robot hands), 3D ICP becomes worth implementing.
- If users report visible gaps on the 1 mm extrusion-edge residual specifically, ship adaptive sample radius (B + C fix).
- If the GLBs themselves get re-authored with proper body/shaft splits, the placement-side accuracy improves and ICP becomes a smaller correction in the first place.

For now: shipped, stable, well-instrumented. Move on to Session 5 (multi-child tangent) and the AI camera-bracket fix as separately-tracked work.

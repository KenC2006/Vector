# Generation eval report

| prompt | overall | compile | contact | settle | interpenetration | ground | structure | mass |
|---|---|---|---|---|---|---|---|---|
| dog | 0.90 | 1.00 | 1.00 | 1.00 | 1.00 | 0.00 | 1.00 | 1.00 |
| humanoid | 0.87 | 1.00 | 1.00 | 1.00 | 0.00 | 1.00 | 0.67 | 1.00 |
| hexapod | 0.80 | 1.00 | 1.00 | 1.00 | 0.00 | 0.32 | 0.67 | 1.00 |
| rover | 1.00 | 1.00 | 1.00 | 1.00 | 0.97 | 1.00 | 1.00 | 1.00 |
| arm | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| sculpture | 0.76 | 1.00 | 0.98 | 1.00 | 0.00 | 0.00 | 0.67 | 1.00 |

## dog
- ground: non-foot lowest links: `shin_fl`, `shin_fr`, `shin_rl`, `shin_rr`

## humanoid
- interpenetration: 467.12cm³ total; worst `pelvis`×`upper_arm_l` (228.97cm³)
- structure: symmetry: 12/24 off-center links lack a mirrored counterpart

## hexapod
- interpenetration: 130.07cm³ total; worst `knee_1`×`shin_5` (37.36cm³)
- ground: non-foot lowest links: `shin_1`, `shin_4`
- structure: symmetry: 11/35 off-center links lack a mirrored counterpart

## rover
- interpenetration: 3.77cm³ total; worst `camera_1`×`sbc_1` (3.77cm³)

## sculpture
- contact: 0 floating / 1 buried — `tail_servo_0` buried (gap -22.5mm)
- interpenetration: 169.39cm³ total; worst `claw_shoulder_l`×`claw_l` (59.13cm³)
- ground: non-foot lowest links: `leg_shin_0`, `leg_shin_1`, `leg_shin_2`, `leg_shin_3`, `leg_shin_4`, `leg_shin_5`
- structure: symmetry: 17/43 off-center links lack a mirrored counterpart

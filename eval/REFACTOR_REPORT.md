# Assembler refactor — final report (2026-06-09)

Branch `refactor/assembler`. Eight workstreams (WS0–WS7) landed; every step
verified against the 19-suite `npm --prefix src run check` gate plus the
generation-eval replay gate (`scripts/eval_generation.py --replay`).

## What changed

| WS | Change |
|---|---|
| 0 | Generation-quality eval harness (compile/contact/interpenetration/ground/MuJoCo-settle/structure/mass metrics) + frozen synthetic baselines |
| 1 | Archetype modes deleted end-to-end; radial multi-child distribution (4-corner parity); raw `xyz`/`rpy` + `link_geometry` always available; `placement_offset_mm`/`splay_angle_deg` deleted; rest poses apply verbatim (no auto-mirroring) |
| 2 | Auto-repairs → structured warnings with `suggested_repair` fed back to Claude; hard errors limited to structural impossibilities; one shared port-compat check across both edit paths |
| 3 | `link_geometry` first-class: bounds/connectors/collision/mass derive from the primitive union; per-primitive `<collision>` emission; donor-preset connectors no longer override shell surfaces |
| 4 | Ports derived FROM connectors; port classes are authored catalog data (`cls`/`single`, 103 fields migrated); geometric face→port resolution; string-prefix heuristics deleted |
| 5 | Primitive anchors (`attach_primitive`/`attach_anchor`): children mount on named primitives' real surfaces; BAD_PRIMITIVE_REF/BAD_ANCHOR/PRIMITIVE_ON_NON_CAD_BODY validators with did-you-mean hints; TS↔Python anchor parity corpus; anchor tables in the AI's spatial context |
| 6 | Orientation grammar fixed: `horizontal+N` parses, `horizontal` works on all six faces, numeric yaw works on side faces, yaw threads into anchored mates |
| 7 | SYSTEM_PROMPT full rewrite (22.8KB → 9.3KB, real semantics only, one placement method per component, warning-feedback contract); stale reject-claims in edit-tool descriptions fixed; `VECTOR_AI_MODEL` env override |

## Eval movement (frozen old-style baselines, replayed through final code)

| prompt | baseline | final | drivers |
|---|---|---|---|
| dog | 0.90 | 0.87 | symmetry dip from verbatim rest poses (intended: per-side authoring is now the contract) |
| humanoid | 0.77 | 0.97 | interpenetration 0.00→0.99 — arms mount on the real torso shell |
| hexapod | 0.80 | 0.83 | interpenetration 0.00→0.27 — radial legs replace center-slot pileup |
| rover | 1.00 | 1.00 | — |
| arm | 1.00 | 1.00 | — |
| sculpture | 0.76 | 0.77 | buried tail servo fixed (mounts on carapace top) |

**New-style demos** (`eval/newstyle/`, authored the way the new prompt teaches —
named primitives + anchors + per-side rest poses):

- sculpture: **0.77 → 0.86** (interpenetration 169cm³ → 13cm³; claws/tail on
  anchored sockets; contact 1.00)
- humanoid: 0.89 with full crouch rest poses (vs 0.97 straight-leg) — the bent
  stance exposes the remaining known issue below.

## Remaining known issues

1. **Crouch geometry vs ground contact**: with authored leg bends, shin-tip
   envelopes dip below the auto-leveled foot pads (dog ground metric 0,
   new-style humanoid ground 0.32). The foot-leveling pass positions the pad
   on the shin's end face but doesn't account for the bent chain's lowest
   point. Candidate fix: ground-offset from the true min-z of the whole chain
   (it already does for the robot — the issue is the pad isn't the contact).
2. **Live before/after eval blocked**: `ANTHROPIC_API_KEY` in `.env` is
   rejected (401 invalid x-api-key) — rotate the key, then run:
   `python scripts/eval_generation.py --live --out eval/baselines` (captures
   real baselines) and compare future changes against them.
3. Geometric symmetry metric is strict about radial layouts (reports
   asymmetric counterparts for odd-position legs); treat as advisory for
   creature designs.

## How to verify

```
npm --prefix src run check                                   # 19 suites
python scripts/eval_generation.py --replay eval/baselines \
    --out eval/runs/<name> --compare eval/baselines/scores.json
python scripts/eval_generation.py --replay eval/newstyle --out eval/runs/<name>
```

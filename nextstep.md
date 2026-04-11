# Next steps — placement, rotation, builder

### Purpose

Unify **placement UX** and **URDF correctness** so the **preview transform = committed transform** (position **and orientation**), with **no hidden global frame hacks**, and a **non-clunky** build loop.

---

### Core interaction model

- **Carry mode** after picking a toolbox part (and optionally "reposition" for an existing part): show a **ghost** in **world space**; **do not require a pre-selected parent** to place.
- **Commit attach** by targeting a **mount/node on any link** (global targets, not parent-first).
- **Cancel** discards pending placement with **no partial URDF edits**.
- Prefer **one pipeline** for: toolbox → place, scene → detach/replace.

---

### Spatial freedom

- While carrying / pre-commit: manipulation is **world-space** (or explicit toggle world/local), **not clamped** to parent AABB/pivot heuristics.
- Optional **free-space drop** at ghost transform (explicit world anchor / temporary root attachment) if you need off-robot staging.

---

### Snap / computePlacement (make solver legible)

- Snap proposes transforms; user can **override**.
- Show **top candidate mounts** (highlight) + **Tab/Shift+Tab** cycle through parent targets; **hysteresis** to prevent flicker.
- **Child mount selection**: Tab also cycles which face of the *carried* part connects — not just the target side.
- Short **rejection reasons** on-screen: occupied, incompatible interface, too far, axis/type mismatch.
- After **manual nudge/rotate**, **freeze auto re-solve** until **Re-snap** (manual beats solver).

---

### Keyboard + discoverability (must have UI hints)

- **Arrows**: nudge (Shift = larger step); include **Z** (PgUp/PgDn or Alt+arrows).
- **R**: rotate mode / stepped rotations; include quick **±90°** about snap normal when relevant.
- **Enter** commit when snapped (optional), **Esc** cancel.
- **Placement HUD**: step sizes, Δxyz/Δrpy, buttons **Snap / Clear snap / Commit / Cancel**, plus on-screen shortcut line.

---

### Rotation & URDF correctness (non-negotiable)

- Treat three frames explicitly: **editor/world**, **parent mount/node frame**, **child mount frame** on inserted part.
- **Default snap** aligns child→parent mounts; **user rotation composes** on top as an explicit offset.
- Persist mating as real URDF semantics: primarily **joint `<origin xyz rpy>`** (fixed joints especially); avoid "looks right in mesh only."
- **Joint axis reconciliation on snap**: when a revolute/prismatic joint is snapped at a new orientation, recompute `<axis xyz>` so it is expressed correctly in the new parent frame. A motor snapped sideways must not still claim `axis 0 0 1`.
- **URDF ↔ viewer convention** (Y-up vs Z-up): any viewer compensation must **round-trip**; acceptance: export → re-import / second viewer matches orientation. The current `groundRobot()` Y-shift is a visual hack and must be replaced with a proper convention transform.

---

### Data you must store per commit (auditability)

- Parent link + **which mount/node** (parent side); child part + **which child mount** (child side).
- Final relative transform between those frames → maps to **committed joint origin** (+ axis defs as needed).
- If this can't be reconstructed from saved URDF alone, the model is under-specified.

---

### Node type system (complete what's already defined)

`attachmentNodes.ts` already defines `shaft`, `bore`, `rail`, `mount_face`, `generic` classes — only `mount_face` is active. Complete the matching logic:
- `shaft` ↔ `bore` (coaxial, aligns Z axes)
- `rail` ↔ `rail` (sliding, aligns X and Z)
- `mount_face` ↔ `mount_face` (current, face-to-face)
- `generic` accepts any type (fallback)

Rejection reason "incompatible interface" should name the actual class mismatch.

---

### Sim mode (broken, needs scoping)

The current sim animation hardcodes joint names (`shoulder_pan`, `shoulder_lift`, etc.) — it will not work on any user-built robot. Fix:
- Drive **all revolute/prismatic joints** found in the current URDF, not a hardcoded list.
- Use each joint's `<limit lower/upper>` to bound motion; use `<axis xyz>` for direction.
- Sim mode should be clearly separated from the builder (no accidental joint animation while placing parts).

---

### Collision visibility

Collision bodies are never shown. Add a **toggle** (e.g. `C` key or toolbar button) to render collision primitives semi-transparently. This is required for the validation gates (backlog item 6) to be actionable — you can't fix what you can't see.

---

### Builder feature backlog (prioritized)

1. **Joint axis reconciliation on snap** — revolute/prismatic joints must recompute `<axis>` in new parent frame at commit time.
2. **Carry mode + ghost + commit/cancel** — no partial edits on cancel; ghost tracks world cursor.
3. **HUD + nudge/rotate controls + solver transparency** — candidates, rejection reasons, freeze-after-manual.
4. **Global attach targets + Tab cycle** — cycle both parent mount targets AND child mount face selection.
5. **Y-up/Z-up round-trip** — replace `groundRobot()` hack; export → re-import must match orientation.
6. **Free-space drop** — explicit world anchor / temporary root for off-robot staging during carry.
7. **Undo/redo for graph edits** — current undo covers URDF text; add structural graph undo with URDF diff view.
8. **Node type system completion** — shaft/bore, rail/rail matching with correct axis alignment.
9. **Sim mode fix** — drive all URDF joints from their actual limits/axes; decouple from builder.
10. **Collision visibility toggle** — render collision primitives semi-transparently on demand.
11. **Validation gates** — topology check, inertia/collision completeness per link, tied to build actions.
12. **Replace-in-place / duplicate subtree / mirror patterns** — replace swaps link definition + re-snaps children; duplicate preserves relative transform.
13. **Export bundle correctness** — URDF + meshes + manifests; MuJoCo load test as acceptance criterion.

---

### Definition of done (binary checks)

- No **silent parenting** based on incidental selection.
- **Preview orientation matches** committed URDF joint origins.
- **MuJoCo load + another URDF viewer** agree with editor orientation (run as explicit test, not eyeballed).
- Users can **nudge/rotate** without fighting snap (override + re-snap).
- Placement never feels **bound to parent bounding** behavior during pre-commit.
- **Joint axes** of revolute joints are geometrically correct after any snap or reposition.
- **Sim mode** animates all revolute joints in any user-built URDF, not just the sample robot.

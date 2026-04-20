# Vector Master Improvement Plan

Engine-focused improvements that strengthen Vector's core AI flow: graph/topology reasoning, placement, and the Claude↔validator↔Gemini loop. Synthesizes findings from competitive research (FreeCAD, forgeCAD, Onshape, Adam) against Vector's current state.

## Scope

**In:** Engine work — graph/topology quality, placement correctness, AI grounding, edit-path integrity. The parts that determine whether Vector produces good designs.

**Out:**
- UX features (Plan/Act gates, parameter sliders, click-to-scope chat) — the editor already shows highlighted changes, undo/redo + checkpoints already cover the "did this go wrong" case. Revisit after the engine is solid.
- Skill bundles, MCP servers, external-agent integrations — standalone-app focus first.
- Demo-path-specific patches (L-bracket rotation P1, torso mesh M1, shoulder π-flip C1, pan-tilt D1, etc.). Handled by adding components and tightening the system prompt as new robot types come up. Tracked in `project_known_issues.md`.

## Workstreams

Four engine workstreams, ordered by dependency.

---

## 1. AssemblyGraph Preservation Across Edits

**Status:** Not started. Prerequisite for #2.

### What
Stop relying on `urdfToAssemblyGraph` as the only source of truth between turns. Persist the original `AssemblyGraph` alongside the generated URDF so multi-turn edits operate on the lossless original, not a re-parsed reconstruction.

### Why
The current round-trip is lossy. Per memory (`project_vector_improvement_plan.md` urdfToAssemblyGraph section):
- `orientation`, `elevation_angle`, `length_mm` don't survive the round-trip — not recoverable from URDF alone.
- `ground_offset` is hardcoded to `true` on reverse-parse.
- Links with non-standard names (not matching `component_id_N`) won't resolve to presets.

Every multi-turn edit silently degrades the design — orientation flips back to default, elevation angles lost, lengths reset. **This is the foundation problem.** Tool-call edits (#2) will compound the degradation if the source-of-truth issue isn't fixed first.

### How
1. Store the canonical `AssemblyGraph` object alongside the URDF whenever a design is generated or edited. Persist with the design state, not just in memory.
2. Edit operations (`design_robot`, `modify_topology`, future tool calls) read from and write to the canonical graph. URDF becomes a serialized projection, not the source.
3. **What goes to Claude on edit-retry:** the `AssemblyGraph` (or a graph diff) — never the URDF. URDF is purely a renderer/export serialization; sending it to the LLM wastes tokens and reintroduces the lossy-round-trip problem we're solving here. Claude reasons over the graph, edits the graph, and the URDF is regenerated downstream.
4. `urdfToAssemblyGraph` stays — needed for the import-existing-URDF path and as a fallback — but is no longer the default ingestion route during edit chains.
5. Add a structured equality check (`graphsEquivalent(a, b)`) so when round-trip happens (rare paths), divergence is detected and logged rather than silently accepted.

### Risks
- State management complexity: now two representations need to stay in sync. Mitigate by treating URDF as derived/read-only outside the serializer.
- Existing edit paths assume URDF→graph→edit→URDF. Refactor needs to preserve their semantics.

### Files
- `src/src/urdfAssembly.ts` — add canonical-graph persistence layer.
- `core/ai/claude_client.py` — pass `AssemblyGraph` (not just URDF) on edit-retry context, per master plan WS2.
- `src/src/viewportChat.ts` — store/retrieve graph alongside chat session.

### Done when
- A 5-turn edit chain (initial design + 4 sequential tweaks) preserves all `attach_rpy`, `length_mm`, `orientation`, `elevation_angle` values from the original generation.
- Fixture: round-trip a known-good graph → URDF → graph and assert structural + parametric equality.

---

## 2. Tool-Call Edit Surface

**Status:** Not started. Depends on #1.

### What
Convert Claude's edit output from a freeform `assembly_graph` JSON blob into structured Anthropic tool calls against named graph mutations:
- `add_link(parent_link, preset_id, attach_face, attach_rpy?)`
- `attach_sensor(parent_link, preset_id, mount_face)`
- `replace_component(link_name, new_preset_id)`
- `set_joint(link_name, joint_type, axis, limits?)`
- `remove_link(link_name, reparent_children?)`

Validator runs *per call* rather than once at the end. Invalid mutations are rejected before they enter the graph; structured errors return to Claude for self-correction within the same turn.

### Why
Every competitor that's serious about LLM-driven CAD has converged on this:
- **Onshape** pushes the LLM up to FeatureScript (typed, validatable DSL).
- **forgeCAD** uses named-face/edge selectors as the only edit handles.
- **FreeCAD's strongest plugin (ghbalf)** wraps 48 validated tools as primary, code-gen as fallback.

Current path: Claude emits a full graph, we validate after. A single bad attach poisons the whole output and triggers a redesign. Per-call validation contains the blast radius and gives Claude immediate, actionable feedback.

This also resolves the **detect→enforce gap** in the port system (per `project_vector_improvement_plan.md` anti-patterns): port mismatches currently get logged but assembly proceeds anyway. With per-call validation, port enforcement happens inside the tool dispatch — detection becomes enforcement automatically.

### How
1. Define the tool schema in `core/ai/claude_client.py` — start with 5-8 high-coverage tools that cover ~90% of edits.
2. Refactor `urdfAssembly.ts` to expose each mutation as a pure function with the same signature as the tool. Most logic already lives in `_performPlacement` and helpers — this is restructuring, not new code.
3. Per-call validation: call the relevant subset of `validateTopology` rules after each tool invocation. Port-occupancy enforcement runs inline. On failure, return structured error (with link names + rule code) so Claude self-corrects mid-conversation.
4. Keep the existing `assembly_graph` path as fallback for initial design generation. Tool calls are for *edits*. Don't try to make Claude build a whole quadruped via 50 tool calls — that's what `design_robot` is for.

### Risks
- Over-decomposing edits inflates token count. Keep tool granularity coarse (link-level, not field-level).
- Tool schemas can drift from `urdfAssembly.ts`. Single source of truth: generate the JSON schema from the TS function signatures, or vice versa.
- Existing redesign-retry heuristic (`topoFailures > 0` per `project_validator_ui_session_landed.md`) needs to coexist with mid-conversation tool-error recovery. Not in conflict, but worth threading carefully.

### Files
- `core/ai/claude_client.py` — tool definitions, dispatch.
- `src/src/urdfAssembly.ts` — mutation entrypoints + per-call validation hooks.
- `src/src/viewportChat.ts` — message loop, error feedback to Claude.

### Done when
- "Add a depth camera to the head" produces a single `attach_sensor` call instead of regenerating the whole graph.
- Validator errors return as structured tool errors and Claude self-corrects within the same turn ≥80% of the time.
- Port mismatches are rejected at the tool boundary — no occupancy increment on mismatched attach.
- Fixture pair: bad tool call rejected with actionable error; valid retry passes.

---

## 3. Gemini Loop Hardening

**Status:** Not started. Independent — can ship alongside #2.

Bundles two related fixes to the Gemini validation loop. They share the same code path (`viewportChat.ts` → `core/server.py` → `ai_validate_assembly`) and reinforce each other: image input gives Gemini stronger signal; classifier filters Gemini's output before it reaches Claude.

### 3a. Reference image pass-through (G3 fix)

**What:** Thread the user's reference image (when uploaded to Claude) through `ai_validate_assembly` so Gemini sees it alongside the rendered output. Currently Gemini judges output in isolation.

**Why:** Per `project_known_issues.md` G3, the validator chain (`viewportChat.ts:838` → `src-tauri/src/lib.rs:466` → `core/server.py:614`) drops the `images` param. Without the reference, Gemini can't flag "you asked for 6 wheels via the photo but got 4." Image-reference ↔ output comparison is impossible. As image input becomes a more central feature, this gap widens.

**How:** ~15 lines across three files. Thread `images` through `ai_validate_assembly` the same way `ai_edit` already does. Prepend to Gemini content list before the screenshots so the reference establishes context first.

**Files:**
- `src/src/viewportChat.ts` — pass images into the validator call.
- `src-tauri/src/lib.rs` — IPC bridge param.
- `core/server.py` — accept and forward `images` to Gemini.

**Done when:** Reference image arrives at Gemini for every validation when one was uploaded with the request. Test: Mars rover with reference photo gets a critique that references the photo's specific features.

### 3b. Critique classifier (G2 + G4 fix)

**What:** A one-shot Haiku classifier that filters Gemini's critiques against catalog/engine capabilities BEFORE passing them to Claude's redesign retry. Drops critiques that demand things Vector can't satisfy.

**Why:** Per G2: Gemini suggests "widen feet to 100mm", "add rocker-bogie", "spread pedestal base", "add display screen" — none exist in the catalog. Claude can't satisfy → keeps redesigning → burns quota and tokens → never converges.

Current mitigation is a regex in `viewportChat.ts` (~line 721) that grows per robot type (G4 — fragile). A real classifier kills both G2 and G4. With G3 landed, Gemini will start producing critiques specifically about reference-vs-output gaps — making feasibility filtering even more important so reference-driven feedback is preserved while infeasible suggestions are dropped.

**How (hybrid — deterministic first, LLM only for ambiguous cases):**
1. **Deterministic pre-filter (first line of defense):**
   - Extract noun phrases / capability keywords from each critique sentence.
   - Fuzzy-match against the catalog (preset names, categories, declared capabilities).
   - **Hard drop:** mentions of components/concepts with zero catalog match (e.g., "rocker-bogie", "display screen", "spreading pedestal base").
   - **Pass through:** mentions that resolve cleanly to existing presets.
   - **Escalate:** ambiguous cases where the critique doesn't name parts directly ("legs are too thin", "torso is wrong proportion").
2. **LLM stage — use Sonnet, not Haiku:** the easy cases were caught deterministically; what survives is genuinely ambiguous. Haiku is too agreeable for nuanced "is this actionable in our catalog" judgments and tends to pass marginal suggestions through. Sonnet's better critical judgment is worth the cost for the remainder.
3. **Output schema:** `{ actionable: bool, reason?: string, dropped_suggestions?: [] }` — strict JSON, structured per critique sentence. No conversational tone.
4. **Few-shot the Sonnet prompt** with explicit drop examples from past sessions ("widen feet to 100mm" → drop, "use a wider baseplate" → keep) so the model anchors on the right severity.
5. Drop the existing regex once the hybrid classifier is shipped and stable.

**Risks:**
- False negatives (classifier drops actionable feedback) hurt design quality. Mitigate by logging dropped critiques for manual review during early testing.
- Sonnet is more expensive than Haiku per call but only invoked on the ambiguous remainder; expected ~30% of critiques reach the LLM stage.
- Deterministic pre-filter quality depends on catalog metadata being clean (capability tags, alternative names). May need a small metadata pass on the catalog first.

**Files:**
- `src/src/viewportChat.ts` — Gemini-loop hook point (existing regex location).
- `core/ai/claude_client.py` (or new module) — Haiku classifier call.

**Done when:**
- Mars rover test (rocker-bogie suggestion) does not trigger redesign — critique dropped with logged reason.
- Quadruped/wheeled/arm tests show no regression — actionable feedback still flows through.
- Reference-image-derived critiques (post-3a) are correctly preserved through the classifier.
- Existing regex removed.

---

## 4. Dynamic Preset Catalog Injection

**Status:** Not started. **Ship LAST** — see "Why last" below.

### What
Stop dumping all 90+ presets into the system prompt. Inject only the catalog entries relevant to the current request and the current graph neighborhood.

### Why
Onshape grounds its FeatureScript LLM by retrieving relevant stdlib docs at request time. Two wins:
- **Reasoning quality** (the bigger one) — Claude has to actively ignore ~80 irrelevant presets every turn. LLM attention leaks to nearby options; "wrong-but-plausible preset selection" is a known failure mode. Sharper affordances = cleaner reasoning.
- **Token cost** — full catalog is large and loaded every turn.
- **On-retry signal** — surfacing "you tried these, here's why they didn't work" is impossible when the prompt already includes everything.

Listed in `project_vector_improvement_plan.md` WS2 as a low-risk deferred item.

### Why last (attribution concern)
This change shifts which components Claude sees on every call — a confounding variable. If shipped alongside #2 (tool-call surface) or #3 (Gemini loop), regressions on the test corpus become hard to attribute: is the wrong subset being selected, or is the new tool dispatch broken, or is the Gemini classifier dropping useful feedback?

Shipping after #1-#3 are baked in and verified means: any quality change observed when #4 lands is attributable to the catalog selector. Clean A/B.

### How
1. Tag each preset with category metadata (`actuator`, `structural`, `sensor`, `mobility`, `coupler`, etc.) — most already have this implicitly via `preset_id` prefix.
2. Build a relevance scorer: keyword match on user prompt + currently-present component categories in the graph + user-explicitly-mentioned preset names.
3. Inject only the top-N (~20-30) most relevant presets into the system prompt. Always include a canonical "core" set (baseplate variants, basic servos, basic extrusions, common couplers) as a floor.
4. On retry: include any presets Claude tried but failed on, plus a hint that more presets are available on request.

### Risks
- A bad relevance scorer hides a needed preset → Claude can't satisfy the request. Mitigate with the always-include floor and an explicit "available presets I haven't shown you" hint.
- Confounding with prior workstreams if shipped early — addressed by the "ship last" ordering.

### Files
- `core/ai/claude_client.py` — system prompt assembly, retrieval pass before each call.
- `src/src/urdfAssembly.ts` (or wherever the preset catalog lives) — category metadata, if not already present.

### Done when
- System prompt size drops by ≥40% on a typical request.
- Quadruped + biped + arm test corpus all pass with the dynamic catalog (no regression vs. full-dump baseline run on the same #1-#3 codebase).
- Fixture: a request with no actuator keywords doesn't include sensors-only presets unless the graph already has actuators present.

---

## Build Sequence

**Phase A — foundation:**
1. **#1 AssemblyGraph preservation** — strict prerequisite for #2 (otherwise tool-call edits compound round-trip losses).

**Phase B — edit surface:**
2. **#2 Tool-call edit surface** — biggest single win for edit quality.

**Phase C — Gemini loop:**
3. **#3 Gemini Loop Hardening** (3a image input + 3b classifier) — fixes the validation feedback channel. Independent of #2; can ship in parallel.

**Phase D — final layer (ship last for clean attribution):**
4. **#4 Dynamic preset catalog** — improves AI grounding but introduces a confounding variable. Land after #1-#3 are verified so any regression is attributable.

**Why this order:**
- #1 first: every later workstream that touches graph state benefits from a non-lossy source of truth.
- #2 next: the structured edit surface is the biggest leverage point and the foundation for any future edit-quality work.
- #3 in parallel with #2 if a second hand is on it — independent code paths.
- #4 last because it changes which presets Claude sees on every call — shipping it alongside other engine changes makes it impossible to A/B regressions cleanly.

## Deferred Engine Work (ship-when triggered)

Engine fixes that are real but not urgent. Documented so the trigger is explicit and they don't get forgotten.

### Anchor-to-Rendered-Mesh Placement (P3 root cause)

**The fix:** Teach the placement engine to compute attach faces from actual rendered mesh AABB/centroid, not from preset `bounding_box_mm` declarations. Currently when a GLB's extents differ from the declared bbox, child components attach to the bbox face while the rendered mesh ends elsewhere → visible gap.

**Why deferred:**
- "Invasive" per known-issues — touches `computeFacePlacement` and `_performPlacement` in the placement hot path.
- Real risk of regressing currently-correct paths (quadruped, wheeled, arm).
- Forces re-baselining the joint-gap fix `fc85d91` (which used preset bbox half-extents).
- Cheap workaround exists: `TRANSLATION_OVERRIDES` table mirroring `ROTATION_OVERRIDES`. Per-component, low-risk, ships in an afternoon.
- Doesn't unblock anything in the current critical path — bipeds/torso work is itself deferred.

**Ship-when trigger:**
- ≥3 presets need `TRANSLATION_OVERRIDES` entries and overrides feel like duct tape, OR
- A new placement bug class surfaces that overrides can't reach, OR
- Bipeds re-enter scope and torso_panel mesh issues become a recurring pain point.

**Files to touch when activated:**
- `src/src/urdfAssembly.ts` — `computeFacePlacement`, `_performPlacement`, half-extent computation.
- `src/src/richVisuals/*` — mesh load may need to expose computed AABB upward.
- `src/main.ts` — render pipeline, possibly grounding calc (P2 may shift as a side effect).

**Interim:** Use `TRANSLATION_OVERRIDES` table for the torso_panel offset. Add other entries as new presets surface the issue.

---

## Out of Scope (intentionally)

- **Plan/Act UI gate** — editor already highlights changes; undo/redo + checkpoints already cover the "did this go wrong" case. Revisit only if the engine work surfaces a need for explicit pre-commit gating.
- **Parameter sliders** — UX polish, defer until engine is solid.
- **Click-to-scope viewport chat** — UX polish, defer.
- **Skill bundle / MCP server / external-agent integrations** — standalone-app focus first.
- **Demo-path patches** (L-bracket rotation, torso mesh re-author, shoulder flip, pan-tilt repair, catalog gaps for display/rocker-bogie/pedestal/wider-feet) — handled by system-prompt additions and component-catalog additions as new robot types come into scope. See `project_known_issues.md` for the catalogue.
- **Monaco editor upgrade** — STACK.md polish item, not engine work.

## Quality Gates (per CLAUDE.md)

Each workstream lands under the existing repo standards:
- Run `code-review` plugin on each logical commit.
- Run `simplify` skill on changes ≥100 lines or new abstractions.
- Type-check with `cd src && npx tsc --noEmit` before commit.
- New validator rules ship with fixture pairs (bad case fires, fix case passes).
- New engine fixes ship with regression fixtures against the existing quadruped/wheeled/arm test corpus.
- Don't commit unless explicitly asked — pattern stays edit → type-check → wait for "commit."

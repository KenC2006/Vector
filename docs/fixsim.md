# Plan: Rotating Node Connections

## Problem

Attachment nodes (`src/src/attachmentNodes.ts`) describe _where_ two components meet, but
the snap/drop path always emits a `fixed` joint. The only reason wheels roll or arms
pivot today is that `urdfAssembly.ts` special-cases component IDs after the fact
(e.g. `isWheel && joint_type === 'revolute' → continuous`, shoulder/elbow depth
heuristics). The user has no way to say "connect these two nodes with a hinge".

Node classes (`mount_face`, `shaft`, `bore`, `rail`, `generic`) hint at _kinematic intent_
but nothing consumes that intent to pick a joint type, axis, or limits.

## Goal

Let a connection between two nodes declare rotation (or translation) as a first-class
property, carried from the node definition through the URDF joint, all the way to MJCF
actuators — without re-introducing component-ID string sniffing.

## Target model

A **connection** is the pairing of a parent node + child node. It should resolve to:

| Field        | Source                                                                   |
| ------------ | ------------------------------------------------------------------------ |
| `joint_type` | `fixed` \| `revolute` \| `continuous` \| `prismatic`                     |
| `axis`       | unit vector in the parent-node local frame                               |
| `limits`     | `{lower, upper, effort, velocity}` for bounded joints                    |
| `origin xyz` | existing face-placement math                                             |
| `origin rpy` | node pair's relative orientation (already captured in `worldQuaternion`) |

The joint type is decided by the pair of node classes, with per-node overrides and
a user-selectable override at drop time.

## Changes

### 1. Extend `AttachmentNodeDef` — `src/src/attachmentNodes.ts`

Add optional kinematic fields to each node def:

```ts
kinematic?: {
  joint_type: 'fixed' | 'revolute' | 'continuous' | 'prismatic'
  axis: [number, number, number]        // in node-local frame
  limits?: { lower: number; upper: number }
  effort?: number
  velocity?: number
}
```

`shaft` nodes default to `continuous` with axis `[0, 0, 1]` (node +Z is the shaft
direction by construction). `bore` nodes get the matching `continuous` so shaft↔bore
pairing yields a rolling joint without string sniffing. `rail` nodes default to
`prismatic` along their axis. `mount_face` stays `fixed`.

### 2. Pair resolver — new `resolveConnectionJoint(parentNode, childNode, override?)`

One function, one source of truth. Given two node defs:

- If either node has `kinematic.joint_type !== 'fixed'`, use it (prefer the "driven"
  side — shaft over bore, rail over slider block).
- If both are `mount_face`, default to `fixed`.
- `override` (from the UI) wins over everything — this is how users opt a face-to-face
  connection into a hinge.

Returns `{ joint_type, axis, limits, effort, velocity }` in the parent-node frame.
Callers (AI topology resolver, manual drop, future paste) all go through this.

### 3. Connection metadata on nodes — `src/src/attachmentNodes.ts` + preset catalog

Augment `componentPortsForPreset` so the existing special cases become _data_:

- Servos/motors: the `top` node becomes `shaft` with `kinematic = { joint_type: 'revolute', axis: [0,0,1], limits: {-π, π}, effort: from preset }`.
- Continuous servos: `joint_type: 'continuous'`.
- Wheels: a `hub` node replaces the generic `bottom` face, class `bore`, axis `[0,1,0]`, `continuous`.
- Linear actuators: a `tip` node, class `rail`, `prismatic`.

Preset JSON grows an optional `kinematic_ports` field; for presets without it, the
face-default generator fills in from `mounting_logic` + category. No behavior change
for plain structural parts.

### 4. Wire the resolver through `urdfAssembly.ts`

Replace the two "special case" spots:

- **Lines ~3474-3484** (wheel → continuous promotion): delete. The shaft/bore pairing
  now emits `continuous` directly.
- **Lines ~3540-3548** (arm depth heuristic for rest pose): keep for _pose_ only, but
  stop using it to pick joint type.

In the URDF emit block (~3531), pull `joint_type / axis / limits` from the resolver
output instead of `comp.joint_type`. `<limit>` emission collapses to a single branch
that handles revolute/prismatic (bounded) and continuous (unbounded, effort/velocity only).

### 5. UI — override at drop

When the user drags a component and hovers a target node, show the resolver's default
joint type as a small badge on the node ring (e.g. "●fixed", "↻revolute", "↔prismatic").
A modifier key (Shift = cycle to revolute, Ctrl = prismatic) or a dropdown on the node
tooltip lets the user override before committing. The override is stored on the
connection — not the node — so reusing the same node for a different child keeps its
default.

### 6. AI topology (`core/ai/claude_client.py`)

Claude already emits `joint_type` per component. Teach the prompt that for shaft/bore
nodes it can omit `joint_type` and the resolver will pick it. Remove the wheel-enforcement
block (~line 815) — it's now redundant. Keep the explicit `attach_rpy` override path.

### 7. MJCF conversion (`core/urdf_to_mjcf.py` or equivalent)

Already maps `continuous → torque motor` and `revolute → position actuator`. Double-check
`prismatic` maps to a linear actuator; add if missing. No structural change — this step
just stops receiving a flood of `fixed` joints that should have been revolute.

## Migration / compatibility

- Existing saved URDFs have explicit `joint_type` on every joint; the resolver is only
  invoked when _building_ a new connection, so they load unchanged.
- The special-case code being removed (wheel promotion, arm depth for joint type) has
  been in place for <10 commits; no external callers depend on it.
- `example.urdf` should regenerate with `continuous` wheel joints and correct axes
  without any post-hoc string sniffing.

## Non-goals

- Multi-DOF joints (ball, universal) — out of scope; URDF doesn't support them natively.
- Closed kinematic loops — still require manual `<mimic>` tags, no UI change.
- Retroactively rewriting legacy saved robots — they keep whatever joint types they had.

## Validation

1. Drop a servo on a plate: connection resolves to `revolute`, shaft axis along +Z.
2. Drop a wheel on a plate bottom: `continuous` around Y, no string match on "wheel".
3. Shift-drop a battery on a plate top: overridden to `revolute` (non-sensical but proves
   the override path).
4. AI generates a 4-wheeled rover: wheels roll in sim; no console warnings about
   "Promoting ... to continuous" (the log line goes away with the special case).
5. `example.urdf`-shaped assembly round-trips: parse → resolver → emit → re-parse
   produces identical joint elements.

## Rollout order

1. Add `kinematic` field + resolver (pure data, no behavior change yet).
2. Populate `componentPortsForPreset` and preset catalog entries for shafts/bores/rails.
3. Swap the URDF emit block to use the resolver; delete wheel/arm special cases.
4. Add UI override badge + modifier key.
5. Simplify AI prompt + Python special cases.
6. Verify MJCF actuator mapping for all four joint types.

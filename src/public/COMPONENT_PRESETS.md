# Component Presets

`generic_presets.json` is the component catalog source for the Vector frontend.

To change a component, edit its entry in this file first. Dimensions,
connectors, mounting logic, visual metadata, and collision metadata are resolved
from that component entry by the unified resolver. Runtime placement code should
not patch dimensions in another file or infer component size from rendered scene
geometry.

During the migration there is still a backend mirror at
`core/presets/generic_presets.json`. Keep the two catalogs in sync until the
backend reads the same catalog directly.

## Bbox is the source of truth

`physical.bounding_box_mm` is canonical for each component. The collision OBJ
and visual GLB must conform to it after `mesh.rotation_rpy` and `mesh.scale`
are applied.

When a mesh disagrees with its bbox, **the mesh is wrong, not the bbox.** Fix
options, in order of preference:

1. Rotate (`mesh.rotation_rpy`) — pure axis-convention swap, no model change.
2. Re-author / re-source the GLB — when the mesh depicts a different part.
3. Scale (`mesh.scale`) uniform — last resort; non-uniform scale deforms.

The mesh-extent baseline at `scripts/mesh-extent-baseline.json` is a debt
list to drive to zero, not a permanent allowlist. Do not grow a bbox to
match a misaligned mesh.

Downstream code (collision bounds, joint origins, carry-ghost outlines,
inertial properties) all derive from the bbox. Authoring fixes that conform
the mesh to the bbox keep these in lockstep automatically.

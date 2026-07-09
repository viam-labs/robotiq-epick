# EPick mesh decimation and `include_realsense` attribute

Date: 2026-07-09
Status: approved

## Problem

Two independent problems, one shared cause.

**Visualization payload.** The app polls `Geometries()` on every frame update. Each call
builds a mesh from the embedded STL and `ToProtobuf()` converts it to PLY over the wire.
The STL at HEAD is 10,348 triangles. We were asked to get the default mesh to **500
triangles or fewer**.

**The RealSense is not modeled as a selectable variant.** The EPick is deployed both with
and without a RealSense camera bolted on, but the module embeds exactly one STL. The STL at
HEAD spans Y to -107mm, which is the camera — meaning every deployment to date has rendered
a camera whether or not one is physically present.

## Current behavior

The gripper describes its shape in two places that share no code and cannot be kept in sync.

- `Kinematics()` (`epick/gripper.go:503`) parses `epick_model.json` and returns a capsule +
  box. The frame system consumes only `Kinematics` (`robot/framesystem/framesystem.go:39`),
  so **these primitives are what the motion planner collision-checks against.**
- `Geometries()` (`epick/gripper.go:491`) ignores the model and builds a mesh from the
  embedded STL with a -196mm Z offset. No planner code path calls it. It feeds the
  visualizer.

`epick/simulated.go:184` duplicates `Geometries()` verbatim.

Extents in the gripper frame (origin at TCP, Z=0 at the suction cup tips). Capsule `l=70` is
total length including hemispherical caps, per `spatialmath/capsule.go`.

|                       | X      | Y            | Z            |
| --------------------- | ------ | ------------ | ------------ |
| Capsule (planner)     | ±34    | ±34          | -196 … -126  |
| Box (planner)         | ±115   | ±75          | -126 … -26   |
| STL at HEAD (visual)  | ±104   | **-107** … 65 | -185 … -10   |
| New plain STL         | ±104   | ±65          | -199 … -10   |
| New RealSense STL     | ±104   | **-107** … 65 | -199 … -10   |

## Decision

Keep the collision primitives exactly as they are. Add an `include_realsense` boolean that
selects the **visualization mesh only**. Default `false` (no RealSense). Apply to both the
real and simulated models. Decimate both STLs offline with a committed script.

### Rejected alternatives

**Unify collision and visualization on the mesh.** Replace the primitives with the STL so the
attribute drives both. Rejected as incorrect, not merely risky: the mesh includes the suction
cups, and collision geometry covering the cups would make the gripper unable to plan a pick
(see "Why the cups are excluded" below).

**Widen the collision box to cover the camera.** Rejected: the camera already contributes its
own collision geometry through the RealSense camera component's frame. Adding it to the
gripper too would double-count the obstacle.

**Runtime `ConservativeDecimate`.** RDK exposes `(*Mesh).ConservativeDecimate`
(`spatialmath/mesh_decimation.go:15`), but it returns an *enclosing convex hull*, not a shape
approximation. At 500 triangles the gripper would render as a blocky shrink-wrapped blob with
the suction cups and bracket cutout filled in. Correct for collision, wrong for
visualization. It also would not shrink the binary.

**Commit two model JSONs with base64 `mesh_data`.** Unreadable diffs, invites drift.

## Why the collision primitives look "wrong" (they are not)

The collision primitives deliberately do not match the mesh. Both discrepancies are load-bearing.

**The suction cups are excluded on purpose.** The box's top face is Z = -26mm; the cups extend
to Z = -10mm. That 16mm band contains real hardware and no collision geometry — because the
TCP sits at the cup tips, and picking an object up means driving the TCP into contact with it.
Collision geometry covering the cups would make every grab approach a collision, and the
planner would refuse to execute it. **Do not "fix" this.** It is why the gripper can pick
anything up at all, and it is why the visualization mesh must never become the collision
geometry.

**The camera is excluded on purpose.** The RealSense reaches Y = -107mm while the collision
box stops at Y = -75mm, but the camera is not missing from the frame system: the RealSense
camera component contributes its own collision box through its own frame. The gripper must not
duplicate it.

The RealSense STL therefore has no collision role whatsoever. Its purpose is visual — it lets
you confirm the camera's frame origin lands where the mesh shows the camera to be.

One genuine looseness remains, recorded but not addressed here: the collision box is ±115 × ±75
around a gripper that is ±104 × ±65, roughly 10mm of padding per side in X and Y.

## Design

### Mesh selection

Both STLs are embedded via `//go:embed`, so both are present in the binary regardless of
config. The attribute selects which bytes get parsed.

```
include_realsense ──► chosen STL bytes  (stored on struct at Reconfigure)
                              │
                              ▼
                        Geometries()   ── polled every frame, pure read
```

`Geometries()` must stay a pure read of a preselected `[]byte` — no branching, no re-parse on
the hot path.

`Kinematics()` is unchanged and continues to return the primitives from `epick_model.json`.

### Shared helper

`gripper.go` and `simulated.go` currently contain byte-identical `Geometries()` bodies.
Extract:

```go
func meshGeometry(stl []byte, label string) ([]spatialmath.Geometry, error)
```

applying the -196mm Z offset. Both models call it with their selected bytes. This is cleanup
in code already being modified, not unrelated refactoring.

### Config

Add to both `Config` (`gripper.go:45`) and `SimConfig` (`simulated.go:38`):

```go
IncludeRealsense bool `json:"include_realsense,omitempty"`
```

Zero value `false` is the desired default, so no defaulting logic is needed. `Validate()`
gains nothing — the field introduces no dependencies and no invalid states. `Reconfigure`
must reselect the mesh bytes.

### Decimation script

`scripts/decimate_stl.py`, run manually, output committed. Not a build step.

- Inputs: source STL, target triangle count, output path.
- **Asserts the input is in meters** (bounding box max extent < 10). RDK's
  `readSTLVertex` (`spatialmath/mesh.go:221`) multiplies every vertex by 1000
  unconditionally. An STL exported in millimeters loads as a 100-meter gripper.
- Quadric decimation via `trimesh.Mesh.simplify_quadric_decimation`.
- Emits binary STL, preserving meters.
- Reports before/after triangle counts and max surface deviation, so shape loss is visible.

Dependencies (`trimesh`, `fast-simplification`, `numpy`) install into a throwaway venv;
`python3 -m ensurepip` and `python3 -m venv` are both confirmed available. A `make decimate`
target wraps venv creation and invocation.

Targets: **plain ≤ 500** (the requested number). RealSense **≤ 1000** — chosen, not
specified, to keep the camera body recognizable enough to verify its frame origin, which is
the mesh's stated purpose. Adjustable via script argument.

### Deployment note

The default flips existing deployments from rendering a camera to not rendering one.
Machines with a RealSense physically mounted (palletizing1) must set
`"include_realsense": true` to keep the current visualization.

## Testing

- `Geometries()` returns a mesh whose Y-extent is ±65 by default and reaches -107 when
  `include_realsense` is true. Extents are computed from the returned triangles.
- Triangle counts are at or under target for both variants.
- `Kinematics()` output is unchanged by the attribute.
- Both models (real and simulated) exhibit identical geometry behavior.
- The script's meters assertion rejects a millimeter-scaled STL.

## Documentation

- `CLAUDE.md`: the STL-must-be-in-meters rule and the reason for it; the regeneration recipe;
  the new attribute; and — most importantly — why the collision primitives exclude the suction
  cups and the camera, so no future reader "corrects" them.
- `README.md`: `include_realsense` in the config reference.

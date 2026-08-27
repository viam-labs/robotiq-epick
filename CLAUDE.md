# Robotiq EPick Viam Module

## What this is

A Viam module implementing `rdk:component:gripper` for the Robotiq EPick vacuum gripper.
Published as `viam:robotiq:epick` (in the viam-dev org, namespace `viam`).

## Architecture

### Socket I/O — single-goroutine, lockless

All TCP communication runs on a single `ioLoop` goroutine. No mutexes.

```
API callers (Grab, Open, etc.)
        │
        ▼
  requests channel ──► ioLoop goroutine ──► TCP socket (port 63352)
        ▲                    │
  response channel ◄─────────┘
                             │
                    keepalive ticker (500ms GET ACT)
```

- `Send()` pushes a `socketRequest` onto the channel and blocks on the response channel.
- `ioLoop` selects between incoming requests, a 500ms keepalive ticker, and a `quit` channel.
- Keepalive timer resets after every real command — no redundant polls during active use.
- `Close()` closes the `quit` channel, ioLoop exits, conn closes.

This design eliminates the "ackACT 1" merged response bug that occurred when a mutex-based
keepalive's GET ACT response collided with a SET ack in the TCP read buffer.

### Auto-reconnect on socket drop
If a write or read fails (broken pipe / reset / read timeout), `ioLoop` re-dials `g.addr`
with capped backoff (200ms → 2s) and retries the command (up to 3 attempts). This is needed
because the URCap socket gets reset by the UR side on protective stops, tool-power cycles, or
when the URCap re-establishes its link to the EPick. Without reconnect, the first such drop
took the gripper offline until a viam-server restart — every subsequent `Grab()` wrote to a
dead socket and returned `write: broken pipe`. A reconnect logs `reconnected to EPick URCap`;
frequent occurrences point to a competing master (e.g. a UR program running Robotiq gripper
nodes), since the URCap socket itself is multi-client (verified: parallel clients coexist).

### Communication protocol
Uses the Robotiq URCap socket protocol (TCP port 63352) — same text protocol as
[viam-modules/robotiq](https://github.com/viam-modules/robotiq) for 2F grippers.
Commands: `SET <reg> <val>\r\n` (returns "ack") and `GET <reg>\r\n` (returns "<reg> <val>").

### UR Firewall gotcha
Port 63352 is blocked by default on the UR. Must unblock in **Settings** > **Security**
on the pendant. The 2F gripper module has the same requirement.

### Why not Modbus RTU?
The EPick connects via the UR tool connector (RS-485). The UR controller owns the serial bus.
We explored alternatives (Modbus TCP port 502, URScript tool_modbus, serial bridge via
port 30002) — all require either a URCap or complex bridging. The URCap socket is simplest.

### Why a separate module from viam-modules/robotiq?
Different semantics (pressure/vacuum vs position/force), different registers, different
control flow. Zero shared code.

## Key files

- `main.go` — module entry point (registers both `epick` and `simulated-epick-vacuum-gripper`)
- `epick/gripper.go` — all gripper logic: ioLoop, Grab, Open, Kinematics, DoCommand
- `epick/simulated.go` — simulated gripper model (no hardware), see below
- `epick/simulated_test.go` — unit tests for the simulated model's hold-delay behavior
- `epick/geometry.go` — the primitive table: dimensions, part names, `visualGeometries`
- `epick/geometry_test.go` — unit tests for the collision model, the render set, and the invariants tying them together
- `epick/epick_model.json` — embedded kinematic model (collision geometry + TCP offset)
- `epick/meshes/*.stl` — full-resolution CAD exports; the source the primitives are fitted to, never embedded
- `epick/registers.go` — EPick register constants (reference, not used at runtime)
- `scripts/fit_primitives.py` — re-derives the primitive table from CAD (`make fit-primitives`)
- `meta.json` — Viam module manifest

## Simulated model

`viam:robotiq:simulated-epick-vacuum-gripper` (`epick/simulated.go`) implements the
gripper API with no hardware/socket — for testing motion plans and gripper state machines.
It reuses the embedded `epick_model.json` and the same `visualGeometries` builder, so
kinematics and rendered geometry match the real model. Holding is time-driven: `Grab()` records the grab time and
`IsHoldingSomething()` returns true once `grab_delay_ms` (default 1000, adjustable via config
or `DoCommand({"set_grab_delay_ms": N})`) has elapsed; `Open()`/`Stop()` clear it immediately.
Shares the package-level embedded `epickModelJSON` var and the `visualGeometries`
helper with `gripper.go`. It takes `include_realsense` too, so the sim renders the same
gripper the hardware model would.

## Vacuum control design

### Grab() — non-blocking, continuous vacuum
`Grab()` switches to advanced mode (MOD=1) with continuous vacuum (POS=0) and no timeout
(SPE=0). The pump runs indefinitely until `Open()` is called. Returns immediately by default
(non-blocking). Pass `extra["blocking"]=true` to wait for object detection.

Re-asserts ACT=1 on every call to recover from deactivation (protective stop, URCap reset).

### Open() — release with GTO re-assertion
Sends POS=100 (release) with GTO toggled around it per the manual. Also re-asserts ACT=1.
Stays in advanced mode so the next Grab() doesn't need to re-initialize.

### IsHoldingSomething() — dual detection
Checks both OBJ register (EPick's built-in detection) and actual pressure. In continuous
vacuum mode (POS=0), OBJ may stay at 0 (regulating) since the target is never "reached".
Pressure fallback: if POS < 90 (~10 kPa vacuum), reports holding=true.

### Keepalive
The EPick faults (0x9) if it receives no communication for 1 second. The ioLoop fires
`GET ACT` every 500ms during idle periods. Timer resets after every real command.

## Frame and kinematics

### Config frame
`translation.z: 196` places the gripper origin at the TCP (suction cup tips).
Other modules attaching geometry to this frame (e.g. held boxes in the palletizer)
correctly appear at the cup tips.

### Kinematic model (epick_model.json)
Embedded JSON parsed via `referenceframe.UnmarshalModelJSON` — this sets `OriginalFile`
so it serializes correctly over gRPC (programmatic models via `ParseConfig` don't).

Six collision primitives at negative Z from the TCP, one per link (a link carries at
most one geometry, so the parts are chained with zero translations between them):

| Link | Shape | Dimensions | Center Z |
|------|-------|-----------|----------|
| `body` | box | 71 x 71 x 126 | -133 |
| `plate` | box | 204.5 x 126.3 x 3.2 | -68.4 |
| `cup-{xp,xn}-{yp,yn}` (x4) | box | 49 x 49 x 44 | -48 |

`referenceframe` labels these `epick:body`, `epick:cup-xp-yp`, and so on
(`referenceframe/model.go`), which is why `visualGeometries` qualifies its labels the
same way — a part has one name whether it arrived as collision geometry or as a render.

### spatialmath.Cylinder can never leave this process

**This is the most important thing in this file.** The wire protocol's geometry
oneof is `Sphere`, `Box`, `Capsule`, `Mesh`, `Pointcloud`. There is no cylinder
message, and `Cylinder.ToProtobuf()` does not return an error -- it panics:

```
panic("Cylinder.ToProtobuf: unimplemented -- no Cylinder message in commonpb")
```

`spatialmath.Cylinder` is an in-process collision primitive only. Version 2.2.0
shipped with cylinders in both `Geometries()` and `epick_model.json` and was
broken on arrival: the gripper's `GetGeometries` handler
(`components/gripper/server.go`, via `referenceframe.NewGeometriesToProto`)
panicked on five of six geometries, so the app's 3D scene rendered nothing.
Cylinders in the model JSON are the same landmine one step removed, reachable
through any RPC that ships a `GeometriesInFrame`
(`referenceframe/transformable.go`).

Every geometry test passed the whole time. None of them crossed the wire.
`TestAllGeometriesSerialize` now does, and fails on exactly this.

So:

- **Round parts in `Geometries()` ship as tessellated meshes.** `roundPart()`
  builds the cylinder for its shape, then returns `ToMesh()`. RDK converts every
  mesh to PLY on the way out ("the visualizer expects all meshes to be in PLY
  format"), which is the same representation the STL this replaced rendered
  from -- so this is a known-good path, not a guess. ~10KB for all 8 geometries.
- **Collision uses boxes.** A box shares the cylinder's bounding box and errs
  outward. Do not "improve" this to a capsule: RDK capsule `length` is
  tip-to-tip, so `r=35.5 l=126` leaves only 55mm at full radius and tapers to a
  point across the 35.5mm where the body meets the plate, under-covering real
  hardware. A capsule always *under*-approximates a flat-ended cylinder.

The cost is the cups' round cross-section: a 49x49 box over-claims about 41% at
the diagonals. That is the safe direction, and still far tighter than the single
230x150x100 slab this model replaced.

### Collision is NOT the render — and the cups are short on purpose

`Kinematics()` returns the primitives above. The frame system consumes only `Kinematics`
(`robot/framesystem/framesystem.go`, the `InputEnabled` interface), so **the primitives are
what the motion planner collision-checks against.** `Geometries()` returns the render set
built by `visualGeometries` and is polled by the app for drawing. No planner code path
calls it.

The two sets are the same parts in the same places, with exactly two differences. Do not
"fix" either:

- **The cups are modeled 44mm long instead of 60mm.** Collision stops at Z=-26; the real
  cups reach Z=-10. The TCP sits at the suction plane, and picking an object up means
  driving the cups into contact with it. Collision geometry across that gap would make
  every grab approach a collision and the planner would refuse to execute it.
  `TestCollisionModelClearsTCP` holds the gap to exactly 26mm.
- **The body is modeled 126mm long instead of 129mm.** Collision stops at the flange
  face, Z=-196. The body's rear boss really reaches Z=-199, 3mm past the flange and into
  the space the arm's own end-effector geometry occupies -- the UR7e's `ee_link` is a
  capsule of r=40, l=170 sitting right there. The capsule this replaced also ended at
  -196, so keeping the rear boundary there means arm-vs-gripper collision behavior does
  not move. `TestCollisionModelStopsAtFlange` holds it.
- **The camera is excluded from collision.** The RealSense reaches Y=-107.2; the collision
  model stops at Y=-65.15. The camera is not missing from the frame system: the RealSense
  *camera component* contributes its own collision geometry through its own frame.
  Duplicating it here would double-count the obstacle. Note that the guard for this must
  assert the collision model stops at its **own** -Y envelope: a threshold set at the
  camera's far edge is vacuous, since the camera body's own minimum (Y=-107.15) already
  sits inside Y=-107.2.

`TestVisualAndCollisionAgree` enforces the rest as a containment invariant: every part the
planner collides against must also be drawn, with the same X/Y footprint, and its Z span
must lie inside what is drawn. Collision geometry may only ever be shorter, never wider and
never longer. The two clips above are asserted at their exact values, so "correcting"
either one fails the test.

## Visualization

`Geometries()` returns the gripper drawn as primitives: the same body and plate the planner
sees, plus the suction cups at their true 60mm length. `include_realsense` (both models,
default `false`) adds two more boxes — the bracket (75 x 125.7 x 9.9 at Y=-42.1, Z=-160.6)
and the camera body (81 x 20.1 x 27.3 at Y=-97.1, Z=-144.2). Six geometries plain, eight
with the camera. `Kinematics()` is identical either way.

Set `include_realsense` true on machines with the camera physically mounted; it exists to
confirm the camera's frame origin lands where the render shows the camera to be.

Both models are `resource.AlwaysRebuild`, so there is no `Reconfigure` — the flag is read
once in the constructor and `Geometries()` rebuilds a handful of primitives per poll.

### Where the numbers come from

The dimensions in `epick/geometry.go` are fitted to the full-resolution CAD exports in
`epick/meshes/`. Splitting an export into connected components shows the gripper is six
solids and every one is a clean box or a right circular cylinder — the cups sample at a
constant 24.5mm radius end to end, so they are not even tapered. That is why primitives
describe this gripper as well as a mesh does.

```bash
make fit-primitives    # prints the table; changes no source file
```

Two things the fit depends on, both learned from the decimation pipeline this replaced:

- **Weld vertices before splitting.** Binary STL stores three unshared vertices per facet,
  so an unwelded mesh splits into one component per triangle.
- **A tessellated cylinder's bounding box understates its radius.** The body's vertex radii
  straddle the true value (35.396 to 35.603, mean 35.500) while its bbox half-width reads
  35.448. Fit the radius from the wall vertices, not the extents.
- **A square box passes a naive radius check trivially.** All eight of its vertices are
  corners, so they sit at one radius -- sqrt(2) x the half-width. The fit rejects this by
  requiring enough distinct circumference vertices, and by requiring the fitted radius to
  fit inside the bounding box.

Exports are authored in **meters** and the fit script scales by 1000. This no longer matters
at runtime — nothing parses an STL any more — but it matters when reading a new export.

## Hardware setup (tested)

Current deployment (palletizing1):
- UR7e at 192.168.1.3
- viam-server host `palletizing1` at 192.168.1.2
- EPick connected through UR tool connector
- URCap socket on 63352 verified reachable and multi-client (two parallel `nc` clients coexist)
- (firmware / serial / URCap version / machine ID on this robot: TBD — confirm on pendant)

Original bench setup (UR5):
- UR5 at 10.1.0.84, firmware 5.22.1, serial 20255700195
- EPick with 4 suction cups connected through UR tool connector
- Robotiq Grippers URCap v3.41.0
- UR Security: port 63352 unblocked
- viam-server at 10.1.2.36 (amd64)
- Machine ID: a075e851-d7fc-4d3a-b4b8-c093dd28c077

## Known behaviors

- Grab() always uses advanced mode + continuous vacuum regardless of config mode setting.
- ACT=1 is re-asserted on every Grab/Open to auto-recover from deactivation.
- VAS register is not supported by the Grippers URCap (returns "?").
- Cardboard (porous): continuous vacuum compensates for air leakage. IsHoldingSomething
  uses pressure fallback since OBJ detection may not trigger.
- Protective stop on UR cuts tool power → vacuum drops → EPick deactivates, and the URCap
  often resets the 63352 socket. The ioLoop auto-reconnects the socket, then the next Grab()
  recovers gripper state via ACT=1 re-assertion. (ACT=1 alone can't recover a dead socket —
  the write fails first; reconnect must happen at the socket layer, which it now does.)
- `include_realsense` changes only what renders. A machine with the camera mounted still plans
  against the same primitives; the camera's own component supplies its collision geometry.
- The palletizer module uses `ComponentName: gripper` for motion.Move and
  `motion.GetPose(gripper, world)` for position — both compose the frame offset correctly.

## Build and deploy

```bash
make build                    # Build binary
make module                   # Build tar.gz
make lint                     # go mod tidy + go vet
make test                     # go test -race ./...
make fit-primitives           # Re-derive the primitive table from epick/meshes/ CAD
viam module upload \
  --version X.Y.Z \
  --platform linux/amd64 \
  --upload bin/module.tar.gz  # Upload to registry
viam module build start \
  --version X.Y.Z             # Cloud build for all platforms
```

## Future work

- [x] Moved to the `viam` namespace (viam-dev org) in the Viam registry
- [x] Decimated the visualization meshes (10,348 -> 360 triangles) and added `include_realsense`
- [x] Replaced the meshes entirely with primitives (6 boxes/cylinders) for both collision and render
- [ ] Add unit tests with mock socket server
- [ ] Support Get3DModels when gripper API adds it (feature requested)

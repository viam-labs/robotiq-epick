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
- `epick/geometry_test.go` — unit tests for mesh selection, triangle budget, meters
- `epick/epick_model.json` — embedded kinematic model (collision geometry + TCP offset)
- `epick/epick_simplified.stl` — embedded visualization mesh, no camera (default)
- `epick/epick_simplified_with_realsense.stl` — embedded visualization mesh, with camera
- `epick/meshes/*.stl` — full-resolution CAD exports; decimation sources, never embedded
- `epick/registers.go` — EPick register constants (reference, not used at runtime)
- `scripts/decimate_stl.py` — regenerates the embedded meshes (`make decimate`)
- `meta.json` — Viam module manifest

## Simulated model

`viam:robotiq:simulated-epick-vacuum-gripper` (`epick/simulated.go`) implements the
gripper API with no hardware/socket — for testing motion plans and gripper state machines.
It reuses the embedded `epick_model.json` + STL, so kinematics and collision geometry match
the real model. Holding is time-driven: `Grab()` records the grab time and
`IsHoldingSomething()` returns true once `grab_delay_ms` (default 1000, adjustable via config
or `DoCommand({"set_grab_delay_ms": N})`) has elapsed; `Open()`/`Stop()` clear it immediately.
Shares the package-level embedded `epickModelJSON`/`epickSTL` vars and the `meshGeometry`
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

Two collision geometries at negative Z from the TCP:
- Capsule (body): centered at Z=-161mm, 68mm dia, 70mm tall
- Box (bracket): centered at Z=-76mm, 230x150x100mm

26mm clearance between collision boundary and TCP for approach.

### Collision is NOT the mesh — and the primitives look wrong on purpose

`Kinematics()` returns the primitives above. The frame system consumes only `Kinematics`
(`robot/framesystem/framesystem.go`, the `InputEnabled` interface), so **the primitives are
what the motion planner collision-checks against.** `Geometries()` returns the STL mesh and
is polled by the app for rendering. No planner code path calls it.

Two apparent gaps in the collision model are deliberate. Do not "fix" either:

- **The suction cups are excluded.** The box's top face is Z=-26mm; the cups reach Z=-10mm.
  The TCP sits at the cup tips, and picking an object up means driving the TCP into contact
  with it. Collision geometry over the cups would make every grab approach a collision and
  the planner would refuse to execute it. This is also why the mesh must never become the
  collision geometry — the mesh includes the cups.
- **The camera is excluded.** The RealSense reaches Y=-107mm; the box stops at Y=-75mm. The
  camera is not missing from the frame system: the RealSense *camera component* contributes
  its own collision geometry through its own frame. Duplicating it here would double-count
  the obstacle.

## Visualization meshes

`include_realsense` (both models, default `false`) selects which embedded STL `Geometries()`
returns. Visualization only — `Kinematics()` is identical either way. Set it `true` on
machines with the camera physically mounted; it exists to confirm the camera's frame origin
lands where the mesh shows the camera to be.

Both STLs are compiled in via `go:embed` regardless of config; the flag picks which bytes get
parsed. The chosen bytes are selected once in the constructor (both models are
`resource.AlwaysRebuild`, so there is no `Reconfigure`) and `Geometries()` is a plain read —
it is polled on every frame update.

### STLs must be in meters

`spatialmath.readSTLVertex` multiplies every vertex by 1000 with no unit check or scale
factor. An STL exported in millimeters loads as a gripper 1000x too large. Everything *else*
in Viam is millimeters (`epick_model.json` dimensions, the -196mm mesh offset, `translation.z`
in the machine config) — only the mesh file is meters. `make decimate` and
`TestMeshesAreInMeters` both assert this.

### Regenerating the meshes

```bash
make decimate    # provisions bin/mesh-venv, rewrites both embedded STLs
```

Sources are the full-resolution CAD exports in `epick/meshes/` (832 and 4,656 triangles).
Outputs are 360 and 1,282 triangles. The run is deterministic — the Hausdorff probe is
seeded, so re-running yields byte-identical files. To swap in a new CAD export, replace the
file in `epick/meshes/` (in **meters**) and re-run.

`scripts/decimate_stl.py` decides face count from a shape-fidelity budget
(`--max-deviation`, default 3.0mm); `--faces` is a hard cap that fails the run rather than
degrading the mesh. Three constraints it encodes, each found the hard way:

- **Weld vertices first.** Binary STL stores three unshared vertices per facet. An unwelded
  mesh has no interior edges, and quadric decimation is edge-collapse — with nothing to
  collapse it just deletes triangles. This silently truncated 57mm off the RealSense mesh.
- **Decimate per component, never the whole mesh.** These STLs are 6–8 disconnected solids.
  Given the whole mesh the simplifier spends the budget on the largest surface and deletes
  small components outright — the suction cups vanish to buy triangles for the mounting plate.
- **Never decimate an open shell.** The four suction cups are open shells. Quadric error sums
  distances to the planes adjacent to a vertex, so boundary vertices look cheap to collapse;
  decimating a cup flattens its rim (~31mm of error on a 60mm cup). Shells are preserved intact.

The quality gate is a **symmetric** Hausdorff distance. A one-sided probe (sample the output,
measure to the input) cannot detect deleted geometry — surviving triangles still lie on the
original surface — and will happily report 0.10mm for a mesh missing a whole feature.

Thin solids are the fragile ones: the 3mm mounting plate and 10mm camera bracket have little
interior volume to constrain the error metric and self-intersect long before thicker parts do.
The bracket also refuses to decimate below ~400 faces, where it shatters; `simplify_quadric_
decimation(face_count=N)` is a request, not a guarantee.

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
make decimate                 # Regenerate embedded STLs from epick/meshes/ sources
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
- [ ] Add unit tests with mock socket server
- [ ] Support Get3DModels when gripper API adds it (feature requested)

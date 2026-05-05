# Robotiq EPick Viam Module

## What this is

A Viam module implementing `rdk:component:gripper` for the Robotiq EPick vacuum gripper.
Published as `shrews-testing:robotiq:epick` (to be moved to `viam-labs` namespace).

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
- `ioLoop` selects between incoming requests and a 500ms keepalive ticker.
- Keepalive timer resets after every real command — no redundant polls during active use.
- `Close()` closes the requests channel, ioLoop exits, conn closes.

This design eliminates the "ackACT 1" merged response bug that occurred when a mutex-based
keepalive's GET ACT response collided with a SET ack in the TCP read buffer.

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

- `main.go` — module entry point
- `epick/gripper.go` — all gripper logic: ioLoop, Grab, Open, Kinematics, DoCommand
- `epick/epick_model.json` — embedded kinematic model (collision geometry + TCP offset)
- `epick/registers.go` — EPick register constants (reference, not used at runtime)
- `meta.json` — Viam module manifest

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

## Hardware setup (tested)

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
- Protective stop on UR cuts tool power → vacuum drops → EPick deactivates.
  Module recovers on next Grab() via ACT=1 re-assertion.
- The palletizer module uses `ComponentName: gripper` for motion.Move and
  `motion.GetPose(gripper, world)` for position — both compose the frame offset correctly.

## Build and deploy

```bash
make build                    # Build binary
make module                   # Build tar.gz
make lint                     # go mod tidy + go vet
make test                     # go test -race ./...
viam module upload \
  --version X.Y.Z \
  --platform linux/amd64 \
  --upload bin/module.tar.gz  # Upload to registry
viam module build start \
  --version X.Y.Z             # Cloud build for all platforms
```

## Future work

- [ ] Move to viam-labs namespace in Viam registry
- [ ] Add unit tests with mock socket server
- [ ] Support Get3DModels when gripper API adds it (feature requested)

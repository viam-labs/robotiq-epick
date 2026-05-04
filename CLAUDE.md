# Robotiq EPick Viam Module

## What this is

A Viam module implementing `rdk:component:gripper` for the Robotiq EPick vacuum gripper.
Currently published as `shrews-testing:robotiq:epick` (to be moved to `viam-labs` namespace).

## Architecture

### Communication
Uses the Robotiq URCap socket protocol (TCP port 63352) — the same text-based protocol
used by [viam-modules/robotiq](https://github.com/viam-modules/robotiq) for 2F grippers.
The Robotiq Grippers URCap (v3.41.0 tested) must be installed on the UR teach pendant.

Commands are `SET <reg> <val>\r\n` (returns "ack") and `GET <reg>\r\n` (returns "<reg> <val>").

### UR Firewall gotcha
Port 63352 listens on localhost by default. The UR's **Security** settings have an
"inbound access" firewall that blocks it externally. Must be unblocked for the
viam-server machine to connect. This also affects the 2F gripper module.

### Why not Modbus RTU?
The EPick connects via the UR tool connector (RS-485). The UR controller owns the serial bus.
Without the URCap, there's no way to send Modbus RTU from an external machine to the tool.
The URCap abstracts this into the socket protocol.

We explored alternatives:
- Direct Modbus TCP (port 502): not available on UR without extra config
- URScript tool_modbus functions: require a URCap daemon, not built into base URScript
- URScript serial bridge via port 30002: works but complex, URCap approach is simpler

### Why a separate module from viam-modules/robotiq?
The 2F grippers use position/force semantics. The EPick uses pressure/vacuum semantics.
Different registers, different control flow, zero shared communication code. The upstream
`viam-modules/robotiq` module only supports the 2F-85/2F-140.

## Key files

- `main.go` — module entry point, registers the model
- `epick/gripper.go` — gripper implementation (socket protocol, Viam API)
- `epick/registers.go` — EPick register constants and bit definitions (reference from the manual, not used at runtime with socket protocol)
- `meta.json` — Viam module manifest

## EPick register reference (from manual sections 4.2-4.7)

Output registers (robot -> gripper):
- Byte 0: ACTION REQUEST — rACT(bit 0), rMOD(bits 1-2), rGTO(bit 3), rATR(bit 4)
- Byte 3: MAX PRESSURE — rPR (formula: rPR = 100 + Pmax_kPa)
- Byte 4: GRIP TIMEOUT — rSP (each unit = 100ms)
- Byte 5: MIN PRESSURE — rFR (formula: rFR = 100 + Pmin_kPa)

Input registers (gripper -> robot):
- Byte 0: GRIPPER STATUS — gACT(0), gMOD(1-2), gGTO(3), gSTA(4-5), gOBJ(6-7)
- Byte 1: STATUS EXT — gVAS(0-1)
- Byte 2: FAULT STATUS — gFLT(0-3), kFLT(4-7)
- Byte 3: PRESSURE ECHO — gPR
- Byte 4: ACTUAL PRESSURE — gPO (Pdiff = gPO - 100 kPa)

Modbus addresses: output base 0x03E8 (1000), input base 0x07D0 (2000).
RS-485: 115200 baud, 8N1, slave ID 9.

## Build and deploy

```bash
make build                    # Build binary to bin/robotiq-epick
make module                   # Build tar.gz for Viam registry
make lint                     # go mod tidy + go vet
make test                     # go test -race ./...
viam module upload \
  --version X.Y.Z \
  --platform linux/amd64 \
  --upload bin/module.tar.gz  # Upload to registry
```

## Hardware setup (tested)

- UR5 at 10.1.0.84, firmware 5.22.1, serial 20255700195
- EPick connected through UR tool connector
- Robotiq Grippers URCap v3.41.0 installed
- UR Security: port 63352 unblocked for inbound access
- viam-server machine at 10.1.2.36 (amd64)
- Machine ID: a075e851-d7fc-4d3a-b4b8-c093dd28c077

## Known behaviors

- In automatic mode, the EPick grip phase is ~2 seconds. Object must be in contact when Grab is called.
- Once object is detected, automatic mode holds indefinitely until Open is called.
- Grab() includes a 500ms settle delay before checking OBJ to handle state transition timing.
- Fault code 6 (grip timeout) clears on next GTO re-assertion.
- VAS register is not supported by the Grippers URCap (returns "?").

## Future work

- [ ] Move to viam-labs namespace in Viam registry
- [ ] Add unit tests with mock socket server
- [ ] Consider PR to viam-modules/robotiq as separate model

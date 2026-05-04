# Robotiq EPick Viam Module

## What this is

A Viam module implementing `rdk:component:gripper` for the Robotiq EPick vacuum gripper.
Model: `viam-labs:robotiq:epick`.

## Architecture

### Communication
Uses the Robotiq URCap socket protocol (TCP port 63352) — the same text-based protocol
used by [viam-modules/robotiq](https://github.com/viam-modules/robotiq) for 2F grippers.
The URCap must be installed on the UR teach pendant.

Commands are `SET <reg> <val>\r\n` (returns "ack") and `GET <reg>\r\n` (returns "<reg> <val>").

### Why not Modbus RTU?
The EPick connects via the UR tool connector (RS-485). The UR controller owns the serial bus.
Without a URCap, there's no way to send Modbus RTU from an external machine to the tool.
The URCap abstracts this into the socket protocol.

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

Output registers (robot → gripper):
- Byte 0: ACTION REQUEST — rACT(bit 0), rMOD(bits 1-2), rGTO(bit 3), rATR(bit 4)
- Byte 3: MAX PRESSURE — rPR (formula: rPR = 100 + Pmax_kPa)
- Byte 4: GRIP TIMEOUT — rSP (each unit = 100ms)
- Byte 5: MIN PRESSURE — rFR (formula: rFR = 100 + Pmin_kPa)

Input registers (gripper → robot):
- Byte 0: GRIPPER STATUS — gACT(0), gMOD(1-2), gGTO(3), gSTA(4-5), gOBJ(6-7)
- Byte 1: STATUS EXT — gVAS(0-1)
- Byte 2: FAULT STATUS — gFLT(0-3), kFLT(4-7)
- Byte 3: PRESSURE ECHO — gPR
- Byte 4: ACTUAL PRESSURE — gPO (Pdiff = gPO - 100 kPa)

Modbus addresses: output base 0x03E8 (1000), input base 0x07D0 (2000).
RS-485: 115200 baud, 8N1, slave ID 9.

## Build and test

```bash
make build    # Build binary to bin/robotiq-epick
make module   # Build tar.gz for Viam registry
make lint     # go mod tidy + go vet
make test     # go test -race ./...
```

## Hardware setup

- UR5 at 10.1.0.84, firmware 5.22.1
- EPick connected through UR tool connector
- Robotiq URCap required (port 63352)

## Future work

- [ ] Install Robotiq URCap on UR pendant (needs admin password)
- [ ] Test grab/release/status on live hardware
- [ ] Add unit tests with mock socket server
- [ ] Publish to Viam registry
- [ ] Consider PR to viam-modules/robotiq as separate model

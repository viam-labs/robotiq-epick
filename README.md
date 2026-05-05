# Robotiq EPick Vacuum Gripper Module

This [Viam module](https://docs.viam.com/) provides a driver for the [Robotiq EPick](https://robotiq.com/products/epick-vacuum-gripper) vacuum gripper connected to a Universal Robots arm.

## Prerequisites

- Robotiq EPick vacuum gripper connected to the UR arm's tool connector
- **Robotiq Grippers URCap installed** on the UR teach pendant (provides the socket server on port 63352)
- The UR controller must be reachable over the network from the machine running viam-server
- Port 63352 must not be blocked by the UR's firewall (see Troubleshooting)

### Installing the Robotiq URCap

1. Download the Robotiq Grippers URCap from [robotiq.com/support](https://robotiq.com/support)
2. Copy the `.urcap` file to a USB drive
3. On the UR teach pendant, switch to **Local** control mode
4. Tap **hamburger menu** (☰) > **Settings** > **System** > **URCaps**
5. Tap **+**, navigate to the USB drive, select the `.urcap` file
6. Restart when prompted
7. After restart: **Installation** > **URCaps** > **Vacuum** > **Dashboard** > **Scan** to detect the EPick
8. Switch back to **Remote** control mode for Viam

### UR Firewall Configuration

The Robotiq URCap socket (port 63352) may be blocked by the UR's security settings. To fix:

1. On the pendant: **Settings** > **Security**
2. Ensure port 63352 is **not** in the "disable inbound access" port range
3. Verify from the viam-server machine: `nc -zv <UR_IP> 63352`

## Configure your EPick gripper

Navigate to the [**CONFIGURE** tab](https://docs.viam.com/configure/) of your machine in the Viam app.

Add the **module** (or search for `robotiq-epick` in the registry):
```json
{
  "type": "registry",
  "name": "shrews-testing_robotiq-epick",
  "module_id": "shrews-testing:robotiq-epick",
  "version": "latest"
}
```

Add the **gripper component** with a frame parented to the arm. The `translation.z: 196` sets the TCP (tool center point) at the suction cup tips, 196mm from the flange:

```json
{
  "name": "vacuum_gripper",
  "model": "shrews-testing:robotiq:epick",
  "type": "gripper",
  "namespace": "rdk",
  "attributes": {
    "host": "10.1.0.84"
  },
  "depends_on": [],
  "frame": {
    "parent": "arm",
    "translation": {
      "x": 0,
      "y": 0,
      "z": 196
    },
    "orientation": {
      "type": "ov_degrees",
      "value": { "x": 0, "y": 0, "z": 1, "th": 0 }
    }
  }
}
```

### Attributes

| Name | Type | Required | Default | Description |
|------|------|----------|---------|-------------|
| `host` | string | **Yes** | - | UR controller IP address |
| `port` | int | No | `63352` | URCap socket port |
| `mode` | string | No | `"automatic"` | Config-level mode hint (Grab always uses advanced mode internally) |
| `max_pressure_pct` | int | No | `60` | Maximum vacuum level (20-100%), when mode is "advanced" |
| `min_pressure_pct` | int | No | `40` | Minimum vacuum level (10-100%), when mode is "advanced" |
| `timeout_ms` | int | No | `3000` | Grip timeout in ms, when mode is "advanced" |

## Frame and Collision Geometry

The frame `translation.z: 196` places the gripper's origin at the **suction cup tips** (196mm from the flange). This is the TCP — the point the motion planner targets when you call `motion.Move`.

The module returns collision geometry via `Kinematics()` positioned at **negative Z** from the TCP (back toward the flange):

```
         TCP (Z=0)           <-- gripper frame origin (cup tips)
    ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄       motion planner targets poses here
    │                    │
    │    ~ clearance ~   │  Z=0 to Z=-26mm (no collision)
    │                    │
    │ ┌────────────────┐ │
    │ │                │ │
    │ │     Box        │ │  Z=-26 to Z=-126mm
    │ │  230 x 150mm   │ │  Bracket + hoses (collision)
    │ │                │ │
    │ └────────────────┘ │
    │                    │
    │  ┌──────────────┐  │
    │  │   Capsule     │  │  Z=-126 to Z=-196mm
    │  │  68mm dia     │  │  EPick body (collision)
    │  └──────────────┘  │
    │                    │
    ━━━━━━━━━━━━━━━━━━━━
    UR Arm Flange (Z=-196)
```

The 26mm clearance between the collision boundary and the TCP ensures the planner allows the cups to approach surfaces. Other modules that attach geometry to this gripper frame (e.g. a held box) place it at the TCP origin — at the cup tips, where the box actually is.

**Dimensions based on actual measurements:**
- EPick body: ~75mm diameter, 70mm tall
- Widest point (bracket + hoses): 210mm x 130mm
- Collision padding: +20mm on all sides
- TCP: 196mm from flange (4-cup configuration, from manual Table 6-4)

## API

Implements the standard [Viam gripper API](https://docs.viam.com/dev/reference/apis/components/gripper/):

| Method | EPick Behavior |
|--------|---------------|
| `Open` | Releases vacuum (opens release valve) |
| `Grab` | Activates vacuum (non-blocking by default, returns immediately) |
| `IsHoldingSomething` | Checks OBJ register + actual pressure for seal detection |
| `Stop` | Stops vacuum regulation |
| `IsMoving` | Returns `true` if an operation is in progress |

### Grab behavior

`Grab()` is **non-blocking by default** — it turns on the vacuum and returns immediately. This is the right workflow for vacuum grippers where the arm still needs to descend onto the workpiece after vacuum is enabled. Use `IsHoldingSomething()` after contact to verify the seal.

Pass `extra["blocking"] = true` for the old behavior (wait up to 5 seconds for object detection).

Internally, `Grab()` always switches to **advanced mode** with **continuous vacuum** (`POS=0`, `SPE=0`) so the pump runs indefinitely until `Open()` is called. This works well with porous materials like cardboard.

### IsHoldingSomething

Checks both the EPick's OBJ register (object detection flags) and actual pressure. In continuous vacuum mode, OBJ may stay at 0 (regulating) even when holding an object, so the pressure fallback (any vacuum >10 kPa) ensures detection works for porous materials.

### DoCommand

**Get full gripper status:**
```json
{"get_status": true}
```
Returns: `object_detected`, `object_status_raw`, `pressure_register`, `pressure_kpa`, `fault_code`, `activation_status`, `mode`.

**Read a raw register:**
```json
{"get": "OBJ"}
```
Available registers: `ACT`, `MOD`, `GTO`, `STA`, `OBJ`, `FLT`, `POS`, `SPE`, `FOR`.

**Write raw registers:**
```json
{"set": {"MOD": "1", "POS": "22", "GTO": "1"}}
```

## Communication Architecture

All socket I/O runs on a **single goroutine** (`ioLoop`). Commands from `Grab()`, `Open()`, `DoCommand()`, etc. are sent via a channel to the I/O goroutine, which sends them on the TCP socket and returns the response. A **keepalive** (`GET ACT`) fires every 500ms on the same goroutine to prevent EPick fault 0x9 (communication timeout after 1 second of silence). The keepalive timer resets after every real command, so it only fires during idle periods.

This lockless design eliminates response interleaving — the keepalive and API commands can never overlap on the socket.

### Protocol

The Robotiq URCap exposes a TCP socket on port 63352:

```
SET <register> <value>\r\n  ->  "ack"
GET <register>\r\n          ->  "<register> <value>"
```

### EPick register mapping

| Register | SET (output) | GET (input) | Description |
|----------|-------------|-------------|-------------|
| `ACT` | Activate (0/1) | Activation echo | Gripper activation |
| `MOD` | Mode (0=auto, 1=advanced) | Mode echo | Operating mode |
| `GTO` | Regulate (0/1) | Regulate echo | Enable vacuum regulation |
| `POS` | Pressure request (0-255) | Actual pressure | 0=continuous, 1-99=grip, 100+=release |
| `SPE` | Grip timeout (0-255) | - | Each unit = 100ms, 0=no timeout |
| `FOR` | Min pressure (0-255) | - | Minimum vacuum threshold |
| `OBJ` | - | Object status (0-3) | 0=regulating, 1=min, 2=max, 3=none |
| `STA` | - | Activation status | 0=not activated, 3=operational |
| `FLT` | - | Fault code | 0=none, see manual for codes |

## Troubleshooting

### Module connects but times out
Port 63352 may be blocked by the UR's security settings. Check **Settings** > **Security** on the pendant and ensure port 63352 is not in the blocked range.

### "model not registered" error
Ensure both the module AND the gripper component are in the config. The module must be added separately from the component.

### Vacuum doesn't start / fault code 7
The gripper lost activation (ACT=0). This can happen after a protective stop or URCap reset. The module re-asserts ACT=1 on every Grab/Open call, so retrying should work. If it persists, check the pendant for safety faults.

### Vacuum turns off mid-move
Usually caused by the EPick communication timeout (fault 0x9) — the keepalive should prevent this. If it still happens, check network stability to the UR controller. A protective stop also cuts tool power.

### Weak suction on cardboard
The module uses continuous vacuum mode (POS=0, pump always ON). If suction is still weak, check that all suction cups have good contact and no air leaks around the edges.

## Building

```bash
make build    # Build binary
make module   # Build module archive (bin/module.tar.gz)
make test     # Run tests
make lint     # Run linter
```

## References

- [Robotiq EPick Instruction Manual (e-Series)](https://assets.robotiq.com/website-assets/support_documents/document/EPick_Instruction_Manual_e-Series_PDF_20210709.pdf)
- [Viam Gripper Component Docs](https://docs.viam.com/dev/reference/apis/components/gripper/)
- [viam-modules/robotiq](https://github.com/viam-modules/robotiq) (2F gripper reference implementation)

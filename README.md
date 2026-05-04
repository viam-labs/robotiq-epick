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

Add the **gripper component** with a frame parented to the arm:

### Automatic mode (default)

The gripper automatically detects vacuum levels, timeout, and hysteresis. It will grip for ~2 seconds and then hold indefinitely once an object is detected.

```json
{
  "name": "epick",
  "model": "shrews-testing:robotiq:epick",
  "type": "gripper",
  "namespace": "rdk",
  "attributes": {
    "host": "10.1.0.84"
  },
  "depends_on": [],
  "frame": {
    "parent": "arm"
  }
}
```

### Advanced mode

User sets desired vacuum levels and timeout. Best for consistent production behavior.

```json
{
  "name": "epick",
  "model": "shrews-testing:robotiq:epick",
  "type": "gripper",
  "namespace": "rdk",
  "attributes": {
    "host": "10.1.0.84",
    "mode": "advanced",
    "max_pressure_pct": 60,
    "min_pressure_pct": 40,
    "timeout_ms": 3000
  },
  "depends_on": [],
  "frame": {
    "parent": "arm"
  }
}
```

### Attributes

| Name | Type | Required | Default | Description |
|------|------|----------|---------|-------------|
| `host` | string | **Yes** | - | UR controller IP address |
| `port` | int | No | `63352` | URCap socket port |
| `mode` | string | No | `"automatic"` | `"automatic"` or `"advanced"` |
| `max_pressure_pct` | int | No | `60` | Maximum vacuum level (20-100%), advanced mode |
| `min_pressure_pct` | int | No | `40` | Minimum vacuum level (10-100%), advanced mode |
| `timeout_ms` | int | No | `3000` | Grip timeout in ms, advanced mode |

## Frame and Collision Geometry

The module returns a kinematic model via `Kinematics()` that defines both the collision geometry and the tool center point (TCP). No `translation` is needed in the frame config — just set `parent: "arm"`.

```
    UR Arm Flange (Z=0)
    ━━━━━━━━━━━━━━━━━━━━
    │                    │
    │  ┌──────────────┐  │
    │  │   Capsule     │  │  Z=0 to Z=70mm
    │  │  68mm dia     │  │  EPick body
    │  │  (collision)  │  │
    │  └──────────────┘  │
    │                    │
    │ ┌────────────────┐ │
    │ │                │ │
    │ │     Box        │ │  Z=70 to Z=170mm
    │ │  230 x 150mm   │ │  Bracket + hoses
    │ │  (collision)   │ │
    │ │                │ │
    │ └────────────────┘ │
    │                    │  Z=170 to Z=196mm
    │    ~ clearance ~   │  No collision (allows approach)
    │                    │
    ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄
         TCP (Z=196mm)      <-- planner targets poses here
        Suction cup tips
```

The collision geometry is intentionally undersized (stops at Z=170mm) so the motion planner allows the suction cups to reach the workpiece surface. The 26mm clearance between the collision boundary and TCP ensures the planner won't reject valid pick/place approaches.

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
| `Grab` | Activates vacuum, returns `true` if object detected |
| `IsHoldingSomething` | Checks suction status with pressure metadata |
| `Stop` | Stops vacuum regulation |
| `IsMoving` | Returns `true` if an operation is in progress |

### Automatic mode behavior

In automatic mode, `Grab` activates the vacuum generator for ~2 seconds. If an object is detected (sufficient vacuum reached), the EPick switches to **holding mode** and maintains vacuum indefinitely until `Open` is called. If no object is detected within the timeout, it stops and reports `success: false`.

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

## Communication Protocol

This module uses the same socket-based text protocol as the [viam-modules/robotiq](https://github.com/viam-modules/robotiq) 2F gripper module. The Robotiq URCap exposes a TCP socket on port 63352 that accepts commands in the format:

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
| `POS` | Pressure request (0-255) | Actual pressure | 0-99=grip, 100+=release |
| `SPE` | Grip timeout (0-255) | - | Each unit = 100ms |
| `FOR` | Min pressure (0-255) | - | Minimum vacuum threshold |
| `OBJ` | - | Object status (0-3) | 0=regulating, 1=min, 2=max, 3=none |
| `STA` | - | Activation status | 0=not activated, 3=operational |
| `FLT` | - | Fault code | 0=none, see manual for codes |

## Troubleshooting

### Module connects but times out
Port 63352 may be blocked by the UR's security settings. Check **Settings** > **Security** on the pendant and ensure port 63352 is not in the blocked range.

### "model not registered" error
Ensure both the module AND the gripper component are in the config. The module must be added separately from the component.

### Grab returns false but object is held
In automatic mode, the EPick's initial grip phase is ~2 seconds. Ensure the object is already in contact with the suction cup when `Grab` is called.

### Fault code 6 after grab
This is "grip timeout" -- the EPick tried to grip but couldn't detect an object within the timeout. The object may not have been sealed against the cup, or there's an air leak.

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

// Package epick implements the Robotiq EPick vacuum gripper driver.
//
// Reference: Robotiq EPick Instruction Manual for e-Series Universal Robots (2021-07-09)
// Section 4.2-4.7: Vacuum Gripper register mapping and Modbus RTU communication.
package epick

// Modbus register addresses.
// Each 16-bit register holds 2 bytes (high byte, low byte).
// Note: Robotiq data bytes are Little Endian within registers.
const (
	// Output (robot -> gripper) register base. Registers 0x03E8..0x03EA (1000..1002).
	outputRegBase = 0x03E8

	// Input (gripper -> robot) register base. Registers 0x07D0..0x07D2 (2000..2002).
	inputRegBase = 0x07D0

	// Number of output registers (3 registers = 6 bytes).
	numOutputRegs = 3

	// Number of input registers (3 registers = 6 bytes).
	numInputRegs = 3
)

// Output byte 0: ACTION REQUEST register bit positions.
const (
	bitRACT = 0 // Activate gripper
	bitRMOD = 1 // Mode (2 bits)
	bitRGTO = 3 // Regulate / go-to
	bitRATR = 4 // Automatic release
)

// rMOD values (gripper mode selection).
const (
	ModeAutomatic byte = 0x00 // Automatic mode: gripper auto-detects vacuum levels
	ModeAdvanced  byte = 0x01 // Advanced mode: user sets min/max vacuum, timeout
)

// rPR (Maximum Vacuum/Pressure Request) notable values.
// Formula: rPR = 100 + Pmax, where Pmax is target differential pressure in kPa.
const (
	PressureContinuous byte = 0x00 // Continuous grip, vacuum always ON
	PressureMaxVacuum  byte = 0x16 // 78% vacuum (max device vacuum)
	PressureMinVacuum  byte = 0x5A // 10% vacuum (min device vacuum)
	PressureRelease    byte = 0x64 // Passive release (ambient pressure)
)

// Input byte 0: GRIPPER STATUS register bit positions and masks.
const (
	bitGACT    = 0 // Activation echo (1 bit)
	bitGMOD    = 1 // Mode echo (2 bits)
	bitGGTO    = 3 // Regulate echo (1 bit)
	bitGSTA    = 4 // Activation status (2 bits)
	bitGOBJ    = 6 // Object detection (2 bits)
	maskGACT   = 0x01
	maskGMOD   = 0x06
	maskGGTO   = 0x08
	maskGSTA   = 0x30
	maskGOBJ   = 0xC0
)

// gSTA values (activation status).
const (
	StaNotActivated byte = 0x00
	StaOperational  byte = 0x03 // bits 4&5 both set
)

// gOBJ values (object detection status).
const (
	ObjUnknown        byte = 0x00 // Regulating towards requested vacuum
	ObjDetectedMin    byte = 0x01 // Object detected, minimum vacuum reached
	ObjDetectedMax    byte = 0x02 // Object detected, maximum vacuum reached
	ObjNoObject       byte = 0x03 // No object, dropped or timeout
)

// Input byte 1: GRIPPER STATUS EXTENSION bit positions.
const (
	bitGVAS  = 0 // Vacuum actuator status (2 bits)
	maskGVAS = 0x03
)

// gVAS values (vacuum actuator status).
const (
	VasStandby         byte = 0x00 // Vacuum generator OFF, valves deasserted
	VasGripping        byte = 0x01 // Vacuum generator ON
	VasPassiveRelease  byte = 0x02 // Releasing to ambient pressure
	VasActiveRelease   byte = 0x03 // Releasing with positive pressure
)

// Input byte 2: FAULT STATUS register.
const (
	maskGFLT = 0x0F // Gripper fault (lower nibble)
	maskKFLT = 0xF0 // Controller fault (upper nibble)
)

// gFLT fault code values.
const (
	FaultNone              byte = 0x0 // No fault
	FaultActionDelayed     byte = 0x5 // Priority: action delayed
	FaultPorousMaterial    byte = 0x3 // Priority: very porous material
	FaultGripTimeout       byte = 0x6 // Priority: gripping timeout, re-assert rGTO
	FaultActivationNotSet  byte = 0x7 // Priority: rACT must be set first
	FaultOverTemp          byte = 0x8 // Minor: max temp exceeded
	FaultNoComm            byte = 0x9 // Minor: no communication for 1 second
	FaultUnderVoltage      byte = 0xA // Major: under minimum voltage
	FaultAutoRelease       byte = 0xB // Major: auto release in progress
	FaultInternalFault     byte = 0xC // Major: internal fault
	FaultAutoReleaseComplete byte = 0xF // Major: auto release completed
)

// outputRegisters holds the 6 output bytes packed into 3 Modbus registers.
type outputRegisters struct {
	actionRequest byte // Byte 0: rACT, rMOD, rGTO, rATR
	reserved1     byte // Byte 1: reserved
	reserved2     byte // Byte 2: reserved
	maxPressure   byte // Byte 3: rPR
	gripTimeout   byte // Byte 4: rSP
	minPressure   byte // Byte 5: rFR
}

// toRegisters packs the output bytes into 3 uint16 Modbus registers.
func (o *outputRegisters) toRegisters() []uint16 {
	return []uint16{
		uint16(o.actionRequest)<<8 | uint16(o.reserved1),
		uint16(o.reserved2)<<8 | uint16(o.maxPressure),
		uint16(o.gripTimeout)<<8 | uint16(o.minPressure),
	}
}

// inputRegisters holds the 6 input bytes unpacked from 3 Modbus registers.
type inputRegisters struct {
	gripperStatus    byte // Byte 0: gACT, gMOD, gGTO, gSTA, gOBJ
	statusExtension  byte // Byte 1: gVAS
	faultStatus      byte // Byte 2: gFLT, kFLT
	maxPressureEcho  byte // Byte 3: gPR
	actualPressure   byte // Byte 4: gPO
	reserved         byte // Byte 5: reserved
}

// fromRegisters unpacks 3 uint16 Modbus registers into input bytes.
func (i *inputRegisters) fromRegisters(regs []uint16) {
	if len(regs) < numInputRegs {
		return
	}
	i.gripperStatus = byte(regs[0] >> 8)
	i.statusExtension = byte(regs[0] & 0xFF)
	i.faultStatus = byte(regs[1] >> 8)
	i.maxPressureEcho = byte(regs[1] & 0xFF)
	i.actualPressure = byte(regs[2] >> 8)
	i.reserved = byte(regs[2] & 0xFF)
}

// Helper methods on inputRegisters.

func (i *inputRegisters) isActivated() bool {
	return (i.gripperStatus & maskGACT) != 0
}

func (i *inputRegisters) isOperational() bool {
	return ((i.gripperStatus & maskGSTA) >> bitGSTA) == StaOperational
}

func (i *inputRegisters) objectStatus() byte {
	return (i.gripperStatus & maskGOBJ) >> bitGOBJ
}

func (i *inputRegisters) isObjectDetected() bool {
	obj := i.objectStatus()
	return obj == ObjDetectedMin || obj == ObjDetectedMax
}

func (i *inputRegisters) vacuumActuatorStatus() byte {
	return i.statusExtension & maskGVAS
}

func (i *inputRegisters) gripperFault() byte {
	return i.faultStatus & maskGFLT
}

// actualPressureKPa returns the differential pressure relative to ambient in kPa.
// Formula from manual: Pdiff = gPO - 100.
// Range: 0x00 = max vacuum (<= -100 kPa), 0x64 = ambient (0 kPa), 0xFF = max pressure (>= 155 kPa).
func (i *inputRegisters) actualPressureKPa() int {
	return int(i.actualPressure) - 100
}

// Package epick implements the Robotiq EPick vacuum gripper driver.
//
// Reference: Robotiq EPick Instruction Manual for e-Series Universal Robots (2021-07-09)
// Section 4.2-4.7: Vacuum Gripper register mapping and Modbus RTU communication.
package epick

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

// gSTA values (activation status).
const (
	StaNotActivated byte = 0x00
	StaOperational  byte = 0x03 // bits 4&5 both set
)

// gOBJ values (object detection status).
const (
	ObjUnknown     byte = 0x00 // Regulating towards requested vacuum
	ObjDetectedMin byte = 0x01 // Object detected, minimum vacuum reached
	ObjDetectedMax byte = 0x02 // Object detected, maximum vacuum reached
	ObjNoObject    byte = 0x03 // No object, dropped or timeout
)

// gVAS values (vacuum actuator status).
const (
	VasStandby        byte = 0x00 // Vacuum generator OFF, valves deasserted
	VasGripping       byte = 0x01 // Vacuum generator ON
	VasPassiveRelease byte = 0x02 // Releasing to ambient pressure
	VasActiveRelease  byte = 0x03 // Releasing with positive pressure
)

// gFLT fault code values.
const (
	FaultNone                byte = 0x0 // No fault
	FaultActionDelayed       byte = 0x5 // Priority: action delayed
	FaultPorousMaterial      byte = 0x3 // Priority: very porous material
	FaultGripTimeout         byte = 0x6 // Priority: gripping timeout, re-assert rGTO
	FaultActivationNotSet    byte = 0x7 // Priority: rACT must be set first
	FaultOverTemp            byte = 0x8 // Minor: max temp exceeded
	FaultNoComm              byte = 0x9 // Minor: no communication for 1 second
	FaultUnderVoltage        byte = 0xA // Major: under minimum voltage
	FaultAutoRelease         byte = 0xB // Major: auto release in progress
	FaultInternalFault       byte = 0xC // Major: internal fault
	FaultAutoReleaseComplete byte = 0xF // Major: auto release completed
)

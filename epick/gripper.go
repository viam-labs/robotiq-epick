// Package epick implements the Robotiq EPick vacuum gripper driver.
//
// Communication uses the Robotiq URCap socket protocol on port 63352.
// This is the same protocol used by the 2F gripper module, adapted
// for vacuum gripper semantics (pressure instead of position).
//
// Reference: Robotiq EPick Instruction Manual for e-Series (2021-07-09)
package epick

import (
	"context"
	"fmt"
	"net"
	"strconv"
	"strings"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils"
)

// Model is the Viam model for the Robotiq EPick vacuum gripper.
var Model = resource.NewModel("shrews-testing", "robotiq", "epick")

const defaultPort = 63352

// Config is the configuration for the EPick gripper.
type Config struct {
	// Host is the UR controller IP address (e.g. "192.168.1.100").
	// The Robotiq URCap must be installed on the UR controller.
	Host string `json:"host"`
	// Port is the URCap socket port. Default: 63352.
	Port int `json:"port,omitempty"`
	// Mode is the gripper operating mode: "automatic" or "advanced". Default: "automatic".
	Mode string `json:"mode,omitempty"`
	// MaxPressurePct is the maximum vacuum level (20-100%) for advanced mode. Default: 60.
	MaxPressurePct int `json:"max_pressure_pct,omitempty"`
	// MinPressurePct is the minimum vacuum level (10-100%) for advanced mode. Default: 40.
	MinPressurePct int `json:"min_pressure_pct,omitempty"`
	// TimeoutMs is the grip timeout in milliseconds for advanced mode. Default: 3000.
	TimeoutMs int `json:"timeout_ms,omitempty"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Host == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "host")
	}
	if cfg.Mode != "" && cfg.Mode != "automatic" && cfg.Mode != "advanced" {
		return nil, nil, fmt.Errorf("invalid mode %q, must be \"automatic\" or \"advanced\"", cfg.Mode)
	}
	if cfg.MaxPressurePct != 0 && (cfg.MaxPressurePct < 20 || cfg.MaxPressurePct > 100) {
		return nil, nil, fmt.Errorf("max_pressure_pct must be 20-100, got %d", cfg.MaxPressurePct)
	}
	if cfg.MinPressurePct != 0 && (cfg.MinPressurePct < 10 || cfg.MinPressurePct > 100) {
		return nil, nil, fmt.Errorf("min_pressure_pct must be 10-100, got %d", cfg.MinPressurePct)
	}
	return nil, nil, nil
}

func init() {
	resource.RegisterComponent(gripper.API, Model, resource.Registration[gripper.Gripper, *Config]{
		Constructor: newEPickGripper,
	})
}

// Default EPick collision geometry dimensions in mm.
// Intentionally undersized vs the physical gripper so the motion planner
// allows the suction cups to reach the workpiece surface.
// Physical widest point is 210x130mm, TCP at Z=196mm.
const (
	collisionX = 230.0 // 210mm widest + 20mm padding
	collisionY = 150.0 // 130mm widest + 20mm padding
	collisionZ = 170.0 // 150mm body height + 20mm top padding (stops before cup tips)
)

type epickGripper struct {
	resource.Named
	resource.AlwaysRebuild
	conn       net.Conn
	conf       *Config
	logger     logging.Logger
	opMgr      *operation.SingleOperationManager
	geometries []spatialmath.Geometry
}

func newEPickGripper(
	ctx context.Context,
	_ resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (gripper.Gripper, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}

	port := defaultPort
	if cfg.Port != 0 {
		port = cfg.Port
	}
	addr := net.JoinHostPort(cfg.Host, strconv.Itoa(port))

	logger.CInfof(ctx, "connecting to EPick URCap at %s", addr)
	conn, err := net.Dial("tcp", addr)
	if err != nil {
		return nil, fmt.Errorf("failed to connect to EPick URCap at %s: %w", addr, err)
	}

	g := &epickGripper{
		Named:  conf.ResourceName().AsNamed(),
		conn:   conn,
		conf:   cfg,
		logger: logger,
		opMgr:  operation.NewSingleOperationManager(),
	}

	// Build default collision geometry for the EPick.
	// Single box covering the gripper body, centered at half the collision height.
	// Stops short of the suction cup tips so the planner can approach surfaces.
	g.geometries = buildDefaultGeometries(g.Name().ShortName())

	// Override with frame geometry from config if provided.
	if conf.Frame != nil && conf.Frame.Geometry != nil {
		cfgGeom, err := conf.Frame.Geometry.ParseConfig()
		if err == nil {
			g.geometries = []spatialmath.Geometry{cfgGeom}
		}
	}

	if err := g.activate(ctx); err != nil {
		conn.Close()
		return nil, fmt.Errorf("failed to activate EPick: %w", err)
	}

	return g, nil
}

// activate sends the initialization sequence for the vacuum gripper.
func (g *epickGripper) activate(ctx context.Context) error {
	g.logger.CInfof(ctx, "activating EPick gripper")

	mode := "0"
	if g.conf.Mode == "advanced" {
		mode = "1"
	}

	cmds := [][]string{
		{"ACT", "1"},  // activate gripper
		{"MOD", mode}, // set operating mode
		{"GTO", "1"},  // enable regulation
	}

	// In advanced mode, set pressure and timeout parameters.
	if g.conf.Mode == "advanced" {
		maxPct := g.conf.MaxPressurePct
		if maxPct == 0 {
			maxPct = 60
		}
		minPct := g.conf.MinPressurePct
		if minPct == 0 {
			minPct = 40
		}
		timeoutMs := g.conf.TimeoutMs
		if timeoutMs == 0 {
			timeoutMs = 3000
		}
		// POS register = pressure request: 100 - pct (for grip)
		cmds = append(cmds, []string{"POS", strconv.Itoa(100 - maxPct)})
		// SPE register = timeout: each unit = 100ms
		cmds = append(cmds, []string{"SPE", strconv.Itoa(timeoutMs / 100)})
		// FOR register = min pressure: 100 - pct
		cmds = append(cmds, []string{"FOR", strconv.Itoa(100 - minPct)})
	}

	for _, cmd := range cmds {
		if err := g.Set(cmd[0], cmd[1]); err != nil {
			return fmt.Errorf("activation command SET %s %s failed: %w", cmd[0], cmd[1], err)
		}
		var waitTime time.Duration
		if cmd[0] == "ACT" {
			waitTime = 1600 * time.Millisecond
		} else {
			waitTime = 500 * time.Millisecond
		}
		if !utils.SelectContextOrWait(ctx, waitTime) {
			return ctx.Err()
		}
	}

	g.logger.CInfof(ctx, "EPick gripper activated")
	return nil
}

// --- Socket protocol ---

// Send writes a message and reads the response.
func (g *epickGripper) Send(msg string) (string, error) {
	_, err := g.conn.Write([]byte(msg))
	if err != nil {
		return "", err
	}
	return g.read()
}

// Set sends "SET <what> <to>\r\n" and expects "ack".
func (g *epickGripper) Set(what, to string) error {
	res, err := g.Send(fmt.Sprintf("SET %s %s\r\n", what, to))
	if err != nil {
		return err
	}
	if res != "ack" {
		return fmt.Errorf("expected ack, got [%s]", res)
	}
	return nil
}

// Get sends "GET <what>\r\n" and returns the response.
func (g *epickGripper) Get(what string) (string, error) {
	return g.Send(fmt.Sprintf("GET %s\r\n", what))
}

func (g *epickGripper) read() (string, error) {
	buf := make([]byte, 128)
	x, err := g.conn.Read(buf)
	if err != nil {
		return "", err
	}
	if x > 100 {
		return "", fmt.Errorf("read too much: %d bytes", x)
	}
	if x == 0 {
		return "", nil
	}
	return strings.TrimSpace(string(buf[0:x])), nil
}

// getInt sends a GET command and parses the integer value from "KEY value" response.
func (g *epickGripper) getInt(what string) (int, error) {
	res, err := g.Get(what)
	if err != nil {
		return 0, err
	}
	parts := strings.SplitN(res, " ", 2)
	if len(parts) != 2 {
		return 0, fmt.Errorf("unexpected response format: %q", res)
	}
	val, err := strconv.Atoi(parts[1])
	if err != nil {
		return 0, fmt.Errorf("failed to parse value from %q: %w", res, err)
	}
	return val, nil
}

// --- Gripper interface ---

// Open releases vacuum (sends release command).
func (g *epickGripper) Open(ctx context.Context, extra map[string]interface{}) error {
	ctx, done := g.opMgr.New(ctx)
	defer done()

	g.logger.CDebugf(ctx, "releasing vacuum (open)")

	// POS >= 100 = release command in automatic mode.
	if err := g.Set("POS", "100"); err != nil {
		return fmt.Errorf("release failed: %w", err)
	}

	// Wait for release to complete.
	return g.waitForObject(ctx, int(ObjNoObject), 5*time.Second)
}

// Grab activates vacuum and returns whether an object was detected.
func (g *epickGripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	ctx, done := g.opMgr.New(ctx)
	defer done()

	g.logger.CDebugf(ctx, "activating vacuum (grab)")

	// POS < 100 = grip command. Lower value = stronger vacuum.
	// In automatic mode, any value < 100 triggers auto grip.
	// In advanced mode, POS = 100 - max_vacuum_pct.
	gripVal := "22" // Default: ~78% vacuum (max device vacuum)
	if g.conf.Mode == "advanced" && g.conf.MaxPressurePct != 0 {
		gripVal = strconv.Itoa(100 - g.conf.MaxPressurePct)
	}

	if err := g.Set("POS", gripVal); err != nil {
		return false, fmt.Errorf("grip failed: %w", err)
	}

	// Wait for object detection or timeout.
	timeoutMs := 5000
	if g.conf.TimeoutMs != 0 {
		timeoutMs = g.conf.TimeoutMs + 2000
	}
	_ = g.waitForGrip(ctx, time.Duration(timeoutMs)*time.Millisecond)

	// Give the EPick a moment to settle, then check final state.
	// In automatic mode the EPick may still be transitioning to holding mode.
	if !utils.SelectContextOrWait(ctx, 500*time.Millisecond) {
		return false, ctx.Err()
	}

	obj, err := g.getInt("OBJ")
	if err != nil {
		return false, err
	}
	// OBJ 1 = min vacuum reached, OBJ 2 = max vacuum reached. Both mean object detected.
	return obj == 1 || obj == 2, nil
}

// IsHoldingSomething checks whether the gripper is currently holding an object.
func (g *epickGripper) IsHoldingSomething(ctx context.Context, extra map[string]interface{}) (gripper.HoldingStatus, error) {
	obj, err := g.getInt("OBJ")
	if err != nil {
		return gripper.HoldingStatus{}, err
	}

	holding := obj == 1 || obj == 2
	meta := map[string]interface{}{
		"object_status_raw": obj,
	}

	// Get actual pressure if available.
	pos, posErr := g.getInt("POS")
	if posErr == nil {
		meta["pressure_register"] = pos
		meta["pressure_kpa"] = pos - 100
	}

	switch obj {
	case 0:
		meta["object_status"] = "regulating"
	case 1:
		meta["object_status"] = "detected_min_vacuum"
	case 2:
		meta["object_status"] = "detected_max_vacuum"
	case 3:
		meta["object_status"] = "no_object"
	}

	return gripper.HoldingStatus{
		IsHoldingSomething: holding,
		Meta:               meta,
	}, nil
}

// Stop disables vacuum regulation by clearing GTO.
func (g *epickGripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	g.logger.CDebugf(ctx, "stopping gripper")
	return g.Set("GTO", "0")
}

// IsMoving returns whether the gripper is actively operating.
func (g *epickGripper) IsMoving(ctx context.Context) (bool, error) {
	return g.opMgr.OpRunning(), nil
}

// Geometries returns the EPick's spatial geometry for collision avoidance.
func (g *epickGripper) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return g.geometries, nil
}

// Kinematics returns the kinematic model (not applicable for vacuum gripper).
func (g *epickGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return nil, nil
}

// CurrentInputs returns current joint positions (not applicable for vacuum gripper).
func (g *epickGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return []referenceframe.Input{}, nil
}

// GoToInputs moves to specified joint positions (not applicable for vacuum gripper).
func (g *epickGripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return nil
}

// Status returns the current gripper status.
func (g *epickGripper) Status(ctx context.Context) (map[string]interface{}, error) {
	return g.getStatusMap()
}

// DoCommand provides direct register access and status feedback.
func (g *epickGripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["get_status"]; ok {
		return g.getStatusMap()
	}

	if rawGet, ok := cmd["get"]; ok {
		reg, ok := rawGet.(string)
		if !ok {
			return nil, fmt.Errorf("get expects a string register name")
		}
		res, err := g.Get(reg)
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{"response": res}, nil
	}

	if rawSet, ok := cmd["set"]; ok {
		setMap, ok := rawSet.(map[string]interface{})
		if !ok {
			return nil, fmt.Errorf("set expects a map of register:value pairs")
		}
		for reg, val := range setMap {
			valStr := fmt.Sprintf("%v", val)
			if err := g.Set(reg, valStr); err != nil {
				return nil, fmt.Errorf("SET %s %s failed: %w", reg, valStr, err)
			}
		}
		return map[string]interface{}{"ok": true}, nil
	}

	return nil, fmt.Errorf("unknown command, supported: get_status, get, set")
}

// Close disconnects from the URCap.
func (g *epickGripper) Close(ctx context.Context) error {
	g.logger.CInfof(ctx, "closing EPick connection")
	if g.conn != nil {
		return g.conn.Close()
	}
	return nil
}

// --- Wait helpers ---

// buildDefaultGeometries creates the EPick's collision geometry.
// Two parts along +Z from the flange:
//   - Cylinder (capsule): 70mm tall from flange, covers the EPick body
//   - Box: remaining 100mm, covers the bracket, hoses, and suction cups
func buildDefaultGeometries(label string) []spatialmath.Geometry {
	var geoms []spatialmath.Geometry

	const cylinderHeight = 70.0
	boxHeight := collisionZ - cylinderHeight // 100mm

	// EPick body: capsule centered at Z=35mm (midpoint of 0..70)
	body, err := spatialmath.NewCapsule(
		spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: cylinderHeight / 2}),
		collisionY/2, // radius = 75mm (150/2), covers body + padding
		cylinderHeight,
		label+"-body",
	)
	if err == nil {
		geoms = append(geoms, body)
	}

	// Bracket + hoses + cups: box centered at Z=120mm (midpoint of 70..170)
	bracket, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: cylinderHeight + boxHeight/2}),
		r3.Vector{X: collisionX, Y: collisionY, Z: boxHeight},
		label+"-bracket",
	)
	if err == nil {
		geoms = append(geoms, bracket)
	}

	return geoms
}

func (g *epickGripper) waitForObject(ctx context.Context, target int, timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		if err := ctx.Err(); err != nil {
			return err
		}
		obj, err := g.getInt("OBJ")
		if err == nil && obj == target {
			return nil
		}
		if !utils.SelectContextOrWait(ctx, 100*time.Millisecond) {
			return ctx.Err()
		}
	}
	return nil // Timeout on release is not necessarily an error.
}

func (g *epickGripper) waitForGrip(ctx context.Context, timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		if err := ctx.Err(); err != nil {
			return err
		}
		obj, err := g.getInt("OBJ")
		if err == nil {
			// OBJ 1 = min vacuum reached, OBJ 2 = max vacuum reached.
			if obj == 1 || obj == 2 {
				return nil
			}
			// OBJ 3 = no object / grip timeout. Stop waiting.
			if obj == 3 {
				return nil
			}
		}
		if !utils.SelectContextOrWait(ctx, 100*time.Millisecond) {
			return ctx.Err()
		}
	}
	return fmt.Errorf("grip timed out waiting for object detection")
}

func (g *epickGripper) getStatusMap() (map[string]interface{}, error) {
	result := map[string]interface{}{}

	if obj, err := g.getInt("OBJ"); err == nil {
		result["object_status_raw"] = obj
		result["object_detected"] = obj == 1 || obj == 2
	}
	if pos, err := g.getInt("POS"); err == nil {
		result["pressure_register"] = pos
		result["pressure_kpa"] = pos - 100
	}
	if flt, err := g.getInt("FLT"); err == nil {
		result["fault_code"] = flt
	}
	if sta, err := g.getInt("STA"); err == nil {
		result["activation_status"] = sta
	}
	if mod, err := g.getInt("MOD"); err == nil {
		result["mode"] = mod
	}

	return result, nil
}

// Package epick implements the Robotiq EPick vacuum gripper driver.
//
// Communication uses the Robotiq URCap socket protocol on port 63352.
// This is the same protocol used by the 2F gripper module, adapted
// for vacuum gripper semantics (pressure instead of position).
//
// All socket I/O runs on a single goroutine to avoid response interleaving.
// A keepalive poll (GET ACT) fires every 500ms to prevent EPick fault 0x9.
//
// Reference: Robotiq EPick Instruction Manual for e-Series (2021-07-09)
package epick

import (
	"context"
	_ "embed"
	"fmt"
	"net"
	"strconv"
	"strings"
	"time"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils"
)

//go:embed epick_model.json
var epickModelJSON []byte

// Model is the Viam model for the Robotiq EPick vacuum gripper.
var Model = resource.NewModel("shrews-testing", "robotiq", "epick")

const defaultPort = 63352

// Config is the configuration for the EPick gripper.
type Config struct {
	Host           string `json:"host"`
	Port           int    `json:"port,omitempty"`
	Mode           string `json:"mode,omitempty"`
	MaxPressurePct int    `json:"max_pressure_pct,omitempty"`
	MinPressurePct int    `json:"min_pressure_pct,omitempty"`
	TimeoutMs      int    `json:"timeout_ms,omitempty"`
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

// socketRequest is sent to the I/O goroutine via the requests channel.
type socketRequest struct {
	msg  string
	resp chan socketResponse
}

type socketResponse struct {
	data string
	err  error
}

type epickGripper struct {
	resource.Named
	resource.AlwaysRebuild
	conf     *Config
	logger   logging.Logger
	opMgr    *operation.SingleOperationManager
	requests chan socketRequest // all socket I/O funnels through here
	done     chan struct{}      // closed when the I/O goroutine exits
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
		Named:    conf.ResourceName().AsNamed(),
		conf:     cfg,
		logger:   logger,
		opMgr:    operation.NewSingleOperationManager(),
		requests: make(chan socketRequest),
		done:     make(chan struct{}),
	}

	// Single goroutine owns the socket. All reads/writes go through it.
	// Keepalive fires on the same goroutine between commands — no races.
	go g.ioLoop(conn)

	if err := g.activate(ctx); err != nil {
		close(g.requests) // signal ioLoop to exit
		<-g.done
		return nil, fmt.Errorf("failed to activate EPick: %w", err)
	}

	return g, nil
}

// ioLoop runs on a single goroutine. It owns the socket and handles
// all send/receive plus keepalive. No locks needed.
func (g *epickGripper) ioLoop(conn net.Conn) {
	defer close(g.done)
	defer conn.Close()

	buf := make([]byte, 128)
	keepalive := time.NewTicker(500 * time.Millisecond)
	defer keepalive.Stop()

	send := func(msg string) (string, error) {
		if _, err := conn.Write([]byte(msg)); err != nil {
			return "", err
		}
		conn.SetReadDeadline(time.Now().Add(2 * time.Second))
		n, err := conn.Read(buf)
		if err != nil {
			return "", err
		}
		return strings.TrimSpace(string(buf[:n])), nil
	}

	for {
		select {
		case req, ok := <-g.requests:
			if !ok {
				return // channel closed, shut down
			}
			data, err := send(req.msg)
			req.resp <- socketResponse{data, err}

			// Reset keepalive timer after any real command,
			// since we just communicated.
			keepalive.Reset(500 * time.Millisecond)

		case <-keepalive.C:
			// Prevent EPick fault 0x9 (no communication for 1 second).
			_, _ = send("GET ACT\n")
		}
	}
}

// Send dispatches a message to the I/O goroutine and waits for the response.
func (g *epickGripper) Send(msg string) (string, error) {
	resp := make(chan socketResponse, 1)
	select {
	case g.requests <- socketRequest{msg, resp}:
	case <-g.done:
		return "", fmt.Errorf("connection closed")
	}
	select {
	case r := <-resp:
		return r.data, r.err
	case <-g.done:
		return "", fmt.Errorf("connection closed")
	}
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

// --- Activation ---

func (g *epickGripper) activate(ctx context.Context) error {
	g.logger.CInfof(ctx, "activating EPick gripper")

	mode := "0"
	if g.conf.Mode == "advanced" {
		mode = "1"
	}

	cmds := [][]string{
		{"ACT", "1"},
		{"MOD", mode},
		{"GTO", "1"},
	}

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
		cmds = append(cmds, []string{"POS", strconv.Itoa(100 - maxPct)})
		cmds = append(cmds, []string{"SPE", strconv.Itoa(timeoutMs / 100)})
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

// --- Gripper interface ---

// Open releases vacuum. Stays in advanced mode for the next Grab().
func (g *epickGripper) Open(ctx context.Context, extra map[string]interface{}) error {
	ctx, done := g.opMgr.New(ctx)
	defer done()

	g.logger.CDebugf(ctx, "releasing vacuum (open)")

	if err := g.Set("ACT", "1"); err != nil {
		return fmt.Errorf("release failed: %w", err)
	}
	if err := g.Set("GTO", "0"); err != nil {
		return fmt.Errorf("release failed: %w", err)
	}
	if err := g.Set("POS", "100"); err != nil {
		return fmt.Errorf("release failed: %w", err)
	}
	if err := g.Set("GTO", "1"); err != nil {
		return fmt.Errorf("release failed: %w", err)
	}

	return g.waitForObject(ctx, int(ObjNoObject), 5*time.Second)
}

// Grab activates vacuum. Non-blocking by default — returns immediately after
// turning on vacuum. Pass extra["blocking"]=true to wait for object detection.
func (g *epickGripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	ctx, done := g.opMgr.New(ctx)
	defer done()

	g.logger.CDebugf(ctx, "activating vacuum (grab)")

	// Ensure gripper is activated (rACT=1 must be set at all times).
	// Re-activation is needed after a URCap reset or module restart.
	if err := g.Set("ACT", "1"); err != nil {
		return false, fmt.Errorf("grip failed: %w", err)
	}

	// Switch to advanced mode with continuous vacuum and no timeout.
	if err := g.Set("GTO", "0"); err != nil {
		return false, fmt.Errorf("grip failed: %w", err)
	}
	if err := g.Set("MOD", "1"); err != nil {
		return false, fmt.Errorf("grip failed: %w", err)
	}

	gripVal := "0"
	if g.conf.Mode == "advanced" && g.conf.MaxPressurePct != 0 {
		gripVal = strconv.Itoa(100 - g.conf.MaxPressurePct)
	}
	if err := g.Set("POS", gripVal); err != nil {
		return false, fmt.Errorf("grip failed: %w", err)
	}
	if err := g.Set("SPE", "0"); err != nil {
		return false, fmt.Errorf("grip failed: %w", err)
	}
	forVal := "90"
	if g.conf.MinPressurePct != 0 {
		forVal = strconv.Itoa(100 - g.conf.MinPressurePct)
	}
	if err := g.Set("FOR", forVal); err != nil {
		return false, fmt.Errorf("grip failed: %w", err)
	}
	if err := g.Set("GTO", "1"); err != nil {
		return false, fmt.Errorf("grip failed: %w", err)
	}

	// Non-blocking by default.
	blocking := false
	if extra != nil {
		if b, ok := extra["blocking"]; ok {
			if v, ok := b.(bool); ok && v {
				blocking = true
			}
		}
	}
	if !blocking {
		g.logger.CDebugf(ctx, "vacuum activated, returning immediately (non-blocking)")
		return false, nil
	}

	timeoutMs := 5000
	if g.conf.TimeoutMs != 0 {
		timeoutMs = g.conf.TimeoutMs + 2000
	}
	_ = g.waitForGrip(ctx, time.Duration(timeoutMs)*time.Millisecond)

	if !utils.SelectContextOrWait(ctx, 500*time.Millisecond) {
		return false, ctx.Err()
	}

	obj, err := g.getInt("OBJ")
	if err != nil {
		return false, err
	}
	return obj == 1 || obj == 2, nil
}

// IsHoldingSomething checks vacuum seal. Also checks actual pressure for
// continuous mode where OBJ may stay at 0.
func (g *epickGripper) IsHoldingSomething(ctx context.Context, extra map[string]interface{}) (gripper.HoldingStatus, error) {
	obj, err := g.getInt("OBJ")
	if err != nil {
		return gripper.HoldingStatus{}, err
	}

	meta := map[string]interface{}{
		"object_status_raw": obj,
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

	holding := obj == 1 || obj == 2

	pos, posErr := g.getInt("POS")
	if posErr == nil {
		meta["pressure_register"] = pos
		meta["pressure_kpa"] = pos - 100
		if pos < 90 {
			holding = true
		}
	}

	return gripper.HoldingStatus{
		IsHoldingSomething: holding,
		Meta:               meta,
	}, nil
}

// Stop disables vacuum regulation.
func (g *epickGripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	g.logger.CDebugf(ctx, "stopping gripper")
	return g.Set("GTO", "0")
}

// IsMoving returns whether an operation is in progress.
func (g *epickGripper) IsMoving(ctx context.Context) (bool, error) {
	return g.opMgr.OpRunning(), nil
}

// Geometries returns collision geometry from the embedded kinematic model.
func (g *epickGripper) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	model, err := g.Kinematics(ctx)
	if err != nil {
		return nil, nil
	}
	gif, err := model.Geometries([]referenceframe.Input{})
	if err != nil {
		return nil, nil
	}
	return gif.Geometries(), nil
}

// Kinematics returns the embedded kinematic model (collision geometry + TCP offset).
func (g *epickGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return referenceframe.UnmarshalModelJSON(epickModelJSON, g.Name().ShortName())
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

// Close shuts down the I/O goroutine and disconnects.
func (g *epickGripper) Close(ctx context.Context) error {
	g.logger.CInfof(ctx, "closing EPick connection")
	close(g.requests)
	<-g.done
	return nil
}

// --- Wait helpers ---

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
	return nil
}

func (g *epickGripper) waitForGrip(ctx context.Context, timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		if err := ctx.Err(); err != nil {
			return err
		}
		obj, err := g.getInt("OBJ")
		if err == nil {
			if obj == 1 || obj == 2 {
				return nil
			}
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

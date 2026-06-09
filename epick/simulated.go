// Simulated EPick vacuum gripper — implements the gripper API without hardware.
//
// It exists to test motion planning and gripper-driven state machines (e.g. the
// palletizer's ENABLING_VACUUM step) without a real robot. It reuses the real
// EPick's embedded kinematic model and collision mesh, so poses and collision
// geometry match the hardware model.
//
// Holding behavior:
//   - Grab()  ("close" / engage vacuum): IsHoldingSomething() becomes true after
//     an adjustable delay (grab_delay_ms), simulating vacuum buildup / detection.
//   - Open()  (release): IsHoldingSomething() becomes false immediately.
package epick

import (
	"context"
	"fmt"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils"
)

// SimModel is the Viam model for the simulated EPick vacuum gripper.
var SimModel = resource.NewModel("viam", "robotiq", "simulated-epick-vacuum-gripper")

// defaultGrabDelayMs is the delay after Grab() before IsHoldingSomething()
// reports true, simulating vacuum buildup. Adjustable via config or DoCommand.
const defaultGrabDelayMs = 1000

// SimConfig configures the simulated gripper.
type SimConfig struct {
	// GrabDelayMs is the delay after Grab() before IsHoldingSomething() returns
	// true. 0 uses defaultGrabDelayMs; set explicitly (including a small value)
	// to simulate faster/slower vacuum buildup.
	GrabDelayMs int `json:"grab_delay_ms,omitempty"`
}

func (cfg *SimConfig) Validate(path string) ([]string, []string, error) {
	if cfg.GrabDelayMs < 0 {
		return nil, nil, fmt.Errorf("grab_delay_ms must be >= 0, got %d", cfg.GrabDelayMs)
	}
	return nil, nil, nil
}

func init() {
	resource.RegisterComponent(gripper.API, SimModel, resource.Registration[gripper.Gripper, *SimConfig]{
		Constructor: newSimGripper,
	})
}

type simGripper struct {
	resource.Named
	resource.AlwaysRebuild
	logger logging.Logger
	opMgr  *operation.SingleOperationManager

	mu        sync.Mutex
	grabDelay time.Duration
	// grabbedAt is when Grab() was last called; zero when released (not holding).
	grabbedAt time.Time
}

func newSimGripper(
	ctx context.Context,
	_ resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (gripper.Gripper, error) {
	cfg, err := resource.NativeConfig[*SimConfig](conf)
	if err != nil {
		return nil, err
	}

	delayMs := cfg.GrabDelayMs
	if delayMs == 0 {
		delayMs = defaultGrabDelayMs
	}

	g := &simGripper{
		Named:     conf.ResourceName().AsNamed(),
		logger:    logger,
		opMgr:     operation.NewSingleOperationManager(),
		grabDelay: time.Duration(delayMs) * time.Millisecond,
	}
	logger.CInfof(ctx, "simulated EPick gripper ready (grab delay %dms)", delayMs)
	return g, nil
}

// Grab simulates engaging vacuum. IsHoldingSomething() reports true once the
// hold delay elapses. Non-blocking by default; pass extra["blocking"]=true to
// wait for the delay and return the holding result.
func (g *simGripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	ctx, done := g.opMgr.New(ctx)
	defer done()

	g.mu.Lock()
	g.grabbedAt = time.Now()
	delay := g.grabDelay
	g.mu.Unlock()
	g.logger.CDebugf(ctx, "sim grab: holding in %v", delay)

	blocking := false
	if extra != nil {
		if b, ok := extra["blocking"].(bool); ok {
			blocking = b
		}
	}
	if !blocking {
		return false, nil
	}

	if !utils.SelectContextOrWait(ctx, delay) {
		return false, ctx.Err()
	}
	return g.holding(), nil
}

// Open simulates releasing vacuum. IsHoldingSomething() becomes false at once.
func (g *simGripper) Open(ctx context.Context, extra map[string]interface{}) error {
	ctx, done := g.opMgr.New(ctx)
	defer done()

	g.mu.Lock()
	g.grabbedAt = time.Time{}
	g.mu.Unlock()
	g.logger.CDebugf(ctx, "sim open: released")
	return nil
}

// holding reports whether the gripper is currently holding: grabbed and the
// hold delay has elapsed since the grab.
func (g *simGripper) holding() bool {
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.grabbedAt.IsZero() {
		return false
	}
	return time.Since(g.grabbedAt) >= g.grabDelay
}

// IsHoldingSomething reports the simulated holding state.
func (g *simGripper) IsHoldingSomething(ctx context.Context, extra map[string]interface{}) (gripper.HoldingStatus, error) {
	g.mu.Lock()
	grabbedAt := g.grabbedAt
	delay := g.grabDelay
	g.mu.Unlock()

	holding := !grabbedAt.IsZero() && time.Since(grabbedAt) >= delay
	meta := map[string]interface{}{
		"simulated":     true,
		"grab_delay_ms": delay.Milliseconds(),
		"grabbed":       !grabbedAt.IsZero(),
	}
	if !grabbedAt.IsZero() && !holding {
		// Still inside the delay window — report how long until holding.
		meta["holding_in_ms"] = (delay - time.Since(grabbedAt)).Milliseconds()
	}

	return gripper.HoldingStatus{IsHoldingSomething: holding, Meta: meta}, nil
}

// Stop releases the simulated grip.
func (g *simGripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	g.mu.Lock()
	g.grabbedAt = time.Time{}
	g.mu.Unlock()
	return nil
}

// IsMoving returns whether an operation is in progress.
func (g *simGripper) IsMoving(ctx context.Context) (bool, error) {
	return g.opMgr.OpRunning(), nil
}

// Geometries returns the EPick collision mesh, positioned like the real model.
func (g *simGripper) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	pose := spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: -196})
	mesh, err := spatialmath.NewMeshFromProto(
		pose,
		&commonpb.Mesh{ContentType: "stl", Mesh: epickSTL},
		g.Name().ShortName(),
	)
	if err != nil {
		return nil, err
	}
	return []spatialmath.Geometry{mesh}, nil
}

// Kinematics returns the embedded kinematic model (shared with the real EPick).
func (g *simGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return referenceframe.UnmarshalModelJSON(epickModelJSON, g.Name().ShortName())
}

// CurrentInputs returns current joint positions (not applicable for vacuum gripper).
func (g *simGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return []referenceframe.Input{}, nil
}

// GoToInputs moves to specified joint positions (not applicable for vacuum gripper).
func (g *simGripper) GoToInputs(ctx context.Context, inputs ...[]referenceframe.Input) error {
	return nil
}

// DoCommand supports adjusting the grab delay at runtime and reading status.
//
//	{"set_grab_delay_ms": 250}  -> change the delay live
//	{"get_status": true}        -> {grabbed, holding, grab_delay_ms}
func (g *simGripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if raw, ok := cmd["set_grab_delay_ms"]; ok {
		ms, ok := raw.(float64) // JSON numbers decode to float64
		if !ok || ms < 0 {
			return nil, fmt.Errorf("set_grab_delay_ms expects a non-negative number, got %v", raw)
		}
		g.mu.Lock()
		g.grabDelay = time.Duration(ms) * time.Millisecond
		g.mu.Unlock()
		g.logger.CInfof(ctx, "sim grab delay set to %vms", ms)
		return map[string]interface{}{"grab_delay_ms": ms}, nil
	}
	if _, ok := cmd["get_status"]; ok {
		g.mu.Lock()
		grabbedAt := g.grabbedAt
		delay := g.grabDelay
		g.mu.Unlock()
		return map[string]interface{}{
			"grabbed":       !grabbedAt.IsZero(),
			"holding":       !grabbedAt.IsZero() && time.Since(grabbedAt) >= delay,
			"grab_delay_ms": delay.Milliseconds(),
		}, nil
	}
	return nil, fmt.Errorf("unknown command, supported: set_grab_delay_ms, get_status")
}

// Close shuts down the simulated gripper.
func (g *simGripper) Close(ctx context.Context) error {
	return nil
}

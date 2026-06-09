package epick

import (
	"context"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/test"
)

func newTestSimGripper(t *testing.T, delay time.Duration) *simGripper {
	t.Helper()
	return &simGripper{
		logger:    logging.NewTestLogger(t),
		opMgr:     operation.NewSingleOperationManager(),
		grabDelay: delay,
	}
}

func isHolding(t *testing.T, g *simGripper) bool {
	t.Helper()
	st, err := g.IsHoldingSomething(context.Background(), nil)
	test.That(t, err, test.ShouldBeNil)
	return st.IsHoldingSomething
}

func TestSimGripperGrabDelay(t *testing.T) {
	ctx := context.Background()
	g := newTestSimGripper(t, 100*time.Millisecond)

	// Nothing grabbed yet.
	test.That(t, isHolding(t, g), test.ShouldBeFalse)

	// Grab (non-blocking): not holding until the delay elapses.
	grabbed, err := g.Grab(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, grabbed, test.ShouldBeFalse)
	test.That(t, isHolding(t, g), test.ShouldBeFalse)

	// After the delay, holding flips true.
	time.Sleep(150 * time.Millisecond)
	test.That(t, isHolding(t, g), test.ShouldBeTrue)

	// Open releases immediately.
	test.That(t, g.Open(ctx, nil), test.ShouldBeNil)
	test.That(t, isHolding(t, g), test.ShouldBeFalse)
}

func TestSimGripperBlockingGrab(t *testing.T) {
	ctx := context.Background()
	g := newTestSimGripper(t, 50*time.Millisecond)

	// Blocking grab waits out the delay and reports holding.
	grabbed, err := g.Grab(ctx, map[string]interface{}{"blocking": true})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, grabbed, test.ShouldBeTrue)
	test.That(t, isHolding(t, g), test.ShouldBeTrue)
}

func TestSimGripperStopReleases(t *testing.T) {
	ctx := context.Background()
	g := newTestSimGripper(t, 0)

	_, err := g.Grab(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, isHolding(t, g), test.ShouldBeTrue) // zero delay -> immediate

	test.That(t, g.Stop(ctx, nil), test.ShouldBeNil)
	test.That(t, isHolding(t, g), test.ShouldBeFalse)
}

func TestSimGripperDoCommandSetDelay(t *testing.T) {
	ctx := context.Background()
	g := newTestSimGripper(t, time.Second)

	resp, err := g.DoCommand(ctx, map[string]interface{}{"set_grab_delay_ms": float64(20)})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["grab_delay_ms"], test.ShouldEqual, float64(20))

	_, err = g.Grab(ctx, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, isHolding(t, g), test.ShouldBeFalse)
	time.Sleep(40 * time.Millisecond)
	test.That(t, isHolding(t, g), test.ShouldBeTrue)
}

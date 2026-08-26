package epick

import (
	"context"
	"encoding/json"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

// eps is the tolerance for comparing dimensions authored to 0.1mm.
const eps = 0.01

// bounds returns the exact axis-aligned min and max corners of a set of
// geometries, in mm. Every part of this gripper is axis-aligned, so the AABB is
// just the center plus a half-extent read off the geometry's config — exact,
// unlike sampling ToPoints, which never lands on a cylinder's extreme.
func bounds(t *testing.T, geoms []spatialmath.Geometry) (minC, maxC r3.Vector) {
	t.Helper()
	test.That(t, len(geoms), test.ShouldBeGreaterThan, 0)

	minC = r3.Vector{X: math.Inf(1), Y: math.Inf(1), Z: math.Inf(1)}
	maxC = r3.Vector{X: math.Inf(-1), Y: math.Inf(-1), Z: math.Inf(-1)}
	for _, g := range geoms {
		cfg, err := spatialmath.NewGeometryConfig(g)
		test.That(t, err, test.ShouldBeNil)

		var half r3.Vector
		switch cfg.Type {
		case spatialmath.BoxType:
			half = r3.Vector{X: cfg.X / 2, Y: cfg.Y / 2, Z: cfg.Z / 2}
		case spatialmath.CylinderType:
			half = r3.Vector{X: cfg.R, Y: cfg.R, Z: cfg.L / 2}
		default:
			t.Fatalf("unexpected geometry type %q for %q", cfg.Type, g.Label())
		}

		c := g.Pose().Point()
		minC = r3.Vector{X: math.Min(minC.X, c.X-half.X), Y: math.Min(minC.Y, c.Y-half.Y), Z: math.Min(minC.Z, c.Z-half.Z)}
		maxC = r3.Vector{X: math.Max(maxC.X, c.X+half.X), Y: math.Max(maxC.Y, c.Y+half.Y), Z: math.Max(maxC.Z, c.Z+half.Z)}
	}
	return minC, maxC
}

// byLabel indexes geometries by label so tests can assert on a named part
// without depending on ordering.
func byLabel(t *testing.T, geoms []spatialmath.Geometry) map[string]spatialmath.Geometry {
	t.Helper()
	out := make(map[string]spatialmath.Geometry, len(geoms))
	for _, g := range geoms {
		test.That(t, g.Label(), test.ShouldNotBeBlank)
		_, dup := out[g.Label()]
		test.That(t, dup, test.ShouldBeFalse)
		out[g.Label()] = g
	}
	return out
}

// qualified builds the label referenceframe gives a model's geometries.
func qualified(part string) string { return partLabel("epick", part) }

func collisionGeometries(t *testing.T) []spatialmath.Geometry {
	t.Helper()
	model, err := referenceframe.UnmarshalModelJSON(epickModelJSON, "epick")
	test.That(t, err, test.ShouldBeNil)
	gif, err := model.Geometries(nil)
	test.That(t, err, test.ShouldBeNil)
	return gif.Geometries()
}

// newGeometrySimGripper builds a simulated gripper with a resource name, which
// Geometries() and Kinematics() both need for the geometry label.
func newGeometrySimGripper(t *testing.T, includeRealsense bool) *simGripper {
	t.Helper()
	return &simGripper{
		Named:            resource.NewName(gripper.API, "epick").AsNamed(),
		logger:           logging.NewTestLogger(t),
		opMgr:            operation.NewSingleOperationManager(),
		includeRealsense: includeRealsense,
	}
}

// The collision model is the EPick reduced to primitives: one body cylinder,
// one mounting plate, and four suction cups.
func TestCollisionModelIsSixPrimitives(t *testing.T) {
	geoms := collisionGeometries(t)
	test.That(t, len(geoms), test.ShouldEqual, 6)

	var cylinders, boxes int
	for _, g := range geoms {
		switch g.(type) {
		case *spatialmath.Cylinder:
			cylinders++
		default:
			boxes++
		}
	}
	test.That(t, cylinders, test.ShouldEqual, 5)
	test.That(t, boxes, test.ShouldEqual, 1)
}

// The body is drawn as a true cylinder matching the CAD. A capsule cannot
// describe it at all: a capsule is a segment swept by a sphere, so its ends are
// always domed, never flat.
func TestVisualBodyIsCylinderMatchingCAD(t *testing.T) {
	visual, err := visualGeometries(false, "epick")
	test.That(t, err, test.ShouldBeNil)

	body, ok := byLabel(t, visual)[qualified(partBody)]
	test.That(t, ok, test.ShouldBeTrue)

	cyl, isCyl := body.(*spatialmath.Cylinder)
	test.That(t, isCyl, test.ShouldBeTrue)

	want, err := spatialmath.NewCylinder(
		spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: bodyVisualCenterZ}),
		bodyRadius, bodyVisualLength, qualified(partBody))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.GeometriesAlmostEqual(cyl, want), test.ShouldBeTrue)
}

// Nothing in the collision model may cross into the approach gap. This is the
// invariant that keeps every grab from registering as a collision, and it is
// why the cups are modeled shorter than they really are.
func TestCollisionModelClearsTCP(t *testing.T) {
	_, maxC := bounds(t, collisionGeometries(t))
	test.That(t, maxC.Z, test.ShouldAlmostEqual, tcpClearanceZ, eps)
}

// The camera is excluded from collision on purpose: the RealSense component
// supplies its own geometry through its own frame.
//
// Assert the collision model stops at its own -Y envelope, not merely short of
// the camera. Thresholding at the camera's far edge would be vacuous: the camera
// body's own minimum is Y=-107.15, inside cameraReachY, so adding it to the
// collision model would not trip a `> cameraReachY` check.
func TestCollisionModelExcludesCamera(t *testing.T) {
	minC, _ := bounds(t, collisionGeometries(t))
	test.That(t, minC.Y, test.ShouldAlmostEqual, collisionReachY, eps)
	test.That(t, collisionReachY, test.ShouldBeGreaterThan, cameraReachY)

	// Neither camera part may appear under any name.
	parts := byLabel(t, collisionGeometries(t))
	for _, name := range []string{partCameraBracket, partCamera} {
		_, present := parts[qualified(name)]
		test.That(t, present, test.ShouldBeFalse)
	}
}

// The collision model stops at the arm's flange face. The body's rear boss
// reaches 3mm further in reality, into the space the arm's own end-effector
// geometry occupies; the model this replaces ended at the flange too, so the
// rear collision boundary must not move.
func TestCollisionModelStopsAtFlange(t *testing.T) {
	minC, maxC := bounds(t, collisionGeometries(t))
	test.That(t, minC.Z, test.ShouldAlmostEqual, flangePlaneZ, eps)
	test.That(t, maxC.Z, test.ShouldBeLessThan, 0.0)
}

// The render draws the body's full length, past the flange face.
func TestVisualBodyReachesPastFlange(t *testing.T) {
	visual, err := visualGeometries(false, "epick")
	test.That(t, err, test.ShouldBeNil)
	minC, _ := bounds(t, visual)
	test.That(t, minC.Z, test.ShouldAlmostEqual, bodyRearZ, eps)
	test.That(t, bodyRearZ, test.ShouldBeLessThan, flangePlaneZ)
}

// The visualization is the true shape: the same body and plate the planner
// sees, but with the suction cups at their real length.
func TestVisualGeometriesUseFullLengthCups(t *testing.T) {
	visual, err := visualGeometries(false, "epick")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(visual), test.ShouldEqual, 6)

	_, maxC := bounds(t, visual)
	test.That(t, maxC.Z, test.ShouldAlmostEqual, cupTipZ, eps)

	cup, ok := byLabel(t, visual)[qualified("cup-xp-yp")].(*spatialmath.Cylinder)
	test.That(t, ok, test.ShouldBeTrue)
	want, err := spatialmath.NewCylinder(
		spatialmath.NewPoseFromPoint(r3.Vector{X: cupOffsetX, Y: cupOffsetY, Z: cupVisualCenterZ}),
		cupRadius, cupVisualLength, qualified("cup-xp-yp"))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.GeometriesAlmostEqual(cup, want), test.ShouldBeTrue)
}

// include_realsense adds the bracket and camera body to the render only.
func TestVisualGeometriesAddCamera(t *testing.T) {
	visual, err := visualGeometries(true, "epick")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(visual), test.ShouldEqual, 8)

	parts := byLabel(t, visual)
	_, hasBracket := parts[qualified(partCameraBracket)]
	_, hasCamera := parts[qualified(partCamera)]
	test.That(t, hasBracket, test.ShouldBeTrue)
	test.That(t, hasCamera, test.ShouldBeTrue)

	minC, _ := bounds(t, visual)
	test.That(t, minC.Y, test.ShouldAlmostEqual, cameraReachY, 0.5)
}

// Every part the planner collides against must also be drawn, at the same place
// and the same width, and must be contained within what is drawn. Collision
// geometry may only ever be shorter along Z, never wider and never longer --
// otherwise the planner is avoiding something the hardware does not have.
//
// Two parts are deliberately shorter, and both are asserted exactly so that a
// future "correction" to either one fails here:
//   - the cups, clipped at the TCP approach gap
//   - the body, clipped at the arm's flange face
func TestVisualAndCollisionAgree(t *testing.T) {
	visual, err := visualGeometries(false, "epick")
	test.That(t, err, test.ShouldBeNil)
	vis := byLabel(t, visual)

	shorter := map[string]struct{ min, max float64 }{
		qualified(partBody):    {flangePlaneZ, plateCenterZ - plateZ/2},
		qualified("cup-xp-yp"): {plateCenterZ - plateZ/2, tcpClearanceZ},
		qualified("cup-xp-yn"): {plateCenterZ - plateZ/2, tcpClearanceZ},
		qualified("cup-xn-yp"): {plateCenterZ - plateZ/2, tcpClearanceZ},
		qualified("cup-xn-yn"): {plateCenterZ - plateZ/2, tcpClearanceZ},
	}

	for _, c := range collisionGeometries(t) {
		v, ok := vis[c.Label()]
		test.That(t, ok, test.ShouldBeTrue)

		cMin, cMax := bounds(t, []spatialmath.Geometry{c})
		vMin, vMax := bounds(t, []spatialmath.Geometry{v})

		// Same footprint in X and Y, exactly.
		test.That(t, cMin.X, test.ShouldAlmostEqual, vMin.X, eps)
		test.That(t, cMax.X, test.ShouldAlmostEqual, vMax.X, eps)
		test.That(t, cMin.Y, test.ShouldAlmostEqual, vMin.Y, eps)
		test.That(t, cMax.Y, test.ShouldAlmostEqual, vMax.Y, eps)

		// Contained along Z.
		test.That(t, cMin.Z, test.ShouldBeGreaterThanOrEqualTo, vMin.Z-eps)
		test.That(t, cMax.Z, test.ShouldBeLessThanOrEqualTo, vMax.Z+eps)

		if want, clipped := shorter[c.Label()]; clipped {
			test.That(t, cMin.Z, test.ShouldAlmostEqual, want.min, eps)
			test.That(t, cMax.Z, test.ShouldAlmostEqual, want.max, eps)
			continue
		}
		// Everything else must match the render exactly.
		test.That(t, spatialmath.GeometriesAlmostEqual(c, v), test.ShouldBeTrue)
	}
}

// The collision cups must match the constants geometry.go documents them by, so
// the JSON and the Go table cannot drift apart silently.
func TestCollisionCupsMatchConstants(t *testing.T) {
	parts := byLabel(t, collisionGeometries(t))
	for _, c := range cupParts {
		cup, ok := parts[qualified(c.name)].(*spatialmath.Cylinder)
		test.That(t, ok, test.ShouldBeTrue)

		want, err := spatialmath.NewCylinder(
			spatialmath.NewPoseFromPoint(r3.Vector{X: c.x, Y: c.y, Z: cupCollisionCenterZ}),
			cupRadius, cupCollisionLength, qualified(c.name))
		test.That(t, err, test.ShouldBeNil)
		test.That(t, spatialmath.GeometriesAlmostEqual(cup, want), test.ShouldBeTrue)
	}
}

// The collision body must likewise match its constants.
func TestCollisionBodyMatchesConstants(t *testing.T) {
	body, ok := byLabel(t, collisionGeometries(t))[qualified(partBody)].(*spatialmath.Cylinder)
	test.That(t, ok, test.ShouldBeTrue)

	want, err := spatialmath.NewCylinder(
		spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: bodyCollisionCenterZ}),
		bodyRadius, bodyCollisionLength, qualified(partBody))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.GeometriesAlmostEqual(body, want), test.ShouldBeTrue)
}

// include_realsense selects what renders. Collision geometry comes from
// epick_model.json and must not move.
func TestKinematicsIgnoresRealsenseFlag(t *testing.T) {
	ctx := context.Background()
	var kinematics []referenceframe.Model
	for _, include := range []bool{false, true} {
		g := newGeometrySimGripper(t, include)
		model, err := g.Kinematics(ctx)
		test.That(t, err, test.ShouldBeNil)
		kinematics = append(kinematics, model)
	}

	withoutGeoms, err := kinematics[0].Geometries(nil)
	test.That(t, err, test.ShouldBeNil)
	withGeoms, err := kinematics[1].Geometries(nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(withGeoms.Geometries()), test.ShouldEqual, len(withoutGeoms.Geometries()))

	for i, want := range withoutGeoms.Geometries() {
		test.That(t, spatialmath.GeometriesAlmostEqual(want, withGeoms.Geometries()[i]), test.ShouldBeTrue)
	}
}

// A wrong json tag would compile and pass every geometry test while silently
// ignoring the operator's config, so decode the attribute as the app sends it.
func TestIncludeRealsenseDecodesFromJSON(t *testing.T) {
	var cfg Config
	err := json.Unmarshal([]byte(`{"host":"1.2.3.4","include_realsense":true}`), &cfg)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, cfg.IncludeRealsense, test.ShouldBeTrue)

	var simCfg SimConfig
	err = json.Unmarshal([]byte(`{"include_realsense":true}`), &simCfg)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, simCfg.IncludeRealsense, test.ShouldBeTrue)

	// Omitted means no camera: a gripper with none bolted on.
	var defaultCfg Config
	err = json.Unmarshal([]byte(`{"host":"1.2.3.4"}`), &defaultCfg)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, defaultCfg.IncludeRealsense, test.ShouldBeFalse)

	// The field must not disturb existing validation.
	_, _, err = cfg.Validate("path")
	test.That(t, err, test.ShouldBeNil)
	_, _, err = simCfg.Validate("path")
	test.That(t, err, test.ShouldBeNil)
}

// The simulated model exists to stand in for the hardware, so it must render
// the same gripper for the same configuration.
func TestSimGripperGeometriesMatchRealModel(t *testing.T) {
	ctx := context.Background()
	for _, include := range []bool{false, true} {
		sim := newGeometrySimGripper(t, include)
		simGeoms, err := sim.Geometries(ctx, nil)
		test.That(t, err, test.ShouldBeNil)

		want, err := visualGeometries(include, sim.Name().ShortName())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(simGeoms), test.ShouldEqual, len(want))

		for i := range want {
			test.That(t, spatialmath.GeometriesAlmostEqual(simGeoms[i], want[i]), test.ShouldBeTrue)
		}
	}
}

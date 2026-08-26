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

// tcpClearance is the gap the collision model must leave between its topmost
// face and the TCP at Z=0. Driving the cups into a surface is how a grab works,
// so collision geometry over that gap makes the planner refuse every approach.
const tcpClearance = 26.0

// cameraReachY is how far the RealSense reaches in -Y, from the CAD export.
// Collision geometry must stay clear of it: the camera component contributes
// its own geometry through its own frame, and duplicating it double-counts.
const cameraReachY = -107.2

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

// The body is a true cylinder, not the capsule it used to be approximated by.
// A capsule cannot describe it: RDK requires length >= 2*radius and always
// gives a capsule domed ends, so the old model covered barely half the body.
func TestBodyIsCylinderMatchingCAD(t *testing.T) {
	body, ok := byLabel(t, collisionGeometries(t))[qualified(partBody)]
	test.That(t, ok, test.ShouldBeTrue)

	cyl, isCyl := body.(*spatialmath.Cylinder)
	test.That(t, isCyl, test.ShouldBeTrue)

	want, err := spatialmath.NewCylinder(
		spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 0, Z: bodyCenterZ}),
		bodyRadius, bodyLength, qualified(partBody))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.GeometriesAlmostEqual(cyl, want), test.ShouldBeTrue)
}

// Nothing in the collision model may cross into the approach gap. This is the
// invariant that keeps every grab from registering as a collision, and it is
// why the cups are modeled shorter than they really are.
func TestCollisionModelClearsTCP(t *testing.T) {
	_, maxC := bounds(t, collisionGeometries(t))
	test.That(t, maxC.Z, test.ShouldAlmostEqual, -tcpClearance, eps)
}

// The camera is excluded from collision on purpose: the RealSense component
// supplies its own geometry through its own frame.
func TestCollisionModelExcludesCamera(t *testing.T) {
	minC, _ := bounds(t, collisionGeometries(t))
	test.That(t, minC.Y, test.ShouldBeGreaterThan, cameraReachY)
}

// The whole collision model must sit at negative Z, between the flange and the
// approach gap.
func TestCollisionModelSpansFlangeToClearance(t *testing.T) {
	minC, maxC := bounds(t, collisionGeometries(t))
	test.That(t, minC.Z, test.ShouldAlmostEqual, flangeZ, eps)
	test.That(t, maxC.Z, test.ShouldBeLessThan, 0.0)
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

// Every part the planner collides against must also be drawn, and drawn in the
// same place. The cups are the sole allowed difference: same axis and radius,
// shorter for collision. This is what keeps the two sets from drifting apart.
func TestVisualAndCollisionAgree(t *testing.T) {
	visual, err := visualGeometries(false, "epick")
	test.That(t, err, test.ShouldBeNil)
	vis := byLabel(t, visual)

	for _, c := range collisionGeometries(t) {
		v, ok := vis[c.Label()]
		test.That(t, ok, test.ShouldBeTrue)

		if cup, isCup := c.(*spatialmath.Cylinder); isCup && c.Label() != qualified(partBody) {
			// Same axis, same radius, longer in the render.
			cMin, cMax := bounds(t, []spatialmath.Geometry{cup})
			vMin, vMax := bounds(t, []spatialmath.Geometry{v})
			test.That(t, cup.Pose().Point().X, test.ShouldAlmostEqual, v.Pose().Point().X, eps)
			test.That(t, cup.Pose().Point().Y, test.ShouldAlmostEqual, v.Pose().Point().Y, eps)
			test.That(t, cMax.X-cMin.X, test.ShouldAlmostEqual, vMax.X-vMin.X, eps)
			test.That(t, cMin.Z, test.ShouldAlmostEqual, vMin.Z, eps)
			test.That(t, cMax.Z, test.ShouldBeLessThan, vMax.Z)
			continue
		}
		test.That(t, spatialmath.GeometriesAlmostEqual(c, v), test.ShouldBeTrue)
	}
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

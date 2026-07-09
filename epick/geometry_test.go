package epick

import (
	"context"
	"encoding/json"
	"math"
	"testing"

	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

// maxVisualTriangles is the budget the app's per-frame Geometries() poll is held
// to. The default mesh must stay under it; regenerate with `make decimate`.
const maxVisualTriangles = 500

// The RealSense camera is the only feature that reaches past Y=-70mm, so the Y
// minimum distinguishes the two meshes without depending on triangle counts.
const (
	plainMinY     = -65.0
	realsenseMinY = -107.2
)

// meshBounds returns the min and max corners of a mesh's triangles, in mm.
func meshBounds(t *testing.T, geoms []spatialmath.Geometry) (minC, maxC [3]float64) {
	t.Helper()
	test.That(t, len(geoms), test.ShouldEqual, 1)
	mesh, ok := geoms[0].(*spatialmath.Mesh)
	test.That(t, ok, test.ShouldBeTrue)

	for i := range minC {
		minC[i], maxC[i] = math.Inf(1), math.Inf(-1)
	}
	for _, tri := range mesh.Triangles() {
		for _, p := range tri.Points() {
			for i, v := range [3]float64{p.X, p.Y, p.Z} {
				minC[i] = math.Min(minC[i], v)
				maxC[i] = math.Max(maxC[i], v)
			}
		}
	}
	return minC, maxC
}

// newGeometrySimGripper builds a simulated gripper with a resource name, which
// Geometries() and Kinematics() both need for the geometry label.
func newGeometrySimGripper(t *testing.T, includeRealsense bool) *simGripper {
	t.Helper()
	return &simGripper{
		Named:  resource.NewName(gripper.API, "epick").AsNamed(),
		logger: logging.NewTestLogger(t),
		opMgr:  operation.NewSingleOperationManager(),
		stl:    meshSTL(includeRealsense),
	}
}

func triangleCount(t *testing.T, geoms []spatialmath.Geometry) int {
	t.Helper()
	mesh, ok := geoms[0].(*spatialmath.Mesh)
	test.That(t, ok, test.ShouldBeTrue)
	return len(mesh.Triangles())
}

func TestMeshSTLSelection(t *testing.T) {
	test.That(t, meshSTL(false), test.ShouldResemble, epickSTL)
	test.That(t, meshSTL(true), test.ShouldResemble, epickRealsenseSTL)
}

// The default mesh must exclude the camera: it is what renders on a gripper with
// no RealSense bolted on.
func TestDefaultMeshExcludesRealsense(t *testing.T) {
	geoms, err := meshGeometry(meshSTL(false), "epick")
	test.That(t, err, test.ShouldBeNil)

	minC, _ := meshBounds(t, geoms)
	test.That(t, minC[1], test.ShouldAlmostEqual, plainMinY, 0.5)
}

func TestRealsenseMeshIncludesCamera(t *testing.T) {
	geoms, err := meshGeometry(meshSTL(true), "epick")
	test.That(t, err, test.ShouldBeNil)

	minC, _ := meshBounds(t, geoms)
	test.That(t, minC[1], test.ShouldAlmostEqual, realsenseMinY, 0.5)
}

// Both meshes describe the same gripper, so every extent except the camera's Y
// reach must agree. Catches a mesh regenerated from the wrong CAD export.
func TestMeshesAgreeAwayFromCamera(t *testing.T) {
	plain, err := meshGeometry(meshSTL(false), "epick")
	test.That(t, err, test.ShouldBeNil)
	rs, err := meshGeometry(meshSTL(true), "epick")
	test.That(t, err, test.ShouldBeNil)

	plainMin, plainMax := meshBounds(t, plain)
	rsMin, rsMax := meshBounds(t, rs)

	// X and Z extents, and the Y maximum, are the gripper body itself.
	test.That(t, rsMin[0], test.ShouldAlmostEqual, plainMin[0], 0.5)
	test.That(t, rsMax[0], test.ShouldAlmostEqual, plainMax[0], 0.5)
	test.That(t, rsMin[2], test.ShouldAlmostEqual, plainMin[2], 0.5)
	test.That(t, rsMax[2], test.ShouldAlmostEqual, plainMax[2], 0.5)
	test.That(t, rsMax[1], test.ShouldAlmostEqual, plainMax[1], 0.5)

	// Only the camera extends the mesh in -Y.
	test.That(t, rsMin[1], test.ShouldBeLessThan, plainMin[1])
}

// Geometries() is polled on every frame update, so the default mesh is held to a
// triangle budget.
func TestDefaultMeshUnderTriangleBudget(t *testing.T) {
	geoms, err := meshGeometry(meshSTL(false), "epick")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, triangleCount(t, geoms), test.ShouldBeLessThanOrEqualTo, maxVisualTriangles)
}

// The STL must be authored in meters: RDK's readSTLVertex multiplies every vertex
// by 1000 with no unit check, so a millimeter export loads as a 100-meter gripper.
// After that scaling a real EPick is ~200mm across.
func TestMeshesAreInMeters(t *testing.T) {
	for name, stl := range map[string][]byte{"plain": meshSTL(false), "realsense": meshSTL(true)} {
		t.Run(name, func(t *testing.T) {
			geoms, err := meshGeometry(stl, "epick")
			test.That(t, err, test.ShouldBeNil)

			minC, maxC := meshBounds(t, geoms)
			for i := range minC {
				test.That(t, maxC[i]-minC[i], test.ShouldBeLessThan, 1000.0)
			}
			test.That(t, maxC[0]-minC[0], test.ShouldBeGreaterThan, 100.0)
		})
	}
}

// include_realsense selects a visualization mesh. Collision geometry comes from
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
	test.That(t, meshSTL(cfg.IncludeRealsense), test.ShouldResemble, epickRealsenseSTL)

	var simCfg SimConfig
	err = json.Unmarshal([]byte(`{"include_realsense":true}`), &simCfg)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, simCfg.IncludeRealsense, test.ShouldBeTrue)

	// Omitted means the plain mesh: a gripper with no camera bolted on.
	var defaultCfg Config
	err = json.Unmarshal([]byte(`{"host":"1.2.3.4"}`), &defaultCfg)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, defaultCfg.IncludeRealsense, test.ShouldBeFalse)
	test.That(t, meshSTL(defaultCfg.IncludeRealsense), test.ShouldResemble, epickSTL)

	// The new field must not disturb existing validation.
	_, _, err = cfg.Validate("path")
	test.That(t, err, test.ShouldBeNil)
	_, _, err = simCfg.Validate("path")
	test.That(t, err, test.ShouldBeNil)
}

// The simulated model exists to stand in for the hardware, so its mesh must match
// the real model's for the same configuration.
func TestSimGripperMeshMatchesRealModel(t *testing.T) {
	ctx := context.Background()
	for _, include := range []bool{false, true} {
		sim := newGeometrySimGripper(t, include)
		simGeoms, err := sim.Geometries(ctx, nil)
		test.That(t, err, test.ShouldBeNil)

		want, err := meshGeometry(meshSTL(include), sim.Name().ShortName())
		test.That(t, err, test.ShouldBeNil)

		test.That(t, triangleCount(t, simGeoms), test.ShouldEqual, triangleCount(t, want))

		simMin, simMax := meshBounds(t, simGeoms)
		wantMin, wantMax := meshBounds(t, want)
		test.That(t, simMin, test.ShouldResemble, wantMin)
		test.That(t, simMax, test.ShouldResemble, wantMax)
	}
}

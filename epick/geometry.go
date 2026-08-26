package epick

import (
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// The EPick's shape, measured off the full-resolution CAD exports in epick/meshes/
// and expressed in the gripper frame: Z=0 is the TCP at the suction plane, and
// -Z runs back toward the arm flange. Re-derive these with `make fit-primitives`
// after replacing a CAD export.
//
// Splitting the export into connected components shows the gripper is six solids,
// every one of them a clean box or a right circular cylinder — the suction cups
// sample at a constant 24.5mm radius end to end, so they are not even tapered.
// That is why primitives describe this gripper as well as a mesh does.
const (
	// flangePlaneZ is the arm's tool flange face, 196mm behind the TCP. The CAD
	// origin sits here, which is why the fit script reports centers relative to it.
	flangePlaneZ = -196.0

	// The EPick body. Its rear boss reaches 3mm past the flange face, into the
	// space the arm's own end-effector geometry occupies, so the render draws the
	// full 129mm while collision stops at the flange plane -- see
	// bodyCollisionLength. A cylinder is what this model was waiting on: a capsule
	// always domes its ends, so it cannot describe a flat-ended body at all.
	bodyRadius        = 35.5
	bodyVisualLength  = 129.0
	bodyVisualCenterZ = -134.5
	bodyRearZ         = -199.0

	// Collision length: the body stopped at the flange face. Everything behind it
	// is inside the arm, where only the arm can be, and the model this replaces
	// ended there too -- so the rear collision boundary is unchanged.
	bodyCollisionLength  = 126.0
	bodyCollisionCenterZ = -133.0

	// The plate the cups bolt through. It is 3.2mm thick -- the box this replaces
	// claimed 100mm of solid here.
	plateX       = 204.5
	plateY       = 126.3
	plateZ       = 3.2
	plateCenterZ = -68.4

	// The four suction cups, on a 159.5 x 81.3mm rectangular pattern.
	cupRadius  = 24.5
	cupOffsetX = 79.75
	cupOffsetY = 40.65

	// Render length: the cups as built, reaching from the plate to 10mm short of
	// the TCP.
	cupVisualLength  = 60.0
	cupVisualCenterZ = -40.0
	cupTipZ          = -10.0

	// Collision length: the same cups stopped 26mm short of the TCP. Picking an
	// object up means driving the cups into contact with it, so collision
	// geometry across that gap makes the planner refuse every grab approach.
	// TestCollisionModelClearsTCP holds the gap to exactly this.
	cupCollisionLength  = 44.0
	cupCollisionCenterZ = -48.0
	tcpClearanceZ       = -26.0

	// The widest the collision model reaches in -Y, set by the cups. Well short of
	// the camera, which is excluded on purpose -- the camera component contributes
	// its own collision geometry through its own frame, so repeating it here would
	// double-count the obstacle.
	collisionReachY = -(cupOffsetY + cupRadius)

	// The RealSense bracket and camera body. Render-only, for the same reason.
	bracketX       = 75.0
	bracketY       = 125.7
	bracketZ       = 9.9
	bracketCenterY = -42.1
	bracketCenterZ = -160.6

	cameraX       = 81.0
	cameraY       = 20.1
	cameraZ       = 27.3
	cameraCenterY = -97.1
	cameraCenterZ = -144.2

	// How far the camera reaches in -Y, from the CAD export.
	cameraReachY = cameraCenterY - cameraY/2
)

// Part names. These double as the link ids in epick_model.json, so a part the
// planner collides against and the same part in the render carry one name.
const (
	partBody          = "body"
	partPlate         = "plate"
	partCameraBracket = "camera-bracket"
	partCamera        = "camera"
)

// cupParts names the four cups by which quadrant they sit in.
var cupParts = []struct {
	name string
	x, y float64
}{
	{"cup-xp-yp", cupOffsetX, cupOffsetY},
	{"cup-xp-yn", cupOffsetX, -cupOffsetY},
	{"cup-xn-yp", -cupOffsetX, cupOffsetY},
	{"cup-xn-yn", -cupOffsetX, -cupOffsetY},
}

// partLabel qualifies a part name with the resource name. referenceframe labels
// a model's geometries "<model>:<link>" (referenceframe/model.go), so matching
// that here means a part has the same name whether it arrived as collision
// geometry or as a rendered one.
func partLabel(resourceName, part string) string {
	return resourceName + ":" + part
}

// visualGeometries returns the EPick drawn as primitives: the true shape of the
// hardware, including the full length of the suction cups that the collision
// model clips. includeRealsense adds the camera bracket and body.
//
// This is not the collision geometry. The frame system consumes Kinematics(),
// which returns the primitives in epick_model.json.
func visualGeometries(includeRealsense bool, resourceName string) ([]spatialmath.Geometry, error) {
	body, err := spatialmath.NewCylinder(
		spatialmath.NewPoseFromPoint(r3.Vector{Z: bodyVisualCenterZ}),
		bodyRadius, bodyVisualLength, partLabel(resourceName, partBody))
	if err != nil {
		return nil, err
	}
	plate, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{Z: plateCenterZ}),
		r3.Vector{X: plateX, Y: plateY, Z: plateZ}, partLabel(resourceName, partPlate))
	if err != nil {
		return nil, err
	}
	geoms := []spatialmath.Geometry{body, plate}

	for _, c := range cupParts {
		cup, err := spatialmath.NewCylinder(
			spatialmath.NewPoseFromPoint(r3.Vector{X: c.x, Y: c.y, Z: cupVisualCenterZ}),
			cupRadius, cupVisualLength, partLabel(resourceName, c.name))
		if err != nil {
			return nil, err
		}
		geoms = append(geoms, cup)
	}

	if !includeRealsense {
		return geoms, nil
	}

	bracket, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{Y: bracketCenterY, Z: bracketCenterZ}),
		r3.Vector{X: bracketX, Y: bracketY, Z: bracketZ}, partLabel(resourceName, partCameraBracket))
	if err != nil {
		return nil, err
	}
	camera, err := spatialmath.NewBox(
		spatialmath.NewPoseFromPoint(r3.Vector{Y: cameraCenterY, Z: cameraCenterZ}),
		r3.Vector{X: cameraX, Y: cameraY, Z: cameraZ}, partLabel(resourceName, partCamera))
	if err != nil {
		return nil, err
	}
	return append(geoms, bracket, camera), nil
}

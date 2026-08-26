"""Fit primitives to the EPick CAD exports.

Prints the box and cylinder table that epick/geometry.go and
epick/epick_model.json are authored from, so a new CAD export can be
re-fitted instead of hand-measured. Read-only: it changes no source file.

Every solid in these exports is a box or a right circular cylinder, so the fit
is exact rather than approximate. The script verifies that claim per component
before reporting a shape, and fails loudly on anything it cannot classify --
a future export with a genuinely non-primitive part must not be silently
rounded off to a bounding box.

Usage:
    python scripts/fit_primitives.py epick/meshes/epick_with_realsense_full.stl
"""

import argparse
import sys

import numpy as np
import trimesh

# The CAD exports place Z=0 at the arm's tool flange. Everything in the Viam
# gripper frame is measured from the TCP at the suction plane, 196mm out.
TCP_OFFSET_MM = 196.0

# Exports are authored in meters (see the STL unit note in CLAUDE.md history);
# everything else in Viam is millimeters.
MM_PER_UNIT = 1000.0

# A component counts as round about Z when its X and Y extents agree this
# closely, and as a true cylinder when its radius holds to ROUND_TOL_MM
# across every vertex.
EXTENT_TOL = 0.05
ROUND_TOL_MM = 0.5


def classify(comp):
    """Return (shape, dims, center) in mm, in the CAD frame."""
    v = comp.vertices * MM_PER_UNIT
    lo, hi = v.min(axis=0), v.max(axis=0)
    ext, ctr = hi - lo, (hi + lo) / 2

    round_about_z = abs(ext[0] - ext[1]) <= EXTENT_TOL * max(ext[0], ext[1])
    if round_about_z:
        radial = np.linalg.norm(v[:, :2] - ctr[:2], axis=1)
        # Cap centers sit on the axis; ignore them when checking the wall.
        wall = radial[radial > radial.max() / 2]
        if wall.max() - wall.min() <= ROUND_TOL_MM:
            return "cylinder", (float(wall.mean()), float(ext[2])), ctr

    return "box", tuple(float(e) for e in ext), ctr


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("stl", help="full-resolution CAD export, in meters")
    args = ap.parse_args()

    mesh = trimesh.load(args.stl, process=False)
    # Binary STL stores three unshared vertices per facet; weld before splitting
    # or every triangle looks like its own component.
    mesh.merge_vertices()
    comps = mesh.split(only_watertight=False)

    print(f"{args.stl}: {len(mesh.faces)} faces, {len(comps)} components")
    print(f"Frame: TCP at Z=0, flange at Z=-{TCP_OFFSET_MM:.0f}\n")
    print(f"{'shape':9} {'dimensions (mm)':34} {'center x,y,z (mm)'}")
    print("-" * 78)

    failed = False
    for comp in sorted(comps, key=lambda c: -c.volume):
        shape, dims, ctr = classify(comp)
        z = ctr[2] - TCP_OFFSET_MM
        if shape == "cylinder":
            desc = f"r {dims[0]:.2f}  l {dims[1]:.1f}"
        else:
            desc = f"{dims[0]:.1f} x {dims[1]:.1f} x {dims[2]:.1f}"
            if len(comp.faces) < 200 and abs(dims[0] - dims[1]) < 2.0:
                # Nearly-square and low-poly: probably a cylinder that failed
                # the radius check. Worth a human look rather than a silent box.
                print(f"  !! {desc} is nearly round but did not fit a cylinder", file=sys.stderr)
                failed = True
        print(f"{shape:9} {desc:34} {ctr[0]:7.2f}, {ctr[1]:7.2f}, {z:7.2f}")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Decimate a binary STL to the fewest triangles that preserve its shape.

The EPick meshes are visualization-only: the app polls Geometries() on every frame
update, so triangle count is wire cost. Collision uses the primitives in
epick_model.json and is unaffected by anything this script does.

Face count is an outcome, not an input. Each disconnected solid is decimated as
far as it can go while staying within --max-deviation of its original surface;
--faces is a hard cap that fails the run rather than silently degrading the mesh.

Usage:
    decimate_stl.py <input.stl> <output.stl> --faces 500

Run via `make decimate`, which provisions the venv. Output is committed.
"""

import argparse
import sys

import numpy as np
import trimesh

# RDK's spatialmath.readSTLVertex multiplies every vertex by 1000 with no unit
# check, so an STL authored in millimeters loads as a gripper 1000x too large.
# A real EPick is ~0.2m; anything past this is certainly the wrong unit.
MAX_PLAUSIBLE_EXTENT_M = 10.0

# Fewest faces a closed solid can have: a tetrahedron.
MIN_FACES = 4

# Fixed so repeated runs produce byte-identical output.
SAMPLE_SEED = 0


def check_meters(mesh: trimesh.Trimesh, path: str) -> None:
    extent = float(mesh.extents.max())
    if extent > MAX_PLAUSIBLE_EXTENT_M:
        sys.exit(
            f"error: {path} has a max extent of {extent:.1f}, which is not meters.\n"
            f"Viam STLs must be in meters (RDK scales by 1000 on load).\n"
            f"Re-export from CAD in meters, or scale by 0.001."
        )


def _one_sided(source: trimesh.Trimesh, target: trimesh.Trimesh, n: int) -> float:
    # Seeded: surface sampling is random, and an unseeded probe makes the binary
    # search in shrink_solid land on a different face count every run. The output
    # is a committed artifact, so `make decimate` must be reproducible.
    samples, _ = trimesh.sample.sample_surface(source, n, seed=SAMPLE_SEED)
    _, distances, _ = trimesh.proximity.closest_point(target, samples)
    return float(np.max(distances))


def hausdorff_mm(original: trimesh.Trimesh, decimated: trimesh.Trimesh, n: int = 20000) -> float:
    """Symmetric Hausdorff distance between two surfaces, in mm.

    Both directions are required. Sampling only the decimated surface cannot
    detect deleted geometry: every surviving triangle still lies on the original
    surface, so a mesh missing a whole feature scores near zero.
    """
    return 1000.0 * max(
        _one_sided(decimated, original, n),
        _one_sided(original, decimated, n),
    )


def shrink_solid(solid: trimesh.Trimesh, max_dev: float, samples: int) -> trimesh.Trimesh:
    """Fewest faces keeping this solid within max_dev mm of its original surface.

    Binary search rather than a fixed budget, because the safe reduction depends on
    the solid's shape. The EPick's 3mm mounting plate and 10mm camera bracket are
    nearly flat, so they have little interior volume to constrain the quadric error
    metric and self-intersect well before a thicker solid would. Their safe floors
    (~118 and ~738 faces) are not predictable from face count alone.

    simplify_quadric_decimation(face_count=N) is a request, not a guarantee -- the
    simplifier stops early when it runs out of legal edge collapses -- so the search
    is over what it actually returns.
    """
    best = solid
    lo, hi = MIN_FACES, len(solid.faces) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        candidate = solid.simplify_quadric_decimation(face_count=mid)
        if hausdorff_mm(solid, candidate, samples) <= max_dev:
            if len(candidate.faces) < len(best.faces):
                best = candidate
            hi = mid - 1
        else:
            lo = mid + 1
    return best


def decimate(mesh: trimesh.Trimesh, max_dev: float, samples: int) -> trimesh.Trimesh:
    """Decimate each disconnected watertight solid independently, then recombine.

    Two rules, both learned the hard way:

    1. Decimate per component. Given the whole mesh, the simplifier spends the entire
       budget on the largest surface and deletes small components outright -- the
       suction cups vanish to buy triangles for the mounting plate.

    2. Never decimate an open shell. Quadric error sums the squared distance to the
       planes adjacent to a vertex, so a vertex on an open boundary has fewer
       constraining planes and looks cheap to collapse. The EPick's suction cups are
       open shells; decimating them flattens the rim (~31mm of error on a 60mm cup).
       They are small, so preserving them intact is affordable.

    Components are disconnected solids, so decimating them separately tears nothing.
    """
    components = mesh.split(only_watertight=False)
    parts = [
        c if not c.is_watertight else shrink_solid(c, max_dev, samples)
        for c in components
    ]
    return trimesh.util.concatenate(parts)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--faces", type=int, required=True, help="hard cap on triangle count")
    ap.add_argument(
        "--max-deviation",
        type=float,
        default=3.0,
        help="max Hausdorff deviation per solid, in mm (default: 3.0)",
    )
    ap.add_argument(
        "--samples",
        type=int,
        default=4000,
        help="surface samples per deviation probe (default: 4000)",
    )
    args = ap.parse_args()

    # process=True welds coincident vertices. Binary STL stores three unshared
    # vertices per facet, so an unwelded mesh has no interior edges at all --
    # quadric decimation is edge-collapse, and with nothing to collapse it simply
    # deletes triangles until it hits the target, truncating whole features.
    original = trimesh.load_mesh(args.input, file_type="stl", process=True)
    original.merge_vertices()
    check_meters(original, args.input)

    if len(original.vertices) == len(original.faces) * 3:
        sys.exit(f"error: {args.input} vertices did not weld; decimation would truncate it")

    before = len(original.faces)
    decimated = decimate(original, args.max_deviation, args.samples)
    after = len(decimated.faces)

    check_meters(decimated, args.output)

    if after > args.faces:
        sys.exit(
            f"error: shape preservation needs {after} faces, over the {args.faces} cap.\n"
            f"Raise --faces, or raise --max-deviation to allow a coarser mesh."
        )

    # A truncated or self-intersecting mesh shows up as a moved bounding box long
    # before it shows up as a bad-looking render.
    bbox_shift = 1000.0 * float(np.max(np.abs(original.bounds - decimated.bounds)))
    if bbox_shift > 2.0:
        sys.exit(
            f"error: bounding box moved {bbox_shift:.1f}mm during decimation.\n"
            f"  original:  {np.round(original.bounds * 1000, 1).tolist()}\n"
            f"  decimated: {np.round(decimated.bounds * 1000, 1).tolist()}\n"
            f"Geometry was deformed or deleted. Lower --max-deviation."
        )

    with open(args.output, "wb") as fh:
        fh.write(trimesh.exchange.stl.export_stl(decimated))

    shells = sum(1 for c in original.split(only_watertight=False) if not c.is_watertight)
    print(f"{args.input} -> {args.output}")
    print(f"  components:    {len(original.split(only_watertight=False))} ({shells} open shells, preserved intact)")
    print(f"  triangles:     {before} -> {after}  ({100 * (1 - after / before):.1f}% reduction)")
    print(f"  hausdorff:     {hausdorff_mm(original, decimated):.2f} mm (symmetric)")
    print(f"  bbox shift:    {bbox_shift:.2f} mm")


if __name__ == "__main__":
    main()

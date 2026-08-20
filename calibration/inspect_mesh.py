#!/usr/bin/env python3
"""
inspect_mesh.py -- report what an STL's origin and axes actually are.

WHY THIS EXISTS
    An STL records triangle coordinates and nothing else. Where its origin sits,
    what units it is in, and which axis a part revolves about are all export-time
    intent that the format simply does not carry. Get any of them wrong and the
    object renders plausibly and is wrong by a fixed amount -- the worst failure
    available to a validation rig.

    Everything here is measured from the geometry, so it can confirm (or refute)
    what an export was supposed to produce.

WHAT IT CHECKS
    * bounding box, size, and where the origin sits relative to it
    * which axis the part revolves about, if any (radial-spread test)
    * how far the origin is from that axis -- the number that decides whether
      rotating the part about its axis moves its origin
    * a verdict against the 'P2D2 - Making a Site' rotor convention:
      X down the spin axis, origin ON the axis at the axial midpoint

USAGE
    python calibration/inspect_mesh.py Rendering/RotorFiles/Rotor.stl
    python calibration/inspect_mesh.py <mesh> --scale 25.4     # file is in inches
"""

import argparse
import math
import os
import struct
import sys


def load_vertices(path):
    """Returns a flat list of (x, y, z). Handles binary and ASCII STL."""
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        header = f.read(84)
        if len(header) < 84:
            sys.exit(f"{path}: too short to be an STL")
        count = struct.unpack("<I", header[80:84])[0]

        if size == 84 + count * 50:                       # binary
            out = []
            for _ in range(count):
                tri = f.read(50)
                for v in range(3):
                    out.append(struct.unpack_from("<fff", tri, 12 + v * 12))
            return out, count, "binary"

    out = []                                              # ASCII fallback
    with open(path, "r", errors="ignore") as f:
        for line in f:
            parts = line.split()
            if parts and parts[0] == "vertex":
                out.append(tuple(float(p) for p in parts[1:4]))
    return out, len(out) // 3, "ascii"


def revolution_axis(verts, bounds):
    """Which axis the part revolves about, by radial-spread test.

    Slicing perpendicular to the true axis of a body of revolution leaves points
    at a near-constant radius from a common centre. The axis with the lowest
    spread wins. Blades and grooves break perfect symmetry, so the winner is
    rarely near zero -- what matters is that it is clearly lowest.
    """
    scores = {}
    for axis, (a, b) in (("X", (1, 2)), ("Y", (0, 2)), ("Z", (0, 1))):
        ai = "XYZ".index(axis)
        lo, hi = bounds[ai]
        span = hi - lo
        if span <= 0:
            continue
        spreads = []
        for k in range(3, 8):                             # interior slices only
            centre = lo + span * k / 10.0
            sl = [v for v in verts if abs(v[ai] - centre) < span * 0.01]
            if len(sl) < 50:
                continue
            ca = sum(v[a] for v in sl) / len(sl)
            cb = sum(v[b] for v in sl) / len(sl)
            radii = [math.hypot(v[a] - ca, v[b] - cb) for v in sl]
            mean = sum(radii) / len(radii)
            if mean > 1e-9:
                spreads.append((max(radii) - min(radii)) / mean)
        if spreads:
            scores[axis] = sum(spreads) / len(spreads)
    return scores


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mesh")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="multiply file units by this to get millimetres "
                         "(25.4 if the file is in inches, 10 if centimetres)")
    args = ap.parse_args()

    if not os.path.exists(args.mesh):
        sys.exit(f"{args.mesh}: not found")

    verts, tris, fmt = load_vertices(args.mesh)
    if not verts:
        sys.exit(f"{args.mesh}: no vertices found")

    s = args.scale
    bounds = [(min(v[a] for v in verts), max(v[a] for v in verts)) for a in range(3)]
    scaled = [(lo * s, hi * s) for lo, hi in bounds]

    print(f"{args.mesh}")
    print(f"  {fmt}, {tris:,} triangles" + (f", scaled x{s:g} -> mm" if s != 1.0 else ""))
    print()
    print("  axis        min        max       size     centre")
    for i, a in enumerate("XYZ"):
        lo, hi = scaled[i]
        print(f"    {a}  {lo:9.1f}  {hi:9.1f}  {hi-lo:9.1f}  {(lo+hi)/2:9.1f}")

    inside = all(lo <= 0 <= hi for lo, hi in scaled)
    centre_dist = math.sqrt(sum(((lo + hi) / 2) ** 2 for lo, hi in scaled))
    print(f"\n  origin inside the bounding box : {'yes' if inside else 'NO'}")
    print(f"  origin to bbox centre          : {centre_dist:.1f} mm")

    scores = revolution_axis(verts, bounds)
    if not scores:
        print("\n  (not enough geometry to test for an axis of revolution)")
        return

    print("\n  radial spread per axis (lowest = axis of revolution):")
    for axis, val in sorted(scores.items(), key=lambda kv: kv[1]):
        print(f"    {axis}  {val:.3f}")

    axis = min(scores, key=scores.get)
    ai = "XYZ".index(axis)
    others = [i for i in range(3) if i != ai]
    off = math.sqrt(sum(((scaled[i][0] + scaled[i][1]) / 2) ** 2 for i in others))

    print(f"\n  axis of revolution     : {axis}")
    print(f"  origin off that axis by: {off:.1f} mm")

    # The decisive property. A rotation about the axis fixes the points ON it,
    # so an origin sitting on the axis is immune to any rotation about it --
    # which is what makes marker mount clocking (which groove) irrelevant.
    print("\n  --- 'Making a Site' rotor convention ---")
    ok_axis = axis == "X"
    ok_on_axis = off < 1.0
    axial_centre = (scaled[ai][0] + scaled[ai][1]) / 2
    ok_mid = abs(axial_centre) < max(1.0, 0.01 * (scaled[ai][1] - scaled[ai][0]))

    print(f"  spin axis is X             : {'yes' if ok_axis else 'NO (it is ' + axis + ')'}")
    print(f"  origin lies ON the axis    : {'yes' if ok_on_axis else f'NO (off by {off:.0f} mm)'}")
    print(f"  origin at axial midpoint   : {'yes' if ok_mid else f'NO (midpoint at {axial_centre:.0f} mm)'}")

    if ok_axis and ok_on_axis and ok_mid:
        print("\n  COMPLIANT. Because the origin lies on the spin axis, rotating the part")
        print("  about that axis leaves the origin fixed -- so which groove a marker")
        print("  mount occupies does not affect the measured origin position.")
    else:
        print("\n  NOT COMPLIANT.")
        if not ok_on_axis:
            for pitch in (5, 10, 15):
                d = 2 * off * math.sin(math.radians(pitch / 2))
                print(f"    origin is off-axis, so one groove pitch of {pitch:2d} deg "
                      f"swings it {d:.0f} mm")
            print("    -> which groove each mount occupies MUST be recorded and modelled.")


if __name__ == "__main__":
    main()

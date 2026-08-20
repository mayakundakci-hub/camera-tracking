#!/usr/bin/env python3
"""
recentre_mesh.py -- move an STL's origin onto its own axis of revolution.

WHY THIS EXISTS
    An STL carries triangle coordinates and nothing else. A CAD model can have a
    perfectly good coordinate system and still export about the GLOBAL origin,
    because the export step writes whatever frame it was pointed at and most
    dialogs default to the world one. The result is a part whose geometry is
    right and whose datum is metres away -- which renders plausibly and is wrong
    by a fixed amount.

    Fixing the export is better than fixing the file. This exists because that is
    not always available, and because a mesh translated by a measured amount is
    an honest artefact as long as the amount is measured, recorded and
    reproducible. All three are the point of this script.

WHAT IT DOES
    Finds the axis of revolution, finds where that axis actually sits, and
    rewrites every vertex so the origin lands ON the axis at the axial midpoint
    -- the 'P2D2 - Making a Site' convention that inspect_mesh.py checks.

    It is a pure translation. Facet normals are directions and a translation
    cannot change them, so they are copied through untouched; the attribute
    bytes are copied too. Nothing is rescaled and no axis is permuted, so if the
    spin axis is not already X this will say so and stop rather than quietly
    reorienting a part.

WHY IT DOES NOT OVERWRITE
    The input is what CAD produced and is the only record of that. Writing
    beside it keeps the two distinguishable, so 'the export is still wrong' stays
    visible instead of being papered over -- and re-running one command after the
    next export restores the fix.

USAGE
    python calibration/recentre_mesh.py in.stl out.stl --scale 25.4
    python calibration/recentre_mesh.py in.stl out.stl --scale 25.4 --dry-run

    --scale affects REPORTING ONLY. The file is written in its own units, so a
    mesh_scale in the object config keeps meaning exactly what it meant before.
"""

import argparse
import math
import os
import struct
import sys


# ---------------------------------------------------------------
# io
# ---------------------------------------------------------------

def read_binary_stl(path):
    """Returns (header, [(normal, v0, v1, v2, attr), ...]). Binary only.

    ASCII is refused rather than handled: this writes binary, and silently
    changing a file's encoding while claiming to have translated it is the kind
    of surprise that shows up three steps later as a tool that cannot read it.
    """
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        header = f.read(80)
        raw = f.read(4)
        if len(raw) < 4:
            sys.exit(f"{path}: too short to be an STL")
        count = struct.unpack("<I", raw)[0]
        if size != 84 + count * 50:
            sys.exit(f"{path}: not a binary STL (size {size:,} does not match a "
                     f"{count:,}-triangle binary file). Re-export as binary STL.")
        tris = []
        for _ in range(count):
            t = f.read(50)
            n = struct.unpack_from("<fff", t, 0)
            v0 = struct.unpack_from("<fff", t, 12)
            v1 = struct.unpack_from("<fff", t, 24)
            v2 = struct.unpack_from("<fff", t, 36)
            attr = struct.unpack_from("<H", t, 48)[0]
            tris.append((n, v0, v1, v2, attr))
    return header, tris


def write_binary_stl(path, header, tris):
    with open(path, "wb") as f:
        f.write(header[:80].ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        for n, v0, v1, v2, attr in tris:
            f.write(struct.pack("<fff", *n))
            f.write(struct.pack("<fff", *v0))
            f.write(struct.pack("<fff", *v1))
            f.write(struct.pack("<fff", *v2))
            f.write(struct.pack("<H", attr))


# ---------------------------------------------------------------
# geometry
# ---------------------------------------------------------------

def solve3(A, b):
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-14:
            return None
        M[c], M[p] = M[p], M[c]
        for r in range(3):
            if r == c:
                continue
            fac = M[r][c] / M[c][c]
            for k in range(c, 4):
                M[r][k] -= fac * M[c][k]
    return [M[i][3] / M[i][i] for i in range(3)]


def fit_circle(pts):
    """Kasa algebraic circle fit. Returns (cx, cy, r)."""
    n = len(pts)
    Sx = sum(p[0] for p in pts); Sy = sum(p[1] for p in pts)
    Sxx = sum(p[0] * p[0] for p in pts); Syy = sum(p[1] * p[1] for p in pts)
    Sxy = sum(p[0] * p[1] for p in pts)
    Sz = sum(p[0] ** 2 + p[1] ** 2 for p in pts)
    Sxz = sum(p[0] * (p[0] ** 2 + p[1] ** 2) for p in pts)
    Syz = sum(p[1] * (p[0] ** 2 + p[1] ** 2) for p in pts)
    sol = solve3([[Sxx, Sxy, Sx], [Sxy, Syy, Sy], [Sx, Sy, n]], [-Sxz, -Syz, -Sz])
    if sol is None:
        return None
    D, E, F = sol
    cx, cy = -D / 2.0, -E / 2.0
    r2 = cx * cx + cy * cy - F
    return (cx, cy, math.sqrt(r2)) if r2 > 0 else None


def revolution_axis(verts, bounds):
    """Same radial-spread test inspect_mesh.py uses, so the two cannot disagree."""
    scores = {}
    for axis, (a, b) in (("X", (1, 2)), ("Y", (0, 2)), ("Z", (0, 1))):
        ai = "XYZ".index(axis)
        lo, hi = bounds[ai]
        span = hi - lo
        if span <= 0:
            continue
        spreads = []
        for k in range(3, 8):
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


def locate_axis(verts, ai, scale):
    """Where the axis of revolution sits, in the two directions across it.

    The bounding-box centre is the obvious estimate and is right whenever the
    part is a full body of revolution, since the box is then the circumscribing
    square of the outer circle. It is NOT right for a sector, or for a disc
    carrying something that sticks out on one side.

    So the outer rim is fitted directly and the two are compared. Agreement means
    either would have done and the number is trustworthy; disagreement is
    reported rather than resolved silently, because which one is right then
    depends on what the part actually is.
    """
    a, b = [i for i in range(3) if i != ai]
    box = ((min(v[a] for v in verts) + max(v[a] for v in verts)) / 2.0,
           (min(v[b] for v in verts) + max(v[b] for v in verts)) / 2.0)

    # Iterate: radii about the current guess, keep the outermost shell, refit.
    # Converges in a couple of passes because each refit only moves the centre
    # by the bias the previous shell selection introduced.
    c = box
    rim = []
    for _ in range(6):
        radii = [(math.hypot(v[a] - c[0], v[b] - c[1]), v) for v in verts]
        rmax = max(r for r, _ in radii)
        rim = [(v[a], v[b]) for r, v in radii if r > 0.995 * rmax]
        if len(rim) < 32:
            break
        f = fit_circle(rim)
        if not f:
            break
        c = (f[0], f[1])

    fitted = fit_circle(rim) if len(rim) >= 32 else None
    if fitted:
        cx, cy, r = fitted
        resid = sum(abs(math.hypot(p[0] - cx, p[1] - cy) - r) for p in rim) / len(rim)
        bins = {int((math.degrees(math.atan2(p[1] - cy, p[0] - cx)) + 180) // 10) for p in rim}
        arc = len(bins) / 36.0
        print(f"  outer rim fit      : centre ({cx*scale:.1f}, {cy*scale:.1f}) mm, "
              f"radius {r*scale:.1f} mm")
        print(f"                       residual {resid*scale:.3f} mm over {arc*100:.0f}% of a turn"
              f" ({len(rim):,} vertices)")
        print(f"  bounding-box centre: ({box[0]*scale:.1f}, {box[1]*scale:.1f}) mm")
        disagree = math.hypot(cx - box[0], cy - box[1]) * scale
        print(f"  the two disagree by: {disagree:.3f} mm")
        if arc < 0.9:
            print("  NOTE: the rim covers less than a full turn, so this is a sector or has a")
            print("        one-sided feature. The fit is the better estimate; the box is not.")
        elif disagree > 1.0:
            print("  NOTE: they disagree by more than a millimetre. Using the fit.")
        return (cx, cy)

    print(f"  bounding-box centre: ({box[0]*scale:.1f}, {box[1]*scale:.1f}) mm"
          "   (no rim fit possible; using it)")
    return box


# ---------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("infile")
    ap.add_argument("outfile")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="file units -> mm, for REPORTING only (25.4 if inches)")
    ap.add_argument("--dry-run", action="store_true",
                    help="measure and report the translation; write nothing")
    args = ap.parse_args()

    if not os.path.exists(args.infile):
        sys.exit(f"{args.infile}: not found")
    if os.path.abspath(args.infile) == os.path.abspath(args.outfile):
        sys.exit("refusing to overwrite the input -- it is the only record of what CAD "
                 "produced, and the difference between the two is what shows the export "
                 "is still wrong")

    header, tris = read_binary_stl(args.infile)
    verts = [v for t in tris for v in t[1:4]]
    s = args.scale

    print(f"{args.infile}")
    print(f"  binary, {len(tris):,} triangles"
          + (f", reported at x{s:g} -> mm" if s != 1.0 else ""))
    print()

    bounds = [(min(v[a] for v in verts), max(v[a] for v in verts)) for a in range(3)]

    scores = revolution_axis(verts, bounds)
    if not scores:
        sys.exit("  not enough geometry to find an axis of revolution")
    axis = min(scores, key=scores.get)
    ai = "XYZ".index(axis)
    ranked = " < ".join(f"{k} {v:.3f}" for k, v in sorted(scores.items(), key=lambda kv: kv[1]))
    print(f"  axis of revolution : {axis}   (radial spread {ranked})")

    if axis != "X":
        sys.exit(f"\n  STOP. The convention wants the spin axis along X and this part revolves\n"
                 f"  about {axis}. That is a REORIENTATION, not a translation, and doing it here\n"
                 f"  would silently decide a handedness the drawing should decide. Re-export\n"
                 f"  with the part turned, then run this.")

    cross = locate_axis(verts, ai, s)
    axial = (bounds[ai][0] + bounds[ai][1]) / 2.0

    shift = [0.0, 0.0, 0.0]
    shift[ai] = -axial
    others = [i for i in range(3) if i != ai]
    shift[others[0]] = -cross[0]
    shift[others[1]] = -cross[1]

    off = math.hypot(cross[0], cross[1])
    print()
    print(f"  origin is off the axis by : {off*s:.1f} mm")
    print(f"  axial midpoint sits at    : {axial*s:.1f} mm")
    print(f"  TRANSLATION TO APPLY      : ({shift[0]:.6f}, {shift[1]:.6f}, {shift[2]:.6f}) "
          f"file units")
    print(f"                              ({shift[0]*s:.2f}, {shift[1]*s:.2f}, {shift[2]*s:.2f}) mm")

    if args.dry_run:
        print("\n  --dry-run: nothing written.")
        return

    moved = []
    for n, v0, v1, v2, attr in tris:
        # Normals are directions. A translation cannot change one, so they are
        # carried through rather than recomputed -- recomputing would silently
        # re-derive facet winding and is a different operation from this one.
        moved.append((n,
                      tuple(v0[k] + shift[k] for k in range(3)),
                      tuple(v1[k] + shift[k] for k in range(3)),
                      tuple(v2[k] + shift[k] for k in range(3)),
                      attr))

    # Provenance in the 80-byte header, so it travels with the file rather than
    # living only in a commit message. Deliberately not starting with "solid",
    # which some readers take as a marker of an ASCII file.
    note = (f"recentred by camera-tracking/calibration/recentre_mesh.py from "
            f"{os.path.basename(args.infile)} by "
            f"({shift[0]:.4f},{shift[1]:.4f},{shift[2]:.4f}) file units")
    write_binary_stl(args.outfile, note.encode("ascii", "replace")[:80], moved)

    # Verify what was written rather than what was intended. Reading it back is
    # the only thing that proves the float32 round-trip landed where the
    # arithmetic said it would.
    _, check = read_binary_stl(args.outfile)
    cv = [v for t in check for v in t[1:4]]
    cb = [(min(v[a] for v in cv), max(v[a] for v in cv)) for a in range(3)]
    ccross = locate_axis(cv, ai, s)
    coff = math.hypot(ccross[0], ccross[1]) * s
    cmid = (cb[ai][0] + cb[ai][1]) / 2.0 * s

    print(f"\n  wrote {args.outfile}")
    print(f"  re-measured: origin off axis {coff:.3f} mm, axial midpoint {cmid:.3f} mm")
    ok = coff < 1.0 and abs(cmid) < max(1.0, 0.01 * (cb[ai][1] - cb[ai][0]) * s)
    print("  " + ("COMPLIANT." if ok else "STILL NOT COMPLIANT -- do not use this file."))
    if ok:
        print("  Confirm independently:")
        print(f"    python calibration/inspect_mesh.py {args.outfile}"
              + (f" --scale {s:g}" if s != 1.0 else ""))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

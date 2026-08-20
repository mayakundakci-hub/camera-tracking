#!/usr/bin/env python3
"""
merge_urdf_subtree.py -- bake a URDF link subtree into one STL at its home pose.

WHY
    A scene object takes ONE mesh, but a hand (or any assembly) is usually a
    chain of links with separate meshes. Rendering one of them shows a fragment.

    The alternative is importing it as an articulated object, but the viewer
    holds one robot description at a time, and in this project the P2D2 cell
    already occupies it. When the assembly's joints do not move during a session
    -- as the tracked hand's do not -- baking the chain at its home pose loses
    nothing.

CORRECTNESS
    Joint origins are composed with their ROTATIONS, not merely summed. This
    matters: in p2d2.urdf, Left_joint_6_to_Left_moat carries a non-zero rpy, and
    the 871 mm offset that follows it lands in X rather than Y. Summing the
    translations alone puts the hand 871 mm from where it belongs.

    Each visual's own <origin> is composed too, and <mesh scale> is applied.

USAGE
    python calibration/merge_urdf_subtree.py Rendering/p2d2.urdf Left_hand_link_base \\
        --out Rendering/Hand/P2D2_hand_assembled.STL

    The output keeps the ROOT link's frame as its origin, which is the frame a
    marker mount would be measured against.
"""

import argparse
import math
import os
import struct
import sys
import xml.etree.ElementTree as ET


def rpy_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr]]


def mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_apply(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def parse_origin(el):
    if el is None:
        return [0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    xyz = [float(v) for v in el.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in el.get("rpy", "0 0 0").split()]
    return xyz, rpy_matrix(*rpy)


def read_binary_stl(path):
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        count = struct.unpack("<I", f.read(84)[80:84])[0]
        if size != 84 + count * 50:
            sys.exit(f"{path}: not a binary STL (ASCII input is not supported)")
        out = []
        for _ in range(count):
            d = f.read(50)
            out.append((struct.unpack_from("<fff", d, 0),
                        struct.unpack_from("<fff", d, 12),
                        struct.unpack_from("<fff", d, 24),
                        struct.unpack_from("<fff", d, 36)))
        return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("urdf")
    ap.add_argument("root_link", help="link whose subtree is merged; its frame becomes the origin")
    ap.add_argument("--out", required=True)
    ap.add_argument("--mesh-root", default=None,
                    help="directory URDF mesh paths are relative to (default: the URDF's own dir)")
    args = ap.parse_args()

    mesh_root = args.mesh_root or os.path.dirname(args.urdf)
    tree = ET.parse(args.urdf).getroot()

    links = {l.get("name"): l for l in tree.findall("link")}
    if args.root_link not in links:
        sys.exit(f"{args.urdf}: no link named '{args.root_link}'")

    children = {}
    for j in tree.findall("joint"):
        xyz, R = parse_origin(j.find("origin"))
        children.setdefault(j.find("parent").get("link"), []).append(
            (j.find("child").get("link"), xyz, R, j.get("name")))

    merged = []
    print(f"merging subtree of '{args.root_link}' from {args.urdf}\n")

    # Depth-first, carrying the accumulated pose. Home pose throughout: joint
    # VALUES are all zero, so only the fixed joint origins contribute.
    stack = [(args.root_link, [0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])]
    while stack:
        name, pos, R = stack.pop()

        for vis in links[name].findall("visual"):
            geom = vis.find("geometry/mesh")
            if geom is None:
                continue
            vxyz, vR = parse_origin(vis.find("origin"))
            vpos = [pos[i] + mat_apply(R, vxyz)[i] for i in range(3)]
            vRot = mat_mul(R, vR)

            scale = [1.0, 1.0, 1.0]
            if geom.get("scale"):
                scale = [float(s) for s in geom.get("scale").split()]

            rel = geom.get("filename").replace("\\", "/")
            path = os.path.join(mesh_root, rel)
            if not os.path.exists(path):
                print(f"  SKIP {rel}  (not found)")
                continue

            tris = read_binary_stl(path)
            for n, a, b, c in tris:
                verts = []
                for v in (a, b, c):
                    sv = [v[i] * scale[i] for i in range(3)]
                    rv = mat_apply(vRot, sv)
                    verts.append(tuple(vpos[i] + rv[i] for i in range(3)))
                merged.append((tuple(mat_apply(vRot, n)), *verts))

            print(f"  {name:28s} {len(tris):7,} tris   at "
                  f"[{vpos[0]:7.1f},{vpos[1]:7.1f},{vpos[2]:7.1f}]")

        for child, xyz, jR, _ in children.get(name, []):
            cpos = [pos[i] + mat_apply(R, xyz)[i] for i in range(3)]
            stack.append((child, cpos, mat_mul(R, jR)))

    if not merged:
        sys.exit("no meshes found in that subtree")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "wb") as f:
        f.write(f"merged from {os.path.basename(args.urdf)}:{args.root_link}"
                .encode()[:80].ljust(80, b"\0"))
        f.write(struct.pack("<I", len(merged)))
        for n, a, b, c in merged:
            f.write(struct.pack("<fff", *n))
            for v in (a, b, c):
                f.write(struct.pack("<fff", *v))
            f.write(struct.pack("<H", 0))

    print(f"\nwrote {args.out}  ({len(merged):,} triangles, {os.path.getsize(args.out):,} bytes)")


if __name__ == "__main__":
    main()

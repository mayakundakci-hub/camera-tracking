#!/usr/bin/env python3
"""Cut one link's subtree out of a URDF and write it as a standalone URDF.

Unlike calibration/merge_urdf_subtree.py, which bakes a subtree into a single STL at
one pose, this keeps the joints -- so the result is still articulated and can be posed
by forward kinematics.

    python scripts/extract_urdf_subtree.py Rendering/p2d2.urdf Left_hand_link_base \
        --out Rendering/p2d2_hand_left.urdf

The named link becomes the new root: the joint that attached it to its old parent is
dropped, which is the whole point -- the subtree stops being positioned by the rest of
the cell and starts at the origin of its own frame.

Mesh filenames are copied through UNCHANGED, so the output must sit in the same
directory as the input for its relative paths to keep resolving.
"""
import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.dom import minidom


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf", type=Path)
    ap.add_argument("root_link")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--name", default=None, help="robot name for the output")
    ap.add_argument("--colorize", action="store_true",
                    help="give every link a distinct visual colour, so which link is which "
                         "can be read straight off the render. Debug aid: it rewrites the "
                         "<material><color> of each visual and changes nothing geometric.")
    args = ap.parse_args()

    if not args.urdf.exists():
        raise SystemExit(f"input not found: {args.urdf}")

    tree = ET.parse(args.urdf)
    robot = tree.getroot()

    links = {l.get("name"): l for l in robot.findall("link")}
    if args.root_link not in links:
        raise SystemExit(f"link '{args.root_link}' not in {args.urdf}. "
                         f"Known: {', '.join(sorted(links))}")

    joints = robot.findall("joint")
    children_of = {}
    for j in joints:
        children_of.setdefault(j.find("parent").get("link"), []).append(j)

    # Walk down from the requested link.
    kept_links, kept_joints = [args.root_link], []
    frontier = [args.root_link]
    while frontier:
        cur = frontier.pop(0)
        for j in children_of.get(cur, []):
            child = j.find("child").get("link")
            kept_joints.append(j)
            if child not in kept_links:
                kept_links.append(child)
                frontier.append(child)

    out = ET.Element("robot", {"name": args.name or f"{args.root_link}_subtree"})
    for name in kept_links:
        out.append(links[name])
    for j in kept_joints:
        out.append(j)

    palette = [
        ("red",     "1 0 0 1"),
        ("green",   "0 0.8 0.2 1"),
        ("blue",    "0.2 0.45 1 1"),
        ("yellow",  "1 0.85 0 1"),
        ("magenta", "1 0 0.85 1"),
        ("cyan",    "0 0.9 1 1"),
        ("orange",  "1 0.5 0 1"),
        ("white",   "1 1 1 1"),
    ]
    if args.colorize:
        print("  colours (link -> what you see on screen):")
        for i, name in enumerate(kept_links):
            cname, rgba = palette[i % len(palette)]
            link = links[name]
            vis = link.find("visual")
            if vis is None:
                print(f"    {name:<24} (no visual to colour)")
                continue
            mat = vis.find("material")
            if mat is None:
                mat = ET.SubElement(vis, "material")
            # A NON-EMPTY name matters. The SolidWorks exporter writes
            # <material name="">, and a URDF parser is entitled to read an unnamed
            # material as a reference into a global material table rather than as an
            # inline definition -- in which case the colour beside it is dropped and
            # the link falls back to the viewer's uniform grey. That is consistent
            # with p2d2.urdf's own hand, which asks for blue and has always drawn grey.
            mat.set("name", f"{name}_{cname}")
            col = mat.find("color")
            if col is None:
                col = ET.SubElement(mat, "color")
            col.set("rgba", rgba)
            print(f"    {name:<24} -> {cname}")

    xml = minidom.parseString(ET.tostring(out)).toprettyxml(indent="  ")
    xml = "\n".join(line for line in xml.splitlines() if line.strip())
    args.out.write_text(xml, encoding="utf-8")

    dropped = [j.get("name") for j in joints
               if j.find("child").get("link") == args.root_link]
    print(f"wrote {args.out}")
    print(f"  root link : {args.root_link}")
    print(f"  links     : {len(kept_links)}  ({', '.join(kept_links)})")
    print(f"  joints    : {len(kept_joints)}")
    if dropped:
        print(f"  dropped the joint that attached it upward: {', '.join(dropped)}")
    if args.out.parent.resolve() != args.urdf.parent.resolve():
        print("  WARNING: output is in a different directory from the input, and mesh "
              "filenames were copied unchanged -- they will not resolve.", file=sys.stderr)


if __name__ == "__main__":
    main()

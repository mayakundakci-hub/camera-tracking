#!/usr/bin/env python3
"""Offline check of the scene manifest and its object files.

Mirrors the loader's structural rules -- unique placement ids, resolvable
references, meshes and URDFs that exist -- so a typo is caught in a second
rather than after a rebuild. It does not check geometry; the backend's
[construct] and [joint] lines are what judge the numbers.

    python scripts/check_scene.py [config/scene.json]
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

SOURCES = {"optitrack", "expected_pose", "joint_state", "static",
           "fused", "projected", "constructed"}

# Keys each source requires beyond the common set.
REQUIRED = {
    "optitrack": ["asset_id"],
    "expected_pose": ["topic"],
    "joint_state": ["urdf", "topic"],
    "static": [],
    "fused": ["inputs"],
    "projected": ["measured", "joint"],
    "constructed": ["inputs", "construction"],
}

CONSTRUCTION_KEYS = {"normal_axis", "normal_in_part", "mount_a", "mount_b",
                     "expect_normal_in_parent"}
JOINT_KEYS = {"zero_pose", "axis_point", "axis", "lower_deg", "upper_deg",
              "reported_arm", "reported_index"}


class Report:
    def __init__(self):
        self.errors = []
        self.warnings = []

    def error(self, where, msg):
        self.errors.append(f"{where}: {msg}")

    def warn(self, where, msg):
        self.warnings.append(f"{where}: {msg}")


def load(path, rep):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except FileNotFoundError:
        rep.error(str(path), "file not found")
    except json.JSONDecodeError as exc:
        rep.error(str(path), f"invalid JSON -- {exc}")
    return None


def check_unknown(obj, known, where, what, rep):
    for key in obj:
        if key.startswith("_"):
            continue
        if key not in known:
            rep.error(where, f"unknown key '{key}' in {what}")


def check_placement(p, where, rep, assets):
    pid = p.get("id", "")
    if not pid:
        rep.error(where, "placement has no 'id'")

    src = p.get("source", "")
    if src not in SOURCES:
        rep.error(where, f"source '{src}' is not one of {sorted(SOURCES)}")
        return pid, src

    for key in REQUIRED[src]:
        if key not in p:
            rep.error(where, f"a '{src}' placement needs '{key}'")

    capture = p.get("capture")
    if capture is not None and capture not in ("latched", "continuous"):
        rep.error(where, f"capture '{capture}' must be 'latched' or 'continuous'")

    if src == "optitrack":
        aid = p.get("asset_id")
        if isinstance(aid, int):
            if aid <= 0:
                rep.error(where, f"asset_id {aid} must be a positive Motive streaming id")
            elif aid in assets:
                rep.error(where, f"asset_id {aid} is already used by '{assets[aid]}'")
            else:
                assets[aid] = pid

    if src == "constructed":
        con = p.get("construction", {})
        check_unknown(con, CONSTRUCTION_KEYS, where, "'construction'", rep)
        for key in ("normal_axis", "normal_in_part"):
            if key not in con:
                rep.error(where, f"a construction needs '{key}'")
        for key in ("mount_a", "mount_b"):
            if key not in con:
                rep.error(where, f"a construction needs '{key}'")
            elif "quat_wxyz" in con[key]:
                rep.error(where, f"'{key}' is a point, not a pose -- drop its 'quat_wxyz'")
        a, b = con.get("mount_a", {}), con.get("mount_b", {})
        if a.get("position_mm") and a.get("position_mm") == b.get("position_mm"):
            rep.error(where, "mount_a and mount_b are the same point")
        if "expect_normal_in_parent" not in con:
            rep.warn(where, "no 'expect_normal_in_parent' -- the construction's "
                            "normal_axis is then unchecked against anything the room knows")

    if src == "projected":
        joint = p.get("joint", {})
        check_unknown(joint, JOINT_KEYS, where, "'joint'", rep)
        for key in ("zero_pose", "axis_point", "axis", "reported_arm", "reported_index"):
            if key not in joint:
                rep.error(where, f"a joint needs '{key}'")
        if "quat_wxyz" in joint.get("axis_point", {}):
            rep.error(where, "'axis_point' is a point, not a pose -- drop its 'quat_wxyz'")
        if ("lower_deg" in joint) != ("upper_deg" in joint):
            rep.error(where, "'lower_deg' and 'upper_deg' must be given together")

    for key in ("visual_mesh", "urdf"):
        if key in p:
            if not (ROOT / p[key]).exists():
                rep.error(where, f"{key} '{p[key]}' does not exist")

    return pid, src


def main(argv):
    manifest_path = Path(argv[1]) if len(argv) > 1 else ROOT / "config" / "scene.json"
    rep = Report()

    manifest = load(manifest_path, rep)
    if manifest is None:
        print("\n".join(rep.errors))
        return 1

    anchor = manifest.get("world_anchor", "")
    if not anchor:
        rep.error(str(manifest_path), "no 'world_anchor'")

    names = manifest.get("objects")
    if not isinstance(names, list) or not names:
        rep.error(str(manifest_path), "'objects' must be a non-empty array")
        names = []

    placements = {}          # id -> (object, source)
    assets = {}              # motive streaming id -> placement id
    references = []          # (where, key, target)
    articulated = []

    for name in names:
        path = manifest_path.parent / "objects" / f"{name}.json"
        obj = load(path, rep)
        if obj is None:
            continue
        where_obj = f"{name}.json"

        if "visual_mesh" in obj and not (ROOT / obj["visual_mesh"]).exists():
            rep.error(where_obj, f"visual_mesh '{obj['visual_mesh']}' does not exist")

        entries = obj.get("placements")
        if not isinstance(entries, list) or not entries:
            rep.error(where_obj, "'placements' must be a non-empty array")
            continue

        for p in entries:
            where = f"{where_obj} [{p.get('id', '?')}]"
            pid, src = check_placement(p, where, rep, assets)
            if pid in placements:
                rep.error(where, f"placement id '{pid}' is already used by "
                                 f"{placements[pid][0]} -- ids are frame names and must be unique")
            elif pid:
                placements[pid] = (where_obj, src)

            if src == "joint_state":
                articulated.append(pid)
            if p.get("parent_frame"):
                references.append((where, "parent_frame", p["parent_frame"]))
            if p.get("measured"):
                references.append((where, "measured", p["measured"]))
            for i in p.get("inputs", []):
                references.append((where, "inputs", i))

    for where, key, target in references:
        if target not in placements:
            rep.error(where, f"{key} names '{target}', which no loaded object defines")

    if anchor and anchor not in placements:
        rep.error(str(manifest_path),
                  f"world_anchor '{anchor}' is not a placement in any loaded object")

    if len(articulated) > 1:
        rep.error(str(manifest_path),
                  "more than one 'joint_state' placement is loaded (" +
                  ", ".join(articulated) + ") -- RobotScene keeps a single robot "
                  "description, so the last one imported wins and the others hold "
                  "their home pose")

    for line in rep.errors:
        print(f"ERROR   {line}")
    for line in rep.warnings:
        print(f"warning {line}")

    if not rep.errors:
        print(f"ok: {len(names)} objects, {len(placements)} placements, "
              f"{len(assets)} Motive bodies ({', '.join(str(a) for a in sorted(assets))})")
        print(f"    anchor: {anchor}")
    return 1 if rep.errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

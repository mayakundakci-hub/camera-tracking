#!/usr/bin/env python3
"""One entry point for every check that does not need the app running.

    python scripts/lint.py                 # config checks + clang-format, report only
    python scripts/lint.py --fix           # ... and rewrite the C++ files in place
    python scripts/lint.py --changed       # C++ scope = files you have touched vs HEAD
    python scripts/lint.py --tidy          # add clang-tidy (needs a configured preset)
    python scripts/lint.py --qml           # add qmllint (needs a configured preset)

Formatting the whole tree at once is a ~2900-line diff, because frontend/src is K&R
and nodes/ is Allman and .clang-format can only pick one. That is a decision for a
commit of its own -- until someone makes it, use --changed, which is the scope that
keeps a review honest without reformatting code nobody touched.

clang-format and clang-tidy are found on PATH, then in the usual LLVM and Visual
Studio locations. A check whose tool is missing is SKIPPED and says so; it never
passes silently.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CPP_DIRS = ["nodes", "backend", "frontend/src", "middleware", "launcher", "tests"]
CPP_SUFFIXES = {".cpp", ".hpp", ".h"}

VS_2022 = Path(r"C:\Program Files\Microsoft Visual Studio\2022")
TOOL_DIRS = [Path(r"C:\Program Files\LLVM\bin"), Path(r"C:\Program Files (x86)\LLVM\bin")]
if VS_2022.exists():
    TOOL_DIRS += sorted(VS_2022.glob("*/VC/Tools/Llvm/*/bin"))


def find_tool(name):
    found = shutil.which(name)
    if found:
        return found
    for d in TOOL_DIRS:
        candidate = d / (name + (".exe" if os.name == "nt" else ""))
        if candidate.exists():
            return str(candidate)
    return None


def cpp_files(changed_against):
    if changed_against:
        out = subprocess.run(
            ["git", "diff", "--name-only", "--diff-filter=d", changed_against],
            cwd=ROOT, capture_output=True, text=True,
        )
        if out.returncode != 0:
            print(f"  git diff against '{changed_against}' failed: {out.stderr.strip()}")
            return None
        names = [ROOT / n.strip() for n in out.stdout.splitlines() if n.strip()]
        return sorted(
            p for p in names
            if p.suffix in CPP_SUFFIXES and p.exists()
            and any(str(p.relative_to(ROOT)).replace("\\", "/").startswith(d) for d in CPP_DIRS)
        )
    return sorted(
        p for d in CPP_DIRS for p in (ROOT / d).rglob("*") if p.suffix in CPP_SUFFIXES
    )


def build_dir():
    """Newest configured preset that exported a compile database."""
    dbs = sorted((ROOT / "out" / "build").glob("*/compile_commands.json"),
                 key=lambda p: p.stat().st_mtime, reverse=True)
    return dbs[0].parent if dbs else None


# checks

def check_configs():
    """Every JSON in the repo parses, and every scene manifest loads."""
    failures = 0
    targets = [ROOT / "config.json", ROOT / "vcpkg.json", ROOT / "vcpkg-configuration.json",
               ROOT / "CMakePresets.json"]
    targets += sorted((ROOT / "config").rglob("*.json"))
    for path in targets:
        if not path.exists():
            continue
        try:
            json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            print(f"  INVALID JSON  {path.relative_to(ROOT)}: {exc}")
            failures += 1

    for manifest in sorted((ROOT / "config").glob("scene*.json")):
        out = subprocess.run([sys.executable, "scripts/check_scene.py", str(manifest.relative_to(ROOT))],
                             cwd=ROOT, capture_output=True, text=True)
        tag = "ok  " if out.returncode == 0 else "FAIL"
        print(f"  {tag} {manifest.relative_to(ROOT)}: {out.stdout.strip().splitlines()[0] if out.stdout.strip() else out.stderr.strip()}")
        failures += out.returncode != 0
    return failures == 0


def check_format(files, fix):
    tool = find_tool("clang-format")
    if not tool:
        print("  SKIPPED - clang-format not found (install LLVM, or VS's 'C++ Clang tools')")
        return None
    if not files:
        print("  nothing in scope")
        return True

    if fix:
        subprocess.run([tool, "-i", "--style=file", *map(str, files)], cwd=ROOT, check=False)
        print(f"  reformatted {len(files)} file(s) in place")
        return True

    offenders = []
    for path in files:
        out = subprocess.run([tool, "--style=file", "--output-replacements-xml", str(path)],
                             cwd=ROOT, capture_output=True, text=True)
        if "<replacement " in out.stdout:
            offenders.append(path.relative_to(ROOT))
    for path in offenders:
        print(f"  needs formatting  {path}")
    print(f"  {len(files) - len(offenders)}/{len(files)} file(s) already formatted")
    return not offenders


def check_tidy(files, jobs):
    tool = find_tool("clang-tidy")
    if not tool:
        print("  SKIPPED - clang-tidy not found (install LLVM)")
        return None
    bdir = build_dir()
    if not bdir:
        print("  SKIPPED - no compile_commands.json; configure a preset first")
        return None

    db = {Path(e["file"]).resolve() for e in json.loads((bdir / "compile_commands.json").read_text())}
    units = [p for p in files if p.suffix == ".cpp" and p.resolve() in db]
    if not units:
        print(f"  nothing in scope that {bdir.name} compiles")
        return True
    print(f"  {len(units)} translation unit(s) against {bdir.name}")

    def run(path):
        out = subprocess.run([tool, "-p", str(bdir), "--quiet", str(path)],
                             cwd=ROOT, capture_output=True, text=True)
        return path, [l for l in out.stdout.splitlines() if ": warning:" in l or ": error:" in l]

    clean = True
    with ThreadPoolExecutor(max_workers=jobs) as pool:
        for path, findings in pool.map(run, units):
            for line in findings:
                print(f"  {line.strip()}")
            clean &= not findings
    return clean


def check_qml():
    bdir = build_dir()
    if not bdir:
        print("  SKIPPED - no configured preset; qmllint runs off the generated CMake target")
        return None
    out = subprocess.run(["cmake", "--build", str(bdir), "--target", "all_qmllint"],
                         cwd=ROOT, capture_output=True, text=True)
    for line in out.stdout.splitlines():
        if ".qml:" in line or "Warning" in line:
            print(f"  {line.strip()}")
    return out.returncode == 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fix", action="store_true", help="rewrite files instead of reporting")
    ap.add_argument("--changed", nargs="?", const="HEAD", metavar="REF",
                    help="limit the C++ scope to files changed against REF (default HEAD)")
    ap.add_argument("--tidy", action="store_true", help="also run clang-tidy")
    ap.add_argument("--qml", action="store_true", help="also run qmllint")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 4) // 2))
    args = ap.parse_args()

    files = cpp_files(args.changed)
    if files is None:
        return 2

    results = {}
    print("[config]")
    results["config"] = check_configs()
    print("[clang-format]")
    results["clang-format"] = check_format(files, args.fix)
    if args.tidy:
        print("[clang-tidy]")
        results["clang-tidy"] = check_tidy(files, args.jobs)
    if args.qml:
        print("[qmllint]")
        results["qmllint"] = check_qml()

    print()
    for name, ok in results.items():
        print(f"  {name:14s} {'skipped' if ok is None else 'ok' if ok else 'FAILED'}")
    return 0 if all(ok is not False for ok in results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())

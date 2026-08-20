#!/usr/bin/env python3
import math
import re
import sys
import xml.etree.ElementTree as ET
from xml.dom import minidom
from pathlib import Path

XACRO = "http://wiki.ros.org/xacro"
EVAL_NS = {k: getattr(math, k) for k in dir(math) if not k.startswith("_")}
_EXPR = re.compile(r"\$\{([^}]*)\}")

_RENDERING = Path(__file__).resolve().parent.parent / "Rendering"
_DEFAULT_SRC = _RENDERING / "URDFSeperatedGlobalRenderMacro.urdf"
_DEFAULT_OUT = _RENDERING / "cell_flat.urdf"


def local(tag: str) -> str:
    return tag.split("}", 1)[1] if tag.startswith("{") else tag


def is_xacro(tag: str) -> bool:
    return tag.startswith("{" + XACRO + "}")


def subst(text, scope):
    if text is None:
        return None

    def repl(m):
        expr = m.group(1)
        try:
            val = eval(expr, {"__builtins__": {}}, {**EVAL_NS, **scope})  # noqa: S307
        except Exception as e:
            raise SystemExit(f"failed to evaluate ${{{expr}}} with scope "
                             f"{sorted(scope)}: {e}")
        return str(val)

    return _EXPR.sub(repl, text)


def collect_macros(path: Path, macros: dict, seen: set):
    path = path.resolve()
    if path in seen:
        return
    seen.add(path)
    if not path.exists():
        raise SystemExit(f"included file not found: {path}")
    root = ET.parse(path).getroot()
    for el in root.iter():
        if is_xacro(el.tag) and local(el.tag) == "macro":
            macros[el.get("name")] = ((el.get("params") or "").split(), list(el))
    for el in root.iter():
        if is_xacro(el.tag) and local(el.tag) == "include":
            collect_macros(path.parent / el.get("filename"), macros, seen)


def expand(elem, scope, macros):
    tag = elem.tag
    if is_xacro(tag):
        name = local(tag)
        if name in ("macro", "include"):
            return []
        if name not in macros:
            raise SystemExit(f"unknown xacro construct or macro: xacro:{name}")
        params, body = macros[name]
        call_scope = {}
        for p in params:
            raw = elem.get(p)
            if raw is None:
                raise SystemExit(f"macro '{name}' call missing param '{p}'")
            call_scope[p] = subst(raw, scope)
        out = []
        for child in body:
            out.extend(expand(child, call_scope, macros))
        return out

    new = ET.Element(local(tag))
    for k, v in elem.attrib.items():
        new.set(local(k), subst(v, scope))
    if elem.text and elem.text.strip():
        new.text = subst(elem.text, scope)
    for child in elem:
        for ec in expand(child, scope, macros):
            new.append(ec)
    return [new]


def main():
    if len(sys.argv) == 1:
        src, out = _DEFAULT_SRC, _DEFAULT_OUT
    elif len(sys.argv) == 3:
        src, out = Path(sys.argv[1]), Path(sys.argv[2])
    else:
        raise SystemExit(__doc__)

    if not src.exists():
        raise SystemExit(f"input not found: {src}")

    macros = {}
    collect_macros(src, macros, set())

    root = ET.parse(src).getroot()
    flat = ET.Element("robot", {"name": root.get("name", "robot")})
    for child in root:
        for ec in expand(child, {}, macros):
            flat.append(ec)

    n_fixed = 0
    for limit in flat.findall(".//limit"):
        if "velocity" not in limit.attrib or "effort" not in limit.attrib:
            n_fixed += 1
        limit.attrib.setdefault("effort", "0")
        limit.attrib.setdefault("velocity", "0")

    xml = minidom.parseString(ET.tostring(flat)).toprettyxml(indent="  ")
    xml = "\n".join(line for line in xml.splitlines() if line.strip())
    out.write_text(xml, encoding="utf-8")

    n_links = len(flat.findall("link"))
    n_joints = len(flat.findall("joint"))
    n_mesh = len(flat.findall(".//mesh"))
    print(f"wrote {out}")
    print(f"  macros expanded: {sorted(macros)}")
    print(f"  links={n_links}  joints={n_joints}  mesh refs={n_mesh}")
    print(f"  <limit> normalized (added effort/velocity=0): {n_fixed}")


if __name__ == "__main__":
    main()

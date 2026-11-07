"""Build the self-contained Sky Beta node (planner modules + course map embedded).

    python tools/build_single_file.py     # -> flyx/pathplan_mission.py (run as jolie_course)

The node imports nothing from this repo: it embeds the *unchanged* source of the tested planner
modules and installs them as an in-memory ``pathplan`` package, so it behaves exactly like the
repo version. Needs rclpy, the ROS message packages and numpy; scipy is used if present. Re-run
after changing any source (tests/test_single_files.py fails while it is stale).
"""
import argparse
import base64
from hashlib import sha256
from pathlib import Path
import subprocess
import sys
import zlib

import numpy as np

ROOT = Path(__file__).resolve().parents[1]          # pathfinding/
SKYBETA_NODE = ROOT / "ros2/dronehacks_pathplan/dronehacks_pathplan/pathplan_mission.py"
# dependency order (each module's relative imports must already be loaded)
PATHPLAN = ["grid_search", "world", "belief", "depth_projection", "online", "ros_node", "planner_process"]


def build_map(altitude):
    sys.path.insert(0, str(ROOT))
    from pathplan.harness import load_published
    from pathplan.world import Envelope, GridSpec, checkpoints_from_route, occupancy
    course, route = load_published()
    spec, env = GridSpec(), Envelope()
    prior = occupancy(course, spec, altitude=altitude, half_height=env.half_height + env.tracking_margin)
    cps = [dict(name=c.name, centre=c.centre.tolist(), normal=c.normal.tolist(), half_width=c.half_width,
                points=c.points, kind=c.kind, other_route=c.other_route)
           for c in checkpoints_from_route(route, course)]
    return prior, cps


def git_commit():
    try:
        out = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], capture_output=True, text=True)
        return out.stdout.strip() or "unknown"
    except OSError:
        return "unknown"


def replace(text, old, new, what):
    if text.count(old) != 1:
        raise SystemExit(f"{what} changed: update the replacement in build_single_file.py:\n{old}")
    return text.replace(old, new)


RUNTIME = '''

def _install_embedded():
    """Register the embedded modules as in-memory packages (idempotent)."""
    import sys, types
    for pkg_name, names in _PACKAGES.items():
        if getattr(sys.modules.get(pkg_name), "_embedded", False):
            continue
        pkg = types.ModuleType(pkg_name)
        pkg.__path__, pkg._embedded = [], True
        sys.modules[pkg_name] = pkg
        for name in names:
            full = pkg_name + "." + name
            mod = types.ModuleType(full)
            mod.__package__ = pkg_name
            mod.__file__ = "<embedded " + pkg_name + "/" + name + ".py>"
            sys.modules[full] = mod
            exec(compile(_SOURCES[full], mod.__file__, "exec"), mod.__dict__)
            setattr(pkg, name, mod)


def _build_planner_embedded(planner_name="theta", altitude=_MAP_ALTITUDE, chicane="window"):
    """Same as pathplan.ros_node.build_planner, from the embedded map (no course files needed)."""
    import base64, hashlib, zlib
    import numpy as np
    _install_embedded()
    from pathplan.online import OnlinePlanner, PlannerConfig
    from pathplan.world import Checkpoint, Envelope, GridSpec
    if abs(altitude - _MAP_ALTITUDE) > 1e-6:
        raise ValueError(f"this file embeds the map for {_MAP_ALTITUDE} m; rebuild it for {altitude} m")
    if chicane != "window":
        raise ValueError("the embedded checkpoints are the window/Gate B route")
    n = _PRIOR_SHAPE[0] * _PRIOR_SHAPE[1]
    bits = np.frombuffer(zlib.decompress(base64.b64decode(_PRIOR)), np.uint8)
    prior = np.unpackbits(bits)[:n].astype(bool).reshape(_PRIOR_SHAPE)
    if hashlib.sha256(prior.tobytes()).hexdigest() != _PRIOR_SHA256:
        raise ValueError("embedded map is corrupt")
    cps = [Checkpoint(c["name"], np.array(c["centre"]), np.array(c["normal"]), c["half_width"], c["points"],
                      kind=c["kind"], other_route=c["other_route"]) for c in _CHECKPOINTS]
    spec, env = GridSpec(), Envelope()
    return OnlinePlanner(prior, spec, env, cps, PlannerConfig(planner=planner_name)), spec, env, prior
'''


def embedded_section(altitude, prior, cps, packages):
    """The course map, module sources and the functions that install them."""
    out = ["\n\n# " + "=" * 92 + "\n# Embedded planner and course map (generated)\n# " + "=" * 92 + "\n"]
    out.append(f"_MAP_ALTITUDE = {altitude!r}\n")
    out.append(f"_PRIOR_SHAPE = {tuple(prior.shape)!r}\n")
    out.append(f"_PRIOR_SHA256 = {sha256(prior.tobytes()).hexdigest()!r}\n")
    packed = base64.b64encode(zlib.compress(np.packbits(prior).tobytes(), 9)).decode()
    out.append(f"_PRIOR = {packed!r}\n")
    out.append(f"_CHECKPOINTS = {cps!r}\n")             # a Python literal (not JSON: True/False)
    out.append(f"_PACKAGES = {packages!r}\n")
    out.append("_SOURCES = {\n")
    for pkg, names in packages.items():
        for m in names:
            src = (ROOT / pkg / f"{m}.py").read_text(encoding="utf-8")
            if "'''" in src:
                raise SystemExit(f"{pkg}/{m}.py contains ''' and cannot be embedded as a raw string")
            out.append(f"    {pkg + '.' + m!r}: r'''{src}''',\n")
    out.append("}\n")
    out.append(RUNTIME)
    return "".join(out)


def split_doc(text):
    """(docstring body, code after it) of a module that starts with a docstring."""
    _, doc, code = text.split('"""', 2)
    return doc, code


def build_skybeta(altitude, prior, cps, commit):
    node = replace(SKYBETA_NODE.read_text(encoding="utf-8"),
                   """    sys.path.insert(0, _find_pathfinding(''))
    from pathplan.planner_process import PlannerProcess
    from pathplan.ros_node import build_planner
    planner = PlannerProcess(build_planner)""", """    _install_embedded()                        # embedded planner (see the end of this file)
    from pathplan.planner_process import PlannerProcess
    planner = PlannerProcess(_build_planner_embedded)""", "pathplan_mission.py")
    doc, code = split_doc(node)
    code = replace(code, "\n\nif __name__ == '__main__':\n    main()\n", "\n", "pathplan_mission.py main guard")
    head = ('"""Single-file skybeta node: our online path planner behind the skybeta interface.\n\n'
            f'Generated by pathfinding/tools/build_single_file.py (commit {commit}) -- do not edit by hand.\n'
            'Sky Beta contract (no /fmu): FlyX / sim via flyx_skybeta/run.sh jolie, the drone directly.\n'
            'Needs rclpy, geometry_msgs, nav_msgs, sensor_msgs, std_msgs, numpy.\n\n'
            '    python3 pathplan_mission.py --ros-args -p shadow:=true -p depth_topic:=<topic>\n\n'
            '--- original node documentation ---\n' + doc + '"""')
    return (head + code.rstrip() + "\n" + embedded_section(altitude, prior, cps, {"pathplan": PATHPLAN})
            + "\n\nif __name__ == '__main__':\n    main()\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--altitude", type=float, default=.9)
    a = ap.parse_args()
    prior, cps = build_map(a.altitude)
    commit = git_commit()
    for name, build in (("pathplan_mission.py", build_skybeta),):
        path = ROOT / "flyx" / name
        text = build(a.altitude, prior, cps, commit)
        compile(text, str(path), "exec")
        path.write_text(text, encoding="utf-8", newline="\n")
        print(f"wrote {path} ({len(text) / 1024:.0f} KB), map {prior.shape} at {a.altitude} m, "
              f"{len(cps)} checkpoints, commit {commit}")


if __name__ == "__main__":
    main()

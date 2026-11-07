"""Build flyx_course/fly_course.py: the template + machmoon/dronehacks-sim mission/core.py (verbatim)
+ a route JSON.  python3 flyx_course/build_fly_course.py <machmoon-repo> [--route mission/course.json]"""
import argparse
import json
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("repo", type=Path)
    p.add_argument("--route", default="mission/course.json")
    a = p.parse_args()
    commit = subprocess.run(["git", "-C", str(a.repo), "rev-parse", "--short", "HEAD"],
                            capture_output=True, text=True).stdout.strip() or "unknown"
    core_src = (a.repo / "mission/core.py").read_text(encoding="utf-8")
    route = json.loads((a.repo / a.route).read_text(encoding="utf-8"))
    out = (HERE / "fly_course_template.py").read_text(encoding="utf-8")
    out = out.replace("@@COMMIT@@", commit).replace("@@ROUTE@@", a.route)
    out = out.replace("@@CORE_SRC@@", repr(core_src)).replace("@@ROUTE_JSON@@", repr(json.dumps(route)))
    (HERE / "fly_course.py").write_text(out, encoding="utf-8", newline="\n")
    print(f"wrote {HERE / 'fly_course.py'}: core.py @ {commit}, route {a.route} ({len(route['waypoints'])} waypoints)")


if __name__ == "__main__":
    main()

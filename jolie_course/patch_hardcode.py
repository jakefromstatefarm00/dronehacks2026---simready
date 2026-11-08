"""Hardcoded whole-course route for a stay-left v4c file (Sky Beta or FlyX), no planner in the loop.

    python3 jolie_course/patch_hardcode.py upload_to_flyx_v4c_skybeta_left.py upload_to_flyx_v4c_skybeta_hard.py
    python3 jolie_course/patch_hardcode.py upload_to_flyx_v4c_left.py upload_to_flyx_v4c_hard.py

The route: the stay-left start (patch_left_start.py), then the path the v4c Theta* planner takes
on the published map from there (a kinematic replay, no depth, no drift; Douglas-Peucker 4 cm),
24 points at 0.9 m, ending over the pad centre. Clearance to published walls along every segment
is at least 0.40 m (drone radius 0.28 m). The setpoint moves along it at 0.35 m/s and never more
than 0.35 m ahead of the drone; no heading command on Sky Beta (the nose stays up the first
corridor, as at takeoff). At the end: LAND (Sky Beta: hold over the pad, the pilot lands).

No obstacle avoidance: a moved wall is NOT avoided. --start-script left gives the planner version,
--start-script none the original.
"""
import sys
from pathlib import Path

ROUTE = [(-2.102, 2.598), (-1.773, 2.577), (-1.473, 2.477), (-1.373, 2.177), (-1.323, 1.127),
         (-0.973, 0.977), (-0.473, 1.027), (-0.323, 1.127), (-0.273, 1.227), (-0.223, 2.327),
         (-0.123, 2.477), (0.177, 2.577), (0.617, 2.577), (1.977, 1.777), (2.177, 1.477),
         (2.277, 1.127), (2.325, -0.612), (1.577, -1.573), (1.377, -1.723), (0.127, -2.373),
         (-0.073, -2.423), (-0.973, -2.073), (-1.781, -1.079), (-2.042, -0.792)]


def build(src, out):
    s = Path(src).read_text(encoding="utf-8").replace("\r\n", "\n")
    sky = "class SkybetaNode(Node)" in s

    def rep(a, b, label, count=1):
        nonlocal s
        n = s.count(a)
        assert n == count, f"{label}: expected {count} match(es), found {n}"
        s = s.replace(a, b)

    s = ('"""HARDCODED ROUTE (jolie_course/patch_hardcode.py): stay-left start, then a fixed 0.9 m route\n'
         'through all five checkpoints to the pad; no planner, no obstacle avoidance.\n'
         '--start-script left = planner version, none = original.\n\n' + s[3:])
    pts = ",\n    ".join(f"({x:+.3f}, {y:+.3f}, .90)" for x, y in ROUTE)
    rep('''    "none": (),
}
''', f'''    "none": (),
}}
# Hardcoded (patch_hardcode.py): the stay-left start, then v4c's planned route on the published map
START_SCRIPTS["hard"] = START_SCRIPTS["left"] + (
    {pts},
)
''', "route")
    rep('''    start_script_no_yaw: bool = False      # Sky Beta: no heading command until NAVIGATE
''', '''    start_script_no_yaw: bool = False      # Sky Beta: no heading command until NAVIGATE
    start_script_then: str = "NAVIGATE"    # after the script: NAVIGATE (planner) or LAND (hardcoded route)
''', "config")
    rep('''            self._go("NAVIGATE", now, "start script done: the planner takes over")
''', '''            if c.start_script_then == "LAND":
                return self._land(now, "hardcoded route done: over the pad")
            self._go("NAVIGATE", now, "start script done: the planner takes over")
''', "end")
    hard = ('start_script_then="LAND" if a.start_script == "hard" else "NAVIGATE", '
            'start_script_speed=.35 if a.start_script == "hard" else .30, '
            'start_script_timeout_s=150.0 if a.start_script == "hard" else 45.0')
    if sky:
        rep("start_script=START_SCRIPTS[a.start_script], start_script_no_yaw=True))",
            f"start_script=START_SCRIPTS[a.start_script], start_script_no_yaw=True,\n        {hard}))", "call")
    else:
        rep("start_script=START_SCRIPTS[a.start_script]))",
            f"start_script=START_SCRIPTS[a.start_script],\n        {hard}))", "call")
    rep('ap.add_argument("--start-script", default="left"', 'ap.add_argument("--start-script", default="hard"', "cli")
    rep('help="left: stay left low past the leaning wall, then the planner; none: original")',
        'help="hard: fixed route; left: stay left low past the leaning wall, then the planner; none: original")', "help")

    compile(s, out, "exec")
    Path(out).write_text(s, encoding="utf-8", newline="\n")
    print(f"wrote {out} ({'Sky Beta' if sky else 'FlyX'})")


if __name__ == "__main__":
    here = Path(__file__).resolve().parent
    build(here / sys.argv[1], here / sys.argv[2])

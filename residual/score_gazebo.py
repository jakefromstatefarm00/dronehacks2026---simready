"""Score GB10 Gazebo real-PX4 flights of Jolie's file, with and without the residual.

    python -m residual.score_gazebo <artifacts dir with gb10-sky-beta-course-vNNN/> v200-v214 v500-v503

Reads each run's native-program.log (result, checkpoints, LAND reason, residual stats), the
journal (state times, sim clock) and truth-poses.csv (Gazebo ground truth) and reports NAVIGATE
time (sim s) and |yaw rate| x speed from truth: the peak within 1 m of the first corridor-to-maze
corner (where dynamics-calib found every VIO divergence start), the peak anywhere in NAVIGATE, and
the time above 0.10 rad m/s^2. (origin/gazebo-jolie's score_run.py was not available.)
"""
import csv
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

CORNER = np.array([-2.35, 2.5])
YV_FREE = .10


def yaw_of(qw, qx, qy, qz):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def score(run):
    log = (run / "native-program.log").read_text(errors="replace")
    res = {"run": run.name.rsplit("-", 1)[-1]}
    m = re.search(r"RESULT: (.*)", log)
    res["result"] = m.group(1).strip() if m else "?"
    m = re.search(r"checkpoints cleared: .*\((\d)/5\)", log)
    res["checkpoints"] = int(m.group(1)) if m else 0
    m = re.search(r"state NAVIGATE -> LAND (.*)", log)
    res["land_reason"] = m.group(1).strip()[:60] if m else ""
    m = re.search(r"residual: (.*)", log)
    res["residual"] = ("ON: " + m.group(1)[:50]) if m and "OFF" not in m.group(1) else "off"
    res["complete"] = res["checkpoints"] == 5 and res["land_reason"].startswith("landing pad reached")
    t_nav = t_land = None
    for line in (run / "journal" / "journal.jsonl").read_text(errors="replace").splitlines():
        if '"kind": "state"' not in line:
            continue
        r = json.loads(line)
        if r["state"] == "NAVIGATE" and t_nav is None:
            t_nav = r["t"]
        if r["state"] == "LAND" and t_land is None:
            t_land = r["t"]
    res["navigate_s"] = round(t_land - t_nav, 1) if t_nav is not None and t_land is not None else None
    rows = list(csv.DictReader(open(run / "truth-poses.csv")))
    t = np.array([float(r["t"]) for r in rows])
    xy = np.array([[float(r["x"]), float(r["y"])] for r in rows])
    z = np.array([float(r["z"]) for r in rows])
    yaw = np.unwrap([yaw_of(float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"])) for r in rows])
    # yaw rate and speed over a 0.2 s window (truth ~48 Hz; its vx/vy columns are not filled)
    k = max(1, int(round(.1 / max(np.median(np.diff(t)), 1e-3))))
    w, v = np.zeros_like(yaw), np.zeros_like(yaw)
    span = np.maximum(t[2 * k:] - t[:-2 * k], 1e-3)
    w[k:-k] = (yaw[2 * k:] - yaw[:-2 * k]) / span
    v[k:-k] = np.linalg.norm(xy[2 * k:] - xy[:-2 * k], axis=1) / span
    air = z > .5
    if t_nav is not None:                                   # NAVIGATE only (journal and truth: sim time)
        air &= (t >= t_nav) & (t <= (t_land if t_land is not None else t[-1]))
    # cut at the first sign of VIO divergence: the mission never commands > 0.25 rad/s or > ~0.6 m/s,
    # so |yaw rate| > 1 rad/s or speed > 1 m/s is the estimator (not the program) driving the vehicle
    bad = np.flatnonzero(air & ((np.abs(w) > 1.0) | (v > 1.0)))
    res["diverged_at_s"] = round(float(t[bad[0]] - t_nav), 1) if len(bad) and t_nav is not None else None
    if len(bad):
        air &= t < t[bad[0]]
    yv = np.abs(w) * v
    near = air & (np.linalg.norm(xy - CORNER, axis=1) < 1.0)
    dt = np.r_[np.diff(t), 0]
    res["corner_yv_peak"] = round(float(yv[near].max()), 3) if near.any() else None
    res["yv_peak"] = round(float(yv[air].max()), 3) if air.any() else None
    res["yv_above_0p10_s"] = round(float(dt[air & (yv > YV_FREE)].sum()), 2)
    res["yaw_rate_peak"] = round(float(np.abs(w[air]).max()), 3) if air.any() else None
    res["speed_mean"] = round(float(v[air].mean()), 3) if air.any() else None
    return res


def expand(spec):
    a, b = spec.lstrip("v").split("-v") if "-" in spec else (spec.lstrip("v"), spec.lstrip("v"))
    return [f"v{i}" for i in range(int(a), int(b) + 1)]


def main(argv=None):
    argv = argv or sys.argv[1:]
    root = Path(argv[0])
    out = []
    for spec in argv[1:]:
        for v in expand(spec):
            run = root / f"gb10-sky-beta-course-{v}"
            if (run / "truth-poses.csv").exists() and "RESULT:" in (run / "native-program.log").read_text(errors="replace"):
                out.append(score(run))
    cols = ["run", "residual", "complete", "checkpoints", "navigate_s", "land_reason", "diverged_at_s", "corner_yv_peak", "yv_peak",
            "yv_above_0p10_s", "yaw_rate_peak", "speed_mean"]
    print("| " + " | ".join(cols) + " |\n|" + "---|" * len(cols))
    for r in out:
        print("| " + " | ".join(str(r.get(c, "")) for c in cols) + " |")
    return out


if __name__ == "__main__":
    main()

"""Score fetched GB10 Gazebo runs (v9xx) for best-v5: truth clearance on the published course, yaw x speed,
in-place yaw near walls, short-view yaw, corner view, planner events, planner ms, host load validity.

    python residual/gz_score.py <dir with vNNN/{native-program.log,journal/journal.jsonl,truth-poses.csv}> <uptime log> [label=v4c:900,902 ...]

Truth is used only here (scoring), never by flight code.
"""
import csv
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
CORNER = np.array([-2.35, 2.5])
LAUNCH = np.array([-2.515, .914])


def yaw_of(qw, qx, qy, qz):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def load_uptime(path):
    out = {}
    for line in Path(path).read_text().splitlines():
        m = re.match(r"v(\d+) (start|end) \d+ .*load average: ([0-9.]+), ([0-9.]+), ([0-9.]+)", line)
        if m:
            out.setdefault(m.group(1), {})[m.group(2)] = float(m.group(3))
    return out


def score(run, course, half_height, occ, spec):
    from residual.sim import exact_clearance
    from residual.eval_v5 import _ray
    log = (run / "native-program.log").read_text(errors="replace")
    res = {"run": run.name}
    m = re.search(r"RESULT: (.*)", log)
    res["result"] = m.group(1).strip()[:50] if m else "?"
    m = re.search(r"checkpoints cleared: .*\((\d)/5\)", log)
    res["cps"] = int(m.group(1)) if m else 0
    m = re.search(r"state NAVIGATE -> LAND (.*)", log)
    res["land"] = m.group(1).strip()[:50] if m else ""
    res["complete"] = res["cps"] == 5 and res["land"].startswith("landing pad reached")
    t_nav = t_land = None
    ms, ev = [], []
    for line in (run / "journal" / "journal.jsonl").read_text(errors="replace").splitlines():
        if '"kind": "state"' in line:
            r = json.loads(line)
            if r["state"] == "NAVIGATE" and t_nav is None:
                t_nav = r["t"]
            if r["state"] == "LAND" and t_land is None:
                t_land = r["t"]
        elif '"kind": "plan"' in line:
            r = json.loads(line)
            ms.append(r.get("planner_ms", r.get("ms", 0.0)))
        elif '"kind": "end"' in line:
            ev = json.loads(line).get("planner_events", [])
    res["nav_s"] = round(t_land - t_nav, 1) if t_nav is not None and t_land is not None else None
    res["plan_ms_max"] = round(max(ms), 0) if ms else None
    res["plan_ms_p95"] = round(float(np.percentile(ms, 95)), 0) if ms else None
    names = [e.get("event") for e in ev]
    for k, n in (("near", "keep-out near miss"), ("forget", "forget depth map"), ("look", "look around"),
                 ("back", "back out to look"), ("escape", "escape")):
        res[k] = names.count(n)
    rows = [r for r in csv.DictReader(open(run / "truth-poses.csv")) if all(r.get(k) not in (None, "") for k in
                                                                          ("t", "x", "y", "z", "qw", "qx", "qy", "qz"))]
    t = np.array([float(r["t"]) for r in rows])
    xy = np.array([[float(r["x"]), float(r["y"])] for r in rows])
    z = np.array([float(r["z"]) for r in rows])
    yaw = np.unwrap([yaw_of(float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"])) for r in rows])
    k = max(1, int(round(.1 / max(np.median(np.diff(t)), 1e-3))))
    w, v = np.zeros_like(yaw), np.zeros_like(yaw)
    span = np.maximum(t[2 * k:] - t[:-2 * k], 1e-3)
    w[k:-k] = (yaw[2 * k:] - yaw[:-2 * k]) / span
    v[k:-k] = np.linalg.norm(xy[2 * k:] - xy[:-2 * k], axis=1) / span
    air = z > .5
    if t_nav is not None:
        air &= (t >= t_nav) & (t <= (t_land if t_land is not None else t[-1]))
    bad = np.flatnonzero(air & ((np.abs(w) > 1.0) | (v > 1.0)))
    res["diverged_at"] = round(float(t[bad[0]] - t_nav), 1) if len(bad) and t_nav is not None else None
    airall = z > .5
    dt = np.r_[np.diff(t), 0]
    # truth clearance over the whole flight above 0.5 m (subsampled to ~10 Hz)
    idx = np.flatnonzero(airall)[::max(1, k // 2)]
    clr = np.array([exact_clearance(course, np.array([xy[i, 0], xy[i, 1], .9]), half_height) for i in idx])
    res["min_clear"] = round(float(clr.min()), 3) if len(clr) else None
    res["contact"] = bool(len(clr) and clr.min() < .28)
    cl_full = np.interp(t, t[idx], clr) if len(idx) > 1 else np.full_like(t, .5)
    if len(bad):
        air &= t < t[bad[0]]
    yv = np.abs(w) * v
    near = air & (np.linalg.norm(xy - CORNER, axis=1) < 1.0)
    res["corner_yv"] = round(float(yv[near].max()), 3) if near.any() else None
    res["yv_peak"] = round(float(yv[air].max()), 3) if air.any() else None
    slow = air & (v < .08)
    res["wall_yaw_rad"] = round(float((np.abs(w) * dt)[slow & (cl_full < .6)].sum()), 2)
    stride = max(1, k // 2)
    sel = np.flatnonzero(air)[::stride]
    views = np.array([_ray(occ, spec, xy[i] + .115 * np.array([math.cos(yaw[i]), math.sin(yaw[i])]), yaw[i]) for i in sel])
    wd = (np.abs(w) * dt)
    chunk = float(np.median(np.diff(t))) * stride
    res["short_view_rad"] = round(float(np.sum(np.abs(w[sel])[views < 1.0]) * chunk), 2) if len(sel) else None
    cn = np.linalg.norm(xy[sel] - CORNER, axis=1) < 1.0
    res["corner_view_min"] = round(float(views[cn].min()), 2) if cn.any() else None
    res["corner_view_mean"] = round(float(views[cn].mean()), 2) if cn.any() else None
    ln = air & (np.linalg.norm(xy - LAUNCH, axis=1) < .6)
    res["launch_turn_rad"] = round(float(wd[ln].sum()), 2)
    return res


def main():
    root, upt = Path(sys.argv[1]), load_uptime(sys.argv[2])
    groups = {}
    for a in sys.argv[3:]:
        lab, _, ids = a.partition("=")
        groups[lab] = [i.strip() for i in ids.split(",") if i.strip()]
    from residual import sim
    st = sim.static_world()
    course, env = st["course"], st["env"]
    fine = st["fine"]
    occ = st["occupancy"](course, fine, altitude=.9, half_height=env.half_height, fine=.0125)
    cols = ["run", "arm", "load s/e", "valid", "complete", "cps", "nav_s", "land", "diverged_at", "contact", "min_clear",
            "corner_yv", "yv_peak", "wall_yaw_rad", "short_view_rad", "corner_view_min", "corner_view_mean",
            "launch_turn_rad", "near", "forget", "look", "back", "escape", "plan_ms_p95", "plan_ms_max"]
    print("| " + " | ".join(cols) + " |\n|" + "---|" * len(cols))
    rows = []
    for lab, ids in groups.items():
        for n in ids:
            run = root / f"v{n}"
            if not all((run / f).exists() for f in ("truth-poses.csv", "native-program.log", "journal/journal.jsonl")):
                print(f"| v{n} | {lab} | (files missing) |", flush=True)
                continue
            r = score(run, course, env.half_height, occ, fine)
            u = upt.get(n, {})
            r["arm"] = lab
            r["load s/e"] = f"{u.get('start', float('nan')):.1f}/{u.get('end', float('nan')):.1f}"
            r["valid"] = bool(u.get("start", 99) <= 20 and u.get("end", 99) <= 20)
            rows.append(r)
            print("| " + " | ".join(str(r.get(c, "")) for c in cols) + " |", flush=True)
    Path(root / "scores.json").write_text(json.dumps(rows, indent=1, default=str))


if __name__ == "__main__":
    main()

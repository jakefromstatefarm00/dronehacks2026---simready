"""Depth-on / depth-off table: score_run.py's truth score + the planner's depth stats + GB10 load.

    python jolie_course/gazebo/depth_table.py <run_dir>... [--out rows.jsonl]

Depth-on runs are the ones launched through jolie_flyx/with_depth.py.  Load comes from
<run_dir>.load.txt (rig/ab_series.sh: one 1-min load average per 10 s during the flight).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from score_run import parse_journal, score  # noqa: E402


def depth_row(run_dir: Path) -> dict:
    r = score(run_dir)
    r["depth_on"] = "with_depth" in r.get("program", "")
    _, end = parse_journal(run_dir / "journal" / "journal.jsonl")
    d, q = (end or {}).get("depth") or {}, (end or {}).get("depth_quality") or {}
    r["depth_used"] = d.get("frames")
    r["depth_matched"] = d.get("matched")
    r["hits_on_walls"] = round(q["hits_on_published_walls"] / q["hits"], 3) if q.get("hits") else None
    r["new_obstacles"] = q.get("hits_new_confirmed")
    lf = run_dir.with_name(run_dir.name + ".load.txt")
    if lf.exists():
        L = [float(x.split()[1]) for x in lf.read_text().split("\n") if x.strip()]
        r["load_mean"], r["load_max"] = round(float(np.mean(L)), 1), round(float(np.max(L)), 1)
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--out", type=Path)
    a = ap.parse_args()
    rows = [depth_row(d) for d in a.runs]
    if a.out:
        a.out.write_text("".join(json.dumps(r) + "\n" for r in rows))
    hdr = ["run", "depth", "ok", "cp", "nav_sim", "a2d_sim", "minclr", "n<.28", "pad", "frames", "on_walls",
           "new_obs", "load_mean", "load_max"]
    print(" | ".join(hdr))
    for r in rows:
        print(" | ".join(str(x) for x in [r["run"], "on" if r["depth_on"] else "off", int(r["completed"]),
              r.get("cleared"), r.get("navigate_s_sim"), r.get("arm_to_disarm_s_sim"), r.get("min_clear_m"),
              r.get("n_pts_under_028"), r.get("pad_err_m"), r.get("depth_used"), r.get("hits_on_walls"),
              r.get("new_obstacles"), r.get("load_mean"), r.get("load_max")]))
    for on in (False, True):
        rs = [r for r in rows if r["depth_on"] == on]
        if not rs:
            continue
        ok = [r for r in rs if r["completed"]]
        nav = [r["navigate_s_sim"] for r in ok if r.get("navigate_s_sim")]
        clr = [r["min_clear_m"] for r in ok if r.get("min_clear_m") is not None]
        s = f"depth {'on ' if on else 'off'}: completed {len(ok)}/{len(rs)}"
        if nav:
            s += f"; NAVIGATE mean {np.mean(nav):.1f} s (min {min(nav):.1f}, max {max(nav):.1f})"
        if clr:
            s += f"; min clearance >= 0.29 m on {sum(c >= .29 for c in clr)}/{len(clr)} completed, worst {min(clr):.3f} m"
        if on:
            w = [r["hits_on_walls"] for r in rs if r.get("hits_on_walls") is not None]
            s += f"; frames used mean {np.mean([r['depth_used'] or 0 for r in rs]):.0f}"
            if w:
                s += f"; hits on published walls {min(w):.0%}-{max(w):.0%}"
            s += f"; new obstacles {[r.get('new_obstacles') for r in rs]}"
        print(s)


if __name__ == "__main__":
    main()

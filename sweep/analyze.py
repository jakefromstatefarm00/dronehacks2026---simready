"""Summaries of runs.csv: per (config, level) completion, time, clearance, contacts, VIO risk.

    python3 analyze.py results/coarse/runs.csv                # table per level
    python3 analyze.py results/coarse/runs.csv --json out.json

completion  = pad reached (the program's LAND at the pad), no crash, within the flight limit
clean       = completion with no touch at all
touch       = true wall clearance below the 0.28 m footprint radius (a prop-tip graze; counted)
crash       = true wall clearance below 0.25 m (the run ends there)
net_touch   = the footprint crossed the arena net line (counted, the run continues: FlyX's net
              did not stop the proven run, but on the drone it is a prop strike)
"""
from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


def load(paths):
    rows = []
    for p in paths:
        with open(p, newline="") as fh:
            rows += list(csv.DictReader(fh))
    for r in rows:
        for k in ("course_s", "min_wall", "min_net", "min_clear", "p5_clear", "pad_err", "fast_yaw_s",
                  "inplace_turn_s", "stall_s", "path_m", "wall_s"):
            r[k] = float(r[k]) if r.get(k) not in (None, "", "None") else np.nan
        for k in ("seed", "checkpoints", "net_touches", "touches", "vio_events", "vio_losses", "escapes", "lookarounds"):
            r[k] = int(float(r[k])) if r.get(k) not in (None, "", "None") else 0
    return rows


def summarise(rows):
    """{(config, level): metrics}"""
    g = defaultdict(list)
    for r in rows:
        g[(r["config"], r["level"])].append(r)
    out = {}
    for key, rs in g.items():
        fin = [r for r in rs if r["outcome"] == "finished"]
        t = np.array([r["course_s"] for r in fin]) if fin else np.array([np.nan])
        mw = np.array([r["min_wall"] for r in rs])
        out[key] = dict(
            n=len(rs), completion=100 * len(fin) / len(rs),
            t_mean=float(np.nanmean(t)) if fin else np.nan, t_p90=float(np.nanpercentile(t, 90)) if fin else np.nan,
            min_wall=float(np.nanmin(mw)), min_wall_p5=float(np.nanpercentile(mw, 5)),
            p5_clear=float(np.nanmean([r["p5_clear"] for r in rs])),
            clean=100 * sum(r["outcome"] == "finished" and r["touches"] == 0 for r in rs) / len(rs),
            crashes=sum(r["outcome"] in ("crash", "contact") for r in rs),
            touch_runs=sum(r["touches"] > 0 for r in rs),
            touches=float(np.mean([r["touches"] for r in rs])),
            net_touch_runs=sum(r["net_touches"] > 0 for r in rs),
            vio_events=float(np.mean([r["vio_events"] for r in rs])),
            vio_losses=float(np.mean([r["vio_losses"] for r in rs])),
            fast_yaw_s=float(np.mean([r["fast_yaw_s"] for r in rs])),
            inplace_s=float(np.mean([r["inplace_turn_s"] for r in rs])),
            checkpoints=float(np.mean([r["checkpoints"] for r in rs])),
            escapes=float(np.mean([r["escapes"] for r in rs])),
            landed_early=sum(r["outcome"] == "landed_early" for r in rs),
            errors=sum(r["outcome"] == "error" for r in rs),
            timeouts=sum(r["outcome"] == "timeout" for r in rs),
            whys=sorted({r["why"][:60] for r in rs if r["why"]})[:4],
            # where runs fail: "after cp k" = k checkpoints cleared (0 = first corridor, 1 = maze to
            # Gate A, 2 = tunnel, 3 = chicane/window, 4 = landing approach)
            fail_at=" ".join(f"{k}:{v}" for k, v in sorted(Counter(
                r["checkpoints"] for r in rs if r["outcome"] != "finished").items())),
        )
    return out


COLS = ["n", "completion", "clean", "t_mean", "t_p90", "min_wall", "min_wall_p5", "p5_clear", "crashes", "touch_runs",
        "net_touch_runs",
        "vio_events", "fast_yaw_s", "inplace_s", "checkpoints", "escapes", "landed_early", "timeouts", "errors"]


def table(summary, configs=None, levels=None):
    levels = levels or sorted({lv for _, lv in summary})
    configs = configs or sorted({c for c, _ in summary})
    lines = ["config     level        " + " ".join(f"{c[:9]:>9s}" for c in COLS)]
    for c in configs:
        for lv in levels:
            m = summary.get((c, lv))
            if m:
                lines.append(f"{c:10s} {lv:12s} " + " ".join(
                    (f"{m[k]:9.3f}" if k.startswith(("min_", "p5")) else f"{m[k]:9.1f}") if isinstance(m[k], float)
                    else f"{m[k]:9d}" for k in COLS))
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+", type=Path)
    ap.add_argument("--configs", nargs="*")
    ap.add_argument("--json", type=Path)
    a = ap.parse_args(argv)
    s = summarise(load(a.csv))
    print(table(s, a.configs))
    if a.json:
        a.json.write_text(json.dumps({f"{c}|{lv}": m for (c, lv), m in s.items()}, indent=1, default=float))


if __name__ == "__main__":
    main()

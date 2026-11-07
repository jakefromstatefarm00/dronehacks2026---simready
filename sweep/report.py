"""Markdown tables for RESULTS.md from final / failmap / oat runs.

    python3 report.py final results/final/runs.csv --names results/candidates.json
    python3 report.py failmap results/failmap/runs.csv
    python3 report.py oat results/oat/runs.csv
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

import analyze


def fmt(v, d=1):
    return "-" if v is None or (isinstance(v, float) and not np.isfinite(v)) else f"{v:.{d}f}"


def final_table(summary, names):
    levels = sorted({lv for _, lv in summary})
    lines = ["| config | level | n | completion % | clean % | time mean / p90 s | min wall m | p5 clear m | "
             "crashes | touch runs | net-touch runs | VIO events | VIO losses | t > 0.35 rad/s | in-place turn s | checkpoints |",
             "|" + "---|" * 16]
    for cid, label in names:
        for lv in levels:
            m = summary.get((cid, lv))
            if not m:
                continue
            lines.append(f"| {label} | {lv} | {m['n']} | {fmt(m['completion'], 0)} | {fmt(m['clean'], 0)} | "
                         f"{fmt(m['t_mean'])} / {fmt(m['t_p90'])} | {fmt(m['min_wall'], 3)} | {fmt(m['p5_clear'], 3)} | "
                         f"{m['crashes']} | {m['touch_runs']} | {m['net_touch_runs']} | {fmt(m['vio_events'], 2)} | "
                         f"{fmt(m['vio_losses'], 2)} | {fmt(m['fast_yaw_s'])} | {fmt(m['inplace_s'])} | "
                         f"{fmt(m['checkpoints'], 2)} |")
    return "\n".join(lines)


def failmap_table(summary, config="base"):
    import failmap
    lines = ["| perturbation (on L0) | n | completion % | clean % | time mean s | p5 clear m | crashes | early lands | "
             "timeouts | VIO events | fails after cp (k:n) | failure reasons |", "|" + "---|" * 12]
    order = ["L0", "L0@rtf1.0"] + [ax["name"] for ax in failmap.AXES]
    for lv in order:
        m = summary.get((config, lv))
        if not m:
            continue
        lines.append(f"| {lv} | {m['n']} | {fmt(m['completion'], 0)} | {fmt(m['clean'], 0)} | {fmt(m['t_mean'])} | "
                     f"{fmt(m['p5_clear'], 3)} | {m['crashes']} | {m['landed_early']} | {m['timeouts']} | "
                     f"{fmt(m['vio_events'], 2)} | {m['fail_at']} | {'; '.join(m['whys'][:2])} |")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("kind", choices=["final", "failmap", "oat"])
    ap.add_argument("csv", nargs="+", type=Path)
    ap.add_argument("--names", type=Path)
    ap.add_argument("--config", default="base")
    a = ap.parse_args(argv)
    s = analyze.summarise(analyze.load(a.csv))
    if a.kind == "final":
        names = [("base", "base (proven)")]
        if a.names:
            names += [(c["id"], c["name"]) for c in json.loads(a.names.read_text())]
        else:
            names += [(c, c) for c in sorted({c for c, _ in s} - {"base"})]
        print(final_table(s, names))
    elif a.kind == "failmap":
        print(failmap_table(s, a.config))
    else:
        cfgs = sorted({c for c, _ in s})
        print(final_table(s, [(c, c) for c in cfgs]))


if __name__ == "__main__":
    main()

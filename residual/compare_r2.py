"""Round 2 decision: base vs current residual (round 1) vs candidate (round 2), 32+ held-out seeds per cell.

    python -m residual.compare_r2 --candidate residual/runs/round2/gate_024.json --seeds 1000-1031 --workers 10

Writes residual/results/round2.md and round2_episodes.jsonl. Rebuild rule (coordinator): adopt the
candidate only if it is no worse than the current residual on L0-L2 (completion and contacts) and
better on C0/C1/D1. Acceptance vs the base is unchanged (evaluate.accept).
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from residual import evaluate        # noqa: E402

LEVELS = ["L0", "L1", "L2", "C0", "C1", "D1"]
KEEP = ["L0", "L1", "L2"]
BETTER = ["C0", "C1", "D1"]


def table(rows, arms):
    out = {}
    for arm in arms:
        rs = [r for r in rows if r["arm"] == arm]
        for r in rs:
            r["residual"] = arm != "base"
        t = evaluate.summarize(rs)
        for lv in LEVELS:
            m = t.get((lv, arm != "base"))
            if m:
                out[(lv, arm)] = m
    return out


def fmt(m):
    lo, hi = m["completion_ci"]
    clo, chi = m["contact_ci"]
    return (f"{100 * m['completion']:.0f} % [{100 * lo:.0f}, {100 * hi:.0f}] | {m['time_mean']:.1f} ± {m['time_ci_half']:.1f} | "
            f"{m['clear_p5']:.3f} | {m['contacts']} [{100 * clo:.0f}-{100 * chi:.0f} %] | {m['corner_yv_peak']:.3f} | "
            f"{m['yv_excess']:.3f} | {m['vio_events']}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--current", type=Path, default=REPO / "residual" / "policy.json")
    ap.add_argument("--candidate", type=Path, required=True)
    ap.add_argument("--seeds", default="1000-1031")
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--out", type=Path, default=REPO / "residual" / "results")
    a = ap.parse_args(argv)
    seeds = evaluate.parse_seeds(a.seeds)
    pol = {"base": None, "round1": json.loads(a.current.read_text()), "round2": json.loads(a.candidate.read_text())}
    tasks, labels = [], []
    for arm, p in pol.items():
        for lv in LEVELS:
            for s in seeds:
                tasks.append((lv, s, p, p is not None))
                labels.append(arm)
    t0 = time.time()
    rows = evaluate.run_tasks(tasks, a.workers)
    for r, arm in zip(rows, labels):
        r["arm"] = arm
    a.out.mkdir(parents=True, exist_ok=True)
    with open(a.out / "round2_episodes.jsonl", "w") as fh:
        for r in rows:
            fh.write(json.dumps(r, default=float) + "\n")
    tb = table(rows, list(pol))
    lines = [f"Round 2: seeds {a.seeds} ({len(seeds)} per cell), candidate {a.candidate.name}, {time.time() - t0:.0f} s wall.",
             "Completion and contact-rate intervals: 95 % Wilson; time: mean ± 95 % normal half-width (completed runs, NAVIGATE sim s).", "",
             "| level | arm | completion [95 % CI] | NAVIGATE s | p5 min clearance m | contacts [rate CI] | corner yaw-rate x speed (mean peak) | yv excess | VIO events |",
             "|---|---|---|---|---|---|---|---|---|"]
    for lv in LEVELS:
        for arm in pol:
            if (lv, arm) in tb:
                lines.append(f"| {lv} | {arm} | " + fmt(tb[(lv, arm)]) + " |")

    def not_worse(x, y, lv):     # x not worse than y at lv
        return tb[(lv, x)]["completion"] >= tb[(lv, y)]["completion"] and tb[(lv, x)]["contacts"] <= tb[(lv, y)]["contacts"]

    def better(x, y, lv):
        bx, by = tb[(lv, x)], tb[(lv, y)]
        return (bx["completion"] > by["completion"] or bx["contacts"] < by["contacts"]) and not_worse(x, y, lv)

    keep_ok = all(not_worse("round2", "round1", lv) for lv in KEEP)
    better_ok = all(not_worse("round2", "round1", lv) for lv in BETTER) and any(better("round2", "round1", lv) for lv in BETTER)
    adopt = keep_ok and better_ok
    lines += ["", f"- round 2 no worse than round 1 on L0-L2: **{keep_ok}**",
              f"- round 2 better than round 1 on C0/C1/D1 (no worse on all, better on at least one): **{better_ok}**",
              f"- **rebuild with round 2: {adopt}**", ""]
    for arm in ("round1", "round2"):
        nw = {lv: not_worse(arm, "base", lv) for lv in LEVELS}
        gain = {lv: (tb[(lv, arm)]["time_mean"] < tb[(lv, "base")]["time_mean"] - .5) or
                    (tb[(lv, arm)]["clear_p5"] > tb[(lv, "base")]["clear_p5"] + .005) for lv in LEVELS}
        lines.append(f"- {arm} vs base: not worse at {[lv for lv in LEVELS if nw[lv]]}, worse at "
                     f"{[lv for lv in LEVELS if not nw[lv]]}; accepted (rule) on these levels: {all(nw.values()) and any(gain.values())}")
    md = "\n".join(lines) + "\n"
    (a.out / "round2.md").write_text(md)
    print(md)
    print("ADOPT" if adopt else "KEEP")


if __name__ == "__main__":
    main()

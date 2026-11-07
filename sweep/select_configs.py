"""Pick Pareto configs from a sweep and write the next stage's config list.

    python3 select_configs.py results/coarse/runs.csv --configs results/coarse/configs.json \
        --refine 12 --per 8 --out results/refine_configs.json
    python3 select_configs.py results/refine/runs.csv ... --finalists 8 --out results/finalists.json

Objectives (per config, over the levels in the CSV):
    completion  mean completion % over levels (and never below the base at any level, when ranking)
    time        mean course time at the FlyX-like level L2 (finished runs)
    safety      mean of the per-level 5th-percentile wall clearance (p5_clear), and touches
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

import analyze
from run_sweep import RANGES

LEVEL_T = "L2"


def per_config(summary):
    cfgs = {}
    for (c, lv), m in summary.items():
        cfgs.setdefault(c, {})[lv] = m
    out = {}
    for c, by in cfgs.items():
        lv = sorted(by)
        out[c] = dict(
            levels=by,
            completion=float(np.mean([by[l]["completion"] for l in lv])),
            worst=float(min(by[l]["completion"] for l in lv)),
            time=by.get(LEVEL_T, by[lv[0]])["t_mean"],
            safety=float(np.mean([by[l]["p5_clear"] for l in lv])),
            touches=float(np.mean([by[l]["touches"] for l in lv])),
            vio=float(np.mean([by[l]["vio_events"] for l in lv])),
            n=sum(by[l]["n"] for l in lv))
    return out


def pareto(stats, keys=(("completion", 1), ("time", -1), ("safety", 1))):
    ids = [c for c in stats if np.isfinite(stats[c]["time"])]
    front = []
    for a in ids:
        va = [s * stats[a][k] for k, s in keys]
        dominated = False
        for b in ids:
            if b == a:
                continue
            vb = [s * stats[b][k] for k, s in keys]
            if all(x >= y for x, y in zip(vb, va)) and any(x > y for x, y in zip(vb, va)):
                dominated = True
                break
        if not dominated:
            front.append(a)
    return front


def beats_base(stats, c, base="base", tol=0.0):
    b = stats[base]["levels"]
    return all(stats[c]["levels"][lv]["completion"] >= b[lv]["completion"] - tol for lv in b if lv in stats[c]["levels"])


def jitter(knobs, rng, scale=.12):
    out = {}
    for k, (lo, hi) in RANGES.items():
        v = knobs.get(k)
        if v is None:
            import embedded
            v = embedded.BASE[k]
        v = float(np.clip(v + rng.normal(0, scale * (hi - lo)), lo, hi))
        out[k] = int(round(v)) if k == "hit_confirm" else round(v, 3)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+", type=Path)
    ap.add_argument("--configs", nargs="+", type=Path, required=True)
    ap.add_argument("--refine", type=int, default=0, help="take this many Pareto/ranked configs and jitter them")
    ap.add_argument("--per", type=int, default=8)
    ap.add_argument("--finalists", type=int, default=0)
    ap.add_argument("--tol", type=float, default=0.0, help="completion tolerance vs base (percentage points)")
    ap.add_argument("--out", type=Path)
    a = ap.parse_args(argv)
    known = {}
    for p in a.configs:
        for c in json.loads(p.read_text()):
            known[c["id"]] = c["knobs"]
    stats = per_config(analyze.summarise(analyze.load(a.csv)))
    front = pareto(stats)
    ok = [c for c in stats if beats_base(stats, c, tol=a.tol)]
    print(f"{len(stats)} configs, Pareto front {len(front)}, completion >= base everywhere: {len(ok)}")
    hdr = f"{'config':10s} {'compl':>6s} {'worst':>6s} {'t_L2':>6s} {'p5clr':>6s} {'touch':>6s} {'vio':>5s}  levels"
    rank = sorted(stats, key=lambda c: (-stats[c]["completion"], stats[c]["time"]))
    print(hdr)
    for c in ["base"] + [c for c in rank if c != "base"][:40]:
        s = stats[c]
        flag = ("P" if c in front else " ") + ("+" if c in ok else " ")
        print(f"{c:10s} {s['completion']:6.1f} {s['worst']:6.1f} {s['time']:6.1f} {s['safety']:6.3f} "
              f"{s['touches']:6.2f} {s['vio']:5.2f} {flag} " +
              " ".join(f"{lv}:{m['completion']:.0f}%/{m['t_mean']:.0f}s" for lv, m in sorted(s['levels'].items())))
    if not a.out:
        return
    rng = np.random.default_rng(7)
    if a.refine:
        # seeds of the next stage: the fastest/safest/most complete among those not worse than base,
        # plus the Pareto front
        pool = sorted(set(front) | set(ok), key=lambda c: (-stats[c]["completion"], stats[c]["time"]))
        by_time = sorted([c for c in pool if c in ok], key=lambda c: stats[c]["time"])
        by_safe = sorted([c for c in pool if c in ok], key=lambda c: -stats[c]["safety"])
        pick = []
        for lst in (pool, by_time, by_safe):
            for c in lst:
                if c not in pick and c != "base" and len(pick) < a.refine:
                    pick.append(c)
                    break
        for c in pool + by_time + by_safe:
            if len(pick) >= a.refine:
                break
            if c not in pick and c != "base":
                pick.append(c)
        out = [{"id": "base", "knobs": {}}]
        for c in pick:
            out.append({"id": c, "knobs": known[c]})
            for j in range(a.per):
                out.append({"id": f"{c}r{j}", "knobs": jitter(known[c] or {}, rng)})
        a.out.write_text(json.dumps(out, indent=1))
        print(f"wrote {len(out)} configs (refining {pick}) to {a.out}")
    if a.finalists:
        cand = [c for c in ok if c != "base"]
        fast = sorted(cand, key=lambda c: stats[c]["time"])
        safe = sorted(cand, key=lambda c: (-stats[c]["completion"], -stats[c]["safety"]))
        bal = sorted(cand, key=lambda c: -(stats[c]["completion"] / 100 - stats[c]["time"] / stats["base"]["time"]
                                           + 4 * (stats[c]["safety"] - stats["base"]["safety"])))
        pick = []
        for lst in (fast, safe, bal, fast[1:], safe[1:], bal[1:], fast[2:], safe[2:], bal[2:]):
            for c in lst:
                if c not in pick:
                    pick.append(c)
                    break
            if len(pick) >= a.finalists:
                break
        out = [{"id": "base", "knobs": {}}] + [{"id": c, "knobs": known[c]} for c in pick]
        a.out.write_text(json.dumps(out, indent=1))
        print(f"wrote finalists {pick} to {a.out}")


if __name__ == "__main__":
    main()

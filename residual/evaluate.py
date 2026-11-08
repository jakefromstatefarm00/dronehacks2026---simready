"""Base vs base+residual on held-out seeds, per curriculum level.

    python -m residual.evaluate --policy residual/policy.json --seeds 1000-1039 --workers 22
    python -m residual.evaluate --base-only --levels L0 L2 --seeds 1000-1011      # baseline only

Writes ``<out>/episodes.jsonl`` and ``<out>/results.md`` (the acceptance table).
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import sys
import time

import numpy as np

REPO = Path(__file__).resolve().parents[1]


def _init_worker():
    if str(REPO) not in sys.path:
        sys.path.insert(0, str(REPO))
    from residual import sim
    sim.base()
    sim.static_world()


def _run(task):
    from residual import sim
    level, seed, params, residual, *rest = task
    variant = rest[0] if rest else "proven"
    try:
        r = sim.run_episode(level, seed, params, residual=residual, variant=variant)
        d = asdict(r)
        d.pop("traj", None)
        d["fitness"] = r.fitness()
        return d
    except Exception as e:                      # a crash in the simulator is reported, never hidden
        import traceback
        return {"level": level, "seed": seed, "residual": residual, "variant": variant, "error": traceback.format_exc(),
                "complete": False, "contact": False, "fitness": -1e3}


_POOL = None


def pool(workers=None):
    global _POOL
    if _POOL is None:
        _POOL = mp.get_context("spawn").Pool(workers or max(1, os.cpu_count() - 2), initializer=_init_worker)
    return _POOL


def run_tasks(tasks, workers=None):
    return pool(workers).map(_run, tasks, chunksize=1)


def wilson(k, n, z=1.96):
    """95 % Wilson score interval for k successes in n trials."""
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def summarize(rows, by_variant=False):
    """Per (level, residual) metrics, or (level, variant, residual) with ``by_variant``.
    Clearance stats are over each run's minimum true clearance; time = NAVIGATE (sim s)."""
    out = {}
    for r in rows:
        key = (r["level"], r.get("variant", "proven"), r["residual"]) if by_variant else (r["level"], r["residual"])
        out.setdefault(key, []).append(r)
    table = {}
    for key, rs in sorted(out.items()):
        done = [r for r in rs if r.get("complete")]
        t = np.array([r["course_time"] for r in done]) if done else np.array([np.nan])
        mc = np.array([r["min_clearance"] for r in rs if r.get("min_clearance") is not None
                       and math.isfinite(r.get("min_clearance", float("nan")))])
        table[key] = {
            "n": len(rs), "errors": sum("error" in r for r in rs),
            "completion": len(done) / len(rs),
            "completion_ci": wilson(len(done), len(rs)),
            "contact_ci": wilson(sum(bool(r.get("contact")) for r in rs), len(rs)),
            "time_ci_half": float(1.96 * np.std(t, ddof=1) / math.sqrt(len(t))) if len(done) > 1 else float("nan"),
            "corner_yv_peak": float(np.mean([r.get("corner_yv_peak", 0) for r in rs])),
            "yv_excess": float(np.mean([r.get("yv_excess", 0) for r in rs])),
            "time_mean": float(np.nanmean(t)), "time_p90": float(np.nanpercentile(t, 90)) if done else float("nan"),
            "clear_min": float(mc.min()) if len(mc) else float("nan"),
            "clear_p5": float(np.percentile(mc, 5)) if len(mc) else float("nan"),
            "clear_median": float(np.median(mc)) if len(mc) else float("nan"),
            "below_0p30_s": float(np.mean([r.get("time_below_target", 0) for r in rs])),
            "contacts": sum(bool(r.get("contact")) for r in rs),
            "vio_risk": float(np.mean([r.get("vio_risk", 0) for r in rs])),
            "vio_events": int(sum(r.get("vio_events", 0) for r in rs)),
            "fitness": float(np.mean([r["fitness"] for r in rs])),
            "outcomes": {o: sum(r.get("outcome") == o for r in rs) for o in sorted({r.get("outcome", "error") for r in rs})},
        }
    return table


def accept(table, levels, variant=None):
    """Never worse than the base on completion or contacts at any level; better on time or clearance."""
    verdict, better = {}, False
    for lv in levels:
        k = (lambda arm: (lv, arm)) if variant is None else (lambda arm: (lv, variant, arm))
        b, r = table.get(k(False)), table.get(k(True))
        if not b or not r:
            continue
        ok = r["completion"] >= b["completion"] and r["contacts"] <= b["contacts"]
        faster = (r["time_mean"] < b["time_mean"] - .5) if math.isfinite(r["time_mean"]) and math.isfinite(b["time_mean"]) else False
        wider = r["clear_p5"] > b["clear_p5"] + .005
        better |= faster or wider
        verdict[lv] = {"not_worse": ok, "faster": faster, "wider": wider}
    return all(v["not_worse"] for v in verdict.values()) and better, verdict


def markdown(table, levels, verdicts=None, variants=("proven",)):
    from residual.sim import LEVEL
    head = ("| level | arm | runs | completion | time mean / p90 (s) | min clearance min / p5 (m) | time < 0.30 m (s) "
            "| contacts | VIO risk (exp. events) | VIO events |\n|---|---|---|---|---|---|---|---|---|---|")
    lines = [head]
    for lv in levels:
        for v in variants:
            for arm in (False, True):
                m = table.get((lv, v, arm))
                if not m:
                    continue
                name = f"{v} base" + (" + residual" if arm else "")
                lines.append(f"| {lv} | {name} | {m['n']} | {100 * m['completion']:.0f} % | "
                             f"{m['time_mean']:.1f} / {m['time_p90']:.1f} | {m['clear_min']:.3f} / {m['clear_p5']:.3f} | "
                             f"{m['below_0p30_s']:.1f} | {m['contacts']} | {m['vio_risk']:.3f} | {m['vio_events']} |")
    lines.append("")
    lines += [f"- **{lv}**: {LEVEL[lv].description}" for lv in levels]
    for name, (ok, verdict) in (verdicts or {}).items():
        lines += ["", f"**{name}: accepted = {ok}**"]
        lines += [f"- {lv}: not worse = {v['not_worse']}, faster = {v['faster']}, wider p5 clearance = {v['wider']}"
                  for lv, v in verdict.items()]
    return "\n".join(lines)


def parse_seeds(s):
    a, b = s.split("-")
    return list(range(int(a), int(b) + 1))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", type=Path, default=REPO / "residual" / "policy.json", help="residual for the proven base")
    ap.add_argument("--drift-policy", type=Path, default=REPO / "residual" / "policy_drift.json",
                    help="residual for the drift (map-alignment) base")
    ap.add_argument("--variants", nargs="+", default=["proven"], choices=["proven", "drift"])
    ap.add_argument("--levels", nargs="+", default=["L0", "L1", "L2", "L3", "L4", "L5"])
    ap.add_argument("--seeds", default="1000-1039", help="held-out seeds (training uses < 1000)")
    ap.add_argument("--workers", type=int, default=None)
    ap.add_argument("--base-only", action="store_true")
    ap.add_argument("--out", type=Path, default=REPO / "residual" / "results")
    a = ap.parse_args(argv)
    seeds = parse_seeds(a.seeds)
    tasks = []
    for v in a.variants:
        tasks += [(lv, s, None, False, v) for lv in a.levels for s in seeds]
        if not a.base_only:
            params = json.loads((a.policy if v == "proven" else a.drift_policy).read_text())
            tasks += [(lv, s, params, True, v) for lv in a.levels for s in seeds]
    t0 = time.time()
    rows = run_tasks(tasks, a.workers)
    a.out.mkdir(parents=True, exist_ok=True)
    with open(a.out / "episodes.jsonl", "w") as fh:
        for r in rows:
            fh.write(json.dumps(r, default=float) + "\n")
    table = summarize(rows, by_variant=True)
    verdicts = {} if a.base_only else {f"{v} residual vs {v} base": accept(table, a.levels, v) for v in a.variants}
    md = markdown(table, a.levels, verdicts, a.variants)
    md += f"\n\nseeds {a.seeds}, {len(rows)} episodes, {time.time() - t0:.0f} s wall\n"
    (a.out / "results.md").write_text(md)
    print(md)
    errs = [r for r in rows if "error" in r]
    if errs:
        print(errs[0]["error"])


if __name__ == "__main__":
    main()

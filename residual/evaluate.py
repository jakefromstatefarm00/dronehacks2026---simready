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
    level, seed, params, residual = task
    try:
        r = sim.run_episode(level, seed, params, residual=residual)
        d = asdict(r)
        d.pop("traj", None)
        d["fitness"] = r.fitness()
        return d
    except Exception as e:                      # a crash in the simulator is reported, never hidden
        import traceback
        return {"level": level, "seed": seed, "residual": residual, "error": traceback.format_exc(),
                "complete": False, "contact": False, "fitness": -1e3}


_POOL = None


def pool(workers=None):
    global _POOL
    if _POOL is None:
        _POOL = mp.get_context("spawn").Pool(workers or max(1, os.cpu_count() - 2), initializer=_init_worker)
    return _POOL


def run_tasks(tasks, workers=None):
    return pool(workers).map(_run, tasks, chunksize=1)


def summarize(rows):
    """Per (level, arm) metrics. Clearance stats are over each run's minimum true clearance."""
    out = {}
    for r in rows:
        out.setdefault((r["level"], r["residual"]), []).append(r)
    table = {}
    for (lv, arm), rs in sorted(out.items()):
        done = [r for r in rs if r.get("complete")]
        t = np.array([r["course_time"] for r in done]) if done else np.array([np.nan])
        mc = np.array([r["min_clearance"] for r in rs if r.get("min_clearance") is not None
                       and math.isfinite(r.get("min_clearance", float("nan")))])
        table[(lv, arm)] = {
            "n": len(rs), "errors": sum("error" in r for r in rs),
            "completion": len(done) / len(rs),
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


def accept(table, levels):
    """Never worse than the base on completion or contacts at any level; better on time or clearance."""
    verdict, better = {}, False
    for lv in levels:
        b, r = table.get((lv, False)), table.get((lv, True))
        if not b or not r:
            continue
        ok = r["completion"] >= b["completion"] and r["contacts"] <= b["contacts"]
        faster = (r["time_mean"] < b["time_mean"] - .5) if math.isfinite(r["time_mean"]) and math.isfinite(b["time_mean"]) else False
        wider = r["clear_p5"] > b["clear_p5"] + .005
        better |= faster or wider
        verdict[lv] = {"not_worse": ok, "faster": faster, "wider": wider}
    return all(v["not_worse"] for v in verdict.values()) and better, verdict


def markdown(table, levels, verdict=None):
    from residual.sim import LEVEL
    head = ("| level | arm | runs | completion | time mean / p90 (s) | min clearance min / p5 (m) | time < 0.30 m (s) "
            "| contacts | VIO risk (exp. events) | VIO events |\n|---|---|---|---|---|---|---|---|---|---|")
    lines = [head]
    for lv in levels:
        for arm in (False, True):
            m = table.get((lv, arm))
            if not m:
                continue
            lines.append(f"| {lv} | {'residual' if arm else 'base'} | {m['n']} | {100 * m['completion']:.0f} % | "
                         f"{m['time_mean']:.1f} / {m['time_p90']:.1f} | {m['clear_min']:.3f} / {m['clear_p5']:.3f} | "
                         f"{m['below_0p30_s']:.1f} | {m['contacts']} | {m['vio_risk']:.3f} | {m['vio_events']} |")
    lines.append("")
    lines += [f"- **{lv}**: {LEVEL[lv].description}" for lv in levels]
    if verdict:
        lines.append("")
        lines += [f"- {lv}: not worse = {v['not_worse']}, faster = {v['faster']}, wider p5 clearance = {v['wider']}"
                  for lv, v in verdict.items()]
    return "\n".join(lines)


def parse_seeds(s):
    a, b = s.split("-")
    return list(range(int(a), int(b) + 1))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", type=Path, default=REPO / "residual" / "policy.json")
    ap.add_argument("--levels", nargs="+", default=["L0", "L1", "L2", "L3", "L4", "L5"])
    ap.add_argument("--seeds", default="1000-1039", help="held-out seeds (training uses < 1000)")
    ap.add_argument("--workers", type=int, default=None)
    ap.add_argument("--base-only", action="store_true")
    ap.add_argument("--out", type=Path, default=REPO / "residual" / "results")
    a = ap.parse_args(argv)
    params = None if a.base_only else json.loads(a.policy.read_text())
    seeds = parse_seeds(a.seeds)
    tasks = [(lv, s, None, False) for lv in a.levels for s in seeds]
    if not a.base_only:
        tasks += [(lv, s, params, True) for lv in a.levels for s in seeds]
    t0 = time.time()
    rows = run_tasks(tasks, a.workers)
    a.out.mkdir(parents=True, exist_ok=True)
    with open(a.out / "episodes.jsonl", "w") as fh:
        for r in rows:
            fh.write(json.dumps(r, default=float) + "\n")
    table = summarize(rows)
    ok, verdict = accept(table, a.levels) if not a.base_only else (None, None)
    md = markdown(table, a.levels, verdict)
    if ok is not None:
        md += f"\n\n**Accepted: {ok}**"
    md += f"\n\nseeds {a.seeds}, {len(rows)} episodes, {time.time() - t0:.0f} s wall\n"
    (a.out / "results.md").write_text(md)
    print(md)
    errs = [r for r in rows if "error" in r]
    if errs:
        print(errs[0]["error"])


if __name__ == "__main__":
    main()

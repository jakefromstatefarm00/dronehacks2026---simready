"""Run A/B cells of ab_sim on a process pool (<= 4 workers), appending JSONL; resumable.

    python run_grid.py --trees <base>/pathfinding <drift>/pathfinding --plan grid.json --out results.jsonl

``--plan`` is a JSON list of Case dicts (see ab_sim.Case). Base cases run on the base tree,
drift/safe cases on the drift tree (each worker imports one tree only).
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def _init(tree):
    sys.path.insert(0, HERE)
    import ab_sim
    ab_sim.use_tree(tree)


def _run(cd):
    import ab_sim
    t0 = time.time()
    try:
        r = ab_sim.run_case(ab_sim.Case(**cd))
    except Exception as e:                       # keep going; record the failure
        import traceback
        r = dict(cd, error=repr(e), tb=traceback.format_exc()[-1500:])
    r["wall_s"] = round(time.time() - t0, 1)
    r["key"] = key(cd)
    return r


def key(cd):
    return json.dumps(cd, sort_keys=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trees", nargs=2, required=True, help="base and drift pathfinding/ dirs")
    ap.add_argument("--plan", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--workers", type=int, default=4)
    a = ap.parse_args()
    plan = json.load(open(a.plan))
    done = set()
    if os.path.exists(a.out):
        for line in open(a.out):
            try:
                r = json.loads(line)
                if "error" not in r:                 # failed runs are retried
                    done.add(r["key"])
            except (ValueError, KeyError):
                pass
    todo = [c for c in plan if key(c) not in done]
    print(f"{len(plan)} cases, {len(done)} done, {len(todo)} to run", flush=True)
    groups = [(a.trees[0], [c for c in todo if c.get("arm", "base") == "base"]),
              (a.trees[1], [c for c in todo if c.get("arm", "base") != "base"])]
    groups = [(t, sorted(cs, key=lambda c: c.get("seed", 0))) for t, cs in groups if cs]
    total = sum(len(cs) for _, cs in groups)
    nw = max(1, min(a.workers, 4))
    share = [max(1, round(nw * len(cs) / total)) for _, cs in groups]
    while sum(share) > nw and max(share) > 1:
        share[share.index(max(share))] -= 1
    import queue as _q
    results = _q.Queue()
    pools = []
    for (tree, cases), w in zip(groups, share):
        pool = mp.get_context("spawn").Pool(w, initializer=_init, initargs=(tree,), maxtasksperchild=40)
        pools.append(pool)
        for c in cases:
            pool.apply_async(_run, (c,), callback=results.put,
                             error_callback=lambda e: results.put({"error": repr(e)}))
    with open(a.out, "a") as fh:
        for i in range(total):
            r = results.get()
            fh.write(json.dumps(r) + "\n")
            fh.flush()
            print(f"[{i + 1}/{total}] {r.get('arm')} d={r.get('drift')} {r.get('variant')} {r.get('noise')} "
                  f"s={r.get('seed')} complete={r.get('complete')} contacts={r.get('contact_steps')} "
                  f"clear={r.get('min_clear')} align={r.get('align_total')} drift_end={r.get('drift_end')} "
                  f"resid={r.get('resid_end')} t={r.get('course_s')} wall={r.get('wall_s')} {r.get('error', '')}",
                  flush=True)
    for pool in pools:
        pool.close()
        pool.join()

if __name__ == "__main__":
    main()

"""Round 2: retrain the proven-base residual on the conditions where it lost runs.

    python -m residual.train_mix --init residual/policy.json --minutes 75 --workers 10

Same ES as train.py, but
* a FIXED condition mix instead of a level ladder (the >= 95 % gate cannot open at L2 / C / D,
  where the base itself completes 19-69 %): L0 10 %, L1 10 %, L2 20 %, C0 20 %, C1 20 %, D1 20 %;
* fitness v2 (sim.EpisodeResult.fitness): any contact = fixed -300, more than the whole time
  reward; yaw-rate x speed above 0.10 rad m/s^2 (the calibrated VIO divergence trigger) penalised;
* every ``gate_every`` iterations the mean policy flies ``gate_seeds`` (>= 32) held-out seeds
  (2000+) in each gate cell and is SAVED (gate_<it>.json) with its numbers, so the final choice
  is made on evidence, not on the last iterate.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from residual import evaluate                                  # noqa: E402
from residual.train import Adam, export, params_of             # noqa: E402

MIX = {"L0": .1, "L1": .1, "L2": .2, "C0": .2, "C1": .2, "D1": .2}
GATE_CELLS = ["L2", "C0", "C1", "D1"]


def summarize(rows):
    return evaluate.summarize(rows)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--init", type=Path, default=REPO / "residual" / "policy.json")
    ap.add_argument("--minutes", type=float, default=75)
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--pairs", type=int, default=10)
    ap.add_argument("--sigma", type=float, default=.2)
    ap.add_argument("--lr", type=float, default=.02)
    ap.add_argument("--gate-every", type=int, default=8)
    ap.add_argument("--gate-seeds", type=int, default=32)
    ap.add_argument("--out", type=Path, default=REPO / "residual" / "runs" / "round2")
    ap.add_argument("--seed", type=int, default=11)
    a = ap.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(a.seed)
    d = json.loads(a.init.read_text())
    theta = np.concatenate([np.r_[np.ravel(w), b] for w, b in d["layers"]])
    theta0 = theta.copy()
    opt = Adam(len(theta), a.lr)
    names, probs = list(MIX), np.array(list(MIX.values()))
    gate_seeds = list(range(2000, 2000 + a.gate_seeds))
    evaluate.pool(a.workers)
    log = open(a.out / "train_log.jsonl", "a")

    def gate(th, it, base_rows=[]):
        if not base_rows:
            base_rows.extend(evaluate.run_tasks([(c, s, None, False) for c in GATE_CELLS for s in gate_seeds], a.workers))
        rows = evaluate.run_tasks([(c, s, params_of(th), True) for c in GATE_CELLS for s in gate_seeds], a.workers)
        tb, tr = summarize(base_rows), summarize(rows)
        out = {c: {"base": {k: tb[(c, False)][k] for k in ("completion", "contacts", "time_mean", "clear_p5")},
                   "residual": {k: tr[(c, True)][k] for k in ("completion", "contacts", "time_mean", "clear_p5")}}
               for c in GATE_CELLS}
        export(th, a.out / f"gate_{it:03d}.json", {"round": 2, "it": it, "gate": out})
        return out

    rec = {"it": 0, "gate": gate(theta, 0)}                         # the starting policy, same seeds
    print(json.dumps(rec), flush=True)
    log.write(json.dumps(rec) + "\n")
    t_end = time.time() + 60 * a.minutes
    it = 0
    while time.time() < t_end:
        t0 = time.time()
        eps = rng.normal(0, 1, (a.pairs, len(theta)))
        lv = rng.choice(names, a.pairs, p=probs)
        seeds = rng.integers(0, 1000, a.pairs)
        tasks = []
        for k in range(a.pairs):
            tasks.append((str(lv[k]), int(seeds[k]), params_of(theta + a.sigma * eps[k]), True))
            tasks.append((str(lv[k]), int(seeds[k]), params_of(theta - a.sigma * eps[k]), True))
        rows = evaluate.run_tasks(tasks, a.workers)
        ok = np.array([("error" not in rows[2 * k]) and ("error" not in rows[2 * k + 1]) for k in range(a.pairs)])
        fp = np.array([rows[2 * k]["fitness"] for k in range(a.pairs)])[ok]
        fm = np.array([rows[2 * k + 1]["fitness"] for k in range(a.pairs)])[ok]
        if len(fp):
            scale = np.std(np.r_[fp, fm]) + 1e-6
            g = ((fp - fm)[:, None] * eps[ok]).sum(0) / (2 * len(fp) * a.sigma) / scale
            theta = theta + opt.step(g) - 1e-3 * a.lr * (theta - theta0)    # decay toward the round-1 policy
        it += 1
        rec = {"it": it, "levels": lv.tolist(), "f_mean": float(np.r_[fp, fm].mean()) if len(fp) else None,
               "complete": float(np.mean([r.get("complete", False) for r in rows])),
               "contacts": int(sum(bool(r.get("contact")) for r in rows)),
               "errors": int((~ok).sum()), "wall_s": round(time.time() - t0, 1)}
        if it % a.gate_every == 0:
            rec["gate"] = gate(theta, it)
        export(theta, a.out / "policy_last.json", {"round": 2, "it": it})
        print(json.dumps(rec), flush=True)
        log.write(json.dumps(rec) + "\n")
        log.flush()
    if it % a.gate_every:
        rec = {"it": it, "gate": gate(theta, it)}
        print(json.dumps(rec), flush=True)
        log.write(json.dumps(rec) + "\n")
    print("done", it, "iterations")


if __name__ == "__main__":
    main()

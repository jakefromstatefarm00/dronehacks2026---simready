"""Train the residual with a curriculum and evolution strategies (no torch needed at runtime).

    python -m residual.train --minutes 75 --workers 22

Why ES and not PPO: the policy is tiny (20 -> 16 -> 3, 387 numbers), the episode is a long
closed loop with a real planner thread in it, and what matters (completion, contact, course time)
arrives at the end. Antithetic ES on common random numbers (theta + eps and theta - eps fly the SAME
world, dynamics draw and event schedule) gives a low-variance estimate of exactly the episode
score, parallelises over CPU cores, and needs no value function. The output layer starts at zero,
so iteration 0 IS the base planner, and every update is a small step away from it.

Curriculum: start at L0; every ``gate_every`` iterations the mean policy flies the gate seeds
(2000+, never trained on) at the current level, and the level advances only when the residual
completes >= 95 % and >= the base on those seeds. Training batches mix in lower levels (25 %)
so earlier skills are kept. Seeds < 1000 train; 1000-1999 are the final acceptance set.
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

from residual import evaluate                      # noqa: E402
from residual.runtime import OBS_DIM, MLPPolicy     # noqa: E402
from residual.sim import LEVELS                     # noqa: E402

SIZES = [OBS_DIM, 16, 3]


def n_params(sizes=SIZES):
    return sum(i * o + o for i, o in zip(sizes[:-1], sizes[1:]))


def init_theta(rng, sizes=SIZES):
    parts = []
    for k, (i, o) in enumerate(zip(sizes[:-1], sizes[1:])):
        last = k == len(sizes) - 2
        parts += [np.zeros(i * o) if last else rng.normal(0, 1 / np.sqrt(i), i * o), np.zeros(o)]
    return np.concatenate(parts)


def export(theta, path, meta):
    pol = MLPPolicy.from_flat(theta, SIZES)
    d = {"format": "jolie-residual-mlp-v1", "obs_dim": OBS_DIM,
         "layers": [(w.tolist(), b.tolist()) for w, b in pol.layers], "meta": meta}
    Path(path).write_text(json.dumps(d))
    return d


def params_of(theta, meta=None):
    pol = MLPPolicy.from_flat(theta, SIZES)
    return {"format": "jolie-residual-mlp-v1", "obs_dim": OBS_DIM,
            "layers": [(w.tolist(), b.tolist()) for w, b in pol.layers], "meta": meta or {}}


class Adam:
    def __init__(self, n, lr):
        self.m, self.v, self.t, self.lr = np.zeros(n), np.zeros(n), 0, lr

    def step(self, g):
        self.t += 1
        self.m = .9 * self.m + .1 * g
        self.v = .999 * self.v + .001 * g * g
        mh, vh = self.m / (1 - .9 ** self.t), self.v / (1 - .999 ** self.t)
        return self.lr * mh / (np.sqrt(vh) + 1e-8)


def gate(theta, level, seeds, base_cache, workers):
    if level not in base_cache:
        rows = evaluate.run_tasks([(level, s, None, False) for s in seeds], workers)
        base_cache[level] = evaluate.summarize(rows)[(level, False)]
    rows = evaluate.run_tasks([(level, s, params_of(theta), True) for s in seeds], workers)
    return base_cache[level], evaluate.summarize(rows)[(level, True)]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--minutes", type=float, default=75)
    ap.add_argument("--workers", type=int, default=22)
    ap.add_argument("--pairs", type=int, default=11, help="antithetic pairs per iteration")
    ap.add_argument("--sigma", type=float, default=.25)
    ap.add_argument("--lr", type=float, default=.03)
    ap.add_argument("--gate-every", type=int, default=4)
    ap.add_argument("--gate-seeds", type=int, default=22)
    ap.add_argument("--start-level", default="L0")
    ap.add_argument("--init", type=Path, default=None, help="continue from an exported policy")
    ap.add_argument("--out", type=Path, default=REPO / "residual" / "runs" / time.strftime("train-%Y%m%d-%H%M%S"))
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(a.seed)
    theta = init_theta(rng)
    if a.init:
        d = json.loads(a.init.read_text())
        theta = np.concatenate([np.r_[np.ravel(w), b] for w, b in d["layers"]])
    opt = Adam(len(theta), a.lr)
    names = [lv.name for lv in LEVELS]
    li = names.index(a.start_level)
    gate_seeds = list(range(2000, 2000 + a.gate_seeds))
    base_cache, history = {}, []
    best = None
    t_end = time.time() + 60 * a.minutes
    evaluate.pool(a.workers)
    it = 0
    log = open(a.out / "train_log.jsonl", "a")
    while time.time() < t_end:
        t0 = time.time()
        eps = rng.normal(0, 1, (a.pairs, len(theta)))
        lvls = [names[li] if (rng.random() > .25 or li == 0) else names[int(rng.integers(0, li))] for _ in range(a.pairs)]
        seeds = rng.integers(0, 1000, a.pairs)
        tasks = []
        for k in range(a.pairs):
            tasks.append((lvls[k], int(seeds[k]), params_of(theta + a.sigma * eps[k]), True))
            tasks.append((lvls[k], int(seeds[k]), params_of(theta - a.sigma * eps[k]), True))
        rows = evaluate.run_tasks(tasks, a.workers)
        fp = np.array([rows[2 * k]["fitness"] for k in range(a.pairs)])
        fm = np.array([rows[2 * k + 1]["fitness"] for k in range(a.pairs)])
        diff = fp - fm
        scale = np.std(np.r_[fp, fm]) + 1e-6
        g = (diff[:, None] * eps).sum(0) / (2 * a.pairs * a.sigma) / scale
        theta = theta + opt.step(g) - 1e-3 * a.lr * theta        # small weight decay toward the base
        rec = {"it": it, "level": names[li], "f_mean": float(np.r_[fp, fm].mean()),
               "complete": float(np.mean([r.get("complete", False) for r in rows])),
               "contacts": int(sum(bool(r.get("contact")) for r in rows)),
               "errors": int(sum("error" in r for r in rows)), "wall_s": round(time.time() - t0, 1),
               "theta_norm": float(np.linalg.norm(theta))}
        if rec["errors"]:
            rec["error"] = next(r["error"] for r in rows if "error" in r)[-400:]
        it += 1
        if it % a.gate_every == 0:
            b, r = gate(theta, names[li], gate_seeds, base_cache, a.workers)
            rec["gate"] = {"base": {k: b[k] for k in ("completion", "time_mean", "clear_p5", "contacts")},
                           "residual": {k: r[k] for k in ("completion", "time_mean", "clear_p5", "contacts")}}
            passed = r["completion"] >= .95 and r["completion"] >= b["completion"] and r["contacts"] <= b["contacts"]
            rec["gate"]["passed"] = passed
            if passed:
                score = (li, r["completion"], -r["time_mean"] + 20 * r["clear_p5"])
                if best is None or score >= best[0]:
                    best = (score, theta.copy())
                    export(theta, a.out / "policy_best.json", {"level": names[li], "it": it, "gate": rec["gate"]})
                if li < len(names) - 1:
                    li += 1
                    rec["advanced_to"] = names[li]
        export(theta, a.out / "policy_last.json", {"level": names[li], "it": it})
        print(json.dumps(rec), flush=True)
        log.write(json.dumps(rec) + "\n")
        log.flush()
        history.append(rec)
    print("done", it, "iterations; reached", names[li])


if __name__ == "__main__":
    main()

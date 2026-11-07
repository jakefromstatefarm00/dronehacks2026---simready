"""Parallel sweep driver: configs x levels x seeds -> runs.csv (resumable, append-only).

    nice -n 10 python3 run_sweep.py coarse --n 160 --levels L0 L2 L3 --seeds 3 --workers 12 --out results/coarse
    nice -n 10 python3 run_sweep.py configs --configs results/finalists.json --levels L0 L1 L2 L3 L4 --seeds 50 ...
    nice -n 10 python3 run_sweep.py failmap --seeds 30 ...        # base config, one perturbation axis at a time

Every config sees the same (level, seed) worlds, dynamics draws, stalls, pauses and noise
streams (common random numbers), so differences between configs are not seed luck.
"""
from __future__ import annotations

import argparse
import csv
import json
import multiprocessing as mp
import os
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

# knob ranges (see README "Ranges"): (low, high), integer knobs rounded
RANGES = {
    "carrot_m": (.30, .90),
    "slow_turn_deg": (30., 120.),
    "min_carrot_frac": (.15, .70),
    "yaw_rate_max": (.15, .40),
    "turn_in_place_deg": (30., 100.),
    "lookahead_m": (.40, 1.20),
    "commit_margin": (.0, .30),
    "replan_every_m": (.25, 1.0),
    "turn_step_deg": (8., 30.),
    "tracking_margin": (.04, .14),
    "soft_margin": (.05, .30),
    "soft_weight": (1., 10.),
    "wall_snap_m": (.10, .35),
    "hit_confirm": (1.5, 6.49),
    "suspect_penalty": (2., 16.),
    "max_turn_rate": (.30, .80),
    "pad_centre_m": (.05, .15),
}
FIELDS = ["config", "level", "seed", "outcome", "why", "course_s", "checkpoints", "cleared", "min_wall", "min_net",
          "net_touches", "touches", "min_clear", "p5_clear", "contact_xy", "pad_err", "vio_events", "vio_losses", "fast_yaw_s", "fast_yaw_moving_s",
          "inplace_turn_s", "stall_s", "path_m", "escapes", "lookarounds", "planner_events", "plan_steps", "replans",
          "wall_s"]


def sobol_configs(n, seed=2026):
    from scipy.stats import qmc
    names = list(RANGES)
    x = qmc.Sobol(len(names), scramble=True, seed=seed).random(n)
    out = []
    for i, row in enumerate(x):
        knobs = {k: RANGES[k][0] + v * (RANGES[k][1] - RANGES[k][0]) for k, v in zip(names, row)}
        knobs["hit_confirm"] = int(round(knobs["hit_confirm"]))
        out.append({"id": f"s{i:03d}", "knobs": {k: round(v, 3) if isinstance(v, float) else v for k, v in knobs.items()}})
    return out


def _worker_init(cache_dir):
    try:
        os.nice(0)                      # inherit the launcher's nice level (started under nice -n 10)
    except OSError:
        pass
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    global CACHE
    CACHE = cache_dir


def _task(args):
    cid, knobs, level, seed, level_override = args
    import sim
    lv = sim.LEVELS[level] if level_override is None else replace(sim.LEVELS[level_override[0]], **level_override[1])
    try:
        r = sim.run(knobs, level, seed, cache_dir=CACHE, level=lv)
    except Exception as e:                     # a crash in the code under test is a result too
        import traceback
        r = {"level": lv.name, "seed": seed, "outcome": "error", "why": repr(e)[:200] + traceback.format_exc()[-300:]}
    r["config"] = cid
    r["level"] = lv.name
    return r


def _truth_task(args):
    level_name, override, seed = args
    import sim
    import embedded as E
    lv = sim.LEVELS[level_name] if override is None else replace(sim.LEVELS[override[0]], **override[1])
    mods = E.install()
    sim.truth_world(mods, lv, seed, CACHE)
    return level_name, seed


def run_tasks(tasks, out_dir, workers, cache_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "runs.csv"
    done = set()
    if path.exists():
        with open(path, newline="") as fh:
            for row in csv.DictReader(fh):
                done.add((row["config"], row["level"], int(row["seed"])))
    todo = [t for t in tasks if (t[0], t[2], t[3]) not in done]
    print(f"{len(tasks)} tasks, {len(tasks) - len(todo)} already done, {len(todo)} to run on {workers} workers",
          flush=True)
    if not todo:
        return path
    ctx = mp.get_context("fork" if sys.platform != "win32" else "spawn")
    # worlds first (each (level, seed) once), so workers never race to build the same one
    worlds = list({(t[2], t[3]): (t[2], t[4], t[3]) for t in todo}.values())
    with ctx.Pool(workers, initializer=_worker_init, initargs=(str(cache_dir),)) as pool:
        for _ in pool.imap_unordered(_truth_task, worlds):
            pass
    print(f"worlds ready ({len(worlds)})", flush=True)
    new = not path.exists()
    t0, n = time.time(), 0
    with open(path, "a", newline="") as fh, ctx.Pool(workers, initializer=_worker_init, initargs=(str(cache_dir),),
                                                    maxtasksperchild=40) as pool:
        w = csv.DictWriter(fh, fieldnames=FIELDS, extrasaction="ignore")
        if new:
            w.writeheader()
        for r in pool.imap_unordered(_task, todo, chunksize=1):
            w.writerow(r)
            fh.flush()
            n += 1
            if n % 25 == 0 or n == len(todo):
                el = time.time() - t0
                print(f"{n}/{len(todo)} runs, {el / 60:.1f} min, eta {(len(todo) - n) * el / n / 60:.1f} min", flush=True)
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["coarse", "configs", "failmap", "calib"])
    ap.add_argument("--n", type=int, default=128, help="coarse: Sobol configs (plus the base)")
    ap.add_argument("--configs", type=Path, help="configs: JSON list of {id, knobs}")
    ap.add_argument("--levels", nargs="+", default=["L0", "L2"])
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--seed0", type=int, default=0)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cache", type=Path, default=HERE / "cache")
    a = ap.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    seeds = range(a.seed0, a.seed0 + a.seeds)
    if a.mode == "coarse":
        configs = [{"id": "base", "knobs": {}}] + sobol_configs(a.n)
        tasks = [(c["id"], c["knobs"], lv, s, None) for s in seeds for lv in a.levels for c in configs]
    elif a.mode == "configs":
        configs = json.loads(a.configs.read_text())
        tasks = [(c["id"], c["knobs"], lv, s, None) for s in seeds for lv in a.levels for c in configs]
    else:
        import failmap
        configs = json.loads(a.configs.read_text()) if a.configs else [{"id": "base", "knobs": {}}]
        axes = failmap.AXES if a.mode == "failmap" else failmap.CALIB
        tasks = [(c["id"], c["knobs"], ov["name"], s, (ov.get("base", "L0"), {k: v for k, v in ov.items() if k != "base"}))
                 for s in seeds for ov in axes for c in configs]
    (a.out / "configs.json").write_text(json.dumps(configs, indent=1))
    run_tasks(tasks, a.out, a.workers, a.cache)


if __name__ == "__main__":
    main()

"""best-v5 A/B on identical seeds: Jolie's v4c (the FlyX 5/5 file) vs v5 candidate switches.

    python residual/eval_v5.py --arms v4c v5off v5 --levels L0 L2 C0 C1 T2 D1 --seeds 1000-1031 --workers 8 --out OUT.jsonl
    python residual/eval_v5.py --table OUT.jsonl

Extra metrics (truth used for scoring only, never by flight code):
  wall_yaw_rad    heading swept (rad) while slow (< 0.08 m/s) within 0.6 m of a true wall, NAVIGATE
  short_view_rad  heading swept (rad) while the camera's centre ray sees a true surface < 1.0 m away, NAVIGATE
  corner_view_min / corner_view_mean  centre-ray view (m) within 1 m of the first corridor-to-maze corner
  launch_turn_rad heading swept within 0.6 m of the launch mark during NAVIGATE (spin after takeoff)
  plan_ms_max / plan_ms_p95 / look_ms_max  planner step wall time (this laptop; FlyX is ~2-4x slower)
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
BASES_DIR = Path(os.environ.get("V5_BASES", REPO / "residual" / "bases"))

# arm -> (single file stem in BASES_DIR, planner options, MissionConfig overrides)
ARMS = {
    "v4c": ("upload_to_flyx_v4c", None, {}),
}


def load_arms():
    p = Path(os.environ.get("V5_ARMS", REPO / "residual" / "v5_arms.json"))
    if p.exists():
        for k, v in json.loads(p.read_text()).items():
            ARMS[k] = (v[0], v[1], v[2] if len(v) > 2 else {})


def parse_seeds(text):
    out = []
    for part in text.split(","):
        a, _, b = part.partition("-")
        out += list(range(int(a), int(b or a) + 1))
    return out


def _ray(occ, spec, xy, yaw, cap=3.5, step=.025):
    import numpy as np
    d = np.arange(.05, cap, step)
    pts = np.asarray(xy, float)[None] + d[:, None] * np.array([math.cos(yaw), math.sin(yaw)])[None]
    rc = np.floor((pts[:, ::-1] + spec.half_size) / spec.resolution).astype(int)
    inside = ((rc >= 0) & (rc < occ.shape[0])).all(-1)
    rc = np.clip(rc, 0, occ.shape[0] - 1)
    hit = occ[rc[:, 0], rc[:, 1]] | ~inside
    return float(d[int(np.argmax(hit))]) if hit.any() else cap


def _one(job):
    arm, level, seed = job
    sys.path.insert(0, str(REPO))
    import numpy as np
    from residual import sim
    load_arms()
    stem, popts, mkw = ARMS[arm]
    sim.BASES[stem] = BASES_DIR / f"{stem}.py"
    m = {"wall_yaw_rad": 0.0, "short_view_rad": 0.0, "corner_views": [], "launch_turn_rad": 0.0,
         "ms": [], "look_ms": [], "ev_t": [], "t": 0.0, "nav_t0": None, "takeoff_turn_rad": 0.0}
    ctx = {}

    def start(renderer, planner, mission, world):
        ctx.update(occ=world["truth_fine"], spec=world["fine"], planner=planner)
        inner = planner.update

        def update(xy, yaw, _inner=inner):
            n0 = len(planner.events)
            cmd = _inner(xy, yaw)
            m["ms"].append(planner.ms)
            if cmd.reason == "look around":
                m["look_ms"].append(planner.ms)
            for e in planner.events[n0:]:
                m["ev_t"].append((round(m["t"], 1), e.get("event"), e.get("at"), e.get("why", "")))
            return cmd
        planner.update = update

    def tick(t, mission, s, speed, c, dt):
        m["t"] = t
        w = abs(s.yaw_rate)
        if mission.state == "TAKEOFF":
            m["takeoff_turn_rad"] += w * dt
        if mission.state != "NAVIGATE":
            return
        if m["nav_t0"] is None:
            m["nav_t0"] = t
        if speed < .08 and c < .6:
            m["wall_yaw_rad"] += w * dt
        if np.linalg.norm(s.p[:2] - np.array([-2.515, .914])) < .6:
            m["launch_turn_rad"] += w * dt
        cam = s.p[:2] + .115 * np.array([math.cos(s.yaw), math.sin(s.yaw)])
        near_corner = np.linalg.norm(s.p[:2] - sim.VIO_CORNER_XY) < 1.0
        if w > .05 or near_corner:
            v = _ray(ctx["occ"], ctx["spec"], cam, s.yaw)
            if v < 1.0:
                m["short_view_rad"] += w * dt
            if near_corner:
                m["corner_views"].append(v)

    sim.START_HOOK, sim.TICK_HOOK = start, tick
    kw = dict(mkw)
    if popts:
        kw["_planner_options"] = popts
    t0 = time.perf_counter()
    r = sim.run_episode(level, seed, variant=stem, mission_kw=kw)
    row = {k: getattr(r, k) for k in ("level", "seed", "complete", "outcome", "course_time", "navigate_time", "cps_true",
                                      "contact", "min_clearance", "time_below_target", "vio_risk", "vio_events",
                                      "vio_losses", "yv_peak", "corner_yv_peak", "yv_excess", "note", "state_times")}
    ms = np.asarray(m["ms"] or [0.0])
    cv = m["corner_views"]
    row.update(arm=arm, turns=r.turns, wall=round(time.perf_counter() - t0, 1),
               wall_yaw_rad=round(m["wall_yaw_rad"], 3), short_view_rad=round(m["short_view_rad"], 3),
               corner_view_min=round(min(cv), 3) if cv else None,
               corner_view_mean=round(float(np.mean(cv)), 3) if cv else None,
               launch_turn_rad=round(m["launch_turn_rad"], 3), takeoff_turn_rad=round(m["takeoff_turn_rad"], 3),
               plan_ms_max=round(float(ms.max()), 1), plan_ms_p95=round(float(np.percentile(ms, 95)), 1),
               plan_ms_mean=round(float(ms.mean()), 1),
               look_ms_max=round(max(m["look_ms"]), 1) if m["look_ms"] else 0.0,
               look_ms_mean=round(float(np.mean(m["look_ms"])), 1) if m["look_ms"] else 0.0,
               events=m["ev_t"][:60])
    return row


def run(arms, levels, seeds, workers, out):
    jobs = [(a, lv, s) for lv in levels for s in seeds for a in arms]
    done = set()
    if out.exists():
        for line in out.read_text().splitlines():
            d = json.loads(line)
            done.add((d["arm"], d["level"], d["seed"]))
    jobs = [j for j in jobs if j not in done]
    out.parent.mkdir(parents=True, exist_ok=True)
    print(f"{len(jobs)} episodes on {workers} workers", flush=True)
    with ProcessPoolExecutor(workers) as ex, open(out, "a") as f:
        for i, row in enumerate(ex.map(_one, jobs, chunksize=1)):
            f.write(json.dumps(row, default=float) + "\n")
            f.flush()
            if i % 10 == 0:
                print(i, row["arm"], row["level"], row["seed"], row["outcome"], row["wall"], flush=True)


def wilson(k, n, z=1.96):
    if n == 0:
        return (math.nan, math.nan)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (100 * (c - h), 100 * (c + h))


def table(path, arms=None, levels=None):
    import numpy as np
    rows = [json.loads(l) for l in Path(path).read_text().splitlines() if l.strip()]
    arms = arms or list(dict.fromkeys(r["arm"] for r in rows))
    levels = levels or list(dict.fromkeys(r["level"] for r in rows))
    hdr = ["level", "arm", "n", "completion [95% Wilson]", "contacts", "VIO ev/loss", "NAVIGATE s (done)", "p5 clear m",
           "stops", "escapes", "near miss", "forgets", "look-arounds", "back-outs", "in-place yaw near walls rad",
           "short-view yaw rad", "corner view min/mean m", "peak yaw x speed (corner)", "launch spin rad",
           "plan ms p95/max", "look ms mean"]
    out = ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
    for lv in levels:
        for arm in arms:
            rs = [r for r in rows if r["level"] == lv and r["arm"] == arm]
            if not rs:
                continue
            n, k = len(rs), sum(r["complete"] for r in rs)
            lo, hi = wilson(k, n)
            nav = [r["course_time"] for r in rs if r["complete"]]
            clr = [r["min_clearance"] for r in rs if r["min_clearance"] == r["min_clearance"]]
            mean = lambda f: float(np.mean([f(r) for r in rs]))
            evc = lambda name: mean(lambda r: sum(1 for e in r["events"] if e[1] == name))
            cvm = [r["corner_view_min"] for r in rs if r["corner_view_min"] is not None]
            cva = [r["corner_view_mean"] for r in rs if r["corner_view_mean"] is not None]
            out.append("| " + " | ".join([
                lv, arm, str(n), f"{100 * k / n:.0f} % [{lo:.0f}, {hi:.0f}]", str(sum(r["contact"] for r in rs)),
                f"{sum(r['vio_events'] for r in rs)}/{sum(r.get('vio_losses', 0) for r in rs)}",
                f"{np.mean(nav):.1f} +/- {np.std(nav):.1f}" if nav else "-",
                f"{np.percentile(clr, 5):.3f}" if clr else "-",
                f"{mean(lambda r: r['turns'].get('stops', 0)):.1f}",
                f"{evc('escape'):.2f}", f"{evc('keep-out near miss'):.2f}", f"{evc('forget depth map'):.2f}",
                f"{evc('look around'):.2f}", f"{evc('back out to look') + evc('back out'):.2f}",
                f"{mean(lambda r: r['wall_yaw_rad']):.2f} (max {max(r['wall_yaw_rad'] for r in rs):.2f})",
                f"{mean(lambda r: r['short_view_rad']):.2f}",
                f"{np.mean(cvm):.2f} / {np.mean(cva):.2f}" if cvm else "-",
                f"{max(r['yv_peak'] for r in rs):.3f} ({max(r['corner_yv_peak'] for r in rs):.3f})",
                f"{mean(lambda r: r['launch_turn_rad']):.2f} (max {max(r['launch_turn_rad'] for r in rs):.2f})",
                f"{np.percentile([r['plan_ms_p95'] for r in rs], 50):.0f}/{max(r['plan_ms_max'] for r in rs):.0f}",
                f"{mean(lambda r: r['look_ms_mean']):.0f}",
            ]) + " |")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arms", nargs="+", default=["v4c"])
    ap.add_argument("--levels", nargs="+", default=["L0", "L2", "C0", "C1", "T2", "D1"])
    ap.add_argument("--seeds", default="1000-1031")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--out", type=Path, default=REPO / "residual" / "results" / "v5" / "episodes.jsonl")
    ap.add_argument("--table", type=Path, default=None)
    a = ap.parse_args()
    load_arms()
    if a.table:
        print(table(a.table, a.arms if a.arms != ["v4c"] else None, None))
        return
    run(a.arms, a.levels, parse_seeds(a.seeds), a.workers, a.out)


if __name__ == "__main__":
    main()

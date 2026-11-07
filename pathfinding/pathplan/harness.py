"""Batch evaluation of online planners over perturbed courses (offline, no flight).

Run from ``pathfinding/``:

    python -m pathplan.harness --variants 10 --output artifacts/pathplan-v1

For each variant the published layout is the drone's prior map and the perturbed
course is the hidden truth.  Every planner flies the same variants; results go to
``summary.json``, ``runs.csv`` and one PNG per variant.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
import time

import numpy as np

from .grid_search import astar
from .mission import MissionConfig, run_mission, time_estimate
from .sensor import DepthCamera
from .variants import KINDS, perturb
from .world import Envelope, GridSpec, checkpoints_from_route, cost_grid, occupancy

ROOT = Path(__file__).resolve().parents[1]          # pathfinding/
REPO = ROOT                                         # course/, config/ and drl/ live here
START_XY = np.array([-2.515, .914])        # launch mark
START_YAW = np.pi / 2                      # facing up the first corridor


def load_published(repo=REPO):
    sys.path.insert(0, str(repo))
    from drl.config import load_config
    _, course, route = load_config(str(repo / "config/course_v3.yaml"))
    return course, route


def static_route(cost, spec, env, checkpoints, start_xy, goal_beyond=.1):
    """A* through the checkpoints on one cost grid. Returns xy polyline or None if infeasible."""
    from .online import snap_goal
    cur = tuple(int(v) for v in spec.to_cell(start_xy))
    out = []
    for cp in checkpoints:
        goal = snap_goal(cost, spec, env, cp, goal_beyond)
        if goal is None:
            return None
        path, c, _ = astar(cost, cur, goal)
        if not path:
            return None
        out.extend(path if not out else path[1:])
        cur = goal
    return spec.to_xy(np.array(out))


def score_run(res, checkpoints):
    """Checkpoint points cleared (rule values), ignoring the speed terms that need other teams' times."""
    return int(sum(c["points"] for c in res.cleared))


def plot_variant(path, spec, prior, truth, results, checkpoints, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ext = [-spec.half_size, spec.half_size, -spec.half_size, spec.half_size]
    fig, axes = plt.subplots(1, len(results), figsize=(5.2 * len(results), 5.4), squeeze=False)
    for ax, res in zip(axes[0], results):
        img = np.ones(prior.shape + (3,))
        both = prior & truth
        img[both] = (.25, .25, .25)
        img[prior & ~truth] = (.95, .65, .65)       # mapped but not there (moved away)
        img[truth & ~prior] = (.30, .45, .95)       # there but not mapped (new / moved here)
        seen = res.belief != prior
        img[seen & res.belief] = (.05, .15, .70)    # discovered by the camera
        ax.imshow(img, origin="lower", extent=ext, interpolation="nearest")
        t = res.trajectory
        ax.plot(t[:, 0], t[:, 1], "-", color="tab:orange", lw=1.8)
        ax.plot(*START_XY, "go", ms=6)
        for cp in checkpoints:
            if cp.kind == "pad":
                ax.add_patch(plt.Circle(cp.centre, .762, fill=False, ls="--", color="green"))
            else:
                lat = np.array([-cp.normal[1], cp.normal[0]]) * cp.half_width
                a, b = cp.centre - lat, cp.centre + lat
                ax.plot([a[0], b[0]], [a[1], b[1]], "g--", lw=1)
        if res.collision_xy:
            ax.plot(*res.collision_xy, "rx", ms=12, mew=3)
        ax.set_title(f"{res.planner}: {res.outcome}, {len(res.cleared)}/{len(checkpoints)} cp, "
                     f"{res.length:.1f} m", fontsize=10)
        ax.set_xticks([]), ax.set_yticks([])
    fig.suptitle(title + "   grey=mapped+real  pink=mapped, gone  blue=real, unmapped  navy=seen", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variants", type=int, default=4, help="variants per kind (nominal always 1)")
    # "boulders" is off by default: at the 0.9 m default altitude the boulders (0.53 m) are below the slab
    ap.add_argument("--kinds", nargs="+", default=["nominal", "walls", "clutter", "mixed"], choices=KINDS)
    ap.add_argument("--planners", nargs="+", default=["static", "astar", "theta"],
                    choices=list(MissionConfig.PLANNERS))
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--altitude", type=float, default=.9)
    ap.add_argument("--resolution", type=float, default=.05)
    ap.add_argument("--chicane", choices=["window", "gate_b"], default="window")
    ap.add_argument("--dropout", type=float, default=0.0, help="fraction of depth rays lost (textureless)")
    ap.add_argument("--max-range", type=float, default=3.5)
    ap.add_argument("--lookahead", type=float, default=.7, help="camera aims this far ahead on the plan; 0 = along motion")
    ap.add_argument("--suspect-penalty", type=float, default=8.0)
    ap.add_argument("--output", type=Path, default=ROOT / "artifacts/pathplan-v1")
    ap.add_argument("--no-plots", action="store_true")
    a = ap.parse_args(argv)

    course, route = load_published()
    spec, env = GridSpec(resolution=a.resolution), Envelope()
    slab = env.half_height + env.tracking_margin
    checkpoints = checkpoints_from_route(route, course, chicane=a.chicane)
    prior = occupancy(course, spec, altitude=a.altitude, half_height=slab)
    prior_cost = cost_grid(prior, spec, env)
    nominal = static_route(prior_cost, spec, env, checkpoints, START_XY)
    if nominal is None:
        raise SystemExit("published layout has no route through the checkpoints at this altitude")
    keep_clear = [(START_XY, .8), (np.asarray(route.centre[route.names.index("pad_hover")][:2]), .8)]
    keep_clear += [(cp.centre, .45) for cp in checkpoints]

    rng = np.random.default_rng(a.seed)
    a.output.mkdir(parents=True, exist_ok=True)
    rows, variants = [], []
    for kind in a.kinds:
        count = 1 if kind == "nominal" else a.variants
        made = 0
        tries = 0
        while made < count and tries < count * 20:
            tries += 1
            vcourse, info = perturb(course, kind, rng, route_xy=nominal, keep_clear=keep_clear)
            truth = occupancy(vcourse, spec, altitude=a.altitude, half_height=slab)
            if static_route(cost_grid(truth, spec, env), spec, env, checkpoints, START_XY) is None:
                continue                       # this variant has no legal route at all; draw another
            vid = f"{kind}-{made:02d}"
            made += 1
            info["id"] = vid
            variants.append(info)
            results = []
            for planner in a.planners:
                cfg = MissionConfig(planner=planner, lookahead_m=a.lookahead, suspect_penalty=a.suspect_penalty,
                                    camera=DepthCamera(dropout=a.dropout, max_range=a.max_range))
                t0 = time.perf_counter()
                res = run_mission(prior, truth, spec, env, checkpoints, START_XY, START_YAW, cfg,
                                  np.random.default_rng(a.seed + made))
                wall = time.perf_counter() - t0
                results.append(res)
                pt = np.asarray(res.plan_time_s) if res.plan_time_s else np.zeros(1)
                rows.append({
                    "variant": vid, "kind": kind, "planner": planner, "outcome": res.outcome,
                    "checkpoints": len(res.cleared), "checkpoint_points": score_run(res, checkpoints),
                    "chicane_via": next((c["via"] for c in res.cleared if c["name"].startswith("4")), ""),
                    "length_m": round(res.length, 3),
                    # path time at fast-f5 limits + in-place yaw at omega_max 0.45 rad/s
                    "time_est_s": round(time_estimate(res.trajectory) + res.turn_in_place_rad / .45, 2)
                    if res.outcome == "finished" else "",
                    "turn_in_place_deg": round(float(np.degrees(res.turn_in_place_rad)), 1),
                    "min_true_clearance_m": round(res.min_true_clearance, 3),
                    "keepout_violations": res.keepout_violations, "replans": res.replans,
                    "plan_calls": res.plan_calls, "expansions": res.expansions, "escapes": res.escapes,
                    "plan_ms_total": round(1e3 * pt.sum(), 1), "plan_ms_max": round(1e3 * pt.max(), 2),
                    "wall_s": round(wall, 2),
                })
                print(f"{vid:12s} {planner:7s} {res.outcome:9s} cp={len(res.cleared)} "
                      f"len={res.length:5.2f} clear={res.min_true_clearance:.3f} replans={res.replans:3d} "
                      f"plan_max={1e3 * pt.max():6.1f}ms", flush=True)
            if not a.no_plots:
                plot_variant(a.output / f"{vid}.png", spec, prior, truth, results, checkpoints,
                             f"{vid}  (prior = published layout, truth = variant)")

    with open(a.output / "runs.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    summary = {"kind": "offline_online_planning_benchmark_not_flight",
               "settings": {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(a).items()},
               "envelope": env.__dict__, "grid_resolution_m": spec.resolution,
               "checkpoints": [cp.name for cp in checkpoints], "variants": variants, "by_planner": {}}
    for planner in a.planners:
        rs = [r for r in rows if r["planner"] == planner]
        fin = [r for r in rs if r["outcome"] == "finished"]
        summary["by_planner"][planner] = {
            "runs": len(rs), "finished": len(fin), "crashed": sum(r["outcome"] == "crashed" for r in rs),
            "mean_checkpoint_points": round(float(np.mean([r["checkpoint_points"] for r in rs])), 2),
            "mean_length_m_finished": round(float(np.mean([r["length_m"] for r in fin])), 3) if fin else None,
            "mean_time_est_s_finished": round(float(np.mean([r["time_est_s"] for r in fin])), 2) if fin else None,
            "worst_plan_ms": max(r["plan_ms_max"] for r in rs),
            "total_expansions": int(sum(r["expansions"] for r in rs)),
        }
    (a.output / "summary.json").write_text(json.dumps(summary, indent=2, default=float))
    print(json.dumps(summary["by_planner"], indent=2))


if __name__ == "__main__":
    main()

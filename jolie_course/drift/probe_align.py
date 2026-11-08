"""Single-frame probe of ``align_to_map`` failure modes (no flight).

For poses every 0.25 m along the nominal route, render depth from the TRUE pose on a (possibly
perturbed) course, project it from the REPORTED pose (true + known error), and compare the
measured shift with the true correction (-error). Reports, per scenario: frames that returned a
shift, the mean/95th-percentile/max error of the shift against the right answer, and the
"false pull" (shift magnitude when the error is zero).

    python probe_align.py <drift-branch>/pathfinding [--safe '{"robust": .08}']
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ab_sim  # noqa: E402


def scenarios():
    yield "zero error, published", dict(err=(0, 0))
    yield "zero error, noisy depth", dict(err=(0, 0), noise="noisy")
    yield "zero error, bad depth", dict(err=(0, 0), noise="bad")
    yield "zero error, walls +-0.15", dict(err=(0, 0), variant="walls")
    yield "zero error, walls +-0.30", dict(err=(0, 0), variant="walls_big")
    yield "zero error, clutter", dict(err=(0, 0), variant="clutter")
    yield "zero error, mount +5 cm fwd", dict(err=(0, 0), mount=(.05, 0, 0))
    yield "zero error, mount 3 deg yaw", dict(err=(0, 0), mount=(0, 0, 3))
    yield "zero error, heading 2 deg", dict(err=(0, 0), dpsi=2)
    for e in (.05, .1, .2, .3):
        yield f"error {e:.2f} m random dir, published", dict(err=e)
        yield f"error {e:.2f} m random dir, walls +-0.15", dict(err=e, variant="walls")
    yield "error 0.10 m, noisy depth", dict(err=.1, noise="noisy")
    yield "error 0.10 m, clutter", dict(err=.1, variant="clutter")


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("tree")
    ap.add_argument("--safe", default="")
    ap.add_argument("--seeds", type=int, default=6)
    a = ap.parse_args()
    ab_sim.use_tree(a.tree)
    from pathplan.depth_projection import (CameraMount, Intrinsics, align_to_map, decode_depth_image,
                                           map_distance_field, project_depth, body_to_course)
    if a.safe:
        from align_safe import make_safe_align
        align_to_map = make_safe_align(json.loads(a.safe))
    P = ab_sim._published()
    spec, env, prior = P["spec"], P["env"], P["prior"]
    dist, grad = map_distance_field(prior, spec)
    route = np.asarray(P["nominal"])
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(route, axis=0), axis=1))]
    q = np.arange(.5, s[-1] - .5, .25)
    poses = np.column_stack([np.interp(q, s, route[:, 0]), np.interp(q, s, route[:, 1])])
    tang = np.gradient(poses, axis=0)
    yaws = np.arctan2(tang[:, 1], tang[:, 0])
    intr = Intrinsics.from_k(ab_sim.K, ab_sim.W, ab_sim.H)
    out = {}
    for name, sc in scenarios():
        errs, pulls, used, n, worse = [], [], 0, 0, 0
        for seed in range(a.seeds):
            occ, _, _ = ab_sim.make_truth(sc.get("variant", "nominal"), seed)
            rng = np.random.default_rng(seed)
            for i, (p, y) in enumerate(zip(poses, yaws)):
                if i % a.seeds != seed % a.seeds and sc.get("variant", "nominal") == "nominal" \
                        and sc.get("noise", "clean") == "clean" and not isinstance(sc["err"], float):
                    continue                                   # deterministic: each pose once
                n += 1
                e = sc["err"]
                if isinstance(e, float):
                    th = rng.uniform(0, 2 * np.pi)
                    e = e * np.array([math.cos(th), math.sin(th)])
                e = np.asarray(e, float)
                mdx, mdy, myaw = sc.get("mount", (0, 0, 0))
                cam = p + ab_sim._rot(y) @ np.array([.115 + mdx, mdy])
                img = ab_sim.render_depth(occ, spec, cam, y + math.radians(myaw), .9, rng, sc.get("noise", "clean"))
                depth = decode_depth_image(img.tobytes(), ab_sim.W, ab_sim.H, "16UC1", ab_sim.W * 2)
                rep_yaw = y + math.radians(sc.get("dpsi", 0))
                pts = []
                project_depth(depth, intr, np.array([*(p + e), .9]), body_to_course(rep_yaw), spec, altitude=.9,
                              half_height=env.half_height + env.tracking_margin, mount=CameraMount(),
                              expected=prior, hit_points=pts)
                shift, info = align_to_map(pts[0] if pts else None, dist, grad, spec)
                if shift is None:
                    continue
                used += 1
                errs.append(float(np.linalg.norm(shift + e)))          # error left after applying it
                pulls.append(float(np.linalg.norm(shift)))
                worse += int(np.linalg.norm(shift + e) > np.linalg.norm(e) + .01)   # made it worse by > 1 cm
        errs = np.asarray(errs) if errs else np.zeros(1)
        out[name] = dict(frames=n, with_shift=used, err_mean=round(float(errs.mean()), 3),
                         err_p95=round(float(np.percentile(errs, 95)), 3), err_max=round(float(errs.max()), 3),
                         pull_mean=round(float(np.mean(pulls)) if pulls else 0.0, 3), worse=worse)
        print(f"{name:38s} frames {n:4d} shift {used:4d}  |error after| mean {out[name]['err_mean']:.3f} "
              f"p95 {out[name]['err_p95']:.3f} max {out[name]['err_max']:.3f}  |shift| mean {out[name]['pull_mean']:.3f} worse>1cm {worse}",
              flush=True)
    print(json.dumps(out))


if __name__ == "__main__":
    main()

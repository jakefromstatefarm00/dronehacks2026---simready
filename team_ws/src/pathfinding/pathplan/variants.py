"""Perturbed copies of the published course, for generality testing.

The organisers say the layout "may change slightly before judging", and the
generality score asks how the software would do on a different course.  Each
variant is a modified copy of the teammate ``drl.course.Course`` primitives, so
the same collision model serves as the hidden "true" world.

Kinds
-----
nominal   the published layout unchanged (prior == truth)
boulders  every boulder moved by up to ``boulder_sigma`` (Gaussian, clipped)
walls     maze dividers, chicane and approach walls shifted along their normal
          and along their length by up to ``wall_shift``
clutter   1-3 unmapped vertical obstacles dropped near the nominal route
mixed     all of the above

The barrier, the tunnel and the window wall are not moved: they define the
checkpoints, and the organisers would republish a change to those.
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

KINDS = ("nominal", "boulders", "walls", "clutter", "mixed")
MOVABLE_WALLS = ("M1", "M2", "M3", "C1", "C2b", "A1")


def _copy(course):
    return replace(course, box_c=course.box_c.copy(), box_u=course.box_u.copy(), box_h=course.box_h.copy(),
                   box_names=list(course.box_names), cyl_xy=course.cyl_xy.copy(), cyl_r=course.cyl_r.copy(),
                   cyl_top=course.cyl_top.copy(), tube=dict(course.tube) if course.tube else None,
                   groups=dict(course.groups))


def perturb(course, kind: str, rng: np.random.Generator, *, route_xy=None, keep_clear=(),
            boulder_sigma=.25, wall_shift=.15, clutter_radius=(.12, .22), clutter_count=(1, 3)):
    """Return (new_course, description dict). ``keep_clear``: [(xy, radius)] where clutter may not go."""
    if kind not in KINDS:
        raise ValueError(f"unknown variant kind {kind!r}; choose from {KINDS}")
    c = _copy(course)
    info = {"kind": kind, "changes": []}
    lim = course.half_size - .3

    if kind in ("boulders", "mixed") and len(c.cyl_xy):
        d = np.clip(rng.normal(0, boulder_sigma, c.cyl_xy.shape), -2 * boulder_sigma, 2 * boulder_sigma)
        c.cyl_xy = np.clip(c.cyl_xy + d, -lim, lim)
        info["changes"].append({"boulders_moved_m": np.linalg.norm(d, axis=1).round(3).tolist()})

    if kind in ("walls", "mixed"):
        for i, name in enumerate(c.box_names):
            if not name.startswith(MOVABLE_WALLS):
                continue
            u = c.box_u[i]
            n = np.array([-u[1], u[0]])
            a, b = rng.uniform(-wall_shift, wall_shift, 2)
            c.box_c[i, :2] += a * u + b * n
            info["changes"].append({"wall": name, "along_m": round(float(a), 3), "across_m": round(float(b), 3)})

    if kind in ("clutter", "mixed"):
        if route_xy is None or len(route_xy) < 2:
            raise ValueError("clutter variants need route_xy (the nominal route) to place obstacles near")
        route_xy = np.asarray(route_xy)
        count = int(rng.integers(clutter_count[0], clutter_count[1] + 1))
        new_xy, new_r = [], []
        tries = 0
        while len(new_xy) < count and tries < 200:
            tries += 1
            k = int(rng.integers(len(route_xy) - 1))
            p = route_xy[k]
            t = route_xy[k + 1] - route_xy[k]
            nrm = np.array([-t[1], t[0]]) / (np.linalg.norm(t) + 1e-9)
            p = p + nrm * rng.uniform(-.25, .25)
            if any(np.linalg.norm(p - np.asarray(q)) < r for q, r in keep_clear):
                continue
            new_xy.append(p)
            new_r.append(rng.uniform(*clutter_radius))
        if new_xy:
            c.cyl_xy = np.vstack([c.cyl_xy, new_xy])
            c.cyl_r = np.r_[c.cyl_r, new_r]
            c.cyl_top = np.r_[c.cyl_top, np.full(len(new_xy), 1.6)]
            info["changes"].append({"clutter_xy": np.round(new_xy, 3).tolist(), "clutter_r": np.round(new_r, 3).tolist()})
    return c, info

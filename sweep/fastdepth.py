"""A faster drop-in for the embedded depth_projection.project_depth with IDENTICAL output.

The flown function spends ~90 % of its time in np.unique(axis=0) (a structured sort of ~10^5
rows). Marking cells in a boolean grid and reading them back with argwhere yields the same cells
in the same (row-major, lexicographic) order. ``check()`` asserts equality against the original
on rendered frames; sim.py installs this only after that check passes in each worker.
"""
from __future__ import annotations

import numpy as np


def make_fast(dp):
    """dp: the embedded pathplan.depth_projection module. Returns project_depth_fast."""
    CameraMount = dp.CameraMount

    def cells_of(rc, n):
        mark = np.zeros((n, n), bool)
        mark[rc[:, 0], rc[:, 1]] = True
        return np.argwhere(mark), mark

    def project_depth(depth_m, intr, position, rotation, spec, *, altitude=.9, half_height=.21,
                      mount=CameraMount(), stride=8, min_range=.35, max_range=3.5,
                      occlusion_depth=.4, expected=None):
        empty = np.empty((0, 2), int)
        n = spec.n
        d = np.asarray(depth_m, float)[::stride, ::stride]
        v, u = np.mgrid[0:d.shape[0], 0:d.shape[1]]
        u, v, z = u.ravel() * stride, v.ravel() * stride, d.ravel()
        ok = np.isfinite(z) & (z >= min_range) & (z <= max_range)
        if not ok.any():
            return empty, empty, empty
        u, v, z = u[ok], v[ok], z[ok]
        pts_cam = np.column_stack([(u - intr.cx) / intr.fx * z, (v - intr.cy) / intr.fy * z, z])
        r_cam = np.asarray(rotation) @ mount.rotation()
        origin = np.asarray(position, float) + np.asarray(rotation) @ np.asarray(mount.offset, float)
        ends = origin + pts_cam @ r_cam.T

        step = spec.resolution / 2
        length = np.linalg.norm(ends - origin, axis=1)
        t = np.arange(0.0, length.max() + step, step)
        # Only samples that can lie in the slab are evaluated: per ray, the t-index window where
        # the (linear) height is within the slab, widened by 2 samples each side; inside it the
        # flown expressions are evaluated exactly, so the kept samples are bit-identical.
        dvec = ends - origin
        dz = dvec[:, 2]
        lo_z, hi_z = altitude - half_height - origin[2], altitude + half_height - origin[2]
        with np.errstate(divide="ignore", invalid="ignore"):
            fa, fb = lo_z / dz, hi_z / dz
        f_lo = np.where(dz == 0, np.where((lo_z <= 0) & (0 <= hi_z), 0.0, 2.0), np.minimum(fa, fb))
        f_hi = np.where(dz == 0, np.where((lo_z <= 0) & (0 <= hi_z), 1.0, -1.0), np.maximum(fa, fb))
        f_lo, f_hi = np.clip(f_lo, 0, 1), np.clip(f_hi, -1, 1)
        i0 = np.maximum(np.floor(f_lo * length / step).astype(int) - 2, 0)
        i1 = np.minimum(np.ceil(np.minimum(f_hi, 1.0) * length / step).astype(int) + 2, len(t) - 1)
        i1 = np.where(f_hi < f_lo, i0 - 1, i1)
        cnt = np.maximum(i1 - i0 + 1, 0)
        ray = np.repeat(np.arange(len(length)), cnt)
        start = np.repeat(i0 - np.r_[0, np.cumsum(cnt)[:-1]], cnt)
        ti = np.arange(cnt.sum()) + start
        tt = t[ti]
        frac = np.minimum(tt / length[ray], 1.0)
        samples = origin + frac[:, None] * dvec[ray]
        before_end = tt < length[ray] - step
        in_slab = np.abs(samples[:, 2] - altitude) <= half_height
        inside = ((samples[:, :2] > -spec.half_size) & (samples[:, :2] < spec.half_size)).all(-1)
        free_mask = before_end & in_slab & inside
        rc_free = spec.to_cell(samples[free_mask][:, :2])
        end_in_slab = (np.abs(ends[:, 2] - altitude) <= half_height) & \
            ((ends[:, :2] > -spec.half_size) & (ends[:, :2] < spec.half_size)).all(-1)
        hit_rc = spec.to_cell(ends[end_in_slab, :2])
        if len(rc_free):
            free, fmark = cells_of(rc_free, n)
        else:
            free, fmark = empty, None
        if len(hit_rc):
            hit, hmark = cells_of(hit_rc, n)
        else:
            hit, hmark = empty, None
        if len(free) and len(hit):
            free = np.argwhere(fmark & ~hmark)
        behind = empty
        if len(hit_rc) and occlusion_depth > 0:
            e = ends[end_in_slab, :2]
            o = origin[:2]
            unexpected = np.ones(len(e), bool) if expected is None else ~expected[hit_rc[:, 0], hit_rc[:, 1]]
            if unexpected.any():
                e = e[unexpected]
                dirs = (e - o) / np.maximum(np.linalg.norm(e - o, axis=1, keepdims=True), 1e-9)
                k = np.arange(1, int(round(occlusion_depth / step)) + 1) * step
                pts = e[:, None, :] + k[None, :, None] * dirs[:, None, :]
                pts = pts.reshape(-1, 2)
                pts = pts[((pts > -spec.half_size) & (pts < spec.half_size)).all(-1)]
                if len(pts):
                    behind, _ = cells_of(spec.to_cell(pts), n)
        return free.astype(int), hit.astype(int), behind.astype(int)

    return project_depth


def same(a, b):
    return all(np.array_equal(x, y) for x, y in zip(a, b))

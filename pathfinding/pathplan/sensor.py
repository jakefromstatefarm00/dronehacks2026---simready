"""Forward depth camera as a 2-D ray caster on the true occupancy slice.

Stands in for the OAK-D S2 stereo depth projected onto the flight slab:
horizontal field of view ``fov_deg`` centred on the camera yaw, valid depth from
``min_range`` (stereo minimum ~0.35 m) to ``max_range``.  Each ray marks the cells
it passes as free and the first occupied cell as occupied; cells closer than
``min_range`` are not observed at all (stereo cannot see them).

Optional ``dropout`` randomly discards whole rays, a crude stand-in for
textureless surfaces where stereo returns no depth.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class DepthCamera:
    fov_deg: float = 72.0          # OAK-D S2 mono HFOV
    min_range: float = .35
    max_range: float = 3.5
    rays: int = 145
    dropout: float = 0.0

    occlusion_depth: float = .4    # how far behind an unexpected hit to suspect more obstacle

    def observe(self, truth_occ: np.ndarray, spec, xy, yaw, rng=None, expected=None):
        """Return (free_cells, occupied_cells, behind_cells) as (K, 2) int arrays of (row, col).

        ``behind_cells``: the cells up to ``occlusion_depth`` past each hit that was not
        expected (``expected`` = prior occupancy; None means every hit is unexpected).
        The camera saw only the obstacle's near face; these are where the rest of it may be.
        """
        half = np.deg2rad(self.fov_deg) / 2
        angles = yaw + np.linspace(-half, half, self.rays)
        if self.dropout and rng is not None:
            angles = angles[rng.random(len(angles)) >= self.dropout]
        step = spec.resolution / 2
        t = np.arange(self.min_range, self.max_range + 1e-9, step)
        dirs = np.column_stack([np.cos(angles), np.sin(angles)])
        pts = np.asarray(xy)[None, None, :] + t[None, :, None] * dirs[:, None, :]   # rays, steps, 2
        rc = np.floor((pts[..., ::-1] + spec.half_size) / spec.resolution).astype(int)
        inside = ((rc >= 0) & (rc < spec.n)).all(-1)
        rc = np.clip(rc, 0, spec.n - 1)
        occ = truth_occ[rc[..., 0], rc[..., 1]] & inside
        # a ray stops at the first occupied sample or when it leaves the grid
        stop = occ | ~inside
        first = np.where(stop.any(1), stop.argmax(1), len(t))
        idx = np.arange(len(t))[None, :]
        free_mask = idx < first[:, None]
        hit_mask = (idx == first[:, None]) & occ
        empty = np.empty((0, 2), int)
        free = np.unique(rc[free_mask], axis=0) if free_mask.any() else empty
        hit = np.unique(rc[hit_mask], axis=0) if hit_mask.any() else empty
        hit_ray = occ[np.arange(len(first)), np.minimum(first, len(t) - 1)] & (first < len(t))
        if expected is not None and hit_ray.any():
            hr = rc[np.arange(len(first)), np.minimum(first, len(t) - 1)]
            hit_ray &= ~expected[hr[:, 0], hr[:, 1]]
        depth = int(round(self.occlusion_depth / step))
        behind_mask = hit_ray[:, None] & (idx > first[:, None]) & (idx <= first[:, None] + depth) & inside
        behind = np.unique(rc[behind_mask], axis=0) if behind_mask.any() else empty
        return free, hit, behind

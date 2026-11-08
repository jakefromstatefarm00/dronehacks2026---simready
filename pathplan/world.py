"""Course -> occupancy, clearance and planning-cost grids at one flight altitude.

The planner works in a horizontal slice at ``altitude``: a cell is occupied when
any obstacle intersects the drone's vertical slab ``altitude +- half_height`` at
that xy.  Flying a fixed altitude below the walls' top (1.727 m) satisfies the
"stay below the walls" rule by construction; the window (z 0.30-1.22 m) and the
tunnel (centre z 0.94 m) are open at the default 0.9 m.  Boulders (top 0.53 m)
and tunnel cradles lie below the slab and are correctly absent at that altitude.

Rasterisation runs at ``fine`` resolution (1 cm, the tunnel shell is 6 mm) and is
max-pooled to the planning resolution, so thin walls are never lost.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import inf

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:                      # numpy-only environments (e.g. a bare sim container)
    distance_transform_edt = None

EDT_CAP_CELLS = 16                        # numpy fallback is exact up to this distance (0.8 m at 5 cm)


def _edt_capped(free, cap_cells=EDT_CAP_CELLS):
    """Exact Euclidean distance (cells) from each ``free`` cell to the nearest non-free cell,
    up to ``cap_cells``; farther cells get ``cap_cells + 1``.  Numpy-only stand-in for
    scipy's distance_transform_edt.  Planning only uses clearance up to keep-out + soft
    margin (0.51 m = 10 cells), so the cap never changes a cost."""
    occ = ~np.asarray(free, bool)
    n0, n1 = occ.shape
    r = cap_cells
    pad = np.zeros((n0 + 2 * r, n1 + 2 * r), bool)
    pad[r:r + n0, r:r + n1] = occ
    best = np.where(occ, 0.0, np.inf)
    for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
            d = (dr * dr + dc * dc) ** .5
            if d == 0 or d > r:
                continue
            shifted = pad[r + dr:r + dr + n0, r + dc:r + dc + n1]
            np.minimum(best, np.where(shifted, d, np.inf), out=best)
    best[np.isinf(best)] = r + 1
    return best


@dataclass(frozen=True)
class GridSpec:
    half_size: float = 3.048       # net half width, m
    resolution: float = .05        # planning cell, m

    @property
    def n(self) -> int:
        return int(round(2 * self.half_size / self.resolution))

    def to_cell(self, xy):
        xy = np.asarray(xy, float)
        rc = np.floor((xy[..., ::-1] + self.half_size) / self.resolution).astype(int)
        return np.clip(rc, 0, self.n - 1)

    def to_xy(self, rc):
        rc = np.asarray(rc, float)
        return (rc[..., ::-1] + .5) * self.resolution - self.half_size


@dataclass(frozen=True)
class Envelope:
    """Drone footprint used for planning. Defaults follow the Sky Beta airframe contract."""
    radius: float = .28            # centre to prop tip, m
    half_height: float = .13
    tracking_margin: float = .08   # allowance for tracking error: hard keep-out = radius + this
    soft_margin: float = .15       # beyond the hard keep-out, cost rises toward walls
    soft_weight: float = 4.0       # cost at the hard boundary is 1 + soft_weight

    @property
    def hard(self):
        return self.radius + self.tracking_margin


def occupancy(course, spec: GridSpec, *, altitude: float, half_height: float, fine: float = .01) -> np.ndarray:
    """Boolean (n, n) grid, row = y, col = x. True where the slab hits an obstacle or the net."""
    k = max(1, int(round(spec.resolution / fine)))
    nf = spec.n * k
    step = 2 * spec.half_size / nf
    centres = (np.arange(nf) + .5) * step - spec.half_size
    X, Y = np.meshgrid(centres, centres)
    occ = np.zeros(X.shape, bool)
    rows = 4096 // max(1, len(course.box_c) + len(course.cyl_xy)) * 64
    flat = np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, altitude)])
    out = occ.ravel()
    # Each sample stands for a step x step square, so test a disc that covers it: the tunnel
    # shell is 6 mm thick and point samples on a 1 cm lattice can step straight over it.
    cover = step * .75
    for i in range(0, len(flat), rows):
        out[i:i + rows] = course.collides(flat[i:i + rows], cover, half_height)
    occ = out.reshape(X.shape)
    coarse = occ.reshape(spec.n, k, spec.n, k).any(axis=(1, 3))
    # the net: the outermost ring of cells is a wall
    coarse[0, :] = coarse[-1, :] = coarse[:, 0] = coarse[:, -1] = True
    return coarse


def clearance(occ: np.ndarray, spec: GridSpec) -> np.ndarray:
    """Distance (m) from each cell centre to the nearest occupied cell boundary (0 inside obstacles)."""
    edt = distance_transform_edt if distance_transform_edt is not None else _edt_capped
    d = edt(~occ) * spec.resolution
    return np.where(occ, 0.0, np.maximum(d - spec.resolution / 2, 0.0))


def cost_grid(occ: np.ndarray, spec: GridSpec, env: Envelope) -> np.ndarray:
    """Per-metre traversal cost >= 1; inf inside the hard keep-out."""
    d = clearance(occ, spec)
    cost = np.ones(occ.shape)
    soft = np.clip((env.hard + env.soft_margin - d) / max(env.soft_margin, 1e-9), 0, 1)
    cost += env.soft_weight * soft ** 2
    cost[d < env.hard] = inf
    return cost


@dataclass
class Checkpoint:
    """A line the whole drone must cross, in order (rules 7.6)."""
    name: str
    centre: np.ndarray            # xy of the line centre
    normal: np.ndarray            # unit xy crossing direction
    half_width: float             # half length of the line / aperture
    points: int = 0               # rule points, for reporting
    kind: str = "line"            # "line": cross it; "pad": stop with the centre within half_width
    # The chicane checkpoint is "enter the landing area through Gate B *or* the window"
    # (rule: window = +5). If True, arriving at the goal past the line by another opening
    # also clears it, recorded as via="other" (no window bonus).
    other_route: bool = False
    # aperture=True (gates, window, tunnel exit): the whole disc must cross inside the span.
    # aperture=False (a line drawn across a walled corridor): the centre crossing inside the
    # span is enough -- the walls bound the drone, and contacts are checked separately.
    aperture: bool = True

    def goal_xy(self, beyond: float) -> np.ndarray:
        if self.kind == "pad":
            return self.centre.copy()
        return self.centre + self.normal * beyond

    def reached(self, a, b, radius: float) -> bool:
        if self.kind == "pad":
            return float(np.linalg.norm(np.asarray(b) - self.centre)) <= self.half_width
        return self.crossed(a, b, radius)

    def crossed(self, a, b, radius: float) -> bool:
        """Does segment a->b cross the line in its direction with the whole disc inside the aperture?"""
        da = float(np.dot(np.asarray(a) - self.centre, self.normal))
        db = float(np.dot(np.asarray(b) - self.centre, self.normal))
        if not (da < radius <= db):
            return False
        t = (radius - da) / (db - da)
        p = np.asarray(a) + t * (np.asarray(b) - np.asarray(a))
        lateral = np.array([-self.normal[1], self.normal[0]])
        margin = radius if self.aperture else 0.0
        return abs(float(np.dot(p - self.centre, lateral))) <= self.half_width - margin + 1e-9


def checkpoints_from_route(route, course, *, chicane: str = "window"):
    """Map the repo's gate list (config/course_v3.yaml) to the five rule checkpoints.

    Only the checkpoint lines are used; the intermediate guideline corners are
    deliberately dropped so the planner, not a waypoint list, finds the maze route.
    ``chicane`` = "window" (+5 points) or "gate_b" (enter the landing area from the north).
    """
    by = {n: i for i, n in enumerate(route.names)}

    def cp(gate, name, pts, half_width=None, other_route=False, aperture=True):
        i = by[gate]
        n = np.asarray(route.normal[i][:2], float)
        return Checkpoint(name, np.asarray(route.centre[i][:2], float), n / np.linalg.norm(n),
                          float(half_width if half_width is not None else route.half_w[i]), pts,
                          other_route=other_route, aperture=aperture)

    # checkpoint 1: the diagonal across the corridor's bend, from the end of maze wall M1
    # (-1.93, 2.12) to the net's corner (-3.05, 3.05): half-span 0.73 m
    cps = [cp("z1_corner1", "1 first corridor", 4, half_width=.73, aperture=False),
           cp("z1_end", "2 maze exit (Gate A)", 7, half_width=.6),
           cp("tube_out", "3 tunnel", 10)]
    if chicane == "window":
        # aim for the window (+5); Gate B still clears the checkpoint if the window is blocked
        cps.append(cp("window", "4 chicane (window or Gate B)", 7, other_route=True))
    pad = route.centre[by["pad_hover"]][:2]
    # pad outer ring r = 30 in (0.762 m); the whole drone must be inside it, keep 8 cm in hand
    cps.append(Checkpoint("5 landing pad", np.asarray(pad, float), np.array([0.0, -1.0]),
                          .762 - .28 - .08, 7, kind="pad"))
    return cps

"""Course model built directly from the organisers' OBJ file.

Every named group in the OBJ becomes one or more simple collision primitives:

* thin walls / barriers / maze dividers  -> oriented boxes (OBB)
* a wall with a window cut in it         -> 4 boxes around the opening
* boulders                               -> vertical cylinders (conservative)
* the oval tube                          -> elliptic-cylinder shell
* tube cradles                           -> boxes
* floor, net, painted markings           -> ignored here (arena bounds handle floor/net)

Everything is vectorised with numpy so thousands of simulated drones can be
collision-checked per step on a laptop CPU.

Course frame (from the README): metres, Z up, origin at the centre of the floor.
"""
from __future__ import annotations

import collections
from dataclasses import dataclass, field

import numpy as np

IGNORE_PREFIXES = ("Floor", "Net", "Guideline", "Launch mark", "Landing pad", "Approach ring")


def load_obj_groups(path: str) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Return (vertices, {group_name: unique vertex positions of that group})."""
    verts, faces, cur = [], collections.OrderedDict(), None
    with open(path) as fh:
        for line in fh:
            t = line.split()
            if not t:
                continue
            if t[0] == "v":
                verts.append([float(x) for x in t[1:4]])
            elif t[0] == "g":
                cur = line[2:].strip()
                faces.setdefault(cur, set())
            elif t[0] == "f":
                for tok in t[1:]:
                    i = int(tok.split("/")[0])
                    faces[cur].add(i - 1 if i > 0 else len(verts) + i)
    V = np.asarray(verts)
    return V, {g: V[sorted(ids)] for g, ids in faces.items() if ids}


@dataclass
class Course:
    # oriented boxes: centre (K,3), unit horizontal axis u (K,2), half extents (K,3) = (along u, across u, z)
    box_c: np.ndarray
    box_u: np.ndarray
    box_h: np.ndarray
    box_names: list
    # vertical cylinders (boulders): centre xy (M,2), radius (M,), z top (M,)
    cyl_xy: np.ndarray
    cyl_r: np.ndarray
    cyl_top: np.ndarray
    # elliptic tube shell along y
    tube: dict | None
    # arena
    half_size: float = 3.048
    net_height: float = 3.048
    groups: dict = field(default_factory=dict)

    # ------------------------------------------------------------------ queries
    def collides(self, p: np.ndarray, r_xy, r_z) -> np.ndarray:
        """p: (N,3) drone centres. The drone is a disc of radius r_xy and half-height r_z
        (r_z should include some allowance for tilt). Obstacles are inflated by that shape
        and the centre is tested against them. Returns bool (N,)."""
        p = np.atleast_2d(p)
        n = p.shape[0]
        rxy = np.broadcast_to(np.asarray(r_xy, dtype=float), (n,))
        rz = np.broadcast_to(np.asarray(r_z, dtype=float), (n,))
        hit = np.zeros(n, dtype=bool)

        if len(self.box_c):
            d = p[:, None, :] - self.box_c[None]  # N,K,3
            u = self.box_u[None]
            lu = d[..., 0] * u[..., 0] + d[..., 1] * u[..., 1]
            ln = -d[..., 0] * u[..., 1] + d[..., 1] * u[..., 0]
            h = self.box_h[None]
            inside = (np.abs(lu) < h[..., 0] + rxy[:, None]) & (np.abs(ln) < h[..., 1] + rxy[:, None]) \
                & (np.abs(d[..., 2]) < h[..., 2] + rz[:, None])
            hit |= inside.any(1)

        if len(self.cyl_xy):
            dxy = np.linalg.norm(p[:, None, :2] - self.cyl_xy[None], axis=-1)
            inside = (dxy < self.cyl_r[None] + rxy[:, None]) & (p[:, None, 2] < self.cyl_top[None] + rz[:, None])
            hit |= inside.any(1)

        if self.tube is not None:
            t = self.tube
            in_y = (p[:, 1] > t["y0"] - rxy) & (p[:, 1] < t["y1"] + rxy)
            ex, ez = p[:, 0] - t["cx"], p[:, 2] - t["cz"]
            inner = (ex / np.maximum(t["a"] - rxy, 1e-3)) ** 2 + (ez / np.maximum(t["b"] - rz, 1e-3)) ** 2 < 1.0
            outer = (ex / (t["a"] + t["wall"] + rxy)) ** 2 + (ez / (t["b"] + t["wall"] + rz)) ** 2 > 1.0
            # within the tube's length (padded at the open ends) the solid part is the shell,
            # thickened by the drone's size on both sides.
            hit |= in_y & ~inner & ~outer
        return hit

    def out_of_bounds(self, p: np.ndarray, r_xy, floor_z: float, ceil_z: float) -> np.ndarray:
        p = np.atleast_2d(p)
        lim = self.half_size - r_xy
        return (np.abs(p[:, 0]) > lim) | (np.abs(p[:, 1]) > lim) | (p[:, 2] < floor_z) | (p[:, 2] > ceil_z)


# ---------------------------------------------------------------------- builders
def _obb_from_points(P: np.ndarray):
    xy = P[:, :2]
    c2 = xy.mean(0)
    cov = np.cov((xy - c2).T)
    w, v = np.linalg.eigh(cov)
    u = v[:, np.argmax(w)]
    if u[0] < 0 or (abs(u[0]) < 1e-9 and u[1] < 0):
        u = -u
    lu = (xy - c2) @ u
    ln = (xy - c2) @ np.array([-u[1], u[0]])
    cu = 0.5 * (lu.max() + lu.min())
    cn = 0.5 * (ln.max() + ln.min())
    c2 = c2 + cu * u + cn * np.array([-u[1], u[0]])
    zlo, zhi = P[:, 2].min(), P[:, 2].max()
    c = np.array([c2[0], c2[1], 0.5 * (zlo + zhi)])
    h = np.array([0.5 * (lu.max() - lu.min()), 0.5 * (ln.max() - ln.min()), 0.5 * (zhi - zlo)])
    return c, u, h


def _split_window_wall(P: np.ndarray):
    """A wall whose vertices include an interior rectangle (the window) -> 4 boxes + window info."""
    c, u, h = _obb_from_points(P)
    zlo, zhi = c[2] - h[2], c[2] + h[2]
    interior = P[(P[:, 2] > zlo + 1e-3) & (P[:, 2] < zhi - 1e-3)]
    lu = (interior[:, :2] - c[:2]) @ u
    wlo, whi = lu.min(), lu.max()
    wzlo, wzhi = interior[:, 2].min(), interior[:, 2].max()
    boxes = []

    def mk(a0, a1, z0, z1):
        cc = c.copy()
        cc[:2] = c[:2] + 0.5 * (a0 + a1) * u
        cc[2] = 0.5 * (z0 + z1)
        return cc, u.copy(), np.array([0.5 * (a1 - a0), h[1], 0.5 * (z1 - z0)])

    boxes.append(mk(-h[0], wlo, zlo, zhi))  # left of window
    boxes.append(mk(whi, h[0], zlo, zhi))  # right of window
    boxes.append(mk(wlo, whi, zlo, wzlo))  # below
    boxes.append(mk(wlo, whi, wzhi, zhi))  # above
    centre_xy = c[:2] + 0.5 * (wlo + whi) * u
    window = dict(
        centre=np.array([centre_xy[0], centre_xy[1], 0.5 * (wzlo + wzhi)]),
        u=u, half_width=0.5 * (whi - wlo), half_height=0.5 * (wzhi - wzlo),
    )
    return boxes, window


def build_course(obj_path: str) -> Course:
    _, G = load_obj_groups(obj_path)
    box_c, box_u, box_h, names = [], [], [], []
    cyl_xy, cyl_r, cyl_top = [], [], []
    tube = None
    features = {}

    for name, P in G.items():
        if name.startswith(IGNORE_PREFIXES):
            continue
        low = name.lower()
        if low.startswith("boulder"):
            lo, hi = P.min(0), P.max(0)
            cyl_xy.append(0.5 * (lo[:2] + hi[:2]))
            cyl_r.append(0.5 * max(hi[0] - lo[0], hi[1] - lo[1]))
            cyl_top.append(hi[2])
            continue
        if low.startswith("tube oval"):
            lo, hi = P.min(0), P.max(0)
            ys = np.unique(P[:, 1].round(4))
            cx, cz = 0.5 * (lo[0] + hi[0]), 0.5 * (lo[2] + hi[2])
            # thin shell: outer semi axes from bbox, wall thickness ~ spread of normalised radius
            a_out, b_out = 0.5 * (hi[0] - lo[0]), 0.5 * (hi[2] - lo[2])
            rr = np.hypot((P[:, 0] - cx) / a_out, (P[:, 2] - cz) / b_out)
            wall = max(float((1 - rr.min()) * min(a_out, b_out)), 0.005)
            tube = dict(cx=cx, cz=cz, a=a_out - wall, b=b_out - wall, wall=wall, y0=ys.min(), y1=ys.max())
            features["tube"] = tube
            continue
        zs = np.unique(P[:, 2].round(4))
        if len(zs) > 2 and "window" in low:
            boxes, window = _split_window_wall(P)
            features["window"] = window
            for c, u, h in boxes:
                box_c.append(c), box_u.append(u), box_h.append(h), names.append(name)
            continue
        c, u, h = _obb_from_points(P)
        box_c.append(c), box_u.append(u), box_h.append(h), names.append(name)

    course = Course(
        box_c=np.array(box_c), box_u=np.array(box_u), box_h=np.array(box_h), box_names=names,
        cyl_xy=np.array(cyl_xy).reshape(-1, 2), cyl_r=np.array(cyl_r), cyl_top=np.array(cyl_top),
        tube=tube, groups=features,
    )
    # pull pad / launch centres from markings for convenience
    for key, gname in [("launch", "Launch mark"), ("pad", "Landing pad 5 ft")]:
        if gname in G:
            features[key] = G[gname].mean(0)
    return course

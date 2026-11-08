"""Published course (pathfinding/course/dronehacks-course-v3.obj) as 2-D footprints, plus the
odom -> course alignment shared by trajectory.py and depth_map.py.

Course frame: metres, Z up, origin at the centre of the floor. The vehicle starts on the launch
mark (-2.515, 0.914) facing pi/2 (+y); its body origin is ~5 cm above the floor at startup.
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
OBJ = REPO_ROOT / "pathfinding" / "course" / "dronehacks-course-v3.obj"
LAUNCH_XY = (-2.515, 0.914)
LAUNCH_HEADING = np.pi / 2
LAUNCH_HEIGHT = 0.05
ARENA_HALF = 3.048


def _load_course_module():
    # load pathfinding/drl/course.py by path: importing the drl package would pull in its deps
    if "rc_bag_course" in sys.modules:
        return sys.modules["rc_bag_course"]
    spec = importlib.util.spec_from_file_location("rc_bag_course", REPO_ROOT / "pathfinding" / "drl" / "course.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["rc_bag_course"] = mod
    spec.loader.exec_module(mod)
    return mod


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


class Alignment:
    """Maps odom-frame points/yaws into the course frame.

    first-pose : the first pose of the trajectory is put on the launch mark, facing LAUNCH_HEADING
                 (robust to an odom origin that is not the startup pose, e.g. VIO reset / PX4 frame).
    odom-origin: the odom origin is the launch mark with heading LAUNCH_HEADING (cuVSLAM convention
                 used by the flight code); the first pose is not used.
    none       : identity; plots stay in the raw odom frame and no course is drawn.
    """

    def __init__(self, mode: str, p0=(0, 0, 0), yaw0: float = 0.0):
        self.mode = mode
        if mode == "first-pose":
            self.p0, self.yaw0 = np.asarray(p0, float), float(yaw0)
        elif mode == "odom-origin":
            self.p0, self.yaw0 = np.zeros(3), 0.0
        elif mode == "none":
            self.p0, self.yaw0 = np.zeros(3), 0.0
        else:
            raise ValueError(f"unknown alignment {mode}")
        if mode == "none":
            self.R = np.eye(3)
            self.t = np.zeros(3)
        else:
            self.R = _rz(LAUNCH_HEADING - self.yaw0)
            self.t = np.array([LAUNCH_XY[0], LAUNCH_XY[1], LAUNCH_HEIGHT]) - self.R @ self.p0

    @property
    def course(self) -> bool:
        return self.mode != "none"

    @property
    def floor_z(self) -> float:
        """Floor height in the output frame (start assumed on the ground)."""
        return 0.0 if self.course else float(self.p0[2]) - LAUNCH_HEIGHT

    def points(self, p):
        return np.asarray(p, float) @ self.R.T + self.t

    def yaw(self, y):
        d = 0.0 if self.mode == "none" else LAUNCH_HEADING - self.yaw0
        y = np.asarray(y, float) + d
        return np.arctan2(np.sin(y), np.cos(y))

    def describe(self) -> str:
        if self.mode == "none":
            return "none (raw odom frame)"
        return (f"{self.mode}: odom ({self.p0[0]:.2f}, {self.p0[1]:.2f}, {self.p0[2]:.2f}) yaw "
                f"{np.degrees(self.yaw0):.1f} deg -> launch mark ({LAUNCH_XY[0]}, {LAUNCH_XY[1]}) "
                f"heading {np.degrees(LAUNCH_HEADING):.0f} deg")


class Footprints:
    """Published obstacles cut at a height band [z_lo, z_hi] as 2-D boxes (OBB) and circles."""

    def __init__(self, z_lo: float = -1e9, z_hi: float = 1e9, obj: Path = OBJ, include_net: bool = True):
        course = _load_course_module().build_course(str(obj))
        self.course = course
        c, u, h, names = [], [], [], []
        for bc, bu, bh, n in zip(course.box_c, course.box_u, course.box_h, course.box_names):
            if bc[2] + bh[2] >= z_lo and bc[2] - bh[2] <= z_hi:
                c.append(bc[:2]), u.append(bu), h.append(bh[:2]), names.append(n)
        t = course.tube
        if t is not None:   # shell cross-section at a few heights inside the band
            A, B = t["a"] + t["wall"], t["b"] + t["wall"]
            yc, hy = 0.5 * (t["y0"] + t["y1"]), 0.5 * (t["y1"] - t["y0"])
            for z in np.linspace(max(z_lo, t["cz"] - B), min(z_hi, t["cz"] + B), 7):
                ez = (z - t["cz"]) / B
                if abs(ez) >= 1:
                    continue
                xo = A * np.sqrt(1 - ez ** 2)
                ei = (z - t["cz"]) / t["b"]
                xi = t["a"] * np.sqrt(1 - ei ** 2) if abs(ei) < 1 else 0.0
                for sgn in ((-1, 1) if xi > 0 else (0,)):
                    xc = t["cx"] + sgn * 0.5 * (xo + xi) if xi > 0 else t["cx"]
                    hx = 0.5 * (xo - xi) if xi > 0 else xo
                    c.append([xc, yc]), u.append([1.0, 0.0]), h.append([max(hx, 0.01), hy]), names.append("Tube")
        if include_net:
            a = ARENA_HALF
            for cc, uu, hh in (([0, a], [1, 0], [a, 0.01]), ([0, -a], [1, 0], [a, 0.01]),
                               ([a, 0], [0, 1], [a, 0.01]), ([-a, 0], [0, 1], [a, 0.01])):
                c.append(cc), u.append(uu), h.append(hh), names.append("Net")
        self.box_c, self.box_u, self.box_h = np.array(c, float), np.array(u, float), np.array(h, float)
        self.box_names = names
        keep = course.cyl_top >= z_lo if len(course.cyl_r) else np.zeros(0, bool)
        self.cyl_xy, self.cyl_r = course.cyl_xy[keep], course.cyl_r[keep]
        self.launch = course.groups.get("launch")
        self.pad = course.groups.get("pad")

    def distance(self, xy) -> tuple[np.ndarray, list]:
        """Distance (m) from each point to the nearest published surface (0 inside) and its name."""
        xy = np.atleast_2d(np.asarray(xy, float))
        D, N = [], []
        if len(self.box_c):
            d = xy[:, None, :] - self.box_c[None]
            u = self.box_u[None]
            lu = d[..., 0] * u[..., 0] + d[..., 1] * u[..., 1]
            ln = -d[..., 0] * u[..., 1] + d[..., 1] * u[..., 0]
            e = np.stack([np.abs(lu) - self.box_h[None, :, 0], np.abs(ln) - self.box_h[None, :, 1]], -1)
            D.append(np.linalg.norm(np.maximum(e, 0), axis=-1))
            N += self.box_names
        if len(self.cyl_r):
            D.append(np.maximum(np.linalg.norm(xy[:, None] - self.cyl_xy[None], axis=-1) - self.cyl_r[None], 0))
            N += ["Boulder"] * len(self.cyl_r)
        if not D:
            return np.full(len(xy), np.inf), [None] * len(xy)
        D = np.concatenate(D, 1)
        k = D.argmin(1)
        return D[np.arange(len(xy)), k], [N[i] for i in k]

    def outline_samples(self, step: float = 0.05, include_net: bool = False) -> tuple[np.ndarray, list]:
        """Points every `step` m along each obstacle outline (for 'was this wall seen?')."""
        pts, names = [], []
        for c, u, h, n in zip(self.box_c, self.box_u, self.box_h, self.box_names):
            if n == "Net" and not include_net:
                continue
            for poly in [self._box_poly(c, u, h)]:
                for a, b in zip(poly[:-1], poly[1:]):
                    k = max(int(np.linalg.norm(b - a) / step), 1)
                    s = np.linspace(0, 1, k, endpoint=False)[:, None]
                    pts.append(a + s * (b - a))
                    names += [n] * k
        for c, r in zip(self.cyl_xy, self.cyl_r):
            k = max(int(2 * np.pi * r / step), 8)
            a = np.linspace(0, 2 * np.pi, k, endpoint=False)
            pts.append(c + r * np.c_[np.cos(a), np.sin(a)])
            names += ["Boulder"] * k
        return (np.concatenate(pts) if pts else np.zeros((0, 2))), names

    @staticmethod
    def _box_poly(c, u, h):
        n = np.array([-u[1], u[0]])
        cs = [c + sx * h[0] * u + sy * h[1] * n for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1))]
        return np.array(cs)

    def draw(self, ax, color="0.25", lw=1.0, fill=True, label="published course"):
        import matplotlib.patches as mp
        first = True
        for c, u, h, n in zip(self.box_c, self.box_u, self.box_h, self.box_names):
            poly = self._box_poly(c, u, h)
            ls = ":" if n == "Net" else "-"
            ax.add_patch(mp.Polygon(poly[:-1], closed=True, fill=fill and n != "Net", fc="0.75", ec=color,
                                    lw=lw, ls=ls, alpha=0.8, label=label if first else None))
            first = False
        for c, r in zip(self.cyl_xy, self.cyl_r):
            ax.add_patch(mp.Circle(c, r, fill=fill, fc="0.75", ec=color, lw=lw, alpha=0.8))
        if self.launch is not None:
            ax.plot(*self.launch[:2], marker="^", color="tab:green", ms=8, ls="", label="launch mark")
        if self.pad is not None:
            ax.add_patch(mp.Circle(self.pad[:2], 0.76, fill=False, ec="tab:green", lw=1, ls="--"))

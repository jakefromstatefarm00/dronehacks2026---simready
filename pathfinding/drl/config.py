"""Load the YAML config and resolve the gate list into arrays."""
from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np
import yaml

from .course import Course, build_course

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@dataclass
class Gates:
    names: list
    centre: np.ndarray      # (G,3)
    normal: np.ndarray      # (G,3) unit, horizontal, direction of travel
    lateral: np.ndarray     # (G,3) unit, horizontal, across the opening
    half_w: np.ndarray      # (G,)
    half_h: np.ndarray      # (G,)
    is_hover: np.ndarray    # (G,) bool
    hover_radius: np.ndarray
    hover_speed: np.ndarray

    def __len__(self):
        return len(self.names)


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def resolve_gates(spec: list, course: Course) -> Gates:
    names, C, N, L, HW, HH, HOV, HR, HS = [], [], [], [], [], [], [], [], []
    for g in spec:
        names.append(g["name"])
        if g["type"] == "hover":
            C.append(g["centre"]); N.append([1, 0, 0]); L.append([0, 1, 0])
            HW.append(0); HH.append(0); HOV.append(True)
            HR.append(g["radius"]); HS.append(g["max_speed"])
            continue
        src = g.get("from")
        m = g.get("margin", 0.0)
        if src in ("tube_entry", "tube_exit"):
            t = course.groups["tube"]
            y = t["y1"] if src == "tube_entry" else t["y0"]
            c = [t["cx"], y, t["cz"]]
            n = [0, -1, 0]  # flown north -> south
            hw, hh = t["a"] - m, t["b"] - m
        elif src == "window":
            w = course.groups["window"]
            c = w["centre"]
            u = w["u"]
            n = np.array([-u[1], u[0], 0.0])
            if g.get("direction", "west") == "west" and n[0] > 0:
                n = -n
            if g.get("direction") == "east" and n[0] < 0:
                n = -n
            hw, hh = w["half_width"] - m, w["half_height"] - m
        else:
            c = g["centre"]
            n = list(g["normal"]) + [0.0] if len(g["normal"]) == 2 else g["normal"]
            hw, hh = g["half_width"], g["half_height"]
        n = _unit(n)
        lat = _unit(np.cross([0, 0, 1], n))
        C.append(c); N.append(n); L.append(lat); HW.append(hw); HH.append(hh)
        HOV.append(False); HR.append(0); HS.append(0)
    a = lambda x, dt=float: np.asarray(x, dtype=dt)
    return Gates(names, a(C), a(N), a(L), a(HW), a(HH), a(HOV, bool), a(HR), a(HS))


def load_config(path: str | None = None):
    path = path or os.path.join(ROOT, "config", "course_v3.yaml")
    with open(path) as fh:
        cfg = yaml.safe_load(fh)
    obj = cfg["course_obj"]
    if not os.path.isabs(obj):
        obj = os.path.join(ROOT, obj)
    course = build_course(obj)
    gates = resolve_gates(cfg["gates"], course)
    return cfg, course, gates

"""Checkpoints: load them from YAML and do the path geometry the mission and the policy need.

A checkpoint file looks like this (see config/checkpoints/):

    frame: course          # 'course' = published-layout metres; 'start' = relative to takeoff
    defaults: {radius: 0.25, corridor: 0.35}
    checkpoints:
      - {name: corner1, xyz: [-2.515, 2.585, 1.0]}
      - {name: window,  xyz: [-0.471, -2.272, 0.762], radius: 0.15, corridor: 0.20}

* radius   - how close (m) counts as "reached".
* corridor - how far (m) the drone may stray sideways from the straight line that LEADS TO this
             checkpoint. Training ends an episode beyond it; the drone node holds position.
* The LAST checkpoint is where the drone lands.

frame 'start': x = forward along the nose direction at takeoff, y = left, z = height above the
ground at the takeoff spot. Good for first tests ("1 m forward and back") with no course map.
frame 'course': published-layout coordinates; needs the launch pose (mission.yaml launch_*).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from team_mission.geometry import CourseFrame, rot_z


@dataclass
class Checkpoint:
    name: str
    xyz: np.ndarray          # in the file's frame
    radius: float
    corridor: float


@dataclass
class CheckpointSpec:
    frame: str
    checkpoints: List[Checkpoint]
    description: str = ''

    @staticmethod
    def from_dict(d: dict) -> 'CheckpointSpec':
        frame = str(d.get('frame', 'course')).lower()
        if frame not in ('course', 'start'):
            raise ValueError(f"checkpoint frame must be 'course' or 'start', got {frame!r}")
        dflt = d.get('defaults', {}) or {}
        r0, c0 = float(dflt.get('radius', 0.25)), float(dflt.get('corridor', 0.35))
        cps = []
        for i, c in enumerate(d.get('checkpoints') or []):
            xyz = np.asarray(c['xyz'], float)
            if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
                raise ValueError(f'checkpoint {i}: xyz must be 3 finite numbers')
            cps.append(Checkpoint(str(c.get('name', f'cp{i + 1}')), xyz,
                                  float(c.get('radius', r0)), float(c.get('corridor', c0))))
        if not cps:
            raise ValueError('checkpoint file has no checkpoints')
        return CheckpointSpec(frame, cps, str(d.get('description', '')))

    @staticmethod
    def load(path: str) -> 'CheckpointSpec':
        import yaml
        with open(path) as fh:
            return CheckpointSpec.from_dict(yaml.safe_load(fh))


@dataclass
class Path:
    """Points in the odom frame. P[0] is the takeoff hover point; P[1:] are the checkpoints."""
    points: np.ndarray                 # (N+1, 3)
    radius: np.ndarray                 # (N+1,) radius[i] for checkpoint i (radius[0] unused)
    corridor: np.ndarray               # (N+1,) corridor[i] for segment i-1 -> i
    names: List[str] = field(default_factory=list)

    def __post_init__(self):
        self.points = np.asarray(self.points, float)
        d = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        self.seg_len = np.concatenate([[0.0], d])          # seg_len[i] = |P[i] - P[i-1]|
        self.cum = np.cumsum(self.seg_len)                 # arc length at checkpoint i

    @property
    def n(self) -> int:
        """Number of checkpoints (excluding the takeoff point)."""
        return len(self.points) - 1

    @property
    def length(self) -> float:
        return float(self.cum[-1])

    def closest(self, p, i: int):
        """Closest point to p on segment i-1 -> i. Returns (point, t in [0, 1])."""
        a, b = self.points[i - 1], self.points[i]
        ab = b - a
        L2 = float(ab @ ab)
        t = 0.0 if L2 < 1e-12 else float(np.clip((np.asarray(p) - a) @ ab / L2, 0.0, 1.0))
        return a + t * ab, t

    def lateral(self, p, i: int) -> np.ndarray:
        """Vector from the closest point on segment i to p (how far off the line we are)."""
        c, _ = self.closest(p, i)
        return np.asarray(p, float) - c

    def off_path(self, p, i: int):
        """How far p is from the path near checkpoint i: the distance to the current segment, or to
        the previous one if that is closer (just after a checkpoint is reached the drone is still on
        the old line, and the clamped distance to the new segment would include the distance left
        to the corner). Returns (distance, allowed corridor of that segment)."""
        d = float(np.linalg.norm(self.lateral(p, i)))
        c = float(self.corridor[i])
        if i > 1:
            d0 = float(np.linalg.norm(self.lateral(p, i - 1)))
            if d0 < d:
                return d0, float(self.corridor[i - 1])
        return d, c

    def progress(self, p, i: int) -> float:
        """Arc length (m) of p projected on the path, when heading for checkpoint i."""
        _, t = self.closest(p, i)
        return float(self.cum[i - 1] + t * self.seg_len[i])

    def point_at(self, s: float) -> np.ndarray:
        """Point at arc length s along the whole path (clamped to the ends)."""
        s = float(np.clip(s, 0.0, self.length))
        i = int(np.searchsorted(self.cum, s, side='left'))
        i = max(1, min(i, self.n))
        L = self.seg_len[i]
        t = 0.0 if L < 1e-9 else (s - self.cum[i - 1]) / L
        return self.points[i - 1] + t * (self.points[i] - self.points[i - 1])

    def reached(self, p, i: int) -> bool:
        """Checkpoint i counts as reached inside its radius, or once we are past it along the
        segment and still close sideways (so a slightly wide pass does not orbit back)."""
        p = np.asarray(p, float)
        if np.linalg.norm(p - self.points[i]) <= self.radius[i]:
            return True
        a, b = self.points[i - 1], self.points[i]
        ab = b - a
        L2 = float(ab @ ab)
        if L2 < 1e-12:
            return False
        t = float((p - a) @ ab / L2)
        return t >= 1.0 and np.linalg.norm(p - (a + min(t, 1.0) * ab)) <= 1.5 * self.radius[i]


def resolve_path(spec: CheckpointSpec, start_ground_odom, start_yaw_odom: float,
                 takeoff_height: float, course_frame: Optional[CourseFrame] = None) -> Path:
    """Build the odom-frame path once the drone's start is known.

    start_ground_odom: where the drone sat when Offboard began (odom metres).
    start_yaw_odom:    its yaw then (used by frame 'start').
    takeoff_height:    it climbs straight up by this much first; that hover point is P[0].
    """
    g = np.asarray(start_ground_odom, float)
    pts = [g + np.array([0.0, 0.0, float(takeoff_height)])]
    if spec.frame == 'course':
        cf = course_frame or CourseFrame()
        pts += [cf.course_to_odom(c.xyz) for c in spec.checkpoints]
    else:
        R = rot_z(start_yaw_odom)
        pts += [g + R @ c.xyz for c in spec.checkpoints]
    radius = np.array([0.0] + [c.radius for c in spec.checkpoints])
    corridor = np.array([spec.checkpoints[0].corridor] + [c.corridor for c in spec.checkpoints])
    return Path(np.array(pts), radius, corridor, ['takeoff'] + [c.name for c in spec.checkpoints])

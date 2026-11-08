"""Real depth image -> map cells (free / hit / behind) for ``OnlinePlanner.observe``.

The drone-side counterpart of ``sensor.DepthCamera``: same output, but from an
actual depth image (OAK-D stereo depth), its pinhole intrinsics and the drone's
pose in the course frame.

Frames
------
camera  OpenCV optical: x right, y down, z forward (what CameraInfo describes).
body    FLU: x forward, y left, z up.  ``CameraMount`` gives the camera pose in it.
course  the published map: x, y on the floor, z up, origin at the floor centre.

For each sampled pixel the 3-D ray from the camera to its depth point is walked
in ``step`` increments. Samples inside the flight slab (altitude +- half_height)
mark their cell free; the end point marks its cell occupied if it lies in the slab
(a floor or ceiling return just ends the ray). Pixels with no depth (0, NaN or
outside [min_range, max_range]) are skipped: no return is not evidence of free
space.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

_OPTICAL_TO_FLU = np.array([[0., 0., 1.],        # forward  = optical z
                            [-1., 0., 0.],       # left     = -optical x
                            [0., -1., 0.]])      # up       = -optical y


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_k(cls, k, width, height):
        """From a row-major 3x3 K (sensor_msgs/CameraInfo.k)."""
        k = np.asarray(k, float).reshape(3, 3)
        return cls(k[0, 0], k[1, 1], k[0, 2], k[1, 2], int(width), int(height))


@dataclass(frozen=True)
class CameraMount:
    """Camera pose in the body FLU frame. Default: OAK-D facing forward, level."""
    # m, OAK-D depth origin from the body origin (centre of mass), Sky Beta CAD
    # (sky-beta.step, imported by the team's sim_stack tools: OAK-D at (0.115, 0, 0.029) m).
    # On the drone use CAMERA_X/Y/Z from /etc/skybeta/drone.env (measured by the organisers).
    offset: tuple = (.115, 0.0, .029)
    pitch_down_deg: float = 0.0

    def rotation(self):
        p = np.deg2rad(self.pitch_down_deg)
        pitch = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        return pitch @ _OPTICAL_TO_FLU


def body_to_course(yaw, roll=0.0, pitch=0.0):
    """Rotation body FLU -> course (z-up, yaw CCW from course +x)."""
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def project_depth(depth_m, intr: Intrinsics, position, rotation, spec, *, altitude=.9, half_height=.21,
                  mount: CameraMount = CameraMount(), stride=8, min_range=.35, max_range=3.5,
                  occlusion_depth=.4, expected=None):
    """Return (free, hit, behind) (K, 2) int cell arrays, like ``DepthCamera.observe``.

    depth_m   (H, W) metres (convert uint16 millimetres first)
    position  body origin in the course frame (3,)
    rotation  body FLU -> course rotation (3, 3), e.g. ``body_to_course(yaw)``
    expected  prior occupancy: hits on expected cells do not create suspect cells
    """
    empty = np.empty((0, 2), int)
    d = np.asarray(depth_m, float)[::stride, ::stride]
    v, u = np.mgrid[0:d.shape[0], 0:d.shape[1]]
    u, v, z = u.ravel() * stride, v.ravel() * stride, d.ravel()
    ok = np.isfinite(z) & (z >= min_range) & (z <= max_range)
    if not ok.any():
        return empty, empty, empty
    u, v, z = u[ok], v[ok], z[ok]
    # back-project to the camera frame (optical), then to course
    pts_cam = np.column_stack([(u - intr.cx) / intr.fx * z, (v - intr.cy) / intr.fy * z, z])
    r_cam = np.asarray(rotation) @ mount.rotation()
    origin = np.asarray(position, float) + np.asarray(rotation) @ np.asarray(mount.offset, float)
    ends = origin + pts_cam @ r_cam.T                                     # (N, 3) course

    step = spec.resolution / 2
    length = np.linalg.norm(ends - origin, axis=1)
    t = np.arange(0.0, length.max() + step, step)                         # samples along the ray, metres
    frac = np.minimum(t[None, :] / length[:, None], 1.0)                  # (N, S)
    samples = origin + frac[..., None] * (ends - origin)[:, None, :]      # (N, S, 3)
    before_end = t[None, :] < length[:, None] - step                      # strictly before the end point
    in_slab = np.abs(samples[..., 2] - altitude) <= half_height
    rc = spec.to_cell(samples[..., :2])
    inside = ((samples[..., :2] > -spec.half_size) & (samples[..., :2] < spec.half_size)).all(-1)

    free_mask = before_end & in_slab & inside
    end_in_slab = (np.abs(ends[:, 2] - altitude) <= half_height) & \
        ((ends[:, :2] > -spec.half_size) & (ends[:, :2] < spec.half_size)).all(-1)
    hit_rc = spec.to_cell(ends[end_in_slab, :2])
    free = np.unique(rc[free_mask], axis=0) if free_mask.any() else empty
    hit = np.unique(hit_rc, axis=0) if len(hit_rc) else empty
    if len(free) and len(hit):
        # a cell that returned a hit this frame is not also free
        hit_set = {tuple(c) for c in hit}
        free = np.array([c for c in free if tuple(c) not in hit_set], int).reshape(-1, 2)

    # cells just behind unexpected hits, along the horizontal ray direction
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
                behind = np.unique(spec.to_cell(pts), axis=0)
    return free, hit, behind


def decode_depth_image(data, width, height, encoding, step=None):
    """sensor_msgs/Image depth -> (H, W) float metres. Supports 16UC1 (mm) and 32FC1 (m)."""
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if encoding in ("16UC1", "mono16"):
        row = step or width * 2
        img = buf.reshape(height, row)[:, :width * 2].copy().view("<u2").astype(float) / 1000.0
    elif encoding == "32FC1":
        row = step or width * 4
        img = buf.reshape(height, row)[:, :width * 4].copy().view("<f4").astype(float)
    else:
        raise ValueError(f"unsupported depth encoding {encoding!r} (expected 16UC1 or 32FC1)")
    img[img <= 0] = np.nan
    return img


def drop_misregistered(free, hit, behind, prior, near_wall):
    """Remove depth evidence that a slightly wrong pose (latency, turning) would invent.

    ``prior`` is the published occupancy; ``near_wall`` the free cells within a few cm of it.
    A return there is the published wall seen a little off, not a new obstacle; a published
    wall cell seen "free" is a ray that slipped past the wall's edge.  Without this, 0.1 s of
    pose lag at 1 rad/s turned 321 cells of corridor into phantom walls (offline replay) and
    the planner ran out of paths.  New obstacles away from walls are kept; walls that really
    moved are still handled by OnlinePlanner's "doubt unconfirmed map" step.
    """
    def keep(cells, mask):
        return cells[~mask[cells[:, 0], cells[:, 1]]] if len(cells) else cells
    # Hits ON a published wall are kept: they change nothing (the cell is already occupied) except
    # marking that wall camera-*confirmed*. Dropping them too meant no published wall was ever
    # confirmed, so "doubt unconfirmed map" made every wall passable - including ones the camera
    # was looking at - and fake-PX4 runs flew into them (clutter-01, 0.0 m).
    return keep(free, prior), keep(hit, near_wall), keep(behind, prior | near_wall)

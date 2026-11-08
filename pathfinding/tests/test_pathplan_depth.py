"""Depth image -> map cells, on synthetic images with known geometry."""
import numpy as np
import pytest

from pathplan.depth_projection import (CameraMount, Intrinsics, body_to_course, decode_depth_image,
                                       project_depth)
from pathplan.world import GridSpec

SPEC = GridSpec()                                   # the course grid: 6.096 m square, 5 cm cells
INTR = Intrinsics(fx=300., fy=300., cx=160., cy=100., width=320, height=200)
MOUNT = CameraMount(offset=(0.0, 0.0, 0.0))


def wall_depth(distance):
    """Depth image of a flat wall facing the camera at ``distance`` metres (optical z is constant)."""
    return np.full((INTR.height, INTR.width), float(distance))


@pytest.mark.parametrize("yaw, axis, sign", [(0.0, 0, 1), (np.pi / 2, 1, 1), (np.pi, 0, -1)])
def test_wall_ahead_is_hit_and_space_before_it_is_free(yaw, axis, sign):
    free, hit, behind = project_depth(wall_depth(2.0), INTR, np.array([0., 0., .9]), body_to_course(yaw),
                                      SPEC, mount=MOUNT, stride=4)
    hx = SPEC.to_xy(hit)[:, axis] * sign
    fx = SPEC.to_xy(free)[:, axis] * sign
    bx = SPEC.to_xy(behind)[:, axis] * sign
    assert len(hit) and np.all(np.abs(hx - 2.0) <= SPEC.resolution)
    # a valid 2 m return means the whole ray in front of it was clear, near part included
    assert len(free) and fx.min() >= -SPEC.resolution and fx.max() < 2.0
    assert len(behind) and bx.min() > 2.0 - SPEC.resolution and bx.max() <= 2.45


def test_expected_wall_creates_no_suspects():
    _, hit, _ = project_depth(wall_depth(2.0), INTR, np.array([0., 0., .9]), np.eye(3), SPEC, mount=MOUNT)
    expected = np.zeros((SPEC.n, SPEC.n), bool)
    expected[hit[:, 0], hit[:, 1]] = True
    _, _, behind = project_depth(wall_depth(2.0), INTR, np.array([0., 0., .9]), np.eye(3), SPEC,
                                 mount=MOUNT, expected=expected)
    assert len(behind) == 0


def test_missing_depth_and_out_of_range_are_ignored():
    d = wall_depth(2.0)
    d[:, :] = np.nan
    assert all(len(x) == 0 for x in project_depth(d, INTR, np.zeros(3), np.eye(3), SPEC, mount=MOUNT))
    far = wall_depth(9.0)                            # beyond max_range: no evidence at all
    assert all(len(x) == 0 for x in project_depth(far, INTR, np.zeros(3), np.eye(3), SPEC, mount=MOUNT))


def test_floor_returns_end_rays_without_hits():
    """Camera tilted 60 deg down sees only floor within range: free space, no obstacles."""
    mount = CameraMount(offset=(0., 0., 0.), pitch_down_deg=60.)
    # depth (optical z) of the floor plane for every pixel, camera at 0.9 m
    v, u = np.mgrid[0:INTR.height, 0:INTR.width]
    rays = np.stack([(u - INTR.cx) / INTR.fx, (v - INTR.cy) / INTR.fy, np.ones_like(u, float)], -1)
    down = (rays @ mount.rotation().T)[..., 2]                       # course-z component per unit optical z
    depth = np.where(down < -1e-6, .9 / -down, np.nan)
    free, hit, _ = project_depth(depth, INTR, np.array([0., 0., .9]), np.eye(3), SPEC, mount=mount,
                                 max_range=3.5, stride=4)
    assert len(hit) == 0 and len(free) > 0


def test_decode_16uc1_and_32fc1():
    mm = np.array([[0, 1000], [2500, 65535]], dtype="<u2")
    img = decode_depth_image(mm.tobytes(), 2, 2, "16UC1")
    assert np.isnan(img[0, 0]) and img[0, 1] == 1.0 and img[1, 0] == 2.5
    m = np.array([[1.5, 0.0]], dtype="<f4")
    img = decode_depth_image(m.tobytes(), 2, 1, "32FC1")
    assert img[0, 0] == 1.5 and np.isnan(img[0, 1])
    with pytest.raises(ValueError):
        decode_depth_image(b"\x00" * 4, 2, 1, "rgb8")


def test_hits_on_published_walls_confirm_them():
    """Hits on a published wall must reach the planner (they confirm it); hits next to it are dropped.
    When they were all dropped, "doubt unconfirmed map" made walls the camera saw passable."""
    from pathplan.depth_projection import drop_misregistered
    prior = np.zeros((10, 10), bool)
    prior[:, 0] = True
    near = np.zeros_like(prior)
    near[:, 1] = True
    hit = np.array([[3, 0], [4, 1], [5, 6]])            # on the wall, next to it, out in the open
    free, kept, behind = drop_misregistered(np.empty((0, 2), int), hit, np.empty((0, 2), int), prior, near)
    assert {tuple(c) for c in kept} == {(3, 0), (5, 6)}

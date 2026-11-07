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



def test_align_to_map_recovers_drift_at_a_corner_and_only_sideways_in_a_corridor():
    """Depth hits on the wall surfaces, seen from a pose that is off by ``drift``."""
    from pathplan.depth_projection import align_to_map, map_distance_field
    from pathplan.world import GridSpec
    spec = GridSpec(half_size=1.5, resolution=.05)
    occ = np.zeros((spec.n, spec.n), bool)
    occ[:, :4] = True                                   # wall face at x = -1.3
    occ[:4, :] = True                                   # wall face at y = -1.3 (corner at bottom-left)
    dist, grad = map_distance_field(occ, spec)
    t = np.linspace(-1.2, 0.0, 40)
    x_wall = np.column_stack([np.full_like(t, -1.3), t])
    y_wall = np.column_stack([t, np.full_like(t, -1.3)])
    drift = np.array([.12, -.08])                       # hits appear shifted by the pose error
    shift, info = align_to_map(np.vstack([x_wall, y_wall]) + drift, dist, grad, spec)
    assert shift is not None and np.allclose(shift, -drift, atol=.03), (shift, info)
    shift, info = align_to_map(np.vstack([x_wall, x_wall + [0, .01]]) + drift, dist, grad, spec)   # one wall only
    assert shift is not None and abs(shift[0] + drift[0]) < .03 and abs(shift[1]) < .01, (shift, info)
    assert align_to_map(x_wall[:10] + drift, dist, grad, spec)[0] is None   # too few points: no guess


def test_align_to_map_is_not_dragged_by_a_moved_wall():
    """Two walls match the map; a third, seen as much, was moved 0.2 m: no drift -> ~no shift."""
    from pathplan.depth_projection import align_to_map, map_distance_field
    from pathplan.world import GridSpec
    spec = GridSpec(half_size=1.5, resolution=.05)
    occ = np.zeros((spec.n, spec.n), bool)
    occ[:, :4] = True                                   # face x = -1.3
    occ[:, -4:] = True                                  # face x = +1.3
    occ[:4, :] = True                                   # face y = -1.3
    dist, grad = map_distance_field(occ, spec)
    t = np.linspace(-1.2, 1.2, 40)
    left = np.column_stack([np.full_like(t, -1.3), t])
    right_moved = np.column_stack([np.full_like(t, 1.1), t])          # really 0.2 m further in
    bottom = np.column_stack([t, np.full_like(t, -1.3)])
    shift, info = align_to_map(np.vstack([left, left, right_moved, bottom]), dist, grad, spec)
    assert shift is None or abs(shift[0]) < .05, (shift, info)

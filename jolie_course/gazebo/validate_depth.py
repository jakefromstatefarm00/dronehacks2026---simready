"""Check the Gazebo depth geometry against truth, with the planner's own projection.

    python jolie_course/gazebo/validate_depth.py <run_dir> [--camera-offset X Y Z]
        [--camera-pitch-down-deg P] [--sweep] [--airborne]

<run_dir> holds depth/depth-frames.npz + depth/depth-info.json (rig/with_depth.py recorder)
and truth-poses.csv (sim/runtime/post_flight.sh; Gazebo world = course frame, base_link FLU).
Each recorded frame is projected with the embedded ``pathplan.depth_projection.project_depth``
of jolie_course/upload_to_flyx.py (the file that flies), at the truth pose nearest its stamp,
and scored like ``planner_process`` scores it: share of slab hits on published walls
(prior | within WALL_SNAP_M of it).  Independently of the slab, every depth point above the
floor is checked against the published course OBJ: its distance to the nearest obstacle
(bisected inflation radius, as score_run.py's clearance) should be ~0 for a correct mount.
--sweep repeats the 3-D check for perturbed mounts (pitch, yaw, offset) to show the minimum.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "pathfinding"))
from drl.course import build_course  # noqa: E402


def load_flyx():
    spec = importlib.util.spec_from_file_location("upload_to_flyx", ROOT / "jolie_course/upload_to_flyx.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)          # defines only; main() is behind __main__
    mod._install_embedded()
    return mod


def quat_wxyz_to_matrix(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def surface_distance(course, P, hi=0.6, iters=12):
    lo, h = np.zeros(len(P)), np.full(len(P), hi)
    inside = course.collides(P, 0.0, 0.0)
    for _ in range(iters):
        mid = (lo + h) / 2
        hit = course.collides(P, mid, mid)
        h = np.where(hit, mid, h)
        lo = np.where(hit, lo, mid)
    return np.where(inside, 0.0, lo)


def points_course(depth, intr, xyz, rot, mount, stride, min_range=.35, max_range=3.5):
    """All depth points (not only the slab) in the course frame, as project_depth computes them."""
    d = depth[::stride, ::stride]
    v, u = np.mgrid[0:d.shape[0], 0:d.shape[1]]
    u, v, z = u.ravel() * stride, v.ravel() * stride, d.ravel()
    ok = np.isfinite(z) & (z >= min_range) & (z <= max_range)
    u, v, z = u[ok], v[ok], z[ok]
    cam = np.column_stack([(u - intr.cx) / intr.fx * z, (v - intr.cy) / intr.fy * z, z])
    origin = xyz + rot @ np.asarray(mount.offset, float)
    return origin + cam @ (rot @ mount.rotation()).T


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--camera-offset", type=float, nargs=3, default=[.115, 0.0, .029])
    ap.add_argument("--camera-pitch-down-deg", type=float, default=0.0)
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--airborne", action="store_true", help="only frames with truth z > 0.5 m")
    ap.add_argument("--max-frames", type=int, default=400)
    a = ap.parse_args(argv)

    fx_mod = load_flyx()
    from pathplan.depth_projection import CameraMount, Intrinsics, project_depth
    from pathplan.world import clearance
    _, spec, env, prior = fx_mod._build_planner_embedded()
    near_wall = (clearance(prior, spec) < .25) & ~prior     # WALL_SNAP_M in planner_process
    course = build_course(str(ROOT / "pathfinding/course/dronehacks-course-v3.obj"))

    dd = a.run_dir / "depth"
    info = json.loads((dd / "depth-info.json").read_text())
    z = np.load(dd / "depth-frames.npz")
    stamps, frames, sub = z["stamps"], z["depth"].astype(float), int(z["subsample"])
    ci = info["camera_info"]
    K = np.array(ci["k"]).reshape(3, 3)
    # frames are depth[::sub, ::sub]: pixel i of the subsampled image is pixel sub*i of the full one
    intr = Intrinsics(K[0, 0] / sub, K[1, 1] / sub, K[0, 2] / sub, K[1, 2] / sub,
                      frames.shape[2], frames.shape[1])
    stride = max(1, 8 // sub)                    # planner default stride 8 on the full image
    truth = np.loadtxt(a.run_dir / "truth-poses.csv", delimiter=",", skiprows=1)

    mount = CameraMount(offset=tuple(a.camera_offset), pitch_down_deg=a.camera_pitch_down_deg)
    print(f"camera_info: {ci['width']}x{ci['height']} fx={K[0,0]:.2f} fy={K[1,1]:.2f} cx={K[0,2]:.2f} "
          f"cy={K[1,2]:.2f} frame_id={ci['frame_id']!r} model={ci['distortion_model']!r}")
    print(f"encoding {info['encoding']}; frames received {info['frames_received']} "
          f"({info['rate_hz_sim']:.1f} Hz sim); recorded {len(stamps)}; receive lag median "
          f"{info['receive_lag_sim_s']['median']*1e3:.0f} ms p95 {info['receive_lag_sim_s']['p95']*1e3:.0f} ms")

    sel = []
    for i, t in enumerate(stamps):
        j = int(np.argmin(np.abs(truth[:, 0] - t)))
        if abs(truth[j, 0] - t) > .05:
            continue
        if a.airborne and truth[j, 3] < .5:
            continue
        sel.append((i, j))
    sel = sel[:a.max_frames]
    print(f"frames with truth within 50 ms: {len(sel)}")

    def evaluate(m, yaw_err=0.0, verbose=False):
        q = {"hits": 0, "on_wall": 0}
        dists = []
        rz = np.array([[np.cos(yaw_err), -np.sin(yaw_err), 0], [np.sin(yaw_err), np.cos(yaw_err), 0], [0, 0, 1]])
        for i, j in sel:
            xyz = truth[j, 1:4]
            rot = quat_wxyz_to_matrix(*truth[j, 7:11]) @ rz
            free, hit, behind = project_depth(frames[i], intr, xyz, rot, spec, altitude=.9,
                                              half_height=env.half_height + env.tracking_margin,
                                              mount=m, stride=stride, expected=prior)
            if len(hit):
                q["hits"] += len(hit)
                q["on_wall"] += int((prior | near_wall)[hit[:, 0], hit[:, 1]].sum())
            P = points_course(frames[i], intr, xyz, rot, m, stride * 2)
            # not the floor (the OBJ has none) nor the Gazebo arena's perimeter beyond the map
            P = P[(P[:, 2] > .08) & (np.abs(P[:, :2]) < spec.half_size - .05).all(1)]
            if len(P):
                dists.append(surface_distance(course, P))
        d = np.concatenate(dists) if dists else np.array([np.nan])
        share = q["on_wall"] / q["hits"] if q["hits"] else float("nan")
        return share, q["hits"], float(np.median(d)), float(np.mean(d < .05)), len(d)

    share, hits, med, within, n = evaluate(mount)
    print(f"mount offset {tuple(a.camera_offset)} pitch_down {a.camera_pitch_down_deg} deg: "
          f"{share:.1%} of {hits} slab hit cells on published walls; 3-D: {n} points, median distance to "
          f"course geometry {med*100:.1f} cm, {within:.1%} within 5 cm")
    if a.sweep:
        print("sweep (3-D median distance / within 5 cm / % slab hits on walls):")
        for label, m, ye in [
                ("pitch_down +5", CameraMount(tuple(a.camera_offset), a.camera_pitch_down_deg + 5), 0),
                ("pitch_down -5", CameraMount(tuple(a.camera_offset), a.camera_pitch_down_deg - 5), 0),
                ("yaw +5", mount, np.deg2rad(5)), ("yaw -5", mount, np.deg2rad(-5)),
                ("offset x +0.10", CameraMount((a.camera_offset[0] + .1, *a.camera_offset[1:]),
                                               a.camera_pitch_down_deg), 0),
                ("offset z +0.10", CameraMount((*a.camera_offset[:2], a.camera_offset[2] + .1),
                                               a.camera_pitch_down_deg), 0)]:
            s, h, md, w, _ = evaluate(m, ye)
            print(f"  {label:16s} {md*100:5.1f} cm  {w:6.1%}  {s:6.1%} of {h}")


if __name__ == "__main__":
    main()

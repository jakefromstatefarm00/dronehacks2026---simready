#!/usr/bin/env python3
"""Build a 2-D occupancy grid of the REAL course from the depth images of an RC run and compare it
with the published course.

    python -I tools/rc_bag/depth_map.py BAG [--out DIR] [--align first-pose|odom-origin|none]
                                            [--stride 3] [--res 0.05] [--band 0.5] [--pose auto|vio|px4]

Each sampled depth frame (every --stride-th; range 0.35-4.0 m; 0 and <= 0.314 m ignored) is projected
with the time-matched pose (cuVSLAM odometry by default; header stamps, nearest within 0.1 s, position
interpolated) through the camera mount (base_link -> camera x 0.1053, z 0.0304, no rotation; optical
frame z forward, x right, y down). Points near the floor are removed (z < floor + 0.15 m); the rest
within +-band of the flight height (median height while airborne) go into a 5 cm grid that counts how
many frames saw each cell. A cell is occupied when seen by >= --min-frames frames.

Outputs (in --out, default ./rc_bag_out/<bag name>):
  depth_map.png         grid + published course + trajectory + new obstacles
  depth_occupancy.npy   uint16 frames-per-cell (row = y, col = x); depth_occupied.npy (bool)
  depth_map.json        grid origin / resolution / frame and all the numbers below
  depth_map.md          report: distance of observed walls to the published ones (median / p90),
                        new obstacles (not in the published map), published walls confirmed by depth
With no pose in the bag (depth-only), it says "no odometry" and still summarises the depth stream.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))   # -I drops the script dir from sys.path

import numpy as np  # noqa: E402

from course_geom import ARENA_HALF, Alignment, Footprints  # noqa: E402
from read_bag import (CAM_OFFSET, Bag, camera_intrinsics, default_out_dir, depth_to_metres,  # noqa: E402
                      load_odometry, load_px4_local, load_px4_odometry, quat_to_R, stamp_s)

DEPTH_TOPIC = "/camera/depth/image_raw"
INFO_TOPIC = "/camera/depth/camera_info"
MIN_VALID = 0.314           # the OAK reports <= 0.314 m for "too close"


# ---------------------------------------------------------------------- depth-only statistics
def depth_stats(bag: Bag, topic: str = DEPTH_TOPIC, stride: int = 3, rmin=0.35, rmax=4.0) -> dict | None:
    if not bag.has(topic):
        return None
    tb = bag.times(topic)
    t, valid, inrange, med, p10, sizes, enc = [], [], [], [], [], set(), set()
    sample = None
    allv = []
    for i, (_, tn, m) in enumerate(bag.messages(topic, stride=stride)):
        d = depth_to_metres(m)
        sizes.add((m.width, m.height)), enc.add(m.encoding)
        v = d[d > MIN_VALID]
        t.append(tn / 1e9)
        valid.append(v.size / d.size)
        inrange.append(((d >= rmin) & (d <= rmax)).sum() / d.size)
        med.append(float(np.median(v)) if v.size else np.nan)
        p10.append(float(np.percentile(v, 10)) if v.size else np.nan)
        if v.size:
            allv.append(np.random.default_rng(i).choice(v, min(v.size, 2000), replace=False))
        if sample is None and i * stride >= len(tb) // 2:
            sample = (tn / 1e9, d)
    allv = np.concatenate(allv) if allv else np.zeros(0)
    dt = np.diff(tb)
    return dict(topic=topic, frames=len(tb), sampled=len(t), duration=float(tb[-1] - tb[0]) if len(tb) > 1 else 0.0,
                rate=float((len(tb) - 1) / (tb[-1] - tb[0])) if len(tb) > 1 and tb[-1] > tb[0] else 0.0,
                max_gap=float(dt.max()) if len(dt) else 0.0, sizes=sorted(sizes), encodings=sorted(enc),
                valid_frac=float(np.mean(valid)) if valid else 0.0,
                inrange_frac=float(np.mean(inrange)) if inrange else 0.0,
                range_pct={p: float(np.percentile(allv, p)) for p in (5, 25, 50, 75, 95)} if allv.size else {},
                t=np.array(t), med=np.array(med), p10=np.array(p10), valid=np.array(valid),
                inrange=np.array(inrange), sample=sample)


def depth_stats_md(s: dict, t0: float | None = None) -> list[str]:
    if s is None:
        return ["- no depth image topic"]
    r = s["range_pct"]
    L = [f"- `{s['topic']}`: {s['frames']} frames over {s['duration']:.1f} s ({s['rate']:.1f} Hz), "
         f"max gap {s['max_gap'] * 1000:.0f} ms, {', '.join(f'{w}x{h}' for w, h in s['sizes'])} {', '.join(s['encodings'])}",
         f"- sampled {s['sampled']} frames: valid pixels (> {MIN_VALID} m) {100 * s['valid_frac']:.1f} %, "
         f"in 0.35-4.0 m {100 * s['inrange_frac']:.1f} %"]
    if r:
        L.append("- valid range percentiles 5/25/50/75/95: " + " / ".join(f"{r[p]:.2f}" for p in (5, 25, 50, 75, 95)) + " m")
    if len(s["med"]):
        k = np.nanargmin(s["p10"]) if np.isfinite(s["p10"]).any() else 0
        L.append(f"- closest scene (10th-percentile range) {np.nanmin(s['p10']):.2f} m at t+{s['t'][k] - (t0 or s['t'][0]):.1f} s")
    return L


def plot_depth_stats(s: dict, out: Path, title: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(12, 3.8), gridspec_kw=dict(width_ratios=[1.6, 1]))
    t = s["t"] - s["t"][0] if len(s["t"]) else s["t"]
    ax = axs[0]
    ax.plot(t, s["med"], label="median valid range (m)", color="tab:blue")
    ax.plot(t, s["p10"], label="10th percentile range (m)", color="tab:red", lw=0.8)
    ax.set_xlabel("time (s)"), ax.set_ylabel("m"), ax.grid(alpha=0.3)
    ax2 = ax.twinx()
    ax2.plot(t, 100 * s["valid"], color="0.5", lw=0.8, label="valid pixels (%)")
    ax2.set_ylabel("valid pixels (%)"), ax2.set_ylim(0, 100)
    ax.legend(loc="upper left", fontsize=7), ax2.legend(loc="upper right", fontsize=7)
    ax.set_title(f"{title}: depth stream", fontsize=9)
    if s["sample"] is not None:
        ts, d = s["sample"]
        im = axs[1].imshow(np.where(d > MIN_VALID, d, np.nan), cmap="turbo", vmin=0.3, vmax=4.0)
        fig.colorbar(im, ax=axs[1], label="m", shrink=0.8)
        axs[1].set_title(f"sample frame t+{ts - s['t'][0]:.1f} s (white = no data)", fontsize=9)
        axs[1].axis("off")
    fig.tight_layout()
    fig.savefig(out / "depth_stats.png", dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------------- mapping
def _components(mask: np.ndarray):
    """8-connected components of a bool grid -> list of (rows, cols) index arrays."""
    seen = np.zeros_like(mask, bool)
    H, W = mask.shape
    comps = []
    for r0, c0 in zip(*np.nonzero(mask)):
        if seen[r0, c0]:
            continue
        stack, rr, cc = [(r0, c0)], [], []
        seen[r0, c0] = True
        while stack:
            r, c = stack.pop()
            rr.append(r), cc.append(c)
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    a, b = r + dr, c + dc
                    if 0 <= a < H and 0 <= b < W and mask[a, b] and not seen[a, b]:
                        seen[a, b] = True
                        stack.append((a, b))
        comps.append((np.array(rr), np.array(cc)))
    return comps


def load_pose(bag: Bag, which: str):
    """-> pose dict (t, pos, quat, yaw, header_time) or None. auto = VIO, else PX4."""
    p = None
    if which in ("auto", "vio"):
        p = load_odometry(bag)
        if p is not None:
            p["header_time"] = True          # match depth header stamps (same Jetson clock)
    if p is None and which in ("auto", "px4"):
        p = load_px4_odometry(bag)
        if p is None:
            p = load_px4_local(bag)
            if p is not None:                # heading only: assume level attitude
                y = p["yaw"]
                p["quat"] = np.c_[np.zeros_like(y), np.zeros_like(y), np.sin(y / 2), np.cos(y / 2)]
        if p is not None:
            p["header_time"] = False         # PX4 stamps are boot time: use bag receive time
    if p is not None:
        o = np.argsort(p["t"], kind="stable")
        for k in ("t", "tb", "pos", "quat", "yaw"):
            p[k] = np.asarray(p[k])[o]
    return p


def run(bag_path, out=None, align="first-pose", stride=3, pix_step=4, rmin=0.35, rmax=4.0, res=0.05,
        band=0.5, floor_margin=0.15, min_frames=2, new_thresh=0.25, min_new_cells=6, pose="auto",
        max_dt=0.1, px4_msgs=None) -> dict:
    out = default_out_dir(bag_path, out)
    title = Path(bag_path).name
    rmin = max(rmin, MIN_VALID + 1e-3)
    with Bag(bag_path, px4_msgs) as bag:
        if not bag.has(DEPTH_TOPIC):
            msg = f"no depth: {DEPTH_TOPIC} is not in this bag (recorded with --no-depth?)"
            print(msg)
            (out / "depth_map.md").write_text(f"# Depth map: {title}\n\n{msg}\n", encoding="utf-8")
            return {"status": "no depth", "out": str(out)}
        P = load_pose(bag, pose)
        if P is None:
            s = depth_stats(bag, stride=stride, rmin=rmin, rmax=rmax)
            plot_depth_stats(s, out, title)
            L = [f"# Depth map: {title}", "",
                 "**no odometry**: the bag has no /visual_slam/tracking/odometry or /fmu/out pose, so the depth "
                 "cannot be placed in the course; no occupancy grid was built. Depth stream summary:", "",
                 *depth_stats_md(s), "", "![depth stats](depth_stats.png)", ""]
            (out / "depth_map.md").write_text("\n".join(L), encoding="utf-8")
            print("\n".join(L[2:-2]))
            print(f"wrote {out}")
            return {"status": "no odometry", "out": str(out), "depth": s}

        A = Alignment(align, P["pos"][0], P["yaw"][0])
        Rs = quat_to_R(P["quat"])
        cpos = A.points(P["pos"])
        floor = A.floor_z
        air = cpos[:, 2] > floor + 0.3
        flight_h = float(np.median(cpos[air, 2])) if air.any() else float(np.median(cpos[:, 2]) + CAM_OFFSET[2])
        z_lo, z_hi = max(flight_h - band, floor + floor_margin), flight_h + band

        pts, frame_ids = [], []
        n_used = n_skip = 0
        K = None
        for i, (_, tn, m) in enumerate(bag.messages(DEPTH_TOPIC, stride=stride)):
            hs = stamp_s(m)
            t = hs if (P["header_time"] and hs is not None) else tn / 1e9
            j = int(np.searchsorted(P["t"], t))
            cand = [k for k in (j - 1, j) if 0 <= k < len(P["t"])]
            k = min(cand, key=lambda k: abs(P["t"][k] - t))
            if abs(P["t"][k] - t) > max_dt:
                n_skip += 1
                continue
            d = depth_to_metres(m)
            if K is None:
                fx, fy, cx, cy, ksrc = camera_intrinsics(bag, m.width, m.height, INFO_TOPIC)
                vv, uu = np.mgrid[0:m.height:pix_step, 0:m.width:pix_step]
                K = ((uu - cx) / fx).ravel(), ((vv - cy) / fy).ravel(), vv.ravel(), uu.ravel()
            ux, vy, vi, ui = K
            z = d[vi, ui]
            ok = (z > MIN_VALID) & (z >= rmin) & (z <= rmax)
            z = z[ok]
            # optical (x right, y down, z fwd) -> body FLU, then the mount offset
            pb = np.c_[z, -ux[ok] * z, -vy[ok] * z] + CAM_OFFSET
            pos = np.array([np.interp(t, P["t"], P["pos"][:, a]) for a in range(3)])
            pw = A.points(pb @ Rs[k].T + pos)
            sel = (pw[:, 2] >= z_lo) & (pw[:, 2] <= z_hi)
            pts.append(pw[sel, :2])
            frame_ids.append(np.full(sel.sum(), i, np.int32))
            n_used += 1
        pts = np.concatenate(pts) if pts else np.zeros((0, 2))
        frame_ids = np.concatenate(frame_ids) if frame_ids else np.zeros(0, np.int32)

    # grid
    if A.course:
        lo = np.array([-ARENA_HALF - 0.25] * 2)
        hi = -lo
    else:
        allxy = np.r_[pts, cpos[:, :2]] if len(pts) else cpos[:, :2]
        lo = np.percentile(allxy, 0.5, axis=0) - 0.5
        hi = np.percentile(allxy, 99.5, axis=0) + 0.5
        hi = np.minimum(hi, lo + 40)
    W, H = (np.ceil((hi - lo) / res).astype(int))
    ij = np.floor((pts - lo) / res).astype(int)
    inside = (ij[:, 0] >= 0) & (ij[:, 0] < W) & (ij[:, 1] >= 0) & (ij[:, 1] < H)
    n_outside = int((~inside).sum())
    cell = ij[inside, 1] * W + ij[inside, 0]
    uniq = np.unique(np.c_[cell, frame_ids[inside]], axis=0) if len(cell) else np.zeros((0, 2), int)
    frames = np.bincount(uniq[:, 0], minlength=W * H).reshape(H, W).astype(np.uint16) if len(uniq) \
        else np.zeros((H, W), np.uint16)
    occ = frames >= min_frames
    rr, cc = np.nonzero(occ)
    cxy = np.c_[lo[0] + (cc + 0.5) * res, lo[1] + (rr + 0.5) * res]

    res_d = dict(status="ok", bag=str(bag_path), align=A.describe(), pose_topic=P["topic"],
                 frames_used=n_used, frames_skipped_no_pose=n_skip, stride=stride, pixel_step=pix_step,
                 range_m=[rmin, rmax], flight_height_m=flight_h, band_m=[z_lo, z_hi], floor_z=floor,
                 grid=dict(origin_xy=lo.tolist(), resolution=res, shape_rows_cols=[int(H), int(W)],
                           frame="course" if A.course else "odom", min_frames=min_frames),
                 points=int(len(pts)), points_outside_grid=n_outside, occupied_cells=int(occ.sum()))

    new_obs, confirmed, fp = [], [], None
    if A.course and len(cxy):
        fp = Footprints(z_lo, z_hi)
        dist, near = fp.distance(cxy)
        in_arena = (np.abs(cxy) <= ARENA_HALF - 0.05).all(1)
        dd = dist[in_arena]
        res_d["wall_distance"] = dict(
            cells_in_arena=int(in_arena.sum()), cells_outside_net=int((~in_arena).sum()),
            median_m=float(np.median(dd)) if dd.size else None,
            p90_m=float(np.percentile(dd, 90)) if dd.size else None,
            within_10cm=float((dd <= 0.10).mean()) if dd.size else None,
            within_25cm=float((dd <= 0.25).mean()) if dd.size else None)
        newmask = np.zeros_like(occ)
        sel = in_arena & (dist > new_thresh)
        newmask[rr[sel], cc[sel]] = True
        dmap = np.full(occ.shape, np.nan)
        dmap[rr, cc] = dist
        for r_, c_ in _components(newmask):
            if len(r_) < min_new_cells:
                continue
            x = lo[0] + (c_ + 0.5) * res
            y = lo[1] + (r_ + 0.5) * res
            dmin, nm = fp.distance(np.c_[x.mean(), y.mean()])
            new_obs.append(dict(centre=[round(float(x.mean()), 3), round(float(y.mean()), 3)],
                                x_range=[round(float(x.min() - res / 2), 3), round(float(x.max() + res / 2), 3)],
                                y_range=[round(float(y.min() - res / 2), 3), round(float(y.max() + res / 2), 3)],
                                cells=int(len(r_)), area_m2=round(len(r_) * res * res, 4),
                                max_frames=int(frames[r_, c_].max()),
                                min_dist_to_published_m=round(float(np.nanmin(dmap[r_, c_])), 3),
                                nearest_published=nm[0]))
        new_obs.sort(key=lambda o: -o["cells"])
        res_d["new_obstacles"] = new_obs
        # published walls confirmed by depth (only walls the drone came within range of)
        samp, names = fp.outline_samples(0.05)
        if len(samp):
            from collections import defaultdict
            near_traj = np.min(np.linalg.norm(samp[:, None] - cpos[None, ::5, :2], axis=-1), axis=1) <= rmax
            occ_tree = cxy
            dmin = np.array([np.min(np.linalg.norm(occ_tree - s_, axis=1)) for s_ in samp]) if len(occ_tree) else np.full(len(samp), np.inf)
            agg = defaultdict(lambda: [0, 0, 0])
            for nm_, nt, dm in zip(names, near_traj, dmin):
                a_ = agg[nm_]
                a_[0] += 1
                a_[1] += int(nt)
                a_[2] += int(nt and dm <= 0.15)
            for nm_, (tot, inr, ok) in sorted(agg.items()):
                confirmed.append(dict(name=nm_, outline_pts=tot, within_range=inr,
                                      confirmed_frac=round(ok / inr, 3) if inr else None))
        res_d["published_confirmed"] = confirmed

    np.save(out / "depth_occupancy.npy", frames)
    np.save(out / "depth_occupied.npy", occ)
    (out / "depth_map.json").write_text(json.dumps(res_d, indent=2), encoding="utf-8")
    _plot_map(out, title, frames, occ, lo, res, cpos, fp, new_obs, A, res_d)
    md = _report_md(title, res_d, new_obs, confirmed)
    (out / "depth_map.md").write_text(md, encoding="utf-8")
    print(md)
    print(f"wrote {out}")
    res_d["out"] = str(out)
    res_d["occupied_xy"] = cxy
    return res_d


def _plot_map(out, title, frames, occ, lo, res, cpos, fp, new_obs, A, d):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.patches as mp
    import matplotlib.pyplot as plt
    H, W = frames.shape
    ext = [lo[0], lo[0] + W * res, lo[1], lo[1] + H * res]
    fig, ax = plt.subplots(figsize=(8, 8))
    img = np.where(occ, np.log1p(frames.astype(float)), np.nan)
    ax.imshow(img, origin="lower", extent=ext, cmap="viridis", interpolation="nearest")
    if fp is not None:
        fp.draw(ax, color="tab:red", lw=1.0, fill=False, label="published course (at flight height)")
    ax.plot(cpos[:, 0], cpos[:, 1], color="white", lw=2.2)
    ax.plot(cpos[:, 0], cpos[:, 1], color="black", lw=0.9, label="trajectory")
    for k, o in enumerate(new_obs):
        (x0, x1), (y0, y1) = o["x_range"], o["y_range"]
        ax.add_patch(mp.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="magenta", lw=1.5,
                                  label="new obstacle (not published)" if k == 0 else None))
        ax.text(x1, y1, str(k + 1), color="magenta", fontsize=8)
    ax.set_xlim(ext[0], ext[1]), ax.set_ylim(ext[2], ext[3])
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)"), ax.set_ylabel("y (m)")
    wd = d.get("wall_distance") or {}
    extra = (f"; observed->published median {wd['median_m']:.2f} m, p90 {wd['p90_m']:.2f} m"
             if wd.get("median_m") is not None else "")
    ax.set_title(f"{title}: depth occupancy ({res * 100:.0f} cm, z {d['band_m'][0]:.2f}-{d['band_m'][1]:.2f} m, "
                 f"{'course' if A.course else 'odom'} frame)\ncolour = log frames per cell{extra}", fontsize=9)
    ax.grid(alpha=0.2)
    ax.legend(loc="upper right", fontsize=7)
    fig.tight_layout()
    fig.savefig(out / "depth_map.png", dpi=140)
    plt.close(fig)


def _report_md(title, d, new_obs, confirmed) -> str:
    L = [f"# Depth map: {title}", "",
         f"- pose: `{d['pose_topic']}`; alignment {d['align']}",
         f"- frames used {d['frames_used']} (stride {d['stride']}, pixel step {d['pixel_step']}); "
         f"skipped (no pose within 0.1 s) {d['frames_skipped_no_pose']}",
         f"- flight height {d['flight_height_m']:.2f} m; mapped band z {d['band_m'][0]:.2f}-{d['band_m'][1]:.2f} m "
         f"(floor {d['floor_z']:.2f} m removed below +0.15 m); range {d['range_m'][0]:.2f}-{d['range_m'][1]:.2f} m",
         f"- {d['points']} points, {d['occupied_cells']} occupied {d['grid']['resolution'] * 100:.0f} cm cells "
         f"(seen in >= {d['grid']['min_frames']} frames)", ""]
    wd = d.get("wall_distance")
    if wd:
        if wd["median_m"] is None:
            L += ["No occupied cells inside the arena.", ""]
        else:
            L += ["## Observed vs published walls", "",
                  "Distance from each occupied cell inside the net to the nearest published surface:",
                  f"**median {wd['median_m']:.3f} m, 90th percentile {wd['p90_m']:.3f} m**; "
                  f"{100 * wd['within_10cm']:.0f} % within 10 cm, {100 * wd['within_25cm']:.0f} % within 25 cm "
                  f"({wd['cells_in_arena']} cells; {wd['cells_outside_net']} cells outside the net ignored).", ""]
        L += ["## New obstacles (occupied, > 0.25 m from anything published)", ""]
        if new_obs:
            L += ["| # | centre x, y (m) | x range | y range | area m^2 | frames | nearest published |",
                  "|---|---|---|---|---|---|---|"]
            for k, o in enumerate(new_obs[:25]):
                L.append(f"| {k + 1} | {o['centre'][0]:.2f}, {o['centre'][1]:.2f} | {o['x_range'][0]:.2f}..{o['x_range'][1]:.2f} "
                         f"| {o['y_range'][0]:.2f}..{o['y_range'][1]:.2f} | {o['area_m2']:.3f} | {o['max_frames']} "
                         f"| {o['nearest_published']} ({o['min_dist_to_published_m']:.2f} m) |")
        else:
            L.append("none")
        L += ["", "## Published walls confirmed by depth", "",
              "Fraction of each published outline (within range of the trajectory) with an occupied cell within 15 cm. "
              "Low values next to the flown path suggest a wall that moved or is missing.", "",
              "| published | outline pts in range | confirmed |", "|---|---|---|"]
        for c in confirmed:
            if c["within_range"]:
                L.append(f"| {c['name']} | {c['within_range']} | {100 * c['confirmed_frac']:.0f} % |")
    else:
        L += ["(alignment none: no comparison with the published course)"]
    L += ["", "![depth map](depth_map.png)", ""]
    return "\n".join(L)


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")   # Windows consoles: never die on a stray character
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--out")
    ap.add_argument("--align", default="first-pose", choices=["first-pose", "odom-origin", "none"])
    ap.add_argument("--pose", default="auto", choices=["auto", "vio", "px4"])
    ap.add_argument("--stride", type=int, default=3, help="use every Nth depth frame")
    ap.add_argument("--pixel-step", type=int, default=4, help="use every Nth pixel in u and v")
    ap.add_argument("--rmin", type=float, default=0.35)
    ap.add_argument("--rmax", type=float, default=4.0)
    ap.add_argument("--res", type=float, default=0.05, help="grid cell (m)")
    ap.add_argument("--band", type=float, default=0.5, help="+- m around the flight height")
    ap.add_argument("--min-frames", type=int, default=2)
    ap.add_argument("--new-thresh", type=float, default=0.25, help="m from published = new obstacle")
    ap.add_argument("--px4-msgs")
    a = ap.parse_args(argv)
    run(a.bag, a.out, a.align, a.stride, a.pixel_step, a.rmin, a.rmax, a.res, a.band,
        min_frames=a.min_frames, new_thresh=a.new_thresh, pose=a.pose, px4_msgs=a.px4_msgs)
    return 0


if __name__ == "__main__":
    sys.exit(main())

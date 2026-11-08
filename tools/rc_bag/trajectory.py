#!/usr/bin/env python3
"""Export the VIO and PX4 trajectories of an RC run to CSV and plot them over the published course.

    python -I tools/rc_bag/trajectory.py BAG [--out DIR] [--align first-pose|odom-origin|none]

Writes  trajectory_vio.csv   cuVSLAM /visual_slam/tracking/odometry (ENU odom) + course-frame columns
        trajectory_px4.csv   PX4 /fmu/out/vehicle_local_position (NED -> ENU), or vehicle_odometry
        trajectory_topdown.png, trajectory_height.png
Each trajectory is aligned by ITS OWN first pose (--align first-pose, default): the start is put on the
launch mark (-2.515, 0.914) facing +y. The PX4 local frame has a different origin/heading than cuVSLAM,
so the overlay shows drift between them, not a frame offset.
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))   # -I drops the script dir from sys.path

import numpy as np  # noqa: E402

from course_geom import Alignment, Footprints  # noqa: E402
from read_bag import (Bag, default_out_dir, load_odometry, load_px4_local,  # noqa: E402
                      load_px4_odometry, load_skybeta_state)

VIO_TOPICS = ("/visual_slam/tracking/odometry",)


def load_all(bag: Bag, align: str):
    """-> dict name -> trajectory dict with 'c' (course/output-frame positions) and 'align'."""
    out = {}
    vio = None
    for t in VIO_TOPICS:
        vio = load_odometry(bag, t)
        if vio is not None:
            break
    px4 = load_px4_local(bag) or load_px4_odometry(bag)
    sky = load_skybeta_state(bag)
    sky_tr = None
    pts = [(t, s["local_enu"]) for t, s in sky if isinstance(s.get("local_enu"), (list, tuple))
           and len(s["local_enu"]) >= 3]
    if pts:
        sky_tr = dict(topic="/skybeta/state local_enu", t=np.array([p[0] for p in pts]),
                      pos=np.array([p[1][:3] for p in pts], float))
        sky_tr["tb"] = sky_tr["t"]
        sky_tr["yaw"] = None
    for name, tr in (("vio", vio), ("px4", px4), ("skybeta", sky_tr)):
        if tr is None:
            continue
        if tr.get("yaw") is not None:
            yaw0 = float(tr["yaw"][0])
        else:   # local_enu has no yaw: it follows the PX4 local frame (flight_interface), else VIO
            ref = px4 or vio
            yaw0 = float(ref["yaw"][0]) if ref else 0.0
        a = Alignment(align, tr["pos"][0], yaw0)
        tr["align"] = a
        tr["c"] = a.points(tr["pos"])
        out[name] = tr
    return out


def write_csv(path: Path, tr: dict, t0: float):
    cols = ["t_rel_s", "t_stamp_s", "x", "y", "z", "yaw_rad", "cx", "cy", "cz", "cyaw_rad"]
    has_cov = "cov" in tr
    if has_cov:
        cols += ["var_x", "var_y", "var_z", "var_roll", "var_pitch", "var_yaw"]
    yaw = tr.get("yaw")
    cyaw = tr["align"].yaw(yaw) if yaw is not None else None
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for i in range(len(tr["t"])):
            row = [f"{tr['tb'][i] - t0:.4f}", f"{tr['t'][i]:.6f}", *(f"{v:.4f}" for v in tr["pos"][i]),
                   f"{yaw[i]:.4f}" if yaw is not None else "", *(f"{v:.4f}" for v in tr["c"][i]),
                   f"{cyaw[i]:.4f}" if cyaw is not None else ""]
            if has_cov:
                row += [f"{v:.6g}" for v in tr["cov"][i]]
            w.writerow(row)


STYLE = {"vio": dict(color="tab:blue", lw=1.6, label="VIO (cuVSLAM odometry)"),
         "px4": dict(color="tab:orange", lw=1.2, ls="--", label="PX4 EKF local position"),
         "skybeta": dict(color="tab:purple", lw=0.8, ls=":", label="/skybeta/state local_enu")}


def plot(trs: dict, out: Path, align: str, t0: float, title: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7.5, 7.5))
    if align != "none":
        mean_z = np.median(next(iter(trs.values()))["c"][:, 2])
        Footprints(mean_z - 0.3, mean_z + 0.3).draw(ax)
        ax.set_xlim(-3.3, 3.3), ax.set_ylim(-3.3, 3.3)
    for name, tr in trs.items():
        c = tr["c"]
        ax.plot(c[:, 0], c[:, 1], **STYLE[name])
        ax.plot(c[0, 0], c[0, 1], "o", color=STYLE[name]["color"], ms=5)
        ax.plot(c[-1, 0], c[-1, 1], "s", color=STYLE[name]["color"], ms=5)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x (m)"), ax.set_ylabel("y (m)")
    frame = "course frame" if align != "none" else "raw odom frame"
    ax.set_title(f"{title}\ntop-down, {frame} (align: {align}); o start, square end", fontsize=9)
    ax.legend(loc="upper right", fontsize=7)
    fig.tight_layout()
    fig.savefig(out / "trajectory_topdown.png", dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 3.6))
    for name, tr in trs.items():
        ax.plot(tr["tb"] - t0, tr["c"][:, 2], **STYLE[name])
    ax.set_xlabel("time since bag start (s)")
    ax.set_ylabel("height above floor (m)" if align != "none" else "z (m, odom)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    ax.set_title(f"{title}: height vs time", fontsize=9)
    fig.tight_layout()
    fig.savefig(out / "trajectory_height.png", dpi=130)
    plt.close(fig)


def stats(tr: dict) -> str:
    c = tr["c"]
    seg = np.linalg.norm(np.diff(c, axis=0), axis=1) if len(c) > 1 else np.zeros(0)
    dur = tr["tb"][-1] - tr["tb"][0]
    return (f"{len(c)} poses over {dur:.1f} s, path {seg.sum():.2f} m, max height {c[:, 2].max():.2f} m, "
            f"end ({c[-1, 0]:.2f}, {c[-1, 1]:.2f}, {c[-1, 2]:.2f})")


def run(bag_path, out=None, align="first-pose", px4_msgs=None) -> dict:
    out = default_out_dir(bag_path, out)
    with Bag(bag_path, px4_msgs) as bag:
        trs = load_all(bag, align)
        starts = [tr["tb"][0] for tr in trs.values()]
    if not trs:
        msg = "no odometry: no /visual_slam/tracking/odometry, /fmu/out/vehicle_local_position or /skybeta/state pose in this bag"
        print(msg)
        (out / "trajectory.txt").write_text(msg + "\n", encoding="utf-8")
        return {"status": "no odometry", "out": str(out)}
    t0 = min(starts)
    lines = []
    for name, tr in trs.items():
        write_csv(out / f"trajectory_{name}.csv", tr, t0)
        lines.append(f"{name:8s} {tr['topic']}: {stats(tr)}")
        lines.append(f"         align {tr['align'].describe()}")
    if "vio" in trs and "px4" in trs:
        # VIO vs PX4 difference at common times (both aligned at their own start)
        v, p = trs["vio"], trs["px4"]
        tt = v["tb"][(v["tb"] >= p["tb"][0]) & (v["tb"] <= p["tb"][-1])]
        if len(tt):
            pi = np.stack([np.interp(tt, p["tb"], p["c"][:, k]) for k in range(3)], 1)
            vi = np.stack([np.interp(tt, v["tb"], v["c"][:, k]) for k in range(3)], 1)
            d = np.linalg.norm(pi - vi, axis=1)
            lines.append(f"VIO vs PX4 EKF: median {np.median(d):.2f} m, max {d.max():.2f} m, final {d[-1]:.2f} m")
    plot(trs, out, align, t0, Path(bag_path).name)
    txt = "\n".join(lines)
    print(txt)
    print(f"wrote {out}")
    (out / "trajectory.txt").write_text(txt + "\n", encoding="utf-8")
    return {"status": "ok", "out": str(out), "trajectories": {k: v for k, v in trs.items()}}


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")   # Windows consoles: never die on a stray character
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--out", help="output folder (default ./rc_bag_out/<bag name>)")
    ap.add_argument("--align", default="first-pose", choices=["first-pose", "odom-origin", "none"])
    ap.add_argument("--px4-msgs", help="folder with px4_msgs .msg files")
    a = ap.parse_args(argv)
    run(a.bag, a.out, a.align, a.px4_msgs)
    return 0


if __name__ == "__main__":
    sys.exit(main())

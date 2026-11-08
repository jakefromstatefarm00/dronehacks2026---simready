#!/usr/bin/env python3
"""Summarise an RC-run bag: topics, flight timeline, RC sticks, battery, VIO health, depth, CPU.

    python -I tools/rc_bag/summarize.py BAG [--out DIR] [--px4-msgs DIR]

Writes summary.md (default ./rc_bag_out/<bag name>/summary.md) and prints it.
tegrastats.log is picked up from the bag folder or its parent (record_rc_run.sh layout).
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))   # -I drops the script dir from sys.path

import numpy as np  # noqa: E402

from depth_map import depth_stats, depth_stats_md  # noqa: E402
from read_bag import Bag, default_out_dir, find_side_file, load_odometry, load_skybeta_state  # noqa: E402

NAV_STATE = {0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION", 4: "AUTO_LOITER", 5: "AUTO_RTL",
             6: "POSITION_SLOW", 10: "ACRO", 12: "AUTO_DESCEND", 13: "TERMINATION", 14: "OFFBOARD",
             15: "STAB", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND", 19: "AUTO_FOLLOW_TARGET", 20: "AUTO_PRECLAND",
             21: "ORBIT", 22: "AUTO_VTOL_TAKEOFF"}
ARMING = {1: "DISARMED", 2: "ARMED"}
GAP_ABS_S = 0.25


def topic_table(bag: Bag, t0: float):
    rows, times = [], {}
    for name in sorted(bag.topics):
        info = bag.topics[name]
        tb = bag.times(name)
        times[name] = tb
        if len(tb) < 2:
            rows.append((name, info.msgtype, len(tb), 0.0, 0.0, 0, info.decodable, tb[0] - t0 if len(tb) else 0))
            continue
        dt = np.diff(tb)
        med = float(np.median(dt))
        gaps = int((dt > max(5 * med, GAP_ABS_S)).sum())
        rate = (len(tb) - 1) / (tb[-1] - tb[0]) if tb[-1] > tb[0] else 0.0
        rows.append((name, info.msgtype, len(tb), rate, float(dt.max()), gaps, info.decodable, tb[0] - t0))
    return rows, times


def changes(series):
    """[(t, value)] -> only the entries where the value changes."""
    out, last = [], object()
    for t, v in series:
        if v != last:
            out.append((t, v))
            last = v
    return out


def skybeta_timeline(bag: Bag, t0: float) -> list[str]:
    st = load_skybeta_state(bag)
    if not st:
        return []
    L = []
    for key in ("px4_link", "armed", "landed", "offboard_active", "nav_state"):
        ch = changes([(t, s.get(key)) for t, s in st if key in s])
        if not ch:
            continue
        L.append(f"- `{key}`: " + ", ".join(f"{v} @ {t - t0:.1f} s" for t, v in ch[:30])
                 + (" ..." if len(ch) > 30 else ""))
    o = [s.get("origin_enu") for _, s in st if s.get("origin_enu") is not None]
    if o:
        L.append(f"- `origin_enu` first {o[0]}, last {o[-1]}")
    return L


def vehicle_status_timeline(bag: Bag, t0: float) -> list[str]:
    topic = bag.first_topic("/fmu/out/vehicle_status_v1", "/fmu/out/vehicle_status")
    if topic is None:
        return []
    arm, nav, fs = [], [], []
    for _, tn, m in bag.messages(topic):
        t = tn / 1e9
        arm.append((t, ARMING.get(int(m.arming_state), str(m.arming_state))))
        nav.append((t, NAV_STATE.get(int(m.nav_state), str(m.nav_state))))
        fs.append((t, bool(getattr(m, "failsafe", False))))
    L = [f"- `{topic}` arming: " + ", ".join(f"{v} @ {t - t0:.1f} s" for t, v in changes(arm)),
         "- nav_state: " + ", ".join(f"{v} @ {t - t0:.1f} s" for t, v in changes(nav)[:40])]
    f = changes(fs)
    if any(v for _, v in f):
        L.append("- failsafe: " + ", ".join(f"{v} @ {t - t0:.1f} s" for t, v in f))
    arms = [(t, v) for t, v in changes(arm)]
    on = None
    total = 0.0
    for t, v in arms:
        if v == "ARMED" and on is None:
            on = t
        elif v != "ARMED" and on is not None:
            total += t - on
            on = None
    if on is not None:
        total += arm[-1][0] - on
    L.append(f"- armed time {total:.1f} s")
    return L


def failsafe_flags(bag: Bag, t0: float) -> list[str]:
    topic = bag.first_topic("/fmu/out/failsafe_flags")
    if topic is None:
        return []
    first = {}
    for _, tn, m in bag.messages(topic):
        for k in m.__dataclass_fields__ if hasattr(m, "__dataclass_fields__") else []:
            v = getattr(m, k)
            if isinstance(v, (bool, np.bool_)) and v and k not in first:
                first[k] = tn / 1e9 - t0
    if not first:
        return ["- failsafe_flags: no flag ever set"]
    return ["- failsafe_flags set (first time): " + ", ".join(f"{k} @ {t:.1f} s" for k, t in sorted(first.items(), key=lambda x: x[1]))]


def rc_activity(bag: Bag, t0: float) -> list[str]:
    topic = bag.first_topic("/fmu/out/manual_control_setpoint")
    if topic is None:
        return ["- no decodable /fmu/out/manual_control_setpoint"]
    t, ax, moving, valid, src = [], [], [], [], set()
    for _, tn, m in bag.messages(topic):
        t.append(tn / 1e9)
        ax.append([float(getattr(m, k, np.nan)) for k in ("roll", "pitch", "yaw", "throttle")])
        moving.append(bool(getattr(m, "sticks_moving", False)))
        valid.append(bool(getattr(m, "valid", True)))
        src.add(int(getattr(m, "data_source", -1)))
    t, ax = np.array(t), np.array(ax)
    L = [f"- {len(t)} msgs, valid {100 * np.mean(valid):.0f} %, data_source {sorted(src)}"]
    for i, k in enumerate(("roll", "pitch", "yaw", "throttle")):
        a = ax[:, i]
        if not np.isfinite(a).any():
            continue
        act = np.abs(a) > (0.05 if k != "throttle" else 2.0)    # throttle: report range only
        L.append(f"- {k}: min {np.nanmin(a):+.2f}, max {np.nanmax(a):+.2f}, mean |x| {np.nanmean(np.abs(a)):.2f}"
                 + (f", deflected > 0.05 for {100 * act.mean():.0f} % of samples" if k != "throttle" else ""))
    mv = np.array(moving)
    if mv.any():
        L.append(f"- sticks_moving {100 * mv.mean():.0f} % of samples, first @ {t[mv][0] - t0:.1f} s, last @ {t[mv][-1] - t0:.1f} s")
    return L


def battery(bag: Bag, t0: float) -> list[str]:
    topic = bag.first_topic("/fmu/out/battery_status", "/fmu/out/battery_status_v1")
    if topic is None:
        return ["- no decodable battery_status"]
    v, c, r, mah, t = [], [], [], [], []
    for _, tn, m in bag.messages(topic):
        t.append(tn / 1e9)
        v.append(float(m.voltage_v)), c.append(float(m.current_a)), r.append(float(m.remaining))
        mah.append(float(getattr(m, "discharged_mah", np.nan)))
    v, c, r, mah = map(np.array, (v, c, r, mah))
    return [f"- voltage {v[0]:.2f} V -> {v[-1]:.2f} V (min {v.min():.2f} V @ {t[int(v.argmin())] - t0:.1f} s)",
            f"- current mean {np.mean(c):.1f} A, max {c.max():.1f} A; remaining {100 * r[0]:.0f} % -> {100 * r[-1]:.0f} %"
            + (f"; discharged {mah[-1] - mah[0]:.0f} mAh" if np.isfinite(mah).all() else "")]


def vio_health(bag: Bag, t0: float) -> list[str]:
    o = load_odometry(bag)
    if o is None:
        return ["- **no odometry**: /visual_slam/tracking/odometry is not in this bag"]
    t, p = o["tb"], o["pos"]
    L = []
    if len(t) > 1:
        dt = np.diff(t)
        step = np.linalg.norm(np.diff(p, axis=0), axis=1)
        gaps = np.nonzero(dt > 0.1)[0]
        jumps = np.nonzero(step > 0.3)[0]
        L.append(f"- {len(t)} poses, {(len(t) - 1) / (t[-1] - t[0]):.1f} Hz, median dt {1000 * np.median(dt):.0f} ms, "
                 f"max gap {1000 * dt.max():.0f} ms @ {t[int(dt.argmax())] - t0:.1f} s")
        L.append(f"- gaps > 100 ms: {len(gaps)}" + (" (" + ", ".join(f"{1000 * dt[i]:.0f} ms @ {t[i] - t0:.1f} s" for i in gaps[:10]) + ")" if len(gaps) else ""))
        L.append(f"- position jumps > 0.3 m between poses: {len(jumps)}"
                 + (" (" + ", ".join(f"{step[i]:.2f} m @ {t[i] - t0:.1f} s" for i in jumps[:10]) + ")" if len(jumps) else ""))
        sp = step / np.maximum(dt, 1e-3)
        L.append(f"- speed median {np.median(sp):.2f} m/s, 99th pct {np.percentile(sp, 99):.2f} m/s; "
                 f"path {step.sum():.1f} m; z range {p[:, 2].min():.2f}..{p[:, 2].max():.2f} m")
    cov = o["cov"]
    if np.any(cov):
        sd = np.sqrt(np.maximum(cov, 0))
        L.append(f"- pose std (sqrt cov diag) x/y/z median {np.median(sd[:, 0]):.3g}/{np.median(sd[:, 1]):.3g}/"
                 f"{np.median(sd[:, 2]):.3g} m, max {sd[:, :3].max():.3g} m; yaw median {np.degrees(np.median(sd[:, 5])):.2g} deg")
        bad = np.nonzero(sd[:, :3].max(1) > 0.1)[0]
        if len(bad):
            L.append(f"- position std > 0.1 m in {len(bad)} poses (first @ {t[bad[0]] - t0:.1f} s)")
    else:
        L.append("- odometry covariance is all zero")
    for topic in ("/visual_slam/tracking/vo_pose_covariance",):
        if bag.has(topic):
            sds = []
            for _, _, m in bag.messages(topic, stride=5):
                c = np.asarray(m.pose.covariance, float).reshape(6, 6)
                sds.append(np.sqrt(np.maximum(np.diag(c)[:3], 0)))
            if sds:
                sds = np.array(sds)
                L.append(f"- `{topic}` position std median {np.median(sds):.3g} m, max {sds.max():.3g} m")
    return L


def tegrastats(path: Path | None) -> list[str]:
    if path is None:
        return ["- no tegrastats.log next to the bag"]
    cpu, ram, gpu, temp, pin = [], [], [], {}, []
    for line in path.read_text(errors="replace").splitlines():
        m = re.search(r"CPU \[([^\]]*)\]", line)
        if m:
            vals = [int(x) for x in re.findall(r"(\d+)%@", m.group(1))]
            if vals:
                cpu.append((np.mean(vals), max(vals)))
        m = re.search(r"RAM (\d+)/(\d+)MB", line)
        if m:
            ram.append((int(m.group(1)), int(m.group(2))))
        m = re.search(r"GR3D_FREQ (\d+)%", line)
        if m:
            gpu.append(int(m.group(1)))
        for k, v in re.findall(r"\b(\w+)@(-?[\d.]+)C\b", line):
            temp.setdefault(k, []).append(float(v))
        m = re.search(r"VDD_IN (\d+)mW", line)
        if m:
            pin.append(int(m.group(1)))
    if not cpu:
        return [f"- `{path.name}`: no CPU lines parsed"]
    c = np.array(cpu)
    L = [f"- `{path}`: {len(c)} samples",
         f"- CPU mean of cores {c[:, 0].mean():.0f} % (max {c[:, 0].max():.0f} %), busiest core max {c[:, 1].max():.0f} %"]
    if ram:
        r = np.array(ram)
        L.append(f"- RAM {r[:, 0].min()}..{r[:, 0].max()} / {r[0, 1]} MB")
    if gpu:
        L.append(f"- GPU (GR3D) mean {np.mean(gpu):.0f} %, max {max(gpu)} %")
    if temp:
        L.append("- temps max: " + ", ".join(f"{k} {max(v):.1f} C" for k, v in temp.items()))
    if pin:
        L.append(f"- VDD_IN mean {np.mean(pin) / 1000:.1f} W, max {max(pin) / 1000:.1f} W")
    return L


def run(bag_path, out=None, px4_msgs=None, depth_stride=10) -> str:
    out = default_out_dir(bag_path, out)
    with Bag(bag_path, px4_msgs, quiet=True) as bag:
        starts = [t for n in bag.topics for t in bag.times(n)[:1]]
        ends = [t for n in bag.topics for t in bag.times(n)[-1:]]
        t0, t1 = (min(starts), max(ends)) if starts else (0.0, 0.0)
        rows, _ = topic_table(bag, t0)
        L = [f"# RC bag summary: {Path(bag_path).name}", "",
             f"- path: `{bag_path}`",
             f"- storage: {', '.join(f.name for f in bag.files)}",
             f"- duration {t1 - t0:.1f} s, {sum(r[2] for r in rows)} messages, {len(rows)} topics"]
        L += [f"- note: {n}" for n in bag.notes]
        L += ["", "## Topics", "", "| topic | type | count | rate Hz | max gap s | gaps | first @ s |",
              "|---|---|---:|---:|---:|---:|---:|"]
        for name, mt, n, rate, mg, gaps, ok, first in rows:
            L.append(f"| `{name}` | {mt}{'' if ok else ' (undecodable)'} | {n} | {rate:.1f} | {mg:.3f} | {gaps} | {first:.1f} |")
        L += ["", f"gaps = intervals > max(5 x median period, {GAP_ABS_S} s).", ""]

        L += ["## Flight timeline", ""]
        tl = skybeta_timeline(bag, t0) + vehicle_status_timeline(bag, t0) + failsafe_flags(bag, t0)
        L += tl or ["- no /skybeta/state or decodable vehicle_status in this bag"]
        L += ["", "## RC sticks", ""] + rc_activity(bag, t0)
        L += ["", "## Battery", ""] + battery(bag, t0)
        L += ["", "## VIO health", ""] + vio_health(bag, t0)
        L += ["", "## Depth", ""] + depth_stats_md(depth_stats(bag, stride=depth_stride), t0)
        L += ["", "## CPU / GPU (tegrastats)", ""] + tegrastats(find_side_file(bag_path, "tegrastats.log"))
        for side in ("notes.txt", "check.txt"):
            f = find_side_file(bag_path, side)
            if f:
                L += ["", f"## {side}", "", "```", *f.read_text(errors="replace").splitlines()[:60], "```"]
    txt = "\n".join(L) + "\n"
    (out / "summary.md").write_text(txt, encoding="utf-8")
    print(txt)
    print(f"wrote {out / 'summary.md'}")
    return txt


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")   # Windows consoles: never die on a stray character
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--out")
    ap.add_argument("--px4-msgs")
    ap.add_argument("--depth-stride", type=int, default=10, help="decode every Nth depth frame for stats")
    a = ap.parse_args(argv)
    run(a.bag, a.out, a.px4_msgs, a.depth_stride)
    return 0


if __name__ == "__main__":
    sys.exit(main())

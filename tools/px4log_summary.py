#!/usr/bin/env python3
"""Summarise PX4 ULogs of the Sky Beta drone: arming, nav mode, in-air, EKF2 vision fusion, PX4 messages.

    python -I tools/px4log_summary.py drone_logs/qgc_2026-11-08/*.ulg

Needs pyulog (pip install pyulog). Read-only. See drone_logs/PX4_LOGS.md for how to read the output.
"""
import sys

import numpy as np
from pyulog import ULog

NAV = {0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "MISSION", 4: "LOITER", 5: "RTL", 14: "OFFBOARD",
       17: "TAKEOFF", 18: "LAND", 20: "PRECLAND"}


def ds(u, name):
    try:
        return u.get_dataset(name)
    except Exception:
        return None


def changes(t, v, fmt=lambda x: str(int(x))):
    return " ".join(f"{t[i]:.1f}s={fmt(v[i])}" for i in range(len(v)) if i == 0 or v[i] != v[i - 1])


def main(files):
    for f in files:
        u = ULog(f)
        t0 = u.start_timestamp
        print(f"\n=== {f}   {(u.last_timestamp - t0) / 1e6:.1f} s")
        for m in u.logged_messages:
            print(f"  msg {(m.timestamp - t0) / 1e6:6.1f}s: {m.message.strip()}")
        vs = ds(u, "vehicle_status") or ds(u, "vehicle_status_v1")
        if vs is not None:
            T = (vs.data["timestamp"] - t0) / 1e6
            print("  arming  :", changes(T, vs.data["arming_state"], lambda a: "ARMED" if a == 2 else "disarmed"))
            print("  nav     :", changes(T, vs.data["nav_state"], lambda n: NAV.get(int(n), str(int(n)))))
        ld = ds(u, "vehicle_land_detected")
        if ld is not None:
            print("  landed  :", changes((ld.data["timestamp"] - t0) / 1e6, ld.data["landed"]))
        lp = ds(u, "vehicle_local_position")
        if lp is not None:
            z = -lp.data["z"]
            print(f"  xy_valid:", changes((lp.data['timestamp'] - t0) / 1e6, lp.data["xy_valid"]),
                  f"  | z rel min {z.min() - z[0]:+.2f} max {z.max() - z[0]:+.2f} m")
        ef = ds(u, "estimator_status_flags")
        if ef is not None:
            T = (ef.data["timestamp"] - t0) / 1e6
            for k in ("cs_ev_pos", "cs_ev_yaw", "cs_yaw_align", "cs_in_air"):
                print(f"  {k:12s}:", changes(T, ef.data[k]))
        a = ds(u, "estimator_aid_src_ev_pos")
        if a is None:
            print("  EV      : no estimator_aid_src_ev_pos -> EKF2 received NO vision in this log")
        else:
            age = (a.data["timestamp"] - a.data["timestamp_sample"]) / 1e3
            T = (a.data["timestamp"] - t0) / 1e6
            print(f"  EV      : samples {T[0]:.1f}-{T[-1]:.1f}s  fused {100 * a.data['fused'].mean():.0f}%  "
                  f"rejected {100 * a.data['innovation_rejected'].mean():.0f}%  "
                  f"age p50 {np.percentile(age, 50):.0f} ms (includes EKF2_EV_DELAY)")
        P = u.initial_parameters
        print("  params  :", " ".join(f"{k}={P[k]:g}" for k in ("EKF2_EV_DELAY", "EKF2_DELAY_MAX", "EKF2_EV_CTRL",
                                                               "EKF2_HGT_REF", "EKF2_MAG_TYPE") if k in P))


if __name__ == "__main__":
    main(sys.argv[1:] or sys.exit(__doc__))

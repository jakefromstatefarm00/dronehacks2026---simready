"""First-corner analysis of GB10 Gazebo runs: truth kinematics, VIO error, features, geometry.

python corner_analysis.py runs/gb10-sky-beta-course-vNNN ...  [--json out.jsonl]
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "pathfinding"))
from drl.course import build_course  # noqa: E402

C = build_course(str(ROOT / "pathfinding/course/dronehacks-course-v3.obj"))
HALF = C.half_size


def yaw_q(qw, qx, qy, qz):
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))


def smooth(x, n):
    k = np.ones(n) / n
    return np.convolve(np.pad(x, (n // 2, n - 1 - n // 2), mode="edge"), k, mode="valid")


def ray(p, yaw, maxd=6.0, step=0.03):
    """distance and what is hit along the heading from p (z kept)"""
    d = np.arange(0.05, maxd, step)
    pts = np.c_[p[0] + d * np.cos(yaw), p[1] + d * np.sin(yaw), np.full_like(d, p[2])]
    hit = C.collides(pts, 0.0, 0.0)
    oob = (np.abs(pts[:, 0]) > HALF) | (np.abs(pts[:, 1]) > HALF)
    idx = np.where(hit | oob)[0]
    if not len(idx):
        return maxd, "none"
    i = idx[0]
    if oob[i] and not hit[i]:
        return float(d[i]), "net"
    q = pts[i:i + 1]
    # which box
    for c, u, h, n in zip(C.box_c, C.box_u, C.box_h, C.box_names):
        dd = q[0] - c
        lu = dd[0] * u[0] + dd[1] * u[1]
        ln = -dd[0] * u[1] + dd[1] * u[0]
        if abs(lu) <= h[0] + .01 and abs(ln) <= h[1] + .01 and abs(dd[2]) <= h[2] + .01:
            return float(d[i]), n.split()[0]
    return float(d[i]), "obj"


def free_radius(P, hi=1.5, iters=16):
    lo, h = np.zeros(len(P)), np.full(len(P), hi)
    for _ in range(iters):
        mid = (lo + h) / 2
        hit = C.collides(P, mid, 0.08) | (np.abs(P[:, 0]) > HALF - mid) | (np.abs(P[:, 1]) > HALF - mid)
        h = np.where(hit, mid, h)
        lo = np.where(hit, lo, mid)
    return lo


def load(run):
    run = Path(run)
    T = np.genfromtxt(run / "truth-poses.csv", delimiter=",", names=True)
    t = T["t"]
    P = np.c_[T["x"], T["y"], T["z"]]
    yaw = np.unwrap(yaw_q(T["qw"], T["qx"], T["qy"], T["qz"]))
    V = [json.loads(l) for l in open(run / "vio-odometry.jsonl")]
    vt = np.array([v["source_s"] for v in V])
    vp = np.array([[v["position"][k] for k in "xyz"] for v in V])
    vq = np.array([[v["orientation"][k] for k in "wxyz"] for v in V])
    vyaw = np.unwrap(yaw_q(*vq.T))
    F = {}
    for l in open(run / "vio-feature-clouds.jsonl"):
        e = json.loads(l)
        F.setdefault(e["topic"].rsplit("_", 1)[-1], []).append((e["source_s"], e["finite_xyz_count"]))
    F = {k: np.array(v) for k, v in F.items()}
    plans = []
    jp = run / "journal" / "journal.jsonl"
    states = {}
    if jp.exists():
        for l in open(jp):
            try:
                e = json.loads(l)
            except ValueError:
                continue
            if e.get("kind") == "plan" and e.get("target"):
                plans.append((e["t"], e["target"][0][0], e["target"][0][1], e["target"][1], e.get("mode")))
            if e.get("kind") == "state":
                states.setdefault(e["state"], e["t"])
    return dict(t=t, P=P, yaw=yaw, vt=vt, vp=vp, vyaw=vyaw, F=F, plans=np.array([p[:4] for p in plans]) if plans else None,
                pmodes=[p[4] for p in plans], states=states)


def analyse(run):
    d = load(run)
    t, P, yaw = d["t"], d["P"], d["yaw"]
    h = .2   # central difference over 0.4 s (truth stamps jitter)
    di = lambda f: (np.interp(t + h, t, f) - np.interp(t - h, t, f)) / (2 * h)
    speed = np.hypot(di(P[:, 0]), di(P[:, 1]))
    yr = di(yaw)
    z0 = np.median(P[:50, 2])
    air = P[:, 2] > z0 + .10
    i0 = int(np.argmax(air))
    t_lift = t[i0]
    # VIO -> course alignment from the 1 s before lift-off (stationary): yaw offset + translation
    m = (d["vt"] > t_lift - 1.5) & (d["vt"] < t_lift - .2)
    if m.sum() < 2:
        m = d["vt"] < t_lift
        m[np.where(m)[0][:-5]] = False
    th = np.mean(np.interp(d["vt"][m], t, yaw) - d["vyaw"][m])
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    vxy = d["vp"][:, :2] @ R.T
    off = np.mean(np.c_[np.interp(d["vt"][m], t, P[:, 0]), np.interp(d["vt"][m], t, P[:, 1])] - vxy[m], axis=0)
    vxy = vxy + off
    txy = np.c_[np.interp(d["vt"], t, P[:, 0]), np.interp(d["vt"], t, P[:, 1])]
    err = np.linalg.norm(vxy - txy, axis=1)
    errv = vxy - txy
    # corner: first time the truth heading falls below 45 deg after lift-off (start heading 90)
    hd = np.degrees(yaw)
    after = np.where((t > t_lift) & (hd < 45))[0]
    row = {"run": Path(run).name.rsplit("-", 1)[-1], "t_lift": round(float(t_lift), 2)}
    if not len(after):
        row["corner"] = None
        return row, d
    ic = after[0]
    tc = t[ic]
    row["t_corner45"] = round(float(tc - t_lift), 2)
    row["corner_xy"] = P[ic, :2].round(2).tolist()
    w = (t > tc - 4) & (t < tc + 1)
    w3 = (t > tc - 3) & (t < tc)
    row["speed_mean_pre3"] = round(float(speed[w3].mean()), 3)
    row["speed_max_pre3"] = round(float(speed[w3].max()), 3)
    row["yawrate_max_pre3"] = round(float(np.abs(yr[w3]).max()), 3)
    row["yawrate_mean_pre3"] = round(float(np.abs(yr[w3]).mean()), 3)
    row["speed_x_yawrate_max"] = round(float((speed * np.abs(yr))[w].max()), 3)
    # heading at turn start (yaw rate > 0.1 first, within 6 s before tc)
    ws = np.where((t > tc - 8) & (t < tc) & (np.abs(yr) > .08))[0]
    if len(ws):
        j = ws[0]
        row["turn_start_dt"] = round(float(tc - t[j]), 2)
        row["turn_start_xy"] = P[j, :2].round(2).tolist()
        row["turn_start_speed"] = round(float(speed[j]), 3)
    fr = free_radius(P[w])
    row["min_clear_corner"] = round(float(fr.min()), 3)
    row["min_clear_corner_xy"] = P[w][np.argmin(fr), :2].round(2).tolist()
    # what the forward camera looks at, sampled every 0.5 s from tc-4 to tc+1
    looks = []
    for tt in np.arange(tc - 4, tc + 1.01, .5):
        k = int(np.searchsorted(t, tt))
        k = min(k, len(t) - 1)
        dist, what = ray(P[k], yaw[k])
        looks.append((round(float(tt - tc), 1), round(float(hd[k]), 0), round(dist, 2), what,
                      round(float(speed[k]), 2), round(float(yr[k]), 2)))
    row["looks"] = looks
    # VIO
    wv = (d["vt"] > tc - 4) & (d["vt"] < tc + 1)
    row["vio_err_at_lift+5"] = round(float(np.interp(t_lift + 5, d["vt"], err)), 3)
    row["vio_err_corner-4"] = round(float(np.interp(tc - 4, d["vt"], err)), 3)
    row["vio_err_corner"] = round(float(np.interp(tc, d["vt"], err)), 3)
    row["vio_err_corner+3"] = round(float(np.interp(tc + 3, d["vt"], err)), 3)
    row["vio_err_corner+6"] = round(float(np.interp(tc + 6, d["vt"], err)), 3)
    row["vio_err_max_flight"] = round(float(err[(d["vt"] > t_lift) & (d["vt"] < t[air].max() if air.any() else 1e9)].max()), 3)
    # growth of error vector in [tc-2, tc+4]
    for a, b, key in ((-3, 0, "vio_drift_rate_pre"), (0, 3, "vio_drift_rate_post")):
        ea = np.array([np.interp(tc + a, d["vt"], errv[:, i]) for i in (0, 1)])
        eb = np.array([np.interp(tc + b, d["vt"], errv[:, i]) for i in (0, 1)])
        row[key] = round(float(np.linalg.norm(eb - ea) / (b - a)), 3)
    # VIO yaw error
    yerr = np.degrees(d["vyaw"] + th - np.interp(d["vt"], t, yaw))
    row["vio_yaw_err_corner+3_deg"] = round(float(np.interp(tc + 3, d["vt"], yerr)), 2)
    for k, (ft, fc) in ((k, v.T) for k, v in d["F"].items()):
        mm = (ft > tc - 3) & (ft < tc + 1)
        ref = (ft > t_lift + 1) & (ft < tc - 5)
        if mm.any():
            row[f"feat_{k}_corner_mean"] = round(float(fc[mm].mean()), 1)
            row[f"feat_{k}_corner_min"] = int(fc[mm].min())
        if ref.any():
            row[f"feat_{k}_corridor_mean"] = round(float(fc[ref].mean()), 1)
    # commanded heading rate from plan targets
    if d["plans"] is not None and len(d["plans"]):
        pl = d["plans"]
        mp = (pl[:, 0] > tc - 4) & (pl[:, 0] < tc + 1)
        if mp.sum() > 2:
            cy = np.unwrap(pl[mp, 3])
            row["cmd_yaw_range_deg"] = [round(float(np.degrees(cy.min())), 0), round(float(np.degrees(cy.max())), 0)]
    return row, d


if __name__ == "__main__":
    args = sys.argv[1:]
    out = None
    if "--json" in args:
        i = args.index("--json")
        out = args[i + 1]
        args = args[:i] + args[i + 2:]
    rows = []
    for r in args:
        try:
            row, _ = analyse(r)
        except Exception as ex:  # noqa: BLE001
            row = {"run": r, "error": repr(ex)}
        rows.append(row)
        print(json.dumps(row))
    if out:
        with open(out, "w") as f:
            for r in rows:
                f.write(json.dumps(r) + "\n")

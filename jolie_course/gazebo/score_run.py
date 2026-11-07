"""Score one GB10 Gazebo flight of a FlyX-style native program on ground truth.

    python jolie_course/gazebo/score_run.py <run_dir> [<run_dir> ...] [--table] [--out rows.jsonl]

<run_dir> is a local copy of artifacts/gb10-sky-beta-course-vNNN with at least
truth-poses.csv (sim/runtime/post_flight.sh), native-program.log and journal/journal.jsonl
(the program's --output); <run_dir>.launch.log next to it is used for the exit code.
Completed = program DONE with 5/5 checkpoints AND truth pad error < 0.40 m AND truth tilt < 60 deg
(the program exit code alone is not enough: v209 crashed after VIO divergence and still exited 0).
One JSON row per run is printed (and appended to --out); --table prints a summary.

Clearance: free radius per airborne truth point against the published course OBJ
(pathfinding/drl/course.py, r_z 0.08), by bisection. 0.28 m is the Sky Beta footprint proxy.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "pathfinding"))
from drl.course import build_course  # noqa: E402

PAD = np.array([-2.0422, -0.7925])
FOOTPRINT = 0.28
R_Z = 0.08
_COURSE = None


def course():
    global _COURSE
    if _COURSE is None:
        _COURSE = build_course(str(ROOT / "pathfinding/course/dronehacks-course-v3.obj"))
    return _COURSE


def free_radius(P, hi=1.5, iters=18):
    C = course()
    out = np.empty(len(P))
    for s in range(0, len(P), 2000):
        p = P[s:s + 2000]
        lo, h = np.zeros(len(p)), np.full(len(p), hi)
        inside = C.collides(p, 0.0, R_Z)
        for _ in range(iters):
            mid = (lo + h) / 2
            hit = C.collides(p, mid, R_Z)
            h = np.where(hit, mid, h)
            lo = np.where(hit, lo, mid)
        out[s:s + 2000] = np.where(inside, 0.0, lo)
    return out


def parse_log(path):
    """State transitions with wall stamps, RESULT and checkpoints lines."""
    r = {"wall_states": [], "result": None, "checkpoints_line": None, "finished": None}
    if not path.exists():
        return r
    for line in path.read_text(errors="replace").splitlines():
        m = re.search(r"\[(\d+\.\d+)\].*state (\S+) -> (\S+)\s*(.*)", line)
        if m:
            r["wall_states"].append((float(m.group(1)), m.group(2), m.group(3), m.group(4).strip()))
        if line.startswith("RESULT:"):
            r["result"] = line.strip()
        if line.startswith("checkpoints cleared:"):
            r["checkpoints_line"] = line.strip()
        if "finished in state" in line:
            r["finished"] = line.strip()
    return r


def parse_journal(path):
    states, end = [], None
    if path.exists():
        for line in path.open(errors="replace"):
            try:
                e = json.loads(line)
            except ValueError:
                continue
            if e.get("kind") == "state":
                states.append((e.get("t"), e.get("state"), e.get("note")))
            elif e.get("kind") == "end":
                end = e
    return states, end


def first(seq, state):
    for item in seq:
        if item[1] == state:
            return item[0]
    return None


def score(run_dir: Path) -> dict:
    run_dir = Path(run_dir)
    row = {"run": run_dir.name.rsplit("-", 1)[-1]}
    launch = run_dir.with_name(run_dir.name + ".launch.log")
    if launch.exists():
        m = re.search(r'"native_program_exit_code":\s*(-?\d+|null)', launch.read_text(errors="replace"))
        row["exit_code"] = None if not m or m.group(1) == "null" else int(m.group(1))
        m = re.search(r'"native_program":\s*"([^"]*)"', launch.read_text(errors="replace"))
        if m:
            row["program"] = m.group(1).split()[0].split("dronehacks-2026-gb10/")[-1]
            row["args"] = " ".join(a for a in m.group(1).split()[1:] if "journal" not in a and a != "--output")
    log = parse_log(run_dir / "native-program.log")
    states, end = parse_journal(run_dir / "journal" / "journal.jsonl")
    row["result"] = log["result"]
    row["final_state"] = end["state"] if end else (states[-1][1] if states else None)
    row["cleared"] = len(end["cleared"]) if end else None
    if end:
        row["cleared_names"] = [c["name"] if isinstance(c, dict) else c for c in end["cleared"]]
        row["depth_frames"] = (end.get("depth") or {}).get("frames")
    row["last_note"] = states[-1][2] if states else None
    # Phase times: journal t is the program's /clock (sim seconds); log stamps are wall.
    for src, seq, sfx in (("sim", states, "sim"), ("wall", [(w, b) for w, a, b, _ in log["wall_states"]], "wall")):
        arm, nav, land, done = (first(seq, s) for s in ("TAKEOFF", "NAVIGATE", "LAND", "DONE"))
        row[f"navigate_s_{sfx}"] = round(land - nav, 2) if nav and land else None
        row[f"arm_to_disarm_s_{sfx}"] = round(done - arm, 2) if arm and done else None
    m = re.search(r"\(([\d.]+) m from its centre\)", " ".join(str(s[2]) for s in states))
    row["program_pad_m"] = float(m.group(1)) if m else None

    tp = run_dir / "truth-poses.csv"
    if tp.exists():
        T = np.genfromtxt(tp, delimiter=",", names=True)
        t, P = T["t"], np.c_[T["x"], T["y"], T["z"]]
        z0 = np.median(P[:50, 2])
        air = P[:, 2] > z0 + 0.10
        if air.any():
            i0, i1 = np.argmax(air), len(air) - 1 - np.argmax(air[::-1])
            row["airborne_s_truth"] = round(float(t[i1] - t[i0]), 2)
            row["airborne_window"] = [round(float(t[i0]), 3), round(float(t[i1]), 3)]
            seg = P[i0:i1 + 1]
            seg = seg[seg[:, 2] > z0 + 0.10]
            fr = free_radius(seg)
            k = int(np.argmin(fr))
            row["min_clear_m"] = round(float(fr[k]), 3)
            row["min_clear_at"] = [round(float(v), 2) for v in seg[k]]
            row["n_pts_under_028"] = int((fr < FOOTPRINT).sum())
            row["frac_under_028"] = round(float((fr < FOOTPRINT).mean()), 4)
            row["n_pts_airborne"] = int(len(seg))
            row["max_z"] = round(float(P[:, 2].max()), 2)
            qw, qx, qy, qz = (T[k][i0:] for k in ("qw", "qx", "qy", "qz"))
            tilt = np.degrees(np.arccos(np.clip(1 - 2 * (qx ** 2 + qy ** 2), -1, 1)))
            row["max_tilt_deg"] = round(float(tilt.max()), 1)
        row["pad_err_m"] = round(float(np.linalg.norm(P[-1, :2] - PAD)), 3)
        row["final_xyz"] = [round(float(v), 3) for v in P[-1]]
    row["completed"] = bool(row.get("final_state") == "DONE" and row.get("cleared") == 5
                            and row.get("pad_err_m", 9) < 0.40
                            and row.get("max_tilt_deg", 0) < 60)
    return row


def table(rows):
    hdr = ["run", "prog", "ok", "state", "cp", "nav_sim", "a2d_sim", "a2d_wall", "air_tr", "minclr", "n<.28", "pad", "tilt"]
    print(" | ".join(hdr))
    for r in rows:
        print(" | ".join(str(x) for x in [r["run"], Path(r.get("program", "?")).name, int(r["completed"]),
              r.get("final_state"), r.get("cleared"), r.get("navigate_s_sim"), r.get("arm_to_disarm_s_sim"),
              r.get("arm_to_disarm_s_wall"), r.get("airborne_s_truth"), r.get("min_clear_m"),
              r.get("n_pts_under_028"), r.get("pad_err_m"), r.get("max_tilt_deg")]))
    by = {}
    for r in rows:
        by.setdefault((r.get("program", "?"), r.get("args", "")), []).append(r)
    for (p, a), rs in by.items():
        ok = [r for r in rs if r["completed"]]
        nav = np.array([r["navigate_s_sim"] for r in ok if r.get("navigate_s_sim")])
        a2d = np.array([r["arm_to_disarm_s_sim"] for r in ok if r.get("arm_to_disarm_s_sim")])
        clr = [r["min_clear_m"] for r in rs if r.get("min_clear_m") is not None]
        s = f"{Path(p).name} {a}: completed {len(ok)}/{len(rs)}"
        if len(nav):
            s += f"; NAVIGATE mean {nav.mean():.1f} p90 {np.percentile(nav, 90):.1f} s"
        if len(a2d):
            s += f"; arm-disarm mean {a2d.mean():.1f} p90 {np.percentile(a2d, 90):.1f} s"
        if clr:
            s += f"; worst min clearance {min(clr):.3f} m (all runs)"
        okc = [r["min_clear_m"] for r in ok if r.get("min_clear_m") is not None]
        if okc:
            s += f", {min(okc):.3f} m (completed runs)"
        print(s)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--table", action="store_true")
    ap.add_argument("--out", type=Path)
    a = ap.parse_args()
    rows = [score(d) for d in a.runs]
    for r in rows:
        print(json.dumps(r))
    if a.out:
        with a.out.open("a") as f:
            for r in rows:
                f.write(json.dumps(r) + "\n")
    if a.table:
        table(rows)

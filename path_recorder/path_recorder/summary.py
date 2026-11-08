"""Check a saved recording (no ROS needed: runs on the Jetson host, in the stack, or on a laptop).

    python3 -m path_recorder.summary                       # newest recording
    python3 -m path_recorder.summary path-20261108-...     # a specific folder (name or path)
    python3 -m path_recorder.summary --list                # every recording, newest last
    ros2 run path_recorder summary                         # same, from a sourced shell

Prints the metadata, sanity checks (spacing, gaps, jumps, depth coverage, closed file) and a
top-down sketch of the path in the arm frame (S = start, E = end, forward is up the page).
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from path_recorder.recording import load, runs_dir  # noqa: E402


def sketch(rows, width=60, height=24):
    pts = [(r["x"], r["y"]) for r in rows if r["x"] is not None]
    if len(pts) < 2:
        return ["(not enough points to draw)"]
    # forward (x) up the page, left (y) to the left of the page
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    span = max(max(xs) - min(xs), max(ys) - min(ys), .5)
    sx, sy = (width - 1) / span, (height - 1) / span / 2.0        # terminal cells are ~2x tall
    grid = [[" "] * width for _ in range(height)]
    for x, y in pts:
        r, c = int((max(xs) - x) * sy * 2), int((max(ys) - y) * sx)
        if 0 <= r < height and 0 <= c < width:
            grid[r][c] = "."
    for (x, y), ch in ((pts[0], "S"), (pts[-1], "E")):
        r, c = int((max(xs) - x) * sy * 2), int((max(ys) - y) * sx)
        if 0 <= r < height and 0 <= c < width:
            grid[r][c] = ch
    rows_out = ["".join(g).rstrip() for g in grid]
    while rows_out and not rows_out[-1]:
        rows_out.pop()
    return [f"top-down, forward = up, left = left, {span:.1f} m across"] + ["|" + r for r in rows_out]


def check(meta, rows):
    out = []
    n = len(rows)
    out.append(("OK" if meta.get("status") == "complete" else "WARN",
                f"file {meta.get('status')}" + (f", ended by {meta.get('ended')}" if meta.get("ended") else
                                                " (still recording, or the recorder was killed: data up to the "
                                                "last point is fine)")))
    out.append(("OK" if n >= 10 else "WARN", f"{n} points over {meta.get('length_m')} m, {meta.get('duration_s')} s "
                                             f"(airborne {meta.get('airborne_s')} s, max height {meta.get('max_height_m')} m)"))
    if n >= 2:
        steps = [math.dist((a["x"], a["y"], a["z"]), (b["x"], b["y"], b["z"])) for a, b in zip(rows, rows[1:])]
        big = [s for s in steps if s > 3 * float(meta.get("spacing_m", .15))]
        out.append(("OK" if not big else "WARN", f"spacing: median {sorted(steps)[len(steps) // 2]:.3f} m, max "
                    f"{max(steps):.3f} m" + (f"; {len(big)} steps > 3x spacing (fast flight or a jump)" if big else "")))
        dts = [b["t"] - a["t"] for a, b in zip(rows, rows[1:]) if a["t"] is not None and b["t"] is not None]
        if dts:
            out.append(("OK", f"longest time between points {max(dts):.1f} s (hovering adds no points)"))
    fl = meta.get("flags", {})
    for k in ("odom_jump", "odom_gap", "px4_no_position"):
        if fl.get(k):
            out.append(("WARN", f"{fl[k]} points flagged {k}"))
    with_depth = sum(1 for r in rows if r["clear_center"] is not None)
    out.append(("OK" if n and with_depth / n > .8 else "WARN", f"depth clearance on {with_depth}/{n} points"))
    if meta.get("anchor", {}).get("started_armed"):
        out.append(("WARN", "recorder started while armed: the anchor is NOT the start spot"))
    if meta.get("aborted_or_short"):
        out.append(("WARN", "short or aborted flight (< 2 s airborne or < 0.5 m)"))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder", nargs="?", help="recording folder (name or path); default: newest")
    ap.add_argument("--dir", type=Path, default=None, help=f"recordings directory (default {runs_dir()})")
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args([x for x in (argv if argv is not None else sys.argv[1:]) if not x.startswith("--ros-args")])
    base = a.dir or runs_dir()
    folders = sorted(p for p in base.glob("path-*") if (p / "meta.json").exists())
    if a.list:
        for p in folders:
            meta, _ = load(p)
            print(f"{p.name}  {meta['points']:>5} pts  {meta['length_m']:>7.2f} m  {meta['duration_s']:>6.1f} s  "
                  f"{meta['status']}" + ("  short/aborted" if meta.get("aborted_or_short") else ""))
        return 0
    if a.folder:
        folder = Path(a.folder) if Path(a.folder).exists() else base / a.folder
    elif folders:
        folder = folders[-1]
    else:
        print(f"no recordings in {base}")
        return 1
    meta, rows = load(folder)
    print(f"recording: {folder}")
    print(f"created {meta['created']}   anchor (odom) {meta['anchor']['odom_position']} yaw {meta['anchor']['odom_yaw']}")
    for level, text in check(meta, rows):
        print(f"[{level:4s}] {text}")
    print()
    for line in sketch(rows):
        print(line)
    print(f"\nfiles: {folder / 'points.csv'}  meta.json  events.jsonl   (copy the whole folder to share it)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

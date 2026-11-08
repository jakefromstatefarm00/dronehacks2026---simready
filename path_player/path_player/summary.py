"""Command-line tool to inspect and validate path_recorder recordings before replay."""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

from path_player.replay import find_latest_recording, load_recording, runs_dir


def print_summary(folder: Path):
    try:
        meta, points = load_recording(folder)
    except Exception as e:
        print(f"Error loading {folder}: {e}")
        return

    print("=" * 65)
    print(f"Recording: {folder.name}")
    print(f"Location:  {folder.resolve()}")
    print("=" * 65)

    created = meta.get("created", "unknown")
    status = meta.get("status", "unknown")
    aborted = meta.get("aborted_or_short", False)
    points_count = len(points)
    duration_s = meta.get("duration_s", points[-1].t - points[0].t if points else 0.0)
    airborne_s = meta.get("airborne_s", 0.0)
    length_m = meta.get("length_m", 0.0)
    max_height_m = meta.get("max_height_m", max((p.z for p in points), default=0.0))

    print(f"Created:           {created}")
    print(f"Status:            {status} " + ("[WARNING: aborted / short flight]" if aborted else "[OK]"))
    print(f"Points:            {points_count}")
    print(f"Recorded Duration: {duration_s:.1f} s (Airborne: {airborne_s:.1f} s)")
    print(f"Total Trajectory:  {length_m:.2f} m")
    print(f"Max Altitude (z):  {max_height_m:.2f} m")

    anchor = meta.get("anchor", {})
    if anchor:
        pos = anchor.get("odom_position", [0, 0, 0])
        yaw_deg = math.degrees(anchor.get("odom_yaw", 0.0))
        print(f"Recorded Anchor:   pos=[{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}], yaw={yaw_deg:.1f} deg")

    speeds = [p.speed for p in points if p.speed > 0]
    if speeds:
        print(f"Speeds:            avg={sum(speeds)/len(speeds):.2f} m/s, max={max(speeds):.2f} m/s")

    flags = meta.get("flags", {})
    if flags:
        print("\nRecorded Flags:")
        for flag_name, count in flags.items():
            print(f"  - {flag_name}: {count}")

    print("\nReplay Pacing Estimates:")
    print(f"  - At 1.0x speed: {duration_s:.1f} s")
    print(f"  - At 0.8x speed: {duration_s / 0.8:.1f} s")
    print(f"  - At 0.5x speed: {duration_s / 0.5:.1f} s")

    p_start = points[0]
    p_end = points[-1]
    print("\nTrajectory Bounds (Arm Frame):")
    print(f"  Start: x={p_start.x:.2f}m, y={p_start.y:.2f}m, z={p_start.z:.2f}m, yaw={math.degrees(p_start.yaw):.1f}°")
    print(f"  End:   x={p_end.x:.2f}m, y={p_end.y:.2f}m, z={p_end.z:.2f}m, yaw={math.degrees(p_end.yaw):.1f}°")
    print("=" * 65)


def main():
    parser = argparse.ArgumentParser(description="Inspect a path_recorder recording folder.")
    parser.add_argument("folder", nargs="?", default="", help="Path to recording folder (default: latest in runs dir)")
    args = parser.parse_args()

    if args.folder:
        target = Path(args.folder)
    else:
        target = find_latest_recording()
        if not target:
            print(f"No recordings found in {runs_dir()}")
            sys.exit(1)
        print(f"Latest recording found: {target.name}")

    print_summary(target)


if __name__ == "__main__":
    main()

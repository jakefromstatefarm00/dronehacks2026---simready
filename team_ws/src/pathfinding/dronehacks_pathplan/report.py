"""Summarise a flight journal (no ROS needed): what happened, why it ended, where it went wrong.

    ros2 run dronehacks_pathplan report                       # newest run in <team_ws>/runs
    python3 -m dronehacks_pathplan.report path/to/journal.jsonl --plot run.png

Prints the state timeline (with the reason for every change), checkpoints, PX4 mode changes
and clamps, the odom-vs-PX4 gap, control-loop stalls, depth quality and the planner's events.
``--plot`` draws the planned/flown positions on the published course map (needs matplotlib).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


def find_runs_dir():
    here = Path(__file__).resolve()
    for p in here.parents:
        if p.name == 'team_ws':
            return p / 'runs'
    return Path.home() / 'dronehacks_runs'


def load(path):
    rows = []
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        try:
            rows.append(json.loads(line))
        except ValueError:
            pass
    return rows


def summarise(rows, out=print):
    t0 = next((r['t'] for r in rows if r.get('t') is not None), 0.0) or 0.0
    rel = lambda r: '   -   ' if r.get('t') is None else f"{r['t'] - t0:7.1f}"
    start = next((r for r in rows if r['kind'] == 'start'), {})
    p = start.get('params', {})
    out(f"run: planner {p.get('planner')}, altitude {p.get('flight_altitude_m')} m, depth "
        f"{p.get('depth_topic') or 'OFF'}, shadow {p.get('shadow')}, frame latched at {p.get('frame_latch')}")
    out('\nstate timeline (s from the first record):')
    for r in rows:
        if r['kind'] == 'state':
            out(f"  {rel(r)}  {r['from']:>11s} -> {r['state']:<11s} {r.get('note', '')}")
        elif r['kind'] == 'px4':
            out(f"  {rel(r)}      PX4: {r['mode']:<9s} armed={r['armed']} landed={r['landed']}"
                + (f" CLAMPED {r['clamped']}" if r.get('clamped') else '')
                + (f" odom-local gap {r['odom_local_gap_m']} m" if r.get('odom_local_gap_m') is not None else ''))
        elif r['kind'] in ('foreign', 'hold_request', 'planner_reset'):
            out(f"  {rel(r)}      {r['kind']}: {r.get('note', '')}")
    plans = [r for r in rows if r['kind'] == 'plan']
    cleared = plans[-1]['cleared'] if plans else []
    out(f'\ncheckpoints cleared: {len(cleared)}: {", ".join(cleared)}')
    if plans:
        modes = {}
        for r in plans:
            modes[r['mode']] = modes.get(r['mode'], 0) + 1
        slow = max(plans, key=lambda r: r.get('planner_ms', 0))
        out(f"planning steps: {len(plans)} ({', '.join(f'{k} {v}' for k, v in modes.items())}); "
            f"slowest planner step {slow.get('planner_ms')} ms")
    gaps = [r['gap_s'] for r in rows if r['kind'] == 'timing']
    if gaps:
        out(f'control-loop stalls > 0.15 s: {len(gaps)}, worst {max(gaps):.2f} s (PX4 drops Offboard at ~1 s)')
    gaps = [r['odom_local_gap_m'] for r in rows if r['kind'] == 'px4' and r.get('odom_local_gap_m') is not None]
    if gaps:
        out(f'odom vs PX4 local position gap: max {max(gaps):.3f} m (large = PX4 frame differs from the vision frame)')
    summ = [r for r in rows if r['kind'] == 'summary']
    if summ:
        s = summ[-1]
        q = s.get('depth_quality') or {}
        if q.get('hits'):
            out(f"depth: {s['depth'].get('given')} frames used, {s['depth'].get('turning')} dropped while turning, "
                f"{q['hits_on_published_walls'] / q['hits']:.0%} of hits on published walls, "
                f"{q.get('hits_new_confirmed')} new obstacle cells")
        ev = [e.get('event') for e in s.get('planner_events', []) if e.get('event')]
        if ev:
            out('planner events (last): ' + '; '.join(str(e) for e in ev[-8:]))
        out(f"\nRESULT: {s.get('result') or s.get('state')}")
    return plans


def plot(rows, path):
    try:
        import matplotlib
    except ImportError:
        print('--plot needs matplotlib (pip install matplotlib on your laptop; copy the journal there)')
        return
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if __package__ in (None, ''):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from pathplan.ros_node import build_planner
    planner, spec, _, prior = build_planner('static', .9)
    fig, ax = plt.subplots(figsize=(7, 7))
    h = spec.half_size
    ax.imshow(prior, origin='lower', extent=(-h, h, -h, h), cmap='Greys', alpha=.6)
    pts = [r['pose'] for r in rows if r['kind'] == 'plan' and r.get('pose')]
    if pts:
        ax.plot([p[0] for p in pts], [p[1] for p in pts], '-', lw=1.5, label='flown (VIO pose)')
    for cp in planner.checkpoints:
        ax.plot(*cp.centre, 'r+', ms=12)
        ax.annotate(cp.name.split(' ')[0], cp.centre, color='r')
    for r in rows:
        if r['kind'] == 'state' and r.get('pose') and r['state'] in ('FAULT', 'TAKEN_OVER', 'LOW_HOLD', 'DESCEND'):
            ax.plot(r['pose'][0], r['pose'][1], 'kx')
            ax.annotate(r['state'], r['pose'][:2], fontsize=8)
    ax.set_aspect('equal')
    ax.legend(loc='upper right')
    ax.set_title('course frame (m)')
    fig.savefig(path, dpi=130, bbox_inches='tight')
    print(f'plot: {path}')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('journal', nargs='?', type=Path, help='journal.jsonl (default: newest run)')
    ap.add_argument('--plot', type=Path, help='write a PNG of the flight on the course map')
    a = ap.parse_args([x for x in (argv if argv is not None else sys.argv[1:]) if not x.startswith('--ros-args')])
    path = a.journal
    if path is None:
        runs = sorted(find_runs_dir().glob('pathplan-*/journal.jsonl'))
        if not runs:
            print(f'no runs in {find_runs_dir()}')
            return 1
        path = runs[-1]
    print(f'journal: {path}')
    rows = load(path)
    summarise(rows)
    if a.plot:
        plot(rows, a.plot)
    return 0


if __name__ == '__main__':
    sys.exit(main())

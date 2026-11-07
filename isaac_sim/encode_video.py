"""Render a recorded organizer-sim flight (record_flight.py output) to MP4, offline.

    python3 encode_video.py <flight-dir> [--course-obj course.obj] [--title "..."]

Left: onboard left camera (what cuVSLAM saw). Right: top-down view of Pegasus ground
truth (blue), cuVSLAM estimate mapped into the truth frame at arming (orange) and the
commanded setpoints (grey), with the course walls if --course-obj is given. Header:
recorder time, time since arming, PX4 arming/nav state, mission status.
Truth is used for display only.
"""
import argparse
import json
import math
import subprocess
from pathlib import Path

import cv2
import numpy as np

NAV = {0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION', 4: 'AUTO_LOITER', 5: 'AUTO_RTL',
       14: 'OFFBOARD', 17: 'AUTO_TAKEOFF', 18: 'AUTO_LAND'}


def load_csv(path):
    if not path.exists() or path.stat().st_size < 50:
        return np.zeros((0, 9))
    return np.atleast_2d(np.loadtxt(path, delimiter=',', skiprows=1))


def course_segments(obj):
    v, segs = [], []
    for line in open(obj):
        t = line.split()
        if not t:
            continue
        if t[0] == 'v':
            v.append([float(x) for x in t[1:4]])
        elif t[0] == 'f':
            idx = [int(x.split('/')[0])-1 for x in t[1:]]
            pts = [v[i] for i in idx]
            if max(p[2] for p in pts) > .3:            # walls/nets only, not floor markings
                for a, b in zip(idx, idx[1:]+idx[:1]):
                    segs.append((v[a][:2], v[b][:2]))
    return segs


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('flight', type=Path)
    p.add_argument('--course-obj', type=Path)
    p.add_argument('--title', default='')
    p.add_argument('--fps', type=float, default=10.)
    p.add_argument('--extent', type=float, default=None, help='half-width of the map view, metres')
    a = p.parse_args()
    d = a.flight
    truth, est, sp = (load_csv(d/f'{k}.csv') for k in ('truth', 'estimate', 'setpoint'))
    events = [json.loads(l) for l in open(d/'events.jsonl')]
    frames = sorted((d/'frames').glob('*.jpg'))
    arm_t = next((e['t'] for e in events if e['kind'] == 'px4_arming_state' and e['value'] == 2), None)
    disarm_t = next((e['t'] for e in events if e['kind'] == 'px4_arming_state' and e['value'] == 1
                     and arm_t is not None and e['t'] > arm_t), None)
    # estimate -> truth frame: align position and yaw at arming (display only)
    if len(est) and len(truth) and arm_t is not None:
        i_t = np.searchsorted(truth[:, 0], arm_t); i_e = np.searchsorted(est[:, 0], arm_t)
        i_t, i_e = min(i_t, len(truth)-1), min(i_e, len(est)-1)
        yaw = lambda q: math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2))
        dy = yaw(truth[i_t, 5:9])-yaw(est[i_e, 5:9])
        R = np.array([[math.cos(dy), -math.sin(dy)], [math.sin(dy), math.cos(dy)]])
        est_xy = (est[:, 2:4]-est[i_e, 2:4])@R.T+truth[i_t, 2:4]
        sp_xy = (sp[:, 2:4]-est[i_e, 2:4])@R.T+truth[i_t, 2:4] if len(sp) else np.zeros((0, 2))
    else:
        est_xy, sp_xy = np.zeros((0, 2)), np.zeros((0, 2))
    segs = course_segments(a.course_obj) if a.course_obj else []
    allxy = np.vstack([truth[:, 2:4]] + ([np.array([s[0] for s in segs])] if segs else []))
    centre = (allxy.min(0)+allxy.max(0))/2 if len(allxy) else np.zeros(2)
    half = a.extent or max(1.5, float(np.abs(allxy-centre).max())+.5) if len(allxy) else 3.
    H = 400; W = 640; M = 400
    to_px = lambda xy: (int(M/2+(xy[0]-centre[0])/half*(M/2-10)), int(M/2-(xy[1]-centre[1])/half*(M/2-10)))
    base = np.full((M, M, 3), 255, np.uint8)
    for s0, s1 in segs:
        cv2.line(base, to_px(s0), to_px(s1), (150, 150, 150), 1, cv2.LINE_AA)
    out = d/'video'; out.mkdir(exist_ok=True)
    tmp = out/'frames'; tmp.mkdir(exist_ok=True)
    state, mission = {}, {}
    ev_i = 0
    times = [float(f.stem) for f in frames]
    for k, (t, f) in enumerate(zip(times, frames)):
        while ev_i < len(events) and events[ev_i]['t'] <= t:
            e = events[ev_i]; ev_i += 1
            if e['kind'] in ('px4_arming_state', 'px4_nav_state', 'vo_state'):
                state[e['kind']] = e['value']
            if e['kind'] == 'mission_status':
                mission = e['value']
        cam = cv2.resize(cv2.imread(str(f)), (W, H))
        m = base.copy()
        for xy, col, upto in ((sp_xy, (170, 170, 170), sp[:, 0] if len(sp) else []),
                              (est_xy, (0, 140, 255), est[:, 0] if len(est) else []),
                              (truth[:, 2:4], (200, 80, 0), truth[:, 0])):
            n = int(np.searchsorted(upto, t)) if len(upto) else 0
            if n > 1:
                cv2.polylines(m, [np.array([to_px(q) for q in xy[:n]], np.int32)], False, col, 2, cv2.LINE_AA)
        n = int(np.searchsorted(truth[:, 0], t)) if len(truth) else 0
        if n:
            cv2.circle(m, to_px(truth[n-1, 2:4]), 6, (200, 80, 0), -1)
        canvas = np.full((H+60, W+M, 3), 30, np.uint8)
        canvas[60:, :W] = cam
        canvas[60:60+M, W:] = m
        since = '' if arm_t is None or t < arm_t else f'  armed +{(min(t, disarm_t) if disarm_t else t)-arm_t:6.1f} s'
        line1 = f'{a.title}   t {t:6.1f} s{since}'
        line2 = (f'PX4 {"ARMED" if state.get("px4_arming_state") == 2 else "disarmed"} '
                 f'{NAV.get(state.get("px4_nav_state"), state.get("px4_nav_state"))}  cuVSLAM vo_state {state.get("vo_state")}  '
                 f'mission {mission.get("state") or ""} next {mission.get("next") or ""}  '
                 f'| map: truth blue, cuVSLAM orange, setpoints grey')
        cv2.putText(canvas, line1, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, line2, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, .45, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.imwrite(str(tmp/f'{k:06d}.jpg'), canvas)
    cmd = ['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(a.fps), '-i', str(tmp/'%06d.jpg'),
           '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '23', str(out/'flight.mp4')]
    subprocess.run(cmd, check=True)
    for f in tmp.glob('*.jpg'):
        f.unlink()
    tmp.rmdir()
    summary = {'frames': len(frames), 'arm_t': arm_t, 'disarm_t': disarm_t,
               'arm_to_disarm_s': None if arm_t is None or disarm_t is None else round(disarm_t-arm_t, 2)}
    (out/'video.json').write_text(json.dumps(summary, indent=1))
    print(out/'flight.mp4', summary)


if __name__ == '__main__':
    main()

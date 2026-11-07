"""One CSV per flight (arm -> disarm) + a JSON sidecar. This is the raw material for refining.

Written by the node at 20 Hz to <log_dir>/flight_<date>_<time>.csv. On the drone the default
log_dir is /root/team_ws/flight_logs (= ~/team_ws/flight_logs on the drone host, so it survives
the container). Columns:

    t                 seconds since the log opened (ROS clock)
    phase, cp         mission phase and the checkpoint index being flown to
    healthy, why      the health verdict and its reason
    armed, offboard, landed, vo_state, odom_age
    px py pz          cuVSLAM position (odom ENU)          vx vy vz  velocity (world)    yaw
    cmd_kind          pose | twist | none
    cmd_x cmd_y cmd_z cmd_yaw   the pose setpoint we sent (the carrot)
    cmd_vz            the twist we sent (touchdown)
    a0..a3            the policy action (FLY only)
    off_path corridor how far off the path, and the allowed corridor
    o0..o19           the full observation the policy saw (FLY only)

Use it to: replay what the policy saw (o0..o19 -> same network -> same action), fit the real
drone's response for the training simulator (team_mission.rl.fit_dynamics), and plot a flight.
"""
from __future__ import annotations

import csv
import json
import math
import os
import time

import numpy as np

from team_mission.control import OBS_DIM

COLUMNS = (['t', 'phase', 'cp', 'healthy', 'why', 'armed', 'offboard', 'landed', 'vo_state', 'odom_age',
            'px', 'py', 'pz', 'vx', 'vy', 'vz', 'yaw', 'cmd_kind', 'cmd_x', 'cmd_y', 'cmd_z', 'cmd_yaw',
            'cmd_vz', 'a0', 'a1', 'a2', 'a3', 'off_path', 'corridor'] + [f'o{k}' for k in range(OBS_DIM)])


def default_log_dir() -> str:
    if os.path.isdir('/root/team_ws'):
        return '/root/team_ws/flight_logs'
    return os.path.expanduser('~/team_mission_logs')


class FlightLogger:
    def __init__(self, log_dir: str = ''):
        self.dir = log_dir or default_log_dir()
        self.fh = None
        self.w = None
        self.t0 = 0.0
        self.path = ''

    @property
    def open(self) -> bool:
        return self.fh is not None

    def start(self, now: float, meta: dict) -> str:
        self.close()
        os.makedirs(self.dir, exist_ok=True)
        stem = os.path.join(self.dir, time.strftime('flight_%Y%m%d_%H%M%S'))
        self.path = stem + '.csv'
        self.fh = open(self.path, 'w', newline='')
        self.w = csv.writer(self.fh)
        self.w.writerow(COLUMNS)
        self.t0 = now
        self.meta_path = stem + '.json'
        self.meta = dict(meta)
        self._write_meta()
        return self.path

    def _write_meta(self):
        with open(self.meta_path, 'w') as fh:
            json.dump(self.meta, fh, indent=2, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))

    def update_meta(self, **kw):
        if self.fh is not None:
            self.meta.update(kw)
            self._write_meta()

    def row(self, now, s, out):
        if self.w is None:
            return
        f = lambda v: '' if v is None or (isinstance(v, float) and not math.isfinite(v)) else (  # noqa: E731
            f'{v:.4f}' if isinstance(v, float) else v)
        pos = s.pos if s.pos is not None else [None] * 3
        vel = s.vel if s.vel is not None else [None] * 3
        cmd = out.position if out.position is not None else [None] * 3
        lg = out.log
        a = lg.get('action', [None] * 4)
        obs = lg.get('obs', [None] * OBS_DIM)
        vz = out.velocity[2] if out.velocity is not None else None
        self.w.writerow([f(now - self.t0), lg.get('phase'), lg.get('cp'), int(bool(lg.get('healthy'))),
                         lg.get('why', ''), int(s.armed), int(s.offboard_active),
                         '' if s.landed is None else int(s.landed), '' if s.vo_state is None else s.vo_state,
                         f(min(s.odom_age, 99.0))]
                        + [f(None if v is None else float(v)) for v in pos]
                        + [f(None if v is None else float(v)) for v in vel]
                        + [f(None if s.yaw is None else float(s.yaw)), out.kind]
                        + [f(None if v is None else float(v)) for v in cmd]
                        + [f(None if out.yaw is None else float(out.yaw)), f(None if vz is None else float(vz))]
                        + [f(None if v is None else float(v)) for v in a]
                        + [f(lg.get('off_path')), f(lg.get('corridor'))]
                        + [f(None if v is None else float(v)) for v in np.asarray(obs, dtype=object)])

    def close(self):
        if self.fh is not None:
            self.fh.close()
        self.fh = self.w = None

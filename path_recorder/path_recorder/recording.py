"""Recording logic, no ROS: turns a stream of odometry + Sky Beta state into saved path points.

Frame of every saved point ("arm frame")
---------------------------------------
Relative to the drone's pose at the moment it was ARMED (sitting on its start spot):

    x   metres forward  (along the drone's heading at arming)
    y   metres left
    z   metres up       (0 = height of the body when armed, on the ground)
    yaw radians, CCW positive, 0 = the heading at arming
    qx qy qz qw   the same heading as a quaternion (rotation about z only)

A later replay arms on the same spot facing the same way, reads its own odom pose at arming
(p0, yaw0) and publishes, for every point, ``/skybeta/setpoint/pose`` (frame_id ``odom``):

    position    = p0 + Rz(yaw0) @ (x, y, 0) + (0, 0, z)
    orientation = quaternion of (yaw0 + yaw)

That is all ``/skybeta/setpoint/pose`` takes (PoseStamped: position + orientation in ENU odom),
so the file is directly replayable. The raw odom pose of the recording flight is stored too.

When a point is saved
---------------------
* the first odometry after arming ("start")
* every ``spacing_m`` (15 cm) of travel from the last saved point ("distance")
* every ``turn_deg`` (15 deg) of heading change from the last saved point ("turn")
* the last odometry before disarming ("end")

Flags on a point (comma-separated): ``odom_gap`` (odometry paused > gap_s before it),
``odom_jump`` (the pose jumped > jump_m between two odometry messages: a cuVSLAM reset or
relocalisation), ``depth_stale`` (no depth image in the last 0.5 s), ``px4_no_position``
(PX4 reported no valid local position), ``started_armed`` (the recorder started while the
drone was already armed: the anchor is not the start spot).
"""
from __future__ import annotations

from collections import deque
import csv
import json
import math
import os
from pathlib import Path
import time

import numpy as np

FORMAT = "path_recorder/1"
COLUMNS = ["idx", "t", "x", "y", "z", "yaw", "qx", "qy", "qz", "qw",
           "vx", "vy", "vz", "speed", "yaw_rate", "landed", "mode",
           "clear_left", "clear_center", "clear_right", "depth_age",
           "odom_x", "odom_y", "odom_z", "odom_qx", "odom_qy", "odom_qz", "odom_qw",
           "reason", "flags"]
NAV_STATE_NAMES = {0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION", 4: "AUTO_LOITER", 5: "AUTO_RTL",
                   10: "ACRO", 12: "DESCEND", 14: "OFFBOARD", 15: "STAB", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND"}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def runs_dir():
    """<team_ws>/runs/paths when installed from a team workspace (symlink build), else ~/path_recordings."""
    env = os.environ.get("PATH_RECORDER_DIR")
    if env:
        return Path(env)
    for p in Path(os.path.realpath(__file__)).parents:
        if p.name == "team_ws":
            return p / "runs" / "paths"
    return Path.home() / "path_recordings"


def clearance(depth_m, fx=None, cx=None, band=(.35, .65), floor=.15, ceiling=8.0, pct=5):
    """(left, centre, right) distance in metres to the nearest thing ahead, from a depth image
    (metres, 0/nan = no data). Uses the middle horizontal band of rows (about flight height,
    not the floor or ceiling), splits the columns in thirds, and takes the 5th percentile of
    valid depths in each third (robust to a few noisy pixels). None where a third has no data.
    The camera only sees ~75 deg forward, so "left"/"right" are the forward-left / forward-right
    parts of the view, not the drone's sides."""
    h, w = depth_m.shape
    rows = depth_m[int(h * band[0]):int(h * band[1]):2, ::2]
    out = []
    for third in np.array_split(rows, 3, axis=1):
        v = third[np.isfinite(third) & (third > floor) & (third < ceiling)]
        out.append(round(float(np.percentile(v, pct)), 3) if v.size >= 20 else None)
    return tuple(out)


def decode_depth(data, width, height, encoding, step):
    """sensor_msgs/Image depth -> metres (float32 array). 16UC1 = millimetres, 32FC1 = metres."""
    if encoding in ("16UC1", "mono16"):
        a = np.frombuffer(data, dtype="<u2", count=height * step // 2).reshape(height, step // 2)[:, :width]
        return a.astype(np.float32) / 1000.0
    if encoding == "32FC1":
        return np.frombuffer(data, dtype="<f4", count=height * step // 4).reshape(height, step // 4)[:, :width].copy()
    raise ValueError(f"unsupported depth encoding {encoding!r} (want 16UC1 or 32FC1)")


class Recording:
    """One arm -> disarm recording: a folder with points.csv, meta.json, events.jsonl.

    Writes every point as it is taken and refreshes meta.json every few seconds, so a crash or
    a power cut mid-flight leaves a usable file."""

    def __init__(self, folder: Path, anchor_pos, anchor_q, started_armed=False, spacing_m=.15, turn_deg=15.0,
                 info=None, wall_time=None):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=False)
        self.p0 = np.asarray(anchor_pos, float)
        self.q0 = tuple(float(v) for v in anchor_q)
        self.yaw0 = yaw_of(*self.q0)
        self.spacing, self.turn = float(spacing_m), math.radians(turn_deg)
        self.started_armed = started_armed
        self.t0 = None
        self.n = 0
        self.length = 0.0
        self.max_z = 0.0
        self.airborne_s = 0.0
        self.last = None                   # (t, rel xyz, rel yaw) of the last saved point
        self.last_pose = None              # (t, rel xyz, rel yaw) of the last odometry
        self.pending_flags = set()
        self.flag_counts = {}
        self.history = deque(maxlen=60)    # (t, rel xyz, rel yaw) for velocity
        self.recent = deque(maxlen=8)      # last saved rows (for the dashboard)
        self.info = dict(info or {})
        self.created = wall_time or time.time()
        self.ended = None
        self._csv_fh = open(self.folder / "points.csv", "w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._csv_fh)
        self._csv.writerow(COLUMNS)
        self._csv_fh.flush()
        self._events = open(self.folder / "events.jsonl", "w", encoding="utf-8")
        self._meta_t = 0.0
        self.event("start", anchor_odom_position=list(self.p0), anchor_odom_yaw=round(self.yaw0, 4),
                   started_armed=started_armed)
        self.write_meta()

    # ---------------------------------------------------------------- frames
    def to_rel(self, pos, q):
        d = np.asarray(pos, float) - self.p0
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        rel = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
        return rel, wrap(yaw_of(*q) - self.yaw0)

    def to_odom(self, rel_xyz, rel_yaw):
        """The inverse (what a replay does with its own anchor): arm frame -> this flight's odom."""
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        x, y, z = rel_xyz
        return self.p0 + np.array([c * x - s * y, s * x + c * y, z]), wrap(self.yaw0 + rel_yaw)

    # ---------------------------------------------------------------- input
    def event(self, kind, **kw):
        if self._events.closed:
            return
        self._events.write(json.dumps({"t": None if self.t0 is None else round(self._now - self.t0, 3),
                                       "kind": kind, **kw}, default=float) + "\n")
        self._events.flush()

    _now = 0.0

    def flag(self, name):
        self.pending_flags.add(name)

    def odometry(self, t, pos, q, landed, nav_state, clear=(None, None, None), depth_age=None, local_valid=True,
                 gap=False, jump=False):
        """One odometry sample (t = seconds, any monotonic clock). Returns the saved row or None."""
        self._now = t
        if self.t0 is None:
            self.t0 = t
        rel, ryaw = self.to_rel(pos, q)
        self._ctx = (tuple(pos), tuple(q), landed, nav_state, clear, depth_age)
        if self.last_pose is not None and not landed:
            self.airborne_s += max(0.0, t - self.last_pose[0])
        self.last_pose = (t, rel, ryaw)
        self.history.append((t, rel, ryaw))
        if gap:
            self.flag("odom_gap")
        if jump:
            self.flag("odom_jump")
        if not local_valid:
            self.flag("px4_no_position")
        if depth_age is None or depth_age > .5:
            self.flag("depth_stale")
        reason = None
        if self.last is None:
            reason = "start"
        else:
            moved = float(np.linalg.norm(rel - self.last[1]))
            if moved >= self.spacing:
                reason = "distance"
            elif abs(wrap(ryaw - self.last[2])) >= self.turn:
                reason = "turn"
        if reason is None:
            return None
        return self._save(t, rel, ryaw, pos, q, landed, nav_state, clear, depth_age, reason)

    def _velocity(self, t):
        old = [h for h in self.history if t - h[0] >= .2]
        if not old:
            return (0.0, 0.0, 0.0), 0.0
        t1, p1, y1 = old[-1]
        tn, pn, yn = self.history[-1]
        dt = max(tn - t1, 1e-3)
        return tuple((pn - p1) / dt), wrap(yn - y1) / dt

    def _save(self, t, rel, ryaw, pos, q, landed, nav_state, clear, depth_age, reason):
        if self.last is not None:
            self.length += float(np.linalg.norm(rel - self.last[1]))
        self.max_z = max(self.max_z, float(rel[2]))
        v, yr = self._velocity(t)
        flags = sorted(self.pending_flags)
        if self.started_armed:
            flags.append("started_armed")
        for f in flags:
            self.flag_counts[f] = self.flag_counts.get(f, 0) + 1
        self.pending_flags = set()
        half = ryaw / 2
        r = lambda v, n=4: "" if v is None else round(float(v), n)
        row = [self.n, r(t - self.t0, 3), r(rel[0]), r(rel[1]), r(rel[2]), r(ryaw),
               0.0, 0.0, r(math.sin(half), 6), r(math.cos(half), 6),
               r(v[0], 3), r(v[1], 3), r(v[2], 3), r(math.hypot(v[0], v[1]), 3), r(yr, 3),
               int(bool(landed)), NAV_STATE_NAMES.get(nav_state, str(nav_state)),
               r(clear[0], 3), r(clear[1], 3), r(clear[2], 3), r(depth_age, 3),
               r(pos[0]), r(pos[1]), r(pos[2]), r(q[0], 6), r(q[1], 6), r(q[2], 6), r(q[3], 6),
               reason, ";".join(flags)]
        self._csv.writerow(row)
        self._csv_fh.flush()
        self.n += 1
        self.last = (t, rel.copy(), ryaw)
        self.recent.append(dict(zip(COLUMNS, row)))
        if t - self._meta_t > 2.0:
            self._meta_t = t
            self.write_meta()
        return row

    # ---------------------------------------------------------------- end
    def close(self, ended="disarm"):
        """Save the final pose as an 'end' point, write meta.json, close the files."""
        if self.ended is not None:
            return
        if self.last_pose is not None and self.last is not None and self.last_pose[0] > self.last[0]:
            t, rel, ryaw = self.last_pose
            pos, q, landed, nav, clear, age = self._ctx
            self._save(t, rel, ryaw, pos, q, landed, nav, clear, age, "end")
        self.ended = ended
        self.event("end", ended=ended, points=self.n, length_m=round(self.length, 3))
        self.write_meta()
        self._csv_fh.close()
        self._events.close()

    @property
    def duration(self):
        return 0.0 if self.t0 is None or self.last_pose is None else self.last_pose[0] - self.t0

    @property
    def aborted(self):
        """Probably not a real path: under 2 s in the air or under 0.5 m of travel."""
        return self.airborne_s < 2.0 or self.length < .5

    def meta(self):
        return {
            "format": FORMAT,
            "created": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.created)),
            "status": "recording" if self.ended is None else "complete",
            "ended": self.ended,
            "aborted_or_short": self.aborted if self.ended else None,
            "points": self.n,
            "length_m": round(self.length, 3),
            "duration_s": round(self.duration, 2),
            "airborne_s": round(self.airborne_s, 2),
            "max_height_m": round(self.max_z, 3),
            "spacing_m": self.spacing,
            "turn_deg": round(math.degrees(self.turn), 1),
            "flags": self.flag_counts,
            "anchor": {"odom_position": [round(float(v), 4) for v in self.p0], "odom_q": list(self.q0),
                       "odom_yaw": round(self.yaw0, 5), "started_armed": self.started_armed},
            "frame": "arm frame: x forward, y left, z up (m), yaw CCW (rad), all relative to the pose at arming",
            "replay": "setpoint position = p0 + Rz(yaw0)*(x, y) + z; yaw = yaw0 + yaw; publish as "
                      "geometry_msgs/PoseStamped frame_id 'odom' on /skybeta/setpoint/pose",
            "columns": COLUMNS,
            **self.info,
        }

    def write_meta(self):
        tmp = self.folder / "meta.json.tmp"
        tmp.write_text(json.dumps(self.meta(), indent=1, default=float), encoding="utf-8")
        os.replace(tmp, self.folder / "meta.json")


def load(folder):
    """(meta dict, list of row dicts with numbers as floats) of a recording folder."""
    folder = Path(folder)
    meta = json.loads((folder / "meta.json").read_text(encoding="utf-8"))
    rows = []
    with open(folder / "points.csv", newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            for k, v in r.items():
                if k not in ("mode", "reason", "flags") and v != "":
                    r[k] = float(v)
                elif v == "" and k not in ("mode", "reason", "flags"):
                    r[k] = None
            rows.append(r)
    return meta, rows

"""Replay logic, no ROS: converts saved path points back into ENU odom setpoint poses.

Inverse of path_recorder/recording.py:
-------------------------------------
path_recorder saved each point in the "arm frame", relative to the pose at arming (p0, yaw0):
    x   metres forward (along drone heading at arming)
    y   metres left
    z   metres up      (0 = ground height when armed)
    yaw radians CCW    (0 = heading at arming)

Replay takes the drone's current odom pose at arming (p0_replay, yaw0_replay) and computes,
for every point or interpolated step along the path:
    position_odom    = p0_replay + Rz(yaw0_replay) @ (x, y, 0) + (0, 0, z)
    orientation_odom = wrap(yaw0_replay + yaw)

Published to /skybeta/setpoint/pose (geometry_msgs/PoseStamped in ENU odom frame).
"""
from __future__ import annotations

import csv
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


def wrap(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw from quaternion (rotation about z)."""
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Return (qx, qy, qz, qw) for pure yaw rotation."""
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def arm_to_odom(p0: Tuple[float, float, float] | np.ndarray,
                yaw0: float,
                rel_xyz: Tuple[float, float, float] | np.ndarray,
                rel_yaw: float) -> Tuple[np.ndarray, float]:
    """Convert arm-frame relative pose to ENU odom frame.

    Matches path_recorder Recording.to_odom():
        position    = p0 + Rz(yaw0) @ (x, y, 0) + (0, 0, z)
        orientation = wrap(yaw0 + rel_yaw)
    """
    c, s = math.cos(yaw0), math.sin(yaw0)
    x, y, z = rel_xyz
    pos = np.asarray(p0, float) + np.array([c * x - s * y, s * x + c * y, z])
    yaw = wrap(yaw0 + rel_yaw)
    return pos, yaw


def odom_to_arm(p0: Tuple[float, float, float] | np.ndarray,
                yaw0: float,
                odom_xyz: Tuple[float, float, float] | np.ndarray,
                odom_yaw: float) -> Tuple[np.ndarray, float]:
    """Convert ENU odom pose to arm-frame relative pose.

    Matches path_recorder Recording.to_rel().
    """
    d = np.asarray(odom_xyz, float) - np.asarray(p0, float)
    c, s = math.cos(yaw0), math.sin(yaw0)
    rel = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
    return rel, wrap(odom_yaw - yaw0)


def runs_dir() -> Path:
    """Locate the paths directory.

    Checks PATH_PLAYER_DIR or PATH_RECORDER_DIR, then looks for
    <team_ws>/runs/paths, falling back to ~/path_recordings.
    """
    env = os.environ.get("PATH_PLAYER_DIR") or os.environ.get("PATH_RECORDER_DIR")
    if env:
        return Path(env)
    for p in Path(os.path.realpath(__file__)).parents:
        if p.name == "team_ws":
            return p / "runs" / "paths"
    return Path.home() / "path_recordings"


def find_latest_recording(base_dir: Optional[Path | str] = None) -> Optional[Path]:
    """Find the most recent valid recording folder in base_dir."""
    directory = Path(base_dir) if base_dir else runs_dir()
    if not directory.exists() or not directory.is_dir():
        return None

    candidates = []
    for item in directory.iterdir():
        if item.is_dir() and (item / "points.csv").is_file():
            candidates.append(item)

    if not candidates:
        return None

    # Sort by directory name (path-YYYYMMDD-HHMMSS-NNN) or mtime
    candidates.sort(key=lambda p: p.name, reverse=True)
    return candidates[0]


@dataclass
class TrajectoryPoint:
    """One point in the trajectory (arm frame coordinates)."""
    idx: int
    t: float
    x: float
    y: float
    z: float
    yaw: float
    speed: float = 0.0
    yaw_rate: float = 0.0
    landed: bool = False
    reason: str = ""
    flags: str = ""

    @property
    def pos(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=float)


def load_recording(folder: Path | str) -> Tuple[Dict[str, Any], List[TrajectoryPoint]]:
    """Load meta.json and points.csv from a path_recorder recording folder."""
    folder = Path(folder)
    if folder.is_file() and folder.name == "points.csv":
        folder = folder.parent

    meta_file = folder / "meta.json"
    meta = json.loads(meta_file.read_text(encoding="utf-8")) if meta_file.is_file() else {}

    points: List[TrajectoryPoint] = []
    csv_file = folder / "points.csv"
    if not csv_file.is_file():
        raise FileNotFoundError(f"Missing points.csv in {folder}")

    with open(csv_file, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            idx = int(row.get("idx", len(points)))
            t = float(row.get("t", 0.0))
            x = float(row.get("x", 0.0))
            y = float(row.get("y", 0.0))
            z = float(row.get("z", 0.0))
            yaw = float(row.get("yaw", 0.0))
            speed = float(row.get("speed", 0.0)) if row.get("speed") else 0.0
            yaw_rate = float(row.get("yaw_rate", 0.0)) if row.get("yaw_rate") else 0.0
            landed = bool(int(float(row.get("landed", 0.0)))) if row.get("landed") else False
            reason = str(row.get("reason", ""))
            flags = str(row.get("flags", ""))

            points.append(TrajectoryPoint(
                idx=idx, t=t, x=x, y=y, z=z, yaw=yaw,
                speed=speed, yaw_rate=yaw_rate, landed=landed,
                reason=reason, flags=flags,
            ))

    if not points:
        raise ValueError(f"No points found in {csv_file}")

    return meta, points


class TrajectoryPlayer:
    """Manages playback of a recorded trajectory in ENU odom frame.

    Supports three playback modes:
    - 'time': follows recorded timestamps t, smoothly interpolating between waypoints.
    - 'adaptive': follows timestamps, but pauses if drone lags behind by > max_lag_m.
    - 'waypoint': advances to point i+1 when actual position is within arrival_radius of point i.
    """

    def __init__(self,
                 points: List[TrajectoryPoint],
                 meta: Optional[Dict[str, Any]] = None,
                 anchor_pos: Optional[Tuple[float, float, float] | np.ndarray] = None,
                 anchor_yaw: float = 0.0,
                 speed_factor: float = 1.0,
                 arrival_radius_m: float = 0.25,
                 mode: str = "adaptive",
                 max_lag_m: float = 0.5,
                 command_yaw: bool = True):
        self.points = list(points)
        self.meta = meta or {}
        self.p0 = np.zeros(3, dtype=float) if anchor_pos is None else np.asarray(anchor_pos, float)
        self.yaw0 = float(anchor_yaw)
        self.speed_factor = max(0.01, float(speed_factor))
        self.arrival_radius = float(arrival_radius_m)
        self.mode = mode.lower()
        self.max_lag = float(max_lag_m)
        self.command_yaw = bool(command_yaw)

        # Precompute path metrics
        self.total_points = len(self.points)
        self.total_duration_s = max(0.0, self.points[-1].t - self.points[0].t)
        self.segment_lengths = [
            float(np.linalg.norm(self.points[i + 1].pos - self.points[i].pos))
            for i in range(self.total_points - 1)
        ]
        self.total_length_m = sum(self.segment_lengths)

        # State tracking
        self.started = False
        self.paused = False
        self.finished = False
        self.current_idx = 0
        self.sim_time = 0.0                  # simulated playback seconds along path (t)
        self.last_update_wall_t = None
        self.active_lag_m = 0.0
        self.last_target_odom = arm_to_odom(self.p0, self.yaw0, self.points[0].pos, self.points[0].yaw)

    def set_anchor(self, pos: Tuple[float, float, float] | np.ndarray, yaw: float):
        """Update the arming anchor (p0, yaw0)."""
        self.p0 = np.asarray(pos, float)
        self.yaw0 = wrap(float(yaw))
        if self.points:
            self.last_target_odom = arm_to_odom(self.p0, self.yaw0, self.points[self.current_idx].pos, self.points[self.current_idx].yaw)

    def start(self, now_s: float):
        """Begin trajectory playback."""
        self.started = True
        self.paused = False
        self.finished = False
        self.current_idx = 0
        self.sim_time = self.points[0].t
        self.last_update_wall_t = now_s
        self.last_target_odom = arm_to_odom(self.p0, self.yaw0, self.points[0].pos, self.points[0].yaw)

    def pause(self):
        """Pause playback."""
        self.paused = True

    def resume(self, now_s: float):
        """Resume playback."""
        self.paused = False
        self.last_update_wall_t = now_s

    def is_finished(self) -> bool:
        return self.finished

    def progress_pct(self) -> float:
        """Percentage of path completed (0.0 to 100.0)."""
        if self.total_points <= 1:
            return 100.0 if self.finished else 0.0
        if self.mode == "waypoint":
            return min(100.0, (self.current_idx / (self.total_points - 1)) * 100.0)
        # Time-based progress
        if self.total_duration_s <= 0.0:
            return 100.0 if self.finished else 0.0
        return min(100.0, (self.sim_time / self.total_duration_s) * 100.0)

    def remaining_distance_m(self, current_pos_odom: Optional[np.ndarray | Tuple[float, float, float]] = None) -> float:
        """Estimate remaining path length in metres."""
        if self.finished or self.current_idx >= self.total_points - 1:
            return 0.0
        rem = sum(self.segment_lengths[self.current_idx:])
        if current_pos_odom is not None:
            # Add distance from current position to next waypoint target
            target_odom, _ = arm_to_odom(self.p0, self.yaw0, self.points[self.current_idx].pos, self.points[self.current_idx].yaw)
            rem += float(np.linalg.norm(np.asarray(current_pos_odom, float) - target_odom))
        return rem

    def update(self,
               now_s: float,
               current_pos_odom: Optional[np.ndarray | Tuple[float, float, float]] = None
               ) -> Tuple[np.ndarray, float, Dict[str, Any]]:
        """Advance trajectory playback and return (target_pos_odom, target_yaw_odom, status_info)."""
        if not self.started:
            # Not started yet: output initial point relative to anchor
            p_first = self.points[0]
            pos, yaw = arm_to_odom(self.p0, self.yaw0, p_first.pos, p_first.yaw)
            return pos, yaw, {"status": "WAITING"}

        dt_wall = 0.0
        if self.last_update_wall_t is not None:
            dt_wall = max(0.0, now_s - self.last_update_wall_t)
        self.last_update_wall_t = now_s

        if self.finished:
            p_last = self.points[-1]
            pos, yaw = arm_to_odom(self.p0, self.yaw0, p_last.pos, p_last.yaw)
            return pos, yaw, {"status": "FINISHED", "lag_m": self.active_lag_m}

        # Check lag if current drone position is provided
        drone_pos = np.asarray(current_pos_odom, float) if current_pos_odom is not None else None

        if self.mode in ("time", "adaptive"):
            # Tentatively advance simulation time
            adv_time = 0.0
            if not self.paused:
                adv_time = dt_wall * self.speed_factor
                self.sim_time += adv_time

            # Find segment [idx, idx+1] containing sim_time
            while self.current_idx < self.total_points - 1 and self.sim_time >= self.points[self.current_idx + 1].t:
                self.current_idx += 1

            if self.current_idx >= self.total_points - 1:
                # Reached final point
                self.finished = True
                p = self.points[-1]
                pos, yaw = arm_to_odom(self.p0, self.yaw0, p.pos, p.yaw)
                self.last_target_odom = (pos, yaw)
                return pos, yaw, {"status": "FINISHED", "lag_m": self.active_lag_m}

            # Interpolate between current_idx and current_idx + 1
            p0 = self.points[self.current_idx]
            p1 = self.points[self.current_idx + 1]
            dt_seg = p1.t - p0.t
            alpha = 0.0 if dt_seg <= 1e-4 else np.clip((self.sim_time - p0.t) / dt_seg, 0.0, 1.0)

            interp_pos = p0.pos + alpha * (p1.pos - p0.pos)
            interp_yaw = wrap(p0.yaw + alpha * wrap(p1.yaw - p0.yaw))

            candidate_pos, candidate_yaw = arm_to_odom(self.p0, self.yaw0, interp_pos, interp_yaw)

            # In adaptive mode, check if candidate target exceeds max allowed lag from drone
            lag_exceeded = False
            if self.mode == "adaptive" and drone_pos is not None:
                lag = float(np.linalg.norm(drone_pos - candidate_pos))
                if lag > self.max_lag:
                    lag_exceeded = True
                    # Revert sim_time advance so trajectory clock pauses until drone catches up
                    self.sim_time -= adv_time
                    candidate_pos, candidate_yaw = self.last_target_odom
                    self.active_lag_m = float(np.linalg.norm(drone_pos - candidate_pos))
                else:
                    self.active_lag_m = lag
            elif drone_pos is not None:
                self.active_lag_m = float(np.linalg.norm(drone_pos - candidate_pos))

            self.last_target_odom = (candidate_pos, candidate_yaw)

            status_info = {
                "status": "LAG_PAUSED" if lag_exceeded else ("PAUSED" if self.paused else "PLAYING"),
                "idx": self.current_idx,
                "sim_time": round(self.sim_time, 2),
                "lag_m": round(self.active_lag_m, 3),
                "progress_pct": round(self.progress_pct(), 1),
            }
            return candidate_pos, candidate_yaw, status_info

        else:
            # Mode == 'waypoint'
            p = self.points[self.current_idx]
            target_pos, target_yaw = arm_to_odom(self.p0, self.yaw0, p.pos, p.yaw)
            self.last_target_odom = (target_pos, target_yaw)

            if drone_pos is not None:
                dist = float(np.linalg.norm(drone_pos - target_pos))
                self.active_lag_m = dist
                if dist <= self.arrival_radius:
                    if self.current_idx < self.total_points - 1:
                        self.current_idx += 1
                        # Re-evaluate target for advanced waypoint
                        p = self.points[self.current_idx]
                        target_pos, target_yaw = arm_to_odom(self.p0, self.yaw0, p.pos, p.yaw)
                        self.last_target_odom = (target_pos, target_yaw)
                    else:
                        self.finished = True

            status_info = {
                "status": "FINISHED" if self.finished else ("PAUSED" if self.paused else "PLAYING"),
                "idx": self.current_idx,
                "dist_to_wp_m": round(self.active_lag_m, 3),
                "progress_pct": round(self.progress_pct(), 1),
            }
            return target_pos, target_yaw, status_info

"""The node's plumbing that needs no ROS (so tests run it): journal, run folder, depth pose
matching, the planning thread, and the Offboard-timestamp check used by check_pipeline.

Ported from the FlyX-proven ``flyx/fly_pathplan.py``: on a CPU-starved lab machine anything slow
on the executor thread (a disk write, a wait on the planner process) stalls the setpoint stream,
and PX4 then drops out of Offboard. So the journal is written by its own thread from a queue,
and planning runs on its own thread, never on the executor.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import queue
import threading
import time
import traceback

DEPTH_MATCH_S = .05         # an image is placed with the odometry pose within this of its stamp
MATCHED_BEFORE_SKIP = 20    # once this many frames matched, an unmatched one fell in a stall: skip it


# ---------------------------------------------------------------------- run folder
def make_run_dir(configured, name, fallbacks=None):
    """Create <dir>/<name>: the configured dir, else ~/flyx-runs, else /tmp/flyx-runs (a
    read-only checkout or a full disk must never stop the node from starting)."""
    if fallbacks is None:
        fallbacks = (Path.home() / 'flyx-runs', Path('/tmp/flyx-runs'))
    tried = []
    for base in [Path(configured)] + [Path(f) for f in fallbacks]:
        out = base / name
        try:
            out.mkdir(parents=True, exist_ok=True)
            probe = out / '.write_test'
            probe.write_text('')
            probe.unlink()
            return out
        except OSError as e:
            tried.append(f'{out}: {e}')
    raise OSError('no writable journal folder: ' + '; '.join(tried))


# ---------------------------------------------------------------------- journal
class Journal:
    """JSON-lines journal written by its own thread: ``log()`` only queues (never blocks on disk)."""

    def __init__(self, path):
        self.path = Path(path)
        self._file = open(self.path, 'w', encoding='utf-8')
        self._queue = queue.Queue()
        self._closed = threading.Event()
        self._writer = threading.Thread(target=self._write_loop, name='journal', daemon=True)
        self._writer.start()

    def log(self, record):
        if not self._closed.is_set():          # callbacks may still fire while shutting down
            self._queue.put(json.dumps(record, default=float) + '\n')

    def _write_loop(self):
        while True:
            line = self._queue.get()
            if line is None:
                return
            self._file.write(line)
            self._file.flush()

    def close(self, timeout=3.0):
        if self._closed.is_set():
            return
        self._closed.set()
        self._queue.put(None)
        self._writer.join(timeout)
        self._file.close()


# ---------------------------------------------------------------------- depth: pose at the image stamp
def match_pose(poses, stamp, stats, match_s=DEPTH_MATCH_S):
    """(position, q, turn rate) from the odometry nearest the image stamp, or None to skip the
    frame. ``poses`` = [(stamp s, position, q)], same clock as the camera. Projecting a frame
    with the latest pose smeared walls during turns. Unstamped / other-clock images get the
    latest pose, but once stamps have matched MATCHED_BEFORE_SKIP times an unmatched frame fell
    in a gap of the pose history (a program stall): it is skipped, not misplaced."""
    if not poses:
        return None
    t, pos, q = min(poses, key=lambda p: abs(p[0] - stamp))
    if stamp > 0 and abs(t - stamp) <= match_s:
        stats['matched'] = stats.get('matched', 0) + 1
        t0, _, q0 = min(poses, key=lambda p: abs(p[0] - (stamp - .15)))
        from pathplan.ros_node import yaw_of_quat
        dyaw = yaw_of_quat(*q) - yaw_of_quat(*q0)
        rate = abs(math.atan2(math.sin(dyaw), math.cos(dyaw))) / max(t - t0, .05)
        return pos, q, rate
    if stats.get('matched', 0) >= MATCHED_BEFORE_SKIP:
        stats['unmatched'] = stats.get('unmatched', 0) + 1
        return None
    stats['fallback'] = stats.get('fallback', 0) + 1
    _, pos, q = poses[-1]
    return pos, q, 0.0


# ---------------------------------------------------------------------- graph check (1 Hz)
VISION_TOPIC = '/fmu/in/vehicle_visual_odometry'


def foreign_publishers(counts, setpoint_topic, own_setpoint=True):
    """'' or why someone else is commanding / feeding PX4, from {topic: publisher count}.
    The mission treats it as a failed check: not READY on the ground, FAULT in the air."""
    found = []
    n = counts.get(setpoint_topic, 0) - (1 if own_setpoint else 0)
    if n > 0:
        found.append(f'{n} other publisher(s) on {setpoint_topic} (another competitor node?)')
    n = counts.get('/skybeta/setpoint/twist', 0)
    if n > 0:
        found.append(f'{n} publisher(s) on /skybeta/setpoint/twist')
    n = counts.get('/skybeta/state', 0)
    if n > 1:
        found.append(f'{n} flight interfaces running (/skybeta/state has {n} publishers)')
    n = counts.get(VISION_TOPIC, 0)
    if n > 1:           # two vision bridges (e.g. sim.launch's beside the lab's): PX4 fuses every sample twice
        found.append(f'{n} vision bridges ({VISION_TOPIC} has {n} publishers): stop one')
    return '; '.join(found)


# ---------------------------------------------------------------------- planning thread
class PlanningLoop:
    """Calls ``step()`` every ``period_s`` on its own thread. A planning step waits on the planner
    process for up to ~1 s; on the executor thread that wait stalled the setpoint stream."""

    def __init__(self, step, period_s=.2, on_error=None):
        self.step, self.period_s, self.on_error = step, period_s, on_error
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='planning', daemon=True)

    def start(self):
        self._thread.start()
        return self

    def _run(self):
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                self.step()
            except Exception:                               # keep planning; the mission sees no new target
                if self.on_error is not None:
                    self.on_error(traceback.format_exc())
            self._stop.wait(max(0.0, self.period_s - (time.monotonic() - t0)))

    def stop(self, timeout=3.0):
        """Stop and wait for the step in flight (before the planner's pipe is closed)."""
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout)


# ---------------------------------------------------------------------- Offboard timestamps (check_pipeline)
FAR_S = 1e6     # stamps this far apart are on different epochs (Unix wall clock vs PX4 boot time)


def offboard_stamp_verdict(ocm, odom, max_offset_s=.5, max_drift_s=.2):
    """Do flight_interface's OffboardControlMode stamps match PX4's own time base?

    ``ocm``, ``odom``: [(arrival monotonic s, timestamp us)] of /fmu/in/offboard_control_mode and
    /fmu/out/vehicle_odometry. PX4 v1.16's uxrce_dds_client maps an incoming stamp with
    ``min(stamp - offset, hrt_now)`` (msg.h.em ucdr_deserialize; offset = the timesync offset, 0
    with UXRCE_DDS_SYNCT=0) and its outgoing stamps with ``+ offset``, and commander's Offboard
    check is ``hrt_now < stamp + COM_OF_LOSS_T``. So on the ROS side, odometry stamps show PX4's
    clock and an OffboardControlMode stamp behind it is what PX4 sees as stale.

    Returns (level, message): OK / WARN / FAIL.
    """
    if len(ocm) < 10 or len(odom) < 10:
        return 'WARN', f'not enough samples (offboard_control_mode {len(ocm)}, vehicle_odometry {len(odom)})'
    odom = sorted(odom)
    arr = [a for a, _ in odom]
    import bisect
    d = []                                  # (arrival, our stamp - PX4's clock then), seconds
    for a, ts in sorted(ocm):
        i = bisect.bisect_left(arr, a)
        j = min(range(max(0, i - 1), min(len(odom), i + 1)), key=lambda k: abs(arr[k] - a))
        px4_now_us = odom[j][1] + (a - arr[j]) * 1e6
        d.append((a, (ts - px4_now_us) * 1e-6))
    offs = sorted(x for _, x in d)
    med = offs[len(offs) // 2]
    if med > FAR_S:
        return 'OK', (f'stamps {med:.3g} s ahead of PX4 time: Unix wall clock vs a PX4 without timesync '
                      '(UXRCE_DDS_SYNCT=0). PX4 clamps a future stamp to its own time on arrival '
                      '(min(stamp, now)), so the Offboard timeout counts arrival gaps')
    if med < -FAR_S:
        return 'FAIL', (f'stamps {-med:.3g} s behind PX4 time (sim time with timesync on?): fresh only by '
                        'unsigned wrap-around in uxrce_dds_client. Run flight_interface on the wall clock')
    n = max(3, len(d) // 4)                 # drift: first quarter vs last quarter of the window
    first = sorted(x for _, x in d[:n])[n // 2]
    last = sorted(x for _, x in d[-n:])[n // 2]
    drift, span = last - first, d[-1][0] - d[0][0]
    lo, hi = offs[max(0, len(offs) // 20)], offs[min(len(offs) - 1, len(offs) * 19 // 20)]
    detail = f'offset {med:+.3f} s (5-95 % {lo:+.3f}..{hi:+.3f}), drift {drift:+.3f} s over {span:.1f} s'
    if abs(drift) > max_drift_s:
        return 'FAIL', (detail + ': the stamps run on another clock rate than PX4 (sim time vs PX4 lockstep?) '
                        '- PX4 will see the Offboard stream as stale')
    if lo < -max_offset_s:
        return 'FAIL', detail + f': behind PX4 time by more than {max_offset_s} s - PX4 sees the stream as stale (COM_OF_LOSS_T)'
    if hi > max_offset_s:
        return 'FAIL', detail + ': stamps in the future of PX4 time - another clock (PX4 clamps them, but the base is wrong)'
    return 'OK', detail + ': same time base as PX4'

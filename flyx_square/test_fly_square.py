"""Offline tests of fly_square.py's SquareMission against a kinematic stand-in for PX4 (no ROS).

    python3 -m pytest -q flyx_square/test_fly_square.py
"""
from math import pi

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fly_square as m  # noqa: E402


class FakePX4:
    """Follows position setpoints (first-order, 1.5 m/s cap), honours Offboard/arm/land."""

    def __init__(self, start=(1.0, -2.0, -0.05), yaw=pi / 2, refuse_arm=False):
        self.tel = m.Telemetry()
        self.tel.position, self.tel.q = start, (cos_half(yaw), 0.0, 0.0, sin_half(yaw))
        self.tel.landed, self.tel.xy_valid, self.tel.z_valid = True, True, True
        self.refuse_arm, self.landing, self.ground_z = refuse_arm, False, start[2]
        self.trace = []

    def step(self, t, sp, cmd, dt=.05, stale_odom=False):
        tel = self.tel
        if cmd:
            if cmd[0] == m.CMD_DO_SET_MODE:
                tel.nav_state = m.NAV_OFFBOARD
            elif cmd[0] == m.CMD_ARM_DISARM and not self.refuse_arm:
                tel.arming_state = m.ARMED
            elif cmd[0] == m.CMD_NAV_LAND:
                self.landing, tel.nav_state = True, 18
        p = list(tel.position)
        if tel.arming_state == m.ARMED:
            if self.landing:
                p[2] = min(self.ground_z, p[2] + .5 * dt)
                if p[2] >= self.ground_z:
                    tel.landed, tel.arming_state = True, m.DISARMED
            elif sp is not None:
                tgt = sp[0]
                for i in range(3):
                    p[i] += max(-1.5 * dt, min(1.5 * dt, (tgt[i] - p[i]) * 2.0 * dt))
                tel.landed = p[2] < self.ground_z - .05 and False or tel.landed and p[2] >= self.ground_z - .02
        tel.position = tuple(p)
        tel.now = t
        tel.sim_now = t if not getattr(self, 'sim_frozen', False) else tel.sim_now
        tel.local_t = tel.status_t = tel.landed_t = t
        if not stale_odom:
            tel.odom_t, tel.odom_sim = t, tel.sim_now
        self.trace.append(tuple(p))


def cos_half(y):
    from math import cos
    return cos(y / 2)


def sin_half(y):
    from math import sin
    return sin(y / 2)


def run(px4, mission, seconds=200, stale_after=None):
    sp = cmd = None
    t = 0.0
    while t < seconds and mission.state not in ("DONE", "FAILED", "TAKEN_OVER"):
        px4.step(t, sp, cmd, stale_odom=stale_after is not None and t > stale_after)
        _, sp, cmd, _ = mission.tick(px4.tel)
        t += .05
    return t


def test_square_completes_and_lands_where_it_took_off():
    px4, mission = FakePX4(), m.SquareMission()
    t = run(px4, mission)
    assert mission.state == "DONE", mission.events[-3:]
    corners = [e["corner"] for e in mission.events if "corner" in e]
    assert corners == ["TAKEOFF", "FORWARD", "UP", "BACK"]
    xs, ys, zs = zip(*px4.trace)
    # forward = heading pi/2 in NED = east (+y); the square spans side m forward and up
    assert max(ys) - (-2.0) > m.SIDE_M - .15 and max(ys) - (-2.0) < m.SIDE_M + .1
    assert abs(min(zs) - (-0.05 - m.HEIGHT_M - m.SIDE_M)) < .1      # highest point
    assert max(abs(x - 1.0) for x in xs) < .05                       # no sideways motion
    assert abs(px4.tel.position[1] - (-2.0)) < .15                   # landed back at take-off
    assert t < 60


def test_refused_arm_fails_before_flying():
    px4, mission = FakePX4(refuse_arm=True), m.SquareMission()
    run(px4, mission)
    assert mission.state == "FAILED" and "arm" in mission.events[-1]["note"]
    assert all(p[2] == -0.05 for p in px4.trace)


def test_stale_odometry_in_flight_lands():
    px4, mission = FakePX4(), m.SquareMission()
    run(px4, mission, stale_after=14.0)          # outage mid-square
    assert mission.land_sent and any(e.get("to") == "LAND" and "odometry lost" in e.get("note", "")
                                     for e in mission.events)


def test_short_local_position_gap_does_not_land():
    """FlyX run 3: vehicle_local_position paused 0.69 s while odometry stayed fresh."""
    px4, mission = FakePX4(), m.SquareMission()
    sp = cmd = None
    t = 0.0
    while t < 200 and mission.state not in ("DONE", "FAILED", "TAKEN_OVER"):
        px4.step(t, sp, cmd)
        if 16.0 < t < 16.7:
            px4.tel.local_t = 16.0                 # local position frozen for 0.7 s
        _, sp, cmd, _ = mission.tick(px4.tel)
        t += .05
    assert mission.state == "DONE", mission.events[-2:]


def _fly(px4, mission, frozen_odom=None, frozen_local=None, seconds=200):
    sp = cmd = None
    t = 0.0
    while t < seconds and mission.state not in ("DONE", "FAILED", "TAKEN_OVER"):
        px4.step(t, sp, cmd, stale_odom=frozen_odom is not None and frozen_odom[0] < t < frozen_odom[1])
        if frozen_local and frozen_local[0] < t < frozen_local[1]:
            px4.tel.local_t = frozen_local[0]
        _, sp, cmd, _ = mission.tick(px4.tel)
        t += .05
    return t


def test_lab_run1_short_odometry_pause_holds_and_completes():
    """FlyX lab run 1: odometry paused ~0.3 s right after arming; the old guard landed."""
    px4, mission = FakePX4(), m.SquareMission()
    _fly(px4, mission, frozen_odom=(14.0, 14.5))
    assert mission.state == "DONE", mission.events[-2:]


def test_lab_run2_one_second_pause_mid_square_holds_and_completes():
    """FlyX lab run 2: 1.08 s odometry gap after 20 s of flight (sim time advanced too)."""
    px4, mission = FakePX4(), m.SquareMission()
    _fly(px4, mission, frozen_odom=(20.0, 21.2))
    assert mission.state == "DONE", mission.events[-2:]
    assert any("pause" in e for e in mission.events) and any("resume_after_s" in e for e in mission.events)


def test_setpoint_does_not_advance_while_paused():
    px4, mission = FakePX4(), m.SquareMission()
    sp = cmd = None
    t, sps = 0.0, []
    while t < 40 and mission.state not in ("DONE", "FAILED"):
        px4.step(t, sp, cmd, stale_odom=17.0 < t < 18.5)
        _, sp, cmd, note = mission.tick(px4.tel)
        if 17.6 < t < 18.4:
            sps.append(sp[0])
        t += .05
    assert sps and all(s == sps[0] for s in sps)          # frozen during the pause


def test_odometry_outage_lands():
    px4, mission = FakePX4(), m.SquareMission()
    _fly(px4, mission, frozen_odom=(16.0, 200.0))
    land = [e for e in mission.events if e.get("to") == "LAND"]
    assert land and "odometry lost" in land[0]["note"]


# FlyX lab run 2 (2026-11-07, journal): measured pauses, as (start offset after arming s, length s)
# odometry and the program's own control loop stalled together (control_gap up to 1.75 s);
# local_position paused up to 2.37 s; sim time kept running (~0.8x real time).
LAB_RUN2_ODOM = [(1.4, 0.31), (2.0, 0.45), (2.5, 0.62), (3.3, 0.43), (13.6, 0.41), (14.2, 0.41), (14.9, 1.41)]
LAB_RUN2_LOCAL = [(2.0, 0.70), (3.4, 1.49), (13.6, 0.43), (14.9, 2.37)]
LAB_RUN2_CONTROL = [(1.7, 0.70), (2.6, 0.62), (14.6, 1.22), (16.4, 1.75)]


def _in(t, spans):
    return any(a <= t < a + d for a, d in spans)


def test_lab_run2_pause_pattern_completes():
    """Replays run 2's measured pauses: odometry, local position, and stalls of the program
    itself (no ticks, so no setpoints) relative to arming. The old version landed; this holds."""
    px4, mission = FakePX4(), m.SquareMission()
    sp = cmd = None
    t, t_arm = 0.0, None
    while t < 300 and mission.state not in ("DONE", "FAILED", "TAKEN_OVER"):
        rel = None if t_arm is None else t - t_arm
        px4.step(t, sp, cmd, stale_odom=rel is not None and _in(rel, LAB_RUN2_ODOM))
        if rel is not None and _in(rel, LAB_RUN2_LOCAL):
            px4.tel.local_t = px4.tel.local_t if px4.tel.local_t < t else t - .05
            px4.tel.local_t = min(px4.tel.local_t, t_arm + max(a for a, d in LAB_RUN2_LOCAL if a <= rel))
        if rel is None or not _in(rel, LAB_RUN2_CONTROL):
            _, sp, cmd, _ = mission.tick(px4.tel)
        else:
            cmd = None                      # the program is stalled: nothing new is sent
        if t_arm is None and mission.state == "FLY":
            t_arm = t
        t += .05
    assert mission.state == "DONE", mission.events[-3:]
    assert any("pause" in e for e in mission.events)


def test_lab_run3_slow_topics_silent_6s_completes():
    """FlyX lab run 3: vehicle_status, land_detected and local_position silent ~6 s together
    (from ~11 s after arming) while odometry kept coming and PX4 stayed in Offboard. The previous
    version landed on "vehicle_status stale"; slow-topic silence must not land."""
    px4, mission = FakePX4(), m.SquareMission()
    sp = cmd = None
    t, t_arm = 0.0, None
    while t < 300 and mission.state not in ("DONE", "FAILED", "TAKEN_OVER"):
        px4.step(t, sp, cmd)
        rel = None if t_arm is None else t - t_arm
        if rel is not None and 11.0 <= rel < 17.0:              # slow topics silent, odometry fine
            px4.tel.status_t = px4.tel.local_t = px4.tel.landed_t = t_arm + 11.0
        _, sp, cmd, _ = mission.tick(px4.tel)
        if t_arm is None and mission.state == "FLY":
            t_arm = t
        t += .05
    assert mission.state == "DONE", mission.events[-3:]


def test_fresh_invalid_estimate_still_lands():
    px4, mission = FakePX4(), m.SquareMission()
    sp = cmd = None
    t = 0.0
    while t < 200 and mission.state not in ("DONE", "FAILED", "TAKEN_OVER"):
        px4.step(t, sp, cmd)
        if mission.state == "FLY" and t > 16.0:
            px4.tel.xy_valid = False                          # a fresh message says: invalid
        _, sp, cmd, _ = mission.tick(px4.tel)
        t += .05
    land = [e for e in mission.events if e.get("to") == "LAND"]
    assert land and "estimator invalid" in land[0]["note"]

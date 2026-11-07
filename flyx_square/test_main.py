"""Offline tests of main.py's SquareMission against a kinematic stand-in for PX4 (no ROS).

    python3 -m pytest -q test_main.py
"""
from math import pi

import main as m


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
        tel.local_t = tel.status_t = tel.landed_t = t
        if not stale_odom:
            tel.odom_t = t
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
    run(px4, mission, stale_after=11.0)          # mid-square (UP leg)
    assert mission.land_sent and any(e.get("to") == "LAND" and "odometry stale" in e.get("note", "")
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

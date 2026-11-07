"""The Sky Beta mission state machine, step by step (no ROS, no planner: a scripted one)."""
from math import cos, pi, sin

import numpy as np
import pytest

from dronehacks_pathplan.mission import Mission, MissionConfig, Telemetry
from pathplan.online import Command


class ScriptedPlanner:
    """Moves the target 1 m up course +y; 'done' once told so."""
    pad_xy = [-2.042, -0.792]
    n_checkpoints = 5

    def __init__(self):
        self.mode, self.events, self.cleared = "move", [], []

    def update(self, xy, yaw):
        if self.mode == "move":
            return Command("move", np.array([xy, [xy[0], xy[1] + 1.0]]), yaw=pi / 2)
        return Command(self.mode)


def q(yaw):
    return (0.0, 0.0, sin(yaw / 2), cos(yaw / 2))


def ground(now, **kw):
    t = Telemetry(now=now, odom_t=now, state_t=now, px4_link=True, armed=False, landed=True, nav_state=1,
                  offboard_active=False, local_valid=True, frozen=False, origin_z=0.0)
    for k, v in kw.items():
        setattr(t, k, v)
    return t


def run_until(m, tel_fn, t0, t1, dt=.05):
    outs, t = [], t0
    while t < t1:
        outs.append(m.tick(tel_fn(t)))
        t += dt
    return outs, t


def ready_mission(cfg=None):
    m = Mission(ScriptedPlanner(), cfg or MissionConfig())
    outs, t = run_until(m, lambda t: ground(t), 0.0, 2.2)
    assert m.state == "READY"
    return m, t


def test_publishes_nothing_until_healthy_then_streams_hold_here():
    m = Mission(ScriptedPlanner())
    outs, _ = run_until(m, lambda t: ground(t, odom_t=None), 0.0, 3.0)        # no odometry
    assert m.state == "WAIT_HEALTH" and all(o.setpoint is None for o in outs)
    assert "odometry" in outs[-1].note
    outs, t = run_until(m, lambda t: ground(t), 3.0, 4.9)                     # healthy, confirming
    assert m.state == "WAIT_HEALTH" and all(o.setpoint is None for o in outs)
    outs, t = run_until(m, lambda t: ground(t), t, 5.5)
    assert m.state == "READY" and outs[-1].setpoint is not None
    assert outs[-1].setpoint[1] is None                                       # yaw not commanded on the ground


@pytest.mark.parametrize("problem", [dict(px4_link=False), dict(local_valid=False), dict(landed=False),
                                     dict(frozen=True), dict(foreign="2 flight interfaces running")])
def test_each_ground_check_blocks_ready(problem):
    m = Mission(ScriptedPlanner())
    outs, _ = run_until(m, lambda t: ground(t, **problem), 0.0, 4.0)
    assert m.state == "WAIT_HEALTH" and all(o.setpoint is None for o in outs)


def test_armed_before_ready_is_never_flown():
    m = Mission(ScriptedPlanner())
    outs, t = run_until(m, lambda t: ground(t, armed=True), 0.0, 4.0)
    assert m.state == "WAIT_HEALTH" and "not flying this arming" in outs[-1].note
    outs, t = run_until(m, lambda t: ground(t, armed=True, offboard_active=True), t, 6.0)
    assert all(o.setpoint is None for o in outs)
    outs, t = run_until(m, lambda t: ground(t), t, 9.0)                       # disarmed: may become READY
    assert m.state == "READY"


def test_course_frame_is_latched_at_arming():
    # SLAM started elsewhere; on the mark the drone sits at odom (1.0, 0.5) heading 30 deg
    pos, yaw = (1.0, 0.5, 0.0), np.radians(30)
    m = Mission(ScriptedPlanner())
    run_until(m, lambda t: ground(t, position=pos, q_xyzw=q(yaw)), 0.0, 2.5)
    m.tick(ground(2.5, position=pos, q_xyzw=q(yaw), armed=True))
    assert m.state == "ARMED"
    xyz, cyaw, _ = m.frame.odometry_to_course(pos, q(yaw))
    assert np.allclose(xyz, [-2.515, .914, .05]) and np.isclose(cyaw, pi / 2)
    # 1 m along the heading in odom = 1 m up the first corridor
    xyz, _, _ = m.frame.odometry_to_course((1.0 + cos(yaw), .5 + sin(yaw), 0.0), q(yaw))
    assert np.allclose(xyz[:2], [-2.515, 1.914])


def test_takeoff_waits_for_offboard_and_climbs_on_a_ramp():
    m, t = ready_mission()
    m.tick(ground(t, armed=True))
    outs, t = run_until(m, lambda t: ground(t, armed=True), t, t + 3)        # armed, still in Altitude
    assert m.state == "ARMED"
    z = []
    for i in range(30):
        o = m.tick(ground(t, armed=True, offboard_active=True, nav_state=14, landed=False))
        z.append(m.frame.to_course(o.setpoint[0])[2])
        t += .05
    assert m.state == "TAKEOFF"
    floor = .05 + .3 + .03                                                    # launch height + floor + margin
    assert z[0] == pytest.approx(floor, abs=.01)                              # never below flight_interface's floor
    assert max(np.diff(z)) <= .4 * .05 + 1e-6                                 # 0.4 m/s ramp


def airborne(t, z, **kw):
    base = dict(armed=True, offboard_active=True, nav_state=14, landed=False, position=(0.0, 0.0, z))
    return ground(t, **{**base, **kw})


def flying_mission():
    m, t = ready_mission()
    m.tick(ground(t, armed=True))
    m.tick(airborne(t + .05, 0.0))
    t += .1
    run_until(m, lambda t: airborne(t, .85), t, t + 3)                        # reach altitude (0.85 + 0.05)
    t += 3
    assert m.state == "NAVIGATE"
    return m, t


def test_offboard_loss_mid_course_stops_commanding():
    m, t = flying_mission()
    m.plan()
    outs, t = run_until(m, lambda t: airborne(t, .85, offboard_active=False, nav_state=2), t, t + 1.0)
    assert m.state == "TAKEN_OVER" and outs[-1].setpoint is None
    assert "POSCTL" in m.result


def test_short_odometry_pause_holds_long_one_faults_once():
    m, t = flying_mission()
    m.plan()
    last = m.tick(airborne(t, .85)).setpoint
    stale = t
    outs, t = run_until(m, lambda t: airborne(t, .85, odom_t=stale), t + .05, t + 1.0)
    assert m.state == "NAVIGATE" and all(o.setpoint == last for o in outs[6:])   # held after 0.3 s
    outs, t = run_until(m, lambda t: airborne(t, .85, odom_t=stale), t, stale + 3.5)
    assert m.state == "FAULT"
    assert sum(o.hold_request for o in outs) == 1 and outs[-1].setpoint is None


def test_heading_is_rate_limited():
    m, t = flying_mission()
    m.target = (np.array([-2.515, 1.9]), -pi / 2)                             # turn around
    yaws = []
    for _ in range(40):
        o = m.tick(airborne(t, .85))
        yaws.append(m.frame.yaw_to_course(o.setpoint[1]))
        t += .05
    steps = np.abs(np.diff(np.unwrap(yaws)))
    assert steps.max() <= .25 * .05 + 1e-6


def test_pad_then_descend_to_the_floor_then_land_request():
    m, t = flying_mission()
    m.planner.mode = "done"
    m.plan()
    pad_odom = m.frame.to_odom((-2.042, -.792, .9))
    over_pad = lambda t, z: ground(t, armed=True, offboard_active=True, nav_state=14, landed=False,
                                   position=(pad_odom[0], pad_odom[1], z))
    m.tick(over_pad(t, .85))
    assert m.state in ("PAD", "DESCEND")
    zs = []
    for i in range(200):
        z = max(.33, .85 - .3 * .05 * i)
        o = m.tick(over_pad(t, z))
        zs.append(m.frame.to_course(o.setpoint[0])[2])
        t += .05
    assert m.state == "LOW_HOLD" and o.land_request
    assert min(zs) == pytest.approx(.05 + .3 + .03, abs=.01)                 # stops at the floor; pilot lands


def test_disarm_ends_the_flight_and_the_next_flight_starts_fresh():
    m, t = flying_mission()
    m.tick(ground(t, armed=False, landed=True))
    assert m.state == "DONE"
    outs, t = run_until(m, lambda t: ground(t), t, t + 3.5)
    assert any(o.reset_planner for o in outs) and m.state == "WAIT_HEALTH"
    outs, t = run_until(m, lambda t: ground(t), t, t + 2.5)
    assert m.state == "READY"


def test_shadow_never_publishes():
    m = Mission(ScriptedPlanner(), MissionConfig(shadow=True))
    outs, t = run_until(m, lambda t: ground(t), 0.0, 3.0)
    outs += run_until(m, lambda t: ground(t, armed=True, offboard_active=True), t, t + 3)[0]
    assert m.state == "SHADOW" and all(o.setpoint is None for o in outs)
    assert m.planning_allowed()

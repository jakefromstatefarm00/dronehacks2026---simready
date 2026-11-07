"""FlyX mission state machine and PX4 frame helpers, offline (no ROS, no PX4)."""
from types import SimpleNamespace

import numpy as np
import pytest

from flyx.mission import Mission, MissionConfig, Telemetry
from flyx.px4_link import (ARMING_STATE_ARMED, ARMING_STATE_DISARMED, CMD_ARM_DISARM, CMD_DO_SET_MODE,
                           CMD_NAV_LAND, NAV_STATE_OFFBOARD, course_to_ned_setpoint, find_topic, odom_to_course)
from pathplan.online import Command as PlanCommand


class FakePlanner:
    """Moves east 1 m from wherever it starts, then reports done (or failed)."""

    def __init__(self, finish="done", after=5):
        self.finish, self.after, self.calls, self.events = finish, after, 0, [{"event": "fake"}]
        self.observed = 0

    def update(self, xy, yaw):
        self.calls += 1
        if self.calls > self.after:
            return PlanCommand(self.finish)
        return PlanCommand("move", np.array([xy, [xy[0] + 1.0, xy[1]]]), 0.0)

    def observe(self, *cells):
        self.observed += 1


def tel(t, **kw):
    """Healthy, landed, disarmed PX4 on the launch mark (NED origin), heading north."""
    base = dict(now=t, odom_t=t, position_ned=(0.0, 0.0, 0.0), q_ned_frd=(1.0, 0.0, 0.0, 0.0),
                status_t=t, arming_state=ARMING_STATE_DISARMED, nav_state=4, landed_t=t, landed=True,
                local_t=t, xy_valid=True, z_valid=True)
    base.update(kw)
    return Telemetry(**base)


def run_until(m, state, t0, step=.05, limit=20.0, **kw):
    t, outs = t0, []
    while t < t0 + limit:
        outs.append(m.tick(tel(t, **kw)))
        if m.state == state:
            return t, outs
        t += step
    raise AssertionError(f"never reached {state}; stuck in {m.state}")


def test_shadow_never_publishes():
    m = Mission(FakePlanner(), MissionConfig(execute=False))
    t, outs = run_until(m, "SHADOW", 0.0)
    for _ in range(40):
        t += .05
        m.plan()
        outs.append(m.tick(tel(t)))
    assert all(o.setpoint is None and o.command is None for o in outs)
    assert m.frame is not None and m.target is not None


def test_waits_while_estimator_invalid_or_not_landed():
    m = Mission(FakePlanner(), MissionConfig(execute=True))
    for i in range(100):
        out = m.tick(tel(i * .05, xy_valid=False))
        assert out.setpoint is None and out.command is None
    assert m.state == "WAIT_HEALTH"
    for i in range(100):
        m.tick(tel(5 + i * .05, landed=False))
    assert m.state == "WAIT_HEALTH"


def test_full_execute_sequence_takeoff_navigate_land():
    m = Mission(FakePlanner(after=5), MissionConfig(execute=True))
    t, outs = run_until(m, "WAIT_OFFBOARD", 0.0)
    assert outs[-1].command.command == CMD_DO_SET_MODE                 # Offboard requested once
    assert all(o.setpoint is not None for o in outs if o.state == "STREAM")
    assert not any(o.command and o.command.command == CMD_ARM_DISARM for o in outs)   # no arm before Offboard
    t, outs = run_until(m, "WAIT_ARM", t, nav_state=NAV_STATE_OFFBOARD)
    assert outs[-1].command.command == CMD_ARM_DISARM
    t, outs = run_until(m, "TAKEOFF", t, nav_state=NAV_STATE_OFFBOARD, arming_state=ARMING_STATE_ARMED)
    sp = outs[-1].setpoint
    alt_ned = sp[0][2]
    assert alt_ned < 0 and np.isclose(-alt_ned, .9 - .05, atol=1e-6)    # 0.9 m above the floor, launch AGL 0.05
    assert np.allclose(sp[0][:2], [0, 0], atol=1e-6)                    # straight up
    # climb: report the drone at altitude
    t, _ = run_until(m, "NAVIGATE", t, nav_state=NAV_STATE_OFFBOARD, arming_state=ARMING_STATE_ARMED,
                     landed=False, position_ned=(0.0, 0.0, -.85))
    for _ in range(10):
        t += .05
        m.plan()
        out = m.tick(tel(t, nav_state=NAV_STATE_OFFBOARD, arming_state=ARMING_STATE_ARMED, landed=False,
                         position_ned=(0.0, 0.0, -.85)))
        if out.command:
            break
    assert m.state == "LAND" and out.command.command == CMD_NAV_LAND
    t, _ = run_until(m, "DONE", t, nav_state=18, arming_state=ARMING_STATE_DISARMED, landed=True)


def airborne_mission(**cfg):
    m = Mission(FakePlanner(after=1000), MissionConfig(execute=True, **cfg))
    t, _ = run_until(m, "WAIT_OFFBOARD", 0.0)
    t, _ = run_until(m, "TAKEOFF", t, nav_state=NAV_STATE_OFFBOARD, arming_state=ARMING_STATE_ARMED)
    t, _ = run_until(m, "NAVIGATE", t, nav_state=NAV_STATE_OFFBOARD, arming_state=ARMING_STATE_ARMED,
                     landed=False, position_ned=(0.0, 0.0, -.85))
    return m, t


AIR = dict(nav_state=NAV_STATE_OFFBOARD, arming_state=ARMING_STATE_ARMED, landed=False, position_ned=(0.0, 0.0, -.85))


def test_stale_pose_in_flight_lands():
    m, t = airborne_mission()
    out = m.tick(tel(t + 1.0, **{**AIR, "odom_t": t}))                  # pose 1 s old
    assert m.state == "LAND" and out.command.command == CMD_NAV_LAND
    assert m.tick(tel(t + 1.1, **AIR)).command is None                   # land sent once


def test_pose_stamped_just_after_the_control_tick_is_fresh():
    """Regression: a callback thread can stamp a report after the loop read the clock."""
    m, t = airborne_mission()
    m.tick(tel(t + .05, **{**AIR, "odom_t": t + .06, "status_t": t + .07, "local_t": t + .06}))
    assert m.state == "NAVIGATE"


def test_slow_vehicle_status_does_not_land():
    """Regression (FlyX run): vehicle_status arrives at ~1.7 Hz and land_detected at ~0.9 Hz.
    Gaps of 1.2 s between status reports must not trigger the airborne guard."""
    m, t = airborne_mission()
    status_t = landed_t = t
    for i in range(200):                                                # 10 s at 20 Hz
        now = t + .05 * (i + 1)
        if now - status_t >= 1.2:
            status_t = now
        if now - landed_t >= 2.0:
            landed_t = now
        m.tick(tel(now, **{**AIR, "status_t": status_t, "landed_t": landed_t}))
    assert m.state == "NAVIGATE", m.events


def test_land_note_names_the_failed_check():
    m, t = airborne_mission()
    m.tick(tel(t + 5.0, **{**AIR, "status_t": t}))                      # status silent for 5 s
    assert m.state == "LAND" and "vehicle_status stale" in m.events[-1]["note"]


def test_clock_jump_does_not_land_and_flight_limit_uses_sim_time():
    """Freshness is on the monotonic clock (tel.now); a 1.2 s sim-time step changes nothing,
    and the flight limit counts sim time (FlyX runs slower than real time)."""
    m, t = airborne_mission(max_flight_s=10.0)
    for i in range(1, 140):                                            # 7 s monotonic
        now = t + .05 * i
        sim = .9 * .05 * i + (1.2 if i > 60 else 0.0)                  # slow sim + one jump
        m.tick(tel(now, sim_now=1000 + sim, **AIR))
    assert m.state == "NAVIGATE", m.events
    m._sim_takeoff = 1000.0
    m.tick(tel(now + .05, sim_now=1011.0, **AIR))
    assert m.state == "LAND" and m.events[-1]["note"] == "flight time limit"


def test_local_position_gap_with_fresh_odometry_does_not_land():
    """Regression (FlyX run 3): local_position paused 0.69 s during takeoff, odometry fresh."""
    m, t = airborne_mission()
    m.tick(tel(t + 1.0, **{**AIR, "local_t": t}))                       # 1 s gap: keep flying
    assert m.state == "NAVIGATE", m.events
    m.tick(tel(t + 3.0, **{**AIR, "local_t": t}))                       # 3 s: estimator flags too old
    assert m.state == "LAND" and "local_position stale" in m.events[-1]["note"]


def test_estimator_invalid_in_flight_lands():
    m, t = airborne_mission()
    m.tick(tel(t + .05, **{**AIR, "xy_valid": False}))
    assert m.state == "LAND"


def test_flight_time_limit_lands():
    m, t = airborne_mission(max_flight_s=2.0)
    m.tick(tel(t + 5.0, **AIR))
    assert m.state == "LAND"


def test_leaving_offboard_hands_vehicle_back():
    m, t = airborne_mission()
    outs = [m.tick(tel(t + .05 * i, **{**AIR, "nav_state": 2})) for i in range(1, 30)]
    assert m.state == "TAKEN_OVER"
    after = outs[[o.state for o in outs].index("TAKEN_OVER"):]
    assert all(o.setpoint is None and o.command is None for o in after)


def test_offboard_refused_fails_without_arming():
    m = Mission(FakePlanner(), MissionConfig(execute=True))
    t, _ = run_until(m, "WAIT_OFFBOARD", 0.0)
    _, outs = run_until(m, "FAILED", t)                                   # nav_state never OFFBOARD
    assert not any(o.command and o.command.command == CMD_ARM_DISARM for o in outs)


def test_planner_failure_lands():
    m = Mission(FakePlanner(finish="failed", after=1), MissionConfig(execute=True))
    t, _ = run_until(m, "WAIT_OFFBOARD", 0.0)
    t, _ = run_until(m, "TAKEOFF", t, nav_state=NAV_STATE_OFFBOARD, arming_state=ARMING_STATE_ARMED)
    t, _ = run_until(m, "NAVIGATE", t, **AIR)
    m.plan()
    m.plan()
    m.tick(tel(t + .05, **AIR))
    assert m.state == "LAND"


# ---------------------------------------------------------------- frames / topics
@pytest.fixture(params=[0.0, 1.0, -2.5])
def frame(request):
    from flyx.px4_link import CourseFrame
    return CourseFrame.from_launch(position_ned=[.3, -.2, -.05], heading_ned=request.param, launch_xy=[-2.515, .914],
                                   body_agl=.05, physical_heading_in_course=np.pi / 2, placement_reviewed=True)


@pytest.mark.parametrize("xy, yaw", [((0.0, 0.0), 0.0), ((1.2, -2.0), 2.0), ((-2.0, 2.5), -1.0)])
def test_setpoint_round_trip(frame, xy, yaw):
    pos, yaw_ned = course_to_ned_setpoint(frame, xy, .9, yaw)
    q = (np.cos(yaw_ned / 2), 0.0, 0.0, np.sin(yaw_ned / 2))
    xyz, yaw_back, rot = odom_to_course(frame, pos, q)
    assert np.allclose(xyz, [*xy, .9]) and np.isclose(np.cos(yaw_back - yaw), 1.0)
    assert np.allclose(rot[:2, 0], [np.cos(yaw), np.sin(yaw)]) and np.allclose(rot[:, 2], [0, 0, 1])


def test_find_topic_suffix_and_ambiguity():
    g = [("/fmu/out/vehicle_status_v1", ["px4_msgs/msg/VehicleStatus"]),
         ("/fmu/in/vehicle_command", ["px4_msgs/msg/VehicleCommand"]),
         ("/fmu/in/vehicle_command_mode_executor", ["px4_msgs/msg/VehicleCommand"])]
    assert find_topic(g, "vehicle_status", "px4_msgs/msg/VehicleStatus") == "/fmu/out/vehicle_status_v1"
    assert find_topic(g, "vehicle_command", "px4_msgs/msg/VehicleCommand", "in") == "/fmu/in/vehicle_command"
    g.append(("/fmu/out/vehicle_status", ["px4_msgs/msg/VehicleStatus"]))
    assert find_topic(g, "vehicle_status", "px4_msgs/msg/VehicleStatus") is None

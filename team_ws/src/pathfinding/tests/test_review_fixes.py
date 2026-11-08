"""Mission fixes from the second review (numbers = the reviewer's findings). No ROS needed."""
import pytest

from dronehacks_pathplan.mission import Mission, MissionConfig
from test_mission import ScriptedPlanner, airborne, flying_mission, ground, ready_mission, run_until


# ---------------------------------------------------------------- 1. px4_link gaps in the air
def test_px4_link_gap_holds_the_last_setpoint_and_faults_only_after_8_s():
    m, t = flying_mission()
    m.plan()
    last = m.tick(airborne(t, .85)).setpoint
    outs, t = run_until(m, lambda t: airborne(t, .85, px4_link=False), t + .05, t + 3.3)   # a lab topic pause
    assert m.state == "NAVIGATE" and not any(o.hold_request for o in outs)
    assert all(o.setpoint == last for o in outs) and "PX4 link down" in outs[-1].note
    outs, t = run_until(m, lambda t: airborne(t, .85), t, t + .5)                          # link back
    assert m.state == "NAVIGATE" and "planner" in outs[-1].note
    outs, t = run_until(m, lambda t: airborne(t, .85, px4_link=False), t, t + 7.9)
    assert m.state == "NAVIGATE"
    outs, t = run_until(m, lambda t: airborne(t, .85, px4_link=False), t, t + .5)
    assert m.state == "FAULT" and sum(o.hold_request for o in outs) == 1 and outs[-1].setpoint is None


# ---------------------------------------------------------------- 2. PX4 failsafe Hold blips
def test_px4_failsafe_hold_is_ridden_out_and_offboard_resumes():
    m, t = flying_mission()
    m.plan()
    last = m.tick(airborne(t, .85)).setpoint
    blip = lambda t: airborne(t, .85, offboard_active=False, nav_state=4)
    outs, t = run_until(m, blip, t + .05, t + 2.0)
    assert m.state == "NAVIGATE" and all(o.setpoint == last for o in outs)
    assert "failsafe Hold" in outs[-1].note and m.px4_failsafe_waits > 0
    outs, t = run_until(m, lambda t: airborne(t, .85), t, t + .5)                          # PX4 back in Offboard
    assert m.state == "NAVIGATE" and outs[-1].setpoint is not None


def test_failsafe_hold_that_does_not_clear_is_taken_over_after_5_s():
    m, t = flying_mission()
    outs, t = run_until(m, lambda t: airborne(t, .85, offboard_active=False, nav_state=4), t, t + 4.9)
    assert m.state == "NAVIGATE"
    outs, t = run_until(m, lambda t: airborne(t, .85, offboard_active=False, nav_state=4), t, t + .3)
    assert m.state == "TAKEN_OVER" and outs[-1].setpoint is None and "failsafe Hold" in m.result


@pytest.mark.parametrize("pilot_mode", [2, 1, 0])          # POSCTL, ALTCTL, MANUAL: the pilot
def test_pilot_mode_after_a_failsafe_hold_is_never_fought(pilot_mode):
    m, t = flying_mission()
    outs, t = run_until(m, lambda t: airborne(t, .85, offboard_active=False, nav_state=4), t, t + 1.0)
    outs, t = run_until(m, lambda t: airborne(t, .85, offboard_active=False, nav_state=pilot_mode), t, t + .1)
    assert m.state == "TAKEN_OVER" and outs[-1].setpoint is None


# ---------------------------------------------------------------- 3. READY through odometry pauses
def test_reviewer_scenario_3_short_pause_before_arming_still_flies():
    m, t = ready_mission()
    stale = t
    outs, t = run_until(m, lambda t: ground(t, odom_t=stale), t, t + .4)   # 0.4 s pause
    assert m.state == "READY" and all(o.setpoint is not None for o in outs)
    outs, t = run_until(m, lambda t: ground(t, odom_t=stale, armed=True), t, t + .1)
    assert m.state == "ARMED"
    outs, t = run_until(m, lambda t: airborne(t, 0.0), t, t + .5)
    assert m.state == "TAKEOFF" and outs[-1].setpoint is not None


def test_ready_leaves_only_after_1_5_s_of_stale_odometry():
    m, t = ready_mission()
    stale = t
    outs, t = run_until(m, lambda t: ground(t, odom_t=stale), t, stale + 1.45)
    assert m.state == "READY" and all(o.setpoint is not None for o in outs)
    outs, t = run_until(m, lambda t: ground(t, odom_t=stale), t, stale + 1.7)
    assert m.state == "WAIT_HEALTH" and outs[-1].setpoint is None


def test_arming_just_after_leaving_ready_requests_hold_once_and_tells_the_pilot():
    m, t = ready_mission()
    stale = t
    run_until(m, lambda t: ground(t, odom_t=stale), t, stale + 1.6)                    # -> WAIT_HEALTH
    assert m.state == "WAIT_HEALTH"
    outs, t = run_until(m, lambda t: ground(t, odom_t=stale, armed=True), stale + 1.6, stale + 3.0)
    assert sum(o.hold_request for o in outs) == 1 and all(o.setpoint is None for o in outs)
    assert "PILOT: DISARM" in outs[-1].note and "/skybeta/hold" in outs[-1].note
    # Offboard granted anyway (state raced the hold): the note says take over and land
    o = m.tick(ground(t, odom_t=stale, armed=True, offboard_active=True, nav_state=14))
    assert "TAKE OVER AND LAND" in o.note and o.setpoint is None and not o.hold_request


def test_arming_long_after_our_last_setpoint_needs_no_hold():
    m = Mission(ScriptedPlanner())
    outs, _ = run_until(m, lambda t: ground(t, odom_t=None, armed=True), 0.0, 2.0)   # never streamed
    assert not any(o.hold_request for o in outs) and "not flying this arming" in outs[-1].note


# ---------------------------------------------------------------- 5. DESCEND / run limit
def test_descend_reaches_low_hold_when_odometry_z_disagrees_with_px4_height():
    m, t = flying_mission()
    m.planner.mode = "done"
    m.plan()
    pad = m.frame.to_odom((-2.042, -.792, .9))
    outs = []
    for _ in range(400):                                       # the vehicle settles 9 cm above the setpoint
        sp = (outs[-1].setpoint[0] if outs and outs[-1].setpoint else pad)
        outs.append(m.tick(airborne(t, 0.0, position=(pad[0], pad[1], sp[2] + .09))))
        t += .05
        if m.state == "LOW_HOLD":
            break
    assert m.state == "LOW_HOLD" and outs[-1].land_request
    descend_s = (.9 - .38) / .3                                # ramp from cruise to the floor setpoint
    assert t - (m.events[-2]["t"]) < descend_s + 3.0 + .5


def test_run_limit_applies_in_takeoff():
    m, t = ready_mission(MissionConfig(max_flight_s=20.0))
    m.tick(ground(t, armed=True))
    outs, t = run_until(m, lambda t: airborne(t, 0.3), t + .05, t + 19.0)   # never gets near altitude
    assert m.state == "TAKEOFF"
    outs, t = run_until(m, lambda t: airborne(t, 0.3), t, t + 1.5)
    assert m.state == "LOW_HOLD" and outs[-1].land_request and "run limit" in m.result


def test_run_limit_applies_in_descend():
    m, t = flying_mission()
    m.cfg.max_flight_s = m.flight_time(airborne(t, .85)) + 1.0
    m._start_descend(airborne(t, .85), None, "planner failed: test")
    outs, t = run_until(m, lambda t: airborne(t, .85), t, t + 1.5)          # stuck high: no floor
    assert m.state == "LOW_HOLD" and outs[-1].land_request and "during the descent" in m.result


# ---------------------------------------------------------------- 8. ground checks, the hold after a FAULT
def test_ready_refused_when_odometry_z_is_off_the_interface_origin():
    m = Mission(ScriptedPlanner())
    outs, _ = run_until(m, lambda t: ground(t, position=(0.0, 0.0, .25), origin_z=0.0), 0.0, 4.0)
    assert m.state == "WAIT_HEALTH" and "origin" in outs[-1].note
    m2 = Mission(ScriptedPlanner())
    run_until(m2, lambda t: ground(t, position=(0.0, 0.0, .06), origin_z=0.0), 0.0, 2.5)  # within 0.1 m
    assert m2.state == "READY"


def test_our_own_hold_is_cleared_by_the_next_ready_but_a_foreign_one_blocks():
    m, t = flying_mission()
    stale = t
    outs, t = run_until(m, lambda t: airborne(t, .85, odom_t=stale), t, stale + 3.5)     # FAULT + hold
    assert m.state == "FAULT" and sum(o.hold_request for o in outs) == 1
    outs, t = run_until(m, lambda t: ground(t, frozen=True), t, t + 8.0)                # landed, disarmed
    assert m.state == "READY" and outs[-1].setpoint is not None                         # clears the hold
    assert "clears the /skybeta/hold" in outs[-1].note
    outs, t = run_until(m, lambda t: ground(t), t, t + 1.5)                             # interface: unfrozen
    assert m._own_hold_t is None
    other = Mission(ScriptedPlanner())                                                  # someone else's hold
    outs, _ = run_until(other, lambda t: ground(t, frozen=True), 0.0, 4.0)
    assert other.state == "WAIT_HEALTH" and "not ours" in outs[-1].note

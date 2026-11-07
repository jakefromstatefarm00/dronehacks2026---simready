"""The mission state machine, flown closed-loop in the simulator (no ROS needed)."""
import math
import os

import numpy as np

from team_mission.control import CarrotPolicy
from team_mission.geometry import CourseFrame
from team_mission.mission import (ABORTED, ARMED_WAIT, DORMANT, FLY, HOLD, TAKEOFF, Inputs, Mission,
                                  MissionParams)
from team_mission.path import CheckpointSpec
from team_mission.rl.mission_sim import run_once

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CP = os.path.join(PKG, 'config', 'checkpoints')


def spec(name='test_line.yaml'):
    return CheckpointSpec.load(os.path.join(CP, name))


def test_full_mission_line_nominal():
    r = run_once(spec(), CarrotPolicy(), CourseFrame(), seed=0, randomize=False)
    assert r['result'] == 'landed and disarmed', r
    assert r['landing_error_m'] < 0.15
    phases = [m[2] for m in r['messages']]
    order = ['DORMANT -> ARMED_WAIT', 'ARMED_WAIT -> TAKEOFF', 'TAKEOFF -> FLY', 'FLY -> DESCEND',
             'DESCEND -> TOUCHDOWN', 'TOUCHDOWN -> DONE']
    idx = [next(k for k, p in enumerate(phases) if p.startswith(o)) for o in order]
    assert idx == sorted(idx)


def test_takeoff_reaches_one_metre_slowly():
    r = run_once(spec(), CarrotPolicy(), CourseFrame(), seed=0, randomize=False)
    tr = r['truth']
    msgs = {m[2].split(':')[0]: m[0] for m in r['messages']}
    t_fly = msgs['TAKEOFF -> FLY']
    z = tr[np.searchsorted(tr[:, 0], t_fly), 3]
    assert 0.9 < z < 1.1
    vz = np.diff(tr[:, 3]) / np.diff(tr[:, 0])
    assert vz.max() < 0.6                                     # slow climb


def test_full_mission_square_randomized_mostly_succeeds():
    ok = sum(run_once(spec('test_square.yaml'), CarrotPolicy(), CourseFrame(), seed=s)['result'] == 'landed and disarmed'
             for s in range(10))
    assert ok >= 9


def test_dormant_publishes_nothing_until_armed():
    m = Mission(spec(), CarrotPolicy())
    out = m.step(Inputs(now=0.0, armed=False, pos=np.zeros(3), yaw=0.0, odom_age=0.0, vo_state=1))
    assert m.phase == DORMANT and out.kind == 'none'
    out = m.step(Inputs(now=0.1, armed=True, nav_state_name='ALTCTL', pos=np.zeros(3), yaw=0.0, odom_age=0.0, vo_state=1))
    out = m.step(Inputs(now=0.15, armed=True, pos=np.zeros(3), yaw=0.0, odom_age=0.0, vo_state=1))
    assert m.phase == ARMED_WAIT and out.kind == 'pose'      # keeps flight_interface's stream alive


def _airborne(m):
    s = dict(armed=True, offboard_active=True, landed=False, pos=np.zeros(3), yaw=0.0, odom_age=0.0, vo_state=1)
    m.step(Inputs(now=0.0, **s))
    m.step(Inputs(now=0.05, **s))
    assert m.phase == TAKEOFF
    return s


def test_tracking_loss_holds_then_resumes():
    m = Mission(spec(), CarrotPolicy(), params=MissionParams(hold_resume_s=1.0))
    s = _airborne(m)
    out = m.step(Inputs(now=0.1, **dict(s, vo_state=2)))
    assert m.phase == HOLD and out.kind == 'pose'
    for k in range(30):
        m.step(Inputs(now=0.15 + 0.05 * k, **s))
    assert m.phase == TAKEOFF


def test_pilot_takeover_aborts_and_goes_quiet():
    m = Mission(spec(), CarrotPolicy())
    s = _airborne(m)
    out = m.step(Inputs(now=0.1, **dict(s, offboard_active=False)))
    assert m.phase == ABORTED and out.kind == 'none'
    out = m.step(Inputs(now=0.2, **dict(s, armed=False, offboard_active=False)))
    assert m.phase == DORMANT


def test_bad_policy_output_holds():
    class Broken:
        def act(self, obs, ctx):
            return np.array([np.nan, 0, 0, 0])
    m = Mission(spec(), Broken(), params=MissionParams(settle_s=0.0))
    s = _airborne(m)
    m.phase = FLY
    m.fly_started = 0.0
    m.i = 1
    out = m.step(Inputs(now=0.2, **dict(s, pos=np.array([0, 0, 1.0]))))
    assert m.phase == HOLD and out.kind == 'pose'


def test_no_odometry_never_climbs_blind():
    m = Mission(spec(), CarrotPolicy())
    s = dict(armed=True, offboard_active=True, landed=True, pos=np.zeros(3), yaw=0.0, odom_age=5.0, vo_state=1)
    m.step(Inputs(now=0.0, **s))
    out = m.step(Inputs(now=0.05, **s))
    assert m.phase == ARMED_WAIT and (out.kind == 'none' or out.position[2] <= 0.01)


def test_course_mission_nominal_with_launch_pose():
    r = run_once(spec('course_v3.yaml'), CarrotPolicy(), CourseFrame(-2.515, 0.914, 0.0, math.radians(90)),
                 seed=0, randomize=False)
    assert r['result'] == 'landed and disarmed', r['result']
    assert r['landing_error_m'] < 0.15

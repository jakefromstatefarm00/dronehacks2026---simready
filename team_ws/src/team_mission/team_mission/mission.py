"""The mission state machine, with no ROS in it (so every rule is unit-tested on a laptop).

Phases, in order:

    DORMANT      publish nothing. Wait until /skybeta/state says armed.
    ARMED_WAIT   the pilot armed. Stream "stay where you are" pose setpoints: flight_interface
                 needs a live team setpoint before it will ask PX4 for Offboard. Wait for
                 offboard_active (the pilot switched to Position and the checks passed).
    TAKEOFF      remember the start pose. Climb straight up to start + takeoff_height (1 m),
                 the target rising at climb_rate (slow), then hover until settled.
    FLY          the policy flies checkpoint to checkpoint (pose "carrot" setpoints).
    HOLD         something is wrong (tracking lost, too far off the path, policy output bad):
                 hold position; resume when healthy again; give up after hold_timeout_s.
    DESCEND      at the last checkpoint: move the hover target down to hover_height above the
                 start ground (0.3 m, flight_interface's floor), slowly, and settle there.
    TOUCHDOWN    send a slow downward velocity (twist) until PX4 detects the landing and
                 disarms itself (COM_DISARM_LAND). flight_interface has no land command and we
                 must not publish to /fmu/in/*, so this IS the proper way to land here.
    DONE         disarmed. Back to DORMANT for the next flight.
    ABORTED      the pilot took over (Offboard lost) or we gave up: publish nothing, wait for
                 disarm. Never fight the pilot.

Anywhere: disarm -> DORMANT.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from team_mission.control import ActionMapper, Limits, PolicyContext, build_observation, speed_scale
from team_mission.geometry import CourseFrame
from team_mission.path import CheckpointSpec, Path, resolve_path

DORMANT, ARMED_WAIT, TAKEOFF, FLY, HOLD, DESCEND, TOUCHDOWN, DONE, ABORTED = (
    'DORMANT', 'ARMED_WAIT', 'TAKEOFF', 'FLY', 'HOLD', 'DESCEND', 'TOUCHDOWN', 'DONE', 'ABORTED')


@dataclass
class MissionParams:
    takeoff_height: float = 1.0        # m above the start, measured by cuVSLAM z
    climb_rate: float = 0.3            # m/s, how fast the takeoff target rises
    takeoff_tolerance: float = 0.08    # m
    settle_s: float = 1.5              # hover this long before moving on (takeoff, descend)
    hover_height: float = 0.3          # m above the start ground before touchdown (>= the 0.3 floor)
    descent_rate: float = 0.2          # m/s, DESCEND target speed
    touchdown_speed: float = 0.2       # m/s, TOUCHDOWN twist
    disarm_timeout_s: float = 8.0      # after landed, warn if PX4 has not disarmed
    odom_timeout_s: float = 0.3        # older odometry = unhealthy
    require_vslam_status: bool = True  # need /visual_slam/status vo_state == 1
    hold_resume_s: float = 2.0         # healthy this long before leaving HOLD
    hold_timeout_s: float = 20.0       # HOLD longer than this -> ABORTED (pilot lands)
    corridor_margin: float = 0.05      # hold when off-path by more than corridor + margin
    max_holds: int = 4                 # more corridor holds than this -> ABORTED
    fly_timeout_s: float = 300.0       # the whole FLY phase


@dataclass
class Inputs:
    now: float                          # seconds (ROS clock)
    armed: bool = False
    offboard_active: bool = False
    landed: Optional[bool] = None
    nav_state_name: str = ''
    pos: Optional[np.ndarray] = None    # odom ENU, metres (cuVSLAM)
    vel: Optional[np.ndarray] = None    # odom ENU world velocity
    yaw: Optional[float] = None
    odom_age: float = 1e9               # seconds since the last odometry message
    vo_state: Optional[int] = None      # 1 = tracking (None = not received)
    extra_ok: Optional[bool] = None     # optional external health (e.g. /team/localization/ok)


@dataclass
class Output:
    kind: str = 'none'                  # 'none' | 'pose' | 'twist'
    position: Optional[np.ndarray] = None
    yaw: Optional[float] = None         # None = do not command yaw
    velocity: Optional[np.ndarray] = None
    yaw_rate: float = 0.0
    log: dict = field(default_factory=dict)


class Mission:
    def __init__(self, spec: CheckpointSpec, policy, limits: Limits = None,
                 params: MissionParams = None, course_frame: CourseFrame = None, logger=None):
        self.spec = spec
        self.policy = policy
        self.limits = limits or Limits()
        self.mapper = ActionMapper(self.limits)
        self.p = params or MissionParams()
        self.course_frame = course_frame or CourseFrame()
        self.log = logger or (lambda level, msg: None)
        self._reset()

    def _reset(self):
        self.phase = DORMANT
        self.phase_since = 0.0
        self.path: Optional[Path] = None
        self.i = 1
        self.start = None
        self.start_yaw = 0.0
        self.z_cmd = 0.0
        self.settled_since = None
        self.prev_action = np.zeros(4)
        self.hold_pos = None
        self.hold_yaw = None
        self.hold_reason = ''
        self.healthy_since = None
        self.holds = 0
        self.landed_since = None
        self.disarm_warned = False
        self.fly_started = None
        self.last_good = None            # (pos, yaw) from the last healthy tick
        self.resume_phase = FLY
        self.hold_kind = ''              # 'health' | 'corridor' | 'policy'
        self.last_now = None
        self.events = []

    def _goto(self, phase, now, why=''):
        if phase != self.phase:
            self.log('warn' if phase in (HOLD, ABORTED) else 'info',
                     f'{self.phase} -> {phase}' + (f': {why}' if why else ''))
            self.events.append((now, self.phase, phase, why))
            self.phase, self.phase_since = phase, now

    # ------------------------------------------------------------------ health
    def healthy(self, s: Inputs):
        if s.pos is None or s.yaw is None:
            return False, 'no odometry'
        if s.odom_age > self.p.odom_timeout_s:
            return False, f'odometry {s.odom_age:.2f} s old'
        if not (np.all(np.isfinite(s.pos)) and math.isfinite(s.yaw)):
            return False, 'odometry not finite'
        if self.p.require_vslam_status and s.vo_state != 1:
            return False, f'cuVSLAM not tracking (vo_state={s.vo_state})'
        if s.extra_ok is False:
            return False, 'external health check says not OK'
        return True, ''

    # ------------------------------------------------------------------ main step
    def step(self, s: Inputs) -> Output:
        now = s.now
        self.dt = 0.05 if self.last_now is None else float(np.clip(now - self.last_now, 0.0, 0.2))
        self.last_now = now
        if not s.armed and self.phase not in (DORMANT,):
            if self.phase == TOUCHDOWN:
                self._goto(DONE, now, 'disarmed after landing')
            self.log('info', f'disarmed in {self.phase} -> DORMANT')
            self._reset()
            return Output()
        ok, why = self.healthy(s)
        if ok:
            self.last_good = (np.asarray(s.pos, float).copy(), float(s.yaw))

        # The pilot took the vehicle back: never fight them.
        if self.phase in (TAKEOFF, FLY, HOLD, DESCEND) and not s.offboard_active:
            self._goto(ABORTED, now, 'Offboard lost - the pilot has control')
        if self.phase == TOUCHDOWN and not s.offboard_active and not s.landed:
            self._goto(ABORTED, now, 'Offboard lost during touchdown - the pilot has control')

        handler = getattr(self, f'_{self.phase.lower()}')
        out = handler(s, ok, why)
        out.log.update(phase=self.phase, cp=self.i, healthy=ok, why=why)
        return out

    # ------------------------------------------------------------------ phases
    def _dormant(self, s, ok, why):
        if s.armed:
            note = '' if s.nav_state_name in ('ALTCTL', '') else (
                f' (armed in {s.nav_state_name}; the procedure is arm in Altitude, then switch to Position)')
            self._goto(ARMED_WAIT, s.now, 'armed' + note)
        return Output()

    def _armed_wait(self, s, ok, why):
        if s.pos is None:
            return Output()
        if s.offboard_active:
            if not ok:
                # Offboard started but we cannot see: hold here, never climb blind
                return self._hold_here(s, f'Offboard started but {why}')
            self.start = np.asarray(s.pos, float).copy()
            self.start_yaw = float(s.yaw)
            cf = self.course_frame
            self.path = resolve_path(self.spec, self.start, self.start_yaw, self.p.takeoff_height, cf)
            self.z_cmd = float(s.pos[2])
            self.settled_since = None
            self._goto(TAKEOFF, s.now, f'start at ({self.start[0]:.2f}, {self.start[1]:.2f}, {self.start[2]:.2f}), '
                                       f'{self.path.n} checkpoints, {self.path.length:.1f} m')
            return self._takeoff(s, ok, why)
        # keep flight_interface's stream alive with "stay where you are"
        return Output('pose', np.asarray(s.pos, float).copy(), None)

    def _takeoff(self, s, ok, why):
        if not ok:
            return self._enter_hold(s, why)
        target = self.start[2] + self.p.takeoff_height
        self.z_cmd = min(target, self.z_cmd + self.p.climb_rate * self.dt)
        if self.z_cmd >= target and abs(s.pos[2] - target) < self.p.takeoff_tolerance:
            self.settled_since = self.settled_since or s.now
            if s.now - self.settled_since >= self.p.settle_s:
                self.i = 1
                self.prev_action = np.zeros(4)
                self.fly_started = s.now
                self._goto(FLY, s.now, f'at {s.pos[2] - self.start[2]:.2f} m, heading for {self.path.names[1]}')
        else:
            self.settled_since = None
        return Output('pose', np.array([self.start[0], self.start[1], self.z_cmd]), self.start_yaw,
                      log={'target_z': target})

    def _fly(self, s, ok, why):
        if not ok:
            return self._enter_hold(s, why)
        P = self.path
        pos, yaw = np.asarray(s.pos, float), float(s.yaw)
        vel = np.zeros(3) if s.vel is None else np.asarray(s.vel, float)
        while self.i < P.n and P.reached(pos, self.i):
            self.log('info', f'reached {P.names[self.i]} ({self.i}/{P.n})')
            self.i += 1
        if self.i == P.n and P.reached(pos, P.n) and np.linalg.norm(vel) < 0.15:
            self.log('info', f'reached {P.names[P.n]} ({P.n}/{P.n}), the landing spot')
            self.settled_since = None
            self.z_cmd = float(pos[2])
            self._goto(DESCEND, s.now, 'end of the path')
            return self._descend(s, ok, why)
        off, corridor = P.off_path(pos, self.i)
        if off > corridor + self.p.corridor_margin:
            self.holds += 1
            return self._enter_hold(s, f'{off:.2f} m off the path (corridor {corridor:.2f} m)', kind='corridor')
        if s.now - self.fly_started > self.p.fly_timeout_s:
            self._goto(ABORTED, s.now, 'FLY took too long')
            return self._hold_here(s, 'timeout')
        obs = build_observation(pos, vel, yaw, P, self.i, self.prev_action, self.limits)
        try:
            a = np.asarray(self.policy.act(obs, PolicyContext(pos, vel, yaw, P, self.i, self.limits)), float)
        except Exception as e:  # a broken policy must not crash the drone
            return self._enter_hold(s, f'policy error: {e}', kind='policy')
        if a.shape != (4,) or not np.all(np.isfinite(a)):
            return self._enter_hold(s, 'policy output not finite', kind='policy')
        a = np.clip(a, -1.0, 1.0)
        cmd = self.mapper.command(a, pos, yaw, speed_scale(P, self.i))
        self.prev_action = a
        return Output('pose', cmd.carrot, cmd.yaw_target,
                      log={'obs': obs, 'action': a, 'v_cmd': cmd.velocity, 'yaw_rate_cmd': cmd.yaw_rate,
                           'off_path': off, 'corridor': corridor})

    def _hold(self, s, ok, why):
        out = Output('pose', self.hold_pos, self.hold_yaw, log={'hold_reason': self.hold_reason})
        if s.now - self.phase_since > self.p.hold_timeout_s:
            self._goto(ABORTED, s.now, f'held {self.p.hold_timeout_s:.0f} s ({self.hold_reason}); pilot please land')
            return out
        if not ok:
            self.healthy_since = None
            return out
        self.healthy_since = self.healthy_since or s.now
        if s.now - self.healthy_since < self.p.hold_resume_s:
            return out
        if self.holds > self.p.max_holds:
            self._goto(ABORTED, s.now, f'{self.holds} corridor holds; pilot please land')
            return out
        if self.hold_kind == 'corridor' and self.path is not None:
            off, corridor = self.path.off_path(np.asarray(s.pos, float), self.i)
            if off > 0.5 * corridor:          # still moving back to the line
                return out
        if self.hold_kind == 'policy':
            self._goto(ABORTED, s.now, 'the policy failed; pilot please land')
            return out
        resume = self.resume_phase
        self.prev_action = np.zeros(4)
        self._goto(resume, s.now, 'healthy again')
        if resume == TAKEOFF:
            self.z_cmd = float(s.pos[2])
        return out

    def _descend(self, s, ok, why):
        if not ok:
            return self._enter_hold(s, why)
        final = self.path.points[-1]
        target = self.start[2] + self.p.hover_height
        self.z_cmd = max(target, self.z_cmd - self.p.descent_rate * self.dt)
        if self.z_cmd <= target and abs(s.pos[2] - target) < self.p.takeoff_tolerance:
            self.settled_since = self.settled_since or s.now
            if s.now - self.settled_since >= self.p.settle_s:
                self._goto(TOUCHDOWN, s.now, f'hovering {s.pos[2] - self.start[2]:.2f} m up; touching down')
        else:
            self.settled_since = None
        return Output('pose', np.array([final[0], final[1], self.z_cmd]), None, log={'target_z': target})

    def _touchdown(self, s, ok, why):
        # No health gate: if tracking drops this close to the ground, going down is the safe way.
        if s.landed:
            self.landed_since = self.landed_since or s.now
            if s.now - self.landed_since > self.p.disarm_timeout_s and not self.disarm_warned:
                self.disarm_warned = True
                self.log('error', f'landed {self.p.disarm_timeout_s:.0f} s ago but PX4 has not disarmed: '
                                  'check COM_DISARM_LAND on the FC; pilot please disarm')
        return Output('twist', velocity=np.array([0.0, 0.0, -self.p.touchdown_speed]), yaw_rate=0.0)

    def _done(self, s, ok, why):
        return Output()

    def _aborted(self, s, ok, why):
        return Output()

    # ------------------------------------------------------------------ holds
    def _hold_here(self, s, reason):
        pos = s.pos if (s.pos is not None and np.all(np.isfinite(s.pos))) else (
            self.last_good[0] if self.last_good else None)
        if pos is None:
            return Output()
        return Output('pose', np.asarray(pos, float).copy(), None, log={'hold_reason': reason})

    def _enter_hold(self, s, reason, kind='health'):
        self.resume_phase = self.phase if self.phase in (TAKEOFF, FLY, DESCEND) else FLY
        if self.last_good is not None:
            self.hold_pos, self.hold_yaw = self.last_good[0].copy(), self.last_good[1]
        elif s.pos is not None:
            self.hold_pos, self.hold_yaw = np.asarray(s.pos, float).copy(), s.yaw
        if kind == 'corridor' and self.path is not None and s.pos is not None:
            # hold ON the path: the closest point of the current segment, at most max_carrot away
            c, _ = self.path.closest(np.asarray(s.pos, float), self.i)
            d = c - np.asarray(s.pos, float)
            n = np.linalg.norm(d)
            if n > self.limits.max_carrot_m:
                d *= self.limits.max_carrot_m / n
            self.hold_pos = np.asarray(s.pos, float) + d
        self.hold_kind = kind
        self.hold_reason = reason
        self.healthy_since = None
        self._goto(HOLD, s.now, reason)
        return Output('pose', self.hold_pos, self.hold_yaw, log={'hold_reason': reason})

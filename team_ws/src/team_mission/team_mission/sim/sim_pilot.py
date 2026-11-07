#!/usr/bin/env python3
"""SIMULATION ONLY. Plays the safety pilot: arm, then hand over, exactly like the real procedure.

On the drone a human does this on the RC transmitter. In Gazebo or the flyx lab nobody holds a
transmitter, so this node does it with PX4 vehicle commands (which is why it refuses to run on
the drone, and why it is not part of the mission node).

Two ways to hand over:

  mode:=handshake  (default, the real procedure)  stream a "sticks centred, throttle low" manual
                   control input, switch PX4 to Altitude, arm, wait 2 s, switch to Position.
                   flight_interface sees "armed, then switched to Position" and requests Offboard.
                   Needs PX4 to accept /fmu/in/manual_control_input. If PX4 refuses Altitude
                   ("no manual control"), use QGroundControl's virtual joystick as the pilot, or:
  mode:=direct     switch PX4 to Hold, arm, then call flight_interface's /skybeta/request_offboard
                   service (launch flight_interface with allow_mode_request:=true). Skips the
                   Altitude -> Position handshake but tests everything else.

It waits for /skybeta/state to show a PX4 link and a valid position, and (optionally) for the
team_mission node to be running. After the flight it exits when PX4 disarms. If nothing has
disarmed after timeout_s, it commands LAND, like a pilot taking over.

    ros2 run team_mission sim_pilot --ros-args -p use_sim_time:=true -p mode:=handshake
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from team_mission.sim import refuse_on_drone

CMD_DO_SET_MODE, CMD_ARM, CMD_LAND = 176, 400, 21
MAIN_ALTCTL, MAIN_POSCTL, MAIN_AUTO, SUB_LOITER = 2, 3, 4, 3


class SimPilot(Node):
    def __init__(self):
        super().__init__('sim_pilot')
        # status_topic / land_states: missions that hold over the pad and wait for the pilot to land
        # (suraj_course, jolie_course: /suraj_course/status, /jolie_course/status, state HOLD / LAND).
        # team_mission lands itself, so the defaults keep the original behaviour.
        for k, v in dict(mode='handshake', wait_s=5.0, timeout_s=240.0, allow_on_drone=False,
                         require_mission=True, status_topic='/team_mission/status',
                         land_states='HOLD,LAND').items():
            self.declare_parameter(k, v)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        refuse_on_drone('sim_pilot', bool(g('allow_on_drone')))
        from px4_msgs.msg import ManualControlSetpoint, VehicleCommand
        self.VC, self.MCS = VehicleCommand, ManualControlSetpoint
        try:
            from skybeta_flight_interface.topics import fmu_topic
            cmd_topic = fmu_topic('in', 'vehicle_command', 'VehicleCommand')
            man_topic = fmu_topic('in', 'manual_control_input', 'ManualControlSetpoint')
        except Exception:
            cmd_topic, man_topic = '/fmu/in/vehicle_command', '/fmu/in/manual_control_input'
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.cmd_pub = self.create_publisher(VehicleCommand, cmd_topic, qos)
        self.man_pub = self.create_publisher(ManualControlSetpoint, man_topic, qos)
        self.create_subscription(String, '/skybeta/state', self.on_state, 10)
        self.create_subscription(String, str(g('status_topic')), self.on_mission, 10)
        self.land_states = {s.strip() for s in str(g('land_states')).split(',') if s.strip()}
        self.mode = str(g('mode'))
        self.wait_s, self.timeout_s = float(g('wait_s')), float(g('timeout_s'))
        self.require_mission = bool(g('require_mission'))
        self.state, self.mission = {}, {}
        self.step_name, self.step_t, self.flight_t = 'WAIT_READY', self.now(), None
        self.was_armed = False
        self.create_timer(0.05, self.tick)
        self.get_logger().warn(f'SIMULATION ONLY pilot, mode={self.mode}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, m):
        try:
            self.state = json.loads(m.data)
        except ValueError:
            pass

    def on_mission(self, m):
        try:
            self.mission = json.loads(m.data)
        except ValueError:
            pass

    def command(self, cmd, p1=0.0, p2=0.0, p3=0.0):
        m = self.VC()
        m.timestamp = int(self.now() * 1e6)
        m.command = cmd
        m.param1, m.param2, m.param3 = float(p1), float(p2), float(p3)
        m.target_system = m.target_component = m.source_system = m.source_component = 1
        m.from_external = True
        self.cmd_pub.publish(m)

    def sticks(self):
        m = self.MCS()
        t = int(self.now() * 1e6)
        m.timestamp = m.timestamp_sample = t
        m.valid = True
        m.data_source = 2          # SOURCE_MAVLINK_0
        m.roll = m.pitch = m.yaw = 0.0
        m.throttle = -1.0          # throttle LOW, like the pilot during autonomy
        m.flaps = m.aux1 = m.aux2 = m.aux3 = m.aux4 = m.aux5 = m.aux6 = 0.0
        self.man_pub.publish(m)

    def goto(self, name):
        self.get_logger().info(f'pilot: {self.step_name} -> {name}')
        self.step_name, self.step_t = name, self.now()

    def tick(self):
        st, t = self.state, self.now()
        dt = t - self.step_t
        if self.mode == 'handshake':
            self.sticks()
        armed = bool(st.get('armed'))
        if self.step_name == 'WAIT_READY':
            ready = st.get('px4_link') and st.get('local_enu') is not None
            if self.require_mission:      # the mission is up and streaming hold setpoints
                ready = ready and (self.mission.get('phase') == 'DORMANT'
                                   or self.mission.get('state') in ('STREAM', 'WAIT_OFFBOARD'))
            if ready and dt > self.wait_s:
                self.goto('MODE')
        elif self.step_name == 'MODE':
            if self.mode == 'handshake':
                self.command(CMD_DO_SET_MODE, 1, MAIN_ALTCTL)
            else:
                self.command(CMD_DO_SET_MODE, 1, MAIN_AUTO, SUB_LOITER)
            if dt > 1.0:
                self.goto('ARM')
        elif self.step_name == 'ARM':
            if not armed and int(dt * 20) % 10 == 0:
                self.command(CMD_ARM, 1.0)
            if armed:
                self.was_armed = True
                self.flight_t = t
                self.goto('HANDOVER')
            elif dt > 10.0:
                self.get_logger().error(f'PX4 would not arm (state: {st.get("nav_state_name")}). '
                                        'Check the PX4 console; try mode:=direct or QGroundControl.')
                self.goto('DONE')
        elif self.step_name == 'HANDOVER':
            if dt > 2.0 and dt < 4.0:
                if self.mode == 'handshake':
                    self.command(CMD_DO_SET_MODE, 1, MAIN_POSCTL)
                elif int(dt * 20) % 10 == 0:
                    self.call_request_offboard()
            if st.get('offboard_active'):
                self.goto('WATCH')
            elif dt > 8.0:
                self.get_logger().error('Offboard did not start: read flight_interface\'s '
                                        '"auto offboard: staying out of Offboard" message')
                self.goto('WATCH')
        elif self.step_name == 'WATCH':
            if self.was_armed and not armed:
                self.get_logger().info(f'disarmed {t - self.flight_t:.1f} s after arming. Flight over.')
                self.goto('DONE')
            elif self.mission.get('state') in self.land_states:
                self.get_logger().info(f'mission reports {self.mission.get("state")} '
                                       f'({self.mission.get("note", "")}): pilot lands')
                self.command(CMD_LAND)
                self.goto('LANDING')
            elif t - self.flight_t > self.timeout_s:
                self.get_logger().error('timeout: pilot takes over and lands')
                self.command(CMD_LAND)
                self.goto('LANDING')
        elif self.step_name == 'LANDING':
            if not armed:
                self.goto('DONE')
            elif int(dt * 20) % 20 == 0:
                self.command(CMD_LAND)
        elif self.step_name == 'DONE':
            raise SystemExit(0)

    def call_request_offboard(self):
        from std_srvs.srv import Trigger
        if not hasattr(self, 'cli'):
            self.cli = self.create_client(Trigger, '/skybeta/request_offboard')
        if self.cli.service_is_ready():
            self.cli.call_async(Trigger.Request())


def main(args=None):
    rclpy.init(args=args)
    n = SimPilot()
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    except KeyboardInterrupt:              # Ctrl-C = the pilot takes over: land if still armed
        if n.state.get('armed'):
            import time
            for _ in range(5):
                n.command(CMD_LAND)
                time.sleep(0.2)
            n.get_logger().warn('stopped while armed: LAND sent')
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

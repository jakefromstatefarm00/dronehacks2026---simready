"""SIMULATION ONLY: a stand-in for the safety pilot (there is no RC pilot in a simulator).

    NEVER ON THE DRONE. On the drone the human pilot does exactly these steps on RC. This node
    publishes /fmu/in/vehicle_command, which team code must not do on hardware; it refuses to
    start unless ``sim_only:=true`` (sim.launch.py and selftest.launch.py set it).

What it does, once per launch (``repeat:=true`` for every READY):

    1. waits until the mission node reports READY (healthy, streaming, on the ground, disarmed)
       and /skybeta/state shows landed + disarmed, for ``start_delay_s``
    2. mode:=drone_like (default) - exactly the pilot's procedure:
           Altitude mode -> arm -> (1 s) -> Position mode
       flight_interface then enters Offboard by itself (auto_offboard_on_arm), as on the drone.
       Needs a simulator that accepts Altitude/Position without an RC (fake PX4: yes; a PX4
       SITL with COM_RC_IN_MODE=4 may refuse - then use mode:=direct).
       mode:=direct - /skybeta/request_offboard (needs allow_mode_request:=true), then arm.
    3. when the mission reports ``land_request`` (LOW_HOLD) for ``land_after_s``, or a FAULT /
       TAKEN_OVER: Land (as the pilot would), then waits for the disarm
    4. prints the mission's result and the journal path

Commands are repeated every second until PX4's state shows them (the link is best-effort).
"""
from __future__ import annotations

import json
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

CMD_DO_SET_MODE, CMD_ARM, CMD_LAND = 176, 400, 21
MAIN_MODE = {'ALTCTL': 2.0, 'POSCTL': 3.0}
NAV = {'ALTCTL': 1, 'POSCTL': 2, 'OFFBOARD': 14, 'AUTO_LAND': 18}


class SimPilot(Node):

    def __init__(self):
        super().__init__('sim_pilot')
        d = ParameterDescriptor(dynamic_typing=True)
        for name, default in (('sim_only', False), ('mode', 'drone_like'), ('start_delay_s', 3.0),
                              ('land_after_s', 3.0), ('repeat', False), ('exit_when_done', False),
                              ('mission_status_topic', '/dronehacks_pathplan/status')):
            self.declare_parameter(name, default, d)
        g = lambda n: self.get_parameter(n).value
        if str(g('sim_only')).lower() not in ('true', '1'):
            raise SystemExit('sim_pilot is SIMULATION ONLY (it sends PX4 commands). Start it with sim_only:=true '
                             'from sim.launch.py / selftest.launch.py, never on the drone.')
        self.mode = str(g('mode'))
        if self.mode not in ('drone_like', 'direct'):
            raise SystemExit("mode must be 'drone_like' or 'direct'")
        self.start_delay, self.land_after = float(g('start_delay_s')), float(g('land_after_s'))
        self.repeat = str(g('repeat')).lower() in ('true', '1')
        self.exit_when_done = str(g('exit_when_done')).lower() in ('true', '1')
        try:
            from skybeta_flight_interface.topics import resolve_endpoints
            topic = resolve_endpoints()['vehicle_command']
        except Exception:                                   # flight interface not installed
            topic = '/fmu/in/vehicle_command'
        from px4_msgs.msg import VehicleCommand
        self.VehicleCommand = VehicleCommand
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.cmd_pub = self.create_publisher(VehicleCommand, topic, qos)
        self.offboard_cli = self.create_client(Trigger, '/skybeta/request_offboard')
        self.create_subscription(String, '/skybeta/state', self.on_state, 10)
        self.create_subscription(String, str(g('mission_status_topic')), self.on_status, 10)
        self.state, self.status = {}, {}
        self.step, self.t_step, self.t_last_cmd, self.ready_since, self.land_since = 'wait_ready', time.monotonic(), 0.0, None, None
        self.create_timer(.1, self.tick)
        self.get_logger().warn(f'SIM PILOT ({self.mode}) - simulation only. Commands on {topic}.')

    def on_state(self, msg):
        try:
            self.state = json.loads(msg.data)
        except ValueError:
            pass

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            pass

    def send(self, command, p1=0.0, p2=0.0, what=''):
        m = self.VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command, m.param1, m.param2 = command, float(p1), float(p2)
        m.target_system, m.target_component, m.source_system, m.source_component = 1, 1, 1, 1
        m.from_external = True
        self.cmd_pub.publish(m)
        self.t_last_cmd = time.monotonic()
        if what:
            self.get_logger().info(f'pilot: {what}')

    def due(self):
        return time.monotonic() - self.t_last_cmd >= 1.0

    def go(self, step, note=''):
        self.step, self.t_step = step, time.monotonic()
        if note:
            self.get_logger().info(f'pilot: {note}')

    def tick(self):
        s, st, now = self.state, self.status, time.monotonic()
        nav, armed, landed = s.get('nav_state', -1), bool(s.get('armed')), bool(s.get('landed'))
        mstate = st.get('state', '')
        if self.step == 'wait_ready':
            if mstate == 'READY' and landed and not armed and s.get('px4_link'):
                self.ready_since = self.ready_since or now
                if now - self.ready_since >= self.start_delay:
                    self.go('mode_alt' if self.mode == 'drone_like' else 'request_offboard',
                            f'mission READY for {self.start_delay:.0f} s - starting the pilot procedure ({self.mode})')
            else:
                self.ready_since = None
        elif self.step == 'mode_alt':                                       # drone_like 1/3
            if nav == NAV['ALTCTL']:
                self.go('arm')
            elif self.due():
                self.send(CMD_DO_SET_MODE, 1.0, MAIN_MODE['ALTCTL'], 'mode switch to Altitude')
            if now - self.t_step > 10:
                self.go('give_up', 'PX4 did not enter Altitude (no RC input allowed?) - try mode:=direct')
        elif self.step == 'arm':                                            # 2/3 (both modes)
            if armed:
                self.go('mode_pos' if self.mode == 'drone_like' else 'flying', 'armed')
            elif self.due():
                self.send(CMD_ARM, 1.0, 0.0, 'arm')
            if now - self.t_step > 10:
                self.go('give_up', 'PX4 refused to arm (see its console / preflight checks)')
        elif self.step == 'mode_pos':                                       # drone_like 3/3
            if nav in (NAV['POSCTL'], NAV['OFFBOARD']):
                self.go('flying', 'Position selected - flight_interface should now enter Offboard')
            elif now - self.t_step > 1.0 and self.due():
                self.send(CMD_DO_SET_MODE, 1.0, MAIN_MODE['POSCTL'], 'mode switch to Position')
        elif self.step == 'request_offboard':                               # direct 1/2
            if nav == NAV['OFFBOARD']:
                self.go('arm', 'Offboard active (requested); arming')
            elif self.due():
                if self.offboard_cli.service_is_ready():
                    self.offboard_cli.call_async(Trigger.Request())
                    self.t_last_cmd = now
                    self.get_logger().info('pilot: /skybeta/request_offboard')
                else:
                    self.get_logger().warn('pilot: /skybeta/request_offboard not available (allow_mode_request:=true?)')
                    self.t_last_cmd = now
            if now - self.t_step > 10:
                self.go('give_up', 'Offboard refused (allow_mode_request:=true? setpoint stream running?)')
        elif self.step == 'flying':
            want_land = st.get('land_request') or mstate in ('FAULT', 'TAKEN_OVER')
            if want_land and armed:
                self.land_since = self.land_since or now
                if now - self.land_since >= self.land_after and nav != NAV['AUTO_LAND'] and self.due():
                    self.send(CMD_LAND, what=f'Land (mission state {mstate}: {st.get("note", "")})')
            else:
                self.land_since = None
            if not armed and now - self.t_step > 3.0 and mstate in ('DONE', 'WAIT_HEALTH'):
                self.get_logger().warn(f'pilot: disarmed. FLIGHT OVER. Result: {st.get("result") or mstate}. '
                                       f'Cleared {st.get("n_cleared")}/{st.get("n_checkpoints")}. '
                                       f'Journal: {st.get("journal")}')
                self.go('wait_ready' if self.repeat else 'finished')
        elif self.step == 'give_up':
            self.go('finished')
        elif self.step == 'finished' and self.exit_when_done and now - self.t_step > 2.0:
            raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SimPilot()
    except SystemExit as e:
        print(e)
        rclpy.try_shutdown()
        return
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

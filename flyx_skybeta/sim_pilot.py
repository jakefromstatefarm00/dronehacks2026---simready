"""SIMULATION ONLY: plays the safety pilot for a Sky Beta-contract program (FlyX lab, Isaac sim).

On the drone a human on RC arms, switches to Position (the flight interface then enters Offboard)
and lands. In sim there is no RC, so this does the same through the paths the organizer's sim
README uses (skybeta/sim/README.md step 7):
  1. wait until the program streams setpoints (its status topic) and /skybeta/state is healthy;
  2. /skybeta/request_offboard (needs the flight interface with allow_mode_request:=true, SITL
     only), then arm (VehicleCommand, as `pxh> commander arm`); retried until PX4 shows both;
  3. land (VehicleCommand NAV_LAND, as `pxh> commander land`) when the program reports HOLD /
     PAUSED / TAKEN_OVER / DONE, on a time limit, or on Ctrl-C; exit once disarmed.
It never publishes setpoints or offboard_control_mode (the flight interface owns those).
Never run this on the drone.

    python3 flyx_skybeta/sim_pilot.py [--status-topic /flyx_course/status] [--limit-s 200]
"""
import argparse
import json
import signal
import sys
import threading
import time

CMD_ARM_DISARM, CMD_NAV_LAND = 400, 21
END_STATES = {"HOLD", "PAUSED", "TAKEN_OVER", "DONE", "FAILED"}
STREAMING_STATES = {"STREAM", "WAIT_FOR_OFFBOARD"}      # flyx_course, pathfinding pathplan_mission


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--status-topic", action="append",
                    help="program status topic(s) (JSON with 'state'); default: flyx_course + pathfinding")
    ap.add_argument("--limit-s", type=float, default=200.0, help="land this long after arming")
    ap.add_argument("--arm-timeout-s", type=float, default=60.0)
    a, _ = ap.parse_known_args(argv)
    topics = a.status_topic or ["/flyx_course/status", "/dronehacks_pathplan/status"]

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    import px4_msgs.msg as px4

    class Pilot(Node):
        def __init__(self):
            super().__init__("sim_pilot")
            self.sky, self.sky_t, self.prog, self.prog_t, self.stream_since = {}, None, None, None, None
            self.create_subscription(String, "/skybeta/state", self.on_sky, 10)
            for t in topics:
                self.create_subscription(String, t, self.on_prog, 10)
            self.cmd_pub = self.create_publisher(px4.VehicleCommand, "/fmu/in/vehicle_command", qos_profile_sensor_data)
            self.offboard_cli = self.create_client(Trigger, "/skybeta/request_offboard")

        def on_sky(self, m):
            try:
                self.sky, self.sky_t = json.loads(m.data), time.monotonic()
            except ValueError:
                pass

        def on_prog(self, m):
            try:
                st = json.loads(m.data).get("state")
            except (ValueError, AttributeError):
                return
            self.prog, self.prog_t = st, time.monotonic()
            if st in STREAMING_STATES:
                self.stream_since = self.stream_since or time.monotonic()
            else:
                self.stream_since = None

        def command(self, command, p1=0.0, p2=0.0):
            c = px4.VehicleCommand()
            c.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            c.command, c.param1, c.param2 = command, float(p1), float(p2)
            c.target_system = c.target_component = c.source_system = c.source_component = 1
            c.from_external = True
            self.cmd_pub.publish(c)

        def request_offboard(self):
            if self.offboard_cli.service_is_ready():
                self.offboard_cli.call_async(Trigger.Request())
                return True
            return False

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Pilot()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    log = lambda s: print(f"[sim_pilot {time.strftime('%H:%M:%S')}] {s}", flush=True)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    armed = lambda: bool(node.sky.get("armed"))
    offboard = lambda: bool(node.sky.get("offboard_active"))
    code = 1
    try:
        log(f"SIMULATION ONLY pilot; watching {topics}")
        while not stop.is_set():                                    # 1. program streaming, PX4 linked
            if node.stream_since and time.monotonic() - node.stream_since > 3 and node.sky.get("px4_link") \
                    and node.sky.get("local_enu") is not None and not armed():
                break
            time.sleep(.2)
        else:
            raise SystemExit
        log(f"program streaming ({node.prog}); requesting Offboard, then arming")
        t0 = time.monotonic()
        while not stop.is_set() and not (armed() and offboard()):  # 2. Offboard + arm
            if time.monotonic() - t0 > a.arm_timeout_s:
                log(f"PX4 did not arm + enter Offboard in {a.arm_timeout_s:.0f} s: {node.sky}")
                raise SystemExit
            if not offboard():
                log("request_offboard: " + ("sent" if node.request_offboard() else "service not ready"))
                time.sleep(1.0)
            if not armed():
                log("arm"); node.command(CMD_ARM_DISARM, 1.0)
            time.sleep(2.0)
        t_arm = time.monotonic()
        log("armed and in Offboard: the program flies")
        while not stop.is_set():                                    # 3. wait for the end
            if node.prog in END_STATES:
                log(f"program reports {node.prog}: landing"); break
            if time.monotonic() - t_arm > a.limit_s:
                log("time limit: landing"); break
            if not armed():
                log("disarmed"); break
            time.sleep(.2)
        code = 0 if node.prog in ("HOLD", "DONE") else 1
    except SystemExit:
        pass
    finally:
        if armed():
            log("land")
            t0 = time.monotonic()
            while armed() and time.monotonic() - t0 < 60:
                node.command(CMD_NAV_LAND)
                time.sleep(2.0)
            log("disarmed" if not armed() else "still armed after 60 s")
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())

"""Read-only telemetry monitor for the Sky Beta drone (publishes nothing, commands nothing).

One persistent ROS 2 node, so rates are real (no per-call discovery delay). Every second it prints
one line and writes one CSV row: per-topic rate and worst gap, odometry speed / jump, IMU motion,
depth centre distance, plus Jetson CPU per core, RAM and the hottest thermal zone.

    python3 telemetry_watch.py [seconds] [out_dir]        # seconds 0 = until Ctrl-C
"""
import csv
import glob
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

try:
    from px4_msgs.msg import SensorCombined
except ImportError:
    SensorCombined = None

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.join("/root/team_ws/runs", time.strftime("telemetry-%Y%m%d-%H%M%S"))
RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)

# name, topic, type, qos, expected minimum Hz (0 = informational)
TOPICS = [
    ("odom", "/visual_slam/tracking/odometry", Odometry, qos_profile_sensor_data, 25),
    ("cam_l", "/oak/left/camera_info", CameraInfo, qos_profile_sensor_data, 25),   # one per left image, tiny
    ("depth", "/camera/depth/image_raw", Image, qos_profile_sensor_data, 20),
    ("oak_imu", "/oak/imu/data", Imu, qos_profile_sensor_data, 150),
    ("state", "/skybeta/state", String, RELIABLE, 5),
    ("jolie", "/jolie_flyx/status", String, RELIABLE, 0),
    ("team", "/dronehacks_pathplan/status", String, RELIABLE, 0),
    ("shadow_sp", "/jolie_flyx/shadow/setpoint_pose", PoseStamped, RELIABLE, 0),
    ("real_sp", "/skybeta/setpoint/pose", PoseStamped, RELIABLE, 0),
]
if SensorCombined is not None:
    TOPICS.insert(4, ("px4_imu", "/fmu/out/sensor_combined", SensorCombined, qos_profile_sensor_data, 80))


def cpu_times():
    out = []
    with open("/proc/stat") as f:
        for line in f:
            if line.startswith("cpu") and line[3].isdigit():
                v = [int(x) for x in line.split()[1:]]
                out.append((sum(v), v[3] + v[4]))
    return out


def mem_used_pct():
    m = {}
    with open("/proc/meminfo") as f:
        for line in f:
            k, v = line.split(":")
            m[k] = int(v.split()[0])
    return 100.0 * (1 - m["MemAvailable"] / m["MemTotal"])


def max_temp():
    best = (0.0, "")
    for z in glob.glob("/sys/class/thermal/thermal_zone*/temp"):
        try:
            t = int(open(z).read()) / 1000.0
            name = open(os.path.join(os.path.dirname(z), "type")).read().strip()
        except Exception:          # some Tegra zones fail to read (EIO / odd decode): skip them
            continue
        if t > best[0]:
            best = (t, name)
    return best


class Watch(Node):
    def __init__(self):
        super().__init__("telemetry_watch")
        os.makedirs(OUT, exist_ok=True)
        self.n = {k: 0 for k, *_ in TOPICS}
        self.last_t = {k: None for k, *_ in TOPICS}
        self.gap = {k: 0.0 for k, *_ in TOPICS}
        self.total = {k: 0 for k, *_ in TOPICS}
        self.worst_gap = {k: 0.0 for k, *_ in TOPICS}
        self.expect = {k: e for k, _, _, _, e in TOPICS}
        self.odom_prev, self.speed, self.jump = None, 0.0, 0.0
        self.max_jump = 0.0
        self.gyro = {"oak_imu": 0.0, "px4_imu": 0.0}
        self.depth_c = float("nan")
        self.state, self.prog = "", ""
        for k, topic, typ, qos, _ in TOPICS:
            self.create_subscription(typ, topic, lambda m, k=k: self.on_msg(k, m), qos)
        self.cpu_prev = cpu_times()
        self.t0 = time.time()
        self.fh = open(os.path.join(OUT, "telemetry.csv"), "w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow(["t"] + [f"{k}_hz" for k, *_ in TOPICS] + [f"{k}_gap" for k, *_ in TOPICS] +
                        ["odom_speed", "odom_jump", "oak_gyro", "px4_gyro", "depth_centre_m",
                         "cpu_max", "cpu_mean", "mem_pct", "temp_c", "state", "program"])
        self.create_timer(1.0, self.tick)
        print(f"telemetry -> {OUT}", flush=True)

    def on_msg(self, k, m):
        now = time.time()
        if self.last_t[k] is not None:
            self.gap[k] = max(self.gap[k], now - self.last_t[k])
        self.last_t[k] = now
        self.n[k] += 1
        self.total[k] += 1
        if k == "odom":
            p = m.pose.pose.position
            cur = np.array([p.x, p.y, p.z])
            if self.odom_prev is not None:
                d = float(np.linalg.norm(cur - self.odom_prev[0]))
                dt = max(now - self.odom_prev[1], 1e-3)
                self.speed = d / dt
                self.jump = max(self.jump, d)
            self.odom_prev = (cur, now)
        elif k in ("oak_imu", "px4_imu"):
            g = ([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] if k == "oak_imu"
                 else list(m.gyro_rad))
            self.gyro[k] = max(self.gyro[k], math.sqrt(sum(v * v for v in g)))
        elif k == "depth":
            if m.encoding in ("16UC1", "mono16"):
                d = np.frombuffer(m.data, np.uint16).reshape(m.height, -1)[:, :m.width]
                c = d[m.height // 2 - 10:m.height // 2 + 10, m.width // 2 - 10:m.width // 2 + 10]
                c = c[c > 0]
                self.depth_c = float(np.median(c)) / 1000.0 if c.size else float("nan")
        elif k == "state":
            try:
                s = json.loads(m.data)
                self.state = f"armed={s.get('armed')} landed={s.get('landed')} nav={s.get('nav_state_name', s.get('nav_state'))} off={s.get('offboard_active')}"
            except ValueError:
                pass
        elif k in ("jolie", "team"):
            self.prog = m.data[:60].replace("\n", " ")

    def tick(self):
        now = time.time()
        cur = cpu_times()
        loads = [100.0 * (1 - (i1 - i0) / max(t1 - t0, 1)) for (t0, i0), (t1, i1) in zip(self.cpu_prev, cur)]
        self.cpu_prev = cur
        temp, tname = max_temp()
        mem = mem_used_pct()
        for k in self.n:      # a topic silent since its last message: the open gap counts too
            if self.last_t[k] is not None:
                self.gap[k] = max(self.gap[k], now - self.last_t[k])
            if self.total[k]:
                self.worst_gap[k] = max(self.worst_gap[k], self.gap[k])
        self.max_jump = max(self.max_jump, self.jump)
        flags = [k for k, e in self.expect.items() if e and self.n[k] < 0.8 * e and now - self.t0 > 6]
        line = (f"[{now - self.t0:5.0f}s] " +
                " ".join(f"{k}:{self.n[k]:3d}" for k in ("odom", "cam_l", "depth", "oak_imu", "px4_imu") if k in self.n) +
                f" | gap odom {self.gap['odom']:.2f}s depth {self.gap['depth']:.2f}s | v {self.speed:4.2f} m/s jump {self.jump:4.2f} m"
                f" | gyro oak {self.gyro['oak_imu']:4.2f} px4 {self.gyro['px4_imu']:4.2f} | depth {self.depth_c:5.2f} m"
                f" | CPU max {max(loads):3.0f}% mean {sum(loads) / len(loads):3.0f}% RAM {mem:3.0f}% {temp:4.1f}C"
                f"{' | LOW: ' + ','.join(flags) if flags else ''}")
        print(line, flush=True)
        self.w.writerow([f"{now - self.t0:.1f}"] + [self.n[k] for k in self.n] + [f"{self.gap[k]:.3f}" for k in self.n] +
                        [f"{self.speed:.3f}", f"{self.jump:.3f}", f"{self.gyro['oak_imu']:.3f}", f"{self.gyro['px4_imu']:.3f}",
                         f"{self.depth_c:.3f}", f"{max(loads):.0f}", f"{sum(loads) / len(loads):.0f}", f"{mem:.0f}",
                         f"{temp:.1f}", self.state, self.prog])
        self.fh.flush()
        for k in self.n:
            self.n[k], self.gap[k] = 0, 0.0
        self.jump, self.gyro = 0.0, {"oak_imu": 0.0, "px4_imu": 0.0}
        if DURATION and now - self.t0 > DURATION:
            raise SystemExit

    def summary(self):
        el = max(time.time() - self.t0, 1e-3)
        lines = ["", "=" * 72, f"SUMMARY over {el:.0f} s  ({OUT})"]
        ok = True
        for k, topic, _, _, e in TOPICS:
            rate = self.total[k] / el
            bad = e and (rate < 0.8 * e or self.worst_gap[k] > 1.0)
            ok &= not bad
            lines.append(f"  {'FAIL' if bad else ' ok '} {topic:38s} {rate:6.1f} Hz  worst gap {self.worst_gap[k]:5.2f} s"
                         f"{'  (expected >= %d Hz)' % e if e else ''}")
        lines.append(f"  largest odometry step between messages: {self.max_jump:.2f} m (a jump > 0.3 m while handheld = VIO glitch)")
        lines.append(f"  RESULT: {'PASS' if ok else 'FAIL'}")
        print("\n".join(lines), flush=True)
        with open(os.path.join(OUT, "summary.txt"), "w") as f:
            f.write("\n".join(lines) + "\n")


def main():
    rclpy.init()
    w = Watch()
    try:
        rclpy.spin(w)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        w.summary()
        w.fh.close()


if __name__ == "__main__":
    main()

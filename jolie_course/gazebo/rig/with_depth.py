#!/usr/bin/env python3
"""SIM ONLY, opt-in: give a GB10 Gazebo run the FlyX-lab depth topics, then run a native program.

    NATIVE_PROGRAM="$HOME/dronehacks-2026-gb10/jolie_flyx/with_depth.py \
        $HOME/dronehacks-2026-gb10/jolie_flyx/upload_to_flyx.py [program args]" \
        nohup sim/runtime/gb10_slot_run.sh <traj> <config> artifacts/gb10-sky-beta-course-vNNN

    queue line:  NNN jolie_flyx/with_depth.py jolie_flyx/upload_to_flyx.py [program args]

The rig's world already renders an ideal depth camera on the airframe (``ideal_depth``,
depth_camera, 640x400, hfov 1.3962634 rad, 15 Hz, pose (0.11466, -0.00015, 0.02891) m in
base_link FLU, no rotation; same pose as the rgb camera).  The rig bridges it to
/camera/ideal_depth/image but not its CameraInfo (gz publishes that on /dronehacks/camera_info,
not on the SDF's camera_info_topic).  This wrapper starts a second ros_gz_bridge in the same
slot that maps

    /dronehacks/ideal_depth   -> /camera/depth/image_raw     (32FC1, metres, +inf = no return)
    /dronehacks/camera_info   -> /camera/depth/camera_info   (gz's own pinhole K)

Header stamps are Gazebo sim time, the clock PX4 SITL and the program's use_sim_time run on.
Rendering load is unchanged (the rig already subscribes the depth camera).  Then it runs the
program (``python3 -u program [args]``), forwards SIGINT/SIGTERM, stops the bridge and exits
with the program's code.  Nothing is sent to PX4 by this file.

A recorder (separate process) saves every ``--depth-record-period`` sim seconds a [::4, ::4]
float16 depth frame + stamp to <run>/depth/depth-frames.npz (and depth-info.json) for offline
geometry checks against truth (jolie_course/gazebo/validate_depth.py).  <run> is the parent of
the program's ``--output`` (launch.sh passes <run>/journal) or ``--depth-record DIR``.

Wrapper options (all others go to the program):
    --depth-record DIR        where to record (default: <program --output>/../depth)
    --depth-record-period S   sim seconds between recorded frames (default 1.0; 0 = no recording)
    --depth-probe-s S         no program: just bridge + record for S wall seconds, exit 0
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

DEPTH_GZ = "/dronehacks/ideal_depth"
DEPTH_ROS = "/camera/depth/image_raw"
INFO_ROS = "/camera/depth/camera_info"
INFO_GZ_CANDIDATES = ("/dronehacks/ideal_depth/camera_info", "/dronehacks/camera_info")
RIG = Path.home() / "dronehacks-2026-gb10"


def log(*a):
    print("[with_depth]", *a, flush=True)


def split_args(argv):
    own = {"--depth-record": None, "--depth-record-period": "1.0", "--depth-probe-s": None}
    rest, i = [], 0
    while i < len(argv):
        if argv[i] in own and i + 1 < len(argv):
            own[argv[i]] = argv[i + 1]
            i += 2
            continue
        rest.append(argv[i])
        i += 1
    # the program is the first *.py among the rest; anything before it (e.g. launch.sh's
    # "--output X") is moved after it
    prog_i = next((k for k, x in enumerate(rest) if x.endswith(".py")), None)
    if prog_i is None:
        return own, None, rest
    program = rest[prog_i]
    return own, program, rest[prog_i + 1:] + rest[:prog_i]


def resolve(path):
    p = Path(path)
    if p.is_absolute() or p.exists():
        return p.resolve()
    for base in (RIG, Path(__file__).resolve().parents[1]):
        if (base / p).exists():
            return (base / p).resolve()
    return p


def gz_topics():
    try:
        out = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True, timeout=20).stdout
        return set(out.split())
    except Exception as exc:                       # noqa: BLE001
        log("gz topic -l failed:", exc)
        return set()


def start_bridge(logdir):
    topics = set()
    for _ in range(30):                            # the sensors are up long before the program starts
        topics = gz_topics()
        if DEPTH_GZ in topics:
            break
        time.sleep(1)
    if DEPTH_GZ not in topics:
        log(f"WARNING: {DEPTH_GZ} not advertised; bridging it anyway")
    info_gz = next((t for t in INFO_GZ_CANDIDATES if t in topics), INFO_GZ_CANDIDATES[-1])
    entries = [
        {"gz_topic_name": DEPTH_GZ, "ros_topic_name": DEPTH_ROS, "gz_type_name": "gz.msgs.Image",
         "ros_type_name": "sensor_msgs/msg/Image", "direction": "GZ_TO_ROS"},
        {"gz_topic_name": info_gz, "ros_topic_name": INFO_ROS, "gz_type_name": "gz.msgs.CameraInfo",
         "ros_type_name": "sensor_msgs/msg/CameraInfo", "direction": "GZ_TO_ROS"},
    ]
    cfg = Path(logdir) / "depth-bridge.yaml"
    cfg.write_text(json.dumps(entries, indent=2) + "\n")
    log(f"bridging {DEPTH_GZ} -> {DEPTH_ROS}, {info_gz} -> {INFO_ROS}")
    handle = open(Path(logdir) / "depth-bridge.log", "wb")
    # the binary itself, not "ros2 run" (killing that wrapper left the bridge orphaned)
    exe = next((Path(p) / "lib/ros_gz_bridge/parameter_bridge" for p in
                os.environ.get("AMENT_PREFIX_PATH", "/opt/ros/jazzy").split(":")
                if (Path(p) / "lib/ros_gz_bridge/parameter_bridge").exists()),
               Path("/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge"))
    return subprocess.Popen([str(exe), "--ros-args",
                             "-r", "__node:=depth_bridge", "-p", f"config_file:={cfg}"],
                            stdout=handle, stderr=subprocess.STDOUT)


# ------------------------------------------------------------------------------ recorder
def recorder(outdir, period):
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    rclpy.init()

    class Rec(Node):
        def __init__(self):
            super().__init__("depth_recorder")
            self.set_parameters([Parameter("use_sim_time", value=True)])
            self.frames, self.stamps, self.lags, self.info = [], [], [], None
            self.count, self.first, self.last, self.encoding, self.frame_id = 0, None, None, None, None
            self.next_t, self.last_dump = 0.0, 0
            self.create_subscription(Image, DEPTH_ROS, self.on_image, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, INFO_ROS, self.on_info, qos_profile_sensor_data)

        def on_info(self, msg):
            if self.info is None:
                self.info = {"k": list(msg.k), "d": list(msg.d), "p": list(msg.p), "width": msg.width,
                             "height": msg.height, "frame_id": msg.header.frame_id,
                             "distortion_model": msg.distortion_model}
                self.dump()

        def on_image(self, msg):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            now = self.get_clock().now().nanoseconds * 1e-9
            self.count += 1
            self.first = t if self.first is None else self.first
            self.last, self.encoding, self.frame_id = t, msg.encoding, msg.header.frame_id
            if now > 0:
                self.lags.append(now - t)
            if period <= 0 or t < self.next_t:
                return
            self.next_t = t + period
            dt = "<f4" if msg.encoding == "32FC1" else "<u2"
            img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
            img = img[:, :msg.width * np.dtype(dt).itemsize].copy().view(dt).astype(np.float32)
            if dt == "<u2":
                img /= 1000.0
            self.frames.append(img[::4, ::4].astype(np.float16))
            self.stamps.append(t)
            if len(self.frames) - self.last_dump >= 10:
                self.dump()

        def dump(self):
            self.last_dump = len(self.frames)
            if self.frames:
                tmp = outdir / "depth-frames.tmp.npz"
                np.savez_compressed(tmp, stamps=np.array(self.stamps), depth=np.stack(self.frames), subsample=4)
                os.replace(tmp, outdir / "depth-frames.npz")
            lags = np.array(self.lags) if self.lags else np.zeros(1)
            info = {"camera_info": self.info, "encoding": self.encoding, "frame_id": self.frame_id,
                    "frames_received": self.count, "first_stamp": self.first, "last_stamp": self.last,
                    "rate_hz_sim": (self.count - 1) / (self.last - self.first)
                    if self.count > 1 and self.last > self.first else None,
                    "receive_lag_sim_s": {"median": float(np.median(lags)), "p95": float(np.percentile(lags, 95))},
                    "recorded": len(self.frames)}
            (outdir / "depth-info.json").write_text(json.dumps(info, indent=1) + "\n")

    node = Rec()
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    while not stop and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
    node.dump()
    node.destroy_node()
    rclpy.try_shutdown()


# ------------------------------------------------------------------------------ main
def main(argv):
    if argv[:1] == ["--_recorder"]:
        recorder(argv[1], float(argv[2]))
        return 0
    own, program, prog_args = split_args(argv)
    probe = own["--depth-probe-s"]
    if program is None and probe is None:
        log("usage: with_depth.py [--depth-record DIR] [--depth-record-period S] program.py [args]")
        return 2
    record = own["--depth-record"]
    if record is None and "--output" in prog_args:
        record = str(Path(prog_args[prog_args.index("--output") + 1]).parent / "depth")
    record = Path(record or tempfile.mkdtemp(prefix="depth-"))
    record.mkdir(parents=True, exist_ok=True)
    period = float(own["--depth-record-period"])

    children = [start_bridge(record)]
    rec_log = open(record / "depth-recorder.log", "wb")
    children.append(subprocess.Popen([sys.executable, "-u", str(Path(__file__).resolve()), "--_recorder",
                                      str(record), str(period)], stdout=rec_log, stderr=subprocess.STDOUT))
    code = 0
    try:
        if probe is not None:
            log(f"probe: bridging + recording for {probe} s, no program")
            time.sleep(float(probe))
        else:
            cmd = [sys.executable, "-u", str(resolve(program)), *prog_args]
            log("running", " ".join(cmd))
            # own session: the rig's killpg(SIGINT) reaches it once, through us, not twice
            prog = subprocess.Popen(cmd, start_new_session=True)
            for sig in (signal.SIGINT, signal.SIGTERM):
                signal.signal(sig, lambda s, _f: prog.send_signal(s))
            while True:
                try:
                    code = prog.wait()
                    break
                except InterruptedError:
                    continue
    finally:
        for c in reversed(children):
            if c.poll() is None:
                c.send_signal(signal.SIGINT)
        for c in reversed(children):
            try:
                c.wait(timeout=5)
            except subprocess.TimeoutExpired:
                c.kill()
                c.wait()
    log("exit", code)
    return code


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

"""Fly pathplan's OnlinePlanner in a plain Gazebo Harmonic world (no PX4, no ROS).

Built for the course world of github.com/machmoon/dronehacks-sim (used as a local test
tool, not part of our code): the Open Robotics X3 quadrotor driven by Gazebo's
MulticopterVelocityControl, ground-truth odometry in the course frame, per-link contacts.

    python3 fly_planner.py --output /out/run1                 # inside that sim's container

Phases: TAKEOFF straight up -> NAVIGATE (planner carrot, look-ahead yaw) -> LAND on the pad
(descend until on the ground) -> DONE.  Contacts with the course and the closest approach
are recorded; the run fails on any course contact.

What this tests: the planner's routes and replanning flown with simple multicopter dynamics
and real collision geometry.  What it does not: VIO, PX4, the skybeta interface, Sky Beta
dynamics.  The pose is simulator truth.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]          # pathfinding/
sys.path.insert(0, str(ROOT))

CLOCK_TOPIC = '/world/dronehacks/clock'
ODOM_TOPIC = '/model/course_quad/odometry'
TWIST_TOPIC = '/course_quad/gazebo/command/twist'
CONTACT_LINKS = ('base_link', 'rotor_0', 'rotor_1', 'rotor_2', 'rotor_3')


def quat_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clip_norm(v, limit):
    n = float(np.linalg.norm(v))
    return v if n <= limit or n == 0 else v * (limit / n)


def build(args):
    """Planner on the published layout, sized for the X3 (radius 0.36 m)."""
    from pathplan.harness import load_published
    from pathplan.online import OnlinePlanner, PlannerConfig
    from pathplan.world import Envelope, GridSpec, checkpoints_from_route, clearance, occupancy
    course, route = load_published()
    spec = GridSpec(resolution=args.resolution)
    env = Envelope(radius=args.radius, tracking_margin=args.tracking_margin)
    prior = occupancy(course, spec, altitude=args.altitude, half_height=env.half_height + env.tracking_margin)
    cps = checkpoints_from_route(route, course)
    pad = cps[-1]
    pad.half_width = .762 - env.radius - env.tracking_margin      # whole X3 inside the pad's outer ring
    planner = OnlinePlanner(prior, spec, env, cps, PlannerConfig(planner=args.planner))
    true_clear = clearance(occupancy(course, spec, altitude=args.altitude, half_height=.03), spec)
    map_clear = clearance(prior, spec)              # the published map's room (allowed: map knowledge)
    return planner, spec, env, true_clear, map_clear


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--planner', default='theta', choices=['theta', 'astar', 'static'])
    ap.add_argument('--altitude', type=float, default=.9)
    ap.add_argument('--radius', type=float, default=.36, help='X3 collision radius (their model: 0.3565 m)')
    ap.add_argument('--tracking-margin', type=float, default=.05)
    ap.add_argument('--resolution', type=float, default=.025,
                    help='planning grid, m (the X3 needs 2.5 cm to fit the chicane; 5 cm closes it)')
    ap.add_argument('--max-speed', type=float, default=.5, help='horizontal speed cap, m/s')
    ap.add_argument('--tight-speed', type=float, default=.15,
                    help='speed cap where the published map leaves no room beyond the keep-out, m/s')
    ap.add_argument('--gain', type=float, default=1.5, help='position P gain, 1/s')
    ap.add_argument('--carrot', type=float, default=.5)
    ap.add_argument('--yaw-gain', type=float, default=1.5)
    ap.add_argument('--max-yaw-rate', type=float, default=.7)
    ap.add_argument('--timeout', type=float, default=240.0, help='simulated seconds')
    ap.add_argument('--depth-topic', default='', help='gz depth image (R_FLOAT32), e.g. /course_quad/depth; '' = map only')
    ap.add_argument('--hfov', type=float, default=1.3962634, help='depth camera horizontal FOV, rad')
    ap.add_argument('--camera-offset', type=float, nargs=3, default=[.25, 0.0, 0.0], help='camera in body FLU, m')
    a = ap.parse_args(argv)

    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.twist_pb2 import Twist
    from gz.transport13 import Node
    from pathplan.ros_node import target_from_command
    from pathplan.depth_projection import CameraMount, Intrinsics, project_depth

    planner, spec, env, true_clear, map_clear = build(a)
    a.output.mkdir(parents=True, exist_ok=False)
    log = open(a.output / 'flight.jsonl', 'w')
    lock = threading.Lock()
    plock = threading.Lock()                 # planner: observe() (depth thread) vs update()
    state = {'clock': None, 'odom': None, 'contacts': []}

    def on_clock(m):
        with lock:
            state['clock'] = m.sim.sec + m.sim.nsec * 1e-9

    def on_odom(m):
        with lock:
            state['odom'] = m

    def on_contacts(m):
        names = [(c.collision1.name, c.collision2.name) for c in m.contact]
        with lock:
            state['contacts'].extend(names)

    node = Node()
    twist_pub = node.advertise(TWIST_TOPIC, Twist)
    node.subscribe(Clock, CLOCK_TOPIC, on_clock)
    node.subscribe(Odometry, ODOM_TOPIC, on_odom)
    for link in CONTACT_LINKS:
        node.subscribe(Contacts, f'/model/course_quad/contact/{link}', on_contacts)
    depth_state = {'frame': None, 'frames': 0, 'observed': 0}
    if a.depth_topic:
        def on_depth(m):
            depth_state['frame'] = (m.width, m.height, m.data)
            depth_state['frames'] += 1
        node.subscribe(Image, a.depth_topic, on_depth)

    def send(v_world, yaw_rate, rot):
        v_body = rot.T @ np.asarray(v_world, float)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = map(float, v_body)
        msg.angular.z = float(yaw_rate)
        twist_pub.publish(msg)

    # planner thread: latest pose in, latest (carrot xy, yaw, mode) out
    shared = {'pose': None, 'target': None, 'mode': 'none', 'plan_ms': []}
    stop = threading.Event()

    def plan_loop():
        while not stop.is_set():
            pose = shared['pose']
            if pose is not None:
                t0 = time.perf_counter()
                with plock:
                    cmd = planner.update(pose[0], pose[1])
                shared['plan_ms'].append(1e3 * (time.perf_counter() - t0))
                shared['target'] = target_from_command(cmd, pose[0], pose[1], a.carrot)
                shared['mode'] = cmd.mode
                if cmd.mode in ('done', 'failed'):
                    return
            time.sleep(.1)

    worker = threading.Thread(target=plan_loop, daemon=True)
    mount = CameraMount(offset=tuple(a.camera_offset))

    def depth_loop():
        # latest depth frame + latest pose -> map cells -> planner (same code as the drone node)
        last = None
        while not stop.is_set():
            frame, pose3 = depth_state['frame'], shared.get('pose3')
            if frame is None or pose3 is None or frame is last:
                time.sleep(.02)
                continue
            last = frame
            w, h, data = frame
            depth = np.frombuffer(data, dtype=np.float32).reshape(h, w).astype(float)
            fx = w / 2 / math.tan(a.hfov / 2)
            intr = Intrinsics(fx, fx, (w - 1) / 2, (h - 1) / 2, w, h)
            pos, rot = pose3
            cells = project_depth(depth, intr, pos, rot, spec, altitude=a.altitude,
                                  half_height=env.half_height + env.tracking_margin, mount=mount,
                                  stride=4, expected=planner.prior)
            with plock:
                planner.observe(*cells)
            depth_state['observed'] += 1

    if a.depth_topic:
        threading.Thread(target=depth_loop, daemon=True).start()
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        with lock:
            ready = state['clock'] is not None and state['odom'] is not None
        if ready and twist_pub.has_connections():
            break
        time.sleep(.05)
    else:
        raise SystemExit('no Gazebo clock / odometry / twist subscriber within 60 s')

    phase, t_start, t_phase = 'TAKEOFF', None, None
    launch_xy = None
    course_contacts, ground_contacts = {}, 0
    min_clear, min_clear_at, path, last_log = math.inf, None, [], -1.0
    last_stamp, result, fault = None, None, None
    t_land_start = None
    print('flying...', flush=True)
    while True:
        with lock:
            now, odom = state['clock'], state['odom']
            contacts, state['contacts'] = state['contacts'], []
        stamp = odom.header.stamp.sec + odom.header.stamp.nsec * 1e-9
        if stamp == last_stamp:
            time.sleep(.003)
            continue
        last_stamp = stamp
        p, q = odom.pose.position, odom.pose.orientation
        pos = np.array([p.x, p.y, p.z])
        rot = quat_matrix(q.x, q.y, q.z, q.w)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
        if t_start is None:
            t_start, t_phase, launch_xy = now, now, pos[:2].copy()
            if math.hypot(pos[0] + 2.5146, pos[1] - .9144) > .2:
                raise SystemExit('vehicle is not on the launch mark: reset the simulation first')
        if now - stamp > .25:
            fault = 'stale_odometry'
        for names in contacts:
            # any obstacle: the course model or generated clutter (not the ground, not ourselves)
            hit = next((n for n in names if 'course_v3' in n or 'clutter_' in n), None)
            if hit:
                course_contacts[hit] = course_contacts.get(hit, 0) + 1
            elif any('ground' in n for n in names):
                ground_contacts += 1
        if phase in ('TAKEOFF', 'NAVIGATE'):
            rc = spec.to_cell(pos[:2])
            c = float(true_clear[rc[0], rc[1]])
            if c < min_clear:
                min_clear, min_clear_at = c, pos[:2].round(3).tolist()
        shared['pose'] = (pos[:2].copy(), yaw)
        shared['pose3'] = (pos.copy(), rot)

        v, yaw_rate = np.zeros(3), 0.0
        if fault or (course_contacts and phase != 'DONE'):
            fault = fault or 'course_contact'
            send(np.zeros(3), 0.0, rot)
            result = 'failed'
            break
        if phase == 'TAKEOFF':
            v = a.gain * (np.array([*launch_xy, a.altitude]) - pos)
            v = clip_norm(v, .4)
            if abs(pos[2] - a.altitude) < .05:
                phase, t_phase = 'NAVIGATE', now
                worker.start()
                print(f'{now - t_start:6.1f} s  NAVIGATE', flush=True)
        elif phase == 'NAVIGATE':
            mode, target = shared['mode'], shared['target']
            if mode == 'done':
                phase, t_land_start = 'LAND', now
                print(f'{now - t_start:6.1f} s  LAND (pad reached)', flush=True)
            elif mode == 'failed':
                fault = 'planner_failed: ' + (planner.events[-1]['event'] if planner.events else '?')
                result = 'failed'
                send(np.zeros(3), 0.0, rot)
                break
            elif target is not None:
                xy, face = target
                # slow down where the published map is tight: tracking error shrinks with speed
                rc = spec.to_cell(pos[:2])
                room = float(np.clip((map_clear[rc[0], rc[1]] - env.hard) / env.soft_margin, 0, 1))
                cap = a.tight_speed + (a.max_speed - a.tight_speed) * room
                v_xy = clip_norm(a.gain * (np.asarray(xy) - pos[:2]), cap)
                v = np.array([v_xy[0], v_xy[1], a.gain * (a.altitude - pos[2])])
                yaw_rate = float(np.clip(a.yaw_gain * wrap(face - yaw), -a.max_yaw_rate, a.max_yaw_rate))
            else:
                v = np.array([0.0, 0.0, a.gain * (a.altitude - pos[2])])
        elif phase == 'LAND':
            v = np.array([0.0, 0.0, -.3])
            if pos[2] < .10 or now - t_land_start > 15:
                phase = 'DONE'
                result = 'completed'
                send(np.zeros(3), 0.0, rot)
                print(f'{now - t_start:6.1f} s  DONE', flush=True)
                break
        send(v, yaw_rate, rot)
        if now - last_log >= .25:
            last_log = now
            path.append([round(float(x), 3) for x in pos])
            log.write(json.dumps({'t': round(now - t_start, 3), 'phase': phase, 'pos': path[-1],
                                  'yaw': round(yaw, 3), 'mode': shared['mode']}) + '\n')
        if now - t_start > a.timeout:
            fault, result = 'timeout', 'failed'
            break

    stop.set()
    send(np.zeros(3), 0.0, rot)
    pad = planner.checkpoints[-1].centre
    land_offset = float(np.hypot(*(pos[:2] - pad)))
    summary = {
        'result': result, 'fault': fault, 'phase': phase,
        'elapsed_sim_s': round(now - t_start, 2),
        'navigate_sim_s': round(now - t_phase, 2) if t_phase else None,
        'checkpoints_cleared': [dict(c) for c in planner.cleared],
        'course_contacts': course_contacts, 'ground_contacts': ground_contacts,
        'min_true_clearance_m': round(min_clear, 3), 'min_clearance_at': min_clear_at,
        'x3_radius_m': a.radius, 'land_offset_from_pad_m': round(land_offset, 3),
        'whole_x3_inside_pad_ring': land_offset + a.radius <= .762,
        'plan_ms_max': round(max(shared['plan_ms'] or [0]), 1),
        'depth_frames_received': depth_state['frames'], 'depth_frames_used': depth_state['observed'],
        'replans': planner.stats['replans'],
        'plan_ms_mean': round(float(np.mean(shared['plan_ms'] or [0])), 1),
        'planner_events': planner.events,
        'settings': {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(a).items()},
    }
    (a.output / 'summary.json').write_text(json.dumps(summary, indent=2, default=float))
    log.close()
    print(json.dumps({k: summary[k] for k in ('result', 'fault', 'elapsed_sim_s', 'course_contacts',
                                              'min_true_clearance_m', 'land_offset_from_pad_m')}), flush=True)
    print('cleared:', [c['name'] for c in planner.cleared], flush=True)
    return 0 if result == 'completed' else 1


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    # gz-transport aborts in its destructors if Python tears it down while its threads run;
    # everything is written by now, so leave without interpreter teardown
    os._exit(code)

"""Competitor window: Jolie's v4c (jolie-flyx-v2 1d44162, 5/5 in FlyX) on the drone.

Starts (1) the organizer's skybeta_flight_interface with the same parameters as the organizer's
competitor example, (2) the team depth relay /oak/stereo/* -> /camera/depth/*, (3) the v4c Sky Beta
program. v4c publishes only /skybeta/setpoint/pose; it never arms, changes mode or lands.
Flow: pilot arms (switch NOT on Position), then switches to Position on the ground -> Offboard -> course
-> holds over the pad -> pilot lands and disarms.
COURSE FRAME: v4c fixes it from the pose when it first becomes healthy (PX4 link up, fresh odometry,
landed, disarmed), NOT at arming. Put the drone on the launch mark facing up the first corridor
BEFORE powering the flight controller, and do not move it afterwards; if it was moved, restart the
competitor window (or the stack) with the drone on the mark.

    fly hard | fly best          # on the Jetson host (jetson/fly): pick the program, live, for now and for the next boot
    ros2 launch jolie_best jolie_best.launch.py                 # boot: the saved choice, else the hardcoded route
    ros2 launch jolie_best jolie_best.launch.py script:=upload_to_flyx_v4c_skybeta.py   # planner v4c explicitly
    ros2 launch jolie_best jolie_best.launch.py shadow:=true    # bench: plans + logs, no setpoints
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PROGRAMS = ("upload_to_flyx_v4c_skybeta_hard.py",   # fixed 0.9 m route through the checkpoints (`fly hard`)
            "upload_to_flyx_v4c_skybeta.py")        # Jolie's planner v4c, FlyX 5/5 (`fly best`)
DEFAULT_PROGRAM = PROGRAMS[0]
CHOICE_FILE = "/root/team_ws/.boot_program"         # written by jetson/fly on the host (~/team_ws/.boot_program)

INTERFACE = {   # identical to the organizer's competitor example / dronehacks_pathplan pathplan.launch.py
    "max_altitude_m": 3.0, "min_altitude_m": 0.3, "max_speed_mps": 1.5, "setpoint_rate_hz": 10.0,
    "allow_mode_request": False, "auto_offboard_on_arm": True,
    "odometry_topic": "/visual_slam/tracking/odometry",
}


def _program(context):
    name = LaunchConfiguration("script").perform(context).strip()
    if not name:                                   # chosen with `fly hard|best` (jetson/fly); else the hard route
        try:
            name = open(CHOICE_FILE).read().strip()
        except OSError:
            name = ""
        name = name if name in PROGRAMS else DEFAULT_PROGRAM
    script = os.path.join(get_package_share_directory("jolie_best"), name)
    off = LaunchConfiguration("camera_offset").perform(context).split()
    cmd = ["python3", "-u", script, "--camera-offset", *off,
           "--depth-topic", LaunchConfiguration("depth_topic").perform(context)]
    if LaunchConfiguration("shadow").perform(context).lower() in ("true", "1", "yes"):
        cmd.append("--shadow")
    os.makedirs("/root/team_ws/runs", exist_ok=True)
    # respawn: v4c exits in DONE / TAKEN_OVER / FAILED (e.g. a health blip while waiting on the ground).
    # A fresh instance waits for healthy + landed + disarmed and the flight interface only enters
    # Offboard after a new disarm -> arm, so a restart can never take an armed vehicle.
    return [ExecuteProcess(cmd=cmd, cwd="/root/team_ws", output="screen", name="jolie_v4c",
                           respawn=True, respawn_delay=3.0, sigterm_timeout="8", sigkill_timeout="12")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("script", default_value="",
                              description="program file in the package share; empty = the choice saved by `fly hard|best` "
                                          f"in {CHOICE_FILE}, else {DEFAULT_PROGRAM}. Options: {', '.join(PROGRAMS)}"),
        DeclareLaunchArgument("shadow", default_value="false", description="true: plan and log only"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("camera_offset", default_value="0.1053 0 0.0304",
                              description="depth camera origin in body FLU, metres (stack TF base_link->oak)"),
        Node(package="skybeta_flight_interface", executable="flight_interface", name="flight_interface",
             output="screen", parameters=[INTERFACE]),
        Node(package="dronehacks_pathplan", executable="depth_relay", name="depth_relay", output="screen"),
        OpaqueFunction(function=_program),
    ])

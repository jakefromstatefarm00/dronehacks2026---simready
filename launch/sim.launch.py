"""SIMULATOR: the drone launch (flight_interface + our node) + a stand-in pilot + the vision bridge
if the simulator lacks one. For the FlyX lab, the Isaac/Pegasus profile, or a Gazebo PX4 SITL.

    ros2 launch dronehacks_pathplan sim.launch.py                         # pilot does Altitude->arm->Position
    ros2 launch dronehacks_pathplan sim.launch.py pilot:=direct           # request Offboard, then arm
    ros2 launch dronehacks_pathplan sim.launch.py pilot:=none             # you fly the pilot part (keyboard RC)
    ros2 launch dronehacks_pathplan sim.launch.py shadow:=true pilot:=none   # plan only

What runs:
  * pathplan.launch.py with use_sim_time:=true and allow_mode_request:=true (sim only)
  * sim_pilot (pilot:=drone_like|direct): arms and lands like the safety pilot. SIM ONLY.
  * Sky Beta's px4_odom_bridge, only if nothing publishes /fmu/in/vehicle_visual_odometry yet
    (vision_bridge:=auto). The FlyX lab runs its own bridge: then this starts nothing extra.
    Never two bridges: PX4 would fuse every vision sample twice.
Never started here: an XRCE agent (the simulator has one), cuVSLAM (the simulator has one).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

PASS_THROUGH = [            # forwarded to pathplan.launch.py
    ('depth_topic', '/camera/depth/image_raw', 'FlyX lab depth; empty = published map only (like the stock drone)'),
    ('camera_info_topic', '', 'empty = sibling camera_info'),
    ('shadow', 'false', 'plan only'),
    ('planner', 'theta', 'theta | astar | static'),
    ('flight_altitude_m', '0.9', 'cruise height'),
    ('carrot_m', '0.5', 'carrot distance'),
    ('max_flight_s', '175.0', 'run limit (sim time)'),
    ('launch_xy_m', '[-2.515, 0.914]', 'launch mark, course frame'),
    ('launch_heading_course_rad', '1.5707963', 'heading on the mark'),
    ('frame_latch', 'arm', 'arm | slam_origin'),
    ('use_sim_time', 'true', 'the simulator publishes /clock'),
    ('allow_mode_request', 'true', 'SIM ONLY: lets pilot:=direct request Offboard'),
    ('odometry_topic', '/visual_slam/tracking/odometry', 'VIO odometry'),
]


def _publishers(topic, wait_s=3.0):
    """How many publishers ``topic`` has right now (own rclpy context, discovery wait)."""
    import time
    import rclpy
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    node = rclpy.create_node('dronehacks_probe', context=ctx)
    try:
        end, n = time.monotonic() + wait_s, 0
        while time.monotonic() < end:
            n = node.count_publishers(topic)
            if n:
                break
            time.sleep(.2)
        return n
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)


def _bridge(context):
    mode = LaunchConfiguration('vision_bridge').perform(context).lower()
    if mode == 'false':
        return []
    if mode == 'auto':
        n = _publishers('/fmu/in/vehicle_visual_odometry')
        if n:
            print(f'[sim.launch] /fmu/in/vehicle_visual_odometry already has {n} publisher(s): '
                  'the simulator bridges vision itself; not starting px4_odom_bridge.')
            return []
        print('[sim.launch] nothing publishes /fmu/in/vehicle_visual_odometry: starting Sky Beta px4_odom_bridge.')
    return [Node(package='oakd_vslam_bringup', executable='px4_odom_bridge', name='px4_odom_bridge',
                 output='screen', parameters=[{
                     'input_topic': LaunchConfiguration('odometry_topic').perform(context),
                     'status_topic': '', 'use_sim_time': True}])]


def _pilot(context):
    mode = LaunchConfiguration('pilot').perform(context).lower()
    if mode == 'none':
        print('[sim.launch] pilot:=none - you are the pilot: arm in Altitude, switch to Position; land at LOW_HOLD.')
        return []
    return [Node(package='dronehacks_pathplan', executable='sim_pilot', name='sim_pilot', output='screen',
                 parameters=[{'sim_only': True, 'mode': mode,
                              'start_delay_s': float(LaunchConfiguration('pilot_delay_s').perform(context)),
                              'use_sim_time': False}])]


def generate_launch_description():
    declared = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in PASS_THROUGH]
    declared += [
        DeclareLaunchArgument('pilot', default_value='drone_like',
                              description='drone_like (Altitude -> arm -> Position) | direct | none'),
        DeclareLaunchArgument('pilot_delay_s', default_value='3.0', description='READY this long before arming'),
        DeclareLaunchArgument('vision_bridge', default_value='auto', description='auto | true | false'),
    ]
    drone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('dronehacks_pathplan'), 'launch', 'pathplan.launch.py'])),
        launch_arguments={n: LaunchConfiguration(n) for n, _, _ in PASS_THROUGH}.items())
    return LaunchDescription(declared + [OpaqueFunction(function=_bridge), drone, OpaqueFunction(function=_pilot)])

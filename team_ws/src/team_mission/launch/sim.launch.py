"""SIMULATION ONLY (Gazebo / flyx lab): the whole mission chain with stand-ins for the pilot and,
optionally, for cuVSLAM. Start PX4 SITL + the simulator + MicroXRCEAgent first.

    # simulator has no cuVSLAM: derive odometry from PX4 (frame-safe; course checkpoints need
    # launch_yaw_deg:=0 and launch_x/y = spawn point)
    ros2 launch team_mission sim.launch.py odom_source:=px4 checkpoints:=test_line.yaml

    # simulator publishes its own VIO / cuVSLAM odometry on some topic (e.g. the team's Gazebo
    # vio_emulator, or the flyx lab's cuVSLAM wrapper):
    ros2 launch team_mission sim.launch.py odom_source:=topic odom_topic:=/gazebo/vio_odom \
        checkpoints:=course_v3.yaml launch_x:=-2.515 launch_y:=0.914 launch_yaw_deg:=90

    pilot:=handshake (arm in Altitude, switch to Position: the real procedure)
    pilot:=direct    (arm in Hold, flight_interface /skybeta/request_offboard)
    pilot:=none      (you are the pilot, e.g. QGroundControl with the virtual joystick)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _all(context):
    share = get_package_share_directory('team_mission')
    g = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    source, pilot = g('odom_source'), g('pilot')
    odom = '/visual_slam/tracking/odometry' if source == 'px4' else g('odom_topic')
    launch_args = {'use_sim_time': 'true', 'require_vslam_status': 'false', 'odom_topic': odom,
                   'allow_mode_request': 'true' if pilot == 'direct' else 'false',
                   'checkpoints': g('checkpoints'), 'policy': g('policy')}
    for n in ('launch_x', 'launch_y', 'launch_z', 'launch_yaw_deg'):
        if g(n) != '':
            launch_args[n] = g(n)
    out = []
    if source == 'px4':
        out.append(Node(package='team_mission', executable='sim_odom_adapter', output='screen',
                        parameters=[{'use_sim_time': True, 'out_topic': odom}]))
    out.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'mission.launch.py')),
                                        launch_arguments=launch_args.items()))
    if pilot in ('handshake', 'direct'):
        out.append(Node(package='team_mission', executable='sim_pilot', output='screen',
                        parameters=[{'use_sim_time': True, 'mode': pilot}]))
    return out


def generate_launch_description():
    args = [DeclareLaunchArgument('odom_source', default_value='px4', description='px4 | topic'),
            DeclareLaunchArgument('odom_topic', default_value='/visual_slam/tracking/odometry'),
            DeclareLaunchArgument('pilot', default_value='handshake', description='handshake | direct | none'),
            DeclareLaunchArgument('checkpoints', default_value='test_line.yaml'),
            DeclareLaunchArgument('policy', default_value='carrot')]
    args += [DeclareLaunchArgument(n, default_value='') for n in ('launch_x', 'launch_y', 'launch_z', 'launch_yaw_deg')]
    return LaunchDescription(args + [OpaqueFunction(function=_all)])

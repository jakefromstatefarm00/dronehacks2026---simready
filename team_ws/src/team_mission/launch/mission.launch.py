"""What the stack's `competitor` window starts: flight_interface + team_mission.

On the drone, in /etc/skybeta/drone.env:
    COMPETITOR_PKG=team_mission
    COMPETITOR_LAUNCH=mission.launch.py
then restart the stack. Or by hand inside the container:
    ros2 launch team_mission mission.launch.py checkpoints:=test_line.yaml policy:=carrot

This replaces competitor_example/competitor.launch.py: the same flight_interface with the same
settings (it owns the heartbeat, the arming handshake and the safety envelope), and our node
instead of square_mission.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

OVERRIDES = ['checkpoints', 'policy', 'launch_x', 'launch_y', 'launch_z', 'launch_yaw_deg',
             'require_vslam_status', 'odom_topic', 'max_xy_speed']


def _nodes(context):
    share = get_package_share_directory('team_mission')
    cfg = LaunchConfiguration('config').perform(context) or os.path.join(share, 'config', 'mission.yaml')
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    overrides = {}
    for n in OVERRIDES:
        v = LaunchConfiguration(n).perform(context)
        if v != '':
            overrides[n] = v
    odom = overrides.get('odom_topic', '/visual_slam/tracking/odometry')
    fi_params = {
        'max_altitude_m': 3.0, 'min_altitude_m': 0.3, 'max_speed_mps': 1.5, 'setpoint_rate_hz': 10.0,
        'auto_offboard_on_arm': True, 'odometry_topic': odom, 'use_sim_time': sim,
        'allow_mode_request': LaunchConfiguration('allow_mode_request').perform(context).lower() == 'true',
    }
    nodes = [Node(package='team_mission', executable='mission_node', name='team_mission', output='screen',
                  emulate_tty=True, parameters=[cfg, overrides, {'use_sim_time': sim}])]
    if LaunchConfiguration('flight_interface').perform(context).lower() == 'true':
        nodes.insert(0, Node(package='skybeta_flight_interface', executable='flight_interface',
                             name='flight_interface', output='screen', emulate_tty=True, parameters=[fi_params]))
    return nodes


def generate_launch_description():
    args = [DeclareLaunchArgument('config', default_value='', description='parameter file (default config/mission.yaml)'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('flight_interface', default_value='true',
                                  description='also start skybeta flight_interface (false if it already runs)'),
            DeclareLaunchArgument('allow_mode_request', default_value='false',
                                  description='SIMULATION ONLY: let sim_pilot mode:=direct request Offboard')]
    args += [DeclareLaunchArgument(n, default_value='', description=f'override {n}') for n in OVERRIDES]
    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])

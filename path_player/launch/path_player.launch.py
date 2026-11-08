"""Launch path_player alongside skybeta_flight_interface.

Usage on drone (/etc/skybeta/drone.env):
    COMPETITOR_PKG=path_player
    COMPETITOR_LAUNCH=path_player.launch.py

Manual run:
    ros2 launch path_player path_player.launch.py
    ros2 launch path_player path_player.launch.py speed_factor:=0.8 mode:=adaptive
    ros2 launch path_player path_player.launch.py interface:=false recording_path:=/path/to/recording
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

INTERFACE_PARAMS = {
    'max_altitude_m': 3.0,
    'min_altitude_m': 0.3,
    'max_speed_mps': 1.5,
    'setpoint_rate_hz': 20.0,
    'allow_mode_request': False,
    'auto_offboard_on_arm': True,
    'odometry_topic': '/visual_slam/tracking/odometry',
}


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='true',
                              description='start skybeta_flight_interface (set false if already running)'),
        DeclareLaunchArgument('recording_path', default_value='',
                              description='explicit path to recording folder; empty = newest in runs dir'),
        DeclareLaunchArgument('mode', default_value='adaptive',
                              description='playback mode: adaptive (lag-capped time), time, or waypoint'),
        DeclareLaunchArgument('speed_factor', default_value='1.0',
                              description='playback speed multiplier (e.g. 0.8 for 80% speed)'),
        DeclareLaunchArgument('arrival_radius_m', default_value='0.25',
                              description='waypoint arrival tolerance in metres'),
        DeclareLaunchArgument('max_lag_m', default_value='0.5',
                              description='max lag before adaptive playback pauses time to let drone catch up'),
        DeclareLaunchArgument('command_yaw', default_value='true',
                              description='whether to command yaw heading from recording'),
        DeclareLaunchArgument('anchor_mode', default_value='armed',
                              description='anchor source: armed (at arming), offboard, recorded, or fixed'),

        Node(
            package='skybeta_flight_interface',
            executable='flight_interface',
            name='flight_interface',
            output='screen',
            parameters=[INTERFACE_PARAMS],
            condition=IfCondition(LaunchConfiguration('interface')),
        ),

        Node(
            package='path_player',
            executable='player',
            name='path_player',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'recording_path': LaunchConfiguration('recording_path'),
                'mode': LaunchConfiguration('mode'),
                'speed_factor': LaunchConfiguration('speed_factor'),
                'arrival_radius_m': LaunchConfiguration('arrival_radius_m'),
                'max_lag_m': LaunchConfiguration('max_lag_m'),
                'command_yaw': LaunchConfiguration('command_yaw'),
                'anchor_mode': LaunchConfiguration('anchor_mode'),
            }],
        ),
    ])

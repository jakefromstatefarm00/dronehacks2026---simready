"""Boot program for RECORDING days: flight_interface (safe: no setpoints ever) + the path recorder.

On the drone, /etc/skybeta/drone.env:
    COMPETITOR_PKG=path_recorder
    COMPETITOR_LAUNCH=path_recorder.launch.py
then every power-on records each arm -> disarm, and the pilot flies on RC as usual.

    ros2 launch path_recorder path_recorder.launch.py                     # interface + recorder (boot)
    ros2 launch path_recorder path_recorder.launch.py interface:=false    # recorder only (another
                                                                          # flight_interface already runs)

Why flight_interface runs at all: it publishes /skybeta/state (armed / landed / mode), which starts
and stops the recordings, and it is the organisers' safety layer. Because nothing here publishes
/skybeta/setpoint/*, its auto-Offboard always refuses ("no team setpoint received"): flipping to
Position keeps the drone in the pilot's Position mode. Its parameters are identical to the
organisers' competitor example.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

INTERFACE = {   # identical to the organizer's competitor example (and jolie_best / dronehacks_pathplan)
    'max_altitude_m': 3.0, 'min_altitude_m': 0.3, 'max_speed_mps': 1.5, 'setpoint_rate_hz': 10.0,
    'allow_mode_request': False, 'auto_offboard_on_arm': True,
    'odometry_topic': '/visual_slam/tracking/odometry',
}


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='true',
                              description='also start skybeta flight_interface (false if one already runs)'),
        DeclareLaunchArgument('spacing_m', default_value='0.15', description='point every N metres'),
        DeclareLaunchArgument('turn_deg', default_value='15.0', description='and every N degrees of turn'),
        DeclareLaunchArgument('output_dir', default_value='', description='empty = <team_ws>/runs/paths'),
        Node(package='skybeta_flight_interface', executable='flight_interface', name='flight_interface',
             output='screen', parameters=[INTERFACE], condition=IfCondition(LaunchConfiguration('interface'))),
        Node(package='path_recorder', executable='recorder', name='path_recorder', output='screen',
             respawn=True, respawn_delay=2.0,
             parameters=[{'spacing_m': LaunchConfiguration('spacing_m'), 'turn_deg': LaunchConfiguration('turn_deg'),
                          'output_dir': LaunchConfiguration('output_dir')}]),
    ])

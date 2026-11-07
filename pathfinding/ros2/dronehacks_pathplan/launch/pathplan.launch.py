"""Launch the flight interface plus our path-planner node.

Forked from competitor_example/launch/competitor.launch.py: ``flight_interface`` is started
exactly as there (it owns the Offboard heartbeat and the safety envelope); only the competitor
node changed.

    ros2 launch dronehacks_pathplan pathplan.launch.py
    ros2 launch dronehacks_pathplan pathplan.launch.py depth_topic:=/oak/stereo/depth shadow:=true
    ros2 launch dronehacks_pathplan pathplan.launch.py allow_mode_request:=true   # SIM ONLY
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    # flight_interface (defaults identical to competitor.launch.py)
    ('max_altitude_m', '3.0', 'ceiling above the odometry origin, metres'),
    ('min_altitude_m', '0.3', 'floor above the odometry origin, metres'),
    ('max_speed_mps', '1.5', 'cap on the norm of a twist setpoint, m/s'),
    ('setpoint_rate_hz', '10.0', 'OffboardControlMode heartbeat rate -- PX4 needs > 2 Hz'),
    ('allow_mode_request', 'false',
     'enable /skybeta/request_offboard. SIM ONLY: on a real vehicle Offboard follows the pilot '
     'arming, then switching to Position'),
    ('auto_offboard_on_arm', 'true',
     'enter Offboard once when the pilot arms, then switches to Position'),
    ('odometry_topic', '/visual_slam/tracking/odometry',
     'source of the ENU pose the altitude band is measured against'),
    # our node
    ('flight_altitude_m', '0.9', 'cruise height above the floor, metres (walls are 1.73 m)'),
    ('carrot_m', '0.5', 'setpoint distance ahead on the plan, metres'),
    ('planner', 'theta', 'theta | astar | static (published map only)'),
    ('depth_topic', "''", 'depth sensor_msgs/Image; empty = published map only'),
    ('camera_info_topic', "''", 'CameraInfo of the depth image; empty = sibling camera_info'),
    ('shadow', 'false', 'true: plan and publish ~/path only, no setpoints (Offboard will not start)'),
    ('pathfinding_path', "''", '<repo>/pathfinding if the node cannot find it from its own checkout'),
    ('competitor', 'true', 'also start our node'),
]


def generate_launch_description():
    declared = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGUMENTS]
    flight_interface = Node(
        package='skybeta_flight_interface',
        executable='flight_interface',
        name='flight_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{k: LaunchConfiguration(k) for k in (
            'max_altitude_m', 'min_altitude_m', 'max_speed_mps', 'setpoint_rate_hz',
            'allow_mode_request', 'auto_offboard_on_arm', 'odometry_topic')}],
    )
    competitor = Node(
        package='dronehacks_pathplan',
        executable='pathplan_mission',
        name='dronehacks_pathplan',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('competitor')),
        parameters=[{k: LaunchConfiguration(k) for k in (
            'flight_altitude_m', 'carrot_m', 'planner', 'depth_topic', 'camera_info_topic',
            'shadow', 'min_altitude_m')}],
        # read before the node starts (it forks the planner process first): not a ROS parameter
        additional_env={'DRONEHACKS_PATHFINDING': LaunchConfiguration('pathfinding_path')},
    )
    return LaunchDescription(declared + [flight_interface, competitor])

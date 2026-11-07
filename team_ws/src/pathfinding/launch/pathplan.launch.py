"""THE DRONE LAUNCH: flight_interface + our path-planning node. The competitor window runs this.

On the drone, /etc/skybeta/drone.env:
    COMPETITOR_PKG=dronehacks_pathplan
    COMPETITOR_LAUNCH=pathplan.launch.py
and the stack's competitor window builds ~/team_ws and runs ``ros2 launch dronehacks_pathplan
pathplan.launch.py``. By hand (after stopping the competitor window):

    ros2 launch dronehacks_pathplan pathplan.launch.py                 # fly (the pilot arms)
    ros2 launch dronehacks_pathplan pathplan.launch.py shadow:=true    # plan only, never fly

``flight_interface`` is started exactly as the organisers' competitor.launch.py starts it (same
arguments, same defaults: auto_offboard_on_arm true, allow_mode_request false); only the
competitor node is ours. In a simulator use sim.launch.py, which includes this file.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# flight_interface: identical to competitor_example/launch/competitor.launch.py
INTERFACE_ARGS = [
    ('max_altitude_m', '3.0', 'ceiling above the odometry origin, metres'),
    ('min_altitude_m', '0.3', 'floor above the odometry origin, metres (our node descends to it at the end)'),
    ('max_speed_mps', '1.5', 'cap on the norm of a twist setpoint, m/s'),
    ('setpoint_rate_hz', '10.0', 'OffboardControlMode heartbeat rate -- PX4 needs > 2 Hz'),
    ('allow_mode_request', 'false',
     'enable /skybeta/request_offboard. SIM ONLY: on a real vehicle Offboard follows the pilot '
     'arming, then switching to Position'),
    ('auto_offboard_on_arm', 'true', 'enter Offboard once when the pilot arms, then switches to Position'),
    ('odometry_topic', '/visual_slam/tracking/odometry', 'source of the ENU pose'),
]
# our node (strings are passed as strings; numbers/bools/lists are parsed)
NODE_ARGS = [
    ('shadow', 'false', 'true: plan and log only, never publish a setpoint (Offboard cannot start)', None),
    ('flight_altitude_m', '0.9', 'cruise height above the floor (walls are 1.73 m)', None),
    ('carrot_m', '0.5', 'setpoint distance ahead on the plan, metres', None),
    ('planner', 'theta', 'theta | astar | static (published map only)', str),
    ('depth_topic', '', 'depth sensor_msgs/Image; empty = published map only (the stock drone has none)', str),
    ('camera_info_topic', '', 'CameraInfo of the depth image; empty = sibling camera_info', str),
    ('camera_offset_flu_m', '[0.115, 0.0, 0.029]', 'depth camera origin in body FLU (drone.env CAMERA_X/Y/Z)', None),
    ('launch_xy_m', '[-2.515, 0.914]', 'launch mark, course frame', None),
    ('launch_heading_course_rad', '1.5707963', 'heading on the mark, course frame (up the first corridor)', None),
    ('launch_height_m', '0.05', 'body origin above the floor on the mark', None),
    ('frame_latch', 'arm', 'arm: course frame from the pose when the pilot arms | slam_origin', str),
    ('max_flight_s', '175.0', 'run limit from takeoff, then descend and hold for the pilot', None),
    ('yaw_rate_max', '0.25', 'rad/s cap on the commanded heading', None),
    ('journal_dir', '', 'where runs go; empty = <team_ws>/runs', str),
    ('planner_process', 'true', 'planner in its own process', None),
    ('use_sim_time', 'false', 'true in a simulator that publishes /clock (both nodes)', None),
    ('competitor', 'true', 'false: flight_interface only', None),
]


def _value(name, vtype):
    lc = LaunchConfiguration(name)
    return ParameterValue(lc, value_type=vtype) if vtype else lc


def generate_launch_description():
    declared = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in INTERFACE_ARGS]
    declared += [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h, _ in NODE_ARGS]
    flight_interface = Node(
        package='skybeta_flight_interface',
        executable='flight_interface',
        name='flight_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{**{n: LaunchConfiguration(n) for n, _, _ in INTERFACE_ARGS},
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    ours = [(n, t) for n, _, _, t in NODE_ARGS if n != 'competitor']
    competitor = Node(
        package='dronehacks_pathplan',
        executable='pathplan_node',
        name='dronehacks_pathplan',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('competitor')),
        parameters=[{**{n: _value(n, t) for n, t in ours},
                     'min_altitude_m': LaunchConfiguration('min_altitude_m'),     # the same floor as the interface
                     'odometry_topic': ParameterValue(LaunchConfiguration('odometry_topic'), value_type=str)}],
    )
    return LaunchDescription(declared + [flight_interface, competitor])

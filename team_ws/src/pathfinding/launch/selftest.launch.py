"""SELF-TEST, NO SIMULATOR: the real flight_interface + our node + the sim pilot against a fake PX4.

    ros2 launch dronehacks_pathplan selftest.launch.py                 # ~1.5 min, ends by itself
    ros2 launch dronehacks_pathplan selftest.launch.py depth:=false    # without the fake depth camera

Checks the ROS side end to end: topic names (versioned /fmu topics), QoS, frames, the
arm -> Position -> Offboard handshake through flight_interface's real auto-offboard, the mission
through all checkpoints, LOW_HOLD -> the pilot's Land -> disarm -> DONE. Read the summary at the
end ("RESULT: ...") and the journal path it prints.

NEVER run this on the drone or beside a simulator: fake_px4 pretends to be PX4 and would
fight the real one (it refuses nothing). Use a separate ROS_DOMAIN_ID if anything else is up.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    depth = LaunchConfiguration('depth')
    fake = Node(package='dronehacks_pathplan', executable='fake_px4', name='fake_px4', output='screen',
                parameters=[{'depth': depth, 'log_file': LaunchConfiguration('fake_log')}])
    drone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('dronehacks_pathplan'), 'launch', 'pathplan.launch.py'])),
        launch_arguments={
            'allow_mode_request': 'true',
            'depth_topic': PythonExpression(["'/camera/depth/image_raw' if '", depth, "'.lower() == 'true' else ''"]),
            'journal_dir': LaunchConfiguration('journal_dir'),
        }.items())
    pilot = Node(package='dronehacks_pathplan', executable='sim_pilot', name='sim_pilot', output='screen',
                 parameters=[{'sim_only': True, 'mode': LaunchConfiguration('pilot'), 'start_delay_s': 2.0,
                              'exit_when_done': True}])
    stop = RegisterEventHandler(OnProcessExit(target_action=pilot, on_exit=[EmitEvent(event=Shutdown(
        reason='self-test flight finished'))]))
    return LaunchDescription([
        DeclareLaunchArgument('depth', default_value='true', description='fake depth camera on/off'),
        DeclareLaunchArgument('pilot', default_value='drone_like', description='drone_like | direct'),
        DeclareLaunchArgument('journal_dir', default_value='', description='empty = <team_ws>/runs'),
        DeclareLaunchArgument('fake_log', default_value='/tmp/fake_px4.json', description='fake PX4 log'),
        fake, drone, pilot, stop,
    ])

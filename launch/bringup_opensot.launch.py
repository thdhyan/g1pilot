from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os
import sys

def generate_launch_description():
    if not os.environ.get("G1_INTERFACE"):
        sys.exit("ERROR: G1_INTERFACE environment variable is not set.\n"
                 "Set it to your network interface, e.g.: export G1_INTERFACE=eno2")

    pkg1_share = FindPackageShare('g1pilot').find('g1pilot')

    interface = LaunchConfiguration("interface")

    navigation_launcher = os.path.join(pkg1_share, 'launch', 'navigation_launcher.launch.py')
    robot_state_launcher = os.path.join(pkg1_share, 'launch', 'robot_state_launcher.launch.py')
    teleoperation_launcher = os.path.join(pkg1_share, 'launch', 'teleoperation_launcher.launch.py')
    manipulation_launcher = os.path.join(pkg1_share, 'launch', 'manipulation_launcher.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument("enable_collision_avoidance", default_value="true"),
        DeclareLaunchArgument(
            "interface",
            default_value=EnvironmentVariable("G1_INTERFACE"),
            description="Network interface for Unitree SDK",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launcher),
            launch_arguments=[("interface", interface)],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_launcher),
            launch_arguments={
                'interface': interface,
                'publish_joint_states': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleoperation_launcher),
            launch_arguments=[("interface", interface)],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manipulation_launcher),
            launch_arguments={
                'interface': interface,
                'enable_collision_avoidance': LaunchConfiguration('enable_collision_avoidance'),
                'send_cmds_to_robot': 'false',
                'publish_joint_states_opensot': 'true',
            }.items(),
        ),
    ])

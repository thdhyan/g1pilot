from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from g1pilot.utils.extract_configuration import extract_configuration

def generate_launch_description():
    interface = LaunchConfiguration("interface")
    use_robot = LaunchConfiguration("use_robot")
    arm_controlled = LaunchConfiguration("arm_controlled")
    enable_arm_ui = LaunchConfiguration("enable_arm_ui")
    ik_use_waist = LaunchConfiguration("ik_use_waist")
    ik_alpha = LaunchConfiguration("ik_alpha")
    ik_max_dq_step = LaunchConfiguration("ik_max_dq_step")
    arm_velocity_limit = LaunchConfiguration("arm_velocity_limit")

    configuration = extract_configuration()

    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value = configuration['general']['interface']),
        DeclareLaunchArgument("use_robot", default_value = configuration['general']['use_robot']),
        DeclareLaunchArgument("arm_controlled", default_value = configuration['general']['arm_controlled']),
        DeclareLaunchArgument("enable_arm_ui", default_value = configuration['general']['enable_arm_ui']),
        DeclareLaunchArgument("ik_use_waist", default_value = configuration['general']['use_waist']),
        DeclareLaunchArgument("ik_alpha", default_value = configuration['inverse_kinematics']['alpha']),
        DeclareLaunchArgument("ik_max_dq_step", default_value = configuration['inverse_kinematics']['max_dq_step']),
        DeclareLaunchArgument("arm_velocity_limit", default_value = configuration['inverse_kinematics']['velocity_limit']),

        Node(
            package='g1pilot',
            executable='arm_controller',
            name='arm_controller',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
            }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='dx3_controller',
            name='dx3_controller',
            parameters=[{
                'arm_controlled': ParameterValue(LaunchConfiguration("arm_controlled"), value_type=str),
                'interface': ParameterValue(LaunchConfiguration("interface"), value_type=str)
            }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='interactive_marker',
            name='interactive_marker',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
            }],
            output='screen'
        ),
    ])

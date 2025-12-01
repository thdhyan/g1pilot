from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from g1pilot.utils.extract_configuration import extract_configuration

def generate_launch_description():

    joystick_name = LaunchConfiguration("joystick_name")

    configuration = extract_configuration()

    return LaunchDescription([
        DeclareLaunchArgument("joystick_name", default_value = configuration['general']['joy_name'],
                              description = 'Name of the wireless control'),

        Node(
            package='g1pilot',
            executable='joystick',
            name='joystick',
            output='screen',
            parameters=[
                {'joystick_name': joystick_name,
                 }],
        ),

        Node(
            package='g1pilot',
            executable='joy_mux',
            name='joy_mux',
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='ui_interface',
            name='ui_interface',
            output='screen'
        ),

    ])

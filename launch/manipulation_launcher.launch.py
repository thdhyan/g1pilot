from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
import sys

def generate_launch_description():
    if not os.environ.get("G1_INTERFACE"):
        sys.exit("ERROR: G1_INTERFACE environment variable is not set.\n"
                 "Set it to your network interface, e.g.: export G1_INTERFACE=eno2")
    interface = LaunchConfiguration("interface")
    use_robot = LaunchConfiguration("use_robot")
    arm_controlled = LaunchConfiguration("arm_controlled")
    enable_arm_ui = LaunchConfiguration("enable_arm_ui")
    ik_use_waist = LaunchConfiguration("ik_use_waist")
    ik_alpha = LaunchConfiguration("ik_alpha")
    ik_max_dq_step = LaunchConfiguration("ik_max_dq_step")
    arm_velocity_limit = LaunchConfiguration("arm_velocity_limit")
    enable_collision_avoidance = LaunchConfiguration("enable_collision_avoidance")
    enable_external_collision_avoidance = LaunchConfiguration("enable_external_collision_avoidance")
    box_pose_topic = LaunchConfiguration("box_pose_topic")
    send_cmds_to_robot = LaunchConfiguration("send_cmds_to_robot")
    publish_joint_states_opensot = LaunchConfiguration("publish_joint_states_opensot")
    use_whole_body = LaunchConfiguration("use_whole_body")
    right_hand_frame_ref = LaunchConfiguration("right_hand_frame_ref")
    left_hand_frame_ref = LaunchConfiguration("left_hand_frame_ref")
    hand_type = LaunchConfiguration("hand_type")
    enable_tts = LaunchConfiguration("enable_tts")
    tts_topic = LaunchConfiguration("tts_topic")

    # Which dexterous hand controller to launch: "dx3" (Unitree Dex3) or "brainco" (Revo2).
    is_dx3 = IfCondition(PythonExpression(["'", hand_type, "' == 'dx3'"]))
    is_brainco = IfCondition(PythonExpression(["'", hand_type, "' == 'brainco'"]))

    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value=EnvironmentVariable("G1_INTERFACE")),
        DeclareLaunchArgument("use_robot", default_value="true"),
        DeclareLaunchArgument("arm_controlled", default_value="both"),
        DeclareLaunchArgument("enable_arm_ui", default_value="true"),
        DeclareLaunchArgument("ik_use_waist", default_value="false"),
        DeclareLaunchArgument("ik_alpha", default_value="0.2"),
        DeclareLaunchArgument("ik_max_dq_step", default_value="0.05"),
        DeclareLaunchArgument("arm_velocity_limit", default_value="2.0"),
        DeclareLaunchArgument("enable_collision_avoidance", default_value="false"),
        DeclareLaunchArgument("enable_external_collision_avoidance", default_value="false"),
        DeclareLaunchArgument("box_pose_topic", default_value="/g1pilot/box_pose"),
        DeclareLaunchArgument("send_cmds_to_robot", default_value="true"),
        DeclareLaunchArgument("publish_joint_states_opensot", default_value="false"),
        DeclareLaunchArgument("use_whole_body", default_value="false"),
        DeclareLaunchArgument("right_hand_frame_ref", default_value="pelvis"),
        DeclareLaunchArgument("left_hand_frame_ref", default_value="pelvis"),
        DeclareLaunchArgument("hand_type", default_value="dx3",
                              description="Dexterous hand controller to launch: 'dx3' or 'brainco'."),
        DeclareLaunchArgument("enable_tts", default_value="true",
                              description="Launch the text-to-speech node."),
        DeclareLaunchArgument("tts_topic", default_value="/g1pilot/say",
                              description="Topic the TTS node speaks (std_msgs/String)."),

        Node(
            package='g1pilot',
            executable='opensot_solver',
            name='opensot_solver',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'enable_collision_avoidance': ParameterValue(enable_collision_avoidance, value_type=bool),
                'enable_external_collision_avoidance': ParameterValue(enable_external_collision_avoidance, value_type=bool),
                'box_pose_topic': box_pose_topic,
                'send_cmds_to_robot': ParameterValue(send_cmds_to_robot, value_type=bool),
                'publish_joint_states_opensot': ParameterValue(publish_joint_states_opensot, value_type=bool),
                'use_whole_body': ParameterValue(use_whole_body, value_type=bool),
                'right_hand_frame_ref': right_hand_frame_ref,
                'left_hand_frame_ref': left_hand_frame_ref,
            }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='dx3_controller',
            name='dx3_controller',
            condition=is_dx3,
            parameters=[{
                'arm_controlled': ParameterValue(LaunchConfiguration("arm_controlled"), value_type=str),
                'interface': ParameterValue(LaunchConfiguration("interface"), value_type=str),
                'use_robot': ParameterValue(use_robot, value_type=bool),
            }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='brainco_controller',
            name='brainco_controller',
            condition=is_brainco,
            parameters=[{
                'arm_controlled': ParameterValue(LaunchConfiguration("arm_controlled"), value_type=str),
                'interface': ParameterValue(LaunchConfiguration("interface"), value_type=str),
                'use_robot': ParameterValue(use_robot, value_type=bool),
            }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='tts_node',
            name='tts_node',
            condition=IfCondition(enable_tts),
            parameters=[{
                'interface': interface,
                'topic': tts_topic,
            }],
            output='screen'
        ),
    ])

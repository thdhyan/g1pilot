#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PointStamped
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from astroviz_interfaces.msg import MotorState, MotorStateList

# BrainCo Revo2: 6 fingers, driven through unitreerobotics/brainco_hand_service
# (a serial<->DDS bridge). The hands hang off one RS485 bus (FTDI FT4232H),
# addressed by id (L=0x7e, R=0x7f) inside the service; here we only speak DDS.
#
# Command channel "rt/brainco/<side>/cmd" carries unitree_go::MotorCmds_ with 6
# entries, finger order: [thumb, thumb_aux, index, middle, ring, pinky].
#   cmds[i].q  -> normalized position, 0.0 = open .. 1.0 = closed
#   cmds[i].dq -> normalized speed,    0.0 = stop .. 1.0 = full speed
# Speed must be > 0 or the finger will not move. These targets are placeholders
# to tune against the real hand.
NUM_FINGERS = 6
CLOSE_VALUES_1 = [1.0, 1.0, 0.0, 1.0, 1.0, 1.0]  # pinch: thumb + index
CLOSE_VALUES_2 = [1.0, 1.0, 1.0, 1.0, 1.0, 0.0]  # close 4 fingers, thumb stays open
OPEN_VALUES    = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]

# Per-pose finger speeds (normalized 0..1, must be > 0 to move), matched index-for-index
# to the position vectors above. Tune per finger/pose as needed.
CLOSE_SPEED_1  = [0.2, 0.5, 1.0, 1.0, 1.0, 1.0]
CLOSE_SPEED_2  = [0.2, 0.5, 1.0, 1.0, 1.0, 1.0]
OPEN_SPEED     = [1.0, 1.0, 0.4, 0.4, 0.4, 0.4]


class BrainCoController(Node):
    def __init__(self):
        super().__init__('brainco_hand_controller')
        self.declare_parameter("interface", "")
        self.declare_parameter("arm_controlled", "both")
        self.declare_parameter("use_robot", True)
        # DDS channel prefix exposed by brainco_hand_service. cmd/state topics
        # are "<prefix>/<side>/cmd" and "<prefix>/<side>/state".
        self.declare_parameter("topic_prefix", "rt/brainco")
        interface = self.get_parameter("interface").get_parameter_value().string_value
        arm_controlled = self.get_parameter("arm_controlled").get_parameter_value().string_value
        topic_prefix = self.get_parameter("topic_prefix").get_parameter_value().string_value
        self.use_robot = bool(self.get_parameter("use_robot").value)
        self.left_gripper_state_publisher = self.create_publisher(MotorStateList, 'g1pilot/brainco/left/motor_state', QoSProfile(depth=10))
        self.right_gripper_state_publisher = self.create_publisher(MotorStateList, 'g1pilot/brainco/right/motor_state', QoSProfile(depth=10))

        self.right_action = None
        self.left_action = None
        self.right_target = OPEN_VALUES
        self.left_target = OPEN_VALUES
        self.right_speed = OPEN_SPEED
        self.left_speed = OPEN_SPEED
        self.send_commands = True

        if not self.use_robot:
            self.get_logger().info("use_robot:=false -> Not connecting to robot (BrainCo commands disabled).")

        if self.use_robot:
            # brainco_hand_service runs on the Unitree DDS domain (0), like the Dex3.
            # Empty interface -> None so the SDK auto-determines a UP interface; an
            # empty string would build a blank Cyclone <Interface> and fail.
            ChannelFactoryInitialize(0, interface if interface else None)

            if arm_controlled in ["right", "both"]:
                self.right_pub = ChannelPublisher(f"{topic_prefix}/right/cmd", MotorCmds_)
                self.right_pub.Init()
                self.right_sub = ChannelSubscriber(f"{topic_prefix}/right/state", MotorStates_)
                self.right_sub.Init(self.right_callback)
                self.create_subscription(PointStamped, "/g1pilot/right_hand/dx3/action", self.right_action_callback, 10)

            if arm_controlled in ["left", "both"]:
                self.left_pub = ChannelPublisher(f"{topic_prefix}/left/cmd", MotorCmds_)
                self.left_pub.Init()
                self.left_sub = ChannelSubscriber(f"{topic_prefix}/left/state", MotorStates_)
                self.left_sub.Init(self.left_callback)
                self.create_subscription(PointStamped, "/g1pilot/left_hand/dx3/action", self.left_action_callback, 10)

        self.reset_service = self.create_service(Trigger, "/g1pilot/dx3/reset", self.reset_callback)

        self.create_timer(0.05, self.publish_commands)

    def reset_callback(self, request, response):
        if hasattr(self, "right_pub"):
            self.right_action = "close_2"
            self.right_target = CLOSE_VALUES_2
            self.right_speed = CLOSE_SPEED_2
        if hasattr(self, "left_pub"):
            self.left_action = "close_2"
            self.left_target = CLOSE_VALUES_2
            self.left_speed = CLOSE_SPEED_2
        response.success = True
        response.message = "Hands closed"
        return response

    def right_action_callback(self, msg: PointStamped):
        if msg.point.x < -0.5:
            self.right_action = "close_1"
            self.right_target = CLOSE_VALUES_1
            self.right_speed = CLOSE_SPEED_1
        elif msg.point.x > 0.5:
            self.right_action = "open"
            self.right_target = OPEN_VALUES
            self.right_speed = OPEN_SPEED
        else:
            self.right_action = "close_2"
            self.right_target = CLOSE_VALUES_2
            self.right_speed = CLOSE_SPEED_2

    def left_action_callback(self, msg: PointStamped):
        if msg.point.x < -0.5:
            self.left_action = "close_1"
            self.left_target = CLOSE_VALUES_1
            self.left_speed = CLOSE_SPEED_1
        elif msg.point.x > 0.5:
            self.left_action = "open"
            self.left_target = OPEN_VALUES
            self.left_speed = OPEN_SPEED
        else:
            self.left_action = "close_2"
            self.left_target = CLOSE_VALUES_2
            self.left_speed = CLOSE_SPEED_2

    def left_callback(self, msg: MotorStates_):
        self.left_gripper_state_publisher.publish(self._to_motor_state_list(msg, "left"))

    def right_callback(self, msg: MotorStates_):
        self.right_gripper_state_publisher.publish(self._to_motor_state_list(msg, "right"))

    def _to_motor_state_list(self, msg: MotorStates_, side: str) -> MotorStateList:
        motor_list_msg = MotorStateList()
        for i in range(len(msg.states)):
            motor_state = MotorState()
            motor_state.name = f"{side}_finger_{i}"
            motor_state.temperature = float(msg.states[i].temperature)
            motor_state.voltage = 0.0  # not reported by brainco_hand_service
            motor_state.position = float(msg.states[i].q)        # normalized 0..1
            motor_state.velocity = float(msg.states[i].dq)       # normalized speed
            motor_list_msg.motor_list.append(motor_state)
        return motor_list_msg

    def create_cmd(self, values, speeds):
        cmd = MotorCmds_(cmds=[unitree_go_msg_dds__MotorCmd_() for _ in range(NUM_FINGERS)])
        for i in range(NUM_FINGERS):
            cmd.cmds[i].q = float(values[i])    # position 0.0 (open) .. 1.0 (closed)
            cmd.cmds[i].dq = float(speeds[i])   # per-finger speed; must be > 0 to move
        return cmd

    def publish_commands(self):
        if not self.send_commands:
            return
        if hasattr(self, "right_pub") and self.right_action is not None:
            self.get_logger().info(f"WRITE right q={self.right_target} action={self.right_action}")
            self.get_logger().info(f"WRITE right dq={self.right_speed} action={self.right_action}")
            self.right_pub.Write(self.create_cmd(self.right_target, self.right_speed))
        if hasattr(self, "left_pub") and self.left_action is not None:
            self.get_logger().info(f"WRITE left q={self.left_target} action={self.left_action}")
            self.get_logger().info(f"WRITE left dq={self.left_speed} action={self.left_action}")
            self.left_pub.Write(self.create_cmd(self.left_target, self.left_speed))


def main(args=None):
    rclpy.init(args=args)
    node = BrainCoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

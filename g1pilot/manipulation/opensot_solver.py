#!/usr/bin/env python3
import subprocess
import threading
import time
import numpy as np
import array

from tf2_ros import TransformBroadcaster
from xbot2_interface import pyxbot2_interface as xbi
from xbot2_interface import pyxbot2_collision
from xbot2_interface import pyaffine3

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

from geometry_msgs.msg import PoseStamped, TransformStamped, WrenchStamped,Point
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState

from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarker, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler

from scipy.spatial.transform import Rotation as R

import pyopensot as pysot
from pyopensot.tasks.velocity import Postural, Cartesian, CoM
from pyopensot.constraints.velocity import JointLimits, VelocityLimits,CartesianPositionConstraint
from pyopensot_collision.constraints.velocity import CollisionAvoidance

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

from g1pilot.utils.common import (
    MotorState,
    G1_29_JointArmIndex,
    G1_29_JointWeakIndex,
    G1_29_JointWaistIndex,
    G1_29_JointIndex,
    DataBuffer,
)

G1_NUM_MOTOR = 29 # 12 body + 17 arm

# Per-joint PD gains and feedforward torque: {joint_name: (kp, kd, tau)}
# From Unitree G1 Table 5.1
JOINT_GAINS = {
    # Left leg
    "kLeftHipPitch":   (600.0, 10.0, 0.0),
    "kLeftHipRoll":    (700.0, 10.0, 0.0),
    "kLeftHipYaw":     (500.0, 10.0, 0.0),
    "kLeftKnee":       (1000.0, 10.0, 0.0),
    "kLeftAnklePitch": (900.0, 10.0, 20.0),  # tau = gravity feedforward
    "kLeftAnkleRoll":  (500.0, 10.0, 0.0),
    # Right leg
    "kRightHipPitch":   (600.0, 10.0, 0.0),
    "kRightHipRoll":    (700.0, 10.0, 0.0),
    "kRightHipYaw":     (500.0, 10.0, 0.0),
    "kRightKnee":       (1000.0, 10.0, 0.0),
    "kRightAnklePitch": (900.0, 10.0, 20.0),  # tau = gravity feedforward
    "kRightAnkleRoll":  (500.0, 10.0, 0.0),
    # Waist
    "kWaistYaw":   (400.0, 10.0, 0.0),
    "kWaistRoll":  (400.0, 10.0, 0.0),
    "kWaistPitch": (400.0, 10.0, 0.0),
    # Left arm
    "kLeftShoulderPitch": (100.0, 2.0, 0.0),
    "kLeftShoulderRoll":  (100.0, 2.0, 0.0),
    "kLeftShoulderYaw":   (50.0,  2.0, 0.0),
    "kLeftElbow":         (50.0,  2.0, 0.0),
    "kLeftWristRoll":     (20.0,  1.0, 0.0),
    "kLeftWristPitch":    (20.0,  1.0, 0.0),
    "kLeftWristyaw":      (20.0,  1.0, 0.0),
    # Right arm
    "kRightShoulderPitch": (100.0, 2.0, 0.0),
    "kRightShoulderRoll":  (100.0, 2.0, 0.0),
    "kRightShoulderYaw":   (50.0,  2.0, 0.0),
    "kRightElbow":         (50.0,  2.0, 0.0),
    "kRightWristRoll":     (20.0,  1.0, 0.0),
    "kRightWristPitch":    (20.0,  1.0, 0.0),
    "kRightWristYaw":      (20.0,  1.0, 0.0),
}

# Build index-based lookup: {motor_index: (kp, kd, tau)}
JOINT_GAINS_BY_ID = {
    G1_29_JointIndex[name].value: gains
    for name, gains in JOINT_GAINS.items()
}

# Self-collision link pairs to monitor
COLLISION_PAIRS = {
    # Left arm vs torso
    ("left_shoulder_yaw_link", "torso_link"),
    ("left_elbow_link", "torso_link"),
    ("left_wrist_roll_link", "torso_link"),
    ("left_wrist_pitch_link", "torso_link"),
    ("left_wrist_yaw_link", "torso_link"),
    ("left_rubber_hand", "torso_link"),
    # Right arm vs torso
    ("right_shoulder_yaw_link", "torso_link"),
    ("right_elbow_link", "torso_link"),
    ("right_wrist_roll_link", "torso_link"),
    ("right_wrist_pitch_link", "torso_link"),
    ("right_wrist_yaw_link", "torso_link"),
    ("right_rubber_hand", "torso_link"),
    # hip
    ("left_rubber_hand", "waist_yaw_link"),
    ("right_rubber_hand", "waist_yaw_link"),
    # pelvis
    ("left_rubber_hand", "pelvis_contour_link"),
    ("right_rubber_hand", "pelvis_contour_link"),
    # Left hand vs legs
    ("left_rubber_hand", "left_hip_pitch_link"),
    ("left_rubber_hand", "left_hip_roll_link"),
    ("left_rubber_hand", "left_hip_yaw_link"),
    ("left_rubber_hand", "left_knee_link"),
    ("left_rubber_hand", "right_hip_pitch_link"),
    ("left_rubber_hand", "right_hip_roll_link"),
    ("left_rubber_hand", "right_hip_yaw_link"),
    ("left_rubber_hand", "right_knee_link"),
    # Right hand vs legs
    ("right_rubber_hand", "left_hip_pitch_link"),
    ("right_rubber_hand", "left_hip_roll_link"),
    ("right_rubber_hand", "left_hip_yaw_link"),
    ("right_rubber_hand", "left_knee_link"),
    ("right_rubber_hand", "right_hip_pitch_link"),
    ("right_rubber_hand", "right_hip_roll_link"),
    ("right_rubber_hand", "right_hip_yaw_link"),
    ("right_rubber_hand", "right_knee_link"),
}

q_init = [
    -0.5,
    0.0,
    0.0,  # hips
    1.0,  # knee
    -0.5,
    0.0,  # ankles
    -0.5,
    0.0,
    0.0,  # hips
    1.0,  # knee
    -0.5,
    0.0,  # ankles
    0.0,
    0.0,
    0.0,  # waist
    0.3,
    0.25,
    0.0,
    1.0,
    0.15,
    0.0,
    0.0,  # arm
    0.3,
    -0.25,
    0.0,
    1.0,
    0.15,
    0.0,
    0.0,
]  # arm


class Mode:
    PR = 0
    AB = 1


class G1CollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__("g1_collision_avoidance_node")
        self.get_logger().info("Starting G1 Collision Avoidance Node")

        self.declare_parameter("use_robot", True)
        self.declare_parameter("enable_collision_avoidance", False)
        self.declare_parameter("interface", "")
        self.declare_parameter("send_cmds_to_robot", True)
        self.declare_parameter("publish_joint_states_opensot", False)
        self.declare_parameter("enable_external_collision_avoidance", False)
        self.declare_parameter("box_pose_topic", "/g1pilot/box_pose")
        self.declare_parameter("use_whole_body", False)
        self.declare_parameter("right_hand_frame_ref", "pelvis")
        self.declare_parameter("left_hand_frame_ref", "pelvis")
        self.interface = str(self.get_parameter("interface").value)
        self.use_robot = bool(self.get_parameter("use_robot").value)
        self.enable_collision_avoidance = bool(self.get_parameter("enable_collision_avoidance").value)
        self.send_cmds_to_robot = bool(self.get_parameter("send_cmds_to_robot").value)
        self.publish_joint_states_opensot = bool(self.get_parameter("publish_joint_states_opensot").value)
        self.enable_external_collision_avoidance = bool(self.get_parameter("enable_external_collision_avoidance").value)
        self.box_pose_topic = str(self.get_parameter("box_pose_topic").value)
        self.use_whole_body = bool(self.get_parameter("use_whole_body").value)

        self.control_dt = 0.005
        self.time = 0.0
        self.t = 0.0
        self.init_duration_s = 3.0

        self.mode = Mode.PR
        self.mode_machine = 0
        self.motors_on = 1

        self.right_hand_pose_ref = None
        self.left_hand_pose_ref = None
        self.emergency_stop = False
        self._initialized = False
        self.start_opensot = False

        self.box_pose = None

        self._resetting = False
        self._last_hand_ref_time = 0.0  # monotonic time of last hand ref msg



        self.client = self.create_client(GetParameters, "/robot_state_publisher/get_parameters")
        self.joint_state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.base_height_publisher = self.create_publisher(Float64, "/base_height", 10)
        self.base_link_broadcaster = TransformBroadcaster(self)

        # statis transform between world and pelvis
        t = TransformStamped()
        t.header.frame_id = "world"
        t.child_frame_id = "pelvis"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.base_link_broadcaster.sendTransform(t)

        self.start_opensot_sub = self.create_subscription(Bool, "/g1pilot/start_opensot", self.start_opensot_callback, 10)
        self.emergency_stop_sub = self.create_subscription(Bool, "/g1pilot/emergency_stop", self.emergency_stop_callback, 10)
        self.righ_hand_subscriber = self.create_subscription(
            PoseStamped, "/g1pilot/right_hand/pose_ref", self.right_hand_pose_ref_callback, 10
        )
        self.left_hand_subscriber = self.create_subscription(
            PoseStamped, "/g1pilot/left_hand/pose_ref", self.left_hand_pose_ref_callback, 10
        )

        self.reset_service = self.create_service(Trigger, "/g1pilot/reset", self.reset_callback)

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service /robot_state_publisher/get_parameters not available, waiting...")

        request = GetParameters.Request()
        request.names = ["robot_description"]
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        self.urdf = None
        if future.result() is not None:
            values = future.result().values
            for val in values:
                self.urdf = val.string_value
        else:
            self.get_logger().error("Failed to get robot_description from parameter server")

        self.interactive_marker_server = InteractiveMarkerServer(self, "teleoperation_markers")
        self.marker_poses = {}
        self.marker_enabled = {}
        self.menu_handler = {}
        self.menu_entry_ids = {}

        self.collision_distances_publisher = self.create_publisher(Marker, 'collision_distances', 10)

        if self.enable_external_collision_avoidance:
            self.box_obstacle_publisher = self.create_publisher(Marker, 'box_obstacle', 10)
            self.box_pose_subscriber = self.create_subscription(
                Marker, self.box_pose_topic, self.box_pose_callback, 10
            )

        self.right_hand_frame_ref = str(self.get_parameter("right_hand_frame_ref").value)
        self.left_hand_frame_ref = str(self.get_parameter("left_hand_frame_ref").value)

        self.motor_state = [MotorState() for _ in range(35)]
        self.lowstate_buffer = DataBuffer()

        self.lowcmd_publisher = None
        self.lowstate_subscriber = None
        self.subscribe_thread = None
        self.crc = None
        self.msg = None
        self.all_motor_q = None

        self._wb_init_active = False
        self._wb_init_done = False
        self._wb_q_start = None
        self._wb_init_start_time = None
        self._wb_init_duration = 3.0

        # Precompute joint index sets (hardware-independent, used by motor command helpers)
        self._leg_joint_ids = [j for j in G1_29_JointIndex if j.value < 12]

        if self.use_robot:
            self.get_logger().info("use_robot=True -> Initializing Unitree DDS interface")
            self.initialize_interface()
        else:
            self.get_logger().warn("use_robot=False -> Running in simulation/visualization mode (publishing only /joint_states)")

        self.initialize()
        self.initialize_imarkers()

        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

    def _subscribe_motor_state(self):
        while rclpy.ok():
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                self.lowstate_buffer.SetData(msg)
                for i in range(len(self.motor_state)):
                    self.motor_state[i].q = msg.motor_state[i].q
                    self.motor_state[i].dq = msg.motor_state[i].dq
            time.sleep(0.001)

    def initialize_interface(self):
        ChannelFactoryInitialize(0, self.interface)

        # Ensure the robot is in FSM=1 (Damp) for LowCmd control.
        # SetFsmId(4) first exits sport mode (200) if active, then Damp() = FSM=1.
        # loco = LocoClient()
        # loco.SetTimeout(10.0)
        # loco.Init()
        # loco.SetFsmId(4)
        # time.sleep(0.5)
        # loco.Damp()
        # time.sleep(0.5)
        # self.get_logger().info("[WB] FSM reset to Damp (FSM=1) via LocoClient")

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init()

        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state, daemon=True)
        self.subscribe_thread.start()

        topic = "rt/lowcmd" if self.use_whole_body else "rt/arm_sdk"
        self.lowcmd_publisher = ChannelPublisher(topic, LowCmd_)
        self.lowcmd_publisher.Init()

        while not self.lowstate_buffer.GetData():
            self.get_logger().info("Waiting for LowState data...")
            time.sleep(0.01)

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        self.all_motor_q = self.get_current_motor_q()

        self._apply_upper_body_cmds(self.all_motor_q)
        if self.use_whole_body:
            self._apply_leg_cmds(self.all_motor_q)

        self._initialized = True

    def get_mode_machine(self) -> int:
        msg = self.lowstate_buffer.GetData()
        return msg.mode_machine

    def get_current_motor_q(self):
        msg = self.lowstate_buffer.GetData()
        if msg is None:
            return np.zeros(29)
        q = np.zeros(29)
        for i in range(29):
            q[i] = msg.motor_state[i].q
        return q

    def _apply_upper_body_cmds(self, q29):
        """Set arm and waist motor commands from a 29-element joint position array."""
        for jid in list(G1_29_JointArmIndex) + list(G1_29_JointWaistIndex):
            kp, kd, tau = JOINT_GAINS_BY_ID[jid.value]
            self.msg.motor_cmd[jid].mode = 1
            self.msg.motor_cmd[jid].kp = kp
            self.msg.motor_cmd[jid].kd = kd
            self.msg.motor_cmd[jid].dq = 0.0
            self.msg.motor_cmd[jid].tau = tau
            self.msg.motor_cmd[jid].q = float(q29[jid.value])

    def _apply_leg_cmds(self, q29):
        """Set leg motor commands from a 29-element joint position array (whole-body mode)."""
        for jid in self._leg_joint_ids:
            kp, kd, tau = JOINT_GAINS_BY_ID[jid.value]
            self.msg.motor_cmd[jid].mode = 1
            self.msg.motor_cmd[jid].kp = kp
            self.msg.motor_cmd[jid].kd = kd
            self.msg.motor_cmd[jid].dq = 0.0
            self.msg.motor_cmd[jid].tau = tau
            self.msg.motor_cmd[jid].q = float(q29[jid.value])

    def _publish_lowcmd(self):
        """Sync mode_machine, set mode_pr, compute CRC and publish LowCmd."""
        self.msg.mode_machine = self.get_mode_machine()
        self.msg.mode_pr = 0 if self.use_whole_body else 1
        try:
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
        except Exception:
            pass
        self.msg.crc = self.crc.Crc(self.msg)
        if self.send_cmds_to_robot:
            self.lowcmd_publisher.Write(self.msg)

    def right_hand_pose_ref_callback(self, msg: PoseStamped):
        self._last_hand_ref_time = time.monotonic()
        if self._resetting:
            return
        if msg.header.frame_id != self.right_hand_frame_ref:
            self.get_logger().error(f"Received right hand pose ref in frame '{msg.header.frame_id}', but expected '{self.right_hand_frame_ref}'")
        else:
            self.right_hand_pose_ref = msg

    def left_hand_pose_ref_callback(self, msg: PoseStamped):
        self._last_hand_ref_time = time.monotonic()
        if self._resetting:
            return
        if msg.header.frame_id != self.left_hand_frame_ref:
            self.get_logger().error(f"Received left hand pose ref in frame '{msg.header.frame_id}', but expected '{self.left_hand_frame_ref}'")
        else:
            self.left_hand_pose_ref = msg

    def reset_callback(self, request, response):
        """Service callback for /g1pilot/reset. Waits 1s for no hand commands, then re-initializes."""
        if self._resetting:
            response.success = False
            response.message = "Reset already in progress"
            return response

        self._resetting = True
        self.get_logger().info("[Reset] Requested. Waiting 1s for hand references to stop...")

        # Wait 1 second, then check if any hand ref arrived during that window
        time.sleep(1.0)

        elapsed_since_last = time.monotonic() - self._last_hand_ref_time
        if elapsed_since_last < 1.0:
            self._resetting = False
            response.success = False
            response.message = "Hand reference received during the last 1s, aborting reset"
            self.get_logger().warn("[Reset] Aborted: hand reference received recently")
            return response

        self.get_logger().info("[Reset] No hand references for 1s. Re-initializing...")

        # Clear hand references
        self.right_hand_pose_ref = None
        self.left_hand_pose_ref = None

        # Re-run initialization
        self.initialize()
        self.initialize_imarkers()

        # Reset whole-body init state so it can be triggered again
        self._wb_init_active = False
        self._wb_init_done = False
        self._wb_q_start = None
        self._wb_init_start_time = None

        self._resetting = False
        self.get_logger().info("[Reset] Initialization complete")

        response.success = True
        response.message = "Reset and re-initialization complete"
        return response

    def start_opensot_callback(self, msg: Bool):
        self.start_opensot = bool(msg.data)

    def emergency_stop_callback(self, msg: Bool):
        self.emergency_stop = bool(msg.data)

    def box_pose_callback(self, msg: Marker):
        self.box_pose = msg

    def initialize_imarkers(self):
        self.marker_enabled = {}
        self.menu_handler = {}

        base_ref, _ = self.base.getReference()
        com_ref, _ = self.com.getReference()
        pose_ref = pyaffine3.Affine3()
        pose_ref.translation = com_ref
        pose_ref.linear = base_ref.linear.copy()
        # self.get_logger().info(f"Initial base pose:\n{pose_ref}")
        # self.make_6dof_marker("base_marker", pose_ref, "world")

        right_hand_ref = self.right_gripper.getReference()
        self.get_logger().info(f"Initial right hand pose:\n{right_hand_ref}")
        self.make_6dof_marker("right_hand_marker", right_hand_ref[0], self.right_hand_frame_ref)

        left_hand_ref = self.left_gripper.getReference()
        self.get_logger().info(f"Initial left hand pose:\n{left_hand_ref}")
        self.make_6dof_marker("left_hand_marker", left_hand_ref[0], self.left_hand_frame_ref)

    def make_6dof_marker(self, name, pose, frame_id):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.name = name
        int_marker.description = '6-DOF Control'
        int_marker.scale = 0.3

        int_marker.pose.position.x = pose.translation[0]
        int_marker.pose.position.y = pose.translation[1]
        int_marker.pose.position.z = pose.translation[2]

        quat_xyzw = R.from_matrix(pose.linear).as_quat() # Format: [x, y, z, w]
        int_marker.pose.orientation.x = quat_xyzw[0]
        int_marker.pose.orientation.y = quat_xyzw[1]
        int_marker.pose.orientation.z = quat_xyzw[2]
        int_marker.pose.orientation.w = quat_xyzw[3]

        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose = int_marker.pose
        self.marker_poses[name] = ps

        self.marker_home_poses = getattr(self, "marker_home_poses", {})
        self.marker_home_poses[name] = PoseStamped()
        self.marker_home_poses[name].header.frame_id = frame_id
        self.marker_home_poses[name].pose = int_marker.pose

        # Add a visible marker (e.g., a cube)
        cube_marker = Marker()
        cube_marker.type = Marker.CUBE
        cube_marker.scale.x = 0.05
        cube_marker.scale.y = 0.05
        cube_marker.scale.z = 0.05
        cube_marker.color.r = 0.0
        cube_marker.color.g = 1.0
        cube_marker.color.b = 0.0
        cube_marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(cube_marker)
        int_marker.controls.append(control)

        # Add 6-DOF controls
        self.add_6dof_controls(int_marker)

        self.marker_enabled[name] = False

        menu = MenuHandler()
        h_enable = menu.insert("Enable", callback=self.process_menu)
        menu.setCheckState(
            h_enable,
            MenuHandler.CHECKED if self.marker_enabled.get(name, True) else MenuHandler.UNCHECKED
        )
        h_reset = menu.insert("Reset", callback=self.process_menu)
        self.menu_handler[name] = menu

        self.menu_entry_ids = getattr(self, "menu_entry_ids", {})
        self.menu_entry_ids[name] = {"enable": h_enable, "reset": h_reset}

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.name = "menu"
        int_marker.controls.append(menu_control)

        self.interactive_marker_server.insert(marker=int_marker, feedback_callback=self.process_feedback)
        menu.apply(self.interactive_marker_server, name)
        self.interactive_marker_server.applyChanges()

        menu.setCheckState(
            h_enable,
            MenuHandler.CHECKED if self.marker_enabled.get(name, True) else MenuHandler.UNCHECKED
        )

        self.menu_handler[name] = menu

        self.menu_entry_ids = getattr(self, "menu_entry_ids", {})
        self.menu_entry_ids[name] = {"enable": h_enable, "reset": h_reset}

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.name = "menu"
        int_marker.controls.append(menu_control)

        self.interactive_marker_server.insert(marker=int_marker, feedback_callback=self.process_feedback)
        menu.apply(self.interactive_marker_server, name)
        self.interactive_marker_server.applyChanges()

    def process_feedback(self, feedback):
        name = feedback.marker_name
        if not self.marker_enabled.get(name, True):
            return

        if name not in self.marker_poses:
            self.marker_poses[name] = PoseStamped()

        self.marker_poses[name].header = feedback.header
        self.marker_poses[name].pose = feedback.pose

    def process_menu(self, feedback):
        name = feedback.marker_name
        ids = self.menu_entry_ids.get(name, {})

        if feedback.menu_entry_id == ids.get("enable"):
            new_state = not self.marker_enabled.get(name, True)
            self.marker_enabled[name] = new_state

            menu = self.menu_handler.get(name, None)
            if menu is not None:
                menu.setCheckState(
                    ids["enable"],
                    MenuHandler.CHECKED if new_state else MenuHandler.UNCHECKED
                )
                menu.reApply(self.interactive_marker_server)
                self.interactive_marker_server.applyChanges()


        elif feedback.menu_entry_id == ids.get("reset"):
            home = self.marker_home_poses.get(name, None)
            if home is not None:
                self.marker_poses[name] = PoseStamped()
                self.marker_poses[name].header = home.header
                self.marker_poses[name].pose = home.pose

                self.interactive_marker_server.setPose(name, home.pose, home.header)
                self.interactive_marker_server.applyChanges()


    def add_6dof_controls(self, marker):
        axes = ['x', 'y', 'z']
        for axis in axes:
            # Rotation
            control = InteractiveMarkerControl()
            control.name = f'rotate_{axis}'
            control.orientation.w = 1.0
            setattr(control.orientation, axis, 1.0)
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            marker.controls.append(control)

            # Translation
            control = InteractiveMarkerControl()
            control.name = f'move_{axis}'
            control.orientation.w = 1.0
            setattr(control.orientation, axis, 1.0)
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            marker.controls.append(control)

    # ----------------------------
    # OpenSoT init
    # ----------------------------
    def initialize(self, manipulation_frame="world"):
        self.get_logger().warning("Initializing XBot2 Model Interface")

        if not self.urdf or len(self.urdf.strip()) < 100:
            self.get_logger().error(f"robot_description invalid. len={0 if not self.urdf else len(self.urdf)}")
            raise RuntimeError("robot_description is empty/invalid -> cannot build ModelInterface2")


        self.model = xbi.ModelInterface2(self.urdf)
        
        self.q = np.zeros(self.model.nq)
        self.q[6] = 1.0  # quaternion w
        self.q[7:] = q_init[0:].copy()

        # Compute pelvis height so feet rest on the ground (z=0):
        # run FK with pelvis at z=0, then offset pelvis by the negative foot z.
        self.model.setJointPosition(self.q)
        self.model.update()
        left_foot_z = self.model.getPose("left_foot_point_contact").translation[2]
        right_foot_z = self.model.getPose("right_foot_point_contact").translation[2]
        self.q[2] = -min(left_foot_z, right_foot_z)

        self.dq = np.zeros(self.model.nv)

        self.model.setJointPosition(self.q)
        self.model.setJointVelocity(self.dq)
        self.model.update()

        self.com = CoM(self.model)
        # Offset CoM reference slightly backward (negative X) for stability
        com_ref_init, _ = self.com.getReference()
        com_ref_init[0] -= 0.01  # 2.5 cm backward
        self.com.setReference(com_ref_init)

        self.get_logger().warning("Initializing OpenSoT Tasks and Constraints")

        self.get_logger().info("Task: Base")
        self.base = Cartesian("base_task", self.model, "pelvis", manipulation_frame)
        self.base.setLambda(0.1)

        self.get_logger().info("Task: Torso")
        self.torso = Cartesian("torso_task", self.model, "torso_link", manipulation_frame)
        self.torso.setLambda(0.1)

        self.get_logger().info("Task: Pelvis")
        self.pelvis = Cartesian("pelvis_task", self.model, "pelvis", manipulation_frame)
        self.pelvis.setLambda(0.1)

        self.get_logger().info("Task: Right Gripper")
        self.right_gripper = Cartesian(
            "right_gripper_task",
            self.model,
            "right_hand_point_contact",
            self.right_hand_frame_ref,
        )
        self.right_gripper.setLambda(0.1)

        self.get_logger().info("Task: Left Gripper")
        self.left_gripper = Cartesian(
            "left_gripper_task",
            self.model,
            "left_hand_point_contact",
            self.left_hand_frame_ref,
        )
        self.left_gripper.setLambda(0.1)

        self.get_logger().info("Task: Postural")
        self.postural = Postural(self.model)
        self.postural.setLambda(0.3) # coman uses 0.01 ?
        self.W = self.postural.getWeight().copy()
        if self.use_whole_body:
            # Zero floating base DOFs so the postural task doesn't fight
            # against leg bending by trying to restore the pelvis pose
            self.W[0:6, 0:6] = 0.0
            # Always target q_init regardless of actual robot state at startup
            q_ref = np.zeros(self.model.nv)
            q_ref[6:] = q_init
            self.postural.setReference(q_ref)
        print(self.W.shape)
        self.postural.setWeight(self.W)

        print(self.model.getNv())
        print(self.model.getNq())

        self.get_logger().info("Constraints: Joint Limits")
        self.qmin, self.qmax = self.model.getJointLimits()
        self.qlims = JointLimits(self.model, self.qmax, self.qmin)
        print(self.qmin)
        self.dqmax = self.model.getVelocityLimits()
        self.dqlims = VelocityLimits(self.model, self.dqmax, self.control_dt)

        # self.collision_avoidance_constraint = None
        if self.enable_collision_avoidance:
            self.get_logger().info("Constraints: Self-Collision Avoidance")
            self.collision_avoidance_constraint = CollisionAvoidance(
                self.model, max_pairs=50, collision_urdf=self.urdf)#, collision_srdf=self.urdf)

            self.collision_avoidance_constraint.setCollisionList(COLLISION_PAIRS)
            self.collision_avoidance_constraint.setBoundScaling(0.1)
            self.collision_avoidance_constraint.setLinkPairThreshold(0.01)
            self.collision_avoidance_constraint.setDetectionThreshold(-1)

        if self.use_whole_body:
            self.get_logger().info("Task: Left Foot (contact constraint)")
            self.left_foot = Cartesian("left_foot_task", self.model, "left_ankle_roll_link", "world")
            self.get_logger().info("Task: Right Foot (contact constraint)")
            self.right_foot = Cartesian("right_foot_task", self.model, "right_ankle_roll_link", "world")
            # No setLambda on foot tasks: used as hard constraints via <<

            # Pelvis height upper bound: prevent pelvis from rising above starting Z
            torso_pos = self.torso.getActualPose()[:3, 3]
            A_torso = np.array([[0.0, 0.0, 1.0]])   # 1x3: select Z
            b_torso = np.array([torso_pos[2]-0.01])       # upper bound = current Z
            self.pelvis_height_constraint = CartesianPositionConstraint(
                self.torso, A_torso, b_torso, 1.0)
            self.get_logger().info(f"Constraint: Torso height <= {torso_pos[2]:.4f} m")

            if self.enable_collision_avoidance:
                self.stack = (
                    self.com % [0, 1]
                    / (self.right_gripper + self.left_gripper  )
                    / (self.torso%[3,4] + self.pelvis%[0,1,3,4,5] +  0.1 * self.pelvis%[2])
                    / self.postural
                    << self.qlims
                    << self.dqlims
                    << self.collision_avoidance_constraint
                    << self.pelvis_height_constraint
                    << (self.left_foot + self.right_foot)
                )
            else:
                self.stack = (
                    (self.com % [0, 1] + self.right_gripper + self.left_gripper)
                    / self.torso_rp
                    / self.postural
                    << self.qlims
                    << self.dqlims
                    << self.pelvis_height_constraint
                    << (self.left_foot + self.right_foot)
                )
        elif self.enable_collision_avoidance:
            self.stack = ((
                self.base#%[0,1,3,4,5]
                / (self.torso % [3, 4, 5] + self.right_gripper + self.left_gripper)
                / self.postural)
                << self.qlims
                << self.dqlims
                << self.collision_avoidance_constraint
            )
        else:
            self.stack = ((
                self.base#%[0,1,3,4,5]
                / (self.torso % [3, 4, 5] + self.right_gripper + self.left_gripper)
                / self.postural)
                << self.qlims
                << self.dqlims
            )

        # self.com_xy = self.com % [0, 1]
        # self.stack = (
        #     self.com_xy
        #     / (self.base % [3, 4, 5] + self.torso % [3, 4, 5] + self.right_gripper + self.left_gripper)
        #     / self.postural
        #     << self.qlims
        #     << self.dqlims
        # )
            
        self.stack.update()
        self.solver = pysot.iHQP(self.stack, eps_regularisation=1e11)

    # ----------------------------
    # Control loop
    # ----------------------------
    def control_loop(self):
        if self._resetting:
            return

        # Whole-body gradual init: triggered by start_opensot, interpolates to q_init first
        if self.use_whole_body and self.start_opensot and not self._wb_init_done:
            if not self._wb_init_active:
                # First tick after start_opensot: read fresh robot state and begin
                self._wb_q_start = self.get_current_motor_q()
                self._wb_init_start_time = time.time()
                self._wb_init_active = True
                self.get_logger().info("[WB] start_opensot received: 3s gradual init to q_init")
            elapsed = time.time() - self._wb_init_start_time
            alpha = min(elapsed / self._wb_init_duration, 1.0)
            if self.use_robot and self.lowcmd_publisher is not None and not self.emergency_stop:
                q_interp = (1.0 - alpha) * self._wb_q_start + alpha * np.array(q_init)
                self._apply_upper_body_cmds(q_interp)
                self._apply_leg_cmds(q_interp)
                self._publish_lowcmd()
            if alpha >= 1.0:
                self._wb_init_active = False
                self._wb_init_done = True
                self.get_logger().info("[WB] Gradual init complete. OpenSoT starting.")
            return

        if self.start_opensot and not self.emergency_stop and (not self.use_whole_body or self._wb_init_done):
            self.model.setJointPosition(self.q)
            self.model.setJointVelocity(self.dq)
            self.model.update()

            # right hand
            use_marker_right = self.marker_enabled.get("right_hand_marker", False) and "right_hand_marker" in self.marker_poses
            if use_marker_right:
                ps = self.marker_poses["right_hand_marker"]
                T = pyaffine3.Affine3()
                T.translation = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
                T.linear = R.from_quat([ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w]).as_matrix()
                self.right_gripper.setReference(T)
            elif self.right_hand_pose_ref is not None:
                if self.right_hand_pose_ref.header.frame_id != self.right_hand_frame_ref:
                    self.get_logger().error(f"Right hand pose ref frame mismatch: got '{self.right_hand_pose_ref.header.frame_id}', expected '{self.right_hand_frame_ref}'")
                else:
                    ps = self.right_hand_pose_ref
                    T = pyaffine3.Affine3()
                    T.translation = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
                    T.linear = R.from_quat([ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w]).as_matrix()
                    self.right_gripper.setReference(T)

                # left hand
            use_marker_left = self.marker_enabled.get("left_hand_marker", False) and "left_hand_marker" in self.marker_poses
            if use_marker_left:
                ps = self.marker_poses["left_hand_marker"]
                T = pyaffine3.Affine3()
                T.translation = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
                T.linear = R.from_quat([ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w]).as_matrix()
                self.left_gripper.setReference(T)
            elif self.left_hand_pose_ref is not None:
                if self.left_hand_pose_ref.header.frame_id != self.left_hand_frame_ref:
                    self.get_logger().error(f"Left hand pose ref frame mismatch: got '{self.left_hand_pose_ref.header.frame_id}', expected '{self.left_hand_frame_ref}'")
                else:
                    ps = self.left_hand_pose_ref
                    T = pyaffine3.Affine3()
                    T.translation = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
                    T.linear = R.from_quat([ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w]).as_matrix()
                    self.left_gripper.setReference(T)

            # external box collision avoidance
            if self.enable_external_collision_avoidance and self.box_pose is not None:
                w_T_box = pyaffine3.Affine3()
                w_T_box.translation = np.array([
                    self.box_pose.pose.position.x,
                    self.box_pose.pose.position.y,
                    self.box_pose.pose.position.z])
                w_T_box.linear = R.from_quat([
                    self.box_pose.pose.orientation.x,
                    self.box_pose.pose.orientation.y,
                    self.box_pose.pose.orientation.z,
                    self.box_pose.pose.orientation.w]).as_matrix()

                if not self.collision_avoidance_constraint.setCollisionShapeActive("box", True):
                    box = pyxbot2_collision.shape.Box()
                    box.size = np.array([
                        self.box_pose.scale.x,
                        self.box_pose.scale.y,
                        self.box_pose.scale.z])
                    self.collision_avoidance_constraint.addCollisionShape(
                        "box", "world", box, w_T_box, [])
                    self.get_logger().info(f"Box added: pos={w_T_box.translation}, size={box.size}")
                else:
                    self.collision_avoidance_constraint.moveCollisionShape("box", w_T_box)

            # solve
            self.stack.update()
            try:
                dq = self.solver.solve()
                self.q = self.model.sum(self.q, dq)
                self.dq = dq

            except Exception as e:
                self.get_logger().error(f"OpenSoT Solver Error: {e}")
                dq = None

        

            if not self.use_robot:
                t = TransformStamped()
                t.header.frame_id = "world"
                t.child_frame_id = "pelvis"
                t.header.stamp = self.get_clock().now().to_msg()
                t.transform.translation.x = self.q[0]
                t.transform.translation.y = self.q[1]
                t.transform.translation.z = self.q[2]
                t.transform.rotation.x = self.q[3]
                t.transform.rotation.y = self.q[4]
                t.transform.rotation.z = self.q[5]
                t.transform.rotation.w = self.q[6]

                self.base_link_broadcaster.sendTransform(t)


            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()


            try:
                js.name = self.model.getJointNames()[1::]
                js.position = self.q[7:].tolist()
            except Exception:
                js.name = []
                js.position = []
                self.get_logger().error("Error getting joint names from model")
            if self.publish_joint_states_opensot:
                self.joint_state_publisher.publish(js)

            msg = Float64()
            msg.data = self.q[2]

            self.base_height_publisher.publish(msg)

            if not self.use_robot:
                return

            if self.lowcmd_publisher is None or self.msg is None or self.crc is None:
                return

            if self.emergency_stop or not self.motors_on:
                for jid in G1_29_JointArmIndex:
                    kp, kd, _ = JOINT_GAINS_BY_ID[jid.value]
                    self.msg.motor_cmd[jid].mode = 0
                    self.msg.motor_cmd[jid].kp = kp
                    self.msg.motor_cmd[jid].kd = kd
                    self.msg.motor_cmd[jid].q = float(self.msg.motor_cmd[jid].q)

                self.msg.crc = self.crc.Crc(self.msg)
                if self.send_cmds_to_robot:
                    self.lowcmd_publisher.Write(self.msg)
                return

            self._apply_upper_body_cmds(self.q[7:])
            if self.use_whole_body:
                self._apply_leg_cmds(self.q[7:])
            self._publish_lowcmd()

            # publish self-collision debugging
            if self.enable_collision_avoidance:
                self.publishCollisionDistances(self.collision_avoidance_constraint.getOrderedWitnessPointVector(), self.get_clock().now().to_msg())

            # publish box obstacle visualization
            if self.enable_external_collision_avoidance and self.box_pose is not None:
                self.publishBoxObstacle(self.get_clock().now().to_msg())

    def publishCollisionDistances(self, collision_distance_points, time):
        marker = Marker()
        marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.0
        marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.header.frame_id = "world"
        marker.header.stamp = time
        marker.ns = "collision_distances"
        marker.id = 0
        marker.scale.x = 0.005  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Opaque

        for point_pairs in collision_distance_points:
            pa = point_pairs[0]
            pb = point_pairs[1]

            point_a = Point()
            point_a.x = pa[0]
            point_a.y = pa[1]
            point_a.z = pa[2]

            point_b = Point()
            point_b.x = pb[0]
            point_b.y = pb[1]
            point_b.z = pb[2]

            marker.points.append(point_a)
            marker.points.append(point_b)


        self.collision_distances_publisher.publish(marker)

    def publishBoxObstacle(self, time):
        marker = Marker()
        marker.header.frame_id = self.box_pose.header.frame_id
        marker.header.stamp = time
        marker.ns = "box_obstacle"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale = self.box_pose.scale
        marker.color.r = 0.8
        marker.color.g = 0.5
        marker.color.b = 0.2
        marker.color.a = 0.5
        marker.pose = self.box_pose.pose
        self.box_obstacle_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = G1CollisionAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

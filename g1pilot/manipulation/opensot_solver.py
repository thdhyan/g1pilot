#!/usr/bin/env python3
import gc
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
from std_msgs.msg import Bool, Float64, Int8
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
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

# Speech moved to the dedicated g1pilot tts_node (subscribes to /g1pilot/say).
# from g1pilot.state.say import synthesize_pcm, play_stream, play_stop, CHUNK_SIZE
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
    "kLeftShoulderPitch": (500.0, 10.0, 0.0),
    "kLeftShoulderRoll":  (400.0, 10.0, 0.0),
    "kLeftShoulderYaw":   (200.0,  5.0, 0.0),
    "kLeftElbow":         (150.0,  5.0, 0.0),
    "kLeftWristRoll":     (100.0,  2.0, 0.0),
    "kLeftWristPitch":    (100.0,  2.0, 0.0),
    "kLeftWristyaw":      (100.0,  2.0, 0.0),
    # Right arm
    "kRightShoulderPitch": (500.0, 10.0, 0.0),
    "kRightShoulderRoll":  (400.0, 10.0, 0.0),
    "kRightShoulderYaw":   (200.0,  5.0, 0.0),
    "kRightElbow":         (150.0,  5.0, 0.0),
    "kRightWristRoll":     (100.0,  2.0, 0.0),
    "kRightWristPitch":    (100.0,  2.0, 0.0),
    "kRightWristYaw":      (100.0,  2.0, 0.0),
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
    # -0.6,
    # 0.0,
    # 0.0,  # hips
    # 1.2,  # knee
    # -0.6,
    # 0.0,  # ankles
    # -0.6,
    # 0.0,
    # 0.0,  # hips
    # 1.2,  # knee
    # -0.6,
    
    ####### 
### old values
    -0.4841,   #  0 LeftHipPitch
    0.0297,    #  1 LeftHipRoll
    -0.0230,   #  2 LeftHipYaw          (hips: roll out, slight yaw)
    1.1874,    #  3 LeftKnee
    -0.7036,   #  4 LeftAnklePitch
    -0.0376,   #  5 LeftAnkleRoll       (roll keeps sole flat)

    -0.4841,   #  6 RightHipPitch
    -0.0297,   #  7 RightHipRoll
    0.0230,    #  8 RightHipYaw         (hips)
    1.1874,    #  9 RightKnee
    -0.7036,   # 10 RightAnklePitch
    0.0376,    # 11 RightAnkleRoll      (ankles)
    0.0,       # 12 WaistYaw
    0.0,       # 13 WaistRoll
    0.0,       # 14 WaistPitch          (# 0.05)
    0.3,       # 15 LeftShoulderPitch   (# 0.3)
    0.25,      # 16 LeftShoulderRoll
    0.0,       # 17 LeftShoulderYaw
    1.0,       # 18 LeftElbow
    0.15,      # 19 LeftWristRoll
    0.0,       # 20 LeftWristPitch
    0.0,       # 21 LeftWristYaw        (left arm)
    0.3,       # 22 RightShoulderPitch
    -0.25,     # 23 RightShoulderRoll
    0.0,       # 24 RightShoulderYaw
    1.0,       # 25 RightElbow
    0.15,      # 26 RightWristRoll
    0.0,       # 27 RightWristPitch
    0.0,       # 28 RightWristYaw
]  # arm


q_posture_ref = [

### new values (measured q_init)
    -0.5418787002563477,    # left_hip_pitch
    0.018867963925004005,   # left_hip_roll
    -0.01564173400402069,   # left_hip_yaw
    1.2276533842086792,     # left_knee
    -0.6694384813308716,    # left_ankle_pitch
    -0.02668987214565277,   # left_ankle_roll
    -0.5376517176628113,    # right_hip_pitch
    -0.013882526196539402,  # right_hip_roll
    0.0033747577108442783,  # right_hip_yaw
    1.2392263412475586,     # right_knee
    -0.6495804190635681,    # right_ankle_pitch
    0.02879408560693264,    # right_ankle_roll
    0.0,    # waist_yaw
    0.006152449641376734,   # waist_roll
    0.05840779468417168,    # waist_pitch
    0.26167556643486023,    # left_shoulder_pitch  (mirror of right)
    0.55,    # left_shoulder_roll   (-right)
    0.1,     # left_shoulder_yaw    (-right)
    0.0746137872338295,     # left_elbow           (= right)
    -0.052670668810606,     # left_wrist_roll      (-right)
    -0.054060839116573334,  # left_wrist_pitch     (= right)
    -0.447550892829895,     # left_wrist_yaw       (-right)
    0.26167556643486023,    # right_shoulder_pitch
    -0.55,   # right_shoulder_roll
    -0.1,    # right_shoulder_yaw
    0.0746137872338295,     # right_elbow
    0.052670668810606,      # right_wrist_roll
    -0.054060839116573334,  # right_wrist_pitch
    0.447550892829895,      # right_wrist_yaw
]


# Shutdown / rest pose: deep folded crouch the robot eases into before
# power-off (and the pose it starts from on power-up). Measured on the robot.
# Only legs+waist (indices 0..14) are commanded toward this; the arms hang
# limp during the park ramp, so the arm entries (15..28) are informational.
PARK_POSE = np.array([
    -2.2180,  0.0151, -0.0742,  2.8548, -0.2496, -0.0284,   #  0..5  left leg
    -2.2428,  0.0181,  0.0793,  2.8483, -0.1675,  0.0203,   #  6..11 right leg
    -0.0055, -0.0167,  0.5270,                              # 12..14 waist
     0.0492,  0.0147,  1.0290,  1.3693, -1.2997,  0.1748, -1.1806,  # 15..21 left arm (limp)
    -0.0373, -0.0991,  2.3825,  1.4383, -1.5155,  0.1812,  1.3074,  # 22..28 right arm (limp)
])


SCAN_AMPLITUDE_RAD = np.deg2rad(20.0)
SCAN_FREQ_HZ = 0.05

LED_OPERATING = (0, 255, 0)     # green: normal teleop
LED_SCANNING = (255, 140, 0)    # orange: scanning mode


class Mode:
    PR = 0
    AB = 1


class G1CollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__("g1_collision_avoidance_node")
        self.get_logger().info("Starting G1 Collision Avoidance Node")

        # Diagnostic: log every cyclic-GC pause so we can confirm whether GC is
        # the source of jitter in the 200 Hz control loop. A line landing near a
        # control-loop overrun (esp. gen=2, a few ms) means GC is the culprit;
        # the fix is gc.freeze()+gc.disable() after init. Remove once confirmed.
        self._gc_t0 = 0.0
        def _gc_cb(phase, info):
            if phase == "start":
                self._gc_t0 = time.perf_counter()
            else:
                dt_ms = (time.perf_counter() - self._gc_t0) * 1e3
                self.get_logger().warn(
                    f"[GC] {dt_ms:.2f} ms, gen={info['generation']}, "
                    f"collected={info['collected']}, uncollectable={info['uncollectable']}"
                )
        gc.callbacks.append(_gc_cb)

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
        # LowCmd send-rate diagnostic: count robot writes and log the count
        # (and effective Hz) every half second. _lowcmd_count = successful
        # writes; _lowcmd_attempts = all tries (including ones that threw).
        self._lowcmd_count = 0
        self._lowcmd_attempts = 0
        self._lowcmd_log_t0 = time.perf_counter()
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
        self.scanning_mode_sub = self.create_subscription(Int8, "/g1pilot/scanning_mode", self.scanning_mode_callback, 10)
        self.righ_hand_subscriber = self.create_subscription(
            PoseStamped, "/g1pilot/right_hand/pose_ref", self.right_hand_pose_ref_callback, 10
        )
        self.left_hand_subscriber = self.create_subscription(
            PoseStamped, "/g1pilot/left_hand/pose_ref", self.left_hand_pose_ref_callback, 10
        )

        self.reset_service = self.create_service(Trigger, "/g1pilot/reset", self.reset_callback)
        self.park_service = self.create_service(Trigger, "/g1pilot/park", self.park_callback)

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

        if self.urdf:
            import re
            _name = re.search(r'<robot\s+name="([^"]+)"', self.urdf)
            self.get_logger().info(
                f"[opensot] Built model from robot_description: "
                f"robot name='{_name.group(1) if _name else '?'}', {len(self.urdf)} chars"
            )

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

        # Init ramps: each entry is a timed smoothstep blend for one joint
        # group ("leg" = legs+waist, "arm" = arms), keyed off a shared t0.
        # delay/duration are chosen per profile when start_opensot fires:
        #   first bring-up -> legs (0s, 15s), then arms (15s, 5s); arms are
        #                     limp until their ramp begins.
        #   reset-restart  -> both (0s, 3s) together, no limp (old behavior).
        # q_start is the blend start pose, captured when each ramp begins.
        # _ramp_t0 is None when no ramp is armed (also the arm_sdk case).
        self._ramp_t0 = None
        self._leg_ramp = {"delay": 0.0, "duration": 15.0, "q_start": None}
        self._arm_ramp = {"delay": 0.0, "duration": 5.0, "q_start": None}
        # Park only: the waist follows the legs after a delay (set in park_callback).
        self._waist_ramp = {"delay": 0.0, "duration": 15.0, "q_start": None}
        # Blend start pose source: measured on first start, last command on a
        # reset-restart (the robot is held at the last command during a reset).
        self._ramp_capture = self.get_current_motor_q
        # False until OpenSoT has run once (selects first-vs-reset profile).
        self._opensot_ever_started = False
        # Park (shutdown) mode: when True the leg ramp targets PARK_POSE and the
        # arms are held limp, instead of tracking the OpenSoT solution.
        self._park_active = False

        self.scanning_mode = 0
        self._scan_active = False
        self._scan_t = 0.0
        self._scan_torso_base_pose = None

        self.audio_client = None  # kept for LED control; speech moved to tts_node
        # self._say_thread = None  # speech moved to tts_node (/g1pilot/say)

        # Precompute joint index sets (hardware-independent, used by motor command helpers)
        self._leg_joint_ids = [j for j in G1_29_JointIndex if j.value < 12]
        self._waist_joint_ids = list(G1_29_JointWaistIndex)
        self._arm_joint_ids = list(G1_29_JointArmIndex)

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

        self.audio_client = AudioClient()
        self.audio_client.SetTimeout(10.0)
        self.audio_client.Init()
        self._set_led(*LED_OPERATING)

        while not self.lowstate_buffer.GetData():
            self.get_logger().info("Waiting for LowState data...")
            time.sleep(0.01)

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        self.all_motor_q = self.get_current_motor_q()

        self._apply_arm_cmds(self.all_motor_q)
        if self.use_whole_body:
            self._apply_leg_cmds(self.all_motor_q)
            self._apply_waist_cmds(self.all_motor_q)

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

    def get_last_cmd_q(self):
        """Read back the last commanded q for all 29 motors from self.msg.
        Falls back to current motor q if no command has been built yet."""
        if self.msg is None:
            return self.get_current_motor_q()
        q = np.zeros(29)
        for i in range(29):
            q[i] = self.msg.motor_cmd[i].q
        return q

    def _ramp_cmd(self, ramp, now, q_target):
        """Resolve one init ramp at time `now`.
        Returns ('limp'|'blend'|'track', q_cmd):
          limp  -> ramp not started yet; drive the group with zero gains
          blend -> smoothstep from the captured start pose toward q_target
          track -> ramp finished (or none armed); follow q_target directly
        """
        if self._ramp_t0 is None:
            return "track", q_target
        t_begin = self._ramp_t0 + ramp["delay"]
        if now < t_begin:
            return "limp", q_target
        a = min((now - t_begin) / ramp["duration"], 1.0) if ramp["duration"] > 0 else 1.0
        if a >= 1.0:
            return "track", q_target
        if ramp["q_start"] is None:
            ramp["q_start"] = self._ramp_capture()
        s = a * a * (3.0 - 2.0 * a)
        return "blend", (1.0 - s) * ramp["q_start"] + s * q_target

    def _apply_motor_cmds(self, q29, jids, limp=False):
        """Drive the given motors from a 29-element joint position array.
        limp=True -> zero gains (joints hang free); limp=False -> full PD."""
        for jid in jids:
            kp, kd, tau = JOINT_GAINS_BY_ID[jid.value]
            self.msg.motor_cmd[jid].mode = 0 if limp else 1
            self.msg.motor_cmd[jid].kp = 0.0 if limp else kp
            self.msg.motor_cmd[jid].kd = 0.0 if limp else kd
            self.msg.motor_cmd[jid].dq = 0.0
            self.msg.motor_cmd[jid].tau = tau
            self.msg.motor_cmd[jid].q = float(q29[jid.value])

    def _apply_arm_cmds(self, q29, limp=False):
        """Arm motor commands (limp=True -> arms hang free)."""
        self._apply_motor_cmds(q29, self._arm_joint_ids, limp=limp)

    def _apply_leg_cmds(self, q29):
        """Leg (hip/knee/ankle) motor commands (whole-body mode)."""
        self._apply_motor_cmds(q29, self._leg_joint_ids)

    def _apply_waist_cmds(self, q29, limp=False):
        """Waist (torso) motor commands (limp=True -> torso hangs free)."""
        self._apply_motor_cmds(q29, self._waist_joint_ids, limp=limp)

    def _publish_lowcmd(self):
        """Sync mode_machine, set mode_pr, compute CRC and publish LowCmd."""
        self.msg.mode_machine = self.get_mode_machine()
        # self.get_logger().info(f"Mode machine = '{self.msg.mode_machine}'")
        self.msg.mode_pr = 0 if self.use_whole_body else 1
        try:
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
        except Exception:
            pass
        self.msg.crc = self.crc.Crc(self.msg)
        if self.send_cmds_to_robot:
            self._lowcmd_attempts += 1
            try:
                self.lowcmd_publisher.Write(self.msg)
                self._lowcmd_count += 1
            except Exception as e:
                self.get_logger().warn(f"[LowCmd] Write failed: {e}")

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

    def _do_full_reset(self):
        """Clear hand refs, reset OpenSoT q to q_init, reset WB init state, move markers off-thread."""
        self._resetting = True
        self.right_hand_pose_ref = None
        self.left_hand_pose_ref = None
        for name in list(self.marker_enabled):
            self.marker_enabled[name] = False
        self.reset_opensot()
        # Re-arm the init ramp: _ramp_t0=None makes control_loop pick a fresh
        # profile on the next tick. Since _opensot_ever_started is already True,
        # that's the reset-restart profile (legs+arms together, 3 s, no limp).
        self._ramp_t0 = None
        self._leg_ramp["q_start"] = None
        self._arm_ramp["q_start"] = None
        self._waist_ramp["q_start"] = None
        self._park_active = False
        self._resetting = False
        # threading.Thread(target=self._reset_marker_poses, daemon=True).start()

    def reset_callback(self, request, response):
        """Service callback for /g1pilot/reset. Re-initializes if no hand commands are active."""
        if self._resetting:
            response.success = False
            response.message = "Reset already in progress"
            return response

        elapsed_since_last = time.monotonic() - self._last_hand_ref_time
        if elapsed_since_last < 1.0:
            response.success = False
            response.message = "Hand references received less than 1s ago, aborting reset"
            self.get_logger().warn(f"[Reset] Aborted: hand reference received {elapsed_since_last:.2f}s ago")
            return response

        self.get_logger().info("[Reset] Re-initializing...")
        self._do_full_reset()
        self.get_logger().info("[Reset] Initialization complete")

        response.success = True
        response.message = "Reset and re-initialization complete"
        return response

    def park_callback(self, request, response):
        """Service callback for /g1pilot/park. Eases legs+waist from the last
        command to PARK_POSE over 15 s while the arms hang limp — the shutdown
        / rest pose. Whole-body only (legs aren't controlled in arm_sdk mode)."""
        if self._resetting:
            response.success = False
            response.message = "Reset in progress"
            return response
        if not self.use_whole_body:
            response.success = False
            response.message = "Park requires whole-body mode (legs not controlled in arm_sdk)"
            self.get_logger().warn("[Park] Ignored: not in whole-body mode")
            return response

        # Drop hand targets/markers so nothing fights the descent (OpenSoT's
        # output is ignored while parked anyway).
        self.right_hand_pose_ref = None
        self.left_hand_pose_ref = None
        for name in list(self.marker_enabled):
            self.marker_enabled[name] = False

        # Two ramps off a shared t0, blending from the last command (the robot
        # is tracking it) toward PARK_POSE. Legs ease first over 15 s; the waist
        # holds, then follows after a 10 s delay over 5 s. Arms are forced limp
        # in the control loop, so the arm ramp is unused here.
        self._ramp_capture = self.get_last_cmd_q
        self._leg_ramp = {"delay": 0.0, "duration": 15.0, "q_start": None}
        self._waist_ramp = {"delay": 0.0, "duration": 15.0, "q_start": None}
        self._ramp_t0 = time.time()
        self._park_active = True
        self.get_logger().info("[Park] Easing to shutdown pose (legs 15 s, waist after 10 s, arms limp)")

        response.success = True
        response.message = "Parking to shutdown pose"
        return response

    def start_opensot_callback(self, msg: Bool):
        self.start_opensot = bool(msg.data)

    def emergency_stop_callback(self, msg: Bool):
        self.emergency_stop = bool(msg.data)

    def scanning_mode_callback(self, msg: Int8):
        new_mode = int(msg.data)
        if new_mode == self.scanning_mode or self._resetting:
            return
        self.scanning_mode = new_mode
        if new_mode == 1:
            self._enter_scanning()
        else:
            self._exit_scanning()

    def _enter_scanning(self):
        if not self.start_opensot:
            self.get_logger().warn("[Scan] start_opensot is False; ignoring")
            self.scanning_mode = 0
            return
        self._do_full_reset()
        self._scan_torso_base_pose = self.model.getPose("torso_link")
        self.torso.setReference(self._scan_torso_base_pose)
        self._activate_stack(self._scan_stack)
        self._scan_t = 0.0
        self._scan_active = True
        self._set_led(*LED_SCANNING)
        # self._say_async("Entering ... scanning ... mode")  # speech moved to tts_node
        self.get_logger().info("[Scan] Entering scanning mode")

    def _set_led(self, r, g, b):
        if self.audio_client is None:
            return
        try:
            self.audio_client.LedControl(r, g, b)
        except Exception as e:
            self.get_logger().error(f"[Led] failed: {e}")

    # Speech moved to the dedicated g1pilot tts_node (publish to /g1pilot/say).
    # def _say_async(self, text):
    #     """Speak text on the G1 speaker without blocking the control loop."""
    #     if self.audio_client is None:
    #         return
    #     if self._say_thread is not None and self._say_thread.is_alive():
    #         return
    #     self._say_thread = threading.Thread(target=self._say_blocking, args=(text,), daemon=True)
    #     self._say_thread.start()
    #
    # def _say_blocking(self, text):
    #     try:
    #         pcm = synthesize_pcm(text)
    #         if not pcm:
    #             return
    #         stream_id = str(int(time.time() * 1000))
    #         start = time.monotonic()
    #         for offset in range(0, len(pcm), CHUNK_SIZE):
    #             play_stream(self.audio_client, stream_id, pcm[offset:offset + CHUNK_SIZE])
    #             time.sleep(1)
    #         audio_seconds = len(pcm) / 32000.0
    #         remaining = audio_seconds + 0.5 - (time.monotonic() - start)
    #         if remaining > 0:
    #             time.sleep(remaining)
    #         play_stop(self.audio_client, stream_id)
    #     except Exception as e:
    #         self.get_logger().error(f"[Say] failed: {e}")

    def _exit_scanning(self):
        self._scan_active = False
        if self._scan_torso_base_pose is not None:
            self.torso.setReference(self._scan_torso_base_pose)
        self._do_full_reset()
        self._activate_stack(self._teleop_stack)
        self._set_led(*LED_OPERATING)
        # self._say_async("Exiting ... scanning ... mode, ... Returning ... to ... operation ... mode")  # speech moved to tts_node
        self.get_logger().info("[Scan] Exiting scanning mode")

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

    def _reset_marker_poses(self):
        """Move existing interactive markers back to current hand FK poses (no rebuild)."""
        right_hand_ref, _ = self.right_gripper.getReference()
        left_hand_ref, _ = self.left_gripper.getReference()
        self._set_marker_pose("right_hand_marker", right_hand_ref, self.right_hand_frame_ref)
        self._set_marker_pose("left_hand_marker", left_hand_ref, self.left_hand_frame_ref)

    def _set_marker_pose(self, name, pose, frame_id):
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = pose.translation[0]
        ps.pose.position.y = pose.translation[1]
        ps.pose.position.z = pose.translation[2]
        quat_xyzw = R.from_matrix(pose.linear).as_quat()  # [x, y, z, w]
        ps.pose.orientation.x = quat_xyzw[0]
        ps.pose.orientation.y = quat_xyzw[1]
        ps.pose.orientation.z = quat_xyzw[2]
        ps.pose.orientation.w = quat_xyzw[3]
        self.marker_poses[name] = ps
        if name in self.marker_home_poses:
            self.marker_home_poses[name].pose = ps.pose
        self.interactive_marker_server.setPose(name, ps.pose, ps.header)
        self.interactive_marker_server.applyChanges()

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

        # Compute world -> pelvis so that feet rest on the ground.
        # With pelvis at origin, run FK to find the left foot pose,
        # then invert it to get the pelvis pose in a frame where the left foot is at the origin.
        self.model.setJointPosition(self.q)
        self.model.update()
        # Match robot_state convention: world origin = left foot contact
        # Inverse of affine T=(R,t) is (R^T, -R^T @ t)
        left_foot_pose = self.model.getPose("left_foot_point_contact")
        R_inv = left_foot_pose.linear.T
        t_inv = -R_inv @ left_foot_pose.translation
        quat = R.from_matrix(R_inv).as_quat()  # [x, y, z, w]
        self.q[0] = t_inv[0]
        self.q[1] = t_inv[1]
        self.q[2] = t_inv[2]
        self.q[3] = quat[0]
        self.q[4] = quat[1]
        self.q[5] = quat[2]
        self.q[6] = quat[3]

        self.dq = np.zeros(self.model.nv)

        self.model.setJointPosition(self.q)
        self.model.setJointVelocity(self.dq)
        self.model.update()

        self.com = CoM(self.model)
        # CoM X offset (+X = forward). TEST: push 5 cm forward to counteract the
        # new robot's backward lean (real CoM sits behind the modeled one).
        # Was -0.01 (1 cm backward) for the old robot. Tune this value.
        com_ref_init, _ = self.com.getReference()
        com_ref_init[0] -= 0.01  # 5 cm forward (test)
        self.com.setReference(com_ref_init)
        self.com.setLambda(0.05)

        self.get_logger().warning("Initializing OpenSoT Tasks and Constraints")

        self.get_logger().info("Task: Base")
        self.base = Cartesian("base_task", self.model, "pelvis", manipulation_frame)
        self.base.setLambda(0.1)

        self.get_logger().info("Task: Torso")
        self.torso = Cartesian("torso_task", self.model, "torso_link", manipulation_frame)
        self.torso.setLambda(0.01)

        self.get_logger().info("Task: Pelvis")
        self.pelvis = Cartesian("pelvis_task", self.model, "pelvis", manipulation_frame)
        self.pelvis.setLambda(0.01)

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
        self.postural.setLambda(0.01) # 0.2 or 0.1 ? coman uses 0.01 ?
        self.W = self.postural.getWeight().copy()
        if self.use_whole_body:
            # Zero floating base DOFs so the postural task doesn't fight
            # against leg bending by trying to restore the pelvis pose
            self.W[0:6, 0:6] = 0.0
            # Always target q_init regardless of actual robot state at startup
            q_ref = np.zeros(self.model.nv)
            q_ref[6:] = q_posture_ref
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
            self._build_whole_body_contact_tasks()
            self._teleop_stack = self._build_stack_whole_body()
            self._scan_stack = self._build_stack_scanning_whole_body()
        else:
            self._teleop_stack = self._build_stack_arm_sdk()
            self._scan_stack = self._build_stack_scanning_arm_sdk()
        self._teleop_stack.update()
        self._scan_stack.update()
        self._teleop_solver = pysot.iHQP(self._teleop_stack, eps_regularisation=1e10)
        self._scan_solver = pysot.iHQP(self._scan_stack, eps_regularisation=1e10)
        self._activate_stack(self._teleop_stack)


    def _build_whole_body_contact_tasks(self):
        """Define foot Cartesian tasks and the pelvis-height constraint used in whole-body mode."""
        self.get_logger().info("Task: Left Foot (contact constraint)")
        self.left_foot = Cartesian("left_foot_task", self.model, "left_ankle_roll_link", "world")
        self.get_logger().info("Task: Right Foot (contact constraint)")
        self.right_foot = Cartesian("right_foot_task", self.model, "right_ankle_roll_link", "world")

        torso_pos = self.torso.getActualPose()[:3, 3]
        A_torso = np.array([[0.0, 0.0, 1.0]])
        b_torso = np.array([torso_pos[2]])
        self.pelvis_height_constraint = CartesianPositionConstraint(
            self.torso, A_torso, b_torso, 1.0
        )
        self.get_logger().info(f"Constraint: Torso height <= {torso_pos[2]:.4f} m")

    def _build_stack_arm_sdk(self):
        """Stack for arm_sdk mode (legs handled externally by Unitree's controller)."""
        stack = (
            self.base
            / (self.torso % [3, 4, 5] + self.right_gripper + self.left_gripper)
            / self.postural
        ) << self.qlims << self.dqlims
        if self.enable_collision_avoidance:
            stack = stack << self.collision_avoidance_constraint
        return stack

    def _build_stack_whole_body(self):
        """Stack for whole-body mode (OpenSoT controls legs; feet pinned, CoM kept stable)."""
        if self.enable_collision_avoidance:
            stack = (
                self.com % [0, 1]
                / (self.right_gripper + self.left_gripper)
                / (self.torso % [3, 4] + self.pelvis % [0, 1, 3, 4, 5] + 0.1 * self.pelvis % [2])
                / self.postural
            ) << self.qlims << self.dqlims << self.collision_avoidance_constraint
        else:
            stack = (
                (self.com % [0, 1] + self.right_gripper + self.left_gripper)
                / self.torso_rp
                / self.postural
            ) << self.qlims << self.dqlims
        return stack << self.pelvis_height_constraint << (self.left_foot + self.right_foot)

    def _build_stack_scanning_arm_sdk(self):
        """Scanning stack for arm_sdk mode: torso oscillation + postural, no hand tracking."""
        stack = (
            self.base
            / self.torso % [3, 4, 5]
            / self.postural
        ) << self.qlims << self.dqlims
        if self.enable_collision_avoidance:
            stack = stack << self.collision_avoidance_constraint
        return stack

    def _build_stack_scanning_whole_body(self):
        """Scanning stack for whole-body mode: torso oscillation + balance, no hand tracking."""
        stack = (
            self.com % [0, 1]
            / (self.torso % [2, 3, 4, 5] + self.pelvis % [0, 1, 3, 4, 5] ) #0.1 * self.pelvis % [2])
            # / self.postural
        ) << self.qlims << self.dqlims
        if self.enable_collision_avoidance:
            stack = stack << self.collision_avoidance_constraint
        return stack << (self.left_foot + self.right_foot)

    def _activate_stack(self, stack):
        """Switch the active stack/solver pair. Both solvers are pre-built in initialize()
        so each keeps its qpOASES warm-start state across mode toggles."""
        self.stack = stack
        self.solver = self._scan_solver if stack is self._scan_stack else self._teleop_solver
        self.stack.update()

    def reset_opensot(self):
        """Reset q/dq to init pose and update the model. Tasks, stack, and solver are kept."""
        self.get_logger().warning("Resetting OpenSoT state (q, dq)")

        self.q = np.zeros(self.model.nq)
        self.q[6] = 1.0  # quaternion w
        self.q[7:] = q_init[0:].copy()

        self.model.setJointPosition(self.q)
        self.model.update()
        left_foot_pose = self.model.getPose("left_foot_point_contact")
        R_inv = left_foot_pose.linear.T
        t_inv = -R_inv @ left_foot_pose.translation
        quat = R.from_matrix(R_inv).as_quat()  # [x, y, z, w]
        self.q[0] = t_inv[0]
        self.q[1] = t_inv[1]
        self.q[2] = t_inv[2]
        self.q[3] = quat[0]
        self.q[4] = quat[1]
        self.q[5] = quat[2]
        self.q[6] = quat[3]

        self.dq = np.zeros(self.model.nv)

        self.model.setJointPosition(self.q)
        self.model.setJointVelocity(self.dq)
        self.model.update()

        # Reset gripper task references to current FK poses so the solver
        # doesn't drive the robot back to stale (pre-reset) targets.
        self.right_gripper.setReference(self.model.getPose("right_hand_point_contact"))
        self.left_gripper.setReference(self.model.getPose("left_hand_point_contact"))

    # ----------------------------
    # Control loop
    # ----------------------------
    def _run_scan_step(self):
        """Oscillate torso yaw reference about the q_init torso orientation."""
        self._scan_t += self.control_dt
        yaw = SCAN_AMPLITUDE_RAD * np.sin(2.0 * np.pi * SCAN_FREQ_HZ * self._scan_t)
        Rz = R.from_euler('z', yaw).as_matrix()
        torso_ref = pyaffine3.Affine3()
        torso_ref.linear = Rz @ self._scan_torso_base_pose.linear
        base_t = self._scan_torso_base_pose.translation
        dz = -np.cos(2.0 * np.pi * SCAN_FREQ_HZ * 0.8 * self._scan_t) * 0.02 + 0.02
        torso_ref.translation = np.array([base_t[0], base_t[1], base_t[2] + dz])
        self.torso.setReference(torso_ref)

    def _apply_hand_ref(self, gripper_task, marker_name, pose_ref, frame_ref):
        """Update gripper task reference from interactive marker or external pose_ref."""
        if self.marker_enabled.get(marker_name, False) and marker_name in self.marker_poses:
            ps = self.marker_poses[marker_name]
        elif pose_ref is None:
            return
        elif pose_ref.header.frame_id != frame_ref:
            self.get_logger().error(
                f"Hand pose ref frame mismatch: got '{pose_ref.header.frame_id}', expected '{frame_ref}'"
            )
            return
        else:
            ps = pose_ref
        T = pyaffine3.Affine3()
        T.translation = np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
        T.linear = R.from_quat([
            ps.pose.orientation.x, ps.pose.orientation.y,
            ps.pose.orientation.z, ps.pose.orientation.w,
        ]).as_matrix()
        gripper_task.setReference(T)

    def control_loop(self):
        now = time.perf_counter()
        dt_log = now - self._lowcmd_log_t0
        if dt_log >= 0.5:
            failed = self._lowcmd_attempts - self._lowcmd_count
            self.get_logger().info(
                f"[LowCmd] sent {self._lowcmd_count}/{self._lowcmd_attempts} "
                f"(failed {failed}) in {dt_log*1e3:.0f} ms "
                f"({self._lowcmd_count / dt_log:.1f} Hz ok, "
                f"{self._lowcmd_attempts / dt_log:.1f} Hz attempted)"
            )
            self._lowcmd_count = 0
            self._lowcmd_attempts = 0
            self._lowcmd_log_t0 = now

        if self._resetting:
            # Hold the last command so joints keep their torque while the reset
            # runs on another thread; otherwise the robot goes limp and falls.
            if (
                self.use_robot
                and self.send_cmds_to_robot
                and self.lowcmd_publisher is not None
                and self.msg is not None
                and not self.emergency_stop
            ):
                self._publish_lowcmd()
            return

        # Whole-body gradual init: run the OpenSoT solver from the start, so
        # self.q evolves toward its steady-state pose (CoM offset, etc.) over
        # the warmup window. The published joint command is a smoothstep blend
        # from the last commanded q into self.q[7:], so the motors transition
        # directly to OpenSoT's converged pose — not via q_init, which would
        # otherwise cause a visible lean once OpenSoT engages and pulls the
        # CoM target.
        if self.use_whole_body and self.start_opensot and self._ramp_t0 is None:
            if not self._opensot_ever_started:
                # Very first start: legs ramp over 15 s; the torso hangs limp for
                # 10 s then ramps over 5 s; the arms hang limp for 15 s then ramp
                # over 5 s. Blend from measured TF.
                self._leg_ramp = {"delay": 0.0, "duration": 15.0, "q_start": None}
                self._waist_ramp = {"delay": 10.0, "duration": 5.0, "q_start": None}
                self._arm_ramp = {"delay": 15.0, "duration": 5.0, "q_start": None}
                self._ramp_capture = self.get_current_motor_q
                self.get_logger().info("[WB] start_opensot: first bring-up (legs 15s, torso limp 10s+5s, arms 15s+5s)")
            else:
                # Reset-restart: legs+waist+arms ramp together over 3 s with full
                # PD, no limp. Blend from the last command (robot held there).
                self._leg_ramp = {"delay": 0.0, "duration": 3.0, "q_start": None}
                self._waist_ramp = {"delay": 0.0, "duration": 3.0, "q_start": None}
                self._arm_ramp = {"delay": 0.0, "duration": 3.0, "q_start": None}
                self._ramp_capture = self.get_last_cmd_q
                self.get_logger().info("[WB] start_opensot: reset restart (legs+waist+arms 3s together)")
            self._ramp_t0 = time.time()
            self._opensot_ever_started = True

        if self.start_opensot and not self.emergency_stop:
            self.model.setJointPosition(self.q)
            self.model.setJointVelocity(self.dq)
            self.model.update()

            if self._scan_active:
                self._run_scan_step()
            else:
                self._apply_hand_ref(
                    self.right_gripper, "right_hand_marker",
                    self.right_hand_pose_ref, self.right_hand_frame_ref,
                )
                self._apply_hand_ref(
                    self.left_gripper, "left_hand_marker",
                    self.left_hand_pose_ref, self.left_hand_frame_ref,
                )

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

            now = time.time()
            q_target = self.q[7:]

            if self._park_active:
                # Shutdown ramp: arms limp, legs ease to the park pose, then the
                # waist follows after its delay (OpenSoT output ignored).
                self._apply_arm_cmds(q_target, limp=True)
                # Legs: ramp immediately.
                _, leg_cmd = self._ramp_cmd(self._leg_ramp, now, PARK_POSE)
                self._apply_leg_cmds(leg_cmd)
                # Waist: hold at the last command until its delay elapses
                # (skip applying = full-PD hold), then ramp.
                waist_state, waist_cmd = self._ramp_cmd(self._waist_ramp, now, PARK_POSE)
                if waist_state != "limp":
                    self._apply_waist_cmds(waist_cmd)
            else:
                # Arms: limp until their ramp begins, then blend, then track.
                arm_state, arm_cmd = self._ramp_cmd(self._arm_ramp, now, q_target)
                self._apply_arm_cmds(arm_cmd, limp=(arm_state == "limp"))

                if self.use_whole_body:
                    # Legs ramp from t0 (never limp).
                    _, leg_cmd = self._ramp_cmd(self._leg_ramp, now, q_target)
                    self._apply_leg_cmds(leg_cmd)
                    # Torso: limp until its ramp begins (first bring-up only;
                    # delay 0 on reset means it never goes limp), then blend.
                    waist_state, waist_cmd = self._ramp_cmd(self._waist_ramp, now, q_target)
                    self._apply_waist_cmds(waist_cmd, limp=(waist_state == "limp"))
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

#!/usr/bin/env python3

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Keyboard teleoperation for the Unitree Z1 manipulator in ROS2.

The node drives the ``joint_trajectory_controller`` through its
``~/joint_trajectory`` topic interface, so the very same code works on the
simulated and on the real robot, provided the bringup is started with
``starting_controller:=joint_trajectory_controller``.

Two control modes are available:

* **joint mode**: pick a joint with ``1``..``6`` and jog it with ``w``/``s``;
* **cartesian mode**: jog the pose of the end effector with the ``wasd``
  cluster, plus ``r``/``f`` for the vertical axis and ``t``/``g``, ``y``/``h``,
  ``u``/``j`` for the roll/pitch/yaw angles.  Cartesian jogging needs a running
  ``move_group`` because the inverse kinematics is delegated to its
  ``/compute_ik`` service.

The node must own a terminal to read the keystrokes, hence it is meant to be
run in a dedicated shell::

    ros2 run z1_examples teleop_keyboard.py

Ever since ROS2, ``ros2 launch`` does not forward the standard input to the
launched processes, so starting this node from a launch file only works when
the launcher inherits a controlling terminal (i.e. when ``ros2 launch`` itself
is run from an interactive shell).
"""

import os
import select
import sys
import termios
import threading
import time
import tty
from queue import Queue

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener


#  ___      _ _   _       _ _           _
# |_ _|_ __(_) |_(_) __ _| (_)______ _| |_ ___ _ __
#  | || '_ \| | __| |/ _` | | |_  / _` | __/ _ \ '__|
#  | || | | | | |_| | (_| | | |/ / (_| | ||  __/ |
# |___|_| |_|_|\__|_|\__,_|_|_/___\__,_|\__\___|_|
#

ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_JOINT = "jointGripper"

# Joint limits as defined in z1_description/urdf/const.xacro.  They are used to
# reject commands that would drive a joint past its mechanical end stop, which
# the joint_trajectory_controller would happily forward to the hardware.
JOINT_LIMITS = {
    "joint1": (-2.6179938779914944, 2.6179938779914944),
    "joint2": (0.0, 2.9670597283903604),
    "joint3": (-2.8797932657906435, 0.0),
    "joint4": (-1.5184364492350666, 1.5184364492350666),
    "joint5": (-1.3439035240356338, 1.3439035240356338),
    "joint6": (-2.792526803190927, 2.792526803190927),
    "jointGripper": (-1.5707963267948966, 0.0),
}

# The gripper fingers touch at q = 0 and are farthest apart at q = -pi/2, as
# measured on the meshes shipped within z1_description.
GRIPPER_CLOSED = 0.0
GRIPPER_OPEN = -1.5707963267948966

# Escape sequences sent by the arrow keys, mapped to symbolic tokens.
ARROW_KEYS = {b"A": "UP", b"B": "DOWN", b"C": "RIGHT", b"D": "LEFT"}

HELP_TEXT = """\
============================ Z1 keyboard teleop ============================
  m            switch between joint and cartesian mode
  [ / ]        decrease / increase the step size (current: {step})
  space        stop: hold the current position
  ?            show this help
  q            quit (Ctrl-C works as well)

  joint mode
    1 .. 6     select the joint to jog (current: {joint})
    w / s      jog the selected joint forward / backward
    o / c      open / close the gripper

  cartesian mode  (needs a running move_group)
    w / s      move along +X / -X of the robot base
    a / d      move along +Y / -Y
    r / f      move along +Z / -Z
    t / g      rotate about +X / -X (roll)
    y / h      rotate about +Y / -Y (pitch)
    u / j      rotate about +Z / -Z (yaw)
    o / c      open / close the gripper
============================================================================
"""


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product of two (x, y, z, w) quaternions."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quat_from_axis_angle(axis: str, angle: float) -> np.ndarray:
    """Quaternion of a rotation of ``angle`` around a base frame axis."""
    half = angle / 2.0
    s = np.sin(half)
    return {
        "x": np.array([s, 0.0, 0.0, np.cos(half)]),
        "y": np.array([0.0, s, 0.0, np.cos(half)]),
        "z": np.array([0.0, 0.0, s, np.cos(half)]),
    }[axis]


#  _  __          _                         _
# | |/ /___ _   _| |__   ___   __ _ _ __ __| |
# | ' // _ \ | | | '_ \ / _ \ / _` | '__/ _` |
# | . \  __/ |_| | |_) | (_) | (_| | | | (_| |
# |_|\_\___|\__, |_.__/ \___/ \__,_|_|  \__,_|
#           |___/
#


class KeyboardReader(threading.Thread):
    """Read single keystrokes from a terminal in raw mode.

    The reader owns the terminal only for the time strictly required to fetch a
    key, so that the shell does not stay in raw mode if the node crashes.
    """

    def __init__(self, queue: Queue):
        super().__init__(daemon=True)
        self._queue = queue
        self._stop_event = threading.Event()
        self._fd = None
        self._saved_attributes = None

    # -- setup ---------------------------------------------------------------
    def _open_terminal(self) -> int:
        """Return a file descriptor of a terminal, or fail with a clear error."""
        try:
            if os.isatty(sys.stdin.fileno()):
                return sys.stdin.fileno()
        except (ValueError, OSError):
            pass

        try:
            return os.open("/dev/tty", os.O_RDONLY)
        except OSError as exc:
            raise RuntimeError(
                "no terminal available for reading the keystrokes. When the "
                "node is started through 'ros2 launch', the standard input is "
                "not forwarded: start 'ros2 run z1_examples teleop_keyboard.py'"
                " from a dedicated shell instead."
            ) from exc

    # -- thread body ---------------------------------------------------------
    def run(self):
        try:
            self._fd = self._open_terminal()
            self._saved_attributes = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        except (RuntimeError, termios.error) as exc:
            self._queue.put(("ERROR", str(exc)))
            return

        try:
            while not self._stop_event.is_set():
                if not select.select([self._fd], [], [], 0.1)[0]:
                    continue
                data = os.read(self._fd, 8)
                if not data:
                    break
                for token in self._decode(data):
                    self._queue.put(("KEY", token))
        finally:
            self.restore()

    def _decode(self, data: bytes):
        """Turn raw bytes into tokens, collapsing arrow key escape sequences."""
        tokens = []
        i = 0
        while i < len(data):
            if data[i:i + 1] == b"\x1b" and data[i + 1:i + 2] == b"[":
                tokens.append(ARROW_KEYS.get(data[i + 2:i + 3], "UNKNOWN"))
                i += 3
            else:
                tokens.append(data[i:i + 1].decode("utf-8", "ignore"))
                i += 1
        return tokens

    def stop(self):
        self._stop_event.set()

    def restore(self):
        if self._fd is not None and self._saved_attributes is not None:
            try:
                termios.tcsetattr(
                    self._fd, termios.TCSADRAIN, self._saved_attributes
                )
            except termios.error:
                pass
            self._saved_attributes = None


#  _  __          _                     _____
# | |/ /___ _   _| |__   ___   _ __ ___|_   _|__ _ __ ___  _ __
# | ' // _ \ | | | '_ \ / _ \ | '__/ _ \| |/ _ \ '_ ` _ \| '_ \
# | . \  __/ |_| | |_) | (_) || | |  __/| |  __/ | | | | | |_) |
# |_|\_\___|\__, |_.__/ \___/ |_|  \___||_|\___|_| |_| |_| .__/
#           |___/                                        |_|
#


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__("z1_keyboard_teleop")

        # --- Parameters -----------------------------------------------------
        self.declare_parameter("controller_name", "joint_trajectory_controller")
        self.declare_parameter("gripper_controller_name", "gripper_controller")
        self.declare_parameter("joint_topic", "joint_states")
        self.declare_parameter("base_frame", "link00")
        self.declare_parameter("tip_frame", "link06")
        self.declare_parameter("planning_group", "z1_arm")
        self.declare_parameter("publish_rate", 20.0)
        # How far ahead of the message timestamp the streaming setpoint is
        # scheduled.  Two competing effects set this value:
        #  * the joint_trajectory_controller drops a trajectory whose start
        #    time is non-zero and whose end already lies in the past
        #    ("Received trajectory with non-zero start time ... that ends in
        #    the past"), which happens when the message waits in the queue for
        #    longer than this value: raise it if that warning shows up;
        #  * every published setpoint restarts the interpolation from the
        #    current state, so the longer the horizon the slower the arm
        #    catches up with the commanded target.
        self.declare_parameter("lookahead", 0.15)
        self.declare_parameter("joint_step", 0.03490658503988659)   # 2 deg
        self.declare_parameter("cartesian_step", 0.005)             # 5 mm
        self.declare_parameter("rotation_step", 0.03490658503988659)
        self.declare_parameter("max_step_scale", 8.0)
        self.declare_parameter("max_lead", 0.35)
        self.declare_parameter("idle_resync_delay", 1.0)
        self.declare_parameter("settle_tolerance", 0.01)
        self.declare_parameter("ik_timeout", 0.05)

        controller_name = self.get_parameter("controller_name").value
        gripper_controller = self.get_parameter("gripper_controller_name").value
        joint_topic = self.get_parameter("joint_topic").value
        self._base_frame = self.get_parameter("base_frame").value
        self._tip_frame = self.get_parameter("tip_frame").value
        self._group = self.get_parameter("planning_group").value
        self._rate = float(self.get_parameter("publish_rate").value)
        self._lookahead = float(self.get_parameter("lookahead").value)
        self._joint_step = float(self.get_parameter("joint_step").value)
        self._cartesian_step = float(self.get_parameter("cartesian_step").value)
        self._rotation_step = float(self.get_parameter("rotation_step").value)
        self._max_step_scale = float(self.get_parameter("max_step_scale").value)
        self._max_lead = float(self.get_parameter("max_lead").value)
        self._idle_resync_delay = float(
            self.get_parameter("idle_resync_delay").value
        )
        self._settle_tolerance = float(self.get_parameter("settle_tolerance").value)
        self._ik_timeout = float(self.get_parameter("ik_timeout").value)

        # --- State ----------------------------------------------------------
        self._mode = "joint"
        self._selected_joint = 0
        self._step_scale = 1.0
        self._positions: dict[str, float] = {}
        self._joint_target = np.zeros(len(ARM_JOINTS))
        self._gripper_target = GRIPPER_CLOSED
        self._cart_position = np.zeros(3)
        self._cart_quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        self._cart_dirty = False
        self._ik_pending = False
        self._have_state = False
        self._last_motion_time = 0.0
        self._last_ik_warning = 0.0
        self._warned_no_controller = False

        # --- Interfaces -----------------------------------------------------
        self._callback_group = ReentrantCallbackGroup()
        self._joint_pub = self.create_publisher(
            JointTrajectory, f"/{controller_name}/joint_trajectory", 10
        )
        self._gripper_pub = self.create_publisher(
            Float64MultiArray, f"/{gripper_controller}/commands", 10
        )
        self.create_subscription(
            JointState, f"/{joint_topic}", self._on_joint_state, 10
        )
        self._ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=self._callback_group
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._key_queue: Queue = Queue()
        self._keyboard = KeyboardReader(self._key_queue)
        self._keyboard.start()

        self.create_timer(1.0 / self._rate, self._on_timer)

        self._print_help()
        self.get_logger().info(
            f"Streaming to /{controller_name}/joint_trajectory at {self._rate} Hz"
        )

    # -- logging helpers -----------------------------------------------------
    def _print_help(self):
        print(HELP_TEXT.format(
            step=self._step_description(), joint=self._joint_label()
        ), flush=True)

    def _step_description(self) -> str:
        scale = self._step_scale
        if self._mode == "joint":
            return f"{np.rad2deg(self._joint_step * scale):.1f} deg"
        return (
            f"{self._cartesian_step * scale * 1000:.1f} mm / "
            f"{np.rad2deg(self._rotation_step * scale):.1f} deg"
        )

    def _joint_label(self) -> str:
        return ARM_JOINTS[self._selected_joint]

    # -- state handling ------------------------------------------------------
    def _on_joint_state(self, msg: JointState):
        self._positions = dict(zip(msg.name, msg.position))
        if not self._have_state and all(j in self._positions for j in ARM_JOINTS):
            # Adopt the measured configuration as the initial command, so that
            # the very first published trajectory does not move the robot.
            self._joint_target = np.array(
                [self._positions[j] for j in ARM_JOINTS]
            )
            if GRIPPER_JOINT in self._positions:
                self._gripper_target = self._positions[GRIPPER_JOINT]
            self._have_state = True

    def _on_timer(self):
        self._drain_keys()
        self._refresh_cartesian_target()
        self._publish()

    def _drain_keys(self):
        while not self._key_queue.empty():
            kind, payload = self._key_queue.get_nowait()
            if kind == "ERROR":
                self.get_logger().error(payload)
                rclpy.shutdown()
                return
            self._on_key(payload)

    # -- key dispatch --------------------------------------------------------
    def _on_key(self, key: str):
        if key in ("q", "\x03", "\x04"):     # q, Ctrl-C, Ctrl-D
            self.get_logger().info("Quitting")
            rclpy.shutdown()
            return
        if key == "?":
            self._print_help()
            return
        if key == "m":
            self._toggle_mode()
            return
        if key == "[":
            self._scale_step(1.0 / 1.5)
            return
        if key == "]":
            self._scale_step(1.5)
            return
        if key == " ":
            self._hold()
            return
        if key in ("o", "c"):
            self._command_gripper(key == "o")
            return
        if self._mode == "joint":
            self._on_joint_key(key)
        else:
            self._on_cartesian_key(key)

    def _toggle_mode(self):
        if self._mode == "joint":
            if not self._ik_client.service_is_ready():
                self.get_logger().warn(
                    "/compute_ik is not available: start move_group (e.g. "
                    "'ros2 launch z1_moveit z1_moveit.launch.py') to enable "
                    "cartesian jogging"
                )
                return
            self._mode = "cartesian"
            self._sync_cartesian_target(force=True)
            label = "cartesian"
        else:
            self._mode = "joint"
            self._joint_target = self._measured_arm_positions()
            label = "joint"
        self.get_logger().info(f"{label} mode (step {self._step_description()})")

    def _scale_step(self, factor: float):
        self._step_scale = clamp(
            self._step_scale * factor, 1.0 / self._max_step_scale,
            self._max_step_scale
        )
        self.get_logger().info(f"step size: {self._step_description()}")

    def _hold(self):
        self._joint_target = self._measured_arm_positions()
        self._gripper_target = self._positions.get(
            GRIPPER_JOINT, self._gripper_target
        )
        self._sync_cartesian_target(force=True)
        self.get_logger().info("stopped")

    # -- joint mode ----------------------------------------------------------
    def _on_joint_key(self, key: str):
        if key in "123456":
            self._selected_joint = int(key) - 1
            self.get_logger().info(f"selected {self._joint_label()}")
            return

        direction = {"w": 1.0, "s": -1.0, "UP": 1.0, "DOWN": -1.0}.get(key)
        if direction is None:
            return

        joint = self._joint_label()
        step = self._joint_step * self._step_scale * direction
        lower, upper = JOINT_LIMITS[joint]
        self._joint_target[self._selected_joint] = clamp(
            self._joint_target[self._selected_joint] + step, lower, upper
        )
        self._last_motion_time = time.monotonic()

    def _measured_arm_positions(self) -> np.ndarray:
        return np.array([
            self._positions.get(j, t)
            for j, t in zip(ARM_JOINTS, self._joint_target)
        ])

    # -- cartesian mode ------------------------------------------------------
    def _on_cartesian_key(self, key: str):
        translation = {
            "w": (0, +1.0), "s": (0, -1.0), "UP": (0, +1.0), "DOWN": (0, -1.0),
            "a": (1, +1.0), "d": (1, -1.0), "LEFT": (1, +1.0), "RIGHT": (1, -1.0),
            "r": (2, +1.0), "f": (2, -1.0),
        }.get(key)
        if translation is not None:
            axis, sign = translation
            self._cart_position[axis] += (
                sign * self._cartesian_step * self._step_scale
            )
        else:
            rotation = {
                "t": ("x", +1.0), "g": ("x", -1.0),
                "y": ("y", +1.0), "h": ("y", -1.0),
                "u": ("z", +1.0), "j": ("z", -1.0),
            }.get(key)
            if rotation is None:
                return
            axis, sign = rotation
            delta = quat_from_axis_angle(
                axis, sign * self._rotation_step * self._step_scale
            )
            # Pre-multiply so that the rotation happens about the axes of the
            # robot base frame, which is what the operator expects.
            self._cart_quaternion = quat_multiply(delta, self._cart_quaternion)

        self._cart_quaternion /= np.linalg.norm(self._cart_quaternion)
        self._cart_dirty = True
        self._last_motion_time = time.monotonic()

    def _sync_cartesian_target(self, force: bool = False):
        """Align the cartesian target with the measured end effector pose.

        While the operator keeps jogging, the target is kept as it is: the
        clamp in :meth:`_apply_ik_solution` already prevents it from running
        away from the real robot.  Once the motion is over *and the robot has
        actually reached the commanded configuration* the two are re-aligned,
        so that the numerical error of the inverse kinematics cannot pile up.
        Re-aligning any earlier would silently drop the part of the motion
        that is still in flight.
        """
        if self._ik_pending:
            return
        if not force:
            if self._cart_dirty:
                return
            if time.monotonic() - self._last_motion_time < self._idle_resync_delay:
                return
            if not self._has_reached_target():
                return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, self._tip_frame, rclpy.time.Time()
            )
        except Exception:
            return
        t = transform.transform.translation
        q = transform.transform.rotation
        self._cart_position = np.array([t.x, t.y, t.z])
        self._cart_quaternion = np.array([q.x, q.y, q.z, q.w])

    def _has_reached_target(self) -> bool:
        """True when the robot sits on the configuration it was commanded."""
        return bool(np.all(
            np.abs(self._measured_arm_positions() - self._joint_target)
            < self._settle_tolerance
        ))

    def _refresh_cartesian_target(self):
        if self._mode != "cartesian" or self._ik_pending:
            return
        self._sync_cartesian_target()
        if not self._cart_dirty or not self._have_state:
            return
        if not self._ik_client.service_is_ready():
            self._cart_dirty = False
            self.get_logger().warn("/compute_ik unavailable, dropping command")
            return

        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self._group
        ik.ik_link_name = self._tip_frame
        ik.avoid_collisions = False
        ik.timeout = Duration(
            sec=int(self._ik_timeout), nanosec=int(self._ik_timeout % 1 * 1e9)
        )
        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self._cart_position[0])
        pose.pose.position.y = float(self._cart_position[1])
        pose.pose.position.z = float(self._cart_position[2])
        pose.pose.orientation = Quaternion(
            x=float(self._cart_quaternion[0]), y=float(self._cart_quaternion[1]),
            z=float(self._cart_quaternion[2]), w=float(self._cart_quaternion[3]),
        )
        ik.pose_stamped = pose
        ik.robot_state = RobotState()
        ik.robot_state.joint_state.name = list(ARM_JOINTS)
        ik.robot_state.joint_state.position = [
            float(v) for v in self._measured_arm_positions()
        ]

        self._cart_dirty = False
        self._ik_pending = True
        future = self._ik_client.call_async(request)
        future.add_done_callback(self._on_ik_done)

    def _on_ik_done(self, future):
        self._ik_pending = False
        try:
            response = future.result()
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn(f"IK request failed: {exc}")
            return

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self._warn_ik_failure(
                f"no IK solution for the requested end effector pose "
                f"(error code {response.error_code.val})"
            )
            self._sync_cartesian_target(force=True)
            return

        solution = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ))
        if not all(j in solution for j in ARM_JOINTS):
            self.get_logger().warn("IK solution is missing some arm joints")
            return
        self._apply_ik_solution(np.array([solution[j] for j in ARM_JOINTS]))

    def _warn_ik_failure(self, message: str):
        """Log at most one IK failure per second, to keep the terminal usable."""
        now = time.monotonic()
        if now - self._last_ik_warning > 1.0:
            self._last_ik_warning = now
            self.get_logger().warn(message)

    def _apply_ik_solution(self, joints: np.ndarray):
        """Accept an IK solution only if it is close to the real configuration.

        This keeps the commanded target at most ``max_lead`` ahead of the
        measured one, so that a burst of keystrokes cannot queue up a huge
        motion, and it rejects the elbow/shoulder flips that a redundant
        solution would otherwise cause.
        """
        measured = self._measured_arm_positions()
        if np.any(np.abs(joints - measured) > self._max_lead):
            self._warn_ik_failure(
                "IK solution too far from the current pose, command dropped"
            )
            self._sync_cartesian_target(force=True)
            return

        for i, joint in enumerate(ARM_JOINTS):
            lower, upper = JOINT_LIMITS[joint]
            joints[i] = clamp(joints[i], lower, upper)

        self._joint_target = joints
        self._last_motion_time = time.monotonic()

    # -- command publishing --------------------------------------------------
    def _command_gripper(self, open_gripper: bool):
        if GRIPPER_JOINT not in self._positions:
            self.get_logger().warn(
                f"'{GRIPPER_JOINT}' is not published on /joint_states: the "
                "robot was probably brought up with 'with_gripper:=false'"
            )
            return
        self._gripper_target = GRIPPER_OPEN if open_gripper else GRIPPER_CLOSED
        self.get_logger().info(
            f"gripper {'open' if open_gripper else 'closed'}"
        )

    def _publish(self):
        if not self._have_state:
            return

        if not self._joint_pub.get_subscription_count():
            if not self._warned_no_controller:
                self._warned_no_controller = True
                self.get_logger().warn(
                    "the joint_trajectory_controller is not listening on "
                    f"'{self._joint_pub.topic_name}': launch the bringup with "
                    "'starting_controller:=joint_trajectory_controller'"
                )
            return

        # Keep the target within `max_lead` of the real configuration, so that
        # holding a key cannot accumulate an unbounded motion request.
        measured = self._measured_arm_positions()
        self._joint_target = np.clip(
            self._joint_target, measured - self._max_lead, measured + self._max_lead
        )
        for i, joint in enumerate(ARM_JOINTS):
            lower, upper = JOINT_LIMITS[joint]
            self._joint_target[i] = clamp(self._joint_target[i], lower, upper)

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in self._joint_target]
        point.velocities = [0.0] * len(ARM_JOINTS)
        point.time_from_start = Duration(
            sec=int(self._lookahead),
            nanosec=int(self._lookahead % 1 * 1e9),
        )

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(ARM_JOINTS)
        trajectory.points = [point]
        self._joint_pub.publish(trajectory)

        if GRIPPER_JOINT in self._positions:
            command = Float64MultiArray()
            command.data = [float(self._gripper_target)]
            self._gripper_pub.publish(command)

    # -- teardown ------------------------------------------------------------
    def destroy_node(self):
        self._keyboard.stop()
        self._keyboard.restore()
        super().destroy_node()


#  __  __      _        _
# |  \/  |__ _(_)_ __  | |    __ _ _ _  __ _ _  _
# | |\/| / _` | | '_ \ | |__ / _` | ' \/ _` | || |
# |_|  |_\__,_|_| .__/ |____|\__,_|_||_\__, |\_,_|
#               |_|                    |___/
#


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # The terminal must go back to its normal behaviour even when the node
        # is killed while it is still reading the keystrokes.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

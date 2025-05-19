#!/usr/bin/env python3
import dataclasses
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from os import environ
from typing import (
    Any,
    Callable,
    Dict,
    Final,
    List,
    Literal,
    Optional,
    Set,
    Tuple,
    Union,
)

import numpy as np
import quaternion as qt
import rclpy
from keyboard_msgs.msg import Key
from numpy.typing import NDArray
from rclpy.duration import Duration
from sensor_msgs.msg import Joy

from motion_stack.api.ros2.ik_api import IkHandler, IkSyncerRos
from motion_stack.api.ros2.joint_api import JointHandler, JointSyncerRos
from motion_stack.core.utils.math import patch_numpy_display_light
from motion_stack.core.utils.pose import Pose
from motion_stack.ros2.utils.conversion import ros_now
from motion_stack.ros2.utils.executor import error_catcher

patch_numpy_display_light()

ALIAS = "operator_node"
MAX_JOINT_SPEED = 0.25
TRANSLATION_SPEED = 100  # mm/s ; full stick will send this speed
ROTATION_SPEED = np.deg2rad(5)  # rad/s ; full stick will send this angular speed

# type def V
ANY: Final[str] = "ANY"
ALWAYS: Final[str] = "ALWAYS"
KeyCodeModifier = Tuple[
    int, Union[int, Literal["ANY"]]
]  # keyboard input: key + modifier
JoyBits = int  # 32 bits to represent all buttons pressed or not
ButtonName = Literal[  # type with all possible buttons
    "NONE",
    "x",
    "o",
    "t",
    "s",
    "L1",
    "R1",
    "L2",
    "R2",
    "share",
    "option",
    "PS",
    "stickLpush",
    "stickRpush",
    "down",
    "right",
    "up",
    "left",
    "stickL",
    "stickR",
]
JoyCodeModifier = Tuple[
    ButtonName, Union[JoyBits, Literal["ANY"]]
]  # joystick input: new press + JoyState
UserInput = Union[  # type of keys to the dict that will give functions to execute
    KeyCodeModifier,
    JoyCodeModifier,
    Literal["ALWAYS"],  # functions associated with "ALWAYS" string will always execute
]
NakedCall = Callable[[], Any]
InputMap = Dict[
    UserInput, List[NakedCall]
]  # User input are linked to a list of function

# Namespace
operator = str(environ.get("OPERATOR"))
INPUT_NAMESPACE = f"/{operator}"

# Keys
NOMOD = Key.MODIFIER_NUM

BUTT_BITS: Dict[ButtonName, int] = {  # button name to bit position
    # butts
    "x": 0,
    "o": 1,
    "t": 2,
    "s": 3,
    # backs
    "L1": 4,
    "R1": 5,
    "L2": 6,
    "R2": 7,
    # options
    "share": 8,
    "option": 9,
    "PS": 10,
    # stick pushed
    "stickLpush": 11,
    "stickRpush": 12,
    # dpad
    "down": 13,
    "right": 14,
    "up": 15,
    "left": 16,
    # sticks
    "stickL": 17,  # left stick not centered
    "stickR": 18,  # right stick not centered
}
BITS_BUTT: Dict[int, ButtonName] = {v: k for k, v in BUTT_BITS.items()}
# Bitmask of each button
BUTT_INTS: Dict[ButtonName, JoyBits] = {
    butt: 1 << bit for butt, bit in BUTT_BITS.items()
}
BUTT_INTS["NONE"] = 0
INTS_BUTT: Dict[JoyBits, ButtonName] = {v: k for k, v in BUTT_INTS.items()}


@dataclass
class JoyState:
    bits: JoyBits = 0
    stickR: NDArray = dataclasses.field(
        default_factory=lambda: np.zeros(2, dtype=float)
    )
    stickL: NDArray = dataclasses.field(
        default_factory=lambda: np.zeros(2, dtype=float)
    )
    R2: float = 0.0
    L2: float = 0.0


class OperatorNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__(ALIAS)

        # keep track of discovered legs
        self.current_legs: Set[int] = set()
        self.joint_handlers: List[JointHandler] = []
        self.joint_syncer: Optional[JointSyncerRos] = None
        self.ik_handlers: List[IkHandler] = []
        self.ik_syncer: Optional[IkSyncerRos] = None
        self.wheel_handlers: List[JointHandler] = []
        self.wheel_syncer: Optional[JointSyncerRos] = None

        # periodically discover legs
        self.create_timer(1.0, self._discover_legs)
        # main control loop
        self.create_timer(1 / 30.0, self.loop)

        # keyboard subscriptions
        self.key_downSUB = self.create_subscription(
            Key, f"{INPUT_NAMESPACE}/keydown", self.key_downSUBCBK, 10
        )
        self.key_upSUB = self.create_subscription(
            Key, f"{INPUT_NAMESPACE}/keyup", self.key_upSUBCBK, 10
        )
        self.joySUB = self.create_subscription(
            Joy, f"{INPUT_NAMESPACE}/joy", self.joySUBCBK, 10
        )

        self.main_map: Final[InputMap] = self.create_main_map()
        self.sub_map: InputMap
        self.enter_main_menu()

        self.selected_legs: List[int] = []
        self.selected_joints: Set[Tuple[int, str]] = set()
        self.selected_joints_inv: Set[Tuple[int, str]] = set()
        self.selected_wheel_joints: Set[Tuple[int, str]] = set()
        self.selected_wheel_joints_inv: Set[Tuple[int, str]] = set()
        self.current_speed = 0.25

        self.ik2TMR = self.create_timer(0.1, self.move_ik)
        self.ik2TMR.cancel()

        self.log_messages: deque[str] = deque(maxlen=3)

        # JoyStick
        self.prev_axes = None
        self.prev_buttons = None
        self.deadzone = 0.05
        self.neutral_threshold = 0.2
        self.default_neutral_axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        self.current_movement = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "x_prev": 0.0,
            "y_prev": 0.0,
            "z_prev": 0.0,
            "roll_prev": 0.0,
            "pitch_prev": 0.0,
            "yaw_prev": 0.0,
        }
        self.prev_joy_state: JoyState = JoyState()
        self.joy_state: JoyState = JoyState()

    @error_catcher
    def _discover_legs(self):
        """Discover /legN/joint_alive services and add new handlers."""
        svc_list = self.get_service_names_and_types()
        found: Set[int] = set()
        for name, types in svc_list:
            if name.endswith("/joint_alive") and "std_srvs/srv/Empty" in types:
                m = re.match(r"^/leg(\d+)/joint_alive$", name)
                if m:
                    found.add(int(m.group(1)))

        new_legs = found - self.current_legs
        self.current_legs = found

        if new_legs:
            self.add_log("I", f"New legs discovered: {sorted(new_legs)}")
            self.joint_handlers = [
                JointHandler(self, l) for l in sorted(self.current_legs)
            ]
            self.ik_handlers = [IkHandler(self, l) for l in sorted(self.current_legs)]
            # rebuild syncers
            self.joint_syncer = JointSyncerRos(self.joint_handlers)
            self.ik_syncer = IkSyncerRos(self.ik_handlers)
            self.wheel_syncer = JointSyncerRos(
                self.joint_handlers, interpolation_delta=np.deg2rad(30)
            )

    def select_leg(self, leg_inds: Optional[List[int]]):
        """Pick which legs you want to control. None => all."""
        if not self.current_legs:
            self.add_log("W", "Cannot select: no legs yet")
            return

        if leg_inds is None:
            # select every discovered leg
            self.selected_legs = sorted(self.current_legs)
        else:
            # only keep those that actually exist
            self.selected_legs = [l for l in leg_inds if l in self.current_legs]
            missing = set(leg_inds) - set(self.selected_legs)
            for bad in sorted(missing):
                self.add_log("W", f"Leg {bad} does not exist")

        self.add_log("I", f"Selected leg(s): {self.selected_legs}")

    def no_no_leg(self):
        """Makes sure no legs are not selected"""
        if self.selected_legs is None:
            self.select_leg(None)
        if not self.selected_legs:
            self.select_leg(None)

    def enter_main_menu(self):
        self.current_mode = "main"

        self.sub_map = {
            (Key.KEY_L, ANY): [self.enter_leg_mode],
            (Key.KEY_J, ANY): [self.enter_joint_mode],
            (Key.KEY_W, ANY): [self.enter_wheel_mode],
        }

    def enter_leg_mode(self):
        self.current_mode = "leg_select"

        submap: InputMap = {
            (Key.KEY_DOWN, ANY): [lambda: self.select_leg(None)],
            (Key.KEY_L, ANY): [lambda: self.select_leg(None)],
        }
        one2nine_keys = [
            (1, Key.KEY_1),
            (2, Key.KEY_2),
            (3, Key.KEY_3),
            (4, Key.KEY_4),
            (5, Key.KEY_5),
            (6, Key.KEY_6),
            (7, Key.KEY_7),
            (8, Key.KEY_8),
            (9, Key.KEY_9),
        ]
        for n, keyb in one2nine_keys:
            submap[(keyb, ANY)] = [lambda n=n: self.select_leg([n])]

        self.sub_map = submap

    def enter_joint_mode(self):
        self.current_mode = "joint_select"
        self.no_no_leg()

        submap: InputMap = {
            (Key.KEY_W, ANY): [lambda: self.move_joints(MAX_JOINT_SPEED)],
            (Key.KEY_S, ANY): [lambda: self.move_joints(-MAX_JOINT_SPEED)],
            (Key.KEY_O, ANY): [lambda: self.move_wheels(self.current_speed)],
            (Key.KEY_L, ANY): [lambda: self.move_wheels(-self.current_speed)],
            (Key.KEY_P, ANY): [lambda: self.move_wheels(0.0)],
            (Key.KEY_0, ANY): [self.move_zero],
        }

        self.sub_map = submap

    def enter_wheel_mode(self):
        self.current_mode = "wheel_select"
        self.no_no_leg()

        self.sub_map = {
            (Key.KEY_W, ANY): [lambda: self.move_joints(MAX_JOINT_SPEED)],
            (Key.KEY_S, ANY): [lambda: self.move_joints(-MAX_JOINT_SPEED)],
            (Key.KEY_O, ANY): [lambda: self.move_wheels(self.current_speed)],
            (Key.KEY_L, ANY): [lambda: self.move_wheels(-self.current_speed)],
            (Key.KEY_P, ANY): [lambda: self.move_wheels(0.0)],
        }

    def enter_ik_mode(self):
        self.current_mode = "ik_select"

        self.sub_map = {
            ("stickL", ANY): [self.start_ik2_timer],
            ("stickR", ANY): [self.start_ik2_timer],
            ("R2", ANY): [self.start_ik2_timer],
            ("L2", ANY): [self.start_ik2_timer],
            ("R1", ANY): [self.start_ik2_timer],
            ("L1", ANY): [self.start_ik2_timer],
            ("L1", ANY): [self.start_ik2_timer],
        }

    def move_ik(self):
        if self.ik_syncer is None:
            return

        bits = self.joy_state.bits
        sticks_active = self.any_pressed(
            bits,
            [
                "stickR",
                "stickL",
                "R2",
                "L2",
                "R1",
                "L1",
            ],
        )

        if not sticks_active:
            self.ik_syncer.last_future.cancel()
            self.ik2TMR.cancel()
            return

        speed_xyz = TRANSLATION_SPEED  # mm/s
        speed_quat = ROTATION_SPEED  # rad/s
        delta_xyz = speed_xyz * self.ik2TMR.timer_period_ns / 1e9
        delta_quat = speed_quat * self.ik2TMR.timer_period_ns / 1e9

        xyz_input = np.empty((3,), dtype=float)

        xyz_input[[0, 1]] = self.joy_state.stickL  # left stick to move
        xyz_input[2] = -self.joy_state.R2 + self.joy_state.L2  # deep triggers to move Z
        r1 = (bits & BUTT_INTS["R1"]) != 0
        l1 = (bits & BUTT_INTS["L1"]) != 0

        x_rot = qt.from_rotation_vector([-l1 + r1, 0, 0])
        y_rot = qt.from_rotation_vector([0, -self.joy_state.stickR[0], 0])
        z_rot = qt.from_rotation_vector([0, 0, self.joy_state.stickR[1]])
        rot = (z_rot * y_rot * x_rot) ** delta_quat

        target_pose_rel = Pose(ros_now(self), xyz_input * delta_xyz, rot)
        target_pose_abs = self.ik_syncer.abs_from_rel(
            {leg: target_pose_rel for leg in self.selected_legs}
        )

        self.ik_syncer.lerp(target_pose_abs)

    def move_zero(self):
        """
        Sends all selected joints to zero.
        """
        if self.joint_syncer is None:
            return

        selected_jnames = sorted(jn for (_, jn) in self.selected_joints)
        selected_jnames_inv = sorted(jn for (_, jn) in self.selected_joints_inv)

        if not selected_jnames and not selected_jnames_inv:
            return

        self.add_log("I", "Sending the joints to zero position.")
        target = {}
        target.update({jname: 0.0 for jname in selected_jnames + selected_jnames_inv})

        zero_future = self.joint_syncer.lerp(target)

        return zero_future

    def start_ik2_timer(self):
        """properly checks and start the timer loop for ik of lvl2"""
        if self.ik_syncer is None:
            return
        if self.ik2TMR.is_canceled():
            elapsed = Duration(nanoseconds=self.ik2TMR.time_since_last_call())
            if elapsed > Duration(seconds=5):
                self.ik_syncer.clear()
            self.ik2TMR.reset()
            self.ik2TMR.callback()

    def recover_all(self):
        self.add_log("W", "REPLACE THIS FUNCTION WITH YOUR RECOVER STRATEGY.")

    def recover(self):
        self.add_log("W", "REPLACE THIS FUNCTION WITH YOUR RECOVER STRATEGY.")
        for leg in self.selected_legs:
            return

    def halt_all(self):
        self.add_log("W", "REPLACE THIS FUNCTION WITH YOUR HALTING STRATEGY.")

    def halt(self):
        self.add_log("W", "REPLACE THIS FUNCTION WITH YOUR HALTING STRATEGY.")
        for leg in self.selected_legs:
            return

    def move_joints(self, speed: float):
        if not self.joint_syncer:
            return

        selected_jnames = sorted(jn for (_, jn) in self.selected_joints)
        selected_jnames_inv = sorted(jn for (_, jn) in self.selected_joints_inv)
        if not selected_jnames and not selected_jnames_inv:
            return

        start_time = ros_now(self)

        def delta_time():
            nonlocal start_time
            now = ros_now(self)
            out = (now - start_time).sec()
            start_time = now
            return out

        target = {jn: speed for jn in selected_jnames}
        target.update({jn: -speed for jn in selected_jnames_inv})
        # self.add_log("I", f"{target}")

        self.joint_syncer.speed_safe(target, delta_time)

    def move_wheels(self, v: float, omega: float = 0.0):
        """
        Args:
            v (float): Linear speed command. Positive drives forward, negative back.
            omega (float): Angular speed command. Positive turns robot to the left,
                           negative to the right.
        """
        if not self.wheel_syncer:
            return

        if v == 0.0 and omega == 0.0:
            self.wheel_syncer.last_future.cancel()
            return

        wheel_jnames = sorted(jn for (_, jn) in self.selected_wheel_joints)
        wheel_jnames_inv = sorted(jn for (_, jn) in self.selected_wheel_joints_inv)
        if not wheel_jnames:
            return

        target = {jname: v for jname in wheel_jnames}
        target.update({jn: -v for jn in wheel_jnames_inv})

        # self.add_log("I", f"{target}")

        start_time = ros_now(self)

        def delta_time():
            nonlocal start_time
            now = ros_now(self)
            out = (now - start_time).sec()
            start_time = now
            return out

        self.wheel_syncer.speed_safe(target, delta_time)

    def create_main_map(self) -> InputMap:
        return {
            (Key.KEY_RETURN, ANY): [self.recover],
            (Key.KEY_RETURN, Key.MODIFIER_LSHIFT): [self.recover_all],
            (Key.KEY_SPACE, ANY): [self.halt],
            (Key.KEY_SPACE, Key.MODIFIER_LSHIFT): [self.halt_all],
            (Key.KEY_ESCAPE, ANY): [self.enter_main_menu],
        }

    @error_catcher
    def loop(self):
        """Executed at ~30Hz."""
        if self.joint_syncer:
            self.joint_syncer.execute()
        if self.ik_syncer:
            self.ik_syncer.execute()
        if self.wheel_syncer:
            self.wheel_syncer.execute()

    @error_catcher
    def key_downSUBCBK(self, msg: Key):
        # self.add_log("W", "ugh")
        code = msg.code
        mod = msg.modifiers & ~(Key.MODIFIER_NUM | Key.MODIFIER_CAPS)
        self.connect_mapping(self.main_map, (code, mod))
        self.connect_mapping(self.sub_map, (code, mod))

    @error_catcher
    def key_upSUBCBK(self, msg: Key):
        self.stop_all_joints()

    def stop_all_joints(self):
        if self.joint_syncer:
            self.joint_syncer.last_future.cancel()
        if self.ik_syncer:
            self.ik_syncer.last_future.cancel()

    def add_log(self, level: str, msg: str):
        """Stash a timestamped log entry for the UI."""
        ts = time.strftime("%H:%M:%S")
        self.log_messages.append(f"{ts} [{level}] {msg}")

    @staticmethod
    def collapseT_KeyCodeModifier(variable: Any) -> Optional[KeyCodeModifier]:
        """Collapses the variable onto a KeyCodeModifier type, or None

        Returns:
            None if variable is not a KCM
            The variable as a KCM type-hint if it is a KCM

        """
        if not isinstance(variable, tuple):
            return None
        if len(variable) != 2:
            return None
        if not isinstance(variable[0], int):
            return None
        if isinstance(variable[1], int):
            return variable
        if variable[1] == ANY:
            return variable
        return None

    @staticmethod
    def collapseT_JoyCodeModifier(variable: Any) -> Optional[JoyCodeModifier]:
        """Collapses the variable onto a JoyCodeModifier type, or None

        Returns:
            None if variable is not a JCM
            The variable as a JCM type-hint if it is a JCM

        """
        if not isinstance(variable, tuple):
            return None
        if len(variable) != 2:
            return None
        if not isinstance(variable[0], str):
            return None
        if isinstance(variable[1], JoyBits):
            return variable
        if variable[0] == ANY:
            return variable
        return None

    @staticmethod
    def remap_onto_any(mapping: InputMap, input: UserInput):
        """runs the input through the INPUTMap as if the key_modifier was any
        if it is already, it does not run it.
        """
        collapsed_KCM = OperatorNode.collapseT_KeyCodeModifier(input)
        if collapsed_KCM is not None:  # is KCM
            if not collapsed_KCM[1] == ANY:
                OperatorNode.connect_mapping(mapping, (collapsed_KCM[0], ANY))

        collapsed_JCM = OperatorNode.collapseT_JoyCodeModifier(input)
        if collapsed_JCM is not None:  # is JCM
            if not collapsed_JCM[1] == ANY:
                OperatorNode.connect_mapping(mapping, (collapsed_JCM[0], ANY))

    @staticmethod
    def connect_mapping(mapping: InputMap, input: UserInput):
        """Given the user input, executes the corresponding function mapping

        Args:
            mapping: Dict of function to execute
            input: key to the entry to execute
        """
        OperatorNode.remap_onto_any(mapping, input)
        if input not in mapping.keys():
            return
        to_execute: List[NakedCall] = mapping[input]
        for f in to_execute:
            f()
        return

    # ───────────────────────────── JoyStick ─────────────────────────────

    def msg_to_JoyBits(self, msg: Joy) -> JoyState:
        """Converts a joy msg to a JoyState"""
        state = JoyState()
        but: List[int] = msg.buttons  # type:ignore
        axes: List[float] = msg.axes  # type:ignore
        sticks_raw: List[float] = [axes[x] for x in [0, 1, 3, 4]]  # type:ignore
        # triggers are already included in the but list
        triggers: List[float] = [axes[x] for x in [2, 5]]  # type:ignore
        dpad_raw: List[float] = axes[-2:]  # type:ignore
        bfield = int(0)
        # buttons
        i = 0
        for i, b in enumerate(but):
            assert b == 1 or b == 0
            bfield = bfield | (b << i)
        # dpad
        next_bit_to_set = i + 1
        dpad = [
            dpad_raw[1] < -0.5,
            dpad_raw[0] < -0.5,
            dpad_raw[1] > 0.5,
            dpad_raw[0] > 0.5,
        ]
        for i, ax in enumerate(dpad):
            i += next_bit_to_set
            ax_active = not np.isclose(ax, 0)
            bfield = bfield | (ax_active << i)
        # sticks
        next_bit_to_set = i + 1
        for i, ax in enumerate(sticks_raw):
            if ax == -0.0:
                sticks_raw[i] = 0.0

        state.stickL = np.array(sticks_raw[:2], dtype=float)
        state.stickL[[0, 1]] = state.stickL[[1, 0]]  # changes xy
        ax_active = not np.isclose(np.linalg.norm(state.stickL), 0, atol=0.2)
        bfield = bfield | (ax_active << BUTT_BITS["stickL"])  # using * is bad

        state.stickR = np.array(sticks_raw[2:], dtype=float)
        state.stickR[[0, 1]] = state.stickR[[1, 0]]  # changes xy
        ax_active = not np.isclose(np.linalg.norm(state.stickR), 0, atol=0.2)
        bfield = bfield | (ax_active << BUTT_BITS["stickR"])  # using * is bad

        state.L2 = (1 - triggers[0]) / 2
        state.R2 = (1 - triggers[1]) / 2

        state.bits = bfield

        return state

    def any_pressed(
        self, bits: JoyBits, button_names: Union[List[ButtonName], ButtonName]
    ) -> bool:
        """Checks if any button in the list is pressed.

        Args:
            bits: set of joybits to check against
            button_names: list of button names to check if True

        Returns:
            True if any bit corresponding to a button is True.
        """
        if isinstance(button_names, str):
            button_names = [button_names]
        for n in button_names:
            assert n in BUTT_INTS.keys(), f"Button {n} does not exist"

        arr_of_joybits = np.array([BUTT_INTS[n] for n in button_names])
        fused_joybits = np.bitwise_or.reduce(arr_of_joybits)
        return (bits & fused_joybits) != 0

    def bits2name(self, bits: JoyBits) -> List[ButtonName]:
        """Converts a bit field to a list of button names"""
        button_names: List[ButtonName] = []
        while bits:  # handles several button pressed at the same time
            # to handle rare edge cases
            isolated_bit = bits & -bits
            name = self.one_bit2name(isolated_bit)
            if name is not None:
                button_names.append(name)
            bits &= bits - 1
        return button_names

    def one_bit2name(self, bits: JoyBits) -> Optional[ButtonName]:
        """Converts a bit field with 1 bit to 1, to a single button name"""
        button_name: Optional[ButtonName] = INTS_BUTT.get(bits)
        if button_name is None:
            self.add_log("W", f"Unknown bit: {(bits & -bits).bit_length() - 1}")
            self.add_log("W", f"{bits:018b}")
        button_name = None if button_name == "NONE" else button_name
        return button_name

    def joy_pressed(self, button_name: ButtonName):
        """Executes for each button that is pressed. Like a callback.

        Args:
            bits: Should only have one single bit set to 1, for 1 single button
        """
        dic_key = (button_name, self.joy_state.bits)
        self.connect_mapping(self.main_map, dic_key)
        self.connect_mapping(self.sub_map, dic_key)

    def joy_released(self, button_name: ButtonName):
        """Executes for each button that is released. Like a callback.

        Args:
            bits: Should only have one single bit set to 1, for 1 single button
        """
        dic_key = (button_name, self.joy_state.bits)
        self.stop_all_joints()

    @error_catcher
    def joySUBCBK(self, msg: Joy):
        """Processes incomming joy messages.
        Converts and stores the received state in self.joy_state .
        executes self.joy_pressed and self.joy_released for each button that changed state

        Args:
            msg: Ros2 Joy message type
        """
        # self.display_JoyBits(0)
        self.prev_joy_state = self.joy_state
        self.joy_state = self.msg_to_JoyBits(msg)

        button_downed: JoyBits = ~self.prev_joy_state.bits & self.joy_state.bits
        downed_names: List[ButtonName] = self.bits2name(button_downed)
        for name in downed_names:
            self.joy_pressed(name)

        button_upped = self.prev_joy_state.bits & ~self.joy_state.bits
        upped_names: List[ButtonName] = self.bits2name(button_upped)
        for name in upped_names:
            self.joy_released(name)
        return


def main():
    rclpy.init()
    node = OperatorNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    from .operator_tui import urwid_main

    urwid_main(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

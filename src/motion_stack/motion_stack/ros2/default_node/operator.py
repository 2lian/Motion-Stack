import dataclasses
from dataclasses import dataclass
from os import environ
from typing import Any, Callable, Dict, Final, Literal, Optional, Tuple, overload

import numpy as np
from keyboard_msgs.msg import Key
from motion_stack_msgs.msg import TargetBody
from motion_stack_msgs.srv import SendTargetBody
from numpy.typing import NDArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.client import Client
from rclpy.node import List, Node, Timer, Union
from rclpy.task import Future
from rclpy.time import Duration
from sensor_msgs.msg import Joy  # joystick, new
from std_msgs.msg import Float64
from std_srvs.srv import Empty, Trigger

import motion_stack.ros2.ros2_asyncio.ros2_asyncio as rao
from motion_stack.api.joint_syncer import SensorSyncWarning
from motion_stack.api.ros2.ik_api import IkHandler, IkSyncerRos
from motion_stack.api.ros2.joint_api import JointHandler, JointSyncerRos
from motion_stack.core.utils.joint_state import JState
from motion_stack.core.utils.math import patch_numpy_display_light
from motion_stack.core.utils.pose import Pose, XyzQuat
from motion_stack.ros2.utils.executor import error_catcher, my_main

patch_numpy_display_light()

ALIAS = "operator_node"

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


class OperatorNode(Node):
    def __init__(self) -> None:
        self.LEG_NUM = 12
        LEG_LIST = [self.LEG_NUM]

        super().__init__(ALIAS)
        self.joint_handlers = [JointHandler(self, l) for l in LEG_LIST]
        self.joint_syncer = JointSyncerRos(self.joint_handlers)
        # self.ik_handlers = [IkHandler(self, l) for l in LEG_LIST]
        # self.ik_syncer = IkSyncerRos(self.ik_handlers)
        self.create_timer(1 / 30, self.loop)
        self.startTMR = self.create_timer(0.1, self.startup)

        self.key_downSUB = self.create_subscription(
            Key, f"{INPUT_NAMESPACE}/keydown", self.key_downSUBCBK, 10
        )
        self.key_upSUB = self.create_subscription(
            Key, f"{INPUT_NAMESPACE}/keyup", self.key_upSUBCBK, 10
        )

        self.main_map: Final[InputMap] = (
            self.create_main_map()
        )  # always executed, must not change to always be available
        self.sub_map: InputMap  # will change

    async def joints_ready(self):
        l = [jh.ready for jh in self.joint_handlers]
        try:
            print("Waiting for joints.")
            await rao.wait_for(self, rao.gather(self, *l), timeout_sec=100)
            print(f"Joints ready.")
            strlist = "\n".join(
                [f"limb {jh.limb_number}: {jh.tracked}" for jh in self.joint_handlers]
            )
            # print(f"Joints are:\n{strlist}")
            return
        except TimeoutError:
            raise TimeoutError("Joint data unavailable after 100 sec")

    # async def ik_ready(self):
    #     l = [ih.ready for ih in self.ik_handlers]
    #     try:
    #         print("Waiting for ik.")
    #         await rao.wait_for(self, rao.gather(self, *l), timeout_sec=100)
    #         print(f"Ik ready.")
    #         strlist = "\n".join(
    #             [f"limb {ih.limb_number}: {ih.ee_pose}" for ih in self.ik_handlers]
    #         )
    #         # print(f"EE poses are:\n{strlist}")
    #         return
    #     except TimeoutError:
    #         raise TimeoutError("Ik data unavailable after 100 sec")

    @error_catcher
    async def main(self):
        await self.joints_ready()
        # await self.ik_ready()

    @error_catcher
    def startup(self):
        rao.ensure_future(self, self.main())
        self.destroy_timer(self.startTMR)
        print("Startup done.")

    @error_catcher
    def loop(self):
        self.joint_syncer.execute()
        # self.ik_syncer.execute()

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
                # we run the connection (again?), replacing the key_modifier with ANY
                OperatorNode.connect_mapping(mapping, (collapsed_KCM[0], ANY))

        collapsed_JCM = OperatorNode.collapseT_JoyCodeModifier(input)
        if collapsed_JCM is not None:  # is JCM
            if not collapsed_JCM[1] == ANY:
                # we run the connection (again?), replacing the key_modifier with ANY
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

    # ------------- KEYBOARD -------------
    @error_catcher
    def key_upSUBCBK(self, msg: Key):
        """Executes when keyboard released"""
        key_char = chr(msg.code)
        key_code = msg.code
        # self.pinfo(f"chr: {chr(msg.code)}, int: {msg.code}")
        self.stop_all_joints()

    @error_catcher
    def key_downSUBCBK(self, msg: Key):
        """Executes when keyboard pressed"""
        key_code = msg.code
        key_modifier = msg.modifiers
        # bitwise operation to set numlock and capslock bit to 0
        key_modifier = key_modifier & ~(Key.MODIFIER_NUM | Key.MODIFIER_CAPS)
        # self.pinfo(f"chr: {chr(msg.code)}, mod: {key_modifier:016b}")
        self.connect_mapping(self.main_map, (key_code, key_modifier))
        self.connect_mapping(self.sub_map, (key_code, key_modifier))
        return

    # logging
    def perror(self, object):
        self.get_logger().error(f"[{ALIAS}] {object}")

    def pwarn(self, object):
        self.get_logger().warn(f"[{ALIAS}] {object}")

    def pinfo(self, object):
        self.get_logger().info(f"[{ALIAS}] {object}")

    def stop_all_joints(self):
        """
        Cancels the joint and IK syncers' futures.
        """
        # self.last_future.cancel()
        # self.ik_syncer.last_future.cancel()
        self.joint_syncer.last_future.cancel()

    def create_main_map(self) -> InputMap:
        """Creates the main input map, mapping user input to functions,
        This is supposed to be constant + always active, unlike the sub_map"""
        main_map: InputMap = {
            # (Key.KEY_RETURN, ANY): [self.recover],
            # (Key.KEY_H, ANY): [self.halt, self.stop_all_joints],
            # (Key.KEY_H, Key.MODIFIER_LSHIFT): [self.halt_all, self.stop_all_joints],
            # (Key.KEY_RETURN, Key.MODIFIER_LSHIFT): [self.recover_all],
            # (Key.KEY_ESCAPE, ANY): [self.enter_waiting_mode],
        }
        return main_map


def main(*args):
    my_main(OperatorNode)

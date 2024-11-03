"""
This node is responsible for controlling movement of Moonbot HERO.
For now keyboard and controller (PS4).

Authors: Elian NEPPEL, Shamistan KARIMOV
Lab: SRL, Moonshot team
"""

from typing import (
    Any,
    Callable,
    Dict,
    Final,
    Literal,
    Optional,
    Sequence,
    Tuple,
    TypeVar,
    overload,
)
import re
import numpy as np
from os import environ
from numpy.typing import NDArray
import quaternion as qt
import rclpy
from rclpy.clock import Time
from rclpy.task import Future
from rclpy.node import Union, List
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.publisher import Duration, Publisher
from EliaNode import (
    Client,
    EliaNode,
    error_catcher,
    np2TargetSet,
    np2tf,
    myMain,
    targetSet2np,
)

from std_msgs.msg import Float64
from geometry_msgs.msg import Transform

from custom_messages.srv import (
    ReturnTargetBody,
    ReturnVect3,
    Vect3,
    TFService,
    ReturnTargetSet,
    SendTargetSet,
    SendTargetBody,
)
from custom_messages.msg import TargetBody, TargetSet
from keyboard_msgs.msg import Key
from sensor_msgs.msg import Joy  # joystick, new
from std_srvs.srv import Empty, Trigger

from easy_robot_control.gait_node import Leg as PureLeg
import ctypes
from dataclasses import dataclass

float_formatter = "{:.2f}".format
np.set_printoptions(formatter={"float_kind": float_formatter})

# type def V
#
ANY: Final[str] = "ANY"
ALWAYS: Final[str] = "ALWAYS"
KeyCodeModifier = Tuple[int, Union[int, Literal["ANY"]]]  # keyboard input: key + modifier
JoyBits = int  # 32 bits to represent all buttons pressed or not
ButtonName = Literal[
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
# jb = int(2**32)
UserInput = Union[
    KeyCodeModifier,
    JoyCodeModifier,
    Literal["ALWAYS"],  # functions associated with "ALWAYS" string will always execute
]  # add you input type here for joystick,
# MUST be an imutable object (or you'll hurt yourself)
NakedCall = Callable[[], Any]
InputMap = Dict[UserInput, List[NakedCall]]  # User input are linked to a list of function
#
# type def ^

# Define scaling constants globally
ACTIVE_AXIS = None
TRANSLATION_SCALE = 20  # translational IK
ROTATION_SCALE = np.deg2rad(1.5)  # rotational IK

operator = str(environ.get("OPERATOR"))
# operator = "elian"
INPUT_NAMESPACE = f"/{operator}"

LEGNUMS_TO_SCAN = [1, 2, 3, 4]

NOMOD = Key.MODIFIER_NUM
MAX_JOINT_SPEED = 0.15
STICKER_TO_ALPHAB: Dict[int, int] = {
    1: 1,
    2: 0,
    3: 3,
    4: 4,
    5: 5,
    6: 6,
    7: 7,
    8: 8,
    9: 2,
}
ALPHAB_TO_STICKER = {v: k for k, v in STICKER_TO_ALPHAB.items()}

DRAGON_MAIN: int = 4
DRAGON_MANIP: int = 2

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
BUTT_INTS: Dict[ButtonName, JoyBits] = {butt: 1 << bit for butt, bit in BUTT_BITS.items()}
BUTT_INTS["NONE"] = 0
INTS_BUTT: Dict[JoyBits, ButtonName] = {v: k for k, v in BUTT_INTS.items()}


@dataclass
class JoyState:
    bits: JoyBits = 0
    stickR: NDArray = np.zeros(2, dtype=float)
    stickL: NDArray = np.zeros(2, dtype=float)
    R2: float = 0.0
    L2: float = 0.0


T = TypeVar("T", NDArray, qt.quaternion)


def clamp2hypersphere(center: T, radius: float, start: T, end: T) -> T:
    """Given start and endpoint of a segment in N dimensions. And a hypersphere.
    Returns the furthest point on the segment that is inside the hypersphere

    Also the interpolation math is wrong for the quaternion, but if small angles it's fine
    quats should be interpolated using slerp not lerp

    Args:
        center: Center of the hypersphere
        radius: radius of the hypersphere
        start: start of the segment
        end: end of the segment

    Returns:
        The furthest point on the segment that is inside the hypersphere
    """
    if isinstance(start, qt.quaternion):
        # recursive recomputes as numpy array
        center_n = qt.as_float_array(center)
        start_n = qt.as_float_array(start)
        end_n = qt.as_float_array(end)
        result = clamp2hypersphere(center_n, radius, start_n, end_n)
        if np.all(result == start_n):
            return start
        if np.all(result == end_n):
            return end
        result = result / np.linalg.norm(result)
        return qt.from_float_array(result)
    assert len(center.shape) == 1
    assert center.shape == start.shape == end.shape
    dims: int = center.shape[0]
    trials = 100
    t = np.linspace(0, 1, trials, endpoint=True).reshape(-1, 1)
    interp = end * t + start * (1 - t)
    dist = interp - center
    inside_hyper = np.linalg.norm(dist, axis=1) < radius
    assert inside_hyper.shape[0] == trials
    selection = interp[inside_hyper]
    if selection.shape[0] == 0:
        return start
    elif selection.shape[0] == trials:
        return end
    furthest = selection[-1, :]
    assert furthest.shape[0] == dims
    return furthest


class Leg(PureLeg):  # overloads the general Leg class with stuff only for Moonbot Hero
    def __init__(self, number: int, parent: EliaNode) -> None:
        super().__init__(number, parent)
        self.recoverCLI = self.parent.create_client(
            Trigger, f"/leg{self.number}/driver/recover"
        )
        self.haltCLI = self.parent.create_client(
            Trigger, f"/leg{self.number}/driver/halt"
        )
        self.last_xyz: Optional[NDArray] = None
        self.last_quat: Optional[qt.quaternion] = None
        self.last_time: Optional[Time] = None
        # ik2 offset movement is considered too fast if outside the sphere centered
        # on the current pose
        self.sphere_xyz_radius: float = 25 #mm
        self.sphere_quat_radius: float = np.rad2deg(2) #rad

    def recover(self) -> Future:
        return self.recoverCLI.call_async(Trigger.Request())

    def halt(self) -> Future:
        return self.haltCLI.call_async(Trigger.Request())

    def reset_ik2_offset(self):
        self.parent.pinfo("ik2 reset")
        self.last_xyz = None
        self.last_quat = None
        self.last_time = None

    def apply_ik2_offset(self, xyz: Optional[NDArray], quat: Optional[qt.quaternion]):
        if self.xyz_now is None or self.quat_now is None:
            self.parent.pwarn(f"[leg#{self.number}] tip_pos UNKNOWN, ik2 ignored")
            return
        if xyz is None:
            xyz = np.zeros(3, dtype=float)
        if quat is None:
            quat = qt.one
        if self.last_xyz is None or self.last_quat is None or self.last_time is None:
            self.last_xyz = self.xyz_now
            self.last_quat = self.quat_now
            self.last_time = self.parent.getNow()
        if self.last_xyz is None or self.last_quat is None or self.last_time is None:
            self.parent.pwarn(f"[leg#{self.number}] tip_pos UNKNOWN, ik2 ignored")
            return

        next_xyz = self.last_xyz + xyz
        next_quat = self.last_quat * quat

        # easy stuff, we cut the 7D segment when it goes out of the hypersphere of center
        # the current pose.
        # and we normalize the 7D segment differently to make a dimensionless space
        # because mm and radians cannot be compared. Hence the hypersphere of size 1.
        radius_for_xyz = self.sphere_xyz_radius
        radius_for_quat = self.sphere_quat_radius

        fused_center = np.empty(3 + 4, dtype=float)
        fused_center[[0, 1, 2]] = self.xyz_now / radius_for_xyz
        fused_center[[3, 4, 5, 6]] = qt.as_float_array(self.quat_now) / radius_for_quat

        fused_start = np.empty(3 + 4, dtype=float)
        fused_start[[0, 1, 2]] = self.last_xyz / radius_for_xyz
        fused_start[[3, 4, 5, 6]] = qt.as_float_array(self.last_quat) / radius_for_quat

        fused_end = np.empty(3 + 4, dtype=float)
        fused_end[[0, 1, 2]] = next_xyz / radius_for_xyz
        fused_end[[3, 4, 5, 6]] = qt.as_float_array(next_quat) / radius_for_quat

        clamp_fused = clamp2hypersphere(fused_center, 1, fused_start, fused_end)
        if np.any(clamp_fused != fused_end):
            self.parent.pwarn("too fast")

        clamp_xyz = clamp_fused[[0, 1, 2]] * radius_for_xyz
        clamp_quat = qt.from_float_array(clamp_fused[[3, 4, 5, 6]] * radius_for_quat)
        # quat needs normalization but who cares

        next_xyz = clamp_xyz
        next_quat = clamp_quat

        self.ik(
            xyz=next_xyz,
            quat=next_quat,
        )
        self.last_xyz = next_xyz
        self.last_quat = next_quat
        self.last_time = self.parent.getNow()


class KeyGaitNode(EliaNode):
    def __init__(self, name: str = "keygait_node"):
        super().__init__(name)
        self.Alias = "K"

        self.leg_aliveCLI: Dict[int, Client] = dict(
            [(l, self.create_client(Empty, f"leg{l}/leg_alive")) for l in LEGNUMS_TO_SCAN]
        )
        self.legs: Dict[int, Leg] = {}

        self.key_downSUB = self.create_subscription(
            Key, f"{INPUT_NAMESPACE}/keydown", self.key_downSUBCBK, 10
        )
        self.key_upSUB = self.create_subscription(
            Key, f"{INPUT_NAMESPACE}/keyup", self.key_upSUBCBK, 10
        )
        self.joySUB = self.create_subscription(
            Joy, f"{INPUT_NAMESPACE}/joy", self.joySUBCBK, 10
        )  # joystick, new
        self.leg_scanTMR = self.create_timer(
            0.5, self.leg_scanTMRCBK, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.next_scan_ind = 0
        self.selected_joint: Union[int, str, None] = None
        self.selected_joint_joy: Union[int, None] = None
        self.selected_legs: List[int] = []

        self.main_map: Final[InputMap] = (
            self.create_main_map()
        )  # always executed, must not change to always be available
        self.sub_map: InputMap  # will change
        self.enter_select_mode()

        wpub = [
            "/leg11/canopen_motor/base_link1_joint_velocity_controller/command",
            "/leg11/canopen_motor/base_link2_joint_velocity_controller/command",
            "/leg12/canopen_motor/base_link1_joint_velocity_controller/command",
            "/leg12/canopen_motor/base_link2_joint_velocity_controller/command",
            "/leg13/canopen_motor/base_link1_joint_velocity_controller/command",
            "/leg13/canopen_motor/base_link2_joint_velocity_controller/command",
        ]
        self.wpub = [self.create_publisher(Float64, n, 10) for n in wpub]

        # joy
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
        self.move_timer = self.create_timer(
            1.2, self.move_timer_callback
        )  # 0.3 sec delay
        self.joint_timer = self.create_timer(
            0.1, self.joint_control_joy        
            )

        # config
        self.config_index = 0  # current
        self.num_configs = 3  # total configs
        self.prev_config_button = False  # prev config

        self.sendTargetBody: Client = self.create_client(SendTargetBody, "go2_targetbody")
        self.execute_in_cbk_group(self.makeTBclient, MutuallyExclusiveCallbackGroup())

        self.config_names = [
            "Joint Control",
            "IK Control",
            "Vehicle Mode",
        ]  # config names

        self.recover_allCLI = [
            self.create_client(Trigger, f"leg{l}/driver/recover")
            for l in [1, 2, 3, 4, 11, 12, 13, 14]
        ]
        self.halt_allCLI = [
            self.create_client(Trigger, f"leg{l}/driver/halt")
            for l in [1, 2, 3, 4, 11, 12, 13, 14]
        ]

        self.prev_joy_state: JoyState = JoyState()
        self.joy_state: JoyState = JoyState()
        self.ik2TMR = self.create_timer(0.1, self.ik2TMRCBK)
        self.ik2TMR.cancel()
        self.speed_ik2_joy_xyz = 100  # mm/s
        self.speed_ik2_joy_quat = 0.15  # rad/s

        # self.axis_mapping = {
        #     "AXIS_LEFT_X": 1,
        #     "AXIS_LEFT_Y": 0,
        #     "AXIS_RIGHT_X": 4,
        #     "AXIS_RIGHT_Y": 3,
        # }

        # self.button_actions = {
        #     0: "BUTTON_X",
        #     4: "BUTTON_L1",
        #     9: "BUTTON_OPTIONS",
        #     10: "BUTTON_PS",
        # }
        
        self.button_pressed = ""

    def makeTBclient(self):
        self.sendTargetBody.wait_for_service()
        self.pinfo(f"SRV [{self.sendTargetBody.srv_name}] connected")

    @error_catcher
    def leg_scanTMRCBK(self):
        """Looks for new legs and joints"""
        potential_leg: int = LEGNUMS_TO_SCAN[self.next_scan_ind]
        self.next_scan_ind = (self.next_scan_ind + 1) % len(LEGNUMS_TO_SCAN)
        has_looped_to_start = 0 == self.next_scan_ind
        if potential_leg in self.legs.keys():
            self.legs[potential_leg].update_joint_pub()
            if has_looped_to_start:
                return  # stops recursion when loops back to 0
            self.leg_scanTMRCBK()  # continue scanning if already scanned
            return

        cli = self.leg_aliveCLI[potential_leg]
        if cli.wait_for_service(0.01):
            self.pinfo(f"Hey there leg{potential_leg}, nice to meet you")
            self.legs[potential_leg] = Leg(potential_leg, self)
            self.leg_scanTMRCBK()  # continue scanning if leg found
            return
        if len(self.legs.keys()) < 1 and not has_looped_to_start:
            self.leg_scanTMRCBK()  # continue scanning if no legs, unless we looped
            return

        return  # stops scanning if all fails

    @error_catcher
    def key_upSUBCBK(self, msg: Key):
        """Executes when keyboard released"""
        key_char = chr(msg.code)
        key_code = msg.code
        # self.pinfo(f"chr: {chr(msg.code)}, int: {msg.code}")
        self.stop_all_joints()

    def stop_all_joints(self):
        """stops all joint by sending the current angle as target.
        if speed was set, sends a speed of 0 instead"""
        for leg in self.legs.values():
            for joint in leg.joints.keys():
                jobj = leg.get_joint_obj(joint)
                if jobj is None:
                    continue
                if jobj.angle is None:
                    continue
                if jobj.speed_target is None:
                    jobj.set_angle(angle=jobj.angle)
                else:
                    jobj.set_speed(0)

    def all_wheel_speed(self, speed):
        """Need a re-work"""
        speed = float(speed)
        self.wpub[0].publish(Float64(data=-speed))
        self.wpub[1].publish(Float64(data=speed))
        self.wpub[2].publish(Float64(data=speed))
        self.wpub[3].publish(Float64(data=-speed))

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

    def dragon_default(self):
        main_leg_ind = DRAGON_MAIN  # default for all moves
        if main_leg_ind in self.get_active_leg_keys():
            main_leg = self.legs[main_leg_ind]  # default for all moves
            main_leg.ik(xyz=[0, -1200, 0], quat=qt.one)

        manip_leg_ind = DRAGON_MANIP
        if manip_leg_ind in self.get_active_leg_keys():
            manip_leg = self.legs[manip_leg_ind]

            manip_leg.set_angle(-np.pi / 2, 0)
            manip_leg.set_angle(np.pi, 5)
            self.sleep(0.1)

            rot = qt.from_rotation_vector(np.array([1, 0, 0]) * np.pi / 2)
            rot = qt.from_rotation_vector(np.array([0, 1, 0]) * np.pi / 2) * rot
            manip_leg.ik(xyz=[-100, 500, 700], quat=rot)

    def dragon_front_left(self):
        rot = qt.from_rotation_vector(np.array([0, 0, 0.1]))
        self.goToTargetBody(bodyQuat=rot, blocking=False)

    def dragon_front_right(self):
        rot = qt.from_rotation_vector(np.array([0, 0, -0.1]))
        self.goToTargetBody(bodyQuat=rot, blocking=False)

    def dragon_back_left(self):
        main_leg_ind = DRAGON_MAIN  # default for all moves
        main_leg = self.legs[main_leg_ind]  # default for all moves
        rot = qt.from_rotation_vector(np.array([0, 0, 0.1]))
        main_leg.move(quat=rot, blocking=False)

    def dragon_back_right(self):
        main_leg_ind = DRAGON_MAIN  # default for all moves
        main_leg = self.legs[main_leg_ind]  # default for all moves
        rot = qt.from_rotation_vector(np.array([0, 0, -0.1]))
        main_leg.move(quat=rot, blocking=False)

    def vehicle_default(self):
        angs = {
            0: np.pi / 2,
            3: 0.0,
            4: 0.0,
            5: np.pi / 2,
            6: 0.0,
            7: 0.0,
            8: np.pi / 2,
        }
        for leg in self.legs.values():
            # for leg in [self.legs[4]]:
            for num, ang in angs.items():
                jobj = leg.get_joint_obj(num)
                if jobj is None:
                    continue
                jobj.set_angle(ang)

    def zero_without_grippers(self):
        angs = {
            0: 0.0,
            3: 0.0,
            4: 0.0,
            5: 0.0,
            6: 0.0,
            7: 0.0,
            8: 0.0,
        }
        for leg in self.legs.values():
            # for leg in [self.legs[4]]:
            for num, ang in angs.items():
                jobj = leg.get_joint_obj(num)
                if jobj is None:
                    continue
                jobj.set_angle(ang)

    @overload
    def goToTargetBody(
        self,
        ts: Optional[NDArray] = None,
        bodyXYZ: Optional[NDArray] = None,
        bodyQuat: Optional[qt.quaternion] = None,
        blocking: Literal[True] = True,
    ) -> Future: ...

    @overload
    def goToTargetBody(
        self,
        ts: Optional[NDArray] = None,
        bodyXYZ: Optional[NDArray] = None,
        bodyQuat: Optional[qt.quaternion] = None,
        blocking: Literal[False] = False,
    ) -> SendTargetBody.Response: ...

    def goToTargetBody(
        self,
        ts: Optional[NDArray] = None,
        bodyXYZ: Optional[NDArray] = None,
        bodyQuat: Optional[qt.quaternion] = None,
        blocking: bool = True,
    ) -> Union[Future, SendTargetBody.Response]:
        target = TargetBody(
            target_set=np2TargetSet(ts),
            body=np2tf(bodyXYZ, bodyQuat),
        )
        request = SendTargetBody.Request(target_body=target)

        if blocking:
            call = self.sendTargetBody.call(request)
            return call
        else:
            call = self.sendTargetBody.call_async(request)
            return call

    def euler_to_quaternion(
        self, roll: float, pitch: float, yaw: float
    ) -> Optional[qt.quaternion]:
        """
        Convert Euler angles to a quaternion.

        Args:
            roll (float): Rotation around the X-axis in radians.
            pitch (float): Rotation around the Y-axis in radians.
            yaw (float): Rotation around the Z-axis in radians.

        Returns:
            qt.quaternion: The resulting quaternion.
        """
        # Create quaternions for each rotation
        qx = qt.from_rotation_vector(np.array([roll, 0, 0]))
        qy = qt.from_rotation_vector(np.array([0, pitch, 0]))
        qz = qt.from_rotation_vector(np.array([0, 0, yaw]))

        # Combine them: Note that quaternion multiplication is not commutative
        q = qz * qy * qx
        return q

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
        # self.pwarn(ax_active)
        bfield = bfield | (ax_active << BUTT_BITS["stickL"])  # using * is bad

        state.stickR = np.array(sticks_raw[2:], dtype=float)
        state.stickR[[0, 1]] = state.stickR[[1, 0]]  # changes xy
        ax_active = not np.isclose(np.linalg.norm(state.stickR), 0, atol=0.2)
        bfield = bfield | (ax_active << BUTT_BITS["stickR"])  # using * is bad

        state.L2 = (1 - triggers[0]) / 2
        state.R2 = (1 - triggers[1]) / 2

        state.bits = bfield

        return state

    def display_JoyBits(self, joy_state: JoyState):
        self.pinfo(f"bits: {self.joy_state.bits:018b}")
        self.pinfo(f"ints: {self.joy_state.bits}")
        self.pinfo(f"stick L: {self.joy_state.stickL}")
        self.pinfo(f"stick R: {self.joy_state.stickR}")
        self.pinfo(f"trig L: {self.joy_state.L2:.2f}")
        self.pinfo(f"trig R: {self.joy_state.R2:.2f}")
        return

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
            self.pinfo(f"Unknown bit: {(bits & -bits).bit_length() - 1}")
            self.pinfo(f"{bits:018b}")
        button_name = None if button_name == "NONE" else button_name
        return button_name

    def joy_pressed(self, button_name: ButtonName):
        """Executes for each button that is pressed. Like a callback.

        Args:
            bits: Should only have one single bit set to 1, for 1 single button
        """
        dic_key = (button_name, self.joy_state.bits)
        self.pinfo(f"pressed: {dic_key}")
        self.connect_mapping(self.main_map, dic_key)
        self.connect_mapping(self.sub_map, dic_key)

    def joy_released(self, button_name: ButtonName):
        """Executes for each button that is released. Like a callback.

        Args:
            bits: Should only have one single bit set to 1, for 1 single button
        """
        dic_key = (button_name, self.joy_state.bits)
        self.stop_all_joints()
        self.pinfo(f"released: {dic_key}")

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

    def joint_control_joy(self):
        # self.pinfo(selected_joint)
        inc_value = 0
        bits = self.joy_state.bits
        sticks_bits = bits & (
        BUTT_INTS["stickR"] | BUTT_INTS["stickL"] | BUTT_INTS["L1"] | BUTT_INTS["L2"] | BUTT_INTS["R1"])
        if sticks_bits == 0:
            self.joint_timer.cancel()
            return
        
        is_stick_L_vert = not np.isclose(self.joy_state.stickL[0], 0, atol=0.7)
        is_stick_L_horiz = not np.isclose(self.joy_state.stickL[1], 0, atol=0.7)
        is_stick_R_vert = not np.isclose(self.joy_state.stickR[0], 0, atol=0.7)
        is_stick_R_horiz = not np.isclose(self.joy_state.stickR[1], 0, atol=0.7)

        is_L1_held = (bits & BUTT_INTS["L1"] )!=0
        is_R1_held = (bits & BUTT_INTS["R1"] )!=0
        is_L2_held = (bits & BUTT_INTS["L2"] )!=0

        stickL_vert_value = self.joy_state.stickL[0]
        stickL_horiz_value = self.joy_state.stickL[1]
        stickR_vert_value = self.joy_state.stickR[0]
        stickR_horiz_value = self.joy_state.stickR[1]

        if is_stick_L_vert and is_L1_held:
            self.selected_joint_joy = STICKER_TO_ALPHAB.get(1)
            inc_value = stickL_vert_value * MAX_JOINT_SPEED
        elif is_stick_L_horiz and is_L1_held:
            self.selected_joint_joy = STICKER_TO_ALPHAB.get(2)
            inc_value = stickL_horiz_value * MAX_JOINT_SPEED
        elif is_stick_R_vert and is_L1_held:
            self.selected_joint_joy = STICKER_TO_ALPHAB.get(9)
            inc_value = stickR_vert_value * MAX_JOINT_SPEED
        elif is_stick_R_horiz and is_L1_held:
            self.selected_joint_joy = STICKER_TO_ALPHAB.get(8)
            inc_value = stickR_horiz_value * MAX_JOINT_SPEED

        elif is_stick_L_vert and is_R1_held:
            self.selected_joint_joy = STICKER_TO_ALPHAB.get(3)
            inc_value = stickL_vert_value * MAX_JOINT_SPEED
        elif is_stick_L_horiz and is_R1_held:
            self.selected_joint_joy = STICKER_TO_ALPHAB.get(4)
            inc_value = stickL_horiz_value * MAX_JOINT_SPEED
        elif is_stick_R_vert and is_R1_held:
            self.selected_joint_joy = STICKER_TO_ALPHAB.get(7)
            inc_value = stickR_vert_value * MAX_JOINT_SPEED
        elif is_stick_R_horiz and is_R1_held:
            self.selected_joint_joy = STICKER_TO_ALPHAB.get(6)
            inc_value = stickR_horiz_value * MAX_JOINT_SPEED

        elif is_stick_L_vert and is_L2_held:
            self.selected_joint_joy = STICKER_TO_ALPHAB.get(5)
            inc_value = stickL_vert_value * MAX_JOINT_SPEED 
        
        for key in self.get_active_leg_keys():
                leg = self.legs[key]
                jobj = leg.get_joint_obj(self.selected_joint_joy)
                if jobj is None:
                    continue
                jobj.set_speed(inc_value)

    @error_catcher
    def move_timer_callback(self):
        if self.config_index == 1:
            # translational movement
            x = self.current_movement.get("x", 0.0)
            y = self.current_movement.get("y", 0.0)
            z = self.current_movement.get("z", 0.0)

            # rotational movement
            roll = self.current_movement.get("roll", 0.0)
            pitch = self.current_movement.get("pitch", 0.0)
            yaw = self.current_movement.get("yaw", 0.0)

            # convert roll, pitch, yaw to quaternion
            if any([roll, pitch, yaw]):
                quat = self.euler_to_quaternion(roll, pitch, yaw)
            else:
                quat = None  # no rotation

            # xyz movement
            if any([x, y, z]):
                xyz = [x, y, z]
            else:
                xyz = None  # no translation

            # call move() with both xyz and quat if any movement is present
            if xyz or quat:
                for leg in self.legs.values():
                    leg.move(xyz=xyz, quat=quat, mvt_type="shift", blocking=False)

    def joint_timer_start(self):
        if self.joint_timer.is_canceled():
            self.joint_timer.reset()

    # check if button is pressed
    def check_button(self, button: int):
        # instead of
        # if self.check_button(button):
        #   ...
        #
        # you could do
        # if button == PRESSED:
        #   ...
        # where PRESSED is a global variable = 1
        return button == 1

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
        collapsed_KCM = KeyGaitNode.collapseT_KeyCodeModifier(input)
        if collapsed_KCM is not None:  # is KCM
            if not collapsed_KCM[1] == ANY:
                # we run the connection (again?), replacing the key_modifier with ANY
                KeyGaitNode.connect_mapping(mapping, (collapsed_KCM[0], ANY))

        collapsed_JCM = KeyGaitNode.collapseT_JoyCodeModifier(input)
        if collapsed_JCM is not None:  # is JCM
            if not collapsed_JCM[1] == ANY:
                # we run the connection (again?), replacing the key_modifier with ANY
                KeyGaitNode.connect_mapping(mapping, (collapsed_JCM[0], ANY))

    @staticmethod
    def connect_mapping(mapping: InputMap, input: UserInput):
        """Given the user input, executes the corresponding function mapping

        Args:
            mapping: Dict of function to execute
            input: key to the entry to execute
        """
        KeyGaitNode.remap_onto_any(mapping, input)
        if input not in mapping.keys():
            return
        to_execute: List[NakedCall] = mapping[input]
        for f in to_execute:
            f()
        return

    def select_leg(self, leg_ind: Optional[List[int]]):
        """Selects the leg(s) for operation. If None select all.

        Args:
            leg_ind: List of leg keys (leg numbers) to use
        """
        if len(self.legs) < 1:
            self.pinfo("Cannot select: no legs yet")
            return
        if leg_ind is None:
            self.selected_legs = list(self.legs.keys())
            self.pinfo(f"Controling: leg {self.selected_legs}")
            return

        self.selected_legs = []
        for l in leg_ind:
            if l in self.legs.keys():
                self.selected_legs.append(l)
            else:
                self.pwarn(f"Leg {l} does not exist")

        self.pinfo(f"Controling: leg {self.selected_legs}")

    def cycle_leg_selection(self, increment: Optional[int]):
        """Cycles the leg selection by increment
        if None, selects all known legs
        """
        if len(self.legs) < 1:
            self.pinfo("Cannot select: no legs yet")
            return

        leg_keys = list(self.legs.keys())
        if increment is None:
            self.selected_legs = leg_keys
        elif len(self.selected_legs) != 1:
            first_leg = leg_keys[0]
            self.selected_legs = [first_leg]
        else:
            next_index: int = (leg_keys.index(self.selected_legs[0]) + increment) % len(
                self.legs
            )
            self.selected_legs = [leg_keys[next_index]]
        self.pinfo(f"Controling: leg {self.selected_legs}")

    def get_active_leg(self, leg_key: Union[List[int], int, None] = None) -> List[Leg]:
        """Return the keys to get the current active legs from the self.legs dict

        Args:
            leg_number: you can specify a leg key if you need instead of using active legs

        Returns:
            list of active leg keys
        """
        return [self.legs[k] for k in self.get_active_leg_keys(leg_key)]

    def get_active_leg_keys(
        self, leg_key: Union[List[int], int, None] = None
    ) -> List[int]:
        """Return the keys to get the current active legs from the self.legs dict

        Args:
            leg_number: you can specify a leg key if you need instead of using active legs

        Returns:
            list of active leg keys
        """
        if leg_key is None:
            if self.selected_legs is None:
                self.select_leg(None)
            return self.selected_legs
        if isinstance(leg_key, int):
            leg_key = [leg_key]
        return [l for l in leg_key if l in self.selected_legs]

    def halt_all(self):
        self.pinfo("HALTING ALL")
        for cli in self.halt_allCLI:
            cli.call_async(Trigger.Request())

    def halt_detected(self):
        self.pinfo("HALTING")
        for leg in self.legs.values():
            leg.halt()

    def recover_all(self, leg_keys: Union[List[int], int, None] = None):
        self.pinfo("RECOVERING ALL")
        for cli in self.recover_allCLI:
            cli.call_async(Trigger.Request())

    def recover_legs(self, leg_keys: Union[List[int], int, None] = None):
        self.pinfo("RECOVERING")
        active_keys = self.get_active_leg_keys(leg_keys)
        for k in active_keys:
            leg = self.legs[k]
            leg.recover()

    def set_joint_speed(
        self,
        speed: float,
        joint: Union[int, str, None] = None,
        leg_number: Optional[int] = None,
    ):
        """Sets joint speed or given joints and legs.
        If Nones, picks the selected or active things

        Args:
            speed:
            joint:
            leg_number:
        """
        if joint is None:
            joint = self.selected_joint
        if joint is None:
            return

        leg_keys = self.get_active_leg_keys(leg_number)

        for k in leg_keys:
            leg = self.legs[k]
            jobj = leg.get_joint_obj(joint)
            if jobj is None:
                continue
            jobj.set_speed(speed)

    def start_ik2_timer(self):
        if self.ik2TMR.is_canceled():
            elapsed = Duration(nanoseconds=self.ik2TMR.time_since_last_call())
            if elapsed > Duration(seconds=2):
                for leg in self.get_active_leg():
                    leg.reset_ik2_offset()
            self.ik2TMR.reset()
            self.ik2TMR.callback()

    def ik2TMRCBK(self):
        bits = self.joy_state.bits
        sticks_bits = bits & (
            BUTT_INTS["stickR"] | BUTT_INTS["stickL"] | BUTT_INTS["R1"] | BUTT_INTS["L1"]
        )
        sticks_active = not (sticks_bits == 0)
        if not sticks_active:
            self.ik2TMR.cancel()
            return

        # maximum we should move this tick
        speed_xyz = 100  # mm/s
        speed_quat = 0.15  # rad/s
        delta_xyz = speed_xyz * self.ik2TMR.timer_period_ns / 1e9
        delta_quat = speed_quat * self.ik2TMR.timer_period_ns / 1e9

        act_legs = self.get_active_leg()
        xyz_input = np.empty((3,), dtype=float)

        if (self.joy_state.bits == (BUTT_INTS["L1"] + BUTT_INTS["stickL"]) or 
            self.joy_state.bits == (BUTT_INTS["L1"] + BUTT_INTS["stickR"]) or
            self.joy_state.bits == (BUTT_INTS["L1"] + BUTT_INTS["stickL"] + BUTT_INTS["stickR"])
            ):
            xyz_input[[0, 1]] = self.joy_state.stickL
            xyz_input[2] = self.joy_state.stickR[0]
            for leg in act_legs:
                leg.apply_ik2_offset(
                    xyz=xyz_input * delta_xyz,
                    quat=None,
            )
            
        if (self.joy_state.bits == (BUTT_INTS["R1"] + BUTT_INTS["stickL"]) or 
            self.joy_state.bits == (BUTT_INTS["R1"] + BUTT_INTS["stickR"]) or
            self.joy_state.bits == (BUTT_INTS["R1"] + BUTT_INTS["stickL"] + BUTT_INTS["stickR"])
            ):
            z_rot = qt.from_rotation_vector([self.joy_state.stickR[0], 0, 0])
            y_rot = qt.from_rotation_vector([0, self.joy_state.stickL[1], 0])
            x_rot = qt.from_rotation_vector([0, 0, self.joy_state.stickL[0]])
            rot = (x_rot * y_rot * z_rot) ** delta_quat
            for leg in act_legs:
                leg.apply_ik2_offset(
                    xyz=None,
                    quat=rot * qt.one,
            )

        # xyz_input[[0, 1]] = self.joy_state.stickL
        # xyz_input[2] = -self.joy_state.R2 + self.joy_state.L2
        # x_rot = qt.from_rotation_vector([self.joy_state.stickR[0], 0, 0])
        # y_rot = qt.from_rotation_vector([0, self.joy_state.stickR[1], 0])
        # rot = (x_rot * y_rot) ** delta_quat

        # if np.linalg.norm(xyz_input) < 0.01:
        # return
        # for leg in act_legs:
        #     leg.apply_ik2_offset(
        #         xyz=xyz_input * delta_xyz,
        #         quat=rot * qt.one,
        #     )

    def select_joint(self, joint_index):
        """can be better"""
        self.selected_joint = joint_index
        all_controled_joints = [
            self.legs[l_key].get_joint_obj(joint_index).joint_name
            for l_key in self.get_active_leg_keys()
            if self.legs[l_key].get_joint_obj(joint_index) is not None
        ]
        self.pinfo(f"Controling joint: {all_controled_joints}")

    def angle_zero(self, leg_number: Union[int, List[int], None] = None):
        """Sets all joint angles to 0 (dangerous)

        Args:
            leg_number: The leg on which to set. If none, applies on the active leg
        """
        leg_keys = self.get_active_leg_keys(leg_number)
        for k in leg_keys:
            leg = self.legs[k]
            leg.go2zero()

    def enter_dragon_mode(self) -> None:
        """Creates the sub input map for dragon

        Returns:
            InputMap for joint control
        """
        self.pinfo(f"Dragon Mode")
        submap: InputMap = {
            (Key.KEY_R, ANY): [self.dragon_default],
            (Key.KEY_D, ANY): [self.dragon_back_left, self.dragon_front_right],
            (Key.KEY_A, ANY): [self.dragon_back_right, self.dragon_front_left],
            (Key.KEY_G, ANY): [self.dragon_front_left],
            (Key.KEY_H, ANY): [self.dragon_front_right],
            (Key.KEY_B, ANY): [self.dragon_back_left],
            (Key.KEY_N, ANY): [self.dragon_back_right],
        }

        self.sub_map = submap

    def enter_leg_mode(self) -> None:
        """Creates the sub input map for leg selection

        Returns:
            InputMap for leg selection
        """
        self.pinfo(
            f"Leg Select Mode: [1,2...] -> Leg 1,2...; [L] [DOWN] -> all Legs; "
            f"[->] -> Next; [<-] Previous"
        )
        submap: InputMap = {
            (Key.KEY_RIGHT, ANY): [lambda: self.cycle_leg_selection(1)],
            (Key.KEY_LEFT, ANY): [lambda: self.cycle_leg_selection(-1)],
            (Key.KEY_DOWN, ANY): [lambda: self.select_leg(None)],
            (Key.KEY_L, ANY): [lambda: self.select_leg(None)],
            ("right", ANY): [lambda: self.cycle_leg_selection(1)],
            ("left", ANY): [lambda: self.cycle_leg_selection(-1)],
            ("down", ANY): [lambda: self.cycle_leg_selection(None)],
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

    def enter_ik2(self) -> None:
        """Creates the sub input map for ik control lvl2 by elian

        Returns:
            InputMap for ik2 control
        """
        self.pinfo(f"IK2 Mode")
        submap: InputMap = {
            ("stickL", ANY): [self.start_ik2_timer],
            ("stickR", ANY): [self.start_ik2_timer],
            # ("R2", ANY): [self.start_ik2_timer],
            # ("L2", ANY): [self.start_ik2_timer],
        }

        self.sub_map = submap

    def enter_joint_mode(self) -> None:
        """Creates the sub input map for joint control

        Returns:
            InputMap for joint control
        """
        self.pinfo(f"Joint Control Mode")
        submap: InputMap = {
            (Key.KEY_W, ANY): [lambda: self.set_joint_speed(MAX_JOINT_SPEED)],
            (Key.KEY_S, ANY): [lambda: self.set_joint_speed(-MAX_JOINT_SPEED)],
            (Key.KEY_0, ANY): [self.angle_zero],

            # joy mapping
            # ("L1", BUTT_INTS["stickL"] + BUTT_INTS["L1"]): [lambda: self.joint_control_joy()],
            ("stickL", BUTT_INTS["stickL"] + BUTT_INTS["L1"]): [self.joint_timer_start],
            ("stickR", BUTT_INTS["stickR"] + BUTT_INTS["L1"]): [self.joint_timer_start],
            ("stickL", BUTT_INTS["stickL"] + BUTT_INTS["R1"]): [self.joint_timer_start],
            ("stickR", BUTT_INTS["stickR"] + BUTT_INTS["R1"]): [self.joint_timer_start],
            ("stickL", BUTT_INTS["stickL"] + BUTT_INTS["L2"]): [self.joint_timer_start],




        }
        one2nine_keys = [
            (0, Key.KEY_1),
            (1, Key.KEY_2),
            (2, Key.KEY_3),
            (3, Key.KEY_4),
            (4, Key.KEY_5),
            (5, Key.KEY_6),
            (6, Key.KEY_7),
            (7, Key.KEY_8),
            (8, Key.KEY_9),
        ]
        for n, keyb in one2nine_keys:
            n = STICKER_TO_ALPHAB[n + 1]
            submap[(keyb, ANY)] = [lambda n=n: self.select_joint(n)]

        self.sub_map = submap

    def no_no_leg(self):
        """Makes sure no legs are not selected"""
        if self.selected_legs is None:
            self.select_leg(None)
        if not self.selected_legs:
            self.select_leg(None)

    def enter_select_mode(self):
        """Mode to select other modes.
        Should always be accessible when pressing ESC key"""
        self.pinfo(
            f"Mode Select Mode: "
            f"J -> Joint, "
            f"L -> Leg, "
            f"D -> Dragon, "
            f"K -> IK2"
        )
        self.pinfo(
            f"Mode Select Mode (Joystick): "
            f"X -> Joint, "
            f"□ -> Leg, "
            f"∆ -> Dragon, "
            f"○ -> IK2"
        )

        self.sub_map = {
            (Key.KEY_J, ANY): [self.no_no_leg, self.enter_joint_mode],
            (Key.KEY_L, ANY): [self.no_no_leg, self.enter_leg_mode],
            (Key.KEY_D, ANY): [self.no_no_leg, self.enter_dragon_mode],
            (Key.KEY_K, ANY): [self.no_no_leg, self.enter_ik2],
            (Key.KEY_SPACE, ANY): [self.halt_detected],
            (Key.KEY_SPACE, Key.MODIFIER_LSHIFT): [self.halt_all],
            ("t", ANY): [self.no_no_leg, self.enter_dragon_mode],
            ("o", ANY): [self.no_no_leg, self.enter_ik2],
            ("x", ANY): [self.no_no_leg, self.enter_joint_mode],
            ("s", ANY): [self.no_no_leg, self.enter_leg_mode],
        }
    
    def easy_mode_wheel(self):

        submap: InputMap = {
            ("up", ANY): [lambda: self.all_wheel_speed(100000)],
            ("down", ANY): [lambda: self.all_wheel_speed(-100000)],
            ("x", ANY): [lambda: self.all_wheel_speed(0)],
        }

        self.sub_map = submap
            
            

    def create_main_map(self) -> InputMap:
        """Creates the main input map, mapping user input to functions,
        This is supposed to be constant + always active, unlike the sub_map"""
        main_map: InputMap = {
            # (Key.KEY_O, ANY): [lambda: self.all_whell_speed(100000)],
            # (Key.KEY_L, ANY): [lambda: self.all_whell_speed(-100000)],
            # (Key.KEY_P, ANY): [lambda: self.all_whell_speed(0)],
            (Key.KEY_RETURN, ANY): [self.recover_legs],
            (Key.KEY_RETURN, Key.MODIFIER_LSHIFT): [self.recover_all],
            (Key.KEY_ESCAPE, ANY): [self.enter_select_mode],
            # (Key.KEY_ESCAPE, ANY): [self.halt_detected],
            # (Key.KEY_ESCAPE, Key.MODIFIER_LSHIFT): [self.halt_all],
            # (Key.KEY_RIGHT, ANY): [lambda: self.cycle_leg_selection(1)],
            # (Key.KEY_LEFT, ANY): [lambda: self.cycle_leg_selection(-1)],
            # (Key.KEY_DOWN, ANY): [lambda: self.cycle_leg_selection(None)],
            # (Key.KEY_O, ANY): [lambda: self.all_wheel_speed(100000)],
            # (Key.KEY_L, ANY): [lambda: self.all_wheel_speed(-100000)],
            # (Key.KEY_P, ANY): [lambda: self.all_wheel_speed(0)],
            # (Key.KEY_C, ANY): [self.recover_legs],
            # (Key.KEY_ESCAPE, ANY): [self.halt_all],
            # joy mapping
            ("option", ANY): [self.enter_select_mode],
            ("R2", ANY): [self.recover_legs],
            ("PS", ANY): [self.halt_all],
            ("R2", BUTT_INTS["L2"] + BUTT_INTS["R2"]): [self.recover_all],
            ("L2", BUTT_INTS["L2"] + BUTT_INTS["R2"]): [self.recover_all],
            
            ("stickLpush", BUTT_INTS["stickLpush"] + BUTT_INTS["stickRpush"]): [
                self.easy_mode_wheel
            ],
            ("stickRpush", BUTT_INTS["stickLpush"] + BUTT_INTS["stickRpush"]): [
                self.easy_mode_wheel
            ],
        }
        return main_map


def main(args=None):
    myMain(KeyGaitNode, multiThreaded=True)


if __name__ == "__main__":
    main()

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
    overload,
)
import re
import numpy as np
from numpy.typing import NDArray
import quaternion as qt
import rclpy
from rclpy.task import Future
from rclpy.node import Union, List
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.publisher import Publisher
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
from std_srvs.srv import Empty

from easy_robot_control.gait_node import Leg, MVT2SRV, AvailableMvt

# type def V
#
ANY: Final[str] = "ANY"
ALWAYS: Final[str] = "ALWAYS"
KeyCodeModifier = Tuple[int, Union[int, Literal["ANY"]]]  # keyboard input: key + modifier
UserInput = Union[
    KeyCodeModifier,
    Tuple[str, str],
    Literal["ALWAYS"],  # functions associated with "ALWAYS" string will always execute
]  # add you input type here for joystick,
# MUST be an imutable object (or you'll hurt yourself)
NakedCall = Callable[[], Any]
InputMap = Dict[UserInput, List[NakedCall]]  # User input are linked to a list of function
#
# type def ^

# LEGNUMS_TO_SCAN = range(10)



# Define scaling constants globally
TRANSLATION_SCALE = 20 # translational IK
ROTATION_SCALE = np.deg2rad(1.5)  # rotational IK

INPUT_NAMESPACE = "/elian/"
LEGNUMS_TO_SCAN = [1, 2]
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

DRAGON_MAIN: int = 1
DRAGON_MANIP: int = 2


class KeyGaitNode(EliaNode):
    def __init__(self, name: str = "keygait_node"):
        super().__init__(name)
        self.Alias = "G"
        # self.setAndBlockForNecessaryClients("mover_alive")

        self.leg_aliveCLI: Dict[int, Client] = dict(
            [(l, self.create_client(Empty, f"leg{l}/leg_alive")) for l in LEGNUMS_TO_SCAN]
        )
        self.legs: Dict[int, Leg] = {}

        self.key_downSUB = self.create_subscription(
            Key, f"{INPUT_NAMESPACE}keydown", self.key_downSUBCBK, 10
        )
        self.key_upSUB = self.create_subscription(
            Key, f"{INPUT_NAMESPACE}keyup", self.key_upSUBCBK, 10
        )
        self.joySUB = self.create_subscription(
            Joy, f"{INPUT_NAMESPACE}joy", self.joySUBCBK, 10
        )  # joystick, new
        self.leg_scanTMR = self.create_timer(
            0.5, self.leg_scanTMRCBK, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.next_scan_ind = 0
        self.selected_joint: Union[int, str, None] = None
        self.active_leg: Union[List[int], int, None] = None

        self.main_map: Final[InputMap] = (
            self.create_main_map()
        )  # always executed, must not change to always be available
        self.sub_map: InputMap = {}  # will change
        self.mode_index: Optional[int] = None
        self.modes: List[NakedCall] = [self.mode_joint, self.mode_dragon]

        wpub = [
            "/leg11/canopen_motor/base_link1_joint_velocity_controller/command",
            "/leg11/canopen_motor/base_link2_joint_velocity_controller/command",
            "/leg13/canopen_motor/base_link1_joint_velocity_controller/command",
            "/leg13/canopen_motor/base_link2_joint_velocity_controller/command",
        ]
        self.wpub = [self.create_publisher(Float64, n, 10) for n in wpub]

        # joy
        self.prev_axes = None
        self.prev_buttons = None
        self.deadzone = 0.05
        self.neutral_threshold = 0.1
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
            "yaw_prev": 0.0
        }
        self.move_timer = self.create_timer(
            1.2, self.move_timer_callback
        )  # 0.3 sec delay

        # config
        self.config_index = 0  # current
        self.num_configs = 3  # total configs
        self.prev_config_button = False  # prev config

        # self.sendTargetBody: Client = self.get_and_wait_Client(
        #     "go2_targetbody", SendTargetBody
        # )

        self.config_names = [
            "Joint Control",
            "IK Control",
            "Vehicle Mode",
        ]  # config names

    @error_catcher
    def leg_scanTMRCBK(self):
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
        key_char = chr(msg.code)
        key_code = msg.code
        # self.pinfo(f"chr: {chr(msg.code)}, int: {msg.code}")
        self.stop_all_joints()

    def stop_all_joints(self):
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

    def all_whell_speed(self, speed):
        speed = float(speed)
        self.wpub[0].publish(Float64(data=-speed))
        self.wpub[1].publish(Float64(data=speed))
        self.wpub[2].publish(Float64(data=speed))
        self.wpub[3].publish(Float64(data=-speed))


    @error_catcher
    def key_downSUBCBK(self, msg: Key):
        key_char = chr(msg.code)
        key_code = msg.code
        key_modifier = msg.modifiers
        self.connect_mapping(self.main_map, (key_code, key_modifier))
        self.connect_mapping(self.sub_map, (key_code, key_modifier))
        return
        # self.pinfo(f"chr: {chr(msg.code)}, int: {msg.code}")
        s: Optional[float] = None
        if key_char == "o":
            s = 10000.0
        if key_char == "p":
            s = 0.0
        if key_char == "l":
            s = -10000.0
        if s is not None:
            self.wpub[0].publish(Float64(data=-s))
            self.wpub[1].publish(Float64(data=s))
            # self.wpub[2].publish(Float64(data=s))
            # self.wpub[3].publish(Float64(data=-s))
        if key_char == "0":
            for leg in self.legs.values():
                leg.go2zero()

        if key_char in [f"{num + 1}" for num in range(9)]:  # +1 to avoid 0
            self.selected_joint = int(key_char) - 1
            self.pinfo(
                f"selected joint {self.selected_joint}: "
                f"{[l.joint_name_list[self.selected_joint] for l in self.legs.values()if self.selected_joint < len(l.joint_name_list)]}"
            )
        if key_char == "r":
            self.dragon_default()

        if self.selected_joint is not None:
            self.joint_control_key(key_char)
            return

        # if statement hell, yes bad, if you unhappy fix it
        DIST = 20
        if key_char == "b":
            self.dragon_front_left()
        if key_char == "n":
            self.dragon_front_right()
        if key_char == "g":
            self.dragon_back_left()
        if key_char == "h":
            self.dragon_back_right()
        if key_char == "w":
            for leg in self.legs.values():
                leg.move(xyz=[DIST, 0, 0], blocking=False)
        elif key_char == "s":
            for leg in self.legs.values():
                leg.move(xyz=[-DIST, 0, 0], blocking=False)
        if key_char == "a":
            for leg in self.legs.values():
                leg.move(xyz=[0, DIST, 0], blocking=False)
        elif key_char == "d":
            for leg in self.legs.values():
                leg.move(xyz=[0, -DIST, 0], blocking=False)

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
        blocking: Literal[False] = False,
    ) -> SendTargetBody.Response: ...

    @overload
    def goToTargetBody(
        self,
        ts: Optional[NDArray] = None,
        bodyXYZ: Optional[NDArray] = None,
        bodyQuat: Optional[qt.quaternion] = None,
        blocking: Literal[True] = True,
    ) -> Future: ...

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

    def joint_control_key(self, key_char):
        if self.selected_joint is None:
            return
        if key_char == "w":
            inc = MAX_JOINT_SPEED
        elif key_char == "s":
            inc = -MAX_JOINT_SPEED
        else:
            return

        for leg in self.legs.values():
            jobj = leg.get_joint_obj(self.selected_joint)
            if jobj is None:
                continue
            jobj.set_speed(inc)

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Optional[qt.quaternion]:
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

    @error_catcher
    def joySUBCBK(self, msg: Joy):
        # normalizing axes to be zero
        joy_axes = [
            0.0 if abs(axis - default) < self.deadzone else round(axis, 2)
            for axis, default in zip(msg.axes, self.default_neutral_axes)
        ]
        joy_buttons = msg.buttons

        # extracting individual axes
        axis_left_y = joy_axes[0]  # horizontal left
        axis_left_x = joy_axes[1]  # vertical left
        axis_right_y = joy_axes[3]  # horizontal right
        axis_right_x = joy_axes[4]  # vertical right

        # detecting if joystick is being used
        axis_held = any(abs(axis) > self.neutral_threshold for axis in joy_axes)
        # self.pinfo(f"poggers?: {axis_held}")

        # comparison with previous state
        axes_changed = self.prev_axes is None or any(
            abs(current - previous) > self.deadzone
            for current, previous in zip(joy_axes, self.prev_axes)
        )
        buttons_changed = self.prev_buttons is None or joy_buttons != self.prev_buttons

        # check if a change or active hold state is detected
        if not axes_changed and not buttons_changed and not axis_held:
            return

        # handle configs
        current_config_button = self.check_button(joy_buttons[9])  # options button
        if current_config_button and not self.prev_config_button:
            # cycles to the next config
            self.config_index = (self.config_index + 1) % self.num_configs
            self.pinfo(
                f"Switched to configuration {self.config_index}: {self.config_names[self.config_index]}"
            )
        # update the prev config
        self.prev_config_button = current_config_button

        # stopping
        stop = self.check_button(joy_buttons[10])  # PS button
        if stop:
            self.stop_all_joints()
            return

        # define button holds
        l1_held = self.check_button(joy_buttons[4])  # L1 button
        r1_held = self.check_button(joy_buttons[5])  # R1 button
        l2_held = self.check_button(joy_buttons[6])  # L2 button


        # Joint Control joy
        if self.config_index == 0:
            # ====== Handle L1 Control ======
            if l1_held and axis_held:
                selected_joint_1 = JOINT_STICKER_NUMBER.get(1)
                selected_joint_2 = JOINT_STICKER_NUMBER.get(2)
                selected_joint_3 = JOINT_STICKER_NUMBER.get(9)
                selected_joint_4 = JOINT_STICKER_NUMBER.get(8)

                # Joint 1 (gripper) with vertical left
                if axis_left_x != 0:
                    inc = axis_left_x * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_1, inc)
                else:
                    self.joint_control_joy(selected_joint_1, 0.0)

                # Joint 2 with horizontal left
                if axis_left_y != 0:
                    inc = axis_left_y * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_2, inc)
                else:
                    self.joint_control_joy(selected_joint_2, 0.0)

                # Joint 9 (gripper) with vertical right
                if axis_right_x != 0:
                    inc = axis_right_x * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_3, inc)
                else:
                    self.joint_control_joy(selected_joint_3, 0.0)

                # Joint 8 with horizontal right
                if axis_right_y != 0:
                    inc = axis_right_y * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_4, inc)
                else:
                    self.joint_control_joy(selected_joint_4, 0.0)
            else:
                # If L1 is not held or axis not held, stop L1's joints
                selected_joint_1 = JOINT_STICKER_NUMBER.get(1)
                selected_joint_2 = JOINT_STICKER_NUMBER.get(2)
                selected_joint_3 = JOINT_STICKER_NUMBER.get(9)
                selected_joint_4 = JOINT_STICKER_NUMBER.get(8)
                self.joint_control_joy(selected_joint_1, 0.0)
                self.joint_control_joy(selected_joint_2, 0.0)
                self.joint_control_joy(selected_joint_3, 0.0)
                self.joint_control_joy(selected_joint_4, 0.0)

            # ====== Handle R1 Control ======
            if r1_held and axis_held:
                selected_joint_1 = JOINT_STICKER_NUMBER.get(3)
                selected_joint_2 = JOINT_STICKER_NUMBER.get(4)
                selected_joint_3 = JOINT_STICKER_NUMBER.get(7)
                selected_joint_4 = JOINT_STICKER_NUMBER.get(6)

                # Joint 3 with vertical left
                if axis_left_x != 0:
                    inc = axis_left_x * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_1, inc)
                else:
                    self.joint_control_joy(selected_joint_1, 0.0)

                # Joint 4 with horizontal left
                if axis_left_y != 0:
                    inc = axis_left_y * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_2, inc)
                else:
                    self.joint_control_joy(selected_joint_2, 0.0)

                # Joint 7 with vertical right
                if axis_right_x != 0:
                    inc = axis_right_x * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_3, inc)
                else:
                    self.joint_control_joy(selected_joint_3, 0.0)

                # Joint 6 with horizontal right
                if axis_right_y != 0:
                    inc = axis_right_y * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_4, inc)
                else:
                    self.joint_control_joy(selected_joint_4, 0.0)
            else:
                # If R1 is not held or axis not held, stop R1's joints
                selected_joint_1 = JOINT_STICKER_NUMBER.get(3)
                selected_joint_2 = JOINT_STICKER_NUMBER.get(4)
                selected_joint_3 = JOINT_STICKER_NUMBER.get(7)
                selected_joint_4 = JOINT_STICKER_NUMBER.get(6)
                self.joint_control_joy(selected_joint_1, 0.0)
                self.joint_control_joy(selected_joint_2, 0.0)
                self.joint_control_joy(selected_joint_3, 0.0)
                self.joint_control_joy(selected_joint_4, 0.0)

            # ====== Handle L2 Control ======
            if l2_held and axis_held:
                selected_joint_1 = JOINT_STICKER_NUMBER.get(5)

                # Joint 5 with vertical left
                if axis_left_x != 0:
                    inc = axis_left_x * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_1, inc)
                else:
                    self.joint_control_joy(selected_joint_1, 0.0)
            else:
                # If L2 is not held or axis not held, stop L2's joint
                selected_joint_1 = JOINT_STICKER_NUMBER.get(5)
                self.joint_control_joy(selected_joint_1, 0.0)
                    
        # IK Control
        if self.config_index == 1:
            if l1_held and axis_held:
                if axis_left_x != 0:
                    x = axis_left_x * TRANSLATION_SCALE
                    self.current_movement["x"] = x
                else:
                    self.current_movement["x"] = 0.0
                if axis_left_y != 0:
                    y = axis_left_y * TRANSLATION_SCALE
                    self.current_movement["y"] = y
                else:
                    self.current_movement["y"] = 0.0
                if axis_right_x != 0:
                    z = axis_right_x * TRANSLATION_SCALE
                    self.current_movement["z"] = z
                else:
                    self.current_movement["z"] = 0.0
            else:
                self.current_movement["x"] = 0.0
                self.current_movement["y"] = 0.0
                self.current_movement["z"] = 0.0

            if r1_held and axis_held:
                if axis_left_x != 0:
                    roll = axis_left_x * ROTATION_SCALE
                    self.current_movement["roll"] = roll
                else:
                    self.current_movement["roll"] = 0.0
                if axis_left_y != 0:
                    pitch = axis_left_y * ROTATION_SCALE
                    self.current_movement["pitch"] = pitch
                else:
                    self.current_movement["pitch"] = 0.0
                if axis_right_x != 0:
                    yaw = axis_right_x * ROTATION_SCALE
                    self.current_movement["yaw"] = yaw
                else:
                    self.current_movement["yaw"] = 0.0
            else:
                self.current_movement["roll"] = 0.0
                self.current_movement["pitch"] = 0.0
                self.current_movement["yaw"] = 0.0

        # if self.config_index == 2:


        # at config 0, zeroing
        if self.config_index == 0:
            zero = self.check_button(joy_buttons[0])  # X button
            if zero:
                self.zero_without_grippers()
            # else:
            #     self.stop_all_joints()


    def joint_control_joy(self, selected_joint, inc_value):
        # self.pinfo(selected_joint)
        for leg in self.legs.values():
            jobj = leg.get_joint_obj(selected_joint)
            if jobj is None:
                continue
            jobj.set_speed(inc_value)

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Optional[qt.quaternion]:
        """
        Converts Euler angles to a quaternion.

        Args:
            roll (float): rotation around the X-axis in radians.
            pitch (float): rotation around the Y-axis in radians.
            yaw (float): rotation around the Z-axis in radians.

        Returns:
            qt.quaternion: the resulting quaternion.
        """
        qx = qt.from_rotation_vector(np.array([roll, 0, 0]))
        qy = qt.from_rotation_vector(np.array([0, pitch, 0]))
        qz = qt.from_rotation_vector(np.array([0, 0, yaw]))
        
        # quaternion multiplication, I think this should be right
        q = qz * qy * qx
        return q

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
                    leg.move(
                        xyz=xyz,
                        quat=quat,
                        mvt_type="shift",
                        blocking=False
                    )

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
    def remap_onto_any(mapping: InputMap, input: UserInput):
        """runs the input through the INPUTMap as if the key_modifier was any
        if it is already, it does not run it.
        """
        collapsed_KCM = KeyGaitNode.collapseT_KeyCodeModifier(input)
        if collapsed_KCM is not None:  # is KCM
            if not collapsed_KCM[1] == ANY:
                # we run the connection (again?), replacing the key_modifier with ANY
                KeyGaitNode.connect_mapping(mapping, (collapsed_KCM[0], ANY))

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

    def cycle_leg_selection(self, increment: Optional[int]):
        """Cycles the leg selection by increment
        if None, selects all known legs
        """
        if len(self.legs) < 1:
            self.pinfo("Cannot select: no legs yet")
            return

        leg_keys = list(self.legs.keys())
        if increment is None:
            self.active_leg = leg_keys
        elif isinstance(self.active_leg, list) or self.active_leg is None:
            first_leg = leg_keys[0]
            self.active_leg = first_leg
        else:
            next_index: int = (leg_keys.index(self.active_leg) + increment) % len(
                self.legs
            )
            self.active_leg = leg_keys[next_index]
        self.pinfo(f"Controling: leg {self.active_leg}")

    def cycle_mode(self):
        if self.mode_index is None:
            self.mode_index = -1
        self.mode_index = (self.mode_index + 1) % len(self.modes)
        self.sub_map = self.modes[self.mode_index]()

    def get_active_leg_keys(
        self, leg_key: Union[List[int], int, None] = None
    ) -> List[int]:
        """Return the keys to get the current active legs from the self.legs dict

        Args:
            leg_number: you can specify a leg key if you need instead of using active legs

        Returns:
            list of active leg keys
        """
        leg_keys: List[int]
        if leg_key is None:
            if self.active_leg is None:
                self.cycle_leg_selection(None)
            if self.active_leg is None:
                return []
            if isinstance(self.active_leg, int):
                leg_keys = [self.active_leg]
            else:
                leg_keys = self.active_leg
        elif isinstance(leg_key, int):
            leg_keys = [leg_key]
        else:
            leg_keys = leg_key.copy()
        return leg_keys

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

    def mode_dragon(self) -> InputMap:
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

        return submap

    def mode_joint(self) -> InputMap:
        """Creates the sub input map for joint control

        Returns:
            InputMap for joint control
        """
        self.pinfo(f"Joint Control Mode")
        submap: InputMap = {
            (Key.KEY_W, ANY): [lambda: self.set_joint_speed(MAX_JOINT_SPEED)],
            (Key.KEY_S, ANY): [lambda: self.set_joint_speed(-MAX_JOINT_SPEED)],
            (Key.KEY_0, ANY): [self.angle_zero],
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

        return submap

    def create_main_map(self) -> InputMap:
        """Creates the main input map, mapping user input to functions,
        This is supposed to be constant + always active, unlike the sub_map"""
        main_map: InputMap = {
            (Key.KEY_RIGHT, ANY): [lambda: self.cycle_leg_selection(1)],
            (Key.KEY_LEFT, ANY): [lambda: self.cycle_leg_selection(-1)],
            (Key.KEY_DOWN, ANY): [lambda: self.cycle_leg_selection(None)],
            (Key.KEY_M, ANY): [lambda: self.cycle_mode()],
            (Key.KEY_O, ANY): [lambda: self.all_whell_speed(100000)],
            (Key.KEY_L, ANY): [lambda: self.all_whell_speed(-100000)],
            (Key.KEY_P, ANY): [lambda: self.all_whell_speed(0)],
        }
        return main_map


def main(args=None):
    myMain(KeyGaitNode, multiThreaded=True)


if __name__ == "__main__":
    main()

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
    Literal["ALWAYS"],
]  # add you input type here for joystick,
# MUST be an imutable object (or you'll hurt yourself)
NakedCall = Callable[[], Any]
InputMap = Dict[UserInput, List[NakedCall]]  # User input are linked to a list of function
#
# type def ^

# LEGNUMS_TO_SCAN = range(10)
LEGNUMS_TO_SCAN = [3, 4]
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
            Key, "keydown", self.key_downSUBCBK, 10
        )
        self.key_upSUB = self.create_subscription(Key, "keyup", self.key_upSUBCBK, 10)
        self.joySUB = self.create_subscription(
            Joy, "joy", self.joySUBCBK, 10
        )  # joystick, new
        self.leg_scanTMR = self.create_timer(
            0.5, self.leg_scanTMRCBK, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.next_scan_ind = 0
        self.selected_joint: Union[int, str, None] = None
        self.chosen_leg: Union[List[int], int, None] = None

        self.main_map: Final[InputMap] = (
            self.create_main_map()
        )  # always executed, must not change to always be available
        self.sub_map: InputMap = {}  # will change
        self.mode_index: Optional[int] = None
        self.modes: List[NakedCall] = [self.mode_joint]

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
        self.current_movement = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.move_timer = self.create_timer(
            0.2, self.move_timer_callback
        )  # 0.2 sec delay

        # config
        self.config_index = 0  # current
        self.num_configs = 3  # total configs
        self.prev_config_button = False  # prev config

        self.sendTargetBody: Client = self.get_and_wait_Client(
            "go2_targetbody", SendTargetBody
        )

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
            self.wpub[2].publish(Float64(data=s))
            self.wpub[3].publish(Float64(data=-s))
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
            self.dragon_front_up()
        if key_char == "n":
            self.dragon_front_down()
        if key_char == "g":
            self.dragon_back_up()
        if key_char == "h":
            self.dragon_back_down()
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
        main_leg_ind = 4  # default for all moves
        if main_leg_ind in self.legs.keys():
            main_leg = self.legs[main_leg_ind]  # default for all moves
            main_leg.ik(xyz=[0, -1200, 0], quat=qt.one)

        manip_leg_ind = 3
        if manip_leg_ind in self.legs.keys():
            manip_leg = self.legs[manip_leg_ind]

            manip_leg.set_angle(-np.pi / 2, 0)
            manip_leg.set_angle(np.pi, 5)
            self.sleep(0.1)

            rot = qt.from_rotation_vector(np.array([1, 0, 0]) * np.pi / 2)
            rot = qt.from_rotation_vector(np.array([0, 1, 0]) * np.pi / 2) * rot
            manip_leg.ik(xyz=[-100, 500, 700], quat=rot)

    def dragon_front_up(self):
        rot = qt.from_rotation_vector(np.array([0, 0, 0.1]))
        self.goToTargetBody(bodyQuat=rot, blocking=False)

    def dragon_front_down(self):
        rot = qt.from_rotation_vector(np.array([0, 0, -0.1]))
        self.goToTargetBody(bodyQuat=rot, blocking=False)

    def dragon_back_down(self):
        main_leg_ind = 4  # default for all moves
        main_leg = self.legs[main_leg_ind]  # default for all moves
        rot = qt.from_rotation_vector(np.array([0, 0, -0.1]))
        main_leg.move(quat=rot, blocking=False)

    def dragon_back_up(self):
        main_leg_ind = 4  # default for all moves
        main_leg = self.legs[main_leg_ind]  # default for all moves
        rot = qt.from_rotation_vector(np.array([0, 0, 0.1]))
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

    @error_catcher
    def joySUBCBK(self, msg: Joy):
        # normalizing axes to be zero
        joy_axes = [
            0.0 if abs(axis - default) < self.deadzone else round(axis, 2)
            for axis, default in zip(msg.axes, self.default_neutral_axes)
        ]
        joy_buttons = msg.buttons

        # detecting if joystick is being used
        axis_held = any(abs(axis) > self.neutral_threshold for axis in joy_axes)

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

        self.prev_axes = joy_axes
        self.prev_buttons = joy_buttons

        # self.pinfo(f"change/hold: axes {joy_axes}, buttons {joy_buttons}")

        # axes
        axis_left_x = joy_axes[1]  # vertical left
        axis_left_y = joy_axes[0]  # horizontal left
        axis_right_x = joy_axes[4]  # vertical right
        axis_right_y = joy_axes[3]  # horizontal right
        l1_held = self.check_button(joy_buttons[4])  # L1 button

        # update movement
        if l1_held and axis_held:
            if self.config_index == 0:
                # config 0
                if axis_left_x != 0:
                    x = axis_left_x * 10
                    self.current_movement["x"] = x
                if axis_left_y != 0:
                    y = axis_left_y * 10
                    self.current_movement["y"] = y
                if axis_right_x != 0:
                    z = axis_right_x * 10
                    self.current_movement["z"] = z
            elif self.config_index == 1:
                # config 1, alternate axes for test
                if axis_left_y != 0:
                    x = axis_left_y * 10
                    self.current_movement["x"] = x
                if axis_right_x != 0:
                    y = axis_right_x * 10
                    self.current_movement["y"] = y
                if axis_left_x != 0:
                    z = axis_left_x * 10
                    self.current_movement["z"] = z
            elif self.config_index == 2:
                # config 2, alternate axes for test
                if axis_right_x != 0:
                    x = axis_right_x * 10
                    self.current_movement["x"] = x
                if axis_left_x != 0:
                    y = axis_left_x * 10
                    self.current_movement["y"] = y
                if axis_left_y != 0:
                    z = axis_left_y * 10
                    self.current_movement["z"] = z

            if self.config_index == 0:
                if axis_left_x == 0:
                    self.current_movement["x"] = 0.0
                if axis_left_y == 0:
                    self.current_movement["y"] = 0.0
                if axis_right_x == 0:
                    self.current_movement["z"] = 0.0
            elif self.config_index == 1:
                if axis_left_y == 0:
                    self.current_movement["x"] = 0.0
                if axis_right_x == 0:
                    self.current_movement["y"] = 0.0
                if axis_left_x == 0:
                    self.current_movement["z"] = 0.0
            elif self.config_index == 2:
                if axis_right_x == 0:
                    self.current_movement["x"] = 0.0
                if axis_left_x == 0:
                    self.current_movement["y"] = 0.0
                if axis_left_y == 0:
                    self.current_movement["z"] = 0.0
        else:
            self.current_movement["x"] = 0.0
            self.current_movement["y"] = 0.0
            self.current_movement["z"] = 0.0

        # at config 0, zeroing
        if self.config_index == 0:
            zero = self.check_button(joy_buttons[0])  # X button
            if zero:
                for leg in self.legs.values():
                    leg.go2zero()
            else:
                self.stop_all_joints()

    # timer, for delay of the joystick input
    def move_timer_callback(self):
        x = self.current_movement.get("x", 0.0)
        y = self.current_movement.get("y", 0.0)
        z = self.current_movement.get("z", 0.0)
        if x != 0.0:
            for leg in self.legs.values():
                leg.move(xyz=[x, 0, 0], blocking=False)
        if y != 0.0:
            for leg in self.legs.values():
                leg.move(xyz=[0, y, 0], blocking=False)
        if z != 0.0:
            for leg in self.legs.values():
                leg.move(xyz=[0, 0, z], blocking=False)

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
        to_execute = mapping[input]
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
            self.chosen_leg = leg_keys
        elif isinstance(self.chosen_leg, list) or self.chosen_leg is None:
            first_leg = leg_keys[0]
            self.chosen_leg = first_leg
        else:
            next_index: int = (leg_keys.index(self.chosen_leg) + increment) % len(
                self.legs
            )
            self.chosen_leg = leg_keys[next_index]
        self.pinfo(f"Controling: leg {self.chosen_leg}")

    def cycle_mode(self):
        if self.mode_index is None:
            self.mode_index = -1
        self.mode_index = (self.mode_index + 1) % len(self.modes)
        self.sub_map = self.modes[self.mode_index]()

    def get_active_leg_keys(
        self, leg_number: Union[List[int], int, None] = None
    ) -> List[int]:
        leg_keys: List[int]
        if leg_number is None:
            if self.chosen_leg is None:
                self.cycle_leg_selection(None)
            if self.chosen_leg is None:
                return []
            if isinstance(self.chosen_leg, int):
                leg_keys = [self.chosen_leg]
            else:
                leg_keys = self.chosen_leg
        elif isinstance(leg_number, int):
            leg_keys = [leg_number]
        else:
            leg_keys = leg_number.copy()
        return leg_keys

    def set_joint_speed(
        self, speed, joint: Union[int, str, None] = None, leg_number: Optional[int] = None
    ):
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
        leg_keys = self.get_active_leg_keys(leg_number)
        for k in leg_keys:
            leg = self.legs[k]
            leg.go2zero()

    def mode_joint(self) -> InputMap:
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
        """Creates the main input map, mapping user input to functions to execute"""
        main_map: InputMap = {
            (Key.KEY_RIGHT, ANY): [lambda: self.cycle_leg_selection(1)],
            (Key.KEY_LEFT, ANY): [lambda: self.cycle_leg_selection(-1)],
            (Key.KEY_DOWN, ANY): [lambda: self.cycle_leg_selection(None)],
            (Key.KEY_M, ANY): [lambda: self.cycle_mode()],
        }
        return main_map


def main(args=None):
    myMain(KeyGaitNode, multiThreaded=True)


if __name__ == "__main__":
    main()

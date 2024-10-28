"""
This node is responsible for controlling movement of Moonbot HERO.
For now keyboard and controller (PS4).

Authors: Elian NEPPEL, Shamistan KARIMOV
Lab: SRL, Moonshot team
"""

from typing import Dict, Final, Literal, Optional, Sequence, overload
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

# LEGNUMS_TO_SCAN = range(10)
LEGNUMS_TO_SCAN = [4]

MAX_JOINT_SPEED = 0.15

JOINT_STICKER_NUMBER: Dict[int, int] = {1:1, 2:0, 3:3, 4:4, 5:5, 6:6, 7:7, 8:8, 9:2}

class KeyGaitNode(EliaNode):
    def __init__(self, name: str = "keygait_node"):
        super().__init__(name)
        self.Alias = "G"
        # self.setAndBlockForNecessaryClients("mover_alive")

        self.leg_aliveCLI: Dict[int, Client] = dict(
            [(l, self.create_client(Empty, f"leg{l}/leg_alive")) for l in LEGNUMS_TO_SCAN]
        )
        self.legs: Dict[int, Leg] = {}

        self.keySUB = self.create_subscription(Key, "keydown", self.keySUBCBK, 10)
        self.nokeySUB = self.create_subscription(Key, "keyup", self.nokeySUBCBK, 10)
        self.joySUB = self.create_subscription(
            Joy, "joy", self.joySUBCBK, 10
        )  # joystick, new
        self.leg_scanTMR = self.create_timer(
            1, self.leg_scanTMRCBK, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.next_leg_to_scan = LEGNUMS_TO_SCAN[0]
        self.selected_joint: Optional[int] = None

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
        self.current_movement = {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.move_timer = self.create_timer(
            0.3, self.move_timer_callback
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
        # self.pinfo("tic")

        l = self.next_leg_to_scan
        self.next_leg_to_scan = LEGNUMS_TO_SCAN[(l + 1) % len(LEGNUMS_TO_SCAN)]
        cli = self.leg_aliveCLI[l]
        if l in self.legs.keys():
            if l == self.next_leg_to_scan:
                return
            self.legs[l].update_joint_pub()
            self.leg_scanTMRCBK()  # continue scanning if already scanned
            return
        if cli.wait_for_service(0.01):
            self.pinfo(f"Hey there leg{l}, nice to meet you")
            self.legs[l] = Leg(l, self)
            self.leg_scanTMRCBK()  # continue scanning if leg found
            return
        if len(self.legs.keys()) < 1:
            # self.pinfo("2")
            # self.sleep(0.2)
            self.leg_scanTMRCBK()  # continue scanning if no legs
            return

        # self.pinfo("3")
        return  # stops scanning if all fails

    @error_catcher
    def nokeySUBCBK(self, msg: Key):
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
    def keySUBCBK(self, msg: Key):
        key_char = chr(msg.code)
        key_code = msg.code
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
            # L1 control
            if l1_held and axis_held:
                selected_joint_1 = JOINT_STICKER_NUMBER.get(1)
                selected_joint_2 = JOINT_STICKER_NUMBER.get(2)
                selected_joint_3 = JOINT_STICKER_NUMBER.get(9)
                selected_joint_4 = JOINT_STICKER_NUMBER.get(8)

                # joint 1 (gripper) with vertical left
                if axis_left_x != 0:
                    inc = axis_left_x * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_1, inc)
                else:
                    self.joint_control_joy(selected_joint_1, 0.0)

                # joint 2 with horizontal left
                if axis_left_y != 0:
                    inc = axis_left_y * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_2, inc)
                else:
                    self.joint_control_joy(selected_joint_2, 0.0)

                # joint 9 (gripper) with vertical right
                if axis_right_x != 0:
                    inc = axis_right_x * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_3, inc)
                else:
                    self.joint_control_joy(selected_joint_3, 0.0)

                # joint 8 with horizontal right
                if axis_right_y != 0:
                    inc = axis_right_y * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_4, inc)
                else:
                    self.joint_control_joy(selected_joint_4, 0.0)
            else:
                selected_joint_1 = JOINT_STICKER_NUMBER.get(1)
                selected_joint_2 = JOINT_STICKER_NUMBER.get(2)
                selected_joint_3 = JOINT_STICKER_NUMBER.get(9)
                selected_joint_4 = JOINT_STICKER_NUMBER.get(8)
                self.joint_control_joy(selected_joint_1, 0.0)
                self.joint_control_joy(selected_joint_2, 0.0)
                self.joint_control_joy(selected_joint_3, 0.0)
                self.joint_control_joy(selected_joint_4, 0.0)

            # R1 control
            if r1_held and axis_held:
                selected_joint_1 = JOINT_STICKER_NUMBER.get(3)
                selected_joint_2 = JOINT_STICKER_NUMBER.get(4)
                selected_joint_3 = JOINT_STICKER_NUMBER.get(7)
                selected_joint_4 = JOINT_STICKER_NUMBER.get(6)

                # joint 3 with vertical left
                if axis_left_x != 0:
                    inc = axis_left_x * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_1, inc)
                else:
                    self.joint_control_joy(selected_joint_1, 0.0)

                # joint 4 with horizontal left
                if axis_left_y != 0:
                    inc = axis_left_y * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_2, inc)
                else:
                    self.joint_control_joy(selected_joint_2, 0.0)

                # joint 7 with vertical right
                if axis_right_x != 0:
                    inc = axis_right_x * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_3, inc)
                else:
                    self.joint_control_joy(selected_joint_3, 0.0)

                # joint 6 with horizontal right
                if axis_right_y != 0:
                    inc = axis_right_y * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_4, inc)
                else:
                    self.joint_control_joy(selected_joint_4, 0.0)
            else:
                selected_joint_1 = JOINT_STICKER_NUMBER.get(3)
                selected_joint_2 = JOINT_STICKER_NUMBER.get(4)
                selected_joint_3 = JOINT_STICKER_NUMBER.get(7)
                selected_joint_4 = JOINT_STICKER_NUMBER.get(6)
                self.joint_control_joy(selected_joint_1, 0.0)
                self.joint_control_joy(selected_joint_2, 0.0)
                self.joint_control_joy(selected_joint_3, 0.0)
                self.joint_control_joy(selected_joint_4, 0.0)

            # L2 control
            if l2_held and axis_held:
                selected_joint_1 = JOINT_STICKER_NUMBER.get(5)

                # joint 5 with vertical left
                if axis_left_x != 0:
                    inc = axis_left_x * MAX_JOINT_SPEED
                    self.joint_control_joy(selected_joint_1, inc)
                else:
                    self.joint_control_joy(selected_joint_1, 0.0)
            else:
                selected_joint_1 = JOINT_STICKER_NUMBER.get(5)
                self.joint_control_joy(selected_joint_1, 0.0)
                
                
        # IK
        if self.config_index == 1:
            if l1_held and axis_held:
                if axis_left_x != 0:
                    x = axis_left_x * 10
                    self.current_movement["x"] = x
                if axis_left_y != 0:
                    y = axis_left_y * 10
                    self.current_movement["y"] = y
                if axis_right_x != 0:
                    z = axis_right_x * 10
                    self.current_movement["z"] = z
            
                if self.config_index == 1:
                    if axis_left_x == 0:
                        self.current_movement["x"] = 0.0
                    if axis_left_y == 0:
                        self.current_movement["y"] = 0.0
                    if axis_right_x == 0:
                        self.current_movement["z"] = 0.0
            else:
                self.current_movement["x"] = 0.0
                self.current_movement["y"] = 0.0
                self.current_movement["z"] = 0.0
            if r1_held and axis_held:
                if axis_left_x != 0:
                    roll = axis_left_x * 10
                    self.current_movement["roll"] = roll
                if axis_left_y != 0:
                    pitch = axis_left_y * 10
                    self.current_movement["pitch"] = pitch
                if axis_right_x != 0:
                    yaw = axis_right_x * 10
                    self.current_movement["yaw"] = yaw
            else:
                self.current_movement["roll"] = 0.0
                self.current_movement["pitch"] = 0.0
                self.current_movement["yaw"] = 0.0
        
        # at config 0, zeroing
        if self.config_index == 0:
            zero = self.check_button(joy_buttons[0])  # X button
            if zero:
                for leg in self.legs.values():
                    leg.go2zero()
            # else:
            #     self.stop_all_joints()


    def joint_control_joy(self, selected_joint, inc_value):
        # self.pinfo(selected_joint)
        for leg in self.legs.values():
            jobj = leg.get_joint_obj(selected_joint)
            if jobj is None:
                continue
            jobj.set_speed(inc_value)

        
    # timer, for delay of the joystick input
    def move_timer_callback(self):
        if self.config_index == 1:
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
        return button == 1


def main(args=None):
    myMain(KeyGaitNode, multiThreaded=True)


if __name__ == "__main__":
    main()

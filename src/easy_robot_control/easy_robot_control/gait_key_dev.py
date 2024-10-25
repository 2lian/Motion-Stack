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

LEGNUMS_TO_SCAN = range(10)
# LEGNUMS_TO_SCAN = [4]


class KeyGaitNode(EliaNode):
    def __init__(self, name: str = "keygait_node"):
        super().__init__(name)
        self.Alias = "G"
        # self.setAndBlockForNecessaryClients("mover_alive")

        self.leg_aliveCLI: Dict[int, Client] = dict(
            [
                (l, self.create_client(Empty, f"leg{l}/leg_alive"))
                for l in LEGNUMS_TO_SCAN
            ]
        )
        self.legs: Dict[int, Leg] = {}

        self.keySUB = self.create_subscription(Key, "keydown", self.keySUBCBK, 10)
        self.nokeySUB = self.create_subscription(Key, "keyup", self.nokeySUBCBK, 10)
        self.joySUB = self.create_subscription(Joy, "joy", self.joySUBCBK, 10)  # joystick, new
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
        self.current_movement = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.move_timer = self.create_timer(0.2, self.move_timer_callback)  # 0.1 sec delay

    @error_catcher
    def leg_scanTMRCBK(self):
        # self.pinfo("tic")

        l = self.next_leg_to_scan
        self.next_leg_to_scan = LEGNUMS_TO_SCAN[(l + 1) % len(LEGNUMS_TO_SCAN)]
        # self.pwarn(self.leg_aliveCLI)
        cli = self.leg_aliveCLI[l]
        if l in self.legs.keys():
            # self.pinfo("1")
            if l == self.next_leg_to_scan:
                return
            self.leg_scanTMRCBK()  # continue scanning if already scanned
            return
        if cli.wait_for_service(0.01):
            self.pinfo(f"Hey there leg{l}, nice to meet you")
            self.legs[l] = Leg(l, self)
            self.leg_scanTMRCBK()  # continue scanning if leg found
            return
        if len(self.legs.keys()) < 1:
            # self.pinfo("2")
            self.sleep(0.2)
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
            self.vehicle_default()  

        if self.selected_joint is not None:
            self.joint_control_key(key_char)
            return

        # if statement hell, yes bad, if you unhappy fix it
        if key_char == "w":
            for leg in self.legs.values():
                leg.move(xyz=[20, 0, 0], blocking=False)
        elif key_char == "s":
            for leg in self.legs.values():
                leg.move(xyz=[-20, 0, 0], blocking=False)

    def vehicle_default(self):
        angs = {
            0: np.pi/2,
            3: 0.0,
            4: 0.0,
            5: np.pi/2,
            6: 0.0,
            7: 0.0,
            8: np.pi/2,
        }
        for leg in self.legs.values():
        # for leg in [self.legs[4]]:
            for num, ang in angs.items():
                jobj = leg.get_joint_obj(num)
                if jobj is None:
                    continue
                jobj.set_angle(ang)

    def joint_control_key(self, key_char):
        if self.selected_joint is None:
            return
        if key_char == "w":
            inc = 1.0
        elif key_char == "s":
            inc = -1.0
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
        is_held = any(abs(axis) > self.neutral_threshold for axis in joy_axes)

        # comparison with previous state
        axes_changed = (
            self.prev_axes is None or
            any(abs(current - previous) > self.deadzone
                for current, previous in zip(joy_axes, self.prev_axes))
        )
        buttons_changed = self.prev_buttons is None or joy_buttons != self.prev_buttons

        # check if a change or active hold state is detected
        if not axes_changed and not buttons_changed and not is_held:
            return

        # update previous state
        self.prev_axes = joy_axes
        self.prev_buttons = joy_buttons

        self.pinfo(f"change/hold: axes {joy_axes}, buttons {joy_buttons}")

        axis_left_x = joy_axes[1]  # vertical left
        axis_left_y = joy_axes[0]  # horizontal left
        axis_right_x = joy_axes[4]  # vertical right
        axis_right_y = joy_axes[3]  # horizontal right
        l1_pressed = self.check_button(joy_buttons[4])  # L1 button
        zero = self.check_button(joy_buttons[9])  # options button

        # update current movement based on joystick input
        if l1_pressed and is_held:
            if axis_left_x != 0:
                x = axis_left_x * 10
                self.current_movement['x'] = x
                # self.pinfo(f"movement x: {x}")
            if axis_left_y != 0:
                y = axis_left_y * 10
                self.current_movement['y'] = y
            if axis_right_x != 0:
                z = axis_right_x * 10
                self.current_movement['z'] = z
            # reset to 0.0 if no movement
            if axis_left_x == 0:
                self.current_movement['x'] = 0.0
            if axis_left_y == 0:
                self.current_movement['y'] = 0.0
            if axis_right_x == 0:
                self.current_movement['z'] = 0.0
        else:
            self.current_movement['x'] = 0.0
            self.current_movement['y'] = 0.0
            self.current_movement['z'] = 0.0

        # go to zero position
        if zero:
            for leg in self.legs.values():
                leg.go2zero()

    # timer, for delay of the joystick input
    def move_timer_callback(self):
        x = self.current_movement.get('x', 0.0)
        y = self.current_movement.get('y', 0.0)
        z = self.current_movement.get('z', 0.0)
        if x != 0.0:
            for leg in self.legs.values():
                leg.move(xyz=[x, 0, 0], blocking=False)
        if y != 0.0:
            for leg in self.legs.values():
                leg.move(xyz=[0, y, 0], blocking=False)
        if z != 0.0:
            for leg in self.legs.values():
                leg.move(xyz=[0, 0, z], blocking=False)

    # check is button is pressed
    def check_button(self, button: int):
        return button == 1


def main(args=None):
    myMain(KeyGaitNode, multiThreaded=True)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import re
import threading
import time
from collections import deque
from os import environ
from typing import Final, List, Optional, Set, Tuple

import numpy as np
import quaternion as qt
import rclpy
from keyboard_msgs.msg import Key
from rclpy.duration import Duration
from sensor_msgs.msg import Joy

from motion_stack.api.ros2.ik_api import IkHandler, IkSyncerRos
from motion_stack.api.ros2.joint_api import JointHandler, JointSyncerRos
from motion_stack.core.utils.math import patch_numpy_display_light
from motion_stack.core.utils.pose import Pose
from motion_stack.ros2.utils.conversion import ros_now
from motion_stack.ros2.utils.executor import error_catcher

from .operator_utils import (
    BUTT_INTS,
    ButtonName,
    InputMap,
    JoyBits,
    JoyState,
    any_pressed,
    bits2name,
    connect_mapping,
    msg_to_JoyBits,
    rel_to_base_link,
)

patch_numpy_display_light()

ALIAS = "operator_node"
TRANSLATION_SPEED = 80  # mm/s ; full stick will send this speed
ROTATION_SPEED = np.deg2rad(5)  # rad/s ; full stick will send this angular speed

# type def V
ANY: Final[str] = "ANY"
ALWAYS: Final[str] = "ALWAYS"

# Namespace
operator = str(environ.get("OPERATOR"))
INPUT_NAMESPACE = f"/{operator}"


class OperatorNode(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__(ALIAS)

        # keep track of discovered legs
        self.current_legs: Set[int] = set()
        self.joint_handlers: List[JointHandler] = []
        self.joint_syncer: Optional[JointSyncerRos] = None
        self.joints_prev = []
        self.ik_handlers: List[IkHandler] = []
        self.ik_syncer: Optional[IkSyncerRos] = None
        self.ik_legs_prev = []
        self.wheel_handlers: List[JointHandler] = []
        self.wheel_syncer: Optional[JointSyncerRos] = None
        self.wheels_prev = []

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
        self.joint_speed = 0.15
        self.wheel_speed = 0.2

        self.ik2TMR = self.create_timer(0.1, self.move_ik)
        self.ik2TMR.cancel()

        self.log_messages: deque[str] = deque(maxlen=6)

        # JoyStick
        self.prev_joy_state = JoyState()
        self.joy_state = JoyState()

        self.ee_mode = False

    @error_catcher
    def _discover_legs(self):
        svc_list = self.get_service_names_and_types()
        found: Set[int] = set()
        for name, types in svc_list:
            if name.endswith("/joint_alive") and "std_srvs/srv/Empty" in types:
                m = re.match(r"^/leg(\d+)/joint_alive$", name)
                if m:
                    found.add(int(m.group(1)))

        new_legs = found - self.current_legs
        self.current_legs = found

        if not new_legs:
            return

        self.add_log("I", f"New legs discovered: {sorted(new_legs)}")

        # recreate handlers
        self.joint_handlers = [JointHandler(self, l) for l in sorted(self.current_legs)]
        self.ik_handlers = [IkHandler(self, l) for l in sorted(self.current_legs)]

        # callbacks
        for jh in self.joint_handlers:
            jh.ready.add_done_callback(self._on_joint_handler_ready)
            jh.ready.add_done_callback(self._on_wheel_handler_ready)

        for ih in self.ik_handlers:
            ih.ready.add_done_callback(self._on_ik_handler_ready)

    def _on_joint_handler_ready(self, future):

        ready_jhs = [jh for jh in self.joint_handlers if jh.ready.done()]
        if not ready_jhs:
            return

        if self.joint_syncer is not None:
            self.joint_syncer.clear()
            self.joint_syncer.last_future.cancel()

        self.joint_syncer = JointSyncerRos(
            ready_jhs, interpolation_delta=np.deg2rad(20)
        )
        legs = [jh.limb_number for jh in ready_jhs]
        # self.add_log("I", f"Joint syncer built for legs: {legs}")

    def _on_ik_handler_ready(self, future):
        if self.ik_syncer is not None:
            self.ik_syncer.last_future.cancel()

        ready_ihs = [ih for ih in self.ik_handlers if ih.ready.done()]
        if not ready_ihs:
            return
        self.ik_syncer = IkSyncerRos(ready_ihs)
        legs = [ih.limb_number for ih in ready_ihs]
        self.add_log("I", f"IK syncer built for legs: {legs}")

    def _on_wheel_handler_ready(self, future):
        if self.wheel_syncer is not None:
            self.wheel_syncer.last_future.cancel()

        ready_jhs = [jh for jh in self.joint_handlers if jh.ready.done()]
        if not ready_jhs:
            return

        self.wheel_syncer = JointSyncerRos(
            ready_jhs,
            interpolation_delta=np.deg2rad(30),
        )
        legs = [jh.limb_number for jh in ready_jhs]
        # self.add_log("I", f"Wheel syncer built for legs: {legs}")

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
            (Key.KEY_I, ANY): [self.enter_ik_mode],
            ("o", ANY): [self.enter_ik_mode],
            ("x", ANY): [self.enter_joint_mode],
            ("s", ANY): [self.enter_leg_mode],
            ("t", ANY): [self.enter_wheel_mode],
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

        if self.joint_syncer is not None:
            self.joint_syncer.clear()

        submap: InputMap = {
            (Key.KEY_W, ANY): [lambda: self.move_joints(self.joint_speed)],
            (Key.KEY_S, ANY): [lambda: self.move_joints(-self.joint_speed)],
            (Key.KEY_O, ANY): [lambda: self.move_wheels(self.wheel_speed)],
            (Key.KEY_L, ANY): [lambda: self.move_wheels(-self.wheel_speed)],
            (Key.KEY_P, ANY): [lambda: self.move_wheels(0.0)],
            (Key.KEY_0, ANY): [self.move_zero],
        }

        self.sub_map = submap

    def enter_wheel_mode(self):
        self.current_mode = "wheel_select"
        self.no_no_leg()

        if self.wheel_syncer is not None:
            self.wheel_syncer.clear()

        self.sub_map = {
            (Key.KEY_W, ANY): [lambda: self.move_joints(self.joint_speed)],
            (Key.KEY_S, ANY): [lambda: self.move_joints(-self.joint_speed)],
            (Key.KEY_O, ANY): [lambda: self.move_wheels(self.wheel_speed)],
            (Key.KEY_L, ANY): [lambda: self.move_wheels(-self.wheel_speed)],
            (Key.KEY_P, ANY): [lambda: self.move_wheels(0.0)],
        }

    def enter_ik_mode(self):
        self.current_mode = "ik_select"
        self.no_no_leg()

        if self.ik_syncer is not None:
            self.ik_syncer.clear()

        self.sub_map = {
            ("stickL", ANY): [self.start_ik2_timer],
            ("stickR", ANY): [self.start_ik2_timer],
            ("R2", ANY): [self.start_ik2_timer],
            ("L2", ANY): [self.start_ik2_timer],
            ("R1", ANY): [self.start_ik2_timer],
            ("L1", ANY): [self.start_ik2_timer],
            ("L1", ANY): [self.start_ik2_timer],
            ("x", ANY): [lambda: self.switch_ik_mode(False)],
            ("o", ANY): [lambda: self.switch_ik_mode(True)],
        }

    def move_ik(self):
        if self.ik_syncer is None:
            return

        bits = self.joy_state.bits
        sticks_active = any_pressed(
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

        ik_by_leg = {ih.limb_number: ih for ih in self.ik_handlers}
        ik_ready_legs = []
        for leg in self.selected_legs:
            ih = ik_by_leg.get(leg)
            if ih and ih.ready.done():
                ik_ready_legs.append(leg)

        if ik_ready_legs != self.ik_legs_prev:
            self.ik_syncer.clear()

        target_pose_stick = Pose(ros_now(self), xyz_input * delta_xyz, rot)
        target_stick = {leg: target_pose_stick for leg in ik_ready_legs}
        target_rel_to_ee = self.ik_syncer.abs_from_rel(target_stick)
        target_rel_to_base = rel_to_base_link(self.ik_syncer, target_stick)

        if self.ee_mode is True:
            target = target_rel_to_ee
        else:
            target = target_rel_to_base

        self.add_log("I", f"{target_rel_to_ee}")
        self.ik_syncer.lerp_toward(target)

        self.ik_legs_prev = ik_ready_legs

    def switch_ik_mode(self, val: Optional[bool] = None):
        if val is None:
            self.ee_mode = not self.ee_mode
        else:
            self.ee_mode = val
        self.add_log("I", f"IK mode relative to end effector: {self.ee_mode}")

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

        self.add_log("I", "Sending the selected joints to zero position.")
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

        if (selected_jnames + selected_jnames_inv) != self.joints_prev:
            self.joint_syncer.clear()

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

        self.joints_prev = selected_jnames + selected_jnames_inv

    def move_wheels(self, v: float, omega: float = 0.0):
        """
        Args:
            v (float): Linear speed command. Positive drives forward, negative back.
        """
        if self.wheel_syncer is None:
            return

        if v == 0.0 and omega == 0.0:
            self.wheel_syncer.clear()
            self.wheel_syncer.last_future.cancel()
            return

        wheel_jnames = sorted(jn for (_, jn) in self.selected_wheel_joints)
        wheel_jnames_inv = sorted(jn for (_, jn) in self.selected_wheel_joints_inv)
        if not wheel_jnames:
            return

        if (wheel_jnames + wheel_jnames_inv) != self.wheels_prev:
            self.wheel_syncer.clear()

        target = {jname: (v + omega) for jname in wheel_jnames}
        target.update({jn: (-v + omega) for jn in wheel_jnames_inv})

        # self.add_log("I", f"{target}")

        start_time = ros_now(self)

        def delta_time():
            nonlocal start_time
            now = ros_now(self)
            out = (now - start_time).sec()
            start_time = now
            return out

        self.wheel_syncer.speed_safe(target, delta_time)

        self.wheels_prev = wheel_jnames + wheel_jnames_inv

    def create_main_map(self) -> InputMap:
        return {
            (Key.KEY_RETURN, ANY): [self.recover],
            (Key.KEY_RETURN, Key.MODIFIER_LSHIFT): [self.recover_all],
            (Key.KEY_SPACE, ANY): [self.halt],
            (Key.KEY_SPACE, Key.MODIFIER_LSHIFT): [self.halt_all],
            (Key.KEY_ESCAPE, ANY): [self.enter_main_menu],
            ("option", ANY): [self.enter_main_menu],
            ("PS", ANY): [self.halt_all],
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
        connect_mapping(self.main_map, (code, mod))
        connect_mapping(self.sub_map, (code, mod))

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

    # ───────────────────────────── JoyStick ─────────────────────────────

    def joy_pressed(self, button_name: ButtonName):
        """Executes for each button that is pressed. Like a callback.

        Args:
            bits: Should only have one single bit set to 1, for 1 single button
        """
        dic_key = (button_name, self.joy_state.bits)
        connect_mapping(self.main_map, dic_key)
        connect_mapping(self.sub_map, dic_key)

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
        self.joy_state = msg_to_JoyBits(msg)

        button_downed: JoyBits = ~self.prev_joy_state.bits & self.joy_state.bits
        downed_names: List[ButtonName] = bits2name(button_downed)
        for name in downed_names:
            self.joy_pressed(name)

        button_upped = self.prev_joy_state.bits & ~self.joy_state.bits
        upped_names: List[ButtonName] = bits2name(button_upped)
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

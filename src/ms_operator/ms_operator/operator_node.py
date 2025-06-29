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
from motion_stack.api.ros2.ik_api import IkHandler, IkSyncerRos
from motion_stack.api.ros2.joint_api import JointHandler, JointSyncerRos
from motion_stack.core.utils.math import patch_numpy_display_light
from motion_stack.core.utils.pose import Flo3, Pose, VelPose, XyzQuat
from motion_stack.ros2.utils.conversion import ros_now
from motion_stack.ros2.utils.executor import error_catcher
from rclpy.duration import Duration
from sensor_msgs.msg import Joy

from .operator_utils import (
    BUTT_INTS,
    ButtonName,
    InputMap,
    JoyBits,
    JoyState,
    any_pressed,
    bits2name,
    connect_mapping,
    delta_time,
    msg_to_JoyBits,
    rel_to_base_link,
    rel_vel_to_base_link,
)

patch_numpy_display_light()

ALIAS = "operator_node"

# type def V
ANY: Final[str] = "ANY"
ALWAYS: Final[str] = "ALWAYS"
LimbNumber = int
JointName = str


class OperatorNode(rclpy.node.Node):
    def __init__(self) -> None:
        """
        Construct and configure the OperatorNode, which provides a text-based
        UI for interactively driving legs, joints, wheels and IK.

        What this does:
        1. Internal data structures
           - `current_legs`: set of discovered leg IDs (initially empty).
           - `joint_handlers`, `ik_handlers`, `wheel_handlers`: lists of handler objects
             created once legs are discovered.
           - `joint_syncer`, `ik_syncer`, `wheel_syncer`: syncers that will
             translate high-level commands into ROS2 messages.
           - `selected_legs`: list of leg IDs the user has chosen to control.
           - `selected_joints`, `selected_joints_inv`, `selected_wheel_joints`,
             `selected_wheel_joints_inv`: sets tracking which individual joints
             (and inverted joints) are active.
           - `joint_speed`, `wheel_speed`: default speed settings applied
             when driving joints or wheels.
           - Logging deque (`log_messages`) to keep the last few status entries.
           - Joystick state buffers (`prev_joy_state`, `joy_state`) for edge detection.
        2. ROS2 timers
           - A 1 Hz timer bound to `_discover_legs`: scans for new `/legN/joint_alive`
             services.
           - A ~30 Hz timer bound to `loop()`: calls any active syncers’ `execute()`
             method to send out drive commands.
        3. Subscriptions
           - `/keydown` and `/keyup` on the configured input namespace, so key presses
             drive the menus and motion functions.
           - `/joy` topic to drive IK with an actual gamepad, diff-detecting button
             presses/releases and analog sticks.
        4. Input mappings
           - Builds `main_map` of global keys (e.g. `<Esc>` returns to main menu).
           - Prepares `sub_map` placeholder, then immediately calls `enter_main_menu()`
             to switch into the top-level menu and install those sub-bindings.
        5. TUI integration
           - After returning from `__init__`, `main()` will spin ROS in a worker
             thread, then hand `self` off to `urwid_main(self)` to drive the text UI.

        In short, all the general Motion-Stack specific things for leg discovery,
        handler/syncer wiring, key/joystick input, and a dynamic interface is set up here;
        you only need to plug in your robot-specific functions or overload the existing ones.
        """
        super().__init__(ALIAS)

        # Legs and their handlers and syncers
        self.current_legs: Set[int] = set()
        self.joint_handlers: List[JointHandler] = []
        self.joint_syncer: Optional[JointSyncerRos] = None
        self.ik_handlers: List[IkHandler] = []
        self.ik_syncer: Optional[IkSyncerRos] = None
        self.wheel_handlers: List[JointHandler] = []
        self.wheel_syncer: Optional[JointSyncerRos] = None

        self.key_downSUB = self.create_subscription(
            Key, f"keydown", self.key_downSUBCBK, 10
        )
        self.key_upSUB = self.create_subscription(Key, f"keyup", self.key_upSUBCBK, 10)
        self.joySUB = self.create_subscription(Joy, f"joy", self.joySUBCBK, 10)

        self.main_map: Final[InputMap] = self.create_main_map()
        self.sub_map: InputMap
        self.enter_main_menu()

        # select legs and joints (normal, inverted)
        self.selected_legs: List[LimbNumber] = []
        self.selected_ik_legs: List[LimbNumber] = []
        self.selected_joints: Set[Tuple[LimbNumber, JointName]] = set()
        self.selected_joints_inv: Set[Tuple[LimbNumber, JointName]] = set()
        self.selected_wheel_joints: Set[Tuple[LimbNumber, JointName]] = set()
        self.selected_wheel_joints_inv: Set[Tuple[LimbNumber, JointName]] = set()
        self.joints_prev = []
        self.ik_legs_prev = []
        self.wheels_prev = []
        self.leg_buffer: str = ""
        self.joint_leg_buffer: str = ""
        self.ik_leg_buffer: str = ""
        self.active_joint_leg: Optional[LimbNumber] = None

        self.declare_parameter("joint_speed", 0.15)
        self.declare_parameter("wheel_speed", 0.2)
        self.declare_parameter("translation_speed", 70.0)
        self.declare_parameter("rotation_speed", np.deg2rad(7))

        self.joint_speed = self.get_parameter("joint_speed").value
        self.wheel_speed = self.get_parameter("wheel_speed").value
        self.translation_speed = self.get_parameter("translation_speed").value
        self.rotation_speed = self.get_parameter("rotation_speed").value

        # self.create_timer(5.0, self._discover_legs)

        self.execTMR = self.create_timer(1 / 30.0, self.loop)

        self.ikTMR = self.create_timer(0.1, self.move_ik)
        self.ikTMR.cancel()

        self.log_messages: deque[str] = deque(maxlen=3)

        self.prev_joy_state = JoyState()
        self.joy_state = JoyState()

        self.ee_mode = True

        self.ik_key_states = {
            Key.KEY_UP: False,
            Key.KEY_DOWN: False,
            Key.KEY_LEFT: False,
            Key.KEY_I: False,
            Key.KEY_K: False,
            Key.KEY_RIGHT: False,
            Key.KEY_W: False,
            Key.KEY_S: False,
            Key.KEY_A: False,
            Key.KEY_D: False,
            Key.KEY_Q: False,
            Key.KEY_E: False,
        }

        self._discover_legs()

    @error_catcher
    def _discover_legs(self):
        """
        Every second: scan ROS2 services for /legN/joint_alive, detect newly appeared legs,
        rebuild JointHandler and IkHandler lists, and hook their `.ready` futures so that
        when their first messages arrive we’ll build the syncers. If legs are removed,
        clear the related selected_* states.
        """
        # 1) Remember the previous set of legs, so we can detect removals:
        prev_legs = set(self.current_legs)

        # 2) Scan ROS‐service list to find current legs again:
        svc_list = self.get_service_names_and_types()
        found: Set[int] = set()
        for name, types in svc_list:
            if name.endswith("/joint_alive") and "std_srvs/srv/Empty" in types:
                m = re.match(r"^/leg(\d+)/joint_alive$", name)
                if m:
                    found.add(int(m.group(1)))

        # 3) Figure out which legs are brand‐new vs which have vanished:
        new_legs = found - prev_legs
        removed_legs = prev_legs - found

        # 4) Update our stored current_legs set:
        self.current_legs = found

        # 5) If any legs went away, scrub them out of all “selected_…” sets:
        if removed_legs:
            # Remove from selected_legs list:
            self.selected_legs = [
                l for l in self.selected_legs if l not in removed_legs
            ]

            self.selected_ik_legs = [
                l for l in self.selected_ik_legs if l not in removed_legs
            ]

            # Drop any (leg, joint_name) tuples in selected_joints / inv that belonged to removed legs:
            self.selected_joints = {
                (leg, jn)
                for (leg, jn) in self.selected_joints
                if leg not in removed_legs
            }
            self.selected_joints_inv = {
                (leg, jn)
                for (leg, jn) in self.selected_joints_inv
                if leg not in removed_legs
            }

            # Likewise for the wheel‐joint selections:
            self.selected_wheel_joints = {
                (leg, jn)
                for (leg, jn) in self.selected_wheel_joints
                if leg not in removed_legs
            }
            self.selected_wheel_joints_inv = {
                (leg, jn)
                for (leg, jn) in self.selected_wheel_joints_inv
                if leg not in removed_legs
            }

            self.add_log(
                "I", f"Removed legs {sorted(removed_legs)} → cleared their selections"
            )

        # 6) If any brand‐new legs appeared, create new Joint/IK handlers for all current legs:
        if new_legs:
            self.add_log("I", f"New legs discovered: {sorted(new_legs)}")

            self.joint_handlers = [
                JointHandler(self, l) for l in sorted(self.current_legs)
            ]
            self.ik_handlers = [IkHandler(self, l) for l in sorted(self.current_legs)]

    def _rebuild_joint_syncer(self, _):
        """
        Callback to (re)build the JointSyncerRos whenever any JointHandler
        becomes ready or whenever the selected legs change.
        Gathers all ready JointHandler for your selected legs, tears down
        the old syncer (if present), and creates a fresh JointSyncerRos.
        """
        ready = [
            jh
            for jh in self.joint_handlers
            if jh.ready.done() and jh.limb_number in self.selected_legs
        ]
        if not ready:
            return
        # clear old if it exists
        if self.joint_syncer is not None:
            self.joint_syncer.clear()
            self.joint_syncer.last_future.cancel()
        self.joint_syncer = JointSyncerRos(ready, interpolation_delta=np.deg2rad(20))

    def _rebuild_wheel_syncer(self, _):
        """
        Callback to (re)build the Wheel (Joint)SyncerRos whenever any JointHandler
        becomes ready or whenever the selected legs change.
        Operates identically to _rebuild_joint_syncer but uses a different
        interpolation_delta tuned for wheels.
        """
        ready = [
            jh
            for jh in self.joint_handlers
            if jh.ready.done() and jh.limb_number in self.selected_legs
        ]
        if not ready:
            return
        if self.wheel_syncer is not None:
            self.wheel_syncer.clear()
            self.wheel_syncer.last_future.cancel()
        self.wheel_syncer = JointSyncerRos(ready, interpolation_delta=np.deg2rad(30))

    def _rebuild_ik_syncer(self, _):
        """
        Callback to (re)build the IkSyncerRos whenever any IkHandler
        becomes ready or whenever the selected legs change.
        Cancels the old syncer (if present) and creates a new one, passing
        in the on_target_delta tuning for smooth IK interpolation.
        """
        ready = [
            ih
            for ih in self.ik_handlers
            if ih.ready.done() and ih.limb_number in self.selected_legs
        ]
        if not ready:
            return
        if self.ik_syncer is not None:
            self.ik_syncer.clear()
            self.ik_syncer.last_future.cancel()
        self.ik_syncer = IkSyncerRos(
            ready, on_target_delta=XyzQuat(40, np.deg2rad(0.001))
        )

    def select_leg(self, leg_inds: Optional[List[LimbNumber]]):
        """
        Pick which legs to control (None ⇒ all). Filters out legs that don’t exist
        and logs warnings for any missing.
        """
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

        # Remove any joint‐selections whose leg is not in selected_legs
        self.selected_joints = {
            (leg, jn) for (leg, jn) in self.selected_joints if leg in self.selected_legs
        }
        self.selected_joints_inv = {
            (leg, jn)
            for (leg, jn) in self.selected_joints_inv
            if leg in self.selected_legs
        }

        # Same for wheel‐joint selections
        self.selected_wheel_joints = {
            (leg, jn)
            for (leg, jn) in self.selected_wheel_joints
            if leg in self.selected_legs
        }
        self.selected_wheel_joints_inv = {
            (leg, jn)
            for (leg, jn) in self.selected_wheel_joints_inv
            if leg in self.selected_legs
        }

        for jh in self.joint_handlers:
            jh.ready.add_done_callback(self._rebuild_joint_syncer)
            jh.ready.add_done_callback(self._rebuild_wheel_syncer)

        for ih in self.ik_handlers:
            ih.ready.add_done_callback(self._rebuild_ik_syncer)

    def no_no_leg(self):
        """
        Ensure at least one leg is selected; if none are, auto-select all discovered legs.
        """
        if self.selected_legs is None:
            self.select_leg(None)
        if not self.selected_legs:
            self.select_leg(None)

    def enter_main_menu(self):
        """
        Switch the TUI into 'main' mode and install top-level key bindings
        for entering each sub-menu.
        """
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
        """
        Switch into 'leg_select' mode; map number keys (including multi-digit entry),
        Backspace, Enter, and 'L'/Down arrow to picking legs.
        """
        self._discover_legs()
        self.current_mode = "leg_select"
        self.leg_buffer = ""

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
            submap.setdefault((keyb, ANY), []).append(lambda n=n: self.select_leg([n]))

        digit_keys = [
            (Key.KEY_0, "0"),
            (Key.KEY_1, "1"),
            (Key.KEY_2, "2"),
            (Key.KEY_3, "3"),
            (Key.KEY_4, "4"),
            (Key.KEY_5, "5"),
            (Key.KEY_6, "6"),
            (Key.KEY_7, "7"),
            (Key.KEY_8, "8"),
            (Key.KEY_9, "9"),
        ]
        for keyb, digit in digit_keys:
            submap.setdefault((keyb, ANY), []).append(
                lambda digit=digit: self._append_leg_digit(digit)
            )

        submap.setdefault((Key.KEY_BACKSPACE, ANY), []).append(
            self._backspace_leg_digit
        )

        submap.setdefault((Key.KEY_RETURN, ANY), []).append(self._confirm_leg_buffer)

        self.sub_map = submap

    def enter_joint_mode(self):
        """
        Switch into 'joint_select' mode; selection of joints for Joint Syncer.
        Map W/S for joint speed, O/L/P for wheel commands, and 0 to zero selected joints.
        Clears the Joint Syncer if it exists.
        """
        self._discover_legs()
        self.current_mode = "joint_select"
        self.no_no_leg()

        self.joint_leg_buffer = ""
        self.active_joint_leg = None

        if self.joint_syncer is not None:
            self.joint_syncer.clear()

        submap: InputMap = {
            (Key.KEY_W, ANY): [lambda: self.move_joints(self.joint_speed)],
            (Key.KEY_S, ANY): [lambda: self.move_joints(-self.joint_speed)],
            (Key.KEY_O, ANY): [lambda: self.move_wheels(self.wheel_speed)],
            (Key.KEY_L, ANY): [lambda: self.move_wheels(-self.wheel_speed)],
            (Key.KEY_P, ANY): [lambda: self.move_wheels(0.0)],
            (Key.KEY_Z, ANY): [self.move_zero],
            (Key.KEY_C, ANY): [
                self.selected_joints.clear,
                self.selected_joints_inv.clear,
            ],
        }

        for keyb, digit in [
            (Key.KEY_0, "0"),
            (Key.KEY_1, "1"),
            (Key.KEY_2, "2"),
            (Key.KEY_3, "3"),
            (Key.KEY_4, "4"),
            (Key.KEY_5, "5"),
            (Key.KEY_6, "6"),
            (Key.KEY_7, "7"),
            (Key.KEY_8, "8"),
            (Key.KEY_9, "9"),
        ]:
            submap.setdefault((keyb, ANY), []).append(
                lambda d=digit: self._append_joint_leg_digit(d)
            )

        # backspace/enter for that buffer
        submap.setdefault((Key.KEY_BACKSPACE, ANY), []).append(
            self._backspace_joint_leg_digit
        )
        submap.setdefault((Key.KEY_RETURN, ANY), []).append(
            self._confirm_joint_leg_buffer
        )

        # once a leg is active, 1–9 toggle that leg’s joint #1–#9
        for idx, keyb in [
            (1, Key.KEY_1),
            (2, Key.KEY_2),
            (3, Key.KEY_3),
            (4, Key.KEY_4),
            (5, Key.KEY_5),
            (6, Key.KEY_6),
            (7, Key.KEY_7),
            (8, Key.KEY_8),
            (9, Key.KEY_9),
        ]:
            submap.setdefault((keyb, ANY), []).append(
                lambda i=idx: self._toggle_joint_for_active_leg(i)
            )

        self.sub_map = submap

    def enter_wheel_mode(self):
        """
        Switch into 'wheel_select' mode; similar to joint mode but selected joints
        are controlled by the Wheel Syncer.
        Clears the Wheel Syncer if it exists.
        """
        self._discover_legs()
        self.current_mode = "wheel_select"
        self.no_no_leg()

        if self.wheel_syncer is not None:
            self.wheel_syncer.clear()

        submap: InputMap = {
            (Key.KEY_W, ANY): [lambda: self.move_joints(self.joint_speed)],
            (Key.KEY_S, ANY): [lambda: self.move_joints(-self.joint_speed)],
            (Key.KEY_O, ANY): [lambda: self.move_wheels(self.wheel_speed)],
            (Key.KEY_L, ANY): [lambda: self.move_wheels(-self.wheel_speed)],
            (Key.KEY_P, ANY): [lambda: self.move_wheels(0.0)],
            (Key.KEY_C, ANY): [
                self.selected_wheel_joints.clear,
                self.selected_wheel_joints_inv.clear,
            ],
        }

        self.sub_map = submap

    def enter_ik_mode(self):
        """
        Switch into 'ik_select' mode; map joystick axes/buttons to starting
        the IK timer and toggling end‐effector vs base‐link reference.
        Clears the IK Syncer if it exists.
        """
        self._discover_legs()
        self.current_mode = "ik_select"
        self.no_no_leg()

        self.ik_leg_buffer = ""

        if self.ik_syncer is not None:
            self.ik_syncer.clear()

        submap = {
            ("stickL", ANY): [self.start_ik_timer],
            ("stickR", ANY): [self.start_ik_timer],
            ("R2", ANY): [self.start_ik_timer],
            ("L2", ANY): [self.start_ik_timer],
            ("R1", ANY): [self.start_ik_timer],
            ("L1", ANY): [self.start_ik_timer],
            ("L1", ANY): [self.start_ik_timer],
            ("x", ANY): [lambda: self.switch_ik_mode(False)],
            ("o", ANY): [lambda: self.switch_ik_mode(True)],
            (Key.KEY_O, ANY): [lambda: self.move_wheels(self.wheel_speed)],
            (Key.KEY_L, ANY): [lambda: self.move_wheels(-self.wheel_speed)],
            (Key.KEY_P, ANY): [lambda: self.move_wheels(0.0)],
            (Key.KEY_M, ANY): [self.switch_ik_mode],
            (Key.KEY_C, ANY): [self.selected_ik_legs.clear],
        }

        digit_keys = [
            (Key.KEY_0, "0"),
            (Key.KEY_1, "1"),
            (Key.KEY_2, "2"),
            (Key.KEY_3, "3"),
            (Key.KEY_4, "4"),
            (Key.KEY_5, "5"),
            (Key.KEY_6, "6"),
            (Key.KEY_7, "7"),
            (Key.KEY_8, "8"),
            (Key.KEY_9, "9"),
        ]
        for keyb, digit in digit_keys:
            submap.setdefault((keyb, ANY), []).append(
                lambda digit=digit: self._append_ik_leg_digit(digit)
            )

        submap.setdefault((Key.KEY_BACKSPACE, ANY), []).append(
            self._backspace_ik_leg_digit
        )

        submap.setdefault((Key.KEY_RETURN, ANY), []).append(self._confirm_ik_leg_buffer)

        self.sub_map = submap

    def move_joints(self, speed: float):
        """
        If any joints are selected, send constant-speed commands to the
        JointSyncerRos using 'speed_safe', flipping sign for inverted joints.
        Clears the Joint Syncer if the selected joints are changed.
        """
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
        Drive the wheel syncer: linear velocity v plus optional omega for
        differential steering; cancel when both are zero.
        Clears the Wheel Syncer if the selected joints are changed.
        """
        if self.wheel_syncer is None:
            return

        if v == 0.0 and omega == 0.0:
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

    def move_ik(self):
        """
        Fired by a 0.1 s timer when any relevant joystick input is active:
        compute Δ-pose from sticks/triggers, assemble per-leg targets
        (relative or absolute), and send to the IK syncer. Stops when sticks go idle.
        Clears the IK Syncer if the selected legs are changed.
        """
        if self.ik_syncer is None:
            return

        ik_by_leg = {ih.limb_number: ih for ih in self.ik_handlers}
        ik_ready_legs = [
            leg
            for leg in self.selected_ik_legs
            if (ih := ik_by_leg.get(leg)) is not None and ih.ready.done()
        ]

        if not ik_ready_legs:
            return

        if ik_ready_legs != self.ik_legs_prev:
            self.ik_syncer.clear()

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
            self.ikTMR.cancel()
            return

        speed_xyz = self.translation_speed  # mm/s
        speed_quat = self.rotation_speed  # rad/s
        delta_xyz = speed_xyz * self.ikTMR.timer_period_ns / 1e9
        delta_quat = speed_quat * self.ikTMR.timer_period_ns / 1e9

        xyz_input = np.empty((3,), dtype=float)

        xyz_input[[0, 1]] = self.joy_state.stickL  # left stick to move
        xyz_input[2] = -self.joy_state.R2 + self.joy_state.L2  # deep triggers to move Z
        r1 = (bits & BUTT_INTS["R1"]) != 0
        l1 = (bits & BUTT_INTS["L1"]) != 0

        x_rot = qt.from_rotation_vector([-l1 + r1, 0, 0])
        y_rot = qt.from_rotation_vector([0, self.joy_state.stickR[0], 0])
        z_rot = qt.from_rotation_vector([0, 0, self.joy_state.stickR[1]])
        rot = (z_rot * y_rot * x_rot) ** delta_quat

        target_pose_stick = Pose(ros_now(self), xyz_input * delta_xyz, rot)
        target_stick = {leg: target_pose_stick for leg in ik_ready_legs}
        target_rel_to_ee = self.ik_syncer.abs_from_rel(target_stick)
        target_rel_to_base = rel_to_base_link(self.ik_syncer, target_stick)

        if self.ee_mode is True:
            target = target_rel_to_ee
        else:
            target = target_rel_to_base

        # self.add_log("I", f"{target_rel_to_ee}")
        self.ik_syncer.lerp_toward(target)

        self.ik_legs_prev = ik_ready_legs

    def move_ik_keyboard(
        self, lin: Flo3 = [0.0, 0.0, 0.0], rvec: Flo3 = [0.0, 0.0, 0.0]
    ):
        """
        Send a small IK velocity update from keyboard input.

        Args:
            lin: 3-vector [vx, vy, vz] (mm/s) for linear velocity.
            rvec: 3-vector [wx, wy, wz] (rad/s) for angular velocity.

        What it does:
          1. Gathers all ready IK legs in `self.selected_ik_legs`.
          2. If that set changed, clears the previous syncer to avoid stale targets.
          3. Computes Δtime since last tick.
          4. Wraps (lin, rvec) into a VelPose stamped now (and transforms
             into base_link frame if not end-effector mode).
          5. Calls `self.ik_syncer.speed_safe(...)` with the per-leg velocities and Δtime.

        Called on each IK-related key press or release.
        """
        if self.ik_syncer is None:
            return

        ik_by_leg = {ih.limb_number: ih for ih in self.ik_handlers}
        ik_ready_legs = [
            leg
            for leg in self.selected_ik_legs
            if (ih := ik_by_leg.get(leg)) is not None and ih.ready.done()
        ]
        if not ik_ready_legs:
            return

        if ik_ready_legs != self.ik_legs_prev:
            self.ik_syncer.clear()

        dt = delta_time(self, self.execTMR.timer_period_ns)

        lin = np.asarray(lin, dtype=float)
        rvec = np.asarray(rvec, dtype=float)

        delta_velpose = VelPose(ros_now(self), lin, rvec)
        target_rel_to_ee = {leg: delta_velpose for leg in ik_ready_legs}
        if self.ee_mode:
            target = target_rel_to_ee
        else:
            target = rel_vel_to_base_link(self.ik_syncer, target_rel_to_ee)
        # self.add_log("I", f"{target}")
        self.ik_syncer.speed_safe(target, dt)

        self.ik_legs_prev = ik_ready_legs

    def _update_ik_from_keys(self):
        """
        Read the current key‐hold flags in self.ik_key_states, compute the
        net translation (lin) and rotation vector (rvec), and then either:
          • call move_ik_keyboard(lin, rvec) to issue a new IK velocity command, or
          • cancel any ongoing IK velocity if no relevant keys are down.
        Ensures you can combine multiple arrow/WSAD/QE presses simultaneously.
        """
        # build net translation
        lin = np.zeros(3, dtype=float)
        if self.ik_key_states[Key.KEY_UP]:
            lin[0] += self.translation_speed
        elif self.ik_key_states[Key.KEY_DOWN]:
            lin[0] -= self.translation_speed

        if self.ik_key_states[Key.KEY_LEFT]:
            lin[1] -= self.translation_speed
        elif self.ik_key_states[Key.KEY_RIGHT]:
            lin[1] += self.translation_speed

        if self.ik_key_states[Key.KEY_I]:
            lin[2] += self.translation_speed
        elif self.ik_key_states[Key.KEY_K]:
            lin[2] -= self.translation_speed

        # build net rotation vector
        rvec = np.zeros(3, dtype=float)
        if self.ik_key_states[Key.KEY_Q]:
            rvec[0] -= self.rotation_speed
        elif self.ik_key_states[Key.KEY_E]:
            rvec[0] += self.rotation_speed

        if self.ik_key_states[Key.KEY_W]:
            rvec[1] -= self.rotation_speed
        elif self.ik_key_states[Key.KEY_S]:
            rvec[1] += self.rotation_speed

        if self.ik_key_states[Key.KEY_A]:
            rvec[2] -= self.rotation_speed
        elif self.ik_key_states[Key.KEY_D]:
            rvec[2] += self.rotation_speed

        if np.any(lin) or np.any(rvec):
            self.move_ik_keyboard(lin=lin, rvec=rvec)
        else:
            if self.ik_syncer is not None:
                self.ik_syncer.last_future.cancel()

    def move_zero(self):
        """
        Send a one-off lerp command to drive all selected joints to zero position.
        Clears the Joint Syncer if the selected joints are changed.
        """
        if self.joint_syncer is None:
            return

        selected_jnames = sorted(jn for (_, jn) in self.selected_joints)
        selected_jnames_inv = sorted(jn for (_, jn) in self.selected_joints_inv)

        if not selected_jnames and not selected_jnames_inv:
            return

        if (selected_jnames + selected_jnames_inv) != self.joints_prev:
            self.joint_syncer.clear()

        self.add_log("I", "Sending the selected joints to zero position.")
        target = {}
        target.update({jname: 0.0 for jname in selected_jnames + selected_jnames_inv})

        zero_future = self.joint_syncer.lerp(target)

        return zero_future

    def switch_ik_mode(self, val: Optional[bool] = None):
        """
        Toggle or set whether IK deltas are interpreted relative to the end
        effector (True) or to the base link (False).
        """
        if val is None:
            self.ee_mode = not self.ee_mode
        else:
            self.ee_mode = val
        self.add_log("I", f"IK mode relative to end effector: {self.ee_mode}")

    def start_ik_timer(self):
        """
        Helper to (re)start the periodic move_ik timer, ensuring a fresh IK
        stream if control resumes after a pause.
        """
        if self.ik_syncer is None:
            return
        if self.ikTMR.is_canceled():
            elapsed = Duration(nanoseconds=self.ikTMR.time_since_last_call())
            if elapsed > Duration(seconds=5):
                self.ik_syncer.clear()
            self.ikTMR.reset()
            self.ikTMR.callback()

    # ───────────────────────────── Keyboard ─────────────────────────────
    @error_catcher
    def key_downSUBCBK(self, msg: Key):
        """
        A callback that connects the pressed key from the keyboard upon arrival of the msg.
        """
        code = msg.code
        mod = msg.modifiers & ~(Key.MODIFIER_NUM | Key.MODIFIER_CAPS)
        connect_mapping(self.main_map, (code, mod))
        connect_mapping(self.sub_map, (code, mod))

        if self.current_mode == "ik_select" and code in self.ik_key_states:
            self.ik_key_states[code] = True
            self._update_ik_from_keys()

    @error_catcher
    def key_upSUBCBK(self, msg: Key):
        """
        A callback that connects the realesed key from the keyboard upon arrival of the msg.
        """
        code = msg.code
        mod = msg.modifiers & ~(Key.MODIFIER_NUM | Key.MODIFIER_CAPS)

        if self.current_mode == "ik_select" and code in self.ik_key_states:
            self.ik_key_states[code] = False
            self._update_ik_from_keys()
        else:
            self.stop_all_joints()

    # ────────────────────────────────────────────────────────────────────

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

    # ────────────────────────────────────────────────────────────────────

    def stop_all_joints(self):
        """
        Utility to clear & cancel both joint and IK syncers’ last futures.
        """
        if self.joint_syncer:
            self.joint_syncer.last_future.cancel()
        if self.ik_syncer:
            self.ik_syncer.last_future.cancel()

    def add_log(self, level: str, msg: str):
        """
        Append a timestamped log entry into the rolling message buffer
        for display in the TUI footer.
        """
        ts = time.strftime("%H:%M:%S")
        self.log_messages.append(f"{ts} [{level}] {msg}")

    def _append_leg_digit(self, d: str):
        """Add one digit to the current leg‐ID buffer."""
        self.leg_buffer += d
        self.add_log("I", f"Leg buffer: {self.leg_buffer}")

    def _backspace_leg_digit(self):
        """Erase the last digit from the leg buffer."""
        self.leg_buffer = self.leg_buffer[:-1]
        self.add_log("I", f"Leg buffer: {self.leg_buffer}")

    def _confirm_leg_buffer(self):
        """Parse and select the buffered leg number, then clear it."""
        if not self.leg_buffer:
            return
        try:
            num = int(self.leg_buffer)
            self.select_leg([num])
        except ValueError:
            self.add_log("W", f"Invalid leg number: {self.leg_buffer}")
        finally:
            self.leg_buffer = ""

    def _append_joint_leg_digit(self, d: str):
        """
        Add a digit to the leg‐ID buffer if no active leg selected.
        Once a leg is active, digit‐presses go straight to joint toggles.
        """
        if self.active_joint_leg is None:
            self.joint_leg_buffer += d
            self.add_log("I", f"Joint-leg buf: {self.joint_leg_buffer}")

    def _backspace_joint_leg_digit(self):
        """
        If we have an active leg, clear it (so you can type a new one).
        Otherwise erase the last digit from the buffer.
        """
        if self.active_joint_leg is not None:
            self.add_log("I", f"Cleared active leg {self.active_joint_leg}")
            self.active_joint_leg = None
        else:
            self.joint_leg_buffer = self.joint_leg_buffer[:-1]
            self.add_log("I", f"Joint-leg buf: {self.joint_leg_buffer}")

    def _confirm_joint_leg_buffer(self):
        if not self.joint_leg_buffer:
            return
        try:
            leg = int(self.joint_leg_buffer)
        except ValueError:
            self.add_log("W", f"Bad leg ID: {self.joint_leg_buffer}")
        else:
            if leg in self.current_legs:
                self.active_joint_leg = leg
                self.add_log("I", f"Active joint leg ▶ {leg}")
            else:
                self.add_log("W", f"Leg {leg} not found")
        finally:
            self.joint_leg_buffer = ""

    def _toggle_joint_for_active_leg(self, idx: int):
        """
        Toggle joint #idx (1-based) on self.active_joint_leg, direct only.
        """
        leg = self.active_joint_leg
        if leg is None:
            self.add_log("W", "No active leg (type its ID + Enter)")
            return

        jh = next(
            (h for h in self.joint_handlers if h.limb_number == leg and h.ready.done()),
            None,
        )
        if not jh:
            self.add_log("W", f"Leg {leg} not ready")
            return

        joints = sorted(jh.tracked)
        if idx - 1 >= len(joints):
            self.add_log("W", f"Leg {leg} has no joint {idx}")
            return

        jn = joints[idx - 1]
        key = (leg, jn)
        # skip if wheel owns it
        if key in self.selected_wheel_joints or key in self.selected_wheel_joints_inv:
            self.add_log("W", f"This joint is being used in the wheel mode.")
            return

        # toggle direct selection only
        if key in self.selected_joints:
            self.selected_joints.remove(key)
        else:
            self.selected_joints.add(key)

        self.add_log("I", f"Joints ▶ {sorted(self.selected_joints)}")

    def _append_ik_leg_digit(self, d: str):
        """Add one digit to the current leg‐ID buffer."""
        self.ik_leg_buffer += d
        self.add_log("I", f"IK Leg buffer: {self.ik_leg_buffer}")

    def _backspace_ik_leg_digit(self):
        """Erase the last digit from the leg buffer."""
        self.ik_leg_buffer = self.ik_leg_buffer[:-1]
        self.add_log("I", f"IK Leg buffer: {self.ik_leg_buffer}")

    def _confirm_ik_leg_buffer(self):
        """Parse and select the buffered leg number, then clear it."""
        if not self.ik_leg_buffer:
            return
        try:
            num = int(self.ik_leg_buffer)
            if num not in self.current_legs:
                return
            self.selected_ik_legs = [num]
            self.add_log("I", f"Selected IK leg: {self.selected_ik_legs}")
        except ValueError:
            self.add_log("W", f"Invalid leg number: {self.ik_leg_buffer}")
        finally:
            self.ik_leg_buffer = ""

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

    def create_main_map(self) -> InputMap:
        """
        Build and return the global key map (Escape ⇒ main menu, Space/Return ⇒ stubs, etc.).
        These keybinds work always.
        """
        return {
            (Key.KEY_R, ANY): [self.recover],
            (Key.KEY_R, Key.MODIFIER_LSHIFT): [self.recover_all],
            (Key.KEY_SPACE, ANY): [self.halt],
            (Key.KEY_SPACE, Key.MODIFIER_LSHIFT): [self.halt_all],
            (Key.KEY_ESCAPE, ANY): [self.clear_screen, self.enter_main_menu],
            ("option", ANY): [self.enter_main_menu],
            ("PS", ANY): [self.halt_all],
        }

    @error_catcher
    def loop(self):
        """
        Runs at ~30 Hz: execute whichever of the three syncers (joint, IK, wheel)
        currently exists.
        """
        if self.joint_syncer is not None:
            self.joint_syncer.execute()
        if self.ik_syncer is not None:
            self.ik_syncer.execute()
        if self.wheel_syncer is not None:
            self.wheel_syncer.execute()

    def clear_screen(self):
        if not hasattr(self, "clear_screen_callback"):
            return
        self.clear_screen_callback()


def main():
    rclpy.init()
    node = OperatorNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    from .operator_tui import OperatorTUI

    tui = OperatorTUI(node)
    node.clear_screen_callback = tui.clear_screen

    tui.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

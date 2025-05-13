#!/usr/bin/env python3
import curses
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
import rclpy
import urwid
from keyboard_msgs.msg import Key
from std_srvs.srv import Empty

from motion_stack.api.ros2.ik_api import IkHandler, IkSyncerRos
from motion_stack.api.ros2.joint_api import JointHandler, JointSyncerRos
from motion_stack.core.utils.math import patch_numpy_display_light
from motion_stack.ros2.utils.conversion import ros_now
from motion_stack.ros2.utils.executor import error_catcher

patch_numpy_display_light()

ALIAS = "operator_node"
MAX_JOINT_SPEED = 0.25

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


@dataclass
class JoyState:
    bits: JoyBits = 0
    stickR: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(2))
    stickL: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(2))
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

        self.main_map: Final[InputMap] = self.create_main_map()
        self.sub_map: InputMap
        self.enter_main_menu()

        self.selected_legs: List[int] = []
        self.selected_joints: Set[Tuple[int, str]] = set()
        self.continuous_turn_joints: Set[Tuple[int, str]] = set()
        self.selected_wheel_legs: Set[int] = set()

        self.log_messages: deque[str] = deque(maxlen=3)

    def recover_all(self):
        self.add_log("I", "RECOVERING ALL")

    def recover(self):
        self.add_log("I", f"RECOVERING: {self.selected_legs}")
        for leg in self.selected_legs:
            return

    def halt_all(self):
        self.add_log("I", "HALTING ALL")

    def halt(self):
        self.add_log("I", f"HALTING: {self.selected_legs}")
        for leg in self.selected_legs:
            return

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
        }

    def enter_leg_mode(self):
        # self.add_log("W", "leeeg")

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

        # strlist = "\n".join(
        #     [
        #         f"limb {jh.limb_number}: {sorted(jh.tracked)}"
        #         for jh in self.joint_handlers
        #     ]
        # )
        # self.add_log("I", f"Joints are:\n{strlist}")

        submap: InputMap = {
            (Key.KEY_W, ANY): [lambda: self.move_joints(MAX_JOINT_SPEED)],
            (Key.KEY_S, ANY): [lambda: self.move_joints(-MAX_JOINT_SPEED)],
        }

        self.sub_map = submap

    def move_joints(self, speed: float, joints: list[str] = []):
        if not self.joint_syncer:
            return

        selected_jnames = sorted(jn for (_, jn) in self.selected_joints)

        start_time = ros_now(self)

        def delta_time():
            now = ros_now(self)
            return (now - start_time).sec()

        target = {jn: speed for jn in selected_jnames}
        self.add_log("I", f"{target}")

        self.joint_syncer.speed_safe(target, delta_time)

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


def urwid_main(node: OperatorNode):
    header = urwid.Text("", align="center")
    body = urwid.SimpleFocusListWalker([])
    listbox = urwid.ListBox(body)
    log_widget = urwid.Text("", align="left")
    log_box = urwid.LineBox(log_widget, title=" Logs ")
    frame = urwid.Frame(header=header, body=listbox, footer=log_box)

    # keep track of what we've already drawn
    state = {
        "mode": None,  # last mode
        "leg_list": [],  # last sorted(node.current_legs)
        "selected_legs": [],  # last node.selected_legs
    }

    leg_checkboxes: Dict[int, urwid.CheckBox] = {}
    joint_checkboxes: Dict[Tuple[int, str], urwid.CheckBox] = {}
    wheel_checkboxes: Dict[int, urwid.CheckBox] = {}

    LEG_COLORS = [
        "dark red",
        "dark green",
        "brown",
        "dark blue",
        "dark magenta",
        "dark cyan",
        "light gray",
        "light red",
        "light green",
        "yellow",
        "light blue",
        "light magenta",
        "light cyan",
        "white",
    ]
    palette = [
        ("disabled", "dark gray", ""),
    ]
    for i, col in enumerate(LEG_COLORS):
        palette.append((f"leg{i}", col, ""))
        palette.append((f"leg{i}_focus", col, ""))

    def on_input(key):
        if key in ("q", "Q"):
            raise urwid.ExitMainLoop()

    def rebuild_main_menu():
        body.clear()
        for label, cb in [
            ("Leg Selection", lambda b: node.enter_leg_mode()),
            ("Joint Selection", lambda b: node.enter_joint_mode()),
            ("Recover", lambda b: node.recover()),
            ("Halt", lambda b: node.halt()),
            ("Quit", lambda b: loop.stop()),
        ]:
            btn = urwid.Button(label)
            urwid.connect_signal(btn, "click", cb)
            body.append(urwid.AttrMap(btn, None, focus_map="reversed"))

    def rebuild_leg_menu(legs: List[int]):
        body.clear()
        leg_checkboxes.clear()

        # one checkbox per leg
        for leg in legs:
            # pre-check if this leg is already selected
            state = leg in node.selected_legs
            cb = urwid.CheckBox(f" leg {leg}", state=state)
            leg_checkboxes[leg] = cb
            body.append(urwid.AttrMap(cb, None, focus_map="reversed"))

        body.append(urwid.Divider())

        # Confirm button
        confirm = urwid.Button("✔ Confirm Selection")

        def on_confirm(btn):
            # gather all checked legs
            chosen = [l for l, cb in leg_checkboxes.items() if cb.get_state()]
            # if none chosen, use None => all
            node.select_leg(chosen or None)

        urwid.connect_signal(confirm, "click", on_confirm)
        body.append(urwid.AttrMap(confirm, None, focus_map="reversed"))

        # Back button
        body.append(urwid.Divider())
        back = urwid.Button("← Back to Main")
        urwid.connect_signal(back, "click", lambda b: node.enter_main_menu())
        body.append(urwid.AttrMap(back, None, focus_map="reversed"))

    def rebuild_joint_menu():
        body.clear()
        joint_checkboxes.clear()
        wheel_checkboxes.clear()

        # only show legs that are selected and have data
        ready = [
            jh
            for jh in node.joint_handlers
            if jh.limb_number in node.selected_legs and jh.ready
        ]
        if not ready:
            body.append(urwid.Text("No ready legs to show — wait for joint data."))
            return

        CHUNK_SIZE = 3
        for idx, jh in enumerate(sorted(ready, key=lambda j: j.limb_number)):
            leg = jh.limb_number
            base_attr = f"leg{idx % len(LEG_COLORS)}"
            focus_attr = f"{base_attr}_focus"

            # sort + chunk the joint list
            jl = sorted(jh.tracked)
            n_cols = (len(jl) + CHUNK_SIZE - 1) // CHUNK_SIZE

            cols: List[Tuple[str, Any]] = []

            # ─── first column: Leg label or wheel checkbox ──────────────
            if leg > 10:
                cb = urwid.CheckBox(
                    f"Leg {leg}", state=(leg in node.selected_wheel_legs)
                )
                wheel_checkboxes[leg] = cb
                leg_widget = urwid.AttrMap(cb, base_attr, focus_map=focus_attr)
            else:
                txt = urwid.Text((base_attr, f"Leg {leg}"), wrap="clip")
                leg_widget = urwid.AttrMap(txt, base_attr, focus_map=focus_attr)
            cols.append(("fixed", 12, leg_widget))

            # ─── then one Pile per chunk (vertical stack) ──────────────
            for c in range(n_cols):
                pile_items = []
                for r in range(CHUNK_SIZE):
                    i = c * CHUNK_SIZE + r
                    if i < len(jl):
                        jn = jl[i]
                        checked = (leg, jn) in node.selected_joints
                        jcb = urwid.CheckBox(jn, state=checked)
                        joint_checkboxes[(leg, jn)] = jcb
                        pile_items.append(
                            urwid.AttrMap(jcb, base_attr, focus_map=focus_attr)
                        )
                    else:
                        pile_items.append(urwid.Text(""))
                pile = urwid.Pile(pile_items)
                cols.append(("weight", 1, pile))

            # add that single Columns row for this leg
            body.append(urwid.Columns(cols, dividechars=1))
            body.append(urwid.Divider())

        # ─── confirm & back ──────────────────────────────────────────
        confirm = urwid.Button("✔ Confirm Joint Selection")

        def on_conf(_):
            node.selected_joints = {
                (l, j) for (l, j), cb in joint_checkboxes.items() if cb.get_state()
            }
            node.selected_wheel_legs = {
                l for l, cb in wheel_checkboxes.items() if cb.get_state()
            }
            node.add_log(
                "I",
                f"Joints: {sorted(node.selected_joints)}   "
                f"Wheels: {sorted(node.selected_wheel_legs)}",
            )

        urwid.connect_signal(confirm, "click", on_conf)
        body.append(urwid.AttrMap(confirm, None, focus_map="reversed"))

        back = urwid.Button("← Back to Main")
        urwid.connect_signal(back, "click", lambda b: node.enter_main_menu())
        body.append(urwid.AttrMap(back, None, focus_map="reversed"))

    def refresh(loop, _):
        mode = node.current_mode
        # update header every time
        header.set_text(f"Mode ▶ {mode}    (q to quit) 🦍")

        # if the mode changed, rebuild its screen
        if mode != state["mode"]:
            state["mode"] = mode
            state["leg_list"] = []  # reset cached legs
            if mode == "main":
                rebuild_main_menu()
            elif mode == "leg_select":
                rebuild_leg_menu(sorted(node.current_legs))
            elif mode == "joint_select":
                rebuild_joint_menu()

        # if in leg_select, but the discovered legs changed, rebuild
        elif mode == "leg_select":
            legs = sorted(node.current_legs)
            if legs != state["leg_list"]:
                state["leg_list"] = legs
                rebuild_leg_menu(legs)

            else:
                sel = sorted(node.selected_legs)
                if sel != state["selected_legs"]:
                    state["selected_legs"] = sel
                    # update each checkbox in place
                    for leg, cb in leg_checkboxes.items():
                        cb.set_state(leg in sel)
                    loop.draw_screen()

        log_widget.set_text("\n".join(node.log_messages))
        loop.draw_screen()
        loop.set_alarm_in(0.5, refresh)

    loop = urwid.MainLoop(frame, palette, unhandled_input=on_input)
    loop.set_alarm_in(0, refresh)
    loop.run()


def main():
    rclpy.init()
    node = OperatorNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    urwid_main(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

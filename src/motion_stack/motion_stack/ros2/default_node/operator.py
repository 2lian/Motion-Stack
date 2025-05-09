#!/usr/bin/env python3
import curses
import dataclasses
import re
import threading
import time
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
from motion_stack.ros2.utils.executor import error_catcher

patch_numpy_display_light()

ALIAS = "operator_node"
ANY: Final[str] = "ANY"
KeyCodeModifier = Tuple[int, Union[int, Literal["ANY"]]]
JoyBits = int
JoyCodeModifier = Tuple[str, Union[JoyBits, Literal["ANY"]]]
UserInput = Union[KeyCodeModifier, JoyCodeModifier, Literal["ALWAYS"]]
NakedCall = Callable[[], Any]
InputMap = Dict[UserInput, List[NakedCall]]

operator = environ.get("OPERATOR", "operator_node")
INPUT_NAMESPACE = f"/{operator}"


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

        # input mappings remain untouched
        self.main_map: Final[InputMap] = self.create_main_map()
        self.sub_map: InputMap = {}

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
        if new_legs:
            # self.get_logger().info(f"New legs discovered: {sorted(new_legs)}")
            for leg in sorted(new_legs):
                self.joint_handlers.append(JointHandler(self, leg))
                self.ik_handlers.append(IkHandler(self, leg))
            # rebuild syncers
            self.joint_syncer = JointSyncerRos(self.joint_handlers)
            self.ik_syncer = IkSyncerRos(self.ik_handlers)

        self.current_legs = found

    @error_catcher
    def loop(self):
        """Executed at ~30Hz."""
        if self.joint_syncer:
            self.joint_syncer.execute()
        if self.ik_syncer:
            self.ik_syncer.execute()

    @error_catcher
    def key_downSUBCBK(self, msg: Key):
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

    @staticmethod
    def collapseT_KeyCodeModifier(variable: Any) -> Optional[KeyCodeModifier]:
        if (
            isinstance(variable, tuple)
            and len(variable) == 2
            and isinstance(variable[0], int)
        ):
            if isinstance(variable[1], int) or variable[1] == ANY:
                return variable  # type: ignore
        return None

    @staticmethod
    def collapseT_JoyCodeModifier(variable: Any) -> Optional[JoyCodeModifier]:
        if (
            isinstance(variable, tuple)
            and len(variable) == 2
            and isinstance(variable[0], str)
        ):
            if isinstance(variable[1], int) or variable[1] == ANY:
                return variable  # type: ignore
        return None

    @staticmethod
    def remap_onto_any(mapping: InputMap, inp: UserInput):
        kcm = OperatorNode.collapseT_KeyCodeModifier(inp)
        if kcm and kcm[1] != ANY:
            OperatorNode.connect_mapping(mapping, (kcm[0], ANY))
        jcm = OperatorNode.collapseT_JoyCodeModifier(inp)
        if jcm and jcm[1] != ANY:
            OperatorNode.connect_mapping(mapping, (jcm[0], ANY))

    @staticmethod
    def connect_mapping(mapping: InputMap, inp: UserInput):
        OperatorNode.remap_onto_any(mapping, inp)
        for fn in mapping.get(inp, []):
            fn()

    def create_main_map(self) -> InputMap:
        return {
            # (Key.KEY_RETURN, ANY): [self.recover],
            # ...
        }


def urwid_main(node: OperatorNode):
    # build an initially empty list of checkboxes
    leg_checkboxes: list[urwid.CheckBox] = []
    list_walker = urwid.SimpleFocusListWalker([])
    listbox = urwid.ListBox(list_walker)

    def refresh(loop=None, user_data=None):
        # rebuild the list whenever `current_legs` changes
        found = sorted(node.current_legs)
        # if the set changed, update the widgets
        if [int(cb.get_label().split()[1]) for cb in leg_checkboxes] != found:
            leg_checkboxes.clear()
            list_walker.clear()
            for leg in found:
                cb = urwid.CheckBox(f"leg {leg}", state=False)
                leg_checkboxes.append(cb)
                list_walker.append(urwid.AttrMap(cb, None, focus_map="reversed"))
        # schedule next refresh in 1s
        loop.set_alarm_in(1.0, refresh)

    # quit on 'q'
    def on_input(key):
        if key in ("q", "Q"):
            raise urwid.ExitMainLoop()

    palette = [("reversed", "standout", "")]
    top = urwid.Frame(
        header=urwid.Text("🦾 🦍 Legs Alive  (press q to quit)", align="center"),
        body=listbox,
    )
    loop = urwid.MainLoop(top, palette, unhandled_input=on_input)
    # kick off the first refresh
    loop.set_alarm_in(0.0, refresh)
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

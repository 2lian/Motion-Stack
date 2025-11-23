"""Overloaded JointNode for Moonbot zero."""

from typing import Any, Callable, Iterable, List

import numpy as np
from motion_stack.api.injection.remapper import Shaper, StateRemapper
from motion_stack.api.ros2.offsetter import setup_lvl0_offsetter
from motion_stack.api.ros2.state_to_topic import (
    StatesToTopic,
    default_joint_to_topic_name,
)
from motion_stack.core.utils.joint_state import JState
from motion_stack.ros2.default_node.lvl1 import DefaultLvl1
from motion_stack.ros2.utils.conversion import ros_to_time
from motion_stack.ros2.utils.executor import error_catcher
from rclpy.node import Timer
from std_msgs.msg import Float64, String


def dyna_topics(attribute: str, joint_name: str) -> str:
    if attribute == "position":
        topic_name = f"/ang_{joint_name}_set"
    else:
        topic_name = default_joint_to_topic_name(attribute, joint_name)
    return topic_name


class ZeroLvl1(DefaultLvl1):
    leg_remap = {
        1: 1,
        2: 2,
        3: 3,
        4: 4,
    }
    remap = {
        f"joint{orig}_{j}": f"joint{new}_{j}"
        for orig, new in leg_remap.items()
        for j in range(1, 4)
    }

    def __init__(self):
        super().__init__()  # default node intialization
        self.core.lvl0_remap = StateRemapper(
            name_map=self.remap,
            state_map={
                jn: Shaper(position=(lambda x: -x)) for jn in self.core.joint_names
            },
            unstate_map={
                jn: Shaper(position=(lambda x: -x)) for jn in self.core.joint_names
            },
        )
        # our new code
        self.stringPUB = self.create_publisher(String, "display_angle_command", 10)
        self.start_time = self.get_clock().now()

        # Enables publishing on individual float topics
        StatesToTopic.setup_lvl0_command(self, joint_to_topic_name=dyna_topics)

    def subscribe_to_lvl0(self, lvl0_input: Callable[[List[JState]], Any]):
        super().subscribe_to_lvl0(lvl0_input)

        sub_names = {
            self.remap[jname]: f"/read_{self.remap[jname]}"
            for jname in self.core.joint_names
        }

        for joint_name, topic_name in sub_names.items():

            def cbk(msg: Float64, joint_name: str = joint_name):
                now = self.core.now()
                # if self.core.leg_num == 1:
                # pass
                # print(joint_name, msg.data)
                lvl0_input([JState(name=joint_name, time=now, position=msg.data)])

            self.create_subscription(Float64, topic_name, cbk, 10)


def main(args=None):
    ZeroLvl1.spin()


if __name__ == "__main__":
    main()

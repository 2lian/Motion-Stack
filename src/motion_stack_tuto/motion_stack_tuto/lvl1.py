"""Overloaded JointNode for Moonbot zero."""

from typing import Iterable, List

import numpy as np
from motion_stack.api.injection.remapper import Shaper, StateRemapper
from motion_stack.api.ros2.offsetter import setup_lvl0_offsetter
from motion_stack.api.ros2.state_to_topic import StatesToTopic
from motion_stack.core.utils.joint_state import JState
from motion_stack.ros2.default_node.lvl1 import DefaultLvl1
from motion_stack.ros2.utils.conversion import ros_to_time
from motion_stack.ros2.utils.executor import error_catcher
from rclpy.node import Timer
from std_msgs.msg import String


class ZeroLvl1(DefaultLvl1):
    def __init__(self):
        super().__init__()  # default node intialization
        # our new code
        self.stringPUB = self.create_publisher(String, "display_angle_command", 10)
        self.sinusiodTMR: Timer = self.create_timer(0.1, self.send_sinusoid)
        self.start_time = self.get_clock().now()

        # Replaces default empty remap by our own
        self.core.lvl0_remap = StateRemapper(
            name_map={
                "joint1_1": "my_new_joint"  # changes the name of "joint1_1"
            },
            state_map={
                "joint1_1": Shaper(position=lambda x: x * 2)  # multiplies command by 2
            },
            unstate_map={
                "joint1_1": Shaper(position=lambda x: x / 2)  # divides sensor by 2
            },
        )

        setup_lvl0_offsetter(self)  # Enables the offsetter

        # Enables publishing on individual float topics
        StatesToTopic.setup_lvl0_command(self)

    @error_catcher
    def send_sinusoid(self):
        """Sends a sinusoidal command on every joint based on the clock.
        Callback of the self.sinTMR timer."""
        # sinusoid creation
        now = self.get_clock().now()
        since_start = ros_to_time(now - self.start_time)
        PERIOD = 10
        AMPLITUDE = 0.1
        sinusoid = np.sin(since_start.sec() * np.pi / PERIOD) * AMPLITUDE

        # joint states data creation
        state_list: List[JState] = []
        for name in self.core.jointHandlerDic.keys():
            state = JState(name=name, time=ros_to_time(now), position=sinusoid)
            state_list.append(state)

        # call of the function handling incomming joint commands from lvl2
        self.core.coming_from_lvl2(state_list)

    def publish_to_lvl0(self, states: List[JState]):
        """Executes our custom code every time state data (commands) needs to be sent down to lvl0."""
        super().publish_to_lvl0(states)  # executes default behavior

        # prepares string
        str_to_send: List[str] = [f"leg {self.core.leg_num}"]
        for joint_state in states:
            if joint_state.position is None:
                continue  # skips empty states with no angles
            str_to_send.append(
                f"lvl1 -> lvl0: {joint_state.name} "
                f"| {np.rad2deg(joint_state.position):.1f}"
            )

        # publishes string
        if str_to_send[1:]:
            self.stringPUB.publish(String(data="\n".join(str_to_send)))


def main(args=None):
    ZeroLvl1.spin()


if __name__ == "__main__":
    main()

"""Overloaded JointNode for Moonbot zero."""

from typing import Iterable, List

from easy_robot_control.EliaNode import myMain, rosTime2Float
from easy_robot_control.injection.offsetter import OffsetterLvl0
from easy_robot_control.injection.topic_pub import StatesToTopic
from easy_robot_control.utils.state_remaper import Shaper, StateRemapper
from easy_robot_control.joint_state_interface import JointNode
from easy_robot_control.utils.joint_state_util import JState

import numpy as np
from rclpy.node import Timer
from std_msgs.msg import String

remap_lvl1 = StateRemapper(
    name_map={"joint1-1": "my-new-joint"}, # changes the name of "joint1-1" is present
    state_map={"joint1-1": Shaper(position=lambda x: x * 2)}, # multiplies command by 2
    unstate_map={"joint1-1": Shaper(position=lambda x: x / 2)}, # divides sensor by 2
)


class ZeroLvl1(JointNode):
    def __init__(self):
        super().__init__()  # default node intialization
        # our new code
        self.stringPUB = self.create_publisher(String, "display_angle_command", 10)
        self.sinusiodTMR: Timer = self.create_timer(0.1, self.send_sinusoid)
        self.start_time = self.getNow()
        self.lvl0_remap = remap_lvl1 # Replaces default empty remap by our own
        self.offsetter = OffsetterLvl0(self) # Enables the offsetter, that's it
        self.state2topic = StatesToTopic(self)  # Publishing to be called in send_to_lvl0

    def send_sinusoid(self):
        """Sends a sinusoidal command on every joint based on the clock.
        Callback of the self.sinTMR timer."""
        # sinusoid creation
        now = self.getNow()
        since_start = rosTime2Float(now - self.start_time)
        PERIOD = 10
        AMPLITUDE = 0.1
        sinusoid = np.sin(since_start * np.pi / PERIOD) * AMPLITUDE

        # joint states data creation
        state_list: List[JState] = []
        for name in self.jointHandlerDic.keys():
            state = JState(name=name, time=now, position=sinusoid)
            state_list.append(state)

        # call of the function handling incomming joint commands from lvl2
        self.coming_from_lvl2(state_list)

    def send_to_lvl0(self, states: List[JState]):
        """Executes our custom code every time state data (commands) needs to be sent down to lvl0."""
        super().send_to_lvl0(states)  # executes default behavior
        # our new code
        self.state2topic.publish(states)

        # prepares string
        str_to_send: List[str] = [f"leg {self.leg_num}"]
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
    myMain(ZeroLvl1)


if __name__ == "__main__":
    main()

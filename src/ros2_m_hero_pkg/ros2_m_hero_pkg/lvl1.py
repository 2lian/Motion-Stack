"""Overloaded JointNode for Moonbot hero."""

import os
from datetime import datetime
from typing import Iterable

from easy_robot_control.EliaNode import get_src_folder, myMain
from easy_robot_control.injection.offsetter import OffsetterLvl0
from easy_robot_control.injection.topic_pub import StatesToTopic
from easy_robot_control.joint_state_interface import JointNode
from easy_robot_control.utils.joint_state_util import JState

from ros2_m_hero_pkg.remap import map_lvl0, map_lvl2

# offset path
WORKSPACE_PATH = os.path.abspath(os.path.join(get_src_folder("ros2_m_hero_pkg"), "../.."))
RECOVERY_PATH = os.path.join(WORKSPACE_PATH, "angle_recovery")
if not os.path.exists(RECOVERY_PATH):
    os.makedirs(RECOVERY_PATH)
OFFSET_PATH = os.path.join(RECOVERY_PATH, "offset.csv")
ANGLE_PATH = os.path.join(
    RECOVERY_PATH,
    f"off{datetime.now().isoformat(timespec='seconds').replace(':', '-')}.csv",
)


class MyStatesToTopic(StatesToTopic):
    def make_topic_name(self, attribute: str, joint_name: str) -> str:
        topic_name = f"canopen_motor/{joint_name}_joint_{attribute}_controller/command"
        return topic_name


class HeroLvl1(JointNode):
    """Overloaded JointNode for moonbot hero v1.
    implements:
        - Loads and apply custom remapping
        - load, save and apply offsets for each joint.
        - sends joint states over individual Float 64 topics, instead of a
            single JointStates topic
    """

    def __init__(self):
        super().__init__()
        # custom mapping
        self.lvl0_remap = map_lvl0.simplify(self.jointHandlerDic.keys())
        self.lvl2_remap = map_lvl2.simplify(self.jointHandlerDic.keys())

        # dependency injection to have offsets available
        self.offsetter = OffsetterLvl0(
            self, angle_path=ANGLE_PATH, offset_path=OFFSET_PATH
        )
        # dependency injection to also publish over float topics
        self.topic_pub = MyStatesToTopic(self)

    def send_to_lvl0(self, states: Iterable[JState]):
        """This function is executed every time data needs to be sent down."""
        super().send_to_lvl0(states)  # executes default just in case
        self.topic_pub.publish(states)

    def wait_for_lower_level(
        self, more_services: Iterable[str] = ..., all_requiered: bool = ...
    ):
        # add driver/init to the services to wait for
        # (this can also be done in the launcher)
        more_services = set(more_services) | {"driver/init"}
        return super().wait_for_lower_level(more_services, all_requiered)


def main(args=None):
    myMain(HeroLvl1)


if __name__ == "__main__":
    main()

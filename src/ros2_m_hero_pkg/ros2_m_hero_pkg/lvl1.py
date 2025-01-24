"""Overloaded JointNode for Moonbot hero."""

import os
from datetime import datetime
from typing import Iterable, List

from motion_stack.core.utils.joint_mapper import reverse_dict
from motion_stack.ros2.default_node.lvl1 import DefaultLvl1, main
from motion_stack.api.ros2.offsetter import setup_lvl0_offsetter
from motion_stack.api.ros2.state_to_topic import StatesToTopic
from motion_stack.ros2.utils.files import get_src_folder

from .remap import map_lvl0, map_lvl2

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


def make_topic_name(attribute: str, joint_name: str) -> str:
    topic_name = f"canopen_motor/{joint_name}_{attribute}_controller/command"
    return topic_name


class HeroLvl1(DefaultLvl1):
    """Customized Lvl1 ros node for moonbot hero v1.

    implements:
        - Loads and apply custom remapping
        - load, save and apply offsets for each joint.
        - sends joint states over individual Float 64 topics, instead of a\
                single JointStates topic
    """

    def __init__(self):
        super().__init__()
        core = self.core
        # custom mapping
        self.declare_parameter("joint_remapping", True)
        self.JOINT_REMAPPING = (
            self.get_parameter("joint_remapping").get_parameter_value().bool_value
        )

        if self.JOINT_REMAPPING:
            core.lvl0_remap = map_lvl0.simplify(core.jointHandlerDic.keys())
            core.lvl0_remap.unname_map = reverse_dict(core.lvl0_remap.name_map)
            core.lvl2_remap = map_lvl2.simplify(core.jointHandlerDic.keys())

        # dependency injection to have offsets available
        setup_lvl0_offsetter(
            self, angle_recovery_path=ANGLE_PATH, offset_path=OFFSET_PATH
        )

        # dependency injection to also publish over float topics
        StatesToTopic.setup_lvl0_command(self, make_topic_name)


def main(args=None):
    HeroLvl1.spin()


if __name__ == "__main__":
    main()

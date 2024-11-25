"""Overloaded JointNode for Moonbot hero."""
import os
from datetime import datetime
from typing import Dict, Final, Iterable, List, Optional

from easy_robot_control.EliaNode import get_src_folder, replace_incompatible_char_ros2
from easy_robot_control.joint_state_interface import JointNode
from easy_robot_control.offsetter import OffsetterLvl0
from EliaNode import myMain
from python_package_include.joint_state_util import JState
from rclpy.node import Publisher
from std_msgs.msg import Float64

from ros2_m_hero_pkg.remap import map_lvl0, map_lvl2

ATTR: Final[List[str]] = ["position", "velocity"]

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


class HeroLvl1(JointNode):
    """Overloaded JointNode for moonbot hero v1.
    implements:
        - load, save and apply offsets for each joint.
        - sends joint states over individual Float 64 topics, instead of a
            single JointStates topic
        - Loads and apply custom remapping
    """

    def __init__(self):
        super().__init__()
        self.lvl0_remap = map_lvl0.simplify(self.jointHandlerDic.keys())
        self.lvl2_remap = map_lvl2.simplify(self.jointHandlerDic.keys())

        self.pub2_dict: Dict[str, Dict[str, Publisher]] = {}

        self.offsetter = OffsetterLvl0(
            self, angle_path=ANGLE_PATH, offset_path=OFFSET_PATH
        )

    def firstSpinCBK(self):
        super().firstSpinCBK()

    def get_create_motor_pub(self, attr: str, name: str) -> Publisher:
        """Return the publisher correspondong to the joint and attribute (pos, speed..).
        If does not exists, creates it

        Args:
            attr: position, velocity or effort
            name: joint name

        Returns:
            pub on topic "canopen_motor/{name}_joint_{attr}_controller/command"
        """
        pudic: Optional[Dict[str, Publisher]] = self.pub2_dict.get(attr)
        if pudic is None:
            pudic = {}
            self.pub2_dict[attr] = pudic
        pub = pudic.get(name)
        if pub is None:
            pub = self.create_publisher(
                Float64,
                replace_incompatible_char_ros2(
                    f"canopen_motor/{name}_joint_{attr}_controller/command"
                ),
                10,
            )
            # self.pinfo(f"created {pub.topic_name}")
            # self.pwarn(f"{self.lvl0_remap.name_map}")
            self.pub2_dict[attr][name] = pub
        return pub

    def send_to_lvl0(self, states: Iterable[JState]):
        """This function is executed every time data needs to be sent down."""
        super().send_to_lvl0(states)  # executes default just in case
        for state in states:
            if state.name is None:
                continue
            for attr in ATTR:
                value = getattr(state, attr, None)
                if value is None:
                    continue
                pub = self.get_create_motor_pub(attr, state.name)
                # self.pwarn({f"{attr}/{state.name}: {value:.2f}"})
                pub.publish(Float64(data=float(value)))


def main(args=None):
    myMain(HeroLvl1)


if __name__ == "__main__":
    main()

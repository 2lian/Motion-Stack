from typing import Dict, Final, Iterable, List, Optional

from easy_robot_control.EliaNode import replace_incompatible_char_ros2
from easy_robot_control.joint_state_interface import JointNode
from EliaNode import myMain
from python_package_include.joint_state_util import JState
from rclpy.node import Publisher
from std_msgs.msg import Float64

from ros2_m_hero_pkg.remap import map_lvl0, map_lvl2

ATTR: Final[List[str]] = ["position", "velocity"]


class HeroLvl1(JointNode):
    def __init__(self):
        super().__init__()
        self.lvl0_remap = map_lvl0.simplify(self.jointHandlerDic.keys())
        self.lvl2_remap = map_lvl2.simplify(self.jointHandlerDic.keys())

        self.pub2_dict: Dict[str, Dict[str, Publisher]] = {}

    def get_create_motor_pub(self, attr: str, name: str) -> Publisher:
        pudic: Optional[Dict[str, Publisher]] = self.pub2_dict.get(attr)
        if pudic is None:
            pudic = {}
            self.pub2_dict[ attr] = pudic
        pub = pudic.get(name)
        if pub is None:
            pub = self.create_publisher(
                Float64,
                replace_incompatible_char_ros2(
                    f"canopen_motor/{name}_joint_{attr}_controller/command"
                ),
                10,
            )
            self.pinfo(f"created {pub.topic_name}")
            # self.pwarn(f"{self.lvl0_remap.name_map}")
            self.pub2_dict[attr][name] = pub
        return pub

    def send_to_lvl0(self, states: Iterable[JState]):
        super().send_to_lvl0(states)
        for state in states:
            if state.name is None:
                continue
            for attr in ATTR:
                value = getattr(state, attr, None)
                pub = self.get_create_motor_pub(attr, state.name)
                if value is None:
                    continue
                pub.publish(Float64(data=float(value)))


def main(args=None):
    myMain(HeroLvl1)


if __name__ == "__main__":
    main()

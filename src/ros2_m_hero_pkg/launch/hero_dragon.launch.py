from typing import Any, Dict, Iterable, List, Union

from ros2_m_hero_pkg.launch.mh_unified import LevelBuilder

# V Change default parameters here V
#   \  /   #
#    \/    #
ROBOT_NAME = "hero_dragon"

BRIDGE_LIMB = 4
MANIP_LIMB = 3
BACK_WHEEL = 4
FRONT_WHEEL = 2  # the one wheel with 2 limb connected

leg_dict: Dict[int, Union[str, int]] = {  # leg number -> end effector
    # 1: 0,
    # 2: 0,
    # 2: "wheel14_c45",
    # 4: f"leg4gripper2_straight",
    # 11: "11wheel_in",
    # 12: "wheel12_in",
    # 13: "13wheel_in",
    # 14: "wheel14_in",
}
leg_dict[BRIDGE_LIMB] = f"wheel1{BACK_WHEEL}_c45"
leg_dict[MANIP_LIMB] = f"leg{MANIP_LIMB}gripper2_straight"
leg_dict[10+BACK_WHEEL] = f"wheel1{BACK_WHEEL}_in"
leg_dict[10+FRONT_WHEEL] = f"wheel1{FRONT_WHEEL}_in"

builder = LevelBuilder(ROBOT_NAME, leg_dict)
params = builder.all_param
levels = builder.make_levels()


def generate_launch_description():
    return builder.make_description()

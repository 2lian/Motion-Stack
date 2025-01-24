from typing import Any, Dict, Iterable, List, Union

from ros2_m_hero_pkg.launch.mh_unified import LevelBuilder, urdf_from_name

# V Change default parameters here V
#   \  /   #
#    \/    #
ROBOT_NAME = "hero_dragon"

BRIDGE_LIMB = 4
MANIP_LIMB = 3
BACK_WHEEL = 4
FRONT_WHEEL = 2  # the one wheel with 2 limb connected

leg_dict: Dict[int, Union[str, int]] = {}  # leg number -> end effector
leg_dict[BRIDGE_LIMB] = f"wheel1{BACK_WHEEL}_c45"
leg_dict[MANIP_LIMB] = f"leg{MANIP_LIMB}gripper2_straight"
leg_dict[10 + BACK_WHEEL] = f"wheel1{BACK_WHEEL}_in"
leg_dict[10 + FRONT_WHEEL] = f"wheel1{FRONT_WHEEL}_in"

builder = LevelBuilder(urdf_from_name(ROBOT_NAME), leg_dict)


def generate_launch_description():
    return builder.make_description()

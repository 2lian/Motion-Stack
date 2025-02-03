from typing import Any, Dict, Iterable, List, Union

from ros2_m_hero_pkg.launch.mh_unified import LevelBuilder, urdf_from_name

# V Change default parameters here V
#   \  /   #
#    \/    #
ROBOT_NAME = "hero_dragon_realsense"

BRIDGE_LIMB = 1
MANIP_LIMB = 4
BACK_WHEEL = 12
FRONT_WHEEL = 11  # the one wheel with 2 limb connected

leg_dict: Dict[int, Union[str, int]] = {  # leg number -> end effector
    BRIDGE_LIMB: f"wheel{BACK_WHEEL}_c45",
    MANIP_LIMB: f"leg{MANIP_LIMB}gripper2_straight",
    BACK_WHEEL: f"wheel{BACK_WHEEL}_in",
    FRONT_WHEEL: f"wheel{FRONT_WHEEL}_in",
}
urdf = urdf_from_name(ROBOT_NAME, options=f"bridge:={BRIDGE_LIMB} manip:={MANIP_LIMB} front_wheel:={FRONT_WHEEL} back_wheel:={BACK_WHEEL}")

builder = LevelBuilder(urdf, leg_dict)


def generate_launch_description():
    return builder.make_description()


from typing import Any, Dict, Iterable, List, Union

from numpy import left_shift

from ros2_m_hero_pkg.launch.mh_unified import LevelBuilder, urdf_from_name

# V Change default parameters here V
#   \  /   #
#    \/    #
ROBOT_NAME = "hero_tricycle"

FRONT_LIMB = 1
LEFT_LIMB = 2
RIGHT_LIMB = 3
FRONT_WHEEL = 11
LEFT_WHEEL = 12
RIGHT_WHEEL = 13

leg_dict: Dict[int, Union[str, int]] = {  # leg number -> end effector
    FRONT_LIMB: f"wheel{FRONT_WHEEL}_center",
    LEFT_LIMB: f"wheel{LEFT_WHEEL}_center",
    RIGHT_LIMB: f"wheel{RIGHT_WHEEL}_center",
    FRONT_WHEEL: f"wheel{FRONT_WHEEL}_in",
    LEFT_WHEEL: f"wheel{LEFT_WHEEL}_in",
    RIGHT_WHEEL: f"wheel{RIGHT_WHEEL}_in",
}
urdf = urdf_from_name(
    ROBOT_NAME,
    options=(
        f"front_limb:={FRONT_LIMB} "
        f"left_limb:={LEFT_LIMB} "
        f"right_limb:={RIGHT_LIMB} "
        f"front_wheel:={FRONT_WHEEL} "
        f"left_wheel:={LEFT_WHEEL} "
        f"right_wheel:={RIGHT_WHEEL} "
    ),
)

builder = LevelBuilder(urdf, leg_dict)


def generate_launch_description():
    return builder.make_description()

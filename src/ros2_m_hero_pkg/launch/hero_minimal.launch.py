from typing import Any, Dict, Iterable, List, Union

from ros2_m_hero_pkg.launch.mh_unified import LevelBuilder, urdf_from_name

ROBOT_NAME = "hero_minimal"

LIMB = 1
WHEEL = 11  # the one wheel with 2 limb connected

leg_dict: Dict[int, Union[str, int]] = {  # leg number -> end effector
    LIMB: f"wheel{WHEEL}_center",
    WHEEL: f"wheel{WHEEL}_in",
}
urdf = urdf_from_name(ROBOT_NAME, options=f"limb:={LIMB} wheel:={WHEEL}")

builder = LevelBuilder(urdf, leg_dict)


def generate_launch_description():
    return builder.make_description()

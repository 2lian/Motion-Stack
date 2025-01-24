from typing import Any, Dict, Final, Iterable, List, Union

from ros2_m_hero_pkg.launch.mh_unified import LevelBuilder, urdf_from_name

LEGS_DIC: Dict[int, Union[str, int]] = {  # leg number -> end effector
    4: "leg4gripper2_straight",
    2: "leg2gripper2_straight",
    1: "leg1gripper2_straight",
    # 4: 0,
    # 11: "11wheel_in",
    # 12: "12wheel_in",
    # 13: "13wheel_in",
    # 14: "14wheel_in",
}

ROBOT_NAME = "hero_3leg"

builder = LevelBuilder(urdf_from_name(ROBOT_NAME), LEGS_DIC)

def generate_launch_description():
    return builder.make_description()


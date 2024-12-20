from typing import Any, Dict, Final, Iterable
from typing import List, Union
from ros2_m_hero_pkg.launch.mh_unified import LevelBuilder


LEGS_DIC: Dict[int, Union[str, int]] = {  # leg number -> end effector
    4: "leg4gripper2_straight",
    2: "leg2gripper2_straight",
    1: "leg1gripper2_straight",
    # 3: 0,
    11: "wheel11_in",
    12: "wheel12_in",
    13: "wheel13_in",
    # 14: "14wheel_in",
}

ROBOT_NAME = "hero_3legwheel"

builder = LevelBuilder(ROBOT_NAME, LEGS_DIC)
params = builder.all_param
levels = builder.make_levels()

def generate_launch_description():
    return builder.make_description()

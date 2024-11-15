from typing import Any, Dict, Final, Iterable
from typing import List, Union
from mh_unified import LevelBuilder


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

builder = LevelBuilder(ROBOT_NAME, LEGS_DIC)
params = builder.all_param
levels = builder.make_levels()

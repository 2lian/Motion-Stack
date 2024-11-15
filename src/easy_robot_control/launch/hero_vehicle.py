from typing import Any, Dict, Iterable, List, Union

from mh_unified import LevelBuilder

# V Change default parameters here V
#   \  /   #
#    \/    #
LEGS_DIC: Dict[int, Union[str, int]] = {  # leg number -> end effector
    # 1: 0,
    # 2: 0,
    4: "12wheel_in",
    # 4: f"leg4gripper2_straight",
    11: "11wheel_in",
    12: "12wheel_in",
    13: "13wheel_in",
    14: "14wheel_in",
}
ROBOT_NAME = "hero_vehicle"

builder = LevelBuilder(ROBOT_NAME, LEGS_DIC)
params = builder.all_param
levels = builder.make_levels()

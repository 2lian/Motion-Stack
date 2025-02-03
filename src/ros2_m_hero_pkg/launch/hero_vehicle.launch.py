from typing import Any, Dict, Iterable, List, Union

from ros2_m_hero_pkg.launch.mh_unified import LevelBuilder, urdf_from_name

# V Change default parameters here V
#   \  /   #
#    \/    #
LEGS_DIC: Dict[int, Union[str, int]] = {  # leg number -> end effector
    1: "wheel12_center",
    # 2: 0,
    # 4: "12wheel_in",
    # 4: f"leg4gripper2_straight",
    11: "11wheel_in",
    12: "12wheel_in",
    # 13: "13wheel_in",
    # 14: "14wheel_in",
}
ROBOT_NAME = "hero_vehicle"

builder = LevelBuilder(urdf_from_name(ROBOT_NAME), LEGS_DIC)

def generate_launch_description():
    return builder.make_description()


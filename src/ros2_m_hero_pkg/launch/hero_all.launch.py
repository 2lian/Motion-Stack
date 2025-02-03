from typing import Any, Dict, Final, Iterable, List, Union

import numpy as np
from launch_ros.parameter_descriptions import ParameterValue
from motion_stack.api.launch.builder import command_from_xacro_path, xacro_path_from_pkg

from ros2_m_hero_pkg.launch.mh_unified import (
    CASE,
    LEG1,
    LEG2,
    LEG3,
    LEG4,
    LevelBuilder,
    is_wheel,
    urdf_from_name,
    xacro_path_from_name,
)

LEGS_DIC: Dict[int, Union[str, int]] = {  # leg number -> end effector
    1: "leg1gripper2_straight",
    2: "leg2gripper2_straight",
    3: "leg3gripper2_straight",
    4: "leg4gripper2_straight",
    11: "wheel11_in",
    12: "wheel12_in",
    13: "wheel13_in",
    14: "wheel14_in",
}

class ModifiedBuilder(LevelBuilder):
    """change 1 function of the LevelBuilder to publish the right tf"""

    def __init__(self, urdf, leg_dict, params_overwrite=dict()):
        super().__init__(urdf, leg_dict, params_overwrite)

    def make_leg_param(self, leg_index: int, ee_name: Union[None, str, int]) -> Dict:
        # here the legs load their individual urdf
        p = super().make_leg_param(leg_index, ee_name)
        if not is_wheel(leg_index):
            p["start_coord"] = [np.nan, np.nan, np.nan]
            p["urdf_path"] = "NONE"
            p["urdf"] = ParameterValue(
                urdf_from_name("hero_7dof", options=f"number:={leg_index}"),
                value_type=str,
            )
        else:
            pass # todo individual wheels later ... one day
        return p


builder = ModifiedBuilder(urdf_from_name("hero_all"), LEGS_DIC)


def generate_launch_description():
    return builder.make_description()

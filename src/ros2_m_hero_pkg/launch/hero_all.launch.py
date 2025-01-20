from typing import Any, Dict, Final, Iterable, List, Union

from motion_stack.api.launch.builder import command_from_xacro_path, xacro_path_from_pkg
import numpy as np
from easy_robot_control.launch.default_params import get_xacro_path

from ros2_m_hero_pkg.launch.mh_unified import (
    CASE,
    LEG1,
    LEG2,
    LEG3,
    LEG4,
    LevelBuilder,
    is_wheel,
)

LEGS_DIC: Dict[int, Union[str, int]] = {  # leg number -> end effector
    1: 0,
    # 2: 0,
    # 3: 0,
    # 4: 0,
    # 11: "wheel11_in",
    # 12: "wheel12_in",
    # 13: "wheel13_in",
    # 14: "wheel14_in",
}

if CASE.name in [LEG1, LEG2, LEG3, LEG4]:
    ROBOT_NAME = f"hero_7dofm{CASE.name}"  # Uses only the leg's urdf if it's a leg
else:
    ROBOT_NAME = f"hero_7dof_all"


class ModifiedBuilder(LevelBuilder):
    """change 1 function of the LevelBuilder to publish the right tf"""

    def make_leg_param(self, leg_index: int, ee_name: Union[None, str, int]) -> Dict:
        # here the legs load their individual urdf, so the origin is gripper1
        # everything else loads the big urdf with everything
        p = super().make_leg_param(leg_index, ee_name)
        if not is_wheel(leg_index):
            p["start_coord"] = [np.nan, np.nan, np.nan]
            hero7dof = "hero_7dof"  # just to get the file path
            hero7dof_path = get_xacro_path(hero7dof)
            xacro_path = (
                hero7dof_path[: -len(hero7dof + ".xacro")]
                + f"hero_7dofm{leg_index}.xacro"
            )
            p["urdf_path"] = xacro_path
            p["urdf"] = command_from_xacro_path(xacro_path)
        return p


builder = ModifiedBuilder(ROBOT_NAME, LEGS_DIC)
params = builder.all_param
levels = builder.make_levels()


def generate_launch_description():
    return builder.make_description()

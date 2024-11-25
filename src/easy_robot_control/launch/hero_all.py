from typing import Any, Dict, Final, Iterable, List, Union

import numpy as np
from default_params import get_xacro_path
from mh_unified import CASE, LEG1, LEG2, LEG3, LEG4, USE_RVIZ, LevelBuilder, is_wheel

LEGS_DIC: Dict[int, Union[str, int]] = {  # leg number -> end effector
    1: 0,
    2: 0,
    3: 0,
    4: 0,
    11: "11wheel_in",
    12: "12wheel_in",
    13: "13wheel_in",
    14: "14wheel_in",
}

if CASE.name in [LEG1, LEG2, LEG3, LEG4]:
    ROBOT_NAME = f"hero_7dof_m{CASE.name}"  # Uses only the leg's urdf if it's a leg
else:
    ROBOT_NAME = f"hero_7dof_all"


class ModifiedBuilder(LevelBuilder):
    """I need to change 1 function of the LevelBuilder"""

    def __init__(self, robot_name: str, legs_dic: Dict[int, Union[str, int]]):
        super().__init__(robot_name, legs_dic)

    def make_leg_param(self, leg_index: int, ee_name: Union[None, str, int]) -> Dict:
        # here the legs load their individual urdf, so the origin is gripper1
        # everything else loads the big urdf with everything
        p = super().make_leg_param(leg_index, ee_name)
        if is_wheel(leg_index) and USE_RVIZ:
            # when using rviz the wheel will publish the world->baselink
            if leg_index == 11:
                p["start_coord"] = [0.0, 0.0, 0.0]
            p["start_effector_name"] = "base_link"
            p["end_effector_name"] = "base_link"
        else:
            p["start_coord"] = [np.nan, np.nan, np.nan]
            hero7dof = "hero_7dof"  # just to get the file path
            hero7dof_path = get_xacro_path(hero7dof)
            xacro_path = (
                hero7dof_path[: -len(hero7dof + ".xacro")]
                + f"hero_7dofm{leg_index}.xacro"
            )
            p["urdf_path"] = xacro_path
        return p


builder = ModifiedBuilder(ROBOT_NAME, LEGS_DIC)
params = builder.all_param
levels = builder.make_levels()

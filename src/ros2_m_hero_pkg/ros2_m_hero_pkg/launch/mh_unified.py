"""
Creates launch files for moonbot hero configurations, working in RVIZ and Reality
"""

import dataclasses
from copy import deepcopy
from os import environ
from time import sleep
from typing import Any, Dict, Iterable, List, Mapping, Optional, Union

import numpy as np
from launch_ros.actions import Node
from motion_stack.api.launch.builder import Command
from motion_stack.api.launch.builder import LevelBuilder as DefaultLvlBlder
from motion_stack.api.launch.builder import command_from_xacro_path, xacro_path_from_pkg
from motion_stack.api.launch.default_params import RVIZ_SIMU_REMAP


@dataclasses.dataclass
class LaunchOptions:
    name: str
    leg_index: Optional[List[int]]
    lvl_to_launch: List[int]


HERO_OVERLOAD_PKG = "ros2_m_hero_pkg"

ALL = "ALL"  # Debugg PC, we run Rviz and everything
NOTH = "NOTHING"  # Debugg PC, we run nothing form the stack
BASE = "BASE"  # Command station, will run 4-5
LEG1 = "1"  # is a leg so we run 1-2-3
LEG2 = "2"
LEG3 = "3"
LEG4 = "4"
WHE1 = "11"  # is a wheel so we run 1
WHE2 = "12"
WHE3 = "13"
WHE4 = "14"


def is_wheel(ind: int) -> bool:
    return int(ind) >= 10


CASES = {
    ALL: LaunchOptions(name=ALL, leg_index=None, lvl_to_launch=[1, 2, 3, 4, 5]),
    NOTH: LaunchOptions(name=NOTH, leg_index=None, lvl_to_launch=[]),
    BASE: LaunchOptions(name=BASE, leg_index=None, lvl_to_launch=[4, 5]),
    LEG1: LaunchOptions(name=LEG1, leg_index=[1], lvl_to_launch=[1, 2, 3]),
    LEG2: LaunchOptions(name=LEG2, leg_index=[2], lvl_to_launch=[1, 2, 3]),
    LEG3: LaunchOptions(name=LEG3, leg_index=[3], lvl_to_launch=[1, 2, 3]),
    LEG4: LaunchOptions(name=LEG4, leg_index=[4], lvl_to_launch=[1, 2, 3]),
    WHE1: LaunchOptions(name=WHE1, leg_index=[11], lvl_to_launch=[1]),
    WHE2: LaunchOptions(name=WHE2, leg_index=[12], lvl_to_launch=[1]),
    WHE3: LaunchOptions(name=WHE3, leg_index=[13], lvl_to_launch=[1]),
    WHE4: LaunchOptions(name=WHE4, leg_index=[14], lvl_to_launch=[1]),
}

MOONBOT_PC_NUMBER = str(environ.get("M_LEG"))  # leg number saved on lattepanda
if MOONBOT_PC_NUMBER in [None, "", "none", "None", "ALL"]:
    MOONBOT_PC_NUMBER = ALL

USER_RVIZ_VAR = str(environ.get("USE_RVIZ"))
M_LEG = str(environ.get("M_LEG"))  # leg number saved on real robot os
# USE_RVIZ = (USER_RVIZ_VAR in [None, "", "None", "TRUE"]) and (
#     M_LEG in ["NOTHING", None, "", "None", "ALL"]
# )

CASE = CASES[MOONBOT_PC_NUMBER]


def clean_leg_dic(
    legs_dic: Mapping[int, Union[str, int]]
) -> Mapping[int, Union[str, int]]:
    """
    Args:
        legs_dic: dict of leg number associated with end effector

    Returns:
        legs_dic but with entries not corresponding to the current case deleted
    """
    if CASE.name in [ALL, BASE, NOTH]:
        leg_ee_out = legs_dic
    elif CASE.name in [LEG1, LEG2, LEG3, LEG4, WHE1, WHE2, WHE3, WHE4]:
        # launches only the end effector associated with the leg/wheel
        leg_num = int(CASE.name)  # see CASE definition, this is always the case
        ee = legs_dic.get(leg_num)
        if ee is None:
            print(f"Wrong case, no end effector associated with {CASE.name}")
            raise Exception(f"Wrong case, no end effector associated with {CASE.name}")
        leg_ee_out = {leg_num: ee}
    else:
        print("wrong case")
        raise Exception(f"Wrong case, {CASE.name} does not exist")
    return leg_ee_out


def get_LEG_EE(legs_dic: Dict[int, Union[str, int]]) -> List[Union[str, int]]:
    """
    Args:
        legs_dic: dict of leg number associated with end effector

    Returns:
        list of end effector that should be used based on the env variable
    """
    if CASE.name in [ALL, BASE, NOTH]:  # awful code
        leg_ee_out = legs_dic.values()  # launches all end effectors
    elif CASE.name in [LEG1, LEG2, LEG3, LEG4, WHE1, WHE2, WHE3, WHE4]:
        val = legs_dic.get(int(CASE.name))
        if val is None:
            print(f"Wrong case, no end effector associated with {CASE.name}")
            raise Exception(f"Wrong case, no end effector associated with {CASE.name}")
        leg_ee_out = [
            val
        ]  # launches only the end effectior associated with the leg/wheel
    else:
        print("wrong case")
        raise Exception(f"Wrong case, {CASE.name} does not exist")
    return list(leg_ee_out)


def get_LEG_IND(legs_dic: Dict[int, Union[str, int]]) -> List[int]:
    """
    Args:
        legs_dic: dict of leg number associated with end effector

    Returns:
        list of leg number that should be used based on the env variable
    """
    if CASE.leg_index is None:
        leg_ind_out = legs_dic.keys()
    else:
        leg_ind_out = CASE.leg_index
    return list(leg_ind_out)


def wheel_joint_names(wheel_leg_index: int) -> List[str]:
    """Returns the joints names in the urdf of a wheel

    Args:
        wheel_leg_index: 11-12-13-14 ...
    """
    return [
        f"wheel{wheel_leg_index}_left_joint",
        f"wheel{wheel_leg_index}_right_joint",
    ]


def gripper_joint_names(leg_index: int) -> List[str]:
    """Returns the joints names in the urdf of a wheel

    Args:
        wheel_leg_index: 11-12-13-14 ...
    """
    return [
        # f"leg{leg_index}gripper1_jaw_left_joint",
        # f"leg{leg_index}gripper1_jaw_right_joint",
        # f"leg{leg_index}gripper2_jaw_left_joint",
        # f"leg{leg_index}gripper2_jaw_right_joint",
        f"leg{leg_index}grip1",
        f"leg{leg_index}grip2",
    ]


print(f"Launch case detected: {CASE.name}")


def xacro_path_from_name(robot_name: str):
    xacro_path = xacro_path_from_pkg("ros2_m_hero_pkg", f"urdf/{robot_name}.xacro")
    return xacro_path


def urdf_from_name(robot_name: str, options: Optional[str] = None) -> Command:
    return command_from_xacro_path(xacro_path_from_name(robot_name), options)


class LevelBuilder(DefaultLvlBlder):
    def __init__(
        self,
        urdf: Union[str, Command],
        leg_dict: Mapping[int, Union[str, int]],
        params_overwrite: Dict[str, Any] = dict(),
    ):
        hero_params = {
            "number_of_legs": len([i for i in leg_dict.keys() if not is_wheel(i)]),
            "leg_list": [i for i in leg_dict.keys() if not is_wheel(i)],
            "ignore_limits": True,
            "speed_mode": True,
            "mvmt_update_rate": 30,
        }
        hero_params.update(deepcopy(params_overwrite))
        params_overwrite = hero_params
        leg_dict = clean_leg_dic(leg_dict)
        super().__init__(
            leg_dict=leg_dict,
            urdf=urdf,
            params_overwrite=params_overwrite,
        )
        if self.USE_SIMU:
            self.all_param["joint_remapping"] = False  # remapping for the motors
        else:
            self.all_param["joint_remapping"] = True

    def process_CLI_args(self):
        super().process_CLI_args()
        dont_use_simu: bool = USER_RVIZ_VAR.lower() in ["n", "0", "no", "false"]
        running_on_leg: bool = M_LEG.isdigit()
        if dont_use_simu or running_on_leg:  # overwrites to false if env var is false
            self.USE_SIMU = False
        self.remaplvl1 = []
        if self.USE_SIMU:
            self.remaplvl1 += RVIZ_SIMU_REMAP

    def lvl_to_launch(self):
        default = set(super().lvl_to_launch())
        specific = set(CASE.lvl_to_launch)
        intersection = default & specific
        return list(intersection)

    def make_leg_param(self, leg_index: int, ee_name: Union[None, str, int]) -> Dict:
        if ee_name is None:
            ee_name = self.legs_dict.get(leg_index)
            if ee_name is None:
                raise Exception(f"{leg_index} not in self.legs_dic")

        leg_param = super().make_leg_param(leg_index, ee_name)

        if is_wheel(leg_index):
            leg_param["add_joints"] += wheel_joint_names(leg_index)
            # start_effector is equal to end_effector, so no joint is created from urdf
            leg_param["start_effector_name"] = ee_name
            leg_param["speed_mode"] = True
            leg_param["leg_list"] = [leg_index]  # useful ??
        else:
            # just adds the 2 grippers manually
            leg_param["add_joints"] += gripper_joint_names(leg_index)

        return leg_param

    def lvl1_params(self) -> List[Dict]:
        overwriten_inplace = super().lvl1_params()
        for param in overwriten_inplace:
            param["services_to_wait"].append("driver/init")
        return overwriten_inplace

    def lvl2_params(self) -> List[Dict]:
        default = super().lvl2_params()
        return [p for p in default if not is_wheel(p["leg_number"])]

    def get_node_lvl1(self, params):
        ns = f"leg{params['leg_number']}"
        return Node(
            package=HERO_OVERLOAD_PKG,
            namespace=ns,
            executable="lvl1",
            name=f"lvl1",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[params],
            remappings=self.remaplvl1,
        )

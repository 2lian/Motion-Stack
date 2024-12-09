"""
Creates launch files for moonbot hero configurations, working in RVIZ and Reality
"""

import dataclasses
from os import environ
from time import sleep
from typing import Any, Dict, Iterable, List, Optional, Union

import numpy as np
from easy_robot_control.launch.default_params import (
    RVIZ_SIMU_REMAP,
    THIS_PACKAGE_NAME,
    default_params,
    enforce_params_type,
    get_xacro_path,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.substitutions import Command


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
USE_RVIZ = (USER_RVIZ_VAR in [None, "", "None", "TRUE"]) and (
    M_LEG in ["NOTHING", None, "", "None", "ALL"]
)

CASE = CASES[MOONBOT_PC_NUMBER]

remaplvl1 = []
if USE_RVIZ:
    remaplvl1 = RVIZ_SIMU_REMAP


def clean_leg_dic(legs_dic: Dict[int, Union[str, int]]) -> Dict[int, Union[str, int]]:
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


def change_param_on_envar(param: Dict) -> Dict:
    """changes the default param based on the env variables"""
    param = param.copy()
    if CASE.name in [ALL, NOTH]:
        param["speed_mode"] = False
    else:
        param["speed_mode"] = True

    if USE_RVIZ:
        param["pure_topic_remap"] = False
        param["speed_mode"] = False
    else:
        param["pure_topic_remap"] = True
    return param


print(f"Launch case detected: {CASE.name}")


class LevelBuilder:
    def __init__(self, robot_name: str, legs_dic: Dict[int, Union[str, int]]):
        self.name = robot_name
        hero7dof = "hero_7dof"  # just to get the file path
        hero7dof_path = get_xacro_path(hero7dof)
        self.xacro_path = (
            hero7dof_path[: -len(hero7dof + ".xacro")] + f"{self.name}.xacro"
        )
        print(self.xacro_path)

        self.legs_dic = clean_leg_dic(legs_dic)

        self.all_param = change_param_on_envar(
            default_params
        )  # params loaded from default_params
        overwrite_default = {
            "robot_name": self.name,
            "urdf_path": self.xacro_path,
            "number_of_legs": len([i for i in self.legs_dic.keys() if not is_wheel(i)]),
            "leg_list": [i for i in self.legs_dic.keys() if not is_wheel(i)],
            "start_coord": [0, 0, 0],
            "start_effector_name": "base_link",
            "speed_mode": True,
            "ignore_limits": True,
            "limit_margin": 0.0,
        }
        self.all_param.update(overwrite_default)
        enforce_params_type(self.all_param)
        self.lvl_to_launch = CASE.lvl_to_launch

        self.remaplvl1 = []
        if USE_RVIZ:
            self.remaplvl1 = RVIZ_SIMU_REMAP

    def make_leg_param(self, leg_index: int, ee_name: Union[None, str, int]) -> Dict:
        if ee_name is None:
            ee_name = self.legs_dic.get(leg_index)
            if ee_name is None:
                raise Exception(f"{leg_index} not in self.legs_dic")

        leg_param: Dict[str, Any] = self.all_param.copy()

        leg_param["leg_number"] = leg_index
        leg_param["end_effector_name"] = str(ee_name)
        if is_wheel(leg_index):
            leg_param["add_joints"] = [  # manually adds 2 joints
                f"{leg_index}wheel_left_joint",  # name of the wheel joint in urdf
                f"{leg_index}wheel_right_joint",
            ]
            # start_effector is equal to end_effector, so no joint is created from urdf
            leg_param["start_effector_name"] = ee_name
            leg_param["speed_mode"] = True
            leg_param["leg_list"] = [leg_index]  # useful ??

        else:
            # just adds the 2 grippers manually
            leg_param["add_joints"] = [
                f"leg{leg_index}grip1",
                f"leg{leg_index}grip2",
            ]

        is_very_first_leg = leg_index == list(self.legs_dic.keys())[0]
        if not is_very_first_leg:
            # only one leg should set and handle the body coords
            # otherwise several legs will fight
            leg_param["start_coord"] = [np.nan, np.nan, np.nan]
        if not USE_RVIZ:
            # if we don't use Rviz this coordinate publishing is useless.
            # It is only for visuals
            leg_param["start_coord"] = [np.nan, np.nan, np.nan]
        return leg_param

    def lvl1_params(self) -> List[Dict]:
        return [self.make_leg_param(k, v) for k, v in self.legs_dic.items()]

    def lvl2_params(self) -> List[Dict]:
        # same as lvl1 but we don't want the wheels
        return [
            self.make_leg_param(k, v) for k, v in self.legs_dic.items() if not is_wheel(k)
        ]

    def lvl3_params(self) -> List[Dict]:
        return self.lvl2_params()  # same

    def lvl4_params(self) -> List[Dict]:
        return [self.all_param]

    def lvl5_params(self) -> List[Dict]:
        return self.lvl4_params()

    def lvl1(self) -> List[Node]:
        if 1 not in self.lvl_to_launch:
            return []
        compiled_xacro = Command([f"xacro ", self.xacro_path], on_stderr="ignore")
        node_list = []
        for param in self.lvl1_params():
            ns = f"leg{param['leg_number']}"
            node_list.append(
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    namespace=ns,
                    arguments=["--ros-args", "--log-level", "warn"],
                    parameters=[
                        {
                            "robot_description": ParameterValue(
                                compiled_xacro, value_type=str
                            ),
                        }
                    ],
                    remappings=[
                        # (intside node, outside node),
                        # ("/joint_states", "/rviz_commands"),
                        ("joint_states", "joint_read"),
                        # ("robot_description", description_topic),
                    ],  # will listen to joint_command not joint_state
                    # not tested with multi robot, will break
                    # arguments=[urdf],
                ),
            )
            node_list.append(
                Node(
                    package=HERO_OVERLOAD_PKG,
                    namespace=ns,
                    executable="lvl1",
                    name=f"joint_node",
                    arguments=["--ros-args", "--log-level", "info"],
                    emulate_tty=True,
                    output="screen",
                    parameters=[param],
                    remappings=self.remaplvl1,
                )
            )
        return node_list

    def lvl2(self) -> List[Node]:
        if 2 not in self.lvl_to_launch:
            return []
        node_list = []
        for param in self.lvl2_params():
            node_list.append(
                Node(
                    package=THIS_PACKAGE_NAME,
                    namespace=f"leg{param['leg_number']}",
                    executable="ik_heavy_node",
                    name=f"ik",
                    arguments=["--ros-args", "--log-level", "info"],
                    emulate_tty=True,
                    output="screen",
                    parameters=[param],
                    remappings=[],
                )
            )
        return node_list

    def lvl3(self) -> List[Node]:
        if 3 not in self.lvl_to_launch:
            return []
        node_list = []
        for param in self.lvl3_params():
            node_list.append(
                Node(
                    package=THIS_PACKAGE_NAME,
                    namespace=f"leg{param['leg_number']}",
                    executable="leg_node",
                    name=f"leg",
                    arguments=["--ros-args", "--log-level", "info"],
                    emulate_tty=True,
                    output="screen",
                    parameters=[param],
                    remappings=[],
                )
            )
        return node_list

    def lvl4(self) -> List[Node]:
        if 4 not in self.lvl_to_launch:
            return []
        node_list = []
        for param in self.lvl4_params():
            node_list.append(
                Node(
                    package=THIS_PACKAGE_NAME,
                    namespace=f"",
                    executable="mover_node",
                    name=f"mover",
                    arguments=["--ros-args", "--log-level", "info"],
                    emulate_tty=True,
                    output="screen",
                    parameters=[param],
                    remappings=[],
                )
            )
        return node_list

    def lvl5(self) -> List[Node]:
        if 5 not in self.lvl_to_launch:
            return []
        node_list = []
        for param in self.lvl5_params():
            node_list.append(
                Node(
                    package=THIS_PACKAGE_NAME,
                    namespace=f"",
                    executable="keygait_node",
                    name=f"keygait_node",
                    arguments=["--ros-args", "--log-level", "info"],
                    emulate_tty=True,
                    output="screen",
                    parameters=[param],
                    remappings=[],
                )
            )
        return node_list

    def make_levels(self) -> List[List[Node]]:
        return [
            self.lvl1(),
            self.lvl2(),
            self.lvl3(),
            self.lvl4(),
            self.lvl5(),
        ]

    def make_description(self) -> LaunchDescription:
        return LaunchDescription([x for xs in self.make_levels()[:4] for x in xs])

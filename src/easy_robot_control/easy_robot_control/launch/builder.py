"""
Creates launch files for moonbot hero configurations, working in RVIZ and Reality
"""

import dataclasses
import sys
from os import environ
from typing import Any, Dict, Iterable, List, Mapping, Optional, Union

import numpy as np
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from easy_robot_control.launch.default_params import (
    RVIZ_SIMU_REMAP,
    default_params,
    enforce_params_type,
    get_xacro_path,
)
from launch.launch_description import LaunchDescription
from launch.substitutions import Command

class LevelBuilder:
    def __init__(
        self,
        robot_name: str,
        leg_dict: Mapping[int, Union[str, int]],
        params_overwrite: dict[str, Any] = dict(),
    ):
        self.name = robot_name
        self.xacro_path = self.get_xacro_path()
        self.params_overwrite = params_overwrite
        self.ms_package = "easy_robot_control"

        self.legs_dict = leg_dict

        self.all_param = default_params.copy()
        self.generate_global_params()
        enforce_params_type(self.all_param)

        self.down_from = 1
        # for arg in sys.argv:
        #     if "MS_down_from_level:=" in arg:
        #         self.down_from = int(arg.split(":=")[1])
        #         break

        self.up_to = 4
        # for arg in sys.argv:
        #     if "MS_up_to_level:=" in arg:
        #         self.up_to = int(arg.split(":=")[1])
        #         break

        self.USE_RVIZ = True
        self.remaplvl1 = []
        if self.USE_RVIZ:
            self.remaplvl1 = RVIZ_SIMU_REMAP

    def lvl_to_launch(self):
        return list(range(self.down_from, self.up_to + 1))

    def get_xacro_path(self):
        p = get_xacro_path(self.name)
        print(p)
        return p

    def generate_global_params(self):
        self.all_param = default_params
        overwrite_default = {
            "robot_name": self.name,
            "urdf_path": self.xacro_path,
            "number_of_legs": len([i for i in self.legs_dict.keys()]),
            "leg_list": [i for i in self.legs_dict.keys()],
        }
        self.all_param.update(overwrite_default)
        self.all_param.update(self.params_overwrite)

    def make_leg_param(self, leg_index: int, ee_name: Union[None, str, int]) -> Dict:
        if ee_name is None:
            ee_name = self.legs_dict.get(leg_index)
            if ee_name is None:
                raise Exception(f"{leg_index} not in self.legs_dic")

        leg_param: Dict[str, Any] = self.all_param.copy()

        leg_param["leg_number"] = leg_index
        leg_param["end_effector_name"] = str(ee_name)

        is_very_first_leg = leg_index == list(self.legs_dict.keys())[0]
        if not is_very_first_leg:
            # only one leg should set and handle the body coords
            # otherwise several legs will fight
            # behavior to be updated and deleted
            leg_param["start_coord"] = [np.nan, np.nan, np.nan]
        if not self.USE_RVIZ:
            # if we don't use Rviz this coordinate publishing is useless.
            # It is only for visuals
            leg_param["start_coord"] = [np.nan, np.nan, np.nan]
        return leg_param

    def lvl1_params(self) -> List[Dict]:
        return [self.make_leg_param(k, v) for k, v in self.legs_dict.items()]

    def lvl2_params(self) -> List[Dict]:
        # same as lvl1 but we don't want the wheels
        return [self.make_leg_param(k, v) for k, v in self.legs_dict.items()]

    def lvl3_params(self) -> List[Dict]:
        return self.lvl2_params()  # same

    def lvl4_params(self) -> List[Dict]:
        return [self.all_param]

    def lvl5_params(self) -> List[Dict]:
        return self.lvl4_params()

    def state_publisher_lvl1(self) -> List[Node]:
        if 1 not in self.lvl_to_launch():
            return []
        compiled_xacro = Command([f"xacro ", self.xacro_path])
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
                        ("joint_states", "joint_read"),
                    ],
                ),
            )
        return node_list

    def get_node_lvl1(self, params):
        ns = f"leg{params['leg_number']}"
        return Node(
            package=self.ms_package,
            namespace=ns,
            executable="joint_node",
            name=f"joint_node",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[params],
            remappings=self.remaplvl1,
        )

    def get_node_lvl2(self, params):
        ns = f"leg{params['leg_number']}"
        return Node(
            package=self.ms_package,
            namespace=ns,
            executable="ik_heavy_node",
            name=f"ik",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[params],
            remappings=[],
        )

    def get_node_lvl3(self, params):
        ns = f"leg{params['leg_number']}"
        return Node(
            package=self.ms_package,
            namespace=ns,
            executable="leg_node",
            name=f"leg",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[params],
            remappings=[],
        )

    def get_node_lvl4(self, params):
        return Node(
            package=self.ms_package,
            executable="mover_node",
            name=f"mover",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[params],
            remappings=[],
        )

    def get_node_lvl5(self, params):
        return Node(
            package=self.ms_package,
            executable="gait_node",
            name=f"gait",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[params],
            remappings=[],
        )

    def lvl1(self) -> List[Node]:
        if 1 not in self.lvl_to_launch():
            return []
        node_list = []
        for param in self.lvl1_params():
            node_list.append(self.get_node_lvl1(param))
        return node_list

    def lvl2(self) -> List[Node]:
        if 2 not in self.lvl_to_launch():
            return []
        node_list = []
        for param in self.lvl2_params():
            node_list.append(self.get_node_lvl2(param))
        return node_list

    def lvl3(self) -> List[Node]:
        if 3 not in self.lvl_to_launch():
            return []
        node_list = []
        for param in self.lvl3_params():
            node_list.append(self.get_node_lvl3(param))
        return node_list

    def lvl4(self) -> List[Node]:
        if 4 not in self.lvl_to_launch():
            return []
        node_list = []
        for param in self.lvl4_params():
            node_list.append(self.get_node_lvl4(param))
        return node_list

    def lvl5(self) -> List[Node]:
        if 5 not in self.lvl_to_launch():
            return []
        node_list = []
        for param in self.lvl5_params():
            node_list.append(self.get_node_lvl5(param))
        return node_list

    def make_levels(self) -> List[List[Node]]:
        return [
            self.lvl1() + self.state_publisher_lvl1(),
            self.lvl2(),
            self.lvl3(),
            self.lvl4(),
            self.lvl5(),
        ]

    def make_description(
        self, levels: Optional[List[List[Node]]] = None
    ) -> LaunchDescription:
        if levels is None:
            levels = self.make_levels()
        return LaunchDescription([x for xs in levels for x in xs])

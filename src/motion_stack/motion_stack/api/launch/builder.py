"""
API to generate launch files
"""

import sys
from copy import deepcopy
from os.path import join
from typing import Any, Dict, Iterable, List, Mapping, Optional, TypeVar, Union

import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.launch_description import LaunchDescription
from launch.substitutions import Command

from .default_params import RVIZ_SIMU_REMAP, default_params, enforce_params_type

T = TypeVar("T")


class LevelBuilder:
    """Builds a launcher for the motion stack generating your nodes

    Note:
        This class is meant to be overloaded and changed for your robot.
        Refere to :ref:`launch-api-label`

    Args:
        robot_name: Name of your robot URDF
        leg_dict: Dictionary linking leg number to end effector name.\
                This informs the API of the number of legs and nodes to launch.
        params_overwrite: Will overwrite the default parameters

    Example::

        from easy_robot_control.launch.builder import LevelBuilder
        ROBOT_NAME = "moonbot_7"  # name of the xacro to load
        LEGS_DIC = {
            1: "end1",
            2: "end2",
            3: "end3",
            4: "end4",
        }
        lvl_builder = LevelBuilder(robot_name=ROBOT_NAME, leg_dict=LEGS_DIC)
        def generate_launch_description():
            return lvl_builder.make_description()
    """

    OLD_PKG = "easy_robot_control"
    MS_PACKAGE = "motion_stack"

    def __init__(
        self,
        urdf_path: str,
        leg_dict: Mapping[int, Union[str, int]],
        params_overwrite: Dict[str, Any] = dict(),
        urdf: Union[None, str, Command] = None,
    ):
        """"""
        self.name = "robotTODO"
        self.xacro_path = urdf_path
        if urdf is None:
            urdf = command_from_xacro_path(urdf_path)
        self.xacro_cmd: Union[str, Command] = urdf
        self.params_overwrite = deepcopy(params_overwrite)

        self.legs_dict = leg_dict

        self.all_param = default_params.copy()
        self.generate_global_params()
        self.process_CLI_args()
        enforce_params_type(self.all_param)

    def make_description(
        self, levels: Optional[List[List[Node]]] = None
    ) -> LaunchDescription:
        """Return the launch description for ros2

        Example::

            def generate_launch_description():
                return lvl_builder.make_description()


        Args:
            levels:
                list of levels, levels being a list of nodes to be launched

        Returns:
            launch description to launch all the nodes
        """
        if levels is None:
            levels = self.make_levels()
        return LaunchDescription(
            [x for xs in levels for x in xs],  # flattens the list
        )

    def process_CLI_args(self):
        self.down_from: int = int(get_cli_argument("MS_down_from_level", 1))
        self.up_to: int = int(get_cli_argument("MS_up_to_level", 4))
        # True by default, become false only if a keyword in this list is used
        self.USE_SIMU: bool = get_cli_argument("MS_simu_mode", "True").lower() not in [
            "false",
            "n",
            "no",
            "0",
        ]

        self.remaplvl1 = []
        if self.USE_SIMU:
            self.remaplvl1 += RVIZ_SIMU_REMAP

    def lvl_to_launch(self) -> List[int]:
        """
        Returns:
            List of int corresponding to the motion stack levels to start

        """
        return list(range(self.down_from, self.up_to + 1))

    def generate_global_params(self):
        """Generates parameters shared by all nodes.
        Based on the default params and overwrites.

        .. Note:
            stores it in self.all_param
        """
        self.all_param = default_params
        overwrite_default = {
            "robot_name": self.name,
            "urdf_path": self.xacro_path,
            "urdf": self.xacro_cmd,
            # will be overwritten later VVV
            "number_of_legs": len([i for i in self.legs_dict.keys()]),
            "leg_list": [i for i in self.legs_dict.keys()],
        }
        self.all_param.update(overwrite_default)
        self.all_param.update(self.params_overwrite)

    def make_leg_param(self, leg_index: int, ee_name: Union[None, str, int]) -> Dict:
        """Based on the leg index/number (and end-effector), returns the parameters corresponding to the leg

        Args:
            leg_index: leg number to create the params for
            ee_name: (Optional) end effector name or number

        Raises:
            Exception:
                Exception(f"{leg_index} not in self.legs_dic")

        Returns:
            Dictionary of ROS2 parameters

        """
        if ee_name is None:
            ee_name = self.legs_dict.get(leg_index)
            if ee_name is None:
                raise Exception(f"{leg_index} not in self.legs_dic")

        leg_param: Dict[str, Any] = deepcopy(self.all_param)

        leg_param["leg_number"] = leg_index
        leg_param["end_effector_name"] = str(ee_name)

        is_very_first_leg = leg_index == list(self.legs_dict.keys())[0]
        if not is_very_first_leg:
            # only one leg should set and handle the body coords
            # otherwise several legs will fight
            # behavior to be updated and deleted
            leg_param["start_coord"] = [np.nan, np.nan, np.nan]
        if not self.USE_SIMU:
            # if we don't use Rviz this coordinate publishing is useless.
            # It is only for visuals
            leg_param["start_coord"] = [np.nan, np.nan, np.nan]
        return leg_param

    def lvl1_params(self) -> List[Dict]:
        """Returns parameters for all lvl1 nodes to be launched

        Returns:
            List of ros2 parameter dictionary, one per node.
        """
        all_params = [self.make_leg_param(k, v) for k, v in self.legs_dict.items()]
        for param in all_params:
            param["services_to_wait"] = ["rviz_interface_alive"]
        return all_params

    def lvl2_params(self) -> List[Dict]:
        """Returns parameters for all lvl2 nodes to be launched

        Returns:
            List of ros2 parameter dictionary, one per node.
        """
        all_params = self.lvl1_params()
        for param in all_params:
            param["services_to_wait"] = ["joint_alive"]
        return all_params

    def lvl3_params(self) -> List[Dict]:
        """Returns parameters for all lvl3 nodes to be launched

        Returns:
            List of ros2 parameter dictionary, one per node.
        """
        all_params = self.lvl2_params()
        for param in all_params:
            param["services_to_wait"] = ["ik_alive"]
        return all_params

    def lvl4_params(self) -> List[Dict]:
        """Returns parameters for all lvl4 nodes to be launched

        Returns:
            List of ros2 parameter dictionary, one per node.
        """
        all_params = [deepcopy(self.all_param)]
        lvl3 = self.lvl3_params()
        all_leg_ind = [n["leg_number"] for n in lvl3]
        for param in all_params:
            param["leg_list"] = all_leg_ind
            param["number_of_legs"] = len(all_leg_ind)
            param["services_to_wait"] = [f"leg{n}/leg_alive" for n in param["leg_list"]]
        return all_params

    def lvl5_params(self) -> List[Dict]:
        """Returns parameters for all lvl5 nodes to be launched

        Returns:
            List of ros2 parameter dictionary, one per node.
        """
        all_params = self.lvl4_params()
        for param in all_params:
            param["services_to_wait"] = ["mover_alive"]
        return all_params

    def state_publisher_lvl1(self) -> List[Node]:
        """.. _builder-state-label:

        Prepares all nodes meant to publish the state of the robot to external tools.

        Along each lvl1 node, it creates:
            - One (customized) joint_state_publisher continuously publishing joint angles
            - One robot_state_publisher continuously publishing robot TF and description.

        Returns:
            List of Nodes to be launched (empty if lvl1 is not to be launched)
        """
        if 1 not in self.lvl_to_launch():
            return []
        compiled_xacro = Command([f"xacro ", self.xacro_path])
        node_list = []
        for param in self.lvl1_params():
            ns = f"leg{param['leg_number']}"
            node_list.append(
                Node(
                    package=self.OLD_PKG,
                    executable="joint_state_publisher",
                    name="joint_state_publisher",
                    namespace=ns,
                    arguments=["--ros-args", "--log-level", "warn"],
                    parameters=[
                        {
                            "source_list": ["joint_read"],
                            "publish_default_positions": True,
                        }
                    ],
                    remappings=[
                        # (intside node, outside node),
                        ("joint_states", "continuous_joint_read"),
                    ],
                ),
            )
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
                        ("joint_states", "continuous_joint_read"),
                    ],
                ),
            )
        return node_list

    def get_node_lvl1(self, params: Dict[str, Any]) -> Node:
        ns = f"leg{params['leg_number']}"
        return Node(
            package=self.MS_PACKAGE,
            namespace=ns,
            executable="lvl1",
            name=f"lvl1",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[params],
            remappings=self.remaplvl1,
        )

    def get_node_lvl2(self, params: Dict[str, Any]) -> Node:
        ns = f"leg{params['leg_number']}"
        return Node(
            package=self.OLD_PKG,
            namespace=ns,
            executable="ik_heavy_node",
            name=f"ik",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[params],
            remappings=[],
        )

    def get_node_lvl3(self, params: Dict[str, Any]) -> Node:
        ns = f"leg{params['leg_number']}"
        return Node(
            package=self.OLD_PKG,
            namespace=ns,
            executable="leg_node",
            name=f"leg",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[params],
            remappings=[],
        )

    def get_node_lvl4(self, params: Dict[str, Any]) -> Node:
        return Node(
            package=self.OLD_PKG,
            executable="mover_node",
            name=f"mover",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[params],
            remappings=[],
        )

    def get_node_lvl5(self, params: Dict[str, Any]) -> Node:
        return Node(
            package=self.OLD_PKG,
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


def xacro_path_from_packer(robot_name: str):
    """retieves the .urdf/.xacro path in the install share folder of urdf_packer.

    Args:
        robot_name: corresponds to <robot_name>.xacro

    Returns:

    """
    ROS2_PACKAGE_WITH_URDF = "urdf_packer"

    return join(
        get_package_share_directory(ROS2_PACKAGE_WITH_URDF),
        "urdf",
        f"{robot_name}",
        f"{robot_name}.xacro",
    )


def xacro_path_from_pkg(package_name: str, xacro_path: str):
    return join(
        get_package_share_directory(package_name),
        xacro_path,
    )


def command_from_xacro_path(path) -> Command:
    return Command([f"xacro ", path])


def get_cli_argument(arg_name: str, default: T) -> Union[T, str]:
    """Returns the CLI argument as a string, or default is none inputed.
    Can be much better optimised, I don't care.
    """
    for arg in sys.argv:
        if f"{arg_name}:=" in arg:
            return arg.split(":=")[1]
    return default

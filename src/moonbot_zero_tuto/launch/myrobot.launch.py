import os
from typing import Any, Dict, List, Mapping, Union

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from motion_stack.api.launch.builder import LevelBuilder, xacro_path_from_pkg

from launch.substitutions import Command


class MyLevelBuilder(LevelBuilder):
    def __init__(
        self,
        urdf_path: str,
        leg_dict: Mapping[int, Union[str, int]],
        params_overwrite: Dict[str, Any] = dict(),
        urdf: Union[None, str, Command] = None,
    ):
        # gets the "COMPUTER_ID" environement variable
        self.COMPUTER_ID = os.environ.get("COMPUTER_ID")
        if self.COMPUTER_ID in ["leg1", "leg2", "leg3", "leg4"]:
            # if running on one of the leg computer
            # we only start the assiciated leg/end-effector
            leg_number = int(self.COMPUTER_ID[-1])
            end_effector: Union[str, int, None] = leg_dict.get(leg_number)
            if end_effector is None:
                raise Exception("leg number has no entry in leg_dict")
            reduced_leg_dict = {leg_number: end_effector}
            leg_dict = reduced_leg_dict
        super().__init__(urdf_path, leg_dict, params_overwrite, urdf)

    def make_levels(self) -> List[List[Node]]:
        if self.COMPUTER_ID in ["leg1", "leg2", "leg3", "leg4"]:
            # if running on one of the leg computer
            # we only start lvl1
            return [self.lvl1()]
        if self.COMPUTER_ID == "robot_brain":
            # if running on the main robot computer
            # we start lvl2-3-4
            return [self.lvl2(), self.lvl3(), self.lvl4()]
        if self.COMPUTER_ID == "ground_station":
            # if running on the ground station
            # we start only lvl5
            return [self.lvl5()]
        # if none of the previous cases, the default behavior runs everything
        return super().make_levels()

    def state_publisher_lvl1(self) -> List[Node]:
        compiled_xacro = Command([f"xacro ", self.xacro_path])
        node_list = []
        leg_namespaces = [f"leg{param['leg_number']}" for param in self.lvl1_params()]
        all_joint_read_topics = [f"{ns}/joint_read" for ns in leg_namespaces]
        node_list.append(
            Node(
                package=self.MS_PACKAGE,
                executable="lazy_joint_state_publisher",
                name="lazy_joint_state_publisher",
                # namespace=ns,
                arguments=["--ros-args", "--log-level", "warn"],
                parameters=[
                    {
                        "source_list": all_joint_read_topics,
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
                # namespace=ns,
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

    # def get_node_lvl1(self, params):
    #     ns = f"leg{params['leg_number']}"
    #     return Node(
    #         package="moonbot_zero_tuto",
    #         namespace=ns,
    #         executable="lvl1",
    #         name=f"lvl1",
    #         arguments=["--ros-args", "--log-level", "info"],
    #         emulate_tty=True,
    #         output="screen",
    #         parameters=[params],
    #         remappings=self.remaplvl1,
    #     )




ROBOT_NAME = "moonbot_7"  # name of the xacro to load

LEGS_DIC = {
    1: "end1",
    2: "end2",
    3: "end3",
    4: "end4",
}

new_params = {
    "std_movement_time": 10.0,
}

lvl_builder = MyLevelBuilder(
    urdf_path=xacro_path_from_pkg(
        package_name="moonbot_zero_tuto", xacro_path="urdf/moonbot_zero.xacro"
    ),
    leg_dict=LEGS_DIC,
    params_overwrite=new_params,
)


def generate_launch_description():
    return lvl_builder.make_description()

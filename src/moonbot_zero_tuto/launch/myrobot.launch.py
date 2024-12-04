from typing import Any, List, Mapping, Union

from easy_robot_control.launch.builder import LevelBuilder, Node, ParameterValue
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.substitutions import Command

REMAP_TO_GLOBAL_JOINT_READ = [("joint_read", "/global_joint_read")]


class MyLevelBuilder(LevelBuilder):
    def state_publisher_lvl1(self) -> List[Node]:
        compiled_xacro = Command([f"xacro ", self.xacro_path])
        node_list = []
        repeat_state_onto = "/global_joint_read"
        node_list.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace="",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            compiled_xacro, value_type=str
                        ),
                    }
                ],
                remappings=[
                    # (intside node, outside node),
                    ("joint_states", repeat_state_onto),
                ],
            ),
        )
        # for param in self.lvl1_params():
        #     leg_namespace = f"leg{param['leg_number']}"
        #     node_list.append(
        #         Node(
        #             package="topic_tools",
        #             executable="relay",
        #             name="relay",
        #             namespace=leg_namespace,
        #             parameters=[
        #                 {
        #                     "input_topic": "joint_read",
        #                     "output_topic": repeat_state_onto,
        #                 }
        #             ],
        #         ),
        #     )
        return node_list

    def get_node_lvl1(self, params):
        ns = f"leg{params['leg_number']}"
        return Node(
            package="moonbot_zero_tuto",
            namespace=ns,
            executable="lvl1",
            name=f"lvl1",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[params],
            remappings=self.remaplvl1 + REMAP_TO_GLOBAL_JOINT_READ,
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
            remappings=[] + REMAP_TO_GLOBAL_JOINT_READ,
        )


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
    robot_name=ROBOT_NAME, leg_dict=LEGS_DIC, params_overwrite=new_params
)


def generate_launch_description():
    return lvl_builder.make_description()

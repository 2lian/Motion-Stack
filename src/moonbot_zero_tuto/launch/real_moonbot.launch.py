import os
from typing import Any, Dict, List, Mapping, Optional, Union

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from motion_stack.api.launch.builder import LevelBuilder, xacro_path_from_pkg

from launch.substitutions import Command


class MyLevelBuilder(LevelBuilder):
    def get_node_lvl1(self, params):
        ns = f"leg{params['leg_number']}"
        return Node(
            package="moonbot_zero_tuto",
            namespace=ns,
            executable="lvl1_dyna",
            name=f"lvl1",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[params],
            remappings=self.remaplvl1,
        )

    def lvl1_params(self) -> List[Dict]:
        overwriten_inplace = super().lvl1_params()
        for param in overwriten_inplace:
            param["services_to_wait"].append("/remapper_alive")
        return overwriten_inplace


urdf_path = xacro_path_from_pkg(
    package_name="moonbot_zero_tuto", xacro_path="urdf/moonbot_zero.xacro"
)

LEGS_DIC = {
    1: "end1",
    2: "end2",
    3: "end3",
    4: "end4",
}

lvl_builder = MyLevelBuilder(
    urdf_path=urdf_path,
    leg_dict=LEGS_DIC,
)


def generate_launch_description():
    return lvl_builder.make_description()

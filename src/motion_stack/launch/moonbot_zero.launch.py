"""Example of a "sub launcher" or launchpy for monbot zero.
"""

from typing import Any, Dict

from launch_ros.actions import Node

from motion_stack.api.launch.builder import LevelBuilder, xacro_path_from_pkg

# V Change default parameters here V
#   \  /   #
#    \/    #
urdf_path = xacro_path_from_pkg(
    package_name="moonbot_zero_tuto", xacro_path="urdf/moonbot_zero.xacro"
)

# leg number -> end effector (number or link name)
LEGS_DIC = {
    1: "end1",
    2: "end2",
    3: "end3",
    4: "end4",
}

lvl_builder = LevelBuilder(
    urdf_path=urdf_path,
    leg_dict=LEGS_DIC,
)
#    /\    #
#   /  \   #
# ^ Change default parameters here ^


def generate_launch_description():
    return lvl_builder.make_description()

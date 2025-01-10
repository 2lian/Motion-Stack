"""Example of a "sub launcher" or launchpy for monbot zero.
"""

from typing import Any, Dict

from easy_robot_control.launch.builder import LevelBuilder as defaultLevelBuilder
from launch_ros.actions import Node


class LevelBuilder(defaultLevelBuilder):
    def get_node_lvl1(self, params: Dict[str, Any]) -> Node:
        ns = f"leg{params['leg_number']}"
        return Node(
            package="motion_stack",
            namespace=ns,
            executable="lvl1",
            name=f"lvl1",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            output="screen",
            parameters=[params],
            remappings=self.remaplvl1,
        )


# V Change default parameters here V
#   \  /   #
#    \/    #
ROBOT_NAME = "moonbot_7"  # name of the xacro to load

# leg number -> end effector (number or link name)
LEGS_DIC = {
    1: "end1",
    # 2: "end2",
    # 3: "end3",
    # 4: "end4",
}

lvl_builder = LevelBuilder(robot_name=ROBOT_NAME, leg_dict=LEGS_DIC)
#    /\    #
#   /  \   #
# ^ Change default parameters here ^


def generate_launch_description():
    return lvl_builder.make_description()

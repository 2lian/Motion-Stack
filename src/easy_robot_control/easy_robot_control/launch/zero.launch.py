"""Example of a "sub launcher" or launchpy for monbot zero.
The variables levels and params must be created. 
levels is a List[List[Node]] corresponding to each level of the stack.
params are the prameters of the stack
"""

from typing import Any, Dict, Iterable, List, Union

from launch_ros.actions import Node

from easy_robot_control.launch.builder import LevelBuilder
from easy_robot_control.launch.default_params import *


# V Change default parameters here V
#   \  /   #
#    \/    #
ROBOT_NAME = "moonbot_7"  # name of the xacro to load
xacro_path = get_xacro_path(ROBOT_NAME)

# leg number -> end effector (number or link name)
LEGS_DIC: Dict[int, Union[str, int]] = {
    1: "end1",
    2: "end2",
    3: "end3",
    4: "end4",
}

zero_builder = LevelBuilder(robot_name=ROBOT_NAME, leg_dict=LEGS_DIC)
#    /\    #
#   /  \   #
# ^ Change default parameters here ^



def generate_launch_description():
    return zero_builder.make_description()

"""Example of a "sub launcher" or launchpy for monbot zero.
"""

from easy_robot_control.launch.builder import LevelBuilder


# V Change default parameters here V
#   \  /   #
#    \/    #
ROBOT_NAME = "moonbot_7"  # name of the xacro to load

# leg number -> end effector (number or link name)
LEGS_DIC = {
    1: "end1",
    2: "end2",
    3: "end3",
    4: "end4",
}

lvl_builder = LevelBuilder(robot_name=ROBOT_NAME, leg_dict=LEGS_DIC)
#    /\    #
#   /  \   #
# ^ Change default parameters here ^



def generate_launch_description():
    return lvl_builder.make_description()

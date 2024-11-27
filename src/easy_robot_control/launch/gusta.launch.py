"""Example of a "sub launcher" or launchpy for monbot zero.
The variables levels and params must be created. 
levels is a List[List[Node]] corresponding to each level of the stack.
params are the prameters of the stack
"""

from easy_robot_control.launch.builder import LevelBuilder

# V Change default parameters here V
#   \  /   #
#    \/    #
ROBOT_NAME = "mglimb_7dof"  # name of the xacro to load

# leg number -> end effector (number or link name)
LEGS_DIC = {
    1: 0,
}

lvl_builder = LevelBuilder(robot_name=ROBOT_NAME, leg_dict=LEGS_DIC)
#    /\    #
#   /  \   #
# ^ Change default parameters here ^


def generate_launch_description():
    return lvl_builder.make_description()

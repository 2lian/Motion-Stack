from os import environ
from os.path import join
from typing import Dict, List, Tuple

from ament_index_python.packages import get_package_share_directory

PkgName = str
LaunchFileName = str
NameSpaceName = str
LaunchPyName = str
# ^ TYPES ^ #

# OS variables can be used to change what is launched.
# this is very usefull when you have several robots/systems.
# This can also be done in the LAUNCHPY you should create for your own robot's motion stack
# settings here go beyond the motion stack, immpacting rviz and other ...
USER_RVIZ_VAR = str(environ.get("USE_RVIZ"))
M_LEG = str(environ.get("M_LEG"))  # leg number saved on real robot os

# if none of those 2 environment vairable are defined, I assume we are not on the robot
# so we launch the interface for rviz
launch_rviz = (USER_RVIZ_VAR in [None, "", "None", "TRUE"]) and (
    M_LEG in ["NOTHING", None, "", "None", "ALL"]
)

if launch_rviz:
    rviz_interface = [("rviz_basic", "rviz.launch.py")]
else:
    rviz_interface = []

# node of levels up to (and including) this one will launched
# lvl 5 makes the robot move immediately, use lvl 4 to avoid that
LAUNCH_UP_TO_LVL: int = 4
LAUNCH_FROM_LVL: int = 1


LAUNCHPY_INDEX: int = 11  # number corresponding to LAUNCHPY_D will be used

# all of my launch setting for my robots, add yours here
LAUNCHPY_D: Dict[int, LaunchPyName] = {
    0: "moonbot_zero",  # example
    9: "mglimb_7dof",
    10: "hero_all",
    11: "ur16_grip",
    12: "hero_3leg",
    13: "hero_3legwheel",
    14: "hero_dragon",
    15: "hero_vehicle",
}

INTERFACES: List[Tuple[PkgName, LaunchFileName]] = (
    [] + rviz_interface
)  # These external launch files will also be run

# namespaces of the robot(s)
# it should handle launching several similar robots on different namespaces, but I haven't tested the feature in a while
NAMESPACES: List[NameSpaceName] = [""]

MOTION_STACK_PKG_NAME: PkgName = "easy_robot_control"
PKG_WITH_LAUNCHER = MOTION_STACK_PKG_NAME
LAUNCHPY: LaunchPyName = LAUNCHPY_D[LAUNCHPY_INDEX]
# All nodes and parameters will be loaded from this f"src/easy_robot_control/launch/{LAUNCHPY}.py"
node_maker = join(
    get_package_share_directory(MOTION_STACK_PKG_NAME),
    "launch",
    f"{LAUNCHPY}.py",
)

LIST_OF_LVL = range(LAUNCH_FROM_LVL - 1, LAUNCH_UP_TO_LVL)

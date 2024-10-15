from os import environ
from typing import Dict, List, Tuple

PkgName = str
LaunchFileName = str
NameSpaceName = str
LaunchPyName = str
# ^ TYPES ^ #

# to avoid using rviz mode on the robot, I use `export USE_RVIZ="TRUE"` on my sim/debug PC
# If you work with the moonbot launchers, it also change behaviors 
# based on this environment setting
USER_RVIZ_VAR = str(environ.get("USE_RVIZ"))  # leg number saved on lattepanda
M_LEG = str(environ.get("M_LEG"))  # leg number saved on real robot

# set this launch_rviz=True or run `export USE_RVIZ="TRUE" to use Rviz
launch_rviz = (USER_RVIZ_VAR == "TRUE") and (
    M_LEG in ["NOTHING", None, "", "None", "ALL"]
)
if launch_rviz:
    rviz_interface = [("rviz_basic", "rviz.launch.py")]
else:
    rviz_interface = []

LAUNCHERPY_INDEX: int = (
    14  # the settings corresponding to this number in LAUNCHPY_D will be used
)

# node of levels up to (and including) this one will launched
# lvl 5 makes the robot move immediately, use lvl 4 to avoid that
LAUNCH_UP_TO_LVL: int = 5
LAUNCH_FROM_LVL: int = 1

INTERFACES: List[Tuple[PkgName, LaunchFileName]] = (
    [] + rviz_interface
)  # These external launch files will also be run

# namespaces of the robot(s)
# it should handle launching several similar robots on different namespaces, but I haven't tested the feature in a while
NAMESPACES: List[NameSpaceName] = [""]

# all of my launch setting for my robots, add yours here
LAUNCHPY_D: Dict[int, LaunchPyName] = {
    # 1: "moonbot_7",
    # 2: "moonbot_45",
    # 3: "moonbot_hero",
    # 4: "moonbot_hero2",
    # 5: "hero_3wheel_1hand",
    # 6: "moonbot_hero3",
    # 7: "gleg_3dof",
    8: "moonbot_hero_onewheel",
    # 9: "mglimb_7dof",
    10: "hero_7dof_real",
    11: "hero_7dof_rviz",
    12: "hero_7dof_base_dual",
    13: "ur16_grip",
    14: "hero_7dof_dragon",
}

# All nodes and parameters will be loaded from this f"src/easy_robot_control/launch/{LAUNCHPY}.py"
# then launched by /launch_stack_rviz.launch.py
LAUNCHPY: LaunchPyName = LAUNCHPY_D[LAUNCHERPY_INDEX]

MOTION_STACK_PKG_NAME: PkgName = "easy_robot_control"
LIST_OF_LVL = range(LAUNCH_FROM_LVL - 1, LAUNCH_UP_TO_LVL)

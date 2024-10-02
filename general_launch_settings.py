from typing import Dict, List, Tuple
PkgName = str
LaunchFileName = str
NameSpaceName = str
RobotName = str
# ^ TYPES ^ #


ROBOT_INDEX_TO_USE: int = 8  # corresponding to the ROBOTS below
# node of levels up to (and including) this one will launched
# lvl 5 makes the robot move immediately, use lvl 4 to avoid that
LAUNCH_UP_TO_LVL: int = 5

INTERFACES: List[Tuple[PkgName, LaunchFileName]] = [
    # ("rviz_basic", "rviz.launch.py"),
]  # Additional launchfiles mainly for Rviz, Sim or motor interfaces

# namespaces of the robot(s)
# it should handle launching several similar robots on different namespaces, but I haven't tested the feature in a while
NAMESPACES: List[NameSpaceName] = [""]

# all of my robots, you can add yours if you want
ROBOTS: Dict[int, RobotName] = {
    1: "moonbot_7",
    2: "moonbot_45",
    3: "moonbot_hero",
    4: "moonbot_hero2",
    5: "hero_3wheel_1hand",
    6: "moonbot_hero3",
    7: "gleg_3dof",
    8: "moonbot_hero_onewheel",
    9: "mglimb_7dof",
}
# The robot  name parameters passed to all the nodes
ROBOT_NAME: RobotName = ROBOTS[ROBOT_INDEX_TO_USE]

MOTION_STACK_PKG_NAME: PkgName = "easy_robot_control"
MOTION_STACK_LEVEL_LAUNCHERS: Dict[int, LaunchFileName] = {
    1: "lvl_01_joints.py",
    2: "lvl_02_ik.py",
    3: "lvl_03_leg.py",
    4: "lvl_04_mover.py",
    5: "lvl_05_gait.py",
}  # lauinch files corresponding to the motion stack levels

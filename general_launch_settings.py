from typing import Dict, List, Tuple


ROBOT_INDEX_TO_USE: int = 8  # corresponding to the ROBOTS below
LAUNCH_UP_TO_LVL: int = 1
# node of levels up to (and including) this one will be launched

INTERFACES: List[Tuple[str, str]] = [
    # ("rviz_basic", "rviz.launch.py"),
]

NAMESPACES: List[str] = [""]  # namespaces of the robot(s)
# NAMESPACES = [f"r{i+1}" for i in range(5)]  # use this to launch several robots
# NAMESPACES = [f"r{i+1}" for i in [0,1,2]]  # use this to launch several robots
# NAMESPACES = [f"r{i+1}" for i in [3,4]]  # use this to launch several robots
# NAMESPACES = [f"r{i+1}" for i in [5,6,7]]  # use this to launch several robots

ROBOTS: Dict[int, str] = {
    1: "moonbot_7",
    2: "moonbot_45",
    3: "moonbot_hero",
    4: "moonbot_hero2",
    5: "hero_3wheel_1hand",
    6: "moonbot_hero3",
    7: "gleg_3dof",
    8: "moonbot_hero_onewheel",
    9: "mglimb_7dof",
}  # you robot / URDF name

RobotName: str = ROBOTS[ROBOT_INDEX_TO_USE]

MOTION_STACK_PKG_NAME: str = "easy_robot_control"
MOTION_STACK_LEVEL_LAUNCHERS: Dict[int, str] = {
    1: "lvl_01_joints.py",
    2: "lvl_02_ik.py",
    3: "lvl_03_leg.py",
    4: "lvl_04_mover.py",
    5: "lvl_05_gait.py",
}  # files corresponding to the motion stack levels

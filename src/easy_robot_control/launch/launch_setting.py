from launch.launch_description import DeclareLaunchArgument
import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    PythonExpression,
)
# from ament_index_python.packages import get_package_share_directory

std_movement_time = 4  # seconds
movement_update_rate = 30.0  # Hz
number_of_legs = 4
wheel_size = 100

ROS2_PACKAGE_WITH_URDF = "rviz_basic"
ROBOT_NAME_DEFAULT = "moonbot_7"
# ROBOT_NAME_DEFAULT = "moonbot_45"
# ROBOT_NAME_DEFAULT = "moonbot_hero"
# ROBOT_NAME_DEFAULT = "hero_3wheel_1hand"
# urdf_path = (
    # get_package_share_directory(ROS2_PACKAGE_WITH_URDF)
    # + f"/urdf/{ROBOT_NAME}/{ROBOT_NAME}{URDF_OR_XACRO}"
# )
def make_xacro_path(launchArgName: str = "robot") -> PathJoinSubstitution:
    """
    Basically does this, but using ros2 parameter substitution on launch
    xacro_path = (
        get_package_share_directory(PACKAGE_NAME)
        + f"/urdf/{ROBOT_NAME}/{ROBOT_NAME}.xacro"
    )"""
    robot_name_arg = LaunchConfiguration(launchArgName, default=ROBOT_NAME_DEFAULT)
    robot_name_val = DeclareLaunchArgument(launchArgName, default_value=ROBOT_NAME_DEFAULT)

    xacro_file_path = PathJoinSubstitution(
        [
            get_package_share_directory(ROS2_PACKAGE_WITH_URDF),
            "urdf",
            robot_name_arg,
            PythonExpression(["'", robot_name_arg, ".xacro'"]),
        ]
    )
    return xacro_file_path

xacro_path = make_xacro_path()





class LegParameters:
    """
    blank object holding standard robot parameters
    """
    def __init__(self,
                 body_to_coxa_mm,
                 coxa_length_mm,
                 femur_length_mm,
                 tibia_length_mm,
                 coxaMax_degree,
                 coxaMin_degree,
                 femurMax_degree,
                 femurMin_degree,
                 tibiaMax_degree,
                 tibiaMin_degree,
                 ):
        self.bodyToCoxa = float(body_to_coxa_mm)
        self.coxaLength = float(coxa_length_mm)
        self.femurLength = float(femur_length_mm)
        self.tibiaLength = float(tibia_length_mm)

        self.coxaMax = float(np.deg2rad(coxaMax_degree))
        self.coxaMin = float(np.deg2rad(coxaMin_degree))
        self.femurMax = float(np.deg2rad(femurMax_degree))
        self.femurMin = float(np.deg2rad(femurMin_degree))
        self.tibiaMax = float(np.deg2rad(tibiaMax_degree))
        self.tibiaMin = float(np.deg2rad(tibiaMin_degree))

        self.minFemurTargetDist = 0.0
        self.update_minFemurTargetDist()

    def update_minFemurTargetDist(self):
        self.minFemurTargetDist = np.abs(self.femurLength + self.tibiaLength * np.exp(1j * self.tibiaMax))


# D1 = 0.181  # Distance between Origin of base and origin of the joint1
# L1 = 0.283 - D1  # Length between joint1 (Near the base joint) and joint2
# L2 = 0.396 - L1 - D1  # Length between joint2 and joint3 (Near the Tip Joint)
# L3 = 0.490 - (L2 + L1 + D1)  # Length between Joint3 and Tip

D1 = 0.181  # Distance between Origin of base and origin of the joint1
L1 = 0.0645 # Length between joint1 (Near the base joint) and joint2
L2 = 0.129 # Length between joint2 and joint3 (Near the Tip Joint)
L3 = 0.16  # Length between Joint3 and Tip

moonbot_leg = LegParameters(
    body_to_coxa_mm=float(D1 * 1000),
    coxa_length_mm=float(L1 * 1000),
    femur_length_mm=float(L2 * 1000),
    tibia_length_mm=float(L3 * 1000),
    coxaMax_degree=60,
    coxaMin_degree=-60,
    femurMax_degree=90,
    femurMin_degree=-97,
    tibiaMax_degree=110,
    tibiaMin_degree=-114
)


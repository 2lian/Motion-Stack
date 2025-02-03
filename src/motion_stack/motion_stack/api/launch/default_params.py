"""Provides and explains all parameters to launch the motion stack
"""

from typing import Any, Dict

default_params: Dict[str, Any] = {
    # set these in your own launcher
    #   \  /   #
    #    \/    #
    "robot_name": None,  #: (str) Name of the robot, not critical
    "urdf": None,  #: raw urdf string
    "number_of_legs": None,  # number of legs in your robot (not used by lvl 1-2-3)
    "leg_number": 0,  # number associated with a leg,
    # if serveral lvl 1-2-3 are running, it is recommanded to use different numbers
    "end_effector_name": 0,  # end effector associated with a leg, (the most important)
    # the kinematic chain used for IK will go
    # from the root link of the URDF (usually base_link)
    # to the end effector link (specified in this parameter).
    # the URDF will be parsed to find this link name. Make sure it exists.
    # you can also provide a number (as a string) instead of a link_name. If you do this
    # the Nth longest kinematic path (sequence of link where each link is connected to
    # exactly one other link) from the root of the URDF will be used for IK
    # Basically, if you use only one limb, set this as "0", and it will pick the right ee.
    "leg_list": [0],  # list of leg numbers
    #    /\    #
    #   /  \   #
    #   ----   #
    "urdf_path": "",  # path to the xacro or urdf to load
    "std_movement_time": 2,  # time lvl3 takes to execute a trajectory
    "mvmt_update_rate": 30.0,  # update rate used through out the stack
    "control_rate": 60.0,  # update rate for speed control PID only
    "start_coord": [0 / 1000, 0 / 1000, 0 / 1000],  # starting position
    # (only affects rviz for now).
    # if set to [np.nan,np.nan,np.nan], world->base_link publishing is disabled.
    # lvl1 is publishing this TF, so if you have several lvl1,
    # only one should have this opition enabled
    "add_joints": [""],  # manually adds joints for lvl1 if they are not in the urdf
    "mirror_angle": False,  # lvl1 assumes position sensor data is the last sent command
    "always_write_position": False,  # deprecated ?
    "start_effector_name": "",  # setting this manually, works with the motion stack,
    # but not for Rviz and ros2's tf, so be carefull.
    # In ros, the baselink must be the root of the tf tree, it cannot have a parent
    # and there can only be one baselink.
    # Leaving this empty and properly setting your URDF baselink is recommended.
    "wheel_size_mm": 230,  # deprecated ?
    "pure_topic_remap": False,  # activates the pure_remap.py remapping
    "speed_mode": False,  # lvl1 will send speed commands to the motors, using angle readings as feedback for a PID.
    "services_to_wait": [""],  # List of services to wait for before initializing
    "WAIT_FOR_LOWER_LEVEL": True,  # waits for services of lower level before initializing
    "ignore_limits": False,  # joint limits set in the URDF will be ignored
    "limit_margin": 0.0,  # adds a additional margin to the limits of the URDF (in rad)
}
"""
the default parameters of the motion stack


.. code-block:: python
   :linenos:

    default_params: Dict[str, Any] = {
        # set these in your own launcher
        #   \  /   #
        #    \/    #
        "robot_name": None,  #: (str) Name of the robot, not critical
        "urdf": None,  #: raw urdf string
        "number_of_legs": None,  # number of legs in your robot (not used by lvl 1-2-3)
        "leg_number": 0,  # number associated with a leg,
        # if serveral lvl 1-2-3 are running, it is recommanded to use different numbers
        "end_effector_name": 0,  # end effector associated with a leg, (the most important)
        # the kinematic chain used for IK will go
        # from the root link of the URDF (usually base_link)
        # to the end effector link (specified in this parameter).
        # the URDF will be parsed to find this link name. Make sure it exists.
        # you can also provide a number (as a string) instead of a link_name. If you do this
        # the Nth longest kinematic path (sequence of link where each link is connected to
        # exactly one other link) from the root of the URDF will be used for IK
        # Basically, if you use only one limb, set this as "0", and it will pick the right ee.
        "leg_list": [0],  # list of leg numbers
        #    /\    #
        #   /  \   #
        #   ----   #
        "urdf_path": "",  # path to the xacro or urdf to load
        "std_movement_time": 2,  # time lvl3 takes to execute a trajectory
        "mvmt_update_rate": 10.0,  # update rate used through out the stack
        "control_rate": 30.0,  # update rate for speed control PID only
        "start_coord": [0 / 1000, 0 / 1000, 0 / 1000],  # starting position
        # (only affects rviz for now).
        # if set to [np.nan,np.nan,np.nan], world->base_link publishing is disabled.
        # lvl1 is publishing this TF, so if you have several lvl1,
        # only one should have this opition enabled
        "add_joints": [""],  # manually adds joints for lvl1 if they are not in the urdf
        "mirror_angle": False,  # lvl1 assumes position sensor data is the last sent command
        "always_write_position": False,  # deprecated ?
        "start_effector_name": "",  # setting this manually, works with the motion stack,
        # but not for Rviz and ros2's tf, so be carefull.
        # In ros, the baselink must be the root of the tf tree, it cannot have a parent
        # and there can only be one baselink.
        # Leaving this empty and properly setting your URDF baselink is recommended.
        "wheel_size_mm": 230,  # deprecated ?
        "pure_topic_remap": False,  # activates the pure_remap.py remapping
        "speed_mode": False,  # lvl1 will send speed commands to the motors, using angle readings as feedback for a PID.
        "services_to_wait": [""],  # List of services to wait for before initializing
        "WAIT_FOR_LOWER_LEVEL": True,  # waits for services of lower level before initializing
        "ignore_limits": False,  # joint limits set in the URDF will be ignored
        "limit_margin": 0.0,  # adds a additional margin to the limits of the URDF (in rad)
    }
"""

THIS_PACKAGE_NAME = "easy_robot_control"
ROS2_PACKAGE_WITH_URDF = "urdf_packer"


def get_xacro_path(robot_name: str):
    """retieves the .urdf/.xacro path in the install share folder

    Args:
        robot_name: corresponds to <robot_name>.xacro

    Returns:

    """
    from os.path import join

    from ament_index_python.packages import get_package_share_directory

    return join(
        get_package_share_directory(ROS2_PACKAGE_WITH_URDF),
        "urdf",
        f"{robot_name}",
        f"{robot_name}.xacro",
    )

def enforce_params_type(parameters: Dict[str, Any]) -> None:
    """enforces types to dic in place

    Args:
        parameters: ros2 parameters dictinary
    """
    parameters["std_movement_time"] = float(parameters["std_movement_time"])
    parameters["mvmt_update_rate"] = float(parameters["mvmt_update_rate"])
    parameters["start_coord"] = [float(x) for x in parameters["start_coord"]]
    parameters["mirror_angle"] = bool(parameters["mirror_angle"])
    parameters["robot_name"] = str(parameters["robot_name"])
    parameters["urdf_path"] = str(parameters["urdf_path"])
    parameters["always_write_position"] = bool(parameters["always_write_position"])
    parameters["start_effector_name"] = str(parameters["start_effector_name"])
    parameters["end_effector_name"] = str(parameters["end_effector_name"])
    parameters["wheel_size_mm"] = float(parameters["wheel_size_mm"])
    parameters["number_of_legs"] = int(parameters["number_of_legs"])
    parameters["pure_topic_remap"] = bool(parameters["pure_topic_remap"])
    parameters["speed_mode"] = bool(parameters["speed_mode"])
    parameters["WAIT_FOR_LOWER_LEVEL"] = bool(parameters["WAIT_FOR_LOWER_LEVEL"])
    parameters["ignore_limits"] = bool(parameters["ignore_limits"])
    parameters["limit_margin"] = float(parameters["limit_margin"])
    parameters["control_rate"] = float(parameters["control_rate"])


# Rviz_simu is in global namespace so we remap the output
# of lvl1 from local namespace (=/.../something) to global namespace (=/)
RVIZ_SIMU_REMAP = [
    ("joint_states", "/joint_states"),
    ("joint_commands", "/joint_commands"),
    # ("smooth_body_rviz", "/smooth_body_rviz"),
    # ("robot_body", "/robot_body"),
    ("rviz_interface_alive", "/rviz_interface_alive"),
]

"""Provides and explains all parameters to launch the motion stack"""

from typing import Any, Dict, List, get_origin

from motion_stack.core.utils import static_executor

#: Default parameters taken from the python core.
default_params: Dict[str, Any] = {
    name: value for name, _, value in static_executor.default_param
}

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
    default_types: Dict[str, type] = {
        name: typ for name, typ, _ in static_executor.default_param
    }

    for name, typ in default_types.items():
        if not name in parameters.keys():
            continue
        if not isinstance(parameters[name], (int, str, float, bool, list)):
            continue # is a ros substitution or smthing, no touchy
        is_list = get_origin(typ) is list or get_origin(typ) is List
        if is_list:
            inner_type = static_executor.extract_inner_type(typ)
            parameters[name] = [inner_type(val) for val in parameters[name]]
        else:
            parameters[name] = typ(parameters[name])


# Rviz_simu is in global namespace so we remap the output
# of lvl1 from local namespace (=/.../something) to global namespace (=/)
RVIZ_SIMU_REMAP = [
    ("joint_states", "/joint_states"),
    ("joint_commands", "/joint_commands"),
    # ("smooth_body_rviz", "/smooth_body_rviz"),
    # ("robot_body", "/robot_body"),
    ("rviz_interface_alive", "/rviz_interface_alive"),
]

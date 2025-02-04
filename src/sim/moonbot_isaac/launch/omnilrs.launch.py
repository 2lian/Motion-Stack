from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """
    Start the interfaces necessary for the motion-stack to communicate with OmniLRS.
    """
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="Run Isaac Sim in headless mode",
    )

    #  Convert beteen Isaac and ROS joint names
    joint_state_converter = Node(
        package="moonbot_isaac",
        executable="isaac_motion_stack_interface",
        name="isaac_motion_stack_interface",
        output="screen",
    )

    interface_alive_service = Node(
        package="moonbot_isaac",
        executable="interface_alive_service",
        name="interface_alive_service",
        output="screen",
    )

    tf_ground_truth_republisher = Node(
        package="moonbot_isaac",
        executable="tf_ground_truth_republisher",
        name="tf_ground_truth_republisher",
        output="screen",
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            headless_arg,
            joint_state_converter,
            interface_alive_service,
            tf_ground_truth_republisher,
        ]
    )

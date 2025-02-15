from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    mocap_gripper = Node(
        package="moonbot_isaac",
        executable="mocap_simulator",
        name="mocap_simulator",
        output="screen",
        parameters=[{
            "ground_truth_frame": "gt__leg1gripper1",
            "mocap_frame": "mocap1gripper1",
            "offset_translation": [0.0, 0.0, -0.1],
        }]
    )

    mocap_gripper2 = Node(
        package="moonbot_isaac",
        executable="mocap_simulator",
        name="mocap_simulator",
        output="screen",
        parameters=[{
            "ground_truth_frame": "gt__leg1gripper1",
            "mocap_frame": "mocap1gripper1_up_rotated",
            "offset_translation": [-0.2, 0.0, 0.0],
            "offset_rotation_rvec": [0.0, 0.0, 3.14],
        }]
    )

    mocap_gripper3 = Node(
        package="moonbot_isaac",
        executable="mocap_simulator",
        name="mocap_simulator",
        output="screen",
        parameters=[{
            "ground_truth_frame": "gt__leg1gripper1",
            "mocap_frame": "mocap1gripper1_slow_random",
            "offset_translation": [0.0, 0.0, -0.1],
            "noise_std_translation": 0.05,
            "noise_std_rotation": 0.5,
            "publish_frequency": 2.0,
        }]
    )

    # The robot is not connected to the world frame, so we connect it here to make visualization easier
    connect_tf_trees = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'gt__base_link', 'base_link']
    )
    

    return LaunchDescription(
        [
            SetParameter(name='use_sim_time', value=True),
            mocap_gripper,
            mocap_gripper2,
            mocap_gripper3,
            connect_tf_trees,
        ]
    )

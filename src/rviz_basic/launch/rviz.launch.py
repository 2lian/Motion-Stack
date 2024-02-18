from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from os.path import join
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_name = "moonbot7"
    urdf_file_name = f'{robot_name}.urdf'
    urdf = os.path.join('./src/rviz_basic/urdf/', urdf_file_name)

    with open(urdf, 'r') as file:
        urdf_content = file.read()

    rviz_config_file_name = 'urdf_vis.rviz'
    rviz_config = os.path.join(
        './src/rviz_basic/rviz2/', rviz_config_file_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription(
        [
            Node(
                package='rviz_basic',
                namespace='',  # Default namespace to be able to see coppeliasim
                executable='rviz_interface',
                name='rviz_interface',
                arguments=['--ros-args', '--log-level', "info"],
                parameters=[{
                    'std_movement_time': float(1.5),
                }]
            ),

            Node(
                package='robot_state_publisher',
                namespace='',  # Default namespace to be able to see coppeliasim
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'use_sim_time': use_sim_time,
                             'robot_description': urdf_content}],
                arguments=[urdf],
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_base_link',
                output='screen',
                arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
            ),
        ],  # all nodes in this list will run in their own thread
    )

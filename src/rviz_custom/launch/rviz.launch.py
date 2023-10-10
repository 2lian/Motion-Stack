from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from os.path import join
from ament_index_python.packages import get_package_share_directory

ik_node_list = [Node(
                package='rviz_custom',
                namespace='',  # Default namespace to be able to see coppeliasim
                executable='ik_node',
                name=f'ik_node_{leg}',
                arguments=['--ros-args', '--log-level', "info"],
                parameters=[{'leg_number': leg}]
            ) for leg in range(4)]

movement_node_list = [Node(
                package='rviz_custom',
                namespace='',  # Default namespace to be able to see coppeliasim
                executable='ik_node',
                name=f'leg_movement_{leg}',
                arguments=['--ros-args', '--log-level', "info"],
                parameters=[{'leg_number': leg}]
            ) for leg in range(4)]

def generate_launch_description():
    robot_name = "moonbot" 
    urdf_file_name = f'{robot_name}.urdf'
    urdf = os.path.join('./src/rviz_custom/urdf/', urdf_file_name)

    with open(urdf, 'r') as file:
        urdf_content = file.read()

    rviz_config_file_name = 'urdf_vis.rviz'
    rviz_config = os.path.join('./src/rviz_custom/rviz2/', rviz_config_file_name)
    print(rviz_config)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription(
        movement_node_list +
        ik_node_list +
        [
            Node(
                package='rviz_custom',
                namespace='',  # Default namespace to be able to see coppeliasim
                executable='rviz_interface',
                name='rviz_interface',
                arguments=['--ros-args', '--log-level', "info"],
            ),

            Node(
                package='robot_state_publisher',
                namespace='',  # Default namespace to be able to see coppeliasim
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf_content}],
                arguments=[urdf],
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
            ),
        ],  # all nodes in this list will run in their own thread
    )

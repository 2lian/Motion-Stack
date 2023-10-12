from launch import LaunchDescription
from launch_ros.actions import Node

package_name = 'easy_robot_control'

ik_node_list = [Node(
                package=package_name,
                namespace='',  # Default namespace to be able to see coppeliasim
                executable='ik_node',
                name=f'ik_node_{leg}',
                arguments=['--ros-args', '--log-level', "info"],
                parameters=[{'leg_number': leg}]
            ) for leg in range(4)]

movement_node_list = [Node(
                package=package_name,
                namespace='',  # Default namespace to be able to see coppeliasim
                executable='leg_movement_node',
                name=f'leg_movement_{leg}',
                arguments=['--ros-args', '--log-level', "info"],
                parameters=[{'leg_number': leg}]
            ) for leg in range(4)]

def generate_launch_description():

    return LaunchDescription(
        movement_node_list +
        ik_node_list,  # all nodes in this list will run in their own thread
    )

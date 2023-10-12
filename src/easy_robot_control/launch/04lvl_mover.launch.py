from launch import LaunchDescription
from launch_ros.actions import Node

package_name = 'easy_robot_control'

ik_node_list = [Node(
                package=package_name,
                namespace='',  # Default namespace
                executable='ik_node',
                name=f'ik_node_{leg}',
                arguments=['--ros-args', '--log-level', "info"],
                parameters=[{'leg_number': leg}]
            ) for leg in range(4)]

movement_node_list = [Node(
                package=package_name,
                namespace='',  # Default namespace
                executable='leg_node',
                name=f'leg_{leg}',
                arguments=['--ros-args', '--log-level', "info"],
                parameters=[{'leg_number': leg}]
            ) for leg in range(4)]

other_nodes = [Node(
                package=package_name,
                namespace='',  # Default namespace
                executable='mover_node',
                name=f'mover',
                arguments=['--ros-args', '--log-level', "info"],
            )]

def generate_launch_description():

    return LaunchDescription(
        movement_node_list +
        ik_node_list +
        other_nodes
    )

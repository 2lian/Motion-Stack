from launch import LaunchDescription
from launch_ros.actions import Node
import sys

package_name = 'easy_robot_control'

# Add the launch directory to the Python path to import the settings without rebuilding
directory_to_add = f'./src/{package_name}/launch'
sys.path.append(directory_to_add)
import launch_setting
from lvl_03_leg import nodeList

other_nodes = [Node(
                package=package_name,
                namespace='',  # Default namespace
                executable='mover_node',
                name=f'mover',
                arguments=['--ros-args', '--log-level', "info"],
            )]

nodeList = nodeList + other_nodes

def generate_launch_description():

    return LaunchDescription(
        nodeList
    )

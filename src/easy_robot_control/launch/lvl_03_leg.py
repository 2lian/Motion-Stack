from launch import LaunchDescription
from launch_ros.actions import Node
import sys

package_name = 'easy_robot_control'

# Add the launch directory to the Python path to import the settings without rebuilding
directory_to_add = f'./src/{package_name}/launch'
sys.path.append(directory_to_add)
import launch_setting
from lvl_02_ik import nodeList

movement_node_list = [Node(
    package=package_name,
    namespace='',  # Default namespace
    executable='leg_node',
    name=f'leg_{leg}',
    arguments=['--ros-args', '--log-level', "info"],
    parameters=[{
        'leg_number': leg,
        'std_movement_time': float(launch_setting.std_movement_time),
        'movement_update_rate': float(launch_setting.movement_update_rate),
                 }]
) for leg in range(4)]

nodeList = nodeList + movement_node_list


def generate_launch_description():
    return LaunchDescription(
        nodeList  # all nodes in this list will run in their own thread
    )

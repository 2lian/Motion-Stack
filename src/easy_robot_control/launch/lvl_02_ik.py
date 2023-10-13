from launch import LaunchDescription
from launch_ros.actions import Node
import sys

package_name = 'easy_robot_control'

# Add the launch directory to the Python path to import the settings without rebuilding
directory_to_add = f'./src/{package_name}/launch'
sys.path.append(directory_to_add)
import launch_setting

robot = launch_setting.moonbot_leg

ik_node_list = [Node(
    package=package_name,
    namespace='',  # Default namespace
    executable='ik_node',
    name=f'ik_node_{leg}',
    arguments=['--ros-args', '--log-level', "info"],
    parameters=[{
        'leg_number': leg,

        'bodyToCoxa': robot.bodyToCoxa,
        'coxaLength': robot.coxaLength,
        'femurLength': robot.femurLength,
        'tibiaLength': robot.tibiaLength,

        'coxaMax': robot.coxaMax,
        'coxaMin': robot.coxaMin,
        'femurMax': robot.femurMax,
        'femurMin': robot.femurMin,
        'tibiaMax': robot.tibiaMax,
        'tibiaMin': robot.tibiaMin,
    }]
) for leg in range(4)]

nodeList = ik_node_list

def generate_launch_description():
    return LaunchDescription(
        nodeList
    )

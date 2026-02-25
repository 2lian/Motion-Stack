import subprocess
from subprocess import Popen
from typing import Any, Dict

from launch_ros.actions import Node

import motion_stack.ros2.communication as comms
from launch.launch_description import LaunchDescription
from motion_stack.api.launch.builder import LevelBuilder, xacro_path_from_pkg
from motion_stack.core.lvl1_rework import Lvl1Param

urdf_path = xacro_path_from_pkg(
    package_name="motion_stack_tuto", xacro_path="urdf/moonbot_zero.xacro"
)
xacro_proc = Popen(f"xacro {urdf_path}", shell=True, stdout=subprocess.PIPE)
xacro_proc.wait()
urdf = xacro_proc.stdout.read().decode()

params_lvl1 = [
    Lvl1Param(urdf=urdf, namespace=f"leg{k}", end_effector_name=k % 4)
    for k in range(4)
]

MINI_SIM_REMAP = [
    ("joint_states", "/joint_states"),
    ("joint_commands", "/joint_commands"),
]


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                executable="lvl1",
                name=f"lvl1_{p.namespace}",
                package="motion_stack",
                namespace=p.namespace,
                arguments=["--ros-args", "--log-level", "warn"],
                output="screen",
                emulate_tty=True,
                parameters=[{"ms_params": p.to_json()}],
                remappings=[] + MINI_SIM_REMAP,
            )
            for p in params_lvl1
        ]
        + [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name=f"rsp_{p.namespace}",
                namespace=p.namespace,
                arguments=["--ros-args", "--log-level", "warn"],
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "robot_description": p.urdf,
                    }
                ],
                remappings=[
                    # (intside node, outside node),
                    ("joint_states", comms.lvl1.output.continuous_joint_state.name),
                    # ("joint_states", "/joint_states"),
                ],
            )
            for p in params_lvl1
        ]
    )

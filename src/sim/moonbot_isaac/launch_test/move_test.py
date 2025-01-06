import os
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from launch_testing_ros import WaitForTopics
import pytest
from std_msgs.msg import String
from launch.actions import ExecuteProcess


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env["PYTHONUNBUFFERED"] = "1"

    sim = ExecuteProcess(
        cmd=["ros2", "launch", "moonbot_isaac", "sim.launch.py"],
        env=proc_env,
        output="screen",
    )
    motion_stack = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "ros2_m_hero_pkg",
            "hero_dragon.launch.py",
            "MS_down_from_level:=1",
            "MS_up_to_level:=3",
            "MS_simu_mode:=true",
        ],
        env=proc_env,
        output="screen",
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.SetParameter(name="use_sim_time", value=True),
            launch_ros.actions.Node(
                package="moonbot_isaac",
                executable="leg_move_test",
                name="leg_move_test_1",
                parameters=[
                    {
                        "tf_service_name": "/leg4/shift",
                        "gt__source_frame": "leg4gripper2",
                        "translate_z": 0.1,
                    }
                ],
            ),
            sim,
            motion_stack,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMotionStackBringup(unittest.TestCase):
    # Wait for the success message
    def test_read_stdout(self):
        with WaitForTopics([("leg_move_test_success", String)], timeout=120.0):
            print("Receiver success message!")


@launch_testing.post_shutdown_test()
class TestHelloWorldShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)

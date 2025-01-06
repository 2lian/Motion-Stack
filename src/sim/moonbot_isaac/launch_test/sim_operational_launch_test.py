import os
import unittest

import launch
import launch.actions
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
        cmd=["ros2", "launch", "easy_robot_control", "moonbot_zero.launch.py", "MS_down_from_level:=1", "MS_up_to_level:=4", "MS_simu_mode:=true"],
        env=proc_env,
        output="screen",
    )

    return launch.LaunchDescription([
        sim,
        motion_stack,
        launch_testing.actions.ReadyToTest()
    ])


class TestFixture(unittest.TestCase):
    def test_check_if_isaac_specific_robot_description_is_published(self):
        with WaitForTopics([('/robot_description_isaac', String)], timeout=15.0):
            print('Topic received messages !')
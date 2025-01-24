import os
import unittest

import launch
import launch.actions
import launch_testing.actions
import launch_testing.markers
import pytest
from launch.actions import ExecuteProcess

# This function specifies the processes to be run for our test
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
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


class TestMotionStackBringup(unittest.TestCase):

    def test_read_stdout(self, proc_output):
        """Wait for the Motion Stack health message."""
        proc_output.assertWaitFor('[M] mover spinning :)', timeout=10, stream='stdout')


@launch_testing.post_shutdown_test()
class TestHelloWorldShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
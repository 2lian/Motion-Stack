from ros2_asyncio.ros2_executor_patch import patch_executor
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
import rclpy
import pytest


@pytest.mark.parametrize("executor_cls", [SingleThreadedExecutor, MultiThreadedExecutor])
def test_patch_executor_requires_certain_version(executor_cls):
    rclpy.init()
    executor = executor_cls()
    # Patch the executor to fix a bug in rclpy.
    executor = patch_executor(executor)
    rclpy.try_shutdown()


@pytest.fixture()
def simple_node():
    # Setup code here
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node(
        'TestExecutor', namespace='/rclpy', context=context)
    yield node
    # Teardown code here
    node.destroy_node()
    rclpy.shutdown(context=context)


def test_patch_catches_InvalidHandle_exception(simple_node):
    assert simple_node.handle is not None

    def _run(executor):
        loop_counter = 0

        def on_loop_counter():
            nonlocal loop_counter
            loop_counter += 1
        tmr = simple_node.create_timer(0.01, on_loop_counter)
        executor.spin_once(timeout_sec=0.05)
        simple_node.destroy_timer(tmr)

    with pytest.raises(rclpy._rclpy_pybind11.InvalidHandle):
        executor = MultiThreadedExecutor(context=simple_node.context)
        executor.add_node(simple_node)
        for _ in range(100):
            _run(executor)

    patch_executor(executor)
    for _ in range(100):
        _run(executor)

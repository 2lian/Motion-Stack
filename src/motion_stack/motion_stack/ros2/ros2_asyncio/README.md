# Ros2 AsyncIO libray

## Introduction

*Asynchronous programming* is a method of concurrent programming that enables efficient execution of tasks, particularly in I/O-bound and network-driven applications.

It enables a program to handle multiple tasks at the same time, while not using OS level threads.

When using the ROS2 Python API (rclpy) together with `ros2_asyncio`, we can use some features of the Python language to achieve more readable code and efficient execution.


## Why not Python's `asyncio`?
We need to be clear on one thing. ROS 2 has no relation with Python’s `asyncio` library https://docs.python.org/3/library/asyncio.html

Yes, the `asyncio` library has many great features, and we normally use Python asynchronous programming always in tandem with it.

But ROS 2 is not making use of it and has its **own** Event Loop, Futures and Task implementations.
See details in https://github.com/ros2/rclpy/issues/279

Hence, when trying it use `asyncio` with ROS 2 you typically run into problems like this:
```bash
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 294, in spin
    self.spin_once()
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 739, in spin_once
    self._spin_once_impl(timeout_sec)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 736, in _spin_once_impl
    raise handler.exception()
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/task.py", line 239, in __call__
    self._handler.send(None)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 437, in handler
    await call_coroutine(entity, arg)
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 391, in _execute_service
    response = await await_or_execute(srv.callback, request, srv.srv_type.Response())
  File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 104, in await_or_execute
    return await callback(*args)
  File "/workspace/aescape_perception/example_event_loop_ros2.py", line 43, in sleep10_callback
    await asyncio.sleep(5)
  File "/usr/lib/python3.10/asyncio/tasks.py", line 599, in sleep
    loop = events.get_running_loop()
RuntimeError: no running event loop
```
## Installation
```bash
pip3 install git+https://github.com/tlangmo/ros2_asyncio.git
```
## How to use `ros2_asyncio`?
`ros2_asyncio` has very similar API to `asyncio` but it is ROS 2 aware.

```python
import ros2_asyncio
context = rclpy.context.Context()
rclpy.init(context=context)
node = rclpy.create_node(
    'TestExecutor', namespace='/rclpy', context=context)
try:
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    async def _do_task(sleep_sec: int):
        for i in range(5):
            await ros2_asyncio.sleep(simple_node, sleep_sec)
            print(f'Hello from task')
    f = executor.create_task(_do_task(1))
    executor.spin_until_future_complete(f)
finally:
    # Teardown code: destroy the node and shutdown rclpy
    node.destroy_node()
    rclpy.shutdown(context=context)
```

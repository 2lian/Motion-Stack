# motion_stack.ros2.ros2_asyncio package

Used to await sleep in callbacks and also fix the bugged executor.

Origin: [https://github.com/tlangmo/ros2_asyncio/tree/main](https://github.com/tlangmo/ros2_asyncio/tree/main)
Author: Tobias Lang

### *async* motion_stack.ros2.ros2_asyncio.sleep(node, timeout_sec)

Ros2 compatible coroutine sleep function.

It works similar to asyncio.sleep() but it is compatible with the Ros2 event loop.
@see [https://www.notion.so/aescape/Asynchronous-programming-with-ROS-2-Python-Api-f26cb40c0f634528816abea0d66d968b](https://www.notion.so/aescape/Asynchronous-programming-with-ROS-2-Python-Api-f26cb40c0f634528816abea0d66d968b)

* **Parameters:**
  * **node** (*Node*)
  * **timeout_sec** (*float*)

### *async* motion_stack.ros2.ros2_asyncio.wait_for(node, coro_or_future, timeout_sec)

Ros2 compatible coroutine to wait for a future with timeout.

It works similar to asyncio.wait_for() but it is compatible with the Ros2 event loop.
@see [https://www.notion.so/aescape/Asynchronous-programming-with-ROS-2-Python-Api-f26cb40c0f634528816abea0d66d968b](https://www.notion.so/aescape/Asynchronous-programming-with-ROS-2-Python-Api-f26cb40c0f634528816abea0d66d968b)

* **Parameters:**
  * **node** (*Node*)
  * **coro_or_future** (*Future* *|* *Awaitable*)
  * **timeout_sec** (*float*)

### motion_stack.ros2.ros2_asyncio.ensure_future(node, coro_or_future)

Wrap a coroutine or an awaitable in a Ros2 future.

* **Return type:**
  `Task`
* **Parameters:**
  **coro_or_future** (*Future* *|* *Awaitable*)

### motion_stack.ros2.ros2_asyncio.gather(node, \*coros_or_futures, return_exceptions=False)

Return a future aggregating results from the given coroutines/futures.

Based on Python’s asyncio/tasks.py

* **Parameters:**
  **node** (*Node*)

## Submodules

## motion_stack.ros2.ros2_asyncio.ros2_asyncio module

### motion_stack.ros2.ros2_asyncio.ros2_asyncio.ensure_future(node, coro_or_future)

Wrap a coroutine or an awaitable in a Ros2 future.

* **Return type:**
  `Task`
* **Parameters:**
  **coro_or_future** (*Future* *|* *Awaitable*)

### *async* motion_stack.ros2.ros2_asyncio.ros2_asyncio.sleep(node, timeout_sec)

Ros2 compatible coroutine sleep function.

It works similar to asyncio.sleep() but it is compatible with the Ros2 event loop.
@see [https://www.notion.so/aescape/Asynchronous-programming-with-ROS-2-Python-Api-f26cb40c0f634528816abea0d66d968b](https://www.notion.so/aescape/Asynchronous-programming-with-ROS-2-Python-Api-f26cb40c0f634528816abea0d66d968b)

* **Parameters:**
  * **node** (*Node*)
  * **timeout_sec** (*float*)

### *async* motion_stack.ros2.ros2_asyncio.ros2_asyncio.wait_for(node, coro_or_future, timeout_sec)

Ros2 compatible coroutine to wait for a future with timeout.

It works similar to asyncio.wait_for() but it is compatible with the Ros2 event loop.
@see [https://www.notion.so/aescape/Asynchronous-programming-with-ROS-2-Python-Api-f26cb40c0f634528816abea0d66d968b](https://www.notion.so/aescape/Asynchronous-programming-with-ROS-2-Python-Api-f26cb40c0f634528816abea0d66d968b)

* **Parameters:**
  * **node** (*Node*)
  * **coro_or_future** (*Future* *|* *Awaitable*)
  * **timeout_sec** (*float*)

### motion_stack.ros2.ros2_asyncio.ros2_asyncio.gather(node, \*coros_or_futures, return_exceptions=False)

Return a future aggregating results from the given coroutines/futures.

Based on Python’s asyncio/tasks.py

* **Parameters:**
  **node** (*Node*)

## motion_stack.ros2.ros2_asyncio.ros2_executor_patch module

### motion_stack.ros2.ros2_asyncio.ros2_executor_patch.patch_executor(executor)

Patch the executor to avoid InvalidHandle exceptions when destroying
entities.

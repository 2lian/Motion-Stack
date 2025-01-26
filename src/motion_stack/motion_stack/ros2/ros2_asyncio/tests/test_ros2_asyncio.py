import time
import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from ros2_asyncio import ros2_asyncio


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


def test_sleep_yields_to_main_loop(simple_node):
    assert simple_node.handle is not None
    executor = SingleThreadedExecutor(context=simple_node.context)
    executor.add_node(simple_node)
    loop_counter = 0

    def on_loop_counter():
        nonlocal loop_counter
        loop_counter += 1
    simple_node.create_timer(0.1, on_loop_counter)

    async def _do():
        await ros2_asyncio.sleep(simple_node, 2)
    f = executor.create_task(_do())
    now = time.perf_counter()
    executor.spin_until_future_complete(f)
    assert time.perf_counter() - now >= 1.95
    assert loop_counter > 10


def test_sleep_0(simple_node):
    assert simple_node.handle is not None
    executor = SingleThreadedExecutor(context=simple_node.context)
    executor.add_node(simple_node)
    loop_counter = 0

    def on_loop_counter():
        nonlocal loop_counter
        loop_counter += 1
    simple_node.create_timer(0.1, on_loop_counter)

    async def _do():
        await ros2_asyncio.sleep(simple_node, 0)
    f = executor.create_task(_do())
    now = time.perf_counter()
    executor.spin_until_future_complete(f)
    assert time.perf_counter() - now < 1
    assert loop_counter == 0


def test_wait_for_raises_TimeoutException(simple_node):
    assert simple_node.handle is not None
    executor = SingleThreadedExecutor(context=simple_node.context)
    executor.add_node(simple_node)

    async def _do():
        work_future = Future()

        def _on_done():
            work_future.set_result(True)
        simple_node.create_timer(2, _on_done)
        with pytest.raises(TimeoutError):
            await ros2_asyncio.wait_for(simple_node, work_future, timeout_sec=1)
    f = executor.create_task(_do())
    executor.spin_until_future_complete(f)


def test_wait_for_does_not_wait_longer_as_needed(simple_node):
    assert simple_node.handle is not None
    executor = SingleThreadedExecutor(context=simple_node.context)
    executor.add_node(simple_node)
    work_done = False
    loop_counter = 0

    def on_loop_counter():
        nonlocal loop_counter
        loop_counter += 1
    simple_node.create_timer(0.1, on_loop_counter)

    async def _do():
        nonlocal work_done
        work_future = Future()

        def _on_done():
            work_future.set_result(True)
        simple_node.create_timer(1, _on_done)
        await ros2_asyncio.wait_for(simple_node, work_future, timeout_sec=10)
        work_done = True
    f = executor.create_task(_do())
    now = time.perf_counter()
    executor.spin_until_future_complete(f)
    elapsed_sec = time.perf_counter() - now
    assert 0.9 < elapsed_sec < 2
    assert work_done


def test_wait_for_accepts_coro(simple_node):
    assert simple_node.handle is not None
    executor = SingleThreadedExecutor(context=simple_node.context)
    executor.add_node(simple_node)

    was_called = False

    async def long_work(timeout_sec):
        nonlocal was_called
        was_called = True
        await ros2_asyncio.sleep(simple_node, timeout_sec)

    async def _do():
        now = time.perf_counter()
        await ros2_asyncio.wait_for(simple_node, long_work(2), timeout_sec=5)
        elapsed_sec = time.perf_counter() - now
        assert 0 < elapsed_sec < 3
        assert was_called
    f = executor.create_task(_do())
    executor.spin_until_future_complete(f)


def test_gather(simple_node):
    assert simple_node.handle is not None
    executor = SingleThreadedExecutor(context=simple_node.context)
    executor.add_node(simple_node)

    num_calls = 0

    async def long_work(timeout_sec):
        nonlocal num_calls
        await ros2_asyncio.sleep(simple_node, timeout_sec)
        num_calls += 1

    async def _do():
        await ros2_asyncio.gather(simple_node, long_work(0.1), long_work(0.2))
    f = executor.create_task(_do())
    executor.spin_until_future_complete(f)
    assert num_calls == 2


def test_gather_raises_exception(simple_node):
    assert simple_node.handle is not None
    executor = SingleThreadedExecutor(context=simple_node.context)
    executor.add_node(simple_node)

    num_calls = 0

    async def work_with_exception():
        await ros2_asyncio.sleep(simple_node, 0)
        raise RuntimeError("This is an error")

    async def work_with_result():
        await ros2_asyncio.sleep(simple_node, 0)
        return 42

    async def _do():
        with pytest.raises(RuntimeError):
            await ros2_asyncio.gather(simple_node, work_with_exception(), work_with_result(), return_exceptions=False)

    f = executor.create_task(_do())
    executor.spin_until_future_complete(f)


def test_gather_returns_exception(simple_node):
    assert simple_node.handle is not None
    executor = SingleThreadedExecutor(context=simple_node.context)
    executor.add_node(simple_node)

    async def work_with_exception():
        await ros2_asyncio.sleep(simple_node, 0)
        raise RuntimeError("This is an error")

    async def work_with_result():
        await ros2_asyncio.sleep(simple_node, 0)
        return 42

    async def _do():
        results = await ros2_asyncio.gather(simple_node, work_with_exception(), work_with_result(), return_exceptions=True)
        assert len(results) == 2
        assert isinstance(results[0], RuntimeError)
        assert results[1] == 42

    f = executor.create_task(_do())
    executor.spin_until_future_complete(f)


def test_gather_coro_immediately_done(simple_node):
    assert simple_node.handle is not None
    executor = SingleThreadedExecutor(context=simple_node.context)
    executor.add_node(simple_node)

    async def work_immediate_done():
        return 1

    async def work_with_delay():
        await ros2_asyncio.sleep(simple_node, 1)
        return 42

    async def _do():
        results = await ros2_asyncio.gather(simple_node, work_immediate_done(), work_with_delay(), return_exceptions=False)
        assert len(results) == 2
        assert results == [1, 42]

    f = executor.create_task(_do())
    executor.spin_until_future_complete(f)


def test_gather_empty_returns_empty_list(simple_node):
    assert simple_node.handle is not None
    executor = SingleThreadedExecutor(context=simple_node.context)
    executor.add_node(simple_node)

    async def _do():
        results = await ros2_asyncio.gather(simple_node, return_exceptions=False)
        assert isinstance(results, list)
        assert len(results) == 0

    f = executor.create_task(_do())
    executor.spin_until_future_complete(f)


def test_gather_handles_duplicate_tasks(simple_node):
    assert simple_node.handle is not None
    executor = SingleThreadedExecutor(context=simple_node.context)
    executor.add_node(simple_node)

    async def work_with_result():
        await ros2_asyncio.sleep(simple_node, 0)
        return 42

    async def _do():
        coro = work_with_result()
        results = await ros2_asyncio.gather(simple_node, coro, coro)
        assert len(results) == 2

    f = executor.create_task(_do())
    executor.spin_until_future_complete(f)


def test_gather_accepts_Future(simple_node):
    assert simple_node.handle is not None
    executor = SingleThreadedExecutor(context=simple_node.context)
    executor.add_node(simple_node)

    num_calls = 0

    async def long_work(timeout_sec):
        nonlocal num_calls
        await ros2_asyncio.sleep(simple_node, timeout_sec)
        num_calls += 1

    async def _do_ok():
        fut = executor.create_task(long_work(1))
        assert isinstance(fut, Future)
        await ros2_asyncio.gather(simple_node, fut)
    f = executor.create_task(_do_ok())
    executor.spin_until_future_complete(f)
    assert num_calls == 1


def test_gather_raise_ValueError_on_return_exceptions_and_Futures(simple_node):
    assert simple_node.handle is not None
    executor = SingleThreadedExecutor(context=simple_node.context)
    executor.add_node(simple_node)

    num_calls = 0

    async def long_work(timeout_sec):
        nonlocal num_calls
        await ros2_asyncio.sleep(simple_node, timeout_sec)
        num_calls += 1

    async def _do_invalid():
        fut = executor.create_task(long_work(1))
        assert isinstance(fut, Future)
        with pytest.raises(ValueError):
            await ros2_asyncio.gather(simple_node, fut, return_exceptions=True)
    f = executor.create_task(_do_invalid())
    executor.spin_until_future_complete(f)

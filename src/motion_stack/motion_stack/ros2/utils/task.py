from dataclasses import dataclass
from typing import Callable



# async def printo():
#     await aio.sleep(0)
#     print("o")
#
# rclpy.get_global_executor().create_task()
# rclpy.callback_groups.MutuallyExclusiveCallbackGroup().add_entity(printo)
#
# async def printh():
#     await aio.sleep(0)
#     print("h")
#
# async def printoh():
#     await printo()
#     await printh()
#
# loop = aio.new_event_loop()
#
# task = loop.create_task(printoh())
# task.add_done_callback(lambda *_: print("yey"))
# while not task.done():
#     print("step")
#     loop.call_soon(loop.stop)
#     loop.run_forever()
# print("step")
# loop.call_soon(loop.stop)
# loop.run_forever()
# print("done")

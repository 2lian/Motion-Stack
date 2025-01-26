import asyncio as ao

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

import motion_stack.ros2.ros2_asyncio.ros2_asyncio as rao
from motion_stack.api.ros2.leg_api import JointHandler, JointSyncerRos
from motion_stack.ros2.utils.executor import error_catcher, my_main


class TestNode(Node):
    def __init__(self) -> None:
        super().__init__("test_node")
        self.handlers = [JointHandler(self, l + 1) for l in range(4)]
        self.syncer = JointSyncerRos(self.handlers)
        self.create_timer(0.1, self.loop)
        self.startTMR = self.create_timer(0.1, self.start)
        self.get_logger().info("init done")

    async def update(self):
        while 1:
            l = [jh._update_tracked()[1] for jh in self.handlers]
            try:
                await rao.wait_for(self, rao.gather(self, *l), timeout_sec=2)
                print(f"Joints ready.")
                print([jh.tracked for jh in self.handlers])
                return
            except TimeoutError:
                print("timedout")

    @error_catcher
    async def main(self):
        await self.update()
        await rao.wait_for(
            self,
            self.syncer.lerp(
                {
                    "leg1joint1": -0.8451095512894327,
                    "leg1joint2": 0.6438672054617042,
                    "leg1joint3": -0.08176167325392356,
                    "leg1joint4": -0.45509022167351343,
                    "leg1joint5": -0.8766009388705699,
                    "leg1joint6": 0.6521011585887132,
                    "leg1joint7": 0.6382180520487218,
                }
            ),
            timeout_sec=100,
        )
        await rao.wait_for(
            self,
            self.syncer.lerp(
                {
                    "leg1joint1": -2.3628941249689794,
                    "leg1joint2": 1.7812884693818785,
                    "leg1joint3": -0.9180423112468289,
                    "leg1joint4": -1.3107960110474275,
                    "leg1joint5": -2.4656408740968927,
                    "leg1joint6": 0.3563304887518892,
                    "leg1joint7": 1.8388284375492987,
                }
            ),
            timeout_sec=100,
        )
        await rao.wait_for(
            self,
            self.syncer.lerp(
                {
                    "leg1joint1": -1.5177845736795468,
                    "leg1joint2": 1.1374212639201744,
                    "leg1joint3": -0.8362806379929053,
                    "leg1joint4": -0.8557057893739141,
                    "leg1joint5": -1.5890399352263227,
                    "leg1joint6": -0.295770669836824,
                    "leg1joint7": 1.200610385500577,
                }
            ),
            timeout_sec=100,
        )
        await rao.wait_for(
            self,
            self.syncer.asap(
                {
                    "leg1joint1": 0,
                    "leg1joint2": 0,
                    "leg1joint3": 0,
                    "leg1joint4": 0,
                    "leg1joint5": 0,
                    "leg1joint6": 0,
                    "leg1joint7": 0,
                }
            ),
            timeout_sec=100,
        )
        print("finished")

    @error_catcher
    def start(self):
        rao.ensure_future(self, self.main())
        # self.executor.create_task(self.update())
        self.destroy_timer(self.startTMR)
        print("destroyed start")

    @error_catcher
    def loop(self):
        self.syncer.execute()
        return


async def spin(executor: SingleThreadedExecutor):
    while rclpy.ok():
        executor.spin_once()
        # Setting the delay to 0 provides an optimized path to allow other tasks to run.
        await ao.sleep(0)


def main(*args):
    my_main(TestNode)
    return
    rclpy.init()

    node = TestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    loop = ao.get_event_loop()
    executor.spin()
    # loop.run_until_complete(spin(executor))

    node.destroy_node()
    rclpy.shutdown()

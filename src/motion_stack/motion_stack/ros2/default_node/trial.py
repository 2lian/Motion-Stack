import asyncio as ao

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

import motion_stack.ros2.ros2_asyncio.ros2_asyncio as rao
from motion_stack.api.ros2.leg_api import JointHandler, JointSyncerRos
from motion_stack.ros2.utils.executor import error_catcher, my_main

import json


class TestNode(Node):
    def __init__(self) -> None:
        filepath = "/home/elian/Downloads/episodes.json"
        f = open(filepath)
        self.path = json.load(fp=f)
        print(self.path[0]["action"]["arm_joint1"])
        print(self.path[1]["action"]["arm_joint1"])
        print(self.step(1))
        super().__init__("test_node")
        self.handlers = [JointHandler(self, l + 1) for l in [2]]
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

    def step(self, n: int):
        data = self.path[n]["action"]
        remap = {
            "arm_joint1": "leg3joint1",
            "arm_joint2": "leg3joint2",
            "arm_joint3": "leg3joint3",
            "arm_joint4": "leg3joint4",
            "arm_joint5": "leg3joint5",
            "arm_joint6": "leg3joint6",
            "arm_joint7": "leg3joint7",
        }
        off = {
           "arm_joint1": -3.1361150423869044,
           "arm_joint2": 0.9799041287896845,
           "arm_joint3": 0.036062767152477006,
           "arm_joint4": -0.60224226913142,
           "arm_joint5": 0.005872011547052069,
           "arm_joint6": 1.5410355751365572,
           "arm_joint7": -1.5180754271989076,
        }
        gain = {
           "arm_joint1": 1,
           "arm_joint2": -1,
           "arm_joint3": 1,
           "arm_joint4": -1,
           "arm_joint5": 1,
           "arm_joint6": 1,
           "arm_joint7": -1,
        }
        out = {remap[k]: v*gain[k]+off[k] for k, v in data.items()}
        return out

    @error_catcher
    async def main(self):
        await self.update()
        for n in range(len(self.path)):
            print(f"{n=}")
            target = self.step(n)
            await rao.wait_for(self, self.syncer.asap(target), timeout_sec=100)

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


def main(*args):
    my_main(TestNode)

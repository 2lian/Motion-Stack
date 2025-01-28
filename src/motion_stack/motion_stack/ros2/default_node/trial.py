import json

from rclpy.node import Node

import motion_stack.ros2.ros2_asyncio.ros2_asyncio as rao
from motion_stack.api.ros2.leg_api import JointHandler, JointSyncerRos
from motion_stack.ros2.utils.executor import error_catcher, my_main


class TestNode(Node):
    def __init__(self) -> None:
        JSON_PATH = "/home/elian/episodes.json"
        self.JSON_TRAJECTORY = json.load(open(JSON_PATH))
        self.LEG_NUM = 1

        super().__init__("test_node")
        self.handlers = [JointHandler(self, l) for l in [self.LEG_NUM]]
        self.syncer = JointSyncerRos(self.handlers)
        self.create_timer(0.1, self.loop)
        self.startTMR = self.create_timer(0.1, self.startup)
        self.get_logger().info("init done")

    async def ready_up(self):
        l = [jh.ready for jh in self.handlers]
        try:
            await rao.wait_for(self, rao.gather(self, *l), timeout_sec=100)
            print(f"Joints ready.")
            strlist = "\n".join(
                [f"limb {jh.limb_number}: {jh.tracked}" for jh in self.handlers]
            )
            print(f"Joints are:\n{strlist}")
            return
        except TimeoutError:
            raise TimeoutError("Joint data unavailable after 100 sec")

    def json_step(self, n: int):
        data = self.JSON_TRAJECTORY[n]["action"]
        remap = {
            "arm_joint1": f"leg{self.LEG_NUM}joint1",
            "arm_joint2": f"leg{self.LEG_NUM}joint2",
            "arm_joint3": f"leg{self.LEG_NUM}joint3",
            "arm_joint4": f"leg{self.LEG_NUM}joint4",
            "arm_joint5": f"leg{self.LEG_NUM}joint5",
            "arm_joint6": f"leg{self.LEG_NUM}joint6",
            "arm_joint7": f"leg{self.LEG_NUM}joint7",
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
        out = {remap[k]: v * gain[k] + off[k] for k, v in data.items()}
        return out

    @error_catcher
    async def main(self):
        await self.ready_up()
        for step in range(len(self.JSON_TRAJECTORY)):
            print(f"{step=}")
            target = self.json_step(step)
            await rao.wait_for(self, self.syncer.lerp(target), timeout_sec=100)

        print("finished")

    @error_catcher
    def startup(self):
        rao.ensure_future(self, self.main())
        self.destroy_timer(self.startTMR)
        print("Startup done.")

    @error_catcher
    def loop(self):
        self.syncer.execute()
        return


def main(*args):
    my_main(TestNode)

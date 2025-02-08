import json

import numpy as np
from rclpy.node import List, Node

import motion_stack.ros2.ros2_asyncio.ros2_asyncio as rao
from motion_stack.api.ros2.ik_api import IkHandler, IkSyncerRos
from motion_stack.api.ros2.joint_api import JointHandler, JointSyncerRos
from motion_stack.core.utils.joint_state import JState
from motion_stack.core.utils.math import patch_numpy_display_light
from motion_stack.core.utils.pose import Pose, XyzQuat
from motion_stack.ros2.utils.executor import error_catcher, my_main

patch_numpy_display_light()

SAVE = "/home/elian/data.csv"
f = open(SAVE, "w")
f.write("")


class TestNode(Node):
    def __init__(self) -> None:
        JSON_PATH = "/home/elian/episodes.json"
        self.JSON_TRAJECTORY = json.load(open(JSON_PATH))
        self.LEG_NUM = 3
        # LEG_LIST = [self.LEG_NUM]
        LEG_LIST = [1, 2, 3, 4]

        super().__init__("test_node")
        self.joint_handlers = [JointHandler(self, l) for l in LEG_LIST]
        self.joint_syncer = JointSyncerRos(self.joint_handlers)
        self.ik_handlers = [IkHandler(self, l) for l in LEG_LIST]
        self.ik_syncer = IkSyncerRos(self.ik_handlers)
        self.create_timer(1 / 30, self.loop)
        # for jh in self.handlers:
        # jh.new_state_cbk.append(lambda *_: self.loop())
        self.startTMR = self.create_timer(0.1, self.startup)
        self.get_logger().info("init done")
        self.__target_save = None

    async def joints_ready(self):
        l = [jh.ready for jh in self.joint_handlers]
        try:
            print("Waiting for joints.")
            await rao.wait_for(self, rao.gather(self, *l), timeout_sec=100)
            print(f"Joints ready.")
            strlist = "\n".join(
                [f"limb {jh.limb_number}: {jh.tracked}" for jh in self.joint_handlers]
            )
            # print(f"Joints are:\n{strlist}")
            return
        except TimeoutError:
            raise TimeoutError("Joint data unavailable after 100 sec")

    async def ik_ready(self):
        l = [ih.ready for ih in self.ik_handlers]
        try:
            print("Waiting for ik.")
            await rao.wait_for(self, rao.gather(self, *l), timeout_sec=100)
            print(f"Ik ready.")
            strlist = "\n".join(
                [f"limb {ih.limb_number}: {ih.ee_pose}" for ih in self.ik_handlers]
            )
            # print(f"EE poses are:\n{strlist}")
            return
        except TimeoutError:
            raise TimeoutError("Ik data unavailable after 100 sec")

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
        out = {
            remap[k]: v * gain[k] + off[k] for k, v in data.items() if k in remap.keys()
        }
        return out

    async def execute_json(self):
        for step in range(len(self.JSON_TRAJECTORY)):
            print(f"{step=}")
            target = self.json_step(step)
            self.__target_save = target
            await rao.wait_for(self, self.joint_syncer.lerp(target), timeout_sec=100)

    async def zero(self):
        target = {}
        for jh in self.joint_handlers:
            target.update({jname: 0.0 for jname in jh.tracked})
        self.__target_save = target
        await rao.wait_for(
            self,
            self.joint_syncer.asap(target),
            timeout_sec=100,
        )

    async def ik_square(self):
        d = 250
        offsets = np.array(
            [
                [0, 0, d],
                [0, d, d],
                [0, d, 0],
                [0, 0, 0],
            ],
            dtype=float,
        )
        start = [h.ee_pose for h in self.ik_handlers]
        for ind in range(offsets.shape[0]):
            target = {
                h.limb_number: Pose(
                    time=s.time,
                    xyz=s.xyz + offsets[ind, :],
                    quat=s.quat,
                )
                for h, s in zip(self.ik_handlers, start)
            }
            await rao.wait_for(self, self.ik_syncer.lerp(target), timeout_sec=100)

    async def ik_circle(self, samples: int = 20):
        s = samples
        s += 1
        radius = 200
        ang = np.linspace(0, 2 * np.pi, s)
        yz = radius * np.exp(1j * ang)
        offsets = np.zeros((s, 3), dtype=float)
        offsets[:, 1] = yz.real
        offsets[:, 2] = yz.imag

        diverg = {
            3: -300,
            1: -200,
            2: -100,
            4: 0,
        }

        start = [h.ee_pose for h in self.ik_handlers]
        for ind in range(offsets.shape[0]):
            target = {
                h.limb_number: Pose(
                    time=s.time,
                    xyz=s.xyz + offsets[ind, :] + np.array([diverg[h.limb_number], 0, 0]),
                    quat=s.quat,
                )
                for h, s in zip(self.ik_handlers, start)
            }
            await rao.wait_for(self, self.ik_syncer.lerp(target), timeout_sec=100)

    @error_catcher
    async def main(self):
        await self.joints_ready()
        await self.ik_ready()
        self.ik_syncer._interpolation_delta = XyzQuat(70, np.deg2rad(10))
        self.ik_syncer._on_target_delta = XyzQuat(3, np.deg2rad(1))
        await self.zero()
        for s in [4, 6, 30]:
            await self.ik_circle(s)
        self.joint_syncer.clear()
        self.ik_syncer.clear()
        await self.zero()
        self.ik_syncer._interpolation_delta = XyzQuat(20, np.deg2rad(2))
        self.ik_syncer._on_target_delta = XyzQuat(10, np.deg2rad(2))
        for s in [4, 6, 30]:
            await self.ik_circle(s)
        self.joint_syncer.clear()
        self.ik_syncer.clear()
        await self.zero()
        print("finished")

    @error_catcher
    def startup(self):
        rao.ensure_future(self, self.main())
        self.destroy_timer(self.startTMR)
        print("Startup done.")

    @error_catcher
    def loop(self):
        self.joint_syncer.execute()
        self.ik_syncer.execute()
        if self.__target_save is None:
            return
        f.write(
            ",".join(
                [f"{self.get_clock().now().nanoseconds/1e9}"]
                + [f"{val:.6f}" for key, val in self.__target_save.items()]
                + [
                    f"{val.position:.6f}"
                    for key, val in self.joint_handlers[0]._states.items()
                    if key in self.__target_save.keys()
                ]
            )
            + "\n"
        )
        return


def main(*args):
    my_main(TestNode)

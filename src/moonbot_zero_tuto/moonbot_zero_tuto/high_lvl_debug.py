"""This gives example of a high level node using the motion stack API

Warning:
    To make this example as easy as possible, async/await is heavily used.
    This is unusual, you do not need and even, should not use async/await with Ros2.
    The motion stack uses generic Future and callback, async/await style
    is not required for the motion stack.

    In this example every time ``await``, is used (on a ros2 Future, or python awaitable),
    the code pauses until the awaitable finishes, however it does not block the ros2 executor.
    Basically, this ``await`` sleeps/waits without blocking ros2 operations
    (incomming/outgoing messages).

    async/await is easier to read, however much more reliable and performant code is
    possible using ros2 future+callback and especially timers.

"""

import time
from typing import Coroutine

import numpy as np
from rclpy.node import List, Node

pass
import motion_stack.ros2.ros2_asyncio.ros2_asyncio as rao
from motion_stack.api.ik_syncer import XyzQuat
from motion_stack.api.ros2.ik_api import IkHandler, IkSyncerRos
from motion_stack.api.ros2.joint_api import JointHandler, JointSyncerRos
from motion_stack.core.utils.math import patch_numpy_display_light, qt
from motion_stack.core.utils.pose import Pose
from motion_stack.ros2.utils.conversion import ros_now
from motion_stack.ros2.utils.executor import error_catcher, my_main

# lighter numpy display
patch_numpy_display_light()


x = 295
z = -240
DEFAULT_STANCE = np.array(
    [
        [x, 0, z],
        [0, x, z],
        [-x, 0, z],
        [0, -x, z],
    ],
    dtype=float,
)

DEFAULT_STANCE += np.array([0, 0, 0])
RADIUS = 100


class TutoNode(Node):

    #: list of limbs number that are controlled
    LIMBS = [
        # 1,
        2,
        # 3,
        # 4,
        5,
        6,
        7,
        8,
        75,
    ]
    # zero_limbs = [1, 2, 3, 4]
    zero_limbs = [5, 6, 7, 8]
#
    def __init__(self) -> None:
        super().__init__("test_node")

        self.create_timer(1 / 30, self.exec_loop)  # regular execution
        induce_bugTMR = self.create_timer(2, self.induce_bugTMRCKB)  # regular execution
        self.startTMR = self.create_timer(0.1, self.startup)  # executed once

        # API objects:

        # Handles ros2 joints lvl1 (subscribers, publishers and more)
        self.joint_handlers = [JointHandler(self, l) for l in self.LIMBS]
        # Syncronises several joints
        self.joint_syncer = JointSyncerRos(self.joint_handlers)

        self.default_stance = np.empty(
            (len(self.LIMBS), 3),
            float,
        )

        # Handles ros2 ik lvl2
        self.ik_handlers = [IkHandler(self, l) for l in self.LIMBS]
        # Syncronises several IK
        self.ik_syncer = IkSyncerRos(
            self.ik_handlers,
            interpolation_delta=XyzQuat(20, np.inf),
            on_target_delta=XyzQuat(20, np.inf),
        )

        self.get_logger().info("init done")

    @error_catcher
    async def main(self):
        # wait for all syncers to be ready
        await self.joints_ready()
        await self.ik_ready()

        # send to all angle at 0.0
        # await self.angles_to_zero()
        await self.moonbot_zero_zero(self.zero_limbs)
        await rao.sleep(self, 1)
        self.overwrite_default_stance()
        # send to default stance
        # quit()
        await self.stance()
        # quit()

        # move end effector in a square (circle with 4 samples)
        # await self.ik_circle(4)
        # await self.stance()

        # move end effector in a circle
        while 1:
            # await self.angles_to_zero()
            await self.ik_circle(4)
        # await self.stance()

        # increase the value of on_target_delta. Each point of the trajectory will be considered done faster, hence decreasing precision, but executing faster.
        self.ik_syncer = IkSyncerRos(
            self.ik_handlers,
            interpolation_delta=XyzQuat(30, np.inf),
            on_target_delta=XyzQuat(30, np.inf),
        )
        await self.ik_circle(100)
        await self.ik_circle(100)
        await self.ik_circle(100)
        await self.ik_circle(100)
        await self.stance()
        print("finished")

    def moonbot_zero_zero(self, leg_indices: List[int]):
        """Goes to the default moonbot zero stance using IK"""
        xyz_targets = DEFAULT_STANCE
        target = {
            leg_num: Pose(
                time=ros_now(self),
                xyz=xyz_targets[arr_ind, :],
                quat=None,
            )
            for arr_ind, leg_num in enumerate(leg_indices)
        }
        return rao.wait_for(self, self.ik_syncer.lerp(target), timeout_sec=100)

    def overwrite_default_stance(self):
        for i, k in enumerate(self.LIMBS):
            self.default_stance[i, :] = self.ik_handlers[i].ee_pose.xyz
        # print(self.DEFAULT_STANCE)

    async def joints_ready(self):
        """Returns once all joints are ready"""
        ready_tasks = [jh.ready for jh in self.joint_handlers]
        try:
            print("Waiting for joints.")
            fused_task = rao.gather(self, *ready_tasks)
            await rao.wait_for(self, fused_task, timeout_sec=100)
            print(f"Joints ready.")
            strlist = "\n".join(
                [f"limb {jh.limb_number}: {jh.tracked}" for jh in self.joint_handlers]
            )
            print(f"Joints are:\n{strlist}")
            return
        except TimeoutError:
            raise TimeoutError("Joint data unavailable after 100 sec")

    async def ik_ready(self):
        """Returns once all ik are ready"""
        ready_tasks = [ih.ready for ih in self.ik_handlers]
        try:
            print("Waiting for ik.")
            fused_task = rao.gather(self, *ready_tasks)
            await rao.wait_for(self, fused_task, timeout_sec=100)
            print(f"Ik ready.")
            strlist = "\n".join(
                [f"limb {ih.limb_number}: {ih.ee_pose}" for ih in self.ik_handlers]
            )
            print(f"EE poses are:\n{strlist}")
            return
        except TimeoutError:
            raise TimeoutError("Ik data unavailable after 100 sec")

    def angles_to_zero(self) -> Coroutine:
        """sends all joints to 0.0"""
        target = {}
        for jh in self.joint_handlers:
            target.update({jname: 0.0 for jname in jh.tracked})

        task = self.joint_syncer.unsafe(target)
        return rao.wait_for(self, task, timeout_sec=100)

    async def ik_circle(self, samples: int = 20):
        """Executes a flat cricle trajectory.

        Args:
            samples: number of sample points making the circle trajectory.
        """
        s = samples
        s += 1
        radius = RADIUS
        ang = np.linspace(0, 2 * np.pi, s)
        yz = radius * np.exp(1j * ang)
        trajectory = np.zeros((s, 3), dtype=float)
        trajectory[:, 0] = yz.real
        trajectory[:, 1] = yz.imag

        # last_target = self.ik_syncer._previous_point(set(self.LIMBS))
        # start_poses = [last_target[h] for h in self.LIMBS]
        for ind in range(trajectory.shape[0]):
            target = {
                handler.limb_number: Pose(
                    time=ros_now(self),
                    xyz=self.default_stance[arr_ind, :] + trajectory[ind, :],
                    quat=None,
                )
                for arr_ind, handler in enumerate(self.ik_handlers)
                # for handler, start in zip(self.ik_handlers, start_poses)
            }
            task = self.ik_syncer.lerp(target)
            await rao.wait_for(self, task, timeout_sec=100)

    def stance(self) -> Coroutine:
        """Goes to the default moonbot zero stance using IK"""
        xyz_targets = self.default_stance
        target = {
            leg_num: Pose(
                time=ros_now(self),
                xyz=xyz_targets[arr_ind, :],
                quat=None,
            )
            for arr_ind, leg_num in enumerate(self.LIMBS)
        }
        return rao.wait_for(self, self.ik_syncer.lerp(target), timeout_sec=100)

    @error_catcher
    def startup(self):
        """Execute once at startup"""
        # Ros2 will executor will handle main()
        rao.ensure_future(self, self.main())

        # destroys timer
        self.destroy_timer(self.startTMR)
        print("Startup done.")

    @error_catcher
    def exec_loop(self):
        """Regularly executes the syncers"""
        self.joint_syncer.execute()
        self.ik_syncer.execute()

    @error_catcher
    def induce_bugTMRCKB(self):
        return
        target = {}
        bug_leg = np.random.randint(0, len(self.joint_handlers) - 1)
        print(bug_leg)

        jh = self.joint_handlers[bug_leg]
        target.update({jname: 0.0 for jname in jh.tracked})

        task = self.joint_syncer.unsafe(target)


def main(*args):
    my_main(TutoNode)

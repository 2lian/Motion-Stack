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

from typing import Coroutine

import numpy as np
from rclpy.node import Node

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


class TutoNode(Node):

    #: list of limbs number that are controlled
    LIMBS = [1, 2]

    def __init__(self) -> None:
        super().__init__("test_node")
        print("initing TutoNode")
        self.create_timer(1 / 30, self.exec_loop)  # regular execution
        self.startTMR = self.create_timer(0.1, self.startup)  # executed once

        # API objects:

        # Handles ros2 joints lvl1 (subscribers, publishers and more)
        self.joint_handlers = [JointHandler(self, l) for l in self.LIMBS]
        # Syncronises several joints
        self.joint_syncer = JointSyncerRos(self.joint_handlers)

        # Handles ros2 ik lvl2
        self.ik_handlers = [IkHandler(self, l) for l in self.LIMBS]
        # Syncronises several IK
        self.ik_syncer = IkSyncerRos(
            self.ik_handlers,
            interpolation_delta=XyzQuat(20, np.inf),
            on_target_delta=XyzQuat(2, np.inf),
        )

        self.get_logger().info("init done")

    @error_catcher
    async def main(self):
        # wait for all handlers to be ready
        await self.joints_ready()
        await self.ik_ready()

        # send to all angle at 0.0
        print("angles to zero starting")
        await self.angles_to_zero()
        print("angles to zero done")
        # send to all angle at pi/3
        # await self.angles_to_test(3.14 / 3 )
        # await self.angles_to_zero()
        
        # send end effectors to a delta pose using IK
        delta_pose = Pose(
            time=ros_now(self),
            xyz=np.array([-100, -100, -100], dtype=float),
            quat=qt.one,
        )
        await self.delta_ee(delta_pose)

        print("finished")

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

        task = self.joint_syncer.asap(target)
        return rao.wait_for(self, task, timeout_sec=100)

    def angles_to_test(self, test_angle) -> Coroutine:
        """sends all joints to 0.0"""
        target = {}
        for jh in self.joint_handlers:
            target.update({jname: test_angle for jname in jh.tracked})

        task = self.joint_syncer.asap(target)
        return rao.wait_for(self, task, timeout_sec=100)

    def delta_ee(self, delta_pose:Pose) -> Coroutine:
        # """moves the end effectors by a dela Pose using IK"""
        
        ee_poses = [ih.ee_pose for ih in self.ik_handlers]
       
        target = {
            leg_num: pose - delta_pose for leg_num, pose in zip(self.LIMBS, ee_poses)
        }

        task = self.ik_syncer.lerp(target)
        return rao.wait_for(self, task, timeout_sec=100)
    
    
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


def main(*args):
    my_main(TutoNode)

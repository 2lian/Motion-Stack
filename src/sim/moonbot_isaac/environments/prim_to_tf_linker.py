import logging
import threading
from dataclasses import dataclass
from queue import Queue
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from moonbot_isaac.tf_transform_monitor import TfTransformMonitor
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.physx import get_physx_interface
from pxr import Gf, Usd

from environments.config import TransformConfig
from environments.utils import set_attr


class PrimToTfLinker:
    def __init__(self, robot_prim: Usd.Prim, fixed_frame: str):
        self.robot_prim = robot_prim
        self.fixed_frame = fixed_frame
        self.world_to_tracked_queue = Queue()
        self.world_to_tracked_thread = threading.Thread(
            target=self.ros2_thread, args=(self.world_to_tracked_queue,)
        )
        self.world_to_tracked_thread.start()

        self._physx = get_physx_interface()
        self._physics_callback = self._physx.subscribe_physics_step_events(
            self.on_physics_step
        )

    def on_physics_step(self, event):
        # Get the latest transform from the ROS2 thread
        while not self.world_to_tracked_queue.empty():
            try:
                # Get and remove item from queue
                transform: TransformStamped = self.world_to_tracked_queue.get_nowait()

                prim = get_prim_at_path(f"{self.robot_prim.GetPath()}/{transform.child_frame_id}")
                if prim is not None and prim.IsValid():
                    set_attr(
                        prim,
                        "xformOp:translate",
                        Gf.Vec3f(
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z,
                        ),
                    )
                    set_attr(
                        prim,
                        "xformOp:orient",
                        Gf.Quatd(
                            transform.transform.rotation.w,
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                        ),
                    )
                    # Mark task as done
                    self.world_to_tracked_queue.task_done()
            except Queue.Empty:
                # Queue became empty between empty() check and get()
                break
    def ros2_thread(self, queue: Queue):
        if not rclpy.ok():
            rclpy.init()

        node = TfTransformMonitor(
            fixed_frame=self.fixed_frame,
            check_period_sec=0.04,
            queue=queue,
        )

        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

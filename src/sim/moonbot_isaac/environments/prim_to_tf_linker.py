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
from pxr import Gf

from environments.config import TransformConfig
from environments.utils import set_attr


@dataclass
class PrimToTfLinkerConfig:
    # The frame of the link in the robot that is tracked by the mocap system
    tracked_frame: str
    # The fixed frame of the mocap system
    fixed_frame: str
    # Transform of the fixed frame in the the Isaac Sim environment
    fixed_frame_offset: Optional[TransformConfig]
    # The path of the prim that represents the tracked link in the Isaac Sim environment
    tracked_prim: str


class PrimToTfLinker:
    def __init__(self, config: PrimToTfLinkerConfig):
        self.config = config
        self.tracked_prim = get_prim_at_path(config.tracked_prim)
        if self.tracked_prim is None or not self.tracked_prim.IsValid():
            raise ValueError(f"Could not find prim at path {config.tracked_prim}")

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
        if not self.world_to_tracked_queue.empty():
            transform: Optional[TransformStamped] = None
            while not self.world_to_tracked_queue.empty():
                transform = self.world_to_tracked_queue.get()
            if transform is not None:
                logging.error(f"Setting transform for {self.tracked_prim.GetPath()}")
                # Apply the transform to the tracked frame
                set_attr(
                    self.tracked_prim,
                    "xformOp:translate",
                    Gf.Vec3f(
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                    ),
                )
                set_attr(
                    self.tracked_prim,
                    "xformOp:orient",
                    Gf.Quatd(
                        transform.transform.rotation.w,
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                    ),
                )

    def ros2_thread(self, queue: Queue):
        if not rclpy.ok():
            rclpy.init()

        node = TfTransformMonitor(
            frame_id=self.config.fixed_frame,
            child_frame_id=self.config.tracked_frame,
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

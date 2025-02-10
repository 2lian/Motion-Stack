import threading
from queue import Queue
from typing import Optional

import omni
import omni.kit.commands
import omni.usd
import rclpy
from geometry_msgs.msg import TransformStamped
from moonbot_isaac.tf_transform_monitor import TfTransformMonitor
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import get_next_free_path
from omni.physx import get_physx_interface
from omni.physx.scripts.utils import createJoint
from pxr import Gf
from pydantic import BaseModel, Field

from environments.config import MocapLinkConfig
from environments.utils import set_attr


class MocapLink:
    def __init__(self, config: MocapLinkConfig):
        self.config = config
        # Create a kinematic prim for the mocap frame
        self.mocap_prim_path = get_next_free_path(
            f"mocap_{config.tracked_frame}", parent="/"
        )
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Xform",
            prim_path=self.mocap_prim_path,
            attributes={},
            select_new_prim=True,
        )
        self.mocap_prim = get_prim_at_path(self.mocap_prim_path)

        omni.kit.commands.execute(
            "AddPhysicsComponent",
            usd_prim=self.mocap_prim,
            component="PhysicsRigidBodyAPI",
        )

        tracked_prim = get_prim_at_path(config.tracked_prim)
        if tracked_prim is None or not tracked_prim.IsValid():
            raise ValueError(f"Could not find prim at path {config.tracked_prim}")

        joint = createJoint(
            omni.usd.get_context().get_stage(),
            joint_type="Fixed",
            from_prim=tracked_prim,
            to_prim=self.mocap_prim,
        )
        set_attr(self.mocap_prim_path, "physics:kinematicEnabled", True)
        set_attr(joint, "physics:excludeFromArticulation", True)

        self._physx = get_physx_interface()
        self._physics_callback = self._physx.subscribe_physics_step_events(
            self.on_physics_step
        )

        self.world_to_mocap_queue = Queue()
        self.world_to_mocap_thread = threading.Thread(
            target=self.ros2_thread, args=(self.world_to_mocap_queue,)
        )
        self.world_to_mocap_thread.start()

    def on_physics_step(self, event):
        # Get the latest transform from the ROS2 thread
        if not self.world_to_mocap_queue.empty():
            transform: Optional[TransformStamped] = None
            while not self.world_to_mocap_queue.empty():
                transform = self.world_to_mocap_queue.get()
            if transform is not None:
                # Apply the transform to the mocap frame
                self.mocap_prim = get_prim_at_path(self.mocap_prim_path)
                set_attr(
                    self.mocap_prim,
                    "xformOp:translate",
                    Gf.Vec3f(
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                    ),
                )
                set_attr(
                    self.mocap_prim,
                    "xformOp:orient",
                    Gf.Quatd(
                        transform.transform.rotation.w,
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                    ),
                )

    def ros2_thread(self, queue: Queue):
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

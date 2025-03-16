import threading
from dataclasses import dataclass
from enum import Enum
import logging

import omni.graph.core as og
import rclpy
from moonbot_isaac.realsense_extrinsics import RealsenseExtrinsicsNode
from omni.isaac.core.utils.stage import get_next_free_path
from omni.usd import get_context

from environments.config import RealsenseCameraConfig


class RealsenseCamera:
    """Add ROS2 api for the simulated Realsense camera"""

    class SensorType(Enum):
        COLOR = "color"
        DEPTH = "depth"

    def __init__(self, robot_prim: str, config: RealsenseCameraConfig):
        self.config = config
        self.robot_prim = robot_prim
        self.color_og_path = f"{robot_prim}/Graphs/ROS_Camera_Color"
        self.depth_og_path = f"{robot_prim}/Graphs/ROS_Camera_Depth"


    def initialize(self):
        # Add camera image publisher graphs
        self.add_image_publisher_graph(self.SensorType.COLOR)
        self.add_image_publisher_graph(self.SensorType.DEPTH)

        # Publish the Realsense specific depth-to-color-sensor extrinsics messages
        self.thread = threading.Thread(target=self.ros2_thread)
        self.thread.start()

    def ros2_thread(self):
        rclpy.init()

        node = RealsenseExtrinsicsNode()

        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    def add_image_publisher_graph(self, type: "RealsenseCamera.SensorType"):
        keys = og.Controller.Keys
        if type == self.SensorType.COLOR:
            camera_prim = f"{self.robot_prim}{self.config.color_camera_prim}"
            og_path = f"{self.robot_prim}/Graphs/ROS_Camera_Color"
            image_topic = self.config.color_image_topic
            camera_info_topic = self.config.color_camera_info_topic
            frame_id = self.config.color_frame_id
            image_type = "rgb"
        elif type == self.SensorType.DEPTH:
            camera_prim = f"{self.robot_prim}{self.config.depth_camera_prim}"
            og_path = f"{self.robot_prim}/Graphs/ROS_Camera_Depth"
            image_topic = self.config.depth_image_topic
            camera_info_topic = self.config.depth_camera_info_topic
            frame_id = self.config.depth_frame_id
            image_type = "depth"
        else:
            raise ValueError(f"Invalid camera type: {type}")
        
        if not get_context().get_stage().GetPrimAtPath(camera_prim).IsValid():
            logging.error(f"Camera prim {camera_prim} does not exist")
            return

        og_path = get_next_free_path(og_path, "")

        og.Controller.edit(
            {"graph_path": og_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    (
                        "CameraInfoPublish",
                        "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
                    ),
                    ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("CameraImagePublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("QoSProfile", "isaacsim.ros2.bridge.ROS2QoSProfile"),
                ],
                keys.SET_VALUES: [
                    ("RenderProduct.inputs:cameraPrim", camera_prim),
                    ("RenderProduct.inputs:width", self.config.width),
                    ("RenderProduct.inputs:height", self.config.height),
                    ("CameraInfoPublish.inputs:topicName", camera_info_topic),
                    ("CameraInfoPublish.inputs:frameId", frame_id),
                    ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                    ("CameraImagePublish.inputs:topicName", image_topic),
                    ("CameraImagePublish.inputs:type", image_type),
                    ("CameraImagePublish.inputs:resetSimulationTimeOnStop", True),
                    ("CameraImagePublish.inputs:frameId", frame_id),
                    ("QoSProfile.inputs:durability", "transientLocal"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                    ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                    (
                        "RenderProduct.outputs:execOut",
                        "CameraInfoPublish.inputs:execIn",
                    ),
                    (
                        "RenderProduct.outputs:renderProductPath",
                        "CameraInfoPublish.inputs:renderProductPath",
                    ),
                    ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
                    (
                        "RenderProduct.outputs:execOut",
                        "CameraImagePublish.inputs:execIn",
                    ),
                    (
                        "RenderProduct.outputs:renderProductPath",
                        "CameraImagePublish.inputs:renderProductPath",
                    ),
                    ("Context.outputs:context", "CameraImagePublish.inputs:context"),
                    (
                        "QoSProfile.outputs:qosProfile",
                        "CameraImagePublish.inputs:qosProfile",
                    ),
                ],
            },
        )

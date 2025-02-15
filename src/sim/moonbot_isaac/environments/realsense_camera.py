import threading
from dataclasses import dataclass
from enum import Enum

import omni.graph.core as og
import rclpy
from moonbot_isaac.realsense_extrinsics import RealsenseExtrinsicsNode
from omni.isaac.core.utils.stage import get_next_free_path
from omni.usd import get_context


@dataclass
class RealsenseCamera:
    """Add ROS2 api for the simulated Realsense camera"""

    color_camera_prim: str = "/robot/camera_color_optical_frame/d435i_color"
    depth_camera_prim: str = "/robot/camera_depth_optical_frame/d435i_depth"
    color_og_path: str = "/Graphs/ROS_Camera_Color"
    depth_og_path: str = "/Graphs/ROS_Camera_Depth"
    color_image_topic: str = "/camera/camera/color/image_raw"
    color_camera_info_topic: str = "/camera/camera/color/camera_info"
    depth_image_topic: str = "/camera/camera/depth/image_rect_raw"
    depth_camera_info_topic: str = "/camera/camera/depth/camera_info"
    color_frame_id: str = "camera_color_optical_frame"
    depth_frame_id: str = "camera_depth_optical_frame"
    # Currently, there is no straightforward way to get the camera resolution from the URDF
    width: int = 640
    height: int = 480

    class SensorType(Enum):
        COLOR = "color"
        DEPTH = "depth"

    def initialize(self) -> bool:
        """Retrun False if the camera prims do not exist"""
        # Check if camera prims exist and
        stage = get_context().get_stage()
        if (
            not stage.GetPrimAtPath(self.color_camera_prim).IsValid()
            or not stage.GetPrimAtPath(self.depth_camera_prim).IsValid()
        ):
            return False

        # Add camera image publisher graphs
        self.add_image_publisher_graph(self.SensorType.COLOR)
        self.add_image_publisher_graph(self.SensorType.DEPTH)

        # Publish the Realsense specific depth-to-color-sensor extrinsics messages
        self.thread = threading.Thread(target=self.ros2_thread)
        self.thread.start()

        return True

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
            camera_prim = self.color_camera_prim
            og_path = self.color_og_path
            image_topic = self.color_image_topic
            camera_info_topic = self.color_camera_info_topic
            frame_id = self.color_frame_id
            image_type = "rgb"
        elif type == self.SensorType.DEPTH:
            camera_prim = self.depth_camera_prim
            og_path = self.depth_og_path
            image_topic = self.depth_image_topic
            camera_info_topic = self.depth_camera_info_topic
            frame_id = self.depth_frame_id
            image_type = "depth"
        else:
            raise ValueError(f"Invalid camera type: {type}")

        og_path = get_next_free_path(og_path, "")

        og.Controller.edit(
            {"graph_path": og_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    (
                        "CameraInfoPublish",
                        "omni.isaac.ros2_bridge.ROS2CameraInfoHelper",
                    ),
                    ("RenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ("RunOnce", "omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),
                    ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("CameraImagePublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("QoSProfile", "omni.isaac.ros2_bridge.ROS2QoSProfile"),
                ],
                keys.SET_VALUES: [
                    ("RenderProduct.inputs:cameraPrim", camera_prim),
                    ("RenderProduct.inputs:width", self.width),
                    ("RenderProduct.inputs:height", self.height),
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

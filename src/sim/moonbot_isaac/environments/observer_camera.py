import omni.graph.core as og
import omni.kit.commands
from omni.isaac.core.utils.stage import get_next_free_path
from pxr import Gf, UsdGeom

from environments.config import (
    ObserverCameraConfig,  # Make sure this import path is correct for your project structure
)
from environments.isaac_utils import apply_transform_config


class ObserverCamera:
    """
    Manages the creation of an observer camera prim and its associated ROS2 publishing graph.
    """

    def __init__(self, config: ObserverCameraConfig):
        self.config = config

    def initialize(self) -> bool:
        """
        Creates the camera prim and the ROS OmniGraph for publishing camera data.
        Returns True on success, False otherwise.
        """
        self._camera_path = get_next_free_path(self.config.camera_path, "/")
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Camera",
            prim_path=self._camera_path,
            attributes={
                "projection": UsdGeom.Tokens.perspective,
                "focalLength": self.config.focal_length,
                "focusDistance": self.config.focus_distance,
                "clippingRange": self.config.clipping_range,
            },
        )
        apply_transform_config(prim=self._camera_path, transform=self.config.transform)

        self._add_ros_graph()
        return True

    def _add_ros_graph(self):
        """
        Creates and configures the OmniGraph for ROS2 communication.
        """
        keys = og.Controller.Keys
        actual_graph_path = get_next_free_path(self.config.graph_path, "/")

        og.Controller.edit(
            {
                "graph_path": actual_graph_path,
                "evaluator_name": "execution",
            },  # Removed pipeline_stage here
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                keys.SET_VALUES: [
                    # RenderProduct Node
                    ("RenderProduct.inputs:cameraPrim", self._camera_path),
                    ("RenderProduct.inputs:width", self.config.width),
                    ("RenderProduct.inputs:height", self.config.height),
                    ("RenderProduct.inputs:enabled", True),
                    # CameraInfoPublish Node
                    (
                        "CameraInfoPublish.inputs:nodeNamespace",
                        self.config.node_namespace,
                    ),
                    (
                        "CameraInfoPublish.inputs:topicName",
                        self.config.camera_info_topic_name,
                    ),  # Relative to namespace
                    ("CameraInfoPublish.inputs:frameId", self.config.frame_id),
                    ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                    ("CameraInfoPublish.inputs:enabled", True),
                    # RGBPublish Node
                    ("RGBPublish.inputs:nodeNamespace", self.config.node_namespace),
                    (
                        "RGBPublish.inputs:topicName",
                        self.config.rgb_topic_name,
                    ),  # Absolute path
                    ("RGBPublish.inputs:frameId", self.config.frame_id),
                    ("RGBPublish.inputs:type", "rgb"),  # As per the original USD
                    ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
                    ("RGBPublish.inputs:enabled", True),
                    # ROS2 Context Node
                    (
                        "Context.inputs:useDomainIDEnvVar",
                        True,
                    ),  # Default, can be configured
                    # ("Context.inputs:domain_id", 0) # Example if not using env var
                    # OnPlaybackTick & RunOnce nodes usually don't need specific values set here beyond connections
                ],
                keys.CONNECT: [
                    # Execution flow: OnPlaybackTick -> RunOnce -> RenderProduct
                    ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                    ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                    # RenderProduct outputs to CameraInfoPublish
                    (
                        "RenderProduct.outputs:execOut",
                        "CameraInfoPublish.inputs:execIn",
                    ),
                    (
                        "RenderProduct.outputs:renderProductPath",
                        "CameraInfoPublish.inputs:renderProductPath",
                    ),
                    # RenderProduct outputs to RGBPublish
                    ("RenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
                    (
                        "RenderProduct.outputs:renderProductPath",
                        "RGBPublish.inputs:renderProductPath",
                    ),
                    # ROS Context to publisher nodes
                    ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
                    ("Context.outputs:context", "RGBPublish.inputs:context"),
                ],
            },
        )

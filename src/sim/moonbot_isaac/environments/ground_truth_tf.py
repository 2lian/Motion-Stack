from dataclasses import dataclass

import omni.graph.core as og
from omni.isaac.core.utils.stage import get_next_free_path
from omni.usd import get_context


@dataclass
class GroundTruthTF:
    """Add ROS2 API for publishing ground truth transforms"""

    robot_prim: str = "/robot"
    og_path: str = "/Graphs/ROS_TF"
    tf_topic: str = "/tf_gt"

    def initialize(self) -> bool:
        """Return False if the robot prim does not exist"""
        # Check if robot prim exists
        stage = get_context().get_stage()
        if not stage.GetPrimAtPath(self.robot_prim).IsValid():
            return False

        # Add ground truth TF publisher graph
        self.add_tf_publisher_graph()
        return True

    def add_tf_publisher_graph(self):
        keys = og.Controller.Keys
        og_path = get_next_free_path(self.robot_prim + self.og_path, "")

        og.Controller.edit(
            {"graph_path": og_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("get_prim_at_path", "omni.replicator.core.OgnGetPrimAtPath"),
                    ("PublisherTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ],
                keys.SET_VALUES: [
                    ("PublisherTF.inputs:topicName", self.tf_topic),
                    ("PublisherTF.inputs:staticPublisher", False),
                    ("get_prim_at_path.inputs:paths", [self.robot_prim]),
                    ("ReadSimTime.inputs:resetOnStop", False),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "get_prim_at_path.inputs:execIn"),
                    ("get_prim_at_path.outputs:execOut", "PublisherTF.inputs:execIn"),
                    ("get_prim_at_path.outputs:prims", "PublisherTF.inputs:targetPrims"),
                    ("Context.outputs:context", "PublisherTF.inputs:context"),
                    ("ReadSimTime.outputs:simulationTime", "PublisherTF.inputs:timeStamp"),
                ],
            },
        )
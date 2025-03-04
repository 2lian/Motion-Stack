from dataclasses import dataclass
from pathlib import Path

import omni.graph.core as og
from omni.isaac.core.utils.stage import get_next_free_path
from omni.usd import get_context


@dataclass
class JointController:
    """Add ROS2 API for joint state control and publishing"""

    robot_prim: str = "/robot"
    og_path: str = "/Graphs/ROS_JointStates"
    joint_state_sub_topic: str = "/joint_commands_isaac"
    joint_state_pub_topic: str = "/joint_states_isaac"

    def initialize(self) -> bool:
        """Return False if the robot prim does not exist"""
        # Check if robot prim exists
        stage = get_context().get_stage()
        if not stage.GetPrimAtPath(self.robot_prim).IsValid():
            return False

        # Add joint controller graph
        self.add_joint_controller_graph()
        return True

    def add_joint_controller_graph(self):
        keys = og.Controller.Keys
        og_path = get_next_free_path(self.robot_prim + self.og_path, "")

        (graph_handle, nodes, _, _) = og.Controller.edit(
            {"graph_path": og_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("joint_state_filter", "omni.graph.scriptnode.ScriptNode"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("get_prim_at_path", "omni.replicator.core.OgnGetPrimAtPath"),
                    ("SubscriberJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("articulation_state", "isaacsim.core.nodes.IsaacArticulationState"),
                    ("ros2_publisher", "isaacsim.ros2.bridge.ROS2Publisher"),
                    ("isaac_time_splitter", "isaacsim.core.nodes.IsaacTimeSplitter"),
                ],
                keys.SET_VALUES: [
                    ("ReadSimTime.inputs:resetOnStop", False),
                    ("get_prim_at_path.inputs:paths", [self.robot_prim]),
                    ("SubscriberJointState.inputs:topicName", self.joint_state_sub_topic),
                    ("ros2_publisher.inputs:topicName", self.joint_state_pub_topic),
                    ("ros2_publisher.inputs:messagePackage", "sensor_msgs"),
                    ("ros2_publisher.inputs:messageName", "JointState"),
                    ("joint_state_filter.inputs:usePath", True),
                    ("joint_state_filter.inputs:scriptPath", str(Path(__file__).parent / "usd/node_scripts/filter_joint_states.py")),
                    
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "get_prim_at_path.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "SubscriberJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ros2_publisher.inputs:execIn"),
                    ("Context.outputs:context", "SubscriberJointState.inputs:context"),
                    ("Context.outputs:context", "ros2_publisher.inputs:context"),
                    ("get_prim_at_path.outputs:prims", "ArticulationController.inputs:targetPrim"),
                    ("get_prim_at_path.outputs:prims", "articulation_state.inputs:targetPrim"),
                    ("get_prim_at_path.outputs:execOut", "ArticulationController.inputs:execIn"),
                    ("get_prim_at_path.outputs:execOut", "articulation_state.inputs:execIn"),
                    ("SubscriberJointState.outputs:jointNames", "joint_state_filter.inputs:jointNames"),
                    ("SubscriberJointState.outputs:positionCommand", "joint_state_filter.inputs:positionCommand"),
                    ("SubscriberJointState.outputs:velocityCommand", "joint_state_filter.inputs:velocityCommand"),
                    ("SubscriberJointState.outputs:effortCommand", "joint_state_filter.inputs:effortCommand"),
                    ("articulation_state.outputs:jointNames", "joint_state_filter.inputs:allowed_joint_names"),
                    ("joint_state_filter.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                    ("joint_state_filter.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                    ("joint_state_filter.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                    ("joint_state_filter.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                    ("ReadSimTime.outputs:simulationTime", "isaac_time_splitter.inputs:time"),
                    ("OnPlaybackTick.outputs:tick", "joint_state_filter.inputs:execIn"),
                ],
                keys.CREATE_ATTRIBUTES: [
                    ("joint_state_filter.inputs:allowed_joint_names", "token[]"),
                    ("joint_state_filter.inputs:jointNames", "token[]"),
                    ("joint_state_filter.inputs:positionCommand", "double[]"),
                    ("joint_state_filter.inputs:velocityCommand", "double[]"),
                    ("joint_state_filter.inputs:effortCommand", "double[]"),
                    ("joint_state_filter.outputs:jointNames", "token[]"),
                    ("joint_state_filter.outputs:positionCommand", "double[]"),
                    ("joint_state_filter.outputs:velocityCommand", "double[]"),
                    ("joint_state_filter.outputs:effortCommand", "double[]"),
                ],
            },
        )
        
        # Connect dynamic values after graph creation
        og_path = graph_handle.get_path_to_graph()
        og.Controller.connect(
            og.Controller.attribute(og_path + "/articulation_state.outputs:jointNames"),
            og.Controller.attribute(og_path + "/ros2_publisher.inputs:name"),
        )
        og.Controller.connect(
            og.Controller.attribute(og_path + "/articulation_state.outputs:jointPositions"),
            og.Controller.attribute(og_path + "/ros2_publisher.inputs:position"),
        )
        og.Controller.connect(
            og.Controller.attribute(og_path + "/articulation_state.outputs:jointVelocities"),
            og.Controller.attribute(og_path + "/ros2_publisher.inputs:velocity"),
        )
        og.Controller.connect(
            og.Controller.attribute(og_path + "/articulation_state.outputs:measuredJointEfforts"),
            og.Controller.attribute(og_path + "/ros2_publisher.inputs:effort"),
        )
        og.Controller.connect(
            og.Controller.attribute(og_path + "/isaac_time_splitter.outputs:seconds"),
            og.Controller.attribute(og_path + "/ros2_publisher.inputs:header:stamp:sec"),
        )
        og.Controller.connect(
            og.Controller.attribute(og_path + "/isaac_time_splitter.outputs:nanoseconds"),
            og.Controller.attribute(og_path + "/ros2_publisher.inputs:header:stamp:nanosec"),
        )

        
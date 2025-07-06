from pathlib import Path
from typing import Any, Dict, List, Literal, Optional, Union

import numpy as np
import toml
from pydantic import BaseModel, Field, field_validator, model_validator


class ConfigError(Exception):
    pass


class CameraConfig(BaseModel):
    translation: List[float] = Field(
        default_factory=lambda: [
            -0.3577958949555765,
            -1.1875695366976564,
            0.632201840815314,
        ]
    )
    target: List[float] = Field(default_factory=lambda: [0.4, 0.4, 0.0])


class LightConfig(BaseModel):
    intensity: float = 1000


class TransformConfig(BaseModel):
    translation: List[float] = Field(default_factory=lambda: [0, 0, 0])
    rotation: List[float] = Field(default_factory=lambda: [1, 0, 0, 0])
    scale: List[float] = Field(default_factory=lambda: [1.0, 1.0, 1.0])


class JointPositionDetail(BaseModel):
    value: float
    degree: bool = False


class RealsenseCameraConfig(BaseModel):
    """Add ROS2 api for the simulated Realsense camera"""

    color_image_topic: str = "/camera/camera/color/image_raw"
    color_camera_info_topic: str = "/camera/camera/color/camera_info"
    depth_image_topic: str = "/camera/camera/depth/image_rect_raw"
    depth_camera_info_topic: str = "/camera/camera/depth/camera_info"
    color_frame_id: str = "camera_color_optical_frame"
    depth_frame_id: str = "camera_depth_optical_frame"
    # Prim path relative to the robot. Should be '/{color_frame_id}/d435i_color'
    color_camera_prim: str = "/camera_color_optical_frame/d435i_color"
    # Prim path relative to the robot. Should be '/{depth_frame_id}/d435i_depth'
    depth_camera_prim: str = "/camera_depth_optical_frame/d435i_depth"
    # Currently, there is no straightforward way to get the camera resolution from the URDF
    width: int = 640
    height: int = 480


class RobotConfig(BaseModel):
    name: str = "robot"
    xacro_path: Optional[str] = None
    # Optional key-value dictionary of XACRO parameters
    xacro_params: Optional[Dict[str, str]] = None
    robot_description_topic: Optional[str] = None
    visualization_mode: bool = False
    # The fixed frame for the robot visualization (like in RViz)
    visualization_fixed_frame: Optional[str] = "world"
    # Initial transform of the robot
    transform: Optional[TransformConfig] = None
    # Initial joint positions for the robot, mapping joint name to position.
    # Values can be a float (radians) or a dict {value: float, degree: bool}
    initial_joint_positions: Optional[Dict[str, float]] = None
    # Implement mimic joints as mimic joints instead of separate joints with different  drives
    parse_mimic_joints: bool = True
    # Do not implement controls for this robot. Always true in visualization mode.
    without_controls: bool = False
    realsense_camera: Optional[RealsenseCameraConfig] = None
    # Publish the ground truth TF with gt__ prefix. Off in visualization mode
    publish_ground_truth_tf: bool = False

    @field_validator("initial_joint_positions", mode="before")
    @classmethod
    def _process_joint_positions(cls, v):
        if not v:
            return v
        processed = {}
        for joint, value in v.items():
            if isinstance(value, dict):
                detail = JointPositionDetail.model_validate(value)
                if detail.degree:
                    processed[joint] = np.deg2rad(detail.value)
                else:
                    processed[joint] = detail.value
            else:
                # Assumes float/int for radians
                processed[joint] = float(value)
        return processed

    @model_validator(mode="before")
    @classmethod
    def check_mutually_exclusive(cls, values):
        if isinstance(values, dict):
            xacro = values.get("xacro_path")
            topic = values.get("robot_description_topic")
            if xacro is not None and topic is not None:
                raise ValueError(
                    "Cannot specify both xacro_path and robot_description_topic"
                )
            if xacro is None and topic is None:
                raise ValueError(
                    "Must specify either xacro_path or robot_description_topic"
                )
        return values


class ObserverCameraConfig(BaseModel):
    """Configuration for the ObserverCamera."""

    graph_path: str = "/observer/graph"  # Default graph suffix

    # Camera prim settings
    camera_path: str = "/observer/camera"
    clipping_range: tuple[float, float] = (0.01, 10000000.0)
    focal_length: float = 18.147562
    focus_distance: float = 400.0
    transform: Optional[TransformConfig] = None

    # ROS OmniGraph settings
    width: int = 1280  # Default width
    height: int = 720  # Default height
    node_namespace: str = "observer_camera"
    camera_info_topic_name: str = "camera_info"  # Relative to node_namespace
    rgb_topic_name: str = "/rgb"  # Can be absolute or relative to node_namespace depending on ROS2CameraHelper behavior
    frame_id: str = "observer_camera_frame"


class RigidBodyConfig(BaseModel):
    # True means the rigid body won't be affected by physics (like gravity or collisions)
    kinematic: bool = False
    # Select collision mesh approximation method
    approximation_shape: Literal[
        "none",
        "convexHull",
        "convexDecomposition",
        "meshSimplification",
        "convexMeshSimplification",
        "boundingCube",
        "boundingSphere",
        "sphereFill",
        "sdf",
    ] = "meshSimplification"


class UsdReferenceConfig(BaseModel):
    """Imports a .usd* file into the simulation."""

    path: str
    # Name to use for the added prim
    name: Optional[str] = None
    rigid_body: Optional[RigidBodyConfig] = None
    # Set prim attributes after loading the USD file (prim paths are relative to the added prim)
    prim_properties: Dict[str, Any] = Field(default_factory=dict)
    transform: Optional[TransformConfig] = None


class GroundPlaneConfig(BaseModel):
    transform: Optional[TransformConfig] = None


class SimConfig(BaseModel):
    robots: List[RobotConfig] = Field(default_factory=list)
    observer_cameras: List[ObserverCameraConfig] = Field(default_factory=list)
    ground: GroundPlaneConfig = Field(default_factory=GroundPlaneConfig)
    camera: CameraConfig = Field(default_factory=CameraConfig)
    light: LightConfig = Field(default_factory=LightConfig)
    usd_references: List[UsdReferenceConfig] = Field(default_factory=list)


# Extended schema for making the TOML configs more ergonomic
class SimConfigToml(SimConfig):
    robot: Union[RobotConfig, List[RobotConfig]] = Field(default_factory=list)
    observer_camera: Union[ObserverCameraConfig, List[ObserverCameraConfig]] = Field(
        default_factory=list
    )
    usd_reference: Union[UsdReferenceConfig, List[UsdReferenceConfig]] = Field(
        default_factory=list
    )

    def to_sim_config(self) -> SimConfig:
        # Consolidate robots from both fields
        kwargs = self.model_dump()

        if isinstance(self.robot, list):
            kwargs["robots"].extend(self.robot)
        else:
            kwargs["robots"].append(self.robot)
        del kwargs["robot"]

        # Consolidate observer cameras
        observer_cameras = []
        if isinstance(self.observer_camera, list):
            observer_cameras.extend(self.observer_camera)
        else:
            observer_cameras.append(self.observer_camera)
        kwargs["observer_cameras"] = observer_cameras
        del kwargs["observer_camera"]

        # Consolidate USD references
        usd_references = []
        if isinstance(self.usd_reference, list):
            usd_references.extend(self.usd_reference)
        else:
            usd_references.append(self.usd_reference)
        kwargs["usd_references"] = usd_references
        del kwargs["usd_reference"]

        return SimConfig(**kwargs)


def load_config(file_path: str) -> SimConfig:
    from environments.ros_utils import replace_package_urls_with_paths

    # if not package:// or absolute path, prefix with this package://'s config path
    if not file_path.startswith("package://") and not Path(file_path).is_absolute():
        prefix = "package://moonbot_isaac/config/"
        file_path = prefix + file_path

    file_path = replace_package_urls_with_paths(file_path)
    try:
        path = Path(file_path)
        data = toml.load(path)
        toml_config = SimConfigToml.model_validate(data)
        return toml_config.to_sim_config()
    except FileNotFoundError:
        raise ConfigError(f"Config file not found at {path}")
    except Exception as e:
        raise ConfigError(f"Config error: {e}")

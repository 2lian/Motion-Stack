from pathlib import Path
from typing import List, Optional

import toml
from pydantic import BaseModel, Field, model_validator

from environments.ros_utils import replace_package_urls_with_paths


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
    robot_description_topic: Optional[str] = None
    visualization_mode: bool = False
    # The fixed frame for the robot visualization (like in RViz)
    visualization_fixed_frame: Optional[str] = "world"
    # Initial transform of the robot
    transform: Optional[TransformConfig] = None
    # Implement mimic joints as mimic joints instead of separate joints with different  drives
    parse_mimic_joints: bool = False
    # Do not implement controls for this robot. Always true in visualization mode.
    without_controls: bool = False
    realsense_camera: Optional[RealsenseCameraConfig] = None
    # Publish the ground truth TF with gt__ prefix. Off in visualization mode
    publish_ground_truth_tf: bool = False

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


class GroundPlaneConfig(BaseModel):
    transform: Optional[TransformConfig] = None


class SimConfig(BaseModel):
    robots: List[RobotConfig] = Field(default_factory=list)
    ground: GroundPlaneConfig = None
    camera: CameraConfig = Field(default_factory=CameraConfig)
    light: LightConfig = Field(default_factory=LightConfig)


def load_config(file_path: str) -> SimConfig:
    file_path = replace_package_urls_with_paths(file_path)
    try:
        path = Path(__file__).parent.parent / "config" / file_path
        data = toml.load(path)
        return SimConfig.model_validate(data)
    except FileNotFoundError:
        raise ConfigError(f"Config file not found at {path}")
    except Exception as e:
        raise ConfigError(f"Config error: {e}")


if __name__ == "__main__":
    print("Creating an example config file")

    config = SimConfig(
        robots=[
            RobotConfig(
                name="robot",
                xacro_path="path/to/robot.xacro",
                transform=TransformConfig(
                    translation=[0, 0, 0.1], rotation=[1, 0, 0, 0]
                ),
            ),
            RobotConfig(
                name="robot",
                robot_description_topic="robot_description",
                visualization_mode=True,
            ),
        ],
        ground=GroundPlaneConfig(
            transform=TransformConfig(translation=[0, 0, 0], rotation=[1, 0, 0, 0])
        ),
        camera=CameraConfig(
            position=[-0.3577958949555765, -1.1875695366976564, 0.632201840815314],
            target=[0.4, 0.4, 0.0],
        ),
        light=LightConfig(intensity=1000),
    )
    config_path = Path(__file__).parent.parent / "config" / "example.toml"
    with open(config_path, "w") as f:
        # Write the config to a file in TOML format
        config_dict = config.model_dump()
        toml.dump(config_dict, f)

    print(f"Config file created at {config_path}")

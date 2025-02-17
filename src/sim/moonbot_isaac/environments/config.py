from typing import List, Optional
from pathlib import Path
import toml
from pydantic import BaseModel, Field


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


class MocapLinkConfig(BaseModel):
    """
    Applies the selected transform in simulation
    It will create a kinematic (not affected by physics) prim for the mocap frame and a
    """
    tracked_frame: str = Field(
        default="base_link",
        description="The frame of the link in the robot that is tracked by the mocap system",
    )
    fixed_frame: str = Field(
        default="world", description="The fixed frame of the mocap system"
    )
    fixed_frame_offset: Optional["TransformConfig"] = Field(
        default=None,
        description="Transform of the fixed frame in the the Isaac Sim environment",
    )
    tracked_prim: Optional[str] = Field(
        default=None,
        description="The path of the prim that represents the tracked link in the Isaac Sim environment",
    )



class TransformConfig(BaseModel):
    translation: List[float] = Field(default_factory=lambda: [0, 0, 0])
    rotation: List[float] = Field(default_factory=lambda: [1, 0, 0, 0])


class RobotConfig(BaseModel):
    name: str = "robot"
    xacro_path: Optional[str] = None
    robot_description_topic: Optional[str] = None
    visualization_mode: bool = False
    preserve_joints_of_links_in_visualization_mode: List[str] = Field(
        default_factory=list
    )
    transform: Optional[TransformConfig] = None

class GroundPlaneConfig(BaseModel):
    transform: Optional[TransformConfig] = None


class SimConfig(BaseModel):
    robot: RobotConfig
    ground: GroundPlaneConfig = None
    camera: CameraConfig = Field(default_factory=CameraConfig)
    light: LightConfig = Field(default_factory=LightConfig)


def load_config(file_path: str) -> SimConfig:
    try:
        path = Path(__file__).parent.parent / "config" / file_path
        data = toml.load(path)
        return SimConfig.parse_obj(data)
    except FileNotFoundError:
        raise ConfigError(f"Config file not found at {path}")
    except Exception as e:
        raise ConfigError(f"Config error: {e}")
    

if __name__ == "__main__":
    print("Creating an example config file")

    config = SimConfig(
        robot=RobotConfig(
            name="robot",
            xacro_path="path/to/robot.xacro",
            robot_description_topic="robot_description",
            visualization_mode=False,
            preserve_joints_of_links_in_visualization_mode=["link1", "link2"],
            transform=TransformConfig(translation=[0, 0, 0], rotation=[1, 0, 0, 0])
        ),
        ground=GroundPlaneConfig(transform=TransformConfig(translation=[0, 0, 0], rotation=[1, 0, 0, 0])),
        camera=CameraConfig(position=[-0.3577958949555765, -1.1875695366976564, 0.632201840815314], target=[0.4, 0.4, 0.0]),
        light=LightConfig(intensity=1000)
    )
    config_path = Path(__file__).parent.parent / "config" / "example.toml"
    with open(config_path, "w") as f:
        # Write the config to a file in TOML format
        config_dict = config.model_dump()
        toml.dump(config_dict, f)

    print(f"Config file created at {config_path}")

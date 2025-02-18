from pathlib import Path
from typing import List, Optional

import toml
from pydantic import BaseModel, Field, model_validator


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


class RobotConfig(BaseModel):
    name: str = "robot"
    xacro_path: Optional[str] = None
    robot_description_topic: Optional[str] = None
    visualization_mode: bool = False
    transform: Optional[TransformConfig] = None

    @model_validator(mode='before')
    @classmethod
    def check_mutually_exclusive(cls, values):
        if isinstance(values, dict):
            xacro = values.get('xacro_path')
            topic = values.get('robot_description_topic')
            if xacro is not None and topic is not None:
                raise ValueError('Cannot specify both xacro_path and robot_description_topic')
            if xacro is None and topic is None:
                raise ValueError('Must specify either xacro_path or robot_description_topic')
        return values


class GroundPlaneConfig(BaseModel):
    transform: Optional[TransformConfig] = None


class SimConfig(BaseModel):
    robots: List[RobotConfig] = Field(default_factory=list)
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
        robots=[
            RobotConfig(
                name="robot",
                xacro_path="path/to/robot.xacro",
                visualization_mode=False,
                transform=TransformConfig(translation=[0, 0, 0], rotation=[1, 0, 0, 0]),
            ),
            RobotConfig(
                name="robot",
                xacro_path="path/to/robot.xacro",
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

# Configuration Options

This document describes the configuration options available for the simulation. This file was LLM generated based on [`config.py`](./environments/config.py).

## CameraConfig

- `translation`: List of float values representing the camera translation. Default is `[-0.3577958949555765, -1.1875695366976564, 0.632201840815314]`.
- `target`: List of float values representing the camera target. Default is `[0.4, 0.4, 0.0]`.

## LightConfig

- `intensity`: Float value representing the light intensity. Default is `1000`.

## TransformConfig

- `translation`: List of float values representing the translation. Default is `[0, 0, 0]`.
- `rotation`: List of float values representing the rotation. Default is `[1, 0, 0, 0]`.

## RobotConfig

- `name`: String representing the robot name. Default is `"robot"`.
- `xacro_path`: Optional string representing the path to the robot xacro file. Mutually exclusive with `robot_description_topic`.
- `robot_description_topic`: Optional string representing the robot description topic. Mutually exclusive with `xacro_path`.
- `visualization_mode`: Boolean indicating if visualization mode is enabled. Default is `False`.
- `visualization_fixed_frame`: Optional string representing the fixed frame for the robot visualization (like in RViz). Default is `"world"`.
- `transform`: Optional `TransformConfig` object representing the initial transform of the robot.

## GroundPlaneConfig

- `transform`: Optional `TransformConfig` object.

## SimConfig

- `robots`: List of `RobotConfig` objects. Default is an empty list.
- `ground`: `GroundPlaneConfig` object.
- `camera`: `CameraConfig` object. Default is a new `CameraConfig` object.
- `light`: `LightConfig` object. Default is a new `LightConfig` object.

## Example

```toml
[[robots]]
name = "robot"
xacro_path = "path/to/robot.xacro"
[robots.transform]
translation = [0, 0, 0.1]
rotation = [1, 0, 0, 0]

[[robots]]
name = "robot"
robot_description_topic = "robot_description"
visualization_mode = true

[ground]
[ground.transform]
translation = [0, 0, 0]
rotation = [1, 0, 0, 0]

[camera]
translation = [-0.3577958949555765, -1.1875695366976564, 0.632201840815314]
target = [0.4, 0.4, 0.0]

[light]
intensity = 1000
```

For more examples, see the [`config`](./config/example.toml) directory.

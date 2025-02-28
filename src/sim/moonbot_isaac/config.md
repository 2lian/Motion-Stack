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

## RealsenseCameraConfig

- `color_image_topic`: String representing the color image topic. Default is `"/camera/camera/color/image_raw"`.
- `color_camera_info_topic`: String representing the color camera info topic. Default is `"/camera/camera/color/camera_info"`.
- `depth_image_topic`: String representing the depth image topic. Default is `"/camera/camera/depth/image_rect_raw"`.
- `depth_camera_info_topic`: String representing the depth camera info topic. Default is `"/camera/camera/depth/camera_info"`.
- `color_frame_id`: String representing the color frame ID. Default is `"camera_color_optical_frame"`.
- `depth_frame_id`: String representing the depth frame ID. Default is `"camera_depth_optical_frame"`.
- `color_camera_prim`: String representing the prim path relative to the robot. Default is `"/camera_color_optical_frame/d435i_color"`.
- `depth_camera_prim`: String representing the prim path relative to the robot. Default is `"/camera_depth_optical_frame/d435i_depth"`.
- `width`: Integer representing the camera width in pixels. Default is `640`.
- `height`: Integer representing the camera height in pixels. Default is `480`.

## RobotConfig

- `name`: String representing the robot name. Default is `"robot"`.
- `xacro_path`: Optional string representing the path to the robot xacro file. Mutually exclusive with `robot_description_topic`.
- `robot_description_topic`: Optional string representing the robot description topic. Mutually exclusive with `xacro_path`.
- `visualization_mode`: Boolean indicating if visualization mode is enabled. Default is `False`.
- `visualization_fixed_frame`: Optional string representing the fixed frame for the robot visualization (like in RViz). Default is `"world"`.
- `transform`: Optional `TransformConfig` object representing the initial transform of the robot.
- `parse_mimic_joints`: Boolean indicating whether to implement mimic joints as mimic joints instead of separate joints with different drives. Default is `False`.
- `without_controls`: Boolean indicating whether to skip implementing controls for this robot. Default is `False`.
- `realsense_camera`: Optional `RealsenseCameraConfig` object for configuring the simulated Realsense camera.
- `publish_ground_truth_tf`: Boolean indicating whether to publish the ground truth TF with gt__ prefix. Default is `False`. Always off in visualization mode.

**Note**: You must specify either `xacro_path` OR `robot_description_topic`, not both. At least one must be provided.

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
parse_mimic_joints = false
without_controls = false
publish_ground_truth_tf = true
[robots.transform]
translation = [0, 0, 0.1]
rotation = [1, 0, 0, 0]
[robots.realsense_camera]
width = 640
height = 480

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

[[blendfiles]]
path = "package://moonbot_isaac/environments/blender/scene.blend"
```

For more examples, see the [`config`](./config/example.toml) directory.

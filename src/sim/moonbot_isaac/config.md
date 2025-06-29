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
- `xacro_params`: Optional dictionary of key-value pairs representing XACRO parameters.
- `robot_description_topic`: Optional string representing the robot description topic. Mutually exclusive with `xacro_path`.
- `visualization_mode`: Boolean indicating if visualization mode is enabled. Default is `False`.
- `visualization_fixed_frame`: Optional string representing the fixed frame for the robot visualization (like in RViz). Default is `"world"`.
- `transform`: Optional `TransformConfig` object representing the initial transform of the robot.
- `initial_joint_positions`: Optional dictionary mapping joint names to initial positions. Values can be floats (radians) or objects with `value` (float) and `degree` (boolean) fields. Default is `None`.
- `parse_mimic_joints`: Boolean indicating whether to implement mimic joints as mimic joints instead of separate joints with different drives. Default is `True`.
- `without_controls`: Boolean indicating whether to skip implementing controls for this robot. Default is `False`.
- `realsense_camera`: Optional `RealsenseCameraConfig` object for configuring the simulated Realsense camera.
- `publish_ground_truth_tf`: Boolean indicating whether to publish the ground truth TF with gt__ prefix. Default is `False`. Always off in visualization mode.

**Note**: You must specify either `xacro_path` OR `robot_description_topic`, not both. At least one must be provided.

## ObserverCameraConfig

- `graph_path`: String representing the graph path for the observer camera. Default is `"/observer/graph"`.
- `camera_path`: String representing the camera prim path for the observer camera. Default is `"/observer/camera"`.
- `clipping_range`: Tuple of two floats representing the near and far clipping distances. Default is `(0.01, 10000000.0)`.
- `focal_length`: Float representing the focal length of the camera in millimeters. Default is `18.147562`.
- `focus_distance`: Float representing the distance from the camera to the focus plane in millimeters. Default is `400.0`.
- `transform`: Optional `TransformConfig` object representing the initial transform of the observer camera.
- `width`: Integer representing the camera width in pixels. Default is `1280`.
- `height`: Integer representing the camera height in pixels. Default is `720`.
- `node_namespace`: String representing the ROS node namespace for the camera. Default is `"observer_camera"`.
- `camera_info_topic_name`: String representing the camera info topic name (relative to `node_namespace`). Default is `"camera_info"`.
- `rgb_topic_name`: String representing the RGB image topic name. Default is `"/rgb"`.
- `frame_id`: String representing the TF frame ID for the camera. Default is `"observer_camera_frame"`.

## RigidBodyConfig

- `kinematic`: Boolean indicating if the rigid body won't be affected by physics (like gravity or collisions). Default is `False`.

## UsdReferenceConfig
Load a USD file in the simulation
- `path`: String representing the path to the USD file to import into the simulation.
- `name`: Optional string representing the name to use for the added prim.
- `rigid_body`: Optional `RigidBodyConfig` object for configuring physics properties.
- `prim_properties`: Dictionary of key-value pairs for setting prim attributes after loading the USD file (prim paths are relative to the added prim). Default is an empty dictionary.
- `transform`: Optional `TransformConfig` object representing the initial transform of the USD reference.

## GroundPlaneConfig

- `transform`: Optional `TransformConfig` object.

## Root fields

- `robot`: One or more `RobotConfig` objects. Default is an empty list.
- `observer_camera`: One or more of `ObserverCameraConfig` objects. Default is an empty list.
- `usd_reference`: One or more `UsdReferenceConfig` objects for importing USD files. Default is an empty list.
- `ground`: `GroundPlaneConfig` object.
- `camera`: `CameraConfig` object. Default is a new `CameraConfig` object.
- `light`: `LightConfig` object. Default is a new `LightConfig` object.

## Example

```toml
#:schema schema.json

[[robot]]
name = "robot"
xacro_path = "path/to/robot.xacro"
parse_mimic_joints = false
without_controls = false
publish_ground_truth_tf = true
[robot.transform]
translation = [0, 0, 0.1]
rotation = [1, 0, 0, 0]
[robot.initial_joint_positions]
joint1 = 1.57  # radians
joint2 = { value = 90, degree = true }  # degrees converted to radians
[robot.realsense_camera]
width = 640
height = 480

[[robot]]
name = "robot2"
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

[[observer_camera]]
node_namespace = "observer_camera1"
frame_id = "observer1_frame"
[observer_camera.transform]
translation = [-0.6, -1.2, 1.6]
rotation = [0.82, 0.46, -0.19, -0.23]

[[usd_reference]]
path = "package://example_moonbot_demo/meshes/environment.usdc"
name = "environment"
[usd_reference.rigid_body]
kinematic = true
approximation_shape = "meshSimplification"
[usd_reference.transform]
translation = [0, 0, 0]
rotation = [0, 0, 0, 1]
[usd_reference.prim_properties]
"some/prim/path.attribute" = "value"

```

For more examples, see the [`config`](./config/example.toml) directory.

## Linting

Use a TOML language extension like [Taplo](https://marketplace.visualstudio.com/items?itemName=tamasfe.even-better-toml) to enable type hints in the config files.

To regenerate the schema file, run `python3 path/to/moonbot_isaac/config/generate_schema.py`


# Your URDF with This Repo

## Example and Overview

A properly working URDF is provided. Refere to it as needed.

- The name of our robot is `moonbot_zero`.
- [src/urdf_packer/urdf/moonbot_7/moonbot_7.xacro](https://github.com/2lian/Moonbot-Motion-Stack/blob/main/src/urdf_packer/urdf/moonbot_7/moonbot_7.xacro): `moonbot_zero`’s URDF, with a few modifications and xacro imports.
- [Meshes for moonbot_hero](https://github.com/2lian/Moonbot-Motion-Stack/blob/main/src/urdf_packer/meshes/moonbot_7/): Meshes are located here.

## Setup Explanation

Let’s go into details on how to set up your URDF, or rather, your `.xacro` file. This is usually not a problem when a single URDF is used at a time – everything (URDF and meshes) is copied into the root `package://` directory, and everyone hopes for the best. Because we offer the possibility of loading and swapping several different robots, we need a better file structure to avoid conflicts.

- **URDF/xacro file location**: Place your `.urdf` or `.xacro` file in `src/urdf_packer/urdf/<name_of_your_robot>/<name_of_your_robot>.xacro`. Follow the steps below to modify it:
  - If you use a `.urdf`, rename it as a `.xacro`.
  - Open your `.xacro` file with a text editor.
  - Modify the top of your `<name_of_your_robot>.xacro` file so it looks like this (replace `|||name_of_your_robot|||` accordingly):
    ```xml
    <?xml version="1.0" ?>
    <robot name="|||name_of_your_robot|||" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:property name="ThisPackage" value="urdf_packer" />
    <xacro:property name="Filename" value="|||name_of_your_robot|||" />
    <xacro:property name="UrdfDataPath" value="package://${ThisPackage}/urdf/${Filename}" />
    <xacro:property name="MeshPath" value="package://${ThisPackage}/meshes/${Filename}" />
    ```
  - (Optional) If you have imports in your .xacro file (e.g., for Gazebo), you can change them as follows:
    ```xml
    <xacro:include filename="materials.xacro" />
    <xacro:include filename="${Filename}.trans" />
    <xacro:include filename="${Filename}.gazebo" />
    ```
  - Change all filename properties of your meshes in the .xacro file to the following pattern. At launch time, xacro will automatically replace ${MeshPath} with the correct path defined earlier:
    ```xml
    <geometry>
      <mesh filename="${MeshPath}/base_link.STL" />
    </geometry>
    ```
  - Only change the `filename="SOMETHING_SOMETHING/<file>.STL"` to `filename="${MeshPath}/<file>.STL"` property in the geometry/mesh. Do not delete other properties that may be necessary for your URDF, such as `<mesh ... scale="0.001 0.001 0.001" />`.
  - If you are not using meshes and want to avoid errors, delete all `<mesh> ... </mesh>` lines from your `.xacro`/`.urdf`.
- **Mesh file location**: Place your `.stl` files inside the folder `src/urdf_packer/meshes/<name_of_your_robot>/`.
- **Select your URDF**:
  The motion stack launch API will handle it for you. You only need to provide the name of your robot to the [LevelBuilder](../api/easy_robot_control/easy_robot_control.launch.md#level-builder-label) like what is done in [the moonbot_zero sublauncher](https://github.com/2lian/Moonbot-Motion-Stack/blob/main/src/easy_robot_control/launch/moonbot_zero.launch.py). In depth tutorial to customize the motion stack for your robot is provided int the [API](api.md#api-label) section.

```python
"""Example of a "sub launcher" or launchpy for monbot zero.
"""

from easy_robot_control.launch.builder import LevelBuilder


# V Change default parameters here V
#   \  /   #
#    \/    #
ROBOT_NAME = "moonbot_7"  # name of the xacro to load

# leg number -> end effector (number or link name)
LEGS_DIC = {
    1: "end1",
    2: "end2",
    3: "end3",
    4: "end4",
}

lvl_builder = LevelBuilder(robot_name=ROBOT_NAME, leg_dict=LEGS_DIC)
#    /\    #
#   /  \   #
# ^ Change default parameters here ^



def generate_launch_description():
    return lvl_builder.make_description()
```

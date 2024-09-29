# How to use your URDF with this repo

## Example and overview
This file is working and properly setup. Please chech it out to understand better.
- The name of our robot is `moonbot_hero`
- [`/src/urdf_packer/urdf/moonbot_hero/moonbot_hero.xacro`](/src/urdf_packer/urdf/moonbot_hero/moonbot_hero.xacro): A few modifications have been performed, and xacro imports are around this file.
- [`/src/urdf_packer/meshes/moonbot_hero/`](/src/urdf_packer/meshes/moonbot_hero): Meshes are placed here.
- [`general_launch_settings.py`](/general_launch_settings.py): Select which URDF to load. This will pass down the urdf to all lauchers and node. You can also change the default, in case ros2 parameter is not passed:
  - [`/src/easy_robot_control/launch/launch_setting.py`](/src/easy_robot_control/launch/launch_setting.py): Sets default urdf at launch for the main motion stack.
  - [`/src/urdf_packer/launch/rviz.launch.py#L12`](/src/urdf_packer/launch/rviz.launch.py): Sets default URDF data for Rviz only.


## Setup explanation

Let's go in details on how to setup your urdf or rather .xacro. This is usually not a problem when a single URDF is used at a time, everything (urdf and meshes) are copied in the root `package://` and everyone hopes and prays for the best. Because we offer the possibility of several diffrent robots being loaded and swaped, we need a better file structure to avoid conflicts.

- Your .urdf or .xacro file should be in [`/src/urdf_packer/urdf/`](/src/urdf_packer/urdf)`<name of your robot>/<name of your robot>.xacro`. You will need to change it following the guidline below.
  - If you use a .urdf, rename it as a .xacro.
  - Open your .xacro with a text editor.
  - Modify the top of your `<name of your robot>.xacro` so it looks like the provided code below (replace `|||name of your robot|||` accordingly):
  ```xml
  <?xml version="1.0" ?>
  <robot name="|||name of your robot|||" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <xacro:property name="ThisPackage" value="urdf_packer" />
  <xacro:property name="Filename" value="|||name of your robot|||" />
  <xacro:property name="UrdfDataPath" value="package://${ThisPackage}/urdf/${Filename}" />
  <xacro:property name="MeshPath" value="package://${ThisPackage}/meshes/${Filename}" />
  ```
  - (Optional) If you had imports in your .xacro (for gazebo for example), you can change them like so:
  ```xml
  <xacro:include filename="materials.xacro" />
  <xacro:include filename="${Filename}.trans" />
  <xacro:include filename="${Filename}.gazebo" />
  ```
  - Change ALL of your meshes `filename` property in the .xacro to the following pattern (at launch time xacro will automatically replace `${MeshPath}` with the correct path we've define 2 steps above):
  ```xml
  <geometry>
    <mesh
    filename="${MeshPath}/base_link.STL" />
  </geometry>
  ```
  - Only change `filename="SOMETHING_SOMETHING/<file>.STL"` to `filename="${MeshPath}/<file>.STL"` property of the geometry/mesh, do not delete other properties that may be necessary to your urdf, for example do not delete `<mesh ... scale="0.001 0.001 0.001" />`.
  - If you are not using meshes and want to avoid errors, you can delete all of the `<mesh>  ...  </mesh>` lines of your xacro / urdf.
- Your meshes .stl should be inside the folder [`/src/urdf_packer/meshes/`](/src/urdf_packer/meshes)`<name of your robot>/`.
- Select your urdf:
  - As a general launch parameter: Set `ROBOT_NAME` as `"<name of your robot>"` (without `.urdf` or `.xacro`) in [`general_launch_setting.py`](general_launch_setting.py#L30). This will pass down to other nodes. (I set this string value with the python code to be able to quickly change it, don't worry about changing the code, as long as it's a string it will be ok)
  - As the default: Assign `"<name of your robot>"` (without `.urdf` or `.xacro`) to the variable `ROBOT_NAME_DEFAULT` in [`/src/urdf_packer/launch/rviz.launch.py#L12`](/src/urdf_packer/launch/rviz.launch.py#L12) and [`/src/easy_robot_control/launch/launch_setting.py#L10`](/src/easy_robot_control/launch/launch_setting.py#L10).

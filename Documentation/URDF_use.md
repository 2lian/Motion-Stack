# How to use your URDF with this repo

## Example and overview
- The name of our robot is `moonbot_hero`
- [`/src/rviz_basic/urdf/moonbot_hero/moonbot_hero.xacro`](/src/rviz_basic/urdf/moonbot_hero/moonbot_hero.xacro): A few modifications have been performed, and xacro imports are around this file.
- [`/src/rviz_basic/meshes/moonbot_hero/`](/src/rviz_basic/meshes/moonbot_hero): Meshes are placed here.
- [`/src/rviz_basic/setup.py#L6`](/src/rviz_basic/setup.py#L6): Lists the folders names that are robots (this manual input might disapear in the future).
- [`/src/rviz_basic/launch/rviz.launch.py#L12`](/src/rviz_basic/launch/rviz.launch.py#L12): Specifies the urdf at launch for the Rviz interface.
- [`/src/easy_robot_control/launch/launch_setting.py#L10`](/src/easy_robot_control/launch/launch_setting.py#L10): Specifies the urdf at launch for the main motion stack.

## Setup explanation
Let's go in details on how to setup you urdf or rather .xacro
- Your .urdf or .xacro file should be in [`/src/rviz_basic/urdf/`](/src/rviz_basic/urdf)`<name of your robot>/<name of your robot>.xacro`. You will need to change it following the guidline below, so that the files path corresponds to this repo. This is usually not a problem when a single URDF is used at a time, everything (urdf and meshes) are copied in the root `package://` and everyone hopes and prays for the best. Because we offer the possibility of several diffrent robots being loaded and swaped, we need a better file structure to avoid conflicts.
  - If you use a .urdf, rename it as a .xacro.
  - Open your .xacro with a text editor.
  - Modify the top of you file like the provided code below (replace `|||name of your robot|||` accordingly):
  ```xml
  <?xml version="1.0" ?>
  <robot name="|||name of your robot|||" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <xacro:property name="ThisPackage" value="rviz_basic" />
  <xacro:property name="Filename" value="|||name of your robot|||" />
  <xacro:property name="UrdfDataPath" value="package://${ThisPackage}/urdf/${Filename}" />
  <xacro:property name="MeshPath" value="package://${ThisPackage}/meshes/${Filename}" />
  ```
  - If you had imports in your .xacro (for gazebo for example), you can change them like so (optional):
  ```xml
  <xacro:include filename="materials.xacro" />
  <xacro:include filename="${Filename}.trans" />
  <xacro:include filename="${Filename}.gazebo" />
  ```
  - Change ALL of your meshes `filename` path in the .xacro to the following pattern (at launch time xacro will automatically replace `${MeshPath}` with the correct path we've define 2 steps above):
  ```xml
  <geometry>
    <mesh
    filename="${MeshPath}/base_link.STL" />
  </geometry>
  ```
  - Only change the `filename="${MeshPath}/base_link.STL"` property of the geometry/mesh, do not delete other properties that may be necessary to your urdf, for example do not delete `scale="0.001 0.001 0.001"`.
  - If you are not using meshes and want to avoid errors, you can delete all of the `<mesh>  ...  </mesh>` lines of your xacro / urdf.
- Your meshes .stl should be inside the folder [`/src/rviz_basic/meshes/`](/src/rviz_basic/meshes)`<name of your robot>/`.
- Add `<name of your robot>` to the `folders` list of [`/src/rviz_basic/setup.py#L6`](/src/rviz_basic/setup.py#L6).
  - At build time, according to [`.../setup.py`](/src/rviz_basic/setup.py), the directories [`/src/rviz_basic/meshes/`](/src/rviz_basic/meshes) and [`/src/rviz_basic/urdf/`](/src/rviz_basic/urdf) will be copied in rviz_basic package's shared directory and the right file structure will be created.
- Assign `<name of your robot>` (without `.urdf` or `.xacro`) to the variable `ROBOT_NAME` in [`/src/rviz_basic/launch/rviz.launch.py#L12`](/src/rviz_basic/launch/rviz.launch.py#L12) and [`/src/easy_robot_control/launch/launch_setting.py#L10`](/src/easy_robot_control/launch/launch_setting.py#L10).
  - This will give the corresponding .xacro path at launchtime as a Ros2 parameter to the nodes, study and change this to create your own launcher.

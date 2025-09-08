# motion_stack.api.launch package

ROS2 launch api

## Submodules

## motion_stack.api.launch.builder module

API to generate launch files

### *class* motion_stack.api.launch.builder.LevelBuilder(leg_dict, urdf_path=None, params_overwrite={}, urdf=None)

Bases: `object`

Builds a launcher for the motion stack generating your nodes

#### NOTE
This class is meant to be overloaded and changed for your robot.
Refere to [Launch API](../../manual/api.md#launch-api-label)

* **Parameters:**
  * **urdf_path** (*str* *|* *None*) – Path of the urdf/xacro. A ROS2 Command will compile it and pass it as a string. See [`api.launch.builder.xacro_path_from_pkg()`](#motion_stack.api.launch.builder.xacro_path_from_pkg) to get a path.
  * **leg_dict** (*Mapping* *[**int* *,* *str* *|* *int* *]*) – Dictionary linking leg number to end effector name.                This informs the API of the number of legs and nodes to launch.
  * **params_overwrite** (*Dict* *[**str* *,* *Any* *]*) – Will overwrite the default parameters
  * **urdf** (*None* *|* *str* *|* *Command*) – Full urdf (or command to compile it). Use this instead of urdf_path to compile with options or more. see [`api.launch.builder.xacro_path_from_pkg()`](#motion_stack.api.launch.builder.xacro_path_from_pkg) and [`api.launch.builder.command_from_xacro_path()`](#motion_stack.api.launch.builder.command_from_xacro_path)

Example:

```default
from motion_stack.api.launch.builder import LevelBuilder, xacro_path_from_pkg

urdf_path = xacro_path_from_pkg(
    package_name="moonbot_zero_tuto", xacro_path="urdf/moonbot_zero.xacro"
)
LEGS_DIC = {
    1: "end1",
    2: "end2",
    3: "end3",
    4: "end4",
}
new_params = {
    "std_movement_time": 10.0,
}

lvl_builder = MyLevelBuilder(
    urdf_path=urdf_path,
    leg_dict=LEGS_DIC,
    params_overwrite=new_params,
)

def generate_launch_description():
    return lvl_builder.make_description()
```

#### OLD_PKG *= 'easy_robot_control'*

**Type:**    `str`

#### MS_PACKAGE *= 'motion_stack'*

**Type:**    `str`

#### make_description(levels=None)

Return the launch description for ros2

Example:

```default
def generate_launch_description():
    return lvl_builder.make_description()
```

* **Parameters:**
  **levels** (*List* *[**List* *[**Node* *]* *]*  *|* *None*) – list of levels, levels being a list of nodes to be launched
* **Returns:**
  launch description to launch all the nodes
* **Return type:**
  `LaunchDescription`

#### process_CLI_args()

#### lvl_to_launch()

* **Returns:**
  List of int corresponding to the motion stack levels to start
* **Return type:**
  `List`[`int`]

#### generate_global_params()

Generates parameters shared by all nodes.
Based on the default params and overwrites.

<!-- Note:
stores it in self.all_param -->

#### make_leg_param(leg_index, ee_name)

Based on the leg index/number (and end-effector), returns the parameters corresponding to the leg

* **Parameters:**
  * **leg_index** (*int*) – leg number to create the params for
  * **ee_name** (*None* *|* *str* *|* *int*) – (Optional) end effector name or number
* **Raises:**
  **Exception** – Exception(f”{leg_index} not in self.legs_dic”)
* **Returns:**
  Dictionary of ROS2 parameters
* **Return type:**
  `Dict`[`str`, `Any`]

#### lvl1_params()

Returns parameters for all lvl1 nodes to be launched

* **Returns:**
  List of ros2 parameter dictionary, one per node.
* **Return type:**
  `List`[`Dict`]

#### lvl2_params()

Returns parameters for all lvl2 nodes to be launched

* **Returns:**
  List of ros2 parameter dictionary, one per node.
* **Return type:**
  `List`[`Dict`]

#### state_publisher_lvl1()

<a id="builder-state-label"></a>

Prepares all nodes meant to publish the state of the robot to external tools.

Along each lvl1 node, it creates:
: - One (customized) joint_state_publisher continuously publishing joint angles
  - One robot_state_publisher continuously publishing robot TF and description.

* **Returns:**
  List of Nodes to be launched (empty if lvl1 is not to be launched)
* **Return type:**
  `List`[`Node`]

#### get_node_lvl1(params)

* **Return type:**
  `Node`
* **Parameters:**
  **params** (*Dict* *[**str* *,* *Any* *]*)

#### get_node_lvl2(params)

* **Return type:**
  `Node`
* **Parameters:**
  **params** (*Dict* *[**str* *,* *Any* *]*)

#### *static* limb_ns(limb_number)

* **Return type:**
  `str`
* **Parameters:**
  **limb_number** (*int*)

#### lvl1()

* **Return type:**
  `List`[`Node`]

#### lvl2()

* **Return type:**
  `List`[`Node`]

#### make_levels()

* **Return type:**
  `List`[`List`[`Node`]]

### motion_stack.api.launch.builder.xacro_path_from_packer(robot_name)

retieves the .urdf/.xacro path in the install share folder of urdf_packer.

* **Parameters:**
  **robot_name** (*str*) – corresponds to <robot_name>.xacro

Returns:

### motion_stack.api.launch.builder.xacro_path_from_pkg(package_name, xacro_path, options=None)

Gets a path from a package, adding options, in view of xacro  compilation.

* **Parameters:**
  * **package_name** (*str*) – Name of the ros2 package
  * **xacro_path** (*str*) – Path of the file in the package’s shared directory
  * **options** (*str* *|* *None*) – string of options to append to the command
* **Returns:**
  path as a string, appended by options

### motion_stack.api.launch.builder.command_from_xacro_path(path, options=None)

Creates ROS2 command to compile xacro at launch time.

* **Return type:**
  `Command`
* **Parameters:**
  * **path** (*str*)
  * **options** (*str* *|* *None*)

### motion_stack.api.launch.builder.T *= TypeVar(T)*

**Type:**    `TypeVar`

Invariant `TypeVar`.

### motion_stack.api.launch.builder.get_cli_argument(arg_name, default)

Returns the CLI argument as a string, or default is none inputed.
Can be much better optimised, I don’t care.

* **Return type:**
  `Union`[[`~T`](#motion_stack.api.launch.builder.T), `str`]
* **Parameters:**
  * **arg_name** (*str*)
  * **default** (*T*)

## motion_stack.api.launch.default_params module

Provides and explains all parameters to launch the motion stack

### motion_stack.api.launch.default_params.default_params *= {'add_joints': [''], 'angle_syncer_delta': 0.17453292519943295, 'control_rate': 30.0, 'end_effector_name': '', 'ignore_limits': False, 'joint_buffer': [0.5, 0.0008726646259971648, 0.00017453292519943296, 1.7453292519943296e-05], 'leg_number': 0, 'limit_margin': 0.0, 'mvmt_update_rate': 10.0, 'speed_mode': False, 'start_effector_name': '', 'urdf': ''}*

**Type:**    `Dict`[`str`, `Any`]

Default parameters taken from the python core.

### motion_stack.api.launch.default_params.get_xacro_path(robot_name)

retieves the .urdf/.xacro path in the install share folder

* **Parameters:**
  **robot_name** (*str*) – corresponds to <robot_name>.xacro

Returns:

### motion_stack.api.launch.default_params.enforce_params_type(parameters)

enforces types to dic in place

* **Parameters:**
  **parameters** (*Dict* *[**str* *,* *Any* *]*) – ros2 parameters dictinary
* **Return type:**
  `None`

# easy_robot_control.launch.builder module

API to generate launch files

<a id="level-builder-label"></a>

### *class* easy_robot_control.launch.builder.LevelBuilder(robot_name, leg_dict, params_overwrite={})

Bases: `object`

Builds a launcher for the motion stack generating your nodes

#### NOTE
This class is meant to be overloaded and changed for your robot.
Refere to [Launch API](../manual/api.md#launch-api-label)

* **Parameters:**
  * **robot_name** (`str`) – Name of your robot URDF
  * **leg_dict** (`Mapping`[`int`, `Union`[`str`, `int`]]) – Dictionary linking leg number to end effector name.                This informs the API of the number of legs and nodes to launch.
  * **params_overwrite** (`Dict`[`str`, `Any`]) – Will overwrite the default parameters

Example:

```default
from easy_robot_control.launch.builder import LevelBuilder
ROBOT_NAME = "moonbot_7"  # name of the xacro to load
LEGS_DIC = {
    1: "end1",
    2: "end2",
    3: "end3",
    4: "end4",
}
lvl_builder = LevelBuilder(robot_name=ROBOT_NAME, leg_dict=LEGS_DIC)
def generate_launch_description():
    return lvl_builder.make_description()
```

#### make_description(levels=None)

Return the launch description for ros2

Example:

```default
def generate_launch_description():
    return lvl_builder.make_description()
```

* **Parameters:**
  **levels** (`Optional`[`List`[`List`[`Node`]]]) – list of levels, levels being a list of nodes to be launched
* **Return type:**
  `LaunchDescription`
* **Returns:**
  launch description to launch all the nodes

#### process_CLI_args()

#### lvl_to_launch()

* **Return type:**
  `List`[`int`]
* **Returns:**
  List of int corresponding to the motion stack levels to start

#### get_xacro_path()

* **Return type:**
  `str`
* **Returns:**
  Path to the robot’s xacro file

#### generate_global_params()

Generates parameters shared by all nodes.
Based on the default params and overwrites.

<!-- Note:
stores it in self.all_param -->

#### make_leg_param(leg_index, ee_name)

Based on the leg index/number (and end-effector), returns the parameters corresponding to the leg

* **Parameters:**
  * **leg_index** (`int`) – leg number to create the params for
  * **ee_name** (`Union`[`None`, `str`, `int`]) – (Optional) end effector name or number
* **Raises:**
  **Exception** – Exception(f”{leg_index} not in self.legs_dic”)
* **Return type:**
  `Dict`
* **Returns:**
  Dictionary of ROS2 parameters

#### lvl1_params()

Returns parameters for all lvl1 nodes to be launched

* **Return type:**
  `List`[`Dict`]
* **Returns:**
  List of ros2 parameter dictionary, one per node.

#### lvl2_params()

Returns parameters for all lvl2 nodes to be launched

* **Return type:**
  `List`[`Dict`]
* **Returns:**
  List of ros2 parameter dictionary, one per node.

#### lvl3_params()

Returns parameters for all lvl3 nodes to be launched

* **Return type:**
  `List`[`Dict`]
* **Returns:**
  List of ros2 parameter dictionary, one per node.

#### lvl4_params()

Returns parameters for all lvl4 nodes to be launched

* **Return type:**
  `List`[`Dict`]
* **Returns:**
  List of ros2 parameter dictionary, one per node.

#### lvl5_params()

Returns parameters for all lvl5 nodes to be launched

* **Return type:**
  `List`[`Dict`]
* **Returns:**
  List of ros2 parameter dictionary, one per node.

#### state_publisher_lvl1()

<a id="builder-state-label"></a>

Prepares all nodes meant to publish the state of the robot to external tools.

Along each lvl1 node, it creates:
: - One (customized) joint_state_publisher continuously publishing joint angles
  - One robot_state_publisher continuously publishing robot TF and description.

* **Return type:**
  `List`[`Node`]
* **Returns:**
  List of Nodes to be launched (empty if lvl1 is not to be launched)

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

#### get_node_lvl3(params)

* **Return type:**
  `Node`
* **Parameters:**
  **params** (*Dict* *[**str* *,* *Any* *]*)

#### get_node_lvl4(params)

* **Return type:**
  `Node`
* **Parameters:**
  **params** (*Dict* *[**str* *,* *Any* *]*)

#### get_node_lvl5(params)

* **Return type:**
  `Node`
* **Parameters:**
  **params** (*Dict* *[**str* *,* *Any* *]*)

#### lvl1()

* **Return type:**
  `List`[`Node`]

#### lvl2()

* **Return type:**
  `List`[`Node`]

#### lvl3()

* **Return type:**
  `List`[`Node`]

#### lvl4()

* **Return type:**
  `List`[`Node`]

#### lvl5()

* **Return type:**
  `List`[`Node`]

#### make_levels()

* **Return type:**
  `List`[`List`[`Node`]]

### easy_robot_control.launch.builder.get_cli_argument(arg_name, default)

Returns the CLI argument as a string, or default is none inputed.
Can be much better optimised, I don’t care.

* **Return type:**
  `Union`[`TypeVar`(`T`), `str`]
* **Parameters:**
  * **arg_name** (*str*)
  * **default** (*T*)

# easy_robot_control.launch package

## Submodules

## easy_robot_control.launch.builder module

generates launchfiles

### easy_robot_control.launch.builder.get_cli_argument(arg_name, default)

Returns the CLI argument as a string, or default is none inputed.
Can be much better optimised, I don’t care.

* **Return type:**
  `Union`[`TypeVar`(`T`), `str`]
* **Parameters:**
  * **arg_name** (*str*)
  * **default** (*T*)

### *class* easy_robot_control.launch.builder.LevelBuilder(robot_name, leg_dict, params_overwrite={})

Bases: `object`

* **Parameters:**
  * **robot_name** (*str*)
  * **leg_dict** (*Mapping* *[**int* *,* *str* *|* *int* *]*)
  * **params_overwrite** (*Dict* *[**str* *,* *Any* *]*)

#### process_CLI_args()

#### lvl_to_launch()

#### get_xacro_path()

#### generate_global_params()

#### make_leg_param(leg_index, ee_name)

* **Return type:**
  `Dict`
* **Parameters:**
  * **leg_index** (*int*)
  * **ee_name** (*None* *|* *str* *|* *int*)

#### lvl1_params()

* **Return type:**
  `List`[`Dict`]

#### lvl2_params()

* **Return type:**
  `List`[`Dict`]

#### lvl3_params()

* **Return type:**
  `List`[`Dict`]

#### lvl4_params()

* **Return type:**
  `List`[`Dict`]

#### lvl5_params()

* **Return type:**
  `List`[`Dict`]

#### state_publisher_lvl1()

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

#### make_description(levels=None)

* **Return type:**
  `LaunchDescription`
* **Parameters:**
  **levels** (*List* *[**List* *[**Node* *]* *]*  *|* *None*)

## easy_robot_control.launch.default_params module

### easy_robot_control.launch.default_params.get_xacro_path(robot_name)

retieves the .urdf/.xacro path in the install share folder

* **Parameters:**
  **robot_name** (`str`) – corresponds to <robot_name>.xacro

Returns:

### easy_robot_control.launch.default_params.enforce_params_type(parameters)

enforces types to dic in place

* **Parameters:**
  **parameters** (`Dict`[`str`, `Any`]) – ros2 parameters dictinary
* **Return type:**
  `None`

### easy_robot_control.launch.default_params.make_state_publisher(xacro_path, description_topic='robot_description', state_topic='ms_state', joint_topics=['leg1/joint_read', 'leg2/joint_read', 'leg3/joint_read', 'leg4/joint_read'])

Deprecated

* **Return type:**
  `List`[`Node`]
* **Parameters:**
  * **description_topic** (*str*)
  * **state_topic** (*str*)
  * **joint_topics** (*List* *[**str* *]*)

## Module contents

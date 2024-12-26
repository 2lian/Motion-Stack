# easy_robot_control.launch package

Public and customizable launch API for the motion stack

<a id="launch-init-label"></a>

## Submodules

## easy_robot_control.launch.builder module

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

## easy_robot_control.launch.default_params module

Provides and explains all parameters to launch the motion stack

<a id="default-params-label"></a>

### easy_robot_control.launch.default_params.default_params *= {'WAIT_FOR_LOWER_LEVEL': True, 'add_joints': [''], 'always_write_position': False, 'control_rate': 30.0, 'end_effector_name': 0, 'ignore_limits': False, 'leg_list': [0], 'leg_number': 0, 'limit_margin': 0.0, 'mirror_angle': False, 'mvmt_update_rate': 10.0, 'number_of_legs': None, 'pure_topic_remap': False, 'robot_name': None, 'services_to_wait': [''], 'speed_mode': False, 'start_coord': [0.0, 0.0, 0.0], 'start_effector_name': '', 'std_movement_time': 2, 'urdf_path': None, 'wheel_size_mm': 230}*

**Type:**    `Dict`[`str`, `Any`]

the default parameters of the motion stack

```python
 default_params: Dict[str, Any] = {  # does this work
     # you must set these in your own launcher
     #        #
     #        #
     "robot_name": None,  #: (str) Name of the robot, not critical
     "urdf_path": None,  # path to the xacro or urdf to load
     "number_of_legs": None,  # number of legs in your robot (not used by lvl 1-2-3)
     "leg_number": 0,  # number associated with a leg,
     # if serveral lvl 1-2-3 are running, it is recommanded to use different numbers
     "end_effector_name": 0,  # end effector associated with a leg, (the most important)
     # the kinematic chain used for IK will go
     # from the root link of the URDF (usually base_link)
     # to the end effector link (specified in this parameter).
     # the URDF will be parsed to find this link name. Make sure it exists.
     # you can also provide a number (as a string) instead of a link_name. If you do this
     # the Nth longest kinematic path (sequence of link where each link is connected to
     # exactly one other link) from the root of the URDF will be used for IK
     # Basically, if you use only one limb, set this as "0", and it will pick the right ee.
     "leg_list": [0],  # list of leg numbers
     #        #
     #        #
     "std_movement_time": 2,  # time lvl3 takes to execute a trajectory
     "mvmt_update_rate": 10.0,  # update rate used through out the stack
     "control_rate": 30.0,  # update rate for speed control PID only
     "start_coord": [0 / 1000, 0 / 1000, 0 / 1000],  # starting position
     # (only affects rviz for now).
     # if set to [np.nan,np.nan,np.nan], world->base_link publishing is disabled.
     # lvl1 is publishing this TF, so if you have several lvl1,
     # only one should have this opition enabled
     "add_joints": [""],  # manually adds joints for lvl1 if they are not in the urdf
     "mirror_angle": False,  # lvl1 assumes position sensor data is the last sent command
     "always_write_position": False,  # deprecated ?
     "start_effector_name": "",  # setting this manually, works with the motion stack,
     # but not for Rviz and ros2's tf, so be carefull.
     # In ros, the baselink must be the root of the tf tree, it cannot have a parent
     # and there can only be one baselink.
     # Leaving this empty and properly setting your URDF baselink is recommended.
     "wheel_size_mm": 230,  # deprecated ?
     "pure_topic_remap": False,  # activates the pure_remap.py remapping
     "speed_mode": False,  # lvl1 will send speed commands to the motors, using angle readings as feedback for a PID.
     "services_to_wait": [""],  # List of services to wait for before initializing
     "WAIT_FOR_LOWER_LEVEL": True,  # waits for services of lower level before initializing
     "ignore_limits": False,  # joint limits set in the URDF will be ignored
     "limit_margin": 0.0,  # adds a additional margin to the limits of the URDF (in rad)
```

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

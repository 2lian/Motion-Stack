# motion_stack.core package

Python core of the motion stack.

There is no ROS2 code in here, only python “nodes”. Those the are skeltons to be run by any runtime, either python, ROS2 or else.

This allows for:

Authors:

## Subpackages

* [motion_stack.core.rtb_fix package](motion_stack.core.rtb_fix.md)
  * [Submodules](motion_stack.core.rtb_fix.md#submodules)
  * [motion_stack.core.rtb_fix.fixed_urdf module](motion_stack.core.rtb_fix.md#module-motion_stack.core.rtb_fix.fixed_urdf)
* [motion_stack.core.utils package](motion_stack.core.utils.md)
  * [Submodules](motion_stack.core.utils.md#submodules)
  * [motion_stack.core.utils.csv module](motion_stack.core.utils.md#module-motion_stack.core.utils.csv)
  * [motion_stack.core.utils.hypersphere_clamp module](motion_stack.core.utils.md#module-motion_stack.core.utils.hypersphere_clamp)
  * [motion_stack.core.utils.joint_mapper module](motion_stack.core.utils.md#module-motion_stack.core.utils.joint_mapper)
  * [motion_stack.core.utils.joint_state module](motion_stack.core.utils.md#module-motion_stack.core.utils.joint_state)
  * [motion_stack.core.utils.math module](motion_stack.core.utils.md#module-motion_stack.core.utils.math)
  * [motion_stack.core.utils.pose module](motion_stack.core.utils.md#module-motion_stack.core.utils.pose)
  * [motion_stack.core.utils.printing module](motion_stack.core.utils.md#module-motion_stack.core.utils.printing)
  * [motion_stack.core.utils.robot_parsing module](motion_stack.core.utils.md#module-motion_stack.core.utils.robot_parsing)
  * [motion_stack.core.utils.static_executor module](motion_stack.core.utils.md#module-motion_stack.core.utils.static_executor)
  * [motion_stack.core.utils.time module](motion_stack.core.utils.md#module-motion_stack.core.utils.time)

## Submodules

## motion_stack.core.lvl1_joint module

Node and its object of level 1.

### *class* motion_stack.core.lvl1_joint.JointHandler(name, parent_node, joint_object, IGNORE_LIM=False, MARGIN=0.0)

Bases: `object`

This handles a single joint.
The main purpose is to update stateSensor and stateCommand. As well as getting the
newest values for those (in order to not continuously publish unchanging data).

* **Parameters:**
  * **name** (*str*) – name of the joint (in the URDF)
  * **parent_node** ([*JointCore*](#motion_stack.core.lvl1_joint.JointCore)) – Parent object handling several joints and messages
  * **joint_object** (*Joint*) – raw joint object from rtb, extracted from the URDF
  * **IGNORE_LIM** (*bool*) – If true, joint limits are ignored
  * **MARGIN** (*float*) – Adds a margin to the joints limits

#### TOL_NO_CHANGE *= JState(name='', time=1000000000, position=0.0017453292519943296, velocity=0.00017453292519943296, effort=1.7453292519943296e-05)*

**Type:**    `Final`[[`JState`](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState)]

tolerance for two state to be identical. Time is also considered,
so 2 states far from each other in time will be considered different
and trigger an update

#### PID_P *= 3*

**Type:**    `int`

P gain of the PID for speed mode. TO BE DEPRECATED

#### PID_D *= 0.1*

**Type:**    `float`

D gain of the PID for speed mode. TO BE DEPRECATED

#### PID_LATE *= 0.0*

**Type:**    `float`

Target will be reached late for smoother motion. TO BE DEPRECATED

#### PID_CLOSE_ENOUGH *= 0.00017453292519943296*

**Type:**    `float64`

TO BE DEPRECATED

#### *property* command

Current command state

* **Return type:**
  [`JState`](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState)

#### *property* sensor

current sendor state

* **Return type:**
  [`JState`](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState)

#### *property* name

Name of the joint

* **Return type:**
  `str`

#### *property* no_limit

True if the joint has not limits

* **Return type:**
  `bool`

#### *property* command_ready

True if no commands have been received

* **Return type:**
  `bool`

#### *property* sensor_ready

True if no sensor data have been received

* **Return type:**
  `bool`

#### set_js_command(js)

Updates the stateCommand to a new js.

* **Parameters:**
  **js** ([*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState))

#### is_new_jssensor(js)

True if js is different enough from the last received.
Also true if stateSensor is more the TOL_NO_CHANGE.time old relative to the new

* **Parameters:**
  **js** ([*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState))

#### set_js_sensor(js)

Updates the stateSensor to a new js.

* **Parameters:**
  **js** ([*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState))

#### set_angle_cmd(angle, time=None)

Updates stateCommand by providing only an angle.
should be avoided as the timestamp will be set to now.

* **Parameters:**
  * **angle** (*float*)
  * **time** ([*Time*](motion_stack.core.utils.md#motion_stack.core.utils.time.Time) *|* *None*)

#### set_speed_cmd(speed, stop_other_commands=False)

Updates stateCommand by providing only an speed.
should be avoided as the timestamp will be set to now.

* **Parameters:**
  * **speed** (*float*)
  * **stop_other_commands** (*bool*)

#### set_effort_cmd(effort)

Updates stateCommand by providing only an effort.
should be avoided as the timestamp will be set to now.

* **Parameters:**
  **effort** (*float*)

#### get_fresh_sensor(reset=True)

returns sensor data that is newer than the last time it was called.

if the sensor data didn’t changed enough to trigger a refresh, this will
be full of None. If a refresh occured, the None will be replaced by the non-None
values in the new sensor data.

example: if you stop sending speed sensor data after sending a bunch of speeds.
This speed will switch to None, it will not  continue to be the last received
speed.
This last received speed is still available in stateSensor.

* **Return type:**
  [`JState`](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState)
* **Parameters:**
  **reset** (*bool*)

#### get_fresh_command(reset=True)

returns command data that is newer than the last time it was called.
full of None is not newer

* **Return type:**
  [`JState`](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState)
* **Parameters:**
  **reset** (*bool*)

### *class* motion_stack.core.lvl1_joint.JointCore(\*args, \*\*kwargs)

Bases: [`FlexNode`](motion_stack.core.utils.md#motion_stack.core.utils.static_executor.FlexNode)

Lvl1

#### send_to_lvl0_callbacks *= []*

**Type:**    `List`[`Callable`[[`List`[[`JState`](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState)]], `None`]]

#### send_to_lvl2_callbacks *= []*

**Type:**    `List`[`Callable`[[`List`[[`JState`](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState)]], `None`]]

#### SENS_VERBOSE_TIMEOUT *= 1*

**Type:**    `int`

duration after which joints with no sensor data are displayed (warning)

#### lvl0_remap

**Type:**    [`StateRemapper`](motion_stack.api.injection.md#motion_stack.api.injection.remapper.StateRemapper)

Remapping around any joint state communication of lvl0. Overwritable

#### lvl2_remap

**Type:**    [`StateRemapper`](motion_stack.api.injection.md#motion_stack.api.injection.remapper.StateRemapper)

Remapping around any joint state communication of lvl2. Overwritable

#### leg_num

**Type:**    `int`

leg number identifier, deduced from the parameters

#### send_to_lvl0(states)

Sends states to lvl0 (commands for motors).
This function is executed every time data needs to be sent down.

#### IMPORTANT
Change/overload this method with what you need.

Or put what you want to execute in self.send_to_lvl0_callbacks

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

#### send_to_lvl2(states)

Sends states to lvl2 (states for ik).
This function is executed every time data needs to be sent up.

#### IMPORTANT
Change/overload this method with what you need.

Or put what you want to execute in self.send_to_lvl0_callbacks

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

#### coming_from_lvl2(states)

Processes incomming commands from lvl2 ik.
Call this function after processing the data into a List[JState]

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

#### coming_from_lvl0(states)

Processes incomming sensor states from lvl0 motors.
Call this function after processing the data into a List[JState]

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

#### send_sensor_up()

pulls and resets fresh sensor data, applies remapping, then sends it to lvl2

#### send_command_down()

pulls and resets fresh command data, applies remapping, then sends it to lvl0

#### sensor_check_verbose()

Checks that all joints are receiving data.
After TIMEOUT, if not, warns the user.

* **Returns:**
  True if all joints have angle data
* **Return type:**
  `bool`

#### send_empty_command_to_lvl0()

Sends a command to lvl0 with no data.

Usefull to initialize lvl0 by giving only the joint names.

#### all_go_zero()

Sends command of angle=0 to all joints

## motion_stack.core.lvl2_ik module

This node is responsible for recieving targets in the body reference frame, and send the
corresponding angles to the motors.

Author: Elian NEPPEL
Lab: SRL, Moonshot team

### motion_stack.core.lvl2_ik.float_formatter()

S.format(

```
*
```

args, 

```
**
```

kwargs) -> str

Return a formatted version of S, using substitutions from args and kwargs.
The substitutions are identified by braces (‘{’ and ‘}’).

### *class* motion_stack.core.lvl2_ik.IKCore(\*args, \*\*kwargs)

Bases: [`FlexNode`](motion_stack.core.utils.md#motion_stack.core.utils.static_executor.FlexNode)

#### stated

**Type:**    `Dict`[`str`, [`JState`](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState)]

recent addition storing the whole state

#### firstSpinCBK()

#### all_limits(et_chain, jobjL)

* **Parameters:**
  * **et_chain** (*ETS*)
  * **jobjL** (*List* *[**Joint* *]*)

#### compute_raw_ik(pose, start, compute_budget=None, mvt_duration=None)

* **Return type:**
  `Tuple`[`Optional`[`ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]], `bool`]
* **Parameters:**
  * **pose** ([*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose))
  * **start** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **compute_budget** ([*Time*](motion_stack.core.utils.md#motion_stack.core.utils.time.Time) *|* *None*)
  * **mvt_duration** ([*Time*](motion_stack.core.utils.md#motion_stack.core.utils.time.Time) *|* *None*)

#### find_next_ik(pose, compute_budget=None, mvt_duration=None)

* **Return type:**
  `ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]
* **Parameters:**
  * **pose** ([*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose))
  * **compute_budget** ([*Time*](motion_stack.core.utils.md#motion_stack.core.utils.time.Time) *|* *None*)
  * **mvt_duration** ([*Time*](motion_stack.core.utils.md#motion_stack.core.utils.time.Time) *|* *None*)

#### ik_target(pose)

recieves target from leg, converts to numpy, computes IK, sends angle
results to joints

* **Parameters:**
  * **msg** – target as Ros2 Vector3
  * **pose** ([*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose))
* **Return type:**
  `None`

#### state_from_lvl1(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

#### send_to_lvl1(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

#### send_current_fk()

* **Return type:**
  [`Pose`](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose)

#### send_to_lvl3(pose)

* **Parameters:**
  **pose** ([*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose))

#### current_fk()

* **Return type:**
  [`Pose`](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose)

### *class* motion_stack.core.lvl2_ik.JointSyncerIk(core, interpolation_delta=0.17453292519943295, on_target_delta=0.17453292519943295)

Bases: [`JointSyncer`](motion_stack.api.md#motion_stack.api.joint_syncer.JointSyncer)

* **Parameters:**
  * **core** ([*IKCore*](#motion_stack.core.lvl2_ik.IKCore))
  * **interpolation_delta** (*float*)
  * **on_target_delta** (*float*)

#### send_to_lvl1(states)

Sends motor command data to lvl1.

#### IMPORTANT
This method must be implemented by the runtime/interface.

#### NOTE
Default ROS2 implementation: `ros2.joint_api.JointSyncerRos.send_to_lvl1()`

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*) – Joint state data to be sent to lvl1

#### *property* sensor

`Dict`[`str`, [`JState`](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState)]

* **Type:**
  rtype

#### *property* FutureT

`Type`[`Future`]

* **Type:**
  rtype

## motion_stack.core.lvl4_mover module

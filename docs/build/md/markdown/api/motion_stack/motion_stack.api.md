# motion_stack.api package

## Subpackages

* [motion_stack.api.injection package](motion_stack.api.injection.md)
  * [Submodules](motion_stack.api.injection.md#submodules)
  * [motion_stack.api.injection.offsetter module](motion_stack.api.injection.md#module-motion_stack.api.injection.offsetter)
  * [motion_stack.api.injection.remapper module](motion_stack.api.injection.md#module-motion_stack.api.injection.remapper)
* [motion_stack.api.launch package](motion_stack.api.launch.md)
  * [Submodules](motion_stack.api.launch.md#submodules)
  * [motion_stack.api.launch.builder module](motion_stack.api.launch.md#module-motion_stack.api.launch.builder)
  * [motion_stack.api.launch.default_params module](motion_stack.api.launch.md#module-motion_stack.api.launch.default_params)
* [motion_stack.api.ros2 package](motion_stack.api.ros2.md)
  * [Submodules](motion_stack.api.ros2.md#submodules)
  * [motion_stack.api.ros2.ik_api module](motion_stack.api.ros2.md#module-motion_stack.api.ros2.ik_api)
  * [motion_stack.api.ros2.joint_api module](motion_stack.api.ros2.md#module-motion_stack.api.ros2.joint_api)
  * [motion_stack.api.ros2.offsetter module](motion_stack.api.ros2.md#module-motion_stack.api.ros2.offsetter)
  * [motion_stack.api.ros2.state_to_topic module](motion_stack.api.ros2.md#module-motion_stack.api.ros2.state_to_topic)

## Submodules

## motion_stack.api.ik_syncer module

Python API to sync the movement of several end_effectors.
This requires ik lvl2 to be running.

#### NOTE
The ros2 implementation is available in `ros2.ik_api`.

This high level API alows for multi-end-effector control and syncronization (over several legs). This is the base class where, receiving and sending data to motion stack lvl2 is left to be implemented.

### motion_stack.api.ik_syncer.FutureType

placeholder type for a Future (ROS2 Future, asyncio or concurrent)

Alias of `Awaitable`

### motion_stack.api.ik_syncer.LimbNumber

A type alias representing the limb number (end effector index).

### motion_stack.api.ik_syncer.MultiPose

A dictionary mapping limb numbers to their corresponding poses.

Alias of `Dict`[`int`, [`Pose`](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose)]

### *class* motion_stack.api.ik_syncer.IkSyncer(interpolation_delta=XyzQuat(xyz=40, quat=0.06981317007977318), on_target_delta=XyzQuat(xyz=40, quat=0.06981317007977318))

Bases: `ABC`

* **Parameters:**
  * **interpolation_delta** ([*XyzQuat*](motion_stack.core.utils.md#motion_stack.core.utils.pose.XyzQuat) *[**float* *,* *float* *]*)
  * **on_target_delta** ([*XyzQuat*](motion_stack.core.utils.md#motion_stack.core.utils.pose.XyzQuat) *[**float* *,* *float* *]*)

#### last_future

**Type:**    `Awaitable`

Future of the latest task/trajectory that was run.

#### execute()

Executes one step of the task/trajectory.

This must be called frequently.

#### lerp(target)

Starts executing a lerp trajectory toward the target.

* **Parameters:**
  **target** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*) – key = joint name ; value = joint angle
* **Returns:**
  Future of the task. Done when sensors are on target.
* **Return type:**
  `Awaitable`

#### asap(target)

Starts executing a asap trajectory toward the target.

* **Parameters:**
  **target** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*) – key = joint name ; value = joint angle
* **Returns:**
  Future of the task. Done when sensorare on target.
* **Return type:**
  `Awaitable`

#### unsafe(target)

Starts executing a unsafe trajectory toward the target.

* **Parameters:**
  **target** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*) – key = joint name ; value = joint angle
* **Returns:**
  Future of the task. Done when sensorare on target.
* **Return type:**
  `Awaitable`

#### *abstract* send_to_lvl1(ee_targets)

Sends ik command to lvl2.

#### IMPORTANT
This method must be implemented by the runtime/interface.

#### NOTE
Default ROS2 implementation: [`ros2.ik_api.IkSyncerRos.send_to_lvl1()`](motion_stack.api.ros2.md#motion_stack.api.ros2.ik_api.IkSyncerRos.send_to_lvl1)

* **Parameters:**
  * **states** – Ik target to be sent to lvl1
  * **ee_targets** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*)

#### *abstract property* FutureT

ROS2 Future, asyncio or concurrent.

#### IMPORTANT
This method must be implemented by the runtime/interface.

Default ROS2 implementation::
: return rclpy.task.Future

* **Returns:**
  The Future class (not an instance).
* **Return type:**
  `type`[`Awaitable`]
* **Type:**
  Class of Future to use

#### abs_from_offset(offset)

Absolute position of the joints that correspond to the given relative offset.

### Example

*joint_1* is at 45 deg, offset is 20 deg. Return will be 65 deg.

* **Parameters:**
  **offset** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*) – Offset dictionary, keys are the joint names, value the offset in rad.
* **Returns:**
  Absolute position.
* **Return type:**
  `Dict`[`int`, [`Pose`](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose)]

#### *abstract property* sensor

Is called when sensor data is need.

#### IMPORTANT
This method must be implemented by the runtime/interface.

#### NOTE
Default ROS2 implementation: [`ros2.joint_api.JointSyncerRos.sensor()`](motion_stack.api.ros2.md#motion_stack.api.ros2.joint_api.JointSyncerRos.sensor)

Returns:

* **Return type:**
  `Dict`[`int`, [`Pose`](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose)]

#### clear()

Resets the trajectory starting point onto the current sensor positions.

Mainly usefull when the trajectory is stuck because it could not reach a target.

#### unsafe_toward(target)

Executes one single unsafe step.

* **Parameters:**
  **target** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*) – key = joint name ; value = joint angle
* **Returns:**
  True if trajectory finished
* **Return type:**
  `bool`

#### asap_toward(target)

Executes one single asap step.

* **Parameters:**
  **target** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*) – key = joint name ; value = joint angle
* **Returns:**
  True if trajectory finished
* **Return type:**
  `bool`

#### lerp_toward(target)

Executes one single lerp step.

* **Parameters:**
  **target** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*) – key = joint name ; value = joint angle
* **Returns:**
  True if trajectory finished
* **Return type:**
  `bool`

## motion_stack.api.joint_syncer module

Python API to sync the movement of several joints.

#### NOTE
The ros2 implementation is available in `ros2.joint_api`.

This high level API alows for multi-joint control and syncronization (over several legs). This is the base class where, receiving and sending data to motion stack lvl1 is left to be implemented.

### motion_stack.api.joint_syncer.FutureType

placeholder type for a Future (ROS2 Future, asyncio or concurrent)

Alias of `Awaitable`

### *class* motion_stack.api.joint_syncer.JointSyncer(interpolation_delta=0.08726646259971647, on_target_delta=0.06981317007977318)

Bases: `ABC`

One instance controls and syncronises several joints, safely executing trajectory to a target.

#### NOTE
This class is an abstract base class, the ros2 implementation is available in [`ros2.joint_api.JointSyncerRos`](motion_stack.api.ros2.md#motion_stack.api.ros2.joint_api.JointSyncerRos).

The trajectory interpolates between two points:

> - The last position (sensor if None, else, last sub-target). This is handled automatically, however `clear` resets it on the current sensor pose.
> - The input target.

Several interpolation strategies are available:

> - LERP: all joints reach the target at the same time.
> - ASAP: joints will reach their tagets indepently, as fast as possible
> - Unsafe: Similar to ASAP except the final target is sent directly to the motor, so the movement will not stop in case of crash, errors, network issue AND you cannot cancel it.

One object instance can only execute one trajectory at a time. However, the joints controled can change between calls of the same instance.

`execute` must be called to compute,update and send the command.

Trajectory tasks return a Future that is ‘done’ when the sensors are on target.

This class is an abstractclass, the ros2 implementation is available in [`ros2.joint_api.JointSyncerRos`](motion_stack.api.ros2.md#motion_stack.api.ros2.joint_api.JointSyncerRos). Hence,  parts of this class are left to be implmented by the interface/runtime:

> - FutureT: Class of Future class to use, ROS2 Future, asyncio or concurrent.
> - sensor: is called when new sensor data is need.
> - send_to_lvl1: is called when command needs to be sent.
* **Parameters:**
  * **interpolation_delta** (*float*) – (rad) During movement, how much error is allowed from the path. if exceeded, movement slows down.
  * **on_target_delta** (*float*) – (rad) Delta at which the trajectory/task is considered finished and the Future is switched to `done`.

#### last_future

Future of the latest task/trajectory that was run.

#### execute()

Executes one step of the task/trajectory.

This must be called frequently.

#### lerp(target)

Starts executing a lerp trajectory toward the target.

* **Parameters:**
  **target** (*Dict* *[**str* *,* *float* *]*) – key = joint name ; value = joint angle
* **Returns:**
  Future of the task. Done when sensors are on target.
* **Return type:**
  `Awaitable`

#### asap(target)

Starts executing a asap trajectory toward the target.

* **Parameters:**
  **target** (*Dict* *[**str* *,* *float* *]*) – key = joint name ; value = joint angle
* **Returns:**
  Future of the task. Done when sensorare on target.
* **Return type:**
  `Awaitable`

#### unsafe(target)

Starts executing a unsafe trajectory toward the target.

* **Parameters:**
  **target** (*Dict* *[**str* *,* *float* *]*) – key = joint name ; value = joint angle
* **Returns:**
  Future of the task. Done when sensorare on target.
* **Return type:**
  `Awaitable`

#### *abstract* send_to_lvl1(states)

Sends motor command data to lvl1.

#### IMPORTANT
This method must be implemented by the runtime/interface.

#### NOTE
Default ROS2 implementation: [`ros2.joint_api.JointSyncerRos.send_to_lvl1()`](motion_stack.api.ros2.md#motion_stack.api.ros2.joint_api.JointSyncerRos.send_to_lvl1)

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*) – Joint state data to be sent to lvl1

#### *abstract property* FutureT

ROS2 Future, asyncio or concurrent.

#### IMPORTANT
This method must be implemented by the runtime/interface.

Default ROS2 implementation::
: return rclpy.task.Future

* **Returns:**
  The Future class (not an instance).
* **Return type:**
  `type`[`Awaitable`]
* **Type:**
  Class of Future to use

#### abs_from_offset(offset)

Absolute position of the joints that correspond to the given relative offset.

### Example

*joint_1* is at 45 deg, offset is 20 deg. Return will be 65 deg.

* **Parameters:**
  **offset** (*Dict* *[**str* *,* *float* *]*) – Offset dictionary, keys are the joint names, value the offset in rad.
* **Returns:**
  Absolute position.
* **Return type:**
  `Dict`[`str`, `float`]

#### *abstract property* sensor

Is called when sensor data is need.

#### IMPORTANT
This method must be implemented by the runtime/interface.

#### NOTE
Default ROS2 implementation: [`ros2.joint_api.JointSyncerRos.sensor()`](motion_stack.api.ros2.md#motion_stack.api.ros2.joint_api.JointSyncerRos.sensor)

Returns:

* **Return type:**
  `Dict`[`str`, [`JState`](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState)]

#### clear()

Resets the trajectory starting point onto the current sensor positions.

Mainly usefull when the trajectory is stuck because it could not reach a target.

#### unsafe_toward(target)

Executes one single unsafe step.

* **Parameters:**
  **target** (*Dict* *[**str* *,* *float* *]*) – key = joint name ; value = joint angle
* **Returns:**
  True if trajectory finished
* **Return type:**
  `bool`

#### asap_toward(target)

Executes one single asap step.

* **Parameters:**
  **target** (*Dict* *[**str* *,* *float* *]*) – key = joint name ; value = joint angle
* **Returns:**
  True if trajectory finished
* **Return type:**
  `bool`

#### lerp_toward(target)

Executes one single lerp step.

* **Parameters:**
  **target** (*Dict* *[**str* *,* *float* *]*) – key = joint name ; value = joint angle
* **Returns:**
  True if trajectory finished
* **Return type:**
  `bool`

#### speed_safe(target, delta_time)

NOT TESTED. USE AT YOUR OWN RISK

* **Parameters:**
  * **target** (*Dict* *[**str* *,* *float* *]*)
  * **delta_time** (*float* *|* *Callable* *[* *[* *]* *,* *float* *]*)
* **Return type:**
  <property object at 0x7feeb9648270>

Returns:

* **Return type:**
  `~.`
* **Parameters:**
  * **target** (*Dict* *[**str* *,* *float* *]*)
  * **delta_time** (*float* *|* *Callable* *[* *[* *]* *,* *float* *]*)

### motion_stack.api.joint_syncer.only_position(js_dict)

Extract velocities from a dict or list of JState. None is ignored

* **Return type:**
  `Dict`[`str`, `float`]
* **Parameters:**
  **js_dict** (*Dict* *[**str* *,* [*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*  *|* *List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

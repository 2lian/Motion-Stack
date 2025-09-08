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
The ros2 implementation is available in [`ros2.ik_api`](motion_stack.api.ros2.md#module-motion_stack.api.ros2.ik_api).

This high level API alows for multi-end-effector control and syncronization (over several legs). This is the base class where, receiving and sending data to motion stack lvl2 is left to be implemented.

### motion_stack.api.ik_syncer.FutureType

placeholder type for a Future (ROS2 Future, asyncio or concurrent)

Alias of `Awaitable`

### motion_stack.api.ik_syncer.LimbNumber

A type alias representing the limb number (end effector index).

### motion_stack.api.ik_syncer.MultiPose

A dictionary mapping limb numbers to their corresponding poses.

Alias of `Dict`[`int`, [`Pose`](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose)]

### *exception* motion_stack.api.ik_syncer.SensorSyncWarning

Bases: `Warning`

### *class* motion_stack.api.ik_syncer.IkSyncer(interpolation_delta=XyzQuat(xyz=40, quat=0.06981317007977318), on_target_delta=XyzQuat(xyz=40, quat=0.06981317007977318))

Bases: `ABC`

One instance controls and syncronises several limbs end-effectors, safely executing trajectory to targets.

#### NOTE
This class is an abstract base class, the ros2 implementation is available in `ros2.joint_api.IkSyncerRos`. Hence,  parts of this class are left to be implmented by the interface/runtime: [`IkSyncer.FutureT()`](#motion_stack.api.ik_syncer.IkSyncer.FutureT), [`IkSyncer.sensor()`](#motion_stack.api.ik_syncer.IkSyncer.sensor), [`IkSyncer.send_to_lvl2()`](#motion_stack.api.ik_syncer.IkSyncer.send_to_lvl2).

#### IMPORTANT
[`IkSyncer.execute()`](#motion_stack.api.ik_syncer.IkSyncer.execute) must be called to compute, update and send the command.

One object instance can only execute one target at a time. However, the limbs or targets can change between calls of the same instance, before the previous task is done.

The trajectory interpolates between two points:

> - The last position (if None: uses sensor, else: last sub-target). This is handled automatically, however `clear` resets the last position to None.
> - The input target.

Several interpolation strategies are available:

> - LERP: [`IkSyncer.lerp()`](#motion_stack.api.ik_syncer.IkSyncer.lerp)
> - ASAP: [`IkSyncer.asap()`](#motion_stack.api.ik_syncer.IkSyncer.asap)
> - Unsafe: [`IkSyncer.unsafe()`](#motion_stack.api.ik_syncer.IkSyncer.unsafe)
* **Parameters:**
  * **interpolation_delta** ([*XyzQuat*](motion_stack.core.utils.md#motion_stack.core.utils.pose.XyzQuat) *[**float* *,* *float* *]*) – (mm, rad) During movement, how much divergence is allowed from the path. If exceeded, movement slows down.
  * **on_target_delta** ([*XyzQuat*](motion_stack.core.utils.md#motion_stack.core.utils.pose.XyzQuat) *[**float* *,* *float* *]*) – (mm, rad) Delta at which the trajectory/task is considered finished and the Future is switched to `done`.

#### last_future

**Type:**    `Awaitable`

Future of the latest task/trajectory that was run.

#### SEND_UNTIL_DONE

When true, the command messages sill stop being published when the sensor data is on target. When false, it stops after sending the target command once. True is improves reliability at the expense of more messages sent, better for lossy networks.

#### execute()

Executes one step of the task/trajectory.

This must be called frequently.

#### clear()

Resets the trajectory starting point onto the current sensor positions.

#### IMPORTANT
Use when:
: - The trajectory is stuck unable to interpolate.
  - External motion happened, thus the last position used by the syncer is no longer valid.

#### lerp(target)

Starts executing a lerp trajectory toward the target.

LERP: all joints reach the target at the same time.

* **Returns:**
  Future of the task. Done when sensors are on target.
* **Return type:**
  `Awaitable`
* **Parameters:**
  **target** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*)

#### asap(target)

Starts executing a asap trajectory toward the target.

ASAP: (Not Implemented) joints will reach their tagets indepently, as fast as possible.

* **Returns:**
  Future of the task. Done when sensors are on target.
* **Return type:**
  `Awaitable`
* **Parameters:**
  **target** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*)

#### unsafe(target)

Starts executing a unsafe trajectory toward the target.

Unsafe: Similar to ASAP except the final target is sent directly to the IK, so the movement will not stop in case of crash, errors, network issue AND you cannot cancel it.

* **Returns:**
  Future of the task. Done when sensors are on target.
* **Return type:**
  `Awaitable`
* **Parameters:**
  **target** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*)

#### speed_safe(target, delta_time)

A Cartesian speed‐safe trajectory that interprets angular velocity
as a rotation vector (axis \* rad/s) instead of a quaternion.

* **Parameters:**
  * **target** (*Dict* *[**int* *,* [*VelPose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.VelPose) *]*) – Mapping limb → VelPose, where
    • VelPose.lin is linear speed (mm/s)
    • VelPose.rvec is rotational speed vector (axis \* rad/s)
  * **delta_time** (*float* *|* *Callable* *[* *[* *]* *,* *float* *]*) – Either a fixed Δt (s) or a zero‐arg callable returning Δt.
* **Returns:**
  A Future that continuously steps the motion until cancelled.
* **Return type:**
  `Awaitable`

#### abs_from_rel(offset)

Absolute position of the MultiPose that corresponds to the given relative offset.

### Example

*joint_1* is at 45 deg, offset is 20 deg. Return will be 65 deg.

* **Parameters:**
  **offset** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*) – Relative postion.
* **Returns:**
  Absolute position.
* **Return type:**
  `Dict`[`int`, [`Pose`](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose)]

#### dummy_print_multipose(data, prefix='')

* **Parameters:**
  * **data** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*)
  * **prefix** (*str*)

#### *abstractmethod* send_to_lvl2(ee_targets)

Sends ik command to lvl2.

#### IMPORTANT
This method must be implemented by the runtime/interface.

#### NOTE
Default ROS2 implementation: [`ros2.ik_api.IkSyncerRos.send_to_lvl2()`](motion_stack.api.ros2.md#motion_stack.api.ros2.ik_api.IkSyncerRos.send_to_lvl2)

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
  `Type`[`Awaitable`]
* **Type:**
  Class of Future to use

#### *abstract property* sensor

Is called when sensor data is needed.

#### IMPORTANT
This method must be implemented by the runtime/interface.

#### NOTE
Default ROS2 implementation: [`ros2.ik_api.IkSyncerRos.sensor()`](motion_stack.api.ros2.md#motion_stack.api.ros2.ik_api.IkSyncerRos.sensor)

Returns:

* **Return type:**
  `Dict`[`int`, [`Pose`](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose)]

#### unsafe_toward(target, start=None)

Executes one single unsafe step.

* **Returns:**
  True if trajectory finished
* **Return type:**
  `bool`
* **Parameters:**
  * **target** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*)
  * **start** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*  *|* *None*)

#### asap_toward(target, start=None)

Executes one single asap step.

* **Returns:**
  True if trajectory finished
* **Return type:**
  `bool`
* **Parameters:**
  * **target** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*)
  * **start** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*  *|* *None*)

#### lerp_toward(target, start=None)

Executes one single lerp step.

* **Returns:**
  True if trajectory finished
* **Return type:**
  `bool`
* **Parameters:**
  * **target** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*)
  * **start** (*Dict* *[**int* *,* [*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose) *]*  *|* *None*)

## motion_stack.api.joint_syncer module

Python API to sync the movement of several joints.

#### NOTE
The ros2 implementation is available in [`ros2.joint_api`](motion_stack.api.ros2.md#module-motion_stack.api.ros2.joint_api).

This high level API alows for multi-joint control and syncronization (over several legs). This is the base class where, receiving and sending data to motion stack lvl1 is left to be implemented.

### motion_stack.api.joint_syncer.FutureType

placeholder type for a Future (ROS2 Future, asyncio or concurrent)

Alias of `Awaitable`

### *exception* motion_stack.api.joint_syncer.SensorSyncWarning

Bases: `Warning`

### *class* motion_stack.api.joint_syncer.JointSyncer(interpolation_delta=0.08726646259971647, on_target_delta=0.06981317007977318)

Bases: `ABC`

One instance controls and syncronises several joints, safely executing trajectory to targets.

#### NOTE
This class is an abstract base class, the ros2 implementation is available in [`ros2.joint_api.JointSyncerRos`](motion_stack.api.ros2.md#motion_stack.api.ros2.joint_api.JointSyncerRos). Hence,  parts of this class are left to be implmented by the interface/runtime: [`JointSyncer.FutureT()`](#motion_stack.api.joint_syncer.JointSyncer.FutureT), [`JointSyncer.sensor()`](#motion_stack.api.joint_syncer.JointSyncer.sensor), `JointSyncer.send_to_lvl2()`.

#### IMPORTANT
[`JointSyncer.execute()`](#motion_stack.api.joint_syncer.JointSyncer.execute) must be called to compute, update and send the command.

One object instance can only execute one target at a time. However, the limbs or targets can change between calls of the same instance, before the previous task is done.

The trajectory interpolates between two points:

> - The last position (if None: uses sensor, else: last sub-target). This is handled automatically, however `clear` resets the last position to None.
> - The input target.

Several interpolation strategies to reach the target are available:

> - LERP: [`JointSyncer.lerp()`](#motion_stack.api.joint_syncer.JointSyncer.lerp)
> - ASAP: [`JointSyncer.asap()`](#motion_stack.api.joint_syncer.JointSyncer.asap)
> - Unsafe: [`JointSyncer.unsafe()`](#motion_stack.api.joint_syncer.JointSyncer.unsafe)
> - Speed: `JointSyncer.speed()`
* **Parameters:**
  * **interpolation_delta** (*float*) – (rad) During movement, how much error is allowed from the path. if exceeded, movement slows down.
  * **on_target_delta** (*float*) – (rad) Delta at which the trajectory/task is considered finished and the Future is switched to `done`.

#### last_future

Future of the latest task/trajectory that was run.

#### SEND_UNTIL_DONE

When true, the command messages sill stop being published when the sensor data is on target. When false, it stops after sending the target command once. True is improves reliability at the expense of more messages sent, better for lossy networks.

#### execute()

Executes one step of the task/trajectory.

This must be called frequently.

#### clear()

Resets the trajectory starting point onto the current sensor positions.

#### IMPORTANT
Use when:
: - The trajectory is stuck unable to interpolate.
  - External motion happened, thus the last position used by the syncer is no longer valid.

#### lerp(target)

Starts executing a lerp trajectory toward the target.

LERP: all joints reach the target at the same time.

* **Parameters:**
  **target** (*Dict* *[**str* *,* *float* *]*) – `key` = joint name ; `value` = joint angle
* **Returns:**
  Future of the task. Done when sensors are on target.
* **Return type:**
  `Awaitable`

#### asap(target)

Starts executing a asap trajectory toward the target.

ASAP: joints will reach their tagets indepently, as fast as possible

* **Parameters:**
  **target** (*Dict* *[**str* *,* *float* *]*) – `key` = joint name ; `value` = joint angle
* **Returns:**
  Future of the task. Done when sensorare on target.
* **Return type:**
  `Awaitable`

#### unsafe(target)

Starts executing a unsafe trajectory toward the target.

Unsafe: Similar to ASAP except the final target is sent directly to the motor, so the movement will not stop in case of crash, errors, network issue AND you cannot cancel it.

* **Parameters:**
  **target** (*Dict* *[**str* *,* *float* *]*) – `key` = joint name ; `value` = joint angle
* **Returns:**
  Future of the task. Done when sensorare on target.
* **Return type:**
  `Awaitable`

#### speed_safe(target, delta_time)

Starts executing a speed safe trajectory at the target speeds.

Speed Safe: Moves the joints at a given set speed and keeps them in sync positon-wise.

#### WARNING
This method is in early developpement and hasn’t been thouroughly tested.

#### NOTE
This sends position commands and not speed commands. This is to avoid dangerous joint runaway if issue arises.

* **Parameters:**
  * **target** (*Dict* *[**str* *,* *float* *]*) – key = joint name ; value = joint speed
  * **delta_time** (*float* *|* *Callable* *[* *[* *]* *,* *float* *]*) – Function giving the elapsed time in seconds (float) since the last time it was called. A constant float value can also be used but it is not recommanded.
* **Returns:**
  Future of the task. This future will never be done unless when canceled.
* **Return type:**
  `Awaitable`

#### ready(joints)

Returns wether a movement using those joints is possible or not.

* **Parameters:**
  **joints** (*Set* *|* *Dict*) – Joints that one wants to use for a movement
* **Returns:**
  - True if movement is possible
  - Missing joints
* **Return type:**
  `Tuple`[`bool`, `Set`]

#### abs_from_rel(offset)

Absolute position of the joints that correspond to the given relative offset.

### Example

*joint_1* is at 45 deg, offset is 20 deg. Return will be 65 deg.

* **Parameters:**
  **offset** (*Dict* *[**str* *,* *float* *]*) – Relative positions.
* **Returns:**
  Absolute position.
* **Return type:**
  `Dict`[`str`, `float`]

#### dummy_print_jstate(data, prefix='')

* **Parameters:**
  * **data** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)
  * **prefix** (*str*)

#### dummy_print_target(data, prefix='')

* **Parameters:**
  * **data** (*Dict* *[**str* *,* *float* *]*)
  * **prefix** (*str*)

#### dummy_print_sensor(data, prefix='')

* **Parameters:**
  * **data** (*Dict* *[**str* *,* [*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)
  * **prefix** (*str*)

#### *abstractmethod* send_to_lvl1(states)

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
  `Type`[`Awaitable`]
* **Type:**
  Class of Future to use

#### *abstract property* sensor

Is called when sensor data is need.

#### IMPORTANT
This method must be implemented by the runtime/interface.

#### NOTE
Default ROS2 implementation: [`ros2.joint_api.JointSyncerRos.sensor()`](motion_stack.api.ros2.md#motion_stack.api.ros2.joint_api.JointSyncerRos.sensor)

Returns:

* **Return type:**
  `Dict`[`str`, [`JState`](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState)]

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

### motion_stack.api.joint_syncer.only_position(js_dict)

Extract positions from a dict or list of JState. None is ignored

* **Return type:**
  `Dict`[`str`, `float`]
* **Parameters:**
  **js_dict** (*Dict* *[**str* *,* [*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*  *|* *List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

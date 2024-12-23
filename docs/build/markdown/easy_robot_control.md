# easy_robot_control package

## Subpackages

* [easy_robot_control.injection package](easy_robot_control.injection.md)
  * [Submodules](easy_robot_control.injection.md#submodules)
  * [easy_robot_control.injection.offsetter module](easy_robot_control.injection.md#module-easy_robot_control.injection.offsetter)
  * [easy_robot_control.injection.topic_pub module](easy_robot_control.injection.md#module-easy_robot_control.injection.topic_pub)
  * [Module contents](easy_robot_control.injection.md#module-easy_robot_control.injection)
* [easy_robot_control.launch package](easy_robot_control.launch.md)
  * [Submodules](easy_robot_control.launch.md#submodules)
  * [easy_robot_control.launch.builder module](easy_robot_control.launch.md#module-easy_robot_control.launch.builder)
  * [easy_robot_control.launch.default_params module](easy_robot_control.launch.md#module-easy_robot_control.launch.default_params)
  * [Module contents](easy_robot_control.launch.md#module-easy_robot_control.launch)
* [easy_robot_control.my_rtb_fix package](easy_robot_control.my_rtb_fix.md)
  * [Submodules](easy_robot_control.my_rtb_fix.md#submodules)
  * [easy_robot_control.my_rtb_fix.fixed_urdf module](easy_robot_control.my_rtb_fix.md#module-easy_robot_control.my_rtb_fix.fixed_urdf)
  * [Module contents](easy_robot_control.my_rtb_fix.md#module-easy_robot_control.my_rtb_fix)
* [easy_robot_control.ros2_numpy package](easy_robot_control.ros2_numpy.md)
  * [Submodules](easy_robot_control.ros2_numpy.md#submodules)
  * [easy_robot_control.ros2_numpy.transformations module](easy_robot_control.ros2_numpy.md#module-easy_robot_control.ros2_numpy.transformations)
  * [Module contents](easy_robot_control.ros2_numpy.md#module-easy_robot_control.ros2_numpy)
* [easy_robot_control.utils package](easy_robot_control.utils.md)
  * [Submodules](easy_robot_control.utils.md#submodules)
  * [easy_robot_control.utils.csv module](easy_robot_control.utils.md#module-easy_robot_control.utils.csv)
  * [easy_robot_control.utils.hero_mapping module](easy_robot_control.utils.md#module-easy_robot_control.utils.hero_mapping)
  * [easy_robot_control.utils.hyper_sphere_clamp module](easy_robot_control.utils.md#module-easy_robot_control.utils.hyper_sphere_clamp)
  * [easy_robot_control.utils.joint_state_util module](easy_robot_control.utils.md#module-easy_robot_control.utils.joint_state_util)
  * [easy_robot_control.utils.math module](easy_robot_control.utils.md#module-easy_robot_control.utils.math)
  * [easy_robot_control.utils.pure_remap module](easy_robot_control.utils.md#module-easy_robot_control.utils.pure_remap)
  * [easy_robot_control.utils.state_remaper module](easy_robot_control.utils.md#module-easy_robot_control.utils.state_remaper)
  * [easy_robot_control.utils.trajectories module](easy_robot_control.utils.md#module-easy_robot_control.utils.trajectories)
  * [Module contents](easy_robot_control.utils.md#module-easy_robot_control.utils)

## Submodules

## easy_robot_control.EliaNode module

I this adds functionalities to the default ros2 Node object.
And slowly became a mess I have to cleanup …

* **Author:**
  Elian NEPPEL
* **Lab:**
  SRL, Moonshot team

### easy_robot_control.EliaNode.tf2np(tf)

converts a TF into a np array and quaternion

* **Parameters:**
  **tf** (`Transform`) – TF to convert
* **Returns:**
  xyz coordinates
  quat: quaternion for the rotation
* **Return type:**
  xyz

### easy_robot_control.EliaNode.np2tf(coord=None, quat=None, sendNone=False)

converts an NDArray and quaternion into a Transform.

* **Parameters:**
  * **coord** (`Union`[`None`, `NDArray`[`Shape`[3], `floating`], `Sequence`[`float`]]) – xyz coordinates
  * **quat** (`Optional`[`quaternion`]) – quaternion for the rotation
  * **sendNone** (*bool*)
* **Returns:**
  resulting TF
* **Return type:**
  tf

### easy_robot_control.EliaNode.np2tfReq(coord=None, quat=None)

converts an NDArray and quaternion into a Transform request for a service.

* **Parameters:**
  * **xyz** – xyz coordinates
  * **quat** (`Optional`[`quaternion`]) – quaternion for the rotation
  * **coord** (*ndarray* *|* *None*)
* **Return type:**
  `TFService_Request`
* **Returns:**
  Resulting Request for a service call

### easy_robot_control.EliaNode.np2TargetSet(arr=None)

Converts a target set message to np array

* **Return type:**
  `TargetSet`
* **Parameters:**
  **arr** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*  *|* *None*)

### easy_robot_control.EliaNode.targetSet2np(ts)

Converts a np array to target set message

* **Return type:**
  `NDArray`[`Shape`[3], `floating`]
* **Parameters:**
  **ts** (*TargetSet*)

### easy_robot_control.EliaNode.error_catcher(func)

This is a wrapper to catch and display exceptions.

#### NOTE
This only needs to be used on functions executed in callbacks. It is not                 necessary everywhere.

### Example

```default
@error_catcher
def foo(..):
    ...
```

* **Parameters:**
  **func** (`Callable`) – Function executed by a callback
* **Returns:**
  warpped function

### easy_robot_control.EliaNode.rosTime2Float(time)

Converts ros2 time objects to seconds as float

* **Parameters:**
  **time** (`Union`[`Time`, `Duration`]) – ros time obj
* **Return type:**
  `float`
* **Returns:**
  corresponding seconds as float value

### easy_robot_control.EliaNode.list_cyanize(l, default_color=None)

Makes each element of a list cyan.

* **Parameters:**
  * **l** (`Iterable`) – Iterable
  * **default_color** (`Optional`[`str`]) – color to go back to outise of the cyan
* **Return type:**
  `str`

Returns:

### easy_robot_control.EliaNode.replace_incompatible_char_ros2(string_to_correct)

Sanitizes strings for use by ros2.

replace character that cannot be used for Ros2 Topics by \_
inserts “WARN” in front if topic starts with incompatible char

* **Return type:**
  `str`
* **Parameters:**
  **string_to_correct** (*str*)

### *class* easy_robot_control.EliaNode.TCOL

Bases: `object`

Colors for  the terminal

#### HEADER *= '\\x1b[95m'*

#### OKBLUE *= '\\x1b[94m'*

#### OKCYAN *= '\\x1b[96m'*

#### OKGREEN *= '\\x1b[92m'*

#### WARNING *= '\\x1b[33;20m'*

#### FAIL *= '\\x1b[91m'*

#### ENDC *= '\\x1b[0m'*

#### BOLD *= '\\x1b[1m'*

#### UNDERLINE *= '\\x1b[4m'*

### easy_robot_control.EliaNode.get_src_folder(package_name)

Absolute path to workspace/src/package_name.

#### NOTE
Meant for debugging. Avoid using this, you should build properly.

* **Parameters:**
  **package_name** (`str`) – workspace/src/package_name
* **Return type:**
  `str`

Returns: Absolute path as str

### easy_robot_control.EliaNode.transform_joint_to_transform_Rx(transform, jointET)

Takes a transform and a joint (TRANSFORM \* +-Rxyz), and returns the
(rotational) transform so that RESULT \* Rx = TRANSFORM \* +-Rxyz.
So the transform now places the x base vector onto the axis.

* **Parameters:**
  * **transform** (`ET`)
  * **jointET** (`ET`)
* **Return type:**
  `ET`
* **Returns:**
  RESULT \* Rx = TRANSFORM \* +-Rxyz

### easy_robot_control.EliaNode.loadAndSet_URDF(urdf_path, end_effector_name=None, start_effector_name=None)

I am so sorry. This works to parse the urdf I don’t have time to explain

#### NOTE
will change, I hate this

* **Parameters:**
  * **urdf_path** (`str`)
  * **end_effector_name** (`Union`[`str`, `int`, `None`])
  * **start_effector_name** (*str* *|* *None*)
* **Return type:**
  `Tuple`[`Robot`, `ETS`, `List`[`str`], `List`[`Joint`], `Optional`[`Link`]]

Returns:

### easy_robot_control.EliaNode.future_list_complete(future_list)

Returns True is all futures in the input list are done.

* **Parameters:**
  **future_list** (`Union`[`List`[`Future`], `Future`]) – a list of futures
* **Return type:**
  `bool`
* **Returns:**
  True if all futures are done

### *class* easy_robot_control.EliaNode.EZRate(parent, frequency, clock=None)

Bases: `object`

* **Parameters:**
  * **parent** (*Node*)
  * **frequency** (*float*)
  * **clock** (*Clock* *|* *None*)

#### sleep()

sleeps (blocking) until next tick

* **Return type:**
  `None`

#### destroy()

Destroys the object

* **Return type:**
  `None`

#### is_ready()

* **Return type:**
  `bool`

### *class* easy_robot_control.EliaNode.EliaNode(name)

Bases: `Node`

* **Parameters:**
  **name** (*str*)

#### wait_for_lower_level(more_services={}, all_requiered=False)

Blocks until all or any service is available.

#### NOTE
- List of waited services is given by services_to_wait ros2 param
- Overload this to wait for an additional service

* **Parameters:**
  * **more_services** (`Iterable`[`str`]) – add more services
  * **all_requiered** (`bool`) – if True, all listed services must be available

#### getNow()

quick: self.get_clock().now()

* **Return type:**
  `Time`

#### sleep(seconds)

sleeps using the node’s clock.

#### NOTE
Special case for good old foxy

* **Parameters:**
  **seconds** (`float`) – time to sleep
* **Return type:**
  `None`

#### wait_on_futures(future_list, wait_Hz=10)

Waits for the completion of a list of futures, checking completion at the
provided rate.

* **Parameters:**
  * **future_list** (`Union`[`List`[`Future`], `Future`]) – List of Future to wait for
  * **wait_Hz** (`float`) – rate at which to wait

#### perror(object, force=False)

Prints/Logs error if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** (`Any`) – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### pwarn(object, force=False)

Prints/Logs warning if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### pinfo(object, force=False)

Prints/Logs info if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **force** (`bool`) – if True the message will print whatever if self.Yapping is.

#### resolve_service_name(service, \*, only_expand=False)

Return a service name expanded and remapped.

#### NOTE
Overloaded to handle missing foxy

* **Parameters:**
  * **service** (`str`) – service name to be expanded and remapped.
  * **only_expand** (`bool`) – if True, remapping rules won’t be applied.
* **Return type:**
  `str`
* **Returns:**
  a fully qualified service name,
  result of applying expansion and remapping to the given service.

#### setAndBlockForNecessaryNodes(necessary_node_names, silent_trial=3, intervalSec=0.5)

Blocks for nodes to be alive

* **Parameters:**
  * **necessary_node_names** (*List* *[**str* *]*  *|* *str*)
  * **silent_trial** (*int* *|* *None*)
  * **intervalSec** (*float* *|* *None*)

#### get_and_wait_Client(service_name, service_type, cbk_grp=None)

Return the client to the corresponding service, wait for it ot be available.

* **Parameters:**
  * **str** (*service_name -*)
  * **service_type** (*service_type - Ros2*)
  * **cbk_grp** (`Optional`[`CallbackGroup`]) – Not important I think but it’s there
  * **service_name** (*str*)
* **Return type:**
  `Client`

Returns:

#### create_EZrate(frequency, clock=None)

Creates a better rate where rate.destroy actually destroys the rate

* **Parameters:**
  * **frequency** (`float`) – frequency of the rate
  * **clock** (`Optional`[`Clock`]) – clock to use
* **Return type:**
  [`EZRate`](#easy_robot_control.EliaNode.EZRate)
* **Returns:**
  EZRate manipulating a Rate object

#### execute_in_cbk_group(fun, callback_group=None)

Executes the given function by adding it as a callback to a callback_group.

#### NOTE
Pretty sure that’s not how it should be done.

* **Parameters:**
  * **fun** (`Callable`) – function to execute
  * **callback_group** (`Optional`[`CallbackGroup`]) – callback group in which to execute the function
* **Returns:**
  holds the future results
  quardian: the guard condition object in the callback_group
* **Return type:**
  future

### easy_robot_control.EliaNode.myMain(nodeClass, multiThreaded=False, args=None)

Main function used through the motion stack.

* **Parameters:**
  * **nodeClass** – Node to spin
  * **multiThreaded** (`bool`) – using multithreaded executor or not
  * **(****)** (*args*)

## easy_robot_control.gait_key_dev module

This node is responsible for controlling movement of Moonbot HERO.
For now keyboard and controller (PS4).

Authors: Elian NEPPEL, Shamistan KARIMOV
Lab: SRL, Moonshot team

### easy_robot_control.gait_key_dev.float_formatter()

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

### *class* easy_robot_control.gait_key_dev.JoyState(bits=0, stickR=array([0.00, 0.00]), stickL=array([0.00, 0.00]), R2=0.0, L2=0.0)

Bases: `object`

* **Parameters:**
  * **bits** (*int*)
  * **stickR** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **stickL** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **R2** (*float*)
  * **L2** (*float*)

#### bits *: `int`* *= 0*

#### stickR *: `ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]]* *= array([0.00, 0.00])*

#### stickL *: `ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]]* *= array([0.00, 0.00])*

#### R2 *: `float`* *= 0.0*

#### L2 *: `float`* *= 0.0*

### *class* easy_robot_control.gait_key_dev.Leg(number, parent)

Bases: [`Leg`](#easy_robot_control.leg_api.Leg)

* **Parameters:**
  * **number** (*int*)
  * **parent** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))

#### recover()

* **Return type:**
  `Future`

#### halt()

* **Return type:**
  `Future`

### *class* easy_robot_control.gait_key_dev.Wheel(number, parent)

Bases: [`Leg`](#easy_robot_control.gait_key_dev.Leg)

* **Parameters:**
  * **number** (*int*)
  * **parent** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))

#### add_joints(all_joints)

* **Return type:**
  `List`[[`JointMini`](#easy_robot_control.leg_api.JointMini)]
* **Parameters:**
  **all_joints** (*List* *[**str* *]*)

#### connect_movement_clients()

#### forward()

#### backward()

#### stop()

### *class* easy_robot_control.gait_key_dev.KeyGaitNode(name='keygait_node')

Bases: [`EliaNode`](#easy_robot_control.EliaNode.EliaNode)

* **Parameters:**
  **name** (*str*)

#### makeTBclient()

#### leg_scanTMRCBK(\*\*kwargs)

#### wheel_scanTMRCBK(\*\*kwargs)

#### refresh_joint_mapping()

joint mapping based on leg number (realguy or MoonbotH)

#### switch_to_grip_ur16()

joint mapping based on leg number (realguy or MoonbotH)

#### key_upSUBCBK(\*\*kwargs)

#### stop_all_joints()

stops all joint by sending the current angle as target.
if speed was set, sends a speed of 0 instead

#### all_wheel_speed(speed)

Need a re-work

#### minimal_wheel_speed(speed)

Need a re-work

#### tricycle_wheel_speed(speed)

Need a re-work

#### dragon_wheel_speed(speed)

Need a re-work

#### wheels_speed(wheels, speed)

Need a re-work

#### key_downSUBCBK(\*\*kwargs)

#### default_3legs()

#### align_with(leg_number)

* **Parameters:**
  **leg_number** (*int*)

#### default_vehicle()

#### default_dragon()

#### dragon_align()

#### dragon_front_left()

#### dragon_front_right()

#### dragon_base_lookup()

#### dragon_base_lookdown()

#### dragon_back_left()

#### dragon_back_right()

#### zero_without_grippers()

#### goToTargetBody(ts=None, bodyXYZ=None, bodyQuat=None, blocking=True)

* **Return type:**
  `Union`[`Future`, `SendTargetBody_Response`]
* **Parameters:**
  * **ts** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
  * **bodyXYZ** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
  * **bodyQuat** (*quaternion* *|* *None*)
  * **blocking** (*bool*)

#### euler_to_quaternion(roll, pitch, yaw)

Convert Euler angles to a quaternion.

* **Parameters:**
  * **roll** (*float*) – Rotation around the X-axis in radians.
  * **pitch** (*float*) – Rotation around the Y-axis in radians.
  * **yaw** (*float*) – Rotation around the Z-axis in radians.
* **Returns:**
  The resulting quaternion.
* **Return type:**
  Quaternion

#### msg_to_JoyBits(msg)

Converts a joy msg to a JoyState

* **Return type:**
  [`JoyState`](#easy_robot_control.gait_key_dev.JoyState)
* **Parameters:**
  **msg** (*Joy*)

#### display_JoyBits(joy_state)

* **Parameters:**
  **joy_state** ([*JoyState*](#easy_robot_control.gait_key_dev.JoyState))

#### any_pressed(bits, button_names)

Checks if any button in the list is pressed.

* **Parameters:**
  * **bits** (`int`) – set of joybits to check against
  * **button_names** (`Union`[`List`[`Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`]], `Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`]]) – list of button names to check if True
* **Return type:**
  `bool`
* **Returns:**
  True if any bit corresponding to a button is True.

#### bits2name(bits)

Converts a bit field to a list of button names

* **Return type:**
  `List`[`Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`]]
* **Parameters:**
  **bits** (*int*)

#### one_bit2name(bits)

Converts a bit field with 1 bit to 1, to a single button name

* **Return type:**
  `Optional`[`Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`]]
* **Parameters:**
  **bits** (*int*)

#### joy_pressed(button_name)

Executes for each button that is pressed. Like a callback.

* **Parameters:**
  * **bits** – Should only have one single bit set to 1, for 1 single button
  * **button_name** (*Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]*)

#### joy_released(button_name)

Executes for each button that is released. Like a callback.

* **Parameters:**
  * **bits** – Should only have one single bit set to 1, for 1 single button
  * **button_name** (*Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]*)

#### get_joint_index(selected_joint)

* **Return type:**
  `Optional`[`int`]
* **Parameters:**
  **selected_joint** (*int*)

#### joySUBCBK(\*\*kwargs)

#### joint_control_joy()

#### joint_timer_start()

#### *static* collapseT_KeyCodeModifier(variable)

Collapses the variable onto a KeyCodeModifier type, or None

* **Return type:**
  `Optional`[`Tuple`[`int`, `Union`[`int`, `Literal`[`'ANY'`]]]]
* **Returns:**
  None if variable is not a KCM
  The variable as a KCM type-hint if it is a KCM
* **Parameters:**
  **variable** (*Any*)

#### *static* collapseT_JoyCodeModifier(variable)

Collapses the variable onto a JoyCodeModifier type, or None

* **Return type:**
  `Optional`[`Tuple`[`Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`], `Union`[`int`, `Literal`[`'ANY'`]]]]
* **Returns:**
  None if variable is not a JCM
  The variable as a JCM type-hint if it is a JCM
* **Parameters:**
  **variable** (*Any*)

#### *static* remap_onto_any(mapping, input)

runs the input through the INPUTMap as if the key_modifier was any
if it is already, it does not run it.

* **Parameters:**
  * **mapping** (*Dict* *[**Tuple* *[**int* *,* *int* *|* *Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Tuple* *[* *~typing.Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]* *,* *int* *|*  *~typing.Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Literal* *[* *'ALWAYS'* *]* *,*  *~typing.List* *[* *~typing.Callable* *[* *[* *]* *,*  *~typing.Any* *]* *]* *]*)
  * **input** (*Tuple* *[**int* *,* *int* *|* *Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Tuple* *[* *~typing.Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]* *,* *int* *|*  *~typing.Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Literal* *[* *'ALWAYS'* *]*)

#### *static* connect_mapping(mapping, input)

Given the user input, executes the corresponding function mapping

* **Parameters:**
  * **mapping** (`Dict`[`Union`[`Tuple`[`int`, `Union`[`int`, `Literal`[`'ANY'`]]], `Tuple`[`Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`], `Union`[`int`, `Literal`[`'ANY'`]]], `Literal`[`'ALWAYS'`]], `List`[`Callable`[[], `Any`]]]) – Dict of function to execute
  * **input** (`Union`[`Tuple`[`int`, `Union`[`int`, `Literal`[`'ANY'`]]], `Tuple`[`Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`], `Union`[`int`, `Literal`[`'ANY'`]]], `Literal`[`'ALWAYS'`]]) – key to the entry to execute

#### select_leg(leg_ind)

Selects the leg(s) for operation. If None select all.

* **Parameters:**
  **leg_ind** (`Optional`[`List`[`int`]]) – List of leg keys (leg numbers) to use

#### cycle_leg_selection(increment)

Cycles the leg selection by increment
if None, selects all known legs

* **Parameters:**
  **increment** (*int* *|* *None*)

#### get_active_leg(leg_key=None)

Return the keys to get the current active legs from the self.legs dict

* **Parameters:**
  * **leg_number** – you can specify a leg key if you need instead of using active legs
  * **leg_key** (*List* *[**int* *]*  *|* *int* *|* *None*)
* **Return type:**
  `List`[[`Leg`](#easy_robot_control.gait_key_dev.Leg)]
* **Returns:**
  list of active leg keys

#### get_active_leg_keys(leg_key=None)

Return the keys to get the current active legs from the self.legs dict

* **Parameters:**
  * **leg_number** – you can specify a leg key if you need instead of using active legs
  * **leg_key** (*List* *[**int* *]*  *|* *int* *|* *None*)
* **Return type:**
  `List`[`int`]
* **Returns:**
  list of active leg keys

#### halt_all()

#### halt_detected()

#### recover_all(leg_keys=None)

* **Parameters:**
  **leg_keys** (*List* *[**int* *]*  *|* *int* *|* *None*)

#### recover_legs(leg_keys=None)

* **Parameters:**
  **leg_keys** (*List* *[**int* *]*  *|* *int* *|* *None*)

#### set_joint_speed(speed, joint=None, leg_number=None)

Sets joint speed or given joints and legs.
If Nones, picks the selected or active things

* **Parameters:**
  * **speed** (*float*)
  * **joint** (*str* *|* *int* *|* *None*)
  * **leg_number** (*int* *|* *None*)

#### start_ik2_timer()

properly checks and start the timer loop for ik of lvl2

#### ik2TMRCBK()

Timer callback responsable for fast ik movement of lvl2

#### send_ik2_offset(xyz=None, quat=None, ee_relative=False)

* **Parameters:**
  * **xyz** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
  * **quat** (*quaternion* *|* *None*)
  * **ee_relative** (*bool*)

#### send_ik2_movement(xyz=None, quat=None, ee_relative=False)

debug

* **Parameters:**
  * **xyz** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
  * **quat** (*quaternion* *|* *None*)
  * **ee_relative** (*bool*)

#### select_joint(joint_index)

can be better

#### angle_zero(leg_number=None)

Sets all joint angles to 0 (dangerous)

* **Parameters:**
  **leg_number** (`Union`[`List`[`int`], `int`, `None`]) – The leg on which to set. If none, applies on the active leg

#### enter_vehicle_mode()

Creates the sub input map for vehicle

* **Return type:**
  `None`
* **Returns:**
  InputMap for joint control

#### enter_dragon_mode()

Creates the sub input map for dragon

* **Return type:**
  `None`
* **Returns:**
  InputMap for joint control

#### enter_tricycle_mode()

Creates the sub input map for tricycle

* **Return type:**
  `None`
* **Returns:**
  InputMap for joint control

#### enter_leg_mode()

Creates the sub input map for leg selection

* **Return type:**
  `None`
* **Returns:**
  InputMap for leg selection

#### ik2_switch_rel_mode(val=None)

* **Parameters:**
  **val** (*bool* *|* *None*)

#### inch()

#### enter_ik2()

Creates the sub input map for ik control lvl2 by elian

* **Return type:**
  `None`
* **Returns:**
  InputMap for ik2 control

#### enter_joint_mode()

Creates the sub input map for joint control

* **Return type:**
  `None`
* **Returns:**
  InputMap for joint control

#### no_no_leg()

Makes sure no legs are not selected

#### enter_select_mode()

Mode to select other modes.
Should always be accessible when pressing ESC key

#### easy_mode()

#### create_main_map()

Creates the main input map, mapping user input to functions,
This is supposed to be constant + always active, unlike the sub_map

* **Return type:**
  `Dict`[`Union`[`Tuple`[`int`, `Union`[`int`, `Literal`[`'ANY'`]]], `Tuple`[`Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`], `Union`[`int`, `Literal`[`'ANY'`]]], `Literal`[`'ALWAYS'`]], `List`[`Callable`[[], `Any`]]]

### easy_robot_control.gait_key_dev.main(args=None)

## easy_robot_control.gait_node module

This node is responsible for chosing targets and positions.

Author: Elian NEPPEL
Lab: SRL, Moonshot team

### easy_robot_control.gait_node.float_formatter()

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

### *class* easy_robot_control.gait_node.JointMini(joint_name, prefix, parent_leg)

Bases: `object`

* **Parameters:**
  * **joint_name** (*str*)
  * **prefix** (*str*)
  * **parent_leg** ([*Leg*](#easy_robot_control.gait_node.Leg))

#### is_active()

#### self_report()

#### *property* effort *: float | None*

#### *property* speed *: float | None*

#### *property* angle *: float | None*

#### apply_speed_target(speed)

Moves the joint at a given speed using position command.
It sends everincreasing angle target
(this is for safety, so it stops when nothing happens).

The next angle target sent cannot be more than MAX_DELTA radian away
from the current sensor angle
(safe, because you can only do small movements with this method)
If it is too far away, the speed value will decrease
to match the max speed of the joint (not exactly, it will be a little higher).

* **Parameters:**
  **speed** (`Optional`[`float`]) – approx speed value (max speed if for 1) or None
* **Return type:**
  `None`

#### speedTMRCBK(\*\*kwargs)

#### apply_angle_target(angle)

Sets angle target for the joint, and cancels speed command

* **Parameters:**
  **angle** (`float`) – angle target
* **Return type:**
  `None`

### *class* easy_robot_control.gait_node.Leg(number, parent)

Bases: `object`

Helps you use lvl 1-2-3 of a leg

* **Parameters:**
  * **number** (*int*)
  * **parent** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))

#### number

leg number

#### parent

parent node spinning

#### joint_name_list

list of joints belonging to the leg

#### *static* do_i_exist(number, parent, timeout=0.1)

Slow do not use to spam scan.
Returns True if the leg is alive

* **Parameters:**
  * **number** (*int*)
  * **parent** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))
  * **timeout** (*float*)

#### self_report()

* **Return type:**
  `str`

#### send_joint_cmd(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### look_for_joints()

scans and updates the list of joints of this leg

#### go2zero()

sends angle target of 0 on all joints

#### get_joint_obj(joint)

* **Return type:**
  `Optional`[[`JointMini`](#easy_robot_control.gait_node.JointMini)]
* **Parameters:**
  **joint** (*int* *|* *str*)

#### get_angle(joint)

Gets an angle from a joint

* **Parameters:**
  **joint** (`Union`[`int`, `str`]) – joint name or number (alphabetically ordered)
* **Return type:**
  `Optional`[`float`]
* **Returns:**
  last recieved angle as float

#### set_angle(angle, joint)

Sends a angle to a joint

* **Parameters:**
  * **angle** (`float`) – rad
  * **joint** (`Union`[`int`, `str`]) – joint name or number (alphabetically ordered)
* **Return type:**
  `bool`
* **Returns:**
  True if message sent

#### ik(xyz=None, quat=None)

Publishes an ik target for the leg () relative to baselink. Motion stack lvl2

* **Parameters:**
  * **xyz** (`Union`[`None`, `ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]], `Sequence`[`float`]])
  * **quat** (`Optional`[`quaternion`])
* **Return type:**
  `None`

#### move(xyz=None, quat=None, mvt_type='shift', blocking=True)

Calls the leg’s movement service. Motion stack lvl3

* **Parameters:**
  * **xyz** (`Union`[`None`, `ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]], `Sequence`[`float`]]) – vector part of the tf
  * **quat** (`Optional`[`quaternion`]) – quat of the tf
  * **mvt_type** (`Literal`[`'shift'`, `'transl'`, `'rot'`, `'hop'`]) – type of movement to call
  * **blocking** (`bool`) – if false returns a Future. Else returns the response
* **Return type:**
  `Union`[`Future`, `TFService_Response`]

Returns:

### *class* easy_robot_control.gait_node.GaitNode

Bases: [`EliaNode`](#easy_robot_control.EliaNode.EliaNode)

#### firstSpinCBK(\*\*kwargs)

#### hero_vehicle()

#### hero_dragon()

#### hero_arm()

#### gustavo()

#### ashutosh(res=None)

* **Return type:**
  `None`

#### shiftCrawlDebbug(res=None)

#### crawl1Wheel(\*\*kwargs)

#### mZeroBasicSetAndWalk(\*\*kwargs)

#### randomPosLoop()

for multi legged robots

#### getTargetSet()

* **Return type:**
  `Future`

#### getTargetSetBlocking()

* **Return type:**
  `ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]]

#### goToTargetBody(ts=None, bodyXYZ=None, bodyQuat=None, blocking=True)

* **Return type:**
  `Union`[`Future`, `SendTargetBody_Response`]
* **Parameters:**
  * **ts** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
  * **bodyXYZ** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
  * **bodyQuat** (*quaternion* *|* *None*)
  * **blocking** (*bool*)

#### crawlToTargetSet(NDArray)

* **Return type:**
  `None`

#### goToDefault()

#### stand()

### easy_robot_control.gait_node.main(args=None)

## easy_robot_control.ik_heavy_node module

This node is responsible for recieving targets in the body reference frame, and send the
corresponding angles to the motors.

Author: Elian NEPPEL
Lab: SRL, Moonshot team

### easy_robot_control.ik_heavy_node.float_formatter()

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

### *class* easy_robot_control.ik_heavy_node.WheelMiniNode(joint_name, wheel_size_mm, parent_node)

Bases: `object`

* **Parameters:**
  * **joint_name** (*str*)
  * **wheel_size_mm** (*float*)
  * **parent_node** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))

#### publish_speed_below(speed)

Sends speed to nodes below

* **Parameters:**
  * **float** (*angle*)
  * **speed** (*float*)
* **Return type:**
  `None`

#### roll(speed)

Increases the angular speed correspongin to the linear speed

* **Parameters:**
  * **float** (*distance*) – distance to roll
  * **speed** (*float*)
* **Return type:**
  `None`

### *class* easy_robot_control.ik_heavy_node.IKNode

Bases: [`EliaNode`](#easy_robot_control.EliaNode.EliaNode)

#### firstSpinCBK(\*\*kwargs)

#### all_limits(et_chain, jobjL)

* **Parameters:**
  * **et_chain** (*ETS*)
  * **jobjL** (*List* *[**Joint* *]*)

#### roll_CBK(\*\*kwargs)

#### compute_raw_ik(xyz, quat, start, compute_budget=None, mvt_duration=None)

* **Return type:**
  `Tuple`[`Optional`[`ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]]], `bool`]
* **Parameters:**
  * **xyz** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **quat** (*quaternion*)
  * **start** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **compute_budget** (*Duration* *|* *None*)
  * **mvt_duration** (*Duration* *|* *None*)

#### find_next_ik(xyz, quat, compute_budget=None, mvt_duration=None)

* **Return type:**
  `ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]]
* **Parameters:**
  * **xyz** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **quat** (*quaternion*)
  * **compute_budget** (*Duration* *|* [*EZRate*](#easy_robot_control.EliaNode.EZRate) *|* *None*)
  * **mvt_duration** (*Duration* *|* *None*)

#### set_ik_CBK(\*\*kwargs)

#### replace_none_target(xyz, quat)

* **Return type:**
  `Tuple`[`ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]], `quaternion`]
* **Parameters:**
  * **xyz** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **quat** (*quaternion*)

#### joint_readSUBCBK(\*\*kwargs)

#### send_command(angles)

* **Parameters:**
  **angles** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)

#### current_fk()

* **Return type:**
  `Tuple`[`ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]], `quaternion`]

#### publish_tip_pos(\*\*kwargs)

### easy_robot_control.ik_heavy_node.main()

## easy_robot_control.joint_state_interface module

### *class* easy_robot_control.joint_state_interface.JointHandler(name, parent_node, joint_object, IGNORE_LIM=False, MARGIN=0.0)

Bases: `object`

This handles a single joint.
The main purpose is to update stateSensor and stateCommand. As well as getting the
newest values for those (in order to not continuously publish unchanging data).

* **Parameters:**
  * **name** (*str*)
  * **parent_node** ([*JointNode*](#easy_robot_control.joint_state_interface.JointNode))
  * **joint_object** (*Joint*)
  * **IGNORE_LIM** (*bool*)
  * **MARGIN** (*float*)

#### load_limit(ignore, jobj=None)

Loads the limit from the (urdf) joint object

* **Parameters:**
  * **ignore** (`bool`) – if limits should be ignored
  * **jobj** (*Joint* *|* *None*)

#### *property* limit_rejected *: bool*

if the limit was rejected when loading (often when not defined in urdf)

* **Type:**
  Returns

#### speakup_when_angle()

start a verbose check every seconds for new angles

#### checkAngle(angle)

True is angle is valid or None

* **Return type:**
  `bool`
* **Parameters:**
  **angle** (*float* *|* *None*)

#### applyAngleLimit(angle)

Clamps the angle between the joints limits

* **Return type:**
  `Tuple`[`float`, `bool`]
* **Parameters:**
  **angle** (*float*)

#### resetAnglesAtZero()

#### update_js_command(js)

Updates the stateCommand to a new js.

* **Parameters:**
  **js** ([*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState))

#### is_new_jssensor(js)

True if js is different enough from the last received.
Also true if stateSensor is more the TOL_NO_CHANGE.time old relative to the new

* **Parameters:**
  **js** ([*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState))

#### setJSSensor(js)

Updates the stateSensor to a new js.

* **Parameters:**
  **js** ([*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState))

#### process_angle_command(angle)

This runs on new js before updating stateCommand

* **Return type:**
  `float`
* **Parameters:**
  **angle** (*float*)

#### process_velocity_command(speed)

This runs on new js before updating stateCommand

* **Return type:**
  `Optional`[`float`]
* **Parameters:**
  **speed** (*float*)

#### process_effort_command(eff)

This runs on new js before updating stateCommand

* **Return type:**
  `float`
* **Parameters:**
  **eff** (*float*)

#### set_effortCBK(\*\*kwargs)

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
  [`JState`](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState)
* **Parameters:**
  **reset** (*bool*)

#### get_freshCommand(reset=True)

returns command data that is newer than the last time it was called.
full of None is not newer

* **Return type:**
  [`JState`](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState)
* **Parameters:**
  **reset** (*bool*)

### *class* easy_robot_control.joint_state_interface.JointNode

Bases: [`EliaNode`](#easy_robot_control.EliaNode.EliaNode)

#### send_to_lvl0(states)

Sends states to lvl0 (commands for motors).
This function is executed every time data needs to be sent down.
Change/overload this method with what you need

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### send_to_lvl2(states)

Sends states to lvl2 (states for ik).
This function is executed every time data needs to be sent up.
Change/overload this method with what you need

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### js_from_lvl0(\*\*kwargs)

#### js_from_lvl2(\*\*kwargs)

#### coming_from_lvl2(states)

Processes incomming commands from lvl2 ik.
Call this function after processing the ros message

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### coming_from_lvl0(states)

Processes incomming sensor states from lvl0 motors.
Call this function after processing the ros message.
Always do super().coming_from_lvl0(states) before your code,
Unless you know what you are doing

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### advertiserSRVCBK(req, res)

Sends an JointState mainly to advertise the names of the joints

* **Return type:**
  `ReturnJointState_Response`
* **Parameters:**
  * **req** (*ReturnJointState_Request*)
  * **res** (*ReturnJointState_Response*)

#### defined_undefined()

Return joints with and without poistion data received yet

* **Return type:**
  `Tuple`[`List`[`str`], `List`[`str`]]
* **Returns:**
  Tuple(List[joint names that did not receive any data],
  List[joint names that have data])

#### angle_read_checkTMRCBK(\*\*kwargs)

#### firstSpinCBK(\*\*kwargs)

#### robot_body_pose_cbk(\*\*kwargs)

#### smoother(x)

smoothes the interval [0, 1] to have a soft start and end
(derivative is zero)

* **Return type:**
  `ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]]
* **Parameters:**
  **x** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)

#### smooth_body_trans(\*\*kwargs)

#### go_zero_allCBK(\*\*kwargs)

### easy_robot_control.joint_state_interface.main(args=None)

## easy_robot_control.lazy_joint_state_publisher module

Overloading the joint_state_publisher package
so it does not publish joint states that are not actively published

Lots of black magic being used

### *class* easy_robot_control.lazy_joint_state_publisher.dummy_pub

Bases: `object`

#### publish(msg)

### *class* easy_robot_control.lazy_joint_state_publisher.LazyJointStatePublisher(description_file)

Bases: `JointStatePublisher`

#### source_cb(msg)

#### delete_inactive_from_msg(msg)

Deletes joints that are not part of self.active_joints from a message

* **Return type:**
  `JointState`
* **Parameters:**
  **msg** (*JointState*)

### easy_robot_control.lazy_joint_state_publisher.main()

## easy_robot_control.leg_api module

Provides api anc controllers to control leg ik and joints directly

Author: Elian NEPPEL
Lab: SRL, Moonshot team

### easy_robot_control.leg_api.float_formatter()

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

### *class* easy_robot_control.leg_api.Pose(time, xyz, quat)

Bases: `object`

* **Parameters:**
  * **time** (*Time*)
  * **xyz** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*)
  * **quat** (*quaternion*)

#### time *: `Time`*

#### xyz *: `NDArray`[`Shape`[3], `floating`]*

#### quat *: `quaternion`*

#### close2zero(atol=(1, 0.01))

* **Return type:**
  `bool`

### *class* easy_robot_control.leg_api.JointMini(joint_name, prefix, parent_leg)

Bases: `object`

* **Parameters:**
  * **joint_name** (*str*)
  * **prefix** (*str*)
  * **parent_leg** ([*Leg*](#easy_robot_control.leg_api.Leg))

#### is_active()

#### self_report()

#### *property* effort *: float | None*

#### *property* speed *: float | None*

#### *property* angle *: float | None*

#### apply_speed_target(speed)

Moves the joint at a given speed using position command.
It sends everincreasing angle target
(this is for safety, so it stops when nothing happens).

The next angle target sent cannot be more than MAX_DELTA radian away
from the current sensor angle
(safe, because you can only do small movements with this method)
If it is too far away, the speed value will decrease
to match the max speed of the joint (not exactly, it will be a little higher).

* **Parameters:**
  **speed** (`Optional`[`float`]) – approx speed value (max speed if for 1) or None
* **Return type:**
  `None`

#### apply_angle_target(angle)

Sets angle target for the joint, and cancels speed command

* **Parameters:**
  **angle** (`float`) – angle target
* **Return type:**
  `None`

### *class* easy_robot_control.leg_api.Leg(number, parent)

Bases: `object`

Helps you use lvl 1-2-3 of a leg

* **Parameters:**
  * **number** (*int*)
  * **parent** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))

#### number

leg number

#### parent

parent node spinning

#### joint_name_list

list of joints belonging to the leg

#### connect_movement_clients()

#### *static* do_i_exist(number, parent, timeout=1)

Slow do not use to spam scan.
Returns True if the leg is alive

* **Parameters:**
  * **number** (*int*)
  * **parent** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))
  * **timeout** (*float*)

#### self_report()

* **Return type:**
  `str`

#### add_joints(all_joints)

* **Return type:**
  `List`[[`JointMini`](#easy_robot_control.leg_api.JointMini)]
* **Parameters:**
  **all_joints** (*List* *[**str* *]*)

#### look_for_joints()

scans and updates the list of joints of this leg

#### go2zero()

sends angle target of 0 on all joints

#### get_joint_obj(joint)

Gets the corresponding joint object is exists

* **Parameters:**
  **joint** (`Union`[`int`, `str`]) – joint name or number (alphabetically ordered)
* **Return type:**
  `Optional`[[`JointMini`](#easy_robot_control.leg_api.JointMini)]

#### get_angle(joint)

Gets an angle from a joint

* **Parameters:**
  **joint** (`Union`[`int`, `str`]) – joint name or number (alphabetically ordered)
* **Return type:**
  `Optional`[`float`]
* **Returns:**
  last recieved angle as float

#### set_angle(angle, joint)

Sends a angle to a joint

* **Parameters:**
  * **angle** (`float`) – rad
  * **joint** (`Union`[`int`, `str`]) – joint name or number (alphabetically ordered)
* **Return type:**
  `bool`
* **Returns:**
  True if message sent

#### ik(xyz=None, quat=None)

Publishes an ik target for the leg () relative to baselink. Motion stack lvl2

* **Parameters:**
  * **xyz** (`Union`[`None`, `NDArray`, `Sequence`[`float`]])
  * **quat** (`Optional`[`quaternion`])
* **Return type:**
  `None`

#### move(xyz=None, quat=None, mvt_type='shift', blocking=True)

Calls the leg’s movement service. Motion stack lvl3

* **Parameters:**
  * **xyz** (`Union`[`None`, `NDArray`, `Sequence`[`float`]]) – vector part of the tf
  * **quat** (`Optional`[`quaternion`]) – quat of the tf
  * **mvt_type** (`Literal`[`'shift'`, `'transl'`, `'rot'`, `'hop'`]) – type of movement to call
  * **blocking** (`bool`) – if false returns a Future. Else returns the response
* **Return type:**
  `Union`[`Future`, `TFService_Response`]

Returns:

### *class* easy_robot_control.leg_api.Ik2(leg)

Bases: `object`

Provides ik2 methods given a leg

* **Parameters:**
  **leg** ([*Leg*](#easy_robot_control.leg_api.Leg))

#### run_task(\*\*kwargs)

#### *property* quat_now *: quaternion | None*

#### *property* xyz_now *: NDArray[Shape[3], floating] | None*

#### *property* now_pose *: [Pose](#easy_robot_control.leg_api.Pose) | None*

#### reset()

Resets the trajectory start point onto the current end effector position

* **Return type:**
  [`Pose`](#easy_robot_control.leg_api.Pose)

#### clear()

Clears the trajectory start point

#### make_abs_pos(xyz, quat, ee_relative=False)

* **Return type:**
  `Optional`[[`Pose`](#easy_robot_control.leg_api.Pose)]
* **Parameters:**
  * **xyz** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*  *|* *None*)
  * **quat** (*quaternion* *|* *None*)
  * **ee_relative** (*bool* *|* *None*)

#### controlled_motion(xyz, quat, ee_relative=False)

Continuously calls self.step_toward(…) in the task timer.

Cancels the previous task and future.
Replaces it with a new function to execute and new future.
Returns the assiciated future (you can cancel it, and know when it’s done).

* **Parameters:**
  * **xyz** (`Optional`[`NDArray`[`Shape`[3], `floating`]]) – target in mm
  * **quat** (`Optional`[`quaternion`]) – target as quaternion
  * **ee_relative** (`Optional`[`bool`]) – if the movement should bee performed relative to the end effector
* **Return type:**
  `Future`

Returns
: Future assiciated with the movement’s task.

#### step_toward(xyz, quat, ee_relative=False)

Sends a single ik command to move toward the target.
Not meant to reach the target!!!

This method is robust to IK errors and motor speed saturation. It will adapt its
speed according to the robot response to keep track with the path.

The command will clamp not farther from the current EE than self.sphere_xyz_radius
and self.sphere_quat_radius.
Increase those values to allow for a wider margin of error
(also leading to higher speed)

The math become imprecise with big deltas in quaternions. See clamp_xyz_quat(…)
Do not use if self.last_pose.quat is opposite to quat.
(idk what will happen but you wont like it)

* **Parameters:**
  * **xyz** (`Optional`[`NDArray`[`Shape`[3], `floating`]]) – target in mm
  * **quat** (`Optional`[`quaternion`]) – target as quaternion
  * **ee_relative** (`Optional`[`bool`]) – if the movement should bee performed relative to the end effector
* **Return type:**
  `bool`

#### offset(xyz, quat, ee_relative=False)

Wrapper around self.step_toward(…),
Origin is placed onto the EE.
ee_relative specifies if the origin should have the same orientation as the EE

* **Parameters:**
  * **xyz** (`Optional`[`NDArray`[`Shape`[3], `floating`]]) – mm to move by
  * **quat** (`Optional`[`quaternion`]) – quaternion to move by
  * **ee_relative** (`Optional`[`bool`]) – True: movement origin is the EE.
    False: movement origin is the baselink.

## easy_robot_control.leg_node module

This node is responsible for recieving targets with trajectories of the
corresponding leg. It will performe smooth movements in the body center frame of reference
or relative to the current end effector position.
This node keeps track of the leg end effector to generate trajectories to the target.

Author: Elian NEPPEL
Lab: SRL, Moonshot team

### *class* easy_robot_control.leg_node.LegNode

Bases: [`EliaNode`](#easy_robot_control.EliaNode.EliaNode)

#### firstSpinCBK(\*\*kwargs)

#### publish_to_roll(\*\*kwargs)

#### smart_roll_cbk(\*\*kwargs)

#### publish_to_ik(\*\*kwargs)

#### trajectory_executor(\*\*kwargs)

#### trajectory_finished_cbk(\*\*kwargs)

#### queue_xyz_empty()

* **Return type:**
  `bool`

#### queue_quat_empty()

* **Return type:**
  `bool`

#### queue_roll_empty()

* **Return type:**
  `bool`

#### pop_xyzq_from_traj(\*\*kwargs)

#### pop_roll_from_traj(\*\*kwargs)

#### get_final_xyz()

* **Return type:**
  `ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]]

#### get_final_quat()

* **Return type:**
  `quaternion`

#### get_final_target()

returns the final position of where the leg is. Or will be at the end of the
current trajectory_queue.

* **Returns:**
  np.array float(3,) - final coordinates
* **Return type:**
  end_point

#### fuse_xyz_trajectory(\*\*kwargs)

#### fuse_quat_trajectory(\*\*kwargs)

#### fuse_roll_trajectory(\*\*kwargs)

#### add_to_trajectory(\*\*kwargs)

#### send_most_recent_tip(\*\*kwargs)

#### smoother(x)

smoothes the interval [0, 1] to have a soft start and end
(derivative is zero)

* **Return type:**
  `ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]]
* **Parameters:**
  **x** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)

#### smoother_complement(\*\*kwargs)

#### rel_transl(\*\*kwargs)

#### rel_rotation(\*\*kwargs)

#### *static* point_with_quat(vect)

* **Parameters:**
  **vect** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)

#### point_toward(\*\*kwargs)

#### point_wheel(\*\*kwargs)

#### shift(\*\*kwargs)

#### rel_hop(\*\*kwargs)

#### tip_pos_received_cbk(\*\*kwargs)

#### check_divergence(\*\*kwargs)

#### overwrite_target(\*\*kwargs)

#### wait_end_of_motion()

waits for the trajectory to end. This function is very bad, but I don’t need
nor have time to do something better.

We should keep track of which trajectory are beeing queued to improve

* **Return type:**
  `None`

#### append_trajectory(trajectory_function)

The function will be executed before the next read of the trajectory sequentialy
This avoids trajectory_function in multiple threads modifying indentical data simultaneously.

* **Parameters:**
  * **function** (*trajectory_function -*) – function to execute
  * **trajectory_function** (*Callable*)
* **Returns:**
  holds the future results of the function if needed
* **Return type:**
  future

#### shift_cbk(\*\*kwargs)

#### rel_transl_srv_cbk(\*\*kwargs)

#### rel_hop_srv_cbk(\*\*kwargs)

#### rot_cbk(\*\*kwargs)

#### point_cbk(request, response)

* **Return type:**
  `TFService_Response`
* **Parameters:**
  * **request** (*TFService_Request*)
  * **response** (*TFService_Response*)

### easy_robot_control.leg_node.main(args=None)

## easy_robot_control.mover_node module

This node is responsible for synchronising several leg movement in order to move the
cente body and perform steps.

Author: Elian NEPPEL
Lab: SRL, Moonshot team

### *class* easy_robot_control.mover_node.MoverNode

Bases: [`EliaNode`](#easy_robot_control.EliaNode.EliaNode)

#### firstSpinCBK(\*\*kwargs)

#### go2_targetbodyCBK(\*\*kwargs)

#### get_targetsetCBK(\*\*kwargs)

#### update_tip_pos(\*\*kwargs)

#### body_tfshift(\*\*kwargs)

#### body_shift(shift)

* **Return type:**
  `None`
* **Parameters:**
  **shift** (*ndarray*)

#### manual_body_translation_rviz(coord, quat=quaternion(1, 0, 0, 0))

* **Return type:**
  `None`
* **Parameters:**
  * **coord** (*ndarray*)
  * **quat** (*quaternion*)

#### set_body_transform_rviz(coord, quat=quaternion(1, 0, 0, 0))

* **Return type:**
  `None`
* **Parameters:**
  * **coord** (*ndarray*)
  * **quat** (*quaternion*)

#### body_tfshift_cbk(request, response)

* **Return type:**
  `TFService_Response`
* **Parameters:**
  * **request** (*TFService_Request*)
  * **response** (*TFService_Response*)

#### multi_transl(target_set)

* **Parameters:**
  **target_set** (*ndarray*)

#### multi_hop(target_set)

* **Parameters:**
  **target_set** (*ndarray*)

#### multi_shift(target_set)

* **Parameters:**
  **target_set** (*ndarray*)

#### multi_rotate(target_set, quat)

* **Parameters:**
  * **target_set** (*ndarray*)
  * **quat** (*quaternion*)

#### move_body_and_hop(body_xyz, target_set, body_quat=None)

* **Parameters:**
  * **body_xyz** (*ndarray*)
  * **target_set** (*ndarray*)
  * **body_quat** (*quaternion* *|* *None*)

### easy_robot_control.mover_node.main(args=None)

## Module contents

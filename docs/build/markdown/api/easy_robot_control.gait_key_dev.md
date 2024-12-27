# easy_robot_control.gait_key_dev module

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

### *class* easy_robot_control.gait_key_dev.JoyState(bits=0, stickR=array([0.0, 0.0]), stickL=array([0.0, 0.0]), R2=0.0, L2=0.0)

Bases: `object`

* **Parameters:**
  * **bits** (*int*)
  * **stickR** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **stickL** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **R2** (*float*)
  * **L2** (*float*)

#### bits *= 0*

**Type:**    `int`

#### stickR *= array([0.0, 0.0])*

**Type:**    `ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]

#### stickL *= array([0.0, 0.0])*

**Type:**    `ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]

#### R2 *= 0.0*

**Type:**    `float`

#### L2 *= 0.0*

**Type:**    `float`

### *class* easy_robot_control.gait_key_dev.Leg(number, parent)

Bases: [`Leg`](easy_robot_control.leg_api.md#easy_robot_control.leg_api.Leg)

* **Parameters:**
  * **number** (*int*)
  * **parent** ([*EliaNode*](easy_robot_control.EliaNode.md#easy_robot_control.EliaNode.EliaNode))

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
  * **parent** ([*EliaNode*](easy_robot_control.EliaNode.md#easy_robot_control.EliaNode.EliaNode))

#### add_joints(all_joints)

* **Return type:**
  `List`[[`JointMini`](easy_robot_control.leg_api.md#easy_robot_control.leg_api.JointMini)]
* **Parameters:**
  **all_joints** (*List* *[**str* *]*)

#### connect_movement_clients()

#### forward()

#### backward()

#### stop()

### *class* easy_robot_control.gait_key_dev.KeyGaitNode(name='keygait_node')

Bases: [`EliaNode`](easy_robot_control.EliaNode.md#easy_robot_control.EliaNode.EliaNode)

* **Parameters:**
  **name** (*str*)

#### makeTBclient()

#### leg_scanTMRCBK()

Looks for new legs and joints

#### wheel_scanTMRCBK()

Looks for new legs and joints

#### refresh_joint_mapping()

joint mapping based on leg number (realguy or MoonbotH)

#### switch_to_grip_ur16()

joint mapping based on leg number (realguy or MoonbotH)

#### key_upSUBCBK(msg)

Executes when keyboard released

* **Parameters:**
  **msg** (*Key*)

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

#### key_downSUBCBK(msg)

Executes when keyboard pressed

* **Parameters:**
  **msg** (*Key*)

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

#### joySUBCBK(msg)

Processes incomming joy messages.
Converts and stores the received state in self.joy_state .
executes self.joy_pressed and self.joy_released for each button that changed state

* **Parameters:**
  **msg** (`Joy`) – Ros2 Joy message type

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

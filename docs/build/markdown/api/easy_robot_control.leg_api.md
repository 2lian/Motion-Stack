# easy_robot_control.leg_api module

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

#### time

**Type:**    `Time`

#### xyz

**Type:**    `NDArray`[`Shape`[`3`], `floating`]

#### quat

**Type:**    `quaternion`

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

#### *property* effort

#### *property* speed

#### *property* angle

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
  * **parent** ([*EliaNode*](easy_robot_control.EliaNode.md#easy_robot_control.EliaNode.EliaNode))

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
  * **parent** ([*EliaNode*](easy_robot_control.EliaNode.md#easy_robot_control.EliaNode.EliaNode))
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

#### run_task()

#### *property* quat_now

#### *property* xyz_now

#### *property* now_pose

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

# easy_robot_control.gait_node module

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

#### speedTMRCBK()

Updates angle based on stored speed. Stops if speed is None

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
  * **parent** ([*EliaNode*](easy_robot_control.EliaNode.md#easy_robot_control.EliaNode.EliaNode))

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
  * **parent** ([*EliaNode*](easy_robot_control.EliaNode.md#easy_robot_control.EliaNode.EliaNode))
  * **timeout** (*float*)

#### self_report()

* **Return type:**
  `str`

#### send_joint_cmd(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState) *]*)

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

Bases: [`EliaNode`](easy_robot_control.EliaNode.md#easy_robot_control.EliaNode.EliaNode)

#### firstSpinCBK()

#### hero_vehicle()

#### hero_dragon()

#### hero_arm()

#### gustavo()

#### ashutosh(res=None)

* **Return type:**
  `None`

#### shiftCrawlDebbug(res=None)

#### crawl1Wheel()

for moonbot hero one arm+wheel only

#### mZeroBasicSetAndWalk()

for moonbot 0

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

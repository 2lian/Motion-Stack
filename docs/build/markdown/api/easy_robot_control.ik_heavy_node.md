# easy_robot_control.ik_heavy_node module

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
  * **parent_node** ([*EliaNode*](easy_robot_control.EliaNode.md#easy_robot_control.EliaNode.EliaNode))

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

Bases: [`EliaNode`](easy_robot_control.EliaNode.md#easy_robot_control.EliaNode.EliaNode)

#### firstSpinCBK()

#### all_limits(et_chain, jobjL)

* **Parameters:**
  * **et_chain** (*ETS*)
  * **jobjL** (*List* *[**Joint* *]*)

#### roll_CBK(msg)

* **Return type:**
  `None`
* **Parameters:**
  **msg** (*Float64* *|* *float*)

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
  * **compute_budget** (*Duration* *|* [*EZRate*](easy_robot_control.EliaNode.md#easy_robot_control.EliaNode.EZRate) *|* *None*)
  * **mvt_duration** (*Duration* *|* *None*)

#### set_ik_CBK(msg)

recieves target from leg, converts to numpy, computes IK, sends angle
results to joints

* **Parameters:**
  **msg** (`Transform`) – target as Ros2 Vector3
* **Return type:**
  `None`

#### replace_none_target(xyz, quat)

* **Return type:**
  `Tuple`[`ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]], `quaternion`]
* **Parameters:**
  * **xyz** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **quat** (*quaternion*)

#### joint_readSUBCBK(js)

* **Parameters:**
  **js** (*JointState*)

#### send_command(angles)

* **Parameters:**
  **angles** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)

#### current_fk()

* **Return type:**
  `Tuple`[`ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]], `quaternion`]

#### publish_tip_pos()

Computes foward kinematics given angles stored in array,
publishes tip position result.
This is executed x ms after an angle reading is received

* **Return type:**
  `None`

### easy_robot_control.ik_heavy_node.main()

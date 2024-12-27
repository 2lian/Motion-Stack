# easy_robot_control.utils.joint_state_util module

### *class* easy_robot_control.utils.joint_state_util.JState(name, position=None, velocity=None, effort=None, time=None)

Bases: `object`

* **Parameters:**
  * **name** (*str*)
  * **position** (*float* *|* *None*)
  * **velocity** (*float* *|* *None*)
  * **effort** (*float* *|* *None*)
  * **time** (*Time* *|* *None*)

#### name

**Type:**    `str`

#### position *= None*

**Type:**    `Optional`[`float`]

#### velocity *= None*

**Type:**    `Optional`[`float`]

#### effort *= None*

**Type:**    `Optional`[`float`]

#### time *= None*

**Type:**    `Optional`[`Time`]

#### copy()

* **Return type:**
  [`JState`](#easy_robot_control.utils.joint_state_util.JState)

### easy_robot_control.utils.joint_state_util.js_from_ros(jsin)

* **Return type:**
  `List`[[`JState`](#easy_robot_control.utils.joint_state_util.JState)]
* **Parameters:**
  **jsin** (*JointState*)

### easy_robot_control.utils.joint_state_util.intersect_names(js_in, names)

* **Return type:**
  `JointState`
* **Parameters:**
  * **js_in** (*JointState*)
  * **names** (*Sequence* *[**str* *]*)

### easy_robot_control.utils.joint_state_util.js_copy(js)

* **Return type:**
  [`JState`](#easy_robot_control.utils.joint_state_util.JState)
* **Parameters:**
  **js** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))

### easy_robot_control.utils.joint_state_util.impose_state(onto, fromm)

* **Return type:**
  [`JState`](#easy_robot_control.utils.joint_state_util.JState)
* **Parameters:**
  * **onto** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))
  * **fromm** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))

### easy_robot_control.utils.joint_state_util.js_changed(j1, j2, delta)

* **Return type:**
  `bool`
* **Parameters:**
  * **j1** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))
  * **j2** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))
  * **delta** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))

### easy_robot_control.utils.joint_state_util.js_diff(j1, j2)

* **Return type:**
  [`JState`](#easy_robot_control.utils.joint_state_util.JState)
* **Parameters:**
  * **j1** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))
  * **j2** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))

### easy_robot_control.utils.joint_state_util.stateOrderinator3000(allStates)

Converts a list  of JState to multiple ros JointStates messages.
Timestamp ignored.

* **Return type:**
  `List`[`JointState`]
* **Parameters:**
  **allStates** (*Iterable* *[*[*JState*](#easy_robot_control.utils.joint_state_util.JState) *]*)

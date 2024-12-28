# motion_stack.core.utils.joint_state module

### *class* motion_stack.core.utils.joint_state.JState(name, time=None, position=None, velocity=None, effort=None)

Bases: `object`

* **Parameters:**
  * **name** (*str*)
  * **time** ([*Time*](motion_stack.core.utils.time.md#motion_stack.core.utils.time.Time) *|* *None*)
  * **position** (*float* *|* *None*)
  * **velocity** (*float* *|* *None*)
  * **effort** (*float* *|* *None*)

#### name

**Type:**    `str`

#### time *= None*

**Type:**    `Optional`[[`Time`](motion_stack.core.utils.time.md#motion_stack.core.utils.time.Time)]

#### position *= None*

**Type:**    `Optional`[`float`]

#### velocity *= None*

**Type:**    `Optional`[`float`]

#### effort *= None*

**Type:**    `Optional`[`float`]

#### getattr(name)

* **Return type:**
  `Any`
* **Parameters:**
  **name** (*str*)

#### copy()

* **Return type:**
  [`JState`](#motion_stack.core.utils.joint_state.JState)

#### *property* is_initialized

### motion_stack.core.utils.joint_state.js_from_dict_list(dil)

* **Return type:**
  `List`[[`JState`](#motion_stack.core.utils.joint_state.JState)]
* **Parameters:**
  **dil** (*Dict* *[**Literal* *[* *'position'* *,*  *'velocity'* *,*  *'effort'* *,*  *'name'* *,*  *'time'* *]* *,*  *~typing.List* *]*)

### motion_stack.core.utils.joint_state.impose_state(onto, fromm)

* **Return type:**
  [`JState`](#motion_stack.core.utils.joint_state.JState)
* **Parameters:**
  * **onto** ([*JState*](#motion_stack.core.utils.joint_state.JState))
  * **fromm** ([*JState*](#motion_stack.core.utils.joint_state.JState))

### motion_stack.core.utils.joint_state.js_changed(j1, j2, delta)

* **Return type:**
  `bool`
* **Parameters:**
  * **j1** ([*JState*](#motion_stack.core.utils.joint_state.JState))
  * **j2** ([*JState*](#motion_stack.core.utils.joint_state.JState))
  * **delta** ([*JState*](#motion_stack.core.utils.joint_state.JState))

### motion_stack.core.utils.joint_state.js_diff(j1, j2)

* **Return type:**
  [`JState`](#motion_stack.core.utils.joint_state.JState)
* **Parameters:**
  * **j1** ([*JState*](#motion_stack.core.utils.joint_state.JState))
  * **j2** ([*JState*](#motion_stack.core.utils.joint_state.JState))

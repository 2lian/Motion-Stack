# motion_stack.core.utils package

## Submodules

## motion_stack.core.utils.csv module

simple csv utilities

### motion_stack.core.utils.csv.update_csv(file_path, new_str, new_float)

* **Return type:**
  `Tuple`[`str`, `Optional`[`str`]]
* **Parameters:**
  * **new_str** (*str*)
  * **new_float** (*float*)

### motion_stack.core.utils.csv.csv_to_dict(file_path)

* **Return type:**
  `Optional`[`Dict`[`str`, `float`]]

## motion_stack.core.utils.joint_mapper module

### motion_stack.core.utils.joint_mapper.operate_sub_shapers(shaper1, shaper2, op)

* **Return type:**
  `Optional`[`Callable`[[`float`], `float`]]
* **Parameters:**
  * **shaper1** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **shaper2** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **op** (*Callable* *[* *[**float* *,* *float* *]* *,* *float* *]*)

### motion_stack.core.utils.joint_mapper.eggify_shapers(inner, outer)

* **Return type:**
  `Optional`[`Callable`[[`float`], `float`]]
* **Parameters:**
  * **inner** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **outer** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)

### *class* motion_stack.core.utils.joint_mapper.Shaper(position=None, velocity=None, effort=None)

Bases: `object`

Holds and applies functions to position, velocity and effort fields.

If None, the indentity is used.

* **Parameters:**
  * **position** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **velocity** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **effort** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)

#### position *= None*

**Type:**    `Optional`[`Callable`[[`float`], `float`]]

#### velocity *= None*

**Type:**    `Optional`[`Callable`[[`float`], `float`]]

#### effort *= None*

**Type:**    `Optional`[`Callable`[[`float`], `float`]]

### motion_stack.core.utils.joint_mapper.reverse_dict(d)

* **Return type:**
  `Dict`
* **Parameters:**
  **d** (*Dict*)

### motion_stack.core.utils.joint_mapper.remap_names(states, mapping)

* **Parameters:**
  * **states** (*List* *[*[*JState*](#motion_stack.core.utils.joint_state.JState) *]*)
  * **mapping** (*Dict* *[**str* *,* *str* *]*)

### motion_stack.core.utils.joint_mapper.apply_shaper(state, shaper)

* **Parameters:**
  * **state** ([*JState*](#motion_stack.core.utils.joint_state.JState))
  * **shaper** ([*Shaper*](#motion_stack.core.utils.joint_mapper.Shaper))

### motion_stack.core.utils.joint_mapper.shape_states(states, mapping)

* **Parameters:**
  * **states** (*List* *[*[*JState*](#motion_stack.core.utils.joint_state.JState) *]*)
  * **mapping** (*Dict* *[**str* *,* [*Shaper*](#motion_stack.core.utils.joint_mapper.Shaper) *]*)

## motion_stack.core.utils.joint_state module

### *class* motion_stack.core.utils.joint_state.JState(name, time=None, position=None, velocity=None, effort=None)

Bases: `object`

* **Parameters:**
  * **name** (*str*)
  * **time** ([*Time*](#motion_stack.core.utils.time.Time) *|* *None*)
  * **position** (*float* *|* *None*)
  * **velocity** (*float* *|* *None*)
  * **effort** (*float* *|* *None*)

#### name

**Type:**    `str`

#### time *= None*

**Type:**    `Optional`[[`Time`](#motion_stack.core.utils.time.Time)]

#### position *= None*

**Type:**    `Optional`[`float`]

#### velocity *= None*

**Type:**    `Optional`[`float`]

#### effort *= None*

**Type:**    `Optional`[`float`]

#### getattr(name: Literal['name']) → str

#### getattr(name: Literal['time']) → [motion_stack.core.utils.time.Time](#motion_stack.core.utils.time.Time)

#### getattr(name: Literal['position', 'velocity', 'effort']) → float

#### getattr(name)

* **Return type:**
  `Any`
* **Parameters:**
  **name** (*str*)

#### copy()

* **Return type:**
  [`JState`](#motion_stack.core.utils.joint_state.JState)

#### *property* is_initialized

`bool`

* **Type:**
  rtype

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

## motion_stack.core.utils.printing module

### *class* motion_stack.core.utils.printing.TCOL

Bases: `object`

Colors for  the terminal

#### HEADER *= '\\x1b[95m'*

**Type:**    `str`

#### OKBLUE *= '\\x1b[94m'*

**Type:**    `str`

#### OKCYAN *= '\\x1b[96m'*

**Type:**    `str`

#### OKGREEN *= '\\x1b[92m'*

**Type:**    `str`

#### WARNING *= '\\x1b[33;20m'*

**Type:**    `str`

#### FAIL *= '\\x1b[91m'*

**Type:**    `str`

#### ENDC *= '\\x1b[0m'*

**Type:**    `str`

#### BOLD *= '\\x1b[1m'*

**Type:**    `str`

#### UNDERLINE *= '\\x1b[4m'*

**Type:**    `str`

### motion_stack.core.utils.printing.list_cyanize(l, default_color=None)

Makes each element of a list cyan.

* **Parameters:**
  * **l** (*Iterable*) – Iterable
  * **default_color** (*str* *|* *None*) – color to go back to outise of the cyan
* **Return type:**
  str

Returns:

* **Return type:**
  `str`
* **Parameters:**
  * **l** (*Iterable*)
  * **default_color** (*str* *|* *None*)

## motion_stack.core.utils.robot_parsing module

Uses rtb to parse the robot URDF data

### motion_stack.core.utils.robot_parsing.get_limit(joint)

Returns the limits of a joint from rtb parsing

* **Return type:**
  `Tuple`[`float`, `float`]
* **Parameters:**
  **joint** (*Joint*)

### motion_stack.core.utils.robot_parsing.load_set_urdf(urdf_path, end_effector_name=None, start_effector_name=None)

I am so sorry. This works to parse the urdf I don’t have time to explain

#### NOTE
will change, I hate this

* **Parameters:**
  * **urdf_path** (*str*)
  * **end_effector_name** (*str* *|* *int* *|* *None*)
  * **start_effector_name** (*str* *|* *None*)
* **Return type:**
  *Tuple*[*Robot*, *ETS*, *List*[str], *List*[*Joint*], *Link* | None]

Returns:

* **Return type:**
  `Tuple`[`Robot`, `ETS`, `List`[`str`], `List`[`Joint`], `Optional`[`Link`]]
* **Parameters:**
  * **urdf_path** (*str*)
  * **end_effector_name** (*str* *|* *int* *|* *None*)
  * **start_effector_name** (*str* *|* *None*)

## motion_stack.core.utils.static_executor module

### motion_stack.core.utils.static_executor.extract_inner_type(list_type)

Extracts the inner type from a typing.List, such as List[float].

* **Parameters:**
  **list_type** (`type`) – A type hint, e.g., List[float].
* **Returns:**
  The inner type of the list, e.g., float.

### *class* motion_stack.core.utils.static_executor.Spinner

Bases: `ABC`

#### alias *= ''*

**Type:**    `str`

#### *abstract* now()

* **Return type:**
  [`Time`](#motion_stack.core.utils.time.Time)

#### *abstract* error(\*args, \*\*kwargs)

* **Return type:**
  `None`

#### *abstract* warn(\*args, \*\*kwargs)

* **Return type:**
  `None`

#### *abstract* info(\*args, \*\*kwargs)

* **Return type:**
  `None`

#### *abstract* debug(\*args, \*\*kwargs)

* **Return type:**
  `None`

#### *abstract* get_parameter(name, value_type, default=None)

* **Return type:**
  `Any`
* **Parameters:**
  * **name** (*str*)
  * **value_type** (*type*)

### *class* motion_stack.core.utils.static_executor.PythonSpinner

Bases: [`Spinner`](#motion_stack.core.utils.static_executor.Spinner)

#### now()

* **Return type:**
  [`Time`](#motion_stack.core.utils.time.Time)

#### change_time(time)

* **Parameters:**
  **time** ([*Time*](#motion_stack.core.utils.time.Time))

#### error(\*args)

* **Return type:**
  `None`

#### warn(\*args)

* **Return type:**
  `None`

#### info(\*args)

* **Return type:**
  `None`

#### debug(\*args)

* **Return type:**
  `None`

### *class* motion_stack.core.utils.static_executor.FlexNode(spinner)

Bases: `object`

* **Parameters:**
  **spinner** ([*Spinner*](#motion_stack.core.utils.static_executor.Spinner))

#### spinner

**Type:**    [`Spinner`](#motion_stack.core.utils.static_executor.Spinner)

must be initialized and spinning already

#### startup_time

**Type:**    [`Time`](#motion_stack.core.utils.time.Time)

#### now()

* **Return type:**
  [`Time`](#motion_stack.core.utils.time.Time)

#### error(\*args)

#### warn(\*args)

#### info(\*args)

#### debug(\*args)

#### *property* ms_param

`Dict`[`str`, `Any`]

* **Type:**
  rtype

## motion_stack.core.utils.time module

### *class* motion_stack.core.utils.time.Time(value=None, \*, sec=None, nano=None)

Bases: `int`

#### nano()

Return the time as nanoseconds.

#### sec()

Return the time as whole seconds.

#### secf()

Return the time as fractional seconds.

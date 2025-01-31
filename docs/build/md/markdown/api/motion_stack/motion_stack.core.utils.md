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

## motion_stack.core.utils.hypersphere_clamp module

Vectorized functions to clamp onto an hypershpere

Author: Elian NEPPEL
Lab: SRL, Moonshot team

### motion_stack.core.utils.hypersphere_clamp.clamp_to_unit_hs(start, end, sampling_step=0.01)

Finds the farthest point on the segment that is inside the unit hypersphere.

* **Return type:**
  `NDArray`[`Shape`[`N`], `float64`]
* **Parameters:**
  * **start** (*NDArray* *[**Shape* *[**N* *]* *,* *float64* *]*)
  * **end** (*NDArray* *[**Shape* *[**N* *]* *,* *float64* *]*)
  * **sampling_step** (*float*)

### motion_stack.core.utils.hypersphere_clamp.clamp_to_sqewed_hs(center, start, end, radii)

Finds the farthest point on the segment that is inside the sqewed hypersphere.

radii of the hypersphere in each dimensions is computed by streching the space in each dimension, then computing relative to the unit hypersphere, then unstreching.

* **Return type:**
  `NDArray`[`Shape`[`N`], `floating`]
* **Parameters:**
  * **center** (*NDArray* *[**Shape* *[**N* *]* *,* *floating* *]*)
  * **start** (*NDArray* *[**Shape* *[**N* *]* *,* *floating* *]*)
  * **end** (*NDArray* *[**Shape* *[**N* *]* *,* *floating* *]*)
  * **radii** (*NDArray* *[**Shape* *[**N* *]* *,* *floating* *]*)

### motion_stack.core.utils.hypersphere_clamp.fuse_xyz_quat(xyz, quat)

* **Return type:**
  `NDArray`[`Shape`[`7`], `floating`]
* **Parameters:**
  * **xyz** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*)
  * **quat** (*quaternion*)

### motion_stack.core.utils.hypersphere_clamp.unfuse_xyz_quat(arr)

* **Return type:**
  `Tuple`[`NDArray`[`Shape`[`3`], `floating`], `quaternion`]
* **Parameters:**
  **arr** (*NDArray* *[**Shape* *[**7* *]* *,* *floating* *]*)

### motion_stack.core.utils.hypersphere_clamp.clamp_xyz_quat(center, start, end, radii)

wrapper for clamp_to_sqewed_hs specialized in one 3D coordinate + one quaternion.

The math for the quaternion is wrong (lerp instead of slerp). So:
Center and start quat should not be opposite from each-other.
Precision goes down if they are far appart.

* **Parameters:**
  * **center** (*Tuple* *[**NDArray* *[**Shape* *[**3* *]* *,* *floating* *]* *,* *quaternion* *]*) – center from which not to diverge
  * **start** (*Tuple* *[**NDArray* *[**Shape* *[**3* *]* *,* *floating* *]* *,* *quaternion* *]*) – start point of the interpolation
  * **end** (*Tuple* *[**NDArray* *[**Shape* *[**3* *]* *,* *floating* *]* *,* *quaternion* *]*) – end point of the interpolation
  * **radii** (*Tuple* *[**float* *,* *float* *]*) – allowed divergence for coord and quat
* **Return type:**
  *Tuple*[*NDArray*[*Shape*[3], *floating*], *quaternion*]

Returns:

* **Return type:**
  `Tuple`[`NDArray`[`Shape`[`3`], `floating`], `quaternion`]
* **Parameters:**
  * **center** (*Tuple* *[**NDArray* *[**Shape* *[**3* *]* *,* *floating* *]* *,* *quaternion* *]*)
  * **start** (*Tuple* *[**NDArray* *[**Shape* *[**3* *]* *,* *floating* *]* *,* *quaternion* *]*)
  * **end** (*Tuple* *[**NDArray* *[**Shape* *[**3* *]* *,* *floating* *]* *,* *quaternion* *]*)
  * **radii** (*Tuple* *[**float* *,* *float* *]*)

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
  * **onto** ([*JState*](#motion_stack.core.utils.joint_state.JState) *|* *None*)
  * **fromm** ([*JState*](#motion_stack.core.utils.joint_state.JState) *|* *None*)

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

## motion_stack.core.utils.math module

### motion_stack.core.utils.math.qt_normalize(q)

* **Parameters:**
  **q** (*quaternion*)

### motion_stack.core.utils.math.qt_repr(q)

* **Return type:**
  `str`
* **Parameters:**
  **q** (*quaternion*)

## motion_stack.core.utils.pose module

### *class* motion_stack.core.utils.pose.Pose(time, xyz, quat)

Bases: `object`

* **Parameters:**
  * **time** ([*Time*](#motion_stack.core.utils.time.Time))
  * **xyz** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*)
  * **quat** (*quaternion*)

#### time

**Type:**    [`Time`](#motion_stack.core.utils.time.Time)

#### xyz

**Type:**    `NDArray`[`Shape`[`3`], `floating`]

#### quat

**Type:**    `quaternion`

#### close2zero(atol=(1, 0.01))

* **Return type:**
  `bool`

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

### motion_stack.core.utils.robot_parsing.replace_incompatible_char_ros2(string_to_correct)

Sanitizes strings for use by ros2.

replace character that cannot be used for Ros2 Topics by \_
inserts “WARN” in front if topic starts with incompatible char

* **Return type:**
  `str`
* **Parameters:**
  **string_to_correct** (*str*)

### motion_stack.core.utils.robot_parsing.get_limit(joint)

Returns the limits of a joint from rtb parsing

* **Return type:**
  `Tuple`[`float`, `float`]
* **Parameters:**
  **joint** (*Joint*)

### motion_stack.core.utils.robot_parsing.make_ee(ee_string, number_default=0)

* **Return type:**
  `Union`[`None`, `str`, `int`]
* **Parameters:**
  * **ee_string** (*str*)
  * **number_default** (*int*)

### motion_stack.core.utils.robot_parsing.joint_by_joint_fk(et_chain, joint_names)

* **Return type:**
  `List`[`Tuple`[`str`, `NDArray`]]
* **Parameters:**
  * **et_chain** (*ETS*)
  * **joint_names** (*List* *[**str* *]*)

### motion_stack.core.utils.robot_parsing.load_set_urdf_raw(urdf, end_effector_name=None, start_effector_name=None)

Enables calling load_set_urdf with the full urdf string instead of the path

* **Return type:**
  `Tuple`[`Robot`, `ETS`, `List`[`str`], `List`[`Joint`], `Optional`[`Link`]]
* **Parameters:**
  * **urdf** (*str*)
  * **end_effector_name** (*str* *|* *int* *|* *None*)
  * **start_effector_name** (*str* *|* *None*)

### motion_stack.core.utils.robot_parsing.load_set_urdf(urdf_path, end_effector_name=None, start_effector_name=None)

I am so sorry. This works to parse the urdf I don’t have time to explain

#### NOTE
will change, I hate this

This is terrible and still in the code

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

Return the time as fractional seconds.

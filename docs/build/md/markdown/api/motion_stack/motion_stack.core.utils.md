# motion_stack.core.utils package

## Submodules

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

## motion_stack.core.utils.state_remapper module

### motion_stack.core.utils.state_remapper.operate_sub_shapers(shaper1, shaper2, op)

* **Return type:**
  `Optional`[`Callable`[[`float`], `float`]]
* **Parameters:**
  * **shaper1** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **shaper2** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **op** (*Callable* *[* *[**float* *,* *float* *]* *,* *float* *]*)

### motion_stack.core.utils.state_remapper.eggify_shapers(inner, outer)

* **Return type:**
  `Optional`[`Callable`[[`float`], `float`]]
* **Parameters:**
  * **inner** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **outer** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)

### *class* motion_stack.core.utils.state_remapper.Shaper(position=None, velocity=None, effort=None)

Bases: `object`

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

### motion_stack.core.utils.state_remapper.reverse_dict(d)

* **Return type:**
  `Dict`
* **Parameters:**
  **d** (*Dict*)

### motion_stack.core.utils.state_remapper.remap_names(states, mapping)

* **Parameters:**
  * **states** (*List* *[*[*JState*](#motion_stack.core.utils.joint_state.JState) *]*)
  * **mapping** (*Dict* *[**str* *,* *str* *]*)

### motion_stack.core.utils.state_remapper.apply_shaper(state, shaper)

* **Parameters:**
  * **state** ([*JState*](#motion_stack.core.utils.joint_state.JState))
  * **shaper** ([*Shaper*](#motion_stack.core.utils.state_remapper.Shaper))

### motion_stack.core.utils.state_remapper.shape_states(states, mapping)

* **Parameters:**
  * **states** (*List* *[*[*JState*](#motion_stack.core.utils.joint_state.JState) *]*)
  * **mapping** (*Dict* *[**str* *,* [*Shaper*](#motion_stack.core.utils.state_remapper.Shaper) *]*)

### *class* motion_stack.core.utils.state_remapper.StateRemapper(name_map={}, unname_map=None, state_map={}, unstate_map={})

Bases: `object`

* **Parameters:**
  * **name_map** (*Dict* *[**str* *,* *str* *]*)
  * **unname_map** (*Dict* *[**str* *,* *str* *]*  *|* *None*)
  * **state_map** (*Dict* *[**str* *,* [*Shaper*](#motion_stack.core.utils.state_remapper.Shaper) *]*)
  * **unstate_map** (*Dict* *[**str* *,* [*Shaper*](#motion_stack.core.utils.state_remapper.Shaper) *]*)

#### namify(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](#motion_stack.core.utils.joint_state.JState) *]*)

#### unnamify(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](#motion_stack.core.utils.joint_state.JState) *]*)

#### shapify(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](#motion_stack.core.utils.joint_state.JState) *]*)

#### unshapify(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](#motion_stack.core.utils.joint_state.JState) *]*)

#### map(states)

mapping used before sending

* **Parameters:**
  **states** (*List* *[*[*JState*](#motion_stack.core.utils.joint_state.JState) *]*)

#### unmap(states)

mapping used before receiving

* **Parameters:**
  **states** (*List* *[*[*JState*](#motion_stack.core.utils.joint_state.JState) *]*)

#### simplify(names_to_keep)

Eliminates (not in place) all entries whose keys are not in names_to_keep.
:returns: new StateRemapper

* **Return type:**
  [`StateRemapper`](#motion_stack.core.utils.state_remapper.StateRemapper)
* **Parameters:**
  **names_to_keep** (*Iterable* *[**str* *]*)

### motion_stack.core.utils.state_remapper.insert_angle_offset(mapper_in, mapper_out, offsets)

Applies an position offsets to a StateRemapper.
the state_map adds the offset, then the unstate_map substracts it (in mapper_out).

Sub_shapers from mapper_in will overwrite sub_shapers in mapper_out if they are
affected by offsets.
mapper_out = mapper_in, may lead to undefined behavior.
Any function shared between in/out may lead to undefined behavior.
Use deepcopy() to avoid issues.

this is a very rough function. feel free to improve

* **Parameters:**
  * **mapper_in** ([*StateRemapper*](#motion_stack.core.utils.state_remapper.StateRemapper)) – original function map to which offset should be added
  * **mapper_out** ([*StateRemapper*](#motion_stack.core.utils.state_remapper.StateRemapper)) – changes will be stored here
  * **offsets** (*Dict* *[**str* *,* *float* *]*)
* **Return type:**
  `None`

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

#### alias *= ''*

**Type:**    `str`

#### *abstract* get_parameter(name, value_type, default=None)

* **Return type:**
  `Any`
* **Parameters:**
  * **name** (*str*)
  * **value_type** (*type*)

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

#### as_integer_ratio()

Return integer ratio.

Return a pair of integers, whose ratio is exactly equal to the original int
and with a positive denominator.

```pycon
>>> (10).as_integer_ratio()
(10, 1)
>>> (-10).as_integer_ratio()
(-10, 1)
>>> (0).as_integer_ratio()
(0, 1)
```

#### bit_count()

Number of ones in the binary representation of the absolute value of self.

Also known as the population count.

```pycon
>>> bin(13)
'0b1101'
>>> (13).bit_count()
3
```

#### bit_length()

Number of bits necessary to represent self in binary.

```pycon
>>> bin(37)
'0b100101'
>>> (37).bit_length()
6
```

#### conjugate()

Returns self, the complex conjugate of any int.

#### denominator

**Type:**    `GetSetDescriptorType`

the denominator of a rational number in lowest terms

#### from_bytes(byteorder, \*, signed=False)

Return the integer represented by the given array of bytes.

bytes
: Holds the array of bytes to convert.  The argument must either
  support the buffer protocol or be an iterable object producing bytes.
  Bytes and bytearray are examples of built-in objects that support the
  buffer protocol.

byteorder
: The byte order used to represent the integer.  If byteorder is ‘big’,
  the most significant byte is at the beginning of the byte array.  If
  byteorder is ‘little’, the most significant byte is at the end of the
  byte array.  To request the native byte order of the host system, use
  <br/>
  ```
  `
  ```
  <br/>
  sys.byteorder’ as the byte order value.

signed
: Indicates whether two’s complement is used to represent the integer.

#### imag

**Type:**    `GetSetDescriptorType`

the imaginary part of a complex number

#### numerator

**Type:**    `GetSetDescriptorType`

the numerator of a rational number in lowest terms

#### real

**Type:**    `GetSetDescriptorType`

the real part of a complex number

#### to_bytes(length, byteorder, \*, signed=False)

Return an array of bytes representing an integer.

length
: Length of bytes object to use.  An OverflowError is raised if the
  integer is not representable with the given number of bytes.

byteorder
: The byte order used to represent the integer.  If byteorder is ‘big’,
  the most significant byte is at the beginning of the byte array.  If
  byteorder is ‘little’, the most significant byte is at the end of the
  byte array.  To request the native byte order of the host system, use
  <br/>
  ```
  `
  ```
  <br/>
  sys.byteorder’ as the byte order value.

signed
: Determines whether two’s complement is used to represent the integer.
  If signed is False and a negative integer is given, an OverflowError
  is raised.

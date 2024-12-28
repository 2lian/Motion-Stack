# motion_stack.core.utils.static_executor module

### *class* motion_stack.core.utils.static_executor.Spinner

Bases: `object`

#### now()

* **Return type:**
  [`Time`](motion_stack.core.utils.time.md#motion_stack.core.utils.time.Time)

#### error()

* **Return type:**
  `None`

#### warn()

* **Return type:**
  `None`

#### info()

* **Return type:**
  `None`

#### debug()

* **Return type:**
  `None`

#### get_parameter(name, value_type, default=None)

* **Parameters:**
  **name** (*str*)

### *class* motion_stack.core.utils.static_executor.PythonSpinner

Bases: [`Spinner`](#motion_stack.core.utils.static_executor.Spinner)

#### now()

* **Return type:**
  [`Time`](motion_stack.core.utils.time.md#motion_stack.core.utils.time.Time)

#### change_time(time)

* **Parameters:**
  **time** ([*Time*](motion_stack.core.utils.time.md#motion_stack.core.utils.time.Time))

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

#### now()

* **Return type:**
  [`Time`](motion_stack.core.utils.time.md#motion_stack.core.utils.time.Time)

#### error(\*args)

#### warn(\*args)

#### info(\*args)

#### debug(\*args)

#### *property* ms_param

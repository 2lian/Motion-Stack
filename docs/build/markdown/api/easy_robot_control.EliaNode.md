# easy_robot_control.EliaNode module

I this adds functionalities to the default ros2 Node object.
And slowly became a mess I have to cleanup …

* **Author:**
  Elian NEPPEL
* **Lab:**
  SRL, Moonshot team

### easy_robot_control.EliaNode.tf2np(tf)

converts a TF into a np array and quaternion

* **Parameters:**
  **tf** (`Transform`) – TF to convert
* **Returns:**
  xyz coordinates
  quat: quaternion for the rotation
* **Return type:**
  xyz

### easy_robot_control.EliaNode.np2tf(coord=None, quat=None, sendNone=False)

converts an NDArray and quaternion into a Transform.

* **Parameters:**
  * **coord** (`Union`[`None`, `NDArray`[`Shape`[3], `floating`], `Sequence`[`float`]]) – xyz coordinates
  * **quat** (`Optional`[`quaternion`]) – quaternion for the rotation
  * **sendNone** (*bool*)
* **Returns:**
  resulting TF
* **Return type:**
  tf

### easy_robot_control.EliaNode.np2tfReq(coord=None, quat=None)

converts an NDArray and quaternion into a Transform request for a service.

* **Parameters:**
  * **xyz** – xyz coordinates
  * **quat** (`Optional`[`quaternion`]) – quaternion for the rotation
  * **coord** (*ndarray* *|* *None*)
* **Return type:**
  `TFService_Request`
* **Returns:**
  Resulting Request for a service call

### easy_robot_control.EliaNode.np2TargetSet(arr=None)

Converts a target set message to np array

* **Return type:**
  `TargetSet`
* **Parameters:**
  **arr** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*  *|* *None*)

### easy_robot_control.EliaNode.targetSet2np(ts)

Converts a np array to target set message

* **Return type:**
  `NDArray`[`Shape`[3], `floating`]
* **Parameters:**
  **ts** (*TargetSet*)

### easy_robot_control.EliaNode.error_catcher(func)

This is a wrapper to catch and display exceptions.

#### NOTE
This only needs to be used on functions executed in callbacks. It is not                 necessary everywhere.

### Example

```default
@error_catcher
def foo(..):
    ...
```

* **Parameters:**
  **func** (`Callable`) – Function executed by a callback
* **Returns:**
  warpped function

### easy_robot_control.EliaNode.rosTime2Float(time)

Converts ros2 time objects to seconds as float

* **Parameters:**
  **time** (`Union`[`Time`, `Duration`]) – ros time obj
* **Return type:**
  `float`
* **Returns:**
  corresponding seconds as float value

### easy_robot_control.EliaNode.list_cyanize(l, default_color=None)

Makes each element of a list cyan.

* **Parameters:**
  * **l** (`Iterable`) – Iterable
  * **default_color** (`Optional`[`str`]) – color to go back to outise of the cyan
* **Return type:**
  `str`

Returns:

### easy_robot_control.EliaNode.replace_incompatible_char_ros2(string_to_correct)

Sanitizes strings for use by ros2.

replace character that cannot be used for Ros2 Topics by \_
inserts “WARN” in front if topic starts with incompatible char

* **Return type:**
  `str`
* **Parameters:**
  **string_to_correct** (*str*)

### *class* easy_robot_control.EliaNode.TCOL

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

### easy_robot_control.EliaNode.get_src_folder(package_name)

Absolute path to workspace/src/package_name.

#### NOTE
Meant for debugging. Avoid using this, you should build properly.

* **Parameters:**
  **package_name** (`str`) – workspace/src/package_name
* **Return type:**
  `str`

Returns: Absolute path as str

### easy_robot_control.EliaNode.transform_joint_to_transform_Rx(transform, jointET)

Takes a transform and a joint (TRANSFORM \* +-Rxyz), and returns the
(rotational) transform so that RESULT \* Rx = TRANSFORM \* +-Rxyz.
So the transform now places the x base vector onto the axis.

* **Parameters:**
  * **transform** (`ET`)
  * **jointET** (`ET`)
* **Return type:**
  `ET`
* **Returns:**
  RESULT \* Rx = TRANSFORM \* +-Rxyz

### easy_robot_control.EliaNode.loadAndSet_URDF(urdf_path, end_effector_name=None, start_effector_name=None)

I am so sorry. This works to parse the urdf I don’t have time to explain

#### NOTE
will change, I hate this

* **Parameters:**
  * **urdf_path** (`str`)
  * **end_effector_name** (`Union`[`str`, `int`, `None`])
  * **start_effector_name** (*str* *|* *None*)
* **Return type:**
  `Tuple`[`Robot`, `ETS`, `List`[`str`], `List`[`Joint`], `Optional`[`Link`]]

Returns:

### easy_robot_control.EliaNode.future_list_complete(future_list)

Returns True is all futures in the input list are done.

* **Parameters:**
  **future_list** (`Union`[`List`[`Future`], `Future`]) – a list of futures
* **Return type:**
  `bool`
* **Returns:**
  True if all futures are done

### *class* easy_robot_control.EliaNode.EZRate(parent, frequency, clock=None)

Bases: `object`

* **Parameters:**
  * **parent** (*Node*)
  * **frequency** (*float*)
  * **clock** (*Clock* *|* *None*)

#### sleep()

sleeps (blocking) until next tick

* **Return type:**
  `None`

#### destroy()

Destroys the object

* **Return type:**
  `None`

#### is_ready()

* **Return type:**
  `bool`

### *class* easy_robot_control.EliaNode.EliaNode(name)

Bases: `Node`

* **Parameters:**
  **name** (*str*)

#### wait_for_lower_level(more_services={}, all_requiered=False)

Blocks until all or any service is available.

#### NOTE
- List of waited services is given by services_to_wait ros2 param
- Overload this to wait for an additional service

* **Parameters:**
  * **more_services** (`Iterable`[`str`]) – add more services
  * **all_requiered** (`bool`) – if True, all listed services must be available

#### getNow()

quick: self.get_clock().now()

* **Return type:**
  `Time`

#### sleep(seconds)

sleeps using the node’s clock.

#### NOTE
Special case for good old foxy

* **Parameters:**
  **seconds** (`float`) – time to sleep
* **Return type:**
  `None`

#### wait_on_futures(future_list, wait_Hz=10)

Waits for the completion of a list of futures, checking completion at the
provided rate.

* **Parameters:**
  * **future_list** (`Union`[`List`[`Future`], `Future`]) – List of Future to wait for
  * **wait_Hz** (`float`) – rate at which to wait

#### perror(object, force=False)

Prints/Logs error if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** (`Any`) – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### pwarn(object, force=False)

Prints/Logs warning if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### pinfo(object, force=False)

Prints/Logs info if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **force** (`bool`) – if True the message will print whatever if self.Yapping is.

#### resolve_service_name(service, \*, only_expand=False)

Return a service name expanded and remapped.

#### NOTE
Overloaded to handle missing foxy

* **Parameters:**
  * **service** (`str`) – service name to be expanded and remapped.
  * **only_expand** (`bool`) – if True, remapping rules won’t be applied.
* **Return type:**
  `str`
* **Returns:**
  a fully qualified service name,
  result of applying expansion and remapping to the given service.

#### setAndBlockForNecessaryNodes(necessary_node_names, silent_trial=3, intervalSec=0.5)

Blocks for nodes to be alive

* **Parameters:**
  * **necessary_node_names** (*List* *[**str* *]*  *|* *str*)
  * **silent_trial** (*int* *|* *None*)
  * **intervalSec** (*float* *|* *None*)

#### get_and_wait_Client(service_name, service_type, cbk_grp=None)

Return the client to the corresponding service, wait for it ot be available.

* **Parameters:**
  * **str** (*service_name -*)
  * **service_type** (*service_type - Ros2*)
  * **cbk_grp** (`Optional`[`CallbackGroup`]) – Not important I think but it’s there
  * **service_name** (*str*)
* **Return type:**
  `Client`

Returns:

#### create_EZrate(frequency, clock=None)

Creates a better rate where rate.destroy actually destroys the rate

* **Parameters:**
  * **frequency** (`float`) – frequency of the rate
  * **clock** (`Optional`[`Clock`]) – clock to use
* **Return type:**
  [`EZRate`](#easy_robot_control.EliaNode.EZRate)
* **Returns:**
  EZRate manipulating a Rate object

#### execute_in_cbk_group(fun, callback_group=None)

Executes the given function by adding it as a callback to a callback_group.

#### NOTE
Pretty sure that’s not how it should be done.

* **Parameters:**
  * **fun** (`Callable`) – function to execute
  * **callback_group** (`Optional`[`CallbackGroup`]) – callback group in which to execute the function
* **Returns:**
  holds the future results
  quardian: the guard condition object in the callback_group
* **Return type:**
  future

### easy_robot_control.EliaNode.myMain(nodeClass, multiThreaded=False, args=None)

Main function used through the motion stack.

* **Parameters:**
  * **nodeClass** – Node to spin
  * **multiThreaded** (`bool`) – using multithreaded executor or not
  * **(****)** (*args*)

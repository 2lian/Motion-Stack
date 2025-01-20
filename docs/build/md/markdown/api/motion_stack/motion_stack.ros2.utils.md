# motion_stack.ros2.utils package

## Submodules

## motion_stack.ros2.utils.conversion module

### motion_stack.ros2.utils.conversion.ros_to_time(time)

* **Return type:**
  [`Time`](motion_stack.core.utils.md#motion_stack.core.utils.time.Time)
* **Parameters:**
  **time** (*Time*)

### motion_stack.ros2.utils.conversion.time_to_ros(time)

* **Return type:**
  `Time`
* **Parameters:**
  **time** ([*Time*](motion_stack.core.utils.md#motion_stack.core.utils.time.Time))

### motion_stack.ros2.utils.conversion.transform_to_pose(tf, time)

* **Return type:**
  [`Pose`](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose)
* **Parameters:**
  * **tf** (*Transform*)
  * **time** ([*Time*](motion_stack.core.utils.md#motion_stack.core.utils.time.Time))

### motion_stack.ros2.utils.conversion.pose_to_transform(pose, sendNone=False)

* **Return type:**
  `Transform`
* **Parameters:**
  * **pose** ([*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose))
  * **sendNone** (*bool*)

## motion_stack.ros2.utils.executor module

### motion_stack.ros2.utils.executor.error_catcher(func)

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
  **func** (*Callable*) – Function executed by a callback
* **Returns:**
  warpped function

### motion_stack.ros2.utils.executor.future_list_complete(future_list)

Returns True is all futures in the input list are done.

* **Parameters:**
  **future_list** (*List* *[**Future* *]*  *|* *Future*) – a list of futures
* **Returns:**
  True if all futures are done
* **Return type:**
  `bool`

### *class* motion_stack.ros2.utils.executor.EZRate(parent, frequency, clock=None)

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

### *class* motion_stack.ros2.utils.executor.Ros2Spinner(node)

Bases: [`Spinner`](motion_stack.core.utils.md#motion_stack.core.utils.static_executor.Spinner)

* **Parameters:**
  **node** (*Node*)

#### get_parameter(name, value_type, default=None)

* **Return type:**
  `Any`
* **Parameters:**
  * **name** (*str*)
  * **value_type** (*type*)

#### wait_for_lower_level(more_services={}, all_requiered=False)

Blocks until all or any service is available.

#### NOTE
- List of waited services is given by services_to_wait ros2 param
- Overload this to wait for an additional service

* **Parameters:**
  * **more_services** (*Iterable* *[**str* *]*) – add more services
  * **all_requiered** (*bool*) – if True, all listed services must be available

#### now()

quick: self.get_clock().now()

* **Return type:**
  [`Time`](motion_stack.core.utils.md#motion_stack.core.utils.time.Time)

#### sleep(seconds)

sleeps using the node’s clock.

#### NOTE
Special case for good old foxy

* **Parameters:**
  **seconds** (*float*) – time to sleep
* **Return type:**
  `None`

#### wait_on_futures(future_list, wait_Hz=10)

Waits for the completion of a list of futures, checking completion at the
provided rate.

* **Parameters:**
  * **future_list** (*List* *[**Future* *]*  *|* *Future*) – List of Future to wait for
  * **wait_Hz** (*float*) – rate at which to wait

#### error(\*args, force=False)

Prints/Logs error if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### warn(\*args, force=False)

Prints/Logs warning if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### info(\*args, force=False)

Prints/Logs info if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **force** (*bool*) – if True the message will print whatever if self.Yapping is.

#### debug(\*args, force=False)

Prints/Logs info if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **force** (*bool*) – if True the message will print whatever if self.Yapping is.

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
  * **cbk_grp** (*CallbackGroup* *|* *None*) – Not important I think but it’s there
  * **service_name** (*str*)
* **Return type:**
  *Client*

Returns:

* **Return type:**
  `Client`
* **Parameters:**
  * **service_name** (*str*)
  * **cbk_grp** (*CallbackGroup* *|* *None*)

#### create_EZrate(frequency, clock=None)

Creates a better rate where rate.destroy actually destroys the rate

* **Parameters:**
  * **frequency** (*float*) – frequency of the rate
  * **clock** (*Clock* *|* *None*) – clock to use
* **Returns:**
  EZRate manipulating a Rate object
* **Return type:**
  [`EZRate`](#motion_stack.ros2.utils.executor.EZRate)

#### execute_in_cbk_group(fun, callback_group=None)

Executes the given function by adding it as a callback to a callback_group.

#### NOTE
Pretty sure that’s not how it should be done.

* **Parameters:**
  * **fun** (*Callable*) – function to execute
  * **callback_group** (*CallbackGroup* *|* *None*) – callback group in which to execute the function
* **Returns:**
  holds the future results
  quardian: the guard condition object in the callback_group
* **Return type:**
  future
* **Return type:**
  `Tuple`[`Future`, `GuardCondition`]

### motion_stack.ros2.utils.executor.my_main(node, multi_threaded=False)

Main function used through the motion stack.

* **Parameters:**
  * **nodeClass** – Node to spin
  * **multiThreaded** – using multithreaded executor or not
  * **(****)** (*args*)
  * **multi_threaded** (*bool*)

## motion_stack.ros2.utils.files module

### motion_stack.ros2.utils.files.get_src_folder(package_name)

Absolute path to workspace/src/package_name.

#### NOTE
Meant for debugging. Avoid using this, you should build properly.

* **Parameters:**
  **package_name** (*str*) – workspace/src/package_name
* **Return type:**
  str

Returns: Absolute path as str

* **Return type:**
  `str`
* **Parameters:**
  **package_name** (*str*)

## motion_stack.ros2.utils.joint_state module

### motion_stack.ros2.utils.joint_state.link_subscription(node, topic_name, callback)

subscribes to a JointState topic, converts the message then calls the callback.

* **Parameters:**
  * **node** (*Node*) – node spinning
  * **topic_name** (*str*) – name of the JointState topic
  * **callback** (*Callable* *[* *[**List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]* *]* *,* *Any* *]*) – callback using not JointState but List[JState] and input

### motion_stack.ros2.utils.joint_state.ros2js_wrap(func)

* **Parameters:**
  * **callback** – function with List[JState] as the input
  * **func** (*Callable* *[* *[**List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]* *]* *,* *Any* *]*)
* **Returns:**
  function with JointState as the input
* **Return type:**
  `Callable`[[`JointState`], `None`]

### motion_stack.ros2.utils.joint_state.ros2js(jsin)

Converts JointState to a List[JState]

* **Return type:**
  `List`[[`JState`](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState)]
* **Parameters:**
  **jsin** (*JointState*)

### *class* motion_stack.ros2.utils.joint_state.JSCallableWrapper(original_callable)

Bases: `object`

* **Parameters:**
  **original_callable** (*Callable* *[* *[**JointState* *]* *,* *None* *]*)

### motion_stack.ros2.utils.joint_state.callable_js_publisher(node, topic_name, \*\*kwargs)

Creates a function publishing a JState on ROS2.

You can then call the function directly with a List[JState] when you wanna send something.

* **Parameters:**
  * **node** (*Node*) – node handling the publisher
  * **topic_name** (*str*) – publisher name
  * **\*\*kwargs** – kwargs for node.create_publisher
* **Returns:**
  A function, converting List[JState] to (several) JointState, then publishing
* **Return type:**
  `Callable`[[`List`[[`JState`](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState)]], `Any`]

### motion_stack.ros2.utils.joint_state.stateOrderinator3000(allStates)

Converts a list  of JState to multiple ros JointStates messages.
Timestamp ignored.

* **Return type:**
  `List`[`JointState`]
* **Parameters:**
  **allStates** (*Iterable* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

## motion_stack.ros2.utils.linking module

### *class* motion_stack.ros2.utils.linking.CallablePublisher(node, topic_type, topic_name, qos=10, \*args, \*\*kwargs)

Bases: `object`

* **Parameters:**
  * **node** (*Node*)
  * **topic_type** (*type*)
  * **topic_name** (*str*)
  * **qos** (*int*)

### motion_stack.ros2.utils.linking.link_startup_action(node, startup_callback, argument)

Creates a callback to be execute on the node’s startup given arguments.

* **Parameters:**
  * **node** (*Node*) – spinning node
  * **lvl1** – lvl1 core
  * **startup_callback** (*Callable*)
  * **argument** (*Any*)
* **Return type:**
  `Future`

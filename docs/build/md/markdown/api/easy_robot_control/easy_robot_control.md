# easy_robot_control package

## Subpackages

* [easy_robot_control.injection package](easy_robot_control.injection.md)
  * [Submodules](easy_robot_control.injection.md#submodules)
  * [easy_robot_control.injection.offsetter module](easy_robot_control.injection.md#module-easy_robot_control.injection.offsetter)
  * [easy_robot_control.injection.topic_pub module](easy_robot_control.injection.md#module-easy_robot_control.injection.topic_pub)
* [easy_robot_control.launch package](easy_robot_control.launch.md)
  * [Submodules](easy_robot_control.launch.md#submodules)
  * [easy_robot_control.launch.builder module](easy_robot_control.launch.md#module-easy_robot_control.launch.builder)
  * [easy_robot_control.launch.default_params module](easy_robot_control.launch.md#module-easy_robot_control.launch.default_params)
* [easy_robot_control.my_rtb_fix package](easy_robot_control.my_rtb_fix.md)
  * [Submodules](easy_robot_control.my_rtb_fix.md#submodules)
  * [easy_robot_control.my_rtb_fix.fixed_urdf module](easy_robot_control.my_rtb_fix.md#module-easy_robot_control.my_rtb_fix.fixed_urdf)
* [easy_robot_control.ros2_numpy package](easy_robot_control.ros2_numpy.md)
  * [Submodules](easy_robot_control.ros2_numpy.md#submodules)
  * [easy_robot_control.ros2_numpy.transformations module](easy_robot_control.ros2_numpy.md#module-easy_robot_control.ros2_numpy.transformations)
* [easy_robot_control.utils package](easy_robot_control.utils.md)
  * [Submodules](easy_robot_control.utils.md#submodules)
  * [easy_robot_control.utils.csv module](easy_robot_control.utils.md#module-easy_robot_control.utils.csv)
  * [easy_robot_control.utils.hero_mapping module](easy_robot_control.utils.md#module-easy_robot_control.utils.hero_mapping)
  * [easy_robot_control.utils.hyper_sphere_clamp module](easy_robot_control.utils.md#module-easy_robot_control.utils.hyper_sphere_clamp)
  * [easy_robot_control.utils.joint_state_util module](easy_robot_control.utils.md#module-easy_robot_control.utils.joint_state_util)
  * [easy_robot_control.utils.math module](easy_robot_control.utils.md#module-easy_robot_control.utils.math)
  * [easy_robot_control.utils.pure_remap module](easy_robot_control.utils.md#module-easy_robot_control.utils.pure_remap)
  * [easy_robot_control.utils.state_remaper module](easy_robot_control.utils.md#module-easy_robot_control.utils.state_remaper)
  * [easy_robot_control.utils.trajectories module](easy_robot_control.utils.md#module-easy_robot_control.utils.trajectories)

## Submodules

## easy_robot_control.EliaNode module

I this adds functionalities to the default ros2 Node object.
And slowly became a mess I have to cleanup …

* **Author:**
  Elian NEPPEL
* **Lab:**
  SRL, Moonshot team

### easy_robot_control.EliaNode.tf2np(tf)

converts a TF into a np array and quaternion

* **Parameters:**
  **tf** (*Transform*) – TF to convert
* **Returns:**
  xyz coordinates
  quat: quaternion for the rotation
* **Return type:**
  xyz
* **Return type:**
  `Tuple`[`NDArray`[`Shape`[`3`], `floating`], `quaternion`]

### easy_robot_control.EliaNode.np2tf(coord=None, quat=None, sendNone=False)

converts an NDArray and quaternion into a Transform.

* **Parameters:**
  * **coord** (*None* *|* *NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*  *|* *Sequence* *[**float* *]*) – xyz coordinates
  * **quat** (*quaternion* *|* *None*) – quaternion for the rotation
  * **sendNone** (*bool*)
* **Returns:**
  resulting TF
* **Return type:**
  tf
* **Return type:**
  `Transform`

### easy_robot_control.EliaNode.np2tfReq(coord=None, quat=None)

converts an NDArray and quaternion into a Transform request for a service.

* **Parameters:**
  * **xyz** – xyz coordinates
  * **quat** (*quaternion* *|* *None*) – quaternion for the rotation
  * **coord** (*ndarray* *|* *None*)
* **Returns:**
  Resulting Request for a service call
* **Return type:**
  `TFService_Request`

### easy_robot_control.EliaNode.np2TargetSet(arr=None)

Converts a target set message to np array

* **Return type:**
  `TargetSet`
* **Parameters:**
  **arr** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*  *|* *None*)

### easy_robot_control.EliaNode.targetSet2np(ts)

Converts a np array to target set message

* **Return type:**
  `NDArray`[`Shape`[`3`], `floating`]
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
  **func** (*Callable*) – Function executed by a callback
* **Returns:**
  warpped function

### easy_robot_control.EliaNode.rosTime2Float(time)

Converts ros2 time objects to seconds as float

* **Parameters:**
  **time** (*Time* *|* *Duration*) – ros time obj
* **Returns:**
  corresponding seconds as float value
* **Return type:**
  `float`

### easy_robot_control.EliaNode.list_cyanize(l, default_color=None)

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
  **package_name** (*str*) – workspace/src/package_name
* **Return type:**
  str

Returns: Absolute path as str

* **Return type:**
  `str`
* **Parameters:**
  **package_name** (*str*)

### easy_robot_control.EliaNode.transform_joint_to_transform_Rx(transform, jointET)

Takes a transform and a joint (TRANSFORM \* +-Rxyz), and returns the
(rotational) transform so that RESULT \* Rx = TRANSFORM \* +-Rxyz.
So the transform now places the x base vector onto the axis.

* **Parameters:**
  * **transform** (*ET*)
  * **jointET** (*ET*)
* **Returns:**
  RESULT \* Rx = TRANSFORM \* +-Rxyz
* **Return type:**
  `ET`

### easy_robot_control.EliaNode.loadAndSet_URDF(urdf_path, end_effector_name=None, start_effector_name=None)

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

### easy_robot_control.EliaNode.future_list_complete(future_list)

Returns True is all futures in the input list are done.

* **Parameters:**
  **future_list** (*List* *[**Future* *]*  *|* *Future*) – a list of futures
* **Returns:**
  True if all futures are done
* **Return type:**
  `bool`

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
  * **more_services** (*Iterable* *[**str* *]*) – add more services
  * **all_requiered** (*bool*) – if True, all listed services must be available

#### getNow()

quick: self.get_clock().now()

* **Return type:**
  `Time`

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

#### perror(object, force=False)

Prints/Logs error if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** (*Any*) – Thing to print
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
  [`EZRate`](#easy_robot_control.EliaNode.EZRate)

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

#### PARAM_REL_TOL *= 1e-06*

**Type:**    `float`

#### add_on_set_parameters_callback(callback)

Add a callback in front to the list of callbacks.

Calling this function will add a callback in self._parameter_callbacks list.

It is considered bad practice to reject changes for “unknown” parameters as this prevents
other parts of the node (that may be aware of these parameters) from handling them.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Return type:**
  `None`

#### add_waitable(waitable)

Add a class that is capable of adding things to the wait set.

* **Parameters:**
  **waitable** (`Waitable`) – An instance of a waitable that the node will add to the waitset.
* **Return type:**
  `None`

#### *property* clients

Get clients that have been created on this node.

* **Return type:**
  `Iterator`[`Client`]

#### *property* context

Get the context associated with the node.

* **Return type:**
  `Context`

#### count_publishers(topic_name)

Return the number of publishers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of publishers.
* **Return type:**
  `int`
* **Returns:**
  the number of publishers on the topic.

#### count_subscribers(topic_name)

Return the number of subscribers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of subscribers.
* **Return type:**
  `int`
* **Returns:**
  the number of subscribers on the topic.

#### create_client(srv_type, srv_name, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service client.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service client.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service client. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Client`

#### create_guard_condition(callback, callback_group=None)

Create a new guard condition.

* **Return type:**
  `GuardCondition`
* **Parameters:**
  * **callback** (*Callable*)
  * **callback_group** (*CallbackGroup* *|* *None*)

#### create_publisher(msg_type, topic, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, publisher_class=<class 'Publisher'>)

Create a new publisher.

* **Parameters:**
  * **msg_type** – The type of ROS messages the publisher will publish.
  * **topic** (`str`) – The name of the topic the publisher will publish to.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the publisher.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the publisher’s event handlers.
    If `None`, then the node’s default callback group is used.
  * **event_callbacks** (`Optional`[`PublisherEventCallbacks`]) – User-defined callbacks for middleware events.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
  * **publisher_class** (*Type* *[**Publisher* *]*)
* **Return type:**
  `Publisher`
* **Returns:**
  The new publisher.

#### create_rate(frequency, clock=None)

Create a Rate object.

* **Parameters:**
  * **frequency** (`float`) – The frequency the Rate runs at (Hz).
  * **clock** (`Optional`[`Clock`]) – The clock the Rate gets time from.
* **Return type:**
  `Rate`

#### create_service(srv_type, srv_name, callback, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service server.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **callback** (`Callable`[[`~SrvTypeRequest`, `~SrvTypeResponse`], `~SrvTypeResponse`]) – A user-defined callback function that is called when a service request
    received by the server.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service server.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service server. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Service`

#### create_subscription(msg_type, topic, callback, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, raw=False)

Create a new subscription.

* **Parameters:**
  * **msg_type** – The type of ROS messages the subscription will subscribe to.
  * **topic** (`str`) – The name of the topic the subscription will subscribe to.
  * **callback** (`Callable`[[`~MsgType`], `None`]) – A user-defined callback function that is called when a message is
    received by the subscription.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the subscription.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the subscription. If `None`, then the
    nodes default callback group is used.
  * **event_callbacks** (`Optional`[`SubscriptionEventCallbacks`]) – User-defined callbacks for middleware events.
  * **raw** (`bool`) – If `True`, then received messages will be stored in raw binary
    representation.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
* **Return type:**
  `Subscription`

#### create_timer(timer_period_sec, callback, callback_group=None, clock=None)

Create a new timer.

The timer will be started and every `timer_period_sec` number of seconds the provided
callback function will be called.

* **Parameters:**
  * **timer_period_sec** (`float`) – The period (s) of the timer.
  * **callback** (`Callable`) – A user-defined callback function that is called when the timer expires.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the timer. If `None`, then the nodes
    default callback group is used.
  * **clock** (`Optional`[`Clock`]) – The clock which the timer gets time from.
* **Return type:**
  `Timer`

#### declare_parameter(name, value=None, descriptor=None, ignore_override=False)

Declare and initialize a parameter.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.EliaNode.EliaNode.add_on_set_parameters_callback) to be called.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **value** (`Optional`[`Any`]) – Value of the parameter to declare.
  * **descriptor** (`Optional`[`ParameterDescriptor`]) – Descriptor for the parameter to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `Parameter`
* **Returns:**
  Parameter with the effectively assigned value.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects the parameter.

#### declare_parameters(namespace, parameters, ignore_override=False)

Declare a list of parameters.

The tuples in the given parameter list shall contain the name for each parameter,
optionally providing a value and a descriptor.
For each entry in the list, a parameter with a name of “namespace.name”
will be declared.
The resulting value for each declared parameter will be returned, considering
parameter overrides set upon node creation as the first choice,
or provided parameter values as the second one.

The name expansion is naive, so if you set the namespace to be “foo.”,
then the resulting parameter names will be like “foo..name”.
However, if the namespace is an empty string, then no leading ‘.’ will be
placed before each name, which would have been the case when naively
expanding “namespace.name”.
This allows you to declare several parameters at once without a namespace.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.EliaNode.EliaNode.add_on_set_parameters_callback) to be called once for each parameter.
If one of those calls fail, an exception will be raised and the remaining parameters will
not be declared.
Parameters declared up to that point will not be undeclared.

* **Parameters:**
  * **namespace** (`str`) – Namespace for parameters.
  * **parameters** (`List`[`Union`[`Tuple`[`str`], `Tuple`[`str`, `Type`], `Tuple`[`str`, `Any`, `ParameterDescriptor`]]]) – List of tuples with parameters to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  Parameter list with the effectively assigned values for each of them.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects any parameter.
* **Raises:**
  TypeError if any tuple in **parameters** does not match the annotated type.

#### *property* default_callback_group

Get the default callback group.

If no other callback group is provided when the a ROS entity is created with the node,
then it is added to the default callback group.

* **Return type:**
  `CallbackGroup`

#### describe_parameter(name)

Get the parameter descriptor of a given parameter.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `ParameterDescriptor`
* **Returns:**
  ParameterDescriptor corresponding to the parameter,
  or default ParameterDescriptor if parameter had not been declared before
  and undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.

#### describe_parameters(names)

Get the parameter descriptors of a given list of parameters.

* **Parameters:**
  * **name** – List of fully-qualified names of the parameters to describe.
  * **names** (*List* *[**str* *]*)
* **Return type:**
  `List`[`ParameterDescriptor`]
* **Returns:**
  List of ParameterDescriptors corresponding to the given parameters.
  Default ParameterDescriptors shall be returned for parameters that
  had not been declared before if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if at least one parameter
  had not been declared before and undeclared parameters are not allowed.

#### destroy_client(client)

Destroy a service client created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **client** (*Client*)

#### destroy_guard_condition(guard)

Destroy a guard condition created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **guard** (*GuardCondition*)

#### destroy_node()

Destroy the node.

Frees resources used by the node, including any entities created by the following methods:

* [`create_publisher()`](#easy_robot_control.EliaNode.EliaNode.create_publisher)
* [`create_subscription()`](#easy_robot_control.EliaNode.EliaNode.create_subscription)
* [`create_client()`](#easy_robot_control.EliaNode.EliaNode.create_client)
* [`create_service()`](#easy_robot_control.EliaNode.EliaNode.create_service)
* [`create_timer()`](#easy_robot_control.EliaNode.EliaNode.create_timer)
* [`create_guard_condition()`](#easy_robot_control.EliaNode.EliaNode.create_guard_condition)

#### destroy_publisher(publisher)

Destroy a publisher created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **publisher** (*Publisher*)

#### destroy_rate(rate)

Destroy a Rate object created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **rate** (*Rate*)

#### destroy_service(service)

Destroy a service server created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **service** (*Service*)

#### destroy_subscription(subscription)

Destroy a subscription created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if succesful, `False` otherwise.
* **Parameters:**
  **subscription** (*Subscription*)

#### destroy_timer(timer)

Destroy a timer created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **timer** (*Timer*)

#### *property* executor

Get the executor if the node has been added to one, else return `None`.

* **Return type:**
  `Optional`[`Executor`]

#### get_client_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service client topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get service clients for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The fist element of each tuple is the service client name
  and the second element is a list of service client types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_clock()

Get the clock used by the node.

* **Return type:**
  `Clock`

#### get_fully_qualified_name()

Get the node’s fully qualified name.

* **Return type:**
  `str`
* **Returns:**
  Fully qualified node name.

#### get_logger()

Get the nodes logger.

#### get_name()

Get the name of the node.

* **Return type:**
  `str`

#### get_namespace()

Get the namespace of the node.

* **Return type:**
  `str`

#### get_node_names()

Get a list of names for discovered nodes.

* **Return type:**
  `List`[`str`]
* **Returns:**
  List of node names.

#### get_node_names_and_namespaces()

Get a list of names and namespaces for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`]]
* **Returns:**
  List of tuples containing two strings: the node name and node namespace.

#### get_node_names_and_namespaces_with_enclaves()

Get a list of names, namespaces and enclaves for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`, `str`]]
* **Returns:**
  List of tuples containing three strings: the node name, node namespace
  and enclave.

#### get_parameter(name)

Get a parameter by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Parameter`
* **Returns:**
  The value for the given parameter name.
  A default Parameter will be returned for an undeclared parameter if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if the parameter is statically typed and
  uninitialized.

#### get_parameter_or(name, alternative_value=None)

Get a parameter or the alternative value.

If the alternative value is None, a default Parameter with the given name and NOT_SET
type will be returned if the parameter was not declared.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **alternative_value** (`Optional`[`Parameter`]) – Alternative parameter to get if it had not been declared before.
* **Return type:**
  `Parameter`
* **Returns:**
  Requested parameter, or alternative value if it hadn’t been declared before or is
  an uninitialized statically typed parameter.

#### get_parameter_type(name)

Get a parameter type by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Type`
* **Returns:**
  The type for the given parameter name.
  A default Parameter.Type.NOT_SET will be returned for an undeclared parameter
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.

#### get_parameter_types(names)

Get a list of parameter types.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Type`]
* **Returns:**
  The values for the given parameter types.
  A default Parameter.Type.NOT_SET will be returned for undeclared parameters
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.

#### get_parameters(names)

Get a list of parameters.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  The values for the given parameter names.
  A default Parameter will be returned for undeclared parameters if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if at least one parameter is statically typed and
  uninitialized.

#### get_parameters_by_prefix(prefix)

Get parameters that have a given prefix in their names as a dictionary.

The names which are used as keys in the returned dictionary have the prefix removed.
For example, if you use the prefix “foo” and the parameters “foo.ping”, “foo.pong”
and “bar.baz” exist, then the returned dictionary will have the keys “ping” and “pong”.
Note that the parameter separator is also removed from the parameter name to create the
keys.

An empty string for the prefix will match all parameters.

If no parameters with the prefix are found, an empty dictionary will be returned.

* **Parameters:**
  **prefix** (`str`) – The prefix of the parameters to get.
* **Return type:**
  `Dict`[`str`, `Union`[`bool`, `int`, `float`, `str`, `bytes`, `Sequence`[`bool`], `Sequence`[`int`], `Sequence`[`float`], `Sequence`[`str`], `None`]]
* **Returns:**
  Dict of parameters with the given prefix.

#### get_publisher_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for publishers of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get publishers for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_publishers_info_by_topic(topic_name, no_mangle=False)

Return a list of publishers on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the publishers.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the publishers on this topic.

#### get_service_names_and_types()

Get a list of service topics for the node.

* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service name and the second element is a list of
  service types.

#### get_service_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service server topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get services for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service server name
  and the second element is a list of service types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriber_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for subscriptions of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get subscriptions for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriptions_info_by_topic(topic_name, no_mangle=False)

Return a list of subscriptions on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the subscriptions.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the subscriptions on this topic.

#### get_topic_names_and_types(no_demangle=False)

Get a list topic names and types for the node.

* **Parameters:**
  **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.

#### *property* guards

Get guards that have been created on this node.

* **Return type:**
  `Iterator`[`GuardCondition`]

#### *property* handle

Get the handle to the underlying rcl_node_t.

Cannot be modified after node creation.

* **Raises:**
  AttributeError if modified after creation.

#### has_parameter(name)

Return True if parameter is declared; False otherwise.

* **Return type:**
  `bool`
* **Parameters:**
  **name** (*str*)

#### *property* publishers

Get publishers that have been created on this node.

* **Return type:**
  `Iterator`[`Publisher`]

#### remove_on_set_parameters_callback(callback)

Remove a callback from list of callbacks.

Calling this function will remove the callback from self._parameter_callbacks list.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Raises:**
  ValueError if a callback is not present in the list of callbacks.
* **Return type:**
  `None`

#### remove_waitable(waitable)

Remove a Waitable that was previously added to the node.

* **Parameters:**
  **waitable** (`Waitable`) – The Waitable to remove.
* **Return type:**
  `None`

#### resolve_topic_name(topic, \*, only_expand=False)

Return a topic name expanded and remapped.

* **Parameters:**
  * **topic** (`str`) – topic name to be expanded and remapped.
  * **only_expand** (`bool`) – if True, remapping rules won’t be applied.
* **Return type:**
  `str`
* **Returns:**
  a fully qualified topic name,
  result of applying expansion and remapping to the given topic.

#### *property* services

Get services that have been created on this node.

* **Return type:**
  `Iterator`[`Service`]

#### set_descriptor(name, descriptor, alternative_value=None)

Set a new descriptor for a given parameter.

The name in the descriptor is ignored and set to **name**.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter to set the descriptor to.
  * **descriptor** (`ParameterDescriptor`) – New descriptor to apply to the parameter.
  * **alternative_value** (`Optional`[`ParameterValue`]) – Value to set to the parameter if the existing value does not
    comply with the new descriptor.
* **Return type:**
  `ParameterValue`
* **Returns:**
  ParameterValue for the given parameter name after applying the new descriptor.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.
* **Raises:**
  ParameterImmutableException if the parameter exists and is read-only.
* **Raises:**
  ParameterValueException if neither the existing value nor the alternative value
  complies with the provided descriptor.

#### set_parameters(parameter_list)

Set parameters for the node, and return the result for the set action.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set in the order they are declared in the list.
If setting a parameter fails due to not being declared, then the
parameters which have already been set will stay set, and no attempt will
be made to set the parameters which come after.

If undeclared parameters are allowed, then all the parameters will be implicitly
declared before being set even if they were not declared beforehand.
Parameter overrides are ignored by this method.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.EliaNode.EliaNode.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node, once for each parameter.
If the callback prevents a parameter from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is
published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `List`[`SetParametersResult`]
* **Returns:**
  The result for each set action as a list.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### set_parameters_atomically(parameter_list)

Set the given parameters, all at one time, and then aggregate result.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set all at once.
If setting a parameter fails due to not being declared, then no parameter will be set set.
Either all of the parameters are set or none of them are set.

If undeclared parameters are allowed for the node, then all the parameters will be
implicitly declared before being set even if they were not declared beforehand.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.EliaNode.EliaNode.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node only once for all parameters.
If the callback prevents the parameters from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `SetParametersResult`
* **Returns:**
  Aggregate result of setting all the parameters atomically.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### *property* subscriptions

Get subscriptions that have been created on this node.

* **Return type:**
  `Iterator`[`Subscription`]

#### *property* timers

Get timers that have been created on this node.

* **Return type:**
  `Iterator`[`Timer`]

#### undeclare_parameter(name)

Undeclare a previously declared parameter.

This method will not cause a callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.EliaNode.EliaNode.add_on_set_parameters_callback) to be called.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before.
* **Raises:**
  ParameterImmutableException if the parameter was created as read-only.

#### *property* waitables

Get waitables that have been created on this node.

* **Return type:**
  `Iterator`[`Waitable`]

### easy_robot_control.EliaNode.myMain(nodeClass, multiThreaded=False, args=None)

Main function used through the motion stack.

* **Parameters:**
  * **nodeClass** – Node to spin
  * **multiThreaded** (*bool*) – using multithreaded executor or not
  * **(****)** (*args*)

## easy_robot_control.gait_key_dev module

This node is responsible for controlling movement of Moonbot HERO.
For now keyboard and controller (PS4).

Authors: Elian NEPPEL, Shamistan KARIMOV
Lab: SRL, Moonshot team

### easy_robot_control.gait_key_dev.float_formatter()

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

### *class* easy_robot_control.gait_key_dev.Leg(number, parent)

Bases: [`Leg`](#easy_robot_control.leg_api.Leg)

* **Parameters:**
  * **number** (*int*)
  * **parent** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))

#### recover()

* **Return type:**
  `Future`

#### halt()

* **Return type:**
  `Future`

#### add_joints(all_joints)

* **Return type:**
  `List`[[`JointMini`](#easy_robot_control.leg_api.JointMini)]
* **Parameters:**
  **all_joints** (*List* *[**str* *]*)

#### connect_movement_clients()

#### *static* do_i_exist(number, parent, timeout=1)

Slow do not use to spam scan.
Returns True if the leg is alive

* **Parameters:**
  * **number** (*int*)
  * **parent** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))
  * **timeout** (*float*)

#### get_angle(joint)

Gets an angle from a joint

* **Parameters:**
  **joint** (*int* *|* *str*) – joint name or number (alphabetically ordered)
* **Returns:**
  last recieved angle as float
* **Return type:**
  `Optional`[`float`]

#### get_joint_obj(joint)

Gets the corresponding joint object is exists

* **Parameters:**
  **joint** (*int* *|* *str*) – joint name or number (alphabetically ordered)
* **Return type:**
  `Optional`[[`JointMini`](#easy_robot_control.leg_api.JointMini)]

#### go2zero()

sends angle target of 0 on all joints

#### ik(xyz=None, quat=None)

Publishes an ik target for the leg () relative to baselink. Motion stack lvl2

* **Parameters:**
  * **xyz** (*None* *|* *NDArray* *[**Any* *,* *Any* *]*  *|* *Sequence* *[**float* *]*)
  * **quat** (*quaternion* *|* *None*)
* **Return type:**
  `None`

#### look_for_joints()

scans and updates the list of joints of this leg

#### move(xyz=None, quat=None, mvt_type='shift', blocking=True)

Calls the leg’s movement service. Motion stack lvl3

* **Parameters:**
  * **xyz** (*None* *|* *NDArray* *[**Any* *,* *Any* *]*  *|* *Sequence* *[**float* *]*) – vector part of the tf
  * **quat** (*quaternion* *|* *None*) – quat of the tf
  * **mvt_type** (*Literal* *[* *'shift'* *,*  *'transl'* *,*  *'rot'* *,*  *'hop'* *]*) – type of movement to call
  * **blocking** (*bool*) – if false returns a Future. Else returns the response
* **Return type:**
  *Future* | *TFService_Response*

Returns:

* **Return type:**
  `Union`[`Future`, `TFService_Response`]
* **Parameters:**
  * **xyz** (*None* *|* *NDArray* *[**Any* *,* *Any* *]*  *|* *Sequence* *[**float* *]*)
  * **quat** (*quaternion* *|* *None*)
  * **mvt_type** (*Literal* *[* *'shift'* *,*  *'transl'* *,*  *'rot'* *,*  *'hop'* *]*)
  * **blocking** (*bool*)

#### self_report()

* **Return type:**
  `str`

#### set_angle(angle, joint)

Sends a angle to a joint

* **Parameters:**
  * **angle** (*float*) – rad
  * **joint** (*int* *|* *str*) – joint name or number (alphabetically ordered)
* **Returns:**
  True if message sent
* **Return type:**
  `bool`

### *class* easy_robot_control.gait_key_dev.Wheel(number, parent)

Bases: [`Leg`](#easy_robot_control.gait_key_dev.Leg)

* **Parameters:**
  * **number** (*int*)
  * **parent** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))

#### add_joints(all_joints)

* **Return type:**
  `List`[[`JointMini`](#easy_robot_control.leg_api.JointMini)]
* **Parameters:**
  **all_joints** (*List* *[**str* *]*)

#### connect_movement_clients()

#### forward()

#### backward()

#### stop()

#### *static* do_i_exist(number, parent, timeout=1)

Slow do not use to spam scan.
Returns True if the leg is alive

* **Parameters:**
  * **number** (*int*)
  * **parent** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))
  * **timeout** (*float*)

#### get_angle(joint)

Gets an angle from a joint

* **Parameters:**
  **joint** (*int* *|* *str*) – joint name or number (alphabetically ordered)
* **Returns:**
  last recieved angle as float
* **Return type:**
  `Optional`[`float`]

#### get_joint_obj(joint)

Gets the corresponding joint object is exists

* **Parameters:**
  **joint** (*int* *|* *str*) – joint name or number (alphabetically ordered)
* **Return type:**
  `Optional`[[`JointMini`](#easy_robot_control.leg_api.JointMini)]

#### go2zero()

sends angle target of 0 on all joints

#### halt()

* **Return type:**
  `Future`

#### ik(xyz=None, quat=None)

Publishes an ik target for the leg () relative to baselink. Motion stack lvl2

* **Parameters:**
  * **xyz** (*None* *|* *NDArray* *[**Any* *,* *Any* *]*  *|* *Sequence* *[**float* *]*)
  * **quat** (*quaternion* *|* *None*)
* **Return type:**
  `None`

#### look_for_joints()

scans and updates the list of joints of this leg

#### move(xyz=None, quat=None, mvt_type='shift', blocking=True)

Calls the leg’s movement service. Motion stack lvl3

* **Parameters:**
  * **xyz** (*None* *|* *NDArray* *[**Any* *,* *Any* *]*  *|* *Sequence* *[**float* *]*) – vector part of the tf
  * **quat** (*quaternion* *|* *None*) – quat of the tf
  * **mvt_type** (*Literal* *[* *'shift'* *,*  *'transl'* *,*  *'rot'* *,*  *'hop'* *]*) – type of movement to call
  * **blocking** (*bool*) – if false returns a Future. Else returns the response
* **Return type:**
  *Future* | *TFService_Response*

Returns:

* **Return type:**
  `Union`[`Future`, `TFService_Response`]
* **Parameters:**
  * **xyz** (*None* *|* *NDArray* *[**Any* *,* *Any* *]*  *|* *Sequence* *[**float* *]*)
  * **quat** (*quaternion* *|* *None*)
  * **mvt_type** (*Literal* *[* *'shift'* *,*  *'transl'* *,*  *'rot'* *,*  *'hop'* *]*)
  * **blocking** (*bool*)

#### recover()

* **Return type:**
  `Future`

#### self_report()

* **Return type:**
  `str`

#### set_angle(angle, joint)

Sends a angle to a joint

* **Parameters:**
  * **angle** (*float*) – rad
  * **joint** (*int* *|* *str*) – joint name or number (alphabetically ordered)
* **Returns:**
  True if message sent
* **Return type:**
  `bool`

### *class* easy_robot_control.gait_key_dev.KeyGaitNode(name='keygait_node')

Bases: [`EliaNode`](#easy_robot_control.EliaNode.EliaNode)

* **Parameters:**
  **name** (*str*)

#### makeTBclient()

#### leg_scanTMRCBK()

Looks for new legs and joints

#### wheel_scanTMRCBK()

Looks for new legs and joints

#### refresh_joint_mapping()

joint mapping based on leg number (realguy or MoonbotH)

#### switch_to_grip_ur16()

joint mapping based on leg number (realguy or MoonbotH)

#### key_upSUBCBK(msg)

Executes when keyboard released

* **Parameters:**
  **msg** (*Key*)

#### stop_all_joints()

stops all joint by sending the current angle as target.
if speed was set, sends a speed of 0 instead

#### all_wheel_speed(speed)

Need a re-work

#### minimal_wheel_speed(speed)

Need a re-work

#### tricycle_wheel_speed(speed)

Need a re-work

#### dragon_wheel_speed(speed)

Need a re-work

#### wheels_speed(wheels, speed)

Need a re-work

#### key_downSUBCBK(msg)

Executes when keyboard pressed

* **Parameters:**
  **msg** (*Key*)

#### default_3legs()

#### align_with(leg_number)

* **Parameters:**
  **leg_number** (*int*)

#### default_vehicle()

#### default_dragon()

#### dragon_align()

#### dragon_front_left()

#### dragon_front_right()

#### dragon_base_lookup()

#### dragon_base_lookdown()

#### dragon_back_left()

#### dragon_back_right()

#### zero_without_grippers()

#### goToTargetBody(ts: numpy.ndarray[Any, numpy.dtype[numpy._typing._array_like._ScalarType_co]] | None = None, bodyXYZ: numpy.ndarray[Any, numpy.dtype[numpy._typing._array_like._ScalarType_co]] | None = None, bodyQuat: quaternion.quaternion | None = None, blocking: Literal[True] = True) → rclpy.task.Future

#### goToTargetBody(ts: numpy.ndarray[Any, numpy.dtype[numpy._typing._array_like._ScalarType_co]] | None = None, bodyXYZ: numpy.ndarray[Any, numpy.dtype[numpy._typing._array_like._ScalarType_co]] | None = None, bodyQuat: quaternion.quaternion | None = None, blocking: Literal[False] = False) → motion_stack_msgs.srv._send_target_body.SendTargetBody_Response

#### goToTargetBody(ts=None, bodyXYZ=None, bodyQuat=None, blocking=True)

* **Return type:**
  `Union`[`Future`, `SendTargetBody_Response`]
* **Parameters:**
  * **ts** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
  * **bodyXYZ** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
  * **bodyQuat** (*quaternion* *|* *None*)
  * **blocking** (*bool*)

#### euler_to_quaternion(roll, pitch, yaw)

Convert Euler angles to a quaternion.

* **Parameters:**
  * **roll** (*float*) – Rotation around the X-axis in radians.
  * **pitch** (*float*) – Rotation around the Y-axis in radians.
  * **yaw** (*float*) – Rotation around the Z-axis in radians.
* **Returns:**
  The resulting quaternion.
* **Return type:**
  Quaternion
* **Return type:**
  `Optional`[`quaternion`]

#### msg_to_JoyBits(msg)

Converts a joy msg to a JoyState

* **Return type:**
  `JoyState`
* **Parameters:**
  **msg** (*Joy*)

#### display_JoyBits(joy_state)

* **Parameters:**
  **joy_state** (*JoyState*)

#### any_pressed(bits, button_names)

Checks if any button in the list is pressed.

* **Parameters:**
  * **bits** (*int*) – set of joybits to check against
  * **button_names** (*List* *[**Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]* *]*  *|*  *~typing.Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]*) – list of button names to check if True
* **Returns:**
  True if any bit corresponding to a button is True.
* **Return type:**
  `bool`

#### bits2name(bits)

Converts a bit field to a list of button names

* **Return type:**
  `List`[`Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`]]
* **Parameters:**
  **bits** (*int*)

#### one_bit2name(bits)

Converts a bit field with 1 bit to 1, to a single button name

* **Return type:**
  `Optional`[`Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`]]
* **Parameters:**
  **bits** (*int*)

#### joy_pressed(button_name)

Executes for each button that is pressed. Like a callback.

* **Parameters:**
  * **bits** – Should only have one single bit set to 1, for 1 single button
  * **button_name** (*Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]*)

#### joy_released(button_name)

Executes for each button that is released. Like a callback.

* **Parameters:**
  * **bits** – Should only have one single bit set to 1, for 1 single button
  * **button_name** (*Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]*)

#### get_joint_index(selected_joint)

* **Return type:**
  `Optional`[`int`]
* **Parameters:**
  **selected_joint** (*int*)

#### joySUBCBK(msg)

Processes incomming joy messages.
Converts and stores the received state in self.joy_state .
executes self.joy_pressed and self.joy_released for each button that changed state

* **Parameters:**
  **msg** (*Joy*) – Ros2 Joy message type

#### joint_control_joy()

#### joint_timer_start()

#### *static* collapseT_KeyCodeModifier(variable)

Collapses the variable onto a KeyCodeModifier type, or None

* **Returns:**
  None if variable is not a KCM
  The variable as a KCM type-hint if it is a KCM
* **Return type:**
  `Optional`[`Tuple`[`int`, `Union`[`int`, `Literal`[`'ANY'`]]]]
* **Parameters:**
  **variable** (*Any*)

#### *static* collapseT_JoyCodeModifier(variable)

Collapses the variable onto a JoyCodeModifier type, or None

* **Returns:**
  None if variable is not a JCM
  The variable as a JCM type-hint if it is a JCM
* **Return type:**
  `Optional`[`Tuple`[`Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`], `Union`[`int`, `Literal`[`'ANY'`]]]]
* **Parameters:**
  **variable** (*Any*)

#### *static* remap_onto_any(mapping, input)

runs the input through the INPUTMap as if the key_modifier was any
if it is already, it does not run it.

* **Parameters:**
  * **mapping** (*Dict* *[**Tuple* *[**int* *,* *int* *|* *Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Tuple* *[* *~typing.Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]* *,* *int* *|*  *~typing.Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Literal* *[* *'ALWAYS'* *]* *,*  *~typing.List* *[* *~typing.Callable* *[* *[* *]* *,*  *~typing.Any* *]* *]* *]*)
  * **input** (*Tuple* *[**int* *,* *int* *|* *Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Tuple* *[* *~typing.Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]* *,* *int* *|*  *~typing.Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Literal* *[* *'ALWAYS'* *]*)

#### *static* connect_mapping(mapping, input)

Given the user input, executes the corresponding function mapping

* **Parameters:**
  * **mapping** (*Dict* *[**Tuple* *[**int* *,* *int* *|* *Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Tuple* *[* *~typing.Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]* *,* *int* *|*  *~typing.Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Literal* *[* *'ALWAYS'* *]* *,*  *~typing.List* *[* *~typing.Callable* *[* *[* *]* *,*  *~typing.Any* *]* *]* *]*) – Dict of function to execute
  * **input** (*Tuple* *[**int* *,* *int* *|* *Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Tuple* *[* *~typing.Literal* *[* *'NONE'* *,*  *'x'* *,*  *'o'* *,*  *'t'* *,*  *'s'* *,*  *'L1'* *,*  *'R1'* *,*  *'L2'* *,*  *'R2'* *,*  *'share'* *,*  *'option'* *,*  *'PS'* *,*  *'stickLpush'* *,*  *'stickRpush'* *,*  *'down'* *,*  *'right'* *,*  *'up'* *,*  *'left'* *,*  *'stickL'* *,*  *'stickR'* *]* *,* *int* *|*  *~typing.Literal* *[* *'ANY'* *]* *]*  *|*  *~typing.Literal* *[* *'ALWAYS'* *]*) – key to the entry to execute

#### select_leg(leg_ind)

Selects the leg(s) for operation. If None select all.

* **Parameters:**
  **leg_ind** (*List* *[**int* *]*  *|* *None*) – List of leg keys (leg numbers) to use

#### cycle_leg_selection(increment)

Cycles the leg selection by increment
if None, selects all known legs

* **Parameters:**
  **increment** (*int* *|* *None*)

#### get_active_leg(leg_key=None)

Return the keys to get the current active legs from the self.legs dict

* **Parameters:**
  * **leg_number** – you can specify a leg key if you need instead of using active legs
  * **leg_key** (*List* *[**int* *]*  *|* *int* *|* *None*)
* **Returns:**
  list of active leg keys
* **Return type:**
  `List`[[`Leg`](#easy_robot_control.gait_key_dev.Leg)]

#### get_active_leg_keys(leg_key=None)

Return the keys to get the current active legs from the self.legs dict

* **Parameters:**
  * **leg_number** – you can specify a leg key if you need instead of using active legs
  * **leg_key** (*List* *[**int* *]*  *|* *int* *|* *None*)
* **Returns:**
  list of active leg keys
* **Return type:**
  `List`[`int`]

#### halt_all()

#### halt_detected()

#### recover_all(leg_keys=None)

* **Parameters:**
  **leg_keys** (*List* *[**int* *]*  *|* *int* *|* *None*)

#### recover_legs(leg_keys=None)

* **Parameters:**
  **leg_keys** (*List* *[**int* *]*  *|* *int* *|* *None*)

#### set_joint_speed(speed, joint=None, leg_number=None)

Sets joint speed or given joints and legs.
If Nones, picks the selected or active things

* **Parameters:**
  * **speed** (*float*)
  * **joint** (*str* *|* *int* *|* *None*)
  * **leg_number** (*int* *|* *None*)

#### start_ik2_timer()

properly checks and start the timer loop for ik of lvl2

#### ik2TMRCBK()

Timer callback responsable for fast ik movement of lvl2

#### send_ik2_offset(xyz=None, quat=None, ee_relative=False)

* **Parameters:**
  * **xyz** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
  * **quat** (*quaternion* *|* *None*)
  * **ee_relative** (*bool*)

#### send_ik2_movement(xyz=None, quat=None, ee_relative=False)

debug

* **Parameters:**
  * **xyz** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
  * **quat** (*quaternion* *|* *None*)
  * **ee_relative** (*bool*)

#### select_joint(joint_index)

can be better

#### angle_zero(leg_number=None)

Sets all joint angles to 0 (dangerous)

* **Parameters:**
  **leg_number** (*List* *[**int* *]*  *|* *int* *|* *None*) – The leg on which to set. If none, applies on the active leg

#### enter_vehicle_mode()

Creates the sub input map for vehicle

* **Returns:**
  InputMap for joint control
* **Return type:**
  `None`

#### enter_dragon_mode()

Creates the sub input map for dragon

* **Returns:**
  InputMap for joint control
* **Return type:**
  `None`

#### enter_tricycle_mode()

Creates the sub input map for tricycle

* **Returns:**
  InputMap for joint control
* **Return type:**
  `None`

#### enter_leg_mode()

Creates the sub input map for leg selection

* **Returns:**
  InputMap for leg selection
* **Return type:**
  `None`

#### ik2_switch_rel_mode(val=None)

* **Parameters:**
  **val** (*bool* *|* *None*)

#### inch()

#### enter_ik2()

Creates the sub input map for ik control lvl2 by elian

* **Returns:**
  InputMap for ik2 control
* **Return type:**
  `None`

#### enter_joint_mode()

Creates the sub input map for joint control

* **Returns:**
  InputMap for joint control
* **Return type:**
  `None`

#### no_no_leg()

Makes sure no legs are not selected

#### enter_select_mode()

Mode to select other modes.
Should always be accessible when pressing ESC key

#### easy_mode()

#### PARAM_REL_TOL *= 1e-06*

**Type:**    `float`

#### add_on_set_parameters_callback(callback)

Add a callback in front to the list of callbacks.

Calling this function will add a callback in self._parameter_callbacks list.

It is considered bad practice to reject changes for “unknown” parameters as this prevents
other parts of the node (that may be aware of these parameters) from handling them.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Return type:**
  `None`

#### add_waitable(waitable)

Add a class that is capable of adding things to the wait set.

* **Parameters:**
  **waitable** (`Waitable`) – An instance of a waitable that the node will add to the waitset.
* **Return type:**
  `None`

#### *property* clients

Get clients that have been created on this node.

* **Return type:**
  `Iterator`[`Client`]

#### *property* context

Get the context associated with the node.

* **Return type:**
  `Context`

#### count_publishers(topic_name)

Return the number of publishers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of publishers.
* **Return type:**
  `int`
* **Returns:**
  the number of publishers on the topic.

#### count_subscribers(topic_name)

Return the number of subscribers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of subscribers.
* **Return type:**
  `int`
* **Returns:**
  the number of subscribers on the topic.

#### create_EZrate(frequency, clock=None)

Creates a better rate where rate.destroy actually destroys the rate

* **Parameters:**
  * **frequency** (*float*) – frequency of the rate
  * **clock** (*Clock* *|* *None*) – clock to use
* **Returns:**
  EZRate manipulating a Rate object
* **Return type:**
  [`EZRate`](#easy_robot_control.EliaNode.EZRate)

#### create_client(srv_type, srv_name, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service client.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service client.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service client. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Client`

#### create_guard_condition(callback, callback_group=None)

Create a new guard condition.

* **Return type:**
  `GuardCondition`
* **Parameters:**
  * **callback** (*Callable*)
  * **callback_group** (*CallbackGroup* *|* *None*)

#### create_main_map()

Creates the main input map, mapping user input to functions,
This is supposed to be constant + always active, unlike the sub_map

* **Return type:**
  `Dict`[`Union`[`Tuple`[`int`, `Union`[`int`, `Literal`[`'ANY'`]]], `Tuple`[`Literal`[`'NONE'`, `'x'`, `'o'`, `'t'`, `'s'`, `'L1'`, `'R1'`, `'L2'`, `'R2'`, `'share'`, `'option'`, `'PS'`, `'stickLpush'`, `'stickRpush'`, `'down'`, `'right'`, `'up'`, `'left'`, `'stickL'`, `'stickR'`], `Union`[`int`, `Literal`[`'ANY'`]]], `Literal`[`'ALWAYS'`]], `List`[`Callable`[[], `Any`]]]

#### create_publisher(msg_type, topic, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, publisher_class=<class 'Publisher'>)

Create a new publisher.

* **Parameters:**
  * **msg_type** – The type of ROS messages the publisher will publish.
  * **topic** (`str`) – The name of the topic the publisher will publish to.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the publisher.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the publisher’s event handlers.
    If `None`, then the node’s default callback group is used.
  * **event_callbacks** (`Optional`[`PublisherEventCallbacks`]) – User-defined callbacks for middleware events.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
  * **publisher_class** (*Type* *[**Publisher* *]*)
* **Return type:**
  `Publisher`
* **Returns:**
  The new publisher.

#### create_rate(frequency, clock=None)

Create a Rate object.

* **Parameters:**
  * **frequency** (`float`) – The frequency the Rate runs at (Hz).
  * **clock** (`Optional`[`Clock`]) – The clock the Rate gets time from.
* **Return type:**
  `Rate`

#### create_service(srv_type, srv_name, callback, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service server.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **callback** (`Callable`[[`~SrvTypeRequest`, `~SrvTypeResponse`], `~SrvTypeResponse`]) – A user-defined callback function that is called when a service request
    received by the server.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service server.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service server. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Service`

#### create_subscription(msg_type, topic, callback, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, raw=False)

Create a new subscription.

* **Parameters:**
  * **msg_type** – The type of ROS messages the subscription will subscribe to.
  * **topic** (`str`) – The name of the topic the subscription will subscribe to.
  * **callback** (`Callable`[[`~MsgType`], `None`]) – A user-defined callback function that is called when a message is
    received by the subscription.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the subscription.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the subscription. If `None`, then the
    nodes default callback group is used.
  * **event_callbacks** (`Optional`[`SubscriptionEventCallbacks`]) – User-defined callbacks for middleware events.
  * **raw** (`bool`) – If `True`, then received messages will be stored in raw binary
    representation.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
* **Return type:**
  `Subscription`

#### create_timer(timer_period_sec, callback, callback_group=None, clock=None)

Create a new timer.

The timer will be started and every `timer_period_sec` number of seconds the provided
callback function will be called.

* **Parameters:**
  * **timer_period_sec** (`float`) – The period (s) of the timer.
  * **callback** (`Callable`) – A user-defined callback function that is called when the timer expires.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the timer. If `None`, then the nodes
    default callback group is used.
  * **clock** (`Optional`[`Clock`]) – The clock which the timer gets time from.
* **Return type:**
  `Timer`

#### declare_parameter(name, value=None, descriptor=None, ignore_override=False)

Declare and initialize a parameter.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.gait_key_dev.KeyGaitNode.add_on_set_parameters_callback) to be called.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **value** (`Optional`[`Any`]) – Value of the parameter to declare.
  * **descriptor** (`Optional`[`ParameterDescriptor`]) – Descriptor for the parameter to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `Parameter`
* **Returns:**
  Parameter with the effectively assigned value.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects the parameter.

#### declare_parameters(namespace, parameters, ignore_override=False)

Declare a list of parameters.

The tuples in the given parameter list shall contain the name for each parameter,
optionally providing a value and a descriptor.
For each entry in the list, a parameter with a name of “namespace.name”
will be declared.
The resulting value for each declared parameter will be returned, considering
parameter overrides set upon node creation as the first choice,
or provided parameter values as the second one.

The name expansion is naive, so if you set the namespace to be “foo.”,
then the resulting parameter names will be like “foo..name”.
However, if the namespace is an empty string, then no leading ‘.’ will be
placed before each name, which would have been the case when naively
expanding “namespace.name”.
This allows you to declare several parameters at once without a namespace.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.gait_key_dev.KeyGaitNode.add_on_set_parameters_callback) to be called once for each parameter.
If one of those calls fail, an exception will be raised and the remaining parameters will
not be declared.
Parameters declared up to that point will not be undeclared.

* **Parameters:**
  * **namespace** (`str`) – Namespace for parameters.
  * **parameters** (`List`[`Union`[`Tuple`[`str`], `Tuple`[`str`, `Type`], `Tuple`[`str`, `Any`, `ParameterDescriptor`]]]) – List of tuples with parameters to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  Parameter list with the effectively assigned values for each of them.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects any parameter.
* **Raises:**
  TypeError if any tuple in **parameters** does not match the annotated type.

#### *property* default_callback_group

Get the default callback group.

If no other callback group is provided when the a ROS entity is created with the node,
then it is added to the default callback group.

* **Return type:**
  `CallbackGroup`

#### describe_parameter(name)

Get the parameter descriptor of a given parameter.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `ParameterDescriptor`
* **Returns:**
  ParameterDescriptor corresponding to the parameter,
  or default ParameterDescriptor if parameter had not been declared before
  and undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.

#### describe_parameters(names)

Get the parameter descriptors of a given list of parameters.

* **Parameters:**
  * **name** – List of fully-qualified names of the parameters to describe.
  * **names** (*List* *[**str* *]*)
* **Return type:**
  `List`[`ParameterDescriptor`]
* **Returns:**
  List of ParameterDescriptors corresponding to the given parameters.
  Default ParameterDescriptors shall be returned for parameters that
  had not been declared before if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if at least one parameter
  had not been declared before and undeclared parameters are not allowed.

#### destroy_client(client)

Destroy a service client created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **client** (*Client*)

#### destroy_guard_condition(guard)

Destroy a guard condition created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **guard** (*GuardCondition*)

#### destroy_node()

Destroy the node.

Frees resources used by the node, including any entities created by the following methods:

* [`create_publisher()`](#easy_robot_control.gait_key_dev.KeyGaitNode.create_publisher)
* [`create_subscription()`](#easy_robot_control.gait_key_dev.KeyGaitNode.create_subscription)
* [`create_client()`](#easy_robot_control.gait_key_dev.KeyGaitNode.create_client)
* [`create_service()`](#easy_robot_control.gait_key_dev.KeyGaitNode.create_service)
* [`create_timer()`](#easy_robot_control.gait_key_dev.KeyGaitNode.create_timer)
* [`create_guard_condition()`](#easy_robot_control.gait_key_dev.KeyGaitNode.create_guard_condition)

#### destroy_publisher(publisher)

Destroy a publisher created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **publisher** (*Publisher*)

#### destroy_rate(rate)

Destroy a Rate object created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **rate** (*Rate*)

#### destroy_service(service)

Destroy a service server created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **service** (*Service*)

#### destroy_subscription(subscription)

Destroy a subscription created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if succesful, `False` otherwise.
* **Parameters:**
  **subscription** (*Subscription*)

#### destroy_timer(timer)

Destroy a timer created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **timer** (*Timer*)

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

#### *property* executor

Get the executor if the node has been added to one, else return `None`.

* **Return type:**
  `Optional`[`Executor`]

#### getNow()

quick: self.get_clock().now()

* **Return type:**
  `Time`

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

#### get_client_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service client topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get service clients for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The fist element of each tuple is the service client name
  and the second element is a list of service client types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_clock()

Get the clock used by the node.

* **Return type:**
  `Clock`

#### get_fully_qualified_name()

Get the node’s fully qualified name.

* **Return type:**
  `str`
* **Returns:**
  Fully qualified node name.

#### get_logger()

Get the nodes logger.

#### get_name()

Get the name of the node.

* **Return type:**
  `str`

#### get_namespace()

Get the namespace of the node.

* **Return type:**
  `str`

#### get_node_names()

Get a list of names for discovered nodes.

* **Return type:**
  `List`[`str`]
* **Returns:**
  List of node names.

#### get_node_names_and_namespaces()

Get a list of names and namespaces for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`]]
* **Returns:**
  List of tuples containing two strings: the node name and node namespace.

#### get_node_names_and_namespaces_with_enclaves()

Get a list of names, namespaces and enclaves for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`, `str`]]
* **Returns:**
  List of tuples containing three strings: the node name, node namespace
  and enclave.

#### get_parameter(name)

Get a parameter by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Parameter`
* **Returns:**
  The value for the given parameter name.
  A default Parameter will be returned for an undeclared parameter if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if the parameter is statically typed and
  uninitialized.

#### get_parameter_or(name, alternative_value=None)

Get a parameter or the alternative value.

If the alternative value is None, a default Parameter with the given name and NOT_SET
type will be returned if the parameter was not declared.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **alternative_value** (`Optional`[`Parameter`]) – Alternative parameter to get if it had not been declared before.
* **Return type:**
  `Parameter`
* **Returns:**
  Requested parameter, or alternative value if it hadn’t been declared before or is
  an uninitialized statically typed parameter.

#### get_parameter_type(name)

Get a parameter type by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Type`
* **Returns:**
  The type for the given parameter name.
  A default Parameter.Type.NOT_SET will be returned for an undeclared parameter
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.

#### get_parameter_types(names)

Get a list of parameter types.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Type`]
* **Returns:**
  The values for the given parameter types.
  A default Parameter.Type.NOT_SET will be returned for undeclared parameters
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.

#### get_parameters(names)

Get a list of parameters.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  The values for the given parameter names.
  A default Parameter will be returned for undeclared parameters if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if at least one parameter is statically typed and
  uninitialized.

#### get_parameters_by_prefix(prefix)

Get parameters that have a given prefix in their names as a dictionary.

The names which are used as keys in the returned dictionary have the prefix removed.
For example, if you use the prefix “foo” and the parameters “foo.ping”, “foo.pong”
and “bar.baz” exist, then the returned dictionary will have the keys “ping” and “pong”.
Note that the parameter separator is also removed from the parameter name to create the
keys.

An empty string for the prefix will match all parameters.

If no parameters with the prefix are found, an empty dictionary will be returned.

* **Parameters:**
  **prefix** (`str`) – The prefix of the parameters to get.
* **Return type:**
  `Dict`[`str`, `Union`[`bool`, `int`, `float`, `str`, `bytes`, `Sequence`[`bool`], `Sequence`[`int`], `Sequence`[`float`], `Sequence`[`str`], `None`]]
* **Returns:**
  Dict of parameters with the given prefix.

#### get_publisher_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for publishers of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get publishers for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_publishers_info_by_topic(topic_name, no_mangle=False)

Return a list of publishers on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the publishers.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the publishers on this topic.

#### get_service_names_and_types()

Get a list of service topics for the node.

* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service name and the second element is a list of
  service types.

#### get_service_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service server topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get services for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service server name
  and the second element is a list of service types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriber_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for subscriptions of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get subscriptions for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriptions_info_by_topic(topic_name, no_mangle=False)

Return a list of subscriptions on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the subscriptions.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the subscriptions on this topic.

#### get_topic_names_and_types(no_demangle=False)

Get a list topic names and types for the node.

* **Parameters:**
  **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.

#### *property* guards

Get guards that have been created on this node.

* **Return type:**
  `Iterator`[`GuardCondition`]

#### *property* handle

Get the handle to the underlying rcl_node_t.

Cannot be modified after node creation.

* **Raises:**
  AttributeError if modified after creation.

#### has_parameter(name)

Return True if parameter is declared; False otherwise.

* **Return type:**
  `bool`
* **Parameters:**
  **name** (*str*)

#### perror(object, force=False)

Prints/Logs error if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** (*Any*) – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### pinfo(object, force=False)

Prints/Logs info if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **force** (*bool*) – if True the message will print whatever if self.Yapping is.

#### *property* publishers

Get publishers that have been created on this node.

* **Return type:**
  `Iterator`[`Publisher`]

#### pwarn(object, force=False)

Prints/Logs warning if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### remove_on_set_parameters_callback(callback)

Remove a callback from list of callbacks.

Calling this function will remove the callback from self._parameter_callbacks list.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Raises:**
  ValueError if a callback is not present in the list of callbacks.
* **Return type:**
  `None`

#### remove_waitable(waitable)

Remove a Waitable that was previously added to the node.

* **Parameters:**
  **waitable** (`Waitable`) – The Waitable to remove.
* **Return type:**
  `None`

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

#### resolve_topic_name(topic, \*, only_expand=False)

Return a topic name expanded and remapped.

* **Parameters:**
  * **topic** (`str`) – topic name to be expanded and remapped.
  * **only_expand** (`bool`) – if True, remapping rules won’t be applied.
* **Return type:**
  `str`
* **Returns:**
  a fully qualified topic name,
  result of applying expansion and remapping to the given topic.

#### *property* services

Get services that have been created on this node.

* **Return type:**
  `Iterator`[`Service`]

#### setAndBlockForNecessaryNodes(necessary_node_names, silent_trial=3, intervalSec=0.5)

Blocks for nodes to be alive

* **Parameters:**
  * **necessary_node_names** (*List* *[**str* *]*  *|* *str*)
  * **silent_trial** (*int* *|* *None*)
  * **intervalSec** (*float* *|* *None*)

#### set_descriptor(name, descriptor, alternative_value=None)

Set a new descriptor for a given parameter.

The name in the descriptor is ignored and set to **name**.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter to set the descriptor to.
  * **descriptor** (`ParameterDescriptor`) – New descriptor to apply to the parameter.
  * **alternative_value** (`Optional`[`ParameterValue`]) – Value to set to the parameter if the existing value does not
    comply with the new descriptor.
* **Return type:**
  `ParameterValue`
* **Returns:**
  ParameterValue for the given parameter name after applying the new descriptor.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.
* **Raises:**
  ParameterImmutableException if the parameter exists and is read-only.
* **Raises:**
  ParameterValueException if neither the existing value nor the alternative value
  complies with the provided descriptor.

#### set_parameters(parameter_list)

Set parameters for the node, and return the result for the set action.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set in the order they are declared in the list.
If setting a parameter fails due to not being declared, then the
parameters which have already been set will stay set, and no attempt will
be made to set the parameters which come after.

If undeclared parameters are allowed, then all the parameters will be implicitly
declared before being set even if they were not declared beforehand.
Parameter overrides are ignored by this method.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.gait_key_dev.KeyGaitNode.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node, once for each parameter.
If the callback prevents a parameter from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is
published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `List`[`SetParametersResult`]
* **Returns:**
  The result for each set action as a list.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### set_parameters_atomically(parameter_list)

Set the given parameters, all at one time, and then aggregate result.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set all at once.
If setting a parameter fails due to not being declared, then no parameter will be set set.
Either all of the parameters are set or none of them are set.

If undeclared parameters are allowed for the node, then all the parameters will be
implicitly declared before being set even if they were not declared beforehand.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.gait_key_dev.KeyGaitNode.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node only once for all parameters.
If the callback prevents the parameters from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `SetParametersResult`
* **Returns:**
  Aggregate result of setting all the parameters atomically.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### sleep(seconds)

sleeps using the node’s clock.

#### NOTE
Special case for good old foxy

* **Parameters:**
  **seconds** (*float*) – time to sleep
* **Return type:**
  `None`

#### *property* subscriptions

Get subscriptions that have been created on this node.

* **Return type:**
  `Iterator`[`Subscription`]

#### *property* timers

Get timers that have been created on this node.

* **Return type:**
  `Iterator`[`Timer`]

#### undeclare_parameter(name)

Undeclare a previously declared parameter.

This method will not cause a callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.gait_key_dev.KeyGaitNode.add_on_set_parameters_callback) to be called.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before.
* **Raises:**
  ParameterImmutableException if the parameter was created as read-only.

#### wait_for_lower_level(more_services={}, all_requiered=False)

Blocks until all or any service is available.

#### NOTE
- List of waited services is given by services_to_wait ros2 param
- Overload this to wait for an additional service

* **Parameters:**
  * **more_services** (*Iterable* *[**str* *]*) – add more services
  * **all_requiered** (*bool*) – if True, all listed services must be available

#### wait_on_futures(future_list, wait_Hz=10)

Waits for the completion of a list of futures, checking completion at the
provided rate.

* **Parameters:**
  * **future_list** (*List* *[**Future* *]*  *|* *Future*) – List of Future to wait for
  * **wait_Hz** (*float*) – rate at which to wait

#### *property* waitables

Get waitables that have been created on this node.

* **Return type:**
  `Iterator`[`Waitable`]

### easy_robot_control.gait_key_dev.main(args=None)

## easy_robot_control.gait_node module

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

`Optional`[`float`]

* **Type:**
  rtype

#### *property* speed

`Optional`[`float`]

* **Type:**
  rtype

#### *property* angle

`Optional`[`float`]

* **Type:**
  rtype

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
  **speed** (*float* *|* *None*) – approx speed value (max speed if for 1) or None
* **Return type:**
  `None`

#### speedTMRCBK()

Updates angle based on stored speed. Stops if speed is None

#### apply_angle_target(angle)

Sets angle target for the joint, and cancels speed command

* **Parameters:**
  **angle** (*float*) – angle target
* **Return type:**
  `None`

### *class* easy_robot_control.gait_node.Leg(number, parent)

Bases: `object`

Helps you use lvl 1-2-3 of a leg

* **Parameters:**
  * **number** (*int*)
  * **parent** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))

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
  * **parent** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))
  * **timeout** (*float*)

#### self_report()

* **Return type:**
  `str`

#### send_joint_cmd(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState) *]*)

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
  **joint** (*int* *|* *str*) – joint name or number (alphabetically ordered)
* **Returns:**
  last recieved angle as float
* **Return type:**
  `Optional`[`float`]

#### set_angle(angle, joint)

Sends a angle to a joint

* **Parameters:**
  * **angle** (*float*) – rad
  * **joint** (*int* *|* *str*) – joint name or number (alphabetically ordered)
* **Returns:**
  True if message sent
* **Return type:**
  `bool`

#### ik(xyz=None, quat=None)

Publishes an ik target for the leg () relative to baselink. Motion stack lvl2

* **Parameters:**
  * **xyz** (*None* *|* *ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *Sequence* *[**float* *]*)
  * **quat** (*quaternion* *|* *None*)
* **Return type:**
  `None`

#### move(xyz: None | numpy.ndarray[Any, numpy.dtype[numpy._typing._array_like._ScalarType_co]] | Sequence[float] = None, quat: quaternion.quaternion | None = None, mvt_type: Literal['shift', 'transl', 'rot', 'hop'] = 'shift', blocking: Literal[True] = True) → rclpy.task.Future

#### move(xyz: None | numpy.ndarray[Any, numpy.dtype[numpy._typing._array_like._ScalarType_co]] | Sequence[float] = None, quat: quaternion.quaternion | None = None, mvt_type: Literal['shift', 'transl', 'rot', 'hop'] = 'shift', blocking: Literal[False] = False) → motion_stack_msgs.srv._tf_service.TFService_Response

#### move(xyz=None, quat=None, mvt_type='shift', blocking=True)

Calls the leg’s movement service. Motion stack lvl3

* **Parameters:**
  * **xyz** (*None* *|* *ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *Sequence* *[**float* *]*) – vector part of the tf
  * **quat** (*quaternion* *|* *None*) – quat of the tf
  * **mvt_type** (*Literal* *[* *'shift'* *,*  *'transl'* *,*  *'rot'* *,*  *'hop'* *]*) – type of movement to call
  * **blocking** (*bool*) – if false returns a Future. Else returns the response
* **Return type:**
  *Future* | *TFService_Response*

Returns:

* **Return type:**
  `Union`[`Future`, `TFService_Response`]
* **Parameters:**
  * **xyz** (*None* *|* *ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *Sequence* *[**float* *]*)
  * **quat** (*quaternion* *|* *None*)
  * **mvt_type** (*Literal* *[* *'shift'* *,*  *'transl'* *,*  *'rot'* *,*  *'hop'* *]*)
  * **blocking** (*bool*)

### *class* easy_robot_control.gait_node.GaitNode

Bases: [`EliaNode`](#easy_robot_control.EliaNode.EliaNode)

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
  `ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]

#### PARAM_REL_TOL *= 1e-06*

**Type:**    `float`

#### add_on_set_parameters_callback(callback)

Add a callback in front to the list of callbacks.

Calling this function will add a callback in self._parameter_callbacks list.

It is considered bad practice to reject changes for “unknown” parameters as this prevents
other parts of the node (that may be aware of these parameters) from handling them.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Return type:**
  `None`

#### add_waitable(waitable)

Add a class that is capable of adding things to the wait set.

* **Parameters:**
  **waitable** (`Waitable`) – An instance of a waitable that the node will add to the waitset.
* **Return type:**
  `None`

#### *property* clients

Get clients that have been created on this node.

* **Return type:**
  `Iterator`[`Client`]

#### *property* context

Get the context associated with the node.

* **Return type:**
  `Context`

#### count_publishers(topic_name)

Return the number of publishers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of publishers.
* **Return type:**
  `int`
* **Returns:**
  the number of publishers on the topic.

#### count_subscribers(topic_name)

Return the number of subscribers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of subscribers.
* **Return type:**
  `int`
* **Returns:**
  the number of subscribers on the topic.

#### create_EZrate(frequency, clock=None)

Creates a better rate where rate.destroy actually destroys the rate

* **Parameters:**
  * **frequency** (*float*) – frequency of the rate
  * **clock** (*Clock* *|* *None*) – clock to use
* **Returns:**
  EZRate manipulating a Rate object
* **Return type:**
  [`EZRate`](#easy_robot_control.EliaNode.EZRate)

#### create_client(srv_type, srv_name, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service client.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service client.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service client. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Client`

#### create_guard_condition(callback, callback_group=None)

Create a new guard condition.

* **Return type:**
  `GuardCondition`
* **Parameters:**
  * **callback** (*Callable*)
  * **callback_group** (*CallbackGroup* *|* *None*)

#### create_publisher(msg_type, topic, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, publisher_class=<class 'Publisher'>)

Create a new publisher.

* **Parameters:**
  * **msg_type** – The type of ROS messages the publisher will publish.
  * **topic** (`str`) – The name of the topic the publisher will publish to.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the publisher.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the publisher’s event handlers.
    If `None`, then the node’s default callback group is used.
  * **event_callbacks** (`Optional`[`PublisherEventCallbacks`]) – User-defined callbacks for middleware events.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
  * **publisher_class** (*Type* *[**Publisher* *]*)
* **Return type:**
  `Publisher`
* **Returns:**
  The new publisher.

#### create_rate(frequency, clock=None)

Create a Rate object.

* **Parameters:**
  * **frequency** (`float`) – The frequency the Rate runs at (Hz).
  * **clock** (`Optional`[`Clock`]) – The clock the Rate gets time from.
* **Return type:**
  `Rate`

#### create_service(srv_type, srv_name, callback, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service server.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **callback** (`Callable`[[`~SrvTypeRequest`, `~SrvTypeResponse`], `~SrvTypeResponse`]) – A user-defined callback function that is called when a service request
    received by the server.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service server.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service server. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Service`

#### create_subscription(msg_type, topic, callback, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, raw=False)

Create a new subscription.

* **Parameters:**
  * **msg_type** – The type of ROS messages the subscription will subscribe to.
  * **topic** (`str`) – The name of the topic the subscription will subscribe to.
  * **callback** (`Callable`[[`~MsgType`], `None`]) – A user-defined callback function that is called when a message is
    received by the subscription.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the subscription.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the subscription. If `None`, then the
    nodes default callback group is used.
  * **event_callbacks** (`Optional`[`SubscriptionEventCallbacks`]) – User-defined callbacks for middleware events.
  * **raw** (`bool`) – If `True`, then received messages will be stored in raw binary
    representation.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
* **Return type:**
  `Subscription`

#### create_timer(timer_period_sec, callback, callback_group=None, clock=None)

Create a new timer.

The timer will be started and every `timer_period_sec` number of seconds the provided
callback function will be called.

* **Parameters:**
  * **timer_period_sec** (`float`) – The period (s) of the timer.
  * **callback** (`Callable`) – A user-defined callback function that is called when the timer expires.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the timer. If `None`, then the nodes
    default callback group is used.
  * **clock** (`Optional`[`Clock`]) – The clock which the timer gets time from.
* **Return type:**
  `Timer`

#### declare_parameter(name, value=None, descriptor=None, ignore_override=False)

Declare and initialize a parameter.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.gait_node.GaitNode.add_on_set_parameters_callback) to be called.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **value** (`Optional`[`Any`]) – Value of the parameter to declare.
  * **descriptor** (`Optional`[`ParameterDescriptor`]) – Descriptor for the parameter to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `Parameter`
* **Returns:**
  Parameter with the effectively assigned value.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects the parameter.

#### declare_parameters(namespace, parameters, ignore_override=False)

Declare a list of parameters.

The tuples in the given parameter list shall contain the name for each parameter,
optionally providing a value and a descriptor.
For each entry in the list, a parameter with a name of “namespace.name”
will be declared.
The resulting value for each declared parameter will be returned, considering
parameter overrides set upon node creation as the first choice,
or provided parameter values as the second one.

The name expansion is naive, so if you set the namespace to be “foo.”,
then the resulting parameter names will be like “foo..name”.
However, if the namespace is an empty string, then no leading ‘.’ will be
placed before each name, which would have been the case when naively
expanding “namespace.name”.
This allows you to declare several parameters at once without a namespace.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.gait_node.GaitNode.add_on_set_parameters_callback) to be called once for each parameter.
If one of those calls fail, an exception will be raised and the remaining parameters will
not be declared.
Parameters declared up to that point will not be undeclared.

* **Parameters:**
  * **namespace** (`str`) – Namespace for parameters.
  * **parameters** (`List`[`Union`[`Tuple`[`str`], `Tuple`[`str`, `Type`], `Tuple`[`str`, `Any`, `ParameterDescriptor`]]]) – List of tuples with parameters to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  Parameter list with the effectively assigned values for each of them.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects any parameter.
* **Raises:**
  TypeError if any tuple in **parameters** does not match the annotated type.

#### *property* default_callback_group

Get the default callback group.

If no other callback group is provided when the a ROS entity is created with the node,
then it is added to the default callback group.

* **Return type:**
  `CallbackGroup`

#### describe_parameter(name)

Get the parameter descriptor of a given parameter.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `ParameterDescriptor`
* **Returns:**
  ParameterDescriptor corresponding to the parameter,
  or default ParameterDescriptor if parameter had not been declared before
  and undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.

#### describe_parameters(names)

Get the parameter descriptors of a given list of parameters.

* **Parameters:**
  * **name** – List of fully-qualified names of the parameters to describe.
  * **names** (*List* *[**str* *]*)
* **Return type:**
  `List`[`ParameterDescriptor`]
* **Returns:**
  List of ParameterDescriptors corresponding to the given parameters.
  Default ParameterDescriptors shall be returned for parameters that
  had not been declared before if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if at least one parameter
  had not been declared before and undeclared parameters are not allowed.

#### destroy_client(client)

Destroy a service client created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **client** (*Client*)

#### destroy_guard_condition(guard)

Destroy a guard condition created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **guard** (*GuardCondition*)

#### destroy_node()

Destroy the node.

Frees resources used by the node, including any entities created by the following methods:

* [`create_publisher()`](#easy_robot_control.gait_node.GaitNode.create_publisher)
* [`create_subscription()`](#easy_robot_control.gait_node.GaitNode.create_subscription)
* [`create_client()`](#easy_robot_control.gait_node.GaitNode.create_client)
* [`create_service()`](#easy_robot_control.gait_node.GaitNode.create_service)
* [`create_timer()`](#easy_robot_control.gait_node.GaitNode.create_timer)
* [`create_guard_condition()`](#easy_robot_control.gait_node.GaitNode.create_guard_condition)

#### destroy_publisher(publisher)

Destroy a publisher created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **publisher** (*Publisher*)

#### destroy_rate(rate)

Destroy a Rate object created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **rate** (*Rate*)

#### destroy_service(service)

Destroy a service server created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **service** (*Service*)

#### destroy_subscription(subscription)

Destroy a subscription created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if succesful, `False` otherwise.
* **Parameters:**
  **subscription** (*Subscription*)

#### destroy_timer(timer)

Destroy a timer created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **timer** (*Timer*)

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

#### *property* executor

Get the executor if the node has been added to one, else return `None`.

* **Return type:**
  `Optional`[`Executor`]

#### getNow()

quick: self.get_clock().now()

* **Return type:**
  `Time`

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

#### get_client_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service client topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get service clients for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The fist element of each tuple is the service client name
  and the second element is a list of service client types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_clock()

Get the clock used by the node.

* **Return type:**
  `Clock`

#### get_fully_qualified_name()

Get the node’s fully qualified name.

* **Return type:**
  `str`
* **Returns:**
  Fully qualified node name.

#### get_logger()

Get the nodes logger.

#### get_name()

Get the name of the node.

* **Return type:**
  `str`

#### get_namespace()

Get the namespace of the node.

* **Return type:**
  `str`

#### get_node_names()

Get a list of names for discovered nodes.

* **Return type:**
  `List`[`str`]
* **Returns:**
  List of node names.

#### get_node_names_and_namespaces()

Get a list of names and namespaces for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`]]
* **Returns:**
  List of tuples containing two strings: the node name and node namespace.

#### get_node_names_and_namespaces_with_enclaves()

Get a list of names, namespaces and enclaves for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`, `str`]]
* **Returns:**
  List of tuples containing three strings: the node name, node namespace
  and enclave.

#### get_parameter(name)

Get a parameter by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Parameter`
* **Returns:**
  The value for the given parameter name.
  A default Parameter will be returned for an undeclared parameter if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if the parameter is statically typed and
  uninitialized.

#### get_parameter_or(name, alternative_value=None)

Get a parameter or the alternative value.

If the alternative value is None, a default Parameter with the given name and NOT_SET
type will be returned if the parameter was not declared.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **alternative_value** (`Optional`[`Parameter`]) – Alternative parameter to get if it had not been declared before.
* **Return type:**
  `Parameter`
* **Returns:**
  Requested parameter, or alternative value if it hadn’t been declared before or is
  an uninitialized statically typed parameter.

#### get_parameter_type(name)

Get a parameter type by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Type`
* **Returns:**
  The type for the given parameter name.
  A default Parameter.Type.NOT_SET will be returned for an undeclared parameter
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.

#### get_parameter_types(names)

Get a list of parameter types.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Type`]
* **Returns:**
  The values for the given parameter types.
  A default Parameter.Type.NOT_SET will be returned for undeclared parameters
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.

#### get_parameters(names)

Get a list of parameters.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  The values for the given parameter names.
  A default Parameter will be returned for undeclared parameters if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if at least one parameter is statically typed and
  uninitialized.

#### get_parameters_by_prefix(prefix)

Get parameters that have a given prefix in their names as a dictionary.

The names which are used as keys in the returned dictionary have the prefix removed.
For example, if you use the prefix “foo” and the parameters “foo.ping”, “foo.pong”
and “bar.baz” exist, then the returned dictionary will have the keys “ping” and “pong”.
Note that the parameter separator is also removed from the parameter name to create the
keys.

An empty string for the prefix will match all parameters.

If no parameters with the prefix are found, an empty dictionary will be returned.

* **Parameters:**
  **prefix** (`str`) – The prefix of the parameters to get.
* **Return type:**
  `Dict`[`str`, `Union`[`bool`, `int`, `float`, `str`, `bytes`, `Sequence`[`bool`], `Sequence`[`int`], `Sequence`[`float`], `Sequence`[`str`], `None`]]
* **Returns:**
  Dict of parameters with the given prefix.

#### get_publisher_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for publishers of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get publishers for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_publishers_info_by_topic(topic_name, no_mangle=False)

Return a list of publishers on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the publishers.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the publishers on this topic.

#### get_service_names_and_types()

Get a list of service topics for the node.

* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service name and the second element is a list of
  service types.

#### get_service_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service server topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get services for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service server name
  and the second element is a list of service types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriber_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for subscriptions of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get subscriptions for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriptions_info_by_topic(topic_name, no_mangle=False)

Return a list of subscriptions on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the subscriptions.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the subscriptions on this topic.

#### get_topic_names_and_types(no_demangle=False)

Get a list topic names and types for the node.

* **Parameters:**
  **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.

#### *property* guards

Get guards that have been created on this node.

* **Return type:**
  `Iterator`[`GuardCondition`]

#### *property* handle

Get the handle to the underlying rcl_node_t.

Cannot be modified after node creation.

* **Raises:**
  AttributeError if modified after creation.

#### has_parameter(name)

Return True if parameter is declared; False otherwise.

* **Return type:**
  `bool`
* **Parameters:**
  **name** (*str*)

#### perror(object, force=False)

Prints/Logs error if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** (*Any*) – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### pinfo(object, force=False)

Prints/Logs info if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **force** (*bool*) – if True the message will print whatever if self.Yapping is.

#### *property* publishers

Get publishers that have been created on this node.

* **Return type:**
  `Iterator`[`Publisher`]

#### pwarn(object, force=False)

Prints/Logs warning if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### remove_on_set_parameters_callback(callback)

Remove a callback from list of callbacks.

Calling this function will remove the callback from self._parameter_callbacks list.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Raises:**
  ValueError if a callback is not present in the list of callbacks.
* **Return type:**
  `None`

#### remove_waitable(waitable)

Remove a Waitable that was previously added to the node.

* **Parameters:**
  **waitable** (`Waitable`) – The Waitable to remove.
* **Return type:**
  `None`

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

#### resolve_topic_name(topic, \*, only_expand=False)

Return a topic name expanded and remapped.

* **Parameters:**
  * **topic** (`str`) – topic name to be expanded and remapped.
  * **only_expand** (`bool`) – if True, remapping rules won’t be applied.
* **Return type:**
  `str`
* **Returns:**
  a fully qualified topic name,
  result of applying expansion and remapping to the given topic.

#### *property* services

Get services that have been created on this node.

* **Return type:**
  `Iterator`[`Service`]

#### setAndBlockForNecessaryNodes(necessary_node_names, silent_trial=3, intervalSec=0.5)

Blocks for nodes to be alive

* **Parameters:**
  * **necessary_node_names** (*List* *[**str* *]*  *|* *str*)
  * **silent_trial** (*int* *|* *None*)
  * **intervalSec** (*float* *|* *None*)

#### set_descriptor(name, descriptor, alternative_value=None)

Set a new descriptor for a given parameter.

The name in the descriptor is ignored and set to **name**.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter to set the descriptor to.
  * **descriptor** (`ParameterDescriptor`) – New descriptor to apply to the parameter.
  * **alternative_value** (`Optional`[`ParameterValue`]) – Value to set to the parameter if the existing value does not
    comply with the new descriptor.
* **Return type:**
  `ParameterValue`
* **Returns:**
  ParameterValue for the given parameter name after applying the new descriptor.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.
* **Raises:**
  ParameterImmutableException if the parameter exists and is read-only.
* **Raises:**
  ParameterValueException if neither the existing value nor the alternative value
  complies with the provided descriptor.

#### set_parameters(parameter_list)

Set parameters for the node, and return the result for the set action.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set in the order they are declared in the list.
If setting a parameter fails due to not being declared, then the
parameters which have already been set will stay set, and no attempt will
be made to set the parameters which come after.

If undeclared parameters are allowed, then all the parameters will be implicitly
declared before being set even if they were not declared beforehand.
Parameter overrides are ignored by this method.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.gait_node.GaitNode.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node, once for each parameter.
If the callback prevents a parameter from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is
published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `List`[`SetParametersResult`]
* **Returns:**
  The result for each set action as a list.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### set_parameters_atomically(parameter_list)

Set the given parameters, all at one time, and then aggregate result.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set all at once.
If setting a parameter fails due to not being declared, then no parameter will be set set.
Either all of the parameters are set or none of them are set.

If undeclared parameters are allowed for the node, then all the parameters will be
implicitly declared before being set even if they were not declared beforehand.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.gait_node.GaitNode.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node only once for all parameters.
If the callback prevents the parameters from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `SetParametersResult`
* **Returns:**
  Aggregate result of setting all the parameters atomically.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### sleep(seconds)

sleeps using the node’s clock.

#### NOTE
Special case for good old foxy

* **Parameters:**
  **seconds** (*float*) – time to sleep
* **Return type:**
  `None`

#### *property* subscriptions

Get subscriptions that have been created on this node.

* **Return type:**
  `Iterator`[`Subscription`]

#### *property* timers

Get timers that have been created on this node.

* **Return type:**
  `Iterator`[`Timer`]

#### undeclare_parameter(name)

Undeclare a previously declared parameter.

This method will not cause a callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.gait_node.GaitNode.add_on_set_parameters_callback) to be called.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before.
* **Raises:**
  ParameterImmutableException if the parameter was created as read-only.

#### wait_for_lower_level(more_services={}, all_requiered=False)

Blocks until all or any service is available.

#### NOTE
- List of waited services is given by services_to_wait ros2 param
- Overload this to wait for an additional service

* **Parameters:**
  * **more_services** (*Iterable* *[**str* *]*) – add more services
  * **all_requiered** (*bool*) – if True, all listed services must be available

#### wait_on_futures(future_list, wait_Hz=10)

Waits for the completion of a list of futures, checking completion at the
provided rate.

* **Parameters:**
  * **future_list** (*List* *[**Future* *]*  *|* *Future*) – List of Future to wait for
  * **wait_Hz** (*float*) – rate at which to wait

#### *property* waitables

Get waitables that have been created on this node.

* **Return type:**
  `Iterator`[`Waitable`]

#### goToTargetBody(ts: numpy.ndarray[Any, numpy.dtype[numpy._typing._array_like._ScalarType_co]] | None = None, bodyXYZ: numpy.ndarray[Any, numpy.dtype[numpy._typing._array_like._ScalarType_co]] | None = None, bodyQuat: quaternion.quaternion | None = None, blocking: Literal[False] = False) → motion_stack_msgs.srv._send_target_body.SendTargetBody_Response

#### goToTargetBody(ts: numpy.ndarray[Any, numpy.dtype[numpy._typing._array_like._ScalarType_co]] | None = None, bodyXYZ: numpy.ndarray[Any, numpy.dtype[numpy._typing._array_like._ScalarType_co]] | None = None, bodyQuat: quaternion.quaternion | None = None, blocking: Literal[True] = True) → rclpy.task.Future

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

## easy_robot_control.ik_heavy_node module

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
  * **parent_node** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))

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

Bases: [`EliaNode`](#easy_robot_control.EliaNode.EliaNode)

#### firstSpinCBK()

#### PARAM_REL_TOL *= 1e-06*

**Type:**    `float`

#### add_on_set_parameters_callback(callback)

Add a callback in front to the list of callbacks.

Calling this function will add a callback in self._parameter_callbacks list.

It is considered bad practice to reject changes for “unknown” parameters as this prevents
other parts of the node (that may be aware of these parameters) from handling them.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Return type:**
  `None`

#### add_waitable(waitable)

Add a class that is capable of adding things to the wait set.

* **Parameters:**
  **waitable** (`Waitable`) – An instance of a waitable that the node will add to the waitset.
* **Return type:**
  `None`

#### all_limits(et_chain, jobjL)

* **Parameters:**
  * **et_chain** (*ETS*)
  * **jobjL** (*List* *[**Joint* *]*)

#### *property* clients

Get clients that have been created on this node.

* **Return type:**
  `Iterator`[`Client`]

#### *property* context

Get the context associated with the node.

* **Return type:**
  `Context`

#### count_publishers(topic_name)

Return the number of publishers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of publishers.
* **Return type:**
  `int`
* **Returns:**
  the number of publishers on the topic.

#### count_subscribers(topic_name)

Return the number of subscribers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of subscribers.
* **Return type:**
  `int`
* **Returns:**
  the number of subscribers on the topic.

#### create_EZrate(frequency, clock=None)

Creates a better rate where rate.destroy actually destroys the rate

* **Parameters:**
  * **frequency** (*float*) – frequency of the rate
  * **clock** (*Clock* *|* *None*) – clock to use
* **Returns:**
  EZRate manipulating a Rate object
* **Return type:**
  [`EZRate`](#easy_robot_control.EliaNode.EZRate)

#### create_client(srv_type, srv_name, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service client.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service client.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service client. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Client`

#### create_guard_condition(callback, callback_group=None)

Create a new guard condition.

* **Return type:**
  `GuardCondition`
* **Parameters:**
  * **callback** (*Callable*)
  * **callback_group** (*CallbackGroup* *|* *None*)

#### create_publisher(msg_type, topic, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, publisher_class=<class 'Publisher'>)

Create a new publisher.

* **Parameters:**
  * **msg_type** – The type of ROS messages the publisher will publish.
  * **topic** (`str`) – The name of the topic the publisher will publish to.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the publisher.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the publisher’s event handlers.
    If `None`, then the node’s default callback group is used.
  * **event_callbacks** (`Optional`[`PublisherEventCallbacks`]) – User-defined callbacks for middleware events.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
  * **publisher_class** (*Type* *[**Publisher* *]*)
* **Return type:**
  `Publisher`
* **Returns:**
  The new publisher.

#### create_rate(frequency, clock=None)

Create a Rate object.

* **Parameters:**
  * **frequency** (`float`) – The frequency the Rate runs at (Hz).
  * **clock** (`Optional`[`Clock`]) – The clock the Rate gets time from.
* **Return type:**
  `Rate`

#### create_service(srv_type, srv_name, callback, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service server.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **callback** (`Callable`[[`~SrvTypeRequest`, `~SrvTypeResponse`], `~SrvTypeResponse`]) – A user-defined callback function that is called when a service request
    received by the server.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service server.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service server. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Service`

#### create_subscription(msg_type, topic, callback, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, raw=False)

Create a new subscription.

* **Parameters:**
  * **msg_type** – The type of ROS messages the subscription will subscribe to.
  * **topic** (`str`) – The name of the topic the subscription will subscribe to.
  * **callback** (`Callable`[[`~MsgType`], `None`]) – A user-defined callback function that is called when a message is
    received by the subscription.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the subscription.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the subscription. If `None`, then the
    nodes default callback group is used.
  * **event_callbacks** (`Optional`[`SubscriptionEventCallbacks`]) – User-defined callbacks for middleware events.
  * **raw** (`bool`) – If `True`, then received messages will be stored in raw binary
    representation.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
* **Return type:**
  `Subscription`

#### create_timer(timer_period_sec, callback, callback_group=None, clock=None)

Create a new timer.

The timer will be started and every `timer_period_sec` number of seconds the provided
callback function will be called.

* **Parameters:**
  * **timer_period_sec** (`float`) – The period (s) of the timer.
  * **callback** (`Callable`) – A user-defined callback function that is called when the timer expires.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the timer. If `None`, then the nodes
    default callback group is used.
  * **clock** (`Optional`[`Clock`]) – The clock which the timer gets time from.
* **Return type:**
  `Timer`

#### declare_parameter(name, value=None, descriptor=None, ignore_override=False)

Declare and initialize a parameter.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.ik_heavy_node.IKNode.add_on_set_parameters_callback) to be called.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **value** (`Optional`[`Any`]) – Value of the parameter to declare.
  * **descriptor** (`Optional`[`ParameterDescriptor`]) – Descriptor for the parameter to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `Parameter`
* **Returns:**
  Parameter with the effectively assigned value.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects the parameter.

#### declare_parameters(namespace, parameters, ignore_override=False)

Declare a list of parameters.

The tuples in the given parameter list shall contain the name for each parameter,
optionally providing a value and a descriptor.
For each entry in the list, a parameter with a name of “namespace.name”
will be declared.
The resulting value for each declared parameter will be returned, considering
parameter overrides set upon node creation as the first choice,
or provided parameter values as the second one.

The name expansion is naive, so if you set the namespace to be “foo.”,
then the resulting parameter names will be like “foo..name”.
However, if the namespace is an empty string, then no leading ‘.’ will be
placed before each name, which would have been the case when naively
expanding “namespace.name”.
This allows you to declare several parameters at once without a namespace.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.ik_heavy_node.IKNode.add_on_set_parameters_callback) to be called once for each parameter.
If one of those calls fail, an exception will be raised and the remaining parameters will
not be declared.
Parameters declared up to that point will not be undeclared.

* **Parameters:**
  * **namespace** (`str`) – Namespace for parameters.
  * **parameters** (`List`[`Union`[`Tuple`[`str`], `Tuple`[`str`, `Type`], `Tuple`[`str`, `Any`, `ParameterDescriptor`]]]) – List of tuples with parameters to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  Parameter list with the effectively assigned values for each of them.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects any parameter.
* **Raises:**
  TypeError if any tuple in **parameters** does not match the annotated type.

#### *property* default_callback_group

Get the default callback group.

If no other callback group is provided when the a ROS entity is created with the node,
then it is added to the default callback group.

* **Return type:**
  `CallbackGroup`

#### describe_parameter(name)

Get the parameter descriptor of a given parameter.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `ParameterDescriptor`
* **Returns:**
  ParameterDescriptor corresponding to the parameter,
  or default ParameterDescriptor if parameter had not been declared before
  and undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.

#### describe_parameters(names)

Get the parameter descriptors of a given list of parameters.

* **Parameters:**
  * **name** – List of fully-qualified names of the parameters to describe.
  * **names** (*List* *[**str* *]*)
* **Return type:**
  `List`[`ParameterDescriptor`]
* **Returns:**
  List of ParameterDescriptors corresponding to the given parameters.
  Default ParameterDescriptors shall be returned for parameters that
  had not been declared before if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if at least one parameter
  had not been declared before and undeclared parameters are not allowed.

#### destroy_client(client)

Destroy a service client created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **client** (*Client*)

#### destroy_guard_condition(guard)

Destroy a guard condition created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **guard** (*GuardCondition*)

#### destroy_node()

Destroy the node.

Frees resources used by the node, including any entities created by the following methods:

* [`create_publisher()`](#easy_robot_control.ik_heavy_node.IKNode.create_publisher)
* [`create_subscription()`](#easy_robot_control.ik_heavy_node.IKNode.create_subscription)
* [`create_client()`](#easy_robot_control.ik_heavy_node.IKNode.create_client)
* [`create_service()`](#easy_robot_control.ik_heavy_node.IKNode.create_service)
* [`create_timer()`](#easy_robot_control.ik_heavy_node.IKNode.create_timer)
* [`create_guard_condition()`](#easy_robot_control.ik_heavy_node.IKNode.create_guard_condition)

#### destroy_publisher(publisher)

Destroy a publisher created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **publisher** (*Publisher*)

#### destroy_rate(rate)

Destroy a Rate object created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **rate** (*Rate*)

#### destroy_service(service)

Destroy a service server created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **service** (*Service*)

#### destroy_subscription(subscription)

Destroy a subscription created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if succesful, `False` otherwise.
* **Parameters:**
  **subscription** (*Subscription*)

#### destroy_timer(timer)

Destroy a timer created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **timer** (*Timer*)

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

#### *property* executor

Get the executor if the node has been added to one, else return `None`.

* **Return type:**
  `Optional`[`Executor`]

#### getNow()

quick: self.get_clock().now()

* **Return type:**
  `Time`

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

#### get_client_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service client topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get service clients for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The fist element of each tuple is the service client name
  and the second element is a list of service client types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_clock()

Get the clock used by the node.

* **Return type:**
  `Clock`

#### get_fully_qualified_name()

Get the node’s fully qualified name.

* **Return type:**
  `str`
* **Returns:**
  Fully qualified node name.

#### get_logger()

Get the nodes logger.

#### get_name()

Get the name of the node.

* **Return type:**
  `str`

#### get_namespace()

Get the namespace of the node.

* **Return type:**
  `str`

#### get_node_names()

Get a list of names for discovered nodes.

* **Return type:**
  `List`[`str`]
* **Returns:**
  List of node names.

#### get_node_names_and_namespaces()

Get a list of names and namespaces for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`]]
* **Returns:**
  List of tuples containing two strings: the node name and node namespace.

#### get_node_names_and_namespaces_with_enclaves()

Get a list of names, namespaces and enclaves for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`, `str`]]
* **Returns:**
  List of tuples containing three strings: the node name, node namespace
  and enclave.

#### get_parameter(name)

Get a parameter by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Parameter`
* **Returns:**
  The value for the given parameter name.
  A default Parameter will be returned for an undeclared parameter if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if the parameter is statically typed and
  uninitialized.

#### get_parameter_or(name, alternative_value=None)

Get a parameter or the alternative value.

If the alternative value is None, a default Parameter with the given name and NOT_SET
type will be returned if the parameter was not declared.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **alternative_value** (`Optional`[`Parameter`]) – Alternative parameter to get if it had not been declared before.
* **Return type:**
  `Parameter`
* **Returns:**
  Requested parameter, or alternative value if it hadn’t been declared before or is
  an uninitialized statically typed parameter.

#### get_parameter_type(name)

Get a parameter type by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Type`
* **Returns:**
  The type for the given parameter name.
  A default Parameter.Type.NOT_SET will be returned for an undeclared parameter
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.

#### get_parameter_types(names)

Get a list of parameter types.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Type`]
* **Returns:**
  The values for the given parameter types.
  A default Parameter.Type.NOT_SET will be returned for undeclared parameters
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.

#### get_parameters(names)

Get a list of parameters.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  The values for the given parameter names.
  A default Parameter will be returned for undeclared parameters if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if at least one parameter is statically typed and
  uninitialized.

#### get_parameters_by_prefix(prefix)

Get parameters that have a given prefix in their names as a dictionary.

The names which are used as keys in the returned dictionary have the prefix removed.
For example, if you use the prefix “foo” and the parameters “foo.ping”, “foo.pong”
and “bar.baz” exist, then the returned dictionary will have the keys “ping” and “pong”.
Note that the parameter separator is also removed from the parameter name to create the
keys.

An empty string for the prefix will match all parameters.

If no parameters with the prefix are found, an empty dictionary will be returned.

* **Parameters:**
  **prefix** (`str`) – The prefix of the parameters to get.
* **Return type:**
  `Dict`[`str`, `Union`[`bool`, `int`, `float`, `str`, `bytes`, `Sequence`[`bool`], `Sequence`[`int`], `Sequence`[`float`], `Sequence`[`str`], `None`]]
* **Returns:**
  Dict of parameters with the given prefix.

#### get_publisher_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for publishers of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get publishers for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_publishers_info_by_topic(topic_name, no_mangle=False)

Return a list of publishers on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the publishers.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the publishers on this topic.

#### get_service_names_and_types()

Get a list of service topics for the node.

* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service name and the second element is a list of
  service types.

#### get_service_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service server topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get services for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service server name
  and the second element is a list of service types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriber_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for subscriptions of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get subscriptions for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriptions_info_by_topic(topic_name, no_mangle=False)

Return a list of subscriptions on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the subscriptions.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the subscriptions on this topic.

#### get_topic_names_and_types(no_demangle=False)

Get a list topic names and types for the node.

* **Parameters:**
  **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.

#### *property* guards

Get guards that have been created on this node.

* **Return type:**
  `Iterator`[`GuardCondition`]

#### *property* handle

Get the handle to the underlying rcl_node_t.

Cannot be modified after node creation.

* **Raises:**
  AttributeError if modified after creation.

#### has_parameter(name)

Return True if parameter is declared; False otherwise.

* **Return type:**
  `bool`
* **Parameters:**
  **name** (*str*)

#### perror(object, force=False)

Prints/Logs error if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** (*Any*) – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### pinfo(object, force=False)

Prints/Logs info if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **force** (*bool*) – if True the message will print whatever if self.Yapping is.

#### *property* publishers

Get publishers that have been created on this node.

* **Return type:**
  `Iterator`[`Publisher`]

#### pwarn(object, force=False)

Prints/Logs warning if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### remove_on_set_parameters_callback(callback)

Remove a callback from list of callbacks.

Calling this function will remove the callback from self._parameter_callbacks list.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Raises:**
  ValueError if a callback is not present in the list of callbacks.
* **Return type:**
  `None`

#### remove_waitable(waitable)

Remove a Waitable that was previously added to the node.

* **Parameters:**
  **waitable** (`Waitable`) – The Waitable to remove.
* **Return type:**
  `None`

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

#### resolve_topic_name(topic, \*, only_expand=False)

Return a topic name expanded and remapped.

* **Parameters:**
  * **topic** (`str`) – topic name to be expanded and remapped.
  * **only_expand** (`bool`) – if True, remapping rules won’t be applied.
* **Return type:**
  `str`
* **Returns:**
  a fully qualified topic name,
  result of applying expansion and remapping to the given topic.

#### *property* services

Get services that have been created on this node.

* **Return type:**
  `Iterator`[`Service`]

#### setAndBlockForNecessaryNodes(necessary_node_names, silent_trial=3, intervalSec=0.5)

Blocks for nodes to be alive

* **Parameters:**
  * **necessary_node_names** (*List* *[**str* *]*  *|* *str*)
  * **silent_trial** (*int* *|* *None*)
  * **intervalSec** (*float* *|* *None*)

#### set_descriptor(name, descriptor, alternative_value=None)

Set a new descriptor for a given parameter.

The name in the descriptor is ignored and set to **name**.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter to set the descriptor to.
  * **descriptor** (`ParameterDescriptor`) – New descriptor to apply to the parameter.
  * **alternative_value** (`Optional`[`ParameterValue`]) – Value to set to the parameter if the existing value does not
    comply with the new descriptor.
* **Return type:**
  `ParameterValue`
* **Returns:**
  ParameterValue for the given parameter name after applying the new descriptor.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.
* **Raises:**
  ParameterImmutableException if the parameter exists and is read-only.
* **Raises:**
  ParameterValueException if neither the existing value nor the alternative value
  complies with the provided descriptor.

#### set_parameters(parameter_list)

Set parameters for the node, and return the result for the set action.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set in the order they are declared in the list.
If setting a parameter fails due to not being declared, then the
parameters which have already been set will stay set, and no attempt will
be made to set the parameters which come after.

If undeclared parameters are allowed, then all the parameters will be implicitly
declared before being set even if they were not declared beforehand.
Parameter overrides are ignored by this method.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.ik_heavy_node.IKNode.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node, once for each parameter.
If the callback prevents a parameter from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is
published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `List`[`SetParametersResult`]
* **Returns:**
  The result for each set action as a list.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### set_parameters_atomically(parameter_list)

Set the given parameters, all at one time, and then aggregate result.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set all at once.
If setting a parameter fails due to not being declared, then no parameter will be set set.
Either all of the parameters are set or none of them are set.

If undeclared parameters are allowed for the node, then all the parameters will be
implicitly declared before being set even if they were not declared beforehand.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.ik_heavy_node.IKNode.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node only once for all parameters.
If the callback prevents the parameters from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `SetParametersResult`
* **Returns:**
  Aggregate result of setting all the parameters atomically.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### sleep(seconds)

sleeps using the node’s clock.

#### NOTE
Special case for good old foxy

* **Parameters:**
  **seconds** (*float*) – time to sleep
* **Return type:**
  `None`

#### *property* subscriptions

Get subscriptions that have been created on this node.

* **Return type:**
  `Iterator`[`Subscription`]

#### *property* timers

Get timers that have been created on this node.

* **Return type:**
  `Iterator`[`Timer`]

#### undeclare_parameter(name)

Undeclare a previously declared parameter.

This method will not cause a callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.ik_heavy_node.IKNode.add_on_set_parameters_callback) to be called.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before.
* **Raises:**
  ParameterImmutableException if the parameter was created as read-only.

#### wait_for_lower_level(more_services={}, all_requiered=False)

Blocks until all or any service is available.

#### NOTE
- List of waited services is given by services_to_wait ros2 param
- Overload this to wait for an additional service

* **Parameters:**
  * **more_services** (*Iterable* *[**str* *]*) – add more services
  * **all_requiered** (*bool*) – if True, all listed services must be available

#### wait_on_futures(future_list, wait_Hz=10)

Waits for the completion of a list of futures, checking completion at the
provided rate.

* **Parameters:**
  * **future_list** (*List* *[**Future* *]*  *|* *Future*) – List of Future to wait for
  * **wait_Hz** (*float*) – rate at which to wait

#### *property* waitables

Get waitables that have been created on this node.

* **Return type:**
  `Iterator`[`Waitable`]

#### roll_CBK(msg)

* **Return type:**
  `None`
* **Parameters:**
  **msg** (*Float64* *|* *float*)

#### compute_raw_ik(xyz, quat, start, compute_budget=None, mvt_duration=None)

* **Return type:**
  `Tuple`[`Optional`[`ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]], `bool`]
* **Parameters:**
  * **xyz** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **quat** (*quaternion*)
  * **start** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **compute_budget** (*Duration* *|* *None*)
  * **mvt_duration** (*Duration* *|* *None*)

#### find_next_ik(xyz, quat, compute_budget=None, mvt_duration=None)

* **Return type:**
  `ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]
* **Parameters:**
  * **xyz** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **quat** (*quaternion*)
  * **compute_budget** (*Duration* *|* [*EZRate*](#easy_robot_control.EliaNode.EZRate) *|* *None*)
  * **mvt_duration** (*Duration* *|* *None*)

#### set_ik_CBK(msg)

recieves target from leg, converts to numpy, computes IK, sends angle
results to joints

* **Parameters:**
  **msg** (*Transform*) – target as Ros2 Vector3
* **Return type:**
  `None`

#### replace_none_target(xyz, quat)

* **Return type:**
  `Tuple`[`ndarray`[`Any`, `dtype`[`+_ScalarType_co`]], `quaternion`]
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
  `Tuple`[`ndarray`[`Any`, `dtype`[`+_ScalarType_co`]], `quaternion`]

#### publish_tip_pos()

Computes foward kinematics given angles stored in array,
publishes tip position result.
This is executed x ms after an angle reading is received

* **Return type:**
  `None`

### easy_robot_control.ik_heavy_node.main()

## easy_robot_control.joint_state_interface module

### *class* easy_robot_control.joint_state_interface.JointHandler(name, parent_node, joint_object, IGNORE_LIM=False, MARGIN=0.0)

Bases: `object`

This handles a single joint.
The main purpose is to update stateSensor and stateCommand. As well as getting the
newest values for those (in order to not continuously publish unchanging data).

* **Parameters:**
  * **name** (*str*)
  * **parent_node** ([*JointNode*](#easy_robot_control.joint_state_interface.JointNode))
  * **joint_object** (*Joint*)
  * **IGNORE_LIM** (*bool*)
  * **MARGIN** (*float*)

#### load_limit(ignore, jobj=None)

Loads the limit from the (urdf) joint object

* **Parameters:**
  * **ignore** (*bool*) – if limits should be ignored
  * **jobj** (*Joint* *|* *None*)

#### *property* limit_rejected

if the limit was rejected when loading (often when not defined in urdf)

* **Return type:**
  `bool`
* **Type:**
  Returns

#### speakup_when_angle()

start a verbose check every seconds for new angles

#### checkAngle(angle)

True is angle is valid or None

* **Return type:**
  `bool`
* **Parameters:**
  **angle** (*float* *|* *None*)

#### applyAngleLimit(angle)

Clamps the angle between the joints limits

* **Return type:**
  `Tuple`[`float`, `bool`]
* **Parameters:**
  **angle** (*float*)

#### resetAnglesAtZero()

#### update_js_command(js)

Updates the stateCommand to a new js.

* **Parameters:**
  **js** ([*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState))

#### is_new_jssensor(js)

True if js is different enough from the last received.
Also true if stateSensor is more the TOL_NO_CHANGE.time old relative to the new

* **Parameters:**
  **js** ([*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState))

#### setJSSensor(js)

Updates the stateSensor to a new js.

* **Parameters:**
  **js** ([*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState))

#### process_angle_command(angle)

This runs on new js before updating stateCommand

* **Return type:**
  `float`
* **Parameters:**
  **angle** (*float*)

#### process_velocity_command(speed)

This runs on new js before updating stateCommand

* **Return type:**
  `Optional`[`float`]
* **Parameters:**
  **speed** (*float*)

#### process_effort_command(eff)

This runs on new js before updating stateCommand

* **Return type:**
  `float`
* **Parameters:**
  **eff** (*float*)

#### set_effortCBK(msg)

Updates stateCommand by providing only an effort.
should be avoided as the timestamp will be set to now.

* **Parameters:**
  **msg** (*Float64* *|* *float*)

#### get_fresh_sensor(reset=True)

returns sensor data that is newer than the last time it was called.
if the sensor data didn’t changed enough to trigger a refresh, this will
be full of None. If a refresh occured, the None will be replaced by the non-None
values in the new sensor data.

example: if you stop sending speed sensor data after sending a bunch of speeds.
This speed will switch to None, it will not  continue to be the last received
speed.
This last received speed is still available in stateSensor.

* **Return type:**
  [`JState`](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState)
* **Parameters:**
  **reset** (*bool*)

#### get_freshCommand(reset=True)

returns command data that is newer than the last time it was called.
full of None is not newer

* **Return type:**
  [`JState`](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState)
* **Parameters:**
  **reset** (*bool*)

### *class* easy_robot_control.joint_state_interface.JointNode

Bases: [`EliaNode`](#easy_robot_control.EliaNode.EliaNode)

Lvl1

#### lvl0_remap

**Type:**    [`StateRemapper`](easy_robot_control.utils.md#easy_robot_control.utils.state_remaper.StateRemapper)

Remapping around any joint state communication of lvl0

#### lvl2_remap

**Type:**    [`StateRemapper`](easy_robot_control.utils.md#easy_robot_control.utils.state_remaper.StateRemapper)

Remapping around any joint state communication of lvl2

#### send_to_lvl0(states)

Sends states to lvl0 (commands for motors).
This function is executed every time data needs to be sent down.
Change/overload this method with what you need

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### send_to_lvl2(states)

Sends states to lvl2 (states for ik).
This function is executed every time data needs to be sent up.
Change/overload this method with what you need

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### js_from_lvl0(msg)

Callback when a JointState arrives from the lvl0 (states from motor).
Converts it into a list of states, then hands it to the general function

* **Parameters:**
  **msg** (*JointState*)

#### js_from_lvl2(msg)

Callback when a JointState arrives from the lvl2 (commands from ik).
Converts it into a list of states, then hands it to the general function

* **Parameters:**
  **msg** (*JointState*)

#### coming_from_lvl2(states)

Processes incomming commands from lvl2 ik.
Call this function after processing the ros message

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### coming_from_lvl0(states)

Processes incomming sensor states from lvl0 motors.
Call this function after processing the ros message.
Always do super().coming_from_lvl0(states) before your code,
Unless you know what you are doing

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### advertiserSRVCBK(req, res)

Sends an JointState mainly to advertise the names of the joints

* **Return type:**
  `ReturnJointState_Response`
* **Parameters:**
  * **req** (*ReturnJointState_Request*)
  * **res** (*ReturnJointState_Response*)

#### PARAM_REL_TOL *= 1e-06*

**Type:**    `float`

#### add_on_set_parameters_callback(callback)

Add a callback in front to the list of callbacks.

Calling this function will add a callback in self._parameter_callbacks list.

It is considered bad practice to reject changes for “unknown” parameters as this prevents
other parts of the node (that may be aware of these parameters) from handling them.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Return type:**
  `None`

#### add_waitable(waitable)

Add a class that is capable of adding things to the wait set.

* **Parameters:**
  **waitable** (`Waitable`) – An instance of a waitable that the node will add to the waitset.
* **Return type:**
  `None`

#### *property* clients

Get clients that have been created on this node.

* **Return type:**
  `Iterator`[`Client`]

#### *property* context

Get the context associated with the node.

* **Return type:**
  `Context`

#### count_publishers(topic_name)

Return the number of publishers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of publishers.
* **Return type:**
  `int`
* **Returns:**
  the number of publishers on the topic.

#### count_subscribers(topic_name)

Return the number of subscribers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of subscribers.
* **Return type:**
  `int`
* **Returns:**
  the number of subscribers on the topic.

#### create_EZrate(frequency, clock=None)

Creates a better rate where rate.destroy actually destroys the rate

* **Parameters:**
  * **frequency** (*float*) – frequency of the rate
  * **clock** (*Clock* *|* *None*) – clock to use
* **Returns:**
  EZRate manipulating a Rate object
* **Return type:**
  [`EZRate`](#easy_robot_control.EliaNode.EZRate)

#### create_client(srv_type, srv_name, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service client.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service client.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service client. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Client`

#### create_guard_condition(callback, callback_group=None)

Create a new guard condition.

* **Return type:**
  `GuardCondition`
* **Parameters:**
  * **callback** (*Callable*)
  * **callback_group** (*CallbackGroup* *|* *None*)

#### create_publisher(msg_type, topic, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, publisher_class=<class 'Publisher'>)

Create a new publisher.

* **Parameters:**
  * **msg_type** – The type of ROS messages the publisher will publish.
  * **topic** (`str`) – The name of the topic the publisher will publish to.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the publisher.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the publisher’s event handlers.
    If `None`, then the node’s default callback group is used.
  * **event_callbacks** (`Optional`[`PublisherEventCallbacks`]) – User-defined callbacks for middleware events.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
  * **publisher_class** (*Type* *[**Publisher* *]*)
* **Return type:**
  `Publisher`
* **Returns:**
  The new publisher.

#### create_rate(frequency, clock=None)

Create a Rate object.

* **Parameters:**
  * **frequency** (`float`) – The frequency the Rate runs at (Hz).
  * **clock** (`Optional`[`Clock`]) – The clock the Rate gets time from.
* **Return type:**
  `Rate`

#### create_service(srv_type, srv_name, callback, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service server.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **callback** (`Callable`[[`~SrvTypeRequest`, `~SrvTypeResponse`], `~SrvTypeResponse`]) – A user-defined callback function that is called when a service request
    received by the server.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service server.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service server. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Service`

#### create_subscription(msg_type, topic, callback, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, raw=False)

Create a new subscription.

* **Parameters:**
  * **msg_type** – The type of ROS messages the subscription will subscribe to.
  * **topic** (`str`) – The name of the topic the subscription will subscribe to.
  * **callback** (`Callable`[[`~MsgType`], `None`]) – A user-defined callback function that is called when a message is
    received by the subscription.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the subscription.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the subscription. If `None`, then the
    nodes default callback group is used.
  * **event_callbacks** (`Optional`[`SubscriptionEventCallbacks`]) – User-defined callbacks for middleware events.
  * **raw** (`bool`) – If `True`, then received messages will be stored in raw binary
    representation.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
* **Return type:**
  `Subscription`

#### create_timer(timer_period_sec, callback, callback_group=None, clock=None)

Create a new timer.

The timer will be started and every `timer_period_sec` number of seconds the provided
callback function will be called.

* **Parameters:**
  * **timer_period_sec** (`float`) – The period (s) of the timer.
  * **callback** (`Callable`) – A user-defined callback function that is called when the timer expires.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the timer. If `None`, then the nodes
    default callback group is used.
  * **clock** (`Optional`[`Clock`]) – The clock which the timer gets time from.
* **Return type:**
  `Timer`

#### declare_parameter(name, value=None, descriptor=None, ignore_override=False)

Declare and initialize a parameter.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.joint_state_interface.JointNode.add_on_set_parameters_callback) to be called.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **value** (`Optional`[`Any`]) – Value of the parameter to declare.
  * **descriptor** (`Optional`[`ParameterDescriptor`]) – Descriptor for the parameter to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `Parameter`
* **Returns:**
  Parameter with the effectively assigned value.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects the parameter.

#### declare_parameters(namespace, parameters, ignore_override=False)

Declare a list of parameters.

The tuples in the given parameter list shall contain the name for each parameter,
optionally providing a value and a descriptor.
For each entry in the list, a parameter with a name of “namespace.name”
will be declared.
The resulting value for each declared parameter will be returned, considering
parameter overrides set upon node creation as the first choice,
or provided parameter values as the second one.

The name expansion is naive, so if you set the namespace to be “foo.”,
then the resulting parameter names will be like “foo..name”.
However, if the namespace is an empty string, then no leading ‘.’ will be
placed before each name, which would have been the case when naively
expanding “namespace.name”.
This allows you to declare several parameters at once without a namespace.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.joint_state_interface.JointNode.add_on_set_parameters_callback) to be called once for each parameter.
If one of those calls fail, an exception will be raised and the remaining parameters will
not be declared.
Parameters declared up to that point will not be undeclared.

* **Parameters:**
  * **namespace** (`str`) – Namespace for parameters.
  * **parameters** (`List`[`Union`[`Tuple`[`str`], `Tuple`[`str`, `Type`], `Tuple`[`str`, `Any`, `ParameterDescriptor`]]]) – List of tuples with parameters to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  Parameter list with the effectively assigned values for each of them.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects any parameter.
* **Raises:**
  TypeError if any tuple in **parameters** does not match the annotated type.

#### *property* default_callback_group

Get the default callback group.

If no other callback group is provided when the a ROS entity is created with the node,
then it is added to the default callback group.

* **Return type:**
  `CallbackGroup`

#### defined_undefined()

Return joints with and without poistion data received yet

* **Returns:**
  Tuple(List[joint names that did not receive any data],
  List[joint names that have data])
* **Return type:**
  `Tuple`[`List`[`str`], `List`[`str`]]

#### describe_parameter(name)

Get the parameter descriptor of a given parameter.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `ParameterDescriptor`
* **Returns:**
  ParameterDescriptor corresponding to the parameter,
  or default ParameterDescriptor if parameter had not been declared before
  and undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.

#### describe_parameters(names)

Get the parameter descriptors of a given list of parameters.

* **Parameters:**
  * **name** – List of fully-qualified names of the parameters to describe.
  * **names** (*List* *[**str* *]*)
* **Return type:**
  `List`[`ParameterDescriptor`]
* **Returns:**
  List of ParameterDescriptors corresponding to the given parameters.
  Default ParameterDescriptors shall be returned for parameters that
  had not been declared before if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if at least one parameter
  had not been declared before and undeclared parameters are not allowed.

#### destroy_client(client)

Destroy a service client created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **client** (*Client*)

#### destroy_guard_condition(guard)

Destroy a guard condition created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **guard** (*GuardCondition*)

#### destroy_node()

Destroy the node.

Frees resources used by the node, including any entities created by the following methods:

* [`create_publisher()`](#easy_robot_control.joint_state_interface.JointNode.create_publisher)
* [`create_subscription()`](#easy_robot_control.joint_state_interface.JointNode.create_subscription)
* [`create_client()`](#easy_robot_control.joint_state_interface.JointNode.create_client)
* [`create_service()`](#easy_robot_control.joint_state_interface.JointNode.create_service)
* [`create_timer()`](#easy_robot_control.joint_state_interface.JointNode.create_timer)
* [`create_guard_condition()`](#easy_robot_control.joint_state_interface.JointNode.create_guard_condition)

#### destroy_publisher(publisher)

Destroy a publisher created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **publisher** (*Publisher*)

#### destroy_rate(rate)

Destroy a Rate object created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **rate** (*Rate*)

#### destroy_service(service)

Destroy a service server created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **service** (*Service*)

#### destroy_subscription(subscription)

Destroy a subscription created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if succesful, `False` otherwise.
* **Parameters:**
  **subscription** (*Subscription*)

#### destroy_timer(timer)

Destroy a timer created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **timer** (*Timer*)

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

#### *property* executor

Get the executor if the node has been added to one, else return `None`.

* **Return type:**
  `Optional`[`Executor`]

#### getNow()

quick: self.get_clock().now()

* **Return type:**
  `Time`

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

#### get_client_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service client topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get service clients for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The fist element of each tuple is the service client name
  and the second element is a list of service client types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_clock()

Get the clock used by the node.

* **Return type:**
  `Clock`

#### get_fully_qualified_name()

Get the node’s fully qualified name.

* **Return type:**
  `str`
* **Returns:**
  Fully qualified node name.

#### get_logger()

Get the nodes logger.

#### get_name()

Get the name of the node.

* **Return type:**
  `str`

#### get_namespace()

Get the namespace of the node.

* **Return type:**
  `str`

#### get_node_names()

Get a list of names for discovered nodes.

* **Return type:**
  `List`[`str`]
* **Returns:**
  List of node names.

#### get_node_names_and_namespaces()

Get a list of names and namespaces for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`]]
* **Returns:**
  List of tuples containing two strings: the node name and node namespace.

#### get_node_names_and_namespaces_with_enclaves()

Get a list of names, namespaces and enclaves for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`, `str`]]
* **Returns:**
  List of tuples containing three strings: the node name, node namespace
  and enclave.

#### get_parameter(name)

Get a parameter by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Parameter`
* **Returns:**
  The value for the given parameter name.
  A default Parameter will be returned for an undeclared parameter if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if the parameter is statically typed and
  uninitialized.

#### get_parameter_or(name, alternative_value=None)

Get a parameter or the alternative value.

If the alternative value is None, a default Parameter with the given name and NOT_SET
type will be returned if the parameter was not declared.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **alternative_value** (`Optional`[`Parameter`]) – Alternative parameter to get if it had not been declared before.
* **Return type:**
  `Parameter`
* **Returns:**
  Requested parameter, or alternative value if it hadn’t been declared before or is
  an uninitialized statically typed parameter.

#### get_parameter_type(name)

Get a parameter type by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Type`
* **Returns:**
  The type for the given parameter name.
  A default Parameter.Type.NOT_SET will be returned for an undeclared parameter
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.

#### get_parameter_types(names)

Get a list of parameter types.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Type`]
* **Returns:**
  The values for the given parameter types.
  A default Parameter.Type.NOT_SET will be returned for undeclared parameters
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.

#### get_parameters(names)

Get a list of parameters.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  The values for the given parameter names.
  A default Parameter will be returned for undeclared parameters if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if at least one parameter is statically typed and
  uninitialized.

#### get_parameters_by_prefix(prefix)

Get parameters that have a given prefix in their names as a dictionary.

The names which are used as keys in the returned dictionary have the prefix removed.
For example, if you use the prefix “foo” and the parameters “foo.ping”, “foo.pong”
and “bar.baz” exist, then the returned dictionary will have the keys “ping” and “pong”.
Note that the parameter separator is also removed from the parameter name to create the
keys.

An empty string for the prefix will match all parameters.

If no parameters with the prefix are found, an empty dictionary will be returned.

* **Parameters:**
  **prefix** (`str`) – The prefix of the parameters to get.
* **Return type:**
  `Dict`[`str`, `Union`[`bool`, `int`, `float`, `str`, `bytes`, `Sequence`[`bool`], `Sequence`[`int`], `Sequence`[`float`], `Sequence`[`str`], `None`]]
* **Returns:**
  Dict of parameters with the given prefix.

#### get_publisher_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for publishers of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get publishers for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_publishers_info_by_topic(topic_name, no_mangle=False)

Return a list of publishers on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the publishers.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the publishers on this topic.

#### get_service_names_and_types()

Get a list of service topics for the node.

* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service name and the second element is a list of
  service types.

#### get_service_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service server topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get services for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service server name
  and the second element is a list of service types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriber_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for subscriptions of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get subscriptions for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriptions_info_by_topic(topic_name, no_mangle=False)

Return a list of subscriptions on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the subscriptions.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the subscriptions on this topic.

#### get_topic_names_and_types(no_demangle=False)

Get a list topic names and types for the node.

* **Parameters:**
  **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.

#### *property* guards

Get guards that have been created on this node.

* **Return type:**
  `Iterator`[`GuardCondition`]

#### *property* handle

Get the handle to the underlying rcl_node_t.

Cannot be modified after node creation.

* **Raises:**
  AttributeError if modified after creation.

#### has_parameter(name)

Return True if parameter is declared; False otherwise.

* **Return type:**
  `bool`
* **Parameters:**
  **name** (*str*)

#### perror(object, force=False)

Prints/Logs error if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** (*Any*) – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### pinfo(object, force=False)

Prints/Logs info if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **force** (*bool*) – if True the message will print whatever if self.Yapping is.

#### *property* publishers

Get publishers that have been created on this node.

* **Return type:**
  `Iterator`[`Publisher`]

#### pwarn(object, force=False)

Prints/Logs warning if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### remove_on_set_parameters_callback(callback)

Remove a callback from list of callbacks.

Calling this function will remove the callback from self._parameter_callbacks list.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Raises:**
  ValueError if a callback is not present in the list of callbacks.
* **Return type:**
  `None`

#### remove_waitable(waitable)

Remove a Waitable that was previously added to the node.

* **Parameters:**
  **waitable** (`Waitable`) – The Waitable to remove.
* **Return type:**
  `None`

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

#### resolve_topic_name(topic, \*, only_expand=False)

Return a topic name expanded and remapped.

* **Parameters:**
  * **topic** (`str`) – topic name to be expanded and remapped.
  * **only_expand** (`bool`) – if True, remapping rules won’t be applied.
* **Return type:**
  `str`
* **Returns:**
  a fully qualified topic name,
  result of applying expansion and remapping to the given topic.

#### *property* services

Get services that have been created on this node.

* **Return type:**
  `Iterator`[`Service`]

#### setAndBlockForNecessaryNodes(necessary_node_names, silent_trial=3, intervalSec=0.5)

Blocks for nodes to be alive

* **Parameters:**
  * **necessary_node_names** (*List* *[**str* *]*  *|* *str*)
  * **silent_trial** (*int* *|* *None*)
  * **intervalSec** (*float* *|* *None*)

#### set_descriptor(name, descriptor, alternative_value=None)

Set a new descriptor for a given parameter.

The name in the descriptor is ignored and set to **name**.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter to set the descriptor to.
  * **descriptor** (`ParameterDescriptor`) – New descriptor to apply to the parameter.
  * **alternative_value** (`Optional`[`ParameterValue`]) – Value to set to the parameter if the existing value does not
    comply with the new descriptor.
* **Return type:**
  `ParameterValue`
* **Returns:**
  ParameterValue for the given parameter name after applying the new descriptor.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.
* **Raises:**
  ParameterImmutableException if the parameter exists and is read-only.
* **Raises:**
  ParameterValueException if neither the existing value nor the alternative value
  complies with the provided descriptor.

#### set_parameters(parameter_list)

Set parameters for the node, and return the result for the set action.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set in the order they are declared in the list.
If setting a parameter fails due to not being declared, then the
parameters which have already been set will stay set, and no attempt will
be made to set the parameters which come after.

If undeclared parameters are allowed, then all the parameters will be implicitly
declared before being set even if they were not declared beforehand.
Parameter overrides are ignored by this method.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.joint_state_interface.JointNode.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node, once for each parameter.
If the callback prevents a parameter from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is
published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `List`[`SetParametersResult`]
* **Returns:**
  The result for each set action as a list.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### set_parameters_atomically(parameter_list)

Set the given parameters, all at one time, and then aggregate result.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set all at once.
If setting a parameter fails due to not being declared, then no parameter will be set set.
Either all of the parameters are set or none of them are set.

If undeclared parameters are allowed for the node, then all the parameters will be
implicitly declared before being set even if they were not declared beforehand.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.joint_state_interface.JointNode.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node only once for all parameters.
If the callback prevents the parameters from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `SetParametersResult`
* **Returns:**
  Aggregate result of setting all the parameters atomically.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### sleep(seconds)

sleeps using the node’s clock.

#### NOTE
Special case for good old foxy

* **Parameters:**
  **seconds** (*float*) – time to sleep
* **Return type:**
  `None`

#### *property* subscriptions

Get subscriptions that have been created on this node.

* **Return type:**
  `Iterator`[`Subscription`]

#### *property* timers

Get timers that have been created on this node.

* **Return type:**
  `Iterator`[`Timer`]

#### undeclare_parameter(name)

Undeclare a previously declared parameter.

This method will not cause a callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.joint_state_interface.JointNode.add_on_set_parameters_callback) to be called.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before.
* **Raises:**
  ParameterImmutableException if the parameter was created as read-only.

#### wait_for_lower_level(more_services={}, all_requiered=False)

Blocks until all or any service is available.

#### NOTE
- List of waited services is given by services_to_wait ros2 param
- Overload this to wait for an additional service

* **Parameters:**
  * **more_services** (*Iterable* *[**str* *]*) – add more services
  * **all_requiered** (*bool*) – if True, all listed services must be available

#### wait_on_futures(future_list, wait_Hz=10)

Waits for the completion of a list of futures, checking completion at the
provided rate.

* **Parameters:**
  * **future_list** (*List* *[**Future* *]*  *|* *Future*) – List of Future to wait for
  * **wait_Hz** (*float*) – rate at which to wait

#### *property* waitables

Get waitables that have been created on this node.

* **Return type:**
  `Iterator`[`Waitable`]

#### angle_read_checkTMRCBK()

Checks that all joints are receiving data.
After 1s, if not warns the user, and starts the verbose check on the joint handler.

#### firstSpinCBK()

#### robot_body_pose_cbk(msg)

* **Parameters:**
  **msg** (*Transform*)

#### smoother(x)

smoothes the interval [0, 1] to have a soft start and end
(derivative is zero)

* **Return type:**
  `ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]
* **Parameters:**
  **x** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)

#### smooth_body_trans(request)

* **Parameters:**
  **request** (*Transform*)

#### go_zero_allCBK(req, resp)

* **Parameters:**
  * **req** (*Empty_Request*)
  * **resp** (*Empty_Response*)

### easy_robot_control.joint_state_interface.main(args=None)

## easy_robot_control.lazy_joint_state_publisher module

Overloading the joint_state_publisher package
so it does not publish joint states that are not actively published

Lots of black magic being used

### *class* easy_robot_control.lazy_joint_state_publisher.dummy_pub

Bases: `object`

#### publish(msg)

### *class* easy_robot_control.lazy_joint_state_publisher.LazyJointStatePublisher(description_file)

Bases: `JointStatePublisher`

#### source_cb(msg)

#### delete_inactive_from_msg(msg)

Deletes joints that are not part of self.active_joints from a message

* **Return type:**
  `JointState`
* **Parameters:**
  **msg** (*JointState*)

#### PARAM_REL_TOL *= 1e-06*

**Type:**    `float`

#### add_on_set_parameters_callback(callback)

Add a callback in front to the list of callbacks.

Calling this function will add a callback in self._parameter_callbacks list.

It is considered bad practice to reject changes for “unknown” parameters as this prevents
other parts of the node (that may be aware of these parameters) from handling them.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Return type:**
  `None`

#### add_waitable(waitable)

Add a class that is capable of adding things to the wait set.

* **Parameters:**
  **waitable** (`Waitable`) – An instance of a waitable that the node will add to the waitset.
* **Return type:**
  `None`

#### *property* clients

Get clients that have been created on this node.

* **Return type:**
  `Iterator`[`Client`]

#### configure_robot(description)

#### *property* context

Get the context associated with the node.

* **Return type:**
  `Context`

#### count_publishers(topic_name)

Return the number of publishers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of publishers.
* **Return type:**
  `int`
* **Returns:**
  the number of publishers on the topic.

#### count_subscribers(topic_name)

Return the number of subscribers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of subscribers.
* **Return type:**
  `int`
* **Returns:**
  the number of subscribers on the topic.

#### create_client(srv_type, srv_name, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service client.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service client.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service client. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Client`

#### create_guard_condition(callback, callback_group=None)

Create a new guard condition.

* **Return type:**
  `GuardCondition`
* **Parameters:**
  * **callback** (*Callable*)
  * **callback_group** (*CallbackGroup* *|* *None*)

#### create_publisher(msg_type, topic, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, publisher_class=<class 'Publisher'>)

Create a new publisher.

* **Parameters:**
  * **msg_type** – The type of ROS messages the publisher will publish.
  * **topic** (`str`) – The name of the topic the publisher will publish to.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the publisher.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the publisher’s event handlers.
    If `None`, then the node’s default callback group is used.
  * **event_callbacks** (`Optional`[`PublisherEventCallbacks`]) – User-defined callbacks for middleware events.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
  * **publisher_class** (*Type* *[**Publisher* *]*)
* **Return type:**
  `Publisher`
* **Returns:**
  The new publisher.

#### create_rate(frequency, clock=None)

Create a Rate object.

* **Parameters:**
  * **frequency** (`float`) – The frequency the Rate runs at (Hz).
  * **clock** (`Optional`[`Clock`]) – The clock the Rate gets time from.
* **Return type:**
  `Rate`

#### create_service(srv_type, srv_name, callback, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service server.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **callback** (`Callable`[[`~SrvTypeRequest`, `~SrvTypeResponse`], `~SrvTypeResponse`]) – A user-defined callback function that is called when a service request
    received by the server.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service server.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service server. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Service`

#### create_subscription(msg_type, topic, callback, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, raw=False)

Create a new subscription.

* **Parameters:**
  * **msg_type** – The type of ROS messages the subscription will subscribe to.
  * **topic** (`str`) – The name of the topic the subscription will subscribe to.
  * **callback** (`Callable`[[`~MsgType`], `None`]) – A user-defined callback function that is called when a message is
    received by the subscription.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the subscription.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the subscription. If `None`, then the
    nodes default callback group is used.
  * **event_callbacks** (`Optional`[`SubscriptionEventCallbacks`]) – User-defined callbacks for middleware events.
  * **raw** (`bool`) – If `True`, then received messages will be stored in raw binary
    representation.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
* **Return type:**
  `Subscription`

#### create_timer(timer_period_sec, callback, callback_group=None, clock=None)

Create a new timer.

The timer will be started and every `timer_period_sec` number of seconds the provided
callback function will be called.

* **Parameters:**
  * **timer_period_sec** (`float`) – The period (s) of the timer.
  * **callback** (`Callable`) – A user-defined callback function that is called when the timer expires.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the timer. If `None`, then the nodes
    default callback group is used.
  * **clock** (`Optional`[`Clock`]) – The clock which the timer gets time from.
* **Return type:**
  `Timer`

#### declare_parameter(name, value=None, descriptor=None, ignore_override=False)

Declare and initialize a parameter.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.lazy_joint_state_publisher.LazyJointStatePublisher.add_on_set_parameters_callback) to be called.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **value** (`Optional`[`Any`]) – Value of the parameter to declare.
  * **descriptor** (`Optional`[`ParameterDescriptor`]) – Descriptor for the parameter to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `Parameter`
* **Returns:**
  Parameter with the effectively assigned value.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects the parameter.

#### declare_parameters(namespace, parameters, ignore_override=False)

Declare a list of parameters.

The tuples in the given parameter list shall contain the name for each parameter,
optionally providing a value and a descriptor.
For each entry in the list, a parameter with a name of “namespace.name”
will be declared.
The resulting value for each declared parameter will be returned, considering
parameter overrides set upon node creation as the first choice,
or provided parameter values as the second one.

The name expansion is naive, so if you set the namespace to be “foo.”,
then the resulting parameter names will be like “foo..name”.
However, if the namespace is an empty string, then no leading ‘.’ will be
placed before each name, which would have been the case when naively
expanding “namespace.name”.
This allows you to declare several parameters at once without a namespace.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.lazy_joint_state_publisher.LazyJointStatePublisher.add_on_set_parameters_callback) to be called once for each parameter.
If one of those calls fail, an exception will be raised and the remaining parameters will
not be declared.
Parameters declared up to that point will not be undeclared.

* **Parameters:**
  * **namespace** (`str`) – Namespace for parameters.
  * **parameters** (`List`[`Union`[`Tuple`[`str`], `Tuple`[`str`, `Type`], `Tuple`[`str`, `Any`, `ParameterDescriptor`]]]) – List of tuples with parameters to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  Parameter list with the effectively assigned values for each of them.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects any parameter.
* **Raises:**
  TypeError if any tuple in **parameters** does not match the annotated type.

#### declare_ros_parameter(name, default, descriptor)

#### *property* default_callback_group

Get the default callback group.

If no other callback group is provided when the a ROS entity is created with the node,
then it is added to the default callback group.

* **Return type:**
  `CallbackGroup`

#### describe_parameter(name)

Get the parameter descriptor of a given parameter.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `ParameterDescriptor`
* **Returns:**
  ParameterDescriptor corresponding to the parameter,
  or default ParameterDescriptor if parameter had not been declared before
  and undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.

#### describe_parameters(names)

Get the parameter descriptors of a given list of parameters.

* **Parameters:**
  * **name** – List of fully-qualified names of the parameters to describe.
  * **names** (*List* *[**str* *]*)
* **Return type:**
  `List`[`ParameterDescriptor`]
* **Returns:**
  List of ParameterDescriptors corresponding to the given parameters.
  Default ParameterDescriptors shall be returned for parameters that
  had not been declared before if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if at least one parameter
  had not been declared before and undeclared parameters are not allowed.

#### destroy_client(client)

Destroy a service client created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **client** (*Client*)

#### destroy_guard_condition(guard)

Destroy a guard condition created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **guard** (*GuardCondition*)

#### destroy_node()

Destroy the node.

Frees resources used by the node, including any entities created by the following methods:

* [`create_publisher()`](#easy_robot_control.lazy_joint_state_publisher.LazyJointStatePublisher.create_publisher)
* [`create_subscription()`](#easy_robot_control.lazy_joint_state_publisher.LazyJointStatePublisher.create_subscription)
* [`create_client()`](#easy_robot_control.lazy_joint_state_publisher.LazyJointStatePublisher.create_client)
* [`create_service()`](#easy_robot_control.lazy_joint_state_publisher.LazyJointStatePublisher.create_service)
* [`create_timer()`](#easy_robot_control.lazy_joint_state_publisher.LazyJointStatePublisher.create_timer)
* [`create_guard_condition()`](#easy_robot_control.lazy_joint_state_publisher.LazyJointStatePublisher.create_guard_condition)

#### destroy_publisher(publisher)

Destroy a publisher created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **publisher** (*Publisher*)

#### destroy_rate(rate)

Destroy a Rate object created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **rate** (*Rate*)

#### destroy_service(service)

Destroy a service server created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **service** (*Service*)

#### destroy_subscription(subscription)

Destroy a subscription created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if succesful, `False` otherwise.
* **Parameters:**
  **subscription** (*Subscription*)

#### destroy_timer(timer)

Destroy a timer created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **timer** (*Timer*)

#### *property* executor

Get the executor if the node has been added to one, else return `None`.

* **Return type:**
  `Optional`[`Executor`]

#### get_client_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service client topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get service clients for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The fist element of each tuple is the service client name
  and the second element is a list of service client types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_clock()

Get the clock used by the node.

* **Return type:**
  `Clock`

#### get_fully_qualified_name()

Get the node’s fully qualified name.

* **Return type:**
  `str`
* **Returns:**
  Fully qualified node name.

#### get_logger()

Get the nodes logger.

#### get_name()

Get the name of the node.

* **Return type:**
  `str`

#### get_namespace()

Get the namespace of the node.

* **Return type:**
  `str`

#### get_node_names()

Get a list of names for discovered nodes.

* **Return type:**
  `List`[`str`]
* **Returns:**
  List of node names.

#### get_node_names_and_namespaces()

Get a list of names and namespaces for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`]]
* **Returns:**
  List of tuples containing two strings: the node name and node namespace.

#### get_node_names_and_namespaces_with_enclaves()

Get a list of names, namespaces and enclaves for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`, `str`]]
* **Returns:**
  List of tuples containing three strings: the node name, node namespace
  and enclave.

#### get_param(name)

#### get_parameter(name)

Get a parameter by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Parameter`
* **Returns:**
  The value for the given parameter name.
  A default Parameter will be returned for an undeclared parameter if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if the parameter is statically typed and
  uninitialized.

#### get_parameter_or(name, alternative_value=None)

Get a parameter or the alternative value.

If the alternative value is None, a default Parameter with the given name and NOT_SET
type will be returned if the parameter was not declared.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **alternative_value** (`Optional`[`Parameter`]) – Alternative parameter to get if it had not been declared before.
* **Return type:**
  `Parameter`
* **Returns:**
  Requested parameter, or alternative value if it hadn’t been declared before or is
  an uninitialized statically typed parameter.

#### get_parameter_type(name)

Get a parameter type by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Type`
* **Returns:**
  The type for the given parameter name.
  A default Parameter.Type.NOT_SET will be returned for an undeclared parameter
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.

#### get_parameter_types(names)

Get a list of parameter types.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Type`]
* **Returns:**
  The values for the given parameter types.
  A default Parameter.Type.NOT_SET will be returned for undeclared parameters
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.

#### get_parameters(names)

Get a list of parameters.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  The values for the given parameter names.
  A default Parameter will be returned for undeclared parameters if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if at least one parameter is statically typed and
  uninitialized.

#### get_parameters_by_prefix(prefix)

Get parameters that have a given prefix in their names as a dictionary.

The names which are used as keys in the returned dictionary have the prefix removed.
For example, if you use the prefix “foo” and the parameters “foo.ping”, “foo.pong”
and “bar.baz” exist, then the returned dictionary will have the keys “ping” and “pong”.
Note that the parameter separator is also removed from the parameter name to create the
keys.

An empty string for the prefix will match all parameters.

If no parameters with the prefix are found, an empty dictionary will be returned.

* **Parameters:**
  **prefix** (`str`) – The prefix of the parameters to get.
* **Return type:**
  `Dict`[`str`, `Union`[`bool`, `int`, `float`, `str`, `bytes`, `Sequence`[`bool`], `Sequence`[`int`], `Sequence`[`float`], `Sequence`[`str`], `None`]]
* **Returns:**
  Dict of parameters with the given prefix.

#### get_publisher_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for publishers of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get publishers for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_publishers_info_by_topic(topic_name, no_mangle=False)

Return a list of publishers on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the publishers.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the publishers on this topic.

#### get_service_names_and_types()

Get a list of service topics for the node.

* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service name and the second element is a list of
  service types.

#### get_service_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service server topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get services for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service server name
  and the second element is a list of service types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriber_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for subscriptions of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get subscriptions for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriptions_info_by_topic(topic_name, no_mangle=False)

Return a list of subscriptions on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the subscriptions.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the subscriptions on this topic.

#### get_topic_names_and_types(no_demangle=False)

Get a list topic names and types for the node.

* **Parameters:**
  **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.

#### *property* guards

Get guards that have been created on this node.

* **Return type:**
  `Iterator`[`GuardCondition`]

#### *property* handle

Get the handle to the underlying rcl_node_t.

Cannot be modified after node creation.

* **Raises:**
  AttributeError if modified after creation.

#### has_parameter(name)

Return True if parameter is declared; False otherwise.

* **Return type:**
  `bool`
* **Parameters:**
  **name** (*str*)

#### init_collada(robot)

#### init_sdf(robot)

#### init_urdf(robot)

#### parse_dependent_joints()

#### *property* publishers

Get publishers that have been created on this node.

* **Return type:**
  `Iterator`[`Publisher`]

#### remove_on_set_parameters_callback(callback)

Remove a callback from list of callbacks.

Calling this function will remove the callback from self._parameter_callbacks list.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Raises:**
  ValueError if a callback is not present in the list of callbacks.
* **Return type:**
  `None`

#### remove_waitable(waitable)

Remove a Waitable that was previously added to the node.

* **Parameters:**
  **waitable** (`Waitable`) – The Waitable to remove.
* **Return type:**
  `None`

#### resolve_service_name(service, \*, only_expand=False)

Return a service name expanded and remapped.

* **Parameters:**
  * **service** (`str`) – service name to be expanded and remapped.
  * **only_expand** (`bool`) – if True, remapping rules won’t be applied.
* **Return type:**
  `str`
* **Returns:**
  a fully qualified service name,
  result of applying expansion and remapping to the given service.

#### resolve_topic_name(topic, \*, only_expand=False)

Return a topic name expanded and remapped.

* **Parameters:**
  * **topic** (`str`) – topic name to be expanded and remapped.
  * **only_expand** (`bool`) – if True, remapping rules won’t be applied.
* **Return type:**
  `str`
* **Returns:**
  a fully qualified topic name,
  result of applying expansion and remapping to the given topic.

#### *property* services

Get services that have been created on this node.

* **Return type:**
  `Iterator`[`Service`]

#### set_descriptor(name, descriptor, alternative_value=None)

Set a new descriptor for a given parameter.

The name in the descriptor is ignored and set to **name**.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter to set the descriptor to.
  * **descriptor** (`ParameterDescriptor`) – New descriptor to apply to the parameter.
  * **alternative_value** (`Optional`[`ParameterValue`]) – Value to set to the parameter if the existing value does not
    comply with the new descriptor.
* **Return type:**
  `ParameterValue`
* **Returns:**
  ParameterValue for the given parameter name after applying the new descriptor.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.
* **Raises:**
  ParameterImmutableException if the parameter exists and is read-only.
* **Raises:**
  ParameterValueException if neither the existing value nor the alternative value
  complies with the provided descriptor.

#### set_parameters(parameter_list)

Set parameters for the node, and return the result for the set action.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set in the order they are declared in the list.
If setting a parameter fails due to not being declared, then the
parameters which have already been set will stay set, and no attempt will
be made to set the parameters which come after.

If undeclared parameters are allowed, then all the parameters will be implicitly
declared before being set even if they were not declared beforehand.
Parameter overrides are ignored by this method.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.lazy_joint_state_publisher.LazyJointStatePublisher.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node, once for each parameter.
If the callback prevents a parameter from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is
published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `List`[`SetParametersResult`]
* **Returns:**
  The result for each set action as a list.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### set_parameters_atomically(parameter_list)

Set the given parameters, all at one time, and then aggregate result.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set all at once.
If setting a parameter fails due to not being declared, then no parameter will be set set.
Either all of the parameters are set or none of them are set.

If undeclared parameters are allowed for the node, then all the parameters will be
implicitly declared before being set even if they were not declared beforehand.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.lazy_joint_state_publisher.LazyJointStatePublisher.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node only once for all parameters.
If the callback prevents the parameters from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `SetParametersResult`
* **Returns:**
  Aggregate result of setting all the parameters atomically.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### set_robot_description_update_cb(user_cb)

#### set_source_update_cb(user_cb)

#### *property* subscriptions

Get subscriptions that have been created on this node.

* **Return type:**
  `Iterator`[`Subscription`]

#### timer_callback()

#### *property* timers

Get timers that have been created on this node.

* **Return type:**
  `Iterator`[`Timer`]

#### undeclare_parameter(name)

Undeclare a previously declared parameter.

This method will not cause a callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.lazy_joint_state_publisher.LazyJointStatePublisher.add_on_set_parameters_callback) to be called.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before.
* **Raises:**
  ParameterImmutableException if the parameter was created as read-only.

#### update(delta)

#### *property* waitables

Get waitables that have been created on this node.

* **Return type:**
  `Iterator`[`Waitable`]

### easy_robot_control.lazy_joint_state_publisher.main()

## easy_robot_control.leg_api module

Provides api anc controllers to control leg ik and joints directly

Author: Elian NEPPEL
Lab: SRL, Moonshot team

### easy_robot_control.leg_api.float_formatter()

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

### *class* easy_robot_control.leg_api.Pose(time, xyz, quat)

Bases: `object`

* **Parameters:**
  * **time** (*Time*)
  * **xyz** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*)
  * **quat** (*quaternion*)

#### time

**Type:**    `Time`

#### xyz

**Type:**    `NDArray`[`Shape`[`3`], `floating`]

#### quat

**Type:**    `quaternion`

#### close2zero(atol=(1, 0.01))

* **Return type:**
  `bool`

### *class* easy_robot_control.leg_api.JointMini(joint_name, prefix, parent_leg)

Bases: `object`

* **Parameters:**
  * **joint_name** (*str*)
  * **prefix** (*str*)
  * **parent_leg** ([*Leg*](#easy_robot_control.leg_api.Leg))

#### is_active()

#### self_report()

#### *property* effort

`Optional`[`float`]

* **Type:**
  rtype

#### *property* speed

`Optional`[`float`]

* **Type:**
  rtype

#### *property* angle

`Optional`[`float`]

* **Type:**
  rtype

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
  **speed** (*float* *|* *None*) – approx speed value (max speed if for 1) or None
* **Return type:**
  `None`

#### apply_angle_target(angle)

Sets angle target for the joint, and cancels speed command

* **Parameters:**
  **angle** (*float*) – angle target
* **Return type:**
  `None`

### easy_robot_control.leg_api.T *= TypeVar(T, NDArray, quaternion)*

**Type:**    `TypeVar`

Invariant `TypeVar` constrained to `nptyping.ndarray.NDArray` and `quaternion.quaternion`.

### *class* easy_robot_control.leg_api.Leg(number, parent)

Bases: `object`

Helps you use lvl 1-2-3 of a leg

* **Parameters:**
  * **number** (*int*)
  * **parent** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))

#### number

leg number

#### parent

parent node spinning

#### joint_name_list

list of joints belonging to the leg

#### connect_movement_clients()

#### *static* do_i_exist(number, parent, timeout=1)

Slow do not use to spam scan.
Returns True if the leg is alive

* **Parameters:**
  * **number** (*int*)
  * **parent** ([*EliaNode*](#easy_robot_control.EliaNode.EliaNode))
  * **timeout** (*float*)

#### self_report()

* **Return type:**
  `str`

#### add_joints(all_joints)

* **Return type:**
  `List`[[`JointMini`](#easy_robot_control.leg_api.JointMini)]
* **Parameters:**
  **all_joints** (*List* *[**str* *]*)

#### look_for_joints()

scans and updates the list of joints of this leg

#### go2zero()

sends angle target of 0 on all joints

#### get_joint_obj(joint)

Gets the corresponding joint object is exists

* **Parameters:**
  **joint** (*int* *|* *str*) – joint name or number (alphabetically ordered)
* **Return type:**
  `Optional`[[`JointMini`](#easy_robot_control.leg_api.JointMini)]

#### get_angle(joint)

Gets an angle from a joint

* **Parameters:**
  **joint** (*int* *|* *str*) – joint name or number (alphabetically ordered)
* **Returns:**
  last recieved angle as float
* **Return type:**
  `Optional`[`float`]

#### set_angle(angle, joint)

Sends a angle to a joint

* **Parameters:**
  * **angle** (*float*) – rad
  * **joint** (*int* *|* *str*) – joint name or number (alphabetically ordered)
* **Returns:**
  True if message sent
* **Return type:**
  `bool`

#### ik(xyz=None, quat=None)

Publishes an ik target for the leg () relative to baselink. Motion stack lvl2

* **Parameters:**
  * **xyz** (*None* *|* *NDArray* *[**Any* *,* *Any* *]*  *|* *Sequence* *[**float* *]*)
  * **quat** (*quaternion* *|* *None*)
* **Return type:**
  `None`

#### move(xyz: None | nptyping.ndarray.NDArray[Any, Any] | Sequence[float] = None, quat: quaternion.quaternion | None = None, mvt_type: Literal['shift', 'transl', 'rot', 'hop'] = 'shift', blocking: Literal[True] = True) → rclpy.task.Future

#### move(xyz: None | nptyping.ndarray.NDArray[Any, Any] | Sequence[float] = None, quat: quaternion.quaternion | None = None, mvt_type: Literal['shift', 'transl', 'rot', 'hop'] = 'shift', blocking: Literal[False] = False) → motion_stack_msgs.srv._tf_service.TFService_Response

#### move(xyz=None, quat=None, mvt_type='shift', blocking=True)

Calls the leg’s movement service. Motion stack lvl3

* **Parameters:**
  * **xyz** (*None* *|* *NDArray* *[**Any* *,* *Any* *]*  *|* *Sequence* *[**float* *]*) – vector part of the tf
  * **quat** (*quaternion* *|* *None*) – quat of the tf
  * **mvt_type** (*Literal* *[* *'shift'* *,*  *'transl'* *,*  *'rot'* *,*  *'hop'* *]*) – type of movement to call
  * **blocking** (*bool*) – if false returns a Future. Else returns the response
* **Return type:**
  *Future* | *TFService_Response*

Returns:

* **Return type:**
  `Union`[`Future`, `TFService_Response`]
* **Parameters:**
  * **xyz** (*None* *|* *NDArray* *[**Any* *,* *Any* *]*  *|* *Sequence* *[**float* *]*)
  * **quat** (*quaternion* *|* *None*)
  * **mvt_type** (*Literal* *[* *'shift'* *,*  *'transl'* *,*  *'rot'* *,*  *'hop'* *]*)
  * **blocking** (*bool*)

### *class* easy_robot_control.leg_api.Ik2(leg)

Bases: `object`

Provides ik2 methods given a leg

* **Parameters:**
  **leg** ([*Leg*](#easy_robot_control.leg_api.Leg))

#### run_task()

#### *property* quat_now

`Optional`[`quaternion`]

* **Type:**
  rtype

#### *property* xyz_now

`Optional`[`NDArray`[`Shape`[`3`], `floating`]]

* **Type:**
  rtype

#### *property* now_pose

`Optional`[[`Pose`](#easy_robot_control.leg_api.Pose)]

* **Type:**
  rtype

#### reset()

Resets the trajectory start point onto the current end effector position

* **Return type:**
  [`Pose`](#easy_robot_control.leg_api.Pose)

#### clear()

Clears the trajectory start point

#### make_abs_pos(xyz, quat, ee_relative=False)

* **Return type:**
  `Optional`[[`Pose`](#easy_robot_control.leg_api.Pose)]
* **Parameters:**
  * **xyz** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*  *|* *None*)
  * **quat** (*quaternion* *|* *None*)
  * **ee_relative** (*bool* *|* *None*)

#### controlled_motion(xyz, quat, ee_relative=False)

Continuously calls self.step_toward(…) in the task timer.

Cancels the previous task and future.
Replaces it with a new function to execute and new future.
Returns the assiciated future (you can cancel it, and know when it’s done).

* **Parameters:**
  * **xyz** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*  *|* *None*) – target in mm
  * **quat** (*quaternion* *|* *None*) – target as quaternion
  * **ee_relative** (*bool* *|* *None*) – if the movement should bee performed relative to the end effector
* **Return type:**
  *Future*

Returns
: Future assiciated with the movement’s task.

* **Return type:**
  `Future`
* **Parameters:**
  * **xyz** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*  *|* *None*)
  * **quat** (*quaternion* *|* *None*)
  * **ee_relative** (*bool* *|* *None*)

#### step_toward(xyz, quat, ee_relative=False)

Sends a single ik command to move toward the target.
Not meant to reach the target!!!

This method is robust to IK errors and motor speed saturation. It will adapt its
speed according to the robot response to keep track with the path.

The command will clamp not farther from the current EE than self.sphere_xyz_radius
and self.sphere_quat_radius.
Increase those values to allow for a wider margin of error
(also leading to higher speed)

The math become imprecise with big deltas in quaternions. See clamp_xyz_quat(…)
Do not use if self.last_pose.quat is opposite to quat.
(idk what will happen but you wont like it)

* **Parameters:**
  * **xyz** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*  *|* *None*) – target in mm
  * **quat** (*quaternion* *|* *None*) – target as quaternion
  * **ee_relative** (*bool* *|* *None*) – if the movement should bee performed relative to the end effector
* **Return type:**
  `bool`

#### offset(xyz, quat, ee_relative=False)

Wrapper around self.step_toward(…),
Origin is placed onto the EE.
ee_relative specifies if the origin should have the same orientation as the EE

* **Parameters:**
  * **xyz** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*  *|* *None*) – mm to move by
  * **quat** (*quaternion* *|* *None*) – quaternion to move by
  * **ee_relative** (*bool* *|* *None*) – True: movement origin is the EE.
    False: movement origin is the baselink.

## easy_robot_control.leg_node module

This node is responsible for recieving targets with trajectories of the
corresponding leg. It will performe smooth movements in the body center frame of reference
or relative to the current end effector position.
This node keeps track of the leg end effector to generate trajectories to the target.

Author: Elian NEPPEL
Lab: SRL, Moonshot team

### *class* easy_robot_control.leg_node.LegNode

Bases: [`EliaNode`](#easy_robot_control.EliaNode.EliaNode)

#### firstSpinCBK()

#### publish_to_roll(roll=None)

publishes roll value towards ik node

* **Parameters:**
  **roll** (*float* *|* *None*)

#### smart_roll_cbk(msg)

* **Parameters:**
  **msg** (*Float64*)

#### publish_to_ik(xyz=None, quat=None)

publishes target towards ik node

* **Parameters:**
  * **target** – np.array float(3,)
  * **xyz** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
  * **quat** (*quaternion* *|* *None*)

#### trajectory_executor()

pops target from trajectory queue and publishes it.
This follows and handle the trajectory_timer

* **Return type:**
  `None`

#### trajectory_finished_cbk()

executes after the last target in the trajectory is processed

* **Return type:**
  `None`

#### queue_xyz_empty()

* **Return type:**
  `bool`

#### queue_quat_empty()

* **Return type:**
  `bool`

#### queue_roll_empty()

* **Return type:**
  `bool`

#### pop_xyzq_from_traj(index=0)

deletes and returns the first value of the trajectory_queue for coordinates
and quaternions.

* **Parameters:**
  **index** (*int*) – if you wanna pop not the first but somewhere else
* **Returns:**
  np.array float(3,) - popped value
* **Return type:**
  value
* **Return type:**
  `Tuple`[`Optional`[`ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]], `Optional`[`quaternion`]]

#### pop_roll_from_traj(index=0)

Deletes and returns the first value of the trajectory_queue for the roll.

* **Parameters:**
  **index** (*int*) – if you wanna pop not the first but somewhere else
* **Returns:**
  np.array float(3,) - popped value
* **Return type:**
  value
* **Return type:**
  `Optional`[`float`]

#### get_final_xyz()

* **Return type:**
  `ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]

#### get_final_quat()

* **Return type:**
  `quaternion`

#### get_final_target()

returns the final position of where the leg is. Or will be at the end of the
current trajectory_queue.

* **Returns:**
  np.array float(3,) - final coordinates
* **Return type:**
  end_point
* **Return type:**
  `Tuple`[`ndarray`[`Any`, `dtype`[`+_ScalarType_co`]], `quaternion`]

#### fuse_xyz_trajectory(xyz_traj=None)

* **Parameters:**
  **xyz_traj** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)

#### fuse_quat_trajectory(quat_traj=None)

* **Parameters:**
  **quat_traj** (*quaternion* *|* *None*)

#### fuse_roll_trajectory(roll_traj=None)

* **Parameters:**
  **roll_traj** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)

#### add_to_trajectory(xyz_traj=None, quat_traj=None)

Adds a trajectory RELATIVE TO THE BODY CENTER to the trajectory queue

* **Parameters:**
  * **new_traj** – np.array float(:, 3) - xyz trajectory RELATIVE TO THE BODY CENTER
  * **quat_traj** (*quaternion* *|* *None*) – qt.quaternion shape(:) - quaternion trajectory RELATIVE TO THE BODY CENTER
  * **xyz_traj** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
* **Return type:**
  `None`

#### send_most_recent_tip(request, response)

Publish the last target sent

* **Parameters:**
  * **request** (*ReturnVect3_Request*) – ReturnVect3.Request - Nothing
  * **response** (*ReturnVect3_Response*) – ReturnVect3.Response - Vector3 (float x3)
* **Returns:**
  ReturnVect3 - Vector3 (float x3)
* **Return type:**
  `ReturnVect3_Response`

#### smoother(x)

smoothes the interval [0, 1] to have a soft start and end
(derivative is zero)

* **Return type:**
  `ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]
* **Parameters:**
  **x** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)

#### smoother_complement(x)

changes the interval [0, 1] to 0->1->0
and smoothes to have a soft start and end

* **Return type:**
  `ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]
* **Parameters:**
  **x** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)

#### rel_transl(xyz=None, quat=None)

performs translation to the target relative to body

* **Parameters:**
  * **target** – np.array float(3,) - target relative to the body center
  * **quat** (*quaternion* *|* *None*) – qt.quaternion - target quaternion relative to body center
  * **xyz** (*ndarray* *|* *None*)
* **Returns:**
  np.array float(3,) - target relative to the body center
* **Return type:**
  target
* **Return type:**
  `int`

#### *static* point_with_quat(vect)

* **Parameters:**
  **vect** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)

#### point_toward(vect)

* **Return type:**
  `int`
* **Parameters:**
  **vect** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)

#### point_wheel(direction)

* **Return type:**
  `None`
* **Parameters:**
  **direction** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)

#### shift(shift=None, quat=None)

performs translation to the target relative to current position

* **Parameters:**
  * **target** – np.array float(3,) - target relative to current position
  * **shift** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
  * **quat** (*quaternion* *|* *None*)
* **Returns:**
  np.array float(3,) - target relative to current position
* **Return type:**
  target
* **Return type:**
  `int`

#### rel_hop(target)

performs jump to the target relative to body

* **Parameters:**
  **target** (*ndarray*) – np.array float(3,) - target relative to the body center
* **Returns:**
  np.array float(3,) - target relative to the body center
* **Return type:**
  target
* **Return type:**
  `ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]

#### tip_pos_received_cbk(msg)

callback when a new end effector position is received

* **Parameters:**
  **msg** (*Vector3*) – real end effector position
* **Return type:**
  `None`

#### check_divergence()

If the real end effector is not on the last target that was sent.
Launches the timer to overwrite the target with the real position.

If target and end effector correpsond, cancel the timer to overwrite the target.

* **Return type:**
  `None`

#### overwrite_target()

overwrites the last sent target with the real position of the end effector.

This will happen on startup, and when there is a problem on the real leg (cannot
perform the movement)
Small divergences are expected (in position and time), hence the timer and the
check_divergence function.

Setting lastTarget to the currentTip all the time is a bas idea, it create
overcorrections.

* **Return type:**
  `None`

#### wait_end_of_motion()

waits for the trajectory to end. This function is very bad, but I don’t need
nor have time to do something better.

We should keep track of which trajectory are beeing queued to improve

* **Return type:**
  `None`

#### append_trajectory(trajectory_function)

The function will be executed before the next read of the trajectory sequentialy
This avoids trajectory_function in multiple threads modifying indentical data simultaneously.

* **Parameters:**
  * **function** (*trajectory_function -*) – function to execute
  * **trajectory_function** (*Callable*)
* **Returns:**
  holds the future results of the function if needed
* **Return type:**
  future
* **Return type:**
  `Future`

#### shift_cbk(request, response)

Callback for leg shift motion

* **Parameters:**
  * **request** (*TFService_Request*) – target relative to current end effector position
  * **response** (*TFService_Response*)
* **Returns:**
  success = True all the time
* **Return type:**
  `TFService_Response`

#### rel_transl_srv_cbk(request, response)

Callback for leg translation motion

* **Parameters:**
  * **request** (*TFService_Request*) – target relative to body
  * **response** (*TFService_Response*)
* **Returns:**
  success = True all the time
* **Return type:**
  `TFService_Response`

#### rel_hop_srv_cbk(request, response)

Callback for leg hopping motion

* **Parameters:**
  * **request** (*TFService_Request*) – target relative to body
  * **response** (*TFService_Response*)
* **Returns:**
  success = True all the time
* **Return type:**
  `TFService_Response`

#### rot_cbk(request, response)

Callback for leg rotation motion

* **Parameters:**
  * **request** (*TFService_Request*) – TF for the rotation and center of the rotation
  * **response** (*TFService_Response*)
* **Returns:**
  success = True all the time
* **Return type:**
  `TFService_Response`

#### point_cbk(request, response)

* **Return type:**
  `TFService_Response`
* **Parameters:**
  * **request** (*TFService_Request*)
  * **response** (*TFService_Response*)

#### PARAM_REL_TOL *= 1e-06*

**Type:**    `float`

#### add_on_set_parameters_callback(callback)

Add a callback in front to the list of callbacks.

Calling this function will add a callback in self._parameter_callbacks list.

It is considered bad practice to reject changes for “unknown” parameters as this prevents
other parts of the node (that may be aware of these parameters) from handling them.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Return type:**
  `None`

#### add_waitable(waitable)

Add a class that is capable of adding things to the wait set.

* **Parameters:**
  **waitable** (`Waitable`) – An instance of a waitable that the node will add to the waitset.
* **Return type:**
  `None`

#### *property* clients

Get clients that have been created on this node.

* **Return type:**
  `Iterator`[`Client`]

#### *property* context

Get the context associated with the node.

* **Return type:**
  `Context`

#### count_publishers(topic_name)

Return the number of publishers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of publishers.
* **Return type:**
  `int`
* **Returns:**
  the number of publishers on the topic.

#### count_subscribers(topic_name)

Return the number of subscribers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of subscribers.
* **Return type:**
  `int`
* **Returns:**
  the number of subscribers on the topic.

#### create_EZrate(frequency, clock=None)

Creates a better rate where rate.destroy actually destroys the rate

* **Parameters:**
  * **frequency** (*float*) – frequency of the rate
  * **clock** (*Clock* *|* *None*) – clock to use
* **Returns:**
  EZRate manipulating a Rate object
* **Return type:**
  [`EZRate`](#easy_robot_control.EliaNode.EZRate)

#### create_client(srv_type, srv_name, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service client.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service client.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service client. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Client`

#### create_guard_condition(callback, callback_group=None)

Create a new guard condition.

* **Return type:**
  `GuardCondition`
* **Parameters:**
  * **callback** (*Callable*)
  * **callback_group** (*CallbackGroup* *|* *None*)

#### create_publisher(msg_type, topic, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, publisher_class=<class 'Publisher'>)

Create a new publisher.

* **Parameters:**
  * **msg_type** – The type of ROS messages the publisher will publish.
  * **topic** (`str`) – The name of the topic the publisher will publish to.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the publisher.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the publisher’s event handlers.
    If `None`, then the node’s default callback group is used.
  * **event_callbacks** (`Optional`[`PublisherEventCallbacks`]) – User-defined callbacks for middleware events.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
  * **publisher_class** (*Type* *[**Publisher* *]*)
* **Return type:**
  `Publisher`
* **Returns:**
  The new publisher.

#### create_rate(frequency, clock=None)

Create a Rate object.

* **Parameters:**
  * **frequency** (`float`) – The frequency the Rate runs at (Hz).
  * **clock** (`Optional`[`Clock`]) – The clock the Rate gets time from.
* **Return type:**
  `Rate`

#### create_service(srv_type, srv_name, callback, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service server.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **callback** (`Callable`[[`~SrvTypeRequest`, `~SrvTypeResponse`], `~SrvTypeResponse`]) – A user-defined callback function that is called when a service request
    received by the server.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service server.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service server. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Service`

#### create_subscription(msg_type, topic, callback, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, raw=False)

Create a new subscription.

* **Parameters:**
  * **msg_type** – The type of ROS messages the subscription will subscribe to.
  * **topic** (`str`) – The name of the topic the subscription will subscribe to.
  * **callback** (`Callable`[[`~MsgType`], `None`]) – A user-defined callback function that is called when a message is
    received by the subscription.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the subscription.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the subscription. If `None`, then the
    nodes default callback group is used.
  * **event_callbacks** (`Optional`[`SubscriptionEventCallbacks`]) – User-defined callbacks for middleware events.
  * **raw** (`bool`) – If `True`, then received messages will be stored in raw binary
    representation.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
* **Return type:**
  `Subscription`

#### create_timer(timer_period_sec, callback, callback_group=None, clock=None)

Create a new timer.

The timer will be started and every `timer_period_sec` number of seconds the provided
callback function will be called.

* **Parameters:**
  * **timer_period_sec** (`float`) – The period (s) of the timer.
  * **callback** (`Callable`) – A user-defined callback function that is called when the timer expires.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the timer. If `None`, then the nodes
    default callback group is used.
  * **clock** (`Optional`[`Clock`]) – The clock which the timer gets time from.
* **Return type:**
  `Timer`

#### declare_parameter(name, value=None, descriptor=None, ignore_override=False)

Declare and initialize a parameter.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.leg_node.LegNode.add_on_set_parameters_callback) to be called.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **value** (`Optional`[`Any`]) – Value of the parameter to declare.
  * **descriptor** (`Optional`[`ParameterDescriptor`]) – Descriptor for the parameter to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `Parameter`
* **Returns:**
  Parameter with the effectively assigned value.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects the parameter.

#### declare_parameters(namespace, parameters, ignore_override=False)

Declare a list of parameters.

The tuples in the given parameter list shall contain the name for each parameter,
optionally providing a value and a descriptor.
For each entry in the list, a parameter with a name of “namespace.name”
will be declared.
The resulting value for each declared parameter will be returned, considering
parameter overrides set upon node creation as the first choice,
or provided parameter values as the second one.

The name expansion is naive, so if you set the namespace to be “foo.”,
then the resulting parameter names will be like “foo..name”.
However, if the namespace is an empty string, then no leading ‘.’ will be
placed before each name, which would have been the case when naively
expanding “namespace.name”.
This allows you to declare several parameters at once without a namespace.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.leg_node.LegNode.add_on_set_parameters_callback) to be called once for each parameter.
If one of those calls fail, an exception will be raised and the remaining parameters will
not be declared.
Parameters declared up to that point will not be undeclared.

* **Parameters:**
  * **namespace** (`str`) – Namespace for parameters.
  * **parameters** (`List`[`Union`[`Tuple`[`str`], `Tuple`[`str`, `Type`], `Tuple`[`str`, `Any`, `ParameterDescriptor`]]]) – List of tuples with parameters to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  Parameter list with the effectively assigned values for each of them.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects any parameter.
* **Raises:**
  TypeError if any tuple in **parameters** does not match the annotated type.

#### *property* default_callback_group

Get the default callback group.

If no other callback group is provided when the a ROS entity is created with the node,
then it is added to the default callback group.

* **Return type:**
  `CallbackGroup`

#### describe_parameter(name)

Get the parameter descriptor of a given parameter.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `ParameterDescriptor`
* **Returns:**
  ParameterDescriptor corresponding to the parameter,
  or default ParameterDescriptor if parameter had not been declared before
  and undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.

#### describe_parameters(names)

Get the parameter descriptors of a given list of parameters.

* **Parameters:**
  * **name** – List of fully-qualified names of the parameters to describe.
  * **names** (*List* *[**str* *]*)
* **Return type:**
  `List`[`ParameterDescriptor`]
* **Returns:**
  List of ParameterDescriptors corresponding to the given parameters.
  Default ParameterDescriptors shall be returned for parameters that
  had not been declared before if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if at least one parameter
  had not been declared before and undeclared parameters are not allowed.

#### destroy_client(client)

Destroy a service client created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **client** (*Client*)

#### destroy_guard_condition(guard)

Destroy a guard condition created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **guard** (*GuardCondition*)

#### destroy_node()

Destroy the node.

Frees resources used by the node, including any entities created by the following methods:

* [`create_publisher()`](#easy_robot_control.leg_node.LegNode.create_publisher)
* [`create_subscription()`](#easy_robot_control.leg_node.LegNode.create_subscription)
* [`create_client()`](#easy_robot_control.leg_node.LegNode.create_client)
* [`create_service()`](#easy_robot_control.leg_node.LegNode.create_service)
* [`create_timer()`](#easy_robot_control.leg_node.LegNode.create_timer)
* [`create_guard_condition()`](#easy_robot_control.leg_node.LegNode.create_guard_condition)

#### destroy_publisher(publisher)

Destroy a publisher created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **publisher** (*Publisher*)

#### destroy_rate(rate)

Destroy a Rate object created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **rate** (*Rate*)

#### destroy_service(service)

Destroy a service server created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **service** (*Service*)

#### destroy_subscription(subscription)

Destroy a subscription created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if succesful, `False` otherwise.
* **Parameters:**
  **subscription** (*Subscription*)

#### destroy_timer(timer)

Destroy a timer created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **timer** (*Timer*)

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

#### *property* executor

Get the executor if the node has been added to one, else return `None`.

* **Return type:**
  `Optional`[`Executor`]

#### getNow()

quick: self.get_clock().now()

* **Return type:**
  `Time`

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

#### get_client_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service client topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get service clients for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The fist element of each tuple is the service client name
  and the second element is a list of service client types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_clock()

Get the clock used by the node.

* **Return type:**
  `Clock`

#### get_fully_qualified_name()

Get the node’s fully qualified name.

* **Return type:**
  `str`
* **Returns:**
  Fully qualified node name.

#### get_logger()

Get the nodes logger.

#### get_name()

Get the name of the node.

* **Return type:**
  `str`

#### get_namespace()

Get the namespace of the node.

* **Return type:**
  `str`

#### get_node_names()

Get a list of names for discovered nodes.

* **Return type:**
  `List`[`str`]
* **Returns:**
  List of node names.

#### get_node_names_and_namespaces()

Get a list of names and namespaces for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`]]
* **Returns:**
  List of tuples containing two strings: the node name and node namespace.

#### get_node_names_and_namespaces_with_enclaves()

Get a list of names, namespaces and enclaves for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`, `str`]]
* **Returns:**
  List of tuples containing three strings: the node name, node namespace
  and enclave.

#### get_parameter(name)

Get a parameter by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Parameter`
* **Returns:**
  The value for the given parameter name.
  A default Parameter will be returned for an undeclared parameter if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if the parameter is statically typed and
  uninitialized.

#### get_parameter_or(name, alternative_value=None)

Get a parameter or the alternative value.

If the alternative value is None, a default Parameter with the given name and NOT_SET
type will be returned if the parameter was not declared.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **alternative_value** (`Optional`[`Parameter`]) – Alternative parameter to get if it had not been declared before.
* **Return type:**
  `Parameter`
* **Returns:**
  Requested parameter, or alternative value if it hadn’t been declared before or is
  an uninitialized statically typed parameter.

#### get_parameter_type(name)

Get a parameter type by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Type`
* **Returns:**
  The type for the given parameter name.
  A default Parameter.Type.NOT_SET will be returned for an undeclared parameter
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.

#### get_parameter_types(names)

Get a list of parameter types.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Type`]
* **Returns:**
  The values for the given parameter types.
  A default Parameter.Type.NOT_SET will be returned for undeclared parameters
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.

#### get_parameters(names)

Get a list of parameters.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  The values for the given parameter names.
  A default Parameter will be returned for undeclared parameters if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if at least one parameter is statically typed and
  uninitialized.

#### get_parameters_by_prefix(prefix)

Get parameters that have a given prefix in their names as a dictionary.

The names which are used as keys in the returned dictionary have the prefix removed.
For example, if you use the prefix “foo” and the parameters “foo.ping”, “foo.pong”
and “bar.baz” exist, then the returned dictionary will have the keys “ping” and “pong”.
Note that the parameter separator is also removed from the parameter name to create the
keys.

An empty string for the prefix will match all parameters.

If no parameters with the prefix are found, an empty dictionary will be returned.

* **Parameters:**
  **prefix** (`str`) – The prefix of the parameters to get.
* **Return type:**
  `Dict`[`str`, `Union`[`bool`, `int`, `float`, `str`, `bytes`, `Sequence`[`bool`], `Sequence`[`int`], `Sequence`[`float`], `Sequence`[`str`], `None`]]
* **Returns:**
  Dict of parameters with the given prefix.

#### get_publisher_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for publishers of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get publishers for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_publishers_info_by_topic(topic_name, no_mangle=False)

Return a list of publishers on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the publishers.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the publishers on this topic.

#### get_service_names_and_types()

Get a list of service topics for the node.

* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service name and the second element is a list of
  service types.

#### get_service_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service server topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get services for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service server name
  and the second element is a list of service types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriber_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for subscriptions of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get subscriptions for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriptions_info_by_topic(topic_name, no_mangle=False)

Return a list of subscriptions on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the subscriptions.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the subscriptions on this topic.

#### get_topic_names_and_types(no_demangle=False)

Get a list topic names and types for the node.

* **Parameters:**
  **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.

#### *property* guards

Get guards that have been created on this node.

* **Return type:**
  `Iterator`[`GuardCondition`]

#### *property* handle

Get the handle to the underlying rcl_node_t.

Cannot be modified after node creation.

* **Raises:**
  AttributeError if modified after creation.

#### has_parameter(name)

Return True if parameter is declared; False otherwise.

* **Return type:**
  `bool`
* **Parameters:**
  **name** (*str*)

#### perror(object, force=False)

Prints/Logs error if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** (*Any*) – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### pinfo(object, force=False)

Prints/Logs info if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **force** (*bool*) – if True the message will print whatever if self.Yapping is.

#### *property* publishers

Get publishers that have been created on this node.

* **Return type:**
  `Iterator`[`Publisher`]

#### pwarn(object, force=False)

Prints/Logs warning if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### remove_on_set_parameters_callback(callback)

Remove a callback from list of callbacks.

Calling this function will remove the callback from self._parameter_callbacks list.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Raises:**
  ValueError if a callback is not present in the list of callbacks.
* **Return type:**
  `None`

#### remove_waitable(waitable)

Remove a Waitable that was previously added to the node.

* **Parameters:**
  **waitable** (`Waitable`) – The Waitable to remove.
* **Return type:**
  `None`

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

#### resolve_topic_name(topic, \*, only_expand=False)

Return a topic name expanded and remapped.

* **Parameters:**
  * **topic** (`str`) – topic name to be expanded and remapped.
  * **only_expand** (`bool`) – if True, remapping rules won’t be applied.
* **Return type:**
  `str`
* **Returns:**
  a fully qualified topic name,
  result of applying expansion and remapping to the given topic.

#### *property* services

Get services that have been created on this node.

* **Return type:**
  `Iterator`[`Service`]

#### setAndBlockForNecessaryNodes(necessary_node_names, silent_trial=3, intervalSec=0.5)

Blocks for nodes to be alive

* **Parameters:**
  * **necessary_node_names** (*List* *[**str* *]*  *|* *str*)
  * **silent_trial** (*int* *|* *None*)
  * **intervalSec** (*float* *|* *None*)

#### set_descriptor(name, descriptor, alternative_value=None)

Set a new descriptor for a given parameter.

The name in the descriptor is ignored and set to **name**.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter to set the descriptor to.
  * **descriptor** (`ParameterDescriptor`) – New descriptor to apply to the parameter.
  * **alternative_value** (`Optional`[`ParameterValue`]) – Value to set to the parameter if the existing value does not
    comply with the new descriptor.
* **Return type:**
  `ParameterValue`
* **Returns:**
  ParameterValue for the given parameter name after applying the new descriptor.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.
* **Raises:**
  ParameterImmutableException if the parameter exists and is read-only.
* **Raises:**
  ParameterValueException if neither the existing value nor the alternative value
  complies with the provided descriptor.

#### set_parameters(parameter_list)

Set parameters for the node, and return the result for the set action.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set in the order they are declared in the list.
If setting a parameter fails due to not being declared, then the
parameters which have already been set will stay set, and no attempt will
be made to set the parameters which come after.

If undeclared parameters are allowed, then all the parameters will be implicitly
declared before being set even if they were not declared beforehand.
Parameter overrides are ignored by this method.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.leg_node.LegNode.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node, once for each parameter.
If the callback prevents a parameter from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is
published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `List`[`SetParametersResult`]
* **Returns:**
  The result for each set action as a list.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### set_parameters_atomically(parameter_list)

Set the given parameters, all at one time, and then aggregate result.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set all at once.
If setting a parameter fails due to not being declared, then no parameter will be set set.
Either all of the parameters are set or none of them are set.

If undeclared parameters are allowed for the node, then all the parameters will be
implicitly declared before being set even if they were not declared beforehand.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.leg_node.LegNode.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node only once for all parameters.
If the callback prevents the parameters from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `SetParametersResult`
* **Returns:**
  Aggregate result of setting all the parameters atomically.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### sleep(seconds)

sleeps using the node’s clock.

#### NOTE
Special case for good old foxy

* **Parameters:**
  **seconds** (*float*) – time to sleep
* **Return type:**
  `None`

#### *property* subscriptions

Get subscriptions that have been created on this node.

* **Return type:**
  `Iterator`[`Subscription`]

#### *property* timers

Get timers that have been created on this node.

* **Return type:**
  `Iterator`[`Timer`]

#### undeclare_parameter(name)

Undeclare a previously declared parameter.

This method will not cause a callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.leg_node.LegNode.add_on_set_parameters_callback) to be called.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before.
* **Raises:**
  ParameterImmutableException if the parameter was created as read-only.

#### wait_for_lower_level(more_services={}, all_requiered=False)

Blocks until all or any service is available.

#### NOTE
- List of waited services is given by services_to_wait ros2 param
- Overload this to wait for an additional service

* **Parameters:**
  * **more_services** (*Iterable* *[**str* *]*) – add more services
  * **all_requiered** (*bool*) – if True, all listed services must be available

#### wait_on_futures(future_list, wait_Hz=10)

Waits for the completion of a list of futures, checking completion at the
provided rate.

* **Parameters:**
  * **future_list** (*List* *[**Future* *]*  *|* *Future*) – List of Future to wait for
  * **wait_Hz** (*float*) – rate at which to wait

#### *property* waitables

Get waitables that have been created on this node.

* **Return type:**
  `Iterator`[`Waitable`]

### easy_robot_control.leg_node.main(args=None)

## easy_robot_control.mover_node module

This node is responsible for synchronising several leg movement in order to move the
cente body and perform steps.

Author: Elian NEPPEL
Lab: SRL, Moonshot team

### *class* easy_robot_control.mover_node.MoverNode

Bases: [`EliaNode`](#easy_robot_control.EliaNode.EliaNode)

#### firstSpinCBK()

* **Return type:**
  `None`

#### go2_targetbodyCBK(req, res)

* **Return type:**
  `SendTargetBody_Response`
* **Parameters:**
  * **req** (*SendTargetBody_Request*)
  * **res** (*SendTargetBody_Response*)

#### get_targetsetCBK(req, res)

* **Return type:**
  `ReturnTargetSet_Response`
* **Parameters:**
  * **req** (*ReturnTargetSet_Request*)
  * **res** (*ReturnTargetSet_Response*)

#### update_tip_pos()

* **Return type:**
  `ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]

#### body_shift(shift)

* **Return type:**
  `None`
* **Parameters:**
  **shift** (*ndarray*)

#### body_tfshift_cbk(request, response)

* **Return type:**
  `TFService_Response`
* **Parameters:**
  * **request** (*TFService_Request*)
  * **response** (*TFService_Response*)

#### multi_transl(target_set)

* **Parameters:**
  **target_set** (*ndarray*)

#### multi_hop(target_set)

* **Parameters:**
  **target_set** (*ndarray*)

#### multi_shift(target_set)

* **Parameters:**
  **target_set** (*ndarray*)

#### multi_rotate(target_set, quat)

* **Parameters:**
  * **target_set** (*ndarray*)
  * **quat** (*quaternion*)

#### move_body_and_hop(body_xyz, target_set, body_quat=None)

* **Parameters:**
  * **body_xyz** (*ndarray*)
  * **target_set** (*ndarray*)
  * **body_quat** (*quaternion* *|* *None*)

#### PARAM_REL_TOL *= 1e-06*

**Type:**    `float`

#### add_on_set_parameters_callback(callback)

Add a callback in front to the list of callbacks.

Calling this function will add a callback in self._parameter_callbacks list.

It is considered bad practice to reject changes for “unknown” parameters as this prevents
other parts of the node (that may be aware of these parameters) from handling them.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Return type:**
  `None`

#### add_waitable(waitable)

Add a class that is capable of adding things to the wait set.

* **Parameters:**
  **waitable** (`Waitable`) – An instance of a waitable that the node will add to the waitset.
* **Return type:**
  `None`

#### *property* clients

Get clients that have been created on this node.

* **Return type:**
  `Iterator`[`Client`]

#### *property* context

Get the context associated with the node.

* **Return type:**
  `Context`

#### count_publishers(topic_name)

Return the number of publishers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of publishers.
* **Return type:**
  `int`
* **Returns:**
  the number of publishers on the topic.

#### count_subscribers(topic_name)

Return the number of subscribers on a given topic.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic is expanded using this node’s namespace and name.
The queried topic name is not remapped.

* **Parameters:**
  **topic_name** (`str`) – the topic_name on which to count the number of subscribers.
* **Return type:**
  `int`
* **Returns:**
  the number of subscribers on the topic.

#### create_EZrate(frequency, clock=None)

Creates a better rate where rate.destroy actually destroys the rate

* **Parameters:**
  * **frequency** (*float*) – frequency of the rate
  * **clock** (*Clock* *|* *None*) – clock to use
* **Returns:**
  EZRate manipulating a Rate object
* **Return type:**
  [`EZRate`](#easy_robot_control.EliaNode.EZRate)

#### create_client(srv_type, srv_name, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service client.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service client.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service client. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Client`

#### create_guard_condition(callback, callback_group=None)

Create a new guard condition.

* **Return type:**
  `GuardCondition`
* **Parameters:**
  * **callback** (*Callable*)
  * **callback_group** (*CallbackGroup* *|* *None*)

#### create_publisher(msg_type, topic, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, publisher_class=<class 'Publisher'>)

Create a new publisher.

* **Parameters:**
  * **msg_type** – The type of ROS messages the publisher will publish.
  * **topic** (`str`) – The name of the topic the publisher will publish to.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the publisher.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the publisher’s event handlers.
    If `None`, then the node’s default callback group is used.
  * **event_callbacks** (`Optional`[`PublisherEventCallbacks`]) – User-defined callbacks for middleware events.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
  * **publisher_class** (*Type* *[**Publisher* *]*)
* **Return type:**
  `Publisher`
* **Returns:**
  The new publisher.

#### create_rate(frequency, clock=None)

Create a Rate object.

* **Parameters:**
  * **frequency** (`float`) – The frequency the Rate runs at (Hz).
  * **clock** (`Optional`[`Clock`]) – The clock the Rate gets time from.
* **Return type:**
  `Rate`

#### create_service(srv_type, srv_name, callback, \*, qos_profile=<rclpy.qos.QoSProfile object>, callback_group=None)

Create a new service server.

* **Parameters:**
  * **srv_type** – The service type.
  * **srv_name** (`str`) – The name of the service.
  * **callback** (`Callable`[[`~SrvTypeRequest`, `~SrvTypeResponse`], `~SrvTypeResponse`]) – A user-defined callback function that is called when a service request
    received by the server.
  * **qos_profile** (`QoSProfile`) – The quality of service profile to apply the service server.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the service server. If `None`, then the
    nodes default callback group is used.
* **Return type:**
  `Service`

#### create_subscription(msg_type, topic, callback, qos_profile, \*, callback_group=None, event_callbacks=None, qos_overriding_options=None, raw=False)

Create a new subscription.

* **Parameters:**
  * **msg_type** – The type of ROS messages the subscription will subscribe to.
  * **topic** (`str`) – The name of the topic the subscription will subscribe to.
  * **callback** (`Callable`[[`~MsgType`], `None`]) – A user-defined callback function that is called when a message is
    received by the subscription.
  * **qos_profile** (`Union`[`QoSProfile`, `int`]) – A QoSProfile or a history depth to apply to the subscription.
    In the case that a history depth is provided, the QoS history is set to
    KEEP_LAST, the QoS history depth is set to the value
    of the parameter, and all other QoS settings are set to their default values.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the subscription. If `None`, then the
    nodes default callback group is used.
  * **event_callbacks** (`Optional`[`SubscriptionEventCallbacks`]) – User-defined callbacks for middleware events.
  * **raw** (`bool`) – If `True`, then received messages will be stored in raw binary
    representation.
  * **qos_overriding_options** (*QoSOverridingOptions* *|* *None*)
* **Return type:**
  `Subscription`

#### create_timer(timer_period_sec, callback, callback_group=None, clock=None)

Create a new timer.

The timer will be started and every `timer_period_sec` number of seconds the provided
callback function will be called.

* **Parameters:**
  * **timer_period_sec** (`float`) – The period (s) of the timer.
  * **callback** (`Callable`) – A user-defined callback function that is called when the timer expires.
  * **callback_group** (`Optional`[`CallbackGroup`]) – The callback group for the timer. If `None`, then the nodes
    default callback group is used.
  * **clock** (`Optional`[`Clock`]) – The clock which the timer gets time from.
* **Return type:**
  `Timer`

#### declare_parameter(name, value=None, descriptor=None, ignore_override=False)

Declare and initialize a parameter.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.mover_node.MoverNode.add_on_set_parameters_callback) to be called.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **value** (`Optional`[`Any`]) – Value of the parameter to declare.
  * **descriptor** (`Optional`[`ParameterDescriptor`]) – Descriptor for the parameter to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `Parameter`
* **Returns:**
  Parameter with the effectively assigned value.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects the parameter.

#### declare_parameters(namespace, parameters, ignore_override=False)

Declare a list of parameters.

The tuples in the given parameter list shall contain the name for each parameter,
optionally providing a value and a descriptor.
For each entry in the list, a parameter with a name of “namespace.name”
will be declared.
The resulting value for each declared parameter will be returned, considering
parameter overrides set upon node creation as the first choice,
or provided parameter values as the second one.

The name expansion is naive, so if you set the namespace to be “foo.”,
then the resulting parameter names will be like “foo..name”.
However, if the namespace is an empty string, then no leading ‘.’ will be
placed before each name, which would have been the case when naively
expanding “namespace.name”.
This allows you to declare several parameters at once without a namespace.

This method, if successful, will result in any callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.mover_node.MoverNode.add_on_set_parameters_callback) to be called once for each parameter.
If one of those calls fail, an exception will be raised and the remaining parameters will
not be declared.
Parameters declared up to that point will not be undeclared.

* **Parameters:**
  * **namespace** (`str`) – Namespace for parameters.
  * **parameters** (`List`[`Union`[`Tuple`[`str`], `Tuple`[`str`, `Type`], `Tuple`[`str`, `Any`, `ParameterDescriptor`]]]) – List of tuples with parameters to declare.
  * **ignore_override** (`bool`) – True if overrides shall not be taken into account; False otherwise.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  Parameter list with the effectively assigned values for each of them.
* **Raises:**
  ParameterAlreadyDeclaredException if the parameter had already been declared.
* **Raises:**
  InvalidParameterException if the parameter name is invalid.
* **Raises:**
  InvalidParameterValueException if the registered callback rejects any parameter.
* **Raises:**
  TypeError if any tuple in **parameters** does not match the annotated type.

#### *property* default_callback_group

Get the default callback group.

If no other callback group is provided when the a ROS entity is created with the node,
then it is added to the default callback group.

* **Return type:**
  `CallbackGroup`

#### describe_parameter(name)

Get the parameter descriptor of a given parameter.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `ParameterDescriptor`
* **Returns:**
  ParameterDescriptor corresponding to the parameter,
  or default ParameterDescriptor if parameter had not been declared before
  and undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.

#### describe_parameters(names)

Get the parameter descriptors of a given list of parameters.

* **Parameters:**
  * **name** – List of fully-qualified names of the parameters to describe.
  * **names** (*List* *[**str* *]*)
* **Return type:**
  `List`[`ParameterDescriptor`]
* **Returns:**
  List of ParameterDescriptors corresponding to the given parameters.
  Default ParameterDescriptors shall be returned for parameters that
  had not been declared before if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if at least one parameter
  had not been declared before and undeclared parameters are not allowed.

#### destroy_client(client)

Destroy a service client created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **client** (*Client*)

#### destroy_guard_condition(guard)

Destroy a guard condition created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **guard** (*GuardCondition*)

#### destroy_node()

Destroy the node.

Frees resources used by the node, including any entities created by the following methods:

* [`create_publisher()`](#easy_robot_control.mover_node.MoverNode.create_publisher)
* [`create_subscription()`](#easy_robot_control.mover_node.MoverNode.create_subscription)
* [`create_client()`](#easy_robot_control.mover_node.MoverNode.create_client)
* [`create_service()`](#easy_robot_control.mover_node.MoverNode.create_service)
* [`create_timer()`](#easy_robot_control.mover_node.MoverNode.create_timer)
* [`create_guard_condition()`](#easy_robot_control.mover_node.MoverNode.create_guard_condition)

#### destroy_publisher(publisher)

Destroy a publisher created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **publisher** (*Publisher*)

#### destroy_rate(rate)

Destroy a Rate object created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **rate** (*Rate*)

#### destroy_service(service)

Destroy a service server created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **service** (*Service*)

#### destroy_subscription(subscription)

Destroy a subscription created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if succesful, `False` otherwise.
* **Parameters:**
  **subscription** (*Subscription*)

#### destroy_timer(timer)

Destroy a timer created by the node.

* **Return type:**
  `bool`
* **Returns:**
  `True` if successful, `False` otherwise.
* **Parameters:**
  **timer** (*Timer*)

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

#### *property* executor

Get the executor if the node has been added to one, else return `None`.

* **Return type:**
  `Optional`[`Executor`]

#### getNow()

quick: self.get_clock().now()

* **Return type:**
  `Time`

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

#### get_client_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service client topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get service clients for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The fist element of each tuple is the service client name
  and the second element is a list of service client types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_clock()

Get the clock used by the node.

* **Return type:**
  `Clock`

#### get_fully_qualified_name()

Get the node’s fully qualified name.

* **Return type:**
  `str`
* **Returns:**
  Fully qualified node name.

#### get_logger()

Get the nodes logger.

#### get_name()

Get the name of the node.

* **Return type:**
  `str`

#### get_namespace()

Get the namespace of the node.

* **Return type:**
  `str`

#### get_node_names()

Get a list of names for discovered nodes.

* **Return type:**
  `List`[`str`]
* **Returns:**
  List of node names.

#### get_node_names_and_namespaces()

Get a list of names and namespaces for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`]]
* **Returns:**
  List of tuples containing two strings: the node name and node namespace.

#### get_node_names_and_namespaces_with_enclaves()

Get a list of names, namespaces and enclaves for discovered nodes.

* **Return type:**
  `List`[`Tuple`[`str`, `str`, `str`]]
* **Returns:**
  List of tuples containing three strings: the node name, node namespace
  and enclave.

#### get_parameter(name)

Get a parameter by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Parameter`
* **Returns:**
  The value for the given parameter name.
  A default Parameter will be returned for an undeclared parameter if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if the parameter is statically typed and
  uninitialized.

#### get_parameter_or(name, alternative_value=None)

Get a parameter or the alternative value.

If the alternative value is None, a default Parameter with the given name and NOT_SET
type will be returned if the parameter was not declared.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
  * **alternative_value** (`Optional`[`Parameter`]) – Alternative parameter to get if it had not been declared before.
* **Return type:**
  `Parameter`
* **Returns:**
  Requested parameter, or alternative value if it hadn’t been declared before or is
  an uninitialized statically typed parameter.

#### get_parameter_type(name)

Get a parameter type by name.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Return type:**
  `Type`
* **Returns:**
  The type for the given parameter name.
  A default Parameter.Type.NOT_SET will be returned for an undeclared parameter
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and the parameter hadn’t been declared beforehand.

#### get_parameter_types(names)

Get a list of parameter types.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Type`]
* **Returns:**
  The values for the given parameter types.
  A default Parameter.Type.NOT_SET will be returned for undeclared parameters
  if undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.

#### get_parameters(names)

Get a list of parameters.

* **Parameters:**
  **names** (`List`[`str`]) – Fully-qualified names of the parameters to get, including their namespaces.
* **Return type:**
  `List`[`Parameter`]
* **Returns:**
  The values for the given parameter names.
  A default Parameter will be returned for undeclared parameters if
  undeclared parameters are allowed.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter hadn’t been declared beforehand.
* **Raises:**
  ParameterUninitializedException if at least one parameter is statically typed and
  uninitialized.

#### get_parameters_by_prefix(prefix)

Get parameters that have a given prefix in their names as a dictionary.

The names which are used as keys in the returned dictionary have the prefix removed.
For example, if you use the prefix “foo” and the parameters “foo.ping”, “foo.pong”
and “bar.baz” exist, then the returned dictionary will have the keys “ping” and “pong”.
Note that the parameter separator is also removed from the parameter name to create the
keys.

An empty string for the prefix will match all parameters.

If no parameters with the prefix are found, an empty dictionary will be returned.

* **Parameters:**
  **prefix** (`str`) – The prefix of the parameters to get.
* **Return type:**
  `Dict`[`str`, `Union`[`bool`, `int`, `float`, `str`, `bytes`, `Sequence`[`bool`], `Sequence`[`int`], `Sequence`[`float`], `Sequence`[`str`], `None`]]
* **Returns:**
  Dict of parameters with the given prefix.

#### get_publisher_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for publishers of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get publishers for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_publishers_info_by_topic(topic_name, no_mangle=False)

Return a list of publishers on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the publishers.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the publishers on this topic.

#### get_service_names_and_types()

Get a list of service topics for the node.

* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service name and the second element is a list of
  service types.

#### get_service_names_and_types_by_node(node_name, node_namespace)

Get a list of discovered service server topics for a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get services for.
  * **node_namespace** (`str`) – Namespace of the remote node.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the service server name
  and the second element is a list of service types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriber_names_and_types_by_node(node_name, node_namespace, no_demangle=False)

Get a list of discovered topics for subscriptions of a remote node.

* **Parameters:**
  * **node_name** (`str`) – Name of a remote node to get subscriptions for.
  * **node_namespace** (`str`) – Namespace of the remote node.
  * **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.
* **Raises:**
  * **NodeNameNonExistentError** – If the node wasn’t found.
  * **RuntimeError** – Unexpected failure.

#### get_subscriptions_info_by_topic(topic_name, no_mangle=False)

Return a list of subscriptions on a given topic.

The returned parameter is a list of TopicEndpointInfo objects, where each will contain
the node name, node namespace, topic type, topic endpoint’s GID, and its QoS profile.

When the no_mangle parameter is true, the provided topic_name should be a valid topic
name for the middleware (useful when combining ROS with native middleware (e.g. DDS) apps).
When the no_mangle parameter is false, the provided topic_name should follow
ROS topic name conventions.

topic_name may be a relative, private, or fully qualified topic name.
A relative or private topic will be expanded using this node’s namespace and name.
The queried topic_name is not remapped.

* **Parameters:**
  * **topic_name** (`str`) – the topic_name on which to find the subscriptions.
  * **no_mangle** (`bool`) – no_mangle if true, topic_name needs to be a valid middleware topic
    name, otherwise it should be a valid ROS topic name. Defaults to false.
* **Return type:**
  `List`[`TopicEndpointInfo`]
* **Returns:**
  a list of TopicEndpointInfo for all the subscriptions on this topic.

#### get_topic_names_and_types(no_demangle=False)

Get a list topic names and types for the node.

* **Parameters:**
  **no_demangle** (`bool`) – If `True`, then topic names and types returned will not be demangled.
* **Return type:**
  `List`[`Tuple`[`str`, `List`[`str`]]]
* **Returns:**
  List of tuples.
  The first element of each tuple is the topic name and the second element is a list of
  topic types.

#### *property* guards

Get guards that have been created on this node.

* **Return type:**
  `Iterator`[`GuardCondition`]

#### *property* handle

Get the handle to the underlying rcl_node_t.

Cannot be modified after node creation.

* **Raises:**
  AttributeError if modified after creation.

#### has_parameter(name)

Return True if parameter is declared; False otherwise.

* **Return type:**
  `bool`
* **Parameters:**
  **name** (*str*)

#### perror(object, force=False)

Prints/Logs error if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** (*Any*) – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### pinfo(object, force=False)

Prints/Logs info if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **force** (*bool*) – if True the message will print whatever if self.Yapping is.

#### *property* publishers

Get publishers that have been created on this node.

* **Return type:**
  `Iterator`[`Publisher`]

#### pwarn(object, force=False)

Prints/Logs warning if Yapping==True (default) or force==True.

* **Parameters:**
  * **object** – Thing to print
  * **bool** (*force -*) – if True the message will print whatever if self.Yapping is.
  * **force** (*bool*)

#### remove_on_set_parameters_callback(callback)

Remove a callback from list of callbacks.

Calling this function will remove the callback from self._parameter_callbacks list.

* **Parameters:**
  **callback** (`Callable`[[`List`[`Parameter`]], `SetParametersResult`]) – The function that is called whenever parameters are set for the node.
* **Raises:**
  ValueError if a callback is not present in the list of callbacks.
* **Return type:**
  `None`

#### remove_waitable(waitable)

Remove a Waitable that was previously added to the node.

* **Parameters:**
  **waitable** (`Waitable`) – The Waitable to remove.
* **Return type:**
  `None`

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

#### resolve_topic_name(topic, \*, only_expand=False)

Return a topic name expanded and remapped.

* **Parameters:**
  * **topic** (`str`) – topic name to be expanded and remapped.
  * **only_expand** (`bool`) – if True, remapping rules won’t be applied.
* **Return type:**
  `str`
* **Returns:**
  a fully qualified topic name,
  result of applying expansion and remapping to the given topic.

#### *property* services

Get services that have been created on this node.

* **Return type:**
  `Iterator`[`Service`]

#### setAndBlockForNecessaryNodes(necessary_node_names, silent_trial=3, intervalSec=0.5)

Blocks for nodes to be alive

* **Parameters:**
  * **necessary_node_names** (*List* *[**str* *]*  *|* *str*)
  * **silent_trial** (*int* *|* *None*)
  * **intervalSec** (*float* *|* *None*)

#### set_descriptor(name, descriptor, alternative_value=None)

Set a new descriptor for a given parameter.

The name in the descriptor is ignored and set to **name**.

* **Parameters:**
  * **name** (`str`) – Fully-qualified name of the parameter to set the descriptor to.
  * **descriptor** (`ParameterDescriptor`) – New descriptor to apply to the parameter.
  * **alternative_value** (`Optional`[`ParameterValue`]) – Value to set to the parameter if the existing value does not
    comply with the new descriptor.
* **Return type:**
  `ParameterValue`
* **Returns:**
  ParameterValue for the given parameter name after applying the new descriptor.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before
  and undeclared parameters are not allowed.
* **Raises:**
  ParameterImmutableException if the parameter exists and is read-only.
* **Raises:**
  ParameterValueException if neither the existing value nor the alternative value
  complies with the provided descriptor.

#### set_parameters(parameter_list)

Set parameters for the node, and return the result for the set action.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set in the order they are declared in the list.
If setting a parameter fails due to not being declared, then the
parameters which have already been set will stay set, and no attempt will
be made to set the parameters which come after.

If undeclared parameters are allowed, then all the parameters will be implicitly
declared before being set even if they were not declared beforehand.
Parameter overrides are ignored by this method.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.mover_node.MoverNode.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node, once for each parameter.
If the callback prevents a parameter from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is
published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `List`[`SetParametersResult`]
* **Returns:**
  The result for each set action as a list.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### set_parameters_atomically(parameter_list)

Set the given parameters, all at one time, and then aggregate result.

If any parameter in the list was not declared beforehand and undeclared parameters are not
allowed for the node, this method will raise a ParameterNotDeclaredException exception.

Parameters are set all at once.
If setting a parameter fails due to not being declared, then no parameter will be set set.
Either all of the parameters are set or none of them are set.

If undeclared parameters are allowed for the node, then all the parameters will be
implicitly declared before being set even if they were not declared beforehand.

If a callback was registered previously with [`add_on_set_parameters_callback()`](#easy_robot_control.mover_node.MoverNode.add_on_set_parameters_callback), it
will be called prior to setting the parameters for the node only once for all parameters.
If the callback prevents the parameters from being set, then it will be reflected in the
returned result; no exceptions will be raised in this case.
For each successfully set parameter, a `ParameterEvent` message is published.

If the value type of the parameter is NOT_SET, and the existing parameter type is
something else, then the parameter will be implicitly undeclared.

* **Parameters:**
  **parameter_list** (`List`[`Parameter`]) – The list of parameters to set.
* **Return type:**
  `SetParametersResult`
* **Returns:**
  Aggregate result of setting all the parameters atomically.
* **Raises:**
  ParameterNotDeclaredException if undeclared parameters are not allowed,
  and at least one parameter in the list hadn’t been declared beforehand.

#### sleep(seconds)

sleeps using the node’s clock.

#### NOTE
Special case for good old foxy

* **Parameters:**
  **seconds** (*float*) – time to sleep
* **Return type:**
  `None`

#### *property* subscriptions

Get subscriptions that have been created on this node.

* **Return type:**
  `Iterator`[`Subscription`]

#### *property* timers

Get timers that have been created on this node.

* **Return type:**
  `Iterator`[`Timer`]

#### undeclare_parameter(name)

Undeclare a previously declared parameter.

This method will not cause a callback registered with
[`add_on_set_parameters_callback()`](#easy_robot_control.mover_node.MoverNode.add_on_set_parameters_callback) to be called.

* **Parameters:**
  **name** (`str`) – Fully-qualified name of the parameter, including its namespace.
* **Raises:**
  ParameterNotDeclaredException if parameter had not been declared before.
* **Raises:**
  ParameterImmutableException if the parameter was created as read-only.

#### wait_for_lower_level(more_services={}, all_requiered=False)

Blocks until all or any service is available.

#### NOTE
- List of waited services is given by services_to_wait ros2 param
- Overload this to wait for an additional service

* **Parameters:**
  * **more_services** (*Iterable* *[**str* *]*) – add more services
  * **all_requiered** (*bool*) – if True, all listed services must be available

#### wait_on_futures(future_list, wait_Hz=10)

Waits for the completion of a list of futures, checking completion at the
provided rate.

* **Parameters:**
  * **future_list** (*List* *[**Future* *]*  *|* *Future*) – List of Future to wait for
  * **wait_Hz** (*float*) – rate at which to wait

#### *property* waitables

Get waitables that have been created on this node.

* **Return type:**
  `Iterator`[`Waitable`]

### easy_robot_control.mover_node.main(args=None)

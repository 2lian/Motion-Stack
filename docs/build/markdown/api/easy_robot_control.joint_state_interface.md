# easy_robot_control.joint_state_interface module

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
  * **ignore** (`bool`) – if limits should be ignored
  * **jobj** (*Joint* *|* *None*)

#### *property* limit_rejected

if the limit was rejected when loading (often when not defined in urdf)

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
  **js** ([*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState))

#### is_new_jssensor(js)

True if js is different enough from the last received.
Also true if stateSensor is more the TOL_NO_CHANGE.time old relative to the new

* **Parameters:**
  **js** ([*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState))

#### setJSSensor(js)

Updates the stateSensor to a new js.

* **Parameters:**
  **js** ([*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState))

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
  [`JState`](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState)
* **Parameters:**
  **reset** (*bool*)

#### get_freshCommand(reset=True)

returns command data that is newer than the last time it was called.
full of None is not newer

* **Return type:**
  [`JState`](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState)
* **Parameters:**
  **reset** (*bool*)

### *class* easy_robot_control.joint_state_interface.JointNode

Bases: [`EliaNode`](easy_robot_control.EliaNode.md#easy_robot_control.EliaNode.EliaNode)

Lvl1

#### lvl0_remap

**Type:**    [`StateRemapper`](easy_robot_control.utils.state_remaper.md#easy_robot_control.utils.state_remaper.StateRemapper)

Remapping around any joint state communication of lvl0

#### lvl2_remap

**Type:**    [`StateRemapper`](easy_robot_control.utils.state_remaper.md#easy_robot_control.utils.state_remaper.StateRemapper)

Remapping around any joint state communication of lvl2

#### send_to_lvl0(states)

Sends states to lvl0 (commands for motors).
This function is executed every time data needs to be sent down.
Change/overload this method with what you need

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### send_to_lvl2(states)

Sends states to lvl2 (states for ik).
This function is executed every time data needs to be sent up.
Change/overload this method with what you need

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState) *]*)

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
  **states** (*List* *[*[*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### coming_from_lvl0(states)

Processes incomming sensor states from lvl0 motors.
Call this function after processing the ros message.
Always do super().coming_from_lvl0(states) before your code,
Unless you know what you are doing

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### advertiserSRVCBK(req, res)

Sends an JointState mainly to advertise the names of the joints

* **Return type:**
  `ReturnJointState_Response`
* **Parameters:**
  * **req** (*ReturnJointState_Request*)
  * **res** (*ReturnJointState_Response*)

#### defined_undefined()

Return joints with and without poistion data received yet

* **Return type:**
  `Tuple`[`List`[`str`], `List`[`str`]]
* **Returns:**
  Tuple(List[joint names that did not receive any data],
  List[joint names that have data])

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
  `ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]]
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

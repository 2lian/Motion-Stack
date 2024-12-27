# easy_robot_control.leg_node module

This node is responsible for recieving targets with trajectories of the
corresponding leg. It will performe smooth movements in the body center frame of reference
or relative to the current end effector position.
This node keeps track of the leg end effector to generate trajectories to the target.

Author: Elian NEPPEL
Lab: SRL, Moonshot team

### *class* easy_robot_control.leg_node.LegNode

Bases: [`EliaNode`](easy_robot_control.EliaNode.md#easy_robot_control.EliaNode.EliaNode)

#### firstSpinCBK()

#### publish_to_roll(roll=None)

publishes roll value towards ik node

* **Parameters:**
  **roll** (`Optional`[`float`])

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
  **index** (`int`) – if you wanna pop not the first but somewhere else
* **Returns:**
  np.array float(3,) - popped value
* **Return type:**
  value

#### pop_roll_from_traj(index=0)

Deletes and returns the first value of the trajectory_queue for the roll.

* **Parameters:**
  **index** (`int`) – if you wanna pop not the first but somewhere else
* **Returns:**
  np.array float(3,) - popped value
* **Return type:**
  value

#### get_final_xyz()

* **Return type:**
  `ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]]

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
  * **quat_traj** (`Optional`[`quaternion`]) – qt.quaternion shape(:) - quaternion trajectory RELATIVE TO THE BODY CENTER
  * **xyz_traj** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
* **Return type:**
  `None`

#### send_most_recent_tip(request, response)

Publish the last target sent

* **Parameters:**
  * **request** (`ReturnVect3_Request`) – ReturnVect3.Request - Nothing
  * **response** (`ReturnVect3_Response`) – ReturnVect3.Response - Vector3 (float x3)
* **Return type:**
  `ReturnVect3_Response`
* **Returns:**
  ReturnVect3 - Vector3 (float x3)

#### smoother(x)

smoothes the interval [0, 1] to have a soft start and end
(derivative is zero)

* **Return type:**
  `ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]]
* **Parameters:**
  **x** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)

#### smoother_complement(x)

changes the interval [0, 1] to 0->1->0
and smoothes to have a soft start and end

* **Return type:**
  `ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]]
* **Parameters:**
  **x** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)

#### rel_transl(xyz=None, quat=None)

performs translation to the target relative to body

* **Parameters:**
  * **target** – np.array float(3,) - target relative to the body center
  * **quat** (`Optional`[`quaternion`]) – qt.quaternion - target quaternion relative to body center
  * **xyz** (*ndarray* *|* *None*)
* **Returns:**
  np.array float(3,) - target relative to the body center
* **Return type:**
  target

#### rel_rotation(quat, center=array([0.0, 0.0, 0.0]))

performs rotation to the target relative to body

* **Parameters:**
  * **quaternion** – qt.quaternion float(4,) - quaternion to rotate by
  * **center** (`ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]]) – np.array float(3,) - center of rotation
  * **quat** (*quaternion*)
* **Return type:**
  `int`
* **Returns:**
  steps until end of mvt

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

#### rel_hop(target)

performs jump to the target relative to body

* **Parameters:**
  **target** (`ndarray`) – np.array float(3,) - target relative to the body center
* **Returns:**
  np.array float(3,) - target relative to the body center
* **Return type:**
  target

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

#### shift_cbk(request, response)

Callback for leg shift motion

* **Parameters:**
  * **request** (`TFService_Request`) – target relative to current end effector position
  * **response** (`TFService_Response`)
* **Return type:**
  `TFService_Response`
* **Returns:**
  success = True all the time

#### rel_transl_srv_cbk(request, response)

Callback for leg translation motion

* **Parameters:**
  * **request** (`TFService_Request`) – target relative to body
  * **response** (`TFService_Response`)
* **Return type:**
  `TFService_Response`
* **Returns:**
  success = True all the time

#### rel_hop_srv_cbk(request, response)

Callback for leg hopping motion

* **Parameters:**
  * **request** (`TFService_Request`) – target relative to body
  * **response** (`TFService_Response`)
* **Return type:**
  `TFService_Response`
* **Returns:**
  success = True all the time

#### rot_cbk(request, response)

Callback for leg rotation motion

* **Parameters:**
  * **request** (`TFService_Request`) – TF for the rotation and center of the rotation
  * **response** (`TFService_Response`)
* **Return type:**
  `TFService_Response`
* **Returns:**
  success = True all the time

#### point_cbk(request, response)

* **Return type:**
  `TFService_Response`
* **Parameters:**
  * **request** (*TFService_Request*)
  * **response** (*TFService_Response*)

### easy_robot_control.leg_node.main(args=None)

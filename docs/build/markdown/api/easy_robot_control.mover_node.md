# easy_robot_control.mover_node module

This node is responsible for synchronising several leg movement in order to move the
cente body and perform steps.

Author: Elian NEPPEL
Lab: SRL, Moonshot team

### *class* easy_robot_control.mover_node.MoverNode

Bases: [`EliaNode`](easy_robot_control.EliaNode.md#easy_robot_control.EliaNode.EliaNode)

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
  `ndarray`[`Any`, `dtype`[`TypeVar`(`_ScalarType_co`, bound= `generic`, covariant=True)]]

#### body_tfshift(shift, rot=quaternion(1, 0, 0, 0))

* **Return type:**
  `None`
* **Parameters:**
  * **shift** (*ndarray*)
  * **rot** (*quaternion*)

#### body_shift(shift)

* **Return type:**
  `None`
* **Parameters:**
  **shift** (*ndarray*)

#### manual_body_translation_rviz(coord, quat=quaternion(1, 0, 0, 0))

* **Return type:**
  `None`
* **Parameters:**
  * **coord** (*ndarray*)
  * **quat** (*quaternion*)

#### set_body_transform_rviz(coord, quat=quaternion(1, 0, 0, 0))

* **Return type:**
  `None`
* **Parameters:**
  * **coord** (*ndarray*)
  * **quat** (*quaternion*)

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

### easy_robot_control.mover_node.main(args=None)

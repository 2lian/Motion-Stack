# easy_robot_control.lazy_joint_state_publisher module

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

### easy_robot_control.lazy_joint_state_publisher.main()

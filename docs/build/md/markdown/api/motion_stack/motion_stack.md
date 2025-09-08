# motion_stack package

Motion Stack package.

<!-- Author:
Elian NEPPEL -->
<!-- Coauthor:
Shamistan KARIMOV
Ashutosh MISHRA -->
<!-- Laboratory:
Space Robotics Lab, Tohoku University -->
<!-- Maintainer:
Elian NEPPEL -->
<!-- Note:
You made a module? add yourself as the author! -->

## Subpackages

* [motion_stack.api package](motion_stack.api.md)
  * [Subpackages](motion_stack.api.md#subpackages)
  * [Submodules](motion_stack.api.md#submodules)
  * [motion_stack.api.ik_syncer module](motion_stack.api.md#module-motion_stack.api.ik_syncer)
  * [motion_stack.api.joint_syncer module](motion_stack.api.md#module-motion_stack.api.joint_syncer)
* [motion_stack.core package](motion_stack.core.md)
  * [Subpackages](motion_stack.core.md#subpackages)
  * [Submodules](motion_stack.core.md#submodules)
  * [motion_stack.core.lvl1_joint module](motion_stack.core.md#module-motion_stack.core.lvl1_joint)
  * [motion_stack.core.lvl2_ik module](motion_stack.core.md#module-motion_stack.core.lvl2_ik)
  * [motion_stack.core.lvl4_mover module](motion_stack.core.md#module-motion_stack.core.lvl4_mover)
* [motion_stack.ros2 package](motion_stack.ros2.md)
  * [Subpackages](motion_stack.ros2.md#subpackages)
  * [Submodules](motion_stack.ros2.md#submodules)
  * [motion_stack.ros2.communication module](motion_stack.ros2.md#module-motion_stack.ros2.communication)

## Submodules

## motion_stack.high_level_louis module

This gives example of a high level node using the motion stack API

#### WARNING
To make this example as easy as possible, async/await is heavily used.
This is unusual, you do not need and even, should not use async/await with Ros2.
The motion stack uses generic Future and callback, async/await style
is not required for the motion stack.

In this example every time `await`, is used (on a ros2 Future, or python awaitable),
the code pauses until the awaitable finishes, however it does not block the ros2 executor.
Basically, this `await` sleeps/waits without blocking ros2 operations
(incomming/outgoing messages).

async/await is easier to read, however much more reliable and performant code is
possible using ros2 future+callback and especially timers.

### *class* motion_stack.high_level_louis.TutoNode

Bases: `Node`

#### LIMBS *= [3, 201]*

**Type:**    `list`

list of limbs number that are controlled

#### main()

#### *async* joints_ready()

Returns once all joints are ready

#### *async* ik_ready()

Returns once all ik are ready

#### angles_to_zero()

sends all joints to 0.0

* **Return type:**
  `Coroutine`

#### angles_to_test(test_angle)

sends all joints to 0.0

* **Return type:**
  `Coroutine`

#### delta_ee(delta_pose)

* **Return type:**
  `Coroutine`
* **Parameters:**
  **delta_pose** ([*Pose*](motion_stack.core.utils.md#motion_stack.core.utils.pose.Pose))

#### startup()

Execute once at startup

#### exec_loop()

Regularly executes the syncers

### motion_stack.high_level_louis.main(\*args)

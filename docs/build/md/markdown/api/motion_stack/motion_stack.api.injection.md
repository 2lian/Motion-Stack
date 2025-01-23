# motion_stack.api.injection package

Dependency injection tools to add functionalities

## Submodules

## motion_stack.api.injection.offsetter module

### *class* motion_stack.api.injection.offsetter.OffsetterLvl0(core, angle_recovery_path=None, offset_path=None)

Bases: `object`

Position offseter for lvl0.
Usefull if your URDF and robot are not aligned.

Features

- Simply provide the JointCore on which to apply the offsets
- Apply an angle offset to any joint of the lvl0 input/output.
- Apply the offset at runtime
- (Optional) Load offsets from a csv on disk.
- (Optional) Save current angles multiplied by -1 in a csv on disk. This saved angle can tell you the last shutdown position of the robot, if you need to recover the offsets from it.

#### NOTE
- You should provide this object an initialized JointCore.
- You need to call by yourself:
  > - [`OffsetterLvl0.apply_offset()`](#motion_stack.api.injection.offsetter.OffsetterLvl0.apply_offset)
  > - [`OffsetterLvl0.save_angle_as_offset()`](../easy_robot_control/easy_robot_control.injection.md#easy_robot_control.injection.offsetter.OffsetterLvl0.save_angle_as_offset)
  > - [`OffsetterLvl0.save_current_offset()`](#motion_stack.api.injection.offsetter.OffsetterLvl0.save_current_offset)
- [`OffsetterLvl0.load_offset()`](#motion_stack.api.injection.offsetter.OffsetterLvl0.load_offset) is called on object initialization.
- See the ros2 wrapper if you running the core through ros2.

* **Parameters:**
  * **core** ([*JointCore*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointCore)) – Initialized JointCore
  * **angle_recovery_path** (*str* *|* *None*) – if None, does not save the current angles on disk
  * **offset_path** (*str* *|* *None*) – if None, does not save or load offsets from disk

#### *property* offsets

Offets being used

* **Return type:**
  `Dict`[`str`, `float`]

#### apply_offset(js_offset)

Offset values will be replaced by new ones then saved on disk.

#### NOTE
Preferably use this to not lose the offset in case of restart

* **Parameters:**
  **js_offset** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*  *|* *None*) – list of offsets
* **Returns:**
  True if all offsets have a joint to be applied to
  String for user debugging
* **Return type:**
  `Tuple`[`bool`, `str`]

#### save_angle_recovery(handlers=None)

Saves current position as the offset to recover to incase of powerloss.

#### NOTE
- Saved in self.angle_path
- To use those saves as offsets, replace the file <self.offset_path> with <self.angle_path>

* **Parameters:**
  **handlers** (*Dict* *[**str* *,* [*JointHandler*](motion_stack.core.md#motion_stack.core.lvl1_joint.JointHandler) *]*  *|* *None*)

#### load_offset()

Loads offset from offset csv. Skips unknown joints.

#### save_current_offset(to_save=None)

DO NOT DO THIS AUTOMATICALLY, IT COULD BE DESTRUCTIVE OF VALUABLE INFO.
overwrites the offset csv with the currently running offsets

* **Parameters:**
  **to_save** (*Dict* *[**str* *,* *float* *]*  *|* *None*)

## motion_stack.api.injection.remapper module

Apply any function to the input/output of the lvl1 joint core.

This allows for static offset, gain and more. So if your URDF is offsetted from the real robot, this is one solution. It can also change names of joints right before publication/reception, in case you are unhappy with your urdf joint names.

The rempper can apply:
: - any function: defined as any float->float function.
  - to any attribute: position, velocity or effort .
  - to any joint: Joint names are linked to a function through a dicitonary.

### *class* motion_stack.api.injection.remapper.StateRemapper(name_map={}, unname_map=None, state_map={}, unstate_map={})

Bases: `object`

Remaps names and applies shaping function to joint state (type: JState).

This remapper is bi-direcitonal, so data can be mapped then be un-mapped. What thing is mapped/unmapped in the motion stack is simple: the outgoing data is mapped, incomming data is unmapped.

So, if you apply this to lvl1, as a mapping for lvl0:

> - `name_map` will change the name of the joints from to URDF name to another name when **sending to the motors**.
> - `unname_map` will change the recieved name to another name when **receiving sensor data**.
> - `state_map` applies a [`joint_mapper.Shaper`](motion_stack.core.utils.md#motion_stack.core.utils.joint_mapper.Shaper) to the joint state when **sending to the motors**.
> - `unstate_map` applies a [`joint_mapper.Shaper`](motion_stack.core.utils.md#motion_stack.core.utils.joint_mapper.Shaper) to the joint state when **recieving from the sensors**.

Example, the motion stack is controlling joint 1:

```default
remap_lvl1 = StateRemapper(
    name_map={"joint1_1": "my_new_joint"}, # "joint1_1" output (usually motor command) is renamed to "my_new_joint"
    unname_map={"another_joint": "joint1_1"}, # "another_joint" intput (usually sensor reading) is renamed to "joint1_1"
    state_map={"joint1_1": Shaper(position=lambda x: x * 2)}, # multiplies command by 2
    unstate_map={"joint1_1": Shaper(position=lambda x: x / 2)}, # divides sensor by 2
)
```

* **Parameters:**
  * **name_map** (*Dict* *[**str* *,* *str* *]*) – joint name mapping from joint_name -> output_name
  * **unname_map** (*Dict* *[**str* *,* *str* *]*  *|* *None*) – joint name unmapping from input_name -> joint_name
  * **state_map** (*Dict* *[**str* *,* [*Shaper*](motion_stack.core.utils.md#motion_stack.core.utils.joint_mapper.Shaper) *]*) – joint state shaper mapping joint_name -> Shaper_function. Shaper_function will be applied on the output.
  * **unstate_map** (*Dict* *[**str* *,* [*Shaper*](motion_stack.core.utils.md#motion_stack.core.utils.joint_mapper.Shaper) *]*) – joint state shaper unmapping joint_name -> UnShaper_function. Shaper_function will be applied on the input.

#### map(states)

Apllies the mapping used when sending

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

#### unmap(states)

Apllies the mapping when receiving

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.md#motion_stack.core.utils.joint_state.JState) *]*)

#### simplify(names_to_keep)

Eliminates (not in place) all entries whose keys are not in names_to_keep.

* **Returns:**
  new StateRemapper to use
* **Return type:**
  [`StateRemapper`](#motion_stack.api.injection.remapper.StateRemapper)
* **Parameters:**
  **names_to_keep** (*Iterable* *[**str* *]*)

#### *classmethod* empty()

### motion_stack.api.injection.remapper.insert_angle_offset(mapper_in, mapper_out, offsets)

Applies an position offsets to an existing StateRemapper shapers.
the state_map adds the offset, then the unstate_map substracts it (in mapper_out).

Sub_shapers from mapper_in will overwrite sub_shapers in mapper_out if they are
affected by offsets.
mapper_out = mapper_in, may lead to undefined behavior.
Any function shared between in/out may lead to undefined behavior.
Use deepcopy() to avoid issues.

this is a very rough function. feel free to improve

* **Parameters:**
  * **mapper_in** ([*StateRemapper*](#motion_stack.api.injection.remapper.StateRemapper)) – original function map to which offset should be added
  * **mapper_out** ([*StateRemapper*](#motion_stack.api.injection.remapper.StateRemapper)) – changes will be stored here
  * **offsets** (*Dict* *[**str* *,* *float* *]*)
* **Returns:**
  Nothing, the result is stored in the mapper_out.
* **Return type:**
  `None`

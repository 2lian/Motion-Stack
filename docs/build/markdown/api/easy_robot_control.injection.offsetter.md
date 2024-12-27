# easy_robot_control.injection.offsetter module

### *class* easy_robot_control.injection.offsetter.OffsetterLvl0(parent, angle_path=None, offset_path=None)

Bases: `object`

Position offseter for lvl0, to be injected into a JointNode.
Usefull if your URDF and robot are not aligned.

Features
: - Inject this into any JointNode
  - Apply an angle offset to any joint of the lvl0 input/output.
  - Use a service to apply the offset at runtime
  - (Optional) Load offsets from a csv on disk.
  - (Optional) Save current angles multiplied by -1 every 3s in a csv on disk.This saved angle can tell you the last shutdown position of the robot, if you need to recover the offsets from it.

#### NOTE
- You should inject this object into a JointNode at the end of initialization.
- You do not need  to call any of those function, just inject this object.

* **Parameters:**
  * **parent** ([`JointNode`](easy_robot_control.joint_state_interface.md#easy_robot_control.joint_state_interface.JointNode))
  * **angle_path** (`Optional`[`str`]) – if None, does not save the current angles on disk
  * **offset_path** (`Optional`[`str`]) – if None, does not save or load offsets from disk

### Example

Injecting in a JointNode:

```default
class Example(JointNode):
    def __init__(self):
        super().__init__()
        self.offsetter = OffsetterLvl0(
            self, angle_path=ANGLE_PATH, offset_path=OFFSET_PATH
        )
```

#### update_mapper(mapper_in=None, mapper_out=None)

Applies the offsets to a StateRemapper.

* **Parameters:**
  * **mapper_in** (`Optional`[[`StateRemapper`](easy_robot_control.utils.state_remaper.md#easy_robot_control.utils.state_remaper.StateRemapper)]) – original map to which offset should be added
  * **mapper_out** (`Optional`[[`StateRemapper`](easy_robot_control.utils.state_remaper.md#easy_robot_control.utils.state_remaper.StateRemapper)]) – affected subshaper of this map will change
* **Return type:**
  `None`

#### update_and_save_offset(js_offset)

Held offset values will be updated then saved on disk.

#### NOTE
Preferably use this to not lose the offset in case of restart

* **Parameters:**
  **js_offset** (`List`[[`JState`](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState)]) – list of offsets
* **Return type:**
  `Tuple`[`bool`, `str`]
* **Returns:**
  True if all offsets have a joint to be applied to
  String for user debugging

#### save_angle_as_offset(handlers=None)

Saves current position as the offset to recover to incase of powerloss.

#### NOTE
- Saved in self.angle_path
- To use those saves as offsets, replace the file <self.offset_path> with <self.angle_path>

* **Parameters:**
  **handlers** (*Dict* *[**str* *,* [*JointHandler*](easy_robot_control.joint_state_interface.md#easy_robot_control.joint_state_interface.JointHandler) *]*  *|* *None*)

#### load_offset()

Loads offset from offset csv. Skips unknown joints.

#### save_current_offset(to_save=None)

DO NOT DO THIS AUTOMATICALLY, IT COULD BE DESTRUCTIVE OF VALUABLE INFO.
overwrites the offset csv with the currently running offsets

* **Parameters:**
  **to_save** (*Dict* *[**str* *,* *float* *]*  *|* *None*)

#### update_offset(js_offset)

Updates offset in memory

* **Parameters:**
  **js_offset** (`List`[[`JState`](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState)]) – list of offsets
* **Return type:**
  `Tuple`[`bool`, `str`]
* **Returns:**
  True if all offsets have a joint to be applied to
  String for user debugging

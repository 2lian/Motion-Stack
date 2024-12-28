# motion_stack.core.utils.robot_parsing module

Uses rtb to parse the robot URDF data

### motion_stack.core.utils.robot_parsing.get_limit(joint)

Returns the limits of a joint from rtb parsing

* **Return type:**
  `Tuple`[`float`, `float`]
* **Parameters:**
  **joint** (*Joint*)

### motion_stack.core.utils.robot_parsing.load_set_urdf(urdf_path, end_effector_name=None, start_effector_name=None)

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

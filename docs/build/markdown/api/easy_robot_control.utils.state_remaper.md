# easy_robot_control.utils.state_remaper module

### easy_robot_control.utils.state_remaper.operate_sub_shapers(shaper1, shaper2, op)

* **Return type:**
  `Optional`[`Callable`[[`float`], `float`]]
* **Parameters:**
  * **shaper1** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **shaper2** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **op** (*Callable* *[* *[**float* *,* *float* *]* *,* *float* *]*)

### easy_robot_control.utils.state_remaper.eggify_shapers(inner, outer)

* **Return type:**
  `Optional`[`Callable`[[`float`], `float`]]
* **Parameters:**
  * **inner** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **outer** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)

### *class* easy_robot_control.utils.state_remaper.Shaper(position=None, velocity=None, effort=None)

Bases: `object`

* **Parameters:**
  * **position** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **velocity** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **effort** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)

#### position *= None*

**Type:**    `Optional`[`Callable`[[`float`], `float`]]

#### velocity *= None*

**Type:**    `Optional`[`Callable`[[`float`], `float`]]

#### effort *= None*

**Type:**    `Optional`[`Callable`[[`float`], `float`]]

### easy_robot_control.utils.state_remaper.reverse_dict(d)

* **Return type:**
  `Dict`
* **Parameters:**
  **d** (*Dict*)

### easy_robot_control.utils.state_remaper.remap_names(states, mapping)

* **Parameters:**
  * **states** (*List* *[*[*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState) *]*)
  * **mapping** (*Dict* *[**str* *,* *str* *]*)

### easy_robot_control.utils.state_remaper.apply_shaper(state, shaper)

* **Parameters:**
  * **state** ([*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState))
  * **shaper** ([*Shaper*](#easy_robot_control.utils.state_remaper.Shaper))

### easy_robot_control.utils.state_remaper.shape_states(states, mapping)

* **Parameters:**
  * **states** (*List* *[*[*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState) *]*)
  * **mapping** (*Dict* *[**str* *,* [*Shaper*](#easy_robot_control.utils.state_remaper.Shaper) *]*)

### *class* easy_robot_control.utils.state_remaper.StateRemapper(name_map={}, unname_map=None, state_map={}, unstate_map={})

Bases: `object`

* **Parameters:**
  * **name_map** (*Dict* *[**str* *,* *str* *]*)
  * **unname_map** (*Dict* *[**str* *,* *str* *]*  *|* *None*)
  * **state_map** (*Dict* *[**str* *,* [*Shaper*](#easy_robot_control.utils.state_remaper.Shaper) *]*)
  * **unstate_map** (*Dict* *[**str* *,* [*Shaper*](#easy_robot_control.utils.state_remaper.Shaper) *]*)

#### namify(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### unnamify(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### shapify(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### unshapify(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### map(states)

mapping used before sending

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### unmap(states)

mapping used before receiving

* **Parameters:**
  **states** (*List* *[*[*JState*](easy_robot_control.utils.joint_state_util.md#easy_robot_control.utils.joint_state_util.JState) *]*)

#### simplify(names_to_keep)

Eliminates (not in place) all entries whose keys are not in names_to_keep.
:rtype: [`StateRemapper`](#easy_robot_control.utils.state_remaper.StateRemapper)
:returns: new StateRemapper

* **Parameters:**
  **names_to_keep** (*Iterable* *[**str* *]*)
* **Return type:**
  [*StateRemapper*](#easy_robot_control.utils.state_remaper.StateRemapper)

### easy_robot_control.utils.state_remaper.insert_angle_offset(mapper_in, mapper_out, offsets)

Applies an position offsets to a StateRemapper.
the state_map adds the offset, then the unstate_map substracts it (in mapper_out).

Sub_shapers from mapper_in will overwrite sub_shapers in mapper_out if they are
affected by offsets.

> mapper_out = mapper_in, may lead to undefined behavior.
> Any function shared between in/out may lead to undefined behavior.
> Use deepcopy() to avoid issues.

this is a very rough function. feel free to improve

* **Parameters:**
  * **mapper_in** ([`StateRemapper`](#easy_robot_control.utils.state_remaper.StateRemapper)) – original function map to which offset should be added
  * **mapper_out** ([`StateRemapper`](#easy_robot_control.utils.state_remaper.StateRemapper)) – changes will be stored here
  * **offsets** (*Dict* *[**str* *,* *float* *]*)
* **Return type:**
  `None`

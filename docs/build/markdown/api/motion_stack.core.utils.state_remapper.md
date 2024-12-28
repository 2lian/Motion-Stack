# motion_stack.core.utils.state_remapper module

### motion_stack.core.utils.state_remapper.operate_sub_shapers(shaper1, shaper2, op)

* **Return type:**
  `Optional`[`Callable`[[`float`], `float`]]
* **Parameters:**
  * **shaper1** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **shaper2** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **op** (*Callable* *[* *[**float* *,* *float* *]* *,* *float* *]*)

### motion_stack.core.utils.state_remapper.eggify_shapers(inner, outer)

* **Return type:**
  `Optional`[`Callable`[[`float`], `float`]]
* **Parameters:**
  * **inner** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)
  * **outer** (*Callable* *[* *[**float* *]* *,* *float* *]*  *|* *None*)

### *class* motion_stack.core.utils.state_remapper.Shaper(position=None, velocity=None, effort=None)

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

### motion_stack.core.utils.state_remapper.reverse_dict(d)

* **Return type:**
  `Dict`
* **Parameters:**
  **d** (*Dict*)

### motion_stack.core.utils.state_remapper.remap_names(states, mapping)

* **Parameters:**
  * **states** (*List* *[*[*JState*](motion_stack.core.utils.joint_state.md#motion_stack.core.utils.joint_state.JState) *]*)
  * **mapping** (*Dict* *[**str* *,* *str* *]*)

### motion_stack.core.utils.state_remapper.apply_shaper(state, shaper)

* **Parameters:**
  * **state** ([*JState*](motion_stack.core.utils.joint_state.md#motion_stack.core.utils.joint_state.JState))
  * **shaper** ([*Shaper*](#motion_stack.core.utils.state_remapper.Shaper))

### motion_stack.core.utils.state_remapper.shape_states(states, mapping)

* **Parameters:**
  * **states** (*List* *[*[*JState*](motion_stack.core.utils.joint_state.md#motion_stack.core.utils.joint_state.JState) *]*)
  * **mapping** (*Dict* *[**str* *,* [*Shaper*](#motion_stack.core.utils.state_remapper.Shaper) *]*)

### *class* motion_stack.core.utils.state_remapper.StateRemapper(name_map={}, unname_map=None, state_map={}, unstate_map={})

Bases: `object`

* **Parameters:**
  * **name_map** (*Dict* *[**str* *,* *str* *]*)
  * **unname_map** (*Dict* *[**str* *,* *str* *]*  *|* *None*)
  * **state_map** (*Dict* *[**str* *,* [*Shaper*](#motion_stack.core.utils.state_remapper.Shaper) *]*)
  * **unstate_map** (*Dict* *[**str* *,* [*Shaper*](#motion_stack.core.utils.state_remapper.Shaper) *]*)

#### namify(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.joint_state.md#motion_stack.core.utils.joint_state.JState) *]*)

#### unnamify(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.joint_state.md#motion_stack.core.utils.joint_state.JState) *]*)

#### shapify(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.joint_state.md#motion_stack.core.utils.joint_state.JState) *]*)

#### unshapify(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.joint_state.md#motion_stack.core.utils.joint_state.JState) *]*)

#### map(states)

mapping used before sending

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.joint_state.md#motion_stack.core.utils.joint_state.JState) *]*)

#### unmap(states)

mapping used before receiving

* **Parameters:**
  **states** (*List* *[*[*JState*](motion_stack.core.utils.joint_state.md#motion_stack.core.utils.joint_state.JState) *]*)

#### simplify(names_to_keep)

Eliminates (not in place) all entries whose keys are not in names_to_keep.
:rtype: [`StateRemapper`](#motion_stack.core.utils.state_remapper.StateRemapper)
:returns: new StateRemapper

* **Parameters:**
  **names_to_keep** (*Iterable* *[**str* *]*)
* **Return type:**
  [*StateRemapper*](#motion_stack.core.utils.state_remapper.StateRemapper)

### motion_stack.core.utils.state_remapper.insert_angle_offset(mapper_in, mapper_out, offsets)

Applies an position offsets to a StateRemapper.
the state_map adds the offset, then the unstate_map substracts it (in mapper_out).

Sub_shapers from mapper_in will overwrite sub_shapers in mapper_out if they are
affected by offsets.

> mapper_out = mapper_in, may lead to undefined behavior.
> Any function shared between in/out may lead to undefined behavior.
> Use deepcopy() to avoid issues.

this is a very rough function. feel free to improve

* **Parameters:**
  * **mapper_in** ([`StateRemapper`](#motion_stack.core.utils.state_remapper.StateRemapper)) – original function map to which offset should be added
  * **mapper_out** ([`StateRemapper`](#motion_stack.core.utils.state_remapper.StateRemapper)) – changes will be stored here
  * **offsets** (*Dict* *[**str* *,* *float* *]*)
* **Return type:**
  `None`

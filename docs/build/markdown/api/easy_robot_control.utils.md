# easy_robot_control.utils package

## Submodules

## easy_robot_control.utils.csv module

simple csv utilities

### easy_robot_control.utils.csv.update_csv(file_path, new_str, new_float)

* **Return type:**
  `Tuple`[`str`, `Optional`[`str`]]
* **Parameters:**
  * **new_str** (*str*)
  * **new_float** (*float*)

### easy_robot_control.utils.csv.csv_to_dict(file_path)

* **Return type:**
  `Optional`[`Dict`[`str`, `float`]]

## easy_robot_control.utils.hero_mapping module

## easy_robot_control.utils.hyper_sphere_clamp module

Vectorized functions to clamp onto an hypershpere

Author: Elian NEPPEL
Lab: SRL, Moonshot team

### easy_robot_control.utils.hyper_sphere_clamp.clamp_to_unit_hs(start, end, sampling_step=0.01)

Finds the farthest point on the segment that is inside the unit hypersphere.

* **Return type:**
  `NDArray`[`Shape`[N], `float64`]
* **Parameters:**
  * **start** (*NDArray* *[**Shape* *[**N* *]* *,* *float64* *]*)
  * **end** (*NDArray* *[**Shape* *[**N* *]* *,* *float64* *]*)
  * **sampling_step** (*float*)

### easy_robot_control.utils.hyper_sphere_clamp.clamp_to_sqewed_hs(center, start, end, radii)

Finds the farthest point on the segment that is inside the sqewed hypersphere.

radii of the hypersphere in each dimensions is computed by streching the space in each dimension, then computing relative to the unit hypersphere, then unstreching.

* **Return type:**
  `NDArray`[`Shape`[N], `floating`]
* **Parameters:**
  * **center** (*NDArray* *[**Shape* *[**N* *]* *,* *floating* *]*)
  * **start** (*NDArray* *[**Shape* *[**N* *]* *,* *floating* *]*)
  * **end** (*NDArray* *[**Shape* *[**N* *]* *,* *floating* *]*)
  * **radii** (*NDArray* *[**Shape* *[**N* *]* *,* *floating* *]*)

### easy_robot_control.utils.hyper_sphere_clamp.fuse_xyz_quat(xyz, quat)

* **Return type:**
  `NDArray`[`Shape`[7], `floating`]
* **Parameters:**
  * **xyz** (*NDArray* *[**Shape* *[**3* *]* *,* *floating* *]*)
  * **quat** (*quaternion*)

### easy_robot_control.utils.hyper_sphere_clamp.unfuse_xyz_quat(arr)

* **Return type:**
  `Tuple`[`NDArray`[`Shape`[3], `floating`], `quaternion`]
* **Parameters:**
  **arr** (*NDArray* *[**Shape* *[**7* *]* *,* *floating* *]*)

### easy_robot_control.utils.hyper_sphere_clamp.clamp_xyz_quat(center, start, end, radii)

wrapper for clamp_to_sqewed_hs specialized in one 3D coordinate + one quaternion.

The math for the quaternion is wrong (lerp instead of slerp). So:
Center and start quat should not be opposite from each-other.
Precision goes down if they are far appart.

* **Parameters:**
  * **center** (`Tuple`[`NDArray`[`Shape`[3], `floating`], `quaternion`]) – center from which not to diverge
  * **start** (`Tuple`[`NDArray`[`Shape`[3], `floating`], `quaternion`]) – start point of the interpolation
  * **end** (`Tuple`[`NDArray`[`Shape`[3], `floating`], `quaternion`]) – end point of the interpolation
  * **radii** (`Tuple`[`float`, `float`]) – allowed divergence for coord and quat
* **Return type:**
  `Tuple`[`NDArray`[`Shape`[3], `floating`], `quaternion`]

Returns:

## easy_robot_control.utils.joint_state_util module

### *class* easy_robot_control.utils.joint_state_util.JState(name, position=None, velocity=None, effort=None, time=None)

Bases: `object`

* **Parameters:**
  * **name** (*str*)
  * **position** (*float* *|* *None*)
  * **velocity** (*float* *|* *None*)
  * **effort** (*float* *|* *None*)
  * **time** (*Time* *|* *None*)

#### name

**Type:**    `str`

#### position *= None*

**Type:**    `Optional`[`float`]

#### velocity *= None*

**Type:**    `Optional`[`float`]

#### effort *= None*

**Type:**    `Optional`[`float`]

#### time *= None*

**Type:**    `Optional`[`Time`]

#### copy()

* **Return type:**
  [`JState`](#easy_robot_control.utils.joint_state_util.JState)

### easy_robot_control.utils.joint_state_util.js_from_ros(jsin)

* **Return type:**
  `List`[[`JState`](#easy_robot_control.utils.joint_state_util.JState)]
* **Parameters:**
  **jsin** (*JointState*)

### easy_robot_control.utils.joint_state_util.intersect_names(js_in, names)

* **Return type:**
  `JointState`
* **Parameters:**
  * **js_in** (*JointState*)
  * **names** (*Sequence* *[**str* *]*)

### easy_robot_control.utils.joint_state_util.js_copy(js)

* **Return type:**
  [`JState`](#easy_robot_control.utils.joint_state_util.JState)
* **Parameters:**
  **js** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))

### easy_robot_control.utils.joint_state_util.impose_state(onto, fromm)

* **Return type:**
  [`JState`](#easy_robot_control.utils.joint_state_util.JState)
* **Parameters:**
  * **onto** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))
  * **fromm** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))

### easy_robot_control.utils.joint_state_util.js_changed(j1, j2, delta)

* **Return type:**
  `bool`
* **Parameters:**
  * **j1** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))
  * **j2** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))
  * **delta** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))

### easy_robot_control.utils.joint_state_util.js_diff(j1, j2)

* **Return type:**
  [`JState`](#easy_robot_control.utils.joint_state_util.JState)
* **Parameters:**
  * **j1** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))
  * **j2** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))

### easy_robot_control.utils.joint_state_util.stateOrderinator3000(allStates)

Converts a list  of JState to multiple ros JointStates messages.
Timestamp ignored.

* **Return type:**
  `List`[`JointState`]
* **Parameters:**
  **allStates** (*Iterable* *[*[*JState*](#easy_robot_control.utils.joint_state_util.JState) *]*)

## easy_robot_control.utils.math module

### easy_robot_control.utils.math.qt_normalize(q)

* **Parameters:**
  **q** (*quaternion*)

### easy_robot_control.utils.math.qt_repr(q)

* **Return type:**
  `str`
* **Parameters:**
  **q** (*quaternion*)

## easy_robot_control.utils.pure_remap module

### easy_robot_control.utils.pure_remap.is_valid_ros2_name(name)

* **Return type:**
  `bool`
* **Parameters:**
  **name** (*str*)

### easy_robot_control.utils.pure_remap.run_shaping(f)

* **Return type:**
  `bool`
* **Parameters:**
  **f** (*Callable* *[* *[**float* *]* *,* *float* *]*)

### easy_robot_control.utils.pure_remap.test_remap(dic_index, key, value)

### easy_robot_control.utils.pure_remap.value(x)

### easy_robot_control.utils.pure_remap.test_shape(dic_index, key, value)

## easy_robot_control.utils.state_remaper module

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
  * **states** (*List* *[*[*JState*](#easy_robot_control.utils.joint_state_util.JState) *]*)
  * **mapping** (*Dict* *[**str* *,* *str* *]*)

### easy_robot_control.utils.state_remaper.apply_shaper(state, shaper)

* **Parameters:**
  * **state** ([*JState*](#easy_robot_control.utils.joint_state_util.JState))
  * **shaper** ([*Shaper*](#easy_robot_control.utils.state_remaper.Shaper))

### easy_robot_control.utils.state_remaper.shape_states(states, mapping)

* **Parameters:**
  * **states** (*List* *[*[*JState*](#easy_robot_control.utils.joint_state_util.JState) *]*)
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
  **states** (*List* *[*[*JState*](#easy_robot_control.utils.joint_state_util.JState) *]*)

#### unnamify(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](#easy_robot_control.utils.joint_state_util.JState) *]*)

#### shapify(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](#easy_robot_control.utils.joint_state_util.JState) *]*)

#### unshapify(states)

* **Parameters:**
  **states** (*List* *[*[*JState*](#easy_robot_control.utils.joint_state_util.JState) *]*)

#### map(states)

mapping used before sending

* **Parameters:**
  **states** (*List* *[*[*JState*](#easy_robot_control.utils.joint_state_util.JState) *]*)

#### unmap(states)

mapping used before receiving

* **Parameters:**
  **states** (*List* *[*[*JState*](#easy_robot_control.utils.joint_state_util.JState) *]*)

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

## easy_robot_control.utils.trajectories module

### *class* easy_robot_control.utils.trajectories.Point(time, coord=None, rot=None)

Bases: `object`

* **Parameters:**
  * **time** (*Time*)
  * **coord** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
  * **rot** (*quaternion* *|* *None*)

#### time

**Type:**    `Time`

#### coord *= None*

**Type:**    `Optional`[`ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]]

#### rot *= None*

**Type:**    `Optional`[`quaternion`]

### *class* easy_robot_control.utils.trajectories.PointDefined(time, coord, rot)

Bases: `object`

* **Parameters:**
  * **time** (*Time*)
  * **coord** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **rot** (*quaternion*)

#### time

**Type:**    `Time`

#### coord

**Type:**    `ndarray`[`Any`, `dtype`[`+_ScalarType_co`]]

#### rot

**Type:**    `quaternion`

### easy_robot_control.utils.trajectories.assessPointDefine(point)

* **Return type:**
  [`PointDefined`](#easy_robot_control.utils.trajectories.PointDefined)
* **Parameters:**
  **point** ([*Point*](#easy_robot_control.utils.trajectories.Point))

### easy_robot_control.utils.trajectories.smooth(relative_end, t)

smoothes the interval [0, 1] to have a soft start and end
(derivative is zero)

* **Return type:**
  `float`
* **Parameters:**
  * **relative_end** (*float*)
  * **t** (*float*)

### easy_robot_control.utils.trajectories.linear(relative_end, t)

* **Return type:**
  `TypeVar`(`TypeP`, `float`, `ndarray`, `quaternion`)
* **Parameters:**
  * **relative_end** (*TypeP*)
  * **t** (*float*)

### easy_robot_control.utils.trajectories.triangle(relative_end, t)

* **Return type:**
  `float`
* **Parameters:**
  * **relative_end** (*float*)
  * **t** (*float*)

### easy_robot_control.utils.trajectories.boomrang(relative_end, t)

* **Return type:**
  `float`
* **Parameters:**
  * **relative_end** (*float*)
  * **t** (*float*)

### easy_robot_control.utils.trajectories.singo(relative_end, t)

* **Return type:**
  `float`
* **Parameters:**
  * **relative_end** (*float*)
  * **t** (*float*)

### easy_robot_control.utils.trajectories.sincom(relative_end, t)

* **Return type:**
  `float`
* **Parameters:**
  * **relative_end** (*float*)
  * **t** (*float*)

### easy_robot_control.utils.trajectories.prolongate(func)

* **Return type:**
  `Callable`[[`float`, `float`], `float`]
* **Parameters:**
  **func** (*Callable* *[* *[**float* *,* *float* *]* *,* *float* *]*)

### easy_robot_control.utils.trajectories.reverseFun(func)

* **Return type:**
  `Callable`[[`float`, `float`], `float`]
* **Parameters:**
  **func** (*Callable* *[* *[**float* *,* *float* *]* *,* *float* *]*)

### easy_robot_control.utils.trajectories.get_interp(interp_str, start, end)

* **Return type:**
  `Callable`[[`Time`], [`PointDefined`](#easy_robot_control.utils.trajectories.PointDefined)]
* **Parameters:**
  * **interp_str** (*str*)
  * **start** ([*Point*](#easy_robot_control.utils.trajectories.Point))
  * **end** ([*Point*](#easy_robot_control.utils.trajectories.Point))

### easy_robot_control.utils.trajectories.globalInterpolator(start, end, spatial_interp, temporal_interp, now)

* **Return type:**
  [`PointDefined`](#easy_robot_control.utils.trajectories.PointDefined)
* **Parameters:**
  * **start** ([*PointDefined*](#easy_robot_control.utils.trajectories.PointDefined))
  * **end** ([*Point*](#easy_robot_control.utils.trajectories.Point))
  * **spatial_interp** (*Callable* *[* *[**TypeP* *,* *float* *]* *,* *TypeP* *]*)
  * **temporal_interp** (*Callable* *[* *[**TypeP* *,* *float* *]* *,* *TypeP* *]*)
  * **now** (*Time*)

### easy_robot_control.utils.trajectories.time_interp(ifunc, t)

* **Parameters:**
  * **ifunc** (*Callable* *[* *[**TypeP* *,* *float* *]* *,* *TypeP* *]*)
  * **t** (*float*)

### easy_robot_control.utils.trajectories.coord_interp(start, end, ifunc, t)

* **Parameters:**
  * **start** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*)
  * **end** (*ndarray* *[**Any* *,* *dtype* *[* *\_ScalarType_co* *]* *]*  *|* *None*)
  * **ifunc** (*Callable* *[* *[**TypeP* *,* *float* *]* *,* *TypeP* *]*)
  * **t** (*float*)

### easy_robot_control.utils.trajectories.quat_interp(start, end, ifunc, t)

* **Parameters:**
  * **start** (*quaternion*)
  * **end** (*quaternion*)
  * **ifunc** (*Callable* *[* *[**TypeP* *,* *float* *]* *,* *TypeP* *]*)
  * **t** (*float*)

### *class* easy_robot_control.utils.trajectories.Interpolator(start, end, spatial_interp=None, temporal_interp=None)

Bases: `object`

* **Parameters:**
  * **start** ([*PointDefined*](#easy_robot_control.utils.trajectories.PointDefined))
  * **end** ([*Point*](#easy_robot_control.utils.trajectories.Point))
  * **spatial_interp** (*None* *|* *Callable* *[* *[**TypeP* *,* *float* *]* *,* *TypeP* *]*  *|* *str*)
  * **temporal_interp** (*None* *|* *Callable* *[* *[**TypeP* *,* *float* *]* *,* *TypeP* *]*  *|* *str*)

#### expired(now)

* **Return type:**
  `bool`
* **Parameters:**
  **now** (*Time*)

#### early(now)

* **Return type:**
  `bool`
* **Parameters:**
  **now** (*Time*)

#### compute(now)

* **Return type:**
  [`PointDefined`](#easy_robot_control.utils.trajectories.PointDefined)
* **Parameters:**
  **now** (*Time*)

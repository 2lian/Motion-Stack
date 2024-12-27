# easy_robot_control.utils.trajectories module

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

# easy_robot_control.utils.hyper_sphere_clamp module

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

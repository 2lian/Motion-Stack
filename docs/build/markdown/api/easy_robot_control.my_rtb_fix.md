# easy_robot_control.my_rtb_fix package

## Submodules

## easy_robot_control.my_rtb_fix.fixed_urdf module

Fixes the URDF object of rtb: [https://github.com/petercorke/robotics-toolbox-python/pull/441](https://github.com/petercorke/robotics-toolbox-python/pull/441)

### Example

Overwrite the library using:

```default
import roboticstoolbox.tools.urdf.urdf as bad

import easy_robot_control.my_rtb_fix.fixed_urdf as fix

bad.URDF.__init__ = fix.URDF.__init__
bad.URDF._recursive_axis_definition = fix.URDF._recursive_axis_definition
bad.URDF.finalize_linking = fix.URDF.finalize_linking
```

#### NOTE
easy_robot_control/_\_init_\_.py overwrites it, so importing easy_robot_control             will change your rtb

@author (Original) Matthew Matl, Github: mmatl
@author (Adapted by) Jesse Haviland
@author (Fixed by) Elian Neppel

### easy_robot_control.my_rtb_fix.fixed_urdf.rotation_fromVec_toVec(from_this_vector, to_this_vector)

Computes the rotation matrix from the first to the second vector.

### easy_robot_control.my_rtb_fix.fixed_urdf.from_this_vector

* **Type:**
  ArrayLike3

### easy_robot_control.my_rtb_fix.fixed_urdf.to_this_vector

* **Type:**
  ArrayLike3

* **Return type:**
  `SO3`
* **Returns:**
  rotation_from_to: SO3
  : Rotation matrix
* **Parameters:**
  * **from_this_vector** (*List* *[**float* *]*  *|* *Tuple* *[**float* *,* *float* *,* *float* *]*  *|* *ndarray* *[**Tuple* *[**Literal* *[**3* *]* *]* *,*  *~numpy.dtype* *[* *~numpy.floating* *]* *]*)
  * **to_this_vector** (*List* *[**float* *]*  *|* *Tuple* *[**float* *,* *float* *,* *float* *]*  *|* *ndarray* *[**Tuple* *[**Literal* *[**3* *]* *]* *,*  *~numpy.dtype* *[* *~numpy.floating* *]* *]*)

### Notes

Vector length is irrelevant.

### *class* easy_robot_control.my_rtb_fix.fixed_urdf.URDF(name, links, joints=None, transmissions=None, materials=None, other_xml=None)

Bases: `URDFType`

#### finalize_linking(childlink, joint)

Finalize the linking process after the link ets is set.

This directly changes childlink in place.
The ets of childlink must be defined prior to this.

* **Parameters:**
  * **childlink** (*rtb.Link*)
  * **joint** (*Joint*)

#### childlink

Link to finalize the definition of.

* **Type:**
  rtb.Link

#### joint

Joint used to define the link.

* **Type:**
  Joint

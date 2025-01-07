#!/usr/bin/env python
"""
Fixes the URDF object of rtb: https://github.com/petercorke/robotics-toolbox-python/pull/441

Example:
    Overwrite the library using::

        import roboticstoolbox.tools.urdf.urdf as bad

        import easy_robot_control.my_rtb_fix.fixed_urdf as fix

        bad.URDF.__init__ = fix.URDF.__init__
        bad.URDF._recursive_axis_definition = fix.URDF._recursive_axis_definition
        bad.URDF.finalize_linking = fix.URDF.finalize_linking

Note:
    `easy_robot_control/__init__.py` overwrites it, so importing easy_robot_control \
            will change your rtb

@author (Original) Matthew Matl, Github: mmatl
@author (Adapted by) Jesse Haviland
@author (Fixed by) Elian Neppel
"""

from typing import Optional, Tuple

import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox.tools.urdf.urdf import URDFType
from spatialmath import SE3, SO3
from spatialmath.base import ArrayLike3, getvector, unitvec


def rotation_fromVec_toVec(
    from_this_vector: ArrayLike3, to_this_vector: ArrayLike3
) -> SO3:
    """
    Computes the rotation matrix from the first to the second vector.

    Attributes
    ----------
        from_this_vector: ArrayLike3
        to_this_vector: ArrayLike3

    Returns
    -------
        rotation_from_to: SO3
            Rotation matrix

    Notes
    -----
        Vector length is irrelevant.
    """
    from_this_vector = getvector(from_this_vector)
    to_this_vector = getvector(to_this_vector)

    is_zero = np.all(np.isclose(from_this_vector, 0))
    if is_zero:
        target_axis = to_this_vector
    else:
        target_axis = unitvec(from_this_vector)

    dt = np.dot(target_axis, to_this_vector)
    crss = np.cross(target_axis, to_this_vector)

    is_parallel = np.all(np.isclose(crss, 0))
    if is_parallel:
        rotation_plane = unitvec(
            np.cross(target_axis, to_this_vector + np.array([1, 1, 1]))
        )
    else:
        rotation_plane = unitvec(crss)

    x = dt
    y = np.linalg.norm(crss)
    rotation_angle = np.arctan2(y, x)

    rotation_from_to = SO3.AngVec(rotation_angle, rotation_plane)
    return rotation_from_to


def _find_standard_joint(joint: "Joint") -> "rtb.ET | None":
    """
    Finds a pure rtb.ET joint corresponding to the URDF joint axis.

    If urdf joint is fixed, returns an empty rtb.ET.SE3.
    If none exists (axis is not +- 1 over x, y or z) returns None.

    Attributes
    ----------
    joint
       joint object read from the urdf.

    Returns
    -------
    std_joint: rtb.ET or None
        if a rtb.ET joint exists:
            Returns rtb.ET of type joint. This is rtb.ET.[SE(), Rx(), Ry(), ..., tz].
        else:
            Returns None.
    """
    std_joint = None
    if joint.joint_type in ("revolute", "continuous"):  # pragma nocover # noqa
        if joint.axis[0] == 1:
            std_joint = rtb.ET.Rx()
        elif joint.axis[0] == -1:
            std_joint = rtb.ET.Rx(flip=True)
        elif joint.axis[1] == 1:
            std_joint = rtb.ET.Ry()
        elif joint.axis[1] == -1:
            std_joint = rtb.ET.Ry(flip=True)
        elif joint.axis[2] == 1:
            std_joint = rtb.ET.Rz()
        elif joint.axis[2] == -1:
            std_joint = rtb.ET.Rz(flip=True)
    elif joint.joint_type == "prismatic":  # pragma nocover
        if joint.axis[0] == 1:
            std_joint = rtb.ET.tx()
        elif joint.axis[0] == -1:
            std_joint = rtb.ET.tx(flip=True)
        elif joint.axis[1] == 1:
            std_joint = rtb.ET.ty()
        elif joint.axis[1] == -1:
            std_joint = rtb.ET.ty(flip=True)
        elif joint.axis[2] == 1:
            std_joint = rtb.ET.tz()
        elif joint.axis[2] == -1:
            std_joint = rtb.ET.tz(flip=True)
    elif joint.joint_type == "fixed":
        std_joint = rtb.ET.SE3(SE3())
    return std_joint


def _find_joint_ets(
    joint: "Joint",
    parent_from_Rx_to_axis: Optional[SE3] = None,
) -> Tuple["rtb.ETS", SE3]:
    """
    Finds the ETS of a urdf joint object to be used by a Link.

    This is based on the following fomula:
        ets(N) = axis(N-1).inv() * transl(N) * rot(N) * axis(N) * [Rx]
    where N is the current joint, and (N-1) the parent joint.

    Attributes
    ----------
        joint: Joint
            Joint from the urdf.
            Used to deduce: transl(N), rot(N), axis(N), [Rx]
        parent_from_Rx_to_axis: Optional[SE3]
            SE3 resulting from the axis orientation of the parent joint
            Used to deduce: axis(N-1)

    Returns
    -------
        ets: ETS
            ETS representing the joint. It ends with a joint.
        from_Rx_to_axis: SE3
            SE3 representing the rotation of the axis attribute of the joint.
    """
    joint_trans = SE3(joint.origin).t
    joint_rot = joint.rpy
    if parent_from_Rx_to_axis is None:
        parent_from_Rx_to_axis = SE3()

    joint_without_axis = SE3(joint_trans) * SE3.RPY(joint_rot)

    std_joint = _find_standard_joint(joint)
    is_simple_joint = std_joint is not None

    if is_simple_joint:
        from_Rx_to_axis = SE3()
        pure_joint = std_joint
    else:  # rotates a Rx joint onto right axis
        joint_axis = joint.axis
        axis_of_Rx = np.array([1, 0, 0], dtype=float)

        rotation_from_Rx_to_axis = rotation_fromVec_toVec(
            from_this_vector=axis_of_Rx, to_this_vector=joint_axis
        )
        from_Rx_to_axis = SE3(rotation_from_Rx_to_axis)

        if joint.joint_type in ("revolute", "continuous"):
            pure_joint = rtb.ET.Rx(flip=0)
        elif joint.joint_type == "prismatic":  # I cannot test this case
            pure_joint = rtb.ET.tx(flip=0)
        else:
            pure_joint = rtb.ET.SE3(SE3())

    listET = []
    emptySE3 = SE3()

    # skips empty SE3
    if not parent_from_Rx_to_axis == emptySE3:
        listET.append(rtb.ET.SE3(parent_from_Rx_to_axis.inv()))
    listET.append(rtb.ET.SE3(joint_without_axis))
    if not from_Rx_to_axis == emptySE3:
        listET.append(rtb.ET.SE3(from_Rx_to_axis))
    if not joint.joint_type == "fixed":
        listET.append(pure_joint)

    ets = rtb.ETS(listET)

    return ets, from_Rx_to_axis


class URDF(URDFType):
    def __init__(
        self,
        name,
        links,
        joints=None,
        transmissions=None,
        materials=None,
        other_xml=None,
    ):
        if joints is None:  # pragma nocover
            joints = []
        if transmissions is None:  # pragma nocover
            transmissions = []
        if materials is None:
            materials = []

        # TODO, what does this next line do?
        # why arent the other things validated
        try:
            self._validate_transmissions()
        except Exception:
            pass

        self.name = name
        self.other_xml = other_xml

        # No setters for these
        self._links = list(links)
        self._joints = list(joints)
        self._transmissions = list(transmissions)
        self._materials = list(materials)
        self._material_map = {}

        for x in self._materials:
            if x.name in self._material_map:
                raise ValueError("Two materials with name {} " "found".format(x.name))
            self._material_map[x.name] = x

        # check for duplicate names
        if len(self._links) > len(
            set([x.name for x in self._links])
        ):  # pragma nocover  # noqa
            raise ValueError("Duplicate link names")
        if len(self._joints) > len(
            set([x.name for x in self._joints])
        ):  # pragma nocover  # noqa
            raise ValueError("Duplicate joint names")
        if len(self._transmissions) > len(
            set([x.name for x in self._transmissions])
        ):  # pragma nocover  # noqa
            raise ValueError("Duplicate transmission names")

        elinks = []
        elinkdict = {}
        # jointdict = {}

        # build the list of links in URDF file order
        for link in self._links:
            elink = rtb.Link(
                name=link.name,
                m=link.inertial.mass,
                r=(
                    link.inertial.origin[:3, 3]
                    if link.inertial.origin is not None
                    else None
                ),
                I=link.inertial.inertia,
            )
            elinks.append(elink)
            elinkdict[link.name] = elink

            # add the inertial parameters

            # add the visuals to visual list
            try:
                elink.geometry = [v.geometry.ob for v in link.visuals]
            except AttributeError:  # pragma nocover
                pass

            #  add collision objects to collision object list
            try:
                elink.collision = [col.geometry.ob for col in link.collisions]
            except AttributeError:  # pragma nocover
                pass

        # connect the links using joint info
        for joint in self._joints:
            # get references to joint's parent and child
            childlink: "rtb.Link" = elinkdict[joint.child]
            parentlink = elinkdict[joint.parent]

            childlink._parent = parentlink  # connect child link to parent
            childlink._joint_name = joint.name
            # Link precise definition will be done recursively later
        self.elinks = elinks

        # TODO, why did you put the base_link on the end?
        # easy to do it here

        # the childlink.ets and other info is set recursively here
        self._recursive_axis_definition()
        return

    def _recursive_axis_definition(
        self,
        parentname: Optional[str] = None,
        parent_from_Rx_to_axis: Optional[SE3] = None,
    ) -> None:
        """
        Recursively sets the ets of all elinks (in place).

        The ets of a link depends on the previous joint axis orientation.
        In a URDF a joint is defined as the following ets:
            ets = translation * rotation * axis * [Rx] * axis.inv()
        where Rx is the variable joint ets, and "axis" rotates the variable joint
        axis, BUT NOT the next link. Hence why Rx is rotated onto the axis, then
        the rotation is canceled by axis.inv().

        A Link is requiered to end with a variable ets -- this is our [Rx].
        The previous formula must therefore be changed and requires recursion:
            ets(N) = axis(N-1).inv() * transl(N) * rot(N) * axis(N) * [Rx]
        where N is the current joint, and (N-1) the parent joint.
        Chaining the ets of the second formula is equivalent to the first formula.

        Attributes
        ----------
            parentname: Optional[str]
                Name of the parent link.
            parent_from_Rx_to_axis: Optional[SE3]
                SE3 resulting from the axis orientation of the parent joint
        """
        if parentname is None:
            # starts again with all orphan links
            for link in self.elinks:
                if link.parent is None:
                    self._recursive_axis_definition(
                        parentname=link.name, parent_from_Rx_to_axis=None
                    )
        if parent_from_Rx_to_axis is None:
            parent_from_Rx_to_axis = SE3()

        for joint in self._joints:  # search  all joint with identical parent
            is_parent = joint.parent == parentname
            if not is_parent:
                continue  # skips to next joint

            ets, from_Rx_to_axis = _find_joint_ets(joint, parent_from_Rx_to_axis)

            for childlink in self.elinks:  # search all link with identical child
                is_child = joint.child == childlink.name
                if not is_child:
                    continue  # skips to next link

                childlink.ets = ets  # sets the ets of the joint
                self.finalize_linking(childlink, joint)

                self._recursive_axis_definition(
                    parentname=childlink.name, parent_from_Rx_to_axis=from_Rx_to_axis
                )

    def finalize_linking(self, childlink: "rtb.Link", joint: "Joint"):
        """
        Finalize the linking process after the link ets is set.

        This directly changes childlink in place.
        The ets of childlink must be defined prior to this.

        Attributes
        ----------
            childlink: rtb.Link
                Link to finalize the definition of.
            joint: Joint
                Joint used to define the link.
        """
        try:
            if childlink.isjoint:
                childlink.qlim = [joint.limit.lower, joint.limit.upper]
        except AttributeError:
            # no joint limits provided
            pass

        # joint friction
        try:
            if joint.dynamics.friction is not None:
                childlink.B = joint.dynamics.friction

            # TODO Add damping
            # joint.dynamics.damping
        except AttributeError:
            pass

        # joint gear ratio
        # TODO, not sure if t.joint.name is a thing
        for t in self.transmissions:  # pragma nocover
            if t.name == joint.name:
                childlink.G = t.actuators[0].mechanicalReduction

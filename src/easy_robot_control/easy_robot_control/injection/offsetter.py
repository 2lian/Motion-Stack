"""Provides a position offseter for lvl0, to be injected into a JointNode.
see the class docstring for details
"""

import csv
from copy import deepcopy
from os import path
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
from motion_stack_msgs.srv import SendJointState
from rclpy.node import Service

from easy_robot_control.EliaNode import error_catcher, list_cyanize
from easy_robot_control.joint_state_interface import JointHandler, JointNode
from easy_robot_control.utils.joint_state_util import JState, js_from_ros
from easy_robot_control.utils.state_remaper import StateRemapper, insert_angle_offset

def update_csv(file_path, new_str: str, new_float: float) -> Tuple[str, Optional[str]]:
    rows = []
    str_found = False
    file_path = path.expanduser(file_path)
    row_save = None

    if not path.exists(file_path):
        # Create the file and write the header
        with open(file_path, mode="w", newline="") as file:
            writer = csv.writer(file)

    with open(file_path, mode="r") as file:
        reader = csv.reader(file)
        for row in reader:
            if row and row[0] == new_str:
                # If the string is found, update the float value
                row_save = row[1]
                row[1] = str(new_float)  # useless ?
                str_found = True
            rows.append(row)

    # If the string is not found, append a new row
    if not str_found:
        rows.append([new_str, str(new_float)])

    # Write the updated data back to the CSV file
    with open(file_path, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerows(rows)

    return new_str, row_save


def csv_to_dict(file_path) -> Optional[Dict[str, float]]:
    if not path.exists(file_path):
        return None
    data_dict = {}

    # Open the CSV file in read mode
    with open(file_path, mode="r") as file:
        reader = csv.reader(file)

        # Iterate through each row in the CSV
        for row in reader:
            # Assuming the first column is string and the second column is float
            data_dict[row[0]] = float(row[1])

    return data_dict


class OffsetterLvl0:
    """Position offseter for lvl0, to be injected into a JointNode.
    Usefull if your URDF and robot are not aligned.
    You should inject this object into a JointNode at the end of initialization.
    Run offsetter.update_mapper() to apply the offsets.
    (sample code below)

    Features:
        - Inject this into your overload of the JointNode
        - Applies an angle offset to any joint of the lvl0 input/output.
        - Use a service to apply the offset at runtime
        - (Optional) Loads offsets from a csv on disk.
        - (Optional) Can receive offsets from a JointStates topic, and save them on disk
        - (Optional) Saves current angles multiplied by -1 every 3s in a csv on disk.
            This can tell you the last shutdown position of the robot if you need it.

    ====================== Injection sample code ====================
    class Example(JointNode):
        def __init__(self):
            super().__init__()
            self.offsetter = OffsetterLvl0(
                self, angle_path=ANGLE_PATH, offset_path=OFFSET_PATH
            )
    """

    def __init__(
        self,
        parent: JointNode,
        angle_path: Optional[str] = None,
        offset_path: Optional[str] = None,
    ) -> None:
        """
        Args:
            parent:
            angle_path: if None, does not save the current angles on disk
            offset_path: if None, does not save or load offsets from disk
        """
        self._offsets: Dict[str, float] = {}
        self.parent = parent
        self._lvl0_save = deepcopy(self.parent.lvl0_remap)
        self.angle_path = angle_path
        self.offset_path = offset_path
        self.load_offset()

        self.save_current_angleTMR = self.parent.create_timer(
            3, self.save_angle_as_offset
        )

        self.set_offsetSRV: Service = self.parent.create_service(
            SendJointState, "set_offset", self.__set_offsetSRVCBK
        )
        self.update_mapper()

    def save_angle_as_offset(self, handlers: Optional[Dict[str, JointHandler]] = None):
        """Saves the position as the offset to recover incase of powerloss"""
        if self.angle_path is None:
            return
        if handlers is None:
            handlers = self.parent.jointHandlerDic
        for name, jobj in handlers.items():
            if jobj.stateSensor.position is None:
                continue
            update_csv(self.angle_path, name, -jobj.stateSensor.position)

    def load_offset(self):
        """Loads offset from offset csv. Skips unknown joints."""
        if self.offset_path is None:
            return
        handlers = self.parent.jointHandlerDic
        off = csv_to_dict(self.offset_path)
        if off is None:
            self.parent.pwarn(f"Cannot load offsets from {self.offset_path}")
            return
        known_joints = handlers.keys() if handlers is not None else off.keys()
        names = set(known_joints) & set(off.keys())
        unknown_names: Set[str] = set(known_joints) - names
        for name, val in off.items():
            if np.isnan(val):
                self._offsets[name] = 0.0
            else:
                self._offsets[name] = off[name]
        if unknown_names:
            self.parent.pwarn(f"Joints not in offset csv: {list_cyanize(unknown_names)}")

    def save_current_offset(self, to_save: Optional[Dict[str, float]] = None):
        """DO NOT DO THIS AUTOMATICALLY, IT COULD BE DESTRUCTIVE OF VALUABLE INFO.
        overwrites the offset csv with the currently running offsets"""
        if self.offset_path is None:
            return
        if to_save is None:
            to_save = self._offsets
        user_disp = ""
        old_d = csv_to_dict(self.offset_path)
        if old_d is None:
            old_d = {}
        for name, off_val in to_save.items():
            old = old_d.get(name)
            if old is None:
                old = 0.0
            if np.isclose(old, off_val):
                continue
            update_csv(self.offset_path, name, off_val)
            user_disp += f"{name}: {old:.4f}->{off_val:.4f}\n"
        self.parent.pinfo(f"{self.offset_path} updated \n{user_disp}")

    def update_and_save_offset(self, js_offset: List[JState]) -> Tuple[bool, str]:
        """Held offset values will be changed and saved on disk for next time"""
        handler = self.parent.jointHandlerDic
        new: Dict[str, float] = {
            js.name: js.position for js in js_offset if js.position is not None
        }
        known = set(handler.keys()) & set(new.keys())
        unchanged = set(handler.keys()) - set(new.keys())
        unused = set(new.keys()) - set(handler.keys())
        self._offsets.update(new)
        self.save_current_offset(new)
        outstr = f"Updated: {known}. "
        if unused:
            return False, outstr + f"Joints unknown: {unused}."
        return True, outstr

    def update_mapper(
        self,
        mapper_in: Optional[StateRemapper] = None,
        mapper_out: Optional[StateRemapper] = None,
    ) -> None:
        """Applies the offsets to a StateRemapper.

        Args:
            mapper_in: original map to which offset should be added
            mapper_out: affected subshaper of this map will change
        """
        if mapper_in is None:
            mapper_in = self._lvl0_save
        if mapper_out is None:
            mapper_out = self.parent.lvl0_remap

        insert_angle_offset(mapper_in, mapper_out, self._offsets)

    @error_catcher
    def __set_offsetSRVCBK(
        self, req: SendJointState.Request, res: SendJointState.Response
    ) -> SendJointState.Response:
        """service to set the offsets of several joints and save"""
        if len(req.js.name) < 1:
            return res
        states: List[JState] = js_from_ros(req.js)  # type: ignore
        res.success, res.message = self.update_and_save_offset(states)
        self.update_mapper()

        return res

from copy import deepcopy
from typing import Dict, List, Optional, Set, Tuple

from motion_stack.core.utils.printing import list_cyanize
import numpy as np

from motion_stack.api.injection.remapper import StateRemapper, insert_angle_offset
from motion_stack.core.lvl1_joint import JointCore, JointHandler
from motion_stack.core.utils.joint_state import JState

from easy_robot_control.utils.csv import csv_to_dict, update_csv


class OffsetterLvl0:
    r"""
    Position offseter for lvl0.
    Usefull if your URDF and robot are not aligned.

    Features

    - Simply provide the JointCore on which to apply the offsets
    - Apply an angle offset to any joint of the lvl0 input/output.
    - Apply the offset at runtime
    - (Optional) Load offsets from a csv on disk.
    - (Optional) Save current angles multiplied by -1 in a csv on disk. This saved angle can tell you the last shutdown position of the robot, if you need to recover the offsets from it.

    Note:
        - You should provide this object an initialized JointCore.
        - You need to call by yourself:

            - \ :py:meth:`.OffsetterLvl0.apply_offset`
            - \ :py:meth:`.OffsetterLvl0.save_angle_as_offset`
            - \ :py:meth:`.OffsetterLvl0.save_current_offset`

        - \ :py:meth:`.OffsetterLvl0.load_offset` is called on object initialization.
        - See the ros2 wrapper if you running the core through ros2.

    Args:
        core: Initialized JointCore
        angle_recovery_path: if None, does not save the current angles on disk
        offset_path: if None, does not save or load offsets from disk
    """

    def __init__(
        self,
        core: JointCore,
        angle_recovery_path: Optional[str] = None,
        offset_path: Optional[str] = None,
    ) -> None:
        """

        """
        self._offsets: Dict[str, float] = {}
        self._core = core
        self._lvl0_save = deepcopy(self._core.lvl0_remap)
        self._angle_path = angle_recovery_path
        self._offset_path = offset_path
        self.load_offset()

        self._update_mapper()

    @property
    def offsets(self) -> Dict[str, float]:
        """Offets being used"""
        return self._offsets

    def _update_mapper(
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
            mapper_out = self._core.lvl0_remap

        insert_angle_offset(mapper_in, mapper_out, self._offsets)

    def apply_offset(self, js_offset: Optional[List[JState]]) -> Tuple[bool, str]:
        """Offset values will be replaced by new ones then saved on disk.

        Note:
            Preferably use this to not lose the offset in case of restart

        Args:
            js_offset: list of offsets

        Returns:
            True if all offsets have a joint to be applied to
            String for user debugging
        """
        if js_offset is None:
            js_offset = [JState(name=n, position=p) for n, p in self._offsets.items()]
        ret = self._update_offset(js_offset)
        self._update_mapper()
        self.save_current_offset()
        return ret

    def save_angle_recovery(self, handlers: Optional[Dict[str, JointHandler]] = None):
        """Saves current position as the offset to recover to incase of powerloss.

        Note:
            - Saved in self.angle_path
            - To use those saves as offsets, replace the file `<self.offset_path>` with `<self.angle_path>`
        """
        if self._angle_path is None:
            return
        if handlers is None:
            handlers = self._core.jointHandlerDic
        for name, jobj in handlers.items():
            if jobj.sensor.position is None:
                continue
            update_csv(self._angle_path, name, -jobj.sensor.position)

    def load_offset(self):
        """Loads offset from offset csv. Skips unknown joints."""
        if self._offset_path is None:
            return
        handlers = self._core.jointHandlerDic
        off = csv_to_dict(self._offset_path)
        if off is None:
            self._core.warn(f"Cannot load offsets from {self._offset_path}")
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
            self._core.warn(f"Joints not in offset csv: {list_cyanize(unknown_names)}")

    def save_current_offset(self, to_save: Optional[Dict[str, float]] = None):
        """DO NOT DO THIS AUTOMATICALLY, IT COULD BE DESTRUCTIVE OF VALUABLE INFO.
        overwrites the offset csv with the currently running offsets"""
        if self._offset_path is None:
            return
        if to_save is None:
            to_save = self._offsets
        user_disp = ""
        old_d = csv_to_dict(self._offset_path)
        if old_d is None:
            old_d = {}
        for name, off_val in to_save.items():
            old = old_d.get(name)
            if old is None:
                old = 0.0
            if np.isclose(old, off_val):
                continue
            update_csv(self._offset_path, name, off_val)
            user_disp += f"{name}: {old:.4f}->{off_val:.4f}\n"
        self._core.info(f"{self._offset_path} updated \n{user_disp}")

    def _update_offset(self, js_offset: List[JState]) -> Tuple[bool, str]:
        """Updates offset in memory

        Args:
            js_offset: list of offsets

        Returns:
            True if all offsets have a joint to be applied to
            String for user debugging
        """
        handler = self._core.jointHandlerDic
        new: Dict[str, float] = {
            js.name: js.position for js in js_offset if js.position is not None
        }
        known = set(handler.keys()) & set(new.keys())
        unchanged = set(handler.keys()) - set(new.keys())
        unused = set(new.keys()) - set(handler.keys())
        for n, val in new.items():
            if not n in self._offsets.keys():
                self._offsets[n] = 0
            self._offsets[n] += val
        outstr = f"Updated: {known}. "
        if unused:
            return False, outstr + f"Joints unknown: {unused}."
        return True, outstr

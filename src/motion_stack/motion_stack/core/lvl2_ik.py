"""
This node is responsible for recieving targets in the body reference frame, and send the
corresponding angles to the motors.

Author: Elian NEPPEL
Lab: SRL, Moonshot team
"""

from asyncio import Future

import matplotlib

from motion_stack.api.joint_syncer import JointSyncer

matplotlib.use("Agg")  # fix for when there is no display

from typing import Any, Callable, Dict, List, Optional, Tuple, Union, Type

import numpy as np
import quaternion as qt
import roboticstoolbox as rtb
from geometry_msgs.msg import Transform, Vector3
from numpy.typing import NDArray
from roboticstoolbox import ET, ETS, Link, Robot
from roboticstoolbox.robot.ET import SE3
from roboticstoolbox.tools import Joint

from .utils.joint_state import JState, impose_state, jattr, jdata, js_changed, jstamp
from .utils.pose import Pose
from .utils.printing import TCOL, list_cyanize
from .utils.robot_parsing import (
    get_limit,
    joint_by_joint_fk,
    load_set_urdf,
    load_set_urdf_raw,
    make_ee,
)
from .utils.static_executor import FlexNode
from .utils.time import NANOSEC, Time

float_formatter = "{:.2f}".format
np.set_printoptions(formatter={"float_kind": float_formatter})

# IK_MAX_VEL = 0.003  # changes depending on the refresh rate idk why. This is bad
IK_MAX_VEL = (
    10  # changes depending on the refresh rate and dimensions idk why. This is bad
)


class IKCore(FlexNode):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.send_to_lvl1_callbacks: List[Callable[[List[JState]], Any]] = []
        self.send_to_lvl3_callbacks: List[Callable[[Pose], Any]] = []
        self.RESET_LAST_SENT: Time = Time(sec=0.5)

        # V Parameters V
        #   \  /   #
        #    \/    #
        self.leg_num = self.ms_param["leg_number"]
        self.spinner.alias = f"IK{self.leg_num}"

        self.REFRESH_RATE = self.ms_param["mvmt_update_rate"]
        self.IGNORE_LIM = self.ms_param["ignore_limits"]
        self.MARGIN: float = self.ms_param["limit_margin"]
        self.SPEED_MODE: bool = self.ms_param["speed_mode"]
        self.ADD_JOINTS: List[str] = list(self.ms_param["add_joints"])
        self.urdf_raw = self.ms_param["urdf"]
        self.start_effector: str | None = self.ms_param["start_effector_name"]
        end_effector: str = self.ms_param["end_effector_name"]

        self.end_effector_name = make_ee(end_effector, self.leg_num)

        (
            self.model,
            self.et_chain,
            self.joint_names,
            self.joints_objects,
            self.last_link,
        ) = load_set_urdf_raw(
            self.urdf_raw, self.end_effector_name, self.start_effector
        )

        # try:
        #     if isinstance(self.end_effector_name, str) and isinstance(
        #         self.start_effector, str
        #     ):
        #         (  # don't want to see this
        #             model2,
        #             ETchain2,
        #             joint_names2,
        #             joints_objects2,
        #             last_link2,
        #         ) = load_set_urdf_raw(
        #             self.urdf_raw, self.start_effector, self.end_effector_name
        #         )
        #         if len(joint_names2) > len(self.joint_names) and len(
        #             joints_objects2
        #         ) > len(self.joints_objects):
        #             joint_names2.reverse()
        #             self.joint_names = joint_names2
        #             joints_objects2.reverse()
        #             self.joints_objects = joints_objects2
        # except:
        #     self.info(f"link tree could not be reversed")

        # self.pinfo(self.model)
        self.et_chain: ETS
        # self.pwarn(len(self.joints_objects))
        # self.ETchain = ETS(self.ETchain.compile())

        self.end_link: Link = self.last_link  # type: ignore
        self.info(f"Kinematic chain is:\n{self.et_chain}")
        jbj_fk = joint_by_joint_fk(self.et_chain, self.joint_names)
        self.info(f"Joint by joint forward kinematics:\n{jbj_fk}")

        # self.ETchain = self.all_limits(self.ETchain, self.joints_objects)
        self.subModel: Robot = rtb.Robot(self.et_chain)
        self.info(
            f"Using base_link: {TCOL.OKCYAN}{self.model.base_link.name}{TCOL.ENDC}"
            f", to ee:  {TCOL.OKCYAN}{self.end_link.name}{TCOL.ENDC}"
        )
        #    /\    #
        #   /  \   #
        # ^ Parameters ^

        self.stated: Dict[str, JState] = {}  #: recent addition storing the whole state
        self.angles: NDArray = np.empty(len(self.joint_names), dtype=float)
        self.angles[:] = np.nan
        self.last_sent: NDArray = self.angles.copy()
        self.lastTimeIK = Time(0)
        self.ready = False

        self.joint_syncer = JointSyncerIk(self)

    def firstSpinCBK(self):
        return

    def all_limits(self, et_chain: ETS, jobjL: List[Joint]):
        li = 0
        for j in et_chain:
            if j.isjoint:
                jobj = jobjL[li]
                li += 1
                self.error(
                    f"{jobj.name}, {jobj.limit.lower}, {jobj.limit.upper}, {j._qlim}"
                )
        return ETS(et_chain)

    def compute_raw_ik(
        self,
        pose: Pose,
        start: NDArray,
        compute_budget: Optional[Time] = None,  #  type: ignore
        mvt_duration: Optional[Time] = None,  #  type: ignore
    ) -> Tuple[Optional[NDArray], bool]:

        computeBudget: Time
        deltaTime: Time
        if mvt_duration is None:
            deltaTime = Time(sec=1 / self.REFRESH_RATE)  # type: ignore
        else:
            deltaTime = mvt_duration
        if compute_budget is None:
            computeBudget = Time(sec=1 / self.REFRESH_RATE)
        else:
            computeBudget = compute_budget
        del compute_budget, mvt_duration  # ugly af

        finish_by: Time = self.now() + computeBudget

        motion: SE3 = SE3(pose.xyz)  # type: ignore
        motion.A[:3, :3] = qt.as_rotation_matrix(pose.quat)  # type: ignore

        if self.angles.shape[0] > 5:
            mask = np.array([1, 1, 1, 1, 1, 1], dtype=float)
        else:
            mask = np.array([1, 1, 1, 0, 0, 0], dtype=float)

        angles: NDArray = self.angles.copy()
        np.nan_to_num(x=angles, nan=0.0, copy=False)
        np.nan_to_num(x=start, nan=0.0, copy=False)
        # self.pinfo(f"start: {start}")
        # for trial in range(4):
        trial = -1
        trialLimit = 20
        bestSolution: Optional[NDArray] = None
        velMaybe: float = 1000000
        validSolFound = False
        compBudgetExceeded = lambda: self.now() > finish_by
        # compBudgetExceeded = lambda: False
        while trial < trialLimit and not compBudgetExceeded():
            # self.pinfo(f"trial {trial}")
            trial += 1
            startingPose = start.copy()

            if trial == 0:
                i = 10
                s = 1
            if trial == 1:
                i = 50
                s = 1
            else:
                i = 50
                s = 1_000
                # s = 100

                stpose = np.empty((s, startingPose.shape[0]), float)
                stpose[:, :] = startingPose.reshape(1, -1)
                r = np.random.rand(stpose.shape[0], stpose.shape[1])
                r = r * 2 - 1
                maxi = 1 / 100
                mini = maxi / 100
                r = r * np.linspace(mini, maxi, s, endpoint=True).reshape(-1, 1)
                startingPose = stpose + r
                # self.pwarn(startingPose)

            ik_result = self.subModel.ik_LM(
                # ik_result = self.subModel.ik_NR(
                Tep=motion,
                q0=startingPose,
                mask=mask,
                ilimit=i,
                slimit=s,
                joint_limits=not self.IGNORE_LIM,
                # pinv=True,
                # pinv_damping=0.2,
                tol=1e-6,
            )
            # self.pwarn(not self.IGNORE_LIM)

            solFound = ik_result[1]

            sol = np.array(ik_result[0], dtype=float)
            sol_im = 1 * np.exp(1j * sol)
            star = 1 * np.exp(1j * start)
            delt = sol_im / star
            real_delta = np.angle(delt)
            real_angles = start + real_delta
            # self.pinfo(sol)
            # self.pwarn(real_angles)
            for ind, a in enumerate(real_angles):
                l = self.joints_objects[ind].limit
                if l is None:
                    continue
                lup = l.upper
                llo = l.lower
                # if limit exceeded we go back to the original solution
                if lup is not None and not self.IGNORE_LIM:
                    if real_angles[ind] > lup:
                        real_angles[ind] = sol[ind]
                if llo is not None and not self.IGNORE_LIM:
                    if real_angles[ind] < llo:
                        real_angles[ind] = sol[ind]

            # self.pwarn(real_angles)
            delta = real_angles - start
            # dist = float(np.linalg.norm(delta, ord=np.inf))
            dist = float(np.linalg.norm(delta, ord=3))
            velocity: float = dist / deltaTime.sec()

            if solFound:
                if abs(velocity) < abs(IK_MAX_VEL):
                    angles = real_angles
                    validSolFound = True
                    velMaybe = velocity
                    bestSolution = real_angles
                    break
                isBetter = velocity < velMaybe
                if isBetter:
                    bestSolution = real_angles
                    velMaybe = velocity

        if compBudgetExceeded():
            self.warn("IK slow, compute terminated")

        return bestSolution, validSolFound

    def find_next_ik(
        self,
        pose: Pose,
        compute_budget: Optional[Time] = None,  #  type: ignore
        mvt_duration: Optional[Time] = None,  #  type: ignore
    ) -> NDArray:

        if not self.ready:
            self.warn(f"{TCOL.FAIL} No angle data available, assuming joints at 0.")

        arriveTime: Time = self.now()
        deltaTime: Time = arriveTime - self.lastTimeIK
        self.lastTimeIK = arriveTime
        ikIsRecent = deltaTime < self.RESET_LAST_SENT
        if ikIsRecent:
            start: NDArray = self.last_sent.copy()
        else:
            start: NDArray = self.angles.copy()

        assert start.shape == self.angles.shape

        bestSolution, validSolutionFound = self.compute_raw_ik(
            pose,
            start,
            compute_budget=compute_budget,  #  type: ignore
            mvt_duration=mvt_duration,  #  type: ignore
        )

        if bestSolution is None:
            self.warn("""no IK found :C""")
            return start

        if not validSolutionFound:
            pass
            self.warn("no continuous IK found :C")

        return bestSolution

    def ik_target(self, pose: Pose) -> None:
        """
        recieves target from leg, converts to numpy, computes IK, sends angle
        results to joints

        Args:
            msg: target as Ros2 Vector3
        """
        pose = self._replace_nan(pose)
        pose.xyz /= 1000

        angles = self.find_next_ik(pose)

        self._send_command(angles)
        return

    def _replace_nan(self, pose: Pose) -> Pose:
        xyz, quat = pose.xyz, pose.quat
        fk = None
        if np.any(np.isnan(xyz)):
            if fk is None:
                fk = self.current_fk()
            xyz = fk.xyz
        if np.any(np.isnan(qt.as_float_array(quat))):
            if fk is None:
                fk = self.current_fk()
            quat = fk.quat
        new = Pose(time=pose.time, xyz=xyz, quat=quat)
        return new

    def state_from_lvl1(self, states: List[JState]):
        di = {j.name: j for j in states}
        joint_of_interest = set(self.joint_names) & set(di.keys())

        self.stated.update(
            {
                name: impose_state(onto=self.stated.get(name), fromm=di[name])
                for name in joint_of_interest
            }
        )

        for name in joint_of_interest:
            ind = self.joint_names.index(name)
            self.angles[ind] = di[name].position
        missing = np.any(np.isnan(self.angles))
        if not missing:
            fk = self.send_current_fk()
            if not self.ready:
                self.info(f"{TCOL.OKGREEN}Forward Kinematics ready :){TCOL.ENDC}: {fk}")
                self.ready = True
            else:
                self.joint_syncer.execute()

    def _send_command(self, angles: NDArray):
        assert self.last_sent.shape == angles.shape
        assert angles.dtype in [float, np.float32]

        self.last_sent: NDArray = angles.copy()
        now = self.now()
        # states = [
            # JState(name=n, position=a, time=now) for n, a in zip(self.joint_names, angles)
        # ]
        # self.send_to_lvl1(states)
        target = {name: angle for name, angle in zip(self.joint_names, angles)}
        try:
            # self.joint_syncer.lerp(target)
            self.joint_syncer.unsafe(target)
        except AssertionError:
            self.warn("Joint syncer not ready.")

        self.joint_syncer.execute()
        return

    def send_to_lvl1(self, states: List[JState]):
        for f in self.send_to_lvl1_callbacks:
            f(states)

    def send_current_fk(self) -> Pose:
        fk = self.current_fk()
        self.send_to_lvl3(fk)
        return fk

    def send_to_lvl3(self, pose: Pose):
        for f in self.send_to_lvl3_callbacks:
            f(pose)

    def current_fk(self) -> Pose:
        fw_result: List[SE3] = self.subModel.fkine(self.angles)  # type: ignore
        rot_matrix = np.array(fw_result[-1].R, dtype=float)
        tip_coord: NDArray = fw_result[-1].t * 1000
        tip_quat: qt.quaternion = qt.from_rotation_matrix(rot_matrix)
        return Pose(time=self.now(), xyz=tip_coord, quat=tip_quat)


class JointSyncerIk(JointSyncer):
    def __init__(
        self,
        core: IKCore,
        interpolation_delta: float = np.deg2rad(10),
        on_target_delta: float = np.deg2rad(10),
    ) -> None:
        super().__init__(interpolation_delta, on_target_delta)
        self._core = core

    def send_to_lvl1(self, states: List[JState]):
        self._core.send_to_lvl1(states)

    @property
    def sensor(self) -> Dict[str, JState]:
        return self._core.stated

    @property
    def FutureT(self) -> Type[Future]:
        return Future

import json
import time
from dataclasses import asdict
from os import environ
from threading import Lock
from typing import Any, Callable, Dict, List

import numpy as np
import zenoh
from geometry_msgs.msg import Transform, TransformStamped
from motion_stack_msgs.srv import ReturnJointState
from rclpy.node import Node
from rclpy.task import Future, Task
from sensor_msgs.msg import JointState

from motion_stack.core.utils.pose import Pose
from motion_stack.core.utils.time import Time

from ...core.utils.joint_state import JState
from ...ros2.base_node.lvl2 import IKCore, Lvl2Node
from ...ros2.communication import lvl2 as comms
from ...ros2.utils.conversion import pose_to_transform, time_to_ros, transform_to_pose
from ...ros2.utils.executor import error_catcher
from ...ros2.utils.joint_state import CallablePublisher, JSCallableWrapper, ros2js_wrap

global_lock = Lock()


class DefaultLvl2(Lvl2Node):
    """Default implementation of the Joint node of lvl2.

    Refer to :py:class:`.ros2.base_node.lvl2.Lvl2Node` for documentation on linking ros2 and python core of lvl1. This only makes use of this base to create the default implementation and give an example.
    """

    alive_srv = comms.alive

    def __init__(self):
        with global_lock:
            print("lvl2 USING ZENOH")
            zenoh_config_file = zenoh.Config.from_file(
                environ["ZENOH_SESSION_CONFIG_URI"]
            )
            self.session = zenoh.open(zenoh_config_file)
            self.zenoh_pub_lvl1: Dict[str, zenoh.Publisher] = dict()
            self.data_pub_lvl1: Dict[str, JState] = dict()

            super().__init__()
            self.tip_pos_PUB: Callable[[Transform], None] = CallablePublisher(
                node=self,
                topic_type=comms.output.tip_pos.type,
                topic_name=comms.output.tip_pos.name,
                qos_profile=comms.output.tip_pos.qos,
            )

    def __del__(self):
        self.zenoh_sub_lvl1.undeclare()
        for key, val in self.zenoh_pub_lvl1.items():
            val.undeclare()
            del self.zenoh_pub_lvl1[key]
        self.session.close()

    def subscribe_to_lvl1(self, lvl1_input: Callable[[List[JState]], Any]):
        """"""

        last_task_dic: Dict[str, Task] = dict()
        last_data_dic: Dict[str, JState] = dict()

        def listener(sample: zenoh.Sample):
            # print(f"Received {sample.kind} ('{sample.key_expr}': '{sample.payload.to_string()}')")
            json_listdic: Dict = json.loads(sample.payload.to_bytes())
            state: JState = JState(
                name=json_listdic["name"],
                time=Time(json_listdic["time"]),
                position=json_listdic["position"],
                velocity=json_listdic["velocity"],
                effort=json_listdic["effort"],
            )
            with global_lock:
                if self.executor is None:
                    return
                last_task = last_task_dic.get(state.name)
                last_data = last_data_dic.get(state.name)
                try:
                    display = f"replaced, missing timestamp\n{last_data=}\n{state=}\n"
                except AttributeError:
                    pass
                if last_task is not None:
                    if last_data is not None:  # cancel if no data
                        if last_data.time is not None and state.time is not None:
                            # cancel if no time data
                            if last_data.time <= state.time:
                                # cancel if new data
                                display = f"replacing:\n  {last_data.time.nano():_} {last_data.name}\n  {state.time.nano():_} {last_data.name}"
                                last_task.cancel()
                        else:
                            last_task.cancel()
                    else:
                        last_task.cancel()

                def fin(fut: Future):
                    nonlocal display
                    if fut.cancelled():
                        print(display)
                    return

                task = self.executor.create_task(
                    callback=lambda **args: lvl1_input([state])
                )

                task.add_done_callback(fin)
                last_task_dic[state.name] = task
                last_data_dic[state.name] = state

        self.zenoh_sub_lvl1 = self.session.declare_subscriber(
            f"ms{self.get_namespace()}/{comms.input.joint_state.name}/**",
            listener,
        )

        # self.create_subscription(
        # comms.input.joint_state.type,
        # comms.input.joint_state.name,
        # ros2js_wrap(lvl1_input),
        # qos_profile=comms.input.joint_state.qos,
        # )

    def subscribe_to_lvl3(self, lvl3_input: Callable[[Pose], Any]):
        """"""

        def cbk(tf: Transform):
            return lvl3_input(transform_to_pose(tf, time=self.core.now()))

        self.create_subscription(
            comms.input.set_ik.type,
            comms.input.set_ik.name,
            cbk,
            qos_profile=comms.input.set_ik.qos,
        )

    def publish_to_lvl1(self, states: List[JState]):
        """"""
        for js in states:
            pub = self.zenoh_pub_lvl1.get(js.name)
            prev = self.data_pub_lvl1.get(js.name)
            if pub is None:
                pub = self.session.declare_publisher(
                    f"ms{self.get_namespace()}/{comms.output.joint_target.name}/{js.name}"
                )
                self.zenoh_pub_lvl1[js.name] = pub
            if prev is not None:
                is_same = prev == js
                if is_same:
                    continue
            self.data_pub_lvl1[js.name] = js
            pub.put(json.dumps(asdict(js), indent=1))

    def publish_to_lvl3(self, pose: Pose):
        """"""
        assert self.executor is not None
        self.executor.create_task(
            callback=lambda **args: self.tip_pos_PUB(pose_to_transform(pose))
        )
        # self.tip_pos_PUB(pose_to_transform(pose))

    def startup_action(self, lvl2: IKCore):
        """"""
        self.create_service(self.alive_srv.type, self.alive_srv.name, lambda *_: None)


def main(*args, **kwargs):
    DefaultLvl2.spin()

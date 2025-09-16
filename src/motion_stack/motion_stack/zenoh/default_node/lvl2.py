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
from sensor_msgs.msg import JointState

from motion_stack.core.utils.pose import Pose

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

            super().__init__()
            raw_publisher: Callable[[JointState], None] = CallablePublisher(
                node=self,
                topic_type=comms.output.joint_target.type,
                topic_name=comms.output.joint_target.name,
                qos_profile=comms.output.joint_target.qos,
            )
            self.wrapped_lvl1_PUB = JSCallableWrapper(raw_publisher)
            self.tip_pos_PUB: Callable[[Transform], None] = CallablePublisher(
                node=self,
                topic_type=comms.output.tip_pos.type,
                topic_name=comms.output.tip_pos.name,
                qos_profile=comms.output.tip_pos.qos,
            )

    def __del__(self):
        self.session.close()
        self.zenoh_sub_lvl1.undeclare()
        for key, val in self.zenoh_pub_lvl1.items():
            val.undeclare()
            del self.zenoh_pub_lvl1[key]

    def subscribe_to_lvl1(self, lvl1_input: Callable[[List[JState]], Any]):
        """"""

        def listener(sample: zenoh.Sample):
            # print(f"Received {sample.kind} ('{sample.key_expr}': '{sample.payload.to_string()}')")
            json_listdic: Dict = json.loads(sample.payload.to_bytes())
            state: JState = JState(**json_listdic)
            with global_lock:
                try:
                    assert self.executor is not None
                    task = self.executor.create_task(
                        callback=lambda **args: lvl1_input([state])
                    )
                    while not task.done():
                        time.sleep(1/1000)
                    # print(f"parsed:\n{state}")
                except:
                    self.session.close()

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
            if pub is None:
                pub = self.session.declare_publisher(
                    f"ms{self.get_namespace()}/{comms.output.joint_target.name}/{js.name}"
                )
                self.zenoh_pub_lvl1[js.name] = pub
            pub.put(json.dumps(asdict(js), indent=1))
        # self.wrapped_lvl1_PUB(states)

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

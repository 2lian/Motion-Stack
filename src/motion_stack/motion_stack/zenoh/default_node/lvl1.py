import json
import logging
import sys
import time
import uuid
from dataclasses import asdict
from os import environ
from threading import Lock
from typing import Any, Callable, Dict, List

import numpy as np
import zenoh
from motion_stack_msgs.srv import ReturnJointState
from rclpy.node import Node
from rclpy.task import Future, Task
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from ...core.lvl1_joint import JointCore
from ...core.utils.joint_state import JState
from ...core.utils.time import Time
from ...ros2.base_node.lvl1 import Lvl1Node
from ...ros2.communication import lvl1 as comms
from ...ros2.utils.conversion import time_to_ros
from ...ros2.utils.executor import error_catcher
from ...ros2.utils.joint_state import CallablePublisher, JSCallableWrapper, ros2js_wrap

global_lock = Lock()


class DefaultLvl1(Lvl1Node):
    """Default implementation of the Joint node of lvl1.

    Refer to :py:class:`.ros2.base_node.lvl1` for documentation on linking ros2 and python core of lvl1. This only makes use of this base to create the default implementation and give an example.

    **Publishers:**

    ==============  ==========  ====
    Topic           Type        Note
    ==============  ==========  ====
    joint_commands  JointState  sent to motors (lvl0)
    joint_read      JointState  sent to IK (lvl2)
    ==============  ==========  ====

    **Subscribers:**

    ==============  ==========  ====
    Topic           Type        Note
    ==============  ==========  ====
    joint_states    JointState  coming from sensors (lvl0)
    joint_set       JointState  coming from IK (lvl2)
    ==============  ==========  ====

    **Service server:**

    ================ ================  ====
    Topic            Type              Note
    ================ ================  ====
    advertise_joints ReturnJointState  JointState with the name of all joints
    ================ ================  ====

    **Timers:**
        - Sends to lvl2, freq.= ROS2_PARAMETER[``mvmt_update_rate``].

    **Startup:**
        - Sends empty message to lvl0 with only joint names.
    """

    alive_srv = comms.alive

    def __init__(self):
        with global_lock:
            print("lvl1 USING ZENOH")
            zenoh_config_file = zenoh.Config.from_file(
                environ["ZENOH_SESSION_CONFIG_URI"]
            )
            self.session = zenoh.open(zenoh_config_file)
            self.zenoh_pub_lvl2: Dict[str, zenoh.Publisher] = dict()

            super().__init__()
            self.queryable = self.session.declare_queryable(
                key_expr=f"ms{self.get_namespace()}/"
                f"available_joints/"
                f"{uuid.uuid4()}",
                handler=self.advertize_zenoh,
            )

            raw_publisher: Callable[[JointState], None] = CallablePublisher(
                node=self,
                topic_type=comms.output.motor_command.type,
                topic_name=comms.output.motor_command.name,
                qos_profile=comms.output.motor_command.qos,
            )
            self.wrapped_pub_lvl0 = JSCallableWrapper(raw_publisher)
            raw_publisher: Callable[[JointState], None] = CallablePublisher(
                node=self,
                topic_type=comms.output.joint_state.type,
                topic_name=comms.output.joint_state.name,
                qos_profile=comms.output.joint_state.qos,
            )
            self.wrapped_pub_lvl2 = JSCallableWrapper(raw_publisher)
            # create_advertise_service(self, self.core)

    def advertize_zenoh(self, query: zenoh.Query) -> None:
        print("\nZenoh got query\n")
        jh_dic = self.core.jointHandlerDic
        data = {name: asdict(jh.sensor) for name, jh in jh_dic.items()}
        for key, val in data.items():
            del val["name"]
        query.reply(
            key_expr=self.queryable.key_expr, payload=json.dumps(data, indent=1)
        )

    def __del__(self):
        self.zenoh_sub_lvl2.undeclare()
        for key, val in self.zenoh_pub_lvl2.items():
            val.undeclare()
            del self.zenoh_pub_lvl2[key]
        self.session.close()

    def subscribe_to_lvl2(self, lvl2_input: Callable[[List[JState]], Any]):
        """"""

        last_task_dic: Dict[str, Task] = dict()
        last_data_dic: Dict[str, JState] = dict()

        def listener(sample: zenoh.Sample):
            nonlocal last_task_dic, last_data_dic
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
                    callback=lambda **args: lvl2_input([state])
                )

                task.add_done_callback(fin)
                last_task_dic[state.name] = task
                last_data_dic[state.name] = state

        self.zenoh_sub_lvl2 = self.session.declare_subscriber(
            f"ms{self.get_namespace()}/{comms.input.joint_target.name}/**",
            listener,
        )

    def subscribe_to_lvl0(self, lvl0_input: Callable[[List[JState]], Any]):
        """"""
        self.create_subscription(
            comms.input.motor_sensor.type,
            comms.input.motor_sensor.name,
            ros2js_wrap(lvl0_input),
            qos_profile=comms.input.motor_sensor.qos,
        )

    def publish_to_lvl0(self, states: List[JState]):
        """"""
        self.wrapped_pub_lvl0(states)

    def publish_to_lvl2(self, states: List[JState]):
        """"""
        for js in states:
            pub = self.zenoh_pub_lvl2.get(js.name)
            if pub is None:
                pub = self.session.declare_publisher(
                    f"ms{self.get_namespace()}/{comms.output.joint_state.name}/{js.name}",
                    reliability=zenoh.Reliability.RELIABLE,
                )
                self.zenoh_pub_lvl2[js.name] = pub
            pub.put(
                json.dumps(asdict(js), indent=0),
                encoding=zenoh.Encoding.APPLICATION_JSON,
            )
        self.wrapped_pub_lvl2(states)

    def frequently_send_to_lvl2(self, send_function: Callable[[], None]):
        """"""
        tmr = self.create_timer(
            1 / self.core.ms_param["mvmt_update_rate"], send_function
        )
        # self.core.error(tmr.timer_period_ns/1e9)

    def startup_action(self, core: JointCore):
        """"""
        core.send_empty_command_to_lvl0()
        # self.get_logger().error("hey")
        self.create_service(
            self.alive_srv.type, self.alive_srv.name, lambda req, res: res
        )


def create_advertise_queriable(session: zenoh.Session, lvl1: JointCore):
    """Creates the advertise_joints service and its callback.

    Callback returns a ReturnJointState.Response wich is a JointState with the name of all joints managed by the node. Other field of JointState are not meant to be used, but are filled with the latest data.

    Args:
        node: spinning node
        lvl1: lvl1 core
    """

    @error_catcher
    def cbk(
        req: ReturnJointState.Request, res: ReturnJointState.Response
    ) -> ReturnJointState.Response:
        names: List[str] = [h._sensor.name for h in lvl1.jointHandlerDic.values()]
        none2nan = lambda x: x if x is not None else np.nan
        res.js = JointState(
            name=names,
            position=[
                none2nan(h._sensor.position) for h in lvl1.jointHandlerDic.values()
            ],
            velocity=[
                none2nan(h._sensor.velocity) for h in lvl1.jointHandlerDic.values()
            ],
            effort=[none2nan(h._sensor.effort) for h in lvl1.jointHandlerDic.values()],
        )

        res.js.header.stamp = time_to_ros(lvl1.now()).to_msg()
        return res


def create_advertise_service(node: Node, lvl1: JointCore):
    """Creates the advertise_joints service and its callback.

    Callback returns a ReturnJointState.Response wich is a JointState with the name of all joints managed by the node. Other field of JointState are not meant to be used, but are filled with the latest data.

    Args:
        node: spinning node
        lvl1: lvl1 core
    """

    @error_catcher
    def cbk(
        req: ReturnJointState.Request, res: ReturnJointState.Response
    ) -> ReturnJointState.Response:
        names: List[str] = [h._sensor.name for h in lvl1.jointHandlerDic.values()]
        none2nan = lambda x: x if x is not None else np.nan
        res.js = JointState(
            name=names,
            position=[
                none2nan(h._sensor.position) for h in lvl1.jointHandlerDic.values()
            ],
            velocity=[
                none2nan(h._sensor.velocity) for h in lvl1.jointHandlerDic.values()
            ],
            effort=[none2nan(h._sensor.effort) for h in lvl1.jointHandlerDic.values()],
        )

        res.js.header.stamp = time_to_ros(lvl1.now()).to_msg()
        return res

    node.create_service(comms.output.advertise.type, comms.output.advertise.name, cbk)


def main(*args, **kwargs):
    DefaultLvl1.spin()
